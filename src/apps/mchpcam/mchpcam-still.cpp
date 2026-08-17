/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * mchpcam-still — single / burst / timelapse still image capture
 *                 with Microchip ISC IPA warmup
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <png.h>
#include <setjmp.h>
#include <signal.h>
#include <string.h>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <vector>

#include <libcamera/property_ids.h>

#include <jpeglib.h>

#include "mchpcam_common.h"

using namespace std::chrono_literals;

/* ------------------------------------------------------------------ */
/* Version string is derived from the linked libcamera library at      */
/* runtime via CameraManager::version() — no hardcoding needed.        */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* Global signal flags                                                  */
/* ------------------------------------------------------------------ */

static std::atomic<bool> g_sigUsr1{ false }; /* --signal: SIGUSR1 triggers capture */
static std::atomic<bool> g_sigTerm{ false }; /* SIGTERM/SIGINT: clean exit */

static void onSigUsr1([[maybe_unused]] int)
{
	g_sigUsr1.store(true);
}
static void onSigTerm([[maybe_unused]] int)
{
	g_sigTerm.store(true);
}

/* ------------------------------------------------------------------ */
/* Filename helpers                                                     */
/* ------------------------------------------------------------------ */

/*
 * Generate output filename with optional counters and time tokens.
 *
 * Supported tokens (same as rpicam-still for scripting compatibility):
 *   %d  — decimal frame counter (zero-padded to 4 digits)
 *   %t  — Unix epoch seconds (integer)
 *   %T  — local time as MMDDHHmmSS (rpicam-still --datetime format)
 */
static std::string makeFilename(const std::string &tmpl,
				unsigned int counter)
{
	std::string out;
	out.reserve(tmpl.size() + 16);

	for (size_t i = 0; i < tmpl.size(); i++) {
		if (tmpl[i] != '%' || i + 1 >= tmpl.size()) {
			out += tmpl[i];
			continue;
		}
		char tok = tmpl[i + 1];
		i++;
		switch (tok) {
		case 'd': {
			char buf[16];
			snprintf(buf, sizeof(buf), "%04u", counter);
			out += buf;
			break;
		}
		case 't': {
			char buf[32];
			snprintf(buf, sizeof(buf), "%ld",
				 static_cast<long>(std::time(nullptr)));
			out += buf;
			break;
		}
		case 'T': {
			std::time_t now = std::time(nullptr);
			struct tm *lt = std::localtime(&now);
			char buf[32];
			strftime(buf, sizeof(buf), "%m%d%H%M%S", lt);
			out += buf;
			break;
		}
		default:
			out += '%';
			out += tok;
			break;
		}
	}
	return out;
}

/* ------------------------------------------------------------------ */
/* Software image transforms                                            */
/* ------------------------------------------------------------------ */

/*
 * Apply horizontal flip to a packed RGB888 buffer in-place.
 * Each row is reversed pixel by pixel.
 */
static void applyHflip(std::vector<uint8_t> &rgb, unsigned int w, unsigned int h)
{
	for (unsigned int row = 0; row < h; ++row) {
		uint8_t *left = rgb.data() + row * w * 3;
		uint8_t *right = left + (w - 1) * 3;
		while (left < right) {
			std::swap(left[0], right[0]);
			std::swap(left[1], right[1]);
			std::swap(left[2], right[2]);
			left += 3;
			right -= 3;
		}
	}
}

/*
 * Apply vertical flip to a packed RGB888 buffer in-place.
 * Top and bottom rows are swapped.
 */
static void applyVflip(std::vector<uint8_t> &rgb, unsigned int w, unsigned int h)
{
	unsigned int rowBytes = w * 3;
	std::vector<uint8_t> tmp(rowBytes);
	for (unsigned int top = 0, bot = h - 1; top < bot; ++top, --bot) {
		std::copy(rgb.begin() + top * rowBytes,
			  rgb.begin() + top * rowBytes + rowBytes,
			  tmp.begin());
		std::copy(rgb.begin() + bot * rowBytes,
			  rgb.begin() + bot * rowBytes + rowBytes,
			  rgb.begin() + top * rowBytes);
		std::copy(tmp.begin(), tmp.end(),
			  rgb.begin() + bot * rowBytes);
	}
}

/* ------------------------------------------------------------------ */
/* MchpCamStill                                                         */
/* ------------------------------------------------------------------ */

class MchpCamStill : public MchpCamCommon
{
public:
	MchpCamStill() {}

	void setImageFormat(const std::string &fmt) { imageFormat_ = fmt; }
	void setJpegQuality(int q) { jpegQuality_ = std::clamp(q, 1, 100); }
	void setPngCompression(int l) { pngCompression_ = std::clamp(l, 0, 9); }
	void setTimeout(unsigned int s) { timeoutSec_ = s; }
	void setPrintMetadata(bool e) { printMetadata_ = e; }
	void setMetadataJson(bool e) { metadataJson_ = e; }
	void setLatestLink(const std::string &lnk) { latestLink_ = lnk; }
	void setKeypress(bool e)
	{
		useKeypress_ = e;
		if (e)
			keypressReceived_.store(false);
	}
	void setSignalTrigger(bool e)
	{
		useSignal_ = e;
		if (e)
			keypressReceived_.store(false);
	}
	/* Burst: capture N frames after convergence */
	void setCaptureCount(unsigned int n) { captureCount_ = n; }
	/* Timelapse: repeat capture every intervalMs milliseconds */
	void setTimelapse(unsigned int intervalMs) { timelapseMs_ = intervalMs; }

	/*
	 * captureStill() — run warmup loop then save the converged frame.
	 * Returns 0 on success, negative on error.
	 */
	int captureStill(const std::string &filename)
	{
		if (requests_.empty()) {
			std::cerr << "No requests — call init() first\n";
			return -EINVAL;
		}

		captureReady_ = false;
		frameCount_ = 0;

		libcamera::ControlList startControls(camera_->controls());
		int ret = camera_->start(&startControls);
		if (ret < 0) {
			std::cerr << "Failed to start camera: " << ret << "\n";
			return ret;
		}
		cameraStarted_ = true;

		/* Queue all requests so the ISC pipeline stays full */
		for (auto &req : requests_)
			camera_->queueRequest(req.get());

		/*
		 * Adaptive per-capture timeout.
		 * timeoutSec_==0 (default): derive from maxFrames_ and sensor fps
		 * so the timeout scales correctly at any framerate.
		 * Formula: (maxFrames / fps) * 1.5 + 2 s.
		 * At 30fps, maxFrames_=150: (150/30)*1.5+2 = 9.5 s.
		 * At 5fps,  maxFrames_=150: (150/5)*1.5+2  = 47 s.
		 * A user-supplied --timeout always overrides.
		 */
		unsigned int effectiveTimeout = timeoutSec_;
		if (effectiveTimeout == 0) {
			float fps = 30.0f;
			auto &controls = camera_->controls();
			auto it = controls.find(
				&libcamera::controls::FrameDurationLimits);
			if (it != controls.end()) {
				int64_t minDurUs = it->second.min().get<int64_t>();
				if (minDurUs > 0)
					fps = 1.0e6f / static_cast<float>(minDurUs);
			}
			effectiveTimeout = static_cast<unsigned int>(
				(static_cast<float>(maxFrames_) / fps) * 1.5f + 2.0f);
		}

		if (useKeypress_) {
			/*
			 * Wait for user to press Enter, then gate captureReady_
			 * via keypressReceived_ so requestComplete() — which owns
			 * savedBuffer_ — is the one that sets captureReady_ and
			 * saves the buffer.  Direct captureReady_.store(true) here
			 * would bypass the savedBuffer_ assignment, producing a
			 * null-buffer capture ("No buffer captured").
			 */
			std::cout << "Press Enter to capture...\n";
			std::string dummy;
			std::getline(std::cin, dummy);
			keypressReceived_.store(true);
			std::unique_lock<std::mutex> lock(captureMutex_);
			captureNotify_.wait_for(
				lock,
				std::chrono::seconds(effectiveTimeout),
				[this] { return captureReady_.load(); });
		} else if (useSignal_) {
			std::cout << "Waiting for SIGUSR1 to capture...\n";
			while (!g_sigUsr1.load() && !g_sigTerm.load())
				std::this_thread::sleep_for(50ms);
			g_sigUsr1.store(false);
			keypressReceived_.store(true);
			std::unique_lock<std::mutex> lock(captureMutex_);
			captureNotify_.wait_for(
				lock,
				std::chrono::seconds(effectiveTimeout),
				[this] { return captureReady_.load(); });
		} else {
			std::cout << "Warming up IPA (" << warmupFrames_
				  << " frames minimum)...\n";
			std::unique_lock<std::mutex> lock(captureMutex_);
			captureNotify_.wait_for(
				lock,
				std::chrono::seconds(effectiveTimeout),
				[this] { return captureReady_.load(); });
		}

		if (!captureReady_.load())
			std::cerr << "Warning: capture timed out after "
				  << effectiveTimeout << " s before convergence\n";

		/* Stop camera */
		stopStreaming();

		if (!savedBuffer_) {
			std::cerr << "No buffer captured\n";
			return -EIO;
		}

		/* Print IPA convergence state */
		std::cout << "Captured after " << frameCount_ << " frame(s)\n";
		std::cout << "  AWB locked : " << (isAwbLocked() ? "yes" : "no") << "\n";
		std::cout << "  AE locked  : " << (isAeLocked() ? "yes" : "no") << "\n";
		if (colourTemperature() > 0)
			std::cout << "  Colour temp: " << colourTemperature() << " K\n";

		if (printMetadata_)
			printAllMetadata();

		saveFrame(savedBuffer_, filename);

		if (!latestLink_.empty())
			updateLatestLink(filename);

		return 0;
	}

	/*
	 * captureLoop() — run warmup once, then take captureCount_ frames
	 * (or timelapse_ frames) keeping the IPA alive between shots.
	 */
	int captureLoop(const std::string &outputTemplate)
	{
		if (requests_.empty()) {
			std::cerr << "No requests — call init() first\n";
			return -EINVAL;
		}

		libcamera::ControlList startControls(camera_->controls());
		int ret = camera_->start(&startControls);
		if (ret < 0) {
			std::cerr << "Failed to start camera: " << ret << "\n";
			return ret;
		}
		cameraStarted_ = true;

		for (auto &req : requests_)
			camera_->queueRequest(req.get());

		/* Derive adaptive timeout (same logic as captureStill) */
		unsigned int effectiveTimeout = timeoutSec_;
		if (effectiveTimeout == 0) {
			float fps = 30.0f;
			auto &controls = camera_->controls();
			auto it = controls.find(
				&libcamera::controls::FrameDurationLimits);
			if (it != controls.end()) {
				int64_t minDurUs = it->second.min().get<int64_t>();
				if (minDurUs > 0)
					fps = 1.0e6f /
					      static_cast<float>(minDurUs);
			}
			effectiveTimeout = static_cast<unsigned int>(
				(static_cast<float>(maxFrames_) / fps) * 1.5f + 2.0f);
		}

		/* Initial warmup — wait for first convergence */
		std::cout << "Initial warmup (" << warmupFrames_
			  << " frames minimum)...\n";
		{
			std::unique_lock<std::mutex> lock(captureMutex_);
			captureNotify_.wait_for(
				lock,
				std::chrono::seconds(effectiveTimeout),
				[this] { return captureReady_.load(); });
		}

		unsigned int shot = 0;
		unsigned int total = (captureCount_ > 0) ? captureCount_
							 : UINT_MAX;

		while (shot < total && !g_sigTerm.load()) {
			/* Wait for IPA convergence (or timeout) before each shot.
			 * On the first iteration captureReady_ is already set by
			 * the initial warmup above, so this is a no-op.
			 * On subsequent timelapse shots, resetConvergenceState()
			 * cleared captureReady_ at the bottom of the previous
			 * iteration — wait here for re-convergence. */
			if (!captureReady_.load()) {
				std::unique_lock<std::mutex> lock(captureMutex_);
				captureNotify_.wait_for(
					lock,
					std::chrono::seconds(effectiveTimeout),
					[this] { return captureReady_.load(); });
			}

			if (useKeypress_) {
				std::cout << "Press Enter for shot "
					  << (shot + 1) << "...\n";
				std::string dummy;
				std::getline(std::cin, dummy);
			} else if (useSignal_) {
				std::cout << "Waiting SIGUSR1 for shot "
					  << (shot + 1) << "...\n";
				while (!g_sigUsr1.load() && !g_sigTerm.load())
					std::this_thread::sleep_for(50ms);
				g_sigUsr1.store(false);
			}

			if (g_sigTerm.load())
				break;

			if (!savedBuffer_) {
				std::cerr << "No buffer for shot "
					  << (shot + 1) << "\n";
				break;
			}

			std::string fname = makeFilename(outputTemplate, shot);
			std::cout << "Shot " << (shot + 1);
			if (total != UINT_MAX)
				std::cout << "/" << total;
			std::cout << " → " << fname << "\n";
			if (colourTemperature() > 0)
				std::cout << "  CT=" << colourTemperature()
					  << "K\n";

			saveFrame(savedBuffer_, fname);
			if (!latestLink_.empty())
				updateLatestLink(fname);

			shot++;

			/* Reset convergence state for the next shot — both timelapse
			 * and burst need this to clear captureReady_, savedBuffer_,
			 * and frameCount_ so the next iteration converges fresh.
			 * Without reset, burst mode saves the same buffer every shot
			 * and the ISC pipeline stops requeueing. */
			if (shot < total)
				resetConvergenceState();

			/* Timelapse: sleep the inter-shot interval */
			if (timelapseMs_ > 0)
				std::this_thread::sleep_for(
					std::chrono::milliseconds(timelapseMs_));
		}

		stopStreaming();

		return 0;
	}

protected:
	void saveFrame(const libcamera::FrameBuffer *buffer,
		       const std::string &filename) override;

private:
	std::string imageFormat_ = "jpeg";
	int jpegQuality_ = 100;
	int pngCompression_ = 6;
	/*
	 * Per-capture wall-clock timeout in seconds.
	 * 0 = auto: derived from maxFrames_ and the sensor framerate after
	 * init(), giving maxFrames_/fps * 1.5 + 2 seconds margin.
	 * Set explicitly via --timeout to override.
	 */
	unsigned int timeoutSec_ = 0;
	bool printMetadata_ = false;
	bool metadataJson_ = false;
	std::string latestLink_;
	bool useKeypress_ = false;
	bool useSignal_ = false;
	unsigned int captureCount_ = 0; /* 0 = single shot */
	unsigned int timelapseMs_ = 0; /* 0 = not a timelapse */

	void saveJpeg(const uint8_t *data, int w, int h,
		      const std::string &path);
	void savePng(const uint8_t *data, int w, int h,
		     const std::string &path);
	void savePgm(const uint8_t *raw16le, size_t len,
		     unsigned int w, unsigned int h,
		     const std::string &path);

	void printAllMetadata() const;
	void updateLatestLink(const std::string &target) const;
};

/* ------------------------------------------------------------------ */
/* saveFrame                                                            */
/* ------------------------------------------------------------------ */

void MchpCamStill::saveFrame(const libcamera::FrameBuffer *buffer,
			     const std::string &filename)
{
	const libcamera::FrameBuffer::Plane plane = buffer->planes()[0];

	void *mapped = mmap(nullptr, plane.length, PROT_READ,
			    MAP_SHARED, plane.fd.get(), plane.offset);
	if (mapped == MAP_FAILED) {
		std::cerr << "mmap failed: " << strerror(errno) << "\n";
		return;
	}

	const uint8_t *data = static_cast<const uint8_t *>(mapped);

	/*
	 * RAW BUFFER PIXEL DUMP — overexposure debug.
	 *
	 * Sample the centre 100×100 pixel region of the DMA buffer BEFORE any
	 * format conversion or JPEG encoding.  This tells us whether the
	 * overexposure (+96/255 vs v4l2-ctl) is already present in the raw
	 * DMA buffer, or is introduced by the RGB conversion / encode path.
	 *
	 * Compare output with simultaneous:
	 *   v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=1 \
	 *            --stream-to=/tmp/raw.bin
	 * then decode at same pixel coordinates.
	 *
	 * Enabled only when MCHPCAM_PIXEL_DUMP env var is set.
	 */
	if (getenv("MCHPCAM_PIXEL_DUMP")) {
		unsigned int cx = width_ / 2;
		unsigned int cy = height_ / 2;
		unsigned int half = 50; /* 100×100 centre crop */
		unsigned int x0 = (cx > half) ? cx - half : 0;
		unsigned int y0 = (cy > half) ? cy - half : 0;
		unsigned int x1 = std::min(cx + half, width_);
		unsigned int y1 = std::min(cy + half, height_);

		double sumR = 0, sumG = 0, sumB = 0;
		uint64_t n = 0;

		if (pixelFormat_ == libcamera::formats::RGB565) {
			for (unsigned int y = y0; y < y1; ++y) {
				for (unsigned int x = x0; x < x1; ++x) {
					size_t idx = (y * width_ + x) * 2;
					uint16_t px = static_cast<uint16_t>(data[idx]) |
						      static_cast<uint16_t>(data[idx + 1] << 8);
					uint8_t r5 = (px >> 11) & 0x1f;
					uint8_t g6 = (px >> 5) & 0x3f;
					uint8_t b5 = px & 0x1f;
					/* Expand to 8-bit (same as JPEG path) */
					sumR += (r5 << 3) | (r5 >> 2);
					sumG += (g6 << 2) | (g6 >> 4);
					sumB += (b5 << 3) | (b5 >> 2);
					++n;
				}
			}
		} else if (pixelFormat_ == libcamera::formats::YUYV) {
			for (unsigned int y = y0; y < y1; ++y) {
				for (unsigned int x = x0; x < x1; x += 2) {
					size_t idx = (y * width_ + x) * 2;
					sumR += data[idx]; /* Y0 as proxy */
					sumG += data[idx];
					sumB += data[idx];
					++n;
				}
			}
		} else if (pixelFormat_ == libcamera::formats::RGB888 ||
			   pixelFormat_ == libcamera::formats::BGR888) {
			bool bgr = (pixelFormat_ == libcamera::formats::BGR888);
			for (unsigned int y = y0; y < y1; ++y) {
				for (unsigned int x = x0; x < x1; ++x) {
					size_t idx = (y * width_ + x) * 3;
					sumR += bgr ? data[idx + 2] : data[idx + 0];
					sumG += data[idx + 1];
					sumB += bgr ? data[idx + 0] : data[idx + 2];
					++n;
				}
			}
		}

		if (n > 0) {
			double mR = sumR / n;
			double mG = sumG / n;
			double mB = sumB / n;
			double mL = 0.299 * mR + 0.587 * mG + 0.114 * mB;
			std::cerr << "[PIXEL_DUMP] RAW DMA BUFFER (before encode)\n"
				  << "[PIXEL_DUMP]   format : "
				  << pixelFormat_.toString() << "\n"
				  << "[PIXEL_DUMP]   region : centre "
				  << (x1 - x0) << "x" << (y1 - y0) << " pixels\n"
				  << "[PIXEL_DUMP]   mean R : " << mR << " / 255\n"
				  << "[PIXEL_DUMP]   mean G : " << mG << " / 255\n"
				  << "[PIXEL_DUMP]   mean B : " << mB << " / 255\n"
				  << "[PIXEL_DUMP]   mean L : " << mL << " / 255\n"
				  << "[PIXEL_DUMP]   target : 117 / 255 (sRGB 18% grey)\n"
				  << "[PIXEL_DUMP]   gap    : " << (mL - 117.0)
				  << " / 255\n";
		}
	}

	using namespace libcamera;

	/*
	 * RAW Bayer path — SRGGB10 (IMX219 default, RGGB, no flip) packed
	 * into 16-bit LE words from the ISC DMA.
	 * Write a 16-bit binary PGM (P5, maxval 1023) so that numpy,
	 * PIL and the calibration tool can read it with a one-liner.
	 * No RGB conversion; no JPEG/PNG encoding.
	 *
	 * libcamera's pipeline may adjust SRGGB10 to R10 (luma-only fourcc)
	 * when the Bayer variant is not explicitly listed in StillCapture
	 * advertised formats — the pixel data layout is identical (16-bit LE
	 * container, 10-bit sample in [9:0]).  Accept both.
	 */
	if (pixelFormat_ == formats::SRGGB10 || pixelFormat_ == formats::R10) {
		std::string out = filename;
		/* Force .pgm extension */
		auto dot = out.rfind('.');
		if (dot == std::string::npos ||
		    out.substr(dot) != ".pgm")
			out = (dot != std::string::npos ? out.substr(0, dot) : out) + ".pgm";
		savePgm(data, plane.length, width_, height_, out);
		munmap(mapped, plane.length);
		return;
	}

	if (pixelFormat_ != formats::RGB565 &&
	    pixelFormat_ != formats::YUYV &&
	    pixelFormat_ != formats::BGR888 &&
	    pixelFormat_ != formats::RGB888) {
		std::cerr << "Unsupported format for encoding: "
			  << pixelFormat_.toString()
			  << " — choose RGB565 (default), YUYV, BGR888 or RGB888\n";
		munmap(mapped, plane.length);
		return;
	}

	/* Convert to packed RGB888 for JPEG/PNG encoder */
	std::vector<uint8_t> rgb(static_cast<size_t>(width_) * height_ * 3);

	if (pixelFormat_ == formats::RGB888) {
		std::copy(data, data + rgb.size(), rgb.begin());

	} else if (pixelFormat_ == formats::BGR888) {
		for (unsigned int i = 0; i < width_ * height_; ++i) {
			rgb[i * 3 + 0] = data[i * 3 + 2];
			rgb[i * 3 + 1] = data[i * 3 + 1];
			rgb[i * 3 + 2] = data[i * 3 + 0];
		}

	} else if (pixelFormat_ == formats::RGB565) {
		for (unsigned int i = 0; i < width_ * height_; ++i) {
			/*
			 * ISC outputs RGB565 little-endian:
			 * byte0 = GGGBBBBB (low byte)
			 * byte1 = RRRRRGGG (high byte)
			 */
			uint16_t px = static_cast<uint16_t>(data[i * 2]) |
				      static_cast<uint16_t>(data[i * 2 + 1] << 8);
			uint8_t r = (px >> 11) & 0x1f;
			uint8_t g = (px >> 5) & 0x3f;
			uint8_t b = px & 0x1f;
			rgb[i * 3 + 0] = (r << 3) | (r >> 2);
			rgb[i * 3 + 1] = (g << 2) | (g >> 4);
			rgb[i * 3 + 2] = (b << 3) | (b >> 2);
		}

	} else { /* YUYV — packed 4:2:2 */
		auto clamp8 = [](float v) -> uint8_t {
			return static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
		};
		for (unsigned int row = 0; row < height_; ++row) {
			for (unsigned int col = 0; col < width_; col += 2) {
				size_t yuvIdx = (row * width_ + col) * 2;
				int y0 = data[yuvIdx + 0];
				int u = data[yuvIdx + 1] - 128;
				int y1 = data[yuvIdx + 2];
				int v = data[yuvIdx + 3] - 128;

				size_t rgbIdx = (row * width_ + col) * 3;
				rgb[rgbIdx + 0] = clamp8(y0 + 1.402f * v);
				rgb[rgbIdx + 1] = clamp8(y0 - 0.344f * u - 0.714f * v);
				rgb[rgbIdx + 2] = clamp8(y0 + 1.772f * u);
				rgb[rgbIdx + 3] = clamp8(y1 + 1.402f * v);
				rgb[rgbIdx + 4] = clamp8(y1 - 0.344f * u - 0.714f * v);
				rgb[rgbIdx + 5] = clamp8(y1 + 1.772f * u);
			}
		}
	}

	munmap(mapped, plane.length);

	/*
	 * Software transform fallback.
	 * The libcamera orientation path handles hflip/vflip at the sensor
	 * level for supported sensors (IMX219 has HW flip registers).
	 * Apply software transform only when the pipeline adjusted away the
	 * requested orientation (orientation_ != Normal as set in init()),
	 * which means the HW path was not taken.
	 */
	if (orientation_ == MchpOrientation::HFlip ||
	    orientation_ == MchpOrientation::Rotate180)
		applyHflip(rgb, width_, height_);
	if (orientation_ == MchpOrientation::VFlip ||
	    orientation_ == MchpOrientation::Rotate180)
		applyVflip(rgb, width_, height_);

	/* Derive output format from filename extension */
	auto ext = [](const std::string &s) -> std::string {
		auto p = s.rfind('.');
		return (p != std::string::npos) ? s.substr(p + 1) : "";
	};

	std::string out = filename;
	std::string fmt = imageFormat_;
	if (fmt.empty() || fmt == "auto") {
		std::string e = ext(filename);
		std::transform(e.begin(), e.end(), e.begin(), ::tolower);
		if (e == "png")
			fmt = "png";
		else if (e == "jpg" || e == "jpeg")
			fmt = "jpeg";
		else
			fmt = "jpeg";
	}

	if (fmt == "png") {
		if (ext(out) != "png")
			out += ".png";
		savePng(rgb.data(), static_cast<int>(width_),
			static_cast<int>(height_), out);
	} else {
		if (ext(out) != "jpg" && ext(out) != "jpeg")
			out += ".jpg";
		saveJpeg(rgb.data(), static_cast<int>(width_),
			 static_cast<int>(height_), out);
	}
}

/* ------------------------------------------------------------------ */
/* Encoders                                                             */
/* ------------------------------------------------------------------ */

/*
 * Note on raw / Bayer capture (--raw flag):
 * Note on raw / Bayer capture (--raw flag):
 * IMX219 default output: SRGGB10_1X10 (RGGB, no flip).
 * The ISC DMA path keeps WB_ENABLE active even for Bayer output
 * (bits_pipeline = WB_ENABLE — confirmed in isc_try_configure_pipeline
 * default: case, kernel driver).  mchpcam-still --raw forces unity WB
 * gains (512 = 1.0× in ISC Q9 scale) so the WB block is a mathematical
 * no-op.  The SRGGB10 pixels in the PGM are therefore equivalent to the
 * true pre-WB sensor data.
 *
 * The ISC histogram tap is on vp_data (before WB) per datasheet §43.6.20
 * Figure 43-38 — AE converges identically in RAW and JPEG modes.
 */

/*
 * Custom libjpeg error manager that uses setjmp/longjmp instead of exit().
 * The default jpeg_std_error error_exit calls exit(), which:
 *   - bypasses all C++ destructors (resource leak)
 *   - leaves a partial JPEG file on disk
 *   - prevents the caller from reporting a clean error
 *
 * This handler jumps back to the setjmp point in saveJpeg() so that the
 * file handle and compressor object can be cleaned up properly.
 * Pattern taken from the libjpeg example.c "Advanced JPEG use" section.
 */
struct MchpJpegErrorMgr {
	struct jpeg_error_mgr pub;
	jmp_buf jmpBuf;
};

static void mchpJpegErrorExit(j_common_ptr cinfo)
{
	MchpJpegErrorMgr *mgr = reinterpret_cast<MchpJpegErrorMgr *>(cinfo->err);
	/* Print the libjpeg error message to stderr before jumping */
	(*cinfo->err->output_message)(cinfo);
	longjmp(mgr->jmpBuf, 1);
}

void MchpCamStill::saveJpeg(const uint8_t *data, int w, int h,
			    const std::string &path)
{
	FILE *f = fopen(path.c_str(), "wb");
	if (!f) {
		std::cerr << "Cannot open " << path << ": "
			  << strerror(errno) << "\n";
		return;
	}

	struct jpeg_compress_struct cinfo;
	MchpJpegErrorMgr jerr;
	cinfo.err = jpeg_std_error(&jerr.pub);
	jerr.pub.error_exit = mchpJpegErrorExit;

	/*
	 * If libjpeg hits a fatal error it calls mchpJpegErrorExit() which
	 * longjmps here. Clean up the compressor and file, remove the partial
	 * output, and return without crashing the process.
	 */
	if (setjmp(jerr.jmpBuf)) {
		jpeg_destroy_compress(&cinfo);
		fclose(f);
		std::remove(path.c_str());
		std::cerr << "JPEG encoding failed for " << path << "\n";
		return;
	}

	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, f);

	cinfo.image_width = static_cast<JDIMENSION>(w);
	cinfo.image_height = static_cast<JDIMENSION>(h);
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, jpegQuality_, TRUE);
	jpeg_start_compress(&cinfo, TRUE);

	while (cinfo.next_scanline < cinfo.image_height) {
		JSAMPROW row = const_cast<uint8_t *>(
			&data[cinfo.next_scanline * w * 3]);
		jpeg_write_scanlines(&cinfo, &row, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	fclose(f);

	std::cout << "Saved JPEG: " << path << "\n";
}

void MchpCamStill::savePng(const uint8_t *data, int w, int h,
			   const std::string &path)
{
	FILE *f = fopen(path.c_str(), "wb");
	if (!f) {
		std::cerr << "Cannot open " << path << ": "
			  << strerror(errno) << "\n";
		return;
	}

	png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING,
						  nullptr, nullptr, nullptr);
	if (!png) {
		fclose(f);
		return;
	}

	png_infop info = png_create_info_struct(png);
	if (!info) {
		png_destroy_write_struct(&png, nullptr);
		fclose(f);
		return;
	}

	if (setjmp(png_jmpbuf(png))) {
		std::cerr << "PNG write error\n";
		png_destroy_write_struct(&png, &info);
		fclose(f);
		return;
	}

	png_init_io(png, f);
	png_set_compression_level(png, pngCompression_);
	png_set_IHDR(png, info, static_cast<png_uint_32>(w),
		     static_cast<png_uint_32>(h), 8, PNG_COLOR_TYPE_RGB,
		     PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
		     PNG_FILTER_TYPE_DEFAULT);

	char keyBuf[] = "Software";
	char textBuf[] = "mchpcam-still / Microchip ISC";
	png_text txt;
	txt.key = keyBuf;
	txt.text = textBuf;
	txt.compression = PNG_TEXT_COMPRESSION_NONE;
	png_set_text(png, info, &txt, 1);

	png_write_info(png, info);
	for (int row = 0; row < h; ++row)
		png_write_row(png,
			      const_cast<png_bytep>(&data[row * w * 3]));
	png_write_end(png, nullptr);
	png_destroy_write_struct(&png, &info);
	fclose(f);

	std::cout << "Saved PNG: " << path
		  << " (compression " << pngCompression_ << ")\n";
}

/* ------------------------------------------------------------------ */
/* savePgm — 16-bit binary PGM for RAW Bayer output (SRGGB10/RGGB)    */
/* ------------------------------------------------------------------ */

/*
 * The ISC DMA outputs 10-bit Bayer pixels packed into 16-bit little-endian
 * words (ISC_RLP_CFG_MODE_DAT10 + ISC_DCFG_IMODE_PACKED16).  The 10-bit
 * sample is in bits [9:0]; bits [15:10] are zero.
 *
 * PGM P5 (binary) format:
 *   - Header:  "P5\n<width> <height>\n<maxval>\n"   (ASCII)
 *   - Pixels:  width × height × 2 bytes, big-endian  (binary)
 *   - maxval:  1023 (2^10 − 1)
 *
 * Each 16-bit LE word from ISC must be byte-swapped to big-endian for PGM.
 * The Bayer pattern is RGGB (IMX219 default, no flip).
 *
 * Reading in Python:
 *   import numpy as np
 *   raw = np.fromfile('frame.pgm', dtype='>u2', offset=<hdr_len>)
 *   bayer = raw.reshape(height, width)   # 10-bit RGGB
 */
void MchpCamStill::savePgm(const uint8_t *raw16le, size_t len,
			   unsigned int w, unsigned int h,
			   const std::string &path)
{
	FILE *f = fopen(path.c_str(), "wb");
	if (!f) {
		std::cerr << "Cannot open " << path << ": "
			  << strerror(errno) << "\n";
		return;
	}

	/* ASCII PGM header */
	fprintf(f, "P5\n%u %u\n1023\n", w, h);

	/*
	 * Convert each 16-bit LE pixel to 16-bit BE for PGM.
	 * Process in 4096-sample chunks to keep stack usage small.
	 */
	const size_t nPixels = static_cast<size_t>(w) * h;
	const size_t expectedBytes = nPixels * 2;
	if (len < expectedBytes) {
		std::cerr << "savePgm: buffer too small ("
			  << len << " < " << expectedBytes << ")\n";
		fclose(f);
		return;
	}

	constexpr size_t kChunk = 4096;
	std::vector<uint8_t> buf(kChunk * 2);
	const uint8_t *src = raw16le;
	size_t remaining = nPixels;

	while (remaining > 0) {
		size_t n = std::min(remaining, kChunk);
		for (size_t i = 0; i < n; ++i) {
			/* Read LE uint16 — low byte first */
			uint16_t px = static_cast<uint16_t>(src[i * 2]) |
				      static_cast<uint16_t>(src[i * 2 + 1] << 8);
			/* Write BE: high byte first */
			buf[i * 2] = static_cast<uint8_t>(px >> 8);
			buf[i * 2 + 1] = static_cast<uint8_t>(px & 0xff);
		}
		fwrite(buf.data(), 2, n, f);
		src += n * 2;
		remaining -= n;
	}

	fclose(f);
	std::cout << "Saved PGM: " << path
		  << " (" << w << "x" << h << " SRGGB10 RGGB)\n";
}

/* ------------------------------------------------------------------ */
/* Metadata helpers                                                     */
/* ------------------------------------------------------------------ */

void MchpCamStill::printAllMetadata() const
{
	for (const auto &req : requests_) {
		bool found = false;
		for (const auto &[st, buf] : req->buffers()) {
			if (buf == savedBuffer_) {
				found = true;
				break;
			}
		}
		if (!found)
			continue;

		/*
		 * The global controls::controls ControlIdMap includes both
		 * standard and vendor-namespace controls (the code generator
		 * in control_ids.cpp.in iterates all vendors into the single
		 * map).  Use find() instead of at() to gracefully skip any
		 * unknown IDs rather than throwing std::out_of_range.
		 */
		auto lookupId = [](unsigned int id) -> const libcamera::ControlId * {
			auto it = libcamera::controls::controls.find(id);
			return it != libcamera::controls::controls.end()
				       ? it->second
				       : nullptr;
		};

		if (metadataJson_) {
			std::cout << "{\n";
			bool first = true;
			for (const auto &[id, val] : req->metadata()) {
				const libcamera::ControlId *cid = lookupId(id);
				if (!cid)
					continue; /* skip truly unknown IDs */
				if (!first)
					std::cout << ",\n";
				first = false;

				/*
				 * Emit proper JSON types — not quoted strings.
				 * ControlValue::toString() returns a human-readable
				 * representation (e.g. "[ 1.23, 4.56 ]" for arrays,
				 * "true"/"false" for booleans) which is not valid JSON
				 * for float arrays.  Emit the raw toString() value
				 * without outer quotes so the Python json.loads()
				 * parser sees the correct type.  toString() for scalars
				 * gives a plain number without quotes; for arrays it
				 * gives "[ v0, v1, ... ]" which Python json.loads()
				 * accepts as a JSON array.  Boolean "true"/"false" are
				 * valid JSON literals.  String-typed controls are
				 * the exception — those need quoting; handle by checking
				 * the control type.
				 */
				std::cout << "  \"" << cid->name() << "\": ";
				if (cid->type() == libcamera::ControlTypeString)
					std::cout << "\"" << val.toString() << "\"";
				else
					std::cout << val.toString();
			}
			std::cout << "\n}\n";
		} else {
			std::cout << "Frame metadata:\n";
			for (const auto &[id, val] : req->metadata()) {
				const libcamera::ControlId *cid = lookupId(id);
				if (!cid)
					continue;
				std::cout << "  " << std::left
					  << std::setw(36) << cid->name()
					  << " " << val.toString() << "\n";
			}
		}
		break;
	}
}

void MchpCamStill::updateLatestLink(const std::string &target) const
{
	/* Create/update a symlink to the latest captured file */
	::unlink(latestLink_.c_str());
	if (::symlink(target.c_str(), latestLink_.c_str()) < 0)
		std::cerr << "Warning: cannot create symlink "
			  << latestLink_ << ": " << strerror(errno) << "\n";
}

/* ------------------------------------------------------------------ */
/* Usage                                                                */
/* ------------------------------------------------------------------ */

static void printUsage(const char *argv0)
{
	std::cout
		<< "Usage: " << argv0 << " [options]\n"
					 "\n"
					 "Discovery:\n"
					 "  --list-cameras             List all available cameras and exit\n"
					 "  --list-controls            List controls supported by the camera and exit\n"
					 "  --list-properties          List camera properties and exit\n"
					 "  --version                  Print version string and exit\n"
					 "\n"
					 "Camera selection:\n"
					 "  -c, --camera=ID            Camera ID or index (default: first available)\n"
					 "\n"
					 "Output:\n"
					 "  -o, --output=FILE          Output file. Supports tokens:\n"
					 "                               %d = counter (0001, 0002, ...)\n"
					 "                               %t = Unix epoch seconds\n"
					 "                               %T = MMDDHHmmSS timestamp\n"
					 "  --image-format=FORMAT      jpeg | png (default: from extension)\n"
					 "  --jpeg-quality=1-100       JPEG quality (default: 100)\n"
					 "  --png-compression=0-9      PNG compression level (default: 6)\n"
					 "  --latest=LINK              Create/update symlink to latest capture\n"
					 "  --metadata                 Print all frame metadata after capture\n"
					 "  --metadata-format=FMT      json | txt (default: txt)\n"
					 "\n"
					 "Stream:\n"
					 "  -f, --format=FMT           RGB565 | YUYV | BGR888 | RGB888\n"
					 "                             (default: RGB565)\n"
					 "  --raw                      Capture 10-bit Bayer (SRGGB10/RGGB) as 16-bit PGM.\n"
					 "                             Forces unity WB gains (512) so the ISC WB\n"
					 "                             block is a mathematical no-op (datasheet\n"
					 "                             §43.6.20: histogram taps vp_data before WB;\n"
					 "                             AE converges normally). AWB is disabled.\n"
					 "                             Output: <file>.pgm  maxval=1023  Bayer GRBG.\n"
					 "                             Use for CCM calibration with microchip_isc.py.\n"
					 "  --width=N                  Capture width  (default: 1920)\n"
					 "  --height=N                 Capture height (default: 1080)\n"
					 "\n"
					 "Capture modes:\n"
					 "  --timeout=SECONDS          Per-capture timeout. Default: auto "
					 "(maxFrames/fps * 1.5 + 2 s)\n"
					 "  --capture=N                Capture N frames (burst or timelapse)\n"
					 "  --timelapse=MS             Timelapse: repeat every MS milliseconds\n"
					 "                             Keeps IPA warm between shots.\n"
					 "                             Use with --capture=N or Ctrl+C to stop.\n"
					 "  -k, --keypress             Wait for Enter keypress before each capture\n"
					 "  -s, --signal               Wait for SIGUSR1 before each capture\n"
					 "                             (enables scripted/remote-triggered capture)\n"
					 "\n"
					 "Image controls:\n"
					 "  --brightness=F             [-1.0, +1.0], neutral 0.0 (default)\n"
					 "  --contrast=F               [ 0.0, 32.0], neutral 1.0 (default)\n"
					 "  --gamma=F                  Display gamma exponent (default: 2.2)\n"
					 "  --saturation=F             [ 0.0,  2.0], neutral 1.0 (default)\n"
					 "  --hue=F                    Hue rotation degrees [-180, +180], 0 (default)\n"
					 "  --sharpness=F              No-op: ISC has no hardware sharpening\n"
					 "\n"
					 "Exposure:\n"
					 "  --ev=F                     EV compensation [-2.0, +2.0] stops (default: 0)\n"
					 "  --metering=MODE            centre | spot | matrix (default: centre)\n"
					 "  --shutter=US               Manual exposure time in microseconds\n"
					 "  --analoggain=F             Manual analogue gain, 1.0=unity (disables AGC)\n"
					 "  --digitalgain=F            Manual digital gain 1.0–16.0\n"
					 "  --framerate=F              Lock output framerate (e.g. 30.0, 15.0)\n"
					 "  --exposure-time-mode=MODE  auto | manual — lock/free shutter independently\n"
					 "  --gain-mode=MODE           auto | manual — lock/free gain independently\n"
					 "                             e.g. fix gain=1x, let AE choose exposure:\n"
					 "                               --gain-mode=manual --analoggain=1.0\n"
					 "  --no-agc                   Disable both AE and AG (--exposure-time-mode=manual\n"
					 "                             --gain-mode=manual combined)\n"
					 "\n"
					 "White balance:\n"
					 "  --awb=MODE                 AWB mode preset:\n"
					 "                               auto         — IPA auto (default)\n"
					 "                               incandescent — ~2800K tungsten\n"
					 "                               tungsten     — ~3200K (alias)\n"
					 "                               fluorescent  — ~4000K cool white\n"
					 "                               indoor       — ~3800K warm LED\n"
					 "                               daylight     — ~5500K D65\n"
					 "                               cloudy       — ~6500K overcast\n"
					 "                               shade        — ~7500K open shade\n"
					 "  --no-awb                   Disable IPA Auto White Balance\n"
					 "  --awbgains=R,B         IPA manual WB gains (disables AWB)\n"
					 "  --red-gain=N               ISC hardware WB R gain 0-8191 (512=1×)\n"
					 "  --blue-gain=N              ISC hardware WB B gain 0-8191\n"
					 "  --green-red-gain=N         ISC hardware WB GR gain 0-8191\n"
					 "  --green-blue-gain=N        ISC hardware WB GB gain 0-8191\n"
					 "  --red-offset=N             ISC hardware WB R offset -4095..4095\n"
					 "  --blue-offset=N            ISC hardware WB B offset\n"
					 "  --green-red-offset=N       ISC hardware WB GR offset\n"
					 "  --green-blue-offset=N      ISC hardware WB GB offset\n"
					 "  Note: hardware register options bypass the IPA. Use --no-awb.\n"
					 "\n"
					 "Colour correction:\n"
					 "  --no-ccm                   Disable IPA Colour Correction Matrix\n"
					 "  --ccm=a,b,c,d,e,f,g,h,i   Manual CCM: 9 floats, row-major RGB→RGB\n"
					 "                             e.g. identity: 1,0,0,0,1,0,0,0,1\n"
					 "\n"
					 "Transform:\n"
					 "  --hflip                    Horizontal mirror (IMX219 hardware HW flip)\n"
					 "  --vflip                    Vertical flip (IMX219 hardware VFlip)\n"
					 "  --rotation=0|180           Image rotation (0 or 180 degrees)\n"
					 "\n"
					 "Algorithm enable/disable:\n"
					 "  --enable-all               Enable AWB + AGC + CCM\n"
					 "  --disable-all              Disable AWB + AGC + CCM\n"
					 "\n"
					 "Warmup:\n"
					 "  --warmup=N                 Minimum frames before saving (default: 12)\n"
					 "\n"
					 "  --noir                     Use imx219_noir.yaml tuning file\n"
					 "                             (sensor without IR-cut filter)\n"
					 "  --tuning-file=PATH         Override tuning YAML path\n"
					 "\n"
					 "  -h, --help                 Print this help\n";
}

/* ------------------------------------------------------------------ */
/* main()                                                               */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
	MchpCamStill cam;

	signal(SIGINT, onSigTerm);
	signal(SIGTERM, onSigTerm);
	signal(SIGHUP, onSigTerm); /* SSH disconnect — treat as graceful stop */
	signal(SIGUSR1, onSigUsr1);

	static const struct option longOpts[] = {
		{ "camera", required_argument, nullptr, 'c' },
		{ "output", required_argument, nullptr, 'o' },
		{ "format", required_argument, nullptr, 'f' },
		{ "help", no_argument, nullptr, 'h' },
		{ "keypress", no_argument, nullptr, 'k' },
		{ "signal", no_argument, nullptr, 's' },
		/* Discovery */
		{ "list-cameras", no_argument, nullptr, 0 },
		{ "list-controls", no_argument, nullptr, 0 },
		{ "list-properties", no_argument, nullptr, 0 },
		{ "version", no_argument, nullptr, 0 },
		/* Capture */
		{ "timeout", required_argument, nullptr, 0 },
		{ "capture", required_argument, nullptr, 0 },
		{ "timelapse", required_argument, nullptr, 0 },
		{ "metadata", no_argument, nullptr, 0 },
		{ "metadata-format", required_argument, nullptr, 0 },
		{ "latest", required_argument, nullptr, 0 },
		{ "width", required_argument, nullptr, 0 },
		{ "height", required_argument, nullptr, 0 },
		{ "image-format", required_argument, nullptr, 0 },
		{ "jpeg-quality", required_argument, nullptr, 0 },
		{ "png-compression", required_argument, nullptr, 0 },
		{ "noir", no_argument, nullptr, 0 },
		{ "tuning-file", required_argument, nullptr, 0 },
		{ "raw", no_argument, nullptr, 0 },
		/* Image controls */
		{ "brightness", required_argument, nullptr, 0 },
		{ "contrast", required_argument, nullptr, 0 },
		{ "gamma", required_argument, nullptr, 0 },
		{ "saturation", required_argument, nullptr, 0 },
		{ "hue", required_argument, nullptr, 0 },
		{ "sharpness", required_argument, nullptr, 0 },
		/* Exposure */
		{ "ev", required_argument, nullptr, 0 },
		{ "metering", required_argument, nullptr, 0 },
		{ "shutter", required_argument, nullptr, 0 },
		{ "analoggain", required_argument, nullptr, 0 },
		{ "digitalgain", required_argument, nullptr, 0 },
		{ "framerate", required_argument, nullptr, 0 },
		/*
		 * Independent AE/AG mode control — mirrors rpicam-apps
		 * --exposure-time-mode and --gain-mode.
		 * Values: auto | manual
		 * Allows locking gain while leaving exposure auto, or vice versa.
		 */
		{ "exposure-time-mode", required_argument, nullptr, 0 },
		{ "gain-mode", required_argument, nullptr, 0 },
		/* AWB */
		{ "awb", required_argument, nullptr, 0 },
		{ "no-awb", no_argument, nullptr, 0 },
		{ "awbgains", required_argument, nullptr, 0 },
		/* Hardware WB register bypass */
		{ "red-gain", required_argument, nullptr, 0 },
		{ "blue-gain", required_argument, nullptr, 0 },
		{ "green-red-gain", required_argument, nullptr, 0 },
		{ "green-blue-gain", required_argument, nullptr, 0 },
		{ "red-offset", required_argument, nullptr, 0 },
		{ "blue-offset", required_argument, nullptr, 0 },
		{ "green-red-offset", required_argument, nullptr, 0 },
		{ "green-blue-offset", required_argument, nullptr, 0 },
		/* AGC */
		{ "no-agc", no_argument, nullptr, 0 },
		/* CCM */
		{ "no-ccm", no_argument, nullptr, 0 },
		{ "ccm", required_argument, nullptr, 0 },
		/* Transforms */
		{ "hflip", no_argument, nullptr, 0 },
		{ "vflip", no_argument, nullptr, 0 },
		{ "rotation", required_argument, nullptr, 0 },
		/* Combined */
		{ "enable-all", no_argument, nullptr, 0 },
		{ "disable-all", no_argument, nullptr, 0 },
		/* Backward-compat aliases */
		{ "disable-awb", no_argument, nullptr, 0 },
		{ "disable-agc", no_argument, nullptr, 0 },
		{ "disable-ccm", no_argument, nullptr, 0 },
		{ "enable-awb", no_argument, nullptr, 0 },
		{ "enable-agc", no_argument, nullptr, 0 },
		{ "enable-ccm", no_argument, nullptr, 0 },
		/* Warmup */
		{ "warmup", required_argument, nullptr, 0 },
		{ nullptr, 0, nullptr, 0 }
	};

	std::string cameraId;
	std::string output = "output.jpg";
	unsigned int width = 0, height = 0;
	unsigned int captureN = 0;
	unsigned int timelapse = 0;
	HwGainParams hwGains;
	bool listCameras = false;
	bool listControls = false;
	bool listProperties = false;
	bool printVersion = false;

	int opt, optIdx = 0;
	while ((opt = getopt_long(argc, argv, "c:o:f:hks",
				  longOpts, &optIdx)) != -1) {
		/*
		 * The numeric options below parse optarg with std::stoi/stof,
		 * which throw std::invalid_argument or std::out_of_range on a
		 * malformed value.  Catch here so a bad argument reports a clear
		 * usage error instead of terminating via an uncaught exception.
		 */
		try {
		switch (opt) {
		case 'c':
			cameraId = optarg;
			break;
		case 'o':
			output = optarg;
			break;
		case 'f':
			cam.setFormat(optarg);
			break;
		case 'h':
			printUsage(argv[0]);
			return 0;
		case 'k':
			cam.setKeypress(true);
			break;
		case 's':
			cam.setSignalTrigger(true);
			break;

		case 0: {
			const char *name = longOpts[optIdx].name;

			if (!strcmp(name, "list-cameras"))
				listCameras = true;
			else if (!strcmp(name, "list-controls"))
				listControls = true;
			else if (!strcmp(name, "list-properties"))
				listProperties = true;
			else if (!strcmp(name, "version"))
				printVersion = true;
			else if (!strcmp(name, "timeout"))
				cam.setTimeout(static_cast<unsigned int>(std::stoi(optarg)));
			else if (!strcmp(name, "capture"))
				captureN = static_cast<unsigned int>(std::stoi(optarg));
			else if (!strcmp(name, "timelapse"))
				timelapse = static_cast<unsigned int>(std::stoi(optarg));
			else if (!strcmp(name, "metadata"))
				cam.setPrintMetadata(true);
			else if (!strcmp(name, "metadata-format")) {
				if (!strcmp(optarg, "json"))
					cam.setMetadataJson(true);
			} else if (!strcmp(name, "latest"))
				cam.setLatestLink(optarg);
			else if (!strcmp(name, "width"))
				width = static_cast<unsigned int>(std::stoi(optarg));
			else if (!strcmp(name, "height"))
				height = static_cast<unsigned int>(std::stoi(optarg));
			else if (!strcmp(name, "image-format"))
				cam.setImageFormat(optarg);
			else if (!strcmp(name, "jpeg-quality"))
				cam.setJpegQuality(std::stoi(optarg));
			else if (!strcmp(name, "png-compression"))
				cam.setPngCompression(std::stoi(optarg));
			else if (!strcmp(name, "noir"))
				setenv("LIBCAMERA_MICROCHIP-ISC_TUNING_FILE",
				       "imx219_noir.yaml", 1);
			else if (!strcmp(name, "raw"))
				cam.setRawCapture(true);
			else if (!strcmp(name, "tuning-file"))
				setenv("LIBCAMERA_MICROCHIP-ISC_TUNING_FILE",
				       optarg, 1);

			/* Image controls */
			else if (!strcmp(name, "brightness"))
				cam.setBrightness(std::stof(optarg));
			else if (!strcmp(name, "contrast"))
				cam.setContrast(std::stof(optarg));
			else if (!strcmp(name, "gamma"))
				cam.setGamma(std::stof(optarg));
			else if (!strcmp(name, "saturation"))
				cam.setSaturation(std::stof(optarg));
			else if (!strcmp(name, "hue"))
				cam.setHue(std::stof(optarg));
			else if (!strcmp(name, "sharpness"))
				cam.setSharpness(std::stof(optarg));

			/* Exposure */
			else if (!strcmp(name, "ev"))
				cam.setExposureValue(std::stof(optarg));
			else if (!strcmp(name, "metering")) {
				std::string m = optarg;
				if (m == "centre" || m == "center")
					cam.setMeteringMode(0);
				else if (m == "spot")
					cam.setMeteringMode(1);
				else if (m == "matrix")
					cam.setMeteringMode(2);
				else
					std::cerr << "Unknown metering: " << m
						  << ". Options: centre spot matrix\n";
			} else if (!strcmp(name, "shutter"))
				cam.setManualExposure(std::stoi(optarg));
			else if (!strcmp(name, "analoggain"))
				cam.setManualGain(std::stof(optarg));
			else if (!strcmp(name, "digitalgain"))
				cam.setDigitalGain(std::stof(optarg));
			else if (!strcmp(name, "framerate"))
				cam.setFramerate(std::stof(optarg));
			/*
			 * Independent exposure time / gain mode control.
			 * "auto"   → IPA controls this knob (default)
			 * "manual" → fix this knob at the current value,
			 *            let the other continue auto-adjusting.
			 *
			 * Example: lock gain at 1x, let AE choose exposure:
			 *   --gain-mode=manual --analoggain=1.0
			 * Example: lock exposure at 20ms, let AGC pick gain:
			 *   --exposure-time-mode=manual --shutter=20000
			 *
			 * Maps to controls::ExposureTimeMode and
			 * controls::AnalogueGainMode (0=auto, 1=manual).
			 */
			else if (!strcmp(name, "exposure-time-mode")) {
				std::string m = optarg;
				if (m == "manual")
					cam.setExposureTimeMode(1);
				else if (m == "auto")
					cam.setExposureTimeMode(0);
				else
					std::cerr << "--exposure-time-mode: "
						     "auto or manual\n";
			} else if (!strcmp(name, "gain-mode")) {
				std::string m = optarg;
				if (m == "manual")
					cam.setGainMode(1);
				else if (m == "auto")
					cam.setGainMode(0);
				else
					std::cerr << "--gain-mode: "
						     "auto or manual\n";
			}

			/* AWB */
			else if (!strcmp(name, "awb"))
				cam.setAwbMode(optarg);
			else if (!strcmp(name, "no-awb") ||
				 !strcmp(name, "disable-awb"))
				cam.setAwbEnabled(false);
			else if (!strcmp(name, "enable-awb"))
				cam.setAwbEnabled(true);
			else if (!strcmp(name, "awbgains")) {
				std::string s = optarg;
				auto comma = s.find(',');
				if (comma == std::string::npos) {
					std::cerr << "--awbgains needs R,B\n";
				} else {
					float r = std::stof(s.substr(0, comma));
					float b = std::stof(s.substr(comma + 1));
					cam.setColourGains(r, b);
				}
			}

			/* HW WB register bypass */
			else if (!strcmp(name, "red-gain"))
				hwGains.redGain = std::stoi(optarg);
			else if (!strcmp(name, "blue-gain"))
				hwGains.blueGain = std::stoi(optarg);
			else if (!strcmp(name, "green-red-gain"))
				hwGains.greenRedGain = std::stoi(optarg);
			else if (!strcmp(name, "green-blue-gain"))
				hwGains.greenBlueGain = std::stoi(optarg);
			else if (!strcmp(name, "red-offset"))
				hwGains.redOffset = std::stoi(optarg);
			else if (!strcmp(name, "blue-offset"))
				hwGains.blueOffset = std::stoi(optarg);
			else if (!strcmp(name, "green-red-offset"))
				hwGains.greenRedOffset = std::stoi(optarg);
			else if (!strcmp(name, "green-blue-offset"))
				hwGains.greenBlueOffset = std::stoi(optarg);

			/* AGC */
			else if (!strcmp(name, "no-agc") ||
				 !strcmp(name, "disable-agc"))
				cam.setAgcEnabled(false);
			else if (!strcmp(name, "enable-agc"))
				cam.setAgcEnabled(true);

			/* CCM */
			else if (!strcmp(name, "no-ccm") ||
				 !strcmp(name, "disable-ccm"))
				cam.setCcmEnabled(false);
			else if (!strcmp(name, "enable-ccm"))
				cam.setCcmEnabled(true);
			else if (!strcmp(name, "ccm")) {
				/* Parse 9 comma-separated floats */
				std::vector<float> m;
				std::string s = optarg;
				size_t pos = 0;
				while (pos < s.size() && m.size() < 9) {
					size_t next = s.find(',', pos);
					m.push_back(std::stof(
						s.substr(pos, next - pos)));
					pos = (next == std::string::npos)
						      ? s.size()
						      : next + 1;
				}
				if (m.size() == 9)
					cam.setCcmOverride(m);
				else
					std::cerr << "--ccm needs exactly 9 "
						     "comma-separated floats\n";
			}

			/* Transforms */
			else if (!strcmp(name, "hflip"))
				cam.setOrientation(MchpOrientation::HFlip);
			else if (!strcmp(name, "vflip"))
				cam.setOrientation(MchpOrientation::VFlip);
			else if (!strcmp(name, "rotation")) {
				int deg = std::stoi(optarg);
				if (deg == 180)
					cam.setOrientation(
						MchpOrientation::Rotate180);
				else if (deg != 0)
					std::cerr << "--rotation only supports "
						     "0 or 180\n";
			}

			/* Combined */
			else if (!strcmp(name, "enable-all")) {
				cam.setAwbEnabled(true);
				cam.setAgcEnabled(true);
				cam.setCcmEnabled(true);
			} else if (!strcmp(name, "disable-all")) {
				cam.setAwbEnabled(false);
				cam.setAgcEnabled(false);
				cam.setCcmEnabled(false);
			}

			/* Warmup */
			else if (!strcmp(name, "warmup"))
				cam.setWarmupFrames(
					static_cast<unsigned int>(
						std::stoi(optarg)));
			break;
		}

		default:
			printUsage(argv[0]);
			return 1;
		}
		} catch (const std::exception &e) {
			std::string optName = opt ? std::string(1, static_cast<char>(opt))
						  : std::string(longOpts[optIdx].name);
			std::cerr << "Invalid argument for option '" << optName
				  << "': " << (optarg ? optarg : "")
				  << " (" << e.what() << ")\n";
			return 1;
		}
	}

	/* -- Version --------------------------------------------------- */
	if (printVersion) {
		std::cout << "mchpcam-still " << libcamera::CameraManager::version()
			  << " (Microchip ISC / libcamera)\n";
		return 0;
	}

	/* -- Discovery commands ---------------------------------------- */
	if (listCameras || listControls || listProperties) {
		libcamera::CameraManager cm2;
		if (cm2.start() < 0) {
			std::cerr << "Failed to start camera manager\n";
			return 1;
		}

		if (listCameras) {
			std::cout << "Available cameras:\n";
			unsigned int idx = 1;
			for (const auto &c : cm2.cameras()) {
				const auto &props = c->properties();
				std::string name;
				const auto &model =
					props.get(libcamera::properties::Model);
				if (model.has_value())
					name = std::string(*model) + " ";
				name += "(" + c->id() + ")";
				std::cout << "  " << idx++ << ": " << name << "\n";
			}
		}

		if (listControls || listProperties) {
			std::shared_ptr<libcamera::Camera> c;
			if (cameraId.empty()) {
				auto cameras = cm2.cameras();
				if (!cameras.empty())
					c = cameras[0];
			} else {
				c = cm2.get(cameraId);
			}

			if (!c) {
				std::cerr << "No camera found\n";
				cm2.stop();
				return 1;
			}
			c->acquire();

			if (listControls) {
				std::cout << "Controls for camera "
					  << c->id() << ":\n";
				for (const auto &[id, info] : c->controls()) {
					std::cout << "  " << std::left
						  << std::setw(32) << id->name()
						  << " " << info.toString()
						  << "\n";
				}
			}

			if (listProperties) {
				std::cout << "Properties for camera "
					  << c->id() << ":\n";
				for (const auto &[key, val] : c->properties()) {
					const libcamera::ControlId *id =
						libcamera::properties::properties.at(key);
					std::cout << "  " << std::left
						  << std::setw(32) << id->name()
						  << " " << val.toString()
						  << "\n";
				}
			}

			c->release();
		}

		cm2.stop();
		return 0;
	}

	/* -- Setup ----------------------------------------------------- */
	/*
	 * Allow specifying only one dimension: fill the missing axis from the
	 * camera default so --width=3264 alone gives 3264×1080 rather than
	 * silently ignoring the request and using 1920×1080.
	 */
	if (width != 0 || height != 0) {
		if (width == 0)
			width = cam.getWidth();
		if (height == 0)
			height = cam.getHeight();
		cam.setResolution(width, height);
	}

	if (hwGains.hasAny()) {
		if (cam.isAwbEnabled())
			std::cerr << "Warning: hardware WB gains set but AWB "
				     "not disabled. Use --no-awb.\n";
		cam.setHwGains(hwGains);
	}

	/* Auto-detect output format from file extension */
	{
		auto dot = output.rfind('.');
		if (dot != std::string::npos) {
			std::string e = output.substr(dot + 1);
			std::transform(e.begin(), e.end(), e.begin(), ::tolower);
			if (e == "png")
				cam.setImageFormat("png");
			else if (e == "pgm")
				cam.setRawCapture(true);
		}
	}

	/* For timelapse/burst: default timeout per shot */
	if ((captureN > 1 || timelapse > 0) && captureN > 0)
		cam.setCaptureCount(captureN);
	if (timelapse > 0)
		cam.setTimelapse(timelapse);

	if (cam.init(cameraId) < 0)
		return 1;

	/* -- Capture --------------------------------------------------- */
	int ret;
	if (captureN > 1 || timelapse > 0) {
		/* Multi-shot: burst or timelapse — camera stays running */
		ret = cam.captureLoop(output);
	} else {
		/* Single shot */
		ret = cam.captureStill(output);
	}

	return ret < 0 ? 1 : 0;
}
