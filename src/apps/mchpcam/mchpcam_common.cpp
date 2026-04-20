/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * mchpcam — common camera application base
 */

#include "mchpcam_common.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <limits.h>
#include <thread>

#include <libcamera/base/span.h>

#include <libcamera/property_ids.h>

#include <libcamera/ipa/microchip_isc_ipa_interface.h>

using namespace std::chrono_literals;

/* ------------------------------------------------------------------ */
/* Construction / destruction                                           */
/* ------------------------------------------------------------------ */

/*
 * Initialiser list order MUST follow the declaration order in the
 * header to satisfy -Werror=reorder.  Check mchpcam_common.h if you
 * add or reorder members.
 */
MchpCamCommon::MchpCamCommon()
	: stream_(nullptr),
	  captureReady_(false),
	  cameraStarted_(false),
	  savedBuffer_(nullptr),
	  width_(1920),
	  height_(1080),
	  pixelFormat_(libcamera::formats::RGB565),
	  brightness_(0.0f),
	  contrast_(1.0f),
	  gamma_(2.2f),
	  saturation_(1.0f),
	  hue_(0.0f),
	  exposureValue_(0.0f),
	  meteringMode_(0),
	  awbEnabled_(true),
	  manualColourGains_(false),
	  colourGainR_(1.0f),
	  colourGainB_(1.0f),
	  agcEnabled_(true),
	  manualExposureUs_(-1),
	  manualGain_(-1.0f),
	  ccmEnabled_(true),
	  warmupFrames_(12),
	  maxFrames_(150),
	  frameCount_(0),
	  settleFrames_(0),
	  awbLocked_(false),
	  aeLocked_(false),
	  colourTemperature_(0)
{
}

MchpCamCommon::~MchpCamCommon()
{
	stop();
}

/* ------------------------------------------------------------------ */
/* Geometry helpers                                                     */
/* ------------------------------------------------------------------ */

void MchpCamCommon::setResolution(unsigned int width, unsigned int height)
{
	width_ = width;
	height_ = height;
}

void MchpCamCommon::setFormat(const std::string &formatStr)
{
	if (formatStr == "YUYV")
		pixelFormat_ = libcamera::formats::YUYV;
	else if (formatStr == "RGB565")
		pixelFormat_ = libcamera::formats::RGB565;
	else if (formatStr == "BGR888")
		pixelFormat_ = libcamera::formats::BGR888;
	else if (formatStr == "RGB888")
		pixelFormat_ = libcamera::formats::RGB888;
	else if (formatStr == "SRGGB10" || formatStr == "SRGGB8") {
		std::cerr << "Error: raw Bayer formats (SRGGB10, SRGGB8) are not supported.\n"
			  << "Raw capture is disabled in mchpcam-still: the ISC bits_pipeline\n"
			  << "excludes gamma/CBC when outputting Bayer, causing the AGC target\n"
			  << "to be in the wrong domain and making captures unusable.\n"
			  << "Use RGB565 (default) or JPEG output instead.\n";
		return;
	} else {
		std::cerr << "Unknown format '" << formatStr
			  << "'. Supported: YUYV RGB565 BGR888 RGB888."
			  << " Using RGB565.\n";
		pixelFormat_ = libcamera::formats::RGB565;
	}
}

/* ------------------------------------------------------------------ */
/* Control helpers                                                      */
/* ------------------------------------------------------------------ */

/*
 * applyControls() — write all currently configured settings into a
 * ControlList that will be attached to a Request.
 *
 * Controls are sent on every queued request so the IPA's queueRequest()
 * path sees them and can update activeState accordingly.  Sending the
 * same value every frame is harmless; the IPA only acts when values
 * differ or when a sticky flag (AwbEnable, AeEnable) is set.
 */
void MchpCamCommon::applyControls(libcamera::ControlList &controls)
{
	using namespace libcamera;

	/* Image quality controls */
	controls.set(controls::Brightness, brightness_);
	controls.set(controls::Contrast, contrast_);
	controls.set(controls::Gamma, gamma_);
	controls.set(controls::Saturation, saturation_);
	/*
	 * Hue is set via the CBC YAML tuning parameter, not a runtime
	 * libcamera control.  controls::Hue is not a standard libcamera
	 * control ID; the ISC applies it through V4L2_CID_HUE internally.
	 * The --hue CLI option is reserved for when a vendor control is
	 * registered for it.
	 */
	(void)hue_;

	/* Exposure controls */
	controls.set(controls::ExposureValue, exposureValue_);
	controls.set(controls::AeMeteringMode, meteringMode_);
	controls.set(controls::AeEnable, agcEnabled_);

	if (!agcEnabled_) {
		if (manualExposureUs_ >= 0)
			controls.set(controls::ExposureTime, manualExposureUs_);
		if (manualGain_ >= 0.0f)
			controls.set(controls::AnalogueGain, manualGain_);
	}

	/* AWB controls */
	controls.set(controls::AwbEnable, awbEnabled_);

	if (manualColourGains_) {
		/*
		 * IPA-level manual WB: R and B float gain multipliers
		 * delivered via the standard ColourGains control.
		 * The IPA's Awb::queueRequest() reads this and stores
		 * to activeState — no hardware register bypass involved.
		 */
		std::array<float, 2> gains{ colourGainR_, colourGainB_ };
		controls.set(controls::ColourGains,
			     Span<const float, 2>(gains.data(), 2));
	}

	/*
	 * Hardware ISC WB register bypass via vendor controls.
	 * Only meaningful when IPA AWB is disabled; values go directly
	 * to the ISC WB block skipping the IPA gain computation.
	 */
	if (hwGains_.hasAny()) {
		using namespace libcamera::controls::microchip;

		if (hwGains_.redGain >= 0)
			controls.set(RedGain.id(),
				     libcamera::ControlValue(hwGains_.redGain));
		if (hwGains_.blueGain >= 0)
			controls.set(BlueGain.id(),
				     libcamera::ControlValue(hwGains_.blueGain));
		if (hwGains_.greenRedGain >= 0)
			controls.set(GreenRedGain.id(),
				     libcamera::ControlValue(hwGains_.greenRedGain));
		if (hwGains_.greenBlueGain >= 0)
			controls.set(GreenBlueGain.id(),
				     libcamera::ControlValue(hwGains_.greenBlueGain));
		if (hwGains_.redOffset != INT_MIN)
			controls.set(RedOffset.id(),
				     libcamera::ControlValue(hwGains_.redOffset));
		if (hwGains_.blueOffset != INT_MIN)
			controls.set(BlueOffset.id(),
				     libcamera::ControlValue(hwGains_.blueOffset));
		if (hwGains_.greenRedOffset != INT_MIN)
			controls.set(GreenRedOffset.id(),
				     libcamera::ControlValue(hwGains_.greenRedOffset));
		if (hwGains_.greenBlueOffset != INT_MIN)
			controls.set(GreenBlueOffset.id(),
				     libcamera::ControlValue(hwGains_.greenBlueOffset));
	}
}

/* ------------------------------------------------------------------ */
/* Camera initialisation                                                */
/* ------------------------------------------------------------------ */

int MchpCamCommon::init(const std::string &cameraId)
{
	using namespace libcamera;

	cameraManager_ = std::make_unique<CameraManager>();
	int ret = cameraManager_->start();
	if (ret) {
		std::cerr << "Failed to start camera manager: " << ret << "\n";
		return ret;
	}

	/* Camera selection */
	if (cameraId.empty()) {
		auto cameras = cameraManager_->cameras();
		if (cameras.empty()) {
			std::cerr << "No cameras found\n";
			return -ENODEV;
		}
		camera_ = cameraManager_->get(cameras[0]->id());
	} else {
		camera_ = cameraManager_->get(cameraId);
	}

	if (!camera_) {
		std::cerr << "Camera not found\n";
		return -ENODEV;
	}

	if (camera_->acquire() < 0) {
		std::cerr << "Failed to acquire camera\n";
		return -EBUSY;
	}

	/* Stream configuration */
	config_ = camera_->generateConfiguration({ StreamRole::StillCapture });
	if (!config_) {
		std::cerr << "Failed to generate camera configuration\n";
		return -EINVAL;
	}

	StreamConfiguration &streamCfg = config_->at(0);
	streamCfg.pixelFormat = pixelFormat_;
	streamCfg.size = Size(width_, height_);
	/*
	 * Four buffers: one per ISC Bayer channel so all four can be in
	 * flight simultaneously during the warmup loop.  The ring lets
	 * requestComplete() requeue without stalling the pipeline.
	 */
	streamCfg.bufferCount = 4;

	CameraConfiguration::Status status = config_->validate();
	if (status == CameraConfiguration::Invalid) {
		std::cerr << "Camera configuration is invalid\n";
		return -EINVAL;
	}
	if (status == CameraConfiguration::Adjusted) {
		width_ = streamCfg.size.width;
		height_ = streamCfg.size.height;
		pixelFormat_ = streamCfg.pixelFormat;
		std::cout << "Camera adjusted config to: "
			  << pixelFormat_.toString()
			  << " " << width_ << "x" << height_ << "\n";
	}

	ret = camera_->configure(config_.get());
	if (ret < 0) {
		std::cerr << "Failed to configure camera: " << ret << "\n";
		return ret;
	}

	std::cout << "Camera configured: " << pixelFormat_.toString()
		  << " " << width_ << "x" << height_ << "\n";

	/* Buffer allocation */
	allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
	for (StreamConfiguration &cfg : *config_) {
		if (allocator_->allocate(cfg.stream()) < 0) {
			std::cerr << "Failed to allocate buffers\n";
			return -ENOMEM;
		}
		stream_ = cfg.stream();
	}

	/* Create one Request per allocated buffer */
	requests_.clear();
	const auto &buffers = allocator_->buffers(stream_);
	for (const auto &buf : buffers) {
		auto req = camera_->createRequest();
		if (!req) {
			std::cerr << "Failed to create request\n";
			return -ENOMEM;
		}

		ControlList ctrls(camera_->controls());
		applyControls(ctrls);
		req->controls() = ctrls;

		if (req->addBuffer(stream_, buf.get()) < 0) {
			std::cerr << "Failed to add buffer to request\n";
			return -ENOMEM;
		}
		requests_.push_back(std::move(req));
	}

	camera_->requestCompleted.connect(this, &MchpCamCommon::requestComplete);

	/* Status summary */
	std::cout << "AWB:  " << (awbEnabled_ ? "auto (IPA)" : "disabled") << "\n";
	if (manualColourGains_)
		std::cout << "ColourGains: R=" << colourGainR_
			  << " B=" << colourGainB_ << " (IPA manual)\n";
	if (hwGains_.hasAny())
		std::cout << "HW register WB bypass active\n";
	std::cout << "AGC:  " << (agcEnabled_ ? "auto" : "manual") << "\n";
	std::cout << "CCM:  " << (ccmEnabled_ ? "on" : "off") << "\n";
	std::cout << "Warmup frames: " << warmupFrames_ << "\n";

	return 0;
}

/* ------------------------------------------------------------------ */
/* Camera stop                                                          */
/* ------------------------------------------------------------------ */

void MchpCamCommon::stop()
{
	if (!camera_)
		return;

	/*
	 * Disconnect the requestCompleted signal BEFORE calling stop().
	 * camera_->stop() transitions the camera to Stopping state and cancels
	 * in-flight requests. Between the stop() call and its return, cancelled
	 * requests fire requestCompleted with RequestCancelled status. If the
	 * signal is still connected, requestComplete() runs and calls
	 * camera_->queueRequest() on the next re-queued request — which is
	 * illegal on a Stopping camera and triggers a libcamera assertion.
	 * Disconnecting first ensures no callbacks fire during teardown.
	 */
	camera_->requestCompleted.disconnect(this);

	if (cameraStarted_.exchange(false))
		camera_->stop();

	if (allocator_ && stream_)
		allocator_->free(stream_);

	allocator_.reset();

	camera_->release();
	camera_.reset();

	if (cameraManager_) {
		cameraManager_->stop();
		cameraManager_.reset();
	}
}

/* ------------------------------------------------------------------ */
/* Request completion — called from libcamera event loop thread         */
/* ------------------------------------------------------------------ */

/*
 * This is the warmup engine.  For each completed frame:
 *
 *  1. Read IPA convergence metadata (AwbLocked, ColourTemperature).
 *  2. Increment the frame counter.
 *  3. If already decided to save (captureReady_ set by a previous
 *     call), return without requeuing — let the pipeline drain.
 *  4. Check convergence:
 *       - AWB OK  : AWB disabled, or AwbLocked metadata is true.
 *       - AE OK   : AGC disabled, or frame count exceeds warmupFrames_
 *                   (the IPA does not yet emit AeLocked metadata).
 *     If both OK and warmup frames elapsed → pick this frame to save.
 *     Force-save after maxFrames_ regardless of lock status.
 *  5. Otherwise reuse the request and requeue for another frame.
 *
 * Calling camera_->queueRequest() from the requestCompleted signal
 * handler is the standard libcamera pattern used by cam(1).
 */
void MchpCamCommon::requestComplete(libcamera::Request *request)
{
	using namespace libcamera;

	if (request->status() == Request::RequestCancelled)
		return;

	const ControlList &meta = request->metadata();

	/* Capture applied sensor state from IPA metadata every frame.
	 * These reflect the settings actually applied to THIS frame by the IPA.
	 * ColourGains is [R, B]; indices 0 and 1. */
	{
		auto expVal = meta.get(controls::ExposureTime);
		if (expVal.has_value())
			appliedExposureUs_ = static_cast<float>(*expVal);

		auto gainVal = meta.get(controls::AnalogueGain);
		if (gainVal.has_value())
			appliedAnalogueGain_ = *gainVal;

		auto cgVal = meta.get(controls::ColourGains);
		if (cgVal.has_value()) {
			convergedColourGainR_ = (*cgVal)[0];
			convergedColourGainB_ = (*cgVal)[1];
		}
	}

	/*
	 * AWB lock — if AWB is disabled it is always "locked" (nothing to
	 * wait for).  Otherwise latch the IPA's AwbLocked metadata when it
	 * arrives; the flag is sticky (never reset to false).
	 */
	if (!awbEnabled_) {
		awbLocked_ = true;
	} else {
		auto v = meta.get(controls::AwbLocked);
		if (v.has_value() && *v)
			awbLocked_ = true;
	}

	/*
	 * AE lock — disabled is immediately locked.  AeStateConverged is
	 * the primary signal; fall back to frame-count after warmupFrames_
	 * in case the IPA does not emit AeState metadata.
	 */
	if (!agcEnabled_) {
		aeLocked_ = true;
	} else {
		auto v = meta.get(controls::AeState);
		if (v.has_value() && *v == controls::AeStateConverged)
			aeLocked_ = true;
		else if (frameCount_ >= warmupFrames_)
			aeLocked_ = true;
	}

	auto ct = meta.get(controls::ColourTemperature);
	if (ct.has_value())
		colourTemperature_ = *ct;

	frameCount_++;

	bool converged = awbLocked_.load() && aeLocked_.load();

	/* Force convergence after maxFrames_ to guarantee eventual capture */
	if (frameCount_ >= maxFrames_)
		converged = true;

	/*
	 * keypressReceived_ starts as true (normal auto-capture mode).
	 * setKeypress(true) / setSignalTrigger(true) reset it to false,
	 * making it a gate: the main thread sets it true again once the
	 * user presses Enter or SIGUSR1 is received. requestComplete()
	 * only saves when this gate is open, so savedBuffer_ is always
	 * set by this path (not by the main thread directly).
	 */
	if (frameCount_ >= warmupFrames_ && converged &&
	    keypressReceived_.load()) {
		/*
		 * ISC WB gain settling: isc_update_profile() commits WB gains
		 * synchronously via ISC_CTRL_UPPRO polling before the ioctl
		 * returns.  One pipeline frame is needed for committed gains
		 * to appear in the DMA output.  kPostLockSettleFrames = 4
		 * gives 3 frames of headroom above that minimum.
		 * Settling is bypassed when maxFrames_ forces capture.
		 */
		bool settled = (frameCount_ >= maxFrames_) ||
			       (++settleFrames_ >= kPostLockSettleFrames);

		if (settled) {
			/*
			 * Only the first requestComplete() call that reaches
			 * this point saves the buffer and wakes the main thread.
			 * Subsequent in-flight completions see exchange() return
			 * true and fall through to the unconditional requeue.
			 */
			if (!captureReady_.exchange(true)) {
				/*
				 * Use stream_-keyed lookup, not begin()->second.
				 * begin() returns the first entry in pointer-address
				 * order which is non-deterministic. In a future
				 * multi-stream config this would silently save the
				 * wrong buffer.
				 */
				savedBuffer_ = request->findBuffer(stream_);
				captureNotify_.notify_one();
			}
		}
		/* fall through — always requeue */
	}

	/*
	 * Requeue while capture has not yet been saved.  Once captureReady_
	 * is set the main thread calls camera_->stop(), which cancels all
	 * still-queued requests (they return via the RequestCancelled early
	 * exit above).  Do NOT requeue after captureReady_ is set — the
	 * camera is in Stopping state and queueRequest() would assert.
	 */
	if (!captureReady_.load()) {
		request->reuse(Request::ReuseBuffers);
		ControlList ctrls(camera_->controls());
		applyControls(ctrls);
		request->controls() = ctrls;
		camera_->queueRequest(request);
	}
}

void MchpCamCommon::saveFrame([[maybe_unused]] const libcamera::FrameBuffer *buffer,
			      [[maybe_unused]] const std::string &filename)
{
	/* Implemented in derived class */
}

bool MchpCamCommon::setAwbMode(const std::string &mode)
{
	if (mode == "auto") {
		awbEnabled_ = true;
		return true;
	}

	/*
	 * Named AWB mode presets are not supported by the ISC IPA —
	 * controls::AwbMode is not in the pipeline's control map.
	 * Simulate by disabling AWB and setting ColourGains to
	 * approximate per-illuminant values.
	 */
	struct AwbPreset {
		const char *name;
		float gainR;
		float gainB;
	};
	static const AwbPreset kPresets[] = {
		{ "incandescent", 3.80f, 1.20f },
		{ "tungsten", 3.80f, 1.20f },
		{ "fluorescent", 3.00f, 1.65f },
		{ "indoor", 3.20f, 1.50f },
		{ "daylight", 2.55f, 2.10f },
		{ "cloudy", 2.30f, 2.40f },
		{ "shade", 2.10f, 2.60f },
	};

	for (const auto &p : kPresets) {
		if (mode == p.name) {
			setColourGains(p.gainR, p.gainB);

			return true;
		}
	}

	std::cerr << "Unknown AWB mode: " << mode
		  << ". Valid: auto incandescent tungsten fluorescent "
		  << "indoor daylight cloudy shade\n";
	return false;
}

void MchpCamCommon::resetConvergenceState()
{
	captureReady_.store(false);
	/* keypressReceived_ is reset to false by setKeypress()/setSignalTrigger().
	 * For normal auto-capture and for timelapse (where the caller sets the
	 * next keypress after calling resetConvergenceState), leave it as-is
	 * unless explicitly gated. captureStill/captureLoop set it via
	 * keypressReceived_.store(true) after the user input. */
	savedBuffer_ = nullptr;
	frameCount_ = 0;
	settleFrames_ = 0;
	/*
	 * Keep awbLocked_ / aeLocked_ — they reflect IPA steady state
	 * which persists between captures in a timelapse session.
	 * The IPA does not reset its convergence state between shots, so
	 * the next shot will converge faster than the first.
	 */
}
