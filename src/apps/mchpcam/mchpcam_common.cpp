/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * mchpcam — common camera application base
 */

#include "mchpcam_common.h"

#include <array>
#include <chrono>
#include <iostream>
#include <limits.h>

#include <libcamera/base/span.h>

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
		std::cerr << "Error: use --raw instead of -f SRGGB10 for raw Bayer capture.\n"
			  << "--raw enables unity WB gains so the ISC WB block is a no-op.\n";
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

	/*
	 * Independent per-knob auto/manual: lock the shutter while gain
	 * auto-tracks, or vice versa.  Only sent when the user set them, so
	 * default behaviour follows AeEnable alone.  The IPA's AGC reads these
	 * and computes exposure/gain manual state independently.
	 */
	if (exposureTimeModeSet_)
		controls.set(controls::ExposureTimeMode, exposureTimeMode_);
	if (gainModeSet_)
		controls.set(controls::AnalogueGainMode, gainMode_);

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
	 * Manual colour-correction matrix (--ccm). When set it overrides the
	 * IPA's CT-interpolated matrix for the request; the IPA applies it with
	 * highest precedence. Best paired with manual AWB so the override is not
	 * fighting a changing illuminant estimate.
	 */
	if (ccmOverrideSet_)
		controls.set(controls::ColourCorrectionMatrix,
			     Span<const float, 9>(ccmOverride_.data(), 9));

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

	/*
	 * --framerate: lock FrameDurationLimits to a single value (min == max)
	 * so the IPA pins the sensor VBLANK to the requested rate. 0 leaves the
	 * frame rate unconstrained.
	 */
	if (framerate_ > 0.0f) {
		int64_t durUs = static_cast<int64_t>(1000000.0f / framerate_);
		std::array<int64_t, 2> limits{ durUs, durUs };
		controls.set(controls::FrameDurationLimits,
			     libcamera::Span<const int64_t, 2>(limits.data(), 2));
	}

	/* --digitalgain: manual sensor digital gain (IMX219 1.0–16.0). */
	if (digitalGain_ >= 1.0f)
		controls.set(controls::DigitalGain, digitalGain_);
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

	/*
	 * Raw Bayer capture mode: override pixel format to SGRBG10 and force
	 * unity WB gains so the ISC WB block (which cannot be disabled for
	 * Bayer output — bits_pipeline = WB_ENABLE) is a mathematical no-op.
	 * Unity = 512 in the ISC 0:4:9 fixed-point gain scale (1.0 × 512).
	 * AWB is disabled so the IPA does not fight the forced gains.
	 * AGC continues to run normally: the histogram tap is pre-WB on
	 * vp_data (datasheet §43.6.20 / Figure 43-38), so AE converges
	 * identically to JPEG mode.
	 */
	if (rawCapture_) {
		/*
		 * IMX219 default output: SRGGB10_1X10 (RGGB, no flip).
		 * Confirmed via media-ctl -p: imx219→CSI2DC→ISC all negotiate
		 * SRGGB10_1X10.  SGRBG10 only appears with hflip=1,vflip=0.
		 */
		pixelFormat_ = libcamera::formats::SRGGB10;
		awbEnabled_ = false;
		hwGains_.redGain = 512;
		hwGains_.blueGain = 512;
		hwGains_.greenRedGain = 512;
		hwGains_.greenBlueGain = 512;
		hwGains_.redOffset = 0;
		hwGains_.blueOffset = 0;
		hwGains_.greenRedOffset = 0;
		hwGains_.greenBlueOffset = 0;
		/*
		 * Raw AGC meters to a higher linear target and the histogram
		 * updates once per 4-frame Bayer cycle, so give it more warmup
		 * cycles to converge before the capture gate opens.
		 */
		if (warmupFrames_ < 40)
			warmupFrames_ = 40;
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
	if (rawCapture_)
		std::cout << "Mode: RAW Bayer (SRGGB10 RGGB, unity WB, PGM output)\n";
	std::cout << "AWB:  " << (awbEnabled_ ? "auto (IPA)" : "disabled") << "\n";
	if (manualColourGains_)
		std::cout << "ColourGains: R=" << colourGainR_
			  << " B=" << colourGainB_ << " (IPA manual)\n";
	if (hwGains_.hasAny())
		std::cout << "HW register WB bypass active"
			  << (rawCapture_ ? " (unity — WB is no-op)" : "") << "\n";
	std::cout << "AGC:  " << (agcEnabled_ ? "auto" : "manual") << "\n";
	std::cout << "CCM:  " << (ccmEnabled_ ? "on" : "off") << "\n";
	std::cout << "Warmup frames: " << warmupFrames_ << "\n";

	return 0;
}

/* ------------------------------------------------------------------ */
/* Camera stop                                                          */
/* ------------------------------------------------------------------ */

void MchpCamCommon::stopStreaming()
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
}

void MchpCamCommon::stop()
{
	if (!camera_)
		return;

	stopStreaming();

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

	bool holdThis;
	bool doRequeue;
	bool wantStillPass;
	{
		/*
		 * Guard the convergence state shared with the main thread
		 * (resetConvergenceState()/requeueSavedRequest()) and publish
		 * savedBuffer_ before captureReady_ so a waiter woken on the
		 * flag never reads a stale buffer.
		 */
		std::lock_guard<std::mutex> lock(captureMutex_);

		if (ct.has_value())
			colourTemperature_ = *ct;

		frameCount_++;

		/*
		 * AE lock is the hard convergence gate — it latches reliably,
		 * either from AeStateConverged or the warmup-frame fallback.
		 * AWB latches only when the scene allows: on a dim or noisy
		 * scene the 4-frame-latency histogram keeps the inter-frame
		 * gain delta above the stability threshold, so the IPA may
		 * never report AWB locked. A still capture must not block on a
		 * lock that never comes, so give AWB a bounded settle window
		 * past warmup and then proceed on AE lock alone.
		 */
		bool awbSettled = awbLocked_.load() ||
				  frameCount_ >= warmupFrames_ + kAwbSettleFrames;
		bool converged = aeLocked_.load() && awbSettled;

		/* Force convergence after maxFrames_ to guarantee capture. */
		if (frameCount_ >= maxFrames_)
			converged = true;

		/*
		 * keypressReceived_ starts true (normal auto-capture) and is a
		 * gate for --keypress/--signal: reset to false, set true again
		 * by the main thread on Enter or SIGUSR1. Arm only while the
		 * gate is open and nothing has been latched yet.
		 */
		if (frameCount_ >= warmupFrames_ && converged &&
		    keypressReceived_.load() && !captureReady_.load()) {
			/* Anchor the force window to when the gate first opened. */
			if (!gateOpenFrame_)
				gateOpenFrame_ = frameCount_;

			/*
			 * Settle with the cheap preview path first, then ask the
			 * pipeline for the still pass. Force the grab once the
			 * window elapses so a scene that never settles, or a
			 * non-post-processed stream, still captures.
			 */
			settleFrames_++;
			bool forced = frameCount_ - gateOpenFrame_ >=
				      kCaptureForceFrames;
			if (!stillRequested_ &&
			    (forced || settleFrames_ >= kPostLockSettleFrames))
				stillRequested_ = true;

			if (stillRequested_) {
				/*
				 * On a post-processed stream wait for the frame
				 * that carries the still pass (IscStillProcess
				 * echoed true) so the saved image is the corrected
				 * one. Streams without the pass, and the force
				 * fallback, save as soon as settling completes.
				 */
				auto flag = meta.get(controls::microchip::IscStillProcess);
				bool processed = noPostproc_ || forced ||
						 !flag.has_value() || *flag;

				if (processed && !captureReady_.exchange(true)) {
					/*
					 * stream_-keyed lookup, not begin(): the
					 * latter is pointer-address ordered and
					 * would save the wrong buffer under a
					 * future multi-stream config.
					 */
					savedBuffer_ = request->findBuffer(stream_);
					savedRequest_ = request;
					/*
					 * Latched — stop requesting the costly
					 * pass on the ring buffers still in flight.
					 */
					stillRequested_ = false;
					captureNotify_.notify_one();
				}
			}
		}

		wantStillPass = stillRequested_ && !noPostproc_;
		holdThis = captureReady_.load() &&
			   request->findBuffer(stream_) == savedBuffer_;
		doRequeue = !captureReady_.load() || continuousCapture_;
	}

	/* Hold the saved buffer out of the ring until the main thread frees it. */
	if (holdThis)
		return;

	/*
	 * Single-shot drains after capture: the main thread then calls
	 * camera_->stop(), and queuing onto a Stopping camera would assert.
	 * Continuous capture keeps the ring in flight so the next shot
	 * re-converges instead of the pipeline running dry.
	 */
	if (doRequeue) {
		request->reuse(Request::ReuseBuffers);
		ControlList ctrls(camera_->controls());
		applyControls(ctrls);
		/*
		 * Ask for the still pass only once settling is done; warmup and
		 * settling frames complete straight from the ISC output, leaving
		 * the single-core A7 free so 3A converges at the video rate.
		 */
		ctrls.set(controls::microchip::IscStillProcess, wantStillPass);
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
	/* Serialise with requestComplete() on the camera thread. */
	std::lock_guard<std::mutex> lock(captureMutex_);

	captureReady_.store(false);
	/* keypressReceived_ is reset to false by setKeypress()/setSignalTrigger().
	 * For normal auto-capture and for timelapse (where the caller sets the
	 * next keypress after calling resetConvergenceState), leave it as-is
	 * unless explicitly gated. captureStill/captureLoop set it via
	 * keypressReceived_.store(true) after the user input. */
	savedBuffer_ = nullptr;
	frameCount_ = 0;
	settleFrames_ = 0;
	gateOpenFrame_ = 0;
	stillRequested_ = false;
	/*
	 * Keep awbLocked_ / aeLocked_ — they reflect IPA steady state
	 * which persists between captures in a timelapse session.
	 * The IPA does not reset its convergence state between shots, so
	 * the next shot will converge faster than the first.
	 */
}

void MchpCamCommon::requeueSavedRequest()
{
	using namespace libcamera;

	Request *req;
	{
		/* Serialise savedRequest_ with requestComplete(). */
		std::lock_guard<std::mutex> lock(captureMutex_);
		if (!savedRequest_ || !cameraStarted_.load())
			return;
		req = savedRequest_;
		savedRequest_ = nullptr;
	}

	req->reuse(Request::ReuseBuffers);
	ControlList ctrls(camera_->controls());
	applyControls(ctrls);
	req->controls() = ctrls;
	camera_->queueRequest(req);
}
