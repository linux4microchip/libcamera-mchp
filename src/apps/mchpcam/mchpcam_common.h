/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024, Microchip Technology Inc.
 *
 * mchpcam — common camera application base class
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <limits.h>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/geometry.h>
#include <libcamera/stream.h>

/*
 * Raw ISC hardware WB register values — bypass the IPA AWB loop entirely.
 *
 * Use only when IPA AWB is explicitly disabled (--no-awb).  Combining
 * with IPA AWB produces gain fights and unpredictable colour.  For
 * normal manual WB use --colour-gains which goes through the IPA.
 */
struct HwGainParams {
	int redGain = -1; /* −1 = not set; valid 0–8191 (512 = 1.0×) */
	int blueGain = -1;
	int greenRedGain = -1;
	int greenBlueGain = -1;
	int redOffset = INT_MIN; /* INT_MIN = not set; valid −4095..4095 */
	int blueOffset = INT_MIN;
	int greenRedOffset = INT_MIN;
	int greenBlueOffset = INT_MIN;

	bool hasAny() const
	{
		return redGain != -1 || blueGain != -1 ||
		       greenRedGain != -1 || greenBlueGain != -1 ||
		       redOffset != INT_MIN || blueOffset != INT_MIN ||
		       greenRedOffset != INT_MIN || greenBlueOffset != INT_MIN;
	}
};

/* Image orientation — mirrors libcamera::Orientation enum */
enum class MchpOrientation {
	Normal = 0, /* no transform */
	HFlip = 1, /* horizontal mirror */
	VFlip = 2, /* vertical flip */
	Rotate180 = 3, /* 180° rotation = HFlip + VFlip */
};

class MchpCamCommon
{
public:
	MchpCamCommon();
	virtual ~MchpCamCommon();

	/* Camera lifecycle */
	int init(const std::string &cameraId = "");
	void stop();
	/*
	 * Reset capture state between timelapse shots. Clears captureReady_,
	 * savedBuffer_ and frame counters without stopping/restarting the
	 * camera or IPA — AWB/AE state persists so the next shot converges
	 * faster than the first.
	 */
	void resetConvergenceState();

	/* Stream geometry — call before init() */
	void setResolution(unsigned int width, unsigned int height);
	void setFormat(const std::string &formatStr);
	unsigned int getWidth() const { return width_; }
	unsigned int getHeight() const { return height_; }

	/*
	 * Image controls — types and neutral values match libcamera spec.
	 *
	 *   Brightness : float  [−1.0,  +1.0],  neutral  0.0
	 *   Contrast   : float  [ 0.0, 128.0],  neutral  1.0
	 *   Gamma      : float  [ 0.1,  10.0],  typical  2.2
	 *   Saturation : float  [ 0.0,  32.0],  neutral  1.0
	 *   Hue        : float  [−180,  +180],  neutral  0.0  (degrees)
	 *   Sharpness  : float  [ 0.0,  16.0],  neutral  1.0
	 *   ExposureValue : float [−2.0, +2.0], neutral  0.0  (EV stops)
	 *   MeteringMode  : int32  0=CentreWeighted  1=Spot  2=Matrix
	 */
	void setBrightness(float v) { brightness_ = v; }
	void setContrast(float v) { contrast_ = v; }
	void setGamma(float v) { gamma_ = v; }
	void setSaturation(float v) { saturation_ = v; }
	void setHue(float v) { hue_ = v; }
	/*
	 * --sharpness: accepted for rpicam-apps compatibility, no-op.
	 * ISC has no hardware sharpening block.
	 */
	void setSharpness(float v) { sharpness_ = v; }
	void setExposureValue(float v) { exposureValue_ = v; }
	void setMeteringMode(int32_t m) { meteringMode_ = m; }

	/* --framerate: locks FrameDurationLimits */
	void setFramerate(float fps) { framerate_ = fps; }

	/* --digitalgain: IMX219 digital gain 1.0–16.0 */
	void setDigitalGain(float g) { digitalGain_ = g; }

	/*
	 * --exposure-time-mode / --gain-mode: independent AE/AG mode.
	 * 0=auto (IPA controls it), 1=manual (user fixes it).
	 * Maps to controls::ExposureTimeMode and controls::AnalogueGainMode.
	 * Allows locking one knob while leaving the other on auto —
	 * e.g. fix gain=1× for low-noise long-exposure while AE adjusts
	 * the shutter, or fix shutter for motion-freeze while AGC picks gain.
	 */
	void setExposureTimeMode(int32_t m)
	{
		exposureTimeMode_ = m;
		exposureTimeModeSet_ = true;
	}
	void setGainMode(int32_t m)
	{
		gainMode_ = m;
		gainModeSet_ = true;
	}

	/* Manual CCM override — 9 floats row-major RGB→RGB matrix */
	void setCcmOverride(const std::vector<float> &m)
	{
		ccmOverride_ = m;
		ccmOverrideSet_ = (m.size() == 9);
	}

	/* Image orientation (hflip/vflip/rotation) */
	void setOrientation(MchpOrientation o) { orientation_ = o; }

	/* AWB */
	void setAwbEnabled(bool e) { awbEnabled_ = e; }
	bool isAwbEnabled() const { return awbEnabled_; }

	/*
	 * Named AWB mode preset — mirrors rpicam-apps --awb option.
	 * Maps to controls::AwbMode enum values.
	 *   auto         → AwbAuto
	 *   incandescent → AwbIncandescent   (~2800K)
	 *   tungsten     → AwbTungsten       (~3200K)
	 *   fluorescent  → AwbFluorescent    (~4000K)
	 *   indoor       → AwbIndoor         (~3800K)
	 *   daylight     → AwbDaylight       (~5500K)
	 *   cloudy       → AwbCloudy         (~6500K)
	 *   custom       → AwbCustom (needs awbgains)
	 */
	bool setAwbMode(const std::string &mode);

	/*
	 * IPA-level manual WB: R and B float gain multipliers (1.0 = neutral).
	 * Disables IPA auto AWB.  GR and GB are left at 1.0 (IPA default).
	 */
	void setColourGains(float r, float b)
	{
		colourGainR_ = r;
		colourGainB_ = b;
		manualColourGains_ = true;
		awbEnabled_ = false;
	}

	/* Hardware ISC register bypass — use with --no-awb only */
	void setHwGains(const HwGainParams &p) { hwGains_ = p; }

	/*
	 * Raw Bayer capture mode (--raw / calibration use).
	 *
	 * Requests SRGGB10 output format (ISC raw passthrough).
	 * The ISC kernel driver keeps WB_ENABLE active even for Bayer output
	 * (bits_pipeline = WB_ENABLE — confirmed in isc_try_configure_pipeline
	 * default: case).  To make this a mathematical no-op, rawCapture_ mode
	 * forces unity WB gains (512 = 1.0× in Q9 ISC scale) and disables IPA
	 * AWB.  AGC continues to run so exposure converges normally.
	 *
	 * The output is a 16-bit little-endian PGM (P5 magic, maxval 1023).
	 * Each pixel occupies 2 bytes; the 10-bit sample is in the low 10 bits.
	 * Bayer pattern: RGGB (IMX219 native, no flip; SRGGB10_1X10).
	 */
	void setRawCapture(bool r) { rawCapture_ = r; }

	/* AGC / exposure */
	void setAgcEnabled(bool e) { agcEnabled_ = e; }
	/*
	 * A manual setpoint locks only its own knob via the per-knob mode
	 * (1 = manual), leaving the other on auto — so --analoggain=1.0 fixes
	 * gain while AE still servos the shutter (the calibration path). Full
	 * manual is --no-agc, which clears agcEnabled_ for both knobs.
	 */
	void setManualExposure(int32_t us)
	{
		manualExposureUs_ = us;
		setExposureTimeMode(1);
	}
	void setManualGain(float g)
	{
		manualGain_ = g;
		setGainMode(1);
	}

	/* CCM */
	void setCcmEnabled(bool e) { ccmEnabled_ = e; }

	/* Save the raw ISC output: skip the software lens-shading pass. */
	void setNoPostproc(bool v) { noPostproc_ = v; }

	/*
	 * Minimum frames before capture is saved; default 12 (covers
	 * three full ISC 4-frame histogram cycles so AWB and AGC have
	 * had at least one complete convergence opportunity each).
	 */
	void setWarmupFrames(unsigned int n) { warmupFrames_ = n; }

	/* Post-capture IPA metadata (valid after captureStill returns) */
	bool isAwbLocked() const { return awbLocked_.load(); }
	bool isAeLocked() const { return aeLocked_.load(); }
	int colourTemperature() const { return colourTemperature_; }

protected:
	/*
	 * Stop the streaming pipeline without releasing the camera. Disconnects
	 * the requestCompleted signal before camera_->stop() so no completion
	 * callback can requeue a request onto a Stopping camera (which would
	 * assert). Leaves camera_, the allocator and buffers intact so the
	 * caller can still read savedBuffer_ afterwards; full teardown is done
	 * by stop().
	 */
	void stopStreaming();

	/* Return the held savedRequest_ to the ring; no-op if none held. */
	void requeueSavedRequest();

	/* libcamera objects */
	std::unique_ptr<libcamera::CameraManager> cameraManager_;
	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
	std::vector<std::unique_ptr<libcamera::Request>> requests_;
	libcamera::Stream *stream_ = nullptr;

	/* Capture handshake — written by requestComplete, read by main */
	std::atomic<bool> captureReady_{ false };
	std::atomic<bool> cameraStarted_{ false };
	const libcamera::FrameBuffer *savedBuffer_ = nullptr;
	/* Request owning savedBuffer_, held out of the ring while it is saved. */
	libcamera::Request *savedRequest_ = nullptr;
	/* Keep the ring in flight across shots (burst/timelapse). */
	bool continuousCapture_ = false;
	/*
	 * keypressReceived_: set by the main thread when the user presses
	 * Enter (--keypress) or SIGUSR1 is received (--signal). Acts as a
	 * gate inside requestComplete() so that captureReady_ and savedBuffer_
	 * are still set by the requestComplete() path (which owns savedBuffer_),
	 * not by the main thread directly.
	 */
	/*
	 * keypressReceived_: true = capture can proceed (default for normal
	 * auto-capture mode). setKeypress(true) / setSignalTrigger(true)
	 * reset this to false; the main thread then sets it back to true
	 * when the user presses Enter or SIGUSR1 arrives.
	 */
	std::atomic<bool> keypressReceived_{ true };
	std::mutex captureMutex_;
	std::condition_variable captureNotify_;

	/* Stream geometry */
	unsigned int width_ = 1920;
	unsigned int height_ = 1080;
	libcamera::PixelFormat pixelFormat_ = libcamera::formats::XRGB8888;

	/* Image controls */
	float brightness_ = 0.0f;
	float contrast_ = 1.0f;
	float gamma_ = 2.2f;
	float saturation_ = 1.0f;
	float hue_ = 0.0f;
	/*
	 * sharpness_ is accepted via --sharpness for rpicam-apps
	 * compatibility but produces a warning: ISC has no hardware
	 * sharpening block. controls::Sharpness is not in this pipeline.
	 */
	float sharpness_ = 1.0f;
	float exposureValue_ = 0.0f;
	int32_t meteringMode_ = 0;

	/* Framerate — locks FrameDurationLimits; 0 = unconstrained */
	float framerate_ = 0.0f;

	/* Digital gain (IMX219: 1.0–16.0); -1 = not set (use sensor default) */
	float digitalGain_ = -1.0f;

	/* Independent AE/AG mode (0=auto, 1=manual); -1 = not set */
	int32_t exposureTimeMode_ = -1;
	bool exposureTimeModeSet_ = false;
	int32_t gainMode_ = -1;
	bool gainModeSet_ = false;

	/* Manual CCM override (9 floats, row-major) */
	std::vector<float> ccmOverride_;
	bool ccmOverrideSet_ = false;

	/* Image orientation — applied to CameraConfiguration */
	MchpOrientation orientation_ = MchpOrientation::Normal;

	/* RAW Bayer capture (--raw) */
	bool rawCapture_ = false;

	/* AWB */
	bool awbEnabled_ = true;
	/* Calibration mode: request IscHistogramBins vendor control. */
	/* awbMode_: named AWB presets not supported by ISC IPA.
	 * setAwbMode() simulates presets via ColourGains instead. */
	bool manualColourGains_ = false;
	float colourGainR_ = 1.0f;
	float colourGainB_ = 1.0f;
	HwGainParams hwGains_;

	/* AGC */
	bool agcEnabled_ = true;
	int32_t manualExposureUs_ = -1;
	float manualGain_ = -1.0f;

	/* CCM */
	bool ccmEnabled_ = true;

	/* Diagnostic: skip the software still post-processing pass on the grab. */
	bool noPostproc_ = false;

	/* Warmup / convergence */
	unsigned int warmupFrames_ = 12;
	/*
	 * Safety timeout: force-capture after this many frames regardless of
	 * AWB/AE lock state. Set to 150 by default (5 s at 30fps). The
	 * adaptive timeout (timeoutSec_) is derived from this value and the
	 * sensor framerate in captureStill() / captureLoop().
	 */
	unsigned int maxFrames_ = 150;
	unsigned int frameCount_ = 0;

	unsigned int settleFrames_ = 0;
	/*
	 * Frame at which the capture gate (past warmup, converged, trigger
	 * received) first opened. The force-grab window is measured from here,
	 * not from stream start, so a late --keypress/--signal trigger still
	 * gets a full settle instead of an immediate unsettled grab.
	 */
	unsigned int gateOpenFrame_ = 0;
	/*
	 * Set once convergence and WB settling are done, so requeued requests
	 * ask the pipeline for the still pass (IscStillProcess). The costly CPU
	 * pass is thus confined to the frame saved, keeping the single-core A7
	 * free during warmup and settling so 3A converges at the video rate.
	 */
	bool stillRequested_ = false;
	/*
	 * Post-lock settling: ISC WB registers are double-buffered but
	 * isc_update_profile() (called from isc_s_ctrl() on every V4L2
	 * setControls ioctl) issues ISC_CTRL_UPPRO synchronously and polls
	 * until the hardware double-buffer commits. New gains are therefore
	 * in hardware by the time the ioctl returns.
	 * One pipeline frame is still needed for the committed gains to
	 * propagate through the ISC processing pipeline and appear in the
	 * DMA output. 4 frames gives 3 frames of headroom above this
	 * minimum, covering any frame-boundary race between the ioctl and
	 * the start of the next frame's exposure.
	 */
	static constexpr unsigned int kPostLockSettleFrames = 8;
	/*
	 * Bounded AWB settle past warmup. AE lock is the hard convergence gate;
	 * AWB latches only when the scene allows, so a still proceeds on AE lock
	 * alone once AWB has had this many frames (several ISC histogram cycles)
	 * to stabilise. Prevents an unlockable dim/noisy scene from deferring
	 * capture to the maxFrames_ fallback with no headroom for the still pass.
	 */
	static constexpr unsigned int kAwbSettleFrames = 24;
	/*
	 * Frames after the gate opens before the still grab is forced, whether
	 * or not a processed frame has arrived. Covers kPostLockSettleFrames of
	 * WB settling plus a few frames for the still pass to run and return.
	 */
	static constexpr unsigned int kCaptureForceFrames = 16;

	std::atomic<bool> awbLocked_{ false };
	std::atomic<bool> aeLocked_{ false };
	int colourTemperature_ = 0;

	virtual void requestComplete(libcamera::Request *request);
	virtual void saveFrame(const libcamera::FrameBuffer *buffer,
			       const std::string &filename);

	void applyControls(libcamera::ControlList &controls);

private:
	/* No copy or assignment */
	MchpCamCommon(const MchpCamCommon &) = delete;
	MchpCamCommon &operator=(const MchpCamCommon &) = delete;
};
