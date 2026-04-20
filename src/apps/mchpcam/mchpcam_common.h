/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
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
	libcamera::PixelFormat getPixelFormat() const { return pixelFormat_; }

	/*
	 * Image controls — types and neutral values match libcamera spec.
	 *
	 *   Brightness : float  [−1.0,  +1.0],  neutral  0.0
	 *   Contrast   : float  [ 0.0,  32.0],  neutral  1.0
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

	/* AGC / exposure */
	void setAgcEnabled(bool e) { agcEnabled_ = e; }
	void setManualExposure(int32_t us)
	{
		manualExposureUs_ = us;
		agcEnabled_ = false;
	}
	void setManualGain(float g)
	{
		manualGain_ = g;
		agcEnabled_ = false;
	}

	/* CCM */
	void setCcmEnabled(bool e) { ccmEnabled_ = e; }

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

	/* Applied exposure/gain metadata (populated from request metadata) */
	int32_t appliedExposureUs() const { return appliedExposureUs_; }
	float appliedAnalogueGain() const { return appliedAnalogueGain_; }

	float convergedColourGainR() const { return convergedColourGainR_; }
	float convergedColourGainB() const { return convergedColourGainB_; }

protected:
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
	libcamera::PixelFormat pixelFormat_ = libcamera::formats::RGB565;

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

	/* Warmup / convergence */
	unsigned int warmupFrames_ = 12;
	/*
	 * Safety timeout: force-capture after this many frames regardless of
	 * AWB/AE lock state. Set to 150 by default (5 s at 30fps). The
	 * adaptive timeout (timeoutSec_) is derived from this value and the
	 * sensor framerate in captureStill() / captureLoop().
	 */
	unsigned int maxFrames_;
	unsigned int frameCount_ = 0;

	/*
	 * Post-lock settling counter. The ISC WB gain registers are
	 * double-buffered; new values written by the IPA take effect only
	 * after the next ISC_CTRL_UPPRO pulse (every 4 frames).
	 */
	unsigned int settleFrames_ = 0;
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
	static constexpr unsigned int kPostLockSettleFrames = 4;

	std::atomic<bool> awbLocked_{ false };
	std::atomic<bool> aeLocked_{ false };
	int colourTemperature_ = 0;

	/* Applied sensor state populated from request metadata */
	int32_t appliedExposureUs_ = 0;
	float appliedAnalogueGain_ = 1.0f;

	/* Converged IPA white balance gains (R, B) read from ColourGains metadata.
	 * Written to the JSON sidecar for calibration tool WB-undo. */
	float convergedColourGainR_ = 1.0f;
	float convergedColourGainB_ = 1.0f;

	virtual void requestComplete(libcamera::Request *request);
	virtual void saveFrame(const libcamera::FrameBuffer *buffer,
			       const std::string &filename);

	void applyControls(libcamera::ControlList &controls);

private:
	/* No copy or assignment */
	MchpCamCommon(const MchpCamCommon &) = delete;
	MchpCamCommon &operator=(const MchpCamCommon &) = delete;
};
