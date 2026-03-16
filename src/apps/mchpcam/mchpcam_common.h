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

class MchpCamCommon
{
public:
	MchpCamCommon();
	virtual ~MchpCamCommon();

	/* Camera lifecycle */
	int init(const std::string &cameraId = "");
	void stop();

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
	 *   ExposureValue : float [−2.0, +2.0], neutral  0.0  (EV stops)
	 *   MeteringMode  : int32  0=CentreWeighted  1=Spot  2=Matrix
	 */
	void setBrightness(float v) { brightness_ = v; }
	void setContrast(float v) { contrast_ = v; }
	void setGamma(float v) { gamma_ = v; }
	void setSaturation(float v) { saturation_ = v; }
	void setHue(float v) { hue_ = v; }
	void setExposureValue(float v) { exposureValue_ = v; }
	void setMeteringMode(int32_t m) { meteringMode_ = m; }

	/* AWB */
	void setAwbEnabled(bool e) { awbEnabled_ = e; }

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
	float exposureValue_ = 0.0f;
	int32_t meteringMode_ = 0;

	/* AWB */
	bool awbEnabled_ = true;
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
	unsigned int maxFrames_ = 60;
	unsigned int frameCount_ = 0;
	/*
	 * Post-lock settling counter.  The ISC WB gain registers are
	 * double-buffered; new values written by the IPA take effect only
	 * after the next ISC_CTRL_UPPRO pulse, which the kernel driver
	 * issues inside the histogram ISR every 4 frames.  Without a
	 * settling period the captured frame still carries the pre-lock
	 * gains, producing a colour cast in the output image.
	 * kPostLockSettleFrames covers the worst-case 4-frame pipeline
	 * flush plus 2 frames of margin.
	 */
	unsigned int settleFrames_ = 0;
	static constexpr unsigned int kPostLockSettleFrames = 2;
	std::atomic<bool> awbLocked_{ false };
	std::atomic<bool> aeLocked_{ false };
	int colourTemperature_ = 0;

	virtual void requestComplete(libcamera::Request *request);
	virtual void saveFrame(const libcamera::FrameBuffer *buffer,
			       const std::string &filename);

	void applyControls(libcamera::ControlList &controls);
	bool isControlSupported(const libcamera::ControlId *id) const;

private:
	/* No copy or assignment */
	MchpCamCommon(const MchpCamCommon &) = delete;
	MchpCamCommon &operator=(const MchpCamCommon &) = delete;
};
