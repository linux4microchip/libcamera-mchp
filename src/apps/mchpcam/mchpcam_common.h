/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* mchpcam - common application header
	*/
#ifndef MCHPCAM_COMMON_H
#define MCHPCAM_COMMON_H
#include <string>
#include <memory>
#include <vector>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/stream.h>
#include <libcamera/property_ids.h>
#include <libcamera/logging.h>

using namespace libcamera;

struct AWBParameters {
		int redGain;
		int blueGain;
		int greenRedGain;
		int greenBlueGain;
		int redOffset;
		int blueOffset;
		int greenRedOffset;
		int greenBlueOffset;

		// Constructor with defaults
		AWBParameters()
			: redGain(1944), blueGain(3404),
				greenRedGain(1103), greenBlueGain(1619),
				redOffset(7928), blueOffset(7936),
				greenRedOffset(7920), greenBlueOffset(7920)
		{}
};

class MchpCamCommon {
public:
	MchpCamCommon();
	virtual ~MchpCamCommon();
	int init(const std::string &cameraId = "");
	void stop();
	void setResolution(unsigned int width, unsigned int height);
	void setFormat(const std::string &format);
	unsigned int getWidth() const { return width_; }
	unsigned int getHeight() const { return height_; }
	virtual void setAWBParameters(const AWBParameters& params) {
		awbParams_ = params;
	}
	virtual void setBrightness(int value);
	virtual void setContrast(int value);
	virtual void setWhiteBalanceAutomatic(bool value);
	virtual void setGamma(int value);
	static int applyAWBDefaults(libcamera::ControlList& controls, const AWBParameters& params);
	virtual void setRedGain(int value);
	virtual void setGreenRedGain(int value);
	virtual void setBlueGain(int value);
	virtual void setGreenBlueGain(int value);
	virtual void setRedOffset(int value);
	virtual void setGreenRedOffset(int value);
	virtual void setBlueOffset(int value);
	virtual void setGreenBlueOffset(int value);

protected:
	std::unique_ptr<libcamera::CameraManager> cameraManager_;
	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
	std::vector<std::unique_ptr<libcamera::Request>> requests_;
	libcamera::Stream *stream_;
	bool captureComplete_;
	unsigned int width_;
	unsigned int height_;
	libcamera::PixelFormat pixelFormat_;
	bool running_;
	int brightness_;
	int contrast_;
	bool whiteBalanceAutomatic_;
	int gamma_;
	AWBParameters awbParams_;

	void requestComplete(libcamera::Request *request);
	virtual void processFrame(const libcamera::FrameBuffer *buffer);
	virtual void saveFrame(const libcamera::FrameBuffer *buffer, const std::string &filename);
	void initializeControls();
	bool isControlSupported(const libcamera::ControlId *id) const;
};
#endif
