/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
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

using namespace libcamera;

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

	virtual void setBrightness(int value);
	virtual void setContrast(int value);
	virtual void setWhiteBalanceAutomatic(bool value);
	virtual void setGamma(int value);

protected:
	std::unique_ptr<libcamera::CameraManager> cameraManager_;
	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
	std::vector<std::unique_ptr<libcamera::Request>> requests_;
	libcamera::Stream *stream_;
	unsigned int width_;
	unsigned int height_;
	libcamera::PixelFormat pixelFormat_;
	bool running_;

	int brightness_;
	int contrast_;
	bool whiteBalanceAutomatic_;
	int gamma_;

	void requestComplete(libcamera::Request *request);
	virtual void processFrame(const libcamera::FrameBuffer *buffer);
	virtual void saveFrame(const libcamera::FrameBuffer *buffer, const std::string &filename);

	void initializeControls();
	bool isControlSupported(const libcamera::ControlId *id) const;
};

#endif
