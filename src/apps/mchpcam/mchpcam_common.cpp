/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * mchpcam - common application
 */

#include "mchpcam_common.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace std::chrono_literals;

MchpCamCommon::MchpCamCommon()
	: stream_(nullptr),
	width_(640),
	height_(480),
	pixelFormat_(formats::YUYV),
	running_(false),
	brightness_(0),
	contrast_(0),
	whiteBalanceAutomatic_(true),
	gamma_(0)
{
}

MchpCamCommon::~MchpCamCommon()
{
	stop();
}

int MchpCamCommon::init(const std::string &cameraId)
{
	cameraManager_ = std::make_unique<CameraManager>();
	cameraManager_->start();

	if (cameraId.empty()) {
	auto cameras = cameraManager_->cameras();
	if (cameras.empty()) {
	    std::cerr << "No cameras available" << std::endl;
	    return -1;
	}
	camera_ = cameraManager_->get(cameras[0]->id());
	} else {
	camera_ = cameraManager_->get(cameraId);
	}

	if (!camera_) {
	std::cerr << "Failed to find camera" << std::endl;
	return -1;
	}

	if (camera_->acquire() < 0) {
	std::cerr << "Failed to acquire camera" << std::endl;
	return -1;
	}

	config_ = camera_->generateConfiguration({ StreamRole::Viewfinder });
	config_->at(0).pixelFormat = pixelFormat_;
	config_->at(0).size = Size(width_, height_);

	std::cout << "Attempting to configure camera with resolution: " << width_ << "x" << height_ << std::endl;

	CameraConfiguration::Status status = config_->validate();
	if (status == CameraConfiguration::Invalid) {
	std::cerr << "Invalid camera configuration" << std::endl;
	return -1;
	}
	if (status == CameraConfiguration::Adjusted) {
	std::cout << "Camera configuration adjusted" << std::endl;
	width_ = config_->at(0).size.width;
	height_ = config_->at(0).size.height;
	}

	if (camera_->configure(config_.get()) < 0) {
	std::cerr << "Failed to configure camera" << std::endl;
	return -1;
	}

	std::cout << "Actual configured resolution: " << width_ << "x" << height_ << std::endl;

	allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
	for (StreamConfiguration &cfg : *config_) {
	if (allocator_->allocate(cfg.stream()) < 0) {
	    std::cerr << "Failed to allocate buffers" << std::endl;
	    return -1;
	}
	}

	stream_ = config_->at(0).stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream_);
	for (unsigned int i = 0; i < buffers.size(); ++i) {
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			return -ENOMEM;
		}

		if (request->addBuffer(stream_, buffers[i].get()) < 0) {
			std::cerr << "Can't set buffer for request" << std::endl;
			return -ENOMEM;
		}

		ControlList &controls = request->controls();
		controls.set(controls::Brightness, brightness_);
		controls.set(controls::Contrast, contrast_);
		controls.set(controls::AwbEnable, whiteBalanceAutomatic_);
		controls.set(controls::Gamma, gamma_);

		requests_.push_back(std::move(request));
	}

	camera_->requestCompleted.connect(this, &MchpCamCommon::requestComplete);

	initializeControls();

	return 0;
}

void MchpCamCommon::stop()
{
	if (camera_) {
		camera_->stop();
		allocator_->free(stream_);
		camera_->release();
	}
	camera_.reset();
	cameraManager_->stop();
}

void MchpCamCommon::setResolution(unsigned int width, unsigned int height)
{
	if ((width == 640 && height == 480) ||
	(width == 1640 && height == 1232) ||
	(width == 1920 && height == 1080) ||
	(width == 3264 && height == 2464)) {
	width_ = width;
	height_ = height;
	std::cout << "Setting resolution to: " << width_ << "x" << height_ << std::endl;
	} else {
	std::cerr << "Unsupported resolution: " << width << "x" << height
		  << ". Falling back to default (640x480)." << std::endl;
	width_ = 640;
	height_ = 480;
	}
}

void MchpCamCommon::setFormat(const std::string &format)
{
	if (format == "YUYV")
		pixelFormat_ = formats::YUYV;
	else if (format == "RGB565")
		pixelFormat_ = formats::RGB565;
	else {
		std::cerr << "Unsupported format: " << format << ". Using default (YUYV)." << std::endl;
		pixelFormat_ = formats::YUYV;
	}
}

void MchpCamCommon::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
	for (auto [stream, buffer] : buffers) {
		const FrameMetadata &metadata = buffer->metadata();
		std::cout << "Sequence: " << std::setw(6) << std::setfill('0') << metadata.sequence
			<< " Timestamp: " << metadata.timestamp << std::endl;

		if (running_) {
			processFrame(buffer);
		}
	}

	request->reuse(Request::ReuseBuffers);
	camera_->queueRequest(request);
}

void MchpCamCommon::processFrame([[maybe_unused]] const FrameBuffer *buffer)
{
	/* Implementation in derived class if needed */
}

void MchpCamCommon::saveFrame([[maybe_unused]] const FrameBuffer *buffer, [[maybe_unused]] const std::string &filename)
{
	/* Implementation in derived class */
}

void MchpCamCommon::initializeControls()
{
	if (camera_) {
		const auto &ctrls = camera_->controls();

		auto initControl = [&](const ControlId *id, auto &value) {
			if (ctrls.count(id)) {
				const ControlInfo &info = ctrls.at(id);
				if (!info.values().empty()) {
					value = info.def().template get<typename std::remove_reference<decltype(value)>::type>();
				} else {
					std::cerr << "Warning: Control " << id->name() << " has no valid values." << std::endl;
				}
			}
		};

		initControl(&controls::Brightness, brightness_);
		initControl(&controls::Contrast, contrast_);
		initControl(&controls::AwbEnable, whiteBalanceAutomatic_);
		initControl(&controls::Gamma, gamma_);
	}
}

bool MchpCamCommon::isControlSupported(const libcamera::ControlId *id) const
{
	return camera_ && camera_->controls().find(id) != camera_->controls().end();
}

void MchpCamCommon::setBrightness(int value)
{
	brightness_ = value;
}

void MchpCamCommon::setContrast(int value)
{
	contrast_ = value;
}

void MchpCamCommon::setWhiteBalanceAutomatic(bool value)
{
	whiteBalanceAutomatic_ = value;
}

void MchpCamCommon::setGamma(int value)
{
	gamma_ = value;
}
