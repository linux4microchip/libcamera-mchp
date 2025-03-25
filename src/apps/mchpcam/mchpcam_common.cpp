/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
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
	width_(1920),
	height_(1080),
	pixelFormat_(formats::RGB565),	// Default to RGB565 for better quality
	running_(false),
	brightness_(1),
	contrast_(18),
	whiteBalanceAutomatic_(false),
	gamma_(1),
	red_component_gain_(1944),
	blue_component_gain_(3404),
	green_red_component_gain_(1103),
	green_blue_component_gain_(1619),
	red_component_offset_(7928),
	blue_component_offset_(7936),
	green_red_component_offset_(7920),
	green_blue_component_offset_(7920)
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
	return -ENODEV;
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
		controls.set(controls::microchip::RedGain.id(), ControlValue(red_component_gain_));
		controls.set(controls::microchip::BlueGain.id(), ControlValue(blue_component_gain_));
		controls.set(controls::microchip::GreenRedGain.id(), ControlValue(green_red_component_gain_));
		controls.set(controls::microchip::GreenBlueGain.id(),ControlValue(green_blue_component_gain_));
		controls.set(controls::microchip::RedOffset.id(), ControlValue(red_component_offset_));
		controls.set(controls::microchip::BlueOffset.id(), ControlValue(blue_component_offset_));
		controls.set(controls::microchip::GreenRedOffset.id(), ControlValue(green_red_component_offset_));
		controls.set(controls::microchip::GreenBlueOffset.id(), ControlValue(green_blue_component_offset_));

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
	camera_.reset();
	}
	if (cameraManager_)
	cameraManager_->stop();
}

void MchpCamCommon::setResolution(unsigned int width, unsigned int height)
{
	// Support common resolutions
	if ((width == 640 && height == 480) ||
	(width == 1280 && height == 720) ||
	(width == 1640 && height == 1232) ||
	(width == 1920 && height == 1080) ||
	(width == 2560 && height == 1920) ||
	(width == 3264 && height == 2464)) {
	width_ = width;
	height_ = height;
	std::cout << "Setting resolution to: " << width_ << "x" << height_ << std::endl;
	} else {
	// Default to 1920x1080 for good balance of quality and speed
	width_ = 1920;
	height_ = 1080;
	std::cerr << "Unsupported resolution: " << width << "x" << height
	<< ". Setting to 1920x1080." << std::endl;
	}
}

void MchpCamCommon::setFormat(const std::string &format)
{
	if (format == "YUYV")
	pixelFormat_ = formats::YUYV;
	else if (format == "RGB565")
	pixelFormat_ = formats::RGB565;
	else {
	std::cerr << "Unsupported format: " << format << ". Options are YUYV, RGB565." << std::endl;
	std::cerr << "Using RGB565 for best quality." << std::endl;
	pixelFormat_ = formats::RGB565;
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
	}
	}
	};

	initControl(&controls::Brightness, brightness_);
	initControl(&controls::Contrast, contrast_);
	initControl(&controls::AwbEnable, whiteBalanceAutomatic_);
	initControl(&controls::Gamma, gamma_);
	initControl(&controls::microchip::RedGain, red_component_gain_);
	initControl(&controls::microchip::BlueGain, blue_component_gain_);
	initControl(&controls::microchip::GreenRedGain, green_red_component_gain_);
	initControl(&controls::microchip::GreenBlueGain, green_blue_component_gain_);
	initControl(&controls::microchip::RedOffset, red_component_offset_);
	initControl(&controls::microchip::BlueOffset, blue_component_offset_);
	initControl(&controls::microchip::GreenRedOffset, green_red_component_offset_);
	initControl(&controls::microchip::GreenBlueOffset, green_blue_component_offset_);
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

void MchpCamCommon::setRedGain(int value)
{
	red_component_gain_ = value;
}

void MchpCamCommon::setGreenRedGain(int value)
{
	green_red_component_gain_ = value;
}

void MchpCamCommon::setBlueGain(int value)
{
	blue_component_gain_ = value;
}

void MchpCamCommon::setGreenBlueGain(int value)
{
	green_blue_component_gain_ = value;
}

void MchpCamCommon::setRedOffset(int value)
{
	red_component_offset_ = value;
}

void MchpCamCommon::setGreenRedOffset(int value)
{
	green_red_component_offset_ = value;
}

void MchpCamCommon::setBlueOffset(int value)
{
	blue_component_offset_ = value;
}

void MchpCamCommon::setGreenBlueOffset(int value)
{
	green_blue_component_offset_ = value;
}
