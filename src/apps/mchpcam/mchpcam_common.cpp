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
	gamma_(1)
{
}

MchpCamCommon::~MchpCamCommon()
{
	stop();
}

int MchpCamCommon::applyAWBDefaults(libcamera::ControlList& controls, const AWBParameters& params)
{
    std::cout << "Applying Default AWB parameters :" << std::endl;
    std::cout << "    Red Gain: " << params.redGain << std::endl;
    std::cout << "    Blue Gain: " << params.blueGain << std::endl;
    std::cout << "    Green-Red Gain: " << params.greenRedGain << std::endl;
    std::cout << "    Green-Blue Gain: " << params.greenBlueGain << std::endl;
    std::cout << "    Red Offset: " << params.redOffset << std::endl;
    std::cout << "    Blue Offset: " << params.blueOffset << std::endl;
    std::cout << "    Green-Red Offset: " << params.greenRedOffset << std::endl;
    std::cout << "    Green-Blue Offset: " << params.greenBlueOffset << std::endl;

    // Use a helper function
    auto setControl = [&](const libcamera::ControlId *id, auto value) {
        controls.set(id->id(), libcamera::ControlValue(value));
    };

    // Set white balance parameters
    setControl(&libcamera::controls::microchip::RedGain, params.redGain);
    setControl(&libcamera::controls::microchip::BlueGain, params.blueGain);
    setControl(&libcamera::controls::microchip::GreenRedGain, params.greenRedGain);
    setControl(&libcamera::controls::microchip::GreenBlueGain, params.greenBlueGain);
    setControl(&libcamera::controls::microchip::RedOffset, params.redOffset);
    setControl(&libcamera::controls::microchip::BlueOffset, params.blueOffset);
    setControl(&libcamera::controls::microchip::GreenRedOffset, params.greenRedOffset);
    setControl(&libcamera::controls::microchip::GreenBlueOffset, params.greenBlueOffset);

    // Disable automatic white balance to use manual parameters
    setControl(&libcamera::controls::AwbEnable, false);
    return 0;
}

int MchpCamCommon::init(const std::string &cameraId)
{
	cameraManager_ = std::make_unique<CameraManager>();
	int ret = cameraManager_->start();
	if (ret) {
	std::cerr << "Failed to start camera manager" << std::endl;
	return ret;
	}

	// Camera selection logic
	if (cameraId.empty()) {
	auto cameras = cameraManager_->cameras();
	if (cameras.empty()) {
	std::cerr << "No cameras available" << std::endl;
	return -ENODEV;
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

	// Generate configuration for still capture
	config_ = camera_->generateConfiguration({ StreamRole::StillCapture });

	// Configure stream with quality settings
	StreamConfiguration &captureConfig = config_->at(0);
	captureConfig.pixelFormat = pixelFormat_;
	captureConfig.size = Size(width_, height_);
	captureConfig.bufferCount = 1;	// Single buffer for speed

	CameraConfiguration::Status status = config_->validate();
	if (status == CameraConfiguration::Invalid) {
	std::cerr << "Invalid camera configuration" << std::endl;
	return -1;
	}

	if (status == CameraConfiguration::Adjusted) {
	std::cout << "Camera configuration adjusted" << std::endl;
	width_ = captureConfig.size.width;
	height_ = captureConfig.size.height;
	pixelFormat_ = captureConfig.pixelFormat;
	}

	ret = camera_->configure(config_.get());
	if (ret < 0) {
	std::cerr << "Failed to configure camera" << std::endl;
	return ret;
	}

	// Show actual configured format after camera configuration
	std::cout << "Camera configured with format: " << captureConfig.pixelFormat.toString()
	<< " (" << width_ << "x" << height_ << ")" << std::endl;

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
	initControl(&controls::microchip::RedGain, awbParams_.redGain);
	initControl(&controls::microchip::BlueGain, awbParams_.blueGain);
	initControl(&controls::microchip::GreenRedGain, awbParams_.greenRedGain);
	initControl(&controls::microchip::GreenBlueGain, awbParams_.greenBlueGain);
	initControl(&controls::microchip::RedOffset, awbParams_.redOffset);
	initControl(&controls::microchip::BlueOffset, awbParams_.blueOffset);
	initControl(&controls::microchip::GreenRedOffset, awbParams_.greenRedOffset);
	initControl(&controls::microchip::GreenBlueOffset, awbParams_.greenBlueOffset);
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
	awbParams_.redGain = value;
}

void MchpCamCommon::setGreenRedGain(int value)
{
	awbParams_.greenRedGain = value;
}

void MchpCamCommon::setBlueGain(int value)
{
	awbParams_.blueGain = value;
}

void MchpCamCommon::setGreenBlueGain(int value)
{
	awbParams_.greenBlueGain = value;
}

void MchpCamCommon::setRedOffset(int value)
{
	awbParams_.redOffset = value;
}

void MchpCamCommon::setGreenRedOffset(int value)
{
	awbParams_.greenRedOffset = value;
}

void MchpCamCommon::setBlueOffset(int value)
{
	awbParams_.blueOffset = value;
}

void MchpCamCommon::setGreenBlueOffset(int value)
{
	awbParams_.greenBlueOffset = value;
}
