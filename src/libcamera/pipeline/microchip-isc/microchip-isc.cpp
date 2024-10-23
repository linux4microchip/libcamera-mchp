/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * Pipeline Handler for Microchip eXtended Image Sensor Controller (XISC)
 */

#include <memory>
#include <iomanip>
#include <cstring>
#include <vector>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <fstream>
#include <string>
#include <sstream>
#include <filesystem>

#include <linux/media.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-controls.h>
#include <linux/version.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/stream.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/transform.h>
#include <libcamera/control_ids.h>

#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(MicrochipISC)

class PipelineHandlerMicrochipISC;

class MicrochipISCCameraData : public Camera::Private
{
public:
	MicrochipISCCameraData(PipelineHandler *pipe, MediaDevice *media)
		: Camera::Private(pipe), media_(media)
	{
		streams_.resize(2);
	}

	int init();
	int setupLinks();
	int initSubdev(const MediaEntity *entity);
	int setupFormats(V4L2SubdeviceFormat *format, V4L2Subdevice::Whence whence);
	unsigned int getMediaBusFormat(PixelFormat *pixelFormat) const;

	struct Configuration {
		uint32_t code;
		Size sensorSize;
		PixelFormat captureFormat;
		Size captureSize;
		std::vector<PixelFormat> outputFormats;
		SizeRange outputSizes;
	};

	std::unique_ptr<CameraSensor> sensor_;
	std::map<std::string, std::unique_ptr<V4L2Subdevice>> iscSubdev_;
	std::unique_ptr<V4L2VideoDevice> iscVideo_;
	std::vector<Stream> streams_;
	std::vector<Configuration> configs_;
	std::map<PixelFormat, std::vector<const Configuration *>> formats_;
	MediaDevice *media_;
private:
	void tryPipeline(unsigned int code, const Size &size);

	MicrochipISCCameraData *cameraData(Camera *camera)
	{
		return static_cast<MicrochipISCCameraData *>(camera->_d());
	}
};

class MicrochipISCCameraConfiguration : public CameraConfiguration
{
public:
	MicrochipISCCameraConfiguration(MicrochipISCCameraData *data)
		: CameraConfiguration(), data_(data)
	{
	}

	Status validate() override;

	const MicrochipISCCameraData::Configuration *pipeConfig() const
	{
		return pipeConfig_;
	}

	static const std::map<PixelFormat, unsigned int> formatsMap_;
	V4L2SubdeviceFormat sensorFormat_;
private:
	MicrochipISCCameraData *data_;
	const MicrochipISCCameraData::Configuration *pipeConfig_;
};

class PipelineHandlerMicrochipISC : public PipelineHandler
{
public:
	PipelineHandlerMicrochipISC(CameraManager *manager)
		: PipelineHandler(manager)
	{
	}

	~PipelineHandlerMicrochipISC()
	{
	}

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
			Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;
	int exportFrameBuffers(Camera *camera, Stream *stream,
			std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;
	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;
	int queueRequestDevice(Camera *camera, Request *request) override;
	bool match(DeviceEnumerator *enumerator) override;
	std::string generateId(const MediaEntity *entity);
private:
	MicrochipISCCameraData *cameraData(Camera *camera)
	{
		return static_cast<MicrochipISCCameraData *>(camera->_d());
	}

	int processControl(ControlList *controls, unsigned int id, const ControlValue &value);
	int processControls(MicrochipISCCameraData *data, Request *request);
	void bufferReady(FrameBuffer *buffer);
	Size findOptimalSize(const std::vector<MicrochipISCCameraData::Configuration>& configs,
			const Size& target);
	PixelFormat findRawFormat(const std::vector<MicrochipISCCameraData::Configuration>& configs);
	bool isSupportedConfiguration(const std::vector<MicrochipISCCameraData::Configuration>& configs,
			const StreamConfiguration& cfg);
};

unsigned int MicrochipISCCameraData::getMediaBusFormat(PixelFormat *pixelFormat) const
{
	auto it = MicrochipISCCameraConfiguration::formatsMap_.find(*pixelFormat);
	if (it != MicrochipISCCameraConfiguration::formatsMap_.end())
		return it->second;

	/* If not found, default format sets to YUYV */
	*pixelFormat = formats::YUYV;
	return MicrochipISCCameraConfiguration::formatsMap_.at(*pixelFormat);
}

const std::map<PixelFormat, unsigned int> MicrochipISCCameraConfiguration::formatsMap_ = {
	{ formats::SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8 },
	{ formats::SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8 },
	{ formats::SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8 },
	{ formats::SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8 },
	{ formats::SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ formats::SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10 },
	{ formats::SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ formats::SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10 },
	{ formats::SBGGR12, MEDIA_BUS_FMT_SBGGR12_1X12 },
	{ formats::SGBRG12, MEDIA_BUS_FMT_SGBRG12_1X12 },
	{ formats::SGRBG12, MEDIA_BUS_FMT_SGRBG12_1X12 },
	{ formats::SRGGB12, MEDIA_BUS_FMT_SRGGB12_1X12 },
	{ formats::YUYV, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::UYVY, MEDIA_BUS_FMT_UYVY8_2X8 },
	{ formats::RGB565, MEDIA_BUS_FMT_RGB565_2X8_LE },
	/* Add other formats as needed */
};

std::unique_ptr<CameraConfiguration>
PipelineHandlerMicrochipISC::generateConfiguration(Camera *camera,
						   Span<const StreamRole> roles)
{
	MicrochipISCCameraData *data = cameraData(camera);
	std::unique_ptr<MicrochipISCCameraConfiguration> config =
		std::make_unique<MicrochipISCCameraConfiguration>(data);

	if (roles.empty())
		return config;

	for (StreamRole role : roles) {
		StreamConfiguration cfg{};

		switch (role) {
		case StreamRole::StillCapture:
			cfg.pixelFormat = formats::RGB565;
			cfg.size = data->sensor_->resolution();
			cfg.bufferCount = 1;
			break;

		case StreamRole::VideoRecording:
			cfg.pixelFormat = formats::YUYV;
			cfg.size = findOptimalSize(data->configs_, Size(1920, 1080));
			cfg.bufferCount = 4;
			break;

		case StreamRole::Viewfinder:
			cfg.pixelFormat = formats::YUYV;
			cfg.size = findOptimalSize(data->configs_, Size(640, 480));
			cfg.bufferCount = 2;
			break;

		case StreamRole::Raw:
			cfg.pixelFormat = findRawFormat(data->configs_);
			cfg.size = data->sensor_->resolution();
			cfg.bufferCount = 1;
			break;

		default:
			cfg.pixelFormat = data->configs_[0].captureFormat;
			cfg.size = data->configs_[0].captureSize;
			cfg.bufferCount = 4;
			break;
		}

		if (data->formats_.find(cfg.pixelFormat) == data->formats_.end()) {
			LOG(MicrochipISC, Warning) << "Unsupported format " << cfg.pixelFormat.toString()
				<< " for role " << static_cast<int>(role)
				<< ". Falling back to default.";
			cfg.pixelFormat = data->configs_[0].captureFormat;
			cfg.size = data->configs_[0].captureSize;
		}

		if (!isSupportedConfiguration(data->configs_, cfg)) {
			LOG(MicrochipISC, Warning) << "Unsupported configuration for role "
				<< static_cast<int>(role) << ". Falling back to default.";
			cfg.pixelFormat = data->configs_[0].captureFormat;
			cfg.size = data->configs_[0].captureSize;
		}

		config->addConfiguration(cfg);
	}

	config->validate();
	return config;
}

Size PipelineHandlerMicrochipISC::findOptimalSize(const std::vector<MicrochipISCCameraData::Configuration>& configs,
						  const Size& target)
{
	Size optimal = configs[0].captureSize;
	for (const auto& config : configs) {
		if (config.captureSize.width <= target.width &&
				config.captureSize.height <= target.height &&
				config.captureSize.width * config.captureSize.height >
				optimal.width * optimal.height) {
			optimal = config.captureSize;
		}
	}
	return optimal;
}

PixelFormat PipelineHandlerMicrochipISC::findRawFormat(const std::vector<MicrochipISCCameraData::Configuration>& configs)
{
	/* List of known raw formats */
	const std::vector<PixelFormat> rawFormats = {
		formats::SBGGR8,
		formats::SGBRG8,
		formats::SGRBG8,
		formats::SRGGB8,
		formats::SBGGR10,
		formats::SGBRG10,
		formats::SGRBG10,
		formats::SRGGB10,
		formats::SBGGR12,
		formats::SGBRG12,
		formats::SGRBG12,
		formats::SRGGB12
		/* Add more raw formats if needed */
	};

	for (const auto& config : configs) {
		if (std::find(rawFormats.begin(), rawFormats.end(), config.captureFormat) != rawFormats.end()) {
			return config.captureFormat;
		}
	}

	return configs[0].captureFormat;  /* Fallback to default if no raw format found */
}

bool PipelineHandlerMicrochipISC::isSupportedConfiguration(const std::vector<MicrochipISCCameraData::Configuration>& configs,
		                                           const StreamConfiguration& cfg)
{
	for (const auto& config : configs) {
		if (config.captureFormat == cfg.pixelFormat && config.captureSize == cfg.size)
			return true;
	}
	return false;
}

bool PipelineHandlerMicrochipISC::match(DeviceEnumerator *enumerator)
{
	DeviceMatch dm("microchip_isc_c");
	dm.add("microchip_isc_scaler");
	dm.add("csi2dc");
	dm.add("dw-csi.0");
	dm.add("microchip_isc_common");

	MediaDevice *media = acquireMediaDevice(enumerator, dm);
	if (!media) {
		return false;
	}

	std::unique_ptr<MicrochipISCCameraData> data = std::make_unique<MicrochipISCCameraData>(this, media);

	int ret;
	for (MediaEntity *entity : media->entities()) {
		switch (entity->function()) {
		case MEDIA_ENT_F_IO_V4L:
			data->iscVideo_ = std::make_unique<V4L2VideoDevice>(entity);
			ret = data->iscVideo_->open();
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to open video device: " << strerror(-ret);
				return false;
			}
			LOG(MicrochipISC, Debug) << "Opened video device: " << entity->name();

			/* Connect the bufferReady signal */
			data->iscVideo_->bufferReady.connect(this, &PipelineHandlerMicrochipISC::bufferReady);
			break;

		default:
			/* Handle all other entities, including MEDIA_ENT_F_CAM_SENSOR and MEDIA_INTF_T_V4L_SUBDEV */
			ret = data->initSubdev(entity);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to initialize subdevice: " << entity->name();
				return false;
			}
			break;
		}
	}

	/* Ensure all necessary components were found. */
	if (!data->sensor_ || data->iscSubdev_.size() < 3 || !data->iscVideo_) {
		LOG(MicrochipISC, Error) << "Unable to find all required entities";
		return false;
	}

	/* Initialize the camera. */
	ret = data->init();
	if (ret) {
		LOG(MicrochipISC, Error) << "Failed to initialize camera data";
		return false;
	}

	/* Create and register the camera. */
	std::set<Stream *> streams;
	for (Stream &stream : data->streams_)
		streams.insert(&stream);

	std::string cameraId = data->sensor_->id();
	std::shared_ptr<Camera> camera = Camera::create(std::move(data), cameraId, streams);
	registerCamera(std::move(camera));

	LOG(MicrochipISC, Info) << "Registered camera '" << cameraId << "'";

	return true;
}

int PipelineHandlerMicrochipISC::configure(Camera *camera, CameraConfiguration *c)
{
	MicrochipISCCameraData *data = cameraData(camera);
	MicrochipISCCameraConfiguration *config =
		static_cast<MicrochipISCCameraConfiguration *>(c);
	int ret;

	LOG(MicrochipISC, Debug) << "Configuring camera: " << camera->id();

	ret = data->setupLinks();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to setup links: " << ret;
		return ret;
	}

	const MicrochipISCCameraData::Configuration *pipeConfig = config->pipeConfig();

	if (data->formats_.find(pipeConfig->captureFormat) == data->formats_.end()) {
		LOG(MicrochipISC, Error) << "Unsupported format " << pipeConfig->captureFormat.toString();
		return -EINVAL;
	}

	V4L2SubdeviceFormat format{};
	format.code = pipeConfig->code;
	format.size = pipeConfig->sensorSize;

	LOG(MicrochipISC, Debug) << "Setting up formats: code=" << format.code
		<< ", size=" << format.size.toString();

	ret = data->setupFormats(&format, V4L2Subdevice::ActiveFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to setup formats: " << ret;
		return ret;
	}

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = data->iscVideo_->toV4L2PixelFormat(pipeConfig->captureFormat);
	captureFormat.size = pipeConfig->captureSize;

	LOG(MicrochipISC, Debug) << "Setting video format: fourcc="
		<< captureFormat.fourcc.toString()
		<< ", size=" << captureFormat.size.toString();

	ret = data->iscVideo_->setFormat(&captureFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to set video format: " << ret;
		return ret;
	}

	for (unsigned int i = 0; i < c->size(); ++i) {
		StreamConfiguration &cfg = c->at(i);
		cfg.setStream(&data->streams_[i]);
		cfg.stride = captureFormat.planes[0].bpl;
		LOG(MicrochipISC, Debug) << "Stream " << i << " configuration: "
			<< "format=" << cfg.pixelFormat.toString()
			<< ", size=" << cfg.size.toString()
			<< ", stride=" << cfg.stride;
	}

	return 0;
}

int PipelineHandlerMicrochipISC::exportFrameBuffers(Camera *camera, Stream *stream,
		                                    std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	unsigned int count = stream->configuration().bufferCount;
	MicrochipISCCameraData *data = cameraData(camera);

	LOG(MicrochipISC, Debug) << "Exporting " << count << " frame buffers for stream "
		<< stream->configuration().toString();

	return data->iscVideo_->exportBuffers(count, buffers);
}

int PipelineHandlerMicrochipISC::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	MicrochipISCCameraData *data = cameraData(camera);
	unsigned int count = 0;

	for (const auto& stream : data->streams_) {
		count += stream.configuration().bufferCount;
	}

	LOG(MicrochipISC, Debug) << "Starting camera " << camera->id() << " with " << count << " buffers";

	int ret = data->iscVideo_->importBuffers(count);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to import buffers: " << strerror(-ret);
		return ret;
	}

	V4L2DeviceFormat format;
	ret = data->iscVideo_->getFormat(&format);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to get current format: " << strerror(-ret);
		return ret;
	}
	LOG(MicrochipISC, Debug) << "Current video device format: " << format.toString();

	ret = data->iscVideo_->streamOn();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to start streaming: " << strerror(-ret);
		data->iscVideo_->releaseBuffers();
		return ret;
	}

	LOG(MicrochipISC, Debug) << "Camera " << camera->id() << " started successfully";
	return 0;
}

int PipelineHandlerMicrochipISC::processControl(ControlList *controls, unsigned int id,
				       const ControlValue &value)

{
	uint32_t cid;

	if (id == controls::Brightness)
		cid = V4L2_CID_BRIGHTNESS;
	else if (id == controls::Contrast)
		cid = V4L2_CID_CONTRAST;
	else if (id == controls::AwbEnable)
		cid = V4L2_CID_AUTO_WHITE_BALANCE;
	else
		return -EINVAL;

	switch (cid) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST: {
		const ControlInfo &v4l2Info = controls->infoMap()->at(cid);
		int32_t min = v4l2Info.min().get<int32_t>();
		int32_t max = v4l2Info.max().get<int32_t>();

		float fval = value.get<float>();
		int32_t val = static_cast<int32_t>(lroundf(fval));
		controls->set(cid, std::clamp(val, min, max));
		break;
	}

	case V4L2_CID_AUTO_WHITE_BALANCE: {
		bool bval = value.get<bool>();
		controls->set(cid, static_cast<int32_t>(bval));
		break;
	}

	default: {
		LOG(MicrochipISC, Debug) << "Control not yet supported";
		controls->set(cid, 0); /* Todo */
		break;
	}
	}

	return 0;
}

int PipelineHandlerMicrochipISC::processControls(MicrochipISCCameraData *data, Request *request)
{
	ControlList controls(data->iscVideo_->controls());

	for (const auto &[id, value] : request->controls()) {
		processControl(&controls, id, value);
	}

	if (controls.empty()) {
		LOG(MicrochipISC, Debug) << "No controls to set";
		return 0;
	}

	for (const auto &ctrl : controls)
		LOG(MicrochipISC, Debug)
			<< "Setting control 0x" << std::hex << ctrl.first
			<< " to " << ctrl.second.toString();

	int ret = data->iscVideo_->setControls(&controls);
	if (ret) {
		LOG(MicrochipISC, Error) << "Failed to set controls: " << ret;
		return ret < 0 ? ret : -EINVAL;
	}

	return 0;
}

int PipelineHandlerMicrochipISC::queueRequestDevice(Camera *camera, Request *request)
{
	MicrochipISCCameraData *data = cameraData(camera);

	LOG(MicrochipISC, Debug) << "Queueing request for camera " << camera->id();

	int ret = processControls(data, request);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to process controls: " << ret;
		return ret;
	}

	for (Stream &stream : data->streams_) {
		FrameBuffer *buffer = request->findBuffer(&stream);
		if (buffer) {
			LOG(MicrochipISC, Debug) << "Queueing buffer for stream "
				<< stream.configuration().toString();
			ret = data->iscVideo_->queueBuffer(buffer);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to queue buffer: " << ret;
				return ret;
			}
		}
	}

	return 0;
}

std::string PipelineHandlerMicrochipISC::generateId(const MediaEntity *entity)
{
	const std::string basePath = "/sys/firmware/devicetree/base";
	std::string deviceNode = entity->deviceNode();

	/* Start from the device node and traverse up the directory structure */
	std::string searchPath = deviceNode;
	while (true) {
		std::string::size_type pos = searchPath.rfind('/');
		if (pos <= 1) {
			LOG(MicrochipISC, Error) << "Cannot find device tree path";
			return deviceNode;  /* Fallback to deviceNode if we can't find the path */
		}
		searchPath = searchPath.substr(0, pos);

		/* Check if this directory exists in the device tree */
		std::string dtPath = basePath + searchPath;
		if (std::filesystem::exists(dtPath) && std::filesystem::is_directory(dtPath)) {
			/* Found a matching directory in the device tree */
			LOG(MicrochipISC, Debug) << "Found device tree path: " << dtPath;
			return searchPath;  /* Return without the base path */
		}

		/* Check if this directory contains a 'compatible' file */
		std::string compatiblePath = dtPath + "/compatible";
		if (std::filesystem::exists(compatiblePath)) {
			std::ifstream compatibleFile(compatiblePath);
			std::string compatibleValue;
			std::getline(compatibleFile, compatibleValue);
			if (compatibleValue.find("microchip,isc") != std::string::npos) {
				LOG(MicrochipISC, Debug) << "Found ISC device tree node: " << searchPath;
				return searchPath;  /* Return without the base path */
			}
		}
	}
}

int MicrochipISCCameraData::initSubdev(const MediaEntity *entity)
{
	if (entity->function() == MEDIA_ENT_F_CAM_SENSOR) {
		sensor_ = std::make_unique<CameraSensor>(entity);
		return sensor_->init();
	}

	/* For other subdevices */
	std::string entityName = entity->name();
	auto subdev = std::make_unique<V4L2Subdevice>(entity);
	int ret = subdev->open();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to open subdevice " << entityName;
		return ret;
	}

	iscSubdev_[entityName] = std::move(subdev);
	LOG(MicrochipISC, Debug) << "Initialized subdevice: " << entityName;

	return 0;
}

int MicrochipISCCameraData::init()
{
	LOG(MicrochipISC, Debug) << "Initializing camera data";

	properties_ = sensor_->properties();

	/* Try all formats from formatsMap_ */
	for (const auto &[pixelFormat, mbusFormat] : MicrochipISCCameraConfiguration::formatsMap_) {
		for (const Size &size : sensor_->sizes(mbusFormat)) {
			tryPipeline(mbusFormat, size);
		}
	}

	if (configs_.empty()) {
		LOG(MicrochipISC, Error) << "No valid configuration found";
		return -EINVAL;
	}

	/* Generate formats map */
	for (const Configuration &config : configs_) {
		formats_[config.captureFormat].push_back(&config);
	}

	/* Ensure all supported formats are in the map */
	for (const auto &[pixelFormat, mbusFormat] : MicrochipISCCameraConfiguration::formatsMap_) {
		if (formats_.find(pixelFormat) == formats_.end()) {
			formats_[pixelFormat] = {};
		}
	}

	LOG(MicrochipISC, Debug) << "About to initialize ControlInfoMap";
	/* Initialize controlInfo_ */
	controlInfo_ = ControlInfoMap({
			{ &controls::Brightness, ControlInfo(-1.0f, 1.0f, 0.0f) },
			{ &controls::Contrast, ControlInfo(0.0f, 2.0f, 1.0f) },
			{ &controls::AwbEnable, ControlInfo(false, true, true) },
			{ &controls::Gamma, ControlInfo(0.0f, 0.0f, 0.0f) },/* actual min, max, default */
			}, controls::controls);

	LOG(MicrochipISC, Debug) << "ControlInfoMap initialized";

	LOG(MicrochipISC, Debug) << "Camera data initialized successfully";
	return 0;
}

int MicrochipISCCameraData::setupFormats(V4L2SubdeviceFormat *format,
		                         V4L2Subdevice::Whence whence)
{
	int ret;

	ret = sensor_->setFormat(format);
	if (ret < 0)
		return ret;

	for (const auto &[name, subdev] : iscSubdev_) {
		ret = subdev->setFormat(0, format, whence);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int MicrochipISCCameraData::setupLinks()
{
	LOG(MicrochipISC, Debug) << "Setting up links";

	int ret = media_->disableLinks();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to disable links";
		return ret;
	}

	for (const MediaEntity *entity : media_->entities()) {
		for (const MediaPad *pad : entity->pads()) {
			if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
				continue;

			for (MediaLink *link : pad->links()) {
				if (link->flags() & MEDIA_LNK_FL_IMMUTABLE)
					continue;

				ret = link->setEnabled(true);
				if (ret < 0) {
					LOG(MicrochipISC, Error) << "Failed to enable link: "
						<< link->source()->entity()->name()
						<< " -> "
						<< link->sink()->entity()->name();
					return ret;
				}
			}
		}
	}

	LOG(MicrochipISC, Debug) << "Links set up successfully";
	return 0;
}

void MicrochipISCCameraData::tryPipeline(unsigned int code, const Size &size)
{
	LOG(MicrochipISC, Debug) << "Trying pipeline with format " << code
		<< " and size " << size.toString();

	V4L2SubdeviceFormat format{};
	format.code = code;
	format.size = size;

	int ret = setupFormats(&format, V4L2Subdevice::TryFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Debug) << "Format not supported for this pipeline";
		return;
	}

	V4L2VideoDevice::Formats videoFormats = iscVideo_->formats(format.code);

	for (const auto &videoFormat : videoFormats) {
		PixelFormat pixelFormat = videoFormat.first.toPixelFormat();
		if (!pixelFormat) {
			LOG(MicrochipISC, Debug) << "Skipping unsupported pixel format";
			continue;
		}

		Configuration config;
		config.code = code;
		config.sensorSize = size;
		config.captureFormat = pixelFormat;
		config.captureSize = format.size;
		config.outputFormats = { pixelFormat };
		config.outputSizes = SizeRange{ format.size, format.size };

		configs_.push_back(config);
		LOG(MicrochipISC, Debug) << "Added configuration: " << pixelFormat.toString()
			<< " " << format.size.toString();
	}
}

CameraConfiguration::Status MicrochipISCCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	if (config_.size() > data_->streams_.size()) {
		config_.resize(data_->streams_.size());
		status = Adjusted;
	}

	for (StreamConfiguration &cfg : config_) {
		auto formatIt = data_->formats_.find(cfg.pixelFormat);
		if (formatIt == data_->formats_.end()) {
			if (!data_->formats_.empty()) {
				cfg.pixelFormat = data_->formats_.begin()->first;
				formatIt = data_->formats_.begin();
				status = Adjusted;
			} else {
				LOG(MicrochipISC, Error) << "No valid formats available";
				return Invalid;
			}
		}

		const std::vector<const MicrochipISCCameraData::Configuration *> &configs = formatIt->second;

		if (configs.empty()) {
			if (!data_->configs_.empty()) {
				pipeConfig_ = &data_->configs_.front();
				status = Adjusted;
			} else {
				LOG(MicrochipISC, Error) << "No valid configurations available";
				return Invalid;
			}
		} else {
			pipeConfig_ = configs.front();
			for (const MicrochipISCCameraData::Configuration *config : configs) {
				if (config->captureSize.width >= cfg.size.width &&
						config->captureSize.height >= cfg.size.height) {
					pipeConfig_ = config;
					break;
				}
			}
		}

		cfg.size = pipeConfig_->captureSize;

		if (cfg.bufferCount < 1)
			cfg.bufferCount = 4;
	}

	return status;
}

void PipelineHandlerMicrochipISC::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();
	if (!request) {
		LOG(MicrochipISC, Warning) << "Buffer " << buffer
			<< " is not associated with a request";
		return;
	}

	LOG(MicrochipISC, Debug) << "Buffer ready: " << buffer
		<< ", request: " << request;

	if (!request->metadata().contains(controls::SensorTimestamp.id()))
		request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	completeBuffer(request, buffer);

	if (request->hasPendingBuffers()) {
		LOG(MicrochipISC, Debug) << "Request " << request
			<< " still has pending buffers";
		return;
	}

	LOG(MicrochipISC, Debug) << "Completing request " << request;
	completeRequest(request);
}

void PipelineHandlerMicrochipISC::stopDevice(Camera *camera)
{
	MicrochipISCCameraData *data = cameraData(camera);

	LOG(MicrochipISC, Debug) << "Stopping device for camera " << camera->id();

	if (data->iscVideo_) {
		data->iscVideo_->streamOff();
		data->iscVideo_->releaseBuffers();
	}

	for (const auto &[name, subdev] : data->iscSubdev_) {
		subdev->close();
	}

	LOG(MicrochipISC, Debug) << "Device stopped for camera " << camera->id();
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMicrochipISC, "microchip-isc")

} /* namespace libcamera */
