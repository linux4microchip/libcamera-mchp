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
#include <sys/mman.h>

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
#include <libcamera/ipa/microchip_isc_ipa_interface.h>
#include <libcamera/ipa/microchip_isc_ipa_proxy.h>


namespace libcamera {

LOG_DEFINE_CATEGORY(MicrochipISC)

class PipelineHandlerMicrochipISC;

class MicrochipISCCameraData : public Camera::Private
{
public:
	MicrochipISCCameraData(PipelineHandler *pipe, MediaDevice *media)
		: Camera::Private(pipe), media_(media), bufferCount(0)
	{
		streams_.resize(2);
		/* Create IPA proxy instead of direct AWB implementation */
		awbIPA_ = IPAManager::createIPA<ipa::microchip_isc::IPAProxyMicrochipISC>(pipe, 1, 1);
		if (awbIPA_)
			awbIPA_->awbComplete.connect(this, &MicrochipISCCameraData::awbComplete);
	}

	void setPipeHandler(PipelineHandlerMicrochipISC *pipe){ pipe_=pipe; }

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
	std::unique_ptr<ipa::microchip_isc::IPAProxyMicrochipISC> awbIPA_;
	unsigned int bufferCount;
private:
	PipelineHandlerMicrochipISC *pipe_;
	void awbComplete([[maybe_unused]] unsigned int bufferId,
			const ControlList &metadata);
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

	int processControlRequest(ControlList *controls, unsigned int id, const ControlValue &value)
	{
		return processControl(controls, id, value);
	}

	const std::vector<Request *> &activeRequests() const
	{
		return activeRequests_;
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
	bool processingComplete_ = false;
	Request *processingRequest_ = nullptr;
private:
	MicrochipISCCameraData *activeData_ = nullptr;
	std::vector<Request *> activeRequests_;
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
			cfg.size = findOptimalSize(data->configs_, Size(1920, 1080));
			cfg.bufferCount = 1;
			break;

		case StreamRole::VideoRecording:
			cfg.pixelFormat = formats::YUYV;
			cfg.size = findOptimalSize(data->configs_, Size(1920, 1080));
			cfg.bufferCount = 4;
			break;

		case StreamRole::Viewfinder:
			cfg.pixelFormat = formats::YUYV;
			cfg.size = findOptimalSize(data->configs_, Size(1920, 1080));
			cfg.bufferCount = 2;
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

	return configs[0].captureFormat;	/* Fallback to default if no raw format found */
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
	data->setPipeHandler(this);

	if (data->awbIPA_) {
		LOG(MicrochipISC, Debug) << "AWB IPA module loaded successfully";
	} else {
		LOG(MicrochipISC, Error) << "Failed to load AWB IPA module";
	}

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
	activeData_= data;
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

	ret = data->setupFormats(&format, V4L2Subdevice::ActiveFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to setup formats: " << ret;
		return ret;
	}

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = data->iscVideo_->toV4L2PixelFormat(pipeConfig->captureFormat);
	captureFormat.size = pipeConfig->captureSize;

	ret = data->iscVideo_->setFormat(&captureFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to set video format: " << ret;
		return ret;
	}

	/* Get the actual format that was set */
	V4L2DeviceFormat actualFormat;
	if (data->iscVideo_->getFormat(&actualFormat) >= 0) {
		LOG(MicrochipISC, Debug) << "Actual format set: " << actualFormat.toString();

		/* Update only the AWB IPA configuration part */
		if (data->awbIPA_) {
			ipa::microchip_isc::MicrochipISCSensorInfo sensorInfo;
			sensorInfo.model = data->sensor_->model();
			sensorInfo.width = actualFormat.size.width;		/* Use the format size we just set */
			sensorInfo.height = actualFormat.size.height;
			sensorInfo.pixelFormat = format.code;

			std::map<unsigned int, IPAStream> streamConfig;
			std::map<unsigned int, ControlInfoMap> entityControls;
			ret = data->awbIPA_->configure(sensorInfo, streamConfig, entityControls);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to configure IPA";
				return ret;
			}
		}
	}

	for (unsigned int i = 0; i < c->size(); ++i) {
		StreamConfiguration &cfg = c->at(i);
		cfg.setStream(&data->streams_[i]);
		cfg.stride = captureFormat.planes[0].bpl;
		cfg.size = captureFormat.size;
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

	/* Export buffers for the specified stream */
	int ret = data->iscVideo_->exportBuffers(count, buffers);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to export buffers for stream " << stream->configuration().toString();
		return ret;
	}

	return 0;
}

int PipelineHandlerMicrochipISC::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	MicrochipISCCameraData *data = cameraData(camera);
	unsigned int count = 0;

	LOG(MicrochipISC, Debug) << "Starting camera " << camera->id();

	/* Disable V4L2 AWB and configure IPA */
	ControlList awbCtrls(data->iscVideo_->controls());
	awbCtrls.set(V4L2_CID_AUTO_WHITE_BALANCE, 0);
	int awbRet = data->iscVideo_->setControls(&awbCtrls);
	if (awbRet < 0) {
		LOG(MicrochipISC, Warning) << "Failed to disable V4L2 AWB, will use as fallback";
		awbCtrls.set(V4L2_CID_AUTO_WHITE_BALANCE, 1);
		data->iscVideo_->setControls(&awbCtrls);
	}

	if (data->awbIPA_) {
		int ipaRet = data->awbIPA_->start();
		if (ipaRet < 0) {
			LOG(MicrochipISC, Error) << "Failed to start IPA";
			return ipaRet;
		}
	}

	/* Count total buffers needed */
	for (const auto& stream : data->streams_) {
		count += stream.configuration().bufferCount;
	}

	LOG(MicrochipISC, Debug) << "Starting camera " << camera->id() << " with " << count << " buffers";

	/* Check if buffers are already imported */
	if (data->bufferCount == 0) {
		int ret = data->iscVideo_->importBuffers(count);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to import buffers: " << strerror(-ret);
			return ret;
		}
		data->bufferCount = count;
	} else {
		LOG(MicrochipISC, Debug) << "Buffers already imported, count: " << data->bufferCount;
	}

	V4L2DeviceFormat format;
	int ret = data->iscVideo_->getFormat(&format);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to get current format: " << strerror(-ret);
		return ret;
	}
	LOG(MicrochipISC, Debug) << "Current video device format: " << format.toString();

	ret = data->iscVideo_->streamOn();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to start streaming: " << strerror(-ret);
		data->iscVideo_->releaseBuffers();
		data->bufferCount = 0;
		return ret;
	}

	LOG(MicrochipISC, Debug) << "Camera " << camera->id() << " started successfully";
	return 0;
}

int PipelineHandlerMicrochipISC::processControl(ControlList *controls, unsigned int id,
						const ControlValue &value)
{
	uint32_t cid;

	/* Map libcamera controls to V4L2 controls */
	if (id == controls::Brightness)
		cid = V4L2_CID_BRIGHTNESS;
	else if (id == controls::Contrast)
		cid = V4L2_CID_CONTRAST;
	else if (id == controls::AwbEnable)
		cid = V4L2_CID_AUTO_WHITE_BALANCE;
	else if (id == controls::microchip::RedGain)
		cid = 0x009819c0; /* Red Gain */
	else if (id == controls::microchip::BlueGain)
		cid = 0x009819c1; /* Blue Gain */
	else if (id == controls::microchip::GreenRedGain)
		cid = 0x009819c2; /* Green-Red Gain */
	else if (id == controls::microchip::GreenBlueGain)
		cid = 0x009819c3; /* Green-Blue Gain */
	else if (id == controls::microchip::RedOffset)
		cid = 0x009819c4; /* Red Offset */
	else if (id == controls::microchip::BlueOffset)
		cid = 0x009819c5; /* Blue Offset */
	else if (id == controls::microchip::GreenRedOffset)
		cid = 0x009819c6; /* Green-Red Offset */
	else if (id == controls::microchip::GreenBlueOffset)
		cid = 0x009819c7; /* Green-Blue Offset */
	else
		return -EINVAL;

	/* Validate and set the control value */
	const ControlInfo &controlInfo = controls->infoMap()->at(cid);

	switch (cid) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST: {
		float fval = value.get<float>();
		int32_t val = static_cast<int32_t>(std::lroundf(fval));
		int32_t min = controlInfo.min().get<int32_t>();
		int32_t max = controlInfo.max().get<int32_t>();
		controls->set(cid, std::clamp(val, min, max));
		break;
	}
	case 0x009819c0: /* Red Gain */
	case 0x009819c1: /* Blue Gain */
	case 0x009819c2: /* Green-Red Gain */
	case 0x009819c3: /* Green-Blue Gain */
	case 0x009819c4: /* Red Offset */
	case 0x009819c5: /* Blue Offset */
	case 0x009819c6: /* Green-Red Offset */
	case 0x009819c7: { /* Green-Blue Offset */
		int32_t val = value.get<int32_t>();
		int32_t min = controlInfo.min().get<int32_t>();
		int32_t max = controlInfo.max().get<int32_t>();
		controls->set(cid, std::clamp(val, min, max));
		break;
	}
	case V4L2_CID_AUTO_WHITE_BALANCE: {
		bool bval = value.get<bool>();
		controls->set(cid, static_cast<int32_t>(bval));
		break;
	}
	default:
		LOG(MicrochipISC, Debug) << "Control not yet supported";
		controls->set(cid, 0);
		break;
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
			return deviceNode;	/* Fallback to deviceNode if we can't find the path */
		}
		searchPath = searchPath.substr(0, pos);

		/* Check if this directory exists in the device tree */
		std::string dtPath = basePath + searchPath;
		if (std::filesystem::exists(dtPath) && std::filesystem::is_directory(dtPath)) {
			/* Found a matching directory in the device tree */
			LOG(MicrochipISC, Debug) << "Found device tree path: " << dtPath;
			return searchPath;	/* Return without the base path */
		}

		/* Check if this directory contains a 'compatible' file */
		std::string compatiblePath = dtPath + "/compatible";
		if (std::filesystem::exists(compatiblePath)) {
			std::ifstream compatibleFile(compatiblePath);
			std::string compatibleValue;
			std::getline(compatibleFile, compatibleValue);
			if (compatibleValue.find("microchip,isc") != std::string::npos) {
				LOG(MicrochipISC, Debug) << "Found ISC device tree node: " << searchPath;
				return searchPath;	/* Return without the base path */
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

	/* Find maximum supported resolution for the video device */
	Size maxVideoSize(0, 0);

	/* Get formats from the video device */
	V4L2VideoDevice::Formats formats = iscVideo_->formats();
	for (const auto &format : formats) {
		for (const auto &sizeRange : format.second) {
			/* Check if this format's maximum size is larger than our current maximum */
			if (sizeRange.max.width > maxVideoSize.width)
				maxVideoSize.width = sizeRange.max.width;
			if (sizeRange.max.height > maxVideoSize.height)
				maxVideoSize.height = sizeRange.max.height;
		}
	}

	LOG(MicrochipISC, Debug) << "Maximum video device resolution: " << maxVideoSize.toString();

	/* Try all formats from formatsMap_ */
	for (const auto &[pixelFormat, mbusFormat] : MicrochipISCCameraConfiguration::formatsMap_) {
		for (const Size &sensorSize : sensor_->sizes(mbusFormat)) {
			/* Create adjusted size that respects video device limits */
			Size adjustedSize = sensorSize;

			/* If the sensor size exceeds video device capability, adjust it */
			if (maxVideoSize.width > 0 && maxVideoSize.height > 0) {
				if (sensorSize.width > maxVideoSize.width) {
					LOG(MicrochipISC, Debug) << "Adjusting width from " << sensorSize.width
								 << " to " << maxVideoSize.width;
					adjustedSize.width = maxVideoSize.width;
				}

				if (sensorSize.height > maxVideoSize.height) {
					LOG(MicrochipISC, Debug) << "Adjusting height from " << sensorSize.height
								 << " to " << maxVideoSize.height;
					adjustedSize.height = maxVideoSize.height;
				}
			}

			tryPipeline(mbusFormat, adjustedSize);
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
			{ &controls::Brightness, ControlInfo(-1024.0f, 1024.0f, 0.0f) },
			{ &controls::Contrast, ControlInfo(-2048.0f, 2048.0f, 16.0f) },
			{ &controls::AwbEnable, ControlInfo(false, true, true) },
			{ &controls::Gamma, ControlInfo(0.0f, 0.0f, 0.0f) },/* actual min, max, default */
			/* Custom controls with values */
			{ &controls::microchip::RedGain, ControlInfo(0, 8191, 512) },
			{ &controls::microchip::BlueGain, ControlInfo(0, 8191, 512) },
			{ &controls::microchip::GreenRedGain, ControlInfo(0, 8191, 512) },
			{ &controls::microchip::GreenBlueGain, ControlInfo(0, 8191, 512) },
			{ &controls::microchip::RedOffset, ControlInfo(-4095, 4095, 0) },
			{ &controls::microchip::BlueOffset, ControlInfo(-4095, 4095, 0) },
			{ &controls::microchip::GreenRedOffset, ControlInfo(-4095, 4095, 0) },
			{ &controls::microchip::GreenBlueOffset, ControlInfo(-4095, 4095, 0) },
		}, controls::controls);

	LOG(MicrochipISC, Debug) << "ControlInfoMap initialized";

	/* Initialize AWB IPA if available */
	if (awbIPA_) {
		ipa::microchip_isc::MicrochipISCSensorInfo sensorInfo;
		sensorInfo.model = sensor_->model();
		sensorInfo.width = sensor_->resolution().width;
		sensorInfo.height = sensor_->resolution().height;

		auto format = sensor_->getFormat(sensor_->mbusCodes(), sensor_->resolution());
		sensorInfo.pixelFormat = format.code;

		/* Create empty stream config and control info maps */
		std::map<unsigned int, IPAStream> streamConfig;
		std::map<unsigned int, ControlInfoMap> entityControls;

		/* Add controls from video device and sensor */
		if (iscVideo_)
			entityControls[0] = iscVideo_->controls();
		if (sensor_)
			entityControls[1] = sensor_->controls();

		/* Configure IPA with all required parameters */
		int ret = awbIPA_->configure(sensorInfo, streamConfig, entityControls);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to configure AWB IPA";
			return ret;
		}
	}

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

void MicrochipISCCameraData::awbComplete([[maybe_unused]] unsigned int bufferId,
					 const ControlList &metadata)
{
	if (!iscVideo_)
		return;

	/* Skip hardware parameter application - we'll handle everything in software */
	/* Add all parameters to the request metadata */
	if (pipe_->processingRequest_) {
		bool metadataAdded = false;

		/* Add AGC and BLC parameters */
		constexpr uint32_t AUTO_GAIN_ID = 0x009819d1;
		constexpr uint32_t BLACK_LEVEL_ID = 0x009819d0;

		/* Forward AGC gain to request metadata */
		if (metadata.contains(AUTO_GAIN_ID)) {
			int32_t gainValue = metadata.get(AUTO_GAIN_ID).get<int32_t>();
			pipe_->processingRequest_->metadata().set(AUTO_GAIN_ID, gainValue);
			LOG(MicrochipISC, Debug) << "Added AGC gain to metadata: " << gainValue;
			metadataAdded = true;
		}

		/* Forward BLC level to request metadata */
		if (metadata.contains(BLACK_LEVEL_ID)) {
			int32_t blackLevel = metadata.get(BLACK_LEVEL_ID).get<int32_t>();
			pipe_->processingRequest_->metadata().set(BLACK_LEVEL_ID, blackLevel);
			LOG(MicrochipISC, Debug) << "Added BLC level to metadata: " << blackLevel;
			metadataAdded = true;
		}

		/* Forward AWB and offsets if available */
		constexpr std::array<uint32_t, 4> AWB_GAIN_IDS = {
			0x009819c0, 0x009819c1, 0x009819c2, 0x009819c3
		};

		constexpr std::array<uint32_t, 4> AWB_OFFSET_IDS = {
			0x009819c4, 0x009819c5, 0x009819c6, 0x009819c7
		};

		/* Add AWB gains to metadata with detailed logging */
		for (const auto &id : AWB_GAIN_IDS) {
			if (metadata.contains(id)) {
				int32_t value = metadata.get(id).get<int32_t>();
				pipe_->processingRequest_->metadata().set(id, value);
				LOG(MicrochipISC, Debug) << "Added AWB	(0x" << std::hex << id << std::dec << ")	to metadata: " << value;
				metadataAdded = true;
			}
		}

		/* Add AWB offsets to metadata with detailed logging */
		for (const auto &id : AWB_OFFSET_IDS) {
			if (metadata.contains(id)) {
				int32_t value = metadata.get(id).get<int32_t>();
				pipe_->processingRequest_->metadata().set(id, value);
				LOG(MicrochipISC, Debug) << "Added AWB	(0x" << std::hex << id << std::dec << ")	to metadata: " << value;
				metadataAdded = true;
			}
		}

		/* Forward CCM coefficients and offsets if available */
		constexpr std::array<uint32_t, 9> CCM_COEFF_IDS = {
			0x009819e0, 0x009819e1, 0x009819e2,  /* CCM_COEFF_00_ID, CCM_COEFF_01_ID, CCM_COEFF_02_ID */
			0x009819e3, 0x009819e4, 0x009819e5,  /* CCM_COEFF_10_ID, CCM_COEFF_11_ID, CCM_COEFF_12_ID */
			0x009819e6, 0x009819e7, 0x009819e8	 /* CCM_COEFF_20_ID, CCM_COEFF_21_ID, CCM_COEFF_22_ID */
		};

		constexpr std::array<uint32_t, 3> CCM_OFFSET_IDS = {
			0x009819e9, 0x009819ea, 0x009819eb	/* CCM_OFFSET_R_ID, CCM_OFFSET_G_ID, CCM_OFFSET_B_ID */
		};

		/* Add CCM matrix coefficients to metadata */
		for (const auto &id : CCM_COEFF_IDS) {
			if (metadata.contains(id)) {
				int32_t value = metadata.get(id).get<int32_t>();
				pipe_->processingRequest_->metadata().set(id, value);
				LOG(MicrochipISC, Debug) << "Added CCM coefficient (0x" << std::hex << id << std::dec << ") to metadata: " << value;
				metadataAdded = true;
			}
		}

		/* Add CCM offsets to metadata */
		for (const auto &id : CCM_OFFSET_IDS) {
			if (metadata.contains(id)) {
				int32_t value = metadata.get(id).get<int32_t>();
				pipe_->processingRequest_->metadata().set(id, value);
				LOG(MicrochipISC, Debug) << "Added CCM offset (0x" << std::hex << id << std::dec << ") to metadata: " << value;
				metadataAdded = true;
			}
		}

		/* Add temperature info if available */
		constexpr uint32_t SCENE_COLOR_CORRECTION_ID = 0x009819d2;
		if (metadata.contains(SCENE_COLOR_CORRECTION_ID)) {
			int32_t value = metadata.get(SCENE_COLOR_CORRECTION_ID).get<int32_t>();
			pipe_->processingRequest_->metadata().set(SCENE_COLOR_CORRECTION_ID, value);
			LOG(MicrochipISC, Debug) << "Added color temperature to metadata: " << value << "K";
			metadataAdded = true;
		}

		if (metadataAdded) {
			LOG(MicrochipISC, Debug) << "Added all ISC processing parameters to request metadata for software processing";
		}
	}

	/* Signal that processing is complete */
	pipe_->processingComplete_ = true;
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

int PipelineHandlerMicrochipISC::queueRequestDevice(Camera *camera, Request *request)
{
	MicrochipISCCameraData *data = cameraData(camera);

	/* Process controls first */
	int ret = processControls(data, request);
	if (ret < 0)
		return ret;

	/* Queue all buffers - don't try to process AWB here */
	for (Stream &stream : data->streams_) {
		FrameBuffer *buffer = request->findBuffer(&stream);
		if (buffer) {
			ret = data->iscVideo_->queueBuffer(buffer);
			if (ret < 0) {
				return ret;
			}
		}
	}

	return 0;
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

	/* Map the buffer memory to access the image data */
	void *mappedMemory = mmap(NULL, buffer->planes()[0].length, PROT_READ, MAP_SHARED,
				  buffer->planes()[0].fd.get(), buffer->planes()[0].offset);

	if (mappedMemory != MAP_FAILED) {
		/* Debug the buffer data */
		const uint8_t* bytes = static_cast<const uint8_t*>(mappedMemory);

		/* Check if buffer contains data */
		bool bufferHasData = false;
		for (size_t i = 0; i < std::min(buffer->planes()[0].length, static_cast<size_t>(100)); i++) {
			if (bytes[i] != 0) {
				bufferHasData = true;
				break;
			}
		}

		LOG(MicrochipISC, Debug) << "Buffer data all zeros: "
					 << (bufferHasData ? "no" : "yes");

		/* Debug first few bytes */
		std::stringstream hexBytes;
		hexBytes << "Buffer data (first 16 bytes): ";
		for (size_t i = 0; i < std::min(buffer->planes()[0].length, static_cast<size_t>(16)); i++) {
			hexBytes << std::hex << std::setw(2) << std::setfill('0')
				 << static_cast<int>(bytes[i]) << " ";
		}
		LOG(MicrochipISC, Debug) << hexBytes.str();

		/* Process the data with the IPA asynchronously */
		processingRequest_ = request;

		/* Create control list with pixel data for the IPA */
		ControlList controls(controls::controls);
		controls.set(ipa::microchip_isc::ISC_PIXEL_VALUES_ID,
			     Span<const uint8_t>(bytes, buffer->planes()[0].length));

		/* Process stats asynchronously - the awbComplete callback will be called later */
		if (activeData_ && activeData_->awbIPA_) {
			LOG(MicrochipISC, Debug) << "Sending buffer to IPA for async processing";
			activeData_->awbIPA_->processStats(controls);
		}

		munmap(mappedMemory, buffer->planes()[0].length);
	} else {
		LOG(MicrochipISC, Error) << "Failed to map buffer memory: " << strerror(errno);
	}

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

	if (data->awbIPA_)
		data->awbIPA_->stop();

	for (const auto &[name, subdev] : data->iscSubdev_) {
		subdev->close();
	}

	LOG(MicrochipISC, Debug) << "Device stopped for camera " << camera->id();
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMicrochipISC, "microchip-isc")

} /* namespace libcamera */
