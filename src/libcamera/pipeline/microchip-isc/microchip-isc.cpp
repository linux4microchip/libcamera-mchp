/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc.  All rights reserved.
 *
 * Pipeline Handler for Microchip eXtended Image Sensor Controller (XISC)
 */

#include <algorithm>
#include <math.h>
#include <errno.h>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string.h>
#include <string>
#include <vector>

#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/color_space.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/stream.h>
#include <libcamera/transform.h>

#include <libcamera/ipa/microchip_isc_ipa_interface.h>
#include <libcamera/ipa/microchip_isc_ipa_proxy.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/microchip_isc_controls.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(MicrochipISC)

class PipelineHandlerMicrochipISC;

/**
 * \class MicrochipISCCameraData
 * \brief Per-camera state for the Microchip ISC pipeline handler
 *
 * Owns the V4L2 video device, ISC subdevices, camera sensor, statistics
 * device, IPA proxy, and per-frame tracking map for a single ISC camera.
 * Created by PipelineHandlerMicrochipISC::match() and owned by the Camera.
 */
class MicrochipISCCameraData : public Camera::Private
{
public:
	MicrochipISCCameraData(PipelineHandler *pipe, std::shared_ptr<MediaDevice> media)
		: Camera::Private(pipe), media_(media),
		  statsEnabled_(false), statsStreaming_(false),
		  videoStreaming_(false)
	{
		streams_.resize(2);
		ipa_ = IPAManager::createIPA<ipa::microchip_isc::IPAProxyMicrochipISC>(pipe, 1, 1);
		if (ipa_) {
			ipa_->statsProcessed.connect(this, &MicrochipISCCameraData::statsProcessed);
			ipa_->setSensorControls.connect(this, &MicrochipISCCameraData::setSensorControls);
			ipa_->setIspControls.connect(this, &MicrochipISCCameraData::setIspControls);
		}
	}

	int init();
	int setupLinks();
	int initSubdev(MediaEntity *entity);
	int setupFormats(V4L2SubdeviceFormat *format, V4L2Subdevice::Whence whence);
	unsigned int getMediaBusFormat(PixelFormat *pixelFormat) const;
	int initStatsDevice();
	int allocateStatsBuffers();
	void freeStatsBuffers();
	void statsProcessed(unsigned int request, const ControlList &metadata);
	void setSensorControls(const ControlList &controls);
	void setIspControls(const ControlList &controls);

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
	std::unique_ptr<V4L2VideoDevice> statsDevice_;
	std::vector<std::unique_ptr<FrameBuffer>> statsBuffers_;
	std::queue<FrameBuffer *> availableStatsBuffers_;
	std::vector<IPABuffer> ipaBuffers_;
	std::vector<Stream> streams_;
	std::vector<Configuration> configs_;
	std::map<PixelFormat, std::vector<const Configuration *>> formats_;
	std::shared_ptr<MediaDevice> media_;
	std::unique_ptr<ipa::microchip_isc::IPAProxyMicrochipISC> ipa_;
	ControlInfoMap ipaControls_;
	bool statsEnabled_;
	bool statsStreaming_;
	bool videoStreaming_;

	struct FrameInfo {
		Request *request = nullptr;
		FrameBuffer *statsBuffer = nullptr;
		bool bufferDone = false;
		bool statsDone = false;
	};

	std::map<uint32_t, FrameInfo> frameInfo_;
	ControlList latestIpaMetadata_;

	FrameInfo *findFrameInfo(Request *request);
	FrameInfo *findFrameInfo(FrameBuffer *buffer);

private:
	void tryPipeline(unsigned int code, const Size &size);
};

/**
 * \class MicrochipISCCameraConfiguration
 * \brief Camera configuration for the Microchip ISC pipeline
 *
 * Validates and adjusts stream configurations against the set of supported
 * ISC pipeline configurations discovered at match() time. Holds a pointer
 * to the selected pipeline configuration for use during configure().
 */
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

/**
 * \class PipelineHandlerMicrochipISC
 * \brief Pipeline handler for the Microchip ISC / XISC image sensor controller
 *
 * Supports the Microchip SAMA5D2 ISC and SAMA7G5 XISC hardware blocks.
 * Handles sensor format negotiation, ISC pipeline link configuration,
 * Bayer histogram statistics delivery to the IPA, and application of
 * IPA-computed AWB/AGC/CCM/gamma/CBC controls to the ISC hardware.
 *
 * The IPA (ipa::microchip_isc) provides adaptive 3A (AWB, AGC/AEC, CCM)
 * and gamma tone mapping using the ISC 4-channel 512-bin histogram.
 */
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

private:
	MicrochipISCCameraData *activeData_ = nullptr;
	friend MicrochipISCCameraData;
	MicrochipISCCameraData *cameraData(Camera *camera)
	{
		return static_cast<MicrochipISCCameraData *>(camera->_d());
	}

	int processControl(ControlList *controls, unsigned int id, const ControlValue &value);
	int processControls(MicrochipISCCameraData *data, Request *request);
	void updateControls(MicrochipISCCameraData *data, const ControlInfoMap &ipaControls);
	void bufferReady(FrameBuffer *buffer);
	void statsBufferReady(FrameBuffer *buffer);
	void tryCompleteRequest(MicrochipISCCameraData *data,
				MicrochipISCCameraData::FrameInfo *info);
	Size findOptimalSize(const std::vector<MicrochipISCCameraData::Configuration> &configs,
			     const Size &target);
	PixelFormat findRawFormat(const std::vector<MicrochipISCCameraData::Configuration> &configs);
	bool isSupportedConfiguration(const std::vector<MicrochipISCCameraData::Configuration> &configs,
				      const StreamConfiguration &cfg);
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

MicrochipISCCameraData::FrameInfo *MicrochipISCCameraData::findFrameInfo(Request *request)
{
	auto it = frameInfo_.find(request->sequence());
	if (it == frameInfo_.end())
		return nullptr;

	return &it->second;
}

MicrochipISCCameraData::FrameInfo *MicrochipISCCameraData::findFrameInfo(FrameBuffer *buffer)
{
	for (auto &[sequence, info] : frameInfo_) {
		if (info.statsBuffer == buffer)
			return &info;
	}

	return nullptr;
}

int MicrochipISCCameraData::allocateStatsBuffers()
{
	if (!statsEnabled_ || !statsDevice_)
		return 0;

	if (!statsBuffers_.empty())
		return 0;

	constexpr unsigned int kStatsBufferCount = 4;
	int ret = statsDevice_->allocateBuffers(kStatsBufferCount, &statsBuffers_);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to allocate stats buffers: " << ret;
		return ret;
	}

	unsigned int ipaBufferId = 1;
	for (auto &buffer : statsBuffers_) {
		buffer->setCookie(ipaBufferId++);
		std::vector<FrameBuffer::Plane> planes(buffer->planes().begin(),
						       buffer->planes().end());
		ipaBuffers_.emplace_back(buffer->cookie(), std::move(planes));
	}

	if (ipa_)
		ipa_->mapBuffers(ipaBuffers_);

	/*
	 * Pre-queue all stats buffers to V4L2 so the stats DMA starts
	 * immediately at streamOn and runs independently of request
	 * queueing.  statsBufferReady() re-queues each buffer immediately
	 * after processing, keeping the ring full at all times.
	 */
	for (auto &buffer : statsBuffers_) {
		int qret = statsDevice_->queueBuffer(buffer.get());
		if (qret < 0) {
			LOG(MicrochipISC, Error)
				<< "Failed to pre-queue stats buffer: " << qret;
			return qret;
		}
	}

	return 0;
}

void MicrochipISCCameraData::freeStatsBuffers()
{
	while (!availableStatsBuffers_.empty())
		availableStatsBuffers_.pop();

	if (!ipaBuffers_.empty() && ipa_) {
		std::vector<unsigned int> ids;
		ids.reserve(ipaBuffers_.size());
		for (const IPABuffer &buffer : ipaBuffers_)
			ids.push_back(buffer.id);
		ipa_->unmapBuffers(ids);
	}
	ipaBuffers_.clear();

	if (statsDevice_)
		statsDevice_->releaseBuffers();

	statsBuffers_.clear();
}

void MicrochipISCCameraData::statsProcessed(unsigned int request,
					    const ControlList &metadata)
{
	/*
	 * Cache the latest IPA metadata for use by future frames.
	 * Stats are produced once per 4-frame Bayer cycle, so many frames
	 * complete before the IPA responds — the cached value is applied to
	 * each completed request in bufferReady() so that AwbLocked and
	 * ColourTemperature reach the application even with async IPC delay.
	 */
	latestIpaMetadata_ = metadata;

	/* Also apply directly if the originating frame is still pending */
	auto it = frameInfo_.find(request);
	if (it != frameInfo_.end()) {
		FrameInfo &info = it->second;
		info.request->metadata().merge(metadata,
					       ControlList::MergePolicy::OverwriteExisting);
	}
}

void MicrochipISCCameraData::setSensorControls(const ControlList &controls)
{
	if (!sensor_ || controls.empty())
		return;

	ControlList sensorControls(sensor_->controls());
	for (const auto &ctrl : controls)
		sensorControls.set(ctrl.first, ctrl.second);

	int ret = sensor_->setControls(&sensorControls);
	if (ret < 0)
		LOG(MicrochipISC, Error) << "Failed to apply sensor controls: " << ret;
}

void MicrochipISCCameraData::setIspControls(const ControlList &controls)
{
	if (!iscVideo_ || controls.empty())
		return;

	ControlList ispControls(iscVideo_->controls());
	for (const auto &ctrl : controls)
		ispControls.set(ctrl.first, ctrl.second);

	/*
	 * WB gain registers (ISC_WB_*) are double-buffered in hardware.
	 * The kernel ISC driver issues ISC_CTRL_UPPRO (profile update) inside
	 * the histogram worker ISR which fires every 4 frames after HISDONE.
	 * New gains written here take effect on the frame after the next
	 * ISC_CTRL_UPPRO pulse — typically 1–4 frames of latency.
	 * There is no V4L2 control to trigger ISC_CTRL_UPPRO directly from
	 * userspace; this latency is inherent to the hardware architecture.
	 */
	int ret = iscVideo_->setControls(&ispControls);
	if (ret < 0)
		LOG(MicrochipISC, Error) << "Failed to apply ISC controls: " << ret;
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
			cfg.colorSpace = ColorSpace::Srgb;
			cfg.size = findOptimalSize(data->configs_, Size(1920, 1080));
			cfg.bufferCount = 4;
			break;

		case StreamRole::VideoRecording:
			cfg.pixelFormat = formats::YUYV;
			cfg.colorSpace = ColorSpace::Sycc;
			cfg.size = findOptimalSize(data->configs_, Size(1920, 1080));
			cfg.bufferCount = 4;
			break;

		case StreamRole::Viewfinder:
			cfg.pixelFormat = formats::YUYV;
			cfg.colorSpace = ColorSpace::Sycc;
			cfg.size = findOptimalSize(data->configs_, Size(1280, 720));
			cfg.bufferCount = 2;
			break;

		case StreamRole::Raw:
			/*
			 * Raw Bayer output: select the highest-bit-depth Bayer
			 * format supported by the ISC at sensor resolution.
			 * IPA statistics are still available for raw streams;
			 * the histogram engine operates on the Bayer data before
			 * any ISC processing block.
			 */
			cfg.pixelFormat = findRawFormat(data->configs_);
			cfg.colorSpace = ColorSpace::Raw;
			cfg.size = findOptimalSize(data->configs_,
						   data->sensor_->resolution());
			cfg.bufferCount = 4;
			break;

		default:
			cfg.pixelFormat = data->configs_[0].captureFormat;
			cfg.colorSpace = ColorSpace::Sycc;
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

Size PipelineHandlerMicrochipISC::findOptimalSize(const std::vector<MicrochipISCCameraData::Configuration> &configs,
						  const Size &target)
{
	Size optimal = configs[0].captureSize;

	for (const auto &config : configs) {
		if (config.captureSize.width <= target.width &&
		    config.captureSize.height <= target.height &&
		    config.captureSize.width * config.captureSize.height >
			    optimal.width * optimal.height) {
			optimal = config.captureSize;
		}
	}

	return optimal;
}

PixelFormat PipelineHandlerMicrochipISC::findRawFormat(const std::vector<MicrochipISCCameraData::Configuration> &configs)
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

	for (const auto &config : configs) {
		if (std::find(rawFormats.begin(), rawFormats.end(), config.captureFormat) != rawFormats.end()) {
			return config.captureFormat;
		}
	}

	return configs[0].captureFormat; /* Fallback to default if no raw format found */
}

bool PipelineHandlerMicrochipISC::isSupportedConfiguration(const std::vector<MicrochipISCCameraData::Configuration> &configs,
							   const StreamConfiguration &cfg)
{
	for (const auto &config : configs) {
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

	std::shared_ptr<MediaDevice> media = acquireMediaDevice(enumerator, dm);
	if (!media) {
		return false;
	}

	std::unique_ptr<MicrochipISCCameraData> data = std::make_unique<MicrochipISCCameraData>(this, media);

	/* Initialize IPA first */
	if (data->ipa_) {
		LOG(MicrochipISC, Debug) << "IPA module loaded successfully";
	} else {
		LOG(MicrochipISC, Error) << "Failed to load IPA module";
		return false;
	}

	int ret;
	bool foundMainVideo = false;
	bool foundStatsVideo = false;

	/* Process all media entities */
	for (MediaEntity *entity : media->entities()) {
		switch (entity->function()) {
		case MEDIA_ENT_F_IO_V4L:
			if (entity->name().find("stats") != std::string::npos) {
				/* Handle stats device */
				LOG(MicrochipISC, Debug) << "Found stats device: " << entity->name();

				data->statsDevice_ = std::make_unique<V4L2VideoDevice>(entity);
				ret = data->statsDevice_->open();
				if (ret < 0) {
					LOG(MicrochipISC, Error) << "Failed to open stats device: " << ret;
					data->statsDevice_.reset();
					data->statsEnabled_ = false;
				} else {
					LOG(MicrochipISC, Debug) << "Opened stats device: " << entity->name();

					/* Set up stats format immediately after opening */
					V4L2DeviceFormat statsFormat;
					statsFormat.fourcc = V4L2PixelFormat(v4l2_fourcc('I', 'S', 'C', 'S'));
					statsFormat.size = Size(0, 0); /* Meta format doesn't need size */
					statsFormat.planesCount = 1;

					ret = data->statsDevice_->setFormat(&statsFormat);
					if (ret < 0) {
						LOG(MicrochipISC, Warning) << "Failed to set stats format: " << ret;
						/* Continue anyway - format might be set later */
					} else {
						LOG(MicrochipISC, Debug) << "Stats device format configured successfully";
					}

					/* Connect the  bufferReady signal AFTER successful open and format setup */
					data->statsDevice_->bufferReady.connect(this,
										&PipelineHandlerMicrochipISC::statsBufferReady);
					data->statsEnabled_ = true;
					foundStatsVideo = true;

					LOG(MicrochipISC, Debug) << "Connected stats buffer ready signal";
					LOG(MicrochipISC, Debug) << "Hardware histogram support enabled and ready";
				}
			} else {
				/* Handle main video device (capture) */
				LOG(MicrochipISC, Debug) << "Found main video device: " << entity->name();

				data->iscVideo_ = std::make_unique<V4L2VideoDevice>(entity);
				ret = data->iscVideo_->open();
				if (ret < 0) {
					LOG(MicrochipISC, Error) << "Failed to open video device: " << strerror(-ret);
					return false;
				}

				LOG(MicrochipISC, Debug) << "Opened video device: " << entity->name();

				/* Connect the bufferReady signal for main video */
				data->iscVideo_->bufferReady.connect(this, &PipelineHandlerMicrochipISC::bufferReady);
				foundMainVideo = true;
			}
			break;

		case MEDIA_ENT_F_CAM_SENSOR:
			/* Handle camera sensor */
			LOG(MicrochipISC, Debug) << "Found camera sensor: " << entity->name();
			ret = data->initSubdev(entity);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to initialize sensor: " << entity->name();
				return false;
			}
			break;

		case MEDIA_ENT_F_VID_IF_BRIDGE:
		case MEDIA_ENT_F_PROC_VIDEO_SCALER:
		case MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER:
			/* Handle ISC subdevices (scaler, CSI2DC, etc.) */
			LOG(MicrochipISC, Debug) << "Found ISC subdevice: " << entity->name();
			ret = data->initSubdev(entity);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to initialize subdevice: " << entity->name();
				return false;
			}
			break;

		default:
			/* Handle any other entities (including unknown types) */
			if (entity->deviceNode().empty()) {
				/* Skip entities without device nodes (pure media entities) */
				LOG(MicrochipISC, Debug) << "Skipping entity without device node: " << entity->name();
				continue;
			}

			LOG(MicrochipISC, Debug) << "Processing entity: " << entity->name()
						 << " (function: 0x" << std::hex << entity->function() << ")";
			ret = data->initSubdev(entity);
			if (ret < 0) {
				LOG(MicrochipISC, Warning) << "Failed to initialize entity: " << entity->name()
							   << " (non-fatal, continuing)";
				/* Don't fail completely for unknown entities */
			}
			break;
		}
	}

	/* Verify all essential components were found */
	if (!data->sensor_) {
		LOG(MicrochipISC, Error) << "Camera sensor not found";
		return false;
	}

	if (!foundMainVideo) {
		LOG(MicrochipISC, Error) << "Main video device not found";
		return false;
	}

	if (data->iscSubdev_.size() < 2) { /* Reduced requirement */
		LOG(MicrochipISC, Error) << "Insufficient ISC subdevices found: " << data->iscSubdev_.size();
		return false;
	}

	/* Log stats device status */
	if (foundStatsVideo) {
		LOG(MicrochipISC, Debug) << "Hardware histogram support: enabled";
		LOG(MicrochipISC, Debug) << "Stats device ready for 512-bin Bayer histograms";
	} else {
		LOG(MicrochipISC, Debug) << "Hardware histogram support: disabled (stats device not found)";
	}

	/* Initialize IPA with tuning file */
	if (data->ipa_) {
		std::string sensorModel = data->sensor_->model();
		std::string ipaTuningFile =
			data->ipa_->configurationFile(sensorModel + ".yaml",
						      "uncalibrated.yaml");
		ipa::microchip_isc::IPAConfigInfo ipaConfig{};
		ret = data->sensor_->sensorInfo(&ipaConfig.sensorInfo);
		if (ret) {
			LOG(MicrochipISC, Error) << "Camera sensor information not available";
			return false;
		}
		ipaConfig.sensorControls = data->sensor_->controls();
		if (data->iscVideo_)
			ipaConfig.ispControls = data->iscVideo_->controls();

		/*
		 * Detect ISC hardware variant from the ISP control set.
		 * V4L2_CID_HUE is only registered by the SAMA7G5/SAM9X7 CBHS
		 * block; its presence uniquely identifies the XISC variant which
		 * also carries DPC and bipartite gamma. The SAMA5D2 ISC registers
		 * none of these three controls.
		 */
		bool isXisc = (data->iscVideo_ &&
			       data->iscVideo_->controls().count(V4L2_CID_HUE) > 0);
		ipaConfig.hwCaps.hasDpc = isXisc;
		ipaConfig.hwCaps.hasBipartiteGamma = isXisc;
		ipaConfig.hwCaps.hasCbhs = isXisc;
		ipaConfig.hwCaps.statsAvailable = foundStatsVideo;

		ControlInfoMap ipaControls;
		ret = data->ipa_->init({ ipaTuningFile, sensorModel }, ipaConfig,
				       &ipaControls);
		if (ret) {
			LOG(MicrochipISC, Error) << "Failed to initialise IPA";
			return false;
		}
		data->ipaControls_ = ipaControls;

		LOG(MicrochipISC, Debug) << "IPA initialised with tuning file: "
					 << ipaTuningFile;
	}

	/* Initialize the camera data */
	ret = data->init();
	if (ret) {
		LOG(MicrochipISC, Error) << "Failed to initialize camera data";
		return false;
	}
	if (data->ipa_)
		updateControls(data.get(), data->ipaControls_);

	/* Create and register the camera */
	std::set<Stream *> streams;
	for (Stream &stream : data->streams_)
		streams.insert(&stream);

	std::string cameraId = data->sensor_->id();
	std::shared_ptr<Camera> camera = Camera::create(std::move(data), cameraId, streams);
	registerCamera(std::move(camera));

	LOG(MicrochipISC, Info) << "Registered camera '" << cameraId << "'";

	/* Final status summary */
	if (foundStatsVideo) {
		LOG(MicrochipISC, Info) << " Camera registration complete with HARDWARE HISTOGRAM support";
	} else {
		LOG(MicrochipISC, Info) << " Camera registration complete without hardware histogram support";
	}

	return true;
}

int PipelineHandlerMicrochipISC::configure(Camera *camera, CameraConfiguration *c)
{
	MicrochipISCCameraData *data = cameraData(camera);
	activeData_ = data;
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
	if (!pipeConfig) {
		LOG(MicrochipISC, Error) << "Invalid configuration";
		return -EINVAL;
	}

	V4L2SubdeviceFormat sensorFormat{};
	sensorFormat.code = pipeConfig->code;
	sensorFormat.size = pipeConfig->sensorSize;

	LOG(MicrochipISC, Debug) << "Using validated config: sensor="
				 << sensorFormat.size.width << "x" << sensorFormat.size.height;

	/* Configure the sensor with validated format */
	ret = data->sensor_->setFormat(&sensorFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to set sensor format: " << ret;
		return ret;
	}

	LOG(MicrochipISC, Debug) << "Sensor format: " << sensorFormat.toString();

	/* Configure the ISC with the sensor format */
	V4L2SubdeviceFormat iscFormat = sensorFormat;
	ret = data->setupFormats(&iscFormat, V4L2Subdevice::ActiveFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to setup ISC formats: " << ret;
		return ret;
	}

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = data->iscVideo_->toV4L2PixelFormat(pipeConfig->captureFormat);
	captureFormat.size = pipeConfig->captureSize;

	LOG(MicrochipISC, Debug) << "Using validated config: capture="
				 << captureFormat.size.width << "x" << captureFormat.size.height;
	LOG(MicrochipISC, Debug) << "Capture format configured: " << captureFormat.toString();

	ret = data->iscVideo_->setFormat(&captureFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to set video device format: " << ret;
		return ret;
	}

	/* Configure IPA if available */
	if (data->ipa_) {
		ipa::microchip_isc::IPAConfigInfo ipaConfig{};
		ret = data->sensor_->sensorInfo(&ipaConfig.sensorInfo);
		if (ret) {
			LOG(MicrochipISC, Error) << "Camera sensor information not available";
			return ret;
		}
		ipaConfig.sensorControls = data->sensor_->controls();
		if (data->iscVideo_)
			ipaConfig.ispControls = data->iscVideo_->controls();

		bool isXisc = (data->iscVideo_ &&
			       data->iscVideo_->controls().count(V4L2_CID_HUE) > 0);
		ipaConfig.hwCaps.hasDpc = isXisc;
		ipaConfig.hwCaps.hasBipartiteGamma = isXisc;
		ipaConfig.hwCaps.hasCbhs = isXisc;
		ipaConfig.hwCaps.statsAvailable = data->statsEnabled_;

		uint8_t bayerOrder = static_cast<uint8_t>(
			data->sensor_->bayerOrder(Transform::Identity));

		ControlInfoMap ipaControls;
		ret = data->ipa_->configure(ipaConfig, bayerOrder, &ipaControls);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to configure IPA: " << ret;
			return ret;
		}
		data->ipaControls_ = ipaControls;
		updateControls(data, ipaControls);
	}

	/* Set stream configurations with validated sizes */
	for (unsigned int i = 0; i < c->size(); ++i) {
		StreamConfiguration &cfg = c->at(i);
		cfg.setStream(&data->streams_[i]);
		cfg.stride = captureFormat.planes[0].bpl;
		cfg.size = captureFormat.size;
	}

	LOG(MicrochipISC, Info) << "Camera configuration complete: " << camera->id();
	return 0;
}

void PipelineHandlerMicrochipISC::updateControls(MicrochipISCCameraData *data,
						 const ControlInfoMap &ipaControls)
{
	ControlInfoMap::Map controls;
	for (const auto &ctrl : data->controlInfo_)
		controls[ctrl.first] = ctrl.second;
	for (const auto &ctrl : ipaControls)
		controls[ctrl.first] = ctrl.second;

	data->controlInfo_ = ControlInfoMap(std::move(controls), controls::controls);
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

	LOG(MicrochipISC, Debug) << "Starting camera: " << camera->id();

	data->frameInfo_.clear();

	/* Configure hardware */
	ControlList awbCtrls(data->iscVideo_->controls());
	awbCtrls.set(V4L2_CID_AUTO_WHITE_BALANCE, 0);
	data->iscVideo_->setControls(&awbCtrls);

	if (data->ipa_) {
		int ipaRet = data->ipa_->start();
		if (ipaRet < 0) {
			LOG(MicrochipISC, Error) << "Failed to start IPA: " << ipaRet;
			return ipaRet;
		}
	}

	/* Allocate and map stats buffers */
	if (data->statsEnabled_) {
		int statsRet = data->allocateStatsBuffers();
		if (statsRet < 0) {
			if (data->ipa_)
				data->ipa_->stop();
			return statsRet;
		}

		LOG(MicrochipISC, Debug) << "Stats buffers allocated, starting streamOn";
		statsRet = data->statsDevice_->streamOn();
		if (statsRet < 0) {
			LOG(MicrochipISC, Error) << "Failed to start stats streaming: " << statsRet;
			data->freeStatsBuffers();
			if (data->ipa_)
				data->ipa_->stop();
			return statsRet;
		}
		data->statsStreaming_ = true;
	}

	/*
	 * Import buffers but do NOT call streamOn() yet. The ISC kernel
	 * driver's start_streaming() dequeues the first buffer from the
	 * DMA list immediately. If STREAMON is issued before any buffer
	 * has been queued via queueRequest() the DMA queue is empty and
	 * subsequent QBUF calls return -ETIMEDOUT.
	 *
	 * streamOn() is deferred to the first queueBuffer() call where
	 * the kernel DMA queue already contains at least one buffer.
	 * This matches the pattern used by mainline libcamera pipelines
	 * (simple, rkisp1) that drive V4L2 devices with the same constraint.
	 */
	unsigned int count = 8;
	int ret = data->iscVideo_->importBuffers(count);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to import buffers: " << ret;
		if (data->statsEnabled_ && data->statsStreaming_) {
			data->statsDevice_->streamOff();
			data->statsStreaming_ = false;
		}
		data->freeStatsBuffers();
		if (data->ipa_)
			data->ipa_->stop();
		return ret;
	}

	data->videoStreaming_ = false;

	LOG(MicrochipISC, Debug) << "Camera ready — video streamOn deferred to first buffer";
	return 0;
}

int PipelineHandlerMicrochipISC::processControl(ControlList *controls, unsigned int id,
						const ControlValue &value)
{
	uint32_t cid;
	bool isIpaHandled = false;

	/* Map libcamera controls to V4L2 controls */
	if (id == controls::Brightness)
		cid = V4L2_CID_BRIGHTNESS;
	else if (id == controls::Contrast)
		cid = V4L2_CID_CONTRAST;
	else if (id == controls::microchip::RedGain)
		cid = V4L2_CID_ISC_WB_R_GAIN;
	else if (id == controls::microchip::BlueGain)
		cid = V4L2_CID_ISC_WB_B_GAIN;
	else if (id == controls::microchip::GreenRedGain)
		cid = V4L2_CID_ISC_WB_GR_GAIN;
	else if (id == controls::microchip::GreenBlueGain)
		cid = V4L2_CID_ISC_WB_GB_GAIN;
	else if (id == controls::microchip::RedOffset)
		cid = V4L2_CID_ISC_WB_R_OFFSET;
	else if (id == controls::microchip::BlueOffset)
		cid = V4L2_CID_ISC_WB_B_OFFSET;
	else if (id == controls::microchip::GreenRedOffset)
		cid = V4L2_CID_ISC_WB_GR_OFFSET;
	else if (id == controls::microchip::GreenBlueOffset)
		cid = V4L2_CID_ISC_WB_GB_OFFSET;
	else if (id == controls::AnalogueGain) {
		cid = V4L2_CID_ANALOGUE_GAIN;
		isIpaHandled = true;
	} else if (id == controls::DigitalGain) {
		cid = V4L2_CID_DIGITAL_GAIN;
		isIpaHandled = true;
	} else if (id == controls::AwbEnable) {
		isIpaHandled = true;
	} else if (id == controls::ExposureTime) {
		cid = V4L2_CID_EXPOSURE;
		isIpaHandled = true;
	} else
		return -EINVAL;

	/* Handle sensor controls differently */
	if (isIpaHandled) {
		return 0; /* Handled by IPA via queueRequest, not mapped to V4L2 */
	}

	switch (cid) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST: {
		/*
		 * Clamp to the hardware range when the control is registered
		 * in the ISP's ControlInfoMap; pass through unclamped otherwise.
		 * Use find() rather than at() to avoid throwing on unknown IDs.
		 */
		float fval = value.get<float>();
		int32_t val = static_cast<int32_t>(std::lroundf(fval));
		const ControlInfoMap *infoMap = controls->infoMap();
		if (infoMap) {
			auto it = infoMap->find(cid);
			if (it != infoMap->end()) {
				int32_t min = it->second.min().get<int32_t>();
				int32_t max = it->second.max().get<int32_t>();
				val = std::clamp(val, min, max);
			}
		}
		controls->set(cid, val);
		break;
	}
	case V4L2_CID_ISC_WB_R_GAIN:
	case V4L2_CID_ISC_WB_B_GAIN:
	case V4L2_CID_ISC_WB_GR_GAIN:
	case V4L2_CID_ISC_WB_GB_GAIN:
	case V4L2_CID_ISC_WB_R_OFFSET:
	case V4L2_CID_ISC_WB_B_OFFSET:
	case V4L2_CID_ISC_WB_GR_OFFSET:
	case V4L2_CID_ISC_WB_GB_OFFSET: {
		int32_t val = value.get<int32_t>();
		const ControlInfoMap *infoMap = controls->infoMap();
		if (infoMap) {
			auto it = infoMap->find(cid);
			if (it != infoMap->end()) {
				int32_t min = it->second.min().get<int32_t>();
				int32_t max = it->second.max().get<int32_t>();
				val = std::clamp(val, min, max);
			}
		}
		controls->set(cid, val);
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

int MicrochipISCCameraData::initSubdev(MediaEntity *entity)
{
	if (entity->function() == MEDIA_ENT_F_CAM_SENSOR) {
		sensor_ = CameraSensorFactoryBase::create(entity);
		if (!sensor_) {
			LOG(MicrochipISC, Error) << "Failed to create camera sensor";
			return -EINVAL;
		}
		return 0;
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
					      { &controls::Gamma, ControlInfo(0.0f, 0.0f, 0.0f) }, /* actual min, max, default */
					      /* NEW: Sensor hardware controls */
					      { &controls::AnalogueGain, ControlInfo(0.0f, 23.2f, 0.0f) }, /* 0-232 mapped to 0-23.2x */
					      { &controls::DigitalGain, ControlInfo(1.0f, 16.0f, 1.0f) }, /* 256-4095 mapped to 1-16x */
					      { &controls::ExposureTime, ControlInfo(4, 3522, 1600) }, /* Direct microsecond values */
					      /* Custom controls with values */
					      { &controls::microchip::RedGain, ControlInfo(0, 8191, 512) },
					      { &controls::microchip::BlueGain, ControlInfo(0, 8191, 512) },
					      { &controls::microchip::GreenRedGain, ControlInfo(0, 8191, 512) },
					      { &controls::microchip::GreenBlueGain, ControlInfo(0, 8191, 512) },
					      { &controls::microchip::RedOffset, ControlInfo(-4095, 4095, 0) },
					      { &controls::microchip::BlueOffset, ControlInfo(-4095, 4095, 0) },
					      { &controls::microchip::GreenRedOffset, ControlInfo(-4095, 4095, 0) },
					      { &controls::microchip::GreenBlueOffset, ControlInfo(-4095, 4095, 0) },
				      },
				      controls::controls);

	LOG(MicrochipISC, Debug) << "ControlInfoMap initialized";

	LOG(MicrochipISC, Debug) << "Camera data initialized successfully";

	/* Initialize hardware stats device */
	int ret = initStatsDevice();
	if (ret < 0) {
		LOG(MicrochipISC, Warning) << "Stats device initialization failed !";
	}
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

int MicrochipISCCameraData::initStatsDevice()
{
	if (!statsDevice_) {
		LOG(MicrochipISC, Warning) << "No stats device found during initialization - hardware histogram unavailable";
		statsEnabled_ = false;
		return 0; /* Not fatal - basic camera operation can continue */
	}

	/* Verify the format matches our expectation */
	V4L2DeviceFormat statsFormat;
	int ret = statsDevice_->getFormat(&statsFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to get stats device format: " << ret;
		statsEnabled_ = false;
		return 0;
	}

	/* Log the stats format details */
	LOG(MicrochipISC, Debug) << "Stats device format: " << statsFormat.fourcc
				 << " size: " << statsFormat.size.width << "x" << statsFormat.size.height;

	LOG(MicrochipISC, Debug) << "ISC hardware stats device ready - signal connected and waiting for histogram data";
	return 0;
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

	MicrochipISCCameraData::FrameInfo frameInfo;
	frameInfo.request = request;
	/*
	 * Stats buffers are pre-queued to V4L2 at startDevice and recycled
	 * immediately in statsBufferReady() — no per-request management.
	 */
	frameInfo.statsDone = !data->statsEnabled_ || !data->ipa_;

	data->frameInfo_[request->sequence()] = frameInfo;

	if (data->ipa_)
		data->ipa_->queueRequest(request->sequence(), request->controls());

	int ret = processControls(data, request);
	if (ret < 0) {
		data->frameInfo_.erase(request->sequence());
		return ret;
	}

	for (Stream &stream : data->streams_) {
		FrameBuffer *buffer = request->findBuffer(&stream);
		if (buffer) {
			ret = data->iscVideo_->queueBuffer(buffer);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to queue buffer: " << ret;
				return ret;
			}
		}
	}

	/*
	 * Deferred streamOn: issue VIDIOC_STREAMON only after the first
	 * buffer has been queued into the kernel DMA list. The ISC
	 * start_streaming() handler immediately dequeues the first entry
	 * from the DMA queue; calling STREAMON before any buffer is present
	 * leaves the queue empty and all subsequent QBUF calls return
	 * -ETIMEDOUT.
	 */
	if (!data->videoStreaming_) {
		ret = data->iscVideo_->streamOn();
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to start video streaming: " << ret;
			return ret;
		}
		data->videoStreaming_ = true;
		LOG(MicrochipISC, Debug) << "Video streamOn issued on first buffer";
	}

	return 0;
}

void PipelineHandlerMicrochipISC::bufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();
	if (!request) {
		LOG(MicrochipISC, Warning) << "Buffer without associated request";
		return;
	}

	MicrochipISCCameraData *data = activeData_;
	if (!data) {
		LOG(MicrochipISC, Error) << "No active camera data";
		return;
	}

	MicrochipISCCameraData::FrameInfo *info = data->findFrameInfo(request);
	if (!info) {
		/*
		 * No frame info — request was requeued after captureReady_ was
		 * set (race between app requeue and main-thread stop()).
		 * Complete it immediately so queuedRequests_ drains and
		 * stop() does not assert.
		 */
		LOG(MicrochipISC, Warning) << "No frame info for request "
					   << request->sequence()
					   << " — completing as stale";
		completeBuffer(request, buffer);
		completeRequest(request);
		return;
	}

	if (buffer->metadata().timestamp) {
		request->metadata().set(controls::SensorTimestamp, buffer->metadata().timestamp);
	}

	/*
	 * Merge the most recent IPA metadata (AwbLocked, ColourTemperature, etc.)
	 * into this request.  The IPA responds asynchronously and statsProcessed()
	 * caches the latest result; applying it here ensures every completed frame
	 * carries up-to-date convergence state for the application.
	 */
	if (!data->latestIpaMetadata_.empty())
		request->metadata().merge(data->latestIpaMetadata_,
					  ControlList::MergePolicy::OverwriteExisting);

	completeBuffer(request, buffer);
	info->bufferDone = true;
	tryCompleteRequest(data, info);
}

void PipelineHandlerMicrochipISC::statsBufferReady(FrameBuffer *buffer)
{
	LOG(MicrochipISC, Debug) << "statsBufferReady called cookie=" << buffer->cookie();
	MicrochipISCCameraData *data = activeData_;
	if (!data) {
		LOG(MicrochipISC, Error) << "No active camera data";
		return;
	}

	/*
	 * Stats buffers cycle independently of image buffers.  Find the
	 * oldest in-flight frame that has not yet been processed (lowest
	 * sequence number with statsDone == false).  This is always the
	 * correct frame because the stats DMA ring and the image DMA ring
	 * advance in lockstep and V4L2 returns buffers in FIFO order.
	 */
	MicrochipISCCameraData::FrameInfo *info = nullptr;
	for (auto &[seq, fi] : data->frameInfo_) {
		if (!fi.statsDone) {
			info = &fi;
			break;
		}
	}

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		/* Stop in progress — mark done so the frame can complete */
		if (info) {
			info->statsDone = true;
			tryCompleteRequest(data, info);
		}
		return; /* Do NOT re-queue — streamOff is imminent */
	}

	/* Re-queue immediately to keep the stats DMA ring alive */
	if (data->statsStreaming_)
		data->statsDevice_->queueBuffer(buffer);

	if (!info) {
		LOG(MicrochipISC, Warning) << "Stats buffer ready with no pending frame";
		return;
	}

	if (!data->ipa_ || !data->sensor_) {
		info->statsDone = true;
		tryCompleteRequest(data, info);
		return;
	}

	static const std::array<uint32_t, 2> sensorCtrlIds = {
		V4L2_CID_EXPOSURE, V4L2_CID_ANALOGUE_GAIN
	};
	ControlList sensorControls = data->sensor_->getControls(sensorCtrlIds);
	data->ipa_->processStats(info->request->sequence(), buffer->cookie(),
				 sensorControls);

	info->statsDone = true;
	tryCompleteRequest(data, info);
}

void PipelineHandlerMicrochipISC::tryCompleteRequest(
	MicrochipISCCameraData *data,
	MicrochipISCCameraData::FrameInfo *info)
{
	if (!info->bufferDone)
		return;

	if (info->request->hasPendingBuffers())
		return;

	/*
	 * Stats buffers run independently and are recycled in
	 * statsBufferReady() — statsDone no longer gates request completion.
	 */
	data->frameInfo_.erase(info->request->sequence());
	completeRequest(info->request);
}

void PipelineHandlerMicrochipISC::stopDevice(Camera *camera)
{
	MicrochipISCCameraData *data = cameraData(camera);

	if (data->ipa_)
		data->ipa_->stop();

	if (data->statsEnabled_ && data->statsStreaming_) {
		data->statsDevice_->streamOff();
		data->statsStreaming_ = false;
	}

	data->freeStatsBuffers();

	if (data->iscVideo_) {
		if (data->videoStreaming_) {
			data->iscVideo_->streamOff();
			data->videoStreaming_ = false;
		}
		data->iscVideo_->releaseBuffers();
	}

	/*
	 * Clear frame info after streamOff so that any bufferReady() callbacks
	 * fired during streamOff can still find their entries and complete cleanly,
	 * avoiding "No frame info for request N" warnings.
	 */
	data->frameInfo_.clear();

	/*
	 * Subdevices (sensor, ISC blocks) are opened for the lifetime of the
	 * pipeline handler, not per-streaming session. Do not close them here;
	 * they are closed when the pipeline handler is destroyed.
	 */
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMicrochipISC, "microchip-isc")

} /* namespace libcamera */
