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

#include <libcamera/base/file.h>
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
#include "libcamera/internal/yaml_parser.h"

#include "isc_postproc.h"

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
		  videoStreaming_(false),
		  startFailed_(false)
	{
		/*
		 * ISC has a single DMA output — only one stream is ever active.
		 * Sized 2 previously for a future multi-stream extension that is
		 * explicitly rejected in generateConfiguration(). Size to 1 to
		 * avoid the queueRequestDevice loop silently skipping streams_[1]
		 * (which has no buffer) on every request, and to avoid confusing
		 * any consumer that iterates streams_ expecting real entries.
		 */
		streams_.resize(1);
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
		/*
		 * sensorSize: the user-visible / config-matching size. This is
		 * the capture size clamped to the ISC video device maximum.
		 * Used by generateConfiguration() to match user requests.
		 *
		 * nativeSensorSize: the actual size the sensor hardware
		 * negotiates to. Some sensors snap to a wider native readout
		 * (e.g. IMX219 snaps 3264→3280 at max resolution). The
		 * subdevice chain (sensor→CSI→ISC scaler) must be configured
		 * at nativeSensorSize so the kernel ISC driver sees the correct
		 * RAW mbus code at the scaler input and sets bits_pipeline with
		 * CFA_ENABLE|WB_ENABLE|GAM_ENABLE. The video device (DMA out)
		 * is set to captureSize; the ISC scaler crops the difference.
		 * Without this separation the kernel falls back to
		 * bits_pipeline=0 (raw passthrough), bypassing all ISP stages.
		 */
		Size sensorSize;
		Size nativeSensorSize;
		PixelFormat captureFormat;
		Size captureSize;
		std::vector<PixelFormat> outputFormats;
		SizeRange outputSizes;
	};

	std::unique_ptr<CameraSensor> sensor_;
	/*
	 * ISC subdevices in pipeline order: sensor → bridge(s) → scaler.
	 * Format negotiation (setupFormats) must propagate from sensor outward;
	 * std::map orders by name (alphabetical) which is not pipeline order
	 * and would set formats on the scaler before the bridge on some systems.
	 * std::vector preserves insertion order from match() which walks the
	 * media graph topology — correct pipeline order.
	 */
	std::vector<std::pair<std::string, std::unique_ptr<V4L2Subdevice>>> iscSubdev_;
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
	/*
	 * startFailed_: set when VIDIOC_STREAMON returns an error.
	 * Subsequent queueBuffer() calls return -EIO immediately rather than
	 * retrying streamOn(), preventing the 4× EREMOTEIO log storm and
	 * ensuring frameInfo_ does not accumulate multiple failed entries
	 * that would each trigger the completeBuffer() ASSERT in stopDevice().
	 * Cleared in startDevice() so the next camera open gets a clean slate.
	 */
	bool startFailed_;

	struct FrameInfo {
		Request *request = nullptr;
		bool bufferDone = false;
		/*
		 * statsDone: set true when processStats() has been called for
		 * this frame. Used only by stopDevice() to distinguish frames
		 * whose image DMA stalled (bufferDone=false) from those that
		 * completed normally. Stats buffers are recycled independently
		 * and do NOT gate request completion (tryCompleteRequest only
		 * checks bufferDone + hasPendingBuffers).
		 */
		bool statsDone = false;
	};

	std::map<uint32_t, FrameInfo> frameInfo_;
	ControlList latestIpaMetadata_;

	/*
	 * Software lens-shading stage. The ISC has no hardware block for it, so
	 * it runs here on the finished output buffer before completion, serving
	 * every client.
	 */
	std::unique_ptr<IscPostProcessor> postproc_;

	FrameInfo *findFrameInfo(Request *request);

private:
	void tryPipeline(unsigned int code, const Size &size, const Size &maxVideoSize);
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
	void postProcDone(FrameBuffer *buffer);
	void completeFrameBuffer(MicrochipISCCameraData *data, FrameBuffer *buffer);
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
	const auto &fmtMap = MicrochipISCCameraConfiguration::formatsMap_;
	auto it = fmtMap.find(*pixelFormat);
	if (it != fmtMap.end())
		return it->second;

	/* Requested format not in the map — fall back to YUYV. */
	*pixelFormat = formats::YUYV;
	auto yuvIt = fmtMap.find(formats::YUYV);
	if (yuvIt == fmtMap.end()) {
		LOG(MicrochipISC, Error) << "YUYV fallback not in formatsMap_";
		return 0;
	}
	return yuvIt->second;
}

MicrochipISCCameraData::FrameInfo *MicrochipISCCameraData::findFrameInfo(Request *request)
{
	auto it = frameInfo_.find(request->sequence());
	if (it == frameInfo_.end())
		return nullptr;

	return &it->second;
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
		info.request->_d()->metadata().merge(metadata,
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
	 * The kernel ISC driver calls isc_update_profile() from isc_s_ctrl()
	 * on every V4L2 setControls ioctl. isc_update_profile() issues
	 * ISC_CTRL_UPPRO and polls until the hardware double-buffer commits
	 * before returning — gains are in hardware by the time the ioctl
	 * returns. One pipeline frame is still needed for the committed gains
	 * to propagate through the ISC processing pipeline and appear in the
	 * DMA output.
	 */
	int ret = iscVideo_->setControls(&ispControls);
	if (ret < 0)
		LOG(MicrochipISC, Error) << "Failed to apply ISC controls: " << ret;
}

/*
 * formatsMap_ maps a libcamera PixelFormat to the V4L2 media-bus format
 * code used by the sensor subdevice chain. This map serves TWO purposes:
 *
 *  1. Sensor enumeration (init): only Bayer mbus codes are meaningful here —
 *     the ISC sensor always outputs raw Bayer data. ISC output formats
 *     (RGB565, YUYV, UYVY) are produced by the ISC post-processing pipeline
 *     and are NOT sensor mbus formats. For sensor enumeration, only the Bayer
 *     entries are used; the ISC output entries yield empty sensor size lists
 *     (sensor_->sizes(MEDIA_BUS_FMT_RGB565_2X8_LE) == {}) and are harmless
 *     but wasteful. See sensorMbusFormats_ below for the correct subset.
 *
 *  2. Format advertisement (validate / getMediaBusFormat): all entries are
 *     valid — the ISC converts Bayer to any of these output formats.
 *
 * The mix of sensor-mbus and ISC-output-format entries in one map is a
 * historical artefact. The correct fix is to use sensorMbusFormats_ for
 * enumeration and formatsMap_ only for advertisement. This is done in init().
 */
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
};

/*
 * Sensor mbus format codes used for pipeline enumeration only.
 * These are the Bayer codes that the ISC sensor outputs; ISC post-processing
 * converts them to ISC output formats (RGB565, YUYV, etc.) downstream.
 * Using only these for sensor_->sizes() enumeration avoids spurious
 * tryPipeline() calls for ISC-output mbus codes that the sensor never
 * produces, which previously caused O(n_formats × n_sizes) duplicate entries
 * in configs_ for every sensor mode.
 */
static const std::vector<unsigned int> kSensorMbusFormats = {
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
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

	/*
	 * The ISC has a single DMA output — it cannot drive two independent
	 * streams simultaneously. Reject multi-stream requests early so the
	 * caller gets a clear nullptr rather than a silently broken config
	 * where both streams share the same pipe config from the last role.
	 */
	if (roles.size() > 1) {
		LOG(MicrochipISC, Error)
			<< "Multi-stream configuration not supported "
			<< "(requested " << roles.size() << " streams, max 1)";
		return nullptr;
	}

	/* Safety: no sensor modes enumerated — hardware not usable. */
	if (data->configs_.empty()) {
		LOG(MicrochipISC, Error) << "No sensor configurations available";
		return nullptr;
	}

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
	/* Guard: callers check configs_.empty() before reaching here, but be safe. */
	if (configs.empty())
		return Size(0, 0);

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
	if (configs.empty())
		return PixelFormat{};

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

	/* Tuning file resolved below; also read for the post-processing modules. */
	std::string ipaTuningFile;

	/* Initialize IPA with tuning file */
	if (data->ipa_) {
		std::string sensorModel = data->sensor_->model();

		/*
		 * Resolve the tuning file for the sensor. configurationFile()
		 * honours the LIBCAMERA_MICROCHIP-ISC_TUNING_FILE environment
		 * variable, the standard libcamera override used to select a
		 * sensor variant (e.g. imx219_noir.yaml) — the --noir and
		 * --tuning-file options of mchpcam-still set it.
		 */
		ipaTuningFile = data->ipa_->configurationFile(sensorModel + ".yaml",
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

	/*
	 * Software lens-shading stage. Created on the CameraManager thread so
	 * its completion signal is delivered back here, on the same thread as
	 * bufferReady().
	 */
	data->postproc_ = std::make_unique<IscPostProcessor>();
	data->postproc_->frameProcessed.connect(this,
						&PipelineHandlerMicrochipISC::postProcDone);

	/*
	 * The lens-shading map is calibration for the output path, not an IPA
	 * algorithm, so it lives in the tuning file's top-level 'modules:' list
	 * rather than 'algorithms:' (the same split rkisp1 uses for its dewarp
	 * parameters). Re-open the resolved tuning file and hand the matching
	 * module block to the post-processor. A file with no 'modules:' section
	 * simply leaves the stage inactive, so behaviour is unchanged from the
	 * pre-calibration tuning files.
	 */
	if (!ipaTuningFile.empty()) {
		File file(ipaTuningFile);
		if (file.open(File::OpenModeFlag::ReadOnly)) {
			std::unique_ptr<YamlObject> root = YamlParser::parse(file);
			if (root && root->contains("modules")) {
				for (const YamlObject &module :
				     (*root)["modules"].asList()) {
					if (module.contains("LensShadingCorrection"))
						data->postproc_->loadConfig(
							module["LensShadingCorrection"]);
				}
			}
		}
	}

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

	/*
	 * Build the subdevice format using the NATIVE sensor size, not the
	 * capture (adjusted) size. The native size is what the sensor hardware
	 * actually outputs (e.g. 3280×2464 for IMX219 at max resolution) and
	 * is what every subdevice in the chain from sensor → CSI → ISC scaler
	 * must be configured with. This ensures the kernel ISC driver sees the
	 * correct RAW mbus code at the scaler input and sets bits_pipeline with
	 * CFA_ENABLE | WB_ENABLE | GAM_ENABLE | ... instead of falling back to
	 * bits_pipeline=0 (raw passthrough) when there is a size mismatch.
	 *
	 * The video device (ISC DMA output) is separately configured with
	 * captureSize (the user-requested, video-device-clamped size). The ISC
	 * scaler crops nativeSensorSize → captureSize transparently.
	 *
	 * For sensors where native == capture (all sizes except max on some
	 * sensors like IMX219) nativeSensorSize == sensorSize and there is no
	 * difference in behaviour.
	 */
	V4L2SubdeviceFormat sensorFormat{};
	sensorFormat.code = pipeConfig->code;
	sensorFormat.size = pipeConfig->nativeSensorSize;

	LOG(MicrochipISC, Debug) << "Configuring subdev chain: native="
				 << sensorFormat.size.width << "x" << sensorFormat.size.height
				 << " capture=" << pipeConfig->captureSize.width
				 << "x" << pipeConfig->captureSize.height;

	/* Configure sensor + full subdevice chain at native sensor size */
	ret = data->setupFormats(&sensorFormat, V4L2Subdevice::ActiveFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to setup ISC formats: " << ret;
		return ret;
	}

	LOG(MicrochipISC, Debug) << "Sensor/subdev format negotiated: " << sensorFormat.toString();

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = data->iscVideo_->toV4L2PixelFormat(pipeConfig->captureFormat);
	captureFormat.size = pipeConfig->captureSize;

	/*
	 * Set colorspace for YUV output formats so the ISC CSC stage is
	 * configured correctly. Without this the kernel leaves colorspace
	 * unset and the CSC matrix is not applied, producing black output.
	 */
	if (pipeConfig->captureFormat == formats::YUYV ||
	    pipeConfig->captureFormat == formats::UYVY ||
	    pipeConfig->captureFormat == formats::YUV420)
		captureFormat.colorSpace = ColorSpace::Srgb;

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

		/*
		 * sensorControls is the ControlInfoMap (min/max/default ranges)
		 * — used for gain/exposure conversion factors.
		 *
		 * sensorLiveControls is a ControlList of the CURRENT values
		 * read from the sensor immediately after setFormat(). These
		 * reflect the actual operating parameters for the active mode:
		 * hblank, vblank, and exposure limits that differ per mode.
		 * The IPA uses these to compute lineDuration correctly for
		 * every supported sensor resolution without hardcoding defaults.
		 */
		ipaConfig.sensorControls = data->sensor_->controls();
		const std::vector<uint32_t> liveCtrlIds = {
			V4L2_CID_HBLANK,
			V4L2_CID_VBLANK,
			V4L2_CID_EXPOSURE,
			V4L2_CID_ANALOGUE_GAIN,
		};
		ipaConfig.sensorLiveControls =
			data->sensor_->getControls(liveCtrlIds);

		/*
		 * nativeSensorWidth is the hardware pixel count per line for
		 * the active sensor mode — the width that, combined with the
		 * live hblank, gives the true line length for lineDuration.
		 * This is the post-negotiation native width (e.g. 3280 for
		 * IMX219 at max resolution) stored in nativeSensorSize by
		 * tryPipeline(). The sensorInfo.outputSize.width reflects the
		 * capture-clamped width (3264) which would give a wrong result.
		 */
		ipaConfig.nativeSensorWidth =
			static_cast<uint32_t>(pipeConfig->nativeSensorSize.width);

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

	/*
	 * Configure the CPU post-processor for the negotiated output format.
	 * The stride carries any line padding the ISC applied, so the filters
	 * address rows correctly.
	 */
	if (data->postproc_)
		data->postproc_->configure(pipeConfig->captureFormat,
					   captureFormat.size,
					   captureFormat.planes[0].bpl);

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

	/* Reset start-failure flag so each new camera open gets a clean slate. */
	data->startFailed_ = false;

	LOG(MicrochipISC, Debug) << "Starting camera: " << camera->id();

	data->frameInfo_.clear();

	/*
	 * Disable kernel AWB before the IPA takes over WB gain control.
	 * When V4L2_CID_AUTO_WHITE_BALANCE=1 the kernel marks the WB gain
	 * registers as V4L2_CTRL_FLAG_INACTIVE and silently discards every
	 * IPA gain write (isc_s_ctrl() returns 0 without touching hardware).
	 * The IPA also sets AWB=0 inside setIspControls(), but that fires
	 * asynchronously on the first stats callback — this synchronous call
	 * at start() time ensures the gains are active from the very first
	 * frame.
	 */
	{
		ControlList awbCtrls(data->iscVideo_->controls());
		awbCtrls.set(V4L2_CID_AUTO_WHITE_BALANCE, static_cast<int32_t>(0));
		int ret = data->iscVideo_->setControls(&awbCtrls);
		if (ret < 0)
			LOG(MicrochipISC, Warning) << "Failed to disable kernel AWB: " << ret;
		else
			LOG(MicrochipISC, Debug) << "Kernel AWB disabled for IPA control";
	}

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

		/*
		 * Do NOT call statsDevice_->streamOn() here. The ISC stats
		 * engine fires HISDONE only when the ISC video pipeline is
		 * running — starting stats DMA before video DMA causes all
		 * stats buffers to remain queued with no completions until
		 * streamOff() returns them as cancelled at stop time.
		 *
		 * stats streamOn() is deferred to queueRequestDevice() where
		 * it is issued immediately after video streamOn(), ensuring
		 * the two DMAs start in the correct order (video first).
		 */
		LOG(MicrochipISC, Debug)
			<< "Stats buffers allocated — streamOn deferred to first video frame";
		data->statsStreaming_ = false; /* set true in queueRequestDevice */
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

	/* Start the CPU post-processor worker thread (no-op when disabled). */
	if (data->postproc_)
		data->postproc_->start();

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
	case V4L2_CID_BRIGHTNESS: {
		/*
		 * libcamera Brightness: [-1.0, +1.0], unity = 0.0
		 * ISC hardware:         [-1024, +1023], unity = 0
		 *
		 * Linear scale: Brightness=0.5 → +512 (half-scale boost).
		 * Formula: val = round(fval * (max - min) / 2 + (max + min) / 2)
		 */
		float fval = value.get<float>();
		int32_t val = static_cast<int32_t>(std::lroundf(fval));
		const ControlInfoMap *infoMap = controls->infoMap();
		if (infoMap) {
			auto it = infoMap->find(cid);
			if (it != infoMap->end()) {
				int32_t min = it->second.min().get<int32_t>();
				int32_t max = it->second.max().get<int32_t>();
				float mid = (static_cast<float>(max) +
					     static_cast<float>(min)) /
					    2.0f;
				float half = (static_cast<float>(max) -
					      static_cast<float>(min)) /
					     2.0f;
				val = static_cast<int32_t>(
					std::lroundf(fval * half + mid));
				val = std::clamp(val, min, max);
			}
		}
		controls->set(cid, val);
		break;
	}
	case V4L2_CID_CONTRAST: {
		/*
		 * libcamera Contrast: [0.0, 128.0], unity = 1.0
		 * ISC hardware:       [-2048, +2047], unity = 16
		 *
		 * The correct mapping is: hw = round(fval * kContrastUnityHw)
		 *   Contrast=1.0  → hw=16  (unity, no change)
		 *   Contrast=2.0  → hw=32  (2× contrast boost)
		 *   Contrast=0.5  → hw=8   (reduced contrast)
		 *   Contrast=128  → hw=2048 → clamped to 2047
		 *
		 * The generic brightness formula (fval * half + mid) is WRONG
		 * for contrast: it maps unity (1.0) to the hardware MAXIMUM
		 * (2047) instead of the hardware unity (16), causing permanent
		 * 128× contrast boost → overexposed output.
		 */
		static constexpr float kContrastUnityHw = 16.0f;
		float fval = value.get<float>();
		int32_t val = static_cast<int32_t>(
			std::lroundf(fval * kContrastUnityHw));
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

	iscSubdev_.emplace_back(entityName, std::move(subdev));
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

	/*
	 * If the video device reports no formats, maxVideoSize stays 0×0.
	 * Without a DMA size limit we cannot safely enumerate capture sizes —
	 * the tryPipeline() clamp would be skipped, configs_ would list
	 * oversized entries that setFormat() later rejects. Treat this as a
	 * hard initialisation failure rather than silently proceeding.
	 * Mirrors RkISP1 which returns -ENODEV if no ISP output formats are
	 * found during match().
	 */
	if (maxVideoSize.width == 0 || maxVideoSize.height == 0) {
		LOG(MicrochipISC, Error)
			<< "Video device reports no supported formats — cannot determine DMA limit";
		return -ENODEV;
	}

	/*
	 * Build the configuration list from the formats the sensor actually
	 * enumerates. Using sensor_->formats() (which calls VIDIOC_ENUM_MBUS_CODE
	 * on the sensor subdevice) is the correct approach — it only yields
	 * codes the sensor hardware physically supports, not every theoretically
	 * possible Bayer code.
	 *
	 * Previously we iterated kSensorMbusFormats (all 12 Bayer codes) and
	 * called sensor_->sizes() for each. IMX219 accepts any Bayer code via
	 * VIDIOC_S_FMT (the kernel driver reinterprets its RGGB output), so
	 * sensor_->sizes(SBGGR8_1X8) returned the same 4 sizes as SRGGB10_1X10.
	 * This produced configs_ entries with SBGGR8 that, after stable_sort,
	 * appeared before SRGGB10 entries at the same area. validate() then
	 * selected SBGGR8 for 1920×1080 — negotiate() agreed, subdevs were
	 * configured with the wrong Bayer order, and start_streaming() returned
	 * EPIPE because the ISC pipeline validation failed. Using sensor_->formats()
	 * restricts enumeration to the sensor's truly supported codes.
	 */
	/*
	 * sensor_->mbusCodes() returns only the codes the sensor hardware
	 * actually enumerates via VIDIOC_ENUM_MBUS_CODE. Filter to Bayer-only
	 * (kSensorMbusFormats) to skip any non-Bayer sensor outputs.
	 */
	for (unsigned int mbusCode : sensor_->mbusCodes()) {
		/* Skip non-Bayer sensor outputs (RGB, YUV, etc.) */
		bool isBayer = false;
		for (unsigned int bayerCode : kSensorMbusFormats) {
			if (mbusCode == bayerCode) {
				isBayer = true;
				break;
			}
		}
		if (!isBayer)
			continue;

		for (const Size &sensorSize : sensor_->sizes(mbusCode)) {
			tryPipeline(mbusCode, sensorSize, maxVideoSize);
		}
	}

	if (configs_.empty()) {
		LOG(MicrochipISC, Error) << "No valid configuration found";
		return -EINVAL;
	}

	/*
	 * Sort configs_ smallest→largest by capture area. This ordering is
	 * assumed by validate() (walk until captureSize >= requested, fall back
	 * to configs.back() as the largest) and by findOptimalSize() (≤ walk
	 * picks best-fit). The sensor driver may return sizes in any order so
	 * we enforce the invariant here rather than relying on driver behaviour.
	 * Mirrors the Simple pipeline which also sorts its config list before
	 * use (simple.cpp: configs sorted by resolution in generateConfiguration).
	 */
	/*
	 * stable_sort: preserves insertion order for equal-area entries
	 * (same sensor mode, different mbus code) — deterministic across runs.
	 * std::sort gives UB for equal elements with a non-total ordering;
	 * area comparison is a valid strict weak order but does not distinguish
	 * entries with the same area, so stable_sort is the correct choice here.
	 * Mirrors the algorithm priority sort (line ~1645) which also uses stable_sort.
	 */
	std::stable_sort(configs_.begin(), configs_.end(),
			 [](const Configuration &a, const Configuration &b) {
				 return a.captureSize.width * a.captureSize.height <
					b.captureSize.width * b.captureSize.height;
			 });

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
	/*
	 * Static ControlInfoMap advertised to applications.
	 *
	 * Ranges match actual ISC hardware register limits:
	 *   Brightness: V4L2_CID_BRIGHTNESS  [-1024, +1023] (asymmetric), unity=0
	 *   Contrast:   V4L2_CID_CONTRAST    [-2048, +2047] (asymmetric), unity=16
	 *   libcamera normalised range for Brightness: [-1.0, +1.0] → scaled to hw
	 *   libcamera normalised range for Contrast:   [0.0, ~128.0] → hw unity=16
	 *
	 * Sensor-mode-dependent controls (ExposureTime, AnalogueGain,
	 * FrameDurationLimits) are given placeholder ranges here that are
	 * overwritten with mode-correct values in configure(). Applications
	 * must re-read controls after configure() if they rely on accurate ranges.
	 *
	 * Gamma is applied via a hardware LUT (V4L2_CID_ISC_GAMMA_*_LUT), not
	 * via a scalar V4L2_CID_GAMMA control. Do NOT advertise Gamma here —
	 * exposing (0,0,0) would tell applications gamma is unsupported.
	 */
	controlInfo_ = ControlInfoMap({
					      /* ISC CBC/CBHS post-processing controls */
					      { &controls::Brightness, ControlInfo(-1.0f, 1.0f, 0.0f) },
					      { &controls::Contrast, ControlInfo(0.0f, 128.0f, 1.0f) },
					      { &controls::AwbEnable, ControlInfo(false, true, true) },
					      /* Sensor controls — ranges overwritten in configure() */
					      { &controls::AnalogueGain, ControlInfo(1.0f, 16.0f, 1.0f) },
					      { &controls::ExposureTime, ControlInfo(100, 33333, 10000) },
					      /* ISC WB register direct controls (bypass IPA) */
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
		if (ret < 0) {
			LOG(MicrochipISC, Error)
				<< "Failed to set format on subdevice " << name
				<< ": " << ret;
			return ret;
		}
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

void MicrochipISCCameraData::tryPipeline(unsigned int code, const Size &size,
					 const Size &maxVideoSize)
{
	/*
	 * Derive the capture size: clamp the native sensor size to the ISC
	 * video device maximum. This is what the ISC DMA output node will
	 * capture and what the application sees. For sensors where the native
	 * size already fits within the video device maximum (most modes),
	 * captureSize == size. For oversized modes (e.g. IMX219 3280 > 3264),
	 * captureSize is reduced to the video device limit and the ISC scaler
	 * crops on hardware.
	 */
	Size captureSize = size;
	if (maxVideoSize.width > 0 && maxVideoSize.height > 0) {
		if (captureSize.width > maxVideoSize.width) {
			LOG(MicrochipISC, Debug) << "Clamping capture width: "
						 << captureSize.width << " -> " << maxVideoSize.width;
			captureSize.width = maxVideoSize.width;
		}
		if (captureSize.height > maxVideoSize.height) {
			LOG(MicrochipISC, Debug) << "Clamping capture height: "
						 << captureSize.height << " -> " << maxVideoSize.height;
			captureSize.height = maxVideoSize.height;
		}
	}

	LOG(MicrochipISC, Debug) << "Trying pipeline: code=" << code
				 << " sensor=" << size.toString()
				 << " capture=" << captureSize.toString();

	/*
	 * Negotiate the subdevice chain at the NATIVE sensor size. This
	 * TryFormat call does not change any hardware state — it discovers
	 * what the sensor and intermediate subdevices actually negotiate to
	 * when this size is requested. The result (format.size after the call)
	 * is the true native sensor output, which may differ from the
	 * requested size if the sensor snaps to a wider readout.
	 */
	V4L2SubdeviceFormat format{};
	format.code = code;
	format.size = size;

	int ret = setupFormats(&format, V4L2Subdevice::TryFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Debug) << "Format not supported for this pipeline";
		return;
	}

	/*
	 * format.size is now the hardware-negotiated native sensor output size.
	 * Store it in nativeSensorSize — configure() will use this to set the
	 * subdevice chain so the kernel ISC driver sees the correct RAW mbus
	 * code and frame size at the scaler input, ensuring bits_pipeline is
	 * set correctly for WB / CCM / gamma processing.
	 */
	Size nativeSize = format.size;

	V4L2VideoDevice::Formats videoFormats = iscVideo_->formats(format.code);

	for (const auto &videoFormat : videoFormats) {
		PixelFormat pixelFormat = videoFormat.first.toPixelFormat(false);
		if (!pixelFormat)
			continue;

		/*
		 * Deduplicate: skip if an identical (captureFormat, captureSize)
		 * entry already exists. When multiple Bayer mbus codes negotiate
		 * to the same native size (e.g. SRGGB10 and SGRBG10 both snap to
		 * 1920×1080), the ISC video device reports the same output formats
		 * for both, producing duplicate configs_ entries. Duplicates waste
		 * memory, inflate the validate() walk, and cause sort instability.
		 */
		bool duplicate = false;
		for (const auto &existing : configs_) {
			if (existing.captureFormat == pixelFormat &&
			    existing.captureSize == captureSize) {
				duplicate = true;
				break;
			}
		}
		if (duplicate)
			continue;

		Configuration config;
		config.code = code;
		config.sensorSize = captureSize; /* user-visible / matching size */
		config.nativeSensorSize = nativeSize; /* actual subdev chain size     */
		config.captureFormat = pixelFormat;
		config.captureSize = captureSize;
		config.outputFormats = { pixelFormat };
		config.outputSizes = SizeRange{ captureSize, captureSize };

		configs_.push_back(config);
		LOG(MicrochipISC, Debug) << "Added configuration: " << pixelFormat.toString()
					 << " capture=" << captureSize.toString()
					 << " native=" << nativeSize.toString();
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
			/*
			 * The requested pixel format has no registered configs —
			 * either it's an unsupported format or formats_ was
			 * populated with an empty vector for it (the "all formats
			 * advertised" pass in init()). Fall back to the first
			 * available config and fix up pixelFormat + size so the
			 * caller's StreamConfiguration is self-consistent.
			 * Without fixing pixelFormat here, configure() would use
			 * pipeConfig_->captureFormat while the caller's cfg still
			 * holds the original unsupported format, causing setFormat
			 * to fail with EINVAL.
			 */
			if (!data_->configs_.empty()) {
				pipeConfig_ = &data_->configs_.front();
				cfg.pixelFormat = pipeConfig_->captureFormat;
				cfg.size = pipeConfig_->captureSize;
				status = Adjusted;
			} else {
				LOG(MicrochipISC, Error) << "No valid configurations available";
				return Invalid;
			}
		} else {
			/*
			 * Select the best-fit sensor mode for the requested
			 * size, following the same pattern as the Simple and
			 * RkISP1 pipeline handlers.
			 *
			 * Pass 1 — sensor mode selection:
			 *   Walk configs smallest→largest. Pick the first
			 *   mode whose captureSize covers the request on both
			 *   axes (no upscaling needed). If the request exceeds
			 *   all available modes (e.g. app asks for 3280×2464
			 *   but ISC DMA limit is 3264×2464), fall back to the
			 *   largest mode — matching Simple pipeline phase-1
			 *   (simple.cpp:1189) and RkISP1's maxPipeConfig path.
			 *   Never fall back to the smallest (configs.front()):
			 *   that silently configured 640×480 for a 3280×2464
			 *   request and the stream appeared to succeed while
			 *   producing wrong-size frames.
			 *
			 * Pass 2 — stream size clamp:
			 *   Once the mode is chosen, clamp the requested size
			 *   to the mode's outputSizes range via boundedTo() /
			 *   expandedTo() — the same axis-independent clamp
			 *   used by IPU3 (std::clamp), Mali-C55 (boundedTo),
			 *   and RkISP1 (boundTo/expandTo). This ensures
			 *   cfg.size is always within the advertised range and
			 *   status is set to Adjusted when clamping occurs.
			 */
			pipeConfig_ = configs.back();
			for (const MicrochipISCCameraData::Configuration *config : configs) {
				if (config->captureSize.width >= cfg.size.width &&
				    config->captureSize.height >= cfg.size.height) {
					pipeConfig_ = config;
					break;
				}
			}
		}

		/*
		 * Clamp requested size to the selected mode's output range.
		 * outputSizes is a SizeRange{captureSize, captureSize} for
		 * fixed-size sensor modes (set at enumerate time). boundedTo()
		 * clamps each axis independently (matches RkISP1/Mali-C55);
		 * expandedTo() floors to the minimum (protects against
		 * requesting 0×0 or sub-minimum sizes).
		 */
		Size clampedSize = cfg.size
					   .boundedTo(pipeConfig_->outputSizes.max)
					   .expandedTo(pipeConfig_->outputSizes.min);
		if (clampedSize != cfg.size) {
			LOG(MicrochipISC, Debug)
				<< "Adjusting stream size " << cfg.size.toString()
				<< " -> " << clampedSize.toString()
				<< " (mode max=" << pipeConfig_->outputSizes.max.toString()
				<< ")";
			cfg.size = clampedSize;
			status = Adjusted;
		} else {
			cfg.size = pipeConfig_->captureSize;
		}

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

	/*
	 * Insert frameInfo before processControls so that any V4L2 event
	 * arriving synchronously during the ioctl can find the entry.
	 * Log a warning on duplicate sequence — should never happen in normal
	 * operation but helps diagnose sequence-counter wraps or pipeline bugs.
	 */
	if (data->frameInfo_.count(request->sequence()))
		LOG(MicrochipISC, Warning)
			<< "Duplicate request sequence " << request->sequence()
			<< " — overwriting stale frame info";
	data->frameInfo_[request->sequence()] = frameInfo;

	int ret = processControls(data, request);
	if (ret < 0) {
		data->frameInfo_.erase(request->sequence());
		return ret;
	}

	/*
	 * Notify the IPA only after processControls() succeeds. Moving the
	 * call here prevents IPA state from advancing (frame counter, gain
	 * history) when the request is about to be abandoned, which would
	 * desynchronise IPA expectations from pipeline reality for all
	 * subsequent frames. Mirrors the common pipeline pattern where
	 * queueRequest is issued only after the V4L2 controls have been
	 * applied successfully.
	 */
	if (data->ipa_)
		data->ipa_->queueRequest(request->sequence(), request->controls());

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
	/*
	 * Deferred streamOn: issue VIDIOC_STREAMON only once, on the first
	 * queueBuffer call. Set videoStreaming_=true before the call so that
	 * concurrent callbacks from other in-flight requests do not also
	 * attempt streamOn (which would return -EPIPE on duplicate calls).
	 * On failure, clear videoStreaming_ and return the error.
	 */
	/*
	 * If a previous streamOn() call failed (EREMOTEIO from a hardware
	 * fault or sensor reset requirement), do not retry on subsequent
	 * queueBuffer() calls. Each retry queues another V4L2 buffer and
	 * inserts another frameInfo_ entry — all of them will be orphaned —
	 * causing multiple ASSERT(ret==1) failures in stopDevice()'s
	 * completeBuffer() loop. Return -EIO immediately; the capture will
	 * time out and the user will need to power-cycle the sensor.
	 */
	if (data->startFailed_)
		return -EIO;

	if (!data->videoStreaming_) {
		data->videoStreaming_ = true; /* set first to prevent re-entry */
		ret = data->iscVideo_->streamOn();
		if (ret < 0) {
			data->videoStreaming_ = false;
			data->startFailed_ = true;
			LOG(MicrochipISC, Error) << "Failed to start video streaming: " << ret
						 << " — sensor may need power-cycle";
			return ret;
		}
		LOG(MicrochipISC, Debug) << "Video streamOn issued on first buffer";

		/*
		 * Start stats DMA immediately after video DMA. The ISC stats
		 * engine fires HISDONE only when the video pipeline is active;
		 * starting stats before video results in zero completions until
		 * streamOff. Starting here guarantees the first HISDONE (after
		 * the first Bayer cycle = 4 frames) delivers live histogram data.
		 */
		if (data->statsEnabled_ && !data->statsStreaming_) {
			int statsRet = data->statsDevice_->streamOn();
			if (statsRet < 0) {
				LOG(MicrochipISC, Error)
					<< "Failed to start stats streaming: " << statsRet;
				/* Non-fatal: video works without IPA stats */
				data->statsEnabled_ = false;
			} else {
				data->statsStreaming_ = true;
				LOG(MicrochipISC, Debug) << "Stats streamOn issued after video";
			}
		}
	}

	return 0;
}

void PipelineHandlerMicrochipISC::bufferReady(FrameBuffer *buffer)
{
	MicrochipISCCameraData *data = activeData_;
	if (!data) {
		LOG(MicrochipISC, Error) << "No active camera data";
		return;
	}

	/*
	 * When the lens-shading stage is active, hand the finished buffer to
	 * its worker thread and complete the request only once filtering is
	 * done (postProcDone()). A cancelled buffer carries no valid pixels,
	 * so it skips the filter and completes directly.
	 */
	if (data->postproc_ && data->postproc_->isEnabled() &&
	    buffer->metadata().status != FrameMetadata::FrameCancelled) {
		data->postproc_->queue(buffer);
		return;
	}

	completeFrameBuffer(data, buffer);
}

void PipelineHandlerMicrochipISC::postProcDone(FrameBuffer *buffer)
{
	if (!activeData_) {
		LOG(MicrochipISC, Error)
			<< "Post-processing completed with no active camera data";
		return;
	}

	completeFrameBuffer(activeData_, buffer);
}

void PipelineHandlerMicrochipISC::completeFrameBuffer(MicrochipISCCameraData *data,
						      FrameBuffer *buffer)
{
	Request *request = buffer->request();
	if (!request) {
		LOG(MicrochipISC, Warning) << "Buffer without associated request";
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

	/* Always set SensorTimestamp — zero is valid (hardware may not provide one). */
	request->_d()->metadata().set(controls::SensorTimestamp,
				      static_cast<int64_t>(buffer->metadata().timestamp));

	/*
	 * Merge the most recent IPA metadata (AwbLocked, ColourTemperature, etc.)
	 * into this request.  The IPA responds asynchronously and statsProcessed()
	 * caches the latest result; applying it here ensures every completed frame
	 * carries up-to-date convergence state for the application.
	 */
	if (!data->latestIpaMetadata_.empty())
		request->_d()->metadata().merge(data->latestIpaMetadata_,
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

	/*
	 * Stop the DMA engines before stopping the IPA. This ordering is
	 * critical: if ipa_->stop() is called first, the IPA may still
	 * receive async statsProcessed() callbacks (queued in the IPC pipe)
	 * after freeStatsBuffers() has unmapped the IPA buffer memory —
	 * resulting in a use-after-free inside the IPA worker thread.
	 *
	 * Correct shutdown sequence:
	 *   1. streamOff video DMA  — no new bufferReady() callbacks
	 *   2. streamOff stats DMA  — no new statsBufferReady() callbacks
	 *   3. freeStatsBuffers()   — safe: no DMA in flight
	 *   4. ipa_->stop()         — safe: no more stats callbacks expected
	 *   5. cancel orphaned frames — complete any requests that stalled
	 *   6. frameInfo_.clear()
	 */
	if (data->iscVideo_ && data->videoStreaming_) {
		data->iscVideo_->streamOff();
		data->videoStreaming_ = false;
	}

	/*
	 * Drain the post-processor worker now. Video DMA is stopped, so no new
	 * bufferReady() can queue more work; stop() joins the thread and then
	 * completes every in-flight buffer — both those already filtered and any
	 * whose filter never ran — through frameProcessed → postProcDone →
	 * completeFrameBuffer, before the frameInfo_ cancel loop below runs. It
	 * must precede releaseBuffers() so the buffers the worker has mapped are
	 * still valid.
	 */
	if (data->postproc_)
		data->postproc_->stop();

	if (data->statsEnabled_ && data->statsStreaming_) {
		data->statsDevice_->streamOff();
		data->statsStreaming_ = false;
	}

	data->freeStatsBuffers();

	if (data->ipa_)
		data->ipa_->stop();

	if (data->iscVideo_)
		data->iscVideo_->releaseBuffers();

	/*
	 * Cancel any requests remaining in frameInfo_ whose image buffer
	 * never completed (bufferDone=false). This covers the case where
	 * the DMA stalled on the last frame before stopDevice — without this
	 * the framework asserts on un-completed requests at camera close time.
	 *
	 * Guard: only call completeBuffer() when bufferDone=false. When
	 * bufferDone=true, bufferReady() already called completeBuffer() and
	 * tryCompleteRequest() erased the entry — if an entry is still in
	 * the map with bufferDone=true it means hasPendingBuffers() returned
	 * true at that point. Call completeRequest() to drain it.
	 * Calling completeBuffer() twice corrupts the framework pending count
	 * and causes the crash at _ZN9libcamera7Request7Private14completeBufferE.
	 * Mirrors RkISP1 stopDevice which checks frameInfo.bufferCompleted
	 * before calling completeBuffer (rkisp1.cpp:stopDevice).
	 */
	for (auto &[seq, info] : data->frameInfo_) {
		if (!info.request)
			continue;
		if (!info.bufferDone) {
			for (auto &[stream, buffer] : info.request->buffers()) {
				buffer->_d()->cancel();
				/*
				 * Guard against double-complete: when streamOn()
				 * fails with EREMOTEIO the ISC kernel driver may
				 * dequeue buffers asynchronously (via the DMA
				 * completion IRQ fired before the ioctl returns)
				 * causing bufferReady() to call completeBuffer()
				 * before stopDevice() runs. A second call here
				 * would fire ASSERT(ret==1) in request.cpp:111.
				 *
				 * Check hasPendingBuffers() which reflects the
				 * live pending_ set: if the buffer was already
				 * completed by bufferReady(), pending_ is empty
				 * and we must skip the duplicate call.
				 */
				if (info.request->hasPendingBuffers())
					completeBuffer(info.request, buffer);
			}
		}
		/*
		 * Only call completeRequest() if the request is still pending.
		 * bufferReady() → tryCompleteRequest() may have already called
		 * completeRequest() for this entry if all buffers completed
		 * before stopDevice() ran (race with kernel IRQ delivery).
		 */
		if (info.request->status() == Request::RequestPending)
			completeRequest(info.request);
	}
	data->frameInfo_.clear();

	/*
	 * Subdevices (sensor, ISC blocks) are opened for the lifetime of the
	 * pipeline handler, not per-streaming session. Do not close them here;
	 * they are closed when the pipeline handler is destroyed.
	 */
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMicrochipISC, "microchip-isc")

} /* namespace libcamera */
