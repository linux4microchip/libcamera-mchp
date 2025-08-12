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
#include <chrono>
#include <thread>
#include <mutex>

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

struct isc_stat_buffer {
	/* Frame metadata */
	uint32_t frame_number;
	uint64_t timestamp;
	uint32_t meas_type;

	/* Raw histogram data for all channels */
	struct {
		uint32_t hist_bins[512];    /* Raw 512 bins from hardware */
		uint32_t hist_min;          /* Raw min value from hardware */
		uint32_t hist_max;          /* Raw max value from hardware */
		uint32_t total_pixels;      /* Raw total pixel count */
	} hist[4];                      /* GR, R, GB, B channels */

	/* Raw frame information */
	uint8_t valid_channels;         /* Bitmask of channels with data */
	uint8_t bayer_pattern;          /* Current Bayer pattern */
	uint16_t reserved[2];           /* Padding for alignment */
} __attribute__((packed));

namespace libcamera {

LOG_DEFINE_CATEGORY(MicrochipISC)

class PipelineHandlerMicrochipISC;

class MicrochipISCCameraData : public Camera::Private
{
public:
	MicrochipISCCameraData(PipelineHandler *pipe, MediaDevice *media)
		: Camera::Private(pipe), media_(media), bufferCount(0),
		statsEnabled_(false), statsStreaming_(false),
		lastHistogramTimestamp_(0), histogramDataReady_(false),
		currentFrameCount_(0), pendingRequest_(nullptr),
		isShuttingDown_(false), stopStatsProcessing_(false)

	{
		streams_.resize(2);
		channelsSeen_.fill(false);
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
	int initStatsDevice();
	/* Stats streaming control methods */
	int startStatsCapture();
	int stopStatsCapture();
	void statsBufferReady(FrameBuffer *buffer);

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
	std::vector<Stream> streams_;
	std::vector<Configuration> configs_;
	std::map<PixelFormat, std::vector<const Configuration *>> formats_;
	MediaDevice *media_;
	std::unique_ptr<ipa::microchip_isc::IPAProxyMicrochipISC> awbIPA_;
	unsigned int bufferCount;
	bool statsEnabled_;
	bool statsStreaming_;

	/* Event coordination variables */
	std::atomic<uint64_t> lastHistogramTimestamp_;
	std::atomic<bool> histogramDataReady_;
	std::unique_ptr<ControlList> cachedHistogramData_;
	std::mutex histogramMutex_;
	static constexpr uint64_t HISTOGRAM_FRESHNESS_NS = 500000000;

	int currentFrameCount_;
	Request *pendingRequest_;
	std::array<bool, 4> channelsSeen_;
	std::atomic<bool> isShuttingDown_;
	std::atomic<bool> stopStatsProcessing_;

	void resetFrameCycling() {
		currentFrameCount_ = 0;
		pendingRequest_ = nullptr;
		channelsSeen_.fill(false);
	}

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
	int applySensorControls(MicrochipISCCameraData *data, const ControlList &controls);
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
	void completeRequestWithBuffer(Request *request, FrameBuffer *buffer);
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

int MicrochipISCCameraData::startStatsCapture()
{
	if (!statsEnabled_ || !statsDevice_) {
		LOG(MicrochipISC, Debug) << "Stats capture not available";
		return -ENODEV;
	}

	if (statsStreaming_) {
		LOG(MicrochipISC, Debug) << "Background histogram already active";
		return 0;
	}

	LOG(MicrochipISC, Info) << "Starting BACKGROUND histogram collection for instant AWB";

	constexpr unsigned int kStatsBufferCount = 4;  /* More buffers for continuous operation */

	int ret = statsDevice_->allocateBuffers(kStatsBufferCount, &statsBuffers_);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to allocate stats buffers: " << ret;
		return ret;
	}

	/* Queue all buffers for continuous operation */
	for (auto &buffer : statsBuffers_) {
		ret = statsDevice_->queueBuffer(buffer.get());
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to queue stats buffer: " << ret;
			statsDevice_->releaseBuffers();
			statsBuffers_.clear();
			return ret;
		}
	}

	/* Start background streaming */
	ret = statsDevice_->streamOn();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to start background histogram streaming: " << ret;
		statsDevice_->releaseBuffers();
		statsBuffers_.clear();
		return ret;
	}

	statsStreaming_ = true;
	histogramDataReady_.store(false, std::memory_order_release);  /* Reset cache state */

	LOG(MicrochipISC, Info) << "Background histogram streaming active!";
	LOG(MicrochipISC, Info) << "Continuous 4-channel histogram collection for instant AWB";
	return 0;
}

int MicrochipISCCameraData::stopStatsCapture()
{
	if (!statsEnabled_ || !statsDevice_ || !statsStreaming_) {
		LOG(MicrochipISC, Debug) << "Stats capture not running or not available";
		return 0;
	}

	LOG(MicrochipISC, Debug) << "Stopping stats capture";

	isShuttingDown_.store(true, std::memory_order_release);

	try {
		statsDevice_->bufferReady.disconnect(this, &MicrochipISCCameraData::statsBufferReady);
		LOG(MicrochipISC, Debug) << "📊 Disconnected statsBufferReady signal";
	} catch (const std::exception& e) {
		LOG(MicrochipISC, Warning) << "Failed to disconnect signal: " << e.what();
	}

	/* Clear histogram cache */
	{
		std::lock_guard<std::mutex> lock(histogramMutex_);
		histogramDataReady_.store(false, std::memory_order_release);
		lastHistogramTimestamp_.store(0, std::memory_order_release);
		cachedHistogramData_.reset();
		LOG(MicrochipISC, Debug) << "Histogram cache cleared";
	}

	int ret = statsDevice_->streamOff();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to stop stats streaming: " << ret;
	}

	try {
		statsDevice_->releaseBuffers();
		statsBuffers_.clear();
	} catch (const std::exception& e) {
		LOG(MicrochipISC, Error) << "Exception during buffer release: " << e.what();
	}

	statsStreaming_ = false;
	LOG(MicrochipISC, Info) << "Hardware histogram collection stopped cleanly";
	return ret;
}

void MicrochipISCCameraData::statsBufferReady(FrameBuffer *buffer)
{
	static int statsCallCount = 0;
	statsCallCount++;

	/* Early exit if we already captured perfect data */
	if (stopStatsProcessing_.load(std::memory_order_acquire)) {
		LOG(MicrochipISC, Debug) << "📊 Perfect histogram already captured, ignoring buffer #" << statsCallCount;
		return;
	}

	if (isShuttingDown_.load(std::memory_order_acquire)) {
		LOG(MicrochipISC, Debug) << "📊 Ignoring stats buffer #" << statsCallCount << " during shutdown";
		return;
	}

	if (!buffer || buffer->planes().empty() || !awbIPA_ || !statsDevice_) {
		LOG(MicrochipISC, Warning) << "📊 Invalid buffer or missing components in statsBufferReady #"
			<< statsCallCount;
		goto requeue_buffer;
	}

	if (!statsStreaming_) {
		LOG(MicrochipISC, Debug) << "📊 Not streaming, ignoring buffer #" << statsCallCount;
		goto requeue_buffer;
	}

	LOG(MicrochipISC, Debug) << "📊 Processing stats buffer #" << statsCallCount;

	{
	size_t bufferSize = buffer->planes()[0].length;
	uint64_t bufferTimestamp = buffer->metadata().timestamp;

	void *mappedMemory = mmap(NULL, bufferSize, PROT_READ, MAP_SHARED,
			buffer->planes()[0].fd.get(), buffer->planes()[0].offset);
	if (mappedMemory == MAP_FAILED) {
		LOG(MicrochipISC, Error) << "❌ Failed to map histogram buffer: " << strerror(errno);
		goto requeue_buffer;
	}

	{
	const struct isc_stat_buffer *statsData = static_cast<const struct isc_stat_buffer*>(mappedMemory);

	if (statsData->valid_channels == 0x0F &&
			statsData->frame_number > 0 &&
			statsData->timestamp > 0) {

		LOG(MicrochipISC, Info) << "🎉 Complete hardware histogram data received! Channels: 0x"
			<< std::hex << (int)statsData->valid_channels;

		uint32_t totalPixels = 0;
		int validChannelCount = 0;

		for (int ch = 0; ch < 4; ch++) {
			if (statsData->valid_channels & (1 << ch)) {
				validChannelCount++;
				totalPixels += statsData->hist[ch].total_pixels;
			}
		}

		if (validChannelCount == 4 && totalPixels > 50000) {
			/* Keep your existing caching logic: */
			std::lock_guard<std::mutex> lock(histogramMutex_);

			if (!isShuttingDown_.load(std::memory_order_acquire)) {
				const uint8_t* raw = static_cast<const uint8_t*>(mappedMemory);
				cachedHistogramData_ = std::make_unique<ControlList>(controls::controls);
				cachedHistogramData_->set(ipa::microchip_isc::ISC_HISTOGRAM_DATA_ID,
						Span<const uint8_t>(raw, bufferSize));

				uint64_t timestamp = (statsData->timestamp > 0) ? statsData->timestamp : bufferTimestamp;
				lastHistogramTimestamp_.store(timestamp, std::memory_order_release);
				histogramDataReady_.store(true, std::memory_order_release);

				/* 🔧 NEW: Stop all future processing */
				stopStatsProcessing_.store(true, std::memory_order_release);

				LOG(MicrochipISC, Info) << "🎯 PERFECT HISTOGRAM CAPTURED - stopping stats processing";

				munmap(mappedMemory, bufferSize);
				return;  /* Don't re-queue */
			}
		}
	}
	}

	munmap(mappedMemory, bufferSize);
	}

requeue_buffer:
	if (!stopStatsProcessing_.load(std::memory_order_acquire) &&
		!isShuttingDown_.load(std::memory_order_acquire) &&
		statsDevice_ && statsStreaming_) {

		int ret = statsDevice_->queueBuffer(buffer);
		if (ret < 0) {
			if (ret == -ESHUTDOWN || ret == -108) {
				LOG(MicrochipISC, Debug) << "📊 Device stopping, buffer re-queue expected to fail";
			} else {
				LOG(MicrochipISC, Error) << "❌ Failed to re-queue stats buffer: " << ret;
				/* Stop trying on error */
				stopStatsProcessing_.store(true, std::memory_order_release);
			}
		} else {
			LOG(MicrochipISC, Debug) << "📊 Re-queued histogram buffer #" << statsCallCount;
		}
	} else {
		LOG(MicrochipISC, Debug) << "📊 Not re-queuing - perfect data captured or shutting down";
	}
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

	/* Initialize IPA first */
	if (data->awbIPA_) {
		LOG(MicrochipISC, Debug) << "AWB IPA module loaded successfully";
	} else {
		LOG(MicrochipISC, Error) << "Failed to load AWB IPA module";
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
					LOG(MicrochipISC, Info) << "✅ Opened stats device: " << entity->name();

					/* Set up stats format immediately after opening */
					V4L2DeviceFormat statsFormat;
					statsFormat.fourcc = V4L2PixelFormat(v4l2_fourcc('I', 'S', 'C', 'S'));
					statsFormat.size = Size(0, 0);  /* Meta format doesn't need size */
					statsFormat.planesCount = 1;

					ret = data->statsDevice_->setFormat(&statsFormat);
					if (ret < 0) {
						LOG(MicrochipISC, Warning) << "Failed to set stats format: " << ret;
						/* Continue anyway - format might be set later */
					} else {
						LOG(MicrochipISC, Info) << "📊 Stats device format configured successfully";
					}

					/* Connect the  bufferReady signal AFTER successful open and format setup */
					data->statsDevice_->bufferReady.connect(data.get(),
							&MicrochipISCCameraData::statsBufferReady);
					data->statsEnabled_ = true;
					foundStatsVideo = true;

					LOG(MicrochipISC, Info) << "📊 Connected statsBufferReady signal to stats device";
					LOG(MicrochipISC, Info) << "📊 Hardware histogram support enabled and ready!";
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

	if (data->iscSubdev_.size() < 2) {  /* Reduced requirement */
		LOG(MicrochipISC, Error) << "Insufficient ISC subdevices found: " << data->iscSubdev_.size();
		return false;
	}

	/* Log stats device status */
	if (foundStatsVideo) {
		LOG(MicrochipISC, Info) << "✅ Hardware histogram support: ENABLED";
		LOG(MicrochipISC, Info) << "📊 Stats device ready for 512-bin Bayer histograms";
	} else {
		LOG(MicrochipISC, Info) << "⚠️  Hardware histogram support: DISABLED (stats device not found)";
		LOG(MicrochipISC, Info) << "📊 Will use software histogram fallback";
	}

	/* Initialize the camera data */
	ret = data->init();
	if (ret) {
		LOG(MicrochipISC, Error) << "Failed to initialize camera data";
		return false;
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
		LOG(MicrochipISC, Info) << " Camera registration complete with software histogram fallback";
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

	LOG(MicrochipISC, Debug) << "🔧 Configuring camera: " << camera->id();

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

	LOG(MicrochipISC, Info) << "🔧 Using validated config: sensor="
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

	LOG(MicrochipISC, Info) << "🔧 Using validated config: capture="
		<< captureFormat.size.width << "x" << captureFormat.size.height;
	LOG(MicrochipISC, Debug) << "Capture format configured: " << captureFormat.toString();

	ret = data->iscVideo_->setFormat(&captureFormat);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to set video device format: " << ret;
		return ret;
	}

	/* Configure IPA if available */
	if (data->awbIPA_) {
		ipa::microchip_isc::MicrochipISCSensorInfo sensorInfo = {};
		sensorInfo.model = data->sensor_->model();
		sensorInfo.width = captureFormat.size.width;
		sensorInfo.height = captureFormat.size.height;
		sensorInfo.pixelFormat = sensorFormat.code;

		LOG(MicrochipISC, Debug) << "Configuring IPA for sensor: " << sensorInfo.model
			<< " (" << sensorInfo.width << "x" << sensorInfo.height << ")";

		std::map<unsigned int, IPAStream> streamConfig;
		std::map<unsigned int, ControlInfoMap> entityControls;

		ret = data->awbIPA_->configure(sensorInfo, streamConfig, entityControls);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to configure IPA: " << ret;
			return ret;
		}
		LOG(MicrochipISC, Debug) << "✅ IPA configured successfully";
	}

	/* Stats device info */
	if (data->statsEnabled_) {
		LOG(MicrochipISC, Info) << "📊 Hardware histogram available - will start FIRST during camera start";
	} else {
		LOG(MicrochipISC, Info) << "📊 Hardware histogram not available - using software processing only";
	}

	/* Set stream configurations with validated sizes */
	for (unsigned int i = 0; i < c->size(); ++i) {
		StreamConfiguration &cfg = c->at(i);
		cfg.setStream(&data->streams_[i]);
		cfg.stride = captureFormat.planes[0].bpl;
		cfg.size = captureFormat.size;
	}

	LOG(MicrochipISC, Info) << "🎯 Camera configuration complete: " << camera->id();
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

	LOG(MicrochipISC, Debug) << "Starting camera: " << camera->id();

	/* Reset shutdown flag */
	data->isShuttingDown_.store(false, std::memory_order_release);

	/* Configure hardware */
	ControlList awbCtrls(data->iscVideo_->controls());
	awbCtrls.set(V4L2_CID_AUTO_WHITE_BALANCE, 0);
	data->iscVideo_->setControls(&awbCtrls);

	if (data->awbIPA_) {
		int ipaRet = data->awbIPA_->start();
		if (ipaRet < 0) {
			LOG(MicrochipISC, Error) << "Failed to start IPA: " << ipaRet;
			return ipaRet;
		}
	}

	/* Start histogram collection with longer setup time */
	if (data->statsEnabled_) {
		int statsRet = data->startStatsCapture();
		if (statsRet < 0) {
			LOG(MicrochipISC, Warning) << "Failed to start histogram capture: " << statsRet;
		} else {
			LOG(MicrochipISC, Info) << "✅ Background histogram collection started first";
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			LOG(MicrochipISC, Info) << "📊 Extended histogram hardware initialization delay completed (500ms)";
		}
	}

	/* Import and start video */
	unsigned int count = 8;
	int ret = data->iscVideo_->importBuffers(count);
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to import buffers: " << ret;
		return ret;
	}
	data->bufferCount = count;

	ret = data->iscVideo_->streamOn();
	if (ret < 0) {
		LOG(MicrochipISC, Error) << "Failed to start video streaming: " << ret;
		return ret;
	}

	LOG(MicrochipISC, Info) << " Camera ready with event-driven histogram coordination";
	return 0;
}

int PipelineHandlerMicrochipISC::processControl(ControlList *controls, unsigned int id,
						const ControlValue &value)
{
	uint32_t cid;
	bool isSensorControl = false;

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
	else if (id == controls::AnalogueGain) {
		cid = 0x009e0903;
		isSensorControl = true;
	}
	else if (id == controls::DigitalGain) {
		cid = 0x009f0905;
		isSensorControl = true;
	}
	else if (id == controls::ExposureTime) {
		cid = 0x00980911;
		isSensorControl = true;
	}
	else
		return -EINVAL;

	/* Handle sensor controls differently */
	if (isSensorControl) {
		return 0; /* Will be handled by applySensorControls() */
	}

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
			/* NEW: Sensor hardware controls */
			{ &controls::AnalogueGain, ControlInfo(0.0f, 23.2f, 0.0f) },        /* 0-232 mapped to 0-23.2x */
			{ &controls::DigitalGain, ControlInfo(1.0f, 16.0f, 1.0f) },         /* 256-4095 mapped to 1-16x */
			{ &controls::ExposureTime, ControlInfo(4, 3522, 1600) },            /* Direct microsecond values */
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

void MicrochipISCCameraData::awbComplete([[maybe_unused]] unsigned int bufferId,
		const ControlList &metadata)
{
	LOG(MicrochipISC, Debug) << "AWB processing complete - applying values to hardware";

	if (!iscVideo_) {
		LOG(MicrochipISC, Warning) << "No video device for applying AWB results";
		return;
	}

	/* Handle ISC controls (white balance gains/offsets) */
	const std::map<uint32_t, std::pair<uint32_t, std::string>> iscControlMap = {
		{ipa::microchip_isc::GREEN_RED_GAIN_ID,    {0x009819c2, "GREEN_RED gain"}},
		{ipa::microchip_isc::RED_GAIN_ID,          {0x009819c0, "RED gain"}},
		{ipa::microchip_isc::GREEN_BLUE_GAIN_ID,   {0x009819c3, "GREEN_BLUE gain"}},
		{ipa::microchip_isc::BLUE_GAIN_ID,         {0x009819c1, "BLUE gain"}},
		{ipa::microchip_isc::GREEN_RED_OFFSET_ID,  {0x009819c6, "GREEN_RED offset"}},
		{ipa::microchip_isc::RED_OFFSET_ID,        {0x009819c4, "RED offset"}},
		{ipa::microchip_isc::GREEN_BLUE_OFFSET_ID, {0x009819c7, "GREEN_BLUE offset"}},
		{ipa::microchip_isc::BLUE_OFFSET_ID,       {0x009819c5, "BLUE offset"}}
	};

	ControlList iscControls(iscVideo_->controls());
	bool hasIscUpdates = false;

	/* Apply ISC controls */
	for (const auto &[ipaControlId, hwControl] : iscControlMap) {
		if (metadata.contains(ipaControlId)) {
			int32_t value = metadata.get(ipaControlId).get<int32_t>();
			iscControls.set(hwControl.first, value);
			hasIscUpdates = true;

			bool isGain = hwControl.second.find("gain") != std::string::npos;
			if (isGain) {
				LOG(MicrochipISC, Info) << "📊 ISC Hardware: " << hwControl.second << " " << value;
			} else {
				LOG(MicrochipISC, Debug) << "📊 ISC Hardware: " << hwControl.second << " " << value;
			}
		}
	}

	/* Apply ISC controls */
	if (hasIscUpdates) {
		int ret = iscVideo_->setControls(&iscControls);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "❌ Failed to write AWB values to ISC hardware: " << ret;
		} else {
			LOG(MicrochipISC, Info) << "✅ AWB values written to ISC hardware";
		}
	}

	/* Apply sensor controls */
	if (pipe_) {
		int ret = pipe_->applySensorControls(this, metadata);
		if (ret < 0) {
			LOG(MicrochipISC, Warning) << "Failed to apply sensor controls: " << ret;
		}
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
	LOG(MicrochipISC, Info) << "Stats device format: " << statsFormat.fourcc
		<< " size: " << statsFormat.size.width << "x" << statsFormat.size.height;

	LOG(MicrochipISC, Info) << "ISC hardware stats device ready - signal connected and waiting for histogram data";
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

int PipelineHandlerMicrochipISC::applySensorControls(MicrochipISCCameraData *data,
		const ControlList &controls)
{
	if (!data->sensor_) {
		LOG(MicrochipISC, Error) << "No sensor available for hardware controls";
		return -ENODEV;
	}

	ControlList sensorControls(data->sensor_->controls());
	bool hasUpdates = false;

	/* Map IPA sensor controls to sensor hardware */
	const std::map<uint32_t, std::pair<uint32_t, std::string>> sensorControlMap = {
		{ipa::microchip_isc::SENSOR_ANALOGUE_GAIN_ID, {0x009e0903, "analogue_gain"}},
		{ipa::microchip_isc::SENSOR_DIGITAL_GAIN_ID,  {0x009f0905,  "digital_gain"}},
		{ipa::microchip_isc::SENSOR_EXPOSURE_ID,      {0x00980911,      "exposure"}}
	};

	for (const auto &[ipaControlId, hwControl] : sensorControlMap) {
		if (controls.contains(ipaControlId)) {
			int32_t value = controls.get(ipaControlId).get<int32_t>();
			sensorControls.set(hwControl.first, value);
			hasUpdates = true;

			LOG(MicrochipISC, Info) << "📊 Sensor Hardware: " << hwControl.second << " = " << value;
		}
	}

	/* Apply to actual sensor hardware */
	if (hasUpdates) {
		int ret = data->sensor_->setControls(&sensorControls);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "❌ Failed to apply sensor controls: " << ret;
			return ret;
		} else {
			LOG(MicrochipISC, Info) << "✅ Sensor hardware controls applied successfully";
		}
	}

	return 0;
}

int PipelineHandlerMicrochipISC::queueRequestDevice(Camera *camera, Request *request)
{
	MicrochipISCCameraData *data = cameraData(camera);

	data->stopStatsProcessing_.store(false, std::memory_order_release);

	data->resetFrameCycling();

	int ret = processControls(data, request);
	if (ret < 0)
		return ret;

	for (Stream &stream : data->streams_) {
		FrameBuffer *buffer = request->findBuffer(&stream);
		if (buffer) {
			ret = data->iscVideo_->queueBuffer(buffer);
			if (ret < 0) {
				LOG(MicrochipISC, Error) << "Failed to queue buffer: " << ret;
				return ret;
			}
			LOG(MicrochipISC, Debug) << "📸 Queued initial buffer for histogram cycling";
		}
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

	if (data->isShuttingDown_.load(std::memory_order_acquire)) {
		LOG(MicrochipISC, Debug) << "Ignoring buffer during shutdown";
		return;
	}

	data->currentFrameCount_++;
	LOG(MicrochipISC, Info) << "📷 Frame " << data->currentFrameCount_ << " ready - monitoring for all 4 histogram channels";

	/* Store the first request for frame cycling */
	if (data->currentFrameCount_ == 1) {
		data->pendingRequest_ = request;
		LOG(MicrochipISC, Info) << "🔄 Starting extended histogram cycling (20 frames like fswebcam -S 20)";
	}

	const int MAX_FRAMES_FOR_HISTOGRAM = 20;

	bool hardwareDataReady = data->histogramDataReady_.load(std::memory_order_acquire);

	if (hardwareDataReady) {
		LOG(MicrochipISC, Info) << "⚡ Frame " << data->currentFrameCount_ << ": SUCCESS! Using hardware histogram";

		std::lock_guard<std::mutex> lock(data->histogramMutex_);
		if (data->cachedHistogramData_ && data->awbIPA_) {
			try {
				data->awbIPA_->processStats(*data->cachedHistogramData_);
				data->histogramDataReady_.store(false, std::memory_order_release);
				LOG(MicrochipISC, Info) << "✅ Hardware histogram processed successfully on frame " << data->currentFrameCount_;

				completeRequestWithBuffer(data->pendingRequest_ ? data->pendingRequest_ : request, buffer);
				LOG(MicrochipISC, Info) << "🎯 SUCCESS: Request completed with HARDWARE AWB after " << data->currentFrameCount_ << " frames";
				data->resetFrameCycling();
				return;

			} catch (const std::exception& e) {
				LOG(MicrochipISC, Error) << "Hardware histogram processing failed: " << e.what();
			}
		}
	}

	if (data->currentFrameCount_ < MAX_FRAMES_FOR_HISTOGRAM) {
		/* Continue cycling - still waiting for hardware histogram */
		LOG(MicrochipISC, Debug) << "🔄 Frame " << data->currentFrameCount_ << "/" << MAX_FRAMES_FOR_HISTOGRAM
			<< ": Continuing (waiting for all 4 histogram channels)";

		/* Re-queue the same buffer to get more frames */
		int ret = data->iscVideo_->queueBuffer(buffer);
		if (ret < 0) {
			LOG(MicrochipISC, Error) << "Failed to re-queue buffer: " << ret;
			/* Force completion on error - fall through to software processing */
		} else {
			return;
		}
	}

	/* TIMEOUT: Use software fallback after MAX_FRAMES_FOR_HISTOGRAM */
	LOG(MicrochipISC, Warning) << "⏰ Timeout after " << data->currentFrameCount_ << " frames - completing with software AWB";

	void *mappedMemory = mmap(NULL, buffer->planes()[0].length, PROT_READ, MAP_SHARED,
			buffer->planes()[0].fd.get(), buffer->planes()[0].offset);

	if (mappedMemory != MAP_FAILED) {
		/* Send pixel data to IPA for software histogram processing */
		ControlList controls(controls::controls);
		controls.set(ipa::microchip_isc::ISC_PIXEL_VALUES_ID,
				Span<const uint8_t>(static_cast<const uint8_t*>(mappedMemory),
					buffer->planes()[0].length));

		if (data->awbIPA_) {
			LOG(MicrochipISC, Debug) << "Sending pixel data to IPA for software AWB processing";
			data->awbIPA_->processStats(controls);
		}

		munmap(mappedMemory, buffer->planes()[0].length);

		completeRequestWithBuffer(data->pendingRequest_ ? data->pendingRequest_ : request, buffer);
		LOG(MicrochipISC, Info) << "🏁 Request completed with software AWB after " << data->currentFrameCount_ << " frames";
		data->resetFrameCycling();

	} else {
		LOG(MicrochipISC, Error) << "Failed to map buffer for software processing: " << strerror(errno);

		completeRequestWithBuffer(data->pendingRequest_ ? data->pendingRequest_ : request, buffer);
		data->resetFrameCycling();
	}
}

void PipelineHandlerMicrochipISC::completeRequestWithBuffer(Request *request, FrameBuffer *buffer)
{
	if (!request) {
		LOG(MicrochipISC, Warning) << "Attempting to complete null request";
		return;
	}

	if (buffer->metadata().timestamp) {
		request->metadata().set(controls::SensorTimestamp, buffer->metadata().timestamp);
	}

	completeBuffer(request, buffer);
	if (!request->hasPendingBuffers()) {
		completeRequest(request);
	}
}

void PipelineHandlerMicrochipISC::stopDevice(Camera *camera)
{
	MicrochipISCCameraData *data = cameraData(camera);
	LOG(MicrochipISC, Debug) << "Stopping device for camera: " << camera->id();

	data->isShuttingDown_.store(true, std::memory_order_release);

	data->resetFrameCycling();

	if (data->statsEnabled_ && data->statsStreaming_) {
		LOG(MicrochipISC, Info) << "Stopping background histogram collection";
		data->stopStatsCapture();
	}

	if (data->iscVideo_) {
		try {
			data->iscVideo_->streamOff();
			data->iscVideo_->releaseBuffers();
			data->bufferCount = 0;
			LOG(MicrochipISC, Debug) << "Main video stopped cleanly";
		} catch (const std::exception& e) {
			LOG(MicrochipISC, Error) << "Video stop failed: " << e.what();
		}
	}

	if (data->awbIPA_) {
		try {
			data->awbIPA_->stop();
			LOG(MicrochipISC, Debug) << "IPA stopped cleanly";
		} catch (const std::exception& e) {
			LOG(MicrochipISC, Error) << "IPA stop failed: " << e.what();
		}
	}

	for (const auto &[name, subdev] : data->iscSubdev_) {
		if (subdev) {
			try {
				subdev->close();
			} catch (...) {
				/* Ignore exceptions during cleanup */
			}
		}
	}

	LOG(MicrochipISC, Info) << "Device stopped cleanly with proper event coordination cleanup";
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMicrochipISC, "microchip-isc")

} /* namespace libcamera */


