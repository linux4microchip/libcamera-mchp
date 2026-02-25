/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * IPA module for Microchip ISC
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <string.h>
#include <vector>

#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"
#include "isc_controls.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_IPA)

namespace ipa::microchip_isc {

static constexpr uint32_t kMaxFrameContexts = 16;

/**
 * \class IPAMicrochipISC
 * \brief IPA module for the Microchip ISC image sensor controller
 *
 * Implements the IPA interface for the Microchip ISC / XISC pipeline handler.
 * Hosts the AWB, AGC, CCM, CBC, and Gamma algorithm chain. Each algorithm
 * operates on the 4-channel 512-bin ISC hardware histogram delivered once
 * per 4-frame Bayer cycle.
 *
 * The algorithm chain shares state through IPAContext:
 * - AWB computes per-channel gains and colour temperature (CT)
 * - CCM interpolates a 3×3 colour correction matrix from the AWB CT
 * - AGC adjusts sensor exposure and analogue gain to hit target brightness
 * - CBC corrects residual brightness/contrast error after AGC has converged
 * - Gamma applies adaptive tone mapping using gain-undone scene luminance
 *
 * Platform capabilities (DPC, bipartite gamma, CBHS saturation) are read
 * from the YAML tuning file hardware section at init() time.
 */

/*
 * Sentinel value passed by the pipeline handler in the bayerOrder argument
 * of configure() to indicate a non-RAW sensor output (YUYV, RGB). In that
 * case the ISC histogram engine produces no valid statistics.
 * Must match the pipeline handler convention in microchip-isc.cpp.
 */
static constexpr uint8_t kBayerOrderInvalid = 0xff;

class IPAMicrochipISC : public IPAMicrochipISCInterface, public Module
{
public:
	IPAMicrochipISC();

	int init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;
	int configure(const IPAConfigInfo &ipaConfig, uint8_t bayerOrder,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void queueRequest(const uint32_t request, const ControlList &controls) override;
	void processStats(unsigned int request, unsigned int bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateSessionConfiguration(const IPACameraSensorInfo &info,
					const ControlInfoMap &sensorControls,
					BayerFormat::Order bayerOrder);
	void setSensorControls();
	void setIspControls();

	std::map<unsigned int, MappedFrameBuffer> buffers_;
	ControlInfoMap sensorControls_;

	std::unique_ptr<CameraSensorHelper> camHelper_;

	struct IPAContext context_;

	/*
	 * Algorithms sorted by priority() for deterministic execution order
	 * in processStats(). Rebuilt on each configure() call.
	 * Dependency chain: BLC(0) → AWB/AGC(10) → Gamma/CCM(20) → CBC(30).
	 */
	std::vector<Algorithm *> sortedAlgorithms_;
};

IPAMicrochipISC::IPAMicrochipISC()
	: context_(kMaxFrameContexts)
{
}

std::string IPAMicrochipISC::logPrefix() const
{
	return "microchip-isc";
}

int IPAMicrochipISC::init(const IPASettings &settings,
			  const IPAConfigInfo &ipaConfig,
			  ControlInfoMap *ipaControls)
{
	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(ISC_IPA, Warning)
			<< "No sensor helper for " << settings.sensorModel
			<< ", using direct gain values";
	}

	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(ISC_IPA, Warning)
			<< "Failed to open tuning file "
			<< settings.configurationFile
			<< ", using algorithm defaults";
		return 0;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data) {
		LOG(ISC_IPA, Warning) << "Failed to parse tuning file";
		return 0;
	}

	uint32_t version = (*data)["version"].get<uint32_t>(0);
	if (version != 1) {
		LOG(ISC_IPA, Warning)
			<< "Unsupported tuning file version " << version
			<< " (expected 1), using algorithm defaults";
		return 0;
	}

	if (!data->contains("algorithms")) {
		LOG(ISC_IPA, Warning) << "Tuning file has no algorithms section";
		return 0;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret) {
		LOG(ISC_IPA, Error) << "Failed to create algorithms";
		return ret;
	}

	/* Populate IPA controls from context */
	*ipaControls = ControlInfoMap(context_.ctrlMap, controls::controls);

	LOG(ISC_IPA, Info) << "ISC IPA initialized with "
			   << algorithms().size() << " algorithms";

	return 0;
}

int IPAMicrochipISC::start()
{
	return 0;
}

void IPAMicrochipISC::stop()
{
	context_.frameContexts.clear();
}

void IPAMicrochipISC::updateSessionConfiguration(
	const IPACameraSensorInfo &info,
	const ControlInfoMap &sensorControls,
	BayerFormat::Order bayerOrder)
{
	context_.configuration.sensor.bayerOrder = bayerOrder;

	/*
	 * Compute lineDuration from the actual HBLANK default rather than
	 * minLineLength.  minLineLength is the hardware minimum (e.g. ~3448
	 * pixels for IMX219) but the sensor typically operates at a longer
	 * default line (e.g. ~4196 pixels); using the minimum would give a
	 * shorter lineDuration and therefore an incorrect flicker-avoidance
	 * half-period in AGC (wrong number of lines per 1/100 s or 1/120 s).
	 * Fall back to minLineLength when HBLANK is not available.
	 */
	uint32_t activeLineLength = info.minLineLength;
	auto hblankIt = sensorControls.find(V4L2_CID_HBLANK);
	if (hblankIt != sensorControls.end()) {
		uint32_t hblank = static_cast<uint32_t>(
			hblankIt->second.def().get<int32_t>());
		activeLineLength = info.outputSize.width + hblank;
	}

	context_.configuration.sensor.lineDuration =
		utils::Duration(static_cast<double>(activeLineLength) * 1e9 / info.pixelRate);

	const ControlInfo &v4l2Exposure =
		sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>();
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>();
	int32_t defExposure = v4l2Exposure.def().get<int32_t>();

	const ControlInfo &v4l2Gain =
		sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	int32_t minGain = v4l2Gain.min().get<int32_t>();
	int32_t maxGain = v4l2Gain.max().get<int32_t>();

	context_.configuration.agc.minShutterSpeed =
		minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.maxShutterSpeed =
		maxExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.defaultExposure = defExposure;

	if (camHelper_) {
		context_.configuration.agc.minAnalogueGain =
			camHelper_->gain(minGain);
		context_.configuration.agc.maxAnalogueGain =
			camHelper_->gain(maxGain);
	} else {
		context_.configuration.agc.minAnalogueGain =
			static_cast<double>(minGain);
		context_.configuration.agc.maxAnalogueGain =
			static_cast<double>(maxGain);
	}
}

int IPAMicrochipISC::configure(const IPAConfigInfo &ipaConfig,
			       uint8_t bayerOrder,
			       ControlInfoMap *ipaControls)
{
	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;
	sensorControls_ = ipaConfig.sensorControls;

	updateSessionConfiguration(info, ipaConfig.sensorControls,
				   static_cast<BayerFormat::Order>(bayerOrder));

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	/* Initialize AGC and AWB to auto mode */
	context_.activeState.agc.autoEnabled = true;

	int32_t minExpUs =
		static_cast<int32_t>(agcCfg.minShutterSpeed.get<std::micro>());
	int32_t maxExpUs =
		static_cast<int32_t>(agcCfg.maxShutterSpeed.get<std::micro>());
	int32_t defExpUs =
		static_cast<int32_t>(agcCfg.defaultExposure *
				     lineDuration.get<std::micro>());
	defExpUs = std::clamp(defExpUs, minExpUs, maxExpUs);

	/* Initialize CCM to identity matrix */
	context_.activeState.ccm.matrix = {
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f
	};
	context_.activeState.ccm.offsets = { 0.0f, 0.0f, 0.0f };

	/* Initialize CBC to neutral (brightness=0, contrast=16=1.0×) */
	context_.activeState.cbc.brightness = 0;
	context_.activeState.cbc.contrast = 16;

	*ipaControls = ControlInfoMap(context_.ctrlMap, controls::controls);

	/*
	 * Build a priority-sorted algorithm list for deterministic processing
	 * order in processStats(). Rebuilding here ensures it reflects any
	 * algorithm set changes after configure(). Stable sort preserves YAML
	 * order for algorithms at the same priority.
	 */
	sortedAlgorithms_.clear();
	for (auto const &a : algorithms())
		sortedAlgorithms_.push_back(static_cast<Algorithm *>(a.get()));
	std::stable_sort(sortedAlgorithms_.begin(), sortedAlgorithms_.end(),
			 [](const Algorithm *a, const Algorithm *b) {
				 return a->priority() < b->priority();
			 });

	return 0;
}

void IPAMicrochipISC::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(
			buffer.id,
			MappedFrameBuffer(
				&fb,
				MappedFrameBuffer::MapFlag::Read));
	}
}

void IPAMicrochipISC::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(id);
	}
}

void IPAMicrochipISC::queueRequest(const uint32_t request,
				   const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(request);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->queueRequest(context_, request, frameContext, controls);
	}
}

void IPAMicrochipISC::processStats(unsigned int request,
				   unsigned int bufferId,
				   const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(request);

	const isc_stat_buffer *stats = reinterpret_cast<isc_stat_buffer *>(
		buffers_.at(bufferId).planes()[0].data());

	/* Record what sensor controls were applied for this frame */
	frameContext.agc.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	if (camHelper_)
		frameContext.agc.analogueGain = camHelper_->gain(
			sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());
	else
		frameContext.agc.analogueGain =
			static_cast<float>(camHelper_ ? camHelper_->gain(rawGain)
						      : static_cast<double>(rawGain));
}

/*
	 * Update per-channel histogram means and capture-frame attribution.
	 *
	 * The ISC cycles GR→R→GB→B (one Bayer channel per frame), but the
	 * kernel driver fires isc_stats_isr() only once after the B channel
	 * completes the 4-frame cycle, delivering all four histograms together
	 * (valid_channels = 0x0f).  All four means are therefore always fresh
	 * in each stats delivery; there is no stale-channel problem.
	 *
	 * Each channel also carries capture_frame — the frame sequence number
	 * at which that channel's histogram was actually measured.  Stored in
	 * activeState.histogram.captureFrame[] for AWB to look up the exposure
	 * and gains that were active on the measurement frame.
	 */
if (context_.configuration.hw.statsAvailable &&
    stats->valid_channels == 0x0f) {
	auto &hist = context_.activeState.histogram;
	for (unsigned int ch = 0; ch < 4; ch++) {
		hist.channelMeans[ch] = calculateChannelMean(stats, ch);
		hist.captureFrame[ch] = stats->hist[ch].capture_frame;
	}
}

/*
	 * Pre-compute per-frame scene analysis before the algorithm loop so
	 * that all algorithms share a single consistent histogram pass.
	 * Luminance is derived from the rolling channel mean cache so that
	 * each channel contributes its most recently measured value rather
	 * than reading all four from a single stats buffer where three are
	 * stale. Spread and clipping still read directly from the stats
	 * buffer (full histogram shape required; per-entry caching is costly).
	 * For frames 0-3 (warmup) or when stats are unavailable, all fields
	 * are zero-initialised; algorithms have their own guards.
	 */
if (context_.configuration.hw.statsAvailable &&
    stats->valid_channels == 0x0f) {
	/*
		 * All four channel means were just updated above from the
		 * current stats delivery (valid_channels = 0x0f, all fresh).
		 * Luminance is used by AGC, CBC, and Gamma. Since the histogram
		 * taps pre-WB, channel means are raw sensor values — no gain
		 * undo is needed. luminanceUndone equals luminance.
		 */
	const auto &hist = context_.activeState.histogram;
	const auto &m = hist.channelMeans;

	float meanG = (m[kHistChannelGR] + m[kHistChannelGB]) / 2.0f;
	frameContext.scene.luminance = 0.299f * m[kHistChannelR] +
				       0.587f * meanG +
				       0.114f * m[kHistChannelB];

	/*
		 * Histogram taps pre-WB Bayer data, so channel means already
		 * represent true sensor signal. luminanceUndone == luminance.
		 */
	frameContext.scene.luminanceUndone = frameContext.scene.luminance;

	frameContext.scene.spread = calculateHistogramSpread(stats);
	frameContext.scene.clipping = calculateClipping(stats);
	frameContext.scene.isDark =
		(frameContext.scene.luminance / kHistogramMax) < 0.02f;
	frameContext.scene.isHighContrast = frameContext.scene.clipping > 0.02f;
} else {
	frameContext.scene.luminance = 0.0f;
	frameContext.scene.spread = 0.0f;
	frameContext.scene.clipping = 0.0f;
	frameContext.scene.isDark = true;
	frameContext.scene.isHighContrast = false;
}

ControlList metadata(controls::controls);

for (Algorithm *algo : sortedAlgorithms_)
	algo->process(context_, request, frameContext, stats, metadata);

setSensorControls();
setIspControls();

statsProcessed.emit(request, metadata);
}

/**
 * \brief Emit sensor controls from AGC active state
 *
 * Converts the AGC-computed exposure (in lines) and gain (as float multiplier)
 * to V4L2 controls and emits the setSensorControls signal.
 */
void IPAMicrochipISC::setSensorControls()
{
	ControlList ctrls(sensorControls_);

	ctrls.set(V4L2_CID_EXPOSURE,
		  static_cast<int32_t>(context_.activeState.agc.exposure));

	int32_t gainCode;
	if (camHelper_)
		gainCode = camHelper_->gainCode(
			context_.activeState.agc.analogueGain);
	else
		gainCode = static_cast<int32_t>(
			context_.activeState.agc.analogueGain);

	ctrls.set(V4L2_CID_ANALOGUE_GAIN, gainCode);

	IPAMicrochipISCInterface::setSensorControls.emit(ctrls);
}

/**
 * \brief Emit ISP controls from AWB and CCM active state
 *
 * Converts AWB gains (float, 1.0 = unity) to hardware scale (512 = 1.0x)
 * and CCM matrix (float, 1.0 = unity) to hardware scale (256 = 1.0x),
 * then emits the setIspControls signal for the ISC subdev.
 */
void IPAMicrochipISC::setIspControls()
{
	ControlList ctrls(sensorControls_);

	/* AWB gains: convert float (1.0=unity) to fixed-point (512=1.0x) */
	const auto &awb = context_.activeState.awb;
	ctrls.set(V4L2_CID_ISC_WB_R_GAIN,
		  static_cast<int32_t>(awb.rGain * 512.0));
	ctrls.set(V4L2_CID_ISC_WB_B_GAIN,
		  static_cast<int32_t>(awb.bGain * 512.0));
	ctrls.set(V4L2_CID_ISC_WB_GR_GAIN,
		  static_cast<int32_t>(awb.grGain * 512.0));
	ctrls.set(V4L2_CID_ISC_WB_GB_GAIN,
		  static_cast<int32_t>(awb.gbGain * 512.0));

	/* AWB offsets */
	ctrls.set(V4L2_CID_ISC_WB_R_OFFSET, awb.rOffset);
	ctrls.set(V4L2_CID_ISC_WB_B_OFFSET, awb.bOffset);
	ctrls.set(V4L2_CID_ISC_WB_GR_OFFSET, awb.grOffset);
	ctrls.set(V4L2_CID_ISC_WB_GB_OFFSET, awb.gbOffset);

	/* CCM: convert float (1.0=unity) to hardware scale (256=1.0x) */
	const auto &ccm = context_.activeState.ccm;
	auto clampCcm = [](float v) -> int32_t {
		int32_t hw = static_cast<int32_t>(std::round(v * 256.0f));
		return std::clamp(hw, -2048, 2047);
	};

	ctrls.set(V4L2_CID_ISC_CC_RR, clampCcm(ccm.matrix[0]));
	ctrls.set(V4L2_CID_ISC_CC_RG, clampCcm(ccm.matrix[1]));
	ctrls.set(V4L2_CID_ISC_CC_RB, clampCcm(ccm.matrix[2]));
	ctrls.set(V4L2_CID_ISC_CC_GR, clampCcm(ccm.matrix[3]));
	ctrls.set(V4L2_CID_ISC_CC_GG, clampCcm(ccm.matrix[4]));
	ctrls.set(V4L2_CID_ISC_CC_GB, clampCcm(ccm.matrix[5]));
	ctrls.set(V4L2_CID_ISC_CC_BR, clampCcm(ccm.matrix[6]));
	ctrls.set(V4L2_CID_ISC_CC_BG, clampCcm(ccm.matrix[7]));
	ctrls.set(V4L2_CID_ISC_CC_BB, clampCcm(ccm.matrix[8]));

	/* CCM offsets */
	ctrls.set(V4L2_CID_ISC_CC_OR, static_cast<int32_t>(ccm.offsets[0]));
	ctrls.set(V4L2_CID_ISC_CC_OG, static_cast<int32_t>(ccm.offsets[1]));
	ctrls.set(V4L2_CID_ISC_CC_OB, static_cast<int32_t>(ccm.offsets[2]));

	/* CBC: adaptive brightness and contrast (ISC CBC block, post-gamma) */
	ctrls.set(V4L2_CID_BRIGHTNESS, context_.activeState.cbc.brightness);
	ctrls.set(V4L2_CID_CONTRAST, context_.activeState.cbc.contrast);

	IPAMicrochipISCInterface::setIspControls.emit(ctrls);
}

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"microchip-isc",
	"microchip-isc",
};

IPAInterface *ipaCreate()
{
	return new ipa::microchip_isc::IPAMicrochipISC();
}
}
