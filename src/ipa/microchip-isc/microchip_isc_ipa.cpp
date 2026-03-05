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
#include <libcamera/base/span.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"
#include "algorithms/common.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"
#include "isc_controls.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_IPA)

namespace ipa::microchip_isc {

static constexpr uint32_t kMaxFrameContexts = 16;

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

	/*
	 * Platform hardware capabilities are provided by the pipeline handler
	 * via IPAConfigInfo::hwCaps. Store them now so algorithms can read
	 * them during their init() calls below. The configure() path also
	 * writes hwCaps on each reconfigure, so these are always current.
	 */
	context_.configuration.hw.hasDpc           = ipaConfig.hwCaps.hasDpc;
	context_.configuration.hw.hasBipartiteGamma = ipaConfig.hwCaps.hasBipartiteGamma;
	context_.configuration.hw.hasCbhs          = ipaConfig.hwCaps.hasCbhs;

	LOG(ISC_IPA, Debug)
		<< "Platform capabilities: hasDpc=" << ipaConfig.hwCaps.hasDpc
		<< " hasBipartiteGamma=" << ipaConfig.hwCaps.hasBipartiteGamma
		<< " hasCbhs=" << ipaConfig.hwCaps.hasCbhs
		<< " statsAvailable=" << ipaConfig.hwCaps.statsAvailable;

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret) {
		LOG(ISC_IPA, Error) << "Failed to create algorithms";
		return ret;
	}

	/* Populate IPA controls from context */
	*ipaControls = ControlInfoMap(ControlInfoMap::Map(context_.ctrlMap), controls::controls);

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

	/*
	 * Hardware capability flags come from the pipeline handler via IPC.
	 * Re-apply on each configure() in case the IPA is shared across
	 * multiple camera instances with different hardware variants.
	 */
	context_.configuration.hw.hasDpc           = ipaConfig.hwCaps.hasDpc;
	context_.configuration.hw.hasBipartiteGamma = ipaConfig.hwCaps.hasBipartiteGamma;
	context_.configuration.hw.hasCbhs          = ipaConfig.hwCaps.hasCbhs;

	/*
	 * statsAvailable requires both a valid Bayer input (bayerOrder ≠
	 * kBayerOrderInvalid) and that the pipeline handler successfully
	 * opened the stats V4L2 metadata node (hwCaps.statsAvailable).
	 */
	context_.configuration.hw.statsAvailable =
		(bayerOrder != kBayerOrderInvalid) && ipaConfig.hwCaps.statsAvailable;

	/*
	 * Initialize activeState defaults BEFORE calling algo->configure() so
	 * that algorithms can safely read activeState during their configure()
	 * and their writes are not overwritten afterwards.
	 */

	/* AGC: start in full auto mode */
	context_.activeState.agc.autoEnabled = true;
	context_.activeState.agc.exposureTimeMode = controls::ExposureTimeModeAuto;
	context_.activeState.agc.analogueGainMode = controls::AnalogueGainModeAuto;

	/*
	 * AWB: start with identity gains. Awb::configure() will replace these
	 * with the YAML reference_gains once the algorithm runs.
	 */
	context_.activeState.awb.rGain = 1.0;
	context_.activeState.awb.bGain = 1.0;
	context_.activeState.awb.grGain = 1.0;
	context_.activeState.awb.gbGain = 1.0;
	context_.activeState.awb.autoEnabled = true;
	context_.activeState.awb.ct = 5500; /* Neutral daylight default */
	context_.activeState.awb.awbConverged = false;

	/*
	 * Black level correction:
	 * On SAMA7G5 (hasDpc=true) the DPC hardware block subtracts the
	 * sensor black level before the WB block runs. Applying WB offsets
	 * here would double-subtract and corrupt colours. Set offsets to 0.
	 *
	 * On SAMA5D2 (hasDpc=false) the WB block is the correct place to
	 * apply BL correction. Per-channel values are loaded by the
	 * BlackLevelCorrection algorithm from the tuning file and stored in
	 * context.configuration.sensor. The ISC pipeline is 12-bit; sensor
	 * BL in 10-bit units is scaled ×4.
	 */
	if (context_.configuration.hw.hasDpc) {
		context_.activeState.awb.rOffset = 0;
		context_.activeState.awb.bOffset = 0;
		context_.activeState.awb.grOffset = 0;
		context_.activeState.awb.gbOffset = 0;
		LOG(ISC_IPA, Debug) << "DPC present: WB BL offsets set to 0";
	} else {
		context_.activeState.awb.rOffset =
			-static_cast<int32_t>(context_.configuration.sensor.blackLevelR * 4);
		context_.activeState.awb.bOffset =
			-static_cast<int32_t>(context_.configuration.sensor.blackLevelB * 4);
		context_.activeState.awb.grOffset =
			-static_cast<int32_t>(context_.configuration.sensor.blackLevelGR * 4);
		context_.activeState.awb.gbOffset =
			-static_cast<int32_t>(context_.configuration.sensor.blackLevelGB * 4);
		LOG(ISC_IPA, Debug)
			<< "No DPC: WB BL offsets R=" << context_.activeState.awb.rOffset
			<< " B=" << context_.activeState.awb.bOffset;
	}

	/* CCM: identity matrix until the first process() call */
	context_.activeState.ccm.matrix = {
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f
	};
	context_.activeState.ccm.offsets = { 0.0f, 0.0f, 0.0f };

	/*
	 * CBC: neutral defaults. Contrast hardware unity differs by platform:
	 * SAMA7G5 CBHS uses 16, SAMA5D2 CBC uses 256. Using the wrong value
	 * would apply a permanent 16× or 1/16× multiplier from the first frame.
	 */
	context_.configuration.hw.contrastUnityHw =
		context_.configuration.hw.hasCbhs ? 16.0f : 256.0f;
	context_.activeState.cbc.brightness = 0;
	context_.activeState.cbc.contrast =
		static_cast<int32_t>(context_.configuration.hw.contrastUnityHw);
	context_.activeState.cbc.saturation = 50;
	context_.activeState.cbc.hue = 0;

	/*
	 * Gamma: Gamma::configure() computes and stores a valid LUT and sets
	 * lutValid = true. Pre-mark as invalid so setIspControls() does not
	 * emit a zero LUT before configure() runs.
	 */
	context_.activeState.gamma.gammaEff = 2.2f;
	context_.activeState.gamma.lutValid = false;

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	*ipaControls = ControlInfoMap(ControlInfoMap::Map(context_.ctrlMap), controls::controls);

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

	/*
	 * Record what sensor controls were applied for this frame. The
	 * pipeline handler should always populate V4L2_CID_EXPOSURE and
	 * V4L2_CID_ANALOGUE_GAIN, but guard with contains() to avoid
	 * crashing on ControlValue::get<T>() of an empty value if a
	 * sensor driver omits them.
	 */
	if (sensorControls.contains(V4L2_CID_EXPOSURE))
		frameContext.agc.exposure =
			sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();

	if (sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
		int32_t rawGain =
			sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();
		frameContext.agc.analogueGain =
			camHelper_ ? camHelper_->gain(rawGain)
				   : static_cast<double>(rawGain);
	}

	/*
	 * Pre-compute per-frame scene analysis before the algorithm loop so
	 * that all algorithms share a single consistent histogram pass.
	 * Algorithms must read frameContext.scene rather than calling
	 * calculateLuminance() / calculateHistogramSpread() / calculateClipping()
	 * directly. For frames 0-3 (warmup) or when stats are unavailable,
	 * all fields are zero-initialised; algorithms have their own guards.
	 */
	if (context_.configuration.hw.statsAvailable && request >= 4) {
		frameContext.scene.luminance = calculateLuminance(stats);
		frameContext.scene.spread    = calculateHistogramSpread(stats);
		frameContext.scene.clipping  = calculateClipping(stats);
		frameContext.scene.isDark    =
			(frameContext.scene.luminance / kHistogramMax) < 0.02f;
		frameContext.scene.isHighContrast = frameContext.scene.clipping > 0.02f;
	} else {
		frameContext.scene.luminance      = 0.0f;
		frameContext.scene.spread         = 0.0f;
		frameContext.scene.clipping       = 0.0f;
		frameContext.scene.isDark         = true;
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
 * \brief Emit ISP controls from AWB, CCM, CBC, and gamma active state
 *
 * Converts AWB gains (float, 1.0 = unity) to hardware scale (512 = 1.0x),
 * CCM matrix (float, 1.0 = unity) to hardware scale (256 = 1.0x), gamma LUT
 * entries, and CBC brightness/contrast/saturation/hue. Saturation and hue
 * are only emitted on platforms with CBHS hardware (hasCbhs = true).
 */
void IPAMicrochipISC::setIspControls()
{
	/*
	 * Ideally this ControlList would be backed by an ISP-specific
	 * ControlInfoMap. It currently uses sensorControls_ because the IPA
	 * IPC interface (mojom) does not carry a separate ISP controls map
	 * without a build-time regeneration that requires the full Yocto
	 * environment. In practice ControlList::set(id, value) does not
	 * validate the control ID against the backing map, so the pipeline
	 * handler receives all ISP controls correctly by their V4L2 IDs.
	 * This is a known limitation to be addressed when the interface is
	 * extended.
	 */
	ControlList ctrls(sensorControls_);

	/* AWB gains: convert float (1.0=unity) to fixed-point (kWbGainScale=1.0x) */
	const auto &awb = context_.activeState.awb;
	ctrls.set(V4L2_CID_ISC_WB_R_GAIN,
		  static_cast<int32_t>(std::round(awb.rGain * kWbGainScale)));
	ctrls.set(V4L2_CID_ISC_WB_B_GAIN,
		  static_cast<int32_t>(std::round(awb.bGain * kWbGainScale)));
	ctrls.set(V4L2_CID_ISC_WB_GR_GAIN,
		  static_cast<int32_t>(std::round(awb.grGain * kWbGainScale)));
	ctrls.set(V4L2_CID_ISC_WB_GB_GAIN,
		  static_cast<int32_t>(std::round(awb.gbGain * kWbGainScale)));

	/* AWB offsets */
	ctrls.set(V4L2_CID_ISC_WB_R_OFFSET, awb.rOffset);
	ctrls.set(V4L2_CID_ISC_WB_B_OFFSET, awb.bOffset);
	ctrls.set(V4L2_CID_ISC_WB_GR_OFFSET, awb.grOffset);
	ctrls.set(V4L2_CID_ISC_WB_GB_OFFSET, awb.gbOffset);

	/* CCM: convert float (1.0=unity) to hardware scale (256=1.0x) */
	const auto &ccm = context_.activeState.ccm;
	auto clampCcm = [](float v) -> int32_t {
		int32_t hw = static_cast<int32_t>(std::round(v * kCcmScale));
		return std::clamp(hw, kCcmMinValue, kCcmMaxValue);
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

	/* CBC: adaptive brightness and contrast (ISC CBC/CBHS block, post-gamma) */
	ctrls.set(V4L2_CID_BRIGHTNESS, context_.activeState.cbc.brightness);
	ctrls.set(V4L2_CID_CONTRAST, context_.activeState.cbc.contrast);

	/*
	 * Saturation and Hue are only available on SAMA7G5 CBHS hardware.
	 * SAMA5D2 CBC exposes brightness and contrast only.
	 */
	if (context_.configuration.hw.hasCbhs) {
		ctrls.set(V4L2_CID_SATURATION, context_.activeState.cbc.saturation);
		ctrls.set(V4L2_CID_HUE, context_.activeState.cbc.hue);
	}

	/* Gamma LUT: per-channel 64-entry 10-bit tone curves */
	const auto &gamma = context_.activeState.gamma;
	if (gamma.lutValid) {
		ctrls.set(V4L2_CID_ISC_GAMMA_B_LUT,
			  ControlValue(Span<const uint32_t>(gamma.lutB.data(), 64)));
		ctrls.set(V4L2_CID_ISC_GAMMA_G_LUT,
			  ControlValue(Span<const uint32_t>(gamma.lutG.data(), 64)));
		ctrls.set(V4L2_CID_ISC_GAMMA_R_LUT,
			  ControlValue(Span<const uint32_t>(gamma.lutR.data(), 64)));
	}

	IPAMicrochipISCInterface::setIspControls.emit(ctrls);
}

} /* namespace ipa::microchip_isc */

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

} /* namespace libcamera */
