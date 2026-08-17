/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto Gain/Exposure Control for Microchip ISC
 */

#include "agc.h"

#include <algorithm>
#include <math.h>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_AGC)

namespace ipa::microchip_isc {

/**
 * \brief Interpolate y_target from a lux-indexed PWL table
 * \param table  Vector of {lux, target} pairs, sorted ascending by lux
 * \param lux    Current estimated scene illuminance
 * \param fallback  Value to return when table is empty
 * \return Interpolated brightness target in histogram units (0–511)
 *
 * Implements the standard piecewise-linear evaluation (libipa
 * Pwl::eval). Linear interpolation between adjacent table points;
 * clamps to first/last entry outside the table range.
 */
static float interpolateYTarget(const std::vector<Agc::YTargetPoint> &table,
				float lux,
				float fallback)
{
	if (table.empty())
		return fallback;

	/* Below first point: return first target */
	if (lux <= table.front().lux)
		return table.front().target * kHistogramMax;

	/* Above last point: return last target */
	if (lux >= table.back().lux)
		return table.back().target * kHistogramMax;

	/* Find bounding segment and interpolate */
	for (size_t i = 1; i < table.size(); i++) {
		if (lux <= table[i].lux) {
			float t = (lux - table[i - 1].lux) /
				  (table[i].lux - table[i - 1].lux);
			float target = table[i - 1].target +
				       t * (table[i].target - table[i - 1].target);
			return target * kHistogramMax;
		}
	}

	return table.back().target * kHistogramMax;
}

/**
 * \brief Initialize AGC from YAML tuning data
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for AGC
 * \return 0 on success
 *
 * Parses convergence parameters and gain/exposure limits from the tuning
 * file. Only MeteringMatrix is supported; the ISC histogram has no spatial
 * information so centre-weighted and spot modes are not implementable.
 */
int Agc::init(IPAContext &context, const YamlObject &tuningData)
{
	targetBrightness_ = tuningData["target_brightness"].get<float>(targetBrightness_);
	clipThreshold_ = tuningData["clip_threshold"].get<float>(clipThreshold_);
	aeLuminanceFloor_ = tuningData["ae_luminance_floor"].get<float>(aeLuminanceFloor_);
	highlightP85Threshold_ = tuningData["highlight_p85_threshold"].get<float>(highlightP85Threshold_);
	highlightP85Strength_ = tuningData["highlight_p85_strength"].get<float>(highlightP85Strength_);
	highlightP85Threshold_ = std::clamp(highlightP85Threshold_, 0.1f, 0.95f);
	highlightP85Strength_ = std::clamp(highlightP85Strength_, 0.0f, 1.0f);
	aeLuminanceFloor_ = std::clamp(aeLuminanceFloor_, 0.0f, 50.0f);
	/* Default EV offset in stops; combined with runtime ExposureValue. */
	evBias_ = std::clamp(tuningData["ev_default"].get<float>(0.0f), -2.0f, 2.0f);
	flickerHz_ = tuningData["flicker_hz"].get<float>(flickerHz_);
	/* Only 50 Hz and 60 Hz are valid mains frequencies; anything else disables. */
	if (flickerHz_ != 50.0f && flickerHz_ != 60.0f)
		flickerHz_ = 0.0f;
	zoneMeteringStrength_ = std::clamp(
		tuningData["zone_metering_strength"].get<float>(zoneMeteringStrength_),
		0.0f, 1.0f);
	iqmQlo_ = std::clamp(tuningData["ae_iqm_q_lo"].get<float>(iqmQlo_), 0.0f, 0.5f);
	iqmQhi_ = std::clamp(tuningData["ae_iqm_q_hi"].get<float>(iqmQhi_), 0.5f, 1.0f);
	highlightQhi_ = std::clamp(
		tuningData["ae_highlight_q_hi"].get<float>(highlightQhi_), 0.0f, 0.5f);
	highlightTarget_ = std::clamp(
		tuningData["ae_highlight_target"].get<float>(highlightTarget_), 0.1f, 1.0f);
	sceneEvThreshold_ = std::clamp(
		tuningData["scene_ev_threshold"].get<float>(sceneEvThreshold_), 0.1f, 1.0f);
	sceneEvBias_ = std::clamp(
		tuningData["scene_ev_bias"].get<float>(sceneEvBias_), -3.0f, 0.0f);
	sceneBrightP10_ = std::clamp(
		tuningData["scene_bright_p10"].get<float>(sceneBrightP10_), 1.0f, 400.0f);

	if (tuningData.contains("convergence")) {
		const YamlObject &conv = tuningData["convergence"];
		convergenceSpeed_ = conv["speed"].get<float>(convergenceSpeed_);
		startupSpeed_ = conv["startup_speed"].get<float>(startupSpeed_);
		startupFrames_ = conv["startup_frames"].get<unsigned int>(startupFrames_);
		stabilityThreshold_ = conv["stability_threshold"].get<float>(stabilityThreshold_);
		convergenceFrames_ = conv["frames"].get<unsigned int>(convergenceFrames_);
		nearTargetFraction_ = conv["near_target_fraction"].get<float>(nearTargetFraction_);
		aeLockFrames_ = conv["lock_frames"].get<unsigned int>(aeLockFrames_);
	}

	if (tuningData.contains("ratio_clamp")) {
		const YamlObject &rc = tuningData["ratio_clamp"];
		ratioMin_ = rc["min"].get<float>(ratioMin_);
		ratioMax_ = rc["max"].get<float>(ratioMax_);
		clipRatioCap_ = rc["clip_cap"].get<float>(clipRatioCap_);
	}

	if (tuningData.contains("gain_thresholds")) {
		const YamlObject &gt = tuningData["gain_thresholds"];
		gainBoostThreshold_ = gt["boost"].get<float>(gainBoostThreshold_);
		gainReductionThreshold_ = gt["reduction"].get<float>(gainReductionThreshold_);
	}

	/*
	 * Lux estimator calibration. Measure once with a handheld lux meter:
	 *   1. Point camera at a scene under known illumination
	 *   2. Let AGC converge (AWB+AE locked)
	 *   3. Record: histogram_Y, exposure_us, analogue_gain, lux reading
	 *   4. Store in imx219.yaml [Agc] lux_reference block
	 * If absent, lux estimation is disabled and y_target[1] is used as a
	 * fixed target (backward-compatible with un-calibrated sensors).
	 */
	if (tuningData.contains("lux_reference")) {
		const YamlObject &lr = tuningData["lux_reference"];
		luxRefY_ = lr["reference_Y"].get<float>(0.0f);
		luxRefExposureUs_ = lr["reference_exposure_us"].get<float>(0.0f);
		luxRefGain_ = lr["reference_gain"].get<float>(1.0f);
		luxRefLux_ = lr["reference_lux"].get<float>(400.0f);
		/* Also push to session config so processStats() can access them */
		context.configuration.agc.refLuxY = luxRefY_;
		context.configuration.agc.refLuxExposureUs = luxRefExposureUs_;
		context.configuration.agc.refLuxGain = luxRefGain_;
		context.configuration.agc.refLux = luxRefLux_;

		/*
		 * IIR smoothing coefficient α ∈ (0,1].
		 * smoothed_lux = α × raw + (1−α) × prev
		 * 1.0 = no smoothing. Default 0.3.
		 */
		float alpha = tuningData["lux_smoothing_alpha"].get<float>(0.3f);
		context.configuration.agc.luxSmoothingAlpha =
			std::clamp(alpha, 0.05f, 1.0f);
	}
	/* Store P85 and EV params to configuration for cross-module visibility */
	context.configuration.agc.highlightP85Threshold = highlightP85Threshold_;
	context.configuration.agc.highlightP85Strength = highlightP85Strength_;
	context.configuration.agc.evBias = evBias_;

	/*
	 * y_target PWL table: maps lux → normalised brightness target [0,1].
	 * Mirrors RPi agc_channel.cpp y_target (BSD-2-Clause):
	 *   The target is normalised to kHistogramMax (511) when used.
	 *   Lower lux → lower target because downstream CBC contrast boost
	 *   and WB gain chain amplify dark scenes more than bright scenes.
	 *   If absent, falls back to fixed targetBrightness_.
	 */
	if (tuningData.contains("y_target")) {
		const auto &pts = tuningData["y_target"].asList();
		for (auto it = pts.begin(); it != pts.end();) {
			float lux = it->get<float>(0.0f);
			++it;
			if (it == pts.end())
				break;
			float target = it->get<float>(0.0f);
			++it;
			yTargetTable_.push_back({ lux, target });
		}
		/*
		 * Sort ascending by lux so interpolateYTarget() binary search
		 * is correct regardless of YAML entry order. Without this, an
		 * out-of-order YAML table (e.g. 300K before 100K) causes the
		 * first matching segment to be wrong and returns a biased target.
		 */
		std::sort(yTargetTable_.begin(), yTargetTable_.end(),
			  [](const YTargetPoint &a, const YTargetPoint &b) {
				  return a.lux < b.lux;
			  });
	}

	/*
	 * convergenceFrames_ is a backward-compat floor: raises convergenceSpeed_
	 * to at least 1/N so convergence is guaranteed within N stats deliveries.
	 * Applied unconditionally when present — an explicit speed= in YAML that
	 * is faster than 1/frames is unaffected (the if-guard ensures we only
	 * raise, never lower). If the intent is a maximum N-frame bound, set
	 * frames= to N; if the intent is a specific speed, set speed= directly
	 * and leave frames= absent. Do not rely on frames= to limit speed.
	 */
	if (convergenceFrames_ > 0) {
		float frameSpeed = 1.0f / static_cast<float>(convergenceFrames_);
		if (frameSpeed > convergenceSpeed_)
			convergenceSpeed_ = frameSpeed;
	}

	convergenceSpeed_ = std::clamp(convergenceSpeed_, 0.01f, 1.0f);
	startupSpeed_ = std::clamp(startupSpeed_, 0.1f, 1.0f);
	nearTargetFraction_ = std::clamp(nearTargetFraction_, 0.0f, 0.5f);
	clipThreshold_ = std::clamp(clipThreshold_, 0.001f, 0.5f);
	ratioMin_ = std::clamp(ratioMin_, 0.1f, 0.9f);
	ratioMax_ = std::clamp(ratioMax_, 1.1f, 8.0f);
	clipRatioCap_ = std::clamp(clipRatioCap_, 0.1f, 1.0f);
	gainBoostThreshold_ = std::clamp(gainBoostThreshold_, 1.0f, 2.0f);
	gainReductionThreshold_ = std::clamp(gainReductionThreshold_, 0.5f, 1.0f);

	/* Advertise ExposureValue: ±2 EV in 0.1-stop steps */
	context.ctrlMap[&controls::ExposureValue] =
		ControlInfo(-2.0f, 2.0f, 0.0f);

	/*
	 * FrameDuration: per-frame read-only metadata reporting actual frame
	 * duration in µs. Range is placeholder; configure() overwrites it with
	 * sensor-mode-specific limits derived from HBLANK/VBLANK.
	 */
	context.ctrlMap[&controls::FrameDuration] =
		ControlInfo(static_cast<int64_t>(0),
			    static_cast<int64_t>(1000000),
			    static_cast<int64_t>(33333));

	/* Advertise ExposureTimeMode, AnalogueGainMode, AeMeteringMode, AeState */
	context.ctrlMap[&controls::AeState] =
		ControlInfo(controls::AeStateIdle, controls::AeStateConverged,
			    controls::AeStateIdle);
	context.ctrlMap[&controls::ExposureTimeMode] =
		ControlInfo({ { ControlValue(controls::ExposureTimeModeAuto),
				ControlValue(controls::ExposureTimeModeManual) } },
			    ControlValue(controls::ExposureTimeModeAuto));
	context.ctrlMap[&controls::AnalogueGainMode] =
		ControlInfo({ { ControlValue(controls::AnalogueGainModeAuto),
				ControlValue(controls::AnalogueGainModeManual) } },
			    ControlValue(controls::AnalogueGainModeAuto));
	/*
	 * Only MeteringMatrix is supported. The ISC hardware histogram does
	 * not carry spatial information; centre-weighted and spot metering
	 * are not implementable without a separate statistics block.
	 */
	context.ctrlMap[&controls::AeMeteringMode] =
		ControlInfo(std::vector<ControlValue>{ ControlValue(controls::MeteringMatrix) },
			    ControlValue(controls::MeteringMatrix));

	LOG(ISC_AGC, Debug)
		<< "AGC initialized: target=" << targetBrightness_
		<< " speed=" << convergenceSpeed_
		<< " startupSpeed=" << startupSpeed_
		<< " startupFrames=" << startupFrames_
		<< " nearTargetFraction=" << nearTargetFraction_
		<< " lockFrames=" << aeLockFrames_
		<< " threshold=" << stabilityThreshold_
		<< " frames=" << convergenceFrames_
		<< " clipThreshold=" << clipThreshold_
		<< " ratioRange=[" << ratioMin_ << "," << ratioMax_ << "]"
		<< " clipCap=" << clipRatioCap_
		<< " gainThresholds=[" << gainReductionThreshold_
		<< "," << gainBoostThreshold_ << "]";

	/* Verify new parameters are loaded */
	LOG(ISC_AGC, Debug)
		<< "AGC init extras: evBias=" << evBias_
		<< " p85thresh=" << highlightP85Threshold_
		<< " zoneStrength=" << zoneMeteringStrength_
		<< " targetBrightness=" << targetBrightness_;

	return 0;
}

/**
 * \brief Configure AGC with sensor limits
 * \param[in] context The IPA context
 * \param[in] configInfo Camera sensor information
 * \return 0 on success
 *
 * Populates session configuration with sensor exposure and gain limits
 * and sets initial active state to sensor defaults.
 */
int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	auto &agc = context.configuration.agc;

	/*
	 * Initialise sensor-limit-derived exposure and gain defaults.
	 * Mode fields (autoEnabled, exposureTimeMode, analogueGainMode) are
	 * owned by IPAMicrochipISC::configure() which runs after all algorithm
	 * configure() calls and is the single authority for shared active state
	 * initialisation.
	 */
	context.activeState.agc.exposure = agc.defaultExposure;
	context.activeState.agc.analogueGain = agc.minAnalogueGain;
	context.activeState.agc.zoneRatio = 1.0f;

	/*
	 * Re-read ev_default and zone_metering_strength from the tuning YAML
	 * in configure() so they take effect on every reconfigure, not just
	 * at init() time.  configure() is called after YAML is fully parsed
	 * so this is the most reliable place to read these values.
	 * This also fixes the init()-vs-configure() race that caused evBias_
	 * to always read as 0.0 (the init() path was compiled but the YAML
	 * object pointer it received was not yet valid on some call paths).
	 */
	evBias_ = std::clamp(
		agc.evBias,
		-2.0f, 2.0f);
	zoneMeteringStrength_ = std::clamp(
		context.configuration.agc.highlightP85Threshold > 0.0f
			? zoneMeteringStrength_
			: 0.0f,
		0.0f, 1.0f);

	/* Initialise lux estimator reference from YAML — 0 means uncalibrated */
	/*
	 * Seed to zero so processStats() uses the first measurement directly
	 * rather than ramping from the arbitrary indoor fallback.
	 */
	context.activeState.lux.lux = 0.0f;

	/* Initialise CBC contrast gain to unity so AGC target is unmodified before first CBC cycle */
	context.activeState.cbc.contrastGainLinear = 1.0f;

	LOG(ISC_AGC, Debug)
		<< "AGC configured: exposure=" << context.activeState.agc.exposure
		<< " gain=" << context.activeState.agc.analogueGain
		<< " evBias=" << evBias_
		<< " p85thresh=" << highlightP85Threshold_
		<< " targetBrightness=" << targetBrightness_;

	return 0;
}

/**
 * \brief Handle per-frame request controls for AGC
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[out] frameContext The frame context to populate
 * \param[in] controls Application-supplied controls for this frame
 *
 * Handles AeEnable to toggle auto/manual mode, ExposureTimeMode and
 * AnalogueGainMode for independent control of exposure and gain, and
 * AeMeteringMode for metering strategy selection. Manual override values
 * are read from ExposureTime and AnalogueGain controls.
 */
void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &agc = frameContext.agc;

	const auto &aeEnable = controls.get(controls::AeEnable);
	if (aeEnable.has_value()) {
		agc.autoEnabled = aeEnable.value();
		context.activeState.agc.autoEnabled = aeEnable.value();
	} else {
		agc.autoEnabled = context.activeState.agc.autoEnabled;
	}

	/* Independent exposure time mode (overrides AeEnable for exposure) */
	const auto &expMode = controls.get(controls::ExposureTimeMode);
	if (expMode.has_value()) {
		agc.exposureTimeMode = expMode.value();
		context.activeState.agc.exposureTimeMode = expMode.value();
	} else {
		agc.exposureTimeMode = context.activeState.agc.exposureTimeMode;
	}

	/* Independent analogue gain mode (overrides AeEnable for gain) */
	const auto &gainMode = controls.get(controls::AnalogueGainMode);
	if (gainMode.has_value()) {
		agc.analogueGainMode = gainMode.value();
		context.activeState.agc.analogueGainMode = gainMode.value();
	} else {
		agc.analogueGainMode = context.activeState.agc.analogueGainMode;
	}

	/* EV compensation: scale target brightness up/down by 2^ev stops */
	const auto &evControl = controls.get(controls::ExposureValue);
	if (evControl.has_value())
		context.activeState.agc.exposureValue =
			std::clamp(evControl.value(), -2.0f, 2.0f);

	bool exposureManual = !agc.autoEnabled ||
			      agc.exposureTimeMode == controls::ExposureTimeModeManual;
	bool gainManual = !agc.autoEnabled ||
			  agc.analogueGainMode == controls::AnalogueGainModeManual;

	if (exposureManual) {
		const auto &exposureTime = controls.get(controls::ExposureTime);
		if (exposureTime.has_value()) {
			uint32_t lines = static_cast<uint32_t>(std::round(
				exposureTime.value() /
				context.configuration.sensor.lineDuration.get<std::micro>()));
			agc.exposure = lines;
			context.activeState.agc.exposure = lines;
		} else {
			agc.exposure = context.activeState.agc.exposure;
		}
	} else {
		agc.exposure = context.activeState.agc.exposure;
	}

	if (gainManual) {
		const auto &gain = controls.get(controls::AnalogueGain);
		if (gain.has_value()) {
			agc.analogueGain = gain.value();
			context.activeState.agc.analogueGain = gain.value();
		} else {
			agc.analogueGain = context.activeState.agc.analogueGain;
		}
	} else {
		agc.analogueGain = context.activeState.agc.analogueGain;
	}

	/*
	 * Manual frame-rate lock. FrameDurationLimits carries [min, max] in
	 * microseconds; the caller sets min == max to pin a rate. Take the
	 * lower bound and convert it to a sensor VBLANK at emit time.
	 */
	const auto &frameDuration = controls.get(controls::FrameDurationLimits);
	if (frameDuration.has_value() && frameDuration->size() > 0 &&
	    (*frameDuration)[0] > 0)
		context.activeState.agc.frameDurationUs = (*frameDuration)[0];

	/* Manual digital gain override; 1.0 leaves the sensor default. */
	const auto &digitalGain = controls.get(controls::DigitalGain);
	if (digitalGain.has_value() && digitalGain.value() >= 1.0f)
		context.activeState.agc.digitalGain = digitalGain.value();
}

/**
 * \brief Process statistics and compute exposure/gain
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context (contains actual applied values)
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] metadata Control list for frame metadata
 *
 * Computes scene luminance from the ISC 4-channel 512-bin histogram and
 * adjusts exposure time and analogue gain independently based on
 * ExposureTimeMode and AnalogueGainMode.
 *
 * Exposure is preferred over gain to minimize noise; gain is only increased
 * when exposure is at its maximum. ExposureTimeMode=Manual locks the
 * shutter while allowing gain to auto-adjust, and vice versa.
 */
void Agc::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const mchp_isc_stat_buffer *stats,
		  ControlList &metadata)
{
	auto &agcState = context.activeState.agc;
	const auto &agcConfig = context.configuration.agc;

	bool exposureManual = !frameContext.agc.autoEnabled ||
			      frameContext.agc.exposureTimeMode == controls::ExposureTimeModeManual;
	bool gainManual = !frameContext.agc.autoEnabled ||
			  frameContext.agc.analogueGainMode == controls::AnalogueGainModeManual;

	/* In fully manual mode, report current values and skip auto computation */
	if (exposureManual && gainManual) {
		int32_t exposureUs = static_cast<int32_t>(
			frameContext.agc.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain, frameContext.agc.analogueGain);
		metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));
		metadata.set(controls::AeState,
			     static_cast<int32_t>(controls::AeStateIdle));
		return;
	}

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable) {
		int32_t exposureUs = static_cast<int32_t>(
			agcState.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain, agcState.analogueGain);
		metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));
		metadata.set(controls::AeState,
			     static_cast<int32_t>(controls::AeStateIdle));
		return;
	}

	/*
	 * Warmup guard: skip AGC computation until the IPA has received at
	 * least one complete 4-channel stats delivery and the first few
	 * frames of sensor stabilisation have passed.
	 */
	if (frame < 3) {
		LOG(ISC_AGC, Debug) << "AGC warmup frame " << frame << ", skipping";
		agcFrameCount_ = 0;
		int32_t exposureUs = static_cast<int32_t>(
			agcState.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain, agcState.analogueGain);
		metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));
		metadata.set(controls::AeState,
			     static_cast<int32_t>(controls::AeStateSearching));
		return;
	}

	/*
	 * Compute WB-corrected luminance, mirroring the reference
	 * computeInitialY() approach.
	 *
	 * The ISC histogram taps pre-WB Bayer data (vp_data, pre-DPC).
	 * Luminance metering: pre-WB histogram with normalised WB correction.
	 *
	 * This mirrors the reference computeInitialY() when the stats tap
	 * is pre-WB (AgcStatsPos::PreWb):
	 *
	 *   double minColourGain = std::min(gainR, gainG, gainB, 1.0);
	 *   minColourGain = std::max(minColourGain, 1.0);
	 *   colourGains /= minColourGain;          // normalise to lowest gain
	 *   sum *= colourGains;                    // apply normalised gains
	 *   ySum = rec601LuminanceFromRGB(sum);    // BT.601 luminance
	 *
	 * WHY NORMALISED GAINS (not raw grey-world gains):
	 *
	 * The ISC histogram is pre-WB (SAMA7G5 datasheet §43.6.20 fig. 43-38,
	 * ISC_HIS_CFG_RAR tap confirmed pre-WB).  Dividing all gains by the
	 * minimum (= the highest-signal channel, GR/GB ≈ 1.0) ensures:
	 *   1. The reference channel (GR, gain=1.0) is unchanged.
	 *   2. Lower-signal channels (R, B) are scaled UP to match GR.
	 *   3. Lowering exposure scales ALL histogram means proportionally,
	 *      so normalised-WB-corrected luminance drops linearly — the AE
	 *      feedback loop closes correctly.
	 *
	 * CONTRAST WITH PURE GREY-WORLD (previous bug):
	 *   Pure grey-world gains (GR/B ratio) are EXPOSURE-DEPENDENT:
	 *   at low exposure, B_net ≈ 5-10 bins for NoIR at 4450K (B near
	 *   noise floor) → bGain = GR/B spikes to 3-5× instead of the
	 *   calibrated 2.0× → WB-corrected luminance over-estimates the
	 *   scene brightness → AE drives higher exposure to compensate →
	 *   the loop diverges instead of converging.
	 *
	 * With the AWB now using CT-based reference_gains_table (not
	 * grey-world), the gains are stable across exposure levels, so
	 * the normalised correction here accurately predicts the post-WB
	 * output luminance regardless of exposure setting.
	 *
	 * TARGET CALIBRATION (SAMA7G5 + IMX219 NoIR):
	 *   18% grey → pixel DN = 0.18 × 1023 = 184.
	 *   Pedestal = 64 DN (IMX219 datasheet).
	 *   Net histogram bin = (184 - 64) / 2 = 60 net bins.
	 *   For a neutral scene after WB, all channels ≈ 60 net bins.
	 *   Normalised-WB luminance = 60 × (0.299 + 0.587 + 0.114) = 60.
	 *   → target_brightness = 60 for 18% grey sRGB output.
	 */
	const auto &hist = context.activeState.histogram;
	float meanR = hist.channelMeans[kHistChannelR];
	float meanGR = hist.channelMeans[kHistChannelGR];
	float meanGB = hist.channelMeans[kHistChannelGB];
	float meanB = hist.channelMeans[kHistChannelB];
	/*
	 * Apply normalised AWB gains: divide all channel gains by the
	 * minimum gain so the reference channel stays at 1× and weaker
	 * channels are scaled up.  This matches the reference pre-WB
	 * luminance metering.
	 */
	const auto &awb = context.activeState.awb;
	float wbR = (awb.rGain > 0.0f) ? awb.rGain : 1.0f;
	float wbGR = (awb.grGain > 0.0f) ? awb.grGain : 1.0f;
	float wbGB = (awb.gbGain > 0.0f) ? awb.gbGain : 1.0f;
	float wbB = (awb.bGain > 0.0f) ? awb.bGain : 1.0f;

	/*
	 * Normalise all WB gains relative to the minimum (the highest-signal
	 * channel, typically GR ≈ 1.0 after AWB converges).  The old
	 * std::max(..., 1.0f) floor prevented normalisation whenever any
	 * channel gain was below 1.0 — which is common during AWB startup —
	 * biasing currentLuminance systematically low.  Guard against zero
	 * only (to avoid divide-by-zero), not against below-unity values.
	 */
	float minWb = std::min({ wbR, wbGR, wbGB, wbB });
	if (minWb < 1e-6f)
		minWb = 1.0f;
	float nR = std::min(meanR * (wbR / minWb), kHistogramMax);
	float nGR = std::min(meanGR * (wbGR / minWb), kHistogramMax);
	float nGB = std::min(meanGB * (wbGB / minWb), kHistogramMax);
	float nB = std::min(meanB * (wbB / minWb), kHistogramMax);
	float nG = (nGR + nGB) / 2.0f;

	float currentLuminance = 0.299f * nR + 0.587f * nG + 0.114f * nB;

	/*
	 * Inter-quantile mean (IQM) scene metering.
	 *
	 * The WB-corrected BT.601 mean above is retained as currentLuminance
	 * for the feedback loop (it must be stable). IQM provides a separate
	 * mid-tone estimate that excludes the darkest iqmQlo fraction and the
	 * brightest (1-iqmQhi) fraction of the luminance histogram.
	 *
	 * For a scene where 12% of pixels are blown out (L>250), the arithmetic
	 * mean is pulled upward by those highlights. IQM(0.02, 0.98) ignores
	 * the top 2% and gives a stable midtone reference:
	 *   iqm = mean(pixels at P2 to P98)
	 * The ratio iqm/currentLuminance < 1 for over-bright scenes → stored
	 * as zoneRatio → scales effectiveTarget down → less exposure.
	 *
	 * Mirrors RPi agc_channel.cpp constraintComputeGain() approach
	 * (BSD-2-Clause, Copyright Raspberry Pi Ltd).
	 */
	context.activeState.agc.zoneRatio = 1.0f;
	if (stats && iqmQhi_ > iqmQlo_ && currentLuminance > 1.0f) {
		float iqm = calculateInterQuantileMean(stats, iqmQlo_, iqmQhi_);
		if (iqm > 0.0f) {
			float ratio = iqm / currentLuminance;
			context.activeState.agc.zoneRatio =
				std::clamp(ratio, 0.30f, 1.0f);
		}
	}

	/*
	 * Highlight constraint: if the brightest highlightQhi_ fraction of
	 * the luminance histogram exceeds highlightTarget_ (normalised 0-511),
	 * clamp the effective target so the AGC reduces exposure until the
	 * highlights fall to the target level.
	 *
	 * This is the "ConstraintHighlight" metering mode:
	 *   newGain = targetY * hist.bins() / iqm(qLo, qHi)
	 *   if newGain < gain: apply upper bound
	 * For us: if top highlightQhi_% > highlightTarget_, reduce effectiveTarget.
	 * Default: top 2% (P98-P100) should not exceed 70% of histogram max.
	 */
	if (stats && highlightQhi_ > 0.0f) {
		float p_hi = calculateInterQuantileMean(stats,
							1.0f - highlightQhi_,
							1.0f);
		float threshold = highlightTarget_ * kHistogramMax;
		if (p_hi > threshold && p_hi > 0.0f) {
			float hlScale = threshold / p_hi;
			context.activeState.agc.zoneRatio =
				std::min(context.activeState.agc.zoneRatio, hlScale);
		}
	}

	/* Count frames from the first active stats delivery */
	agcFrameCount_++;

	/*
	 * Two-phase convergence speed selection.
	 *
	 * Phase 1 — startup (agcFrameCount_ <= startupFrames_):
	 *   Use startupSpeed_ (default 1.0) so exposure and gain jump
	 *   directly to their target values in the first few frames.
	 *   This is a fast-start approach: reach roughly correct exposure
	 *   before the stability lock check can fire,
	 *   preventing false-convergence at the initial low luminance.
	 *
	 * Phase 2 — steady state (agcFrameCount_ > startupFrames_):
	 *   Switch to convergenceSpeed_ (default 0.3) for smooth IIR
	 *   tracking. This prevents overshooting and banding when the
	 *   scene changes gradually after the initial climb.
	 *
	 * Clipping always overrides startup speed to avoid burning
	 * highlights on a bright scene after a dark-to-bright cut.
	 */
	bool inStartup = (startupFrames_ > 0 && agcFrameCount_ <= startupFrames_);

	/*
	 * Determine the brightness target in pre-WB RAW histogram units.
	 *
	 * When a y_target PWL table is configured (imx219.yaml y_target block),
	 * interpolate the target from the current lux estimate. The target
	 * adapts to scene brightness so the full pipeline
	 * output stays close to the photographic reference level regardless of
	 * light level, analogue gain, WB chain, or CBC contrast boost.
	 *
	 * The lux estimate is computed in processStats() from current exposure,
	 * gain and histogram Y relative to the calibrated reference point.
	 * A dim scene (low lux) → lower target → less gain → less CBC boost
	 * → correct output brightness instead of overexposure.
	 *
	 * When no y_target table is configured (uncalibrated sensor), fall back
	 * to the fixed targetBrightness_ from YAML/header default. This gives
	 * the same behaviour as before on sensors without a lux calibration.
	 *
	 * Apply EV compensation on top of the lux-derived target.
	 */
	float currentLux = context.activeState.lux.lux;
	float baseTarget = interpolateYTarget(yTargetTable_, currentLux,
					      targetBrightness_);

	/*
	 * EV scale: 2^ev. Avoid std::pow() in the common case (ev=0) since
	 * pow(2,0)=1 is trivially known and transcendental functions are
	 * expensive on the Cortex-A7 without an FPU pow instruction.
	 */

	/*
	 * Scene entropy-based automatic EV bias.
	 *
	 * Uniformly bright scenes (white walls, overcast sky, lit studio)
	 * have low histogram entropy: nearly all pixels cluster at one
	 * luminance level. The AGC correctly exposes for the mean, but the
	 * result looks washed-out because there is no headroom for highlights.
	 *
	 * When entropy falls below scene_ev_threshold AND the luminance is
	 * above 1.5× the metering target, the scene is globally over-bright.
	 * Apply scene_ev_bias (default -1.5 stops) to under-expose and create
	 * headroom for the gamma CDF stretch that follows.
	 *
	 * The combined effect with an aggressive CDF blend is equivalent to
	 * the first pass of CLAHE: reduce exposure to avoid clipping, then
	 * redistribute the histogram to fill the full output range.
	 */
	/*
	 * Dual-condition scene classifier for flat/bright scenes.
	 *
	 * When P10 of the pre-gamma histogram exceeds scene_bright_p10 (few
	 * shadow pixels) AND the normalised histogram entropy is below
	 * scene_ev_threshold (compressed luminance distribution), the scene
	 * is globally over-bright.  Apply scene_ev_bias stops of deliberate
	 * under-exposure to create headroom for the CDF tone mapping.
	 */
	float sceneEvAdjust = 0.0f;
	if (stats && !inStartup) {
		float p10 = calculateHistogramPercentile(stats, 0.10f);
		float entNorm = calculateHistogramEntropy(stats);
		if ((p10 > sceneBrightP10_) && (entNorm < sceneEvThreshold_))
			sceneEvAdjust = sceneEvBias_;
	}

	float totalEv = evBias_ + sceneEvAdjust + agcState.exposureValue;
	float evScale = (totalEv == 0.0f) ? 1.0f : std::pow(2.0f, totalEv);
	float zoneScale = context.activeState.agc.zoneRatio;
	float effectiveTarget = std::clamp(baseTarget * evScale * zoneScale,
					   aeLuminanceFloor_, kHistogramMax);

	/*
	 * CBC feedback: if CBC is applying a contrast boost (contrastGainLinear
	 * > 1.0), the post-CBC output is brighter than the pre-CBC histogram
	 * that AGC measures. Divide the histogram target by contrastGainLinear
	 * so AGC exposes to a lower raw level — exactly enough that after CBC
	 * stretches it, the final output hits the intended brightness.
	 *
	 * Without this, CBC-boosted scenes (flat histograms like grey cards)
	 * are permanently overexposed: AGC locks at a raw level that looks
	 * correct pre-CBC, but the downstream CBC gain makes it too bright.
	 *
	 * contrastGainLinear is initialised to 1.0 in configure() and updated
	 * by CBC::process() every frame, so on the first few frames before CBC
	 * runs this is a safe no-op.
	 */
	float cbcGain = context.activeState.cbc.contrastGainLinear;
	if (cbcGain > 1.0f)
		effectiveTarget = std::clamp(effectiveTarget / cbcGain,
					     aeLuminanceFloor_, kHistogramMax);

	/*
	 * P85 highlight metering: when 85% of pixels exceed the configured
	 * threshold the scene is globally over-bright. Scale effectiveTarget
	 * down proportionally so the AGC reduces exposure until P85 falls
	 * to the threshold, preserving headroom for highlights.
	 */
	float p85Scale = 1.0f;
	if (stats && highlightP85Threshold_ > 0.0f) {
		float p85Bin = calculateHistogramPercentile(stats, 0.85f);
		float threshBin = highlightP85Threshold_ * kHistogramMax;
		if (p85Bin > threshBin && p85Bin > 0.0f) {
			float rawScale = threshBin / p85Bin;
			/* Blend with strength so 0=disabled, 1=full correction */
			p85Scale = 1.0f + (rawScale - 1.0f) *
						  std::clamp(highlightP85Strength_, 0.0f, 1.0f);
			effectiveTarget = std::clamp(effectiveTarget * p85Scale,
						     aeLuminanceFloor_,
						     kHistogramMax);
		}
	}

	/* Legacy hist_max highlight protection (single bright object) */
	float highlightFrac = frameContext.scene.clipping;
	if (stats) {
		static constexpr unsigned int kHiThreshold = 350;
		unsigned int maxBin = 0;
		for (unsigned int ch = 0; ch < 4; ch++) {
			if (stats->hist[ch].hist_max > maxBin)
				maxBin = stats->hist[ch].hist_max;
		}
		if (maxBin > kHiThreshold) {
			float excess = static_cast<float>(maxBin - kHiThreshold) /
				       static_cast<float>(kHistBins - 1 - kHiThreshold);
			float targetScale = std::clamp(1.0f - 0.68f * excess, 0.32f, 1.0f);
			effectiveTarget = std::clamp(effectiveTarget * targetScale,
						     aeLuminanceFloor_, kHistogramMax);
			/* Repurpose highlightFrac for the log line */
			highlightFrac = excess;
		}
	}

	LOG(ISC_AGC, Debug)
		<< "AGC target: lux=" << currentLux
		<< " baseTarget=" << baseTarget
		<< " evScale=" << evScale
		<< " cbcGain=" << cbcGain
		<< " highlightFrac=" << highlightFrac
		<< " effectiveTarget=" << effectiveTarget;

	/*
	 * Three-region speed selection — mirrors the reference filterExposure():
	 *
	 * 1. Startup (agcFrameCount_ <= startupFrames_):
	 *    speed = startupSpeed_ (default 1.0 — direct jump).
	 *    Bypasses IIR so exposure+gain reach target in 1–3 frames.
	 *    Clipping exits startup early to prevent highlight burns.
	 *
	 * 2. Near-target (|error| < nearTargetFraction_ * target):
	 *    speed = sqrt(convergenceSpeed_) — ~2× faster than steady state.
	 *    Eliminates micro-oscillation near the stability boundary where
	 *    the standard IIR rate leaves the loop hovering for many frames.
	 *    Default band is ±20% (nearTargetFraction_ = 0.2).
	 *
	 * 3. Steady state:
	 *    speed = convergenceSpeed_ (default 0.2) — smooth tracking.
	 *    Prevents overshoot and fluorescent banding on gradual changes.
	 */
	float activeSpeed;
	float error = std::abs(effectiveTarget - currentLuminance);
	bool nearTarget = (nearTargetFraction_ > 0.0f && effectiveTarget > 0.0f &&
			   error < nearTargetFraction_ * effectiveTarget);

	if (inStartup)
		activeSpeed = startupSpeed_;
	else if (nearTarget)
		activeSpeed = std::sqrt(convergenceSpeed_);
	else
		activeSpeed = convergenceSpeed_;

	/* Guard against zero lineDuration (sensor not configured or pixelRate=0). */
	if (context.configuration.sensor.lineDuration.get<std::nano>() <= 0) {
		LOG(ISC_AGC, Warning) << "lineDuration is zero, skipping AGC compute";
		return;
	}

	/*
	 * Round rather than truncate: Duration::get<>() returns double.
	 * Truncation would cap at N-1 lines when the true limit is N due to
	 * floating-point representation, underexposing dark scenes by one line.
	 */
	uint32_t maxExposureLimit = static_cast<uint32_t>(
		std::round(agcConfig.maxShutterSpeed.get<std::nano>() /
			   context.configuration.sensor.lineDuration.get<std::nano>()));
	/*
	 * Hardware limit detection: true when both exposure and gain are at
	 * (or very close to) their sensor maximums and cannot increase further.
	 * The 95% threshold (not 99%) accounts for the IIR rounding and the
	 * startup simultaneous-gain formula that may settle slightly below the
	 * hardware ceiling without triggering the gain-boost path.
	 */
	bool atHardwareLimit = !exposureManual && !gainManual &&
			       agcState.exposure >= maxExposureLimit &&
			       agcState.analogueGain >= agcConfig.maxAnalogueGain * 0.95f;

	/*
	 * Stability check: frame is stable if luminance is within the
	 * stability band OR if both sensor knobs are at their hardware
	 * maximum and the startup phase is complete. In the latter case
	 * the sensor has done everything it can — there is no point
	 * waiting indefinitely for a target it physically cannot reach.
	 * Mirrors the common pattern where frameCount > startupFrames + lockFrames
	 * eventually forces a lock even on scenes beyond the sensor's range.
	 */
	bool frameStable = (currentLuminance >= aeLuminanceFloor_ &&
			    error < stabilityThreshold_) ||
			   (atHardwareLimit && !inStartup);

	/*
	 * AE lock requires aeLockFrames_ consecutive frames within the
	 * stability band (default 5). A single on-target frame is not
	 * enough — the ISC 4-frame histogram latency means the sensor
	 * output may still be slewing while the first stable stats arrive.
	 * The consecutive count ensures the pipeline has fully settled.
	 * Lock is broken immediately if the scene shifts out of the band
	 * by more than 1.5× the stability threshold (hysteresis), unless
	 * the sensor is at its hardware limit (lock is sticky at the limit).
	 */
	if (frameStable) {
		if (aeLockCount_ < aeLockFrames_)
			aeLockCount_++;
	} else if (!atHardwareLimit && error > stabilityThreshold_ * 1.5f) {
		aeLockCount_ = 0;
	}
	bool aeStable = (aeLockCount_ >= aeLockFrames_);

	if (!aeStable) {
		float ratio = effectiveTarget / std::max(currentLuminance, 1.0f);
		ratio = std::clamp(ratio, ratioMin_, ratioMax_);

		/*
		 * Anti-windup for highlight clipping: in high-contrast outdoor
		 * scenes the luminance mean can appear low because large shadow
		 * areas drag the average down, even though highlights are
		 * already saturated. Without this guard the control loop boosts
		 * exposure further, causing oscillation. Cap the ratio to force
		 * a reduction whenever significant clipping is detected.
		 * Also exit startup phase early when clipping is detected —
		 * the scene is bright, no need for aggressive boost.
		 */
		float clipping = frameContext.scene.clipping;
		if (clipping > clipThreshold_) {
			ratio = std::min(ratio, clipRatioCap_);
			inStartup = false;
			activeSpeed = convergenceSpeed_;
			LOG(ISC_AGC, Debug)
				<< "Clipping " << clipping * 100.0f
				<< "% > " << clipThreshold_ * 100.0f
				<< "%, capping ratio=" << ratio;
		}

		uint32_t maxExposure = std::max(1u, maxExposureLimit);
		uint32_t minExposure = std::max(1u, static_cast<uint32_t>(
							    agcConfig.minShutterSpeed / context.configuration.sensor.lineDuration));

		/* Update exposure if not in manual mode */
		if (!exposureManual) {
			float currentExposure = static_cast<float>(frameContext.agc.exposure);
			float targetExposure = currentExposure * ratio;
			float newExposure = activeSpeed * targetExposure +
					    (1.0f - activeSpeed) * currentExposure;

			uint32_t exposure = std::clamp(static_cast<uint32_t>(newExposure),
						       minExposure, maxExposure);
			agcState.exposure = exposure;

			/*
			 * Flicker avoidance: round exposure to the nearest
			 * multiple of the mains half-period (1/(2*Hz) seconds)
			 * to prevent banding under fluorescent / LED lighting.
			 * A half-period multiple ensures exposure is a symmetric
			 * integral number of half-cycles for both 50 and 60 Hz.
			 * Disabled during startup phase to avoid rounding errors
			 * interfering with the initial coarse exposure jump.
			 */
			if (!inStartup && flickerHz_ > 0.0f) {
				float lineDurationUs =
					context.configuration.sensor.lineDuration
						.get<std::micro>();
				float halfPeriodLines =
					(500000.0f / flickerHz_) / lineDurationUs;
				if (halfPeriodLines >= 1.0f) {
					uint32_t nPeriods = static_cast<uint32_t>(
						std::round(static_cast<float>(agcState.exposure) / halfPeriodLines));
					/* Ensure at least one half-period */
					if (nPeriods == 0)
						nPeriods = 1;
					agcState.exposure = std::clamp(
						static_cast<uint32_t>(nPeriods * halfPeriodLines),
						minExposure, maxExposure);
				}
			}
		}

		/* Update gain if not in manual mode */
		if (!gainManual) {
			float analogueGain;

			if (exposureManual) {
				/*
				 * Exposure is manually locked: gain must cover
				 * the full brightness adjustment since exposure
				 * cannot move. Apply ratio directly to the
				 * current gain so the loop converges.
				 */
				analogueGain = agcState.analogueGain * ratio;
			} else if (inStartup && ratio > gainBoostThreshold_) {
				/*
				 * Startup phase with significant underexposure:
				 * move both exposure and gain simultaneously so
				 * the scene reaches target brightness in the
				 * minimum number of frames. Split the brightness
				 * correction across both knobs — exposure handles
				 * the portion up to maxExposure, gain covers the
				 * rest. This avoids the multi-frame exposure-only
				 * climb before gain is touched, significantly
				 * cutting convergence time for typical scenes.
				 */
				float totalBrightness =
					static_cast<float>(frameContext.agc.exposure) *
					agcState.analogueGain * ratio;
				float newGain = (maxExposure > 0)
							? totalBrightness / static_cast<float>(maxExposure)
							: agcConfig.maxAnalogueGain;
				analogueGain = std::max(newGain, agcConfig.minAnalogueGain);
			} else if (agcState.exposure >= maxExposure && ratio > gainBoostThreshold_) {
				/*
				 * Exposure is maxed and the scene is still
				 * underexposed: scale gain by the same ratio
				 * used for exposure. Since exposure is clamped at
				 * maxExposure and cannot move, all remaining
				 * brightness correction falls to gain.
				 * Use the current IPA gain (agcState.analogueGain)
				 * not the sensor-applied value (frameContext) to
				 * avoid stale-value drift after startup phase.
				 */
				analogueGain = agcState.analogueGain * ratio;
			} else if (ratio < gainReductionThreshold_ &&
				   agcState.exposure <= minExposure) {
				/*
				 * Scene overexposed AND exposure is already at
				 * minimum: exposure cannot go lower, so reduce
				 * gain toward minimum to correct overexposure.
				 * Exposure is always preferred over gain to
				 * minimise noise; gain is only reduced once
				 * the exposure knob is exhausted.
				 */
				analogueGain = agcConfig.minAnalogueGain;
			} else {
				/*
				 * Steady state: exposure is converging or can
				 * be reduced further. Hold the current gain
				 * and let the exposure loop handle correction;
				 * adjusting gain simultaneously in steady state
				 * causes oscillation and prevents stable lock.
				 */
				analogueGain = agcState.analogueGain;
			}

			analogueGain = std::clamp(analogueGain,
						  agcConfig.minAnalogueGain,
						  agcConfig.maxAnalogueGain);

			/* Temporal smoothing prevents sudden gain jumps */
			analogueGain = activeSpeed * analogueGain +
				       (1.0f - activeSpeed) *
					       frameContext.agc.analogueGain;

			agcState.analogueGain = analogueGain;
		}

		LOG(ISC_AGC, Debug)
			<< "AGC: luminance=" << currentLuminance
			<< " target=" << effectiveTarget
			<< " ev=" << agcState.exposureValue
			<< " ratio=" << ratio
			<< " speed=" << activeSpeed
			<< (inStartup ? " [startup]" : nearTarget ? " [near]"
								  : "")
			<< " lock=" << aeLockCount_ << "/" << aeLockFrames_
			<< " -> exposure=" << agcState.exposure
			<< " gain=" << agcState.analogueGain
			<< (exposureManual ? " (exp locked)" : "")
			<< (gainManual ? " (gain locked)" : "");
	} else {
		LOG(ISC_AGC, Debug)
			<< "AGC stable: luminance=" << currentLuminance
			<< " target=" << effectiveTarget
			<< " lock=" << aeLockCount_ << "/" << aeLockFrames_;
	}

	int32_t exposureUs = static_cast<int32_t>(
		agcState.exposure *
		context.configuration.sensor.lineDuration.get<std::micro>());
	metadata.set(controls::ExposureTime, exposureUs);
	metadata.set(controls::AnalogueGain, agcState.analogueGain);
	metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));

	/*
	 * Report AeState. Both exposure and gain are considered when deciding
	 * if the AEGC loop has converged, so convergence is declared only when
	 * both are within the stability band. Manual-only modes are reported
	 * as Idle since the algorithm is not computing values.
	 */
	int32_t aeState;
	if (exposureManual && gainManual) {
		aeState = controls::AeStateIdle;
	} else if (aeStable) {
		aeState = controls::AeStateConverged;
	} else {
		aeState = controls::AeStateSearching;
	}
	metadata.set(controls::AeState, aeState);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
