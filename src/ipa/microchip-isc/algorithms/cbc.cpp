/* SPDX-License-Identifier: LGPL-2.1-or-later */

/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Adaptive Contrast/Brightness Control for Microchip ISC
 */

#include "cbc.h"

#include <algorithm>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_CBC)

namespace ipa::microchip_isc {

/**
 * \brief Initialize the CBC algorithm from YAML tuning data
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for CBC
 * \return 0 on success
 */
int Cbc::init(IPAContext &context, const YamlObject &tuningData)
{
	defaultBrightness_ = tuningData["default_brightness"].get<int32_t>(defaultBrightness_);
	contrastStrength_ = tuningData["contrast_strength"].get<float>(contrastStrength_);
	targetSpread_ = tuningData["target_spread"].get<float>(targetSpread_);
	smoothingFactor_ = tuningData["smoothing_factor"].get<float>(smoothingFactor_);
	defaultSaturation_ = tuningData["saturation"].get<int32_t>(defaultSaturation_);
	defaultHue_ = tuningData["hue"].get<int32_t>(defaultHue_);
	contrastMaxGain_ = tuningData["contrast_max_gain"].get<float>(contrastMaxGain_);
	darkSceneMinLuminance_ = tuningData["dark_scene_min_luminance"].get<float>(darkSceneMinLuminance_);

	defaultBrightness_ = std::clamp(defaultBrightness_,
					static_cast<int32_t>(kBrightnessMin),
					static_cast<int32_t>(kBrightnessMax));
	contrastStrength_ = std::clamp(contrastStrength_, 0.0f, 1.0f);
	targetSpread_ = std::clamp(targetSpread_, 0.1f, 1.0f);
	smoothingFactor_ = std::clamp(smoothingFactor_, 0.0f, 0.95f);
	defaultSaturation_ = std::clamp(defaultSaturation_, 0, 100);
	defaultHue_ = std::clamp(defaultHue_, -180, 180);
	contrastMaxGain_ = std::clamp(contrastMaxGain_, 1.0f, 8.0f);

	/*
	 * The standard Saturation control and the Microchip Hue vendor
	 * control are only available on SAMA7G5 CBHS hardware.  The SAMA5D2
	 * CBC block has no saturation or hue registers, so advertising these
	 * controls unconditionally would expose ranges the hardware cannot
	 * honour.  Restrict registration to platforms where hasCbhs is set.
	 */
	/*
	 * Register the standard image quality controls so that apps (and
	 * --list-controls) can discover them and so that queueRequest()
	 * receives them in the request ControlList.
	 *
	 * Brightness: float [-1.0, +1.0], neutral 0.0
	 *   Converted to ISC CBHS register range [-1024, +1023]:
	 *   hw = clamp(v * 1023, -1024, 1023)   (mirrors rkisp1 cproc.cpp)
	 *
	 * Contrast: float [0.0, 2.0], neutral 1.0
	 *   Passed to the adaptive contrast engine as a user multiplier on
	 *   top of the histogram-derived gain. 1.0 = no user adjustment.
	 *   Stored as float in activeState; CBC process() multiplies the
	 *   blended contrast gain by this value before writing to hardware.
	 */
	context.ctrlMap[&controls::Brightness] = ControlInfo(-1.0f, 1.0f, 0.0f);
	context.ctrlMap[&controls::Contrast] = ControlInfo(0.0f, 2.0f, 1.0f);

	if (context.configuration.hw.hasCbhs) {
		context.ctrlMap[&controls::Saturation] =
			ControlInfo(0.0f, 2.0f, 1.0f);
		context.ctrlMap[&controls::microchip::Hue] =
			ControlInfo(-180, 180, 0);
	}

	return 0;
}

/**
 * \brief Apply YAML saturation and hue defaults to activeState on configure
 * \param[in] context The IPA context
 * \param[in] sensorInfo Camera sensor information (unused)
 * \return 0 on success
 *
 * Sets the initial saturation and hue from the YAML tuning values so that
 * the first frame uses the tuned defaults rather than the hardcoded 16/0.
 */
int Cbc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	context.activeState.cbc.brightness = defaultBrightness_;
	if (context.configuration.hw.hasCbhs) {
		context.activeState.cbc.saturation = defaultSaturation_;
		context.activeState.cbc.hue = defaultHue_;
	}
	return 0;
}

/**
 * \brief Handle per-frame request controls for CBC
 * \param[in] context The IPA context
 * \param[in] frame The frame number (unused)
 * \param[in] frameContext The frame context (unused)
 * \param[in] controls Application-supplied controls for this frame
 *
 * Handles the Saturation control for SAMA7G5 CBHS hardware. The libcamera
 * Saturation value (float, 1.0 = normal) is converted to the hardware
 * register range using the SAMA7G5 unity value of 16.
 */
void Cbc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	/*
	 * Brightness: libcamera float [-1.0, +1.0] → ISC CBHS int [-1024,+1023]
	 * Conversion: hw = round(v * 1023)  — mirrors rkisp1 cproc.cpp
	 * convertBrightness: v * 128 (8-bit CPROC); we scale to 1023 for ISC.
	 * Stored in activeState; process() writes it to the ISP control list.
	 */
	const auto &brightness = controls.get(controls::Brightness);
	if (brightness.has_value()) {
		context.activeState.cbc.brightness = static_cast<int32_t>(
			std::clamp(brightness.value() * 1023.0f,
				   kBrightnessMin, kBrightnessMax));
	}

	/*
	 * Contrast: libcamera float [0.0, 2.0], neutral 1.0.
	 * Stored as a user multiplier; process() applies it on top of the
	 * adaptive histogram-derived contrast gain. 1.0 = no user adjustment.
	 * Neutral (1.0) does not override the adaptive algorithm — it scales it.
	 */
	const auto &contrast = controls.get(controls::Contrast);
	if (contrast.has_value())
		context.activeState.cbc.userContrastMultiplier =
			std::clamp(contrast.value(), 0.0f, 2.0f);

	/*
	 * Saturation: libcamera float [0.0, 2.0], neutral 1.0.
	 * Converted to ISC CBHS register: hw = v * kSaturationScale (16).
	 */
	const auto &saturation = controls.get(controls::Saturation);
	if (saturation.has_value()) {
		int32_t sat = static_cast<int32_t>(
			std::clamp(saturation.value() * kSaturationScale,
				   0.0f, kSaturationScale * 2.0f));
		context.activeState.cbc.saturation = sat;
	}

	/*
	 * Hue is a Microchip vendor control (controls::microchip::Hue).
	 * Only effective on SAMA7G5 CBHS hardware; ignored on SAMA5D2.
	 */
	const auto &hue = controls.get(controls::microchip::Hue);
	if (hue.has_value())
		context.activeState.cbc.hue =
			std::clamp(hue.value(), -180, 180);
}

/**
 * \brief Process statistics and compute contrast adjustment
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] metadata Control list for frame metadata
 *
 * Applies a static hardware brightness offset (from tuning) and an adaptive
 * contrast adjustment based on the scene histogram spread.
 *
 * Brightness: a fixed signed offset [-1024, 1023] from the tuning file.
 * AGC is the sole owner of the luminance control loop; CBC brightness is a
 * static pipeline trim only. A dynamic feedback loop here would create two
 * competing PI controllers on the same output variable, causing oscillation.
 *
 * Contrast: a multiplier (hardware unity = 16) derived from the P5–P95
 * histogram spread. Flat scenes (fog, overcast) get contrast boosted to
 * fill the target spread fraction. The gain is blended with unity by
 * contrast_strength to tune between subtle and aggressive enhancement.
 *
 * Contrast is temporally smoothed to avoid visible flickering.
 */
void Cbc::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const mchp_isc_stat_buffer *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	auto &cbc = context.activeState.cbc;

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable)
		return;

	/*
	 * Wait for the first complete 4-channel histogram cycle before
	 * adjusting contrast.
	 */
	if (frame < 3) {
		LOG(ISC_CBC, Debug) << "CBC warmup frame " << frame << ", skipping";
		return;
	}

	/*
	 * Brightness is a static tuning offset, not a feedback loop.
	 * AGC owns luminance; CBC brightness only corrects static pipeline
	 * offsets (e.g. sensor pedestal not fully removed by BLC).
	 */
	/*
	 * Use the user-requested brightness from activeState (set by
	 * queueRequest() from controls::Brightness). Falls back to the YAML
	 * default when no user request has been made (activeState initialised
	 * to defaultBrightness_ in configure()).
	 */
	float brightness = static_cast<float>(context.activeState.cbc.brightness);

	/* --- Contrast ---
	 *
	 * Measure the scene's dynamic range via the P5–P95 histogram
	 * spread. If the scene is flat (spread < targetSpread_), scale up
	 * contrast to fill the target fraction of the output range.
	 *
	 * Hardware unity is 16 (output = (contrast/16) * input + brightness).
	 */
	float spread = frameContext.scene.spread;
	float contrastGain = 1.0f;
	float lumNorm = frameContext.scene.luminance / 511.0f;

	/*
	 * Dark-scene guard: below darkSceneMinLuminance_ the AGC is already at
	 * max gain, so a contrast stretch only amplifies noise and clips the
	 * output. Hold contrast at unity and let AGC own luminance. The default
	 * (0.03 ≈ 15 bins) sits just above the AGC floor (8 bins) so the stretch
	 * does not engage as AE reaches its limit.
	 */
	bool darkScene = (lumNorm < darkSceneMinLuminance_);

	if (!darkScene && spread > 0.01f && spread < targetSpread_) {
		/*
		 * Scene does not fill the target spread: stretch contrast.
		 * Clamp to avoid extreme corrections in near-zero spread
		 * scenes (e.g. lens cap on).
		 */
		contrastGain = std::clamp(targetSpread_ / spread, 1.0f, contrastMaxGain_);
	}

	if (darkScene) {
		LOG(ISC_CBC, Debug)
			<< "Dark scene (lum=" << frameContext.scene.luminance
			<< " lumNorm=" << lumNorm
			<< " < " << darkSceneMinLuminance_
			<< ") — contrast stretch suppressed";
	}

	/*
	 * Blend the computed gain with unity (1.0) using contrastStrength_.
	 * At strength=0 contrast stays at unity; at strength=1 the full
	 * histogram-derived gain is applied.
	 */
	/*
	 * Apply user contrast multiplier (from controls::Contrast, default 1.0).
	 * This scales the adaptive gain so:
	 *   user=1.0 → pure adaptive (no change)
	 *   user=1.5 → 50% stronger contrast than adaptive alone
	 *   user=0.5 → 50% weaker contrast
	 * Mirrors rkisp1 cproc.cpp pattern: user value scales the ISP param.
	 */
	float userMul = context.activeState.cbc.userContrastMultiplier;
	float blendedGain = (1.0f + (contrastGain - 1.0f) * contrastStrength_) * userMul;

	/*
	 * Feed the effective contrast multiplier back to the AGC only when
	 * the CBHS block is actually in the active pipeline.  For RGB565
	 * output the ISC pipeline is CFA→WB→CC→GAM — CBHS is absent — so
	 * writing a contrastGain > 1.0 here would cause the AGC to divide
	 * its effectiveTarget by that factor, producing systematic
	 * underexposure with no corresponding hardware boost to compensate.
	 */
	context.activeState.cbc.contrastGainLinear =
		context.configuration.hw.hasCbhs ? blendedGain : 1.0f;

	/* Convert to hardware units: unity = contrastUnityHw (16 on SAMA7G5, 256 on SAMA5D2) */
	float targetContrastHw = blendedGain * context.configuration.hw.contrastUnityHw;

	/* Temporal smoothing */
	float contrast = smoothingFactor_ * static_cast<float>(cbc.contrast) +
			 (1.0f - smoothingFactor_) * targetContrastHw;

	cbc.brightness = static_cast<int32_t>(
		std::clamp(brightness, kBrightnessMin, kBrightnessMax));
	cbc.contrast = static_cast<int32_t>(
		std::clamp(contrast, kContrastMin, kContrastMax));

	/*
	 * The CBHS saturation register is left at its nominal (user/YAML)
	 * value here. CCM-boost compensation is applied once at emit time in
	 * setIspControls(), reading the nominal each frame, so it must not be
	 * folded back into the active state.
	 */

	LOG(ISC_CBC, Debug)
		<< "CBC: brightness=" << cbc.brightness
		<< " spread=" << spread
		<< " contrastGain=" << contrastGain
		<< " contrast=" << cbc.contrast
		<< " saturation=" << cbc.saturation
		<< " (ccmSatBoost=" << context.activeState.ccm.satBoost << ")"
		<< " hue=" << cbc.hue;
}

REGISTER_IPA_ALGORITHM(Cbc, "Cbc")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
