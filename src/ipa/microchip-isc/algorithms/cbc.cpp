/* SPDX-License-Identifier: LGPL-2.1-or-later */

/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Adaptive Contrast/Brightness Control for Microchip ISC
 */

#include "cbc.h"

#include <algorithm>
#include <cmath>

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
	targetLuminance_ = tuningData["target_luminance"].get<float>(targetLuminance_);
	brightnessGain_ = tuningData["brightness_gain"].get<float>(brightnessGain_);
	contrastStrength_ = tuningData["contrast_strength"].get<float>(contrastStrength_);
	targetSpread_ = tuningData["target_spread"].get<float>(targetSpread_);
	smoothingFactor_ = tuningData["smoothing_factor"].get<float>(smoothingFactor_);
	defaultSaturation_ = tuningData["saturation"].get<int32_t>(defaultSaturation_);
	defaultHue_ = tuningData["hue"].get<int32_t>(defaultHue_);
	contrastMaxGain_ = tuningData["contrast_max_gain"].get<float>(contrastMaxGain_);

	targetLuminance_ = std::clamp(targetLuminance_, 0.0f, kHistogramMax);
	brightnessGain_ = std::clamp(brightnessGain_, 0.0f, 0.5f);
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
	if (context.configuration.hw.hasCbhs) {
		context.ctrlMap[&controls::Saturation] =
			ControlInfo(0.0f, 2.0f, 1.0f);
		context.ctrlMap[&controls::microchip::Hue] =
			ControlInfo(-180, 180, 0);
	}

	LOG(ISC_CBC, Debug)
		<< "CBC initialized: targetLum=" << targetLuminance_
		<< " brightnessGain=" << brightnessGain_
		<< " contrastStrength=" << contrastStrength_
		<< " targetSpread=" << targetSpread_
		<< " smoothing=" << smoothingFactor_
		<< " saturation=" << defaultSaturation_
		<< " hue=" << defaultHue_;

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
	const auto &saturation = controls.get(controls::Saturation);
	if (saturation.has_value()) {
		/* Convert libcamera scale (1.0=normal) to hardware (16=unity). */
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
 * \brief Process statistics and compute adaptive brightness and contrast
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] metadata Control list for frame metadata
 *
 * Computes two complementary adjustments applied by the ISC CBC hardware
 * block (which sits after gamma in the pipeline):
 *
 * Brightness: a signed offset [-1024, 1023] applied as a fine trim on top
 * of AGC. Only a small fraction of the residual luminance error is corrected
 * here to avoid fighting the AGC control loop.
 *
 * Contrast: a multiplier with hardware unity at 16, range [-2048, 2047] on
 * SAMA7G5. The P5–P95 histogram spread is used to detect flat scenes (fog,
 * overcast) where expanding contrast improves perceived detail. The gain is
 * blended with unity by the contrast_strength parameter so that the effect
 * can be tuned between subtle and aggressive.
 *
 * Both values are temporally smoothed to avoid visible flickering.
 *
 * The ISC histogram collects one Bayer channel per frame; a complete
 * 4-channel set requires 4 frames. CBC computation is skipped during the
 * initial warmup period.
 */
void Cbc::process(IPAContext &context,
		  const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  const isc_stat_buffer *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	auto &cbc = context.activeState.cbc;

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable)
		return;

	/*
	 * Wait for the first complete 4-channel histogram cycle before
	 * adjusting brightness or contrast.
	 */
	if (frame < 4) {
		LOG(ISC_CBC, Debug) << "CBC warmup frame " << frame << ", skipping";
		return;
	}

	/* --- Brightness ---
	 *
	 * Compute a small correction for residual brightness error not
	 * fully handled by AGC. The hardware brightness register adds a
	 * signed offset to the pixel value after gamma.
	 *
	 * The ISC histogram operates in the pre-gamma linear domain
	 * (0–511). The luminance target is expressed in the same space.
	 */
	float lum = calculateLuminance(stats);
	float lumError = targetLuminance_ - lum;
	float targetBrightnessHw = lumError * brightnessGain_;

	/* Clamp to a conservative sub-range to avoid overdriving */
	targetBrightnessHw = std::clamp(targetBrightnessHw,
					-kBrightnessDriveLimit, kBrightnessDriveLimit);

	/* Temporal smoothing */
	float brightness = smoothingFactor_ * static_cast<float>(cbc.brightness) +
			   (1.0f - smoothingFactor_) * targetBrightnessHw;

	/* --- Contrast ---
	 *
	 * Measure the scene's dynamic range via the P5–P95 histogram
	 * spread. If the scene is flat (spread < targetSpread_), scale up
	 * contrast to fill the target fraction of the output range.
	 *
	 * Hardware unity is 16 (output = (contrast/16) * input + brightness).
	 */
	float spread = calculateHistogramSpread(stats);
	float contrastGain = 1.0f;

	if (spread > 0.01f && spread < targetSpread_) {
		/*
		 * Scene does not fill the target spread: stretch contrast.
		 * Clamp to avoid extreme corrections in near-zero spread
		 * scenes (e.g. lens cap on).
		 */
		contrastGain = std::clamp(targetSpread_ / spread, 1.0f, contrastMaxGain_);
	}

	/*
	 * Blend the computed gain with unity (1.0) using contrastStrength_.
	 * At strength=0 contrast stays at unity; at strength=1 the full
	 * histogram-derived gain is applied.
	 */
	float blendedGain = 1.0f + (contrastGain - 1.0f) * contrastStrength_;

	/* Convert to hardware units: unity = contrastUnityHw (16 on SAMA7G5, 256 on SAMA5D2) */
	float targetContrastHw = blendedGain * context.configuration.hw.contrastUnityHw;

	/* Temporal smoothing */
	float contrast = smoothingFactor_ * static_cast<float>(cbc.contrast) +
			 (1.0f - smoothingFactor_) * targetContrastHw;

	cbc.brightness = static_cast<int32_t>(
		std::clamp(brightness, kBrightnessMin, kBrightnessMax));
	cbc.contrast = static_cast<int32_t>(
		std::clamp(contrast, kContrastMin, kContrastMax));

	LOG(ISC_CBC, Debug)
		<< "CBC: lum=" << lum
		<< " lumError=" << lumError
		<< " brightness=" << cbc.brightness
		<< " spread=" << spread
		<< " contrastGain=" << contrastGain
		<< " contrast=" << cbc.contrast
		<< " saturation=" << cbc.saturation
		<< " hue=" << cbc.hue;
}

REGISTER_IPA_ALGORITHM(Cbc, "Cbc")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
