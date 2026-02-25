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
int Cbc::init([[maybe_unused]] IPAContext &context,
	      const YamlObject &tuningData)
{
	targetLuminance_ = tuningData["target_luminance"].get<float>(targetLuminance_);
	brightnessGain_ = tuningData["brightness_gain"].get<float>(brightnessGain_);
	contrastStrength_ = tuningData["contrast_strength"].get<float>(contrastStrength_);
	targetSpread_ = tuningData["target_spread"].get<float>(targetSpread_);
	smoothingFactor_ = tuningData["smoothing_factor"].get<float>(smoothingFactor_);

	targetLuminance_ = std::clamp(targetLuminance_, 0.0f, 511.0f);
	brightnessGain_ = std::clamp(brightnessGain_, 0.0f, 0.5f);
	contrastStrength_ = std::clamp(contrastStrength_, 0.0f, 1.0f);
	targetSpread_ = std::clamp(targetSpread_, 0.1f, 1.0f);
	smoothingFactor_ = std::clamp(smoothingFactor_, 0.0f, 0.95f);

	LOG(ISC_CBC, Debug)
		<< "CBC initialized: targetLum=" << targetLuminance_
		<< " brightnessGain=" << brightnessGain_
		<< " contrastStrength=" << contrastStrength_
		<< " targetSpread=" << targetSpread_
		<< " smoothing=" << smoothingFactor_;

	return 0;
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
	targetBrightnessHw = std::clamp(targetBrightnessHw, -256.0f, 256.0f);

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
		contrastGain = std::clamp(targetSpread_ / spread, 1.0f, 3.0f);
	}

	/*
	 * Blend the computed gain with unity (1.0) using contrastStrength_.
	 * At strength=0 contrast stays at unity; at strength=1 the full
	 * histogram-derived gain is applied.
	 */
	float blendedGain = 1.0f + (contrastGain - 1.0f) * contrastStrength_;

	/* Convert to hardware units: unity = 16 */
	float targetContrastHw = blendedGain * 16.0f;

	/* Temporal smoothing */
	float contrast = smoothingFactor_ * static_cast<float>(cbc.contrast) +
			 (1.0f - smoothingFactor_) * targetContrastHw;

	cbc.brightness = static_cast<int32_t>(
		std::clamp(brightness, -1024.0f, 1023.0f));
	cbc.contrast = static_cast<int32_t>(
		std::clamp(contrast, -2048.0f, 2047.0f));

	LOG(ISC_CBC, Debug)
		<< "CBC: lum=" << lum
		<< " lumError=" << lumError
		<< " brightness=" << cbc.brightness
		<< " spread=" << spread
		<< " contrastGain=" << contrastGain
		<< " contrast=" << cbc.contrast;
}

REGISTER_IPA_ALGORITHM(Cbc, "Cbc")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
