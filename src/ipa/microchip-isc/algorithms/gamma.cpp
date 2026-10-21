/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Adaptive Gamma Tone Mapping for Microchip ISC
 */

#include "gamma.h"

#include <algorithm>
#include <math.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

/* Shoulder slope compression factor: the gamma derivative at the
 * rolloff threshold is multiplied by this to produce the actual
 * slope of the linear shoulder. Lower values compress more.
 * Exposed as tone_rolloff_slope in the tuning YAML (default 0.35). */
namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_Gamma)

namespace ipa::microchip_isc {

/**
 * \brief Initialize the Gamma algorithm from YAML tuning data
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for Gamma
 * \return 0 on success
 */
int Gamma::init([[maybe_unused]] IPAContext &context,
		const YamlObject &tuningData)
{
	gamma_ = tuningData["gamma"].get<float>(gamma_);
	strength_ = tuningData["strength"].get<float>(strength_);
	targetOutput_ = tuningData["target_output"].get<float>(targetOutput_);
	smoothingFactor_ = tuningData["smoothing_factor"].get<float>(smoothingFactor_);
	darkSceneThreshold_ = tuningData["dark_scene_threshold"].get<float>(darkSceneThreshold_);
	toeInput_ = tuningData["toe_input"].get<float>(toeInput_);

	gamma_ = std::clamp(gamma_, 0.5f, 5.0f);
	strength_ = std::clamp(strength_, 0.0f, 1.0f);
	targetOutput_ = std::clamp(targetOutput_, 0.1f, 0.9f);
	smoothingFactor_ = std::clamp(smoothingFactor_, 0.0f, 0.95f);
	darkSceneThreshold_ = std::clamp(darkSceneThreshold_, 0.0f, 0.1f);
	toeInput_ = std::clamp(toeInput_, 0.01f, 0.15f);
	toneRolloffStart_ = tuningData["tone_rolloff_start"].get<float>(toneRolloffStart_);
	toneRolloffStart_ = std::clamp(toneRolloffStart_, 0.5f, 0.99f);
	rolloffSlopeFactor_ = std::clamp(
		tuningData["tone_rolloff_slope"].get<float>(rolloffSlopeFactor_),
		0.05f, 1.0f);

	return 0;
}

/**
 * \brief Pre-compute the initial LUT from the base gamma exponent
 * \param[in] context The IPA context
 * \param[in] sensorInfo Camera sensor information (unused)
 * \return 0 on success
 */
int Gamma::configure(IPAContext &context,
		     [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	bool bipartite = context.configuration.hw.hasBipartiteGamma;
	context.activeState.gamma.gammaEff = gamma_;
	/* No histogram available at configure time — pure power-law only */
	computeLut(gamma_, context.activeState.gamma.lutB, bipartite,
		   toeInput_, toneRolloffStart_);
	context.activeState.gamma.lutG = context.activeState.gamma.lutB;
	context.activeState.gamma.lutR = context.activeState.gamma.lutB;
	context.activeState.gamma.lutValid = true;

	return 0;
}

/**
 * \brief Compute a 64-entry 10-bit tone-mapping LUT
 * \param[in] gammaExp The gamma exponent (output = input^(1/gammaExp))
 * \param[out] lut 64-element array to fill
 * \param[in] bipartite When true, use a linear toe for better shadow detail
 *
 * Each entry i maps input range [i*16 .. (i+1)*16 - 1] to a 10-bit output
 * value. Entry 0 is always 0 (black point).
 *
 * Standard mode: pure power-law (output = input^(1/gamma)).
 *
 * Bipartite mode (SAMA7G5): a linear segment below toeInput replaces the
 * bottom of the power curve, lifting dark tones and preventing shadow
 * crushing. The linear slope is set so that y(toeInput) is C0 continuous
 * (value matches the power curve at x = toeInput). The derivative is NOT
 * matched (C1 continuity would require a negative offset on the power curve).
 *
 * Each LUT entry spans 16/1023 ≈ 0.01565 normalised input units.  To avoid
 * a visible step at the toe kink, toeInput must be aligned to a LUT entry
 * boundary (i.e. an integer multiple of 16/1023).  At the recommended
 * toeInput = 0.0313 (= 32/1023) the kink falls exactly at entry-2 boundary,
 * the transition is absorbed by quantisation, and no step artefact appears.
 * Misaligned values such as 0.04 (mid-entry 2–3) produce a 34-level step
 * that is visible as a faint shadow gradient discontinuity.
 */
void Gamma::computeLut(float gammaExp, std::array<uint32_t, 64> &lut,
		       bool bipartite, float toeInput, float rolloffStart,
		       float rolloffSlopeFactor)
{
	constexpr float kInputMax = 1023.0f;
	const float invGamma = 1.0f / gammaExp;

	/*
	 * Bipartite toe: linear segment below toeInput.
	 * The slope is chosen so that the linear toe meets the power curve
	 * at x = toeInput, making y(toeInput) continuous.
	 *
	 * slope = pow(toeInput, invGamma) / toeInput
	 */
	const float yAtToe = std::pow(toeInput, invGamma);
	/*
	 * Avoid division by zero: toeInput is clamped to [0.01, 0.15] by
	 * init() for the normal YAML path, but computeLut() is also called
	 * directly from configure() before init() runs.  When toeInput is
	 * zero (bipartite disabled) the toe branch is never entered anyway,
	 * so the slope value is irrelevant; use 1.0 as a safe sentinel.
	 */
	const float toeSlope = (toeInput > 0.0f) ? (yAtToe / toeInput) : 1.0f;

	for (unsigned int i = 0; i < 64; i++) {
		float x = static_cast<float>(i * 16) / kInputMax;
		float y;

		/*
		 * C1-continuous linear shoulder above rolloffStart. The slope
		 * is the gamma derivative at the shoulder, scaled down to
		 * compress highlights. Set tone_rolloff_start: 1.0 to disable.
		 */
		if (rolloffStart < 1.0f && x > rolloffStart) {
			float yAtShoulder = std::pow(rolloffStart, invGamma);
			float slopeAtShoulder = invGamma *
						std::pow(rolloffStart, invGamma - 1.0f);
			float targetSlope = slopeAtShoulder * rolloffSlopeFactor;
			float y_shoulder = yAtShoulder +
					   targetSlope * (x - rolloffStart);
			x = std::pow(std::clamp(y_shoulder, 0.0f, 1.0f),
				     gammaExp);
		}

		if (x == 0.0f) {
			y = 0.0f;
		} else if (bipartite && x < toeInput) {
			/* Linear toe: y = slope * x, matched to power at toeInput */
			y = toeSlope * x;
		} else {
			/* Standard power-law gamma */
			y = std::pow(x, invGamma);
		}

		lut[i] = static_cast<uint32_t>(
			std::clamp(std::round(y * kInputMax), 0.0f, kInputMax));
	}
}

/**
 * \brief Update the gamma LUT from per-frame histogram statistics
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext Per-frame context (unused)
 * \param[in] stats ISC statistics buffer
 * \param[out] metadata Frame metadata (unused)
 *
 * When strength > 0, the effective gamma exponent is nudged each frame to
 * steer the post-gamma mean luminance toward target_output.  The update is
 * smoothed with an exponential moving average to prevent flickering.
 *
 * The resulting LUT is the same for all three channels (B, G, R), relying
 * on AWB and CCM to handle any per-channel colour differences.  Separate
 * per-channel LUTs are stored in the active state to allow a future
 * extension to per-channel tone mapping without API changes.
 */
void Gamma::process(IPAContext &context,
		    const uint32_t frame,
		    IPAFrameContext &frameContext,
		    [[maybe_unused]] const mchp_isc_stat_buffer *stats,
		    [[maybe_unused]] ControlList &metadata)
{
	auto &gammaState = context.activeState.gamma;

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable)
		return;

	/* Skip until the first complete 4-channel stats delivery has passed. */
	if (frame < 3)
		return;

	/*
	 * Nothing to adapt without a gamma nudge — the LUT from configure()
	 * (pure power-law) is used as-is.
	 */
	if (strength_ == 0.0f)
		return;

	/*
	 * Use the raw scene luminance derived from the pre-WB histogram.
	 * The ISC histogram taps pre-WB (post-DPC), so channel means are
	 * raw sensor values independent of the hardware WB gains.
	 */
	float lum = frameContext.scene.luminance;
	float lumNorm = lum / kHistogramMax;

	if (lumNorm > darkSceneThreshold_ && lumNorm > 0.0f && lumNorm < 1.0f) {
		/*
		 * Compute the gamma exponent that maps the current linear
		 * luminance to targetOutput_:
		 *   targetOutput = lumNorm ^ (1 / gammaAdapt)
		 *   gammaAdapt = log(lumNorm) / log(targetOutput)
		 */
		float gammaAdapt = std::log(lumNorm) / std::log(targetOutput_);
		gammaAdapt = std::clamp(gammaAdapt, 0.5f, 5.0f);

		float gammaTarget = gamma_ + strength_ * (gammaAdapt - gamma_);
		float gammaEff = smoothingFactor_ * gammaState.gammaEff +
				 (1.0f - smoothingFactor_) * gammaTarget;
		gammaState.gammaEff = gammaEff;

		LOG(ISC_Gamma, Debug)
			<< "Gamma: lum=" << lum
			<< " lumNorm=" << lumNorm
			<< " gammaAdapt=" << gammaAdapt
			<< " gammaEff=" << gammaEff
			<< " bipartite=" << context.configuration.hw.hasBipartiteGamma;
	} else if (lumNorm <= darkSceneThreshold_) {
		LOG(ISC_Gamma, Debug) << "Scene too dark, skipping gamma adapt";
	}

	bool bipartite = context.configuration.hw.hasBipartiteGamma;

	/*
	 * Compute the new LUT from the updated gammaEff, then blend with the
	 * previous LUT for temporal smoothing.
	 */
	std::array<uint32_t, 64> newLut;
	computeLut(gammaState.gammaEff, newLut, bipartite, toeInput_,
		   toneRolloffStart_, rolloffSlopeFactor_);

	for (unsigned int i = 0; i < 64; i++) {
		float smoothed = smoothingFactor_ * static_cast<float>(gammaState.lutB[i]) +
				 (1.0f - smoothingFactor_) * static_cast<float>(newLut[i]);
		gammaState.lutB[i] = static_cast<uint32_t>(std::round(smoothed));
	}

	/*
	 * Enforce LUT monotonicity. Integer rounding of the smoothed float
	 * sequence can produce local inversions (lut[i] < lut[i-1]).
	 * The ISC hardware requires a non-decreasing LUT — a non-monotone
	 * table causes incorrect tone mapping over the affected range.
	 */
	for (unsigned int i = 1; i < 64; i++)
		gammaState.lutB[i] = std::max(gammaState.lutB[i], gammaState.lutB[i - 1]);

	gammaState.lutG = gammaState.lutB;
	gammaState.lutR = gammaState.lutB;
}

REGISTER_IPA_ALGORITHM(Gamma, "Gamma")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
