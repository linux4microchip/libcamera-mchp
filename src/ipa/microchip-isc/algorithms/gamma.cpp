/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Adaptive Gamma Tone Mapping for Microchip ISC
 */

#include "gamma.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

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

	gamma_ = std::clamp(gamma_, 0.5f, 5.0f);
	strength_ = std::clamp(strength_, 0.0f, 1.0f);
	targetOutput_ = std::clamp(targetOutput_, 0.1f, 0.9f);
	smoothingFactor_ = std::clamp(smoothingFactor_, 0.0f, 0.95f);

	LOG(ISC_Gamma, Debug)
		<< "Gamma initialized: gamma=" << gamma_
		<< " strength=" << strength_
		<< " targetOutput=" << targetOutput_
		<< " smoothing=" << smoothingFactor_;

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
	context.activeState.gamma.gammaEff = gamma_;
	computeLut(gamma_, context.activeState.gamma.lutB);
	computeLut(gamma_, context.activeState.gamma.lutG);
	computeLut(gamma_, context.activeState.gamma.lutR);
	return 0;
}

/**
 * \brief Compute a 64-entry 10-bit power-law LUT
 * \param[in] gammaExp The gamma exponent (output = input^(1/gammaExp))
 * \param[out] lut 64-element array to fill
 *
 * Each entry i maps input range [i*16 .. (i+1)*16 - 1] to a 10-bit output
 * value.  Entry 0 is always 0 (black point) and the hardware linearly
 * interpolates to 1023 at the end of the last segment.
 */
void Gamma::computeLut(float gammaExp, std::array<uint32_t, 64> &lut)
{
	constexpr float kInputMax = 1023.0f;
	const float invGamma = 1.0f / gammaExp;

	for (unsigned int i = 0; i < 64; i++) {
		float x = static_cast<float>(i * 16) / kInputMax;
		float y = (x > 0.0f) ? std::pow(x, invGamma) : 0.0f;
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
		    [[maybe_unused]] IPAFrameContext &frameContext,
		    const isc_stat_buffer *stats,
		    [[maybe_unused]] ControlList &metadata)
{
	auto &gammaState = context.activeState.gamma;

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable)
		return;

	/* Skip until the first complete 4-channel stats delivery has passed. */
	if (frame < 3 || strength_ == 0.0f)
		return;

	/*
	 * The ISC histogram is in the pre-gamma linear domain (0–511).
	 * Normalise to [0, 1] for the exponent computation.
	 */
	float lum = calculateLuminance(stats);
	float lumNorm = lum / 511.0f;

	if (lumNorm < 0.01f) {
		LOG(ISC_Gamma, Debug) << "Scene too dark, skipping gamma adapt";
		return;
	}

	/*
	 * Compute the gamma exponent that would map the current linear
	 * luminance to targetOutput_:
	 *   targetOutput = lumNorm ^ (1 / gammaAdapt)
	 *   gammaAdapt = log(lumNorm) / log(targetOutput)
	 */
	float gammaAdapt = std::log(lumNorm) / std::log(targetOutput_);
	gammaAdapt = std::clamp(gammaAdapt, 0.5f, 5.0f);

	/* Blend adaptive exponent with the base gamma. */
	float gammaTarget = gamma_ + strength_ * (gammaAdapt - gamma_);

	/* Temporal smoothing. */
	float gammaEff = smoothingFactor_ * gammaState.gammaEff +
			 (1.0f - smoothingFactor_) * gammaTarget;
	gammaState.gammaEff = gammaEff;

	computeLut(gammaEff, gammaState.lutB);
	computeLut(gammaEff, gammaState.lutG);
	computeLut(gammaEff, gammaState.lutR);

	LOG(ISC_Gamma, Debug)
		<< "Gamma: lum=" << lum
		<< " lumNorm=" << lumNorm
		<< " gammaAdapt=" << gammaAdapt
		<< " gammaEff=" << gammaEff;
}

REGISTER_IPA_ALGORITHM(Gamma, "Gamma")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
