/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Color Correction Matrix for Microchip ISC
 */

#include "ccm.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_CCM)

namespace ipa::microchip_isc {

/**
 * \brief Initialize CCM from YAML tuning data
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for CCM
 * \return 0 on success
 *
 * Parses per-CT matrix profiles from the tuning file. Each profile
 * contains a color temperature, a 3x3 matrix, and optional offsets.
 * Profiles are sorted by descending CT (ascending reciprocal megakelvin,
 * MRD) for correct interpolation in MRD space.
 */
int Ccm::init([[maybe_unused]] IPAContext &context,
	      const YamlObject &tuningData)
{
	if (!tuningData.contains("matrices"))
		return 0;

	const YamlObject &matrices = tuningData["matrices"];
	for (const auto &[name, entry] : matrices.asDict()) {
		auto ct = entry["ct"].get<uint32_t>();
		auto ccmList = entry["ccm"].getList<float>();

		if (!ct || !ccmList || ccmList->size() != 9) {
			LOG(ISC_CCM, Warning)
				<< "Skipping invalid CCM entry: " << name;
			continue;
		}

		CcmProfile profile;
		profile.name = name;
		profile.ct = *ct;
		std::copy(ccmList->begin(), ccmList->end(),
			  profile.matrix.begin());

		auto offList = entry["offsets"].getList<float>();
		if (offList && offList->size() == 3)
			std::copy(offList->begin(), offList->end(),
				  profile.offsets.begin());
		else
			profile.offsets = { 0.0f, 0.0f, 0.0f };

		profiles_.push_back(std::move(profile));

		LOG(ISC_CCM, Debug)
			<< "CCM profile: " << name << " ct=" << *ct;
	}

	/*
	 * Sort profiles by descending CT (= ascending MRD = 1e6/CT).
	 * Interpolation is performed in reciprocal megakelvin (MRD) space
	 * because CT is inversely proportional to wavelength: the perceptual
	 * difference between 3000 K and 4000 K is far larger than between
	 * 5500 K and 6500 K despite equal Kelvin spacing. Linear interpolation
	 * in MRD space correctly reflects those perceptual differences.
	 */
	std::sort(profiles_.begin(), profiles_.end(),
		  [](const CcmProfile &a, const CcmProfile &b) {
			  return a.ct > b.ct; /* descending CT = ascending MRD */
		  });

	LOG(ISC_CCM, Debug)
		<< "CCM initialized with " << profiles_.size() << " profiles";

	return 0;
}

/**
 * \brief Reset CCM state on (re-)configure
 * \param[in] context The IPA context (unused)
 * \param[in] sensorInfo Camera sensor information (unused)
 * \return 0 on success
 *
 * Resets lastCt_ so the first frame after configure() does not trigger a
 * spurious "large scene change" snap.
 */
int Ccm::configure([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	lastCt_ = 0.0f;
	return 0;
}

/**
 * \brief Interpolate a CCM for a given color temperature using MRD space
 * \param[in] ct Target color temperature in Kelvin
 * \param[out] matrix Interpolated 3x3 matrix
 * \param[out] offsets Interpolated RGB offsets
 *
 * Linearly interpolates between the two nearest calibration profiles in
 * reciprocal megakelvin (MRD = 1e6/CT) space. Profiles are sorted descending
 * CT (ascending MRD) so profiles_.front() is the warmest (highest CT, lowest
 * MRD) and profiles_.back() is the coolest (lowest CT, highest MRD).
 *
 * MRD interpolation produces perceptually uniform colour matrix transitions:
 * the Planckian locus has nearly uniform spacing in MRD, so equal MRD steps
 * produce equal chromaticity steps, unlike Kelvin where equal steps produce
 * large chromaticity changes at low CT and small changes at high CT.
 */
void Ccm::interpolateMatrix(float ct,
			    std::array<float, 9> &matrix,
			    std::array<float, 3> &offsets) const
{
	/* Identity fallback when no profiles are loaded */
	if (profiles_.empty()) {
		matrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		offsets = { 0.0f, 0.0f, 0.0f };
		return;
	}

	/* Convert query CT to MRD; clamp to avoid division by zero */
	float mrd = (ct > 1.0f) ? 1.0e6f / ct : 1.0e6f;

	/*
	 * Profiles are sorted descending CT (ascending MRD).
	 * front() = warmest (highest CT, lowest MRD).
	 * back()  = coolest (lowest CT, highest MRD).
	 */
	float mrdFront = 1.0e6f / static_cast<float>(profiles_.front().ct);
	float mrdBack = 1.0e6f / static_cast<float>(profiles_.back().ct);

	/* At or below the warmest profile (query CT is high / MRD is low) */
	if (mrd <= mrdFront) {
		matrix = profiles_.front().matrix;
		offsets = profiles_.front().offsets;
		return;
	}

	/* At or above the coolest profile (query CT is low / MRD is high) */
	if (mrd >= mrdBack) {
		matrix = profiles_.back().matrix;
		offsets = profiles_.back().offsets;
		return;
	}

	/* Find bracketing profiles in MRD space and interpolate */
	for (size_t i = 0; i < profiles_.size() - 1; i++) {
		float mrdA = 1.0e6f / static_cast<float>(profiles_[i].ct);
		float mrdB = 1.0e6f / static_cast<float>(profiles_[i + 1].ct);
		/* mrdA < mrdB because profiles sorted descending CT */
		if (mrd >= mrdA && mrd <= mrdB) {
			float range = mrdB - mrdA;
			if (range < 1e-6f) {
				matrix = profiles_[i].matrix;
				offsets = profiles_[i].offsets;
				return;
			}
			float t = (mrd - mrdA) / range;

			for (size_t j = 0; j < 9; j++)
				matrix[j] = (1.0f - t) * profiles_[i].matrix[j] +
					    t * profiles_[i + 1].matrix[j];
			for (size_t j = 0; j < 3; j++)
				offsets[j] = (1.0f - t) * profiles_[i].offsets[j] +
					     t * profiles_[i + 1].offsets[j];
			return;
		}
	}

	/* Unreachable with correctly sorted profiles, but safe fallback */
	matrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	offsets = { 0.0f, 0.0f, 0.0f };
}

/**
 * \brief Process statistics and compute color correction matrix
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] metadata Control list for frame metadata
 *
 * Reads the scene colour temperature from AWB (single source of truth),
 * interpolates the CCM in MRD space, and applies adaptive temporal smoothing.
 * CCM smoothing speed is governed purely by CT rate of change (snap/fast
 * thresholds) — it does not depend on the AWB convergence flag to avoid
 * a one-frame timing mismatch where CCM switches to slow mode on the same
 * frame AWB declares stability while CT is still drifting from smoothing
 * momentum.
 */
void Ccm::process(IPAContext &context,
		  const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  [[maybe_unused]] const isc_stat_buffer *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	/*
	 * Use the colour temperature computed by AWB from gain-undone channel
	 * means. AWB is the single authority for CT estimation; reading it
	 * here ensures both AWB gains and the CCM matrix track the same
	 * illuminant. Fall back to 5500 K (neutral daylight) on the first
	 * frame before AWB has produced an estimate.
	 */
	unsigned int rawCt = context.activeState.awb.ct;
	float ct = (rawCt > 0) ? static_cast<float>(rawCt) : 5500.0f;

	std::array<float, 9> matrix;
	std::array<float, 3> offsets;
	interpolateMatrix(ct, matrix, offsets);

	/*
	 * Adaptive temporal smoothing based on CT rate of change.
	 * Large CT jumps indicate scene changes and warrant immediate
	 * matrix snap (sf=0). Moderate changes use a faster-than-normal
	 * factor to track the changing illuminant estimate without lag.
	 *
	 * The AWB convergence flag is intentionally not used here: it
	 * introduces a cross-module timing dependency where CCM's mode
	 * changes on the same frame AWB declares stability while CT is
	 * still drifting, causing visible colour casts on transitions.
	 */
	auto &ccm = context.activeState.ccm;
	if (frame > 0) {
		float ctDelta = std::abs(ct - lastCt_);
		float sf = smoothingFactor_;
		if (ctDelta > 1500.0f)
			sf = 0.0f;         /* Snap immediately on scene change */
		else if (ctDelta > 500.0f)
			sf = smoothingFactor_ * 0.5f;  /* Faster for moderate change */

		for (size_t i = 0; i < 9; i++)
			matrix[i] = sf * ccm.matrix[i] + (1.0f - sf) * matrix[i];
		for (size_t i = 0; i < 3; i++)
			offsets[i] = sf * ccm.offsets[i] + (1.0f - sf) * offsets[i];
	}

	lastCt_ = ct;
	ccm.matrix = matrix;
	ccm.offsets = offsets;

	LOG(ISC_CCM, Debug)
		<< "CCM: ct=" << ct
		<< " diag=[" << matrix[0] << ", " << matrix[4] << ", " << matrix[8] << "]";
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
