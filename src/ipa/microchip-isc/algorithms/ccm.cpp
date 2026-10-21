/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Microchip Technology Inc.
 *
 * Color Correction Matrix for Microchip ISC
 */

#include "ccm.h"

#include <algorithm>
#include <math.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

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
int Ccm::init(IPAContext &context,
	      const YamlObject &tuningData)
{
	/*
	 * Advertise manual CCM (controls::ColourCorrectionMatrix) so an
	 * application can override the CT-interpolated matrix. Registered
	 * before the early return so the control is available even when the
	 * tuning file carries no calibrated matrices. Range mirrors rkisp1's
	 * CCM control; the default is the identity matrix.
	 */
	static const std::array<float, 9> kIdentityCcm = {
		1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
	};
	context.ctrlMap[&controls::ColourCorrectionMatrix] = ControlInfo(
		ControlValue(-8.0f), ControlValue(7.993f),
		ControlValue(Span<const float, 9>(kIdentityCcm)));

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

	smoothingFactor_ = tuningData["smoothing_factor"].get<float>(smoothingFactor_);
	ctFallback_ = tuningData["ct_fallback"].get<float>(ctFallback_);
	saturation_ = tuningData["saturation"].get<float>(saturation_);
	satBoostFactor_ = tuningData["sat_boost_factor"].get<float>(satBoostFactor_);
	smoothingFactor_ = std::clamp(smoothingFactor_, 0.0f, 0.99f);
	ctFallback_ = std::clamp(ctFallback_, 2500.0f, 10000.0f);
	saturation_ = std::clamp(saturation_, 0.0f, 1.0f);
	satBoostFactor_ = std::clamp(satBoostFactor_, 0.0f, 2.0f);

	if (tuningData.contains("ct_snap")) {
		const YamlObject &snap = tuningData["ct_snap"];
		ctSnapThreshold_ = snap["snap_threshold"].get<float>(ctSnapThreshold_);
		ctFastThreshold_ = snap["fast_threshold"].get<float>(ctFastThreshold_);
		ctFastFactor_ = snap["fast_factor"].get<float>(ctFastFactor_);
	}

	ctSnapThreshold_ = std::clamp(ctSnapThreshold_, 100.0f, 5000.0f);
	ctFastThreshold_ = std::clamp(ctFastThreshold_, 50.0f, ctSnapThreshold_);
	ctFastFactor_ = std::clamp(ctFastFactor_, 0.0f, 1.0f);

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
int Ccm::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	/*
	 * Initialize lastCt_ to the AWB's current CT estimate (or the fallback
	 * if AWB hasn't run yet). Using 0.0 would cause |ct - 0| = ct ≈ 5500 K
	 * on the first frame, triggering the ctSnapThreshold_ path and snapping
	 * the CCM matrix unnecessarily even when the scene hasn't changed.
	 */
	unsigned int awbCt = context.activeState.awb.ct;
	lastCt_ = (awbCt > 0) ? static_cast<float>(awbCt) : ctFallback_;
	/* Initialise satBoost to unity so CBC does not over-correct on first frame */
	context.activeState.ccm.satBoost = 1.0f;
	return 0;
}

/**
 * \brief Latch a per-request manual colour-correction matrix
 * \param[in] context The IPA context (unused)
 * \param[in] frame The frame number (unused)
 * \param[out] frameContext The frame context for this request
 * \param[in] controls The controls attached to the request
 *
 * Reads controls::ColourCorrectionMatrix and records it in the frame context.
 * When present it takes precedence over the CT-interpolated matrix in
 * process(); when absent the automatic path runs. The override is stored
 * per-frame so it never leaks into a later request that omits the control.
 */
void Ccm::queueRequest([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	frameContext.ccm.manualSet = false;

	const auto &ccmMatrix = controls.get(controls::ColourCorrectionMatrix);
	if (ccmMatrix) {
		std::copy(ccmMatrix->begin(), ccmMatrix->end(),
			  frameContext.ccm.matrix.begin());
		frameContext.ccm.manualSet = true;
		LOG(ISC_CCM, Debug) << "Manual CCM latched from control";
	}
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
		/* Guard: profiles_[i].ct == 0 would divide by zero; init()
		 * filters ct=0 entries so this is defensive only. */
		if (!profiles_[i].ct || !profiles_[i + 1].ct)
			continue;
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
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const mchp_isc_stat_buffer *stats,
		  [[maybe_unused]] ControlList &metadata)
{
	auto &ccm = context.activeState.ccm;
	std::array<float, 9> matrix;
	std::array<float, 3> offsets;

	/*
	 * Colour temperature computed by AWB from the raw pre-WB channel means.
	 * AWB is the single authority for CT estimation, so both the AWB gains
	 * and the interpolated CCM track the same illuminant. Falls back to
	 * 5500 K (neutral daylight) before AWB has produced an estimate.
	 */
	unsigned int rawCt = context.activeState.awb.ct;
	float ct = (rawCt > 0) ? static_cast<float>(rawCt) : ctFallback_;

	if (frameContext.ccm.manualSet) {
		/*
		 * Explicit override takes precedence over CT interpolation. It
		 * is applied directly with no temporal smoothing so the
		 * requested matrix takes effect immediately; offsets are zeroed
		 * because the control carries only the 3×3 matrix. lastCt_ is
		 * kept in step with the AWB estimate so the automatic path
		 * resumes smoothly if the override is dropped.
		 */
		matrix = frameContext.ccm.matrix;
		offsets = { 0.0f, 0.0f, 0.0f };
		lastCt_ = ct;
	} else {
		interpolateMatrix(ct, matrix, offsets);

		/*
		 * Scale chroma toward identity: I + s*(M - I) shrinks the
		 * off-diagonals but leaves row sums at unity, so white is
		 * preserved and no channel underflows the CC clamp.
		 */
		for (size_t i = 0; i < 9; i++) {
			float identity = (i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f;
			matrix[i] = identity + saturation_ * (matrix[i] - identity);
		}

		/*
		 * Adaptive temporal smoothing based on CT rate of change.
		 * Large CT jumps indicate scene changes and warrant immediate
		 * matrix snap (sf=0). Moderate changes or AWB still converging
		 * use a faster-than-normal factor to track the changing
		 * illuminant estimate in sync with AWB gains, preventing a
		 * transient mismatch where new AWB gains are applied with a
		 * lagging colour matrix.
		 *
		 * The CT-delta guard takes priority: CCM speed is driven by how
		 * fast CT is changing, not solely by the AWB stable flag, so
		 * the matrix does not unnecessarily slam on the same frame AWB
		 * declares stable while CT has already settled for multiple
		 * frames.
		 */
		if (frame > 0) {
			float ctDelta = std::abs(ct - lastCt_);
			float sf = smoothingFactor_;
			if (ctDelta > ctSnapThreshold_) {
				/* Large scene change: snap immediately */
				sf = 0.0f;
			} else if (ctDelta > ctFastThreshold_ ||
				   !context.activeState.awb.awbConverged) {
				/*
				 * Moderate CT change or AWB still converging:
				 * use faster adaptation so the colour matrix
				 * tracks the changing illuminant estimate in
				 * sync with the AWB gains. Prevents a transient
				 * mismatch where new gains are applied with an
				 * old matrix during scene changes.
				 */
				sf = smoothingFactor_ * ctFastFactor_;
			}

			for (size_t i = 0; i < 9; i++)
				matrix[i] = sf * ccm.matrix[i] +
					    (1.0f - sf) * matrix[i];
			for (size_t i = 0; i < 3; i++)
				offsets[i] = sf * ccm.offsets[i] +
					     (1.0f - sf) * offsets[i];
		}

		lastCt_ = ct;
	}

	ccm.matrix = matrix;
	ccm.offsets = offsets;

	/*
	 * Estimate the saturation boost from the matrix off-diagonals:
	 *   satBoost = 1 + k * Σ|off_diagonals| / trace   (k = sat_boost_factor)
	 * CBC reads this to scale down the CBHS saturation register so the
	 * combined CCM × CBHS saturation stays near the nominal value.
	 */
	{
		float trace = matrix[0] + matrix[4] + matrix[8];
		float offDiagSum = 0.0f;
		for (size_t i = 0; i < 9; i++) {
			if (i != 0 && i != 4 && i != 8)
				offDiagSum += std::abs(matrix[i]);
		}
		/*
		 * Guard against near-zero trace. A well-formed energy-preserving
		 * CCM has trace ≈ 3. A pathological YAML (e.g. all-zero matrix)
		 * gives trace=0 → division by zero. A very small trace (e.g.
		 * trace=0.01 from a scale-only matrix) gives a huge satBoost
		 * (~hundreds) which would drive CBHS saturation to near-zero —
		 * visually identical to a full-desaturate. Guard with a generous
		 * floor (0.5 = 1/6 of nominal 3.0) and fall back to no boost.
		 */
		ccm.satBoost = (trace >= 0.5f)
				       ? 1.0f + satBoostFactor_ * offDiagSum / trace
				       : 1.0f;
	}

	LOG(ISC_CCM, Debug)
		<< "CCM: ct=" << ct
		<< " diag=[" << matrix[0] << ", " << matrix[4] << ", " << matrix[8] << "]"
		<< " satBoost=" << ccm.satBoost;
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
