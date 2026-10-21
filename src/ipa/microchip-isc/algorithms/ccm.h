/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Color Correction Matrix for Microchip ISC
 */

#pragma once

#include <array>
#include <string>
#include <vector>

#include "algorithm.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Ccm : public Algorithm
{
public:
	Ccm() = default;
	~Ccm() = default;
	unsigned int priority() const override { return 20; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &sensorInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mchp_isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	struct CcmProfile {
		std::string name;
		uint32_t ct;
		std::array<float, 9> matrix;
		std::array<float, 3> offsets;
	};

	void interpolateMatrix(float ct,
			       std::array<float, 9> &matrix,
			       std::array<float, 3> &offsets) const;

	std::vector<CcmProfile> profiles_;

	/* Temporal smoothing factor (YAML: smoothing_factor) */
	float smoothingFactor_ = 0.7f;

	/* Adaptive smoothing: CT delta above which matrix snaps instantly (YAML: ct_snap.snap_threshold) */
	float ctSnapThreshold_ = 1500.0f;
	/* CT delta above which faster-than-normal transition is used (YAML: ct_snap.fast_threshold) */
	float ctFastThreshold_ = 500.0f;
	/* Speed multiplier for fast CT transitions (YAML: ct_snap.fast_factor) */
	float ctFastFactor_ = 0.5f;

	/* Fallback CT before AWB has produced an estimate (YAML: ct_fallback) */
	float ctFallback_ = 5500.0f;

	/*
	 * Chroma scale toward identity (YAML: saturation). Bounds the CCM
	 * off-diagonals so the unsigned ISC CC clamp cannot black out a channel.
	 */
	float saturation_ = 0.6f;

	/* CCM->CBHS saturation coupling coefficient (YAML: sat_boost_factor) */
	float satBoostFactor_ = 0.25f;

	/* ---- Runtime state — NOT tuning params ---- */
	float lastCt_ = 0.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
