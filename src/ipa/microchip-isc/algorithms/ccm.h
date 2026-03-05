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
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const isc_stat_buffer *stats,
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

	/*
	 * Adaptive smoothing thresholds for CT-change-based snap.
	 * When |ctDelta| > ctSnapThreshold_ the matrix snaps immediately
	 * (sf=0); when > ctFastThreshold_ the speed is multiplied by
	 * ctFastFactor_ for a faster-than-normal transition.
	 * Tunable via YAML ct_snap section.
	 */
	float ctSnapThreshold_ = 1500.0f;
	float ctFastThreshold_ = 500.0f;
	float ctFastFactor_ = 0.5f;

	/*
	 * Fallback colour temperature (Kelvin) used on the first frame before
	 * AWB has produced a CT estimate (activeState.awb.ct == 0).
	 * Tunable via YAML ct_fallback (range 2500–10000, typical 5500 K).
	 */
	float ctFallback_ = 5500.0f;

	/* Previous CT for adaptive smoothing */
	float lastCt_ = 0.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
