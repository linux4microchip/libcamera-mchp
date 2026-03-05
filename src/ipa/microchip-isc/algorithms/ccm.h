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

	/* Temporal smoothing factor */
	float smoothingFactor_ = 0.7f;

	/* Previous CT for adaptive smoothing */
	float lastCt_ = 5500.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
