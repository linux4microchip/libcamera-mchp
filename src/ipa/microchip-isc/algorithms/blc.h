/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Microchip Technology Inc.
 *
 * Black Level Correction for Microchip ISC
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::microchip_isc {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection() = default;
	~BlackLevelCorrection() = default;

	unsigned int priority() const override { return 0; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &sensorInfo) override;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
