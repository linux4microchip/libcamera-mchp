/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto White Balance for Microchip ISC
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Awb : public Algorithm
{
public:
	Awb() = default;
	~Awb() = default;
	unsigned int priority() const override { return 10; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	/* Tuning parameters */
	float smoothingFactor_ = 0.2f;
	float damping_ = 1.0f;
	float stabilityThreshold_ = 0.015f;
	unsigned int convergenceFrames_ = 0;

	/*
	 * Minimum scene luminance (in histogram units, 0–511) below which
	 * AWB updates are suppressed. Near-dark histogram data is unreliable
	 * due to ISC 8-bit quantization and causes erratic gain swings.
	 */
	float darkSceneThreshold_ = 16.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
