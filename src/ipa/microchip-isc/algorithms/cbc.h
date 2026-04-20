/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Adaptive Contrast/Brightness Control for Microchip ISC
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Cbc : public Algorithm
{
public:
	Cbc() = default;
	~Cbc() = default;

	unsigned int priority() const override { return 30; }

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
	/*
	 * Tuning parameters — ALL set by init() from YAML.
	 * No in-class initialisers for tuning values.
	 */

	/* Static brightness offset after gamma (YAML: default_brightness) */
	int32_t defaultBrightness_ = 0; /* 0 = no static offset */

	/* Contrast enhancement strength [0-1] (YAML: contrast_strength) */
	float contrastStrength_ = 0.4f; /* moderate enhancement */

	/* Target P5-P95 spread as fraction of histogram range (YAML: target_spread) */
	float targetSpread_ = 0.65f; /* 65% of 511-bin range */

	/* Temporal smoothing factor (YAML: smoothing_factor) */
	float smoothingFactor_ = 0.65f; /* prevents CBC brightness flicker */

	/* Default hardware saturation (YAML: saturation) — unity=16, range 0-100 */
	int32_t defaultSaturation_ = 60; /* slightly vivid */

	/* Default hue rotation in degrees (YAML: hue) */
	int32_t defaultHue_ = 0; /* no rotation */

	/* Maximum contrast gain from histogram stretch (YAML: contrast_max_gain) */
	float contrastMaxGain_ = 3.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
