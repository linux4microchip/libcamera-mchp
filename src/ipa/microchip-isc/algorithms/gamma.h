/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Adaptive Gamma Tone Mapping for Microchip ISC
 */

#pragma once

#include <array>
#include <stdint.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Gamma : public Algorithm
{
public:
	Gamma() = default;
	~Gamma() = default;

	unsigned int priority() const override { return 20; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &sensorInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mchp_isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	/* Returns compressionGain: ratio of LUT output to gamma 2.2 output
	 * at the reference entry (entry 8). Used by AGC feedback. */
	static float computeLut(float gammaExp,
				std::array<uint32_t, 64> &lut,
				bool bipartite,
				float toeInput,
				float rolloffStart,
				const std::array<float, 64> *cdf,
				float cdfStrength,
				float rolloffSlopeFactor = 0.35f);

	/*
	 * Tuning parameters — all set by init() from YAML.
	 * In-class initialisers mirror imx219.yaml defaults so that a
	 * missing YAML key never reads uninitialised memory (UB).
	 * Pattern: RPi agc_channel.h ctor init-list, same principle.
	 */

	/* Base gamma exponent (YAML: gamma) */
	float gamma_ = 2.2f;

	/* Adaptive nudge strength [0-1] (YAML: strength) */
	float strength_ = 0.1f;

	/* Target post-gamma normalised luminance [0-1] (YAML: target_output) */
	float targetOutput_ = 0.35f;

	/* Temporal smoothing of effective gamma exponent (YAML: smoothing_factor) */
	float smoothingFactor_ = 0.3f;

	/* Normalised luminance floor below which adaptation is skipped (YAML: dark_scene_threshold) */
	float darkSceneThreshold_ = 0.01f;

	/* Bipartite linear toe breakpoint (YAML: toe_input) — must align to LUT entry */
	float toeInput_ = 0.0313f; /* 32/1023 = entry-2 boundary */

	/* Highlight roll-off start normalised [0.5-1.0], 1.0=disabled (YAML: tone_rolloff_start) */
	float toneRolloffStart_ = 0.85f;
	float rolloffSlopeFactor_ = 0.35f; /* shoulder compression factor */

	/* CDF blend strength [0-1] (YAML: cdf_strength) */
	float cdfStrength_ = 0.05f;
	/* Scene-adaptive CDF boost for flat/over-bright scenes */
	float adaptiveCdfTrigger_  = 0.25f;  // spread below this triggers boost
	float adaptiveCdfMaxBoost_ = 0.50f;  // maximum boost to add
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
