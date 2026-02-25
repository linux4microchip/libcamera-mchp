/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto Gain/Exposure Control for Microchip ISC
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Agc : public Algorithm
{
public:
	Agc() = default;
	~Agc() = default;
	unsigned int priority() const override { return 10; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	/* Tuning parameters */
	float targetBrightness_ = 128.0f;
	float convergenceSpeed_ = 0.3f;
	float stabilityThreshold_ = 10.0f;
	unsigned int convergenceFrames_ = 0;

	/*
	 * Fraction of clipped pixels that triggers highlight anti-windup.
	 * Above this threshold the control loop is prevented from increasing
	 * exposure even when the luminance mean appears low (which occurs in
	 * high-contrast outdoor scenes with large dark areas).
	 */
	float clipThreshold_ = 0.02f;

	/*
	 * Exposure ratio clamp limits and highlight anti-windup cap.
	 * ratioMin/Max bound the per-frame step size to prevent large
	 * oscillations on scene changes. clipRatioCap is applied when
	 * clipping exceeds clipThreshold_ to force a reduction.
	 * All three are tunable via YAML ratio_clamp section.
	 */
	float ratioMin_ = 0.25f;
	float ratioMax_ = 2.5f;
	float clipRatioCap_ = 0.85f;

	/*
	 * Gain update thresholds. Gain is boosted only when exposure is maxed
	 * AND ratio > gainBoostThreshold_; gain is pushed toward minimum when
	 * ratio < gainReductionThreshold_. Tunable via YAML gain_thresholds.
	 */
	float gainBoostThreshold_ = 1.05f;
	float gainReductionThreshold_ = 0.95f;

	/*
	 * Minimum luminance (histogram units) for AE stability check.
	 * Below this floor the scene is treated as unstable regardless of
	 * how close the mean is to targetBrightness_. Prevents false
	 * convergence in near-black scenes where histogram noise dominates.
	 * Tunable via YAML ae_luminance_floor.
	 */
	float aeLuminanceFloor_ = 0.5f;

	/*
	 * Mains flicker avoidance: round exposure to the nearest multiple of
	 * the mains half-period to prevent banding under 50 Hz / 60 Hz lighting.
	 * 0 = disabled (default, suitable for outdoor or DC-lit environments).
	 * Set to 50 or 60 via YAML flicker_hz to match local mains frequency.
	 */
	float flickerHz_ = 0.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
