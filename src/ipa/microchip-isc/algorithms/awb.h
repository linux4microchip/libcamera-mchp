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

	/*
	 * Per-channel gain clamp limits. Wider limits allow faster adaptation
	 * but risk oscillation in extreme scenes (snow, candlelight). Set via
	 * YAML gain_limits; defaults match the grey-world practical range.
	 */
	float gainMin_ = 0.2f;
	float gainMax_ = 3.0f;
	float gbGainMin_ = 0.9f;
	float gbGainMax_ = 1.1f;

	/*
	 * Colour temperature limits in Kelvin. Values outside this range
	 * indicate histogram artefacts or extremely unusual illuminants and
	 * are clamped to prevent nonsensical CCM matrix lookups.
	 */
	float ctMin_ = 2000.0f;
	float ctMax_ = 10000.0f;

	/*
	 * Reference (neutral) gains applied on configure() so the first
	 * frames have sensible colour before the AWB loop converges.
	 * Set to the D65 grey-card gains measured on the production sensor.
	 */
	float refRGain_ = 1.0f;
	float refBGain_ = 1.0f;
	float refGrGain_ = 1.0f;
	float refGbGain_ = 1.0f;

	/*
	 * Blend fraction [0.0–1.0] of the highlight-based illuminant estimate
	 * mixed into the grey-world gain computation. Near-saturated pixels
	 * (top 20% intensity) are closer to the true illuminant colour than
	 * the full-frame mean, improving AWB accuracy in high-saturation scenes.
	 * 0.0 = pure grey-world; 0.3 gives a good balance for outdoor shooting.
	 * Tunable via YAML highlight_weight.
	 */
	float highlightWeight_ = 0.3f;

	/*
	 * Set true when the maximum per-channel delta is below
	 * stabilityThreshold_ for the current frame. Written to
	 * controls::AwbLocked in metadata.
	 */
	bool awbStable_ = false;
	/*
	 * AWB re-enters converging state only when the per-channel delta
	 * exceeds this threshold. Set to 2x stabilityThreshold_ in init()
	 * to provide hysteresis against colour flicker near the boundary.
	 */
	float resumeThreshold_ = 0.030f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
