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

/**
 * \brief Gamma tone-mapping algorithm for the ISC gamma LUT block
 *
 * Computes a 64-entry 10-bit LUT per channel from a parametric power-law
 * curve.  The exponent is optionally nudged each frame to bring the
 * post-gamma mean luminance toward a configurable target, giving a mild
 * scene-adaptive brightening / darkening that complements AGC.
 *
 * The three per-channel LUTs are written to the kernel via the
 * ISC_CID_GAMMA_{B,G,R}_LUT V4L2 array controls, which were introduced
 * to allow the IPA to bypass the fixed preset curves selected by
 * V4L2_CID_GAMMA.
 */
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
		     const isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	static void computeLut(float gammaExp, std::array<uint32_t, 64> &lut,
			       bool bipartite = false, float toeInput = 0.04f,
			       float rolloffStart = 1.0f,
			       const std::array<float, 64> *cdf = nullptr,
			       float cdfStrength = 0.0f);

	/* Base gamma exponent (e.g. 2.2 for sRGB-like encoding). */
	float gamma_ = 2.2f;

	/*
	 * Adaptive strength [0.0–1.0].  At 0 the curve is purely parametric;
	 * at 1 the exponent is fully adjusted to hit target_output each frame.
	 */
	float strength_ = 0.3f;

	/*
	 * Target post-gamma normalised mean luminance [0.0–1.0].
	 * 0.45 ≈ mid-grey on a display with gamma 2.2 applied to 18% grey.
	 */
	float targetOutput_ = 0.45f;

	/* Temporal smoothing of the effective gamma [0.0–1.0]. */
	float smoothingFactor_ = 0.3f;

	/*
	 * Normalised input luminance floor below which adaptive gamma update
	 * is skipped. Near-black scenes produce unreliable luminance means
	 * that would cause runaway gamma adaptation. Tunable via YAML
	 * dark_scene_threshold (range 0.0–0.1, typical 0.01).
	 */
	float darkSceneThreshold_ = 0.01f;

	/*
	 * Normalised input breakpoint for the bipartite (linear toe) segment.
	 * Below this fraction of full-scale the LUT uses a linear slope
	 * matched to the power curve, lifting dark tones. Only active when
	 * hasBipartiteGamma is true. Tunable via YAML toe_input (range
	 * 0.01–0.15, typical 0.04).
	 */
	float toeInput_ = 0.04f;

	/*
	 * Histogram CDF blend strength [0.0–1.0].
	 * At 0.0 the LUT is a pure power-law (current behaviour).
	 * At 1.0 full histogram equalisation is applied: each LUT entry maps
	 * its input quantile directly to output, producing maximum local
	 * contrast regardless of scene distribution.
	 * Intermediate values blend the two, giving locally-adaptive contrast
	 * enhancement that scales with scene flatness. Temporal LUT smoothing
	 * prevents flicker from frame-to-frame histogram variation.
	 * Tunable via YAML cdf_strength (typical 0.25–0.4).
	 */
	float cdfStrength_ = 0.0f;

	/*
	 * Normalised output level above which highlight roll-off is applied.
	 * Above this point the tone curve is compressed exponentially so
	 * near-saturated values never reach pure white, preventing burned-out
	 * highlights in bright outdoor scenes. 1.0 = disabled (no roll-off).
	 * Tunable via YAML tone_rolloff_start (range 0.5–1.0, typical 0.85).
	 */
	float toneRolloffStart_ = 1.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
