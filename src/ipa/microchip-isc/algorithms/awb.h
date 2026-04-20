/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto White Balance for Microchip ISC
 */

#pragma once

#include <utility>
#include <vector>

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
	/*
	 * Tuning parameters — populated by init() from YAML.
	 * In-class initializers provide safe defaults if a YAML section is
	 * absent. The get<T>(member_) pattern in init() then overrides with
	 * the YAML value when present, or keeps the default when absent.
	 */

	/* IIR smoothing factor: fraction of history to retain (YAML: convergence.speed) */
	float smoothingFactor_ = 0.7f;

	/* Startup instant-convergence speed (YAML: convergence.startup_speed) */
	float startupSpeed_ = 1.0f;

	/* Stats deliveries at startup_speed (YAML: convergence.startup_frames) */
	unsigned int startupFrames_ = 8;

	/* Damping applied on top of IIR rate (YAML: convergence.damping) */
	float damping_ = 1.0f;

	/* Per-channel fractional delta for stable declaration (YAML: convergence.stability_threshold) */
	float stabilityThreshold_ = 0.015f;

	/* Resume threshold = stabilityThreshold_ * 2 — set by init() */
	float resumeThreshold_ = 0.03f;

	/* Backward-compat speed floor (YAML: convergence.frames) */
	unsigned int convergenceFrames_ = 0;

	/* Minimum scene luminance for AWB update (YAML: min_scene_luminance) */
	float darkSceneThreshold_ = 8.0f;

	/* Per-channel gain limits (YAML: r_b_min, r_b_max) */
	float gainMin_ = 0.2f;
	float gainMax_ = 16.0f;

	/* Green-balance gain limits (YAML: gb_gain_min, gb_gain_max) */
	float gbGainMin_ = 0.8f;
	float gbGainMax_ = 1.25f;

	/* CT clamp limits (YAML: color_temperature.min/max) */
	float ctMin_ = 1800.0f;
	float ctMax_ = 15000.0f;

	/* Reference gains at configure() before AWB converges (YAML: reference_gains) */
	float refRGain_ = 2.55f;
	float refBGain_ = 2.10f;
	float refGrGain_ = 1.0f;
	float refGbGain_ = 1.0f;

	/* Fallback CT when no prior estimate exists (YAML: ct_fallback) */
	float defaultCt_ = 5500.0f;

	/* Per-CT reference gain table (YAML: reference_gains_table) */
	struct RefGainEntry {
		float ct;
		float r;
		float b;
	};
	std::vector<RefGainEntry> refGainTable_;

	std::pair<float, float> interpolateRefGains(float ct) const;

	/* Highlight-blend weight (YAML: highlight_weight) */
	float highlightWeight_ = 0.3f; /* 0=pure grey-world, 1=pure highlight */

	/*
	 * Robertson CT correction PWL table (YAML: ct_correction_pwl).
	 * Maps Robertson-estimated CT → corrected CT using piecewise linear
	 * interpolation. Calibrated from ColorChecker captures under known
	 * illuminants. Without calibration the ct_color_matrix approximation
	 * causes systematic errors — up to +47% at 2860K for IMX219.
	 *
	 * Format: pairs of [robertson_K, corrected_K], sorted ascending
	 * by robertson_K. Empty = no correction applied.
	 */
	struct CtCorrectionPoint {
		float robertson; /* Robertson raw estimate in K */
		float corrected; /* True CCT in K               */
	};
	std::vector<CtCorrectionPoint> ctCorrectionPwl_;

	float applyCtCorrection(float rawCt) const;

	/* ---- Runtime state — NOT tuning params ---- */
	unsigned int awbFrameCount_ = 0;
	bool awbStable_ = false;

	/*
	 * Counter for consecutive frames where the scene was too dark for
	 * AWB to compute gains. After kDarkStableFrames consecutive dark
	 * frames, the gains are considered stable (frozen at seed values
	 * from reference_gains_table) and AwbLocked is set to true.
	 * This matches RPi's approach: no AWB lock concept, but the IPA
	 * should indicate convergence when gains are stable regardless of
	 * whether they were computed or frozen.
	 */
	static constexpr unsigned int kDarkStableFrames = 5;
	unsigned int darkFrameCount_ = 0;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
