/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto Gain/Exposure Control for Microchip ISC
 */

#pragma once

#include <vector>

#include "libipa/pwl.h"

#include "algorithm.h"
#include "common.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Agc : public Algorithm
{
public:
	Agc() = default;
	~Agc() = default;

	struct YTargetPoint {
		float lux;
		float target; /* normalised [0,1], multiplied by kHistogramMax at use */
	};

	unsigned int priority() const override { return 10; }

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mchp_isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	/*
	 * Tuning parameters — ALL set by init() from YAML, never from here.
	 * No in-class initialisers for tuning values: if a required key is
	 * missing from YAML the behaviour must be deterministic and logged,
	 * not silently fall back to a hidden .h constant. Safe fallbacks are
	 * in init() as get<T>(fallback) arguments and documented in YAML.
	 *
	 * Runtime state (counters, flags) keeps its = 0 / = false initialiser.
	 */

	/*
	 * Tuning parameter defaults — values match imx219.yaml to ensure the
	 * algorithm produces a sane image even when the YAML key is absent.
	 * Pattern mirrors libipa AgcMeanLuminance, which initialises all
	 * members via the constructor init-list.
	 * Using .get<float>(member_) as default is UB when member_ is
	 * uninitialised — these initialisers fix that.
	 */

	/* Fallback fixed target when y_target table is absent (YAML: target_brightness) */
	float targetBrightness_ = 60.0f; /* 18% grey net bins, bl=64 DN */

	/*
	 * Linear mid-grey target for raw Bayer output (YAML: raw_target_brightness).
	 * Raw frames have no gamma/tone lift, so metering to the gamma-tuned lux
	 * curve leaves them near-black. ~0.18 x kHistogramMax net bins.
	 */
	float rawTargetBrightness_ = 90.0f;

	/* IIR convergence speed in steady state (YAML: convergence.speed) */
	float convergenceSpeed_ = 0.2f; /* default steady-state IIR rate */

	/* Speed used for first startupFrames_ stats deliveries (YAML: convergence.startup_speed) */
	float startupSpeed_ = 1.0f; /* instant jump on first frames */

	/* Stats deliveries at startup speed (YAML: convergence.startup_frames) */
	unsigned int startupFrames_ = 3;

	/* Near-target band for sqrt(speed) accelerator (YAML: convergence.near_target_fraction) */
	float nearTargetFraction_ = 0.2f; /* ±20% near-target band */

	/* Luminance stability band for AE lock (YAML: convergence.stability_threshold) */
	float stabilityThreshold_ = 5.0f; /* histogram bins */

	/* Backward-compat: minimum convergence rate = 1/frames (YAML: convergence.frames) */
	unsigned int convergenceFrames_ = 0; /* 0 = disabled */

	/* Consecutive stable deliveries required before AeStateConverged (YAML: convergence.lock_frames) */
	unsigned int aeLockFrames_ = 3;

	/* Fraction of clipped pixels triggering highlight anti-windup (YAML: clip_threshold) */
	float clipThreshold_ = 0.02f;

	/* Per-frame brightness ratio limits (YAML: ratio_clamp.min/max/clip_cap) */
	float ratioMin_ = 0.25f;
	float ratioMax_ = 2.5f;
	float clipRatioCap_ = 0.85f;

	/* Gain step thresholds (YAML: gain_thresholds.boost/reduction) */
	float gainBoostThreshold_ = 1.05f;
	float gainReductionThreshold_ = 0.95f;

	/* Minimum scene luminance for AE stability check (YAML: ae_luminance_floor) */
	float aeLuminanceFloor_ = 3.0f; /* net bins post-pedestal */

	/* Mains flicker avoidance: 50.0 or 60.0 Hz, 0.0 = disabled (YAML: flicker_hz) */
	float flickerHz_ = 0.0f; /* disabled by default */

	/*
	 * Slowest auto frame rate for low-light frame extension (YAML:
	 * min_frame_rate). The AGC grows the frame to raise the exposure
	 * ceiling before engaging gain, down to this floor; gain covers
	 * anything below it. 0 disables extension.
	 */
	float minFrameRate_ = 10.0f;

	/* Lux estimator calibration (YAML: lux_reference.*) — luxRefY_=0 disables */
	float luxRefY_ = 0.0f; /* 0 = uncalibrated, use fallback lux */
	float luxRefExposureUs_ = 0.0f;
	float luxRefGain_ = 1.0f;
	float luxRefLux_ = 400.0f; /* fallback indoor office lux */

	/* Lux-adaptive brightness target curve (YAML: y_target) */
	ipa::Pwl yTargetPwl_;

	/* ---- Runtime state — NOT tuning params, kept zero-initialised ---- */
	unsigned int aeLockCount_ = 0;
	unsigned int agcFrameCount_ = 0;
	/* P85 highlight-protection metering */
	float highlightP85Threshold_ = 0.55f; // fraction of HIST_MAX [0,1]
	float highlightP85Strength_ = 1.0f;
	float evBias_ = 0.0f; // YAML default EV bias
	float zoneMeteringStrength_ = 0.7f;
	/* IQM quantile range for midtone metering */
	float iqmQlo_ = 0.02f;
	float iqmQhi_ = 0.98f;
	/* Highlight constraint: top fraction and normalised target */
	float highlightQhi_ = 0.02f;
	float highlightTarget_ = 0.70f; // normalised highlight target [0,1]
	/* Scene entropy-based automatic EV bias */
	float sceneEvThreshold_ = 0.35f; // entropy below this = flat/bright scene
	float sceneEvBias_ = -1.5f; // EV offset to apply (stops)
	float sceneBrightP10_ = 50.0f; // P10 above this = scene has no shadows
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
