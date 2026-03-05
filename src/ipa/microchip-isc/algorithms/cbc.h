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
		     const isc_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	/*
	 * Static hardware brightness offset applied after gamma.
	 * Range: [-1024, 1023] in hardware units (0 = no offset).
	 * This is a one-time tuning offset, not a dynamic feedback loop:
	 * AGC owns the luminance control loop; CBC brightness is a static
	 * trim to correct residual sensor or pipeline offsets.
	 */
	int32_t defaultBrightness_ = 0;

	/*
	 * Contrast enhancement strength [0.0–1.0]. At 0.0 the contrast
	 * register is left at unity (16). At 1.0 the full stretch-to-target
	 * gain is applied. Values around 0.4 give natural-looking results.
	 */
	float contrastStrength_ = 0.4f;

	/*
	 * Target P5–P95 spread as a fraction of the full histogram range.
	 * When the scene spread is below this value the contrast is boosted
	 * to make the scene fill the target fraction of the output range.
	 */
	float targetSpread_ = 0.65f;

	/* Temporal smoothing factor [0.0–1.0]: higher = more damping. */
	float smoothingFactor_ = 0.25f;

	/*
	 * Default hardware saturation level (0–100, unity = 16 on SAMA7G5).
	 * Applied via V4L2_CID_SATURATION on SAMA7G5 (hasCbhs = true).
	 * Values above 16 boost colour vividness; below 16 reduce it.
	 */
	int32_t defaultSaturation_ = 16;

	/*
	 * Default hue rotation in degrees (-180 to +180, neutral = 0).
	 * Applied via V4L2_CID_HUE on SAMA7G5 (hasCbhs = true).
	 */
	int32_t defaultHue_ = 0;

	/*
	 * Maximum contrast gain applied by the histogram stretch algorithm.
	 * Limits extreme corrections in near-zero spread scenes (e.g. lens cap).
	 * Tunable via YAML contrast_max_gain (range 1.0–8.0, typical 3.0).
	 */
	float contrastMaxGain_ = 3.0f;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
