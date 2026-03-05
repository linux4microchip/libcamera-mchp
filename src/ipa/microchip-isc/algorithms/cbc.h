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
	 * Target scene luminance in histogram units (0–511).
	 * Brightness offset is computed as a fraction of the deviation
	 * from this target after AGC has already corrected the main level.
	 */
	float targetLuminance_ = 128.0f;

	/*
	 * Fraction of the residual luminance error applied as a brightness
	 * offset. Kept small (< 0.25) to avoid fighting AGC.
	 */
	float brightnessGain_ = 0.15f;

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
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
