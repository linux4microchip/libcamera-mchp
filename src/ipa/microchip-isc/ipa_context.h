/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC IPA Context
 */

#pragma once

#include <array>
#include <stdint.h>

#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include "libcamera/internal/bayer_format.h"

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::microchip_isc {

struct IPASessionConfiguration {
	struct {
		utils::Duration minShutterSpeed;
		utils::Duration maxShutterSpeed;
		uint32_t defaultExposure;
		float minAnalogueGain;
		float maxAnalogueGain;
	} agc;

	struct {
		BayerFormat::Order bayerOrder;
		utils::Duration lineDuration;
		/*
		 * Sensor black level in 10-bit units (e.g. 64 for IMX219).
		 * Per-channel values fall back to blackLevel when absent in YAML.
		 * On SAMA7G5 (hasDpc=true) these are not applied to WB offsets
		 * because the DPC hardware block subtracts BL before the WB block.
		 */
		uint32_t blackLevel;
		uint32_t blackLevelR;
		uint32_t blackLevelGR;
		uint32_t blackLevelGB;
		uint32_t blackLevelB;
		/*
		 * 3×3 row-major sensor-to-XYZ colour matrix used for
		 * Robertson (1968) CCT estimation. Row 0 = [X from R,
		 * X from G, X from B], row 1 = Y, row 2 = Z. Normalised
		 * to D65 white. Loaded from YAML Awb.ct_color_matrix; falls
		 * back to approximate IMX219 values when absent.
		 */
		std::array<float, 9> ctColorMatrix;
	} sensor;

	struct {
		/*
		 * hasDpc: SAMA7G5 DPC subtracts black level before the WB
		 * block. Setting WB offsets for BL correction on top would
		 * double-apply the subtraction and corrupt colours.
		 */
		bool hasDpc;

		/*
		 * hasBipartiteGamma: SAMA7G5 GAM block supports a linear toe
		 * segment for better shadow gradation. When true, computeLut()
		 * uses a piecewise linear-toe + power-law curve.
		 */
		bool hasBipartiteGamma;

		/*
		 * hasCbhs: SAMA7G5 CBHS block adds Saturation control
		 * (V4L2_CID_SATURATION) beyond the basic brightness/contrast
		 * available on SAMA5D2 CBC.
		 */
		bool hasCbhs;

		/*
		 * statsAvailable: ISC histogram statistics are only valid for
		 * RAW (Bayer) sensor input. False for sensors that output
		 * YUYV or RGB directly.
		 */
		bool statsAvailable;
	} hw;
};

struct IPAActiveState {
	struct {
		uint32_t exposure;
		float analogueGain;
		bool autoEnabled;
		int32_t exposureTimeMode; /* 0=auto, 1=manual (independent of gain) */
		int32_t analogueGainMode; /* 0=auto, 1=manual (independent of exposure) */
		/*
		 * EV compensation in stops [-2, +2]. Positive values increase
		 * target brightness; negative values decrease it. Applied as a
		 * multiplier (2^ev) to the YAML target_brightness each frame.
		 */
		float exposureValue;
		/*
		 * True once AGC has declared the exposure stable (luminance
		 * within stabilityThreshold_ of the EV-adjusted target).
		 * Read by CBC to inhibit its brightness correction while AGC
		 * is actively converging, preventing the two loops from fighting.
		 */
		bool aeConverged;
	} agc;

	struct {
		double rGain;
		double bGain;
		double grGain;
		double gbGain;
		int32_t rOffset;
		int32_t bOffset;
		int32_t grOffset;
		int32_t gbOffset;
		bool autoEnabled;
		/*
		 * Estimated scene colour temperature in Kelvin, computed by AWB
		 * from gain-undone channel means. Single source of truth shared
		 * with CCM so both modules track the same illuminant estimate.
		 */
		unsigned int ct;
	} awb;

	struct {
		std::array<float, 9> matrix;
		std::array<float, 3> offsets;
	} ccm;

	struct {
		/*
		 * Hardware brightness offset: -1024 to 1023, neutral = 0.
		 * Applied as: output = input + brightness.
		 */
		int32_t brightness;
		/*
		 * Hardware contrast multiplier: -2048 to 2047, neutral = 16.
		 * Applied as: output = (contrast / 16) * input + brightness.
		 */
		int32_t contrast;
		/*
		 * Hardware saturation: 0–100, unity = 16 on SAMA7G5 CBHS.
		 * Only applied on SAMA7G5 (hasCbhs = true) via V4L2_CID_SATURATION.
		 */
		int32_t saturation;
		/*
		 * Hardware hue rotation in degrees: -180 to +180, neutral = 0.
		 * Only applied on SAMA7G5 (hasCbhs = true) via V4L2_CID_HUE.
		 */
		int32_t hue;
	} cbc;

	struct {
		/*
		 * Per-channel histogram means from the most recent stats delivery.
		 * The ISC cycles GR→R→GB→B, but isc_stats_isr() fires only after
		 * the B channel completes, delivering all 4 channels together with
		 * valid_channels = 0x0f.  All four means are therefore always fresh
		 * at delivery time and updated together in processStats().
		 * Indexed by kHistChannelGR/R/GB/B constants from common.h.
		 */
		std::array<float, 4> channelMeans;
		/*
		 * Frame sequence number at which each channel's histogram was
		 * measured (snapshotted in the kernel at HISDONE IRQ time).
		 * Because each Bayer channel is measured on a different frame
		 * within the 4-frame cycle, the four capture_frame values differ
		 * by 0–3.  AWB uses these to look up the exposure and gains that
		 * were active when each channel was measured, enabling per-channel
		 * exposure normalization before gain computation.
		 */
		std::array<uint32_t, 4> captureFrame;
	} histogram;

	struct {
		/*
		 * Effective gamma exponent used to generate the LUT each frame.
		 * Temporally smoothed to avoid flickering.
		 */
		float gammaEff;
		/*
		 * 64-entry 10-bit LUT for each channel (B, G, R).
		 * Element i is the output value for input range [i*16..(i+1)*16-1].
		 */
		std::array<uint32_t, 64> lutB;
		std::array<uint32_t, 64> lutG;
		std::array<uint32_t, 64> lutR;
	} gamma;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		float analogueGain;
		bool autoEnabled;
		int32_t exposureTimeMode; /* 0=auto, 1=manual */
		int32_t analogueGainMode; /* 0=auto, 1=manual */
	} agc;

	struct {
		double rGain;
		double bGain;
		double grGain;
		double gbGain;
		bool autoEnabled;
	} awb;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
