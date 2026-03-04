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
		 * Applied as a fixed WB offset subtraction before AWB gains.
		 * Zero means no black level correction.
		 */
		uint32_t blackLevel;
	} sensor;
};

struct IPAActiveState {
	struct {
		uint32_t exposure;
		float analogueGain;
		bool autoEnabled;
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
