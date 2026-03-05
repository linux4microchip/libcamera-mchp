/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Common definitions and utilities for Microchip ISC IPA algorithms
 */

#pragma once

#include <array>
#include <stdint.h>

#include "../isc_stats.h"
#include "libipa/histogram.h"

namespace libcamera {

namespace ipa::microchip_isc {

/*
 * ISC histogram channel indices matching the hardware layout.
 * The ISC provides 4-channel 512-bin histograms via DMA.
 */
static constexpr unsigned int kHistChannelGR = 0;
static constexpr unsigned int kHistChannelR = 1;
static constexpr unsigned int kHistChannelGB = 2;
static constexpr unsigned int kHistChannelB = 3;
static constexpr unsigned int kHistBins = 512;

/*
 * Maximum plausible pixel count per histogram channel. Bin totals
 * exceeding this threshold indicate a DMA error or uninitialised
 * stats buffer and should be rejected. 16 MP covers the largest
 * sensors this IPA targets.
 */
static constexpr uint64_t kMaxHistPixels = 16000000ULL;

/* White balance gain hardware scaling: 512 = 1.0x */
static constexpr float kWbGainScale = 512.0f;

/* CCM hardware scaling: 256 = 1.0x */
static constexpr int32_t kCcmScale = 256;
static constexpr int32_t kCcmMaxValue = 2047;
static constexpr int32_t kCcmMinValue = -2048;

/* ISC histogram intensity range maximum */
static constexpr float kHistogramMax = 511.0f;

/* CBC/CBHS hardware brightness register limits */
static constexpr float kBrightnessMin = -1024.0f;
static constexpr float kBrightnessMax = 1023.0f;
/*
 * Conservative brightness drive limit: sub-range of the hardware register
 * used to avoid overdriving the AGC control loop. Small enough that CBC
 * brightness only corrects residual error rather than competing with AGC.
 */
static constexpr float kBrightnessDriveLimit = 256.0f;

/* CBC/CBHS hardware contrast register limits */
static constexpr float kContrastMin = -2048.0f;
static constexpr float kContrastMax = 2047.0f;

/*
 * Saturation scale factor: libcamera float (1.0 = normal) × kSaturationScale
 * = ISC CBHS hardware register value (16 = unity).
 */
static constexpr float kSaturationScale = 16.0f;

/*
 * Default sensor-to-XYZ colour matrix for Robertson (1968) CCT estimation.
 * Approximate values for IMX219 derived from published spectral responsivity.
 * Replace via YAML Awb.ct_color_matrix for better accuracy (±200 K vs ±500 K).
 * Calibrate with a Macbeth ColorChecker under known illuminants. * Row-major: row 0 = X, row 1 = Y, row 2 = Z.
 */
inline constexpr std::array<float, 9> kDefaultCtMatrix = {
	0.4900f,
	0.3100f,
	0.1200f,
	0.2200f,
	0.6600f,
	0.0700f,
	0.0200f,
	0.1000f,
	0.8700f,
};

float calculateChannelMean(const mchp_isc_stat_buffer *stats, unsigned int channel);
float calculateHighlightMean(const mchp_isc_stat_buffer *stats, unsigned int channel);
void calculateLuminanceCdf(const mchp_isc_stat_buffer *stats,
			    std::array<float, 64> &lut_cdf);
float calculateWeightedLuminance(const mchp_isc_stat_buffer *stats,
				 const std::array<float, kHistBins> &weights);float calculateClipping(const isc_stat_buffer *stats);float calculateHistogramSpread(const isc_stat_buffer *stats);float estimateColorTemperature(const isc_stat_buffer *stats);
float estimateColorTemperature(float meanR, float meanG, float meanB,
			       const std::array<float, 9> *ctColorMatrix = nullptr);
} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
