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

float calculateChannelMean(const isc_stat_buffer *stats, unsigned int channel);
float calculateLuminance(const isc_stat_buffer *stats);
float calculateClipping(const isc_stat_buffer *stats);
float calculateHistogramSpread(const isc_stat_buffer *stats);
float estimateColorTemperature(const isc_stat_buffer *stats);
} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
