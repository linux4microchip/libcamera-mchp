/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC statistics buffer definition
 *
 * Mirror of the kernel UAPI struct mchp_isc_stat_buffer defined in
 * include/uapi/linux/media/microchip/isc-config.h.  Must be kept in
 * sync with the kernel layout; any divergence silently corrupts all
 * per-frame histogram reads.
 *
 * Layout (natural C alignment, no packing):
 *   timestamp_ns (8) + frame_number (4) + meas_type (4)
 *   + hist[4] × mchp_isc_hist_channel (2072 each)
 *   + valid_channels (1) + bayer_pattern (1) + reserved[3] (6)
 */

#pragma once

#include <stdint.h>

/* Number of histogram bins per Bayer channel. */
static constexpr unsigned int kIscHistBins = 512;
/* Number of Bayer channels in the stats buffer (GR, R, GB, B). */
static constexpr unsigned int kIscHistChannels = 4;

/* Measurement type flags (meas_type bitmask). */
static constexpr uint32_t kIscStatMeasHist = 1u << 0;

/* bayer_pattern values matching MCHP_ISC_BAYCFG_* kernel constants. */
static constexpr uint8_t kIscBaycfgGRGR = 0;
static constexpr uint8_t kIscBaycfgRGRG = 1;
static constexpr uint8_t kIscBaycfgGBGB = 2;
static constexpr uint8_t kIscBaycfgBGBG = 3;

/**
 * Per-channel histogram statistics.
 * Matches kernel struct mchp_isc_hist_channel.
 *
 * capture_frame and capture_ts are snapshotted at HISDONE IRQ time in
 * the kernel, so they reflect the frame sequence and timestamp of the
 * frame on which each channel's histogram was actually measured — not
 * the frame on which the stats buffer was delivered to userspace.
 * Use these to look up the exposure and gains that were active when
 * each channel was measured, enabling per-channel exposure normalization.
 */
struct mchp_isc_hist_channel {
	uint32_t hist_bins[kIscHistBins];
	uint32_t hist_min;
	uint32_t hist_max;
	uint32_t total_pixels;
	uint32_t capture_frame; /**< Frame sequence when channel was measured */
	uint64_t capture_ts; /**< ktime_get_ns() at HISDONE for this channel */
};

/**
 * Complete ISC 3A statistics buffer delivered via V4L2_META_FMT_ISC_STAT_3A.
 * Matches kernel struct mchp_isc_stat_buffer.
 */
struct mchp_isc_stat_buffer {
	uint64_t timestamp_ns; /**< Frame timestamp in nanoseconds */
	uint32_t frame_number; /**< Sequential frame counter */
	uint32_t meas_type; /**< Bitmask of kIscStatMeas* flags */
	struct mchp_isc_hist_channel hist[kIscHistChannels]; /**< GR, R, GB, B */
	uint8_t valid_channels; /**< Bitmask of valid channels in hist[] */
	uint8_t bayer_pattern; /**< Active CFA pattern (kIscBaycfg*) */
	/*
	 * Padding to match the kernel UAPI layout exactly.
	 * Must be uint8_t[6], NOT uint16_t[3]: after two consecutive uint8_t
	 * fields, a uint16_t array would require 2-byte alignment, potentially
	 * inserting 0 or 1 byte of implicit padding before it depending on
	 * the ABI. uint8_t requires only 1-byte alignment — no implicit
	 * padding, matching the C99 kernel struct layout byte-for-byte.
	 */
	uint8_t reserved[6];
};

/*
 * Compile-time size check against the kernel UAPI struct.
 * mchp_isc_hist_channel: 512*4 + 4 + 4 + 4 + 4 + 8 = 2072 bytes
 * mchp_isc_stat_buffer:  8 + 4 + 4 + 4*2072 + 1 + 1 + 6 = 8308 bytes
 * If this assert fires, the kernel UAPI has changed and this header
 * must be updated — otherwise every histogram read is corrupted silently.
 */
/*
 * Size breakdown (ARM 32-bit / natural alignment):
 *   mchp_isc_hist_channel:
 *     hist_bins[512]    = 2048
 *     hist_min/max/total/capture_frame (4×uint32) = 16
 *     capture_ts (uint64, 8-byte aligned at offset 2064) = 8
 *     sizeof = 2072
 *   mchp_isc_stat_buffer:
 *     timestamp_ns(8) + frame_number(4) + meas_type(4) = 16
 *     hist[4] = 4 × 2072 = 8288
 *     valid_channels(1) + bayer_pattern(1) + reserved[6](6) = 8
 *     sizeof = 8312
 */
static_assert(sizeof(mchp_isc_stat_buffer) == 8312,
	      "mchp_isc_stat_buffer size mismatch with kernel UAPI — "
	      "update isc_stats.h to match include/uapi/linux/media/microchip/isc-config.h");
