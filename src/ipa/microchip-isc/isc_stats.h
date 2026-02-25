/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC statistics buffer definition
 *
 * The ISC hardware provides a DMA-delivered statistics buffer containing
 * 512-bin histograms for each of the four Bayer channels (GR, R, GB, B).
 * This structure matches the kernel driver's stat buffer layout.
 */

#pragma once

#include <stdint.h>

struct isc_stat_buffer {
	uint32_t frame_number;
	uint64_t timestamp;
	uint32_t meas_type;

	struct {
		uint32_t hist_bins[512];
		uint32_t hist_min;
		uint32_t hist_max;
		uint32_t total_pixels;
	} hist[4]; /* GR, R, GB, B channels */

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
	uint16_t reserved[3];
};
