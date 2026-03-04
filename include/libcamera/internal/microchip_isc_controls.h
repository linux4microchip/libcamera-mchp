/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC V4L2 control definitions
 *
 * These match the kernel driver's ISC-specific V4L2 controls.
 * The ISC hardware exposes white balance gains, offsets, and a
 * 3x3 color correction matrix via V4L2 user controls.
 */

#pragma once

#include <linux/v4l2-controls.h>

/* White balance gains (fixed-point, 512 = 1.0x) */
#define V4L2_CID_ISC_WB_R_GAIN (V4L2_CID_USER_ATMEL_ISC_BASE + 0)
#define V4L2_CID_ISC_WB_B_GAIN (V4L2_CID_USER_ATMEL_ISC_BASE + 1)
#define V4L2_CID_ISC_WB_GR_GAIN (V4L2_CID_USER_ATMEL_ISC_BASE + 2)
#define V4L2_CID_ISC_WB_GB_GAIN (V4L2_CID_USER_ATMEL_ISC_BASE + 3)

/* White balance offsets */
#define V4L2_CID_ISC_WB_R_OFFSET (V4L2_CID_USER_ATMEL_ISC_BASE + 4)
#define V4L2_CID_ISC_WB_B_OFFSET (V4L2_CID_USER_ATMEL_ISC_BASE + 5)
#define V4L2_CID_ISC_WB_GR_OFFSET (V4L2_CID_USER_ATMEL_ISC_BASE + 6)
#define V4L2_CID_ISC_WB_GB_OFFSET (V4L2_CID_USER_ATMEL_ISC_BASE + 7)

/* Color correction matrix (3x3 + 3 offsets, hardware scale: 256 = 1.0) */
#define V4L2_CID_ISC_CC_RR		(V4L2_CID_USER_ATMEL_ISC_BASE + 8)
#define V4L2_CID_ISC_CC_RG		(V4L2_CID_USER_ATMEL_ISC_BASE + 9)
#define V4L2_CID_ISC_CC_RB		(V4L2_CID_USER_ATMEL_ISC_BASE + 10)
#define V4L2_CID_ISC_CC_OR		(V4L2_CID_USER_ATMEL_ISC_BASE + 11)
#define V4L2_CID_ISC_CC_GR		(V4L2_CID_USER_ATMEL_ISC_BASE + 12)
#define V4L2_CID_ISC_CC_GG		(V4L2_CID_USER_ATMEL_ISC_BASE + 13)
#define V4L2_CID_ISC_CC_GB		(V4L2_CID_USER_ATMEL_ISC_BASE + 14)
#define V4L2_CID_ISC_CC_OG		(V4L2_CID_USER_ATMEL_ISC_BASE + 15)
#define V4L2_CID_ISC_CC_BR		(V4L2_CID_USER_ATMEL_ISC_BASE + 16)
#define V4L2_CID_ISC_CC_BG		(V4L2_CID_USER_ATMEL_ISC_BASE + 17)
#define V4L2_CID_ISC_CC_BB		(V4L2_CID_USER_ATMEL_ISC_BASE + 18)
#define V4L2_CID_ISC_CC_OB		(V4L2_CID_USER_ATMEL_ISC_BASE + 19)

/*
 * Per-channel gamma LUT override controls (64 x U32, range 0-1023).
 * Element i is the 10-bit output for input range [i*16 .. (i+1)*16-1].
 * Setting any of these overrides the preset V4L2_CID_GAMMA curve.
 * Writing V4L2_CID_GAMMA re-selects the preset and clears the override.
 */
#define V4L2_CID_ISC_GAMMA_B_LUT	(V4L2_CID_USER_ATMEL_ISC_BASE + 20)
#define V4L2_CID_ISC_GAMMA_G_LUT	(V4L2_CID_USER_ATMEL_ISC_BASE + 21)
#define V4L2_CID_ISC_GAMMA_R_LUT	(V4L2_CID_USER_ATMEL_ISC_BASE + 22)
