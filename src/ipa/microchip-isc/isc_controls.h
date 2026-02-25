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

#include "libcamera/internal/microchip_isc_controls.h"
