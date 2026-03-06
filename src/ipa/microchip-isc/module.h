/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC IPA Module
 */

#pragma once

#include <libcamera/ipa/microchip_isc_ipa_interface.h>

#include <libipa/module.h>

#include "ipa_context.h"
#include "isc_stats.h"

namespace libcamera {

namespace ipa::microchip_isc {

/*
 * ISC has no params DMA buffer -- ISP parameters (AWB gains, CCM matrix)
 * are applied via V4L2 subdev controls. Use a minimal placeholder struct
 * to satisfy the Module template interface.
 */
struct IscParams {
	uint32_t unused;
};

using Module = ipa::Module<IPAContext, IPAFrameContext, IPACameraSensorInfo,
			   IscParams, mchp_isc_stat_buffer>;

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
