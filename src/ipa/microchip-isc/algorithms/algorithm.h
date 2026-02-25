/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC control algorithm interface
 */

#pragma once

#include <libipa/algorithm.h>

#include "../module.h"

namespace libcamera {

namespace ipa::microchip_isc {

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
public:
	/*
	 * ISC-specific execution priority for algorithm ordering in
	 * processStats().  Not part of libipa::Algorithm; defined here
	 * to enforce the ISC dependency chain without coupling algorithms
	 * directly.  Lower values run earlier:
	 *   BLC (0) -> AWB/AGC (10) -> Gamma/CCM (20) -> CBC (30)
	 * Algorithms at the same priority may run in any order.
	 */
	virtual unsigned int priority() const { return 100; }
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
