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
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
