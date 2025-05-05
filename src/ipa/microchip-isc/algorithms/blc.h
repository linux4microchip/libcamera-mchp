/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Black Level Correction algorithm
	*/

#ifndef IPA_MICROCHIP_ISC_BLC_H
#define IPA_MICROCHIP_ISC_BLC_H

#include "common.h"

namespace libcamera {
namespace ipa::microchip_isc {

class BLC {
public:
	BLC();

	void configure(const MicrochipISCSensorInfo &sensorInfo);

	void process(const ImageStats &stats, ControlList &results);

private:
	/* Constants */
	static constexpr uint32_t BLACK_LEVEL_ID = 0x009819d0;

	static constexpr int kDefaultBlackLevel = 16;
	static constexpr int kMinBlackLevel = 40;  /* Minimum target black level */
	static constexpr int kMaxBlackLevel = 48;  /* Maximum target black level */

	/* Sensor info */
	ImageInfo sensorInfo_;

	/* Analyze histogram to determine optimal black level */
	int calculateBlackLevel(const std::array<uint32_t, 256>& yHistogram, float meanLuminance);
};

LOG_DECLARE_CATEGORY(ISC_BLC)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_BLC_H */
