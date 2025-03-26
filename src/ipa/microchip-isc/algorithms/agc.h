/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Auto Gain Control algorithm
	*/

#ifndef IPA_MICROCHIP_ISC_AGC_H
#define IPA_MICROCHIP_ISC_AGC_H

#include "common.h"

namespace libcamera {
namespace ipa::microchip_isc {

class AGC {
public:
	AGC();

	void configure(const MicrochipISCSensorInfo &sensorInfo);

	void process(const ImageStats &stats, ControlList &results);

private:
	/* Constants */
	static constexpr uint32_t AUTO_GAIN_ID = 0x009819d1;

	static constexpr int32_t kDefaultGain = 1024;
	static constexpr int32_t kMinGain = 1024;  /* Minimum gain */
	static constexpr int32_t kMaxGain = 4096;  /* Maximum gain */
	static constexpr float kTargetMean = 140.0f;	/* Target brightness level */

	/* Sensor info */
	ImageInfo sensorInfo_;

	/* Analyze histogram to determine optimal gain */
	int32_t calculateGain(const std::array<uint32_t, 256>& yHistogram, float meanLuminance);
};

LOG_DECLARE_CATEGORY(ISC_AGC)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_AGC_H */
