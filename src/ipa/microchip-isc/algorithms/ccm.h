/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Color Correction Matrix algorithm
	*/

#ifndef IPA_MICROCHIP_ISC_CCM_H
#define IPA_MICROCHIP_ISC_CCM_H

#include "common.h"

namespace libcamera {
namespace ipa::microchip_isc {

class CCM {
public:
	struct CCMCoefficients {
	/* 3x3 color correction matrix plus offset values */
	std::array<float, 9> matrix;
	std::array<float, 3> offset;
	};

	CCM();

	void configure(const MicrochipISCSensorInfo &sensorInfo);

	void process(const ImageStats &stats, ControlList &results);
	uint32_t estimateColorTemperature(const ImageStats &stats);
	void modulateCCMStrength(CCMCoefficients &ccm, const ImageStats &stats);
	CCMCoefficients calculateCCM(uint32_t colorTemperature);

private:

	/* Constants */
	static constexpr uint32_t SCENE_COLOR_CORRECTION_ID = 0x009819d2;

	/* Control IDs for CCM coefficients */
	static constexpr uint32_t CCM_COEFF_00_ID = 0x009819e0;
	static constexpr uint32_t CCM_COEFF_01_ID = 0x009819e1;
	static constexpr uint32_t CCM_COEFF_02_ID = 0x009819e2;
	static constexpr uint32_t CCM_COEFF_10_ID = 0x009819e3;
	static constexpr uint32_t CCM_COEFF_11_ID = 0x009819e4;
	static constexpr uint32_t CCM_COEFF_12_ID = 0x009819e5;
	static constexpr uint32_t CCM_COEFF_20_ID = 0x009819e6;
	static constexpr uint32_t CCM_COEFF_21_ID = 0x009819e7;
	static constexpr uint32_t CCM_COEFF_22_ID = 0x009819e8;
	static constexpr uint32_t CCM_OFFSET_R_ID = 0x009819e9;
	static constexpr uint32_t CCM_OFFSET_G_ID = 0x009819ea;
	static constexpr uint32_t CCM_OFFSET_B_ID = 0x009819eb;

	/* Pre-calibrated matrices for different lighting conditions */
	static const CCMCoefficients kDaylightCCM;
	static const CCMCoefficients kTungstenCCM;
	static const CCMCoefficients kFluorescentCCM;
	static const CCMCoefficients kShadesCCM;
	static const CCMCoefficients kCloudyCCM;

	/* Sensor info */
	ImageInfo sensorInfo_;

	/* Apply CCM coefficients to the control list */
	void applyCoefficients(const CCMCoefficients &coeffs, ControlList &results);
};

LOG_DECLARE_CATEGORY(ISC_CCM)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_CCM_H */
