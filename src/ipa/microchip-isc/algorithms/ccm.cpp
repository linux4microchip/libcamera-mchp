/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Color Correction Matrix algorithm
	*/
#include "ccm.h"
namespace libcamera {
namespace ipa::microchip_isc {
LOG_DEFINE_CATEGORY(ISC_CCM)
/* Fine-tuned matrices for different lighting conditions */
const CCM::CCMCoefficients CCM::kDaylightCCM = {
	.matrix = {
	1.75f, -0.55f, -0.2f,  /* Less aggressive red channel */
	-0.35f, 1.65f, -0.3f,  /* Slightly reduced green channel */
	-0.15f, -0.7f, 1.85f	 /* Slightly reduced blue channel */
	},
	.offset = {0.0f, 0.0f, 0.0f}
};
const CCM::CCMCoefficients CCM::kTungstenCCM = {
	.matrix = {
	1.55f, -0.35f, -0.2f,  /* Reduce red in warm light */
	-0.25f, 1.45f, -0.2f,  /* Slightly reduce green */
	-0.15f, -0.5f, 1.65f	 /* Increase blue for warm light balance */
	},
	.offset = {0.0f, 0.0f, 0.0f}
};
const CCM::CCMCoefficients CCM::kFluorescentCCM = {
	.matrix = {
	1.65f, -0.45f, -0.2f,
	-0.25f, 1.55f, -0.3f,
	-0.15f, -0.65f, 1.8f
	},
	.offset = {0.0f, 0.0f, 0.0f}
};
const CCM::CCMCoefficients CCM::kShadesCCM = {
	.matrix = {
	1.85f, -0.65f, -0.2f,
	-0.35f, 1.75f, -0.4f,
	-0.25f, -0.85f, 2.1f	 /* Slightly reduced to avoid over-blue */
	},
	.offset = {0.0f, 0.0f, 0.0f}
};
const CCM::CCMCoefficients CCM::kCloudyCCM = {
	.matrix = {
	1.8f, -0.55f, -0.25f,
	-0.35f, 1.7f, -0.35f,
	-0.15f, -0.75f, 1.9f
	},
	.offset = {0.0f, 0.0f, 0.0f}
};
CCM::CCM()
{
}
void CCM::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	LOG(ISC_CCM, Debug) << "Configuring Color Correction Matrix for sensor "
	<< sensorInfo.model
	<< " resolution: " << sensorInfo.width << "x"
	<< sensorInfo.height;
	sensorInfo_.width = sensorInfo.width;
	sensorInfo_.height = sensorInfo.height;
	sensorInfo_.format = sensorInfo.pixelFormat;
}
void CCM::process(const ImageStats &stats, ControlList &results)
{
	/* Extract color temperature from statistics */
	uint32_t colorTemperature = estimateColorTemperature(stats);

	LOG(ISC_CCM, Debug) << "Processing CCM for color temperature: " << colorTemperature << "K";

	/* Calculate appropriate CCM for this temperature */
	CCMCoefficients ccm = calculateCCM(colorTemperature);

	/* Apply strength modulation based on scene properties */
	modulateCCMStrength(ccm, stats);

	/* Apply coefficients to results */
	applyCoefficients(ccm, results);

	/* Set scene color correction info */
	results.set(SCENE_COLOR_CORRECTION_ID, static_cast<int32_t>(colorTemperature));
}
uint32_t CCM::estimateColorTemperature(const ImageStats &stats)
{
	/* Estimate color temperature from RGB means */
	uint32_t colorTemperature = 5500;  /* Default */

	if (stats.meanR > 0 && stats.meanB > 0) {
	float rToB = stats.meanR / stats.meanB;

	/* More precise estimation */
	if (rToB > 1.5f) {
	colorTemperature = 2700;	/* Very warm (tungsten) */
	} else if (rToB > 1.2f) {
	colorTemperature = 3000 + static_cast<int>((1.5f - rToB) * 2500.0f);
	} else if (rToB > 1.0f) {
	colorTemperature = 4000 + static_cast<int>((1.2f - rToB) * 5000.0f);
	} else if (rToB > 0.8f) {
	colorTemperature = 5000 + static_cast<int>((1.0f - rToB) * 5000.0f);
	} else if (rToB > 0.7f) {
	colorTemperature = 6000 + static_cast<int>((0.8f - rToB) * 5000.0f);
	} else {
	colorTemperature = 6500 + static_cast<int>((0.7f - rToB) * 3500.0f);
	}
	}

	return colorTemperature;
}
CCM::CCMCoefficients CCM::calculateCCM(uint32_t colorTemperature)
{
	/* Simple interpolation between pre-calibrated matrices based on color temperature */
	if (colorTemperature <= 3000) {
	return kTungstenCCM;
	} else if (colorTemperature <= 4000) {
	/* Linear interpolation between tungsten and fluorescent */
	float factor = (colorTemperature - 3000) / 1000.0f;
	CCMCoefficients result;
	for (size_t i = 0; i < 9; i++) {
	result.matrix[i] = kTungstenCCM.matrix[i] * (1.0f - factor) +
	kFluorescentCCM.matrix[i] * factor;
	}
	for (size_t i = 0; i < 3; i++) {
	result.offset[i] = kTungstenCCM.offset[i] * (1.0f - factor) +
	kFluorescentCCM.offset[i] * factor;
	}
	return result;
	} else if (colorTemperature <= 5500) {
	/* Linear interpolation between fluorescent and daylight */
	float factor = (colorTemperature - 4000) / 1500.0f;
	CCMCoefficients result;
	for (size_t i = 0; i < 9; i++) {
	result.matrix[i] = kFluorescentCCM.matrix[i] * (1.0f - factor) +
	kDaylightCCM.matrix[i] * factor;
	}
	for (size_t i = 0; i < 3; i++) {
	result.offset[i] = kFluorescentCCM.offset[i] * (1.0f - factor) +
	kDaylightCCM.offset[i] * factor;
	}
	return result;
	} else if (colorTemperature <= 6500) {
	/* Linear interpolation between daylight and cloudy */
	float factor = (colorTemperature - 5500) / 1000.0f;
	CCMCoefficients result;
	for (size_t i = 0; i < 9; i++) {
	result.matrix[i] = kDaylightCCM.matrix[i] * (1.0f - factor) +
	kCloudyCCM.matrix[i] * factor;
	}
	for (size_t i = 0; i < 3; i++) {
	result.offset[i] = kDaylightCCM.offset[i] * (1.0f - factor) +
	kCloudyCCM.offset[i] * factor;
	}
	return result;
	} else {
	return kShadesCCM;	/* Very cool light */
	}
}
void CCM::modulateCCMStrength(CCMCoefficients &ccm, const ImageStats &stats)
{
	/* Modulate CCM strength based on scene characteristics */
	float brightness = stats.meanY / 255.0f;	/* 0-1 scale */
	float contrast = (stats.maxY - stats.minY) / 255.0f;	/* 0-1 scale */

	/* For low light/low contrast scenes, reduce correction strength to avoid noise amplification */
	float strengthMultiplier = 1.0f;

	if (brightness < 0.2f || contrast < 0.3f) {
	/* Low light or low contrast - reduce strength */
	strengthMultiplier = 0.85f;
	} else if (brightness > 0.8f && contrast > 0.7f) {
	/* Very bright, high contrast - can be more aggressive */
	strengthMultiplier = 1.05f;
	}

	/* Apply strength modulation only to the non-diagonal elements */
	/* This preserves overall brightness while reducing color shifts */
	for (size_t i = 0; i < 9; i++) {
	if (i != 0 && i != 4 && i != 8) {  /* Non-diagonal elements */
	/* Reduce strength of color correction */
	ccm.matrix[i] *= strengthMultiplier;
	}
	}
}
void CCM::applyCoefficients(const CCMCoefficients &coeffs, ControlList &results)
{
	/* Apply matrix coefficients in fixed-point format (1024 = 1.0) */
	constexpr int32_t kFixedPointScale = 1024;

	/* Matrix coefficients */
	results.set(CCM_COEFF_00_ID, static_cast<int32_t>(coeffs.matrix[0] * kFixedPointScale));
	results.set(CCM_COEFF_01_ID, static_cast<int32_t>(coeffs.matrix[1] * kFixedPointScale));
	results.set(CCM_COEFF_02_ID, static_cast<int32_t>(coeffs.matrix[2] * kFixedPointScale));
	results.set(CCM_COEFF_10_ID, static_cast<int32_t>(coeffs.matrix[3] * kFixedPointScale));
	results.set(CCM_COEFF_11_ID, static_cast<int32_t>(coeffs.matrix[4] * kFixedPointScale));
	results.set(CCM_COEFF_12_ID, static_cast<int32_t>(coeffs.matrix[5] * kFixedPointScale));
	results.set(CCM_COEFF_20_ID, static_cast<int32_t>(coeffs.matrix[6] * kFixedPointScale));
	results.set(CCM_COEFF_21_ID, static_cast<int32_t>(coeffs.matrix[7] * kFixedPointScale));
	results.set(CCM_COEFF_22_ID, static_cast<int32_t>(coeffs.matrix[8] * kFixedPointScale));

	/* Offset values */
	results.set(CCM_OFFSET_R_ID, static_cast<int32_t>(coeffs.offset[0] * kFixedPointScale));
	results.set(CCM_OFFSET_G_ID, static_cast<int32_t>(coeffs.offset[1] * kFixedPointScale));
	results.set(CCM_OFFSET_B_ID, static_cast<int32_t>(coeffs.offset[2] * kFixedPointScale));

	LOG(ISC_CCM, Debug) << "Applied CCM coefficients in fixed-point format (scale="
	<< kFixedPointScale << ")";
}
} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
