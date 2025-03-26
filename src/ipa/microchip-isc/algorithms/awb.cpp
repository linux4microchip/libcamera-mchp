/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Auto White Balance algorithm
	*/
#include "awb.h"
namespace libcamera {
namespace ipa::microchip_isc {
LOG_DEFINE_CATEGORY(ISC_AWB)

AWB::AWB()
	: currentMode_(WB_AUTO),
	manualRedGain_(currentRedGain),
	manualBlueGain_(currentBlueGain),
	manualGreenRedGain_(currentGreenRedGain),
	manualGreenBlueGain_(currentGreenBlueGain)
{
}

void AWB::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	LOG(ISC_AWB, Debug) << "Configuring AWB for sensor " << sensorInfo.model
	<< " resolution: " << sensorInfo.width << "x" << sensorInfo.height;
	sensorInfo_.width = sensorInfo.width;
	sensorInfo_.height = sensorInfo.height;
	sensorInfo_.pixelFormat = sensorInfo.pixelFormat;
}

void AWB::setMode(WBMode mode)
{
	if (mode >= WB_AUTO && mode < WB_MODE_MAX) {
	currentMode_ = mode;
	LOG(ISC_AWB, Debug) << "AWB mode set to " << static_cast<int>(mode);
	}
}

void AWB::setManualWB(int32_t redGain, int32_t blueGain, int32_t greenRedGain, int32_t greenBlueGain)
{
	manualRedGain_ = std::clamp(redGain, kMinGain, kMaxGain);
	manualBlueGain_ = std::clamp(blueGain, kMinGain, kMaxGain);
	manualGreenRedGain_ = std::clamp(greenRedGain, kMinGain, kMaxGain);
	manualGreenBlueGain_ = std::clamp(greenBlueGain, kMinGain, kMaxGain);
	LOG(ISC_AWB, Debug) << "Manual WB set - R:" << manualRedGain_
	<< " B:" << manualBlueGain_
	<< " GR:" << manualGreenRedGain_
	<< " GB:" << manualGreenBlueGain_;
}

void AWB::process(const ImageStats &stats, ControlList &results)
{
	/* If using preset mode other than auto, apply preset values */
	if (currentMode_ != WB_AUTO) {
	applyPresetWB(results, currentMode_);
	return;
	}

	/* Otherwise, proceed with automatic AWB algorithm */
	/* Find neutral regions */
	std::vector<ImageRegion> neutralRegions;
	for (const auto& region : stats.regions) {
	if (region.isNeutral) {
	neutralRegions.push_back(region);
	}
	}
	/* If no neutral regions, use all regions */
	if (neutralRegions.empty()) {
	neutralRegions = stats.regions;
	}

	/* Calculate average RGB values from neutral regions */
	float sumR = 0, sumG = 0, sumB = 0;
	for (const auto& region : neutralRegions) {
	sumR += region.meanR;
	sumG += region.meanG;
	sumB += region.meanB;
	}
	float avgR = (neutralRegions.size() > 0) ? sumR / neutralRegions.size() : 128;
	float avgG = (neutralRegions.size() > 0) ? sumG / neutralRegions.size() : 128;
	float avgB = (neutralRegions.size() > 0) ? sumB / neutralRegions.size() : 128;

	LOG(ISC_AWB, Debug) << "Raw channel averages: R=" << avgR
	<< " G=" << avgG
	<< " B=" << avgB;

	/* Calculate color temperature */
	uint32_t colorTemp = estimateCCT(avgR, avgG, avgB);

	/* Calculate white balance gains with improved heuristics */
	calculateWBGains(avgR, avgG, avgB, colorTemp, results);

	LOG(ISC_AWB, Debug) << "AWB gains: R=" << results.get(RED_GAIN_ID).get<int32_t>()
	<< " GR=" << results.get(GREEN_RED_GAIN_ID).get<int32_t>()
	<< " B=" << results.get(BLUE_GAIN_ID).get<int32_t>()
	<< " GB=" << results.get(GREEN_BLUE_GAIN_ID).get<int32_t>()
	<< " CCT=" << colorTemp << "K";
	LOG(ISC_AWB, Debug) << "AWB offsets: R=" << results.get(RED_OFFSET_ID).get<int32_t>()
	<< " GR=" << results.get(GREEN_RED_OFFSET_ID).get<int32_t>()
	<< " B=" << results.get(BLUE_OFFSET_ID).get<int32_t>()
	<< " GB=" << results.get(GREEN_BLUE_OFFSET_ID).get<int32_t>();
}

void AWB::applyBlendedValues(ControlList &results,
	int32_t baseRedGain, int32_t baseBlueGain,
	int32_t baseGreenRedGain, int32_t baseGreenBlueGain,
	int32_t baseRedOffset, int32_t baseBlueOffset,
	int32_t baseGreenRedOffset, int32_t baseGreenBlueOffset)
{
	/* Blend algorithm-calculated gains with current hardware values */
	int32_t redGain = static_cast<int32_t>(hwFactor * currentRedGain + algoFactor * baseRedGain);
	int32_t blueGain = static_cast<int32_t>(hwFactor * currentBlueGain + algoFactor * baseBlueGain);
	int32_t greenRedGain = static_cast<int32_t>(hwFactor * currentGreenRedGain + algoFactor * baseGreenRedGain);
	int32_t greenBlueGain = static_cast<int32_t>(hwFactor * currentGreenBlueGain + algoFactor * baseGreenBlueGain);

	/* Blend offsets */
	int32_t redOffset = static_cast<int32_t>(hwFactor * currentRedOffset + algoFactor * baseRedOffset);
	int32_t blueOffset = static_cast<int32_t>(hwFactor * currentBlueOffset + algoFactor * baseBlueOffset);
	int32_t greenRedOffset = static_cast<int32_t>(hwFactor * currentGreenRedOffset + algoFactor * baseGreenRedOffset);
	int32_t greenBlueOffset = static_cast<int32_t>(hwFactor * currentGreenBlueOffset + algoFactor * baseGreenBlueOffset);

	/* Clamp values to valid ranges */
	redGain = std::clamp(redGain, kMinGain, kMaxGain);
	blueGain = std::clamp(blueGain, kMinGain, kMaxGain);
	greenRedGain = std::clamp(greenRedGain, kMinGain, kMaxGain);
	greenBlueGain = std::clamp(greenBlueGain, kMinGain, kMaxGain);

	redOffset = std::clamp(redOffset, kMinOffset, kMaxOffset);
	blueOffset = std::clamp(blueOffset, kMinOffset, kMaxOffset);
	greenRedOffset = std::clamp(greenRedOffset, kMinOffset, kMaxOffset);
	greenBlueOffset = std::clamp(greenBlueOffset, kMinOffset, kMaxOffset);

	/* Set the blended values in results */
	results.set(RED_GAIN_ID, redGain);
	results.set(BLUE_GAIN_ID, blueGain);
	results.set(GREEN_RED_GAIN_ID, greenRedGain);
	results.set(GREEN_BLUE_GAIN_ID, greenBlueGain);
	results.set(RED_OFFSET_ID, redOffset);
	results.set(BLUE_OFFSET_ID, blueOffset);
	results.set(GREEN_RED_OFFSET_ID, greenRedOffset);
	results.set(GREEN_BLUE_OFFSET_ID, greenBlueOffset);
}

void AWB::applyPresetWB(ControlList &results, WBMode mode)
{
	/* We're going to specify relative adjustments to the current values */
	/* These multipliers will be converted to absolute values */
	float redMultiplier, blueMultiplier, grMultiplier, gbMultiplier;
	int32_t baseRedOffset = currentRedOffset;
	int32_t baseBlueOffset = currentBlueOffset;
	int32_t baseGreenRedOffset = currentGreenRedOffset;
	int32_t baseGreenBlueOffset = currentGreenBlueOffset;

	/* Define preset values as multipliers of current values */
	switch (mode) {
	case WB_DAYLIGHT:
	/* Daylight/Sunny (~5500K) */
	redMultiplier = 0.8f;			 /* Reduce red to 80%  */
	blueMultiplier = 1.3f;		 /* Boost blue by 30% */
	grMultiplier = 1.0f;			 /* No change to green-red */
	gbMultiplier = 1.0f;			 /* No change to green-blue */
	break;

	case WB_CLOUDY:
	/* Cloudy (~6500K) */
	redMultiplier = 0.9f;			 /* Slightly reduce red */
	blueMultiplier = 1.2f;		 /* Boost blue */
	grMultiplier = 1.0f;
	gbMultiplier = 1.0f;
	break;

	case WB_TUNGSTEN:
	/* Tungsten/Incandescent (~3200K) */
	redMultiplier = 0.7f;			 /* Reduce red significantly */
	blueMultiplier = 1.7f;		 /* Boost blue significantly */
	grMultiplier = 1.0f;
	gbMultiplier = 1.05f;			 /* Slight boost to green-blue */
	/* Apply subtle offsets for tungsten lighting */
	baseRedOffset -= 50;
	baseBlueOffset += 80;
	break;

	case WB_FLUORESCENT:
	/* Fluorescent (~4000K)  */
	redMultiplier = 0.75f;		 /* Reduce red */
	blueMultiplier = 1.5f;		 /* Boost blue */
	grMultiplier = 1.0f;
	gbMultiplier = 1.03f;			 /* Slight boost to green-blue */
	break;

	case WB_SHADE:
	/* Shade (~7500K) */
	redMultiplier = 1.0f;			 /* Keep red same */
	blueMultiplier = 1.1f;		 /* Slightly boost blue */
	grMultiplier = 1.0f;
	gbMultiplier = 1.0f;
	/* Apply subtle offsets for shade */
	baseRedOffset += 40;
	baseBlueOffset -= 30;
	break;

	case WB_MANUAL:
	/* Use manually set values - no blending for manual mode */
	results.set(RED_GAIN_ID, manualRedGain_);
	results.set(BLUE_GAIN_ID, manualBlueGain_);
	results.set(GREEN_RED_GAIN_ID, manualGreenRedGain_);
	results.set(GREEN_BLUE_GAIN_ID, manualGreenBlueGain_);
	/* Use current offsets for manual mode */
	results.set(RED_OFFSET_ID, currentRedOffset);
	results.set(BLUE_OFFSET_ID, currentBlueOffset);
	results.set(GREEN_RED_OFFSET_ID, currentGreenRedOffset);
	results.set(GREEN_BLUE_OFFSET_ID, currentGreenBlueOffset);

	LOG(ISC_AWB, Debug) << "Applied manual WB with gains - R:" << manualRedGain_
	<< " B:" << manualBlueGain_
	<< " GR:" << manualGreenRedGain_
	<< " GB:" << manualGreenBlueGain_;
	return;

	default:
	/* Default to no change if unknown mode */
	redMultiplier = 1.0f;
	blueMultiplier = 1.0f;
	grMultiplier = 1.0f;
	gbMultiplier = 1.0f;
	break;
	}

	/* Convert relative multipliers to absolute values based on current hardware values */
	int32_t baseRedGain = static_cast<int32_t>(currentRedGain * redMultiplier);
	int32_t baseBlueGain = static_cast<int32_t>(currentBlueGain * blueMultiplier);
	int32_t baseGreenRedGain = static_cast<int32_t>(currentGreenRedGain * grMultiplier);
	int32_t baseGreenBlueGain = static_cast<int32_t>(currentGreenBlueGain * gbMultiplier);

	/* Apply blending for preset modes */
	applyBlendedValues(results, baseRedGain, baseBlueGain, baseGreenRedGain, baseGreenBlueGain,
	baseRedOffset, baseBlueOffset, baseGreenRedOffset, baseGreenBlueOffset);

	LOG(ISC_AWB, Debug) << "Applied preset WB mode " << static_cast<int>(mode)
	<< " with blended gains - R:" << results.get(RED_GAIN_ID).get<int32_t>()
	<< " B:" << results.get(BLUE_GAIN_ID).get<int32_t>()
	<< " GR:" << results.get(GREEN_RED_GAIN_ID).get<int32_t>()
	<< " GB:" << results.get(GREEN_BLUE_GAIN_ID).get<int32_t>();
}

void AWB::calculateWBGains(float avgR, float avgG, float avgB, uint32_t colorTemp, ControlList &results)
{
	/* Calculate theoretical multipliers based on grey world assumption */
	/* Green is our reference channel */
	float redGainMultiplier = (avgR > 0) ? avgG / avgR : 1.0f;
	float blueGainMultiplier = (avgB > 0) ? avgG / avgB : 1.0f;

	/* Limit maximum gain to avoid color shifts in low light */
	const float maxGainMultiplier = 1.8f;
	redGainMultiplier = std::min(redGainMultiplier, maxGainMultiplier);
	blueGainMultiplier = std::min(blueGainMultiplier, maxGainMultiplier);

	/* Get base color temperature weights based on CCT */
	float redWeight = 1.0f;
	float blueWeight = 1.0f;

	/* Adjust weights based on color temperature */
	if (colorTemp < 4000) {
	/* Warm light (tungsten) - boost blue slightly */
	blueWeight = 1.1f;
	redWeight = 0.95f;
	} else if (colorTemp > 7000) {
	/* Cool light (shade) - boost red slightly */
	redWeight = 1.1f;
	blueWeight = 0.95f;
	}

	/* Apply weights to gain multipliers */
	redGainMultiplier *= redWeight;
	blueGainMultiplier *= blueWeight;

	/* Convert theoretical multipliers to absolute gain values based on current hardware values */
	int32_t baseRedGain = static_cast<int32_t>(currentRedGain * redGainMultiplier);
	int32_t baseBlueGain = static_cast<int32_t>(currentBlueGain * blueGainMultiplier);

	/* Green channels usually stay at neutral values */
	int32_t baseGreenRedGain = currentGreenRedGain;
	int32_t baseGreenBlueGain = currentGreenBlueGain;

	/* Fine-tune green channel gains based on color temperature */
	if (colorTemp < 4000) {
	/* Warm light - slightly adjust green-red channel */
	baseGreenRedGain = static_cast<int32_t>(currentGreenRedGain * 1.03f);
	} else if (colorTemp > 7000) {
	/* Cool light - slightly adjust green-blue channel */
	baseGreenBlueGain = static_cast<int32_t>(currentGreenBlueGain * 1.03f);
	}

	/* Calculate offsets based on color temperature */
	int32_t baseRedOffset = currentRedOffset;
	int32_t baseBlueOffset = currentBlueOffset;
	int32_t baseGreenRedOffset = currentGreenRedOffset;
	int32_t baseGreenBlueOffset = currentGreenBlueOffset;

	/* Apply offset adjustments based on color temperature */
	if (colorTemp < 3500) {
	/* For very warm light, subtle offsets can help with shadow areas */
	baseRedOffset -= 100;  /* Reduce red slightly in shadows */
	baseBlueOffset += 70;  /* Add a bit more blue to shadows */
	} else if (colorTemp > 7000) {
	/* For very cool light, adjust to reduce blue cast in dark areas */
	baseRedOffset += 80;	 /* Add slight red to balance */
	baseBlueOffset -= 120; /* Reduce excessive blue */
	}

	/* Apply blending for auto mode */
	applyBlendedValues(results, baseRedGain, baseBlueGain, baseGreenRedGain, baseGreenBlueGain,
	baseRedOffset, baseBlueOffset, baseGreenRedOffset, baseGreenBlueOffset);
}

uint32_t AWB::estimateCCT(float r, [[maybe_unused]] float g, float b)
{
	/* Improved CCT estimation based on R/B ratio */
	float rToB = (b > 0) ? r / b : 1.0f;

	/* Mapping based on R/B ratio to color temperature */
	uint32_t cct;
	if (rToB > 1.5f) {
	/* Very warm (tungsten, ~2700K) */
	cct = 2700;
	} else if (rToB > 1.2f) {
	/* Warm (halogen, ~3500K) */
	cct = 3500 + static_cast<uint32_t>((1.5f - rToB) * 1600.0f);
	} else if (rToB > 1.0f) {
	/* Neutral to warm (4200-5500K) */
	cct = 4200 + static_cast<uint32_t>((1.2f - rToB) * 5000.0f);
	} else if (rToB > 0.8f) {
	/* Neutral to cool (5500-6500K) */
	cct = 5500 + static_cast<uint32_t>((1.0f - rToB) * 5000.0f);
	} else if (rToB > 0.6f) {
	/* Cool (daylight, ~6500-7500K) */
	cct = 6500 + static_cast<uint32_t>((0.8f - rToB) * 5000.0f);
	} else {
	/* Very cool (overcast, ~7500K+) */
	cct = 7500 + static_cast<uint32_t>((0.6f - rToB) * 3000.0f);
	}

	return std::clamp(cct, 2500u, 10000u);
}

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
