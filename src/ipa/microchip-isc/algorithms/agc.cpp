/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Auto Gain Control algorithm
	*/
#include "agc.h"
namespace libcamera {
namespace ipa::microchip_isc {
LOG_DEFINE_CATEGORY(ISC_AGC)
AGC::AGC()
{
}
void AGC::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	LOG(ISC_AGC, Debug) << "Configuring Automatic Gain Control for sensor "
	<< sensorInfo.model
	<< " resolution: " << sensorInfo.width << "x"
	<< sensorInfo.height;
	sensorInfo_.width = sensorInfo.width;
	sensorInfo_.height = sensorInfo.height;
	sensorInfo_.format = sensorInfo.pixelFormat;
}
void AGC::process(const ImageStats &stats, ControlList &results)
{
	/* Calculate optimal gain from luminance histogram with improved noise considerations */
	int32_t gain = calculateGain(stats.yHistogram, stats.meanY);

	/* Set the result */
	results.set(AUTO_GAIN_ID, gain);
	LOG(ISC_AGC, Debug) << "Automatic Gain Control - Calculated Gain: " << gain
	<< " (" << (gain / 1024.0f) << "x)"
	<< " from mean luminance: " << stats.meanY;
}
int32_t AGC::calculateGain(const std::array<uint32_t, 256>& yHistogram, float meanLuminance)
{
	/* Target mean luminance around 135-140 (good brightness without overexposure) */
	float targetLuminance = kTargetMean;

	/* Adjust target based on scene brightness to avoid over-amplifying dark scenes */
	if (meanLuminance < 70.0f) {
	/* Dark scene - be more conservative with gain to avoid noise */
	targetLuminance = std::min(kTargetMean, meanLuminance * 1.5f);
	} else if (meanLuminance > 160.0f) {
	/* Bright scene - don't apply as much gain */
	targetLuminance = meanLuminance;
	}

	/* Calculate dark region ratio */
	uint32_t darkPixels = 0;
	uint32_t totalPixels = 0;

	for (size_t i = 0; i < yHistogram.size(); i++) {
	if (i < 50) {  /* Consider pixels below 50 as "dark" */
	darkPixels += yHistogram[i];
	}
	totalPixels += yHistogram[i];
	}

	float darkRatio = (totalPixels > 0) ?
	static_cast<float>(darkPixels) / totalPixels : 0.0f;

	/* Calculate initial gain */
	float gainFactor = targetLuminance / (meanLuminance + 0.001f);

	/* Limit maximum gain based on dark ratio to avoid amplifying noise */
	const float maxGainBase = 1.7f;  /* Base maximum gain */
	float maxGain = maxGainBase;

	/* Reduce maximum gain if there are many dark areas (high noise potential) */
	if (darkRatio > 0.3f) {
	maxGain = maxGainBase - (darkRatio - 0.3f) * 0.5f;
	maxGain = std::max(1.3f, maxGain);	/* Don't go below 1.3x */
	}

	/* Apply maximum gain limit */
	gainFactor = std::min(gainFactor, maxGain);

	/* Apply minimum gain of 1.0 */
	gainFactor = std::max(1.0f, gainFactor);

	/* Convert to fixed-point representation */
	int32_t calculatedGain = static_cast<int32_t>(kDefaultGain * gainFactor);

	/* Ensure gain is in a reasonable range */
	return std::clamp(calculatedGain, kMinGain, kMaxGain);
}
} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
