/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Black Level Correction algorithm
	*/
#include "blc.h"
namespace libcamera {
namespace ipa::microchip_isc {
LOG_DEFINE_CATEGORY(ISC_BLC)
BLC::BLC()
{
}
void BLC::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	LOG(ISC_BLC, Debug) << "Configuring Black Level Correction for sensor "
	<< sensorInfo.model
	<< " resolution: " << sensorInfo.width << "x"
	<< sensorInfo.height;
	sensorInfo_.width = sensorInfo.width;
	sensorInfo_.height = sensorInfo.height;
	sensorInfo_.format = sensorInfo.pixelFormat;
}
void BLC::process(const ImageStats &stats, ControlList &results)
{
	/* Calculate optimal black level from luminance histogram with improved shadow preservation */
	int blackLevel = calculateBlackLevel(stats.yHistogram, stats.meanY);

	/* Set the result */
	results.set(BLACK_LEVEL_ID, blackLevel);
	LOG(ISC_BLC, Debug) << "Black Level Correction - Calculated Level: " << blackLevel;
}
int BLC::calculateBlackLevel(const std::array<uint32_t, 256>& yHistogram, float meanLuminance)
{
	/* Find the darkest 3% of pixels (reduced from 5% to be less aggressive) */
	uint32_t totalPixels = 0;
	for (const auto& count : yHistogram) {
	totalPixels += count;
	}

	/* Use a smaller percentage for dark pixel detection (3%) */
	uint32_t darkPixelCount = totalPixels / 33;
	uint32_t cumulativeCount = 0;
	uint32_t darkSum = 0;

	/* Calculate average value of the darkest pixels */
	for (size_t i = 0; i < yHistogram.size(); i++) {
	if (cumulativeCount < darkPixelCount) {
	uint32_t pixelsToUse = std::min(darkPixelCount - cumulativeCount, yHistogram[i]);
	darkSum += pixelsToUse * i;
	cumulativeCount += pixelsToUse;
	} else {
	break;
	}
	}

	/* Calculate dark average */
	float darkAverage = (cumulativeCount > 0) ?
	(darkSum / static_cast<float>(cumulativeCount)) : 0;

	/* Adjust black level based on average scene brightness */
	float scaleFactor = 1.0f;

	if (meanLuminance < 80.0f) {
	/* In dark scenes, use a lower scaling factor to preserve shadow details */
	scaleFactor = 0.9f;
	} else if (meanLuminance > 160.0f) {
	/* In bright scenes, be more aggressive with black level */
	scaleFactor = 1.15f;
	}

	/* More conservative approach to black level calculation */
	int blackLevel = static_cast<int>(darkAverage * scaleFactor);

	/* Limit maximum black level to avoid crushing shadows */
	int maxBlackLevel = 48;  /* Reduced from 58 to preserve more shadow detail */

	/* Clamp to desired range */
	return std::clamp(blackLevel, kMinBlackLevel, maxBlackLevel);
}
} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
