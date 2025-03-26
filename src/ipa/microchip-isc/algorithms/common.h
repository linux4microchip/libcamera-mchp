/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Common utilities for image processing algorithms
	*/

#ifndef IPA_MICROCHIP_ISC_COMMON_H
#define IPA_MICROCHIP_ISC_COMMON_H

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>

#include <vector>
#include <array>
#include <algorithm>
#include <cmath>

namespace libcamera {
namespace ipa::microchip_isc {

/* Structure to hold image information */
struct ImageInfo {
	unsigned int width;
	unsigned int height;
	uint32_t format;
};

/* Structure to represent a region in the image */
struct ImageRegion {
	float meanR;
	float meanG;
	float meanB;
	float meanY;
	bool isNeutral;
};

/* Structure to hold image statistics */
struct ImageStats {
	/* Histograms */
	std::array<uint32_t, 256> yHistogram;  /* Luminance histogram */
	std::array<uint32_t, 256> rHistogram;  /* Red channel histogram */
	std::array<uint32_t, 256> gHistogram;  /* Green channel histogram */
	std::array<uint32_t, 256> bHistogram;  /* Blue channel histogram */

	/* Mean values */
	float meanY;
	float meanR;
	float meanG;
	float meanB;

	/* Min/Max values */
	uint8_t minY, maxY;
	uint8_t minR, maxR;
	uint8_t minG, maxG;
	uint8_t minB, maxB;

	/* Region-based statistics */
	std::vector<ImageRegion> regions;
};

/* Generate statistics from YUYV buffer */
void generateStatsFromYUYV(const uint8_t* data, const ImageInfo& info, ImageStats& stats);

/* Generate statistics from RGB565 buffer */
void generateStatsFromRGB565(const uint8_t* data, const ImageInfo& info, ImageStats& stats);

/* Generate statistics from RGBP (24-bit RGB) buffer */
void generateStatsFromRGBP(const uint8_t* data, const ImageInfo& info, ImageStats& stats);

void generateStatsFromBayer(const uint8_t* data, const ImageInfo& info, ImageStats& stats);


/* Utility function to convert YUV to RGB */
void convertYUVtoRGB(uint8_t y, uint8_t u, uint8_t v,
	uint8_t& r, uint8_t& g, uint8_t& b);

/* Utility function to convert RGB to YUV */
void convertRGBtoYUV(uint8_t r, uint8_t g, uint8_t b,
	uint8_t& y, uint8_t& u, uint8_t& v);

/* Calculate if a region is neutral (for AWB) */
bool isNeutralRegion(float r, float g, float b,
	float saturationThreshold = 0.2f,
	float minValue = 30.0f);

LOG_DECLARE_CATEGORY(IPACommon)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_COMMON_H */
