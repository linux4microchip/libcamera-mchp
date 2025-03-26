/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Common utilities for image processing algorithms
	*/
#include "common.h"

namespace libcamera {
namespace ipa::microchip_isc {
LOG_DEFINE_CATEGORY(IPACommon)
void generateStatsFromYUYV(const uint8_t* data, const ImageInfo& info, ImageStats& stats)
{
	const unsigned int width = info.width;
	const unsigned int height = info.height;
	/* Clear histograms */
	stats.yHistogram.fill(0);
	stats.rHistogram.fill(0);
	stats.gHistogram.fill(0);
	stats.bHistogram.fill(0);
	/* Reset statistics */
	stats.meanY = stats.meanR = stats.meanG = stats.meanB = 0.0f;
	stats.minY = stats.minR = stats.minG = stats.minB = 255;
	stats.maxY = stats.maxR = stats.maxG = stats.maxB = 0;
	/* Region size for statistics */
	constexpr int regionSize = 16;
	const int regionsX = (width + regionSize - 1) / regionSize;
	const int regionsY = (height + regionSize - 1) / regionSize;
	/* Prepare regions */
	stats.regions.resize(regionsX * regionsY);
	/* Accumulators for mean calculation */
	uint64_t sumY = 0, sumR = 0, sumG = 0, sumB = 0;
	uint32_t totalPixels = 0;
	/* Process each region */
	for (int ry = 0; ry < regionsY; ry++) {
	for (int rx = 0; rx < regionsX; rx++) {
	/* Region boundaries */
	int startX = rx * regionSize;
	int startY = ry * regionSize;
	int endX = std::min(startX + regionSize, static_cast<int>(width));
	int endY = std::min(startY + regionSize, static_cast<int>(height));
	/* Accumulators for region stats */
	float regionSumR = 0.0f, regionSumG = 0.0f, regionSumB = 0.0f, regionSumY = 0.0f;
	int regionPixelCount = 0;
	/* Process pixels in this region */
	for (int y = startY; y < endY; y++) {
	for (int x = startX; x < endX; x += 2) {
	/* Calculate index in YUYV data (2 bytes per pixel) */
	size_t idx = (y * width + x) * 2;
	/* Extract YUYV components */
	uint8_t y0 = data[idx];
	uint8_t u = data[idx + 1];
	uint8_t y1 = data[idx + 2];
	uint8_t v = data[idx + 3];
	/* Update Y histogram */
	stats.yHistogram[y0]++;
	stats.yHistogram[y1]++;
	/* Convert to RGB for both pixels */
	uint8_t r0, g0, b0, r1, g1, b1;
	convertYUVtoRGB(y0, u, v, r0, g0, b0);
	convertYUVtoRGB(y1, u, v, r1, g1, b1);
	/* Update RGB histograms */
	stats.rHistogram[r0]++; stats.rHistogram[r1]++;
	stats.gHistogram[g0]++; stats.gHistogram[g1]++;
	stats.bHistogram[b0]++; stats.bHistogram[b1]++;
	/* Track min/max values */
	stats.minY = std::min<uint8_t>(stats.minY, std::min<uint8_t>(y0, y1));
	stats.maxY = std::max<uint8_t>(stats.maxY, std::max<uint8_t>(y0, y1));
	stats.minR = std::min<uint8_t>(stats.minR, std::min<uint8_t>(r0, r1));
	stats.maxR = std::max<uint8_t>(stats.maxR, std::max<uint8_t>(r0, r1));
	stats.minG = std::min<uint8_t>(stats.minG, std::min<uint8_t>(g0, g1));
	stats.maxG = std::max<uint8_t>(stats.maxG, std::max<uint8_t>(g0, g1));
	stats.minB = std::min<uint8_t>(stats.minB, std::min<uint8_t>(b0, b1));
	stats.maxB = std::max<uint8_t>(stats.maxB, std::max<uint8_t>(b0, b1));
	/* Accumulate for global means */
	sumY += y0 + y1;
	sumR += r0 + r1;
	sumG += g0 + g1;
	sumB += b0 + b1;
	totalPixels += 2;
	/* Accumulate for region stats */
	regionSumY += y0 + y1;
	regionSumR += r0 + r1;
	regionSumG += g0 + g1;
	regionSumB += b0 + b1;
	regionPixelCount += 2;
	}
	}
	/* Calculate region means */
	if (regionPixelCount > 0) {
	auto& region = stats.regions[ry * regionsX + rx];
	region.meanY = regionSumY / regionPixelCount;
	region.meanR = regionSumR / regionPixelCount;
	region.meanG = regionSumG / regionPixelCount;
	region.meanB = regionSumB / regionPixelCount;
	/* Determine if the region is neutral for AWB */
	region.isNeutral = isNeutralRegion(
	region.meanR, region.meanG, region.meanB);
	}
	}
	}
	/* Calculate global means */
	if (totalPixels > 0) {
	stats.meanY = static_cast<float>(sumY) / totalPixels;
	stats.meanR = static_cast<float>(sumR) / totalPixels;
	stats.meanG = static_cast<float>(sumG) / totalPixels;
	stats.meanB = static_cast<float>(sumB) / totalPixels;
	}
}

void generateStatsFromRGBP(const uint8_t* data, const ImageInfo& info, ImageStats& stats)
{
	const unsigned int width = info.width;
	const unsigned int height = info.height;
	/* Clear histograms */
	stats.yHistogram.fill(0);
	stats.rHistogram.fill(0);
	stats.gHistogram.fill(0);
	stats.bHistogram.fill(0);
	/* Reset statistics */
	stats.meanY = stats.meanR = stats.meanG = stats.meanB = 0.0f;
	stats.minY = stats.minR = stats.minG = stats.minB = 255;
	stats.maxY = stats.maxR = stats.maxG = stats.maxB = 0;
	/* Region size for statistics */
	constexpr int regionSize = 16;
	const int regionsX = (width + regionSize - 1) / regionSize;
	const int regionsY = (height + regionSize - 1) / regionSize;
	/* Prepare regions */
	stats.regions.resize(regionsX * regionsY);
	/* Accumulators for mean calculation */
	uint64_t sumY = 0, sumR = 0, sumG = 0, sumB = 0;
	uint32_t totalPixels = 0;

	/* Process each region */
	for (int ry = 0; ry < regionsY; ry++) {
	for (int rx = 0; rx < regionsX; rx++) {
	/* Region boundaries */
	int startX = rx * regionSize;
	int startY = ry * regionSize;
	int endX = std::min(startX + regionSize, static_cast<int>(width));
	int endY = std::min(startY + regionSize, static_cast<int>(height));
	/* Accumulators for region stats */
	float regionSumR = 0.0f, regionSumG = 0.0f, regionSumB = 0.0f, regionSumY = 0.0f;
	int regionPixelCount = 0;

	/* Process pixels in this region */
	for (int y = startY; y < endY; y++) {
	for (int x = startX; x < endX; x++) {
	/* Calculate index in RGBP data (3 bytes per pixel) */
	size_t idx = (y * width + x) * 3;

	/* Extract RGB components (8 bits per channel) */
	uint8_t r = data[idx];
	uint8_t g = data[idx + 1];
	uint8_t b = data[idx + 2];

	/* Calculate luminance (Y) */
	uint8_t yValue = static_cast<uint8_t>(0.299f*r + 0.587f*g + 0.114f*b);

	/* Update histograms */
	stats.yHistogram[yValue]++;
	stats.rHistogram[r]++;
	stats.gHistogram[g]++;
	stats.bHistogram[b]++;

	/* Track min/max values */
	stats.minY = std::min<uint8_t>(stats.minY, yValue);
	stats.maxY = std::max<uint8_t>(stats.maxY, yValue);
	stats.minR = std::min<uint8_t>(stats.minR, r);
	stats.maxR = std::max<uint8_t>(stats.maxR, r);
	stats.minG = std::min<uint8_t>(stats.minG, g);
	stats.maxG = std::max<uint8_t>(stats.maxG, g);
	stats.minB = std::min<uint8_t>(stats.minB, b);
	stats.maxB = std::max<uint8_t>(stats.maxB, b);

	/* Accumulate for global means */
	sumY += yValue;
	sumR += r;
	sumG += g;
	sumB += b;
	totalPixels++;

	/* Accumulate for region stats */
	regionSumY += yValue;
	regionSumR += r;
	regionSumG += g;
	regionSumB += b;
	regionPixelCount++;
	}
	}

	/* Calculate region means */
	if (regionPixelCount > 0) {
	auto& region = stats.regions[ry * regionsX + rx];
	region.meanY = regionSumY / regionPixelCount;
	region.meanR = regionSumR / regionPixelCount;
	region.meanG = regionSumG / regionPixelCount;
	region.meanB = regionSumB / regionPixelCount;
	/* Determine if the region is neutral for AWB */
	region.isNeutral = isNeutralRegion(
	region.meanR, region.meanG, region.meanB);
	}
	}
	}

	/* Calculate global means */
	if (totalPixels > 0) {
	stats.meanY = static_cast<float>(sumY) / totalPixels;
	stats.meanR = static_cast<float>(sumR) / totalPixels;
	stats.meanG = static_cast<float>(sumG) / totalPixels;
	stats.meanB = static_cast<float>(sumB) / totalPixels;
	}
}

void generateStatsFromRGB565(const uint8_t* data, const ImageInfo& info, ImageStats& stats)
{
	const unsigned int width = info.width;
	const unsigned int height = info.height;
	/* Clear histograms */
	stats.yHistogram.fill(0);
	stats.rHistogram.fill(0);
	stats.gHistogram.fill(0);
	stats.bHistogram.fill(0);
	/* Reset statistics */
	stats.meanY = stats.meanR = stats.meanG = stats.meanB = 0.0f;
	stats.minY = stats.minR = stats.minG = stats.minB = 255;
	stats.maxY = stats.maxR = stats.maxG = stats.maxB = 0;
	/* Region size for statistics */
	constexpr int regionSize = 16;
	const int regionsX = (width + regionSize - 1) / regionSize;
	const int regionsY = (height + regionSize - 1) / regionSize;
	/* Prepare regions */
	stats.regions.resize(regionsX * regionsY);
	/* Accumulators for mean calculation */
	uint64_t sumY = 0, sumR = 0, sumG = 0, sumB = 0;
	uint32_t totalPixels = 0;
	/* Process each region */
	for (int ry = 0; ry < regionsY; ry++) {
	for (int rx = 0; rx < regionsX; rx++) {
	/* Region boundaries */
	int startX = rx * regionSize;
	int startY = ry * regionSize;
	int endX = std::min(startX + regionSize, static_cast<int>(width));
	int endY = std::min(startY + regionSize, static_cast<int>(height));
	/* Accumulators for region stats */
	float regionSumR = 0.0f, regionSumG = 0.0f, regionSumB = 0.0f, regionSumY = 0.0f;
	int regionPixelCount = 0;
	/* Process pixels in this region */
	for (int y = startY; y < endY; y++) {
	for (int x = startX; x < endX; x++) {
	/* Calculate index in RGB565 data (2 bytes per pixel) */
	size_t idx = (y * width + x) * 2;
	/* Extract RGB565 value (little endian) */
	uint16_t pixel = static_cast<uint16_t>(data[idx]) |
	(static_cast<uint16_t>(data[idx + 1]) << 8);
	/* Extract RGB components (5 bits R, 6 bits G, 5 bits B) */
	uint8_t r = (pixel >> 11) & 0x1F;
	uint8_t g = (pixel >> 5) & 0x3F;
	uint8_t b = pixel & 0x1F;
	/* Expand to 8 bits */
	r = (r << 3) | (r >> 2);
	g = (g << 2) | (g >> 4);
	b = (b << 3) | (b >> 2);
	/* Calculate luminance (Y) */
	uint8_t yValue = static_cast<uint8_t>(0.299f*r + 0.587f*g + 0.114f*b);
	/* Update histograms */
	stats.yHistogram[yValue]++;
	stats.rHistogram[r]++;
	stats.gHistogram[g]++;
	stats.bHistogram[b]++;
	/* Track min/max values */
	stats.minY = std::min<uint8_t>(stats.minY, yValue);
	stats.maxY = std::max<uint8_t>(stats.maxY, yValue);
	stats.minR = std::min<uint8_t>(stats.minR, r);
	stats.maxR = std::max<uint8_t>(stats.maxR, r);
	stats.minG = std::min<uint8_t>(stats.minG, g);
	stats.maxG = std::max<uint8_t>(stats.maxG, g);
	stats.minB = std::min<uint8_t>(stats.minB, b);
	stats.maxB = std::max<uint8_t>(stats.maxB, b);
	/* Accumulate for global means */
	sumY += yValue;
	sumR += r;
	sumG += g;
	sumB += b;
	totalPixels++;
	/* Accumulate for region stats */
	regionSumY += yValue;
	regionSumR += r;
	regionSumG += g;
	regionSumB += b;
	regionPixelCount++;
	}
	}
	/* Calculate region means */
	if (regionPixelCount > 0) {
	auto& region = stats.regions[ry * regionsX + rx];
	region.meanY = regionSumY / regionPixelCount;
	region.meanR = regionSumR / regionPixelCount;
	region.meanG = regionSumG / regionPixelCount;
	region.meanB = regionSumB / regionPixelCount;
	/* Determine if the region is neutral for AWB */
	region.isNeutral = isNeutralRegion(
	region.meanR, region.meanG, region.meanB);
	}
	}
	}
	/* Calculate global means */
	if (totalPixels > 0) {
	stats.meanY = static_cast<float>(sumY) / totalPixels;
	stats.meanR = static_cast<float>(sumR) / totalPixels;
	stats.meanG = static_cast<float>(sumG) / totalPixels;
	stats.meanB = static_cast<float>(sumB) / totalPixels;
	}
}

void generateStatsFromBayer(const uint8_t* data, const ImageInfo& info, ImageStats& stats)
{
	const unsigned int width = info.width;
	const unsigned int height = info.height;

	/* Clear histograms */
	stats.yHistogram.fill(0);
	stats.rHistogram.fill(0);
	stats.gHistogram.fill(0);
	stats.bHistogram.fill(0);

	/* Reset statistics */
	stats.meanY = stats.meanR = stats.meanG = stats.meanB = 0.0f;
	stats.minY = stats.minR = stats.minG = stats.minB = 255;
	stats.maxY = stats.maxR = stats.maxG = stats.maxB = 0;

	/* Region size for statistics */
	constexpr int regionSize = 16;
	const int regionsX = (width + regionSize - 1) / regionSize;
	const int regionsY = (height + regionSize - 1) / regionSize;

	/* Prepare regions */
	stats.regions.resize(regionsX * regionsY);

	/* Accumulators for mean calculation */
	uint64_t sumR = 0, sumG1 = 0, sumG2 = 0, sumB = 0;
	uint32_t countR = 0, countG1 = 0, countG2 = 0, countB = 0;

	/* Process each region */
	for (int ry = 0; ry < regionsY; ry++) {
	for (int rx = 0; rx < regionsX; rx++) {
	/* Region boundaries */
	int startX = rx * regionSize;
	int startY = ry * regionSize;
	int endX = std::min(startX + regionSize, static_cast<int>(width));
	int endY = std::min(startY + regionSize, static_cast<int>(height));

	/* Accumulators for this region */
	uint64_t regionSumR = 0, regionSumG = 0, regionSumB = 0, regionSumY = 0;
	int regionCountR = 0, regionCountG = 0, regionCountB = 0, regionPixelCount = 0;

	/* Process pixels in this region, handling the RGGB pattern */
	for (int y = startY; y < endY; y++) {
	for (int x = startX; x < endX; x++) {
	/* Calculate index for packed 10-bit data (assuming 16-bit packing) */
	size_t idx = (y * width + x) * 2; /* 2 bytes per pixel */

	/* Extract 10-bit value packed in 16 bits */
	uint16_t pixel = data[idx] | (data[idx + 1] << 8);
	uint16_t value = pixel & 0x03FF; /* Mask to get 10 bits */

	/* Scale to 8-bit for histograms and processing */
	uint8_t value8bit = value >> 2; /* 10-bit to 8-bit */

	/* Determine pixel type based on Bayer pattern (RGGB) */
	bool isRed = (y % 2 == 0) && (x % 2 == 0);
	bool isBlue = (y % 2 == 1) && (x % 2 == 1);
	bool isGreen = !isRed && !isBlue;
	bool isGreen1 = (y % 2 == 0) && (x % 2 == 1); /* Green in red row */
	/* bool isGreen2 = (y % 2 == 1) && (x % 2 == 0); // Green in blue row */

	/* Update appropriate histogram and statistics */
	if (isRed) {
	stats.rHistogram[value8bit]++;
	sumR += value;
	countR++;
	regionSumR += value;
	regionCountR++;

	/* Track min/max */
	stats.minR = std::min(stats.minR, value8bit);
	stats.maxR = std::max(stats.maxR, value8bit);
	}
	else if (isBlue) {
	stats.bHistogram[value8bit]++;
	sumB += value;
	countB++;
	regionSumB += value;
	regionCountB++;

	/* Track min/max */
	stats.minB = std::min(stats.minB, value8bit);
	stats.maxB = std::max(stats.maxB, value8bit);
	}
	else if (isGreen) {
	stats.gHistogram[value8bit]++;

	if (isGreen1) {
	sumG1 += value;
	countG1++;
	} else { /* isGreen2 */
	sumG2 += value;
	countG2++;
	}

	regionSumG += value;
	regionCountG++;

	/* Track min/max */
	stats.minG = std::min(stats.minG, value8bit);
	stats.maxG = std::max(stats.maxG, value8bit);
	}

	/* Estimate luminance (simple average) */
	/* In a real implementation, you might want to use proper weights */
	regionSumY += value;
	regionPixelCount++;

	/* Update Y histogram (using simple average for Y) */
	stats.yHistogram[value8bit]++;
	}
	}

	/* Calculate region means */
	if (regionPixelCount > 0) {
	auto& region = stats.regions[ry * regionsX + rx];

	region.meanR = regionCountR > 0 ? static_cast<float>(regionSumR) / regionCountR : 0;
	region.meanG = regionCountG > 0 ? static_cast<float>(regionSumG) / regionCountG : 0;
	region.meanB = regionCountB > 0 ? static_cast<float>(regionSumB) / regionCountB : 0;
	region.meanY = static_cast<float>(regionSumY) / regionPixelCount;

	/* Determine if the region is neutral (for AWB) */
	region.isNeutral = isNeutralRegion(
	region.meanR, region.meanG, region.meanB);
	}
	}
	}

	/* Calculate global means */
	uint32_t countG = countG1 + countG2;
	uint64_t sumG = sumG1 + sumG2;

	if (countR > 0)
	stats.meanR = static_cast<float>(sumR) / countR;
	if (countG > 0)
	stats.meanG = static_cast<float>(sumG) / countG;
	if (countB > 0)
	stats.meanB = static_cast<float>(sumB) / countB;

	/* Calculate overall luminance (simplified) */
	stats.meanY = (0.2126f * stats.meanR + 0.7152f * stats.meanG + 0.0722f * stats.meanB);

	/* Min/max for Y (estimate from RGB) */
	stats.minY = std::min({stats.minR, stats.minG, stats.minB});
	stats.maxY = std::max({stats.maxR, stats.maxG, stats.maxB});

}

void convertYUVtoRGB(uint8_t y, uint8_t u, uint8_t v,
	uint8_t& r, uint8_t& g, uint8_t& b)
{
	int c = y - 16;
	int d = u - 128;
	int e = v - 128;
	int r_temp = (298 * c + 409 * e + 128) >> 8;
	int g_temp = (298 * c - 100 * d - 208 * e + 128) >> 8;
	int b_temp = (298 * c + 516 * d + 128) >> 8;
	r = std::clamp(r_temp, 0, 255);
	g = std::clamp(g_temp, 0, 255);
	b = std::clamp(b_temp, 0, 255);
}
void convertRGBtoYUV(uint8_t r, uint8_t g, uint8_t b,
	uint8_t& y, uint8_t& u, uint8_t& v)
{
	int y_temp = (77 * r + 150 * g + 29 * b + 128) >> 8;
	int u_temp = ((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128;
	int v_temp = ((128 * r - 107 * g - 21 * b + 128) >> 8) + 128;
	y = std::clamp(y_temp, 0, 255);
	u = std::clamp(u_temp, 0, 255);
	v = std::clamp(v_temp, 0, 255);
}
bool isNeutralRegion(float r, float g, float b,
	float saturationThreshold,
	float minValue)
{
	float min_val = std::min({r, g, b});
	float max_val = std::max({r, g, b});
	if (max_val < minValue)
	return false;  /* Too dark */
	if (max_val == 0)
	return false;  /* Avoid division by zero */
	float saturation = (max_val - min_val) / max_val;
	return (saturation < saturationThreshold);
}
} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
