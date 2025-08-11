/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Professional Image Processing Algorithm Common Utilities
 */
#include "common.h"
#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace libcamera {
namespace ipa::microchip_isc {

LOG_DEFINE_CATEGORY(ISC_COMMON)

bool validateImageStats(const ImageStats &stats)
{
	/* Check for reasonable pixel counts */
	uint32_t totalPixels = stats.totalPixelsGR + stats.totalPixelsR +
		stats.totalPixelsGB + stats.totalPixelsB;
	if (totalPixels < 100) {  /* More lenient for early frames */
		LOG(ISC_COMMON, Warning) << "Low pixel count: " << totalPixels
			<< " (GR:" << stats.totalPixelsGR
			<< " R:" << stats.totalPixelsR
			<< " GB:" << stats.totalPixelsGB
			<< " B:" << stats.totalPixelsB << ")";
		return false;
	}
	/* Check channel means are reasonable */
	float channelMeans[4] = {stats.meanGR, stats.meanR, stats.meanGB, stats.meanB};
	const char* channelNames[4] = {"GR", "R", "GB", "B"};
	for (int c = 0; c < 4; c++) {
		if (channelMeans[c] < 0.0f || channelMeans[c] > 512.0f) {
			LOG(ISC_COMMON, Warning) << "Invalid channel mean " << channelNames[c]
				<< "[" << c << "]: " << channelMeans[c];
			return false;
		}
	}
	return stats.isValid;
}

void convertHardwareStatsToImageStats(const ControlList &hwStats, ImageStats &imageStats)
{
	/* Initialize defaults */
	imageStats = {};
	imageStats.isValid = false;
	if (!hwStats.contains(ISC_HISTOGRAM_DATA_ID)) {
		LOG(ISC_COMMON, Warning) << "No histogram data in ControlList";
		return;
	}
	/* Extract and validate the buffer */
	Span<const uint8_t> histData = hwStats.get(ISC_HISTOGRAM_DATA_ID).get<Span<const uint8_t>>();
	if (histData.size() < sizeof(isc_stat_buffer)) {
		LOG(ISC_COMMON, Error) << "Buffer too small: " << histData.size();
		return;
	}
	const isc_stat_buffer* hwBuffer = reinterpret_cast<const isc_stat_buffer*>(histData.data());
	/* Convert histogram data */
	for (int c = 0; c < 4; c++) {
		std::array<uint32_t, 512> *targetHist;
		switch (c) {
			case 0: targetHist = &imageStats.histogramGR; break;
			case 1: targetHist = &imageStats.histogramR; break;
			case 2: targetHist = &imageStats.histogramGB; break;
			case 3: targetHist = &imageStats.histogramB; break;
			default: continue;
		}
		/* Copy histogram */
		for (int bin = 0; bin < 512; bin++) {
			(*targetHist)[bin] = hwBuffer->hist[c].hist_bins[bin];
		}
	}
	/* Copy pixel totals */
	imageStats.totalPixelsGR = hwBuffer->hist[0].total_pixels;
	imageStats.totalPixelsR = hwBuffer->hist[1].total_pixels;
	imageStats.totalPixelsGB = hwBuffer->hist[2].total_pixels;
	imageStats.totalPixelsB = hwBuffer->hist[3].total_pixels;
	/* Calculate means from histograms */
	calculateMeansFromHistograms(imageStats);
	/* Calculate quality metrics */
	uint32_t totalPixels = imageStats.totalPixelsGR + imageStats.totalPixelsR +
		imageStats.totalPixelsGB + imageStats.totalPixelsB;
	if (totalPixels > 0) {
		uint32_t overexposed = 0, underexposed = 0;
		/* Count overexposed pixels (top 2% of histogram) */
		for (int bin = 501; bin < 512; bin++) {
			overexposed += imageStats.histogramGR[bin] + imageStats.histogramR[bin] +
				imageStats.histogramGB[bin] + imageStats.histogramB[bin];
		}
		/* Count underexposed pixels (bottom 15% of histogram) */
		for (int bin = 0; bin < 77; bin++) {
			underexposed += imageStats.histogramGR[bin] + imageStats.histogramR[bin] +
				imageStats.histogramGB[bin] + imageStats.histogramB[bin];
		}
		imageStats.overexposureRatio = static_cast<float>(overexposed) / totalPixels;
		imageStats.underexposureRatio = static_cast<float>(underexposed) / totalPixels;
		/* Calculate contrast */
		uint32_t shadows = 0, highlights = 0;
		for (int bin = 0; bin < 128; bin++) {
			shadows += imageStats.histogramGR[bin] + imageStats.histogramR[bin] +
				imageStats.histogramGB[bin] + imageStats.histogramB[bin];
		}
		for (int bin = 384; bin < 512; bin++) {
			highlights += imageStats.histogramGR[bin] + imageStats.histogramR[bin] +
				imageStats.histogramGB[bin] + imageStats.histogramB[bin];
		}
		imageStats.contrast = static_cast<float>(shadows + highlights) / totalPixels;
	}
	imageStats.isValid = true;
	LOG(ISC_COMMON, Debug) << "Converted: pixels="
		<< imageStats.totalPixelsGR << "," << imageStats.totalPixelsR
		<< "," << imageStats.totalPixelsGB << "," << imageStats.totalPixelsB
		<< " means=[" << imageStats.meanGR << "," << imageStats.meanR
		<< "," << imageStats.meanGB << "," << imageStats.meanB << "]";
}

void calculateMeansFromHistograms(ImageStats &imageStats)
{
	const std::array<uint32_t, 512>* histograms[4] = {
		&imageStats.histogramGR, &imageStats.histogramR,
		&imageStats.histogramGB, &imageStats.histogramB
	};
	float* means[4] = {
		&imageStats.meanGR, &imageStats.meanR,
		&imageStats.meanGB, &imageStats.meanB
	};
	const uint32_t* totals[4] = {
		&imageStats.totalPixelsGR, &imageStats.totalPixelsR,
		&imageStats.totalPixelsGB, &imageStats.totalPixelsB
	};
	for (int c = 0; c < 4; c++) {
		if (*totals[c] == 0) {
			*means[c] = 0.0f;
			continue;
		}
		uint64_t weightedSum = 0;
		for (int bin = 0; bin < 512; bin++) {
			/* Map bin index to luminance value (0-255) */
			uint32_t binValue = (bin * 255) / 511;
			weightedSum += (*histograms[c])[bin] * binValue;
		}
		*means[c] = static_cast<float>(weightedSum) / *totals[c];
	}
	LOG(ISC_COMMON, Debug) << "Calculated means: GR=" << imageStats.meanGR
		<< " R=" << imageStats.meanR
		<< " GB=" << imageStats.meanGB
		<< " B=" << imageStats.meanB;
}

void generateStatsFromFormat(const uint8_t *data, const ImageInfo &info, ImageStats &stats)
{
	(void)data;
	(void)info;
	(void)stats;
	/* TODO: Implement software histogram generation from image data */
	LOG(ISC_COMMON, Warning) << "Software stats generation not yet implemented";
}

void dumpImageStats(const ImageStats &stats, const std::string &prefix)
{
	LOG(ISC_COMMON, Info) << prefix << " ImageStats: "
		<< "GR[mean=" << stats.meanGR << " count=" << stats.totalPixelsGR << "] "
		<< "R[mean=" << stats.meanR << " count=" << stats.totalPixelsR << "] "
		<< "GB[mean=" << stats.meanGB << " count=" << stats.totalPixelsGB << "] "
		<< "B[mean=" << stats.meanB << " count=" << stats.totalPixelsB << "] "
		<< "valid=" << (stats.isValid ? "YES" : "NO");
}

float calculateLuminanceFromBayer(const ImageStats &stats)
{
	/* ITU-R BT.709 weights adapted for Bayer pattern */
	float luminance = 0.299f * stats.meanR +
		0.587f * (stats.meanGR + stats.meanGB) / 2.0f +
		0.114f * stats.meanB;
	return luminance;
}

float calculateContrastFromBayer(const ImageStats &stats)
{
	/* Use green channel for contrast calculation (most populated in Bayer) */
	uint32_t totalPixels = stats.totalPixelsGR;
	if (totalPixels == 0) return 0.0f;
	/* Calculate percentiles */
	uint32_t cumulativePixels = 0;
	uint32_t p10Value = 0, p90Value = 0;
	uint32_t target10 = totalPixels * 0.1f;
	uint32_t target90 = totalPixels * 0.9f;
	for (int bin = 0; bin < 512; bin++) {
		cumulativePixels += stats.histogramGR[bin];
		if (cumulativePixels >= target10 && p10Value == 0) {
			p10Value = (bin * 255) / 511;
		}
		if (cumulativePixels >= target90 && p90Value == 0) {
			p90Value = (bin * 255) / 511;
			break;
		}
	}
	return (p90Value - p10Value) / 255.0f;
}

bool detectOverexposure(const ImageStats &stats, float threshold)
{
	uint32_t overexposedPixels = 0;
	uint32_t totalPixels = stats.totalPixelsGR + stats.totalPixelsR +
		stats.totalPixelsGB + stats.totalPixelsB;
	/* Count pixels in top 2% of histogram (bins 501-511 for 512-bin) */
	for (int bin = 501; bin < 512; bin++) {
		overexposedPixels += stats.histogramGR[bin] + stats.histogramR[bin] +
			stats.histogramGB[bin] + stats.histogramB[bin];
	}
	float overexposureRatio = static_cast<float>(overexposedPixels) / std::max(totalPixels, 1u);
	return overexposureRatio > threshold;
}

bool detectUnderexposure(const ImageStats &stats, float threshold)
{
	uint32_t underexposedPixels = 0;
	uint32_t totalPixels = stats.totalPixelsGR + stats.totalPixelsR +
		stats.totalPixelsGB + stats.totalPixelsB;
	/* Count pixels in bottom 15% of histogram (bins 0-76 for 512-bin) */
	for (int bin = 0; bin < 77; bin++) {
		underexposedPixels += stats.histogramGR[bin] + stats.histogramR[bin] +
			stats.histogramGB[bin] + stats.histogramB[bin];
	}
	float underexposureRatio = static_cast<float>(underexposedPixels) / std::max(totalPixels, 1u);
	return underexposureRatio > threshold;
}

float calculatePercentile(const std::array<uint32_t, 512> &histogram, float percentile)
{
	uint32_t totalPixels = 0;
	for (uint32_t count : histogram) {
		totalPixels += count;
	}
	if (totalPixels == 0) return 0.0f;
	uint32_t targetPixels = static_cast<uint32_t>(totalPixels * percentile / 100.0f);
	uint32_t cumulativePixels = 0;
	for (int bin = 0; bin < 512; bin++) {
		cumulativePixels += histogram[bin];
		if (cumulativePixels >= targetPixels) {
			return static_cast<float>(bin * 255) / 511.0f;
		}
	}
	return 255.0f;
}

uint32_t findHistogramPeak(const std::array<uint32_t, 512> &histogram, int startBin, int endBin)
{
	uint32_t maxCount = 0;
	uint32_t peakBin = startBin;
	for (int bin = startBin; bin <= endBin && bin < 512; bin++) {
		if (histogram[bin] > maxCount) {
			maxCount = histogram[bin];
			peakBin = bin;
		}
	}
	return peakBin;
}

float calculateHistogramVariance(const std::array<uint32_t, 512> &histogram)
{
	uint32_t totalPixels = 0;
	float weightedSum = 0.0f;
	for (int bin = 0; bin < 512; bin++) {
		totalPixels += histogram[bin];
		weightedSum += bin * histogram[bin];
	}
	if (totalPixels == 0) return 0.0f;
	float mean = weightedSum / totalPixels;
	float variance = 0.0f;
	for (int bin = 0; bin < 512; bin++) {
		float deviation = bin - mean;
		variance += deviation * deviation * histogram[bin];
	}
	return variance / totalPixels;
}

float calculateHistogramEntropy(const std::array<uint32_t, 512> &histogram)
{
	uint32_t totalPixels = 0;
	for (uint32_t count : histogram) {
		totalPixels += count;
	}
	if (totalPixels == 0) return 0.0f;
	float entropy = 0.0f;
	for (uint32_t count : histogram) {
		if (count > 0) {
			float probability = static_cast<float>(count) / totalPixels;
			entropy -= probability * std::log2(probability);
		}
	}

	return entropy;
}

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
