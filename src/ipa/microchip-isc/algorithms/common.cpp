/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Professional Image Processing Algorithm Common Utilities with Spatial Zone Analysis
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

/* Enhanced UnifiedSceneAnalyzer Implementation with Spatial Analysis */

UnifiedSceneAnalyzer::UnifiedSceneAnalyzer()
	: previousOutdoorState_(false),
	consecutiveOutdoorFrames_(0),
	consecutiveIndoorFrames_(0)
{
}

UnifiedSceneAnalysis UnifiedSceneAnalyzer::analyzeScene(const ImageStats &stats)
{
	UnifiedSceneAnalysis analysis = {};

	/* Step 1: Perform spatial zone analysis */
	analysis.spatialAnalysis = performSpatialZoneAnalysis(stats);

	/* Step 2: Detect and handle washout scenarios */
	analysis.washoutAnalysis = analyzeWashoutPatterns(analysis.spatialAnalysis, stats);

	/* Step 3: Basic scene characteristics from existing code */
	float luminance = calculateLuminanceFromBayer(stats);
	float contrast = calculateContrastFromBayer(stats);
	analysis.illuminationStrength = luminance;
	analysis.dynamicRange = contrast;
	analysis.chromaticityShift = calculateChromaticityShift(stats);
	analysis.hasSkinTonesDetected = detectSkinTones(stats);

	/* Step 4: Enhanced scene classification using spatial analysis */
	analysis.colorTemperature = analysis.spatialAnalysis.dominantCCT;
	analysis.lightSource = analyzeLightSourceWithSpatial(stats, analysis.spatialAnalysis);
	analysis.lightSourceConfidence = analysis.spatialAnalysis.sceneConfidence;
	analysis.environment = analyzeEnvironmentWithSpatial(stats, analysis.spatialAnalysis);
	analysis.environmentConfidence = analysis.spatialAnalysis.isOutdoorScene ? 0.9f : 0.7f;

	/* Step 5: Keep existing exposure analysis but enhance with spatial data */
	analysis.exposureComplexity = analyzeExposureComplexity(stats);
	analysis.requiresHDRProcessing = requiresHDRProcessing(stats);

	/* Step 6: Add spatial-specific parameters to fix issues */
	analysis.requiresGentleCorrection = analysis.spatialAnalysis.isMixedLighting ||
		(analysis.lightSourceConfidence < 0.8f);
	analysis.mixedLightingRatio = calculateMixedLightingRatio(analysis.spatialAnalysis);

	/* Step 7: Enhanced confidence calculation using spatial data */
	analysis.overallConfidence = calculateOverallConfidenceWithSpatial(analysis);

	LOG(ISC_COMMON, Info) << "Enhanced Scene Analysis: "
		<< lightSourceTypeToString(analysis.lightSource)
		<< " (" << analysis.colorTemperature << "K) "
		<< "Mixed=" << analysis.spatialAnalysis.isMixedLighting
		<< " Outdoor=" << analysis.spatialAnalysis.isOutdoorScene
		<< " Confidence=" << analysis.overallConfidence;

	return analysis;
}

SpatialZoneAnalysis UnifiedSceneAnalyzer::performSpatialZoneAnalysis(const ImageStats &stats)
{
	SpatialZoneAnalysis spatial = {};

	LOG(ISC_COMMON, Debug) << "Starting spatial zone analysis";

	/* Analyze each spatial zone using histogram patterns */
	spatial.topZone = analyzeZoneFromHistogram(stats, ZoneType::TOP_ZONE);
	spatial.centerZone = analyzeZoneFromHistogram(stats, ZoneType::CENTER_ZONE);
	spatial.bottomZone = analyzeZoneFromHistogram(stats, ZoneType::BOTTOM_ZONE);
	spatial.leftZone = analyzeZoneFromHistogram(stats, ZoneType::LEFT_ZONE);
	spatial.rightZone = analyzeZoneFromHistogram(stats, ZoneType::RIGHT_ZONE);

	/* Calculate cross-zone relationships */
	spatial.verticalGradient = calculateVerticalGradient(spatial);
	spatial.horizontalBalance = calculateHorizontalBalance(spatial);
	spatial.cctVariance = calculateCCTVariance(spatial);

	/* Detect spatial lighting patterns */
	spatial.isMixedLighting = detectMixedLightingPattern(spatial);
	spatial.isOutdoorScene = detectOutdoorLightingPattern(spatial);

	/* Determine dominant scene characteristics */
	spatial.dominantCCT = calculateDominantCCT(spatial);
	spatial.sceneConfidence = calculateSpatialConfidence(spatial);

	LOG(ISC_COMMON, Info) << "Spatial analysis complete: mixed=" << spatial.isMixedLighting
		<< " outdoor=" << spatial.isOutdoorScene
		<< " dominantCCT=" << spatial.dominantCCT;

	return spatial;
}

ZoneCharacteristics UnifiedSceneAnalyzer::analyzeZoneFromHistogram(
		const ImageStats &stats, ZoneType zone)
{
	ZoneCharacteristics zoneChar = {};

	/* Get zone-specific weighting patterns */
	ZoneWeights weights = getZoneWeights(zone);

	/* Calculate zone characteristics from weighted histograms */
	zoneChar.avgBrightness = calculateWeightedBrightness(stats, weights);
	zoneChar.variance = calculateWeightedVariance(stats, weights);
	zoneChar.peakPosition = findWeightedPeak(stats, weights);
	zoneChar.highlightRatio = calculateWeightedHighlights(stats, weights);
	zoneChar.shadowRatio = calculateWeightedShadows(stats, weights);
	zoneChar.hasClipping = detectWeightedClipping(stats, weights);

	/* Estimate CCT for this zone */
	zoneChar.cct = estimateZoneCCT(stats, weights);

	/* Calculate zone confidence */
	zoneChar.confidence = calculateZoneConfidence(zoneChar, stats);

	LOG(ISC_COMMON, Debug) << "Zone " << static_cast<int>(zone)
		<< ": bright=" << zoneChar.avgBrightness
		<< " cct=" << zoneChar.cct
		<< " conf=" << zoneChar.confidence;

	return zoneChar;
}

ZoneWeights UnifiedSceneAnalyzer::getZoneWeights(ZoneType zone)
{
	ZoneWeights weights = {};

	/* Initialize all weights to 1.0 by default */
	for (int i = 0; i < 512; i++) {
		weights.histogramWeights[i] = 1.0;
	}

	switch (zone) {
		case ZoneType::TOP_ZONE:
			/* Top zone: Sky/ceiling - emphasize bright regions */
			for (int i = 0; i < 512; i++) {
				if (i > 350) {
					weights.histogramWeights[i] = 2.0;      /* Strong weight for bright pixels */
				} else if (i > 200) {
					weights.histogramWeights[i] = 1.2;      /* Moderate weight for mid-tones */
				} else {
					weights.histogramWeights[i] = 0.4;      /* Low weight for shadows */
				}
			}
			weights.brightnessBias = 0.3;
			break;

		case ZoneType::CENTER_ZONE:
			/* Center zone: Main subject - emphasize mid-tones */
			for (int i = 0; i < 512; i++) {
				if (i >= 100 && i <= 400) {
					weights.histogramWeights[i] = 1.5;      /* Emphasize mid-range */
				} else {
					weights.histogramWeights[i] = 0.7;      /* De-emphasize extremes */
				}
			}
			weights.brightnessBias = 0.0;
			break;

		case ZoneType::BOTTOM_ZONE:
			/* Bottom zone: Ground/floor - emphasize shadows */
			for (int i = 0; i < 512; i++) {
				if (i < 150) {
					weights.histogramWeights[i] = 2.0;      /* Strong weight for shadows */
				} else if (i < 300) {
					weights.histogramWeights[i] = 1.2;      /* Moderate weight for mid-tones */
				} else {
					weights.histogramWeights[i] = 0.5;      /* Low weight for highlights */
				}
			}
			weights.brightnessBias = -0.3;
			break;

		case ZoneType::LEFT_ZONE:
		case ZoneType::RIGHT_ZONE:
			/* Side zones: Uniform weighting for side lighting detection */
			for (int i = 0; i < 512; i++) {
				weights.histogramWeights[i] = 1.0;
			}
			weights.brightnessBias = 0.0;
			break;
	}

	/* Channel weighting for CCT calculation */
	weights.channelWeights[0] = 1.3;  /* R channel - important for CCT */
	weights.channelWeights[1] = 1.0;  /* Gr channel */
	weights.channelWeights[2] = 1.0;  /* Gb channel */
	weights.channelWeights[3] = 1.3;  /* B channel - important for CCT */

	return weights;
}

double UnifiedSceneAnalyzer::calculateWeightedBrightness(const ImageStats &stats,
		const ZoneWeights &weights)
{
	/* Use green channels (most representative for luminance) */
	double grBrightness = calculateWeightedChannelAverage(
			stats.histogramGR, stats.totalPixelsGR, weights);
	double gbBrightness = calculateWeightedChannelAverage(
			stats.histogramGB, stats.totalPixelsGB, weights);

	/* Average the two green channels */
	double avgGreenBrightness = (grBrightness + gbBrightness) / 2.0;

	/* Apply brightness bias */
	avgGreenBrightness += weights.brightnessBias * 50.0;  /* Scale bias */

	return std::clamp(avgGreenBrightness, 0.0, 255.0);
}

double UnifiedSceneAnalyzer::calculateWeightedVariance(const ImageStats &stats,
		const ZoneWeights &weights)
{
	/* Calculate variance from green channels */
	double grVariance = calculateWeightedChannelVariance(
			stats.histogramGR, stats.totalPixelsGR, weights);
	double gbVariance = calculateWeightedChannelVariance(
			stats.histogramGB, stats.totalPixelsGB, weights);

	return (grVariance + gbVariance) / 2.0;
}

double UnifiedSceneAnalyzer::findWeightedPeak(const ImageStats &stats,
		const ZoneWeights &weights)
{
	/* Find peak in weighted green channel histogram */
	double maxWeightedCount = 0.0;
	int peakBin = 0;

	for (int i = 0; i < 512; i++) {
		double weightedCount = (stats.histogramGR[i] + stats.histogramGB[i]) *
			weights.histogramWeights[i];
		if (weightedCount > maxWeightedCount) {
			maxWeightedCount = weightedCount;
			peakBin = i;
		}
	}

	return static_cast<double>(peakBin * 255) / 511.0;  /* Convert to 0-255 range */
}

double UnifiedSceneAnalyzer::calculateWeightedHighlights(const ImageStats &stats,
		const ZoneWeights &weights)
{
	double totalWeightedPixels = 0.0;
	double highlightWeightedPixels = 0.0;

	for (int i = 0; i < 512; i++) {
		double pixelCount = stats.histogramGR[i] + stats.histogramGB[i];
		double weightedCount = pixelCount * weights.histogramWeights[i];

		totalWeightedPixels += weightedCount;

		/* Count highlights (top 25% of histogram) */
		if (i > 384) {  /* 384/512 = 75% */
			highlightWeightedPixels += weightedCount;
		}
	}

	return (totalWeightedPixels > 0) ? (highlightWeightedPixels / totalWeightedPixels) : 0.0;
}

double UnifiedSceneAnalyzer::calculateWeightedShadows(const ImageStats &stats,
		const ZoneWeights &weights)
{
	double totalWeightedPixels = 0.0;
	double shadowWeightedPixels = 0.0;

	for (int i = 0; i < 512; i++) {
		double pixelCount = stats.histogramGR[i] + stats.histogramGB[i];
		double weightedCount = pixelCount * weights.histogramWeights[i];

		totalWeightedPixels += weightedCount;

		/* Count shadows (bottom 25% of histogram) */
		if (i < 128) {  /* 128/512 = 25% */
			shadowWeightedPixels += weightedCount;
		}
	}

	return (totalWeightedPixels > 0) ? (shadowWeightedPixels / totalWeightedPixels) : 0.0;
}

bool UnifiedSceneAnalyzer::detectWeightedClipping(const ImageStats &stats,
		const ZoneWeights &weights)
{
	double totalWeightedPixels = 0.0;
	double clippedWeightedPixels = 0.0;

	/* Check top 2% of histogram for clipping */
	for (int i = 500; i < 512; i++) {
		double pixelCount = stats.histogramGR[i] + stats.histogramGB[i];
		clippedWeightedPixels += pixelCount * weights.histogramWeights[i];
	}

	for (int i = 0; i < 512; i++) {
		double pixelCount = stats.histogramGR[i] + stats.histogramGB[i];
		totalWeightedPixels += pixelCount * weights.histogramWeights[i];
	}

	double clippingRatio = (totalWeightedPixels > 0) ?
		(clippedWeightedPixels / totalWeightedPixels) : 0.0;

	return clippingRatio > 0.02;  /* 2% threshold for clipping */
}

double UnifiedSceneAnalyzer::estimateZoneCCT(const ImageStats &stats,
		const ZoneWeights &weights)
{
	double weightedR = calculateWeightedChannelAverage(
			stats.histogramR, stats.totalPixelsR, weights);
	double weightedB = calculateWeightedChannelAverage(
			stats.histogramB, stats.totalPixelsB, weights);

	if (weightedB < 0.1) weightedB = 0.1;  /* Prevent division by zero */

	double rbRatio = weightedR / weightedB;

	/* Enhanced CCT estimation with zone-specific calibration */
	double cct;
	if (rbRatio > 1.7) {
		cct = 2200 + (2.5 - rbRatio) * 600;      /* Very warm tungsten */
	} else if (rbRatio > 1.3) {
		cct = 2800 + (1.7 - rbRatio) * 1000;     /* Warm incandescent */
	} else if (rbRatio > 1.0) {
		cct = 3800 + (1.3 - rbRatio) * 1200;     /* Neutral fluorescent */
	} else if (rbRatio > 0.7) {
		cct = 5200 + (1.0 - rbRatio) * 1400;     /* Cool daylight */
	} else {
		cct = 6500 + (0.8 - rbRatio) * 1500;     /* Very cool */
	}

	return std::clamp(cct, 2000.0, 8500.0);
}

double UnifiedSceneAnalyzer::calculateZoneConfidence(const ZoneCharacteristics &zone,
		const ImageStats &stats)
{
	double confidence = 0.8;  /* Base confidence */

	/* Reduce confidence for extreme brightness */
	if (zone.avgBrightness < 10.0 || zone.avgBrightness > 245.0) {
		confidence *= 0.7;
	}

	/* Reduce confidence for very low variance (potential noise) */
	if (zone.variance < 100.0) {
		confidence *= 0.8;
	}

	/* Reduce confidence for clipping */
	if (zone.hasClipping) {
		confidence *= 0.9;
	}

	/* Boost confidence for good peak position */
	if (zone.peakPosition > 50.0 && zone.peakPosition < 200.0) {
		confidence *= 1.1;
	}

	/* Check total pixel count for reliability */
	uint32_t totalPixels = stats.totalPixelsGR + stats.totalPixelsR +
		stats.totalPixelsGB + stats.totalPixelsB;
	if (totalPixels < 10000) {
		confidence *= 0.6;
	}

	return std::clamp(confidence, 0.2, 0.95);
}

double UnifiedSceneAnalyzer::calculateWeightedChannelAverage(
		const std::array<uint32_t, 512> &histogram,
		uint32_t totalPixels,
		const ZoneWeights &weights)
{
	if (totalPixels == 0) return 0.0;

	double weightedSum = 0.0;
	double totalWeight = 0.0;

	for (int i = 0; i < 512; i++) {
		double binWeight = weights.histogramWeights[i];
		double binValue = static_cast<double>(i * 255) / 511.0;  /* Convert to 0-255 */

		weightedSum += histogram[i] * binValue * binWeight;
		totalWeight += histogram[i] * binWeight;
	}

	return (totalWeight > 0) ? (weightedSum / totalWeight) : 0.0;
}

double UnifiedSceneAnalyzer::calculateWeightedChannelVariance(
		const std::array<uint32_t, 512> &histogram,
		uint32_t totalPixels,
		const ZoneWeights &weights)
{
	if (totalPixels == 0) return 0.0;

	/* First calculate weighted mean */
	double weightedMean = calculateWeightedChannelAverage(histogram, totalPixels, weights);

	double varianceSum = 0.0;
	double totalWeight = 0.0;

	for (int i = 0; i < 512; i++) {
		double binValue = static_cast<double>(i * 255) / 511.0;
		double deviation = binValue - weightedMean;
		double binWeight = weights.histogramWeights[i];

		varianceSum += histogram[i] * deviation * deviation * binWeight;
		totalWeight += histogram[i] * binWeight;
	}

	return (totalWeight > 0) ? (varianceSum / totalWeight) : 0.0;
}

double UnifiedSceneAnalyzer::calculateVerticalGradient(const SpatialZoneAnalysis &spatial)
{
	if (spatial.topZone.confidence < 0.5 || spatial.bottomZone.confidence < 0.5) {
		return 0.0;
	}

	double topBrightness = spatial.topZone.avgBrightness;
	double bottomBrightness = spatial.bottomZone.avgBrightness;
	double avgBrightness = (topBrightness + bottomBrightness) / 2.0;

	if (avgBrightness < 0.01) return 0.0;

	return (topBrightness - bottomBrightness) / avgBrightness;
}

double UnifiedSceneAnalyzer::calculateHorizontalBalance(const SpatialZoneAnalysis &spatial)
{
	if (spatial.leftZone.confidence < 0.5 || spatial.rightZone.confidence < 0.5) {
		return 0.0;
	}

	double leftBrightness = spatial.leftZone.avgBrightness;
	double rightBrightness = spatial.rightZone.avgBrightness;
	double avgBrightness = (leftBrightness + rightBrightness) / 2.0;

	if (avgBrightness < 0.01) return 0.0;

	return std::abs(leftBrightness - rightBrightness) / avgBrightness;
}

double UnifiedSceneAnalyzer::calculateCCTVariance(const SpatialZoneAnalysis &spatial)
{
	std::vector<double> validCCTs;

	if (spatial.topZone.confidence > 0.6) validCCTs.push_back(spatial.topZone.cct);
	if (spatial.centerZone.confidence > 0.6) validCCTs.push_back(spatial.centerZone.cct);
	if (spatial.bottomZone.confidence > 0.6) validCCTs.push_back(spatial.bottomZone.cct);

	if (validCCTs.size() < 2) return 0.0;

	double meanCCT = 0.0;
	for (double cct : validCCTs) {
		meanCCT += cct;
	}
	meanCCT /= validCCTs.size();

	double variance = 0.0;
	for (double cct : validCCTs) {
		double deviation = cct - meanCCT;
		variance += deviation * deviation;
	}

	return variance / validCCTs.size();
}

bool UnifiedSceneAnalyzer::detectMixedLightingPattern(const SpatialZoneAnalysis &spatial)
{
	/* Mixed lighting indicators */
	double topBottomCCTDiff = std::abs(spatial.topZone.cct - spatial.bottomZone.cct);
	double leftRightCCTDiff = std::abs(spatial.leftZone.cct - spatial.rightZone.cct);
	double maxCCTDiff = std::max(topBottomCCTDiff, leftRightCCTDiff);

	bool hasHighCCTVariance = spatial.cctVariance > 300000;  /* 550K variance threshold */
	bool hasSignificantCCTDiff = maxCCTDiff > 600;  /* 600K difference between zones */
	bool hasMultipleValidZones = (spatial.topZone.confidence > 0.6) +
		(spatial.centerZone.confidence > 0.6) +
		(spatial.bottomZone.confidence > 0.6) >= 2;

	bool isMixed = hasHighCCTVariance && hasSignificantCCTDiff && hasMultipleValidZones;

	LOG(ISC_COMMON, Debug) << "Mixed lighting check: cctVar=" << spatial.cctVariance
		<< " maxDiff=" << maxCCTDiff
		<< " validZones=" << hasMultipleValidZones
		<< " -> mixed=" << isMixed;

	return isMixed;
}

bool UnifiedSceneAnalyzer::detectOutdoorLightingPattern(const SpatialZoneAnalysis &spatial)
{
	/* Get overall scene brightness for absolute checks */
	float overallBrightness = spatial.centerZone.avgBrightness;

	/* Outdoor characteristics - require minimum absolute brightness */
	bool hasVerticalGradient = spatial.verticalGradient > 0.35;
	bool hasBrightTop = spatial.topZone.avgBrightness > spatial.bottomZone.avgBrightness * 1.3 &&
		spatial.topZone.avgBrightness > 80.0;  /* Absolute brightness guard */
	bool hasTopVariance = spatial.topZone.variance > 1200;
	bool hasCoolTop = spatial.topZone.cct > kOutdoorCCTMinimum &&
		overallBrightness > 60.0;  /* Absolute brightness guard */
	bool hasGoodConfidence = spatial.topZone.confidence > 0.65 &&
		spatial.bottomZone.confidence > 0.65;
	bool noSevereClipping = !spatial.topZone.hasClipping;

	/* Strong indoor rejection criteria - tightened threshold */
	bool hasLowOverallBrightness = overallBrightness < 50.0;  /* Tightened from 70.0 */
	bool hasNeutralCCT = spatial.dominantCCT > 3800 && spatial.dominantCCT < 5200;
	bool hasLowVariance = spatial.centerZone.variance < 800;
	bool hasUniformLighting = spatial.cctVariance < 50000;

	/* Definite indoor - reject outdoor immediately */
	if (hasLowOverallBrightness && hasNeutralCCT && hasLowVariance && hasUniformLighting) {
		LOG(ISC_COMMON, Info) << "Strong indoor characteristics - rejecting outdoor: "
			<< "brightness=" << overallBrightness
			<< " CCT=" << spatial.dominantCCT
			<< " variance=" << spatial.centerZone.variance;
		consecutiveOutdoorFrames_ = 0;
		consecutiveIndoorFrames_++;
		previousOutdoorState_ = false;
		return false;
	}

	/* Count outdoor criteria met */
	int outdoorCriteria = hasVerticalGradient + hasBrightTop + hasTopVariance +
		hasCoolTop + hasGoodConfidence + noSevereClipping;
	float outdoorScore = static_cast<float>(outdoorCriteria) / 6.0f;

	/* Apply hysteresis - different thresholds based on previous state */
	float threshold;
	if (previousOutdoorState_) {
		threshold = kOutdoorStickyThreshold;
		LOG(ISC_COMMON, Debug) << "Hysteresis: was outdoor, staying outdoor needs "
			<< (threshold * 6) << "/6 criteria";
	} else {
		threshold = kOutdoorEntryThreshold;
		LOG(ISC_COMMON, Debug) << "Hysteresis: was indoor, becoming outdoor needs "
			<< (threshold * 6) << "/6 criteria";
	}

	bool currentOutdoorState = (outdoorScore >= threshold);

	/* Update consecutive frame counters */
	if (currentOutdoorState) {
		consecutiveOutdoorFrames_++;
		consecutiveIndoorFrames_ = 0;
	} else {
		consecutiveIndoorFrames_++;
		consecutiveOutdoorFrames_ = 0;
	}

	/* Require minimum consecutive frames before state change */
	if (currentOutdoorState != previousOutdoorState_) {
		if (currentOutdoorState && consecutiveOutdoorFrames_ < kMinFramesForStateChange) {
			LOG(ISC_COMMON, Info) << "Outdoor detected but awaiting confirmation ("
				<< consecutiveOutdoorFrames_ << "/"
				<< kMinFramesForStateChange << " frames)";
			return previousOutdoorState_;
		}
		if (!currentOutdoorState && consecutiveIndoorFrames_ < kMinFramesForStateChange) {
			LOG(ISC_COMMON, Info) << "Indoor detected but awaiting confirmation ("
				<< consecutiveIndoorFrames_ << "/"
				<< kMinFramesForStateChange << " frames)";
			return previousOutdoorState_;
		}
	}

	previousOutdoorState_ = currentOutdoorState;

	LOG(ISC_COMMON, Debug) << "Outdoor detection: gradient=" << spatial.verticalGradient
		<< " topBright=" << spatial.topZone.avgBrightness
		<< " bottomBright=" << spatial.bottomZone.avgBrightness
		<< " topCCT=" << spatial.topZone.cct
		<< " overallBright=" << overallBrightness
		<< " score=" << outdoorScore
		<< " threshold=" << threshold
		<< " criteria=" << outdoorCriteria << "/6"
		<< " -> outdoor=" << currentOutdoorState
		<< " (consecutive="
		<< (currentOutdoorState ? consecutiveOutdoorFrames_ : consecutiveIndoorFrames_)
		<< " frames)";

	return currentOutdoorState;
}

void UnifiedSceneAnalyzer::resetOutdoorDetectionState()
{
	previousOutdoorState_ = false;
	consecutiveOutdoorFrames_ = 0;
	consecutiveIndoorFrames_ = 0;
	LOG(ISC_COMMON, Info) << "Outdoor detection state reset";
}

double UnifiedSceneAnalyzer::calculateDominantCCT(const SpatialZoneAnalysis &spatial)
{
	double weightedSum = 0.0;
	double totalWeight = 0.0;

	/* Weight CCTs by zone importance and confidence */
	if (spatial.centerZone.confidence > 0.5) {
		double weight = spatial.centerZone.confidence * 1.5;  /* Center zone most important */
		weightedSum += spatial.centerZone.cct * weight;
		totalWeight += weight;
	}

	if (spatial.topZone.confidence > 0.5) {
		double weight = spatial.topZone.confidence;
		weightedSum += spatial.topZone.cct * weight;
		totalWeight += weight;
	}

	if (spatial.bottomZone.confidence > 0.5) {
		double weight = spatial.bottomZone.confidence;
		weightedSum += spatial.bottomZone.cct * weight;
		totalWeight += weight;
	}

	return (totalWeight > 0) ? (weightedSum / totalWeight) : 4500.0;  /* Default neutral CCT */
}

double UnifiedSceneAnalyzer::calculateSpatialConfidence(const SpatialZoneAnalysis &spatial)
{
	/* Average confidence across valid zones */
	std::vector<double> validConfidences;

	if (spatial.topZone.confidence > 0.3) validConfidences.push_back(spatial.topZone.confidence);
	if (spatial.centerZone.confidence > 0.3) validConfidences.push_back(spatial.centerZone.confidence);
	if (spatial.bottomZone.confidence > 0.3) validConfidences.push_back(spatial.bottomZone.confidence);

	if (validConfidences.empty()) return 0.4;  /* Low default confidence */

	double avgConfidence = 0.0;
	for (double conf : validConfidences) {
		avgConfidence += conf;
	}
	avgConfidence /= validConfidences.size();

	/* Boost confidence for clear patterns */
	if (spatial.isOutdoorScene) {
		avgConfidence = std::min(0.95, avgConfidence * 1.1);
	}

	/* Reduce confidence for mixed lighting */
	if (spatial.isMixedLighting) {
		avgConfidence *= 0.8;
	}

	return std::clamp(avgConfidence, 0.3, 0.95);
}

WashoutAnalysis UnifiedSceneAnalyzer::analyzeWashoutPatterns(
		const SpatialZoneAnalysis &spatial, const ImageStats &stats)
{
	WashoutAnalysis washout = {};

	/* FIXED: Actually use stats for washout validation */

	/* Calculate overall exposure metrics from histogram data */
	float overallLuminance = calculateLuminanceFromBayer(stats);
	float contrast = calculateContrastFromBayer(stats);
	bool hasOverexposure = detectOverexposure(stats, 0.02f);

	/* Check for clipping in different zones */
	bool topClipping = spatial.topZone.hasClipping;
	bool centerClipping = spatial.centerZone.hasClipping;
	bool overallHighlights = spatial.topZone.highlightRatio > 0.12 ||
		spatial.centerZone.highlightRatio > 0.12;

	/* Validate washout detection with histogram statistics */
	if (topClipping && spatial.isOutdoorScene) {
		/* Outdoor washout: Bright sky clipping - validate with histogram */
		if (hasOverexposure && overallLuminance > 180.0f) {
			washout.hasOutdoorWashout = true;
			washout.washoutLocation = ZoneType::TOP_ZONE;
			washout.washoutSeverity = std::min(1.0, spatial.topZone.highlightRatio * 2.5);
			LOG(ISC_COMMON, Info) << "Outdoor washout confirmed: luminance=" << overallLuminance
				<< " overexp=" << hasOverexposure;
		} else {
			/* Spatial suggests washout but histogram doesn't support it */
			washout.washoutSeverity = std::min(0.5, spatial.topZone.highlightRatio);
			LOG(ISC_COMMON, Debug) << "Mild outdoor washout: histogram doesn't fully support spatial analysis";
		}
	} else if (centerClipping || (overallHighlights && !spatial.isOutdoorScene)) {
		/* Indoor washout: Bright artificial lights - validate with histogram */
		if (hasOverexposure || contrast > 0.7f) {
			washout.hasIndoorWashout = true;
			washout.washoutLocation = centerClipping ? ZoneType::CENTER_ZONE : ZoneType::TOP_ZONE;
			washout.washoutSeverity = std::min(1.0, spatial.centerZone.highlightRatio * 3.0);
			LOG(ISC_COMMON, Info) << "Indoor washout confirmed: contrast=" << contrast
				<< " overexp=" << hasOverexposure;
		} else {
			/* Spatial suggests washout but histogram shows normal distribution */
			washout.washoutSeverity = std::min(0.3, spatial.centerZone.highlightRatio);
			LOG(ISC_COMMON, Debug) << "Mild indoor washout: normal histogram distribution";
		}
	}

	/* Use histogram data to determine if special handling is actually needed */
	washout.needsSpecialHandling = (washout.hasIndoorWashout || washout.hasOutdoorWashout) &&
		(hasOverexposure || overallLuminance > 200.0f);

	LOG(ISC_COMMON, Debug) << "Washout analysis: indoor=" << washout.hasIndoorWashout
		<< " outdoor=" << washout.hasOutdoorWashout
		<< " severity=" << washout.washoutSeverity
		<< " needsHandling=" << washout.needsSpecialHandling;

	return washout;
}

LightSourceType UnifiedSceneAnalyzer::analyzeLightSourceWithSpatial(
		const ImageStats &stats, const SpatialZoneAnalysis &spatial)
{
	/* Calculate key metrics from histogram data */
	float overallLuminance = calculateLuminanceFromBayer(stats);
	float avgGreen = (stats.meanGR + stats.meanGB) / 2.0f;
	float rbRatio = (avgGreen > 0) ? (stats.meanR / std::max(stats.meanB, 1.0f)) : 1.0f;

	/* === TIGHTENED: Localized daylight only for SEVERELY underexposed scenes === */
	bool hasLocalizedBrightness = (spatial.topZone.avgBrightness > 30.0) ||   /* Raised from 20 */
		(spatial.centerZone.avgBrightness > 20.0);  /* Raised from 15 */
	bool sceneIsUnderexposed = (overallLuminance < 25.0);  /* TIGHTENED from 40.0 */
	bool hasDaylightColorTemp = (spatial.dominantCCT >= 3900 && spatial.dominantCCT <= 5500);
	bool hasStrongContrast = (spatial.topZone.avgBrightness > overallLuminance * 2.5);  /* Raised from 1.8 */
	bool hasLowVarianceAcrossZones = (spatial.cctVariance < 500000);

	/* Only trigger for SEVERE underexposure with strong evidence */
	if (hasLocalizedBrightness && sceneIsUnderexposed && hasDaylightColorTemp &&
			hasStrongContrast && hasLowVarianceAcrossZones) {
		LOG(ISC_COMMON, Info) << "Localized daylight in SEVERELY underexposed scene: "
			<< "topBright=" << spatial.topZone.avgBrightness
			<< " centerBright=" << spatial.centerZone.avgBrightness
			<< " overall=" << overallLuminance
			<< " contrast=" << (spatial.topZone.avgBrightness / overallLuminance)
			<< "x (RARE case - camera severely underexposed)";
		return LightSourceType::DAYLIGHT;
	}

	/* Fast-path: Strong outdoor indicators even during hysteresis confirmation */
	bool strongOutdoorIndicators =
		(spatial.topZone.cct > kOutdoorCCTMinimum) &&
		(spatial.verticalGradient > 0.30) &&
		(spatial.topZone.avgBrightness > spatial.bottomZone.avgBrightness * 1.2) &&
		(overallLuminance > 150.0f || spatial.topZone.avgBrightness > 200.0f);

	/* Outdoor detection: confirmed state OR strong indicators during transition */
	if (spatial.isOutdoorScene || strongOutdoorIndicators) {
		if (overallLuminance > 120.0f && rbRatio < 1.2f) {
			LOG(ISC_COMMON, Info) << "Daylight confirmed: luminance=" << overallLuminance
				<< " rbRatio=" << rbRatio
				<< (strongOutdoorIndicators ? " (strong indicators)" : " (confirmed)");
			return LightSourceType::DAYLIGHT;
		} else if (overallLuminance < 80.0f) {
			LOG(ISC_COMMON, Debug) << "Outdoor but low luminance - cloudy conditions";
			return LightSourceType::CLOUDY;
		}
		return LightSourceType::DAYLIGHT;
	}

	/* Mixed lighting detection */
	if (spatial.isMixedLighting) {
		float contrast = calculateContrastFromBayer(stats);
		if (contrast > 0.5f) {
			LOG(ISC_COMMON, Debug) << "Mixed lighting confirmed by high contrast: " << contrast;
			return LightSourceType::MIXED_SOURCES;
		}
	}

	/* Single lighting source classification */
	double dominantCCT = spatial.dominantCCT;

	if (dominantCCT < 3200.0f) {
		/* Warm incandescent range */
		if (rbRatio > 1.4f) {
			LOG(ISC_COMMON, Debug) << "Incandescent confirmed: CCT=" << dominantCCT
				<< " rbRatio=" << rbRatio;
			return LightSourceType::INCANDESCENT;
		} else if (spatial.centerZone.variance > 1500) {
			LOG(ISC_COMMON, Debug) << "Warm CCT but high variance - fluorescent misclassified";
			return LightSourceType::FLUORESCENT;
		}
		return LightSourceType::INCANDESCENT;

	} else if (dominantCCT >= 3800.0f && dominantCCT <= 5200.0f) {
		/* Critical decision zone: LED vs Fluorescent */
		double ledScore = 0.0;
		double fluorescentScore = 0.0;

		/* Low variance strongly indicates LED (no flicker) */
		if (spatial.centerZone.variance < 500) {
			ledScore += 3.0;
		} else if (spatial.centerZone.variance < 800) {
			ledScore += 1.5;
		} else if (spatial.centerZone.variance > 1200) {
			fluorescentScore += 2.0;
		}

		/* Consistent CCT across zones indicates LED */
		if (spatial.cctVariance < 80000) {
			ledScore += 2.0;
		} else if (spatial.cctVariance > 250000) {
			fluorescentScore += 2.0;
		}

		/* Neutral R/B ratio indicates LED */
		if (rbRatio > 0.95f && rbRatio < 1.15f) {
			ledScore += 1.5;
		}

		/* Low overall luminance with moderate variance can indicate fluorescent */
		if (overallLuminance < 50.0f && spatial.centerZone.variance > 800) {
			fluorescentScore += 1.0;
		}

		/* Make decision based on scores */
		if (ledScore > fluorescentScore + 1.0) {
			LOG(ISC_COMMON, Info) << "LED detected: score=" << ledScore
				<< " vs fluorescent=" << fluorescentScore
				<< " (CCT=" << dominantCCT
				<< ", variance=" << spatial.centerZone.variance << ")";
			return LightSourceType::LED;
		} else if (fluorescentScore > ledScore) {
			LOG(ISC_COMMON, Info) << "Fluorescent detected: score=" << fluorescentScore
				<< " vs LED=" << ledScore
				<< " (CCT=" << dominantCCT
				<< ", variance=" << spatial.centerZone.variance << ")";
			return LightSourceType::FLUORESCENT;
		} else {
			LOG(ISC_COMMON, Debug) << "Borderline case - defaulting to LED (modern lighting)";
			return LightSourceType::LED;
		}

	} else if (dominantCCT > 6000.0f) {
		/* Cool daylight range */
		if (rbRatio < 0.9f && overallLuminance > 100.0f) {
			return LightSourceType::DAYLIGHT;
		} else if (spatial.centerZone.variance < 800) {
			return LightSourceType::LED;
		}
		return LightSourceType::DAYLIGHT;

	} else {
		/* 3200-3800K range */
		if (spatial.sceneConfidence < 0.6 || overallLuminance < 10.0f) {
			return LightSourceType::UNKNOWN_SOURCE;
		}
		return spatial.centerZone.variance < 800 ? LightSourceType::LED : LightSourceType::FLUORESCENT;
	}
}

EnvironmentType UnifiedSceneAnalyzer::analyzeEnvironmentWithSpatial(
		const ImageStats &stats, const SpatialZoneAnalysis &spatial)
{
	/* Calculate environmental indicators from histogram */
	float overallLuminance = calculateLuminanceFromBayer(stats);
	float contrast = calculateContrastFromBayer(stats);
	bool hasOverexposure = detectOverexposure(stats, 0.02f);

	/* Use spatial patterns for environment classification */
	if (spatial.isOutdoorScene) {
		/* Validate outdoor with histogram characteristics */
		if (overallLuminance > 120.0f && (contrast > 0.5f || hasOverexposure)) {
			LOG(ISC_COMMON, Debug) << "Outdoor confirmed: bright=" << overallLuminance
				<< " contrast=" << contrast << " overexp=" << hasOverexposure;
			return EnvironmentType::OUTDOOR;
		} else if (overallLuminance < 80.0f) {
			/* Spatial says outdoor but histogram shows indoor-like levels */
			LOG(ISC_COMMON, Debug) << "Spatial outdoor but low histogram brightness - mixed environment";
			return EnvironmentType::INDOOR_MIXED;
		}
		return EnvironmentType::OUTDOOR;
	}

	if (spatial.isMixedLighting) {
		/* Validate mixed lighting with histogram distribution */
		if (contrast > 0.4f && overallLuminance > 50.0f) {
			LOG(ISC_COMMON, Debug) << "Indoor mixed confirmed: contrast=" << contrast
				<< " luminance=" << overallLuminance;
			return EnvironmentType::INDOOR_MIXED;
		} else if (overallLuminance < 60.0f) {
			/* Mixed spatial but low overall brightness */
			LOG(ISC_COMMON, Debug) << "Mixed spatial but low brightness - artificial dominant";
			return EnvironmentType::INDOOR_ARTIFICIAL;
		}
		return EnvironmentType::INDOOR_MIXED;
	}

	/* Single lighting source - use histogram to distinguish artificial vs mixed */
	if (overallLuminance < 70.0f && contrast < 0.4f) {
		/* Low, uniform lighting suggests artificial */
		LOG(ISC_COMMON, Debug) << "Low uniform lighting - indoor artificial";
		return EnvironmentType::INDOOR_ARTIFICIAL;
	} else if (overallLuminance > 90.0f && contrast > 0.3f) {
		/* Higher brightness with some contrast suggests mixed sources */
		LOG(ISC_COMMON, Debug) << "Moderate brightness with contrast - indoor mixed";
		return EnvironmentType::INDOOR_MIXED;
	}

	/* Default based on overall brightness */
	return (overallLuminance < 80.0f) ? EnvironmentType::INDOOR_ARTIFICIAL : EnvironmentType::INDOOR_MIXED;
}

double UnifiedSceneAnalyzer::calculateMixedLightingRatio(const SpatialZoneAnalysis &spatial)
{
	if (!spatial.isMixedLighting) {
		return 1.0;  /* No mixing - full correction strength */
	}

	/* Calculate dominant vs secondary lighting ratio */
	double topCCT = spatial.topZone.cct;
	double centerCCT = spatial.centerZone.cct;
	double bottomCCT = spatial.bottomZone.cct;

	/* Find CCT range */
	double minCCT = std::min({topCCT, centerCCT, bottomCCT});
	double maxCCT = std::max({topCCT, centerCCT, bottomCCT});

	if (maxCCT - minCCT > 1000) {
		return 0.6;  /* Strong mixing - reduce correction to 60% */
	} else if (maxCCT - minCCT > 500) {
		return 0.8;  /* Moderate mixing - reduce correction to 80% */
	}

	return 0.9;  /* Mild mixing - reduce correction to 90% */
}

float UnifiedSceneAnalyzer::calculateOverallConfidenceWithSpatial(const UnifiedSceneAnalysis &analysis)
{
	/* Base confidence from spatial analysis */
	float spatialConfidence = analysis.spatialAnalysis.sceneConfidence;

	/* Boost confidence for clear patterns */
	if (analysis.spatialAnalysis.isOutdoorScene) {
		spatialConfidence = std::min(0.95f, spatialConfidence + 0.1f);
	}

	/* Reduce confidence for mixed lighting */
	if (analysis.spatialAnalysis.isMixedLighting) {
		spatialConfidence *= 0.8f;
	}

	/* Combine with light source and environment confidence */
	float combinedConfidence = (spatialConfidence +
			analysis.lightSourceConfidence +
			analysis.environmentConfidence) / 3.0f;

	return std::clamp(combinedConfidence, 0.3f, 0.95f);
}

/* Original scene analysis methods - keeping for compatibility */
LightSourceType UnifiedSceneAnalyzer::analyzeLightSource(const ImageStats &stats)
{
	float colorTemp = estimateColorTemperature(stats);
	if (colorTemp < 3200.0f) {
		return LightSourceType::INCANDESCENT;
	} else if (colorTemp >= 4000.0f && colorTemp <= 5000.0f) {
		return LightSourceType::FLUORESCENT;
	} else if (colorTemp > 6000.0f) {
		return LightSourceType::DAYLIGHT;
	} else {
		return LightSourceType::UNKNOWN_SOURCE;
	}
}

EnvironmentType UnifiedSceneAnalyzer::analyzeEnvironment(const ImageStats &stats)
{
	float luminance = calculateLuminanceFromBayer(stats);
	float contrast = calculateContrastFromBayer(stats);
	if (luminance > 150.0f && contrast > 0.6f) {
		return EnvironmentType::OUTDOOR;
	} else if (luminance < 80.0f) {
		return EnvironmentType::INDOOR_ARTIFICIAL;
	} else {
		return EnvironmentType::INDOOR_MIXED;
	}
}

ExposureComplexity UnifiedSceneAnalyzer::analyzeExposureComplexity(const ImageStats &stats)
{
	float contrast = calculateContrastFromBayer(stats);
	float luminance = calculateLuminanceFromBayer(stats);
	if (contrast > 0.8f) {
		return ExposureComplexity::HIGH_DYNAMIC_RANGE;
	} else if (luminance < 50.0f) {
		return ExposureComplexity::LOW_LIGHT;
	} else {
		return ExposureComplexity::UNIFORM;
	}
}

float UnifiedSceneAnalyzer::calculateColorTemperature(const ImageStats &stats)
{
	return estimateColorTemperature(stats);
}

float UnifiedSceneAnalyzer::calculateChromaticityShift(const ImageStats &stats)
{
	float avgGreen = (stats.meanGR + stats.meanGB) / 2.0f;
	float avgRB = (stats.meanR + stats.meanB) / 2.0f;
	if (avgRB > 0) {
		return std::abs(avgGreen - avgRB) / avgRB;
	}
	return 0.0f;
}

float UnifiedSceneAnalyzer::calculateDynamicRange(const ImageStats &stats)
{
	return calculateContrastFromBayer(stats);
}

float UnifiedSceneAnalyzer::calculateIlluminationStrength(const ImageStats &stats)
{
	return calculateLuminanceFromBayer(stats);
}

bool UnifiedSceneAnalyzer::detectSkinTones(const ImageStats &stats)
{
	float avgGreen = (stats.meanGR + stats.meanGB) / 2.0f;
	if (avgGreen > 0) {
		float rg_ratio = stats.meanR / avgGreen;
		return (rg_ratio > 1.1f && rg_ratio < 1.4f);
	}
	return false;
}

bool UnifiedSceneAnalyzer::requiresHDRProcessing(const ImageStats &stats)
{
	return calculateContrastFromBayer(stats) > 0.8f;
}

float UnifiedSceneAnalyzer::calculateOverallConfidence(const UnifiedSceneAnalysis &analysis)
{
	return (analysis.lightSourceConfidence + analysis.environmentConfidence) / 2.0f;
}

/* Helper function implementations */
float estimateColorTemperature(const ImageStats &stats)
{
	float avgGreen = (stats.meanGR + stats.meanGB) / 2.0f;
	if (avgGreen == 0) return 5500.0f;
	float rg_ratio = stats.meanR / avgGreen;
	float bg_ratio = stats.meanB / avgGreen;
	float rb_ratio = rg_ratio / std::max(bg_ratio, 0.1f);
	if (rb_ratio > 1.3f) {
		return 2800.0f;
	} else if (rb_ratio < 0.8f) {
		return 6500.0f;
	} else {
		return 4200.0f;
	}
}

std::string lightSourceTypeToString(LightSourceType type)
{
	switch (type) {
		case LightSourceType::DAYLIGHT: return "Daylight";
		case LightSourceType::INCANDESCENT: return "Incandescent";
		case LightSourceType::FLUORESCENT: return "Fluorescent";
		case LightSourceType::LED: return "LED";
		case LightSourceType::FLASH: return "Flash";
		case LightSourceType::CLOUDY: return "Cloudy";
		case LightSourceType::SHADE: return "Shade";
		case LightSourceType::MIXED_SOURCES: return "Mixed";
		default: return "Unknown";
	}
}

std::string environmentTypeToString(EnvironmentType type)
{
	switch (type) {
		case EnvironmentType::OUTDOOR: return "Outdoor";
		case EnvironmentType::INDOOR_ARTIFICIAL: return "Indoor_Artificial";
		case EnvironmentType::INDOOR_MIXED: return "Indoor_Mixed";
		default: return "Unknown";
	}
}

std::string exposureComplexityToString(ExposureComplexity complexity)
{
	switch (complexity) {
		case ExposureComplexity::UNIFORM: return "Uniform";
		case ExposureComplexity::BACKLIGHT: return "Backlight";
		case ExposureComplexity::LOW_LIGHT: return "Low_Light";
		case ExposureComplexity::HIGH_DYNAMIC_RANGE: return "HDR";
		case ExposureComplexity::MIXED_LIGHTING: return "Mixed_Lighting";
		default: return "Unknown";
	}
}

uint32_t countHistogramPeaks(const std::array<uint32_t, 512> &histogram)
{
	uint32_t peakCount = 0;
	uint32_t totalPixels = 0;

	for (uint32_t count : histogram) {
		totalPixels += count;
	}

	if (totalPixels == 0) return 0;

	uint32_t threshold = totalPixels / 200;  /* Peak must be >0.5% of total */

	for (int i = 5; i < 507; i++) {  /* Avoid edges */
		if (histogram[i] > threshold) {
			/* Check if it's a local maximum */
			bool isLocalMax = true;
			for (int j = i-3; j <= i+3; j++) {
				if (histogram[j] > histogram[i]) {
					isLocalMax = false;
					break;
				}
			}
			if (isLocalMax) {
				peakCount++;
				i += 5;  /* Skip nearby bins */
			}
		}
	}

	return peakCount;
}

float calculateHighlightRatio(const ImageStats &stats)
{
	uint32_t totalPixels = stats.totalPixelsGR + stats.totalPixelsR +
		stats.totalPixelsGB + stats.totalPixelsB;
	uint32_t highlightPixels = 0;

	/* Count pixels in top 15% of histogram */
	for (int bin = 435; bin < 512; bin++) {  /* 435/512 = 85% */
		highlightPixels += stats.histogramGR[bin] + stats.histogramR[bin] +
			stats.histogramGB[bin] + stats.histogramB[bin];
	}

	return (totalPixels > 0) ? (static_cast<float>(highlightPixels) / totalPixels) : 0.0f;
}

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

