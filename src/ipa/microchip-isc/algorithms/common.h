/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Professional Image Processing Algorithm Common Definitions
 */
#ifndef __LIBCAMERA_IPA_MICROCHIP_ISC_COMMON_H__
#define __LIBCAMERA_IPA_MICROCHIP_ISC_COMMON_H__

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <span>
#include <memory>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>

/* Forward declarations */
namespace libcamera {
class ControlList;
}

namespace libcamera {
namespace ipa::microchip_isc {

/* Bayer pattern enumeration */
enum class BayerPattern {
	RGGB = 0,
	GRBG,
	GBRG,
	BGGR
};

/* Image information structure */
struct ImageInfo {
	uint32_t width;
	uint32_t height;
	uint32_t format;
	BayerPattern bayerPattern;
};

/* Hardware-specific structures */
struct isc_stat_buffer {
    uint32_t frame_number;
    uint64_t timestamp;
    uint32_t meas_type;
    struct {
        uint32_t hist_bins[512];
        uint32_t hist_min;
        uint32_t hist_max;
        uint32_t total_pixels;
    } hist[4];
    uint8_t valid_channels;
    uint8_t bayer_pattern;
    uint16_t reserved[2];
} __attribute__((packed));

/* Unified image statistics structure */
struct ImageStats {
	/* Bayer channel statistics */
	float meanGR, meanR, meanGB, meanB;
	uint32_t totalPixelsGR, totalPixelsR, totalPixelsGB, totalPixelsB;
	/* Histogram data */
	std::array<uint32_t, 512> histogramGR;
	std::array<uint32_t, 512> histogramR;
	std::array<uint32_t, 512> histogramGB;
	std::array<uint32_t, 512> histogramB;
	/* Quality metrics */
	float overexposureRatio;
	float underexposureRatio;
	float contrast;
	/* Validation flag */
	bool isValid;
};

	/* Metadata */
	BayerPattern bayerPattern;
	uint8_t validChannels;  /* Bitmask: 0x0F = all valid */
	bool hasSoftwareStats;  /* True if from software processing */

	/* Quality indicators */
	float noiseEstimate;
	float sharpnessMetric;
	uint32_t overexposedPixels;
	uint32_t underexposedPixels;
};

/* Image format info for software processing */
struct ImageInfo {
	uint32_t width;
	uint32_t height;
	uint32_t format;
	BayerPattern bayerPattern;
};

/* AWB context structure */
struct AWBContext {
	std::string selectedAlgorithm;
	float confidence;
	bool isConverged;
	float stabilityMetric;
	float dominantColorTemperature;
	float chromaticityShift;
	/* Scene characteristics */
	bool fluorescentDetected;
	bool incandescentDetected;
	bool daylightDetected;
	float greenCastStrength;
	float chromaticityShift;
	float dominantColorTemperature;
	bool isConverged;
	float stabilityMetric;
	uint32_t convergenceFrames;
};

/* White balance result structure */
struct WhiteBalanceResult {
	std::string algorithmName;
	float colorTemperatureEstimate;
	float algorithmConfidence;
	float redGain, greenRedGain, greenBlueGain, blueGain;
	int32_t redOffset, greenRedOffset, greenBlueOffset, blueOffset;
	float chromaticityCorrection;
	float stabilityMetric;
};

/* Exposure result structure */
struct ExposureResult {
	uint32_t exposureTime;
	uint32_t analogueGain;
	uint32_t digitalGain;
	float targetLuminance;
	float actualLuminance;
	std::string exposureStrategy;
	bool requiresHDR;
};

/* Forward declarations */
class UnifiedSceneAnalyzer;

/* Utility functions */
bool validateImageStats(const ImageStats &stats);
void calculateMeansFromHistograms(ImageStats &imageStats);
void convertHardwareStatsToImageStats(const libcamera::ControlList &hwStats, ImageStats &imageStats);
float calculateLuminanceFromBayer(const ImageStats &stats);
float calculateContrastFromBayer(const ImageStats &stats);
bool detectOverexposure(const ImageStats &stats, float threshold = 0.02f);
bool detectUnderexposure(const ImageStats &stats, float threshold = 0.15f);

/* Histogram analysis functions */
float calculatePercentile(const std::array<uint32_t, 512> &histogram, float percentile);
uint32_t findHistogramPeak(const std::array<uint32_t, 512> &histogram, int startBin = 0, int endBin = 511);
float calculateHistogramVariance(const std::array<uint32_t, 512> &histogram);
float calculateHistogramEntropy(const std::array<uint32_t, 512> &histogram);
uint32_t countHistogramPeaks(const std::array<uint32_t, 512> &histogram);
float calculateHighlightRatio(const ImageStats &stats);

/* Software processing functions */
void generateStatsFromFormat(const uint8_t *data, const ImageInfo &info, ImageStats &stats);
void dumpImageStats(const ImageStats &stats, const std::string &prefix);

/* Color space and illumination estimation */
float estimateColorTemperature(const ImageStats &stats);
float calculateNeutralPointConfidence(const ImageStats &stats);
float calculateContrastRatio(const ImageStats &stats);
float calculateChromaticVariance(const ImageStats &stats);
float calculateShadowDistribution(const ImageStats &stats);
float calculateHighlightDistribution(const ImageStats &stats);
float calculateSaturationLevel(const ImageStats &stats);

/* Software processing functions */
void generateStatsFromFormat(const uint8_t *data, const ImageInfo &info, ImageStats &stats);
void dumpImageStats(const ImageStats &stats, const char *prefix);

/* Hardware-specific structures */
struct isc_stat_buffer {
    uint32_t frame_number;
    uint64_t timestamp;
    uint32_t meas_type;
    struct {
        uint32_t hist_bins[512];
        uint32_t hist_min;
        uint32_t hist_max;
        uint32_t total_pixels;
    } hist[4];
    uint8_t valid_channels;
    uint8_t bayer_pattern;
    uint16_t reserved[2];
} __attribute__((packed));

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_MICROCHIP_ISC_COMMON_H__ */
