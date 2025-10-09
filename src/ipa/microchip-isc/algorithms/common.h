/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Common definitions and utilities for Microchip ISC IPA with Spatial Zone Analysis
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

/* Light source types */
enum class LightSourceType {
	UNKNOWN_SOURCE = 0,
	DAYLIGHT,
	INCANDESCENT,
	FLUORESCENT,
	LED,
	FLASH,
	CLOUDY,
	SHADE,
	MIXED_SOURCES
};

/* Environment types */
enum class EnvironmentType {
	UNKNOWN_ENVIRONMENT = 0,
	OUTDOOR,
	INDOOR_ARTIFICIAL,
	INDOOR_MIXED
};

/* Exposure complexity levels */
enum class ExposureComplexity {
	UNIFORM = 0,
	BACKLIGHT,
	LOW_LIGHT,
	HIGH_DYNAMIC_RANGE,
	MIXED_LIGHTING
};

/* Convergence states */
enum class ConvergenceState {
	INITIALIZING = 0,
	CONVERGING,
	CONVERGED,
	FAILED
};

/* NEW: Spatial zone analysis structures */
enum class ZoneType {
    TOP_ZONE,       /* Sky/ceiling - 0-30% height */
    CENTER_ZONE,    /* Main subject - 30-70% height */
    BOTTOM_ZONE,    /* Ground/floor - 70-100% height */
    LEFT_ZONE,      /* Left side lighting */
    RIGHT_ZONE      /* Right side lighting */
};

struct ZoneCharacteristics {
    double avgBrightness;      /* Average brightness level */
    double cct;                /* Color temperature estimate */
    double variance;           /* Brightness distribution spread */
    double peakPosition;       /* Histogram peak location */
    double highlightRatio;     /* Percentage of bright pixels */
    double shadowRatio;        /* Percentage of dark pixels */
    bool hasClipping;          /* Severe overexposure detected */
    double confidence;         /* Zone analysis confidence */
};

struct ZoneWeights {
    double histogramWeights[512];  /* Weight for each histogram bin */
    double channelWeights[4];      /* Weight for R, Gr, Gb, B channels */
    double brightnessBias;         /* Bias toward bright/dark regions */
};

struct SpatialZoneAnalysis {
    ZoneCharacteristics topZone;
    ZoneCharacteristics centerZone;
    ZoneCharacteristics bottomZone;
    ZoneCharacteristics leftZone;
    ZoneCharacteristics rightZone;

    /* Cross-zone comparisons */
    double verticalGradient;   /* Brightness difference top-to-bottom */
    double horizontalBalance;  /* Left-right brightness balance */
    double cctVariance;        /* CCT variation across zones */
    bool isMixedLighting;      /* Multiple light sources detected */
    bool isOutdoorScene;       /* Outdoor lighting pattern detected */

    /* Overall scene characteristics */
    double dominantCCT;        /* Primary scene color temperature */
    double sceneConfidence;    /* Overall classification confidence */
};

struct WashoutAnalysis {
    bool hasIndoorWashout;     /* Bright indoor lights causing washout */
    bool hasOutdoorWashout;    /* Bright sky/sun causing washout */
    double washoutSeverity;    /* How severe the washout (0.0-1.0) */
    ZoneType washoutLocation;  /* Where washout is occurring */
    bool needsSpecialHandling; /* Requires special exposure/gain handling */
};

/* Enhanced unified scene analysis structure with spatial analysis */
struct UnifiedSceneAnalysis {
	/* Basic light source analysis */
	LightSourceType lightSource;
	float lightSourceConfidence;
	float colorTemperature;
	/* Environment classification */
	EnvironmentType environment;
	float environmentConfidence;
	/* Illumination characteristics */
	float illuminationStrength;
	float chromaticityShift;
	float dynamicRange;
	/* Exposure analysis */
	ExposureComplexity exposureComplexity;
	bool requiresHDRProcessing;
	/* Quality metrics */
	bool hasSkinTonesDetected;
	float overallConfidence;

	/* NEW: Spatial analysis results */
	SpatialZoneAnalysis spatialAnalysis;  /* Spatial zone analysis results */
	WashoutAnalysis washoutAnalysis;      /* Washout detection results */
	bool requiresGentleCorrection;        /* Needs reduced correction strength */
	double mixedLightingRatio;            /* Primary/secondary light ratio */
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
	float sceneBrightness;
	float outdoorConfidence;
	float cctReliability;
	bool suppressFluorescent;
	/* Convergence information */
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

/* String conversion functions */
std::string lightSourceTypeToString(LightSourceType type);
std::string environmentTypeToString(EnvironmentType type);
std::string exposureComplexityToString(ExposureComplexity complexity);

/* Enhanced Unified Scene Analyzer class with spatial analysis */
class UnifiedSceneAnalyzer {
public:
	UnifiedSceneAnalyzer();
	UnifiedSceneAnalysis analyzeScene(const ImageStats &stats);
	void resetOutdoorDetectionState();

private:
	/* Original scene analysis methods */
	LightSourceType analyzeLightSource(const ImageStats &stats);
	EnvironmentType analyzeEnvironment(const ImageStats &stats);
	ExposureComplexity analyzeExposureComplexity(const ImageStats &stats);
	float calculateColorTemperature(const ImageStats &stats);
	float calculateChromaticityShift(const ImageStats &stats);
	float calculateDynamicRange(const ImageStats &stats);
	float calculateIlluminationStrength(const ImageStats &stats);
	bool detectSkinTones(const ImageStats &stats);
	bool requiresHDRProcessing(const ImageStats &stats);
	float calculateOverallConfidence(const UnifiedSceneAnalysis &analysis);

	/* NEW: Spatial analysis methods */
	SpatialZoneAnalysis performSpatialZoneAnalysis(const ImageStats &stats);
	WashoutAnalysis analyzeWashoutPatterns(const SpatialZoneAnalysis &spatial, const ImageStats &stats);

	/* Zone analysis methods */
	ZoneCharacteristics analyzeZoneFromHistogram(const ImageStats &stats, ZoneType zone);
	ZoneWeights getZoneWeights(ZoneType zone);

	/* Weighted calculation methods */
	double calculateWeightedBrightness(const ImageStats &stats, const ZoneWeights &weights);
	double calculateWeightedVariance(const ImageStats &stats, const ZoneWeights &weights);
	double findWeightedPeak(const ImageStats &stats, const ZoneWeights &weights);
	double calculateWeightedHighlights(const ImageStats &stats, const ZoneWeights &weights);
	double calculateWeightedShadows(const ImageStats &stats, const ZoneWeights &weights);
	bool detectWeightedClipping(const ImageStats &stats, const ZoneWeights &weights);
	double estimateZoneCCT(const ImageStats &stats, const ZoneWeights &weights);
	double calculateZoneConfidence(const ZoneCharacteristics &zone, const ImageStats &stats);

	/* Channel analysis methods */
	double calculateWeightedChannelAverage(const std::array<uint32_t, 512> &histogram,
	                                      uint32_t totalPixels, const ZoneWeights &weights);
	double calculateWeightedChannelVariance(const std::array<uint32_t, 512> &histogram,
	                                       uint32_t totalPixels, const ZoneWeights &weights);

	/* Cross-zone analysis methods */
	double calculateVerticalGradient(const SpatialZoneAnalysis &spatial);
	double calculateHorizontalBalance(const SpatialZoneAnalysis &spatial);
	double calculateCCTVariance(const SpatialZoneAnalysis &spatial);
	bool detectMixedLightingPattern(const SpatialZoneAnalysis &spatial);
	bool detectOutdoorLightingPattern(const SpatialZoneAnalysis &spatial);
	double calculateDominantCCT(const SpatialZoneAnalysis &spatial);
	double calculateSpatialConfidence(const SpatialZoneAnalysis &spatial);

	/* Enhanced scene analysis methods */
	LightSourceType analyzeLightSourceWithSpatial(const ImageStats &stats, const SpatialZoneAnalysis &spatial);
	EnvironmentType analyzeEnvironmentWithSpatial(const ImageStats &stats, const SpatialZoneAnalysis &spatial);
	double calculateMixedLightingRatio(const SpatialZoneAnalysis &spatial);
	float calculateOverallConfidenceWithSpatial(const UnifiedSceneAnalysis &analysis);

	/* Scene classification state tracking (hysteresis) */
	bool previousOutdoorState_;
	uint32_t consecutiveOutdoorFrames_;
	uint32_t consecutiveIndoorFrames_;

	/* Hysteresis constants */
	static constexpr float kOutdoorStickyThreshold = 0.50f;
	static constexpr float kOutdoorEntryThreshold = 0.67f;
	static constexpr uint32_t kMinFramesForStateChange = 2;
	static constexpr float kOutdoorCCTMinimum = 4000.0f;
};

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_MICROCHIP_ISC_COMMON_H__ */
