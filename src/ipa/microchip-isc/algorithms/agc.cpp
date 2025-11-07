/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Advanced Auto Gain Control with Unified Scene Analysis
 */
#include "agc.h"
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace libcamera {
namespace ipa::microchip_isc {

LOG_DEFINE_CATEGORY(ISC_AGC)

	AGC::AGC()
	: convergenceState_(ConvergenceState::INITIALIZING),
	frameCount_(0), convergenceFrames_(0), stableFrameCount_(0),
	hasValidHistory_(false)
{
	sceneAnalyzer_ = std::make_unique<UnifiedSceneAnalyzer>();

	/* Initialize with safe defaults */
	lastResult_ = {
		.exposureTime = 1600,      /*1.6ms */
		.analogueGain = 0,
		.digitalGain = 256,         /* 1.0x */
		.targetLuminance = 128.0f,
		.actualLuminance = 128.0f,
		.exposureStrategy = "Initial",
		.requiresHDR = false
	};

	/* Clear history */
	for (int i = 0; i < 8; i++) {
		resultHistory_[i] = lastResult_;
	}
}

void AGC::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	sensorInfo_ = sensorInfo;
	sensorModel_ = sensorInfo.model;

	/* Initialize sensor-specific exposure curves */
	initializeSensorCurves();

	/* Update limits based on sensor capabilities */
	config_.minExposureTime = static_cast<uint32_t>(sensorInfo.minExposure);
	config_.maxExposureTime = static_cast<uint32_t>(sensorInfo.maxExposure);
	config_.minAnalogueGain = static_cast<uint32_t>(sensorInfo.minAnalogGain);
	config_.maxAnalogueGain = static_cast<uint32_t>(sensorInfo.maxAnalogGain);
	config_.minDigitalGain = static_cast<uint32_t>(sensorInfo.minDigitalGain);
	config_.maxDigitalGain = static_cast<uint32_t>(sensorInfo.maxDigitalGain);

	/* Reset state - baselines will be selected dynamically based on scene */
	hasValidHistory_ = false;
	convergenceState_ = ConvergenceState::INITIALIZING;
	frameCount_ = 0;
	convergenceFrames_ = 0;
	stableFrameCount_ = 0;

	LOG(ISC_AGC, Info) << "AGC configured: exposure=" << config_.minExposureTime
		<< "-" << config_.maxExposureTime << "μs"
		<< " gain=" << config_.minAnalogueGain << "-" << config_.maxAnalogueGain
		<< " (scene-aware baselines enabled)";

}

void AGC::process(const ImageStats &stats, ControlList &results)
{
	frameCount_++;

	LOG(ISC_AGC, Debug) << "=== AGC PROCESS START (Frame " << frameCount_ << ") ===";

	if (!validateImageStats(stats)) {
		results.set(SENSOR_EXPOSURE_ID, static_cast<int32_t>(600));
		results.set(SENSOR_ANALOGUE_GAIN_ID, static_cast<int32_t>(48));
		results.set(SENSOR_DIGITAL_GAIN_ID, static_cast<int32_t>(256));
		return;
	}

	/* Perform unified scene analysis */
	currentSceneAnalysis_ = sceneAnalyzer_->analyzeScene(stats);

	LOG(ISC_AGC, Info) << "Scene: "
		<< exposureComplexityToString(currentSceneAnalysis_.exposureComplexity)
		<< " + " << lightSourceTypeToString(currentSceneAnalysis_.lightSource)
		<< " + " << environmentTypeToString(currentSceneAnalysis_.environment)
		<< " L:" << std::fixed << std::setprecision(1) << currentSceneAnalysis_.illuminationStrength;

	/* Calculate exposure based on unified scene analysis */
	ExposureResult result = calculateExposure(stats, currentSceneAnalysis_);

	/* Update convergence state */
	updateConvergenceState(result);

	/* Apply temporal smoothing if not initial frames */
	if (frameCount_ > 1 && convergenceState_ != ConvergenceState::INITIALIZING) {
		applyTemporalSmoothing(result);
	}

	/* Validate and clamp result */
	clampExposureParameters(result);
	if (!validateExposureResult(result)) {
		LOG(ISC_AGC, Warning) << "Invalid exposure result - using safe defaults";
		result = lastResult_;
	}

	/* Apply overexposure protection */
	applyOverexposureProtection(result, stats);

	/* Store result */
	lastResult_ = result;

	/* Update history */
	for (int i = 7; i > 0; i--) {
		resultHistory_[i] = resultHistory_[i-1];
	}
	resultHistory_[0] = result;
	hasValidHistory_ = (frameCount_ >= 3);

	/* Apply to control list using correct mojom control IDs */
	results.set(SENSOR_EXPOSURE_ID, static_cast<int32_t>(result.exposureTime));
	results.set(SENSOR_ANALOGUE_GAIN_ID, static_cast<int32_t>(result.analogueGain));
	results.set(SENSOR_DIGITAL_GAIN_ID, static_cast<int32_t>(result.digitalGain));

	LOG(ISC_AGC, Info) << "Applied: E:" << result.exposureTime
		<< " A:" << result.analogueGain
		<< " D:" << result.digitalGain
		<< " (" << result.exposureStrategy << ")"
		<< " Target:" << std::fixed << std::setprecision(1) << result.targetLuminance
		<< " Actual:" << result.actualLuminance
		<< " State:" << static_cast<int>(convergenceState_);
}

void AGC::initializeSensorCurves()
{
	/* IMX219 NoIR - brightness to max exposure mapping */
	sensorExposureCurves_["imx219"] = {
		.points = {
			{200.0f, 100},   /* Very bright sunlight */
			{170.0f, 120},   /* Bright outdoor */
			{150.0f, 160},   /* Slightly less bright */
			{130.0f, 200},   /* Moderate bright */
			{110.0f, 250},   /* Mild outdoor */
			{90.0f, 320},    /* Cloudy */
			{70.0f, 400},    /* Overcast */
			{50.0f, 550},    /* Heavy overcast */
			{35.0f, 700},    /* Dusk/dawn */
			{0.0f, 1000}     /* Very dim */
		}
	};

	/* Validate curve is sorted descending by brightness */
	for (auto &[sensorName, curve] : sensorExposureCurves_) {
		auto &points = curve.points;

		if (points.size() < 2) {
			LOG(ISC_AGC, Error) << "Sensor " << sensorName
				<< " has too few calibration points: " << points.size();
			continue;
		}

		for (size_t i = 0; i < points.size() - 1; i++) {
			if (points[i].first <= points[i+1].first) {
				LOG(ISC_AGC, Error) << "Sensor " << sensorName
					<< " curve points not sorted descending at index " << i
					<< " (brightness " << points[i].first
					<< " -> " << points[i+1].first << ")";
			}
			if (points[i].second >= points[i+1].second) {
				LOG(ISC_AGC, Warning) << "Sensor " << sensorName
					<< " exposure not increasing with lower brightness at index " << i;
			}
		}

		LOG(ISC_AGC, Info) << "Loaded " << points.size()
			<< " calibration points for sensor: " << sensorName;
	}
}

uint32_t AGC::interpolateExposure(float brightness) const
{
	/* Get sensor-specific curve using stored sensor model */
	auto curveIt = sensorExposureCurves_.find(sensorModel_);
	if (curveIt == sensorExposureCurves_.end()) {
		LOG(ISC_AGC, Warning) << "No exposure curve for sensor " << sensorModel_;
		return calculateAdaptiveExposure(brightness);
	}

	const auto &points = curveIt->second.points;
	if (points.empty()) {
		LOG(ISC_AGC, Warning) << "Empty exposure curve for " << sensorModel_;
		return calculateAdaptiveExposure(brightness);
	}

	/* Clamp brightness to curve range */
	float minBrightness = points.back().first;
	float maxBrightness = points.front().first;

	if (brightness >= maxBrightness - 5.0f) {  /* Dead zone: 195-250+ */
		LOG(ISC_AGC, Debug) << "Brightness " << brightness
			<< " in dead zone (>=" << (maxBrightness - 5.0f)
			<< "), using minimum exposure " << points.front().second;
		return points.front().second;
	}

	if (brightness <= minBrightness) {
		LOG(ISC_AGC, Debug) << "Brightness " << brightness
			<< " below curve min " << minBrightness
			<< ", using maximum exposure " << points.back().second;
		return points.back().second;
	}

	/* Find bracketing points for interpolation */
	for (size_t i = 0; i < points.size() - 1; i++) {
		if (brightness >= points[i + 1].first &&
				brightness <= points[i].first) {

			/* Linear interpolation - do all math in float to avoid overflow */
			float b1 = points[i + 1].first;
			float b2 = points[i].first;
			float e1 = static_cast<float>(points[i + 1].second);
			float e2 = static_cast<float>(points[i].second);

			float ratio = (brightness - b1) / (b2 - b1);
			float interpolatedFloat = e1 + ratio * (e2 - e1);

			/* Clamp to positive and cast to uint32_t */
			uint32_t interpolated = static_cast<uint32_t>(std::max(0.0f, interpolatedFloat));

			LOG(ISC_AGC, Debug) << "Interpolated: brightness=" << brightness
				<< " between [" << b1 << "," << b2 << "]"
				<< " exposure=" << interpolated << "us"
				<< " (from " << e1 << " to " << e2 << ")";
			return interpolated;
		}
	}

	/* Fallback */
	LOG(ISC_AGC, Warning) << "Interpolation failed for brightness " << brightness
		<< " on sensor " << sensorModel_;
	return calculateAdaptiveExposure(brightness);
}

AGC::BaselineConfig AGC::selectBaseline(const UnifiedSceneAnalysis &scene) const
{
	BaselineConfig baseline;

	/* Outdoor: short exposure */
	if (scene.environment == EnvironmentType::OUTDOOR) {
		baseline = { 300, 23, 256, "🌞 Outdoor" };
		return baseline;
	}

	/* Indoor: select by common light sources */
	switch (scene.lightSource) {
		case LightSourceType::LED:
			baseline = { 1600, 23, 256, "🏠 LED" };
			break;

		case LightSourceType::FLUORESCENT:
			baseline = { 1200, 23, 256, "💡 Fluorescent" };
			break;

		case LightSourceType::INCANDESCENT:
			baseline = { 2000, 30, 256, "🔥 Incandescent" };
			break;

		case LightSourceType::DAYLIGHT:
		case LightSourceType::CLOUDY:
		case LightSourceType::SHADE:
			/* Natural light indoors */
			baseline = { 400, 23, 256, "☀️ Natural-Light" };
			break;

		case LightSourceType::MIXED_SOURCES:
			baseline = { 1400, 25, 256, "🌈 Mixed" };
			break;

		case LightSourceType::FLASH:
		case LightSourceType::UNKNOWN_SOURCE:
		default:
			baseline = { 1600, 23, 256, "❓ Default" };
			break;
	}

	return baseline;
}

ExposureResult AGC::calculateExposure(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	/* ========== SCENE-AWARE BASELINE SELECTION ========== */
	BaselineConfig baseline = selectBaseline(scene);

	LOG(ISC_AGC, Debug)
		<< "Baseline: " << baseline.name
		<< " E=" << baseline.exposure
		<< "us A=" << baseline.analogueGain
		<< " D=" << baseline.digitalGain;

	/* Initialize result with scene-appropriate baseline */
	ExposureResult result;
	result.exposureTime = baseline.exposure;
	result.analogueGain = baseline.analogueGain;
	result.digitalGain = baseline.digitalGain;
	result.exposureStrategy = std::string(baseline.name);

	float currentLuminance = calculateLuminanceFromBayer(stats);
	result.actualLuminance = currentLuminance;

	/* Route to appropriate strategy based on scene complexity */
	switch (scene.exposureComplexity) {
		case ExposureComplexity::BACKLIGHT:
			result.targetLuminance = 110.0f;
			applyBacklightCompensation(result, scene);
			break;

		case ExposureComplexity::LOW_LIGHT:
			if (scene.lightSource == LightSourceType::LED) {
				result.targetLuminance = 85.0f;
			} else {
				result.targetLuminance = 140.0f;
			}
			applyLowLightOptimization(result, scene);
			break;

		case ExposureComplexity::HIGH_DYNAMIC_RANGE:
			result.targetLuminance = 120.0f;
			applyHDRStrategy(result, scene);

			/* HDR strategy uses calibrated/calculated exposure - return immediately */
			LOG(ISC_AGC, Debug) << "HDR strategy completed with calibrated exposure";
			return result;

		default:
			if (scene.lightSource == LightSourceType::LED) {
				result.targetLuminance = 95.0f;
			} else {
				result.targetLuminance = config_.targetLuminance;
			}
			applyUniformStrategy(result, scene);
			break;
	}

	/* Skip indoor targetRatio for outdoor - calibration handles it */
	if (scene.environment == EnvironmentType::OUTDOOR) {
		return result;
	}

	/* Indoor: targetRatio adjustment for uniform scenes */
	float targetRatio = result.targetLuminance / std::max(currentLuminance, 1.0f);

	if (currentLuminance < 40.0f) {
		targetRatio = std::clamp(targetRatio, 1.0f, 5.0f);
	} else {
		targetRatio = std::clamp(targetRatio, 0.5f, 2.5f);
	}

	if (scene.lightSource == LightSourceType::LED) {
		result.exposureTime = std::min(
				static_cast<uint32_t>(result.exposureTime * targetRatio * 1.4f),
				config_.maxExposureTime
				);
	} else {
		result.analogueGain = std::min(
				static_cast<uint32_t>(result.analogueGain * std::sqrt(targetRatio)),
				config_.maxAnalogueGain
				);
		result.exposureTime = std::min(
				static_cast<uint32_t>(result.exposureTime * std::sqrt(targetRatio)),
				config_.maxExposureTime
				);
	}

	return result;
}

void AGC::applyBacklightCompensation(ExposureResult &result, const UnifiedSceneAnalysis &scene)
{
	/* Use scene data to determine backlight compensation strategy */
	float compressionFactor = 0.6f;
	if (scene.dynamicRange > 0.8f) {
		compressionFactor = 0.5f;  /* More aggressive for extreme backlight */
	} else if (scene.dynamicRange < 0.6f) {
		compressionFactor = 0.7f;  /* Less aggressive for mild backlight */
	}

	/* Environment-specific adjustments */
	uint32_t maxExposureLimit = 800;
	if (scene.environment == EnvironmentType::OUTDOOR) {
		maxExposureLimit = 600;  /* More conservative outdoors */
	} else if (scene.environment == EnvironmentType::INDOOR_MIXED) {
		maxExposureLimit = 900;  /* Slightly more flexible indoors */
	}

	/* Apply exposure limitation based on scene analysis */
	if (result.exposureTime > maxExposureLimit) {
		result.exposureTime = maxExposureLimit;
		LOG(ISC_AGC, Info) << "Backlight: Limited exposure to " << result.exposureTime
			<< "μs (DR:" << std::fixed << std::setprecision(2) << scene.dynamicRange
			<< " Env:" << environmentTypeToString(scene.environment) << ")";
	}

	/* Compensate with gain based on illumination strength */
	if (scene.illuminationStrength < 120.0f) {
		float gainCompensation = 1.3f;
		result.analogueGain = std::min(
				static_cast<uint32_t>(result.analogueGain * gainCompensation),
				config_.maxAnalogueGain
				);
	}

	/* Adjust target luminance based on scene characteristics */
	result.targetLuminance *= compressionFactor;

	result.exposureStrategy += " + BacklightCompensation(DR:" +
		std::to_string(scene.dynamicRange).substr(0,4) + ")";
}

void AGC::applyLowLightOptimization(ExposureResult &result, const UnifiedSceneAnalysis &scene)
{
	/* STEP 1: Set target luminance with simplified bright detection */
	bool hasBrightTop = (scene.spatialAnalysis.topZone.avgBrightness > 40.0f);

	if (scene.lightSource == LightSourceType::LED) {
		if (hasBrightTop) {
			/* Bright ceiling lights detected */
			result.targetLuminance = std::clamp(scene.illuminationStrength * 4.5f, 105.0f, 135.0f);
			LOG(ISC_AGC, Info) << "LED with bright top (zone="
				<< scene.spatialAnalysis.topZone.avgBrightness
				<< "): balanced target=" << result.targetLuminance;
		} else {
			/* Normal LED low-light */
			result.targetLuminance = std::clamp(scene.illuminationStrength * 5.0f, 115.0f, 145.0f);
			LOG(ISC_AGC, Info) << "LED low-light: balanced target="
				<< result.targetLuminance;
		}
	} else if (scene.lightSource == LightSourceType::DAYLIGHT) {
		result.targetLuminance = std::clamp(scene.illuminationStrength * 6.0f, 100.0f, 180.0f);
	} else {
		/* Other light sources */
		result.targetLuminance = std::clamp(scene.illuminationStrength * 4.5f, 105.0f, 145.0f);
	}

	/* Determine optimization level based on actual brightness */
	float optimizationLevel = 1.0f;
	if (scene.illuminationStrength < 30.0f) {
		optimizationLevel = 2.5f;
	} else if (scene.illuminationStrength < 50.0f) {
		optimizationLevel = 1.8f;
	}

	/* Light source specific exposure limits */
	uint32_t maxSafeExposure = 25000;

	if (scene.lightSource == LightSourceType::LED) {
		maxSafeExposure = 33000;
		LOG(ISC_AGC, Info) << "LED low-light: extended exposure limits";
	} else if (scene.lightSource == LightSourceType::INCANDESCENT) {
		maxSafeExposure = 20000;
	} else if (scene.lightSource == LightSourceType::FLUORESCENT) {
		maxSafeExposure = 16667;
	} else if (scene.lightSource == LightSourceType::DAYLIGHT) {
		maxSafeExposure = 33000;
		LOG(ISC_AGC, Info) << "Daylight low-light: extended exposure limits";
	}

	/* Environment adjustments */
	if (scene.environment == EnvironmentType::OUTDOOR) {
		maxSafeExposure = std::min(maxSafeExposure, 15000u);
	}

	/* STEP 2: Set exposure to maximum safe value */
	uint32_t targetExposure = static_cast<uint32_t>(
			std::min(static_cast<float>(config_.maxExposureTime),
				static_cast<float>(maxSafeExposure) * optimizationLevel)
			);

	result.exposureTime = targetExposure;
	LOG(ISC_AGC, Debug) << "Exposure set to: " << result.exposureTime
		<< " (max safe: " << maxSafeExposure << ")";

	/* STEP 3: Calculate gain needed to reach target */
	float currentBrightness = scene.illuminationStrength;
	float targetBrightness = result.targetLuminance;

	/* Get baseline for reference */
	BaselineConfig baseline = selectBaseline(scene);

	/* Check if exposure is maxed */
	bool exposureIsMaxed = (result.exposureTime >= config_.maxExposureTime * 0.95f);

	if (exposureIsMaxed && currentBrightness < targetBrightness * 0.85f) {
		/* Calculate direct gain ratio needed */
		float totalGainNeeded = targetBrightness / std::max(currentBrightness, 1.0f);

		/* Get sensor's gain range */
		uint32_t minGain = config_.minAnalogueGain;
		uint32_t maxGain = config_.maxAnalogueGain;

		/* Handle sensors that report 0 as min gain */
		if (minGain == 0) {
			minGain = std::max(1u, maxGain / 10);
			LOG(ISC_AGC, Debug) << "Sensor reports minGain=0, using safe minimum: " << minGain;
		}

		/* Calculate target gain directly from total ratio */
		uint32_t targetGain = static_cast<uint32_t>(minGain * totalGainNeeded * 1.30f);

		/* Safety clamp */
		targetGain = std::clamp(targetGain, minGain, static_cast<uint32_t>(maxGain * 0.8f));

		result.analogueGain = targetGain;

		/* Calculate achieved multiplier for logging */
		float achievedMultiplier = static_cast<float>(result.analogueGain) /
			std::max(static_cast<float>(minGain), 1.0f);

		LOG(ISC_AGC, Info) << "Applied analogue gain (DIRECT): " << result.analogueGain
			<< " (ratio=" << std::fixed << std::setprecision(2) << totalGainNeeded
			<< "x from minGain=" << minGain
			<< ", achieved=" << achievedMultiplier << "x"
			<< " | range=" << minGain << "-" << maxGain << ")";

		/* Warn if we're near the limit */
		if (result.analogueGain >= maxGain * 0.75f) {
			LOG(ISC_AGC, Warning) << "Gain high at " << result.analogueGain
				<< " (max=" << maxGain << ") - scene needs "
				<< totalGainNeeded << "x total boost";
		}

		result.exposureStrategy += " + Gain(" + std::to_string(result.analogueGain) + ")";

	} else if (!exposureIsMaxed) {
		/* Exposure has headroom, use baseline gain */
		result.analogueGain = baseline.analogueGain;
		LOG(ISC_AGC, Debug) << "Exposure not maxed, using baseline gain: " << result.analogueGain;

	} else {
		/* Exposure maxed but brightness is close enough to target */
		result.analogueGain = baseline.analogueGain;
		LOG(ISC_AGC, Debug) << "Brightness close to target ("
			<< currentBrightness << "/" << targetBrightness
			<< "), using baseline gain: " << result.analogueGain;
	}

	result.exposureStrategy += " + LowLightOptimization(L:" +
		std::to_string(scene.illuminationStrength).substr(0,4) +
		" Src:" + lightSourceTypeToString(scene.lightSource) + ")";

	LOG(ISC_AGC, Info) << "Low-light optimization complete: E=" << result.exposureTime
		<< " A=" << result.analogueGain
		<< " D=" << result.digitalGain
		<< " target=" << result.targetLuminance
		<< " current=" << currentBrightness;
}

void AGC::applyHDRStrategy(ExposureResult &result, const UnifiedSceneAnalysis &scene)
{
	if (!scene.requiresHDRProcessing) {
		result.exposureStrategy += " + HDRStrategy(NotRequired)";
		return;
	}

	/* Detect bright high-contrast scenes - focus on bright zones, not overall average */
	bool hasVeryBrightTop = (scene.spatialAnalysis.topZone.avgBrightness > 180.0f);
	bool hasHighContrast = (scene.dynamicRange > 0.7f);
	bool hasDaylightCCT = (scene.colorTemperature > 4000.0f && scene.colorTemperature < 6500.0f);

	/* Strong brightness gradient indicates outdoor through window */
	bool hasStrongGradient = (scene.spatialAnalysis.verticalGradient > 0.5);

	/* Either overall bright OR has very bright top with gradient (window scenes) */
	bool isBrightScene = (scene.illuminationStrength > 140.0f) ||
		(hasVeryBrightTop && hasStrongGradient);

	/* Bright daylight HDR scenes (outdoor through window, bright indoor, etc.) */
	if (isBrightScene && hasVeryBrightTop && hasHighContrast && hasDaylightCCT) {
		/* Use top zone brightness for exposure calculation when it dominates */
		float exposureBrightness;
		if (hasStrongGradient && scene.spatialAnalysis.topZone.avgBrightness > scene.illuminationStrength * 1.4f) {
			/* Window scene: weight toward top zone to prevent overexposure */
			exposureBrightness = scene.illuminationStrength * 0.4f +
				scene.spatialAnalysis.topZone.avgBrightness * 0.6f;
			LOG(ISC_AGC, Debug) << "Window scene detected: using weighted brightness="
				<< exposureBrightness;
		} else {
			exposureBrightness = scene.illuminationStrength;
		}

		/* Use sensor calibration curve for optimal exposure */
		uint32_t calibratedExposure = interpolateExposure(exposureBrightness);

		/* Cap at maximum for HDR to prevent motion blur and maintain highlight detail */
		uint32_t maxExposureForBrightHDR = 200;  /* 0.2ms maximum */
		result.exposureTime = std::clamp(calibratedExposure,
				config_.minExposureTime,
				std::min(maxExposureForBrightHDR, config_.maxExposureTime));

		/* Minimal gain for bright scenes - use sensor's minimum */
		uint32_t minGain = (config_.minAnalogueGain == 0) ?
			std::max(1u, config_.maxAnalogueGain / 10) :
			config_.minAnalogueGain;

		/* Allow slight gain increase only for very high contrast scenes */
		uint32_t maxGainForBrightScene = minGain + 30;
		result.analogueGain = std::clamp(result.analogueGain, minGain, maxGainForBrightScene);

		/* Lower target luminance to preserve highlights in bright scenes */
		result.targetLuminance = 75.0f + (exposureBrightness - 140.0f) * 0.1f;
		result.targetLuminance = std::clamp(result.targetLuminance, 75.0f, 95.0f);

		result.requiresHDR = true;
		result.exposureStrategy += " + HDRStrategy(BrightCalibrated E=" +
			std::to_string(result.exposureTime) + "us G=" +
			std::to_string(result.analogueGain) + " weighted=" +
			std::to_string(static_cast<int>(exposureBrightness)) + ")";

		LOG(ISC_AGC, Info) << "Bright daylight HDR: calibrated=" << calibratedExposure
			<< "us actual=" << result.exposureTime
			<< "us G=" << result.analogueGain
			<< " (brightness=" << scene.illuminationStrength
			<< " topZone=" << scene.spatialAnalysis.topZone.avgBrightness
			<< " weighted=" << exposureBrightness
			<< " gradient=" << scene.spatialAnalysis.verticalGradient
			<< " CCT=" << scene.colorTemperature << "K)";
		return;
	}

	/* Outdoor HDR: use calibration curve */
	if (scene.environment == EnvironmentType::OUTDOOR) {
		uint32_t calibratedExposure = interpolateExposure(scene.illuminationStrength);

		/* Cap for HDR motion blur prevention */
		uint32_t maxHDROutdoor = 400;  /* 0.4ms maximum for outdoor HDR */
		result.exposureTime = std::min(calibratedExposure, maxHDROutdoor);

		/* Minimal gain for outdoor HDR */
		uint32_t minGain = (config_.minAnalogueGain == 0) ?
			std::max(1u, config_.maxAnalogueGain / 10) :
			config_.minAnalogueGain;
		result.analogueGain = std::clamp(result.analogueGain,
				minGain,
				minGain + 50);

		result.targetLuminance = 90.0f;
		result.requiresHDR = true;
		result.exposureStrategy += " + HDRStrategy(OutdoorCalibrated cap=" +
			std::to_string(result.exposureTime) + "us DR:" +
			std::to_string(scene.dynamicRange).substr(0,4) + ")";

		LOG(ISC_AGC, Info) << "Outdoor HDR: calibrated=" << calibratedExposure
			<< "us actual=" << result.exposureTime
			<< "us (brightness=" << scene.illuminationStrength << ")";
		return;
	}

	/* Indoor HDR strategy */
	uint32_t exposureLimit = 1200;
	uint32_t gainLimit = 768;

	if (scene.dynamicRange > 0.8f) {
		exposureLimit = 1000;
		gainLimit = 512;
	} else if (scene.dynamicRange < 0.6f) {
		exposureLimit = 1500;
		gainLimit = 1024;
	}

	/* Light source considerations */
	if (scene.lightSource == LightSourceType::FLUORESCENT) {
		exposureLimit = std::min(exposureLimit, 16667u);
	}

	result.exposureTime = std::min(result.exposureTime, exposureLimit);
	result.analogueGain = std::clamp(result.analogueGain, 256u, gainLimit);
	result.requiresHDR = true;
	result.targetLuminance = std::clamp(scene.illuminationStrength * 0.8f, 90.0f, 120.0f);

	result.exposureStrategy += " + HDRStrategy(Indoor DR:" +
		std::to_string(scene.dynamicRange).substr(0,4) + ")";

	LOG(ISC_AGC, Info) << "Indoor HDR: E=" << result.exposureTime
		<< "us G=" << result.analogueGain
		<< " target=" << result.targetLuminance;
}

void AGC::applyUniformStrategy(ExposureResult &result, const UnifiedSceneAnalysis &scene)
{
	/* Outdoor: use sensor-calibrated exposure curve */
	if (scene.environment == EnvironmentType::OUTDOOR) {
		uint32_t maxOutdoorExposure = interpolateExposure(scene.illuminationStrength);

		result.exposureTime = std::min(result.exposureTime, maxOutdoorExposure);
		result.targetLuminance = 70.0f;

		/* Use baseline gain for outdoor */
		BaselineConfig baseline = selectBaseline(scene);
		result.analogueGain = baseline.analogueGain;
		result.digitalGain = baseline.digitalGain;

		result.exposureStrategy += " + Outdoor(cap=" +
			std::to_string(maxOutdoorExposure) + "us)";

		LOG(ISC_AGC, Debug) << "Outdoor interpolated exposure: "
			<< maxOutdoorExposure << "us (brightness="
			<< scene.illuminationStrength << ")";
		return;
	}

	/* Indoor: balance exposure vs gain */
	float exposurePreference = 1.0f;

	if (scene.lightSource == LightSourceType::FLUORESCENT) {
		exposurePreference = 0.8f;  /* Reduce for flicker */
	} else if (scene.lightSource == LightSourceType::LED) {
		exposurePreference = 1.1f;  /* LED is stable */
	} else if (scene.lightSource == LightSourceType::INCANDESCENT) {
		exposurePreference = 1.05f;  /* Tungsten is stable */
	}

	/* Get baseline for scene-appropriate starting point */
	BaselineConfig baseline = selectBaseline(scene);

	/* Calculate adjustment from baseline */
	float targetRatio = result.targetLuminance / std::max(scene.illuminationStrength, 1.0f);

	/* Apply ratio to baseline exposure */
	result.exposureTime = std::clamp(
			static_cast<uint32_t>(baseline.exposure * targetRatio * exposurePreference),
			config_.minExposureTime,
			config_.maxExposureTime
			);

	/* Start with baseline gain */
	result.analogueGain = baseline.analogueGain;

	/* Optimize exposure/gain trade-off when gain is high but exposure has headroom */
	if (result.analogueGain > 512 && result.exposureTime < config_.maxExposureTime / 2) {
		float gainReduction = 0.8f;
		float exposureIncrease = (1.0f / gainReduction) * exposurePreference;

		uint32_t newExposure = std::min(
				static_cast<uint32_t>(result.exposureTime * exposureIncrease),
				config_.maxExposureTime
				);

		if (newExposure > result.exposureTime * 1.2f) {
			result.analogueGain = static_cast<uint32_t>(result.analogueGain * gainReduction);
			result.exposureTime = newExposure;
			LOG(ISC_AGC, Debug) << "Indoor: Optimized E/G trade-off";
		}
	}

	result.targetLuminance = std::clamp(scene.illuminationStrength * 0.9f, 100.0f, 130.0f);
	result.exposureStrategy += " + Indoor";
}

void AGC::updateConvergenceState(const ExposureResult &result)
{
	float luminanceDiff = std::abs(result.actualLuminance - result.targetLuminance);

	if (luminanceDiff < 10.0f) {
		stableFrameCount_++;
		if (stableFrameCount_ > 3) {
			convergenceState_ = ConvergenceState::CONVERGED;
		} else {
			convergenceState_ = ConvergenceState::CONVERGING;
		}
	} else {
		stableFrameCount_ = 0;
		convergenceState_ = ConvergenceState::CONVERGING;
	}
}

void AGC::applyTemporalSmoothing(ExposureResult &result)
{
	if (!hasValidHistory_) return;

	float smoothingFactor;
	if (currentSceneAnalysis_.environment == EnvironmentType::OUTDOOR) {
		smoothingFactor = 0.2f;  /* Light smoothing for outdoor */
	} else if (convergenceState_ == ConvergenceState::CONVERGED) {
		smoothingFactor = 0.1f;  /* Less smoothing when converged */
	} else {
		smoothingFactor = 0.3f;  /* Normal indoor smoothing */
	}

	result.exposureTime = static_cast<uint32_t>(
			smoothingFactor * lastResult_.exposureTime +
			(1.0f - smoothingFactor) * result.exposureTime
			);

	result.analogueGain = static_cast<uint32_t>(
			smoothingFactor * lastResult_.analogueGain +
			(1.0f - smoothingFactor) * result.analogueGain
			);
}

void AGC::clampExposureParameters(ExposureResult &result)
{
	result.exposureTime = std::clamp(result.exposureTime, config_.minExposureTime, config_.maxExposureTime);
	result.analogueGain = std::clamp(result.analogueGain, config_.minAnalogueGain, config_.maxAnalogueGain);
	result.digitalGain = std::clamp(result.digitalGain, config_.minDigitalGain, config_.maxDigitalGain);
}

bool AGC::validateExposureResult(const ExposureResult &result)
{
	return result.exposureTime >= config_.minExposureTime &&
		result.exposureTime <= config_.maxExposureTime &&
		result.analogueGain >= config_.minAnalogueGain &&
		result.analogueGain <= config_.maxAnalogueGain;
}

void AGC::applyOverexposureProtection(ExposureResult &result, const ImageStats &stats)
{
	if (detectOverexposure(stats, 0.02f)) {  /* 2% overexposed pixels */
		result.exposureTime = static_cast<uint32_t>(result.exposureTime * 0.8f);
		result.exposureStrategy += " + OverexposureProtection";
		LOG(ISC_AGC, Warning) << "Overexposure detected - reducing exposure";
	}
}

void AGC::setAWBContext(const AWBContext &context)
{
	awbContext_ = context;
}

void AGC::resetConvergence()
{
	convergenceState_ = ConvergenceState::INITIALIZING;
	frameCount_ = 0;
	convergenceFrames_ = 0;
	stableFrameCount_ = 0;
}

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

