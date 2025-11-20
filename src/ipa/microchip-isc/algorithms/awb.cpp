/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Advanced Auto White Balance with Unified Scene Analysis
 */
#include "awb.h"
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace libcamera {
namespace ipa::microchip_isc {

LOG_DEFINE_CATEGORY(ISC_AWB)

	AWB::AWB()
	: convergenceState_(ConvergenceState::INITIALIZING),
	frameCount_(0), stableFrameCount_(0)
{
	/* Initialize scene analyzer */
	sceneAnalyzer_ = std::make_unique<UnifiedSceneAnalyzer>();

	/* Initialize algorithms */
	algorithms_.push_back(std::make_unique<UnifiedGreyWorldWhiteBalance>());
	algorithms_.push_back(std::make_unique<LEDWhiteBalance>());          /* High priority for modern lighting */
	algorithms_.push_back(std::make_unique<IncandescentWhiteBalance>());
	algorithms_.push_back(std::make_unique<FluorescentWhiteBalance>());
	algorithms_.push_back(std::make_unique<DaylightWhiteBalance>());
	algorithms_.push_back(std::make_unique<NeutralRegionWhiteBalance>());
	algorithms_.push_back(std::make_unique<MultiIlluminantWhiteBalance>());

	/* Initialize with neutral defaults */
	lastResult_ = {
		.algorithmName = "Initial",
		.colorTemperatureEstimate = 5500.0f,
		.algorithmConfidence = 0.5f,
		.redGain = 1.0f, .greenRedGain = 1.0f, .greenBlueGain = 1.0f, .blueGain = 1.0f,
		.redOffset = 0, .greenRedOffset = 0, .greenBlueOffset = 0, .blueOffset = 0,
		.chromaticityCorrection = 0.0f,
		.stabilityMetric = 1.0f
	};
}

void AWB::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	sensorInfo_ = sensorInfo;
	convergenceState_ = ConvergenceState::INITIALIZING;
	frameCount_ = 0;
	stableFrameCount_ = 0;

	LOG(ISC_AWB, Debug) << "AWB configured for sensor " << sensorInfo.model
		<< " with " << algorithms_.size() << " algorithms";
}

void AWB::process(const ImageStats &stats, ControlList &results)
{
	frameCount_++;

	LOG(ISC_AWB, Debug) << "=== AWB PROCESS START (Frame " << frameCount_ << ") ===";

	if (!validateImageStats(stats)) {
		LOG(ISC_AWB, Warning) << "Invalid image statistics - skipping AWB";
		return;
	}

	/* Perform unified scene analysis */
	lastSceneAnalysis_ = sceneAnalyzer_->analyzeScene(stats);

	LOG(ISC_AWB, Debug) << "Scene: "
		<< lightSourceTypeToString(lastSceneAnalysis_.lightSource)
		<< " (" << lastSceneAnalysis_.colorTemperature << "K)"
		<< " " << environmentTypeToString(lastSceneAnalysis_.environment)
		<< " Confidence: " << std::fixed << std::setprecision(2) << lastSceneAnalysis_.overallConfidence;

	/* Select best algorithm for this scene */
	WhiteBalanceAlgorithm* selectedAlgorithm = selectBestAlgorithm(lastSceneAnalysis_);

	if (!selectedAlgorithm) {
		LOG(ISC_AWB, Warning) << "No suitable AWB algorithm found - using last result";
		return;
	}

	/* Process with selected algorithm */
	WhiteBalanceResult result = selectedAlgorithm->process(stats, lastSceneAnalysis_);

	LOG(ISC_AWB, Info) << "Selected algorithm: " << selectedAlgorithm->getName()
		<< " CCT: " << result.colorTemperatureEstimate << "K"
		<< " Confidence: " << result.algorithmConfidence;

	/* Apply temporal smoothing */
	if (frameCount_ > 1 && convergenceState_ != ConvergenceState::INITIALIZING) {
		applyTemporalSmoothing(result);
	}

	/* Validate and clamp result */
	clampGainsAndOffsets(result);
	if (!validateResult(result)) {
		LOG(ISC_AWB, Warning) << "Invalid AWB result - using last result";
		result = lastResult_;
	}

	/* Update convergence state */
	updateConvergenceState(result);

	/* Store result */
	lastResult_ = result;

	/* Apply to hardware */
	applyResultToHardware(result, results);

	LOG(ISC_AWB, Debug) << "Applied: R:" << std::fixed << std::setprecision(3) << result.redGain
		<< " GR:" << result.greenRedGain
		<< " GB:" << result.greenBlueGain
		<< " B:" << result.blueGain
		<< " State:" << static_cast<int>(convergenceState_);
}

WhiteBalanceAlgorithm* AWB::selectBestAlgorithm(const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceAlgorithm* bestAlgorithm = nullptr;
	float bestScore = 0.0f;
	std::string bestAlgorithmName;

	LOG(ISC_AWB, Debug) << "=== Algorithm Selection Process ===";
	LOG(ISC_AWB, Debug) << "Scene: " << lightSourceTypeToString(scene.lightSource)
		<< " confidence=" << scene.overallConfidence;

	for (auto &algorithm : algorithms_) {
		std::string algName = algorithm->getName();

		/* Check if algorithm is applicable */
		if (!algorithm->isApplicable(scene)) {
			LOG(ISC_AWB, Debug) << "  " << algName << ": NOT applicable";
			continue;
		}

		/* Base score from scene confidence */
		float score = scene.overallConfidence;
		LOG(ISC_AWB, Debug) << "  " << algName << ": base score=" << score;

		/* Apply strong boosts for matching light source types */
		if (algName == "LED" && scene.lightSource == LightSourceType::LED) {
			score += 1.0f;  /* Very strong boost for LED */
			LOG(ISC_AWB, Debug) << "    LED BOOST: +1.0 -> score=" << score;

		} else if (algName == "Fluorescent" && scene.lightSource == LightSourceType::FLUORESCENT) {
			score += 0.8f;
			LOG(ISC_AWB, Debug) << "    FLUORESCENT BOOST: +0.8 -> score=" << score;

		} else if (algName == "Incandescent" && scene.lightSource == LightSourceType::INCANDESCENT) {
			score += 0.8f;
			LOG(ISC_AWB, Debug) << "    INCANDESCENT BOOST: +0.8 -> score=" << score;

		} else if (algName == "Daylight" &&
				(scene.lightSource == LightSourceType::DAYLIGHT ||
					scene.lightSource == LightSourceType::CLOUDY)) {
			score += 0.8f;
			LOG(ISC_AWB, Debug) << "    DAYLIGHT BOOST: +0.8 -> score=" << score;

		} else if (algName == "UnifiedGreyWorld") {
			/* Grey World gets NO boost - it's the fallback */
			score += 0.0f;
			LOG(ISC_AWB, Debug) << "    Grey World (fallback): no boost -> score=" << score;
		}

		/* Update best if this score is higher */
		if (score > bestScore) {
			bestScore = score;
			bestAlgorithm = algorithm.get();
			bestAlgorithmName = algName;
			LOG(ISC_AWB, Debug) << "    *** NEW BEST: " << algName << " score=" << score;
		} else {
			LOG(ISC_AWB, Debug) << "    Not better than current best (" << bestScore << ")";
		}
	}

	/* Fallback to Grey World if nothing selected */
	if (!bestAlgorithm) {
		bestAlgorithm = algorithms_[0].get();  /* Grey World is always first */
		bestAlgorithmName = "UnifiedGreyWorld (fallback)";
		LOG(ISC_AWB, Debug) << "No algorithm selected - using Grey World fallback";
	}

	LOG(ISC_AWB, Debug) << "=== SELECTED: " << bestAlgorithmName
		<< " (score=" << bestScore << ") ===";

	return bestAlgorithm;
}

void AWB::applyTemporalSmoothing(WhiteBalanceResult &result)
{
	float smoothingFactor = 0.2f;
	if (convergenceState_ == ConvergenceState::CONVERGED) {
		smoothingFactor = 0.1f;  /* Less smoothing when converged */
	}

	/* Smooth gains */
	result.redGain = smoothingFactor * lastResult_.redGain + (1.0f - smoothingFactor) * result.redGain;
	result.greenRedGain = smoothingFactor * lastResult_.greenRedGain + (1.0f - smoothingFactor) * result.greenRedGain;
	result.greenBlueGain = smoothingFactor * lastResult_.greenBlueGain + (1.0f - smoothingFactor) * result.greenBlueGain;
	result.blueGain = smoothingFactor * lastResult_.blueGain + (1.0f - smoothingFactor) * result.blueGain;

	/* Smooth offsets */
	result.redOffset = static_cast<int32_t>(smoothingFactor * lastResult_.redOffset + (1.0f - smoothingFactor) * result.redOffset);
	result.greenRedOffset = static_cast<int32_t>(smoothingFactor * lastResult_.greenRedOffset + (1.0f - smoothingFactor) * result.greenRedOffset);
	result.greenBlueOffset = static_cast<int32_t>(smoothingFactor * lastResult_.greenBlueOffset + (1.0f - smoothingFactor) * result.greenBlueOffset);
	result.blueOffset = static_cast<int32_t>(smoothingFactor * lastResult_.blueOffset + (1.0f - smoothingFactor) * result.blueOffset);
}

void AWB::updateConvergenceState(const WhiteBalanceResult &result)
{
	/* Check stability across multiple metrics */
	float gainDiff = std::abs(result.redGain - lastResult_.redGain) +
		std::abs(result.blueGain - lastResult_.blueGain);
	float cctDiff = std::abs(result.colorTemperatureEstimate - lastResult_.colorTemperatureEstimate);

	if (gainDiff < 0.02f && cctDiff < 100.0f) {
		stableFrameCount_++;
		if (stableFrameCount_ > 5) {
			convergenceState_ = ConvergenceState::CONVERGED;
		} else {
			convergenceState_ = ConvergenceState::CONVERGING;
		}
	} else {
		stableFrameCount_ = 0;
		convergenceState_ = ConvergenceState::CONVERGING;
	}
}

void AWB::applyResultToHardware(const WhiteBalanceResult &result, ControlList &results)
{
	/* Convert to hardware gains (fixed point) */
	results.set(RED_GAIN_ID, static_cast<int32_t>(result.redGain * 512.0f));
	results.set(GREEN_RED_GAIN_ID, static_cast<int32_t>(result.greenRedGain * 512.0f));
	results.set(GREEN_BLUE_GAIN_ID, static_cast<int32_t>(result.greenBlueGain * 512.0f));
	results.set(BLUE_GAIN_ID, static_cast<int32_t>(result.blueGain * 512.0f));

	/* Apply offsets */
	results.set(RED_OFFSET_ID, result.redOffset);
	results.set(GREEN_RED_OFFSET_ID, result.greenRedOffset);
	results.set(GREEN_BLUE_OFFSET_ID, result.greenBlueOffset);
	results.set(BLUE_OFFSET_ID, result.blueOffset);

	/* Export analysis results for CCM integration */
	results.set(ISC_AWB_CCT_ESTIMATE_ID, static_cast<int32_t>(result.colorTemperatureEstimate));
	results.set(ISC_AWB_FLUORESCENT_STRENGTH_ID,
			(lastSceneAnalysis_.lightSource == LightSourceType::FLUORESCENT) ? 0.8f : 0.0f);
	results.set(ISC_AWB_GREEN_CAST_CORRECTION_ID, result.chromaticityCorrection);
}

bool AWB::validateResult(const WhiteBalanceResult &result)
{
	return result.redGain > 0.1f && result.redGain < 4.0f &&
		result.greenRedGain > 0.1f && result.greenRedGain < 4.0f &&
		result.greenBlueGain > 0.1f && result.greenBlueGain < 4.0f &&
		result.blueGain > 0.1f && result.blueGain < 4.0f &&
		result.colorTemperatureEstimate > 1000.0f && result.colorTemperatureEstimate < 15000.0f;
}

void AWB::clampGainsAndOffsets(WhiteBalanceResult &result)
{
	result.redGain = std::clamp(result.redGain, 0.2f, 3.0f);
	result.greenRedGain = std::clamp(result.greenRedGain, 0.2f, 3.0f);
	result.greenBlueGain = std::clamp(result.greenBlueGain, 0.2f, 3.0f);
	result.blueGain = std::clamp(result.blueGain, 0.2f, 3.0f);

	result.redOffset = std::clamp(result.redOffset, -127, 127);
	result.greenRedOffset = std::clamp(result.greenRedOffset, -127, 127);
	result.greenBlueOffset = std::clamp(result.greenBlueOffset, -127, 127);
	result.blueOffset = std::clamp(result.blueOffset, -127, 127);
}

void AWB::setAWBContext(const AWBContext &context)
{
	awbContext_ = context;
}

/* Algorithm Implementations */
WhiteBalanceResult UnifiedGreyWorldWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = scene.colorTemperature;

	/* Calculate grey world gains with spatial awareness */
	float avgGreen = (stats.meanGR + stats.meanGB) * 0.5f;

	/* Use spatial zone information for better gain calculation */
	float gainScaling = 1.0f;

	if (scene.spatialAnalysis.isMixedLighting) {
		/* Mixed lighting - use center zone for more accurate gains */
		if (scene.spatialAnalysis.centerZone.confidence > 0.7) {
			/* Weight gains based on center zone characteristics */
			float centerBrightness = static_cast<float>(scene.spatialAnalysis.centerZone.avgBrightness);
			gainScaling = std::clamp(centerBrightness / 128.0f, 0.8f, 1.2f);
			LOG(ISC_AWB, Debug) << "Using center zone scaling: " << gainScaling;
		}
		result.algorithmConfidence = 0.85f;  /* High confidence for mixed lighting */

	} else if (scene.spatialAnalysis.isOutdoorScene) {
		/* Outdoor scenes - grey world works well */
		gainScaling = 0.95f;
		result.algorithmConfidence = 0.8f;

	} else {
		/* Indoor scenes - standard grey world */
		gainScaling = 1.0f;
		result.algorithmConfidence = 0.7f;
	}

	/* Apply spatial-aware grey world gains */
	result.redGain = std::max(1.0f, (avgGreen / std::max(stats.meanR, 1.0f)) * gainScaling);
	result.greenRedGain = 1.0f;
	result.greenBlueGain = 1.0f;
	result.blueGain = std::max(1.0f, (avgGreen / std::max(stats.meanB, 1.0f)) * gainScaling);

	/* No offsets for grey world */
	result.redOffset = result.greenRedOffset = result.greenBlueOffset = result.blueOffset = 0;

	result.chromaticityCorrection = scene.chromaticityShift;
	result.stabilityMetric = scene.spatialAnalysis.isMixedLighting ? 0.9f : 0.8f;

	return result;
}

bool UnifiedGreyWorldWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Grey world enhanced applicability using spatial analysis */

	/* Excellent for mixed lighting scenarios */
	if (scene.spatialAnalysis.isMixedLighting) {
		LOG(ISC_AWB, Debug) << "Grey world excellent for mixed lighting";
		return true;
	}

	/* Good for outdoor scenes with varied illumination */
	if (scene.spatialAnalysis.isOutdoorScene) {
		LOG(ISC_AWB, Debug) << "Grey world suitable for outdoor environment";
		return true;
	}

	/* Good fallback when spatial confidence is low */
	if (scene.spatialAnalysis.sceneConfidence < 0.6) {
		LOG(ISC_AWB, Debug) << "Grey world as fallback for low spatial confidence";
		return true;
	}

	/* Less suitable for high-confidence single lighting sources */
	if (scene.overallConfidence > 0.85 && !scene.spatialAnalysis.isMixedLighting) {
		if (scene.lightSource == LightSourceType::FLUORESCENT ||
				scene.lightSource == LightSourceType::INCANDESCENT) {
			LOG(ISC_AWB, Debug) << "Grey world not needed for high-confidence single source";
			return false;
		}
	}

	/* Always applicable as final fallback */
	return true;
}

WhiteBalanceResult IncandescentWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = 2850.0f;

	float avgGreen = (stats.meanGR + stats.meanGB) * 0.5f;
	float rbRatio = stats.meanR / std::max(stats.meanB, 1.0f);

	/* Calculate correction strength using spatial confidence */
	double correctionStrength = 1.0;

	/* Validate tungsten classification with histogram data */
	if (rbRatio < 1.2f) {
		/* R/B ratio too low for tungsten - reduce confidence */
		correctionStrength *= 0.7;
		LOG(ISC_AWB, Debug) << "Low R/B ratio for tungsten: " << rbRatio;
	}

	/* Use avgGreen for additional validation */
	if (avgGreen < 30.0f) {
		/* Very dark scene - boost tungsten correction */
		correctionStrength *= 1.1;
		LOG(ISC_AWB, Debug) << "Dark scene - boosting tungsten correction: avgGreen=" << avgGreen;
	} else if (avgGreen > 180.0f) {
		/* Very bright scene - reduce tungsten correction */
		correctionStrength *= 0.8;
		LOG(ISC_AWB, Debug) << "Bright scene - reducing tungsten correction: avgGreen=" << avgGreen;
	}

	/* Check for fluorescent misclassification using histogram variance */
	float rVariance = calculateHistogramVariance(stats.histogramR);
	float gVariance = calculateHistogramVariance(stats.histogramGR);
	if (gVariance > 1500 && rVariance > 1200) {
		/* High variance suggests fluorescent, not tungsten */
		correctionStrength *= 0.6;
		LOG(ISC_AWB, Debug) << "High variance suggests fluorescent misclassification";
	}

	/* Reduce strength for mixed lighting scenarios */
	if (scene.requiresGentleCorrection || scene.spatialAnalysis.isMixedLighting) {
		correctionStrength *= scene.mixedLightingRatio;
		LOG(ISC_AWB, Debug) << "Reducing tungsten correction for mixed lighting: " << correctionStrength;
	} else if (scene.overallConfidence < 0.8) {
		correctionStrength *= scene.overallConfidence;
	}

	/* Apply scaled tungsten corrections */
	result.redGain = 1.0f + (0.8f - 1.0f) * correctionStrength;
	result.greenRedGain = 1.0f;
	result.greenBlueGain = 1.0f;
	result.blueGain = 1.0f + (2.2f - 1.0f) * correctionStrength;

	/* No offsets for tungsten (smooth light source) */
	result.redOffset = result.greenRedOffset = result.greenBlueOffset = result.blueOffset = 0;

	result.algorithmConfidence = 0.9f * correctionStrength;
	result.chromaticityCorrection = scene.chromaticityShift * correctionStrength;
	result.stabilityMetric = scene.spatialAnalysis.isMixedLighting ? 0.7f : 0.9f;

	return result;
}

bool IncandescentWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Enhanced applicability using spatial analysis */
	bool basicApplicable = (scene.lightSource == LightSourceType::INCANDESCENT);

	if (!basicApplicable) return false;

	/* Not applicable for mixed lighting - let other algorithms handle */
	if (scene.spatialAnalysis.isMixedLighting) {
		LOG(ISC_AWB, Debug) << "Incandescent not suitable for mixed lighting";
		return false;
	}

	/* Not applicable for outdoor scenes */
	if (scene.spatialAnalysis.isOutdoorScene) {
		LOG(ISC_AWB, Debug) << "Incandescent not suitable for outdoor scene";
		return false;
	}

	/* Check spatial confidence for tungsten characteristics */
	if (scene.spatialAnalysis.sceneConfidence < 0.6) {
		LOG(ISC_AWB, Debug) << "Low spatial confidence for incandescent";
		return false;
	}

	return true;
}

WhiteBalanceResult FluorescentWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = 4200.0f;

	/* Analyze histogram for fluorescent characteristics */
	float greenVariance = (calculateHistogramVariance(stats.histogramGR) +
			calculateHistogramVariance(stats.histogramGB)) / 2.0f;

	/* Check for fluorescent flicker patterns in histogram */
	uint32_t greenPeaks = countHistogramPeaks(stats.histogramGR) +
		countHistogramPeaks(stats.histogramGB);

	/* Calculate correction strength based on spatial confidence */
	double correctionStrength = 1.0;

	/* Validate fluorescent classification using histogram data */
	if (greenVariance > 1200) {
		/* High green variance supports fluorescent classification */
		correctionStrength *= 1.1;  /* Boost confidence */
		LOG(ISC_AWB, Debug) << "High green variance supports fluorescent: " << greenVariance;
	} else if (greenVariance < 600) {
		/* Low variance suggests LED, not fluorescent */
		correctionStrength *= 0.7;
		LOG(ISC_AWB, Debug) << "Low variance suggests LED, not fluorescent: " << greenVariance;
	}

	if (greenPeaks > 2) {
		/* Multiple peaks suggest fluorescent flicker */
		correctionStrength *= 1.05;
		LOG(ISC_AWB, Debug) << "Multiple histogram peaks support fluorescent";
	}

	if (scene.requiresGentleCorrection || scene.spatialAnalysis.isMixedLighting) {
		correctionStrength *= scene.mixedLightingRatio;  /* Use calculated ratio */
		LOG(ISC_AWB, Debug) << "Reducing fluorescent correction strength to "
			<< correctionStrength << " for mixed lighting";
	} else if (scene.overallConfidence < 0.8) {
		correctionStrength *= scene.overallConfidence;
	}

	/* Apply scaled fluorescent corrections */
	result.redGain = 1.0f + (1.3f - 1.0f) * correctionStrength;
	result.greenRedGain = 1.0f + (0.7f - 1.0f) * correctionStrength;
	result.greenBlueGain = 1.0f + (0.7f - 1.0f) * correctionStrength;
	result.blueGain = 1.0f + (1.1f - 1.0f) * correctionStrength;

	/* Scale offsets similarly */
	result.redOffset = static_cast<int32_t>(20 * correctionStrength);
	result.greenRedOffset = result.greenBlueOffset = static_cast<int32_t>(-30 * correctionStrength);
	result.blueOffset = static_cast<int32_t>(10 * correctionStrength);

	result.algorithmConfidence = 0.95f * correctionStrength;
	result.chromaticityCorrection = scene.chromaticityShift;
	result.stabilityMetric = scene.spatialAnalysis.isMixedLighting ? 0.6f : 0.8f;

	return result;
}

bool FluorescentWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Basic fluorescent light source check */
	if (scene.lightSource != LightSourceType::FLUORESCENT) {
		LOG(ISC_AWB, Debug) << "Fluorescent not applicable - light source is "
			<< lightSourceTypeToString(scene.lightSource);
		return false;
	}

	/* Not applicable for outdoor scenes */
	if (scene.spatialAnalysis.isOutdoorScene) {
		LOG(ISC_AWB, Debug) << "Fluorescent not applicable for outdoor scene";
		return false;
	}

	/* Require minimum spatial confidence */
	if (scene.spatialAnalysis.sceneConfidence < 0.5) {
		LOG(ISC_AWB, Debug) << "Fluorescent not applicable - low spatial confidence: "
			<< scene.spatialAnalysis.sceneConfidence;
		return false;
	}

	/* CCT should be in fluorescent range */
	double dominantCCT = scene.spatialAnalysis.dominantCCT;
	if (dominantCCT < 3500 || dominantCCT > 5200) {
		LOG(ISC_AWB, Debug) << "Fluorescent not applicable - CCT outside range: " << dominantCCT;
		return false;
	}

	/* CRITICAL: Reject if variance is too low (indicates LED, not fluorescent) */
	if (scene.spatialAnalysis.centerZone.variance < 700) {
		LOG(ISC_AWB, Debug) << "Fluorescent rejected - low variance indicates LED: "
			<< scene.spatialAnalysis.centerZone.variance;
		return false;
	}

	/* Look for fluorescent flicker characteristics */
	bool hasTopFluorescentCCT = scene.spatialAnalysis.topZone.cct > 3800 &&
		scene.spatialAnalysis.topZone.cct < 5000;
	bool hasCenterFluorescentCCT = scene.spatialAnalysis.centerZone.cct > 3800 &&
		scene.spatialAnalysis.centerZone.cct < 5000;

	if (!hasTopFluorescentCCT && !hasCenterFluorescentCCT) {
		LOG(ISC_AWB, Debug) << "Fluorescent not applicable - no zones in fluorescent CCT range";
		return false;
	}

	/* Require reasonable variance for fluorescent flicker */
	bool hasModerateVariance = scene.spatialAnalysis.centerZone.variance > 800 &&
		scene.spatialAnalysis.centerZone.variance < 2500;

	if (!hasModerateVariance) {
		LOG(ISC_AWB, Debug) << "Fluorescent not applicable - variance outside flicker range: "
			<< scene.spatialAnalysis.centerZone.variance;
		return false;
	}

	/* Mixed lighting handling */
	if (scene.spatialAnalysis.isMixedLighting) {
		if (scene.spatialAnalysis.cctVariance < 250000) {
			LOG(ISC_AWB, Debug) << "Fluorescent not applicable - insufficient CCT variance for mixed";
			return false;
		}
		if (scene.overallConfidence < 0.7) {
			LOG(ISC_AWB, Debug) << "Fluorescent not applicable - low confidence for mixed lighting";
			return false;
		}
	}

	LOG(ISC_AWB, Debug) << "Fluorescent applicable: CCT=" << dominantCCT
		<< " variance=" << scene.spatialAnalysis.centerZone.variance
		<< " confidence=" << scene.overallConfidence;

	return true;
}

WhiteBalanceResult LEDWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = scene.colorTemperature;

	/* Use stats for LED-specific analysis */
	float avgLuminance = calculateLuminanceFromBayer(stats);
	float contrast = calculateContrastFromBayer(stats);
	float avgGreen = (stats.meanGR + stats.meanGB) / 2.0f;

	/* Calculate green cast strength */
	float rbMean = (stats.meanR + stats.meanB) / 2.0f;
	float greenCast = 0.0f;
	if (rbMean > 0.1f) {
		greenCast = (avgGreen - rbMean) / rbMean;
		greenCast = std::max(0.0f, greenCast);
	}

	/* Calculate yellow cast (excess R+G relative to B) */
	float yellowCast = 0.0f;
	if (stats.meanB > 0.1f) {
		float rgMean = (stats.meanR + avgGreen) / 2.0f;
		yellowCast = (rgMean - stats.meanB) / stats.meanB;
		yellowCast = std::max(0.0f, yellowCast);
	}

	/* Use the MAXIMUM of green or yellow cast - yellow is common in LED */
	float totalCast = std::max(greenCast, yellowCast);

	LOG(ISC_AWB, Debug) << "LED cast detection: green=" << std::fixed
		<< std::setprecision(3) << greenCast
		<< " yellow=" << yellowCast
		<< " → total=" << totalCast;

	/* Determine correction strength using smooth linear interpolation */
	double correctionStrength = 0.0;

	if (totalCast <= 0.0f) {
		correctionStrength = 0.05;
		LOG(ISC_AWB, Debug) << "LED: No cast detected";
	} else if (totalCast <= 0.015f) {
		/* Very mild range: 0.0 to 0.015 → strength 0.05 to 0.15 */
		correctionStrength = 0.05 + (totalCast / 0.015f) * (0.15 - 0.05);
		LOG(ISC_AWB, Debug) << "LED: Very mild cast: " << totalCast;
	} else if (totalCast <= 0.03f) {
		/* Mild range: 0.015 to 0.03 → strength 0.15 to 0.30 */
		float t = (totalCast - 0.015f) / (0.03f - 0.015f);
		correctionStrength = 0.15 + t * (0.30 - 0.15);
		LOG(ISC_AWB, Debug) << "LED: Mild cast: " << totalCast;
	} else if (totalCast <= 0.08f) {
		/* Moderate range: 0.03 to 0.08 → strength 0.30 to 0.50 */
		float t = (totalCast - 0.03f) / (0.08f - 0.03f);
		correctionStrength = 0.30 + t * (0.50 - 0.30);
		LOG(ISC_AWB, Debug) << "LED: Moderate cast: " << totalCast;
	} else if (totalCast <= 0.15f) {
		/* Strong range: 0.08 to 0.15 → strength 0.50 to 0.70 */
		float t = (totalCast - 0.08f) / (0.15f - 0.08f);
		correctionStrength = 0.50 + t * (0.70 - 0.50);
		LOG(ISC_AWB, Debug) << "LED: Strong cast: " << totalCast;
	} else {
		/* Severe: > 0.15 → cap at 0.70 */
		correctionStrength = 0.70;
		LOG(ISC_AWB, Debug) << "LED: Severe cast (capped): " << totalCast;
	}

	LOG(ISC_AWB, Debug) << "Interpolated correction strength: "
		<< std::fixed << std::setprecision(3) << correctionStrength;

	/* Adjust for extreme brightness conditions */
	if (avgLuminance < 20.0f && correctionStrength < 0.3) {
		correctionStrength = std::max(correctionStrength, 0.20);
		LOG(ISC_AWB, Debug) << "Very dark scene - boosting correction to: "
			<< correctionStrength;
	} else if (avgLuminance > 200.0f && correctionStrength > 0.1) {
		correctionStrength *= 0.8;
		LOG(ISC_AWB, Debug) << "Very bright scene - reducing correction to: "
			<< correctionStrength;
	}

	/* Adjust for high contrast scenes */
	if (contrast > 0.7f && correctionStrength > 0.2) {
		correctionStrength *= 0.85;
		LOG(ISC_AWB, Debug) << "High contrast - reducing correction to: "
			<< correctionStrength;
	}

	/* Reduce correction for mixed lighting scenarios */
	if (scene.spatialAnalysis.isMixedLighting) {
		correctionStrength *= scene.mixedLightingRatio;
		LOG(ISC_AWB, Debug) << "Mixed lighting - reducing correction to: "
			<< correctionStrength;
	}

	/* Apply corrections based on CCT - LED-specific gentle approach */
	if (scene.colorTemperature < 4000.0f) {
		/* Warm white LEDs (2700-4000K) */
		result.redGain = 1.0f + (1.08f - 1.0f) * correctionStrength;
		result.greenRedGain = 1.0f + (0.92f - 1.0f) * correctionStrength;
		result.greenBlueGain = 1.0f + (0.92f - 1.0f) * correctionStrength;
		result.blueGain = 1.0f + (1.15f - 1.0f) * correctionStrength;
		LOG(ISC_AWB, Debug) << "Warm LED correction applied";
	} else if (scene.colorTemperature > 5000.0f) {
		/* Cool white LEDs (5000-6500K) */
		result.redGain = 1.0f + (1.12f - 1.0f) * correctionStrength;
		result.greenRedGain = 1.0f + (0.94f - 1.0f) * correctionStrength;
		result.greenBlueGain = 1.0f + (0.94f - 1.0f) * correctionStrength;
		result.blueGain = 1.0f + (1.08f - 1.0f) * correctionStrength;
		LOG(ISC_AWB, Debug) << "Cool LED correction applied";
	} else {
		/* Neutral white LEDs (4000-5000K) - minimal correction needed */
		result.redGain = 1.0f + (0.99f - 1.0f) * correctionStrength;
		result.greenRedGain = 1.0f + (0.866f - 1.0f) * correctionStrength;
		result.greenBlueGain = 1.0f + (0.866f - 1.0f) * correctionStrength;
		result.blueGain = 1.0f + (1.145f - 1.0f) * correctionStrength;
		LOG(ISC_AWB, Debug) << "Neutral LED correction applied";
	}

	/* Safety clamps to prevent overcorrection */
	result.redGain = std::clamp(result.redGain, 0.85f, 1.25f);
	result.greenRedGain = std::clamp(result.greenRedGain, 0.85f, 1.15f);
	result.greenBlueGain = std::clamp(result.greenBlueGain, 0.85f, 1.15f);
	result.blueGain = std::clamp(result.blueGain, 0.85f, 1.25f);

	/* No offsets for stable LED lighting */
	result.redOffset = static_cast<int32_t>(8 * correctionStrength);
	result.greenRedOffset = static_cast<int32_t>(-12 * correctionStrength);
	result.greenBlueOffset = static_cast<int32_t>(-12 * correctionStrength);
	result.blueOffset = static_cast<int32_t>(8 * correctionStrength);

	result.algorithmConfidence = 0.85f;
	result.chromaticityCorrection = totalCast * 0.3f;  /* Very gentle */
	result.stabilityMetric = 0.95f;  /* LEDs are very stable */

	LOG(ISC_AWB, Debug) << "LED AWB: CCT=" << scene.colorTemperature << "K"
		<< " greenCast=" << std::fixed << std::setprecision(3) << greenCast
		<< " yellowCast=" << yellowCast
		<< " totalCast=" << totalCast
		<< " strength=" << correctionStrength
		<< " → R:" << std::setprecision(4) << result.redGain
		<< " G:" << result.greenRedGain
		<< " B:" << result.blueGain
		<< " (hw: R=" << static_cast<int>(result.redGain * 512)
		<< " G=" << static_cast<int>(result.greenRedGain * 512)
		<< " B=" << static_cast<int>(result.blueGain * 512) << ")";

	return result;
}

bool LEDWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Only applicable for LED light sources */
	if (scene.lightSource != LightSourceType::LED) {
		return false;
	}

	/* Require reasonable spatial confidence */
	if (scene.spatialAnalysis.sceneConfidence < 0.5) {
		LOG(ISC_AWB, Debug) << "LED not applicable - low spatial confidence: "
			<< scene.spatialAnalysis.sceneConfidence;
		return false;
	}

	/* Check for LED-specific characteristics in spatial analysis */
	if (scene.spatialAnalysis.centerZone.variance > 1000) {
		LOG(ISC_AWB, Debug) << "LED questionable - high variance suggests misclassification: "
			<< scene.spatialAnalysis.centerZone.variance;
		/* Still applicable but log the concern */
	}

	LOG(ISC_AWB, Debug) << "LED algorithm applicable: variance="
		<< scene.spatialAnalysis.centerZone.variance
		<< " confidence=" << scene.spatialAnalysis.sceneConfidence;

	return true;
}

WhiteBalanceResult DaylightWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = 5500.0f;

	/* Analyze histogram for outdoor characteristics */
	float overallContrast = calculateContrastFromBayer(stats);
	float highlightRatio = calculateHighlightRatio(stats);

	/* Calculate correction strength using spatial analysis */
	double correctionStrength = 1.0;

	/* Validate outdoor scene using histogram data */
	if (overallContrast > 0.6f && highlightRatio > 0.15f) {
		/* High contrast + highlights support outdoor classification */
		correctionStrength = 0.3;  /* Minimal corrections for true outdoor */
		LOG(ISC_AWB, Debug) << "Histogram confirms outdoor scene - minimal correction";

	} else if (scene.spatialAnalysis.isOutdoorScene) {
		/* Spatial says outdoor but histogram suggests otherwise */
		correctionStrength = 0.6;  /* Moderate corrections */
		LOG(ISC_AWB, Debug) << "Spatial outdoor but histogram suggests mixed conditions";

	} else if (scene.spatialAnalysis.isMixedLighting) {
		/* Mixed daylight + indoor */
		correctionStrength = scene.mixedLightingRatio * 0.6;

	} else {
		/* Indoor daylight simulation */
		correctionStrength = scene.overallConfidence;
	}

	/* Handle washout using histogram data */
	if (scene.washoutAnalysis.hasOutdoorWashout && highlightRatio > 0.2f) {
		correctionStrength *= 0.4;  /* Very gentle for severe outdoor washout */
		LOG(ISC_AWB, Debug) << "Severe outdoor washout detected - minimal correction";
	}

	/* Use sky CCT if available and validated by histogram */
	if (scene.spatialAnalysis.isOutdoorScene && scene.spatialAnalysis.topZone.confidence > 0.8) {
		result.colorTemperatureEstimate = scene.spatialAnalysis.topZone.cct;
	}

	/* Apply scaled daylight corrections */
	result.redGain = 1.0f + (1.02f - 1.0f) * correctionStrength;
	result.greenRedGain = 1.0f;
	result.greenBlueGain = 1.0f;
	result.blueGain = 1.0f + (0.98f - 1.0f) * correctionStrength;

	result.redOffset = result.greenRedOffset = result.greenBlueOffset = result.blueOffset = 0;

	result.algorithmConfidence = scene.spatialAnalysis.isOutdoorScene ? 0.95f : (0.8f * correctionStrength);
	result.chromaticityCorrection = scene.chromaticityShift * correctionStrength;
	result.stabilityMetric = scene.spatialAnalysis.isOutdoorScene ? 0.95f : 0.8f;

	return result;
}

bool DaylightWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Enhanced applicability using spatial analysis */

	/* Highly applicable for confirmed outdoor scenes */
	if (scene.spatialAnalysis.isOutdoorScene) {
		LOG(ISC_AWB, Debug) << "Daylight highly applicable for outdoor scene";
		return true;
	}

	/* Applicable for daylight light source */
	if (scene.lightSource == LightSourceType::DAYLIGHT) {
		return true;
	}

	/* Applicable for mixed lighting with daylight component */
	if (scene.lightSource == LightSourceType::MIXED_SOURCES &&
			scene.spatialAnalysis.topZone.cct > 5000) {  /* Cool top zone suggests daylight */
		LOG(ISC_AWB, Debug) << "Daylight applicable for mixed lighting with cool top zone";
		return true;
	}

	return false;
}

WhiteBalanceResult NeutralRegionWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = scene.colorTemperature;

	float avgLuminance = calculateLuminanceFromBayer(stats);
	float contrast = calculateContrastFromBayer(stats);

	/* Neutral region processing using spatial analysis */
	double correctionStrength = 1.0;

	/* Validate neutral regions using histogram characteristics */
	if (avgLuminance > 80.0f && avgLuminance < 180.0f && contrast < 0.6f) {
		/* Good neutral lighting conditions */
		correctionStrength = 1.1;
		LOG(ISC_AWB, Debug) << "Good neutral conditions detected";
	} else if (contrast > 0.8f) {
		/* High contrast may interfere with neutral region detection */
		correctionStrength = 0.7;
		LOG(ISC_AWB, Debug) << "High contrast - reducing neutral correction";
	}

	/* Use center zone for neutral region analysis */
	if (scene.spatialAnalysis.centerZone.confidence > 0.7) {
		/* High center zone confidence - use for neutral correction */
		double centerCCT = scene.spatialAnalysis.centerZone.cct;
		result.colorTemperatureEstimate = centerCCT;
		/* Scale correction based on how neutral the center zone is */
		if (centerCCT > 3500 && centerCCT < 5500) {
			correctionStrength *= 1.1;  /* Good neutral range - boost correction */
		} else {
			correctionStrength *= 0.8;  /* Outside neutral range - reduce correction */
		}
	}

	/* Reduce correction for mixed lighting */
	if (scene.spatialAnalysis.isMixedLighting) {
		correctionStrength *= scene.mixedLightingRatio;
	}

	/* Reduce correction for outdoor scenes (natural colors preferred) */
	if (scene.spatialAnalysis.isOutdoorScene) {
		correctionStrength *= 0.6;
	}

	/* Apply neutral region corrections */
	result.redGain = 1.0f + (1.1f - 1.0f) * correctionStrength;
	result.greenRedGain = 1.0f;
	result.greenBlueGain = 1.0f;
	result.blueGain = 1.0f + (1.1f - 1.0f) * correctionStrength;

	result.redOffset = result.greenRedOffset = result.greenBlueOffset = result.blueOffset = 0;

	result.algorithmConfidence = scene.spatialAnalysis.centerZone.confidence * 0.8f;
	result.chromaticityCorrection = scene.chromaticityShift * correctionStrength;
	result.stabilityMetric = 0.9f;

	return result;
}

bool NeutralRegionWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Applicable when skin tones detected (original logic) */
	if (scene.hasSkinTonesDetected) {
		return true;
	}

	/* Enhanced applicability using spatial analysis */
	/* Good for high overall confidence scenes */
	if (scene.overallConfidence > 0.8f && !scene.spatialAnalysis.isMixedLighting) {
		LOG(ISC_AWB, Debug) << "Neutral region applicable for high confidence scene";
		return true;
	}

	/* Good when center zone has high confidence and neutral CCT */
	if (scene.spatialAnalysis.centerZone.confidence > 0.8 &&
			scene.spatialAnalysis.centerZone.cct > 3500 &&
			scene.spatialAnalysis.centerZone.cct < 5500) {
		LOG(ISC_AWB, Debug) << "Neutral region applicable for neutral center zone";
		return true;
	}

	return false;
}

WhiteBalanceResult MultiIlluminantWhiteBalance::process(const ImageStats &stats, const UnifiedSceneAnalysis &scene)
{
	WhiteBalanceResult result;
	result.algorithmName = getName();
	result.colorTemperatureEstimate = scene.colorTemperature;

	float overallContrast = calculateContrastFromBayer(stats);
	float avgLuminance = calculateLuminanceFromBayer(stats);

	/* Multi-illuminant processing using spatial zone analysis */
	if (!scene.spatialAnalysis.isMixedLighting) {
		/* Fallback to neutral if not actually mixed lighting */
		result.redGain = result.greenRedGain = result.greenBlueGain = result.blueGain = 1.0f;
		result.algorithmConfidence = 0.5f;
		LOG(ISC_AWB, Debug) << "Multi-illuminant fallback - no mixed lighting detected";
		return result;
	}

	/* Validate mixed lighting with histogram characteristics */
	if (overallContrast < 0.4f) {
		/* Low contrast suggests uniform lighting, not mixed */
		result.redGain = result.greenRedGain = result.greenBlueGain = result.blueGain = 1.0f;
		result.algorithmConfidence = 0.3f;
		LOG(ISC_AWB, Debug) << "Multi-illuminant: low contrast suggests uniform lighting";
		return result;
	}

	/* Calculate zone-specific contributions */
	double topWeight = scene.spatialAnalysis.topZone.confidence;
	double centerWeight = scene.spatialAnalysis.centerZone.confidence;
	double bottomWeight = scene.spatialAnalysis.bottomZone.confidence;
	double totalWeight = topWeight + centerWeight + bottomWeight;

	if (totalWeight < 0.1) {
		/* Insufficient zone data - use neutral */
		result.redGain = result.greenRedGain = result.greenBlueGain = result.blueGain = 1.0f;
		result.algorithmConfidence = 0.3f;
		LOG(ISC_AWB, Debug) << "Multi-illuminant: insufficient zone confidence";
		return result;
	}

	/* Weight corrections based on zone CCTs and confidence */
	double avgZoneCCT = (scene.spatialAnalysis.topZone.cct * topWeight +
			scene.spatialAnalysis.centerZone.cct * centerWeight +
			scene.spatialAnalysis.bottomZone.cct * bottomWeight) / totalWeight;

	/* Multi-illuminant correction strength based on CCT variance and histogram validation */
	double correctionStrength = scene.mixedLightingRatio;

	/* Adjust strength based on histogram characteristics */
	if (avgLuminance < 60.0f) {
		correctionStrength *= 1.2;  /* Boost for low light mixed conditions */
	} else if (avgLuminance > 200.0f) {
		correctionStrength *= 0.8;  /* Reduce for bright mixed conditions */
	}

	/* Apply zone-weighted corrections */
	if (avgZoneCCT < 3500) {
		/* Warm-dominant mixed lighting */
		result.redGain = 1.0f + (0.9f - 1.0f) * correctionStrength;
		result.greenRedGain = 1.0f;
		result.greenBlueGain = 1.0f;
		result.blueGain = 1.0f + (1.4f - 1.0f) * correctionStrength;
	} else if (avgZoneCCT > 5500) {
		/* Cool-dominant mixed lighting */
		result.redGain = 1.0f + (1.2f - 1.0f) * correctionStrength;
		result.greenRedGain = 1.0f + (0.95f - 1.0f) * correctionStrength;
		result.greenBlueGain = 1.0f + (0.95f - 1.0f) * correctionStrength;
		result.blueGain = 1.0f + (0.9f - 1.0f) * correctionStrength;
	} else {
		/* Neutral mixed lighting */
		result.redGain = 1.0f + (1.1f - 1.0f) * correctionStrength;
		result.greenRedGain = 1.0f + (0.95f - 1.0f) * correctionStrength;
		result.greenBlueGain = 1.0f + (0.95f - 1.0f) * correctionStrength;
		result.blueGain = 1.0f + (1.2f - 1.0f) * correctionStrength;
	}

	/* No offsets for multi-illuminant (too complex) */
	result.redOffset = result.greenRedOffset = result.greenBlueOffset = result.blueOffset = 0;

	result.algorithmConfidence = scene.spatialAnalysis.sceneConfidence * 0.9f;
	result.chromaticityCorrection = scene.chromaticityShift;
	result.stabilityMetric = 0.7f;  /* Lower stability due to complex lighting */

	LOG(ISC_AWB, Debug) << "Multi-illuminant: avgCCT=" << avgZoneCCT
		<< " strength=" << correctionStrength
		<< " zones=" << topWeight << "/" << centerWeight << "/" << bottomWeight
		<< " contrast=" << overallContrast;

	return result;
}

bool MultiIlluminantWhiteBalance::isApplicable(const UnifiedSceneAnalysis &scene)
{
	/* Specifically designed for mixed lighting scenarios */
	if (!scene.spatialAnalysis.isMixedLighting) {
		LOG(ISC_AWB, Debug) << "Multi-illuminant not applicable - no mixed lighting";
		return false;
	}

	/* Check if we have sufficient zone confidence for multi-illuminant processing */
	int validZones = (scene.spatialAnalysis.topZone.confidence > 0.5) +
		(scene.spatialAnalysis.centerZone.confidence > 0.5) +
		(scene.spatialAnalysis.bottomZone.confidence > 0.5);

	if (validZones < 2) {
		LOG(ISC_AWB, Debug) << "Multi-illuminant not applicable - insufficient valid zones";
		return false;
	}

	/* Require significant CCT variance to justify multi-illuminant approach */
	if (scene.spatialAnalysis.cctVariance < 200000) {  /* 450K variance threshold */
		LOG(ISC_AWB, Debug) << "Multi-illuminant not applicable - insufficient CCT variance";
		return false;
	}

	LOG(ISC_AWB, Debug) << "Multi-illuminant applicable: validZones=" << validZones
		<< " cctVar=" << scene.spatialAnalysis.cctVariance;
	return true;
}

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

