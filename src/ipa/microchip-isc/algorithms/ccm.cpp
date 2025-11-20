/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Professional Color Correction Matrix with Unified Scene Analysis
 */
#include "ccm.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <iomanip>

namespace libcamera {
namespace ipa::microchip_isc {

LOG_DEFINE_CATEGORY(ISC_CCM)

	CCM::CCM()
	: is_converged_(false), frame_count_(0), contextFluorescentStrength_(0.0f)
	{
		/* Initialize unified scene analyzer for fallback */
		sceneAnalyzer_ = std::make_unique<UnifiedSceneAnalyzer>();

		temporal_history_.reserve(TEMPORAL_HISTORY_SIZE);
		convergence_history_.reserve(CONVERGENCE_HISTORY_SIZE);

		/* Initialize with identity matrix */
		current_profile_ = {
			.name = "Identity",
			.cct_min = 0, .cct_max = 12000,
			.matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
			.offset = {0.0f, 0.0f, 0.0f},
			.saturation_boost = 1.0f,
			.accuracy_weight = 0.5f,
			.fluorescent_bias = 0.0f,
			.tungsten_bias = 0.0f
		};
	}

void CCM::configure(const MicrochipISCSensorInfo &sensorInfo)
{
	LOG(ISC_CCM, Debug) << "Configuring Professional CCM for unified scene analysis - sensor "
		<< sensorInfo.model << " resolution: " << sensorInfo.width << "x" << sensorInfo.height;

	sensorInfo_ = sensorInfo;
	initializeProfessionalProfiles();
	temporal_history_.clear();
	convergence_history_.clear();
	is_converged_ = false;
	frame_count_ = 0;
	contextFluorescentStrength_ = 0.0f;

	LOG(ISC_CCM, Debug) << "CCM configured with " << ccm_profiles_.size()
		<< " professional profiles (hardware scaling: 256=1.0, range: "
		<< V4L2_MIN_VALUE << " to " << V4L2_MAX_VALUE << ")";
}

void CCM::initializeProfessionalProfiles()
{
	ccm_profiles_.clear();

	/* Identity - Safe fallback */
	ccm_profiles_.push_back({
			.name = "Identity",
			.cct_min = 0, .cct_max = 12000,
			.matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
			.offset = {0.0f, 0.0f, 0.0f},
			.saturation_boost = 1.0f,
			.accuracy_weight = 0.3f,
			.fluorescent_bias = 0.0f,
			.tungsten_bias = 0.0f
			});

	/* NEW: LED Neutral - Minimal correction for modern LED lighting */
	ccm_profiles_.push_back({
			.name = "LED_Neutral",
			.cct_min = 3800, .cct_max = 5200,
			.matrix = {
			1.00f, -0.01f, -0.01f,    /* Red: minimal boost */
			-0.01f, 1.00f,  0.01f,    /* Green: nearly neutral */
			0.00f, -0.02f,  1.03f     /* Blue: minimal boost */
			},
			.offset = {0.0f, 0.0f, 0.0f},
			.saturation_boost = 1.05f,
			.accuracy_weight = 0.95f,
			.fluorescent_bias = 0.0f,
			.tungsten_bias = 0.0f
			});

	/* Aggressive Tungsten Correction */
	ccm_profiles_.push_back({
			.name = "Tungsten_Dramatic",
			.cct_min = 2200, .cct_max = 3200,
			.matrix = {
			1.20f, -0.10f, -0.10f,
			-0.30f, 1.25f, 0.05f,
			-0.25f, -0.90f, 2.15f
			},
			.offset = {-25.0f, 15.0f, 55.0f},
			.saturation_boost = 1.35f,
			.accuracy_weight = 0.92f,
			.fluorescent_bias = 0.0f,
			.tungsten_bias = 1.0f
			});

	/* Anti-Fluorescent Green Cast */
	ccm_profiles_.push_back({
			.name = "Fluorescent_AntiGreen",
			.cct_min = 3800, .cct_max = 5200,
			.matrix = {
			2.00f, -0.60f, -0.40f,
			-0.60f, 0.60f, 0.00f,
			-0.20f, -0.40f, 1.60f
			},
			.offset = {40.0f, -80.0f, 35.0f},
			.saturation_boost = 1.6f,
			.accuracy_weight = 0.90f,
			.fluorescent_bias = 1.0f,
			.tungsten_bias = 0.0f
			});

	/* Extreme Anti-Fluorescent for severe cases */
	ccm_profiles_.push_back({
			.name = "Fluorescent_Extreme",
			.cct_min = 3800, .cct_max = 5200,
			.matrix = {
			2.20f, -0.75f, -0.45f,
			-0.70f, 0.50f, 0.20f,
			-0.25f, -0.50f, 1.75f
			},
			.offset = {50.0f, -100.0f, 60.0f},
			.saturation_boost = 1.8f,
			.accuracy_weight = 0.95f,
			.fluorescent_bias = 1.0f,
			.tungsten_bias = 0.0f
			});

	/* Enhanced Daylight */
	ccm_profiles_.push_back({
			.name = "Daylight_Vivid",
			.cct_min = 5200, .cct_max = 6800,
			.matrix = {
			1.55f, -0.35f, -0.20f,
			-0.25f, 1.45f, -0.20f,
			0.05f, -0.35f, 1.30f
			},
			.offset = {15.0f, -10.0f, 5.0f},
			.saturation_boost = 1.25f,
			.accuracy_weight = 0.95f,
			.fluorescent_bias = 0.0f,
			.tungsten_bias = 0.0f
			});

	/* Cool Light Balance */
	ccm_profiles_.push_back({
			.name = "CoolLight_Warmed",
			.cct_min = 6800, .cct_max = 9000,
			.matrix = {
			1.80f, -0.40f, -0.40f,
			-0.20f, 1.40f, -0.20f,
			0.15f, -0.50f, 1.35f
			},
			.offset = {20.0f, -5.0f, -25.0f},
			.saturation_boost = 1.30f,
			.accuracy_weight = 0.88f,
			.fluorescent_bias = 0.0f,
			.tungsten_bias = 0.0f
			});

	/* Mixed Lighting Adaptive */
	ccm_profiles_.push_back({
			.name = "Mixed_Adaptive",
			.cct_min = 3500, .cct_max = 5500,
			.matrix = {
			1.35f, -0.25f, -0.10f,
			-0.25f, 1.35f, -0.10f,
			-0.10f, -0.45f, 1.55f
			},
			.offset = {0.0f, -15.0f, 20.0f},
			.saturation_boost = 1.15f,
			.accuracy_weight = 0.85f,
			.fluorescent_bias = 0.5f,
			.tungsten_bias = 0.3f
			});

	LOG(ISC_CCM, Debug) << "Initialized " << ccm_profiles_.size()
		<< " CCM profiles including LED_Neutral";
}

void CCM::process(const ImageStats &stats, ControlList &results)
{
	/* Use internal scene analyzer as fallback */
	UnifiedSceneAnalysis scene = sceneAnalyzer_->analyzeScene(stats);
	processWithSceneAnalysis(stats, scene, results);
}

void CCM::processWithSceneAnalysis(const ImageStats &stats, const UnifiedSceneAnalysis &scene, ControlList &results)
{
	frame_count_++;

	/* Use ImageStats to validate scene analysis quality and enhance processing */
	uint32_t totalPixels = stats.totalPixelsGR + stats.totalPixelsR + stats.totalPixelsGB + stats.totalPixelsB;
	float avgBrightness = calculateLuminanceFromBayer(stats);
	bool hasOverexposure = detectOverexposure(stats, 0.02f);
	bool hasUnderexposure = detectUnderexposure(stats, 0.15f);

	LOG(ISC_CCM, Debug) << "Processing CCM frame " << frame_count_
		<< " with unified scene analysis - " << totalPixels << " total pixels"
		<< " brightness=" << std::fixed << std::setprecision(1) << avgBrightness
		<< " overexp=" << (hasOverexposure ? "YES" : "NO")
		<< " underexp=" << (hasUnderexposure ? "YES" : "NO");

	try {
		/* Validate scene analysis against actual image statistics */
		if (totalPixels < 1000) {
			LOG(ISC_CCM, Warning) << "Insufficient pixel data for reliable CCM - using identity";
			applyIdentityMatrix(results);
			return;
		}

		/* Cross-validate scene brightness with actual statistics */
		float brightnessDifference = std::abs(avgBrightness - scene.illuminationStrength);
		if (brightnessDifference > 50.0f) {
			LOG(ISC_CCM, Warning) << "Scene analysis brightness mismatch: measured="
				<< avgBrightness << " vs scene=" << scene.illuminationStrength;
		}

		/* Use statistics to enhance CCM selection */
		if (hasOverexposure && scene.lightSource == LightSourceType::FLUORESCENT) {
			/* Apply more aggressive fluorescent correction for overexposed scenes */
			contextFluorescentStrength_ = std::min(1.0f, contextFluorescentStrength_ + 0.2f);
			LOG(ISC_CCM, Debug) << "Enhanced fluorescent correction for overexposed scene";
		}

		/* Calculate optimal CCM for this specific scene */
		CCMProfile optimal_ccm = calculateOptimalCCM(scene);

		/* Adjust saturation based on exposure statistics */
		if (hasUnderexposure) {
			optimal_ccm.saturation_boost *= 1.1f;  /* Boost saturation for underexposed scenes */
		} else if (hasOverexposure) {
			optimal_ccm.saturation_boost *= 0.9f;  /* Reduce saturation for overexposed scenes */
		}

		/* Apply temporal smoothing to prevent flickering */
		CCMProfile smoothed_ccm = temporalSmoothing(optimal_ccm);

		/* Validate and apply to hardware */
		if (validateCCMMatrix(smoothed_ccm)) {
			AccuracyMetrics metrics = evaluateCCMAccuracy(smoothed_ccm, scene);
			applyCCMToHardware(smoothed_ccm, results);
			current_profile_ = smoothed_ccm;

			/* Update convergence history */
			convergence_history_.push_back(smoothed_ccm);
			if (convergence_history_.size() > CONVERGENCE_HISTORY_SIZE) {
				convergence_history_.erase(convergence_history_.begin());
			}

			logProfileApplication(smoothed_ccm, metrics);
		} else {
			LOG(ISC_CCM, Warning) << "CCM validation failed - applying identity matrix";
			applyIdentityMatrix(results);
		}

		logSceneAnalysis(scene);
	} catch (const std::exception &e) {
		LOG(ISC_CCM, Warning) << "CCM processing failed: " << e.what()
			<< " - applying identity matrix";
		applyIdentityMatrix(results);
	}
}

void CCM::setSceneAnalysisContext(float fluorescentStrength)
{
	contextFluorescentStrength_ = fluorescentStrength;
	LOG(ISC_CCM, Debug) << "Updated scene analysis context: fluorescent strength="
		<< fluorescentStrength;
}

CCM::CCMProfile CCM::calculateOptimalCCM(const UnifiedSceneAnalysis &scene)
{
	LOG(ISC_CCM, Debug) << "=== CCM Profile Selection ===";
	LOG(ISC_CCM, Debug) << "Scene: " << lightSourceTypeToString(scene.lightSource)
		<< " CCT=" << scene.colorTemperature << "K"
		<< " outdoor=" << (scene.environment == EnvironmentType::OUTDOOR ? "1" : "0");

	CCMProfile bestProfile = ccm_profiles_[0];  /* Identity fallback */
	float bestScore = 0.0f;
	std::string bestProfileName = "Identity (fallback)";

	for (const auto &profile : ccm_profiles_) {
		float score = 0.0f;

		/* Base score from profile accuracy weight */
		score += profile.accuracy_weight * 0.5f;
		LOG(ISC_CCM, Debug) << "  " << profile.name << ": base=" << score;

		/* CCT range scoring - critical for profile selection */
		if (scene.colorTemperature >= profile.cct_min &&
				scene.colorTemperature <= profile.cct_max) {
			score += 1.0f;
			LOG(ISC_CCM, Debug) << "    CCT in range: +1.0 -> " << score;
		} else {
			/* Penalize profiles outside CCT range */
			uint32_t distance = std::min(
					std::abs(static_cast<int32_t>(scene.colorTemperature) - static_cast<int32_t>(profile.cct_min)),
					std::abs(static_cast<int32_t>(scene.colorTemperature) - static_cast<int32_t>(profile.cct_max))
					);
			float penalty = std::min(1.0f, distance / 1000.0f);
			score -= penalty;
			LOG(ISC_CCM, Debug) << "    CCT outside range: -" << penalty << " -> " << score;
		}

		/* Profile-specific scoring with outdoor/indoor context */
		if (profile.name == "Identity") {
			/* Identity - neutral baseline, no special handling */

		} else if (profile.name == "LED_Neutral") {
			/* LED is indoor-only */
			if (scene.lightSource == LightSourceType::LED) {
				if (scene.environment == EnvironmentType::OUTDOOR) {
					score -= 3.0f;  /* LED cannot exist outdoors */
					LOG(ISC_CCM, Debug) << "    OUTDOOR overrides LED: -3.0 -> " << score;
				} else {
					score += 2.5f;  /* Strong indoor LED match */
					LOG(ISC_CCM, Debug) << "    LED MATCH (indoor): +2.5 -> " << score;
				}
			} else if (profile.fluorescent_bias > 0.5f || profile.tungsten_bias > 0.5f) {
				score -= 5.0f;  /* Reject aggressive profiles */
				LOG(ISC_CCM, Debug) << "    LED rejects aggressive profile: -5.0 -> " << score;
			}

			/* Additional outdoor rejection for LED profile */
			if (scene.environment == EnvironmentType::OUTDOOR) {
				score -= 2.0f;  /* General outdoor penalty */
				LOG(ISC_CCM, Debug) << "    OUTDOOR rejects indoor LED profile: -2.0 -> " << score;
			}

		} else if (profile.name == "Tungsten_Dramatic") {
			/* Tungsten is indoor-only */
			if (scene.lightSource == LightSourceType::INCANDESCENT) {
				score += 2.0f;
				LOG(ISC_CCM, Debug) << "    TUNGSTEN MATCH: +2.0 -> " << score;
			}

			if (scene.environment == EnvironmentType::OUTDOOR) {
				score -= 3.5f;  /* Strong rejection - tungsten is always indoor */
				LOG(ISC_CCM, Debug) << "    OUTDOOR rejects tungsten: -3.5 -> " << score;
			}

		} else if (profile.name == "Fluorescent_AntiGreen") {
			/* Fluorescent is indoor-only */
			if (scene.lightSource == LightSourceType::FLUORESCENT) {
				score += 2.0f;
				LOG(ISC_CCM, Debug) << "    FLUORESCENT MATCH: +2.0 -> " << score;
			} else if (scene.lightSource == LightSourceType::LED) {
				score -= 5.0f;
				LOG(ISC_CCM, Debug) << "    LED rejects aggressive profile: -5.0 -> " << score;
			}

			if (scene.environment == EnvironmentType::OUTDOOR) {
				score -= 3.5f;  /* Strong rejection - fluorescent is indoor */
				LOG(ISC_CCM, Debug) << "    OUTDOOR rejects fluorescent: -3.5 -> " << score;
			}

		} else if (profile.name == "Fluorescent_Extreme") {
			/* Extreme fluorescent is indoor-only */
			if (scene.lightSource == LightSourceType::FLUORESCENT) {
				/* Check if extreme correction is needed */
				if (scene.chromaticityShift > 0.7f) {
					score += 2.0f;
					LOG(ISC_CCM, Debug) << "    EXTREME FLUORESCENT: +2.0 -> " << score;
				} else {
					score += 1.8f;
					LOG(ISC_CCM, Debug) << "    FLUORESCENT MATCH: +1.8 -> " << score;
				}
			} else if (scene.lightSource == LightSourceType::LED) {
				score -= 5.0f;
				LOG(ISC_CCM, Debug) << "    LED rejects aggressive profile: -5.0 -> " << score;
			}

			if (scene.environment == EnvironmentType::OUTDOOR) {
				score -= 3.5f;  /* Strong rejection */
				LOG(ISC_CCM, Debug) << "    OUTDOOR rejects fluorescent extreme: -3.5 -> " << score;
			}

		} else if (profile.name == "Daylight_Vivid") {
			/* Daylight is primarily outdoor */
			if (scene.lightSource == LightSourceType::DAYLIGHT) {
				score += 2.5f;  /* Increased from 1.8 */
				LOG(ISC_CCM, Debug) << "    DAYLIGHT MATCH: +2.5 -> " << score;
			}

			/* Strong outdoor preference */
			if (scene.environment == EnvironmentType::OUTDOOR) {
				score += 1.5f;  /* Additional outdoor boost */
				LOG(ISC_CCM, Debug) << "    OUTDOOR confirms daylight: +1.5 -> " << score;
			} else {
				score -= 1.0f;  /* Mild penalty for indoor daylight (less common) */
				LOG(ISC_CCM, Debug) << "    INDOOR reduces daylight preference: -1.0 -> " << score;
			}

		} else if (profile.name == "CoolLight_Warmed") {
			/* Cool light is typically indoor (office LED, fluorescent) */
			if (scene.environment == EnvironmentType::OUTDOOR) {
				score -= 2.5f;  /* Reject for outdoor */
				LOG(ISC_CCM, Debug) << "    OUTDOOR rejects cool light: -2.5 -> " << score;
			}

		} else if (profile.name == "Mixed_Adaptive") {
			/* Mixed lighting handling */
			if (scene.spatialAnalysis.isMixedLighting) {
				score += 1.5f;
				LOG(ISC_CCM, Debug) << "    MIXED LIGHTING MATCH: +1.5 -> " << score;
			}
			/* Mixed_Adaptive works both indoor and outdoor - no penalty */
		}

		/* Scene confidence weighting */
		score *= scene.overallConfidence;
		LOG(ISC_CCM, Debug) << "    After confidence (" << scene.overallConfidence << "): " << score;

		/* Update best if this is better */
		if (score > bestScore) {
			bestScore = score;
			bestProfile = profile;
			bestProfileName = profile.name;
			LOG(ISC_CCM, Debug) << "    *** NEW BEST: " << profile.name << " score=" << score;
		} else {
			LOG(ISC_CCM, Debug) << "    Not better than current best (" << bestScore << ")";
		}
	}

	LOG(ISC_CCM, Info) << "Selected CCM: " << bestProfileName
		<< " (score=" << bestScore << ")";

	return bestProfile;
}

CCM::CCMProfile CCM::createAdaptiveProfile(const UnifiedSceneAnalysis &scene)
{
	CCMProfile adaptive;
	adaptive.name = "Adaptive_Generated";
	adaptive.cct_min = static_cast<uint32_t>(scene.colorTemperature) - 500;
	adaptive.cct_max = static_cast<uint32_t>(scene.colorTemperature) + 500;
	adaptive.accuracy_weight = 0.7f;

	/* Start with identity matrix */
	adaptive.matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	adaptive.offset = {0.0f, 0.0f, 0.0f};
	adaptive.saturation_boost = 1.0f;
	adaptive.fluorescent_bias = (scene.lightSource == LightSourceType::FLUORESCENT) ? 1.0f : 0.0f;
	adaptive.tungsten_bias = (scene.lightSource == LightSourceType::INCANDESCENT) ? 1.0f : 0.0f;

	/* Apply scene-based dynamic adjustments using unified scene analysis */
	if (scene.lightSource == LightSourceType::FLUORESCENT || scene.chromaticityShift > 0.3f) {
		adaptive.matrix[4] *= 0.80f;  /* Reduce green channel */
		adaptive.matrix[0] *= 1.25f;  /* Boost red against green cast */
		adaptive.matrix[8] *= 1.15f;  /* Boost blue for balance */
		adaptive.offset[1] = -30.0f;  /* Negative green offset */
		adaptive.saturation_boost = 1.3f;
		LOG(ISC_CCM, Debug) << "Adaptive: Applied fluorescent correction";
	}

	if (scene.lightSource == LightSourceType::INCANDESCENT || scene.colorTemperature < 3500.0f) {
		adaptive.matrix[8] *= 1.5f;   /* Strong blue boost for tungsten */
		adaptive.matrix[0] *= 0.95f;  /* Slight red reduction */
		adaptive.offset[2] = 40.0f;   /* Blue offset */
		adaptive.saturation_boost = 1.25f;
		LOG(ISC_CCM, Debug) << "Adaptive: Applied tungsten correction";
	}

	if (scene.lightSource == LightSourceType::DAYLIGHT || scene.environment == EnvironmentType::OUTDOOR) {
		adaptive.matrix[0] *= 1.2f;   /* Enhance red for outdoor scenes */
		adaptive.matrix[8] *= 1.1f;   /* Enhance blue for sky */
		adaptive.saturation_boost = 1.15f;
		LOG(ISC_CCM, Debug) << "Adaptive: Applied daylight enhancement";
	}

	return adaptive;
}

float CCM::calculateColorAccuracy(const CCMProfile &profile, uint32_t scene_cct)
{
	if (scene_cct >= profile.cct_min && scene_cct <= profile.cct_max) {
		return profile.accuracy_weight;
	}

	uint32_t distance = std::min(
			std::abs(int32_t(scene_cct) - int32_t(profile.cct_min)),
			std::abs(int32_t(scene_cct) - int32_t(profile.cct_max))
			);
	float penalty = std::min(1.0f, distance / 1500.0f);
	return profile.accuracy_weight * (1.0f - penalty);
}

CCM::CCMProfile CCM::temporalSmoothing(const CCMProfile &new_profile)
{
	if (temporal_history_.empty() || frame_count_ <= 3) {
		return new_profile;
	}

	/* Check if profile type changed */
	if (new_profile.name != current_profile_.name) {
		LOG(ISC_CCM, Debug) << "Profile change: " << current_profile_.name
			<< " -> " << new_profile.name << " (applying transition smoothing)";
		CCMProfile smoothed = new_profile;
		float smoothing_factor = 0.4f; /* Moderate smoothing for transitions */

		for (size_t i = 0; i < 9; i++) {
			smoothed.matrix[i] = smoothing_factor * current_profile_.matrix[i] +
				(1.0f - smoothing_factor) * new_profile.matrix[i];
		}
		for (size_t i = 0; i < 3; i++) {
			smoothed.offset[i] = smoothing_factor * current_profile_.offset[i] +
				(1.0f - smoothing_factor) * new_profile.offset[i];
		}
		return smoothed;
	}

	/* Same profile - apply stability smoothing */
	CCMProfile smoothed = new_profile;
	float smoothing_factor = 0.7f; /* Strong smoothing for stability */

	for (size_t i = 0; i < 9; i++) {
		smoothed.matrix[i] = smoothing_factor * current_profile_.matrix[i] +
			(1.0f - smoothing_factor) * new_profile.matrix[i];
	}

	/* Update history */
	temporal_history_.push_back(smoothed);
	if (temporal_history_.size() > TEMPORAL_HISTORY_SIZE) {
		temporal_history_.erase(temporal_history_.begin());
	}

	return smoothed;
}

bool CCM::validateCCMMatrix(const CCMProfile &profile)
{
	/* Check matrix determinant */
	float det = matrixDeterminant3x3(profile.matrix);
	if (std::abs(det) < 0.1f) {
		LOG(ISC_CCM, Warning) << "Matrix determinant too small: " << det;
		return false;
	}

	/* Check matrix element ranges */
	for (float value : profile.matrix) {
		if (std::abs(value) > 3.0f) {
			LOG(ISC_CCM, Warning) << "Matrix element out of range: " << value;
			return false;
		}
	}

	/* Check offset ranges */
	for (float offset : profile.offset) {
		if (std::abs(offset) > 127.0f) {
			LOG(ISC_CCM, Warning) << "Offset out of range: " << offset;
			return false;
		}
	}

	return true;
}

float CCM::matrixDeterminant3x3(const std::array<float, 9> &matrix)
{
	return matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
		matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
		matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
}

void CCM::applyCCMToHardware(const CCMProfile &profile, ControlList &results)
{
	/* Convert matrix values to hardware scale (256 = 1.0) using mojom control IDs */
	results.set(V4L2_CID_ISC_CC_RR, clampHardwareValue(profile.matrix[0] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_RG, clampHardwareValue(profile.matrix[1] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_RB, clampHardwareValue(profile.matrix[2] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_GR, clampHardwareValue(profile.matrix[3] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_GG, clampHardwareValue(profile.matrix[4] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_GB, clampHardwareValue(profile.matrix[5] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_BR, clampHardwareValue(profile.matrix[6] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_BG, clampHardwareValue(profile.matrix[7] * HW_MATRIX_SCALE));
	results.set(V4L2_CID_ISC_CC_BB, clampHardwareValue(profile.matrix[8] * HW_MATRIX_SCALE));

	/* Apply offsets (no scaling) */
	results.set(V4L2_CID_ISC_CC_OR, static_cast<int32_t>(profile.offset[0]));
	results.set(V4L2_CID_ISC_CC_OG, static_cast<int32_t>(profile.offset[1]));
	results.set(V4L2_CID_ISC_CC_OB, static_cast<int32_t>(profile.offset[2]));

	LOG(ISC_CCM, Debug) << "Applied CCM to hardware: " << profile.name
		<< " (RR=" << clampHardwareValue(profile.matrix[0] * HW_MATRIX_SCALE)
		<< ", GG=" << clampHardwareValue(profile.matrix[4] * HW_MATRIX_SCALE)
		<< ", BB=" << clampHardwareValue(profile.matrix[8] * HW_MATRIX_SCALE) << ")";
}

void CCM::applyIdentityMatrix(ControlList &results)
{
	results.set(V4L2_CID_ISC_CC_RR, IDENTITY_VALUE);
	results.set(V4L2_CID_ISC_CC_RG, 0);
	results.set(V4L2_CID_ISC_CC_RB, 0);
	results.set(V4L2_CID_ISC_CC_GR, 0);
	results.set(V4L2_CID_ISC_CC_GG, IDENTITY_VALUE);
	results.set(V4L2_CID_ISC_CC_GB, 0);
	results.set(V4L2_CID_ISC_CC_BR, 0);
	results.set(V4L2_CID_ISC_CC_BG, 0);
	results.set(V4L2_CID_ISC_CC_BB, IDENTITY_VALUE);
	results.set(V4L2_CID_ISC_CC_OR, 0);
	results.set(V4L2_CID_ISC_CC_OG, 0);
	results.set(V4L2_CID_ISC_CC_OB, 0);

	LOG(ISC_CCM, Debug) << "Applied identity matrix";
}

int32_t CCM::clampHardwareValue(float value)
{
	int32_t hw_value = static_cast<int32_t>(std::round(value));
	return std::clamp(hw_value, V4L2_MIN_VALUE, V4L2_MAX_VALUE);
}

CCM::AccuracyMetrics CCM::evaluateCCMAccuracy(const CCMProfile &profile, const UnifiedSceneAnalysis &scene)
{
	AccuracyMetrics metrics = {};
	metrics.neutral_accuracy = calculateColorAccuracy(profile, static_cast<uint32_t>(scene.colorTemperature));
	metrics.skin_tone_accuracy = profile.accuracy_weight *
		(scene.hasSkinTonesDetected ? 0.9f : 0.7f);
	metrics.saturation_preservation = std::clamp(profile.saturation_boost, 0.8f, 1.0f);
	metrics.overall_score = (metrics.neutral_accuracy + metrics.skin_tone_accuracy +
			metrics.saturation_preservation) / 3.0f;
	return metrics;
}

void CCM::logSceneAnalysis(const UnifiedSceneAnalysis &scene)
{
	if (frame_count_ % 30 == 0) {
		LOG(ISC_CCM, Debug) << "Unified Scene Analysis: CCT=" << scene.colorTemperature << "K"
			<< " source=" << lightSourceTypeToString(scene.lightSource)
			<< " env=" << environmentTypeToString(scene.environment)
			<< " chromaticity=" << std::fixed << std::setprecision(2)
			<< scene.chromaticityShift
			<< " confidence=" << scene.overallConfidence;
	}
}

void CCM::logProfileApplication(const CCMProfile &profile, const AccuracyMetrics &metrics)
{
	LOG(ISC_CCM, Debug) << "Applied " << profile.name
		<< " (accuracy: " << std::fixed << std::setprecision(1)
		<< (metrics.overall_score * 100) << "%, saturation: +"
		<< ((profile.saturation_boost - 1.0f) * 100) << "%)";

	if (frame_count_ % 30 == 0) {
		LOG(ISC_CCM, Debug) << "Matrix: [" << std::setprecision(2)
			<< profile.matrix[0] << "," << profile.matrix[1] << "," << profile.matrix[2] << "; "
			<< profile.matrix[3] << "," << profile.matrix[4] << "," << profile.matrix[5] << "; "
			<< profile.matrix[6] << "," << profile.matrix[7] << "," << profile.matrix[8] << "]";
	}
}

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */
