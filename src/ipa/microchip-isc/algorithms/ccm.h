/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Professional Color Correction Matrix with Unified Scene Analysis
 */
#ifndef IPA_MICROCHIP_ISC_CCM_H
#define IPA_MICROCHIP_ISC_CCM_H

#include <libcamera/base/log.h>
#include <libcamera/controls.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>
#include "common.h"
#include <array>
#include <vector>
#include <string>
#include <memory>

namespace libcamera {
class ControlList;

namespace ipa::microchip_isc {

class CCM {
public:
	/* Professional CCM profile structure */
	struct CCMProfile {
		std::string name;
		uint32_t cct_min, cct_max;
		std::array<float, 9> matrix;            /* 3x3 transformation matrix */
		std::array<float, 3> offset;            /* RGB offset values */
		float saturation_boost;                 /* Saturation enhancement */
		float accuracy_weight;                  /* Profile accuracy rating */
		float fluorescent_bias;                 /* Fluorescent lighting preference */
		float tungsten_bias;                    /* Tungsten lighting preference */
	};

	/* Accuracy metrics */
	struct AccuracyMetrics {
		float neutral_accuracy;
		float skin_tone_accuracy;
		float saturation_preservation;
		float overall_score;
	};

	CCM();
	void configure(const MicrochipISCSensorInfo &sensorInfo);

	/* Updated interface to use unified scene analysis */
	void process(const ImageStats &stats, ControlList &results);
	void processWithSceneAnalysis(const ImageStats &stats, const UnifiedSceneAnalysis &scene, ControlList &results);

	/* Scene analysis configuration */
	void setSceneAnalysisConfig(const SceneAnalysisConfig &config) { sceneAnalyzer_->setConfig(config); }

	/* Scene analysis context setter for IPA integration */
	void setSceneAnalysisContext(float fluorescentStrength);

private:
	/* Hardware scaling constants */
	static constexpr int32_t HW_MATRIX_SCALE = 256;     /* Hardware: 1.0 = 256 */
	static constexpr int32_t V4L2_MAX_VALUE = 2047;     /* 12-bit signed max */
	static constexpr int32_t V4L2_MIN_VALUE = -2048;    /* 12-bit signed min */
	static constexpr int32_t IDENTITY_VALUE = 256;      /* Identity matrix value */

	/* Algorithm constants */
	static constexpr uint32_t DEFAULT_CCT = 5500;
	static constexpr size_t TEMPORAL_HISTORY_SIZE = 4;
	static constexpr size_t CONVERGENCE_HISTORY_SIZE = 8;

	/* Unified scene analyzer for fallback */
	std::unique_ptr<UnifiedSceneAnalyzer> sceneAnalyzer_;

	/* Predefined CCM profiles */
	std::vector<CCMProfile> ccm_profiles_;
	CCMProfile current_profile_;

	/* Algorithm state */
	MicrochipISCSensorInfo sensorInfo_;
	std::vector<CCMProfile> temporal_history_;
	std::vector<CCMProfile> convergence_history_;
	bool is_converged_;
	uint32_t frame_count_;
	float contextFluorescentStrength_;  /* Context from IPA */

	/* Core methods */
	void initializeProfessionalProfiles();
	CCMProfile calculateOptimalCCM(const UnifiedSceneAnalysis &scene);
	CCMProfile temporalSmoothing(const CCMProfile &new_profile);

	/* Profile selection and blending */
	CCMProfile createAdaptiveProfile(const UnifiedSceneAnalysis &scene);
	float calculateColorAccuracy(const CCMProfile &profile, uint32_t scene_cct);

	/* Validation and hardware interface */
	bool validateCCMMatrix(const CCMProfile &profile);
	float matrixDeterminant3x3(const std::array<float, 9> &matrix);
	void applyCCMToHardware(const CCMProfile &profile, ControlList &results);
	void applyIdentityMatrix(ControlList &results);
	int32_t clampHardwareValue(float value);

	/* Quality assessment */
	AccuracyMetrics evaluateCCMAccuracy(const CCMProfile &profile, const UnifiedSceneAnalysis &scene);

	/* Logging and diagnostics */
	void logSceneAnalysis(const UnifiedSceneAnalysis &scene);
	void logProfileApplication(const CCMProfile &profile, const AccuracyMetrics &metrics);
};

LOG_DECLARE_CATEGORY(ISC_CCM)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_CCM_H */
