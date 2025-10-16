/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Advanced Auto Gain Control with Unified Scene Analysis
 */
#ifndef IPA_MICROCHIP_ISC_AGC_H
#define IPA_MICROCHIP_ISC_AGC_H

#include <libcamera/base/log.h>
#include <libcamera/controls.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>
#include "common.h"
#include <memory>

namespace libcamera {
class ControlList;

namespace ipa::microchip_isc {

class AGC {
public:
	AGC();
	void configure(const MicrochipISCSensorInfo &sensorInfo);
	void process(const ImageStats &stats, ControlList &results);

	/* Context and state management */
	void setAWBContext(const AWBContext &context);
	void resetConvergence();
	/* Scene analysis configuration */
	void setSceneAnalysisConfig(const SceneAnalysisConfig &config) { sceneAnalyzer_->setConfig(config); }
	ExposureResult getLastResult() const { return lastResult_; }
	ConvergenceState getConvergenceState() const { return convergenceState_; }
	bool isConverged() const { return convergenceState_ == ConvergenceState::CONVERGED; }

private:
	/* Core algorithm configuration */
	struct AGCConfig {
		uint32_t minExposureTime = 100;      /* 0.1ms */
		uint32_t maxExposureTime = 33000;    /* 33ms */
		uint32_t minAnalogueGain = 256;      /* 1.0x */
		uint32_t maxAnalogueGain = 8192;     /* 32x */
		uint32_t minDigitalGain = 256;       /* 1.0x */
		uint32_t maxDigitalGain = 1024;      /* 4.0x */
		float targetLuminance = 128.0f;
	};

	/* Scene analyzer for intelligent exposure decisions */
	std::unique_ptr<UnifiedSceneAnalyzer> sceneAnalyzer_;

	/* Algorithm state */
	AGCConfig config_;
	MicrochipISCSensorInfo sensorInfo_;
	std::string sensorModel_;
	ConvergenceState convergenceState_;
	UnifiedSceneAnalysis currentSceneAnalysis_;
	AWBContext awbContext_;

	/* Processing state */
	uint32_t frameCount_;
	uint32_t convergenceFrames_;
	uint32_t stableFrameCount_;
	ExposureResult lastResult_;
	ExposureResult resultHistory_[8];
	bool hasValidHistory_;

	/* Core processing methods */
	ExposureResult calculateExposure(const ImageStats &stats, const UnifiedSceneAnalysis &scene);
	void updateConvergenceState(const ExposureResult &result);
	void applyTemporalSmoothing(ExposureResult &result);

	/* Exposure strategy methods */
	void applyBacklightCompensation(ExposureResult &result, const UnifiedSceneAnalysis &scene);
	void applyLowLightOptimization(ExposureResult &result, const UnifiedSceneAnalysis &scene);
	void applyHDRStrategy(ExposureResult &result, const UnifiedSceneAnalysis &scene);
	void applyUniformStrategy(ExposureResult &result, const UnifiedSceneAnalysis &scene);

	/* Validation and protection */
	void clampExposureParameters(ExposureResult &result);
	bool validateExposureResult(const ExposureResult &result);
	void applyOverexposureProtection(ExposureResult &result, const ImageStats &stats);

	/* Sensor-specific outdoor exposure calibration with interpolation */
	struct ExposureCurve {
		std::vector<std::pair<float, uint32_t>> points;  /* brightness -> exposure */
	};
	std::map<std::string, ExposureCurve> sensorExposureCurves_;

	void initializeSensorCurves();
	uint32_t interpolateExposure(float brightness) const;
	uint32_t calculateAdaptiveExposure(float brightness) const;

};

LOG_DECLARE_CATEGORY(ISC_AGC)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_AGC_H */

