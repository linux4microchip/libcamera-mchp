/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Advanced Auto White Balance with Unified Scene Analysis
 */
#ifndef IPA_MICROCHIP_ISC_AWB_H
#define IPA_MICROCHIP_ISC_AWB_H

#include <libcamera/base/log.h>
#include <libcamera/controls.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>
#include "common.h"
#include <memory>
#include <vector>

namespace libcamera {
class ControlList;

namespace ipa::microchip_isc {

/* Base class for AWB algorithms */
class WhiteBalanceAlgorithm {
	public:
		virtual ~WhiteBalanceAlgorithm() = default;
		virtual WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) = 0;
		virtual bool isApplicable(const UnifiedSceneAnalysis &scene) = 0;
		virtual std::string getName() const = 0;
};

/* Specific algorithm implementations */
class UnifiedGreyWorldWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "UnifiedGreyWorld"; }
};

class IncandescentWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "Incandescent"; }
};

class FluorescentWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "Fluorescent"; }
};

class LEDWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "LED"; }
};

class DaylightWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "Daylight"; }
};

class NeutralRegionWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "NeutralRegion"; }
};

class MultiIlluminantWhiteBalance : public WhiteBalanceAlgorithm {
	public:
		WhiteBalanceResult process(const ImageStats &stats, const UnifiedSceneAnalysis &scene) override;
		bool isApplicable(const UnifiedSceneAnalysis &scene) override;
		std::string getName() const override { return "MultiIlluminant"; }
};

/* Main AWB class */
class AWB {
	public:
		AWB();
		void configure(const MicrochipISCSensorInfo &sensorInfo);
		void process(const ImageStats &stats, ControlList &results);

		/* Context and state management */
		void setAWBContext(const AWBContext &context);
		WhiteBalanceResult getLastResult() const { return lastResult_; }
		UnifiedSceneAnalysis getLastSceneAnalysis() const { return lastSceneAnalysis_; }
		ConvergenceState getConvergenceState() const { return convergenceState_; }
		bool isConverged() const { return convergenceState_ == ConvergenceState::CONVERGED; }
		bool isFluorescentIllumination() const { return lastSceneAnalysis_.lightSource == LightSourceType::FLUORESCENT; }

	private:
		/* Algorithm management */
		std::vector<std::unique_ptr<WhiteBalanceAlgorithm>> algorithms_;
		std::unique_ptr<UnifiedSceneAnalyzer> sceneAnalyzer_;

		/* Processing state */
		MicrochipISCSensorInfo sensorInfo_;
		ConvergenceState convergenceState_;
		AWBContext awbContext_;
		UnifiedSceneAnalysis lastSceneAnalysis_;
		WhiteBalanceResult lastResult_;
		uint32_t frameCount_;
		uint32_t stableFrameCount_;

		/* Algorithm selection and processing */
		WhiteBalanceAlgorithm* selectBestAlgorithm(const UnifiedSceneAnalysis &scene);
		void applyTemporalSmoothing(WhiteBalanceResult &result);
		void updateConvergenceState(const WhiteBalanceResult &result);
		void applyResultToHardware(const WhiteBalanceResult &result, ControlList &results);

		/* Validation */
		bool validateResult(const WhiteBalanceResult &result);
		void clampGainsAndOffsets(WhiteBalanceResult &result);
};

LOG_DECLARE_CATEGORY(ISC_AWB)

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* IPA_MICROCHIP_ISC_AWB_H */
