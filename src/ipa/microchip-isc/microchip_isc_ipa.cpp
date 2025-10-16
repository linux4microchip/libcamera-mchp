/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * IPA module for Microchip ISC with Unified Scene Classification System
 *
 * UNIFIED APPROACH OVERVIEW:
 * - Single scene analyzer feeds consistent scene understanding to all algorithms
 * - AWB uses scene analysis to select fluorescent/daylight/incandescent algorithms
 * - AGC uses scene complexity for backlight/HDR/low-light strategies
 * - CCM uses light source detection for anti-green/blue-boost matrices
 * - Eliminates conflicts between individual algorithm scene detection systems
 */

#include <linux/media-bus-format.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>

/* Unified algorithm implementations */
#include "algorithms/common.h"
#include "algorithms/awb.h"
#include "algorithms/agc.h"
#include "algorithms/ccm.h"
#include <iomanip>

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_IPA)

namespace ipa::microchip_isc {

class IPAMicrochipISC : public IPAMicrochipISCInterface {
public:
	IPAMicrochipISC()
	: awb_(std::make_unique<AWB>()),
	agc_(std::make_unique<AGC>()),
	ccm_(std::make_unique<CCM>()),
	frameCount_(0)
	{
		LOG(ISC_IPA, Info) << "Microchip ISC IPA initialized with unified scene classification";
		LOG(ISC_IPA, Info) << "Architecture: One scene analyzer -> All three algorithms";
	}

	int32_t init([[maybe_unused]] const IPASettings &settings, bool *metadataSupport) override
	{
		LOG(ISC_IPA, Debug) << "Initializing unified scene classification IPA";
		*metadataSupport = true;
		frameCount_ = 0;
		return 0;
	}

	int32_t start() override
	{
		LOG(ISC_IPA, Debug) << "Starting unified scene classification processing";
		frameCount_ = 0;
		return 0;
	}

	void stop() override
	{
		LOG(ISC_IPA, Debug) << "Stopping unified scene classification processing";
	}

	int32_t configure(
		const MicrochipISCSensorInfo &sensorInfo,
		[[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
		[[maybe_unused]] const std::map<unsigned int, ControlInfoMap> &entityControls
	) override
	{
		/*
		 * Validate sensor information from pipeline handler.
		 * The pipeline handler is responsible for querying sensor capabilities
		 * from V4L2 controls or using the sensor database fallback.
		 */
		LOG(ISC_IPA, Info) << "Configuring IPA for sensor: " << sensorInfo.model
			<< " (" << sensorInfo.width << "x" << sensorInfo.height << ")";
		LOG(ISC_IPA, Info) << "Exposure range: " << sensorInfo.minExposure
			<< "-" << sensorInfo.maxExposure << "μs";
		LOG(ISC_IPA, Info) << "Analog gain range: " << sensorInfo.minAnalogGain
			<< "-" << sensorInfo.maxAnalogGain;
		LOG(ISC_IPA, Info) << "Digital gain range: " << sensorInfo.minDigitalGain
			<< "-" << sensorInfo.maxDigitalGain;

		/* Validate limits before passing to algorithms */
		if (sensorInfo.minExposure <= 0 || sensorInfo.maxExposure <= 0 ||
		    sensorInfo.minExposure >= sensorInfo.maxExposure ||
		    sensorInfo.minAnalogGain >= sensorInfo.maxAnalogGain ||
		    sensorInfo.minDigitalGain >= sensorInfo.maxDigitalGain) {
			LOG(ISC_IPA, Error) << "Invalid sensor capabilities from pipeline handler";
			LOG(ISC_IPA, Error) << "Pipeline must provide valid sensor limits";
			return -1;
		}

		/* Store image information */
		imageInfo_.width = sensorInfo.width;
		imageInfo_.height = sensorInfo.height;
		imageInfo_.format = sensorInfo.pixelFormat;

		/* Configure algorithms with validated limits */
		awb_->configure(sensorInfo);
		agc_->configure(sensorInfo);
		ccm_->configure(sensorInfo);

		return 0;
	}

	void processStats(const ControlList &stats) override
	{
		frameCount_++;

		/* Read algorithm enable flags from application */
		if (stats.contains(IPA_ALGORITHM_ENABLE_ID)) {
			algorithmEnableFlags_ = stats.get(IPA_ALGORITHM_ENABLE_ID).get<int32_t>();
			LOG(ISC_IPA, Debug) << "Algorithm flags: 0x" << std::hex << algorithmEnableFlags_;
		}

		LOG(ISC_IPA, Debug) << "=== UNIFIED SCENE PROCESSING (Frame " << frameCount_ << ") ===";

		/* Debug available controls periodically to avoid log spam */
		if (frameCount_ % 30 == 1) {
			LOG(ISC_IPA, Debug) << "Stats contains " << stats.size() << " controls:";
			for (const auto &[id, value] : stats) {
				LOG(ISC_IPA, Debug) << "Control ID: 0x" << std::hex << id;
			}
		}

		/* Check for scene analysis mode control and pass to algorithms */
		if (stats.contains(ISC_SCENE_ANALYSIS_MODE_ID)) {
			int32_t mode = stats.get(ISC_SCENE_ANALYSIS_MODE_ID).get<int32_t>();

			/* Update scene analyzer configuration */
			SceneAnalysisConfig sceneConfig;
			if (mode == 1) {  /* STILL_MODE */
				sceneConfig.enableHysteresis = false;
				sceneConfig.minFramesForStateChange = 1;
				LOG(ISC_IPA, Info) << "Still capture mode - immediate scene classification";
			} else {  /* VIDEO_MODE */
				sceneConfig.enableHysteresis = true;
				sceneConfig.minFramesForStateChange = 2;
			}

			if (awb_) {
				awb_->setSceneAnalysisConfig(sceneConfig);
			}

			if (agc_) {
				agc_->setSceneAnalysisConfig(sceneConfig);
			}

			if (ccm_) {
				ccm_->setSceneAnalysisConfig(sceneConfig);
			}

		}

		/*
		 * UNIFIED STATISTICS GENERATION:
		 * Generate ImageStats for consistent scene analysis across all algorithms.
		 * Priority: Hardware histogram > Software pixel processing > Fallback
		 */
		ImageStats imageStats;
		bool statsGenerated = false;
		std::string statsSource = "NONE";

		/* PRIORITY 1: Hardware histogram data (preferred for accuracy and performance) */
		if (stats.contains(ISC_HISTOGRAM_DATA_ID)) {
			LOG(ISC_IPA, Debug) << "Using hardware histogram data for unified scene analysis";

			try {
				/* Convert hardware histogram to unified ImageStats format */
				convertHardwareStatsToImageStats(stats, imageStats);

				if (validateImageStats(imageStats)) {
					statsGenerated = true;
					statsSource = "HARDWARE";
					LOG(ISC_IPA, Info) << "Hardware stats: GR=" << std::fixed << std::setprecision(1)
						<< imageStats.meanGR << " R=" << imageStats.meanR
						<< " GB=" << imageStats.meanGB << " B=" << imageStats.meanB;
				} else {
					LOG(ISC_IPA, Warning) << "Hardware stats validation failed";
					/* Allow early frames to proceed with incomplete data for algorithm initialization */
					if (frameCount_ <= 3) {
						LOG(ISC_IPA, Info) << "Allowing early frame " << frameCount_
							<< " to proceed with incomplete hardware stats";
						statsGenerated = true;
						statsSource = "HARDWARE_PARTIAL";
					}
				}
			} catch (const std::exception &e) {
				LOG(ISC_IPA, Error) << "Hardware stats conversion failed: " << e.what();
			}
		}

		/* PRIORITY 2: Software processing fallback (when hardware stats unavailable) */
		if (!statsGenerated && stats.contains(ISC_PIXEL_VALUES_ID)) {
			LOG(ISC_IPA, Debug) << "Falling back to software pixel processing for scene analysis";

			try {
				const ControlValue &pixelData = stats.get(ISC_PIXEL_VALUES_ID);
				const auto values = pixelData.get<Span<const uint8_t>>();

				LOG(ISC_IPA, Debug) << "Processing format 0x" << std::hex << imageInfo_.format
					<< " (" << std::dec << values.size() << " bytes)";

				/* Generate statistics from raw pixel data using format-specific processing */
				generateStatsFromFormat(values.data(), imageInfo_, imageStats);

				if (validateImageStats(imageStats)) {
					statsGenerated = true;
					statsSource = "SOFTWARE";
					LOG(ISC_IPA, Debug) << "Software stats generated successfully";
				} else {
					LOG(ISC_IPA, Warning) << "Software stats validation failed";
				}
			} catch (const std::exception &e) {
				LOG(ISC_IPA, Error) << "Software stats generation failed: " << e.what();
			}
		}

		/* FALLBACK: No valid data sources available */
		if (!statsGenerated) {
			if (!stats.contains(ISC_PIXEL_VALUES_ID) && !stats.contains(ISC_HISTOGRAM_DATA_ID)) {
				LOG(ISC_IPA, Error) << "No pixel data or histogram data available - cannot proceed";
			} else {
				LOG(ISC_IPA, Error) << "Failed to generate valid statistics from available data";
			}
			return;  /* Cannot proceed without valid statistics */
		}

		/* Debug unified statistics periodically */
		if (frameCount_ % 60 == 1) {
			dumpImageStats(imageStats, "UNIFIED_" + statsSource);
		}

		/*
		 * UNIFIED SCENE ANALYSIS PROCESSING:
		 * All three algorithms receive the same ImageStats for consistent scene interpretation.
		 * This eliminates conflicts between individual algorithm scene detection systems.
		 */
		ControlList results(controls::controls);

		try {
			/*
			 * AWB PROCESSING with unified scene analysis:
			 * - Uses scene analysis to select fluorescent/daylight/incandescent algorithms
			 * - Detects green cast from fluorescent lighting
			 * - Applies anti-green-cast or daylight-optimized algorithms based on scene
			 */
			if ((algorithmEnableFlags_ & IPA_ALGORITHM_AWB) && awb_) {
				LOG(ISC_IPA, Debug) << "Processing AWB with unified scene analysis";
				awb_->process(imageStats, results);
				LOG(ISC_IPA, Debug) << "AWB convergence: "
					<< (awb_->isConverged() ? "CONVERGED" : "CONVERGING");
			}

			/*
			 * AGC PROCESSING with unified scene analysis:
			 * - Uses scene complexity for backlight/HDR/low-light strategies
			 * - Applies exposure limitation + gain compensation for backlight scenes
			 * - Maximizes exposure time and optimizes noise vs SNR for low light
			 * - Uses conservative exposure for HDR scenes to preserve highlights/shadows
			 * - Prefers exposure over gain for outdoor scenes for better image quality
			 */
			if ((algorithmEnableFlags_ & IPA_ALGORITHM_AGC) && agc_) {
				LOG(ISC_IPA, Debug) << "Processing AGC with unified scene analysis";
				agc_->process(imageStats, results);
				LOG(ISC_IPA, Debug) << "AGC convergence: "
					<< (agc_->isConverged() ? "CONVERGED" : "CONVERGING");
			}

			/*
			 * CCM PROCESSING with unified scene analysis:
			 * - Uses light source detection for scene-specific color correction
			 * - Applies aggressive anti-green matrix profiles for fluorescent scenes
			 * - Applies blue-boost matrices for tungsten warmth correction
			 * - Applies vivid enhancement matrices for daylight scenes
			 * - Uses scene color temperature to select appropriate correction strength
			 */
			if ((algorithmEnableFlags_ & IPA_ALGORITHM_CCM) && ccm_) {
				LOG(ISC_IPA, Debug) << "Processing CCM with unified scene analysis";
				ccm_->process(imageStats, results);
				LOG(ISC_IPA, Debug) << "CCM processing complete";
			}

			/*
			 * RESULTS PROCESSING:
			 * Log and emit the unified control updates to hardware.
			 * All algorithms contribute to a single, consistent control list.
			 */
			if (!results.empty()) {
				LOG(ISC_IPA, Info) << "Unified processing complete (" << statsSource << ") - sending "
					<< results.size() << " control updates";

				/* Debug control updates periodically */
				if (frameCount_ % 60 == 1) {
					LOG(ISC_IPA, Debug) << "Control updates:";
					for (const auto &[id, value] : results) {
						LOG(ISC_IPA, Debug) << "  0x" << std::hex << id
							<< " = " << std::dec << value.toString();
					}
				}
			} else {
				LOG(ISC_IPA, Warning) << "No control updates generated from unified processing";
			}

			/* Emit unified processing results to pipeline handler */
			awbComplete.emit(0, results);

		} catch (const std::exception &e) {
			LOG(ISC_IPA, Error) << "Unified algorithm processing failed: " << e.what();
			/* Continue execution - don't crash the pipeline */
		}

		/* Periodic status logging */
		if (frameCount_ % 30 == 0) {
			LOG(ISC_IPA, Info) << "Unified scene classification frame " << frameCount_
				<< " (" << statsSource << ") processing complete";
		}
	}

private:
	/*
	 * UNIFIED ALGORITHM INSTANCES:
	 * Each algorithm uses unified scene analysis for consistent scene interpretation.
	 * - AWB: Scene-specific algorithm selection
	 * - AGC: Scene-complexity-based exposure strategies
	 * - CCM: Light-source-specific color correction matrices
	 */
	std::unique_ptr<AWB> awb_;
	std::unique_ptr<AGC> agc_;
	std::unique_ptr<CCM> ccm_;
	uint32_t algorithmEnableFlags_ = IPA_ALGORITHM_ALL;

	/*
	 * COMMON STATE:
	 * Shared information used across all algorithms for unified scene analysis.
	 */
	ImageInfo imageInfo_;      /* Common image format information */
	unsigned int frameCount_;  /* Frame counter for logging and algorithm state */
};

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

/*
 * External IPA module interface:
 * Standard libcamera IPA module registration and creation functions.
 */
extern "C" {
	extern const struct libcamera::IPAModuleInfo ipaModuleInfo;
	extern libcamera::IPAInterface *ipaCreate();
}

const struct libcamera::IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"microchip-isc",
	"microchip-isc-unified",  /* Identifies this as unified scene classification version */
};

libcamera::IPAInterface *ipaCreate()
{
	return new libcamera::ipa::microchip_isc::IPAMicrochipISC();
}
