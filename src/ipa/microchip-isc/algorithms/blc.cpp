/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Microchip Technology Inc.
 *
 * Black Level Correction for Microchip ISC
 *
 * Black level source priority (mirrors RkISP1 blc.cpp): sensor helper,
 * then YAML black_level (deprecated override, warns on divergence), then
 * a 64 DN fallback for sensor bringup. Helper values are 16-bit; the IPA
 * works in 10-bit, so convert bl_10bit = bl_16bit >> 6 (IMX219: 4096 → 64).
 */

#include "blc.h"

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_BLC)

namespace ipa::microchip_isc {

/*
 * On SAMA7G5 the kernel driver programs a fixed 64 DN pedestal in the DPC
 * block (ISC_DPC_CFG.BLOFF, §43.6.4), scaled to the sensor bus width at
 * stream-on. There is no V4L2 control to change it, so on hasDpc platforms
 * the IPA leaves the WB offsets neutral and lets DPC own the pedestal; AWB
 * re-anchors per channel each frame. On hasDpc=false the WB offsets carry
 * the -blackLevel subtraction instead.
 */

/**
 * \brief Initialise black level from sensor helper + YAML
 */
int BlackLevelCorrection::init(IPAContext &context,
			       const YamlObject &tuningData)
{
	/* Sensor helper returns 16-bit; convert to 10-bit (>> 6). */
	uint32_t helperBl = 64; /* safe fallback if no helper registered */
	bool helperAvailable = false;

	if (context.camHelper && context.camHelper->blackLevel()) {
		int16_t bl16 = *context.camHelper->blackLevel();
		if (bl16 > 0) {
			helperBl = static_cast<uint32_t>(bl16) >> 6;
			helperAvailable = true;
		}
	}

	if (!helperAvailable) {
		LOG(ISC_BLC, Warning)
			<< "No CameraSensorHelper registered for this sensor "
			   "(or no blackLevel provided). "
			   "Falling back to YAML / hardcoded 64 DN. "
			   "Add the sensor to camera_sensor_helper.cpp for "
			   "automatic black level lookup.";
	}

	/*
	 * YAML override: deprecated, warns on divergence. Use helperBl as the
	 * .get<>() default so an absent key falls through to the helper value.
	 */
	uint32_t yamlBl = tuningData["black_level"].get<uint32_t>(helperBl);
	bool yamlExplicit = tuningData["black_level"].get<uint32_t>(0xFFFFFFFF) != 0xFFFFFFFF;

	uint32_t bl;
	if (yamlExplicit && yamlBl != helperBl && helperAvailable) {
		LOG(ISC_BLC, Warning)
			<< "YAML black_level=" << yamlBl
			<< " DN overrides sensor helper value "
			<< helperBl << " DN. "
			<< "Remove black_level from the tuning YAML to use "
			   "the sensor helper value automatically. "
			   "Mismatch causes wrong pedestal subtraction in "
			   "AWB and AGC histogram calculations.";
		bl = yamlBl; /* honour explicit YAML override */
	} else {
		bl = helperAvailable ? helperBl : yamlBl;
	}

	/* Per-channel values fall back to bl when absent */
	uint32_t blR = tuningData["black_level_r"].get<uint32_t>(bl);
	uint32_t blGR = tuningData["black_level_gr"].get<uint32_t>(bl);
	uint32_t blGB = tuningData["black_level_gb"].get<uint32_t>(bl);
	uint32_t blB = tuningData["black_level_b"].get<uint32_t>(bl);

	context.configuration.sensor.blackLevel = bl;
	context.configuration.sensor.blackLevelR = blR;
	context.configuration.sensor.blackLevelGR = blGR;
	context.configuration.sensor.blackLevelGB = blGB;
	context.configuration.sensor.blackLevelB = blB;

	return 0;
}

/**
 * \brief Set the per-frame WB offset defaults for the active platform
 */
int BlackLevelCorrection::configure(IPAContext &context,
				    [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	/*
	 * Pedestal subtraction.
	 *
	 * On platforms with a DPC block (hasDpc=true): the hardware removes a
	 * uniform pedestal before the WB stage, so start neutral. AWB then
	 * sets the per-channel floor offset per frame from the histogram.
	 *
	 * On platforms without hardware DPC (hasDpc=false): the pedestal is
	 * removed via the WB offset registers set below.
	 */
	if (context.configuration.hw.hasDpc) {
		/* neutral default — AWB owns the per-channel offset per frame */
		context.activeState.awb.rOffset = 0;
		context.activeState.awb.bOffset = 0;
		context.activeState.awb.grOffset = 0;
		context.activeState.awb.gbOffset = 0;
	} else {
		/*
		 * No hardware DPC — subtract the pedestal through the WB offset
		 * block.  Use the per-channel black levels so a sensor with an
		 * asymmetric pedestal (different R/GR/GB/B offsets) is corrected
		 * exactly; they default to the scalar bl when the YAML omits the
		 * per-channel keys, so IMX219 (all 64) is unaffected.
		 */
		const auto &sensor = context.configuration.sensor;
		context.activeState.awb.rOffset =
			-static_cast<int32_t>(sensor.blackLevelR);
		context.activeState.awb.bOffset =
			-static_cast<int32_t>(sensor.blackLevelB);
		context.activeState.awb.grOffset =
			-static_cast<int32_t>(sensor.blackLevelGR);
		context.activeState.awb.gbOffset =
			-static_cast<int32_t>(sensor.blackLevelGB);
	}

	return 0;
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
