/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Black Level Correction for Microchip ISC
 *
 * Black level source priority (mirrors RkISP1 blc.cpp):
 *
 *  1. CameraSensorHelper::blackLevel() — compiled-in datasheet value.
 *     This is the most reliable source: a silicon property specified by
 *     the sensor manufacturer, constant across temperature and units.
 *     For IMX219: blackLevel_ = 4096 (16-bit) = 64 DN (10-bit).
 *
 *  2. YAML tuning file black_level key — present for backward compat.
 *     Overrides the sensor helper value when explicitly set. Emits a
 *     warning if the value differs from the sensor helper so developers
 *     notice accidental divergence. This path is deprecated: prefer
 *     relying on the sensor helper and removing black_level from YAML.
 *
 *  3. Hardcoded fallback 64 DN — used only when neither the sensor
 *     helper nor the YAML provides a value (new sensor bringup).
 *     This is safer than 0: a zero black level disables pedestal
 *     subtraction entirely and corrupts all AWB/AGC calculations.
 *
 * Bit-width note:
 *   CameraSensorHelper returns 16-bit values (range 0–65535) matching the
 *   V4L2 SensorBlackLevels control convention. The ISC IPA operates in
 *   10-bit (0–1023). Convert: bl_10bit = bl_16bit >> 6.
 *   IMX219: 4096 >> 6 = 64. IMX477: 16384 >> 6 = 256.
 */

#include "blc.h"

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_BLC)

namespace ipa::microchip_isc {

/*
 * ISC DPC block black offset register (SAMA7G5 §43.6.4 — ISC_DPC_CFG.BLOFF).
 *
 * The kernel driver (microchip-sama7g5-isc.c) hardcodes 64 DN (10-bit) as
 * the DPC black offset, scaled to the actual sensor bus width at stream-on:
 *   8-bit:  bloff = 16   (64 >> 2)
 *   10-bit: bloff = 64
 *   12-bit: bloff = 255  (min(64 << 2, 255))
 *
 * This static 64 DN is baked into the kernel driver. The IPA cannot change
 * it via any V4L2 control — there is no V4L2_CID_ISC_DPC_BLACK_OFFSET.
 * A future improvement would add such a control and have the kernel read it
 * from the sensor helper (camHelper->blackLevel()), matching RkISP1's BLC.
 *
 * BLC signal flow on SAMA7G5:
 *   Sensor → PFE → DPC(BLOFF=64) → [histogram tap] → WB(offsets=0) → ...
 *                                                          ↑
 *                                              IPA sets WB offsets to 0
 *                                              because DPC already handles BL
 *
 * The IPA's role here:
 *   1. Layer 2: WB offset registers — set to 0 (SAMA7G5, hasDpc=true)
 *                                     set to -blackLevel (SAMA5D2, hasDpc=false)
 *   2. Layer 3: Software histogram pedestal subtraction in processStats()
 *      → subtracts (blackLevel/2) bins from channelMeans so AWB/AGC see
 *        "net" scene signal without the pedestal. This is CRITICAL — without
 *        it, AWB computes rGain = meanGR/meanR where both means include the
 *        pedestal, inflating the ratio for channels where pedestal differs.
 *   3. Validation: warns if YAML black_level ≠ DPC hardware value so
 *      developers notice when the kernel and IPA disagree.
 */

/**
 * \brief Initialise black level from sensor helper + YAML
 *
 * Priority order mirrors RkISP1 (camera_sensor_helper.cpp):
 *   1. camHelper_->blackLevel()  — sensor datasheet, compiled-in
 *   2. YAML black_level          — explicit override (deprecated, warns)
 *   3. 64 DN hardcoded           — bringup fallback
 */
int BlackLevelCorrection::init(IPAContext &context,
			       const YamlObject &tuningData)
{
	/*
	 * Tier 1: sensor helper — most reliable source.
	 * CameraSensorHelper::blackLevel() returns 16-bit values.
	 * Convert to 10-bit by right-shifting 6 bits (÷ 64).
	 * IMX219: 4096 >> 6 = 64 DN. IMX477: 16384 >> 6 = 256 DN.
	 */
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
	 * Tier 2: YAML override — deprecated but kept for backward compat.
	 * When the YAML explicitly sets black_level, use it but warn if it
	 * differs from the sensor helper value. This matches RkISP1's
	 * "Deprecated: black levels overwritten by tuning file" warning.
	 *
	 * Use helperBl as the .get<>() default so an absent key falls
	 * through to the sensor helper value (tier 1), not to zero or 64.
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
 * \brief Validate BLC against DPC hardware offset on each configure()
 *
 * On SAMA7G5 (hasDpc=true): the kernel ISC driver programs
 * ISC_DPC_CFG.BLOFST from the sensor's V4L2_CID_ANALOGUE_GAIN_OFFSET
 * (or defaults to 64 for imx219). We now compare against the sensor
 * helper value rather than a hardcoded 64 — so this check remains
 * correct when a different sensor with a different pedestal is used.
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
