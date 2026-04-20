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

#include <optional>

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
 *   Sensor → PFE → [histogram tap] → DPC(BLOFF=64) → WB(offsets=0) → ...
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
			LOG(ISC_BLC, Debug)
				<< "Sensor helper black level: "
				<< bl16 << " (16-bit) = "
				<< helperBl << " DN (10-bit)";
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

	LOG(ISC_BLC, Debug)
		<< "Black levels (10-bit): "
		<< "R=" << blR << " GR=" << blGR
		<< " GB=" << blGB << " B=" << blB
		<< " (scalar=" << bl << ")"
		<< (helperAvailable ? " [sensor helper]" : " [YAML/fallback]");

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
	const uint32_t bl = context.configuration.sensor.blackLevel;

	if (context.configuration.hw.hasDpc) {
		/*
		 * SAMA7G5: DPC hardware subtracts the black offset before
		 * the WB block. The kernel programs ISC_DPC_CFG.BLOFST from
		 * the sensor helper (or 64 for imx219). If our tuning YAML
		 * black_level differs, the IPA's histogram pedestal subtraction
		 * (blackLevel/2 bins in processStats) will use a different
		 * value than what the DPC block actually subtracts — corrupting
		 * AWB means and AGC luminance.
		 *
		 * We use the sensor helper value as the reference, not a
		 * hardcoded 64, so this check stays correct across sensors.
		 */
		uint32_t dpcOffset = 64; /* kernel default for imx219 */
		if (context.camHelper && context.camHelper->blackLevel())
			dpcOffset = static_cast<uint32_t>(
					    *context.camHelper->blackLevel()) >>
				    6;

		if (bl != dpcOffset) {
			LOG(ISC_BLC, Warning)
				<< "hasDpc=true: tuning black_level=" << bl
				<< " DN != kernel DPC offset=" << dpcOffset
				<< " DN. "
				   "Histogram pedestal subtraction will be wrong. "
				   "Set black_level="
				<< dpcOffset
				<< " in the tuning YAML (or remove black_level "
				   "to use the sensor helper value automatically).";
		}
	} else {
		/*
		 * SAMA5D2 / SAM9X7: no DPC hardware — pedestal subtracted
		 * via WB offset registers programmed by configure(). A zero
		 * black_level disables subtraction entirely.
		 */
		if (bl == 0) {
			LOG(ISC_BLC, Warning)
				<< "hasDpc=false: black_level=0 means no pedestal "
				   "subtraction via WB offsets. "
				   "Set black_level to the sensor pedestal (e.g. 64 "
				   "for IMX219) to avoid colour bias.";
		}
	}

	return 0;
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
