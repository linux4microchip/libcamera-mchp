/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Black Level Correction for Microchip ISC
 */

#include "blc.h"

#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_BLC)

namespace ipa::microchip_isc {

/**
 * \brief Read sensor black level from the tuning file
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for BlackLevelCorrection
 * \return 0 on success
 *
 * Reads the sensor black level pedestal from the BlackLevelCorrection tuning
 * block and stores it in IPASessionConfiguration::sensor. The IPA configure()
 * method uses these values to set WB block offsets for platforms that lack
 * hardware DPC (hasDpc=false). On SAMA7G5 (hasDpc=true) DPC hardware
 * subtracts the pedestal before the WB block, so configure() zeroes offsets.
 *
 * All per-channel values fall back to the global black_level when absent.
 */
int BlackLevelCorrection::init(IPAContext &context,
			       const YamlObject &tuningData)
{
	uint32_t bl = tuningData["black_level"].get<uint32_t>(0);
	context.configuration.sensor.blackLevel = bl;
	context.configuration.sensor.blackLevelR = tuningData["black_level_r"].get<uint32_t>(bl);
	context.configuration.sensor.blackLevelGR = tuningData["black_level_gr"].get<uint32_t>(bl);
	context.configuration.sensor.blackLevelGB = tuningData["black_level_gb"].get<uint32_t>(bl);
	context.configuration.sensor.blackLevelB = tuningData["black_level_b"].get<uint32_t>(bl);

	LOG(ISC_BLC, Debug)
		<< "Black levels: R=" << context.configuration.sensor.blackLevelR
		<< " GR=" << context.configuration.sensor.blackLevelGR
		<< " GB=" << context.configuration.sensor.blackLevelGB
		<< " B=" << context.configuration.sensor.blackLevelB;

	return 0;
}

/*
 * On SAMA7G5 (hasDpc=true) the ISC DPC block subtracts a hardware black offset
 * before the WB block. The kernel driver hardcodes this to 64
 * (ISC_DPC_CFG_BLOFF = 64, ISC_DPC_CFG register, ISC block datasheet §11).
 * The tuning file black_level must match this constant; a mismatch means the
 * gain-undo arithmetic in AWB and AGC will use the wrong pedestal.
 * On SAMA5D2 (hasDpc=false) the DPC block is absent; the WB block R/G/B offset
 * registers (ISC_WB_O_RGR, ISC_WB_O_BGR) are programmed with the pedestal from
 * this tuning parameter (ISC block datasheet §8.1).
 */
static constexpr uint32_t kDpcHardcodedBlackOffset = 64;

/**
 * \brief Validate DPC/BLC configuration on each sensor mode change
 * \param[in] context The IPA context
 * \param[in] sensorInfo Camera sensor information (unused)
 * \return 0 on success
 *
 * On SAMA7G5 (hasDpc=true): warns if the tuning file black_level does not
 * match the kernel-hardcoded DPC offset (64). A mismatch means AWB gain-undo
 * and histogram normalisation use the wrong pedestal.
 *
 * On SAMA5D2 (hasDpc=false): warns if black_level is zero, which means no
 * pedestal is subtracted via WB offsets, leaving a colour bias that AGC and
 * AWB cannot fully compensate.
 */
int BlackLevelCorrection::configure(IPAContext &context,
				    [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	if (context.configuration.hw.hasDpc) {
		/*
		 * SAMA7G5: DPC block hardcodes the black offset to 64.
		 * Warn if the tuning file specifies a different value so that
		 * developers notice the mismatch before it silently corrupts
		 * AWB gain-undo calculations.
		 */
		if (context.configuration.sensor.blackLevel != kDpcHardcodedBlackOffset) {
			LOG(ISC_BLC, Warning)
				<< "hasDpc=true: tuning black_level="
				<< context.configuration.sensor.blackLevel
				<< " does not match the kernel ISC DPC hardware offset ("
				<< kDpcHardcodedBlackOffset
				<< "). Update black_level in the tuning YAML to "
				<< kDpcHardcodedBlackOffset
				<< " to match ISC_DPC_CFG (datasheet §11).";
		}
	} else {
		/*
		 * SAMA5D2 / SAM9X7: no DPC hardware; pedestal subtracted via
		 * WB offset registers. A zero black_level skips subtraction.
		 */
		if (context.configuration.sensor.blackLevel == 0) {
			LOG(ISC_BLC, Warning)
				<< "hasDpc=false but black_level=0 in tuning file: "
				   "no black level will be subtracted via WB offsets. "
				   "Set black_level to the sensor pedestal (e.g. 64) "
				   "to avoid colour bias (ISC WB block, datasheet §8.1).";
		}
	}
	return 0;
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
