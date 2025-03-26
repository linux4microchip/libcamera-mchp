/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* Auto White Balance algorithm
	*/
#ifndef __LIBCAMERA_IPA_MICROCHIP_ISC_AWB_H__
#define __LIBCAMERA_IPA_MICROCHIP_ISC_AWB_H__

#include <array>
#include <cstdint>
#include <libcamera/controls.h>

#include "common.h"

namespace libcamera {
namespace ipa::microchip_isc {

LOG_DECLARE_CATEGORY(ISC_AWB)

class AWB {
public:
	/* White balance preset modes */
	enum WBMode {
	WB_AUTO = 0,
	WB_DAYLIGHT,
	WB_CLOUDY,
	WB_TUNGSTEN,
	WB_FLUORESCENT,
	WB_SHADE,
	WB_MANUAL,
	WB_MODE_MAX
	};

	AWB();
	void configure(const MicrochipISCSensorInfo &sensorInfo);
	void process(const ImageStats &stats, ControlList &results);

	/* Set white balance mode */
	void setMode(WBMode mode);
	/* Set manual white balance values (for WB_MANUAL mode) */
	void setManualWB(int32_t redGain, int32_t blueGain, int32_t greenRedGain, int32_t greenBlueGain);

private:
	void calculateWBGains(float avgR, float avgG, float avgB, uint32_t colorTemp, ControlList &results);
	uint32_t estimateCCT(float r, float g, float b);
	void applyPresetWB(ControlList &results, WBMode mode);
	void applyBlendedValues(ControlList &results, int32_t baseRedGain, int32_t baseBlueGain,
	int32_t baseGreenRedGain, int32_t baseGreenBlueGain,
	int32_t baseRedOffset, int32_t baseBlueOffset,
	int32_t baseGreenRedOffset, int32_t baseGreenBlueOffset);

	/* Control IDs for the AWB parameters */
	static constexpr uint32_t RED_GAIN_ID = 0x009819c0;
	static constexpr uint32_t BLUE_GAIN_ID = 0x009819c1;
	static constexpr uint32_t GREEN_RED_GAIN_ID = 0x009819c2;
	static constexpr uint32_t GREEN_BLUE_GAIN_ID = 0x009819c3;
	static constexpr uint32_t RED_OFFSET_ID = 0x009819c4;
	static constexpr uint32_t BLUE_OFFSET_ID = 0x009819c5;
	static constexpr uint32_t GREEN_RED_OFFSET_ID = 0x009819c6;
	static constexpr uint32_t GREEN_BLUE_OFFSET_ID = 0x009819c7;

	/* Gain constants based on driver definition */
	static constexpr int32_t kMinGain = 0;				 /* Min value from driver */
	static constexpr int32_t kMaxGain = 8191;			 /* Max value from driver */

	/* Offset constants */
	static constexpr int32_t kMinOffset = -8191;
	static constexpr int32_t kMaxOffset = 8191;

	/* Current hardware values (used as baseline) */
	static constexpr int32_t currentRedGain = 1944;
	static constexpr int32_t currentBlueGain = 3404;
	static constexpr int32_t currentGreenRedGain = 1103;
	static constexpr int32_t currentGreenBlueGain = 1619;
	static constexpr int32_t currentRedOffset = 7928;
	static constexpr int32_t currentBlueOffset = 7936;
	static constexpr int32_t currentGreenRedOffset = 7920;
	static constexpr int32_t currentGreenBlueOffset = 7920;

	/* Blend factors (70% hardware current, 30% algorithm value) */
	static constexpr float hwFactor = 0.7f;
	static constexpr float algoFactor = 0.3f;

	MicrochipISCSensorInfo sensorInfo_;
	WBMode currentMode_;

	/* Manual white balance values */
	int32_t manualRedGain_;
	int32_t manualBlueGain_;
	int32_t manualGreenRedGain_;
	int32_t manualGreenBlueGain_;
};

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_MICROCHIP_ISC_AWB_H__ */
