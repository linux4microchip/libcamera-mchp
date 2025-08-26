/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
	* Copyright (C) 2024 Microchip Technology Inc.	All rights reserved.
	*
	* IPA module for Microchip ISC
	*/
#include <linux/media-bus-format.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/microchip_isc_ipa_interface.h>

#include "algorithms/common.h"
#include "algorithms/awb.h"
#include "algorithms/agc.h"
#include "algorithms/ccm.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_IPA)

namespace ipa::microchip_isc {

class IPAMicrochipISC : public IPAMicrochipISCInterface {
public:
	IPAMicrochipISC()
	: awb_(std::make_unique<AWB>()),
	agc_(std::make_unique<AGC>()),
	ccm_(std::make_unique<CCM>())
	{
	}

	int32_t init([[maybe_unused]] const IPASettings &settings, bool *metadataSupport) override
	{
	LOG(ISC_IPA, Debug) << "Initializing Microchip ISC IPA";
	*metadataSupport = true;
	return 0;
	}

	int32_t start() override
	{
	LOG(ISC_IPA, Debug) << "Starting Microchip ISC IPA";
	return 0;
	}

	void stop() override
	{
	LOG(ISC_IPA, Debug) << "Stopping Microchip ISC IPA";
	}

	int32_t configure(
	const MicrochipISCSensorInfo &sensorInfo,
	[[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
	[[maybe_unused]] const std::map<unsigned int, ControlInfoMap> &entityControls
	) override
	{
	LOG(ISC_IPA, Debug) << "Configuring IPA for sensor " << sensorInfo.model;

	/* Store image information */
	imageInfo_.width = sensorInfo.width;
	imageInfo_.height = sensorInfo.height;
	imageInfo_.format = sensorInfo.pixelFormat;

	/* Configure all algorithms */
	awb_->configure(sensorInfo);
	agc_->configure(sensorInfo);
	ccm_->configure(sensorInfo);

	return 0;
	}
	void processStats(const ControlList &stats) override
	{
	LOG(ISC_IPA, Debug) << "IPAMicrochipISC processing stats";
	/* Debug what's available in the stats ControlList */
	LOG(ISC_IPA, Debug) << "Stats contains " << stats.size() << " controls:";
	for (const auto &[id, value] : stats) {
	LOG(ISC_IPA, Debug) << "Control ID: 0x" << std::hex << id;
	}

	/* Check for required pixel data */
	if (!stats.contains(ISC_PIXEL_VALUES_ID)) {
	LOG(ISC_IPA, Error) << "Missing pixel values in stats";
	return;
	}

	/* Extract pixel data */
	const ControlValue &pixelData = stats.get(ISC_PIXEL_VALUES_ID);
	const auto values = pixelData.get<Span<const uint8_t>>();

	/* Process common statistics once for all algorithms */
	ImageStats imageStats;

	/* Detect format from the configured image info and generate statistics */
	if (imageInfo_.format == MEDIA_BUS_FMT_YUYV8_2X8 ||
	imageInfo_.format == MEDIA_BUS_FMT_UYVY8_2X8) {
	/* YUV format */
	LOG(ISC_IPA, Debug) << "Processing YUYV format";
	generateStatsFromYUYV(values.data(), imageInfo_, imageStats);
	} else if (imageInfo_.format == MEDIA_BUS_FMT_RGB565_2X8_LE) {
	/* RGB565 format */
	LOG(ISC_IPA, Debug) << "Processing RGB565 format";
	generateStatsFromRGB565(values.data(), imageInfo_, imageStats);
	} else if (imageInfo_.format == MEDIA_BUS_FMT_RGB888_1X24 ||
	imageInfo_.format == MEDIA_BUS_FMT_BGR888_1X24 ||
	imageInfo_.format == 0x1018) {	/* RGBP8_1X24 might have this code in some systems */
	/* RGBP format - handle 24-bit RGB packed */
	LOG(ISC_IPA, Debug) << "Processing RGBP/RGB888 format";
	generateStatsFromRGBP(values.data(), imageInfo_, imageStats);
	} else {
	/* Unknown format - log the actual format code */
	LOG(ISC_IPA, Warning) << "Unknown format 0x" << std::hex << imageInfo_.format
	<< ", defaulting to Raw Bayer processing";
	generateStatsFromBayer(values.data(), imageInfo_, imageStats);
	}

	/* Create results container */
	ControlList results(controls::controls);

	/* Process each algorithm using the common statistics */
	awb_->process(imageStats, results);
	agc_->process(imageStats, results);
	ccm_->process(imageStats, results);

	/* Emit complete signal */
	awbComplete.emit(0, results);
	}


private:
	/* Unique pointers to algorithms */
	std::unique_ptr<AWB> awb_;
	std::unique_ptr<AGC> agc_;
	std::unique_ptr<CCM> ccm_;

	/* Common image information */
	ImageInfo imageInfo_;
};

} /* namespace ipa::microchip_isc */
} /* namespace libcamera */

/*
	* External IPA module interface
	*/
extern "C" {
extern const struct libcamera::IPAModuleInfo ipaModuleInfo;
extern libcamera::IPAInterface *ipaCreate();
}

const struct libcamera::IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"microchip-isc",
	"microchip-isc",
};

libcamera::IPAInterface *ipaCreate()
{
	return new libcamera::ipa::microchip_isc::IPAMicrochipISC();
}
