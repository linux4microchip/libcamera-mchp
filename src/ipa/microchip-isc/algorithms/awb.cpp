/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto White Balance for Microchip ISC
 */

#include "awb.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_AWB)

namespace ipa::microchip_isc {

/**
 * \brief Initialize the AWB algorithm from YAML tuning data
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for AWB
 * \return 0 on success
 */
int Awb::init([[maybe_unused]] IPAContext &context,
	      const YamlObject &tuningData)
{
	if (tuningData.contains("convergence")) {
		const YamlObject &conv = tuningData["convergence"];
		smoothingFactor_ = conv["speed"].get<float>(smoothingFactor_);
		damping_ = conv["damping"].get<float>(damping_);
		stabilityThreshold_ = conv["stability_threshold"].get<float>(stabilityThreshold_);
		resumeThreshold_ = stabilityThreshold_ * 2.0f;
		convergenceFrames_ = conv["frames"].get<unsigned int>(convergenceFrames_);
		darkSceneThreshold_ = conv["min_scene_luminance"].get<float>(darkSceneThreshold_);
	}

	if (tuningData.contains("gain_limits")) {
		const YamlObject &gl = tuningData["gain_limits"];
		gainMin_ = gl["r_b_min"].get<float>(gainMin_);
		gainMax_ = gl["r_b_max"].get<float>(gainMax_);
		gbGainMin_ = gl["gb_min"].get<float>(gbGainMin_);
		gbGainMax_ = gl["gb_max"].get<float>(gbGainMax_);
	}

	if (tuningData.contains("reference_gains")) {
		const YamlObject &rg = tuningData["reference_gains"];
		refRGain_ = rg["r"].get<double>(refRGain_);
		refBGain_ = rg["b"].get<double>(refBGain_);
		refGrGain_ = rg["gr"].get<double>(refGrGain_);
		refGbGain_ = rg["gb"].get<double>(refGbGain_);
	}

	highlightWeight_ = tuningData["highlight_weight"].get<float>(highlightWeight_);
	highlightWeight_ = std::clamp(highlightWeight_, 0.0f, 1.0f);

	if (tuningData.contains("color_temperature")) {
		const YamlObject &ct = tuningData["color_temperature"];
		ctMin_ = ct["min"].get<float>(ctMin_);
		ctMax_ = ct["max"].get<float>(ctMax_);
	}

	damping_ = std::clamp(damping_, 0.0f, 1.0f);
	smoothingFactor_ = std::clamp(smoothingFactor_, 0.0f, 0.99f);
	darkSceneThreshold_ = std::clamp(darkSceneThreshold_, 0.0f, 255.0f);
	gainMin_ = std::clamp(gainMin_, 0.1f, 1.0f);
	gainMax_ = std::clamp(gainMax_, 1.0f, 8.0f);
	gbGainMin_ = std::clamp(gbGainMin_, 0.5f, 1.0f);
	gbGainMax_ = std::clamp(gbGainMax_, 1.0f, 2.0f);
	ctMin_ = std::clamp(ctMin_, 1000.0f, 4000.0f);
	ctMax_ = std::clamp(ctMax_, 5000.0f, 20000.0f);

	/*
	 * Load the sensor-to-XYZ colour matrix for Robertson (1968) CCT
	 * estimation. Tunable via ct_color_matrix in the Awb tuning block.
	 * Defaults to kDefaultCtMatrix when absent.
	 */
	context.configuration.sensor.ctColorMatrix = kDefaultCtMatrix;
	if (tuningData.contains("ct_color_matrix")) {
		auto matList = tuningData["ct_color_matrix"].getList<float>();
		if (matList && matList->size() == 9) {
			std::copy(matList->begin(), matList->end(),
				  context.configuration.sensor.ctColorMatrix.begin());
			LOG(ISC_AWB, Debug) << "Loaded ct_color_matrix from tuning";
		}
	}

	LOG(ISC_AWB, Debug)
		<< "AWB initialized: smoothing=" << smoothingFactor_
		<< " damping=" << damping_
		<< " threshold=" << stabilityThreshold_
		<< " frames=" << convergenceFrames_
		<< " darkThreshold=" << darkSceneThreshold_
		<< " gainRange=[" << gainMin_ << "," << gainMax_ << "]"
		<< " ref=[R=" << refRGain_ << " B=" << refBGain_ << "]"
		<< " ctRange=[" << ctMin_ << "," << ctMax_ << "]";

	return 0;
}

/**
 * \brief Set initial AWB gains from tuning reference on (re-)configure
 * \param[in] context The IPA context
 * \param[in] sensorInfo Camera sensor information (unused)
 * \return 0 on success
 *
 * Applies the YAML reference_gains as the starting white balance so the
 * first frames have correct colour before the AWB loop converges. Without
 * this, identity gains (1.0/1.0/1.0/1.0) are used for the first ~8 frames,
 * producing a visibly incorrect white balance during startup.
 */
int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &sensorInfo)
{
	context.activeState.awb.rGain = refRGain_;
	context.activeState.awb.bGain = refBGain_;
	context.activeState.awb.grGain = refGrGain_;
	context.activeState.awb.gbGain = refGbGain_;
	awbStable_ = false;

	LOG(ISC_AWB, Debug)
		<< "AWB configured with reference gains: R=" << refRGain_
		<< " GR=" << refGrGain_ << " GB=" << refGbGain_
		<< " B=" << refBGain_;

	return 0;
}

/**
 * \brief Handle per-frame request controls for AWB
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[out] frameContext The frame context to populate
 * \param[in] controls Application-supplied controls for this frame
 *
 * Records the gains currently in activeState into frameContext so that
 * process() can undo the applied gains when correcting the histogram.
 * Handles AwbEnable and ColourGains for manual white balance.
 */
void Awb::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &awb = frameContext.awb;

	const auto &awbEnable = controls.get(controls::AwbEnable);
	if (awbEnable.has_value()) {
		awb.autoEnabled = awbEnable.value();
		context.activeState.awb.autoEnabled = awbEnable.value();
	} else {
		awb.autoEnabled = context.activeState.awb.autoEnabled;
	}

	if (!awb.autoEnabled) {
		const auto &colourGains = controls.get(controls::ColourGains);
		if (colourGains.has_value()) {
			context.activeState.awb.rGain = (*colourGains)[0];
			context.activeState.awb.bGain = (*colourGains)[1];
			/*
			 * ColourGains[R, B] is relative to green as the unity
			 * reference (libcamera API convention).  The ISC WB block
			 * has four independent Bayer channels (GR, R, GB, B); set
			 * the two green channels to 1.0 so the hardware treats green
			 * as neutral and only R and B are scaled by the user gains.
			 * This matches the intent of the ColourGains control and
			 * prevents stale auto-AWB green-balance gains from corrupting
			 * the manual white balance.
			 */
			context.activeState.awb.grGain = 1.0f;
			context.activeState.awb.gbGain = 1.0f;
		}
	}

	/*
	 * Record the gains for this frame context. These are used in
	 * process() for ColourGains metadata reporting in manual mode.
	 * Note: gain-undo is NOT needed in process() because the ISC
	 * histogram taps pre-WB data (vp_data) — channel means are raw
	 * sensor values independent of the hardware WB gains.
	 */
	awb.rGain = context.activeState.awb.rGain;
	awb.bGain = context.activeState.awb.bGain;
	awb.grGain = context.activeState.awb.grGain;
	awb.gbGain = context.activeState.awb.gbGain;
}

/**
 * \brief Process statistics and compute white balance gains
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context (contains applied gains)
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] metadata Control list for frame metadata
 *
 * Implements grey-world white balance: assumes the average scene colour is
 * neutral grey. Gains are computed to equalise the mean of each Bayer
 * channel to the green reference. Temporal smoothing prevents flickering.
 *
 * The ISC histogram is collected after white balance gains are applied by
 * the hardware. The previously applied gains stored in frameContext are
 * used to undo this effect and recover the true sensor illuminant signal.
 *
 * Color temperature is estimated from gain-undone R/B means and written to
 * activeState.awb.ct for use by CCM (single source of truth).
 */
void Awb::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const isc_stat_buffer *stats,
		  ControlList &metadata)
{
	auto &awb = context.activeState.awb;

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable) {
		std::array<float, 2> gains = {
			static_cast<float>(awb.rGain),
			static_cast<float>(awb.bGain),
		};
		metadata.set(controls::ColourGains, gains);
		metadata.set(controls::AwbLocked, awbStable_);
		if (awb.ct > 0)
			metadata.set(controls::ColourTemperature,
				     static_cast<int32_t>(awb.ct));
		return;
	}

	/* In manual mode, report current gains and skip auto computation */
	if (!frameContext.awb.autoEnabled) {
		std::array<float, 2> gains = {
			static_cast<float>(awb.rGain),
			static_cast<float>(awb.bGain),
		};
		metadata.set(controls::ColourGains, gains);
		metadata.set(controls::AwbLocked, awbStable_);
		if (awb.ct > 0)
			metadata.set(controls::ColourTemperature,
				     static_cast<int32_t>(awb.ct));
		return;
	}

	/* Calculate per-channel means from hardware histograms (gain-applied) */
	float meanGR = calculateChannelMean(stats, kHistChannelGR);
	float meanR = calculateChannelMean(stats, kHistChannelR);
	float meanGB = calculateChannelMean(stats, kHistChannelGB);
	float meanB = calculateChannelMean(stats, kHistChannelB);

	/*
	 * Guard against AWB updates in near-dark scenes. The check uses
	 * gain-applied histogram values so that darkSceneThreshold_ is
	 * calibrated directly in histogram units (0–511), matching the
	 * YAML documentation. Using gain-undone values would make the
	 * effective threshold dependent on the current AWB gains and would
	 * produce premature freezing whenever rGain or bGain exceed ~1.0.
	 */
	float meanGApplied = (meanGR + meanGB) / 2.0f;
	float lumApplied = 0.299f * meanR + 0.587f * meanGApplied + 0.114f * meanB;

	if (lumApplied >= darkSceneThreshold_) {
		/*
		 * The ISC histogram is collected after white balance gains are
		 * applied by the hardware. Undo the previously applied gains
		 * stored in frameContext to recover the true sensor illuminant
		 * signal for CT estimation and grey-world gain computation.
		 */
		if (frameContext.awb.rGain > 0.0 && frameContext.awb.bGain > 0.0 &&
		    frameContext.awb.grGain > 0.0 && frameContext.awb.gbGain > 0.0) {
			meanR /= frameContext.awb.rGain;
			meanB /= frameContext.awb.bGain;
			meanGR /= frameContext.awb.grGain;
			meanGB /= frameContext.awb.gbGain;
		}

		/*
		 * Blend full-frame grey-world means with highlight-based means.
		 * Near-saturated pixels (top 20% intensity, gain-undone) better
		 * approximate the illuminant colour than the full-frame mean in
		 * scenes with strong dominant colours (sky, foliage, skin tones).
		 * highlightWeight_ = 0 gives pure grey-world.
		 */
		if (highlightWeight_ > 0.0f) {
			float hlGR = calculateHighlightMean(stats, kHistChannelGR);
			float hlR  = calculateHighlightMean(stats, kHistChannelR);
			float hlGB = calculateHighlightMean(stats, kHistChannelGB);
			float hlB  = calculateHighlightMean(stats, kHistChannelB);

			/* Gain-undo highlight means with the same applied gains */
			if (frameContext.awb.rGain > 0.0 &&
			    frameContext.awb.bGain > 0.0 &&
			    frameContext.awb.grGain > 0.0 &&
			    frameContext.awb.gbGain > 0.0) {
				hlR  /= frameContext.awb.rGain;
				hlB  /= frameContext.awb.bGain;
				hlGR /= frameContext.awb.grGain;
				hlGB /= frameContext.awb.gbGain;
			}

			float w = highlightWeight_;
			meanGR = (1.0f - w) * meanGR + w * hlGR;
			meanR  = (1.0f - w) * meanR  + w * hlR;
			meanGB = (1.0f - w) * meanGB + w * hlGB;
			meanB  = (1.0f - w) * meanB  + w * hlB;
		}

		/* avgGreen for Robertson CT estimation only */
		float meanG = (meanGR + meanGB) / 2.0f;

		/*
		 * Estimate colour temperature from gain-undone channel means
		 * using Robertson's (1968) 31-point Planckian locus method
		 * with the sensor-to-XYZ matrix from the tuning file. AWB is
		 * the single authority for CT; CCM reads activeState.awb.ct.
		 */
		float rawCt = estimateColorTemperature(
			meanR, meanG, meanB,
			&context.configuration.sensor.ctColorMatrix);

		/*
		 * Grey world: normalise R and B to GR (not avgGreen).
		 * grGain = 1.0 (GR is the reference); rGain and bGain must
		 * use the same reference so all four channels equalise to GR.
		 */
		double rGain = std::clamp(static_cast<double>(meanGR / meanR),
					  static_cast<double>(gainMin_),
					  static_cast<double>(gainMax_));
		double bGain = std::clamp(static_cast<double>(meanGR / meanB),
					  static_cast<double>(gainMin_),
					  static_cast<double>(gainMax_));
		double grGain = 1.0;
		double gbGain = std::clamp(static_cast<double>(meanGR / meanGB),
					   static_cast<double>(gbGainMin_),
					   static_cast<double>(gbGainMax_));

		if (frame > 0 && awb.rGain > 0.0) {
			double rDelta = std::abs(rGain - awb.rGain) / awb.rGain;
			double bDelta = std::abs(bGain - awb.bGain) / awb.bGain;
			double gbDelta = std::abs(gbGain - awb.gbGain) / awb.gbGain;

			/*
			 * Hysteresis: enter stable below stabilityThreshold_;
			 * only exit when delta exceeds resumeThreshold_ (2x).
			 * Prevents colour flicker near the stability boundary.
			 */
			double maxDelta = std::max({ rDelta, bDelta, gbDelta });
			if (!awbStable_)
				awbStable_ = maxDelta < stabilityThreshold_;
			else if (maxDelta > resumeThreshold_)
				awbStable_ = false;

			if (awbStable_) {
				/* Within stability threshold: skip update */
				LOG(ISC_AWB, Debug) << "AWB stable, keeping gains";
			} else {
				float update = 1.0f - smoothingFactor_;
				if (convergenceFrames_ > 0)
					update = std::max(update,
							  1.0f / convergenceFrames_);
				update *= damping_;
				update = std::clamp(update, 0.01f, 1.0f);

				rGain = awb.rGain + update * (rGain - awb.rGain);
				bGain = awb.bGain + update * (bGain - awb.bGain);
				gbGain = awb.gbGain + update * (gbGain - awb.gbGain);

				awb.rGain = rGain;
				awb.bGain = bGain;
				awb.grGain = grGain;
				awb.gbGain = gbGain;
				awb.rOffset = 0;
				awb.bOffset = 0;
				awb.grOffset = 0;
				awb.gbOffset = 0;

				LOG(ISC_AWB, Debug)
					<< "AWB gains: R=" << rGain
					<< " GR=" << grGain
					<< " GB=" << gbGain
					<< " B=" << bGain
					<< " (means: R=" << meanR
					<< " GR=" << meanGR
					<< " GB=" << meanGB
					<< " B=" << meanB << ")";
			}
		} else {
			/* First frame: set gains directly */
			awb.rGain = rGain;
			awb.bGain = bGain;
			awb.grGain = grGain;
			awb.gbGain = gbGain;

			LOG(ISC_AWB, Debug)
				<< "AWB gains: R=" << rGain
				<< " GR=" << grGain
				<< " GB=" << gbGain
				<< " B=" << bGain
				<< " (means: R=" << meanR
				<< " GR=" << meanGR
				<< " GB=" << meanGB
				<< " B=" << meanB << ")";
		}
	} else {
		/* First frame: set gains directly */
		awb.rGain = rGain;
		awb.bGain = bGain;
		awb.grGain = grGain;
		awb.gbGain = gbGain;
	}

	/*
		 * Update colour temperature with temporal smoothing.
		 * Apply damping_ in addition to smoothingFactor_ so CT and
		 * gains converge at the same rate — without it CT leads ahead
		 * of the gains, causing a transient CCM matrix mismatch during
		 * scene transitions (wrong matrix applied with old AWB gains).
		 */
	float prevCt = (awb.ct > 0) ? static_cast<float>(awb.ct) : rawCt;
	float update = 1.0f - smoothingFactor_;
	if (convergenceFrames_ > 0)
		update = std::max(update, 1.0f / convergenceFrames_);
	update *= damping_;
	update = std::clamp(update, 0.01f, 1.0f);
	float smoothedCt = prevCt + update * (rawCt - prevCt);
	awb.ct = static_cast<unsigned int>(
		std::clamp(smoothedCt, ctMin_, ctMax_));

	LOG(ISC_AWB, Debug) << "AWB CT: raw=" << rawCt
			    << " K smoothed=" << awb.ct << " K";
}
else
{
	LOG(ISC_AWB, Debug)
		<< "Scene too dark for AWB (lum=" << lumApplied
		<< " < threshold=" << darkSceneThreshold_
		<< "), CT frozen at " << awb.ct << " K";
}

std::array<float, 2> gains = {
	static_cast<float>(awb.rGain),
	static_cast<float>(awb.bGain),
};
metadata.set(controls::ColourGains, gains);
metadata.set(controls::AwbLocked, awbStable_);

if (awb.ct > 0)
	metadata.set(controls::ColourTemperature, static_cast<int32_t>(awb.ct));
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
