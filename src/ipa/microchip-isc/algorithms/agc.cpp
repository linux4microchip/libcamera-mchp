/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto Gain/Exposure Control for Microchip ISC
 */

#include "agc.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "common.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_AGC)

namespace ipa::microchip_isc {

/**
 * \brief Initialize AGC from YAML tuning data
 * \param[in] context The IPA context
 * \param[in] tuningData YAML tuning parameters for AGC
 * \return 0 on success
 */
int Agc::init([[maybe_unused]] IPAContext &context,
	      const YamlObject &tuningData)
{
	targetBrightness_ = tuningData["target_brightness"].get<float>(targetBrightness_);
	clipThreshold_ = tuningData["clip_threshold"].get<float>(clipThreshold_);

	if (tuningData.contains("convergence")) {
		const YamlObject &conv = tuningData["convergence"];
		convergenceSpeed_ = conv["speed"].get<float>(convergenceSpeed_);
		stabilityThreshold_ = conv["stability_threshold"].get<float>(stabilityThreshold_);
		convergenceFrames_ = conv["frames"].get<unsigned int>(convergenceFrames_);
	}

	if (convergenceFrames_ > 0) {
		float frameSpeed = 1.0f / convergenceFrames_;
		convergenceSpeed_ = std::max(convergenceSpeed_, frameSpeed);
	}

	convergenceSpeed_ = std::clamp(convergenceSpeed_, 0.01f, 1.0f);
	clipThreshold_ = std::clamp(clipThreshold_, 0.001f, 0.5f);

	LOG(ISC_AGC, Debug)
		<< "AGC initialized: target=" << targetBrightness_
		<< " speed=" << convergenceSpeed_
		<< " threshold=" << stabilityThreshold_
		<< " frames=" << convergenceFrames_
		<< " clipThreshold=" << clipThreshold_;

	return 0;
}

/**
 * \brief Configure AGC with sensor limits
 * \param[in] context The IPA context
 * \param[in] configInfo Camera sensor information
 * \return 0 on success
 *
 * Populates session configuration with sensor exposure and gain limits
 * and sets initial active state to sensor defaults.
 */
int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	auto &agc = context.configuration.agc;

	/*
	 * Set initial exposure to the midpoint of the sensor range.
	 * The sensor driver will clamp to its actual limits.
	 */
	context.activeState.agc.exposure = agc.defaultExposure;
	context.activeState.agc.analogueGain = agc.minAnalogueGain;
	context.activeState.agc.autoEnabled = true;

	LOG(ISC_AGC, Debug)
		<< "AGC configured: exposure=" << context.activeState.agc.exposure
		<< " gain=" << context.activeState.agc.analogueGain;

	return 0;
}

/**
 * \brief Handle per-frame request controls for AGC
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[out] frameContext The frame context to populate
 * \param[in] controls Application-supplied controls for this frame
 *
 * Handles AeEnable to toggle auto/manual mode, and ExposureTime /
 * AnalogueGain for manual override. Records the current active state
 * so process() can use it as the feedback operating point.
 */
void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	auto &agc = frameContext.agc;

	const auto &aeEnable = controls.get(controls::AeEnable);
	if (aeEnable.has_value()) {
		agc.autoEnabled = aeEnable.value();
		context.activeState.agc.autoEnabled = aeEnable.value();
	} else {
		agc.autoEnabled = context.activeState.agc.autoEnabled;
	}

	if (!agc.autoEnabled) {
		const auto &exposureTime = controls.get(controls::ExposureTime);
		if (exposureTime.has_value()) {
			uint32_t lines = static_cast<uint32_t>(std::round(
				exposureTime.value() /
				context.configuration.sensor.lineDuration.get<std::micro>()));
			agc.exposure = lines;
			context.activeState.agc.exposure = lines;
		} else {
			agc.exposure = context.activeState.agc.exposure;
		}

		const auto &gain = controls.get(controls::AnalogueGain);
		if (gain.has_value()) {
			agc.analogueGain = gain.value();
			context.activeState.agc.analogueGain = gain.value();
		} else {
			agc.analogueGain = context.activeState.agc.analogueGain;
		}
	} else {
		/* Record current active state for feedback in process() */
		agc.exposure = context.activeState.agc.exposure;
		agc.analogueGain = context.activeState.agc.analogueGain;
	}
}

/**
 * \brief Process statistics and compute exposure/gain
 * \param[in] context The IPA context
 * \param[in] frame The frame number
 * \param[in] frameContext The frame context (contains actual applied values)
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] metadata Control list for frame metadata
 *
 * Computes scene luminance from the ISC 4-channel 512-bin histogram and
 * adjusts exposure time and analogue gain to reach the target brightness.
 * Exposure is preferred over gain to minimize noise; gain is only increased
 * when exposure is at its maximum.
 *
 * Uses the actual applied sensor values from frameContext as the feedback
 * operating point for accurate closed-loop control.
 */
void Agc::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const isc_stat_buffer *stats,
		  ControlList &metadata)
{
	auto &agcState = context.activeState.agc;
	const auto &agcConfig = context.configuration.agc;

	/* In manual mode, report current values and skip auto computation */
	if (!frameContext.agc.autoEnabled) {
		int32_t exposureUs = static_cast<int32_t>(
			frameContext.agc.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain,
			     static_cast<float>(frameContext.agc.analogueGain));
		return;
	}

	/*
	 * Warmup guard: skip AGC computation until the IPA has received at
	 * least one complete 4-channel stats delivery and the first few
	 * frames of sensor stabilisation have passed.
	 */
	if (frame < 3) {
		LOG(ISC_AGC, Debug) << "AGC warmup frame " << frame << ", skipping";
		int32_t exposureUs = static_cast<int32_t>(
			agcState.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain,
			     static_cast<float>(agcState.analogueGain));
		return;
	}

	float currentLuminance = calculateLuminance(stats);

	if (currentLuminance >= 0.5f &&
	    std::abs(targetBrightness_ - currentLuminance) >= stabilityThreshold_) {
		float ratio = targetBrightness_ / currentLuminance;
		ratio = std::clamp(ratio, 0.25f, 4.0f);

		/*
		 * Anti-windup for highlight clipping: in high-contrast outdoor
		 * scenes the luminance mean can appear low because large shadow
		 * areas drag the average down, even though highlights are
		 * already saturated. Without this guard the control loop boosts
		 * exposure further, causing oscillation. Cap the ratio to force
		 * a reduction whenever significant clipping is detected.
		 */
		float clipping = calculateClipping(stats);
		if (clipping > clipThreshold_) {
			ratio = std::min(ratio, 0.85f);
			LOG(ISC_AGC, Debug)
				<< "Clipping " << clipping * 100.0f
				<< "% > " << clipThreshold_ * 100.0f
				<< "%, capping ratio=" << ratio;
		}

		/*
		 * Use the actual applied exposure (read back from sensor
		 * controls in processStats) as the feedback operating point.
		 */
		float currentExposure = static_cast<float>(frameContext.agc.exposure);
		float targetExposure = currentExposure * ratio;
		float newExposure = convergenceSpeed_ * targetExposure +
				    (1.0f - convergenceSpeed_) * currentExposure;

		uint32_t maxExposure = static_cast<uint32_t>(
			agcConfig.maxShutterSpeed / context.configuration.sensor.lineDuration);
		uint32_t minExposure = static_cast<uint32_t>(
			agcConfig.minShutterSpeed / context.configuration.sensor.lineDuration);

		/* Update exposure if not in manual mode */
		if (!exposureManual) {
			float currentExposure = static_cast<float>(frameContext.agc.exposure);
			float targetExposure = currentExposure * ratio;
			float newExposure = convergenceSpeed_ * targetExposure +
					    (1.0f - convergenceSpeed_) * currentExposure;

			uint32_t exposure = std::clamp(static_cast<uint32_t>(newExposure),
						       minExposure, maxExposure);
			agcState.exposure = exposure;

			/*
			 * Flicker avoidance: round exposure to the nearest
			 * multiple of the mains half-period (1/(2*Hz) seconds)
			 * to prevent banding under fluorescent / LED lighting.
			 * A half-period multiple ensures exposure is a symmetric
			 * integral number of half-cycles for both 50 and 60 Hz.
			 */
			if (flickerHz_ > 0.0f) {
				float lineDurationUs =
					context.configuration.sensor.lineDuration
						.get<std::micro>();
				float halfPeriodLines =
					(500000.0f / flickerHz_) / lineDurationUs;
				if (halfPeriodLines >= 1.0f) {
					uint32_t nPeriods = static_cast<uint32_t>(
						std::round(static_cast<float>(agcState.exposure) / halfPeriodLines));
					/* Ensure at least one half-period */
					if (nPeriods == 0)
						nPeriods = 1;
					agcState.exposure = std::clamp(
						static_cast<uint32_t>(nPeriods * halfPeriodLines),
						minExposure, maxExposure);
				}
			}
		}

		/* Update gain if not in manual mode */
		if (!gainManual) {
			float analogueGain;

			if (exposureManual) {
				/*
				 * Exposure is manually locked: gain must cover
				 * the full brightness adjustment since exposure
				 * cannot move. Apply ratio directly to the
				 * current gain so the loop converges.
				 */
				analogueGain = agcState.analogueGain * ratio;
			} else if (agcState.exposure >= maxExposure && ratio > gainBoostThreshold_) {
				/*
				 * Exposure is maxed and the scene is still
				 * underexposed: boost gain proportionally to
				 * the remaining brightness deficit.
				 */
				float remaining =
					(static_cast<float>(frameContext.agc.exposure) *
					 ratio) /
					maxExposure;
				analogueGain = agcConfig.minAnalogueGain * remaining;
			} else if (ratio < gainReductionThreshold_ &&
				   agcState.exposure <= minExposure) {
				/*
				 * Scene overexposed AND exposure is already at
				 * minimum: exposure cannot go lower, so reduce
				 * gain toward minimum to correct overexposure.
				 * Exposure is always preferred over gain to
				 * minimise noise; gain is only reduced once
				 * the exposure knob is exhausted.
				 */
				analogueGain = agcConfig.minAnalogueGain;
			} else {
				/*
				 * Exposure is still converging or can be
				 * reduced further: hold the current gain.
				 * Let the exposure loop handle the correction;
				 * adjusting gain simultaneously causes
				 * oscillation and prevents stable convergence.
				 */
				analogueGain = agcState.analogueGain;
			}

			/*
		 * Split into exposure time and analogue gain.
		 * Prefer exposure over gain: only use gain when exposure is maxed.
		 */
			double analogueGain = agcConfig.minAnalogueGain;
			if (exposure >= maxExposure && ratio > 1.05f) {
				float remaining = newExposure / maxExposure;
				analogueGain = agcConfig.minAnalogueGain * remaining;
				analogueGain = std::clamp(analogueGain,
							  agcConfig.minAnalogueGain,
							  agcConfig.maxAnalogueGain);

				/* Temporal smoothing prevents sudden gain jumps */
				analogueGain = convergenceSpeed_ * analogueGain +
					       (1.0f - convergenceSpeed_) *
						       frameContext.agc.analogueGain;

				agcState.analogueGain = analogueGain;
			}

			/* Temporal smoothing for gain using actual applied gain */
			if (frame > 0) {
				analogueGain = convergenceSpeed_ * analogueGain +
					       (1.0 - convergenceSpeed_) *
						       frameContext.agc.analogueGain;
			}

			agcState.exposure = exposure;
			agcState.analogueGain = analogueGain;

			LOG(ISC_AGC, Debug)
				<< "AGC: luminance=" << currentLuminance
				<< " target=" << targetBrightness_
				<< " ratio=" << ratio
				<< " -> exposure=" << exposure
				<< " gain=" << analogueGain;
		} else {
			LOG(ISC_AGC, Debug)
				<< "AGC stable (luminance=" << currentLuminance << ")";
		}

		int32_t exposureUs = static_cast<int32_t>(
			agcState.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain, static_cast<float>(agcState.analogueGain));
	}

	REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
