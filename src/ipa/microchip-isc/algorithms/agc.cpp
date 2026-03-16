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
 *
 * Parses convergence parameters and gain/exposure limits from the tuning
 * file. Only MeteringMatrix is supported; the ISC histogram has no spatial
 * information so centre-weighted and spot modes are not implementable.
 */
int Agc::init(IPAContext &context, const YamlObject &tuningData)
{
	targetBrightness_ = tuningData["target_brightness"].get<float>(targetBrightness_);
	clipThreshold_ = tuningData["clip_threshold"].get<float>(clipThreshold_);
	aeLuminanceFloor_ = tuningData["ae_luminance_floor"].get<float>(aeLuminanceFloor_);
	aeLuminanceFloor_ = std::clamp(aeLuminanceFloor_, 0.0f, 50.0f);
	flickerHz_ = tuningData["flicker_hz"].get<float>(flickerHz_);
	/* Only 50 Hz and 60 Hz are valid mains frequencies; anything else disables. */
	if (flickerHz_ != 50.0f && flickerHz_ != 60.0f)
		flickerHz_ = 0.0f;

	if (tuningData.contains("convergence")) {
		const YamlObject &conv = tuningData["convergence"];
		convergenceSpeed_ = conv["speed"].get<float>(convergenceSpeed_);
		stabilityThreshold_ = conv["stability_threshold"].get<float>(stabilityThreshold_);
		convergenceFrames_ = conv["frames"].get<unsigned int>(convergenceFrames_);
	}

	if (tuningData.contains("ratio_clamp")) {
		const YamlObject &rc = tuningData["ratio_clamp"];
		ratioMin_ = rc["min"].get<float>(ratioMin_);
		ratioMax_ = rc["max"].get<float>(ratioMax_);
		clipRatioCap_ = rc["clip_cap"].get<float>(clipRatioCap_);
	}

	if (tuningData.contains("gain_thresholds")) {
		const YamlObject &gt = tuningData["gain_thresholds"];
		gainBoostThreshold_ = gt["boost"].get<float>(gainBoostThreshold_);
		gainReductionThreshold_ = gt["reduction"].get<float>(gainReductionThreshold_);
	}

	if (convergenceFrames_ > 0) {
		float frameSpeed = 1.0f / convergenceFrames_;
		convergenceSpeed_ = std::max(convergenceSpeed_, frameSpeed);
	}

	convergenceSpeed_ = std::clamp(convergenceSpeed_, 0.01f, 1.0f);
	clipThreshold_ = std::clamp(clipThreshold_, 0.001f, 0.5f);
	ratioMin_ = std::clamp(ratioMin_, 0.1f, 0.9f);
	ratioMax_ = std::clamp(ratioMax_, 1.1f, 8.0f);
	clipRatioCap_ = std::clamp(clipRatioCap_, 0.1f, 1.0f);
	gainBoostThreshold_ = std::clamp(gainBoostThreshold_, 1.0f, 2.0f);
	gainReductionThreshold_ = std::clamp(gainReductionThreshold_, 0.5f, 1.0f);

	/* Advertise ExposureValue: ±2 EV in 0.1-stop steps */
	context.ctrlMap[&controls::ExposureValue] =
		ControlInfo(-2.0f, 2.0f, 0.0f);

	/*
	 * FrameDuration: per-frame read-only metadata reporting actual frame
	 * duration in µs. Range is placeholder; configure() overwrites it with
	 * sensor-mode-specific limits derived from HBLANK/VBLANK.
	 */
	context.ctrlMap[&controls::FrameDuration] =
		ControlInfo(static_cast<int64_t>(0),
			    static_cast<int64_t>(1000000),
			    static_cast<int64_t>(33333));

	/* Advertise ExposureTimeMode, AnalogueGainMode, AeMeteringMode, AeState */
	context.ctrlMap[&controls::AeState] =
		ControlInfo(controls::AeStateIdle, controls::AeStateConverged,
			    controls::AeStateIdle);
	context.ctrlMap[&controls::ExposureTimeMode] =
		ControlInfo({ { ControlValue(controls::ExposureTimeModeAuto),
				ControlValue(controls::ExposureTimeModeManual) } },
			    ControlValue(controls::ExposureTimeModeAuto));
	context.ctrlMap[&controls::AnalogueGainMode] =
		ControlInfo({ { ControlValue(controls::AnalogueGainModeAuto),
				ControlValue(controls::AnalogueGainModeManual) } },
			    ControlValue(controls::AnalogueGainModeAuto));
	/*
	 * Only MeteringMatrix is supported. The ISC hardware histogram does
	 * not carry spatial information; centre-weighted and spot metering
	 * are not implementable without a separate statistics block.
	 */
	context.ctrlMap[&controls::AeMeteringMode] =
		ControlInfo(std::vector<ControlValue>{ ControlValue(controls::MeteringMatrix) },
			    ControlValue(controls::MeteringMatrix));

	LOG(ISC_AGC, Debug)
		<< "AGC initialized: target=" << targetBrightness_
		<< " speed=" << convergenceSpeed_
		<< " threshold=" << stabilityThreshold_
		<< " frames=" << convergenceFrames_
		<< " clipThreshold=" << clipThreshold_
		<< " ratioRange=[" << ratioMin_ << "," << ratioMax_ << "]"
		<< " clipCap=" << clipRatioCap_
		<< " gainThresholds=[" << gainReductionThreshold_
		<< "," << gainBoostThreshold_ << "]";

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
	 * Initialise sensor-limit-derived exposure and gain defaults.
	 * Mode fields (autoEnabled, exposureTimeMode, analogueGainMode) are
	 * owned by IPAMicrochipISC::configure() which runs after all algorithm
	 * configure() calls and is the single authority for shared active state
	 * initialisation.
	 */
	context.activeState.agc.exposure = agc.defaultExposure;
	context.activeState.agc.analogueGain = agc.minAnalogueGain;

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
 * Handles AeEnable to toggle auto/manual mode, ExposureTimeMode and
 * AnalogueGainMode for independent control of exposure and gain, and
 * AeMeteringMode for metering strategy selection. Manual override values
 * are read from ExposureTime and AnalogueGain controls.
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

	/* Independent exposure time mode (overrides AeEnable for exposure) */
	const auto &expMode = controls.get(controls::ExposureTimeMode);
	if (expMode.has_value()) {
		agc.exposureTimeMode = expMode.value();
		context.activeState.agc.exposureTimeMode = expMode.value();
	} else {
		agc.exposureTimeMode = context.activeState.agc.exposureTimeMode;
	}

	/* Independent analogue gain mode (overrides AeEnable for gain) */
	const auto &gainMode = controls.get(controls::AnalogueGainMode);
	if (gainMode.has_value()) {
		agc.analogueGainMode = gainMode.value();
		context.activeState.agc.analogueGainMode = gainMode.value();
	} else {
		agc.analogueGainMode = context.activeState.agc.analogueGainMode;
	}

	/* EV compensation: scale target brightness up/down by 2^ev stops */
	const auto &evControl = controls.get(controls::ExposureValue);
	if (evControl.has_value())
		context.activeState.agc.exposureValue =
			std::clamp(evControl.value(), -2.0f, 2.0f);

	bool exposureManual = !agc.autoEnabled ||
			      agc.exposureTimeMode == controls::ExposureTimeModeManual;
	bool gainManual = !agc.autoEnabled ||
			  agc.analogueGainMode == controls::AnalogueGainModeManual;

	if (exposureManual) {
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
	} else {
		agc.exposure = context.activeState.agc.exposure;
	}

	if (gainManual) {
		const auto &gain = controls.get(controls::AnalogueGain);
		if (gain.has_value()) {
			agc.analogueGain = gain.value();
			context.activeState.agc.analogueGain = gain.value();
		} else {
			agc.analogueGain = context.activeState.agc.analogueGain;
		}
	} else {
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
 * adjusts exposure time and analogue gain independently based on
 * ExposureTimeMode and AnalogueGainMode.
 *
 * Exposure is preferred over gain to minimize noise; gain is only increased
 * when exposure is at its maximum. ExposureTimeMode=Manual locks the
 * shutter while allowing gain to auto-adjust, and vice versa.
 */
void Agc::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const mchp_isc_stat_buffer *stats,
		  ControlList &metadata)
{
	auto &agcState = context.activeState.agc;
	const auto &agcConfig = context.configuration.agc;

	bool exposureManual = !frameContext.agc.autoEnabled ||
			      frameContext.agc.exposureTimeMode == controls::ExposureTimeModeManual;
	bool gainManual = !frameContext.agc.autoEnabled ||
			  frameContext.agc.analogueGainMode == controls::AnalogueGainModeManual;

	/* In fully manual mode, report current values and skip auto computation */
	if (exposureManual && gainManual) {
		int32_t exposureUs = static_cast<int32_t>(
			frameContext.agc.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain, frameContext.agc.analogueGain);
		metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));
		metadata.set(controls::AeState,
			     static_cast<int32_t>(controls::AeStateIdle));
		return;
	}

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable) {
		int32_t exposureUs = static_cast<int32_t>(
			agcState.exposure *
			context.configuration.sensor.lineDuration.get<std::micro>());
		metadata.set(controls::ExposureTime, exposureUs);
		metadata.set(controls::AnalogueGain, agcState.analogueGain);
		metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));
		metadata.set(controls::AeState,
			     static_cast<int32_t>(controls::AeStateIdle));
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
		metadata.set(controls::AnalogueGain, agcState.analogueGain);
		metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));
		metadata.set(controls::AeState,
			     static_cast<int32_t>(controls::AeStateSearching));
		return;
	}

	/*
	 * Use the shared scene luminance pre-computed in processStats() from
	 * the BT.601-weighted histogram means.  Only MeteringMatrix is
	 * supported; the ISC histogram has no spatial information so
	 * centre-weighted and spot modes read the same full-frame value.
	 */
	float currentLuminance = frameContext.scene.luminance;

	/*
	 * Apply EV compensation: shift the target brightness by 2^ev stops.
	 * Positive EV brightens (e.g. +1 EV → 2× target); negative darkens.
	 * Clamp to the valid histogram range [aeLuminanceFloor_, kHistogramMax].
	 */
	float evScale = std::pow(2.0f, agcState.exposureValue);
	float effectiveTarget = std::clamp(targetBrightness_ * evScale,
					   aeLuminanceFloor_, kHistogramMax);

	bool aeStable = currentLuminance >= aeLuminanceFloor_ &&
			std::abs(effectiveTarget - currentLuminance) < stabilityThreshold_;
	agcState.aeConverged = aeStable;

	if (!aeStable) {
		float ratio = effectiveTarget / currentLuminance;
		ratio = std::clamp(ratio, ratioMin_, ratioMax_);

		/*
		 * Anti-windup for highlight clipping: in high-contrast outdoor
		 * scenes the luminance mean can appear low because large shadow
		 * areas drag the average down, even though highlights are
		 * already saturated. Without this guard the control loop boosts
		 * exposure further, causing oscillation. Cap the ratio to force
		 * a reduction whenever significant clipping is detected.
		 */
		float clipping = frameContext.scene.clipping;
		if (clipping > clipThreshold_) {
			ratio = std::min(ratio, clipRatioCap_);
			LOG(ISC_AGC, Debug)
				<< "Clipping " << clipping * 100.0f
				<< "% > " << clipThreshold_ * 100.0f
				<< "%, capping ratio=" << ratio;
		}

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
					 ratio) / maxExposure;
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

			analogueGain = std::clamp(analogueGain,
						  agcConfig.minAnalogueGain,
						  agcConfig.maxAnalogueGain);

			/* Temporal smoothing prevents sudden gain jumps */
			analogueGain = convergenceSpeed_ * analogueGain +
				       (1.0f - convergenceSpeed_) *
				       frameContext.agc.analogueGain;

			agcState.analogueGain = analogueGain;
		}

		LOG(ISC_AGC, Debug)
			<< "AGC: luminance=" << currentLuminance
			<< " target=" << effectiveTarget
			<< " ev=" << agcState.exposureValue
			<< " ratio=" << ratio
			<< " -> exposure=" << agcState.exposure
			<< " gain=" << agcState.analogueGain
			<< (exposureManual ? " (exposure locked)" : "")
			<< (gainManual ? " (gain locked)" : "");
	} else {
		LOG(ISC_AGC, Debug)
			<< "AGC stable (luminance=" << currentLuminance
			<< " target=" << effectiveTarget << ")";
	}

	int32_t exposureUs = static_cast<int32_t>(
		agcState.exposure *
		context.configuration.sensor.lineDuration.get<std::micro>());
	metadata.set(controls::ExposureTime, exposureUs);
	metadata.set(controls::AnalogueGain, agcState.analogueGain);
	metadata.set(controls::FrameDuration, static_cast<int64_t>(exposureUs));

	/*
	 * Report AeState. Both exposure and gain are considered when deciding
	 * if the AEGC loop has converged, so convergence is declared only when
	 * both are within the stability band. Manual-only modes are reported
	 * as Idle since the algorithm is not computing values.
	 */
	int32_t aeState;
	if (exposureManual && gainManual) {
		aeState = controls::AeStateIdle;
	} else if (aeStable) {
		aeState = controls::AeStateConverged;
	} else {
		aeState = controls::AeStateSearching;
	}
	metadata.set(controls::AeState, aeState);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
