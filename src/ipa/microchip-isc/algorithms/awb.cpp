/* SPDX-License-Identifier: LGPL-2.1-or-later */

/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Auto White Balance for Microchip ISC
 */

#include "awb.h"

#include <algorithm>
#include <math.h>

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
		startupSpeed_ = conv["startup_speed"].get<float>(startupSpeed_);
		startupFrames_ = conv["startup_frames"].get<unsigned int>(0);
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
		refRGain_ = rg["r"].get<float>(refRGain_);
		refBGain_ = rg["b"].get<float>(refBGain_);
		refGrGain_ = rg["gr"].get<float>(refGrGain_);
		refGbGain_ = rg["gb"].get<float>(refGbGain_);
	}

	/*
	 * Per-CT reference gain table: list of {ct, r, b} entries in the
	 * tuning YAML. At configure() the IPA interpolates starting gains
	 * from the last-known colour temperature so the first frame has
	 * near-correct white balance without waiting for AWB to converge.
	 * Entries must be in ascending CT order.
	 */
	defaultCt_ = tuningData["ct_fallback"].get<float>(defaultCt_);
	if (tuningData.contains("reference_gains_table")) {
		for (const auto &entry : tuningData["reference_gains_table"].asList()) {
			float ct = entry["ct"].get<float>(0.0f);
			float r = entry["r"].get<float>(refRGain_);
			float b = entry["b"].get<float>(refBGain_);
			if (ct > 0.0f)
				refGainTable_.push_back({ ct, r, b });
		}
		/* Ensure ascending order so interpolation is correct */
		std::sort(refGainTable_.begin(), refGainTable_.end(),
			  [](const RefGainEntry &a, const RefGainEntry &b) {
				  return a.ct < b.ct;
			  });
		LOG(ISC_AWB, Debug)
			<< "Loaded " << refGainTable_.size()
			<< " per-CT reference gain entries";
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

	/*
	 * Advertise all controls emitted in metadata or accepted as input.
	 * Without registration here the controls do not appear in the camera's
	 * ControlInfoMap, so applications cannot query their ranges.
	 */
	context.ctrlMap[&controls::AwbEnable] =
		ControlInfo(false, true, true);
	context.ctrlMap[&controls::AwbLocked] =
		ControlInfo(false, true, false);
	/*
	 * ColourGains: [R gain, B gain] float pair. Clamp range matches the
	 * grey-world gain limits from the tuning file; default is neutral (1.0).
	 */
	context.ctrlMap[&controls::ColourGains] =
		ControlInfo(Span<const float>({ gainMin_, gainMin_ }),
			    Span<const float>({ gainMax_, gainMax_ }),
			    Span<const float>({ 1.0f, 1.0f }));
	context.ctrlMap[&controls::ColourTemperature] =
		ControlInfo(static_cast<int32_t>(ctMin_),
			    static_cast<int32_t>(ctMax_),
			    5500);

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
	/*
	 * Choose starting gains from the per-CT table when available.
	 * Use the last-known CT (awb.ct) if valid, otherwise fall back to
	 * defaultCt_ from the tuning file. This ensures the first captured
	 * frame has approximately correct white balance whether the camera
	 * is indoors (~3000 K) or outdoors (~6500 K) without waiting for
	 * the AWB loop to converge across illuminants.
	 */
	float startCt = (context.activeState.awb.ct > 0)
				? static_cast<float>(context.activeState.awb.ct)
				: defaultCt_;

	if (!refGainTable_.empty()) {
		auto [r, b] = interpolateRefGains(startCt);
		context.activeState.awb.rGain = r;
		context.activeState.awb.bGain = b;
	} else {
		context.activeState.awb.rGain = refRGain_;
		context.activeState.awb.bGain = refBGain_;
	}
	context.activeState.awb.grGain = refGrGain_;
	context.activeState.awb.gbGain = refGbGain_;
	awbStable_ = false;

	LOG(ISC_AWB, Debug)
		<< "AWB configured with reference gains: R="
		<< context.activeState.awb.rGain
		<< " GR=" << refGrGain_ << " GB=" << refGbGain_
		<< " B=" << context.activeState.awb.bGain
		<< " (CT=" << startCt << " K)";

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
 * The ISC histogram taps wb_data — the Bayer stream AFTER the WB block
 * (post-WB). Channel means already reflect the current hardware gains.
 * Grey-world gains must therefore use multiplicative correction
 * (current_gain x ratio) not a direct ratio, to avoid converging at
 * sqrt(K) instead of K. See gain computation below.
 *
 * Color temperature is estimated from gain-undone R/B means and written to
 * activeState.awb.ct for use by CCM (single source of truth).
 */
void Awb::process(IPAContext &context,
		  const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const mchp_isc_stat_buffer *stats,
		  ControlList &metadata)
{
	auto &awb = context.activeState.awb;

	/* Track calls from first stats delivery for startup speed selection */
	awbFrameCount_++;

	/* Skip computation when histogram stats are not available (non-RAW input) */
	if (!context.configuration.hw.statsAvailable) {
		std::array<float, 2> gains = {
			awb.rGain,
			awb.bGain,
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
			awb.rGain,
			awb.bGain,
		};
		metadata.set(controls::ColourGains, gains);
		metadata.set(controls::AwbLocked, awbStable_);
		if (awb.ct > 0)
			metadata.set(controls::ColourTemperature,
				     static_cast<int32_t>(awb.ct));
		return;
	}

	/*
	 * Read per-channel means from activeState. All four channels are
	 * always fresh — the ISC stats buffer delivers all four together once
	 * per 4-frame cycle (isc_stats_isr() fires only after B completes).
	 *
	 * Per-channel exposure normalization:
	 * The ISC measures each Bayer channel on a different frame within the
	 * 4-frame cycle (GR on frame N, R on N+1, GB on N+2, B on N+3).
	 * captureFrame[] records the actual frame sequence number for each
	 * channel. When AGC is ramping, the exposure between frames N and N+3
	 * may differ — computing rGain = meanGR / meanR directly would
	 * introduce a systematic error proportional to the AGC step size.
	 *
	 * Normalize each channel mean to a common exposure reference (the GR
	 * channel's frame) so that gains are computed at equivalent exposures.
	 * Use the frameContexts ring buffer to look up the actual exposure and
	 * gain applied when each channel was measured.
	 *
	 * Skip normalization when frameContexts history is shallow (first few
	 * frames after stream-on) or when channels are measured on the same
	 * frame (steady-state AGC — most common case, normalization is a no-op).
	 */
	const auto &hist = context.activeState.histogram;
	float meanGR = hist.channelMeans[kHistChannelGR];
	float meanR = hist.channelMeans[kHistChannelR];
	float meanGB = hist.channelMeans[kHistChannelGB];
	float meanB = hist.channelMeans[kHistChannelB];

	/*
	 * Normalize R, GB, B means to the GR frame's exposure.
	 * ratio = (exposure_GR * gain_GR) / (exposure_ch * gain_ch)
	 * Each channel mean is multiplied by this ratio so that all means
	 * are expressed as if they were measured at the GR frame's exposure.
	 *
	 * If frameContexts doesn't have history far enough back, or the
	 * frameContext lookup fails, fall through without normalization.
	 */
	{
		const uint32_t grFrame = hist.captureFrame[kHistChannelGR];

		auto getExpGain = [&](uint32_t frameSeq, float &expOut, float &gainOut) {
			/*
			 * FCQueue::get() returns a FrameContext & by reference
			 * using modular ring-buffer arithmetic; it never throws.
			 * Guard against an uninitialised slot with exposure == 0
			 * rather than a try/catch, which is the correct idiom.
			 */
			const auto &fc = static_cast<const IPAFrameContext &>(
				context.frameContexts.get(frameSeq));
			if (fc.agc.exposure == 0) {
				expOut = context.activeState.agc.exposure;
				gainOut = context.activeState.agc.analogueGain;
				return false;
			}
			expOut = static_cast<float>(fc.agc.exposure);
			gainOut = fc.agc.analogueGain;
			return true;
		};

		float grExp, grGain;
		getExpGain(grFrame, grExp, grGain);
		float grEV = grExp * grGain;

		auto normalize = [&](float mean, unsigned int ch) -> float {
			if (hist.captureFrame[ch] == grFrame)
				return mean; /* same frame — no normalization needed */
			float chExp, chGain;
			getExpGain(hist.captureFrame[ch], chExp, chGain);
			float chEV = chExp * chGain;
			if (chEV < 1e-6f)
				return mean;
			return mean * (grEV / chEV);
		};

		meanR = normalize(meanR, kHistChannelR);
		meanGB = normalize(meanGB, kHistChannelGB);
		meanB = normalize(meanB, kHistChannelB);
	}

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

	float rawCt = 0.0f;

	if (lumApplied >= darkSceneThreshold_) {
		/*
		 * Blend full-frame grey-world means with highlight-based means.
		 * Near-saturated pixels (top 20% intensity, gain-undone) better
		 * approximate the illuminant colour than the full-frame mean in
		 * scenes with strong dominant colours (sky, foliage, skin tones).
		 * highlightWeight_ = 0 gives pure grey-world.
		 *
		 * Dynamic weight: when the gain-undone scene mean has a large
		 * chromatic divergence from neutral (|R/G - 1| or |B/G - 1|
		 * exceeds 0.2), the scene has a dominant colour and grey-world
		 * is unreliable. In that case the highlight estimate — which
		 * samples pixels near the specular highlights where the
		 * illuminant colour is most visible — is a better reference.
		 * Increase the effective weight proportionally to divergence,
		 * capped at 0.8 to retain some grey-world stability.
		 */
		if (highlightWeight_ > 0.0f) {
			float hlGR = calculateHighlightMean(stats, kHistChannelGR);
			float hlR = calculateHighlightMean(stats, kHistChannelR);
			float hlGB = calculateHighlightMean(stats, kHistChannelGB);
			float hlB = calculateHighlightMean(stats, kHistChannelB);

			/*
			 * Chromaticity divergence: how far the full-frame
			 * mean deviates from a neutral R/G = B/G = 1.
			 * A scene with a strong blue sky has meanB >> meanG,
			 * giving bNorm >> 1 and large divergence. The factor
			 * 0.5 scales divergence into blend weight space so that
			 * a 0.4 divergence (e.g. 40% colour cast) adds 0.2 to
			 * the base highlightWeight_, and the cap of 0.8 ensures
			 * we always keep at least 20% grey-world contribution.
			 */
			float meanGpre = (meanGR + meanGB) / 2.0f;
			float rNorm = (meanGpre > 0.0f) ? meanR / meanGpre : 1.0f;
			float bNorm = (meanGpre > 0.0f) ? meanB / meanGpre : 1.0f;
			float divergence = std::max(std::abs(rNorm - 1.0f),
						    std::abs(bNorm - 1.0f));
			float effectiveWeight = std::clamp(
				highlightWeight_ + 0.5f * divergence,
				highlightWeight_, 0.8f);

			meanGR = (1.0f - effectiveWeight) * meanGR + effectiveWeight * hlGR;
			meanR = (1.0f - effectiveWeight) * meanR + effectiveWeight * hlR;
			meanGB = (1.0f - effectiveWeight) * meanGB + effectiveWeight * hlGB;
			meanB = (1.0f - effectiveWeight) * meanB + effectiveWeight * hlB;

			LOG(ISC_AWB, Debug)
				<< "AWB highlight blend: divergence=" << divergence
				<< " effectiveWeight=" << effectiveWeight;
		}

		/* avgGreen for Robertson CT estimation only */
		float meanG = (meanGR + meanGB) / 2.0f;

		/*
		 * Estimate colour temperature from gain-undone channel means
		 * using Robertson's (1968) 31-point Planckian locus method
		 * with the sensor-to-XYZ matrix from the tuning file. AWB is
		 * the single authority for CT; CCM reads activeState.awb.ct.
		 */
		rawCt = estimateColorTemperature(
			meanR, meanG, meanB,
			&context.configuration.sensor.ctColorMatrix);

		/*
		 * CT-based gain lookup: interpolate the calibrated R/B gains
		 * from reference_gains_table at the current estimated CT.
		 *
		 * This mirrors the RPi IPA (awb.cpp lines 157-161):
		 *   gainR = 1.0 / ctR.eval(temperatureK)
		 *   gainB = 1.0 / ctB.eval(temperatureK)
		 * and libipa/awb_grey.cpp gainsFromColourTemperature():
		 *   gains = colourGainCurve_->getInterpolated(colourTemperature)
		 *
		 * WHY NOT pure grey-world (GR/B ratio):
		 *   For a NoIR sensor under warm illumination (4450K), the B
		 *   channel has inherently low signal (B_net ≈ 10-25 bins at
		 *   the correct exposure).  The SAMA7G5 histogram tap is
		 *   pre-WB pre-demosaic (ISC_HIS_CFG_RAR), so the raw B mean
		 *   approaches the noise floor at reduced exposure settings.
		 *   Grey-world then computes bGain = GR/B ≈ 3-5× — far above
		 *   the calibrated 2.0× — creating an AE/AWB feedback loop:
		 *   AE reduces exposure → B drops → bGain spikes → output
		 *   stays bright → AE cannot reach the target luminance.
		 *
		 *   CT-based gains from the calibration table are computed at
		 *   a specific illuminant level and are STABLE regardless of
		 *   sensor exposure — they represent the physical R/B channel
		 *   imbalance for a given scene colour temperature, not the
		 *   momentary histogram ratio which is exposure-dependent.
		 *
		 * If no reference_gains_table is available, fall back to
		 * grey-world (consistent with libipa/awb_grey.cpp behaviour).
		 */
		float targetR, targetB;
		if (!refGainTable_.empty()) {
			/*
			 * Use rawCt (computed just above) rather than the
			 * smoothed awb.ct from the previous frame.  awb.ct is
			 * only updated after the gain computation below, so
			 * reading it here would lag the CT estimate by one
			 * frame, causing the colour matrix and the gains to
			 * track different colour temperatures during illuminant
			 * transitions.
			 */
			float ctForGains = (rawCt > 0.0f) ? rawCt : defaultCt_;
			auto [refR, refB] = interpolateRefGains(ctForGains);
			targetR = refR;
			targetB = refB;
		} else {
			/* Grey-world fallback when no calibration table */
			targetR = meanGR / std::max(meanR, 1.0f);
			targetB = meanGR / std::max(meanB, 1.0f);
		}

		float rGain = std::clamp(targetR, gainMin_, gainMax_);
		float bGain = std::clamp(targetB, gainMin_, gainMax_);
		float grGain = 1.0f;
		float gbGain = std::clamp(awb.gbGain * (meanGR / std::max(meanGB, 1.0f)),
					  gbGainMin_, gbGainMax_);

		if (frame > 0 && awb.rGain > 0.0f) {
			float rDelta = std::abs(rGain - awb.rGain) / awb.rGain;
			float bDelta = std::abs(bGain - awb.bGain) / awb.bGain;
			float gbDelta = std::abs(gbGain - awb.gbGain) / awb.gbGain;

			/*
			 * Hysteresis: enter stable below stabilityThreshold_;
			 * only exit when delta exceeds resumeThreshold_ (2x).
			 * Prevents colour flicker near the stability boundary.
			 */
			float maxDelta = std::max({ rDelta, bDelta, gbDelta });
			if (!awbStable_)
				awbStable_ = maxDelta < stabilityThreshold_;
			else if (maxDelta > resumeThreshold_)
				awbStable_ = false;

			if (awbStable_) {
				/* Within stability threshold: skip update */
				LOG(ISC_AWB, Debug) << "AWB stable, keeping gains";
			} else {
				/*
				 * Use startup_speed for the first startupFrames_
				 * AWB process() calls (awbFrameCount_, counted from
				 * the first stats callback, not the global frame
				 * number which starts at warmupFrames_).  This
				 * mirrors the RPi IPA approach: instant convergence
				 * during sensor startup so gains reach steady state
				 * before the stability lock check fires.  After that
				 * switch to the lower tracking speed for smooth
				 * temporal adaptation without flicker.
				 */
				/*
				 * smoothingFactor_ is the IIR "keep" factor:
				 *   update = 1 - smoothingFactor_
				 * So smoothingFactor_=0.7 → update=0.3 (30% step).
				 * startupSpeed_ is the convergence step size directly:
				 *   startupSpeed_=1.0 → update=1.0 (instant jump).
				 * Use update directly for startup, derive from
				 * smoothingFactor_ for steady-state tracking.
				 */
				float update;
				if (startupFrames_ > 0 &&
				    awbFrameCount_ <= startupFrames_)
					update = startupSpeed_;
				else
					update = 1.0f - smoothingFactor_;
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
				/*
			 * Do not touch the WB offset registers here.  On
			 * platforms without a DPC block (hasDpc=false) the
			 * BlackLevelCorrection algorithm sets rOffset/bOffset/
			 * grOffset/gbOffset to subtract the sensor pedestal via
			 * the WB stage.  Zeroing them here would re-introduce
			 * the pedestal on every AWB update frame, defeating BLC.
			 */

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
	}

	/*
	 * Update colour temperature with temporal smoothing.  Apply damping_
	 * in addition to smoothingFactor_ so CT and gains converge at the
	 * same rate — without it CT leads the gains, causing a transient CCM
	 * matrix mismatch during scene transitions.
	 */
	/*
	 * CT smoothing is only updated when the scene is bright enough to
	 * have produced a valid CT estimate.  When lumApplied < threshold,
	 * rawCt is 0.0f (never computed).  Running the IIR with rawCt=0
	 * would exponentially decay awb.ct toward ctMin_ over dark frames,
	 * poisoning the CCM matrix on the next bright-scene transition.
	 */
	if (lumApplied >= darkSceneThreshold_) {
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

	if (lumApplied < darkSceneThreshold_)
		LOG(ISC_AWB, Debug)
			<< "Scene too dark for AWB (lum=" << lumApplied
			<< " < threshold=" << darkSceneThreshold_
			<< "), CT frozen at " << awb.ct << " K";

	/* Publish convergence state for CCM to pace its own smoothing */
	awb.awbConverged = awbStable_;

	std::array<float, 2> gains = { awb.rGain, awb.bGain };
	metadata.set(controls::ColourGains, gains);
	metadata.set(controls::AwbLocked, awbStable_);

	if (awb.ct > 0)
		metadata.set(controls::ColourTemperature, static_cast<int32_t>(awb.ct));

	/*
	 * Emit pre-WB histogram channel means as vendor metadata.
	 *
	 * The ISC histogram tap (ISC_HIS_CFG_RAR) is before the WB block so
	 * these values are unaffected by the IPA gain registers. They represent
	 * the raw scene illuminant chromaticity and are the primary data source
	 * for the microchip_isc.py calibration tool.
	 *
	 * Exposing them here through the standard metadata path — rather than
	 * relying on debug log scraping — provides a stable, versioned interface
	 * that survives log format changes and log level suppression.
	 *
	 * Array order: [GR, R, GB, B] — matches ISC hardware channel indices
	 * ISC_HIS_CFG_MODE_GR=0, R=1, GB=2, B=3.
	 */
	const auto &histState = context.activeState.histogram;
	std::array<float, 4> histMeans = {
		histState.channelMeans[kHistChannelGR],
		histState.channelMeans[kHistChannelR],
		histState.channelMeans[kHistChannelGB],
		histState.channelMeans[kHistChannelB],
	};
	metadata.set(controls::microchip::IscHistogramMeans, histMeans);

	/*
	 * Emit full histogram bins only on the first frame after the IPA
	 * starts, and on every frame where the frameContext flags it as
	 * requested.  The payload is 8 KB per frame (2048 × int32); emitting
	 * it unconditionally imposes 240 KB/s of unnecessary IPC overhead in
	 * the normal capture path.
	 *
	 * TODO: gate on a per-request opt-in control once libcamera supports
	 * request-level vendor control forwarding to IPA process().
	 */
	if (frameContext.agc.exposure == 0 ||
	    context.activeState.awb.autoEnabled) {
		std::array<int32_t, kIscHistChannels * kHistBins> bins;
		const unsigned int channels[kIscHistChannels] = {
			kHistChannelGR, kHistChannelR, kHistChannelGB, kHistChannelB
		};
		for (unsigned int ch = 0; ch < kIscHistChannels; ch++) {
			const auto &raw = histState.bins[channels[ch]];
			for (unsigned int i = 0; i < kHistBins; i++)
				bins[ch * kHistBins + i] = static_cast<int32_t>(raw[i]);
		}
		metadata.set(controls::microchip::IscHistogramBins, bins);
	}
}

/**
 * \brief Interpolate R and B reference gains from the per-CT table
 * \param[in] ct Colour temperature in Kelvin
 * \return { r_gain, b_gain } linearly interpolated at ct
 *
 * Clamps to the table endpoints for CT values outside the calibrated range.
 */
std::pair<float, float> Awb::interpolateRefGains(float ct) const
{
	if (refGainTable_.empty())
		return { refRGain_, refBGain_ };

	/* Below lowest entry: clamp to first */
	if (ct <= refGainTable_.front().ct)
		return { refGainTable_.front().r, refGainTable_.front().b };

	/* Above highest entry: clamp to last */
	if (ct >= refGainTable_.back().ct)
		return { refGainTable_.back().r, refGainTable_.back().b };

	/* Find bracketing entries and linearly interpolate */
	for (size_t i = 1; i < refGainTable_.size(); ++i) {
		if (ct <= refGainTable_[i].ct) {
			const auto &lo = refGainTable_[i - 1];
			const auto &hi = refGainTable_[i];
			float t = (ct - lo.ct) / (hi.ct - lo.ct);
			return { lo.r + t * (hi.r - lo.r),
				 lo.b + t * (hi.b - lo.b) };
		}
	}

	return { refGainTable_.back().r, refGainTable_.back().b };
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
