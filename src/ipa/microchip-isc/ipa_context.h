/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Microchip ISC IPA Context
 */

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <stdint.h>

#include <libcamera/base/utils.h>

#include <libcamera/controls.h>

#include <libipa/camera_sensor_helper.h>
#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::microchip_isc {

struct IPASessionConfiguration {
	struct {
		utils::Duration minShutterSpeed;
		utils::Duration maxShutterSpeed;
		uint32_t defaultExposure;
		float minAnalogueGain;
		float maxAnalogueGain;
		/*
		 * Lux estimator calibration.
		 * Measure once: point camera at a scene with known lux,
		 * wait for AGC lock, record exposure/gain/histogramY and
		 * measure with a handheld lux meter. Store in imx219.yaml.
		 * refLuxY == 0 disables the lux estimator (default: 400 lux).
		 */
		float refLuxY = 0.0f; /* histogram BT.601 Y at reference; 0 = uncalibrated */
		float refLuxExposureUs = 0.0f; /* exposure time in µs at reference point */
		float refLuxGain = 1.0f; /* analogue gain at reference point */
		float refLux = 400.0f; /* measured lux at reference point */
		float luxSmoothingAlpha = 0.3f; /* IIR alpha for lux estimate smoothing */
		float highlightP85Threshold = 0.55f; /* P85 highlight metering threshold */
		float highlightP85Strength = 1.0f; /* P85 correction strength */
		float evBias = 0.0f; /* YAML default EV bias, -0.5 = mobile-like */
	} agc;

	struct {
		utils::Duration lineDuration;
		/* Pixel rate in Hz — needed to recompute lineDuration from live hblank */
		double pixelRate;
		/*
		 * Active output height in lines and the sensor VBLANK range,
		 * captured at configure() time. Used to convert a requested
		 * frame duration (controls::FrameDurationLimits) into a sensor
		 * VBLANK value so a locked frame rate can be applied without the
		 * AGC exposure loop having to own timing. vblankMax == vblankMin
		 * (both 0) disables the conversion.
		 */
		uint32_t outputHeight;
		int32_t vblankMin;
		int32_t vblankMax;
		/*
		 * Sensor black level in 10-bit units (e.g. 64 for IMX219).
		 * Per-channel values fall back to blackLevel when absent in YAML.
		 * On SAMA7G5 (hasDpc=true) these are not applied to WB offsets
		 * because the DPC hardware block subtracts BL before the WB block.
		 */
		uint32_t blackLevel;
		uint32_t blackLevelR;
		uint32_t blackLevelGR;
		uint32_t blackLevelGB;
		uint32_t blackLevelB;
		/*
		 * 3×3 row-major sensor-to-XYZ colour matrix used for
		 * Robertson (1968) CCT estimation. Row 0 = [X from R,
		 * X from G, X from B], row 1 = Y, row 2 = Z. Normalised
		 * to D65 white. Loaded from YAML Awb.ct_color_matrix; falls
		 * back to approximate IMX219 values when absent.
		 */
		std::array<float, 9> ctColorMatrix;
	} sensor;

	struct {
		/*
		 * hasDpc: SAMA7G5 DPC subtracts black level before the WB
		 * block. Setting WB offsets for BL correction on top would
		 * double-apply the subtraction and corrupt colours.
		 */
		bool hasDpc;

		/*
		 * hasBipartiteGamma: SAMA7G5 GAM block supports a linear toe
		 * segment for better shadow gradation. When true, computeLut()
		 * uses a piecewise linear-toe + power-law curve.
		 */
		bool hasBipartiteGamma;

		/*
		 * hasCbhs: SAMA7G5 CBHS block adds Saturation control
		 * (V4L2_CID_SATURATION) beyond the basic brightness/contrast
		 * available on SAMA5D2 CBC.
		 */
		bool hasCbhs;

		/*
		 * statsAvailable: ISC histogram statistics are only valid for
		 * RAW (Bayer) sensor input. False for sensors that output
		 * YUYV or RGB directly.
		 */
		bool statsAvailable;

		/*
		 * contrastUnityHw: hardware register value that represents
		 * unity contrast (output = input, no gain applied).
		 * SAMA7G5 CBHS: 16  (output = (contrast/16) * input)
		 * SAMA5D2 CBC:  256 (output = (contrast/256) * input)
		 * Set from hasCbhs at configure() time.
		 */
		float contrastUnityHw;
	} hw;
};

struct IPAActiveState {
	struct {
		uint32_t exposure = 0;
		float analogueGain = 1.0f;
		bool autoEnabled = true;
		int32_t exposureTimeMode = 0; /* 0=auto, 1=manual (independent of gain) */
		int32_t analogueGainMode = 0; /* 0=auto, 1=manual (independent of exposure) */
		/*
		 * EV compensation in stops [-2, +2]. Positive values increase
		 * target brightness; negative values decrease it. Applied as a
		 * multiplier (2^ev) to the YAML target_brightness each frame.
		 */
		float exposureValue = 0.0f;
		/* Zone metering scale: <1 reduces exposure for bright scenes. */
		float zoneRatio = 1.0f;
		/*
		 * Manual frame-duration request in microseconds (0 = free-run
		 * at the sensor default). Set from controls::FrameDurationLimits
		 * and converted to a sensor VBLANK at emit time, so locking the
		 * frame rate does not require the AGC exposure loop to own timing.
		 */
		int64_t frameDurationUs = 0;
		/*
		 * Manual digital gain multiplier (1.0 = unity / sensor default).
		 * Set from controls::DigitalGain and emitted as
		 * V4L2_CID_DIGITAL_GAIN. Analogue gain is preferred by the AGC
		 * loop; this is an explicit override for extra range.
		 */
		float digitalGain = 1.0f;
	} agc;

	struct {
		float rGain;
		float bGain;
		float grGain;
		float gbGain;
		int32_t rOffset;
		int32_t bOffset;
		int32_t grOffset;
		int32_t gbOffset;
		bool autoEnabled;
		/*
		 * Estimated scene colour temperature in Kelvin, computed by AWB
		 * from gain-undone channel means. Single source of truth shared
		 * with CCM so both modules track the same illuminant estimate.
		 */
		unsigned int ct;
		/*
		 * True once AWB has declared the gains stable (per-channel
		 * delta below stabilityThreshold_). Read by CCM to match its
		 * convergence speed: when false CCM uses a faster smoothing
		 * factor so the colour matrix tracks the changing CT estimate
		 * in sync with the gains, preventing a mismatch where new AWB
		 * gains are applied with an old CCM matrix.
		 */
		bool awbConverged;
	} awb;

	struct {
		std::array<float, 9> matrix;
		std::array<float, 3> offsets;
		/*
		 * Estimated saturation boost produced by the current CCM matrix,
		 * expressed as a multiplier relative to unity (1.0 = no boost).
		 * Computed by CCM from the off-diagonal magnitude: a matrix with
		 * strong cross-channel corrections amplifies colour saturation as
		 * a side effect. CBC reads this to apply a compensating reduction
		 * to the CBHS saturation register so that the combined pipeline
		 * saturation (CCM × CBHS) stays close to the user-intent value.
		 *
		 * Approximation: satBoost ≈ 1 + k * Σ|off_diagonals| / trace
		 * where k = 0.25 (empirically calibrated on IMX219).
		 * Initialised to 1.0 (no boost) until the first CCM update.
		 */
		float satBoost = 1.0f;
	} ccm;

	struct {
		/*
		 * Hardware brightness offset: -1024 to 1023, neutral = 0.
		 * Applied as: output = input + brightness.
		 */
		int32_t brightness;
		/*
		 * Hardware contrast multiplier: -2048 to 2047, neutral = 16.
		 * Applied as: output = (contrast / 16) * input + brightness.
		 */
		int32_t contrast;
		/*
		 * Hardware saturation: 0–100, unity = 16 on SAMA7G5 CBHS.
		 * Only applied on SAMA7G5 (hasCbhs = true) via V4L2_CID_SATURATION.
		 */
		int32_t saturation;
		/*
		 * Hardware hue rotation in degrees: -180 to +180, neutral = 0.
		 * Only applied on SAMA7G5 (hasCbhs = true) via V4L2_CID_HUE.
		 */
		int32_t hue;
		/*
		 * User-requested contrast multiplier from controls::Contrast.
		 * Applied on top of the adaptive histogram contrast gain.
		 * 1.0 = no user adjustment. Initialised to 1.0.
		 */
		float userContrastMultiplier = 1.0f;
		/*
		 * Effective contrast multiplier in linear gain units (1.0 = unity).
		 * Set by CBC each frame from the blended histogram-derived gain.
		 * Read by AGC to correct its effective target: when CBC will apply
		 * contrast boost, the AGC must target a proportionally lower
		 * pre-contrast luminance so the final output stays at the
		 * photographic reference level after the full pipeline.
		 * Initialised to 1.0 (no boost).
		 */
		float contrastGainLinear;
	} cbc;

	struct {
		/*
		 * Estimated scene illuminance in lux. Computed each stats
		 * cycle from the current exposure, analogue gain and histogram
		 * mean Y relative to a calibrated reference point.
		 * Default 400 lux — typical indoor office lighting.
		 * Used by the AGC y_target PWL to adapt the brightness target
		 * to the scene light level.
		 */
		float lux;
	} lux;

	struct {
		/*
		 * Per-channel histogram means from the most recent stats delivery.
		 * The ISC cycles GR→R→GB→B, but isc_stats_isr() fires only after
		 * the B channel completes, delivering all 4 channels together with
		 * valid_channels = 0x0f.  All four means are therefore always fresh
		 * at delivery time and updated together in processStats().
		 * Indexed by kHistChannelGR/R/GB/B constants from common.h.
		 */
		std::array<float, 4> channelMeans;
		/*
		 * Full 512-bin histogram counts per channel (GR, R, GB, B).
		 * Copied from the stats buffer in processStats() for use by
		 * calibration metadata (IscHistogramBins) and AGC CDF queries.
		 * Indexed [channel][bin], channel order matches kHistChannel*.
		 */
		std::array<std::array<uint32_t, 512>, 4> bins = {};
		/*
		 * Frame sequence number at which each channel's histogram was
		 * measured (snapshotted in the kernel at HISDONE IRQ time).
		 * Because each Bayer channel is measured on a different frame
		 * within the 4-frame cycle, the four capture_frame values differ
		 * by 0–3.  AWB uses these to look up the exposure and gains that
		 * were active when each channel was measured, enabling per-channel
		 * exposure normalization before gain computation.
		 */
		std::array<uint32_t, 4> captureFrame;
	} histogram;

	struct {
		/*
		 * Effective gamma exponent used to generate the LUT each frame.
		 * Temporally smoothed to avoid flickering.
		 */
		float gammaEff;
		/*
		 * 64-entry 10-bit LUT for each channel (B, G, R).
		 * Element i is the output value for input range [i*16..(i+1)*16-1].
		 */
		std::array<uint32_t, 64> lutB;
		std::array<uint32_t, 64> lutG;
		std::array<uint32_t, 64> lutR;
		/*
		 * True once Gamma::configure() has computed a valid LUT.
		 * setIspControls() skips LUT emission until this is set to
		 * prevent sending an all-zero table on the first frame.
		 */
		bool lutValid;
	} gamma;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		float analogueGain;
		bool autoEnabled;
		int32_t exposureTimeMode; /* 0=auto, 1=manual */
		int32_t analogueGainMode; /* 0=auto, 1=manual */
	} agc;

	struct {
		bool autoEnabled;
	} awb;

	struct {
		/*
		 * Manual colour-correction matrix from
		 * controls::ColourCorrectionMatrix. When manualSet is true the
		 * CCM algorithm bypasses CT interpolation and loads this matrix
		 * directly, so the override applies for one frame only and does
		 * not leak into subsequent auto frames.
		 */
		std::array<float, 9> matrix;
		bool manualSet;
	} ccm;

	/*
	 * Per-frame scene analysis: computed once in processStats() before
	 * the algorithm loop and shared by all algorithms to avoid redundant
	 * histogram passes. Algorithms must read these values rather than
	 * calling calculateHistogramSpread() / calculateClipping() directly.
	 *
	 * luminance:      BT.601-weighted mean from the pre-WB histogram
	 *                 (histogram units 0–511). The ISC histogram taps
	 *                 vp_data (pre-DPC, pre-WB), so channel means are
	 *                 raw sensor values independent of AWB gains. Used
	 *                 by AGC, CBC, and Gamma.
	 * spread:         P5–P95 inter-percentile range / 512 (0–1).
	 * clipping:       fraction of pixels in the top 4 bins (0–1).
	 */
	struct {
		float luminance;
		float spread;
		float clipping;
	} scene;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;

	/*
	 * Sensor helper — provides sensor-specific parameters compiled from
	 * datasheets (black level, gain formula). Mirrors RkISP1 IPAContext
	 * which also carries camHelper. Owned by the IPA instance; this is
	 * a non-owning pointer valid for the IPA lifetime.
	 * May be nullptr for sensors not registered in camera_sensor_helper.cpp.
	 */
	const ipa::CameraSensorHelper *camHelper = nullptr;
};

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
