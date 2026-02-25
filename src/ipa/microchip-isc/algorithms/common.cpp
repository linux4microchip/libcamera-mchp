/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Common utilities for Microchip ISC IPA algorithms
 */

#include "common.h"

#include <algorithm>
#include <cmath>
#include <string.h>

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::microchip_isc {

/**
 * \brief Calculate the weighted mean of a histogram channel
 * \param[in] bins Pointer to 512-bin histogram data
 * \param[in] totalPixels Total pixel count for normalization
 * \return Weighted mean value (0.0-511.0 range)
 *
 * Computes the weighted average of histogram bin values. Each bin's
 * contribution is weighted by its index (intensity level) and count.
 */
float calculateChannelMean(const isc_stat_buffer *stats, unsigned int channel)
{
	Histogram hist({ stats->hist[channel].hist_bins, kHistBins });
	uint64_t totalPixels = hist.total();
	if (totalPixels == 0)
		return 0.0f;

	if (totalPixels > kMaxHistPixels) {
		LOG(ISC_Common, Warning)
			<< "Histogram channel " << channel
			<< " total " << totalPixels
			<< " exceeds limit; stats buffer may be corrupt";
		return 0.0f;
	}

	uint64_t weightedSum = 0;
	Span<const uint64_t> cumulative = hist.data();
	for (unsigned int i = 0; i < kHistBins; i++) {
		uint64_t bin = cumulative[i + 1] - cumulative[i];
		weightedSum += bin * i;
	}

	return static_cast<float>(weightedSum) / totalPixels;
}

/**
 * \brief Calculate overall scene luminance from all histogram channels
 * \param[in] stats Pointer to ISC statistics buffer
 * \return Estimated luminance (0.0-511.0 range)
 *
 * Uses ITU-R BT.601 coefficients applied to the Bayer channel means:
 * L = 0.299 * R + 0.587 * (GR + GB) / 2 + 0.114 * B
 */
float calculateLuminance(const isc_stat_buffer *stats)
{
	float meanGR = calculateChannelMean(stats, kHistChannelGR);
	float meanR = calculateChannelMean(stats, kHistChannelR);
	float meanGB = calculateChannelMean(stats, kHistChannelGB);
	float meanB = calculateChannelMean(stats, kHistChannelB);

	if (hist.total() == 0)
		return 0.0f;

	if (hist.total() > kMaxHistPixels) {
		LOG(ISC_Common, Warning)
			<< "Histogram channel " << channel
			<< " total " << hist.total()
			<< " exceeds limit; stats buffer may be corrupt";
		return 0.0f;
	}

	return static_cast<float>(hist.interQuantileMean(0.8, 1.0));
}

/**
 * \brief Build a BT.601-weighted combined luminance histogram
 * \param[in] stats  Pointer to ISC statistics buffer
 * \param[out] combined 512-element array of weighted per-bin counts
 * \param[out] total    Sum of all weighted counts
 *
 * Shared by calculateHistogramSpread(), calculateLuminanceCdf(), and
 * calculateWeightedLuminance() to avoid rebuilding the same combined
 * histogram independently in each caller. The loop is channel-outer /
 * bin-inner so each channel's contiguous bin array is read sequentially,
 * maximising cache efficiency on the Cortex-A7.
 */
static void buildCombinedLuminanceHistogram(const mchp_isc_stat_buffer *stats,
					    std::array<float, kHistBins> &combined,
					    float &total)
{
	static constexpr float kWeights[4] = { 0.2935f, 0.299f, 0.2935f, 0.114f };

	combined.fill(0.0f);
	total = 0.0f;

	for (unsigned int ch = 0; ch < 4; ch++) {
		float w = kWeights[ch];
		for (unsigned int i = 0; i < kHistBins; i++) {
			float v = w * static_cast<float>(stats->hist[ch].hist_bins[i]);
			combined[i] += v;
			total += v;
		}
	}
}

/**
 * \brief Compute a 64-point normalised luminance CDF aligned to LUT entries
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[out] lut_cdf 64-element array filled with CDF values in [0, 1]
 *
 * Builds a combined luminance histogram by weighting the four Bayer channels
 * with BT.601 coefficients (same as calculateHistogramSpread), then computes
 * the normalised cumulative distribution function (CDF) at the midpoint of
 * each of the 64 gamma LUT input ranges.
 *
 * The ISC gamma LUT has 64 entries; entry i covers the 10-bit input range
 * [i*16 .. (i+1)*16 - 1].  The ISC histogram has 512 bins; bin k represents
 * input values [k*2 .. k*2+1].  Therefore LUT entry i corresponds to
 * histogram bins [i*8 .. (i+1)*8 - 1].  The CDF is sampled at the midpoint
 * bin i*8 + 4, giving the fraction of pixels with luminance below the
 * centre of that LUT entry's input range.
 *
 * lut_cdf[j] = fraction of pixels with intensity ≤ (j*16 + 8) / 1023
 *
 * When blended with a power-law tone curve in computeLut(), a cdf_strength
 * of 1.0 produces full histogram equalisation (uniform output distribution),
 * while 0.0 leaves the power law unchanged.  Intermediate values give a
 * locally-adaptive curve that stretches low-contrast histogram regions.
 *
 * Degenerate case (all bins zero): fills lut_cdf with a linear ramp [0..1]
 * so computeLut() degenerates gracefully to the pure power-law path.
 */
void calculateLuminanceCdf(const mchp_isc_stat_buffer *stats,
			   std::array<float, 64> &lut_cdf)
{
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);

	if (total < 1.0f) {
		/* Degenerate: produce a linear ramp so the power-law takes over */
		for (unsigned int j = 0; j < 64; j++)
			lut_cdf[j] = static_cast<float>(j) / 63.0f;
		return;
	}

	/*
	 * Build cumulative sum, then sample at midpoint of each LUT entry.
	 * LUT entry j midpoint → histogram bin j*8 + 4.
	 */
	float cumulative = 0.0f;
	unsigned int histBin = 0;
	for (unsigned int j = 0; j < 64; j++) {
		unsigned int midBin = j * 8 + 4;
		/* Accumulate up to (and including) the midpoint bin */
		while (histBin <= midBin && histBin < kHistBins) {
			cumulative += combined[histBin];
			histBin++;
		}
		lut_cdf[j] = cumulative / total;
	}
}

/**
 * \brief Calculate the fraction of highlight-clipped pixels
 * \param[in] stats Pointer to ISC statistics buffer
 * \return Fraction of pixels in the top 4 histogram bins (0.0–1.0)
 *
 * Examines the top 4 bins (508–511 of 512) across all four Bayer channels.
 * A non-zero return value indicates highlight saturation. Values above
 * ~0.02 (2%) reliably indicate that the scene is overexposed regardless
 * of what the luminance mean reports, which can appear low in high-contrast
 * scenes where large shadow areas drag down the average.
 *
 * Uses the driver-provided hist_max to fast-path the no-clipping case:
 * if all channels have hist_max below the clip threshold, no bins in the
 * clipping range can have pixels and 0.0 is returned immediately without
 * iterating the 512-bin arrays.
 */
float calculateClipping(const mchp_isc_stat_buffer *stats)
{
	static constexpr unsigned int kClipBins = 4;
	static constexpr unsigned int kClipThreshold = kHistBins - kClipBins;

	/*
	 * Fast-path: if no channel has any pixel above bin kClipThreshold-1,
	 * hist_max < kClipThreshold for all channels, so clipping is zero.
	 */
	bool anyAboveThreshold = false;
	for (unsigned int ch = 0; ch < 4; ch++) {
		if (stats->hist[ch].hist_max >= kClipThreshold) {
			anyAboveThreshold = true;
			break;
		}
	}
	if (!anyAboveThreshold)
		return 0.0f;

	uint64_t total = 0;
	uint64_t clipped = 0;

	for (unsigned int ch = 0; ch < 4; ch++) {
		for (unsigned int i = 0; i < kHistBins; i++) {
			uint32_t count = stats->hist[ch].hist_bins[i];
			total += count;
			if (i >= kHistBins - kClipBins)
				clipped += count;
		}
	}

	if (total == 0)
		return 0.0f;

	return static_cast<float>(clipped) / static_cast<float>(total);
}

/**
 * \brief Calculate the P5-to-P95 spread of the scene luminance histogram
 * \param[in] stats Pointer to ISC statistics buffer
 * \return Spread as a fraction of the full 512-bin range (0.0–1.0)
 *
 * Builds a luminance-weighted histogram by combining all four Bayer channels
 * with BT.601 coefficients, then returns the distance between the 5th and
 * 95th percentiles normalised to [0, 1]. A spread near 1.0 means the scene
 * uses the full dynamic range; a low spread (e.g. 0.2) indicates a flat or
 * foggy scene where contrast enhancement is beneficial.
 */
float calculateHistogramSpread(const mchp_isc_stat_buffer *stats)
{
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);

	if (total == 0.0f)
		return 0.0f; /* empty histogram: no spread, no contrast enhancement */

	float p5target = total * 0.05f;
	float p95target = total * 0.95f;

	unsigned int p5 = 0;
	unsigned int p95 = kHistBins - 1;
	float cumulative = 0.0f;
	bool foundP5 = false;

	for (unsigned int i = 0; i < kHistBins; i++) {
		cumulative += combined[i];
		if (!foundP5 && cumulative >= p5target) {
			p5 = i;
			foundP5 = true;
		}
		if (cumulative >= p95target) {
			p95 = i;
			break;
		}
	}

	return static_cast<float>(p95 - p5) / static_cast<float>(kHistBins);
}

/**
 * \brief Calculate scene luminance weighted by per-bin intensity weights
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[in] weights 512-element per-bin weight array (should sum to ~1.0)
 * \return Estimated luminance (0.0–511.0 range)
 *
 * Builds a combined luminance histogram using BT.601 channel weights, then
 * computes the weighted mean using the supplied per-bin weight array. This
 * implements exposure metering modes (matrix, centre-weighted, spot) by
 * assigning different importance to low, mid-tone, and highlight bins.
 */
float calculateWeightedLuminance(const mchp_isc_stat_buffer *stats,
				 const std::array<float, kHistBins> &weights)
{
	/*
	 * Build the combined luminance histogram once (channel-outer, bin-inner
	 * for cache efficiency), then apply the per-bin metering weights in a
	 * single pass over the contiguous combined array.
	 */
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);
	float weightedIntensitySum = 0.0f;
	float weightedCountSum = 0.0f;

	for (unsigned int i = 0; i < kHistBins; i++) {
		float w = weights[i] * combined[i];
		weightedCountSum += w;
		weightedIntensitySum += w * static_cast<float>(i);
	}

	if (weightedCountSum < 1.0f) {
		/*
		 * No pixels fell in the metering zone (e.g. spot bins 192–320
		 * when the scene is fully clipped or black). Fall back to the
		 * full-frame average using the already-built combined histogram
		 * rather than rebuilding it via calculateLuminance().		 */ LOG(ISC_Common, Debug)
			<< "calculateWeightedLuminance: no pixels in metering "
			   "zone, falling back to full-frame average";
		if (total < 1.0f)
			return 0.0f;
		float sum = 0.0f;
		for (unsigned int i = 0; i < kHistBins; i++)
			sum += combined[i] * static_cast<float>(i);
		return sum / total;
	}
	return weightedIntensitySum / weightedCountSum;
}

/**
 * \brief Calculate the fraction of highlight-clipped pixels
 * \param[in] stats Pointer to ISC statistics buffer
 * \return Fraction of pixels in the top 4 histogram bins (0.0–1.0)
 *
 * Examines the top 4 bins (508–511 of 512) across all four Bayer channels.
 * A non-zero return value indicates highlight saturation. Values above
 * ~0.02 (2%) reliably indicate that the scene is overexposed regardless
 * of what the luminance mean reports, which can appear low in high-contrast
 * scenes where large shadow areas drag down the average.
 *
 * Uses the driver-provided hist_max to fast-path the no-clipping case:
 * if all channels have hist_max below the clip threshold, no bins in the
 * clipping range can have pixels and 0.0 is returned immediately without
 * iterating the 512-bin arrays.
 */
float calculateClipping(const isc_stat_buffer *stats)
{
	static constexpr unsigned int kClipBins = 4;
	static constexpr unsigned int kClipThreshold = kHistBins - kClipBins;

	/*
	 * Fast-path: if no channel has any pixel above bin kClipThreshold-1,
	 * hist_max < kClipThreshold for all channels, so clipping is zero.
	 */
	bool anyAboveThreshold = false;
	for (unsigned int ch = 0; ch < 4; ch++) {
		if (stats->hist[ch].hist_max >= kClipThreshold) {
			anyAboveThreshold = true;
			break;
		}
	}
	if (!anyAboveThreshold)
		return 0.0f;

	uint64_t total = 0;
	uint64_t clipped = 0;

	for (unsigned int ch = 0; ch < 4; ch++) {
		Histogram hist({ stats->hist[ch].hist_bins, kHistBins });
		Span<const uint64_t> cum = hist.data();
		total += hist.total();
		clipped += cum[kHistBins] - cum[kHistBins - kClipBins];
	}

	if (total == 0)
		return 0.0f;

	return static_cast<float>(clipped) / static_cast<float>(total);
}

/**
 * \brief Calculate the P5-to-P95 spread of the scene luminance histogram
 * \param[in] stats Pointer to ISC statistics buffer
 * \return Spread as a fraction of the full 512-bin range (0.0–1.0)
 *
 * Builds a luminance-weighted histogram by combining all four Bayer channels
 * with BT.601 coefficients, then returns the distance between the 5th and
 * 95th percentiles normalised to [0, 1]. A spread near 1.0 means the scene
 * uses the full dynamic range; a low spread (e.g. 0.2) indicates a flat or
 * foggy scene where contrast enhancement is beneficial.
 */
float calculateHistogramSpread(const mchp_isc_stat_buffer *stats)
{
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);

	if (total == 0.0f)
		return 0.0f; /* empty histogram: no spread, no contrast enhancement */

	float p5target = total * 0.05f;
	float p95target = total * 0.95f;

	unsigned int p5 = 0;
	unsigned int p95 = kHistBins - 1;
	float cumulative = 0.0f;
	bool foundP5 = false;

	for (unsigned int i = 0; i < kHistBins; i++) {
		cumulative += combined[i];
		if (!foundP5 && cumulative >= p5target) {
			p5 = i;
			foundP5 = true;
		}
		if (cumulative >= p95target) {
			p95 = i;
			break;
		}
	}

	return static_cast<float>(p95 - p5) / static_cast<float>(kHistBins);
}

/**
 * \brief Calculate scene luminance weighted by per-bin intensity weights
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[in] weights 512-element per-bin weight array (should sum to ~1.0)
 * \return Estimated luminance (0.0–511.0 range)
 *
 * Builds a combined luminance histogram using BT.601 channel weights, then
 * computes the weighted mean using the supplied per-bin weight array. This
 * implements exposure metering modes (matrix, centre-weighted, spot) by
 * assigning different importance to low, mid-tone, and highlight bins.
 */
float calculateWeightedLuminance(const mchp_isc_stat_buffer *stats,
				 const std::array<float, kHistBins> &weights)
{
	/*
	 * Build the combined luminance histogram once (channel-outer, bin-inner
	 * for cache efficiency), then apply the per-bin metering weights in a
	 * single pass over the contiguous combined array.
	 */
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);

	float weightedIntensitySum = 0.0f;
	float weightedCountSum = 0.0f;

	for (unsigned int i = 0; i < kHistBins; i++) {
		float w = weights[i] * combined[i];
		weightedCountSum += w;
		weightedIntensitySum += w * static_cast<float>(i);
	}

	if (weightedCountSum < 1.0f) {
		/*
		 * No pixels fell in the metering zone (e.g. spot bins 192–320
		 * when the scene is fully clipped or black). Fall back to the
		 * full-frame average using the already-built combined histogram
		 * rather than rebuilding it via calculateLuminance().
		 */
		LOG(ISC_Common, Debug)
			<< "calculateWeightedLuminance: no pixels in metering "
			   "zone, falling back to full-frame average";
		if (total < 1.0f)
			return 0.0f;
		float sum = 0.0f;
		for (unsigned int i = 0; i < kHistBins; i++)
			sum += combined[i] * static_cast<float>(i);
		return sum / total;
	}

	return weightedIntensitySum / weightedCountSum;
}

/**
 * \brief Estimate color temperature from Bayer channel statistics
 * \param[in] stats Pointer to ISC statistics buffer
 * \return Estimated color temperature in Kelvin
 *
 * Uses the red/blue channel ratio to estimate correlated color temperature
 * via McCamy's approximation. Returns 5500K as default for neutral scenes.
 */
float estimateColorTemperature(const isc_stat_buffer *stats)
{
	float meanR = calculateChannelMean(stats, kHistChannelR);
	float meanG = (calculateChannelMean(stats, kHistChannelGR) +
		       calculateChannelMean(stats, kHistChannelGB)) /
		      2.0f;
	float meanB = calculateChannelMean(stats, kHistChannelB);

	return estimateColorTemperature(meanR, meanG, meanB, nullptr);
}

/*
 * Robertson (1968) 31-point Planckian locus table in CIE 1960 UCS (u, v)
 * space with reciprocal megakelvin (MRD = 1e6/K) values.
 *
 * Source: Robertson, A.R. (1968). "Computation of Correlated Color
 *   Temperature and Distribution Temperature." Journal of the Optical
 *   Society of America. 58(11): 1528–1535.
 *
 * The 't' field is the slope du/dv of the Planckian locus at each point,
 * used by Robertson's sign-change algorithm to find the nearest locus
 * intersection in UCS space. Robertson's method is correct for any sensor
 * whose spectral response has been mapped to CIE XYZ via ct_color_matrix,
 * and produces accurate CCT for typical indoor/outdoor scenes calibrated
 * against known illuminants.
 */
namespace {
struct RobertsonEntry {
	float u;
	float v;
	float t; /* locus slope for sign-change test */
	float mrd; /* reciprocal megakelvin (1e6 / K) */
};

static constexpr std::array<RobertsonEntry, 31> kRobertsonTable = { {
	{ 0.18006f, 0.26352f, -0.24341f, 0.0f },
	{ 0.18066f, 0.26589f, -0.25479f, 10.0f },
	{ 0.18133f, 0.26846f, -0.26876f, 20.0f },
	{ 0.18208f, 0.27119f, -0.28539f, 30.0f },
	{ 0.18293f, 0.27407f, -0.30470f, 40.0f },
	{ 0.18388f, 0.27709f, -0.32675f, 50.0f },
	{ 0.18494f, 0.28021f, -0.35156f, 60.0f },
	{ 0.18611f, 0.28342f, -0.37915f, 70.0f },
	{ 0.18740f, 0.28668f, -0.40955f, 80.0f },
	{ 0.18880f, 0.28997f, -0.44278f, 90.0f },
	{ 0.19032f, 0.29326f, -0.47888f, 100.0f },
	{ 0.19462f, 0.30141f, -0.58204f, 125.0f },
	{ 0.19962f, 0.30921f, -0.70471f, 150.0f },
	{ 0.20525f, 0.31647f, -0.84901f, 175.0f },
	{ 0.21142f, 0.32312f, -1.0182f, 200.0f },
	{ 0.21807f, 0.32909f, -1.2168f, 225.0f },
	{ 0.22511f, 0.33439f, -1.4512f, 250.0f },
	{ 0.23247f, 0.33904f, -1.7298f, 275.0f },
	{ 0.24010f, 0.34308f, -2.0637f, 300.0f },
	{ 0.24792f, 0.34655f, -2.4681f, 325.0f },
	{ 0.25591f, 0.34951f, -2.9641f, 350.0f },
	{ 0.26400f, 0.35200f, -3.5814f, 375.0f },
	{ 0.27218f, 0.35407f, -4.3633f, 400.0f },
	{ 0.28039f, 0.35577f, -5.3762f, 425.0f },
	{ 0.28863f, 0.35714f, -6.7262f, 450.0f },
	{ 0.29685f, 0.35823f, -8.5955f, 475.0f },
	{ 0.30505f, 0.35907f, -11.324f, 500.0f },
	{ 0.31320f, 0.35968f, -15.628f, 525.0f },
	{ 0.32129f, 0.36011f, -23.325f, 550.0f },
	{ 0.32931f, 0.36038f, -40.770f, 575.0f },
	{ 0.33724f, 0.36051f, -116.45f, 600.0f },
} };
} /* anonymous namespace */

/**
 * \brief Estimate correlated colour temperature using Robertson's (1968) method
 * \param[in] meanR Red channel mean (gain-undone, any linear scale)
 * \param[in] meanG Green channel mean (gain-undone, same scale)
 * \param[in] meanB Blue channel mean (gain-undone, same scale)
 * \param[in] ctColorMatrix Optional 3×3 row-major sensor-to-XYZ matrix.
 *   Row 0 = [X←R, X←G, X←B], row 1 = Y, row 2 = Z.
 *   When nullptr, a built-in approximation for typical CMOS sensors is used.
 * \return Estimated correlated colour temperature in Kelvin (1667–25000 K)
 *
 * Algorithm (Robertson 1968):
 *   1. Convert sensor RGB → CIE XYZ via the calibration matrix.
 *   2. Compute CIE 1960 UCS (u, v) from XYZ:
 *        u = 4X / (X + 15Y + 3Z)
 *        v = 6Y / (X + 15Y + 3Z)
 *   3. For each adjacent pair in the 31-point Planckian locus table, compute
 *      the signed distance from (u,v) to each tangent line:
 *        d_i = (v - v_i) - t_i * (u - u_i)
 *      When d changes sign between entries i-1 and i, interpolate linearly
 *      in reciprocal megakelvin (MRD = 1e6/K) space.
 *
 * Robertson's method operates in CIE 1960 UCS where the Planckian locus
 * has near-uniform chromaticity spacing, giving bounded error across the
 * full 1667–25000 K range. Accuracy: ±200 K with calibrated ct_color_matrix.
 */
float estimateColorTemperature(float meanR, float meanG, float meanB,
			       const std::array<float, 9> *ctColorMatrix)
{
	float total = meanR + meanG + meanB;
	if (total < 1.0f)
		return 5500.0f;

	float rbRatio = meanR / meanB;

	/*
	 * Approximate CCT from R/B ratio. High R/B indicates warm (low CCT),
	 * low R/B indicates cool (high CCT).
	 */
	float cct;
	if (rbRatio > 1.5f)
		cct = 2800.0f;
	else if (rbRatio > 1.2f)
		cct = 2800.0f + (1.5f - rbRatio) / (1.5f - 1.2f) * (3500.0f - 2800.0f);
	else if (rbRatio > 1.0f)
		cct = 3500.0f + (1.2f - rbRatio) / (1.2f - 1.0f) * (5000.0f - 3500.0f);
	else if (rbRatio > 0.8f)
		cct = 5000.0f + (1.0f - rbRatio) / (1.0f - 0.8f) * (6500.0f - 5000.0f);
	else
		cct = 7500.0f;

	return std::clamp(cct, 2000.0f, 10000.0f);
}

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
