/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Microchip Technology Inc. All rights reserved.
 *
 * Common utilities for Microchip ISC IPA algorithms
 */

#include "common.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string.h>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(ISC_Common)

namespace ipa::microchip_isc {

/**
 * \brief Calculate the weighted mean of a histogram channel
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[in] channel Bayer channel index (kHistChannelGR/R/GB/B)
 * \return Weighted mean bin value (0.0–511.0 range)
 */
float calculateChannelMean(const mchp_isc_stat_buffer *stats, unsigned int channel)
{
	if (channel >= 4)
		return 0.0f;

	uint64_t total = 0;
	uint64_t weightedSum = 0;

	for (unsigned int i = 0; i < kHistBins; i++) {
		uint32_t count = stats->hist[channel].hist_bins[i];
		total += count;
		weightedSum += static_cast<uint64_t>(count) * i;
	}

	if (total == 0 || total > kMaxHistPixels)
		return 0.0f;

	return static_cast<float>(weightedSum) / static_cast<float>(total);
}

/**
 * \brief Calculate the mean intensity of the top 20% brightest pixels
 * \param[in] stats Pointer to ISC statistics buffer
 * \param[in] channel Bayer channel index
 * \param[in] pedestalBins Number of pedestal bins to skip (blackLevel/2)
 * \return Mean intensity of the top 20% pixels (0.0–511.0 range)
 */
float calculateHighlightMean(const mchp_isc_stat_buffer *stats,
			     unsigned int channel,
			     unsigned int pedestalBins)
{
	if (channel >= 4)
		return 0.0f;

	uint64_t total = 0;
	for (unsigned int i = pedestalBins; i < kHistBins; i++)
		total += stats->hist[channel].hist_bins[i];

	if (total == 0 || total > kMaxHistPixels)
		return 0.0f;

	/* Find P80 bin threshold */
	uint64_t p80target = total * 80 / 100;
	uint64_t cumulative = 0;
	unsigned int p80bin = kHistBins - 1;
	for (unsigned int i = pedestalBins; i < kHistBins; i++) {
		cumulative += stats->hist[channel].hist_bins[i];
		if (cumulative >= p80target) {
			p80bin = i;
			break;
		}
	}

	/* Mean of bins from P80 to P100 */
	uint64_t hlTotal = 0;
	uint64_t hlSum = 0;
	for (unsigned int i = p80bin; i < kHistBins; i++) {
		uint32_t count = stats->hist[channel].hist_bins[i];
		hlTotal += count;
		hlSum += static_cast<uint64_t>(count) * i;
	}

	if (hlTotal == 0)
		return 0.0f;

	return static_cast<float>(hlSum) / static_cast<float>(hlTotal);
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
			   std::array<float, 64> &lut_cdf,
			   [[maybe_unused]] unsigned int pedestalBins)
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
float calculateHistogramSpread(const mchp_isc_stat_buffer *stats,
			       [[maybe_unused]] unsigned int pedestalBins)
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
 * \brief Return the histogram bin at a given percentile of the luminance CDF
 */
float calculateHistogramPercentile(const mchp_isc_stat_buffer *stats, float pct)
{
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);

	if (total == 0.0f)
		return 0.0f;

	float target = total * std::clamp(pct, 0.0f, 1.0f);
	float cumulative = 0.0f;
	for (unsigned int i = 0; i < kHistBins; i++) {
		cumulative += combined[i];
		if (cumulative >= target)
			return static_cast<float>(i);
	}
	return static_cast<float>(kHistBins - 1);
}

/**
 * \brief Compute the CDF bin index at a given quantile of the histogram
 * \param[in] combined  Combined BT.601 luminance histogram (512 bins)
 * \param[in] total     Total pixel count
 * \param[in] quantile  Quantile in [0.0, 1.0]
 * \return Fractional bin index corresponding to that quantile
 *
 * Mirrors libipa Histogram::quantile() (BSD-2-Clause, Raspberry Pi Ltd).
 */
static float histQuantile(const std::array<float, kHistBins> &combined,
			  float total, float quantile)
{
	float target = quantile * total;
	float cumulative = 0.0f;
	for (unsigned int i = 0; i < kHistBins; i++) {
		float prev = cumulative;
		cumulative += combined[i];
		if (cumulative >= target) {
			/* Linear interpolation within the bin */
			float frac = combined[i] > 0.0f
					     ? (target - prev) / combined[i]
					     : 0.0f;
			return static_cast<float>(i) + frac;
		}
	}
	return static_cast<float>(kHistBins - 1);
}

/**
 * \brief Compute the mean of histogram bins between two fractional bin indices
 * \param[in] combined  BT.601 luminance histogram
 * \param[in] binLo     Lower fractional bin index
 * \param[in] binHi     Upper fractional bin index
 * \return Weighted mean bin value in [binLo, binHi]
 *
 * Mirrors libipa Histogram::interBinMean() (BSD-2-Clause, Raspberry Pi Ltd).
 */
static float histInterBinMean(const std::array<float, kHistBins> &combined,
			      float binLo, float binHi)
{
	float sumBinFreq = 0.0f, cumulFreq = 0.0f;
	for (float binNext = std::floor(binLo) + 1.0f;
	     binNext <= std::ceil(binHi);
	     binLo = binNext, binNext += 1.0f) {
		int bin = static_cast<int>(std::floor(binLo));
		if (bin < 0 || bin >= static_cast<int>(kHistBins))
			break;
		float weight = std::min(binNext, binHi) - binLo;
		float freq = combined[bin] * weight;
		sumBinFreq += static_cast<float>(bin) * freq;
		cumulFreq += freq;
	}
	return cumulFreq > 0.0f ? (sumBinFreq / cumulFreq + 0.5f)
				: binHi;
}

/**
 * \brief Inter-quantile mean of the combined BT.601 luminance histogram
 * \param[in] stats   ISC statistics buffer
 * \param[in] qLo     Lower quantile [0.0, 1.0]  e.g. 0.02
 * \param[in] qHi     Upper quantile [0.0, 1.0]  e.g. 0.98
 * \return Mean luminance bin of the pixels between qLo and qHi
 *
 * Returns the mean of the middle (qHi-qLo) fraction of the luminance
 * distribution, ignoring the darkest qLo fraction and the brightest
 * (1-qHi) fraction.  This is the core metering primitive used by the
 * Raspberry Pi and libipa AGC implementations to achieve robust AE
 * that is insensitive to small bright objects or dark corners.
 *
 * iqm(0.02, 0.98) ignores the darkest 2% and brightest 2% of pixels,
 * targeting exposure at the genuine midtone content of the scene.
 * iqm(0.98, 1.0)  measures only the brightest 2%: used by the
 * highlight-constraint mode to clamp gain and prevent clipping.
 */
float calculateInterQuantileMean(const mchp_isc_stat_buffer *stats,
				 float qLo, float qHi)
{
	std::array<float, kHistBins> combined;
	float total;
	buildCombinedLuminanceHistogram(stats, combined, total);
	if (total == 0.0f)
		return 0.0f;

	float pLo = histQuantile(combined, total, qLo);
	float pHi = histQuantile(combined, total, qHi);
	if (pHi <= pLo)
		return pLo;

	return histInterBinMean(combined, pLo, pHi);
}

/**
 * \brief Build highlight-avoidance metering weights for the luminance histogram
 * \param[out] weights   512-element weight array to fill
 * \param[in]  strength  0.0=flat/uniform  1.0=full highlight-avoidance
 *
 * Exponentially-decaying weights that give full weight to shadow/mid-tone
 * bins and progressively less weight to bright bins — mirrors mobile-phone
 * highlight-priority AE:
 *   bin 0   (black):  weight=1.0
 *   bin 256 (50%):    weight~0.47  (strength=1.0)
 *   bin 460 (90%):    weight~0.05
 *
 * Result: AGC targets exposure so mid-tones are correct; highlights may
 * clip slightly but shadows/mid-tones are never washed out.
 */
void buildHighlightAvoidanceWeights(std::array<float, kHistBins> &weights,
				    float strength)
{
	for (unsigned int i = 0; i < kHistBins; i++) {
		float n = static_cast<float>(i) / static_cast<float>(kHistBins);
		weights[i] = std::exp(-strength * 3.5f * n * n);
	}
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

	const std::array<float, 9> &m = ctColorMatrix ? *ctColorMatrix : kDefaultCtMatrix;

	/* Convert sensor RGB → CIE XYZ */
	float X = m[0] * meanR + m[1] * meanG + m[2] * meanB;
	float Y = m[3] * meanR + m[4] * meanG + m[5] * meanB;
	float Z = m[6] * meanR + m[7] * meanG + m[8] * meanB;

	/*
	 * CIE 1960 UCS (u, v) chromaticity from XYZ.
	 * This space has more uniform spacing along the Planckian locus than
	 * CIE 1931 (x, y), which is why Robertson's method uses it.
	 */
	float denom = X + 15.0f * Y + 3.0f * Z;
	if (denom < 0.001f)
		return 5500.0f;

	float u = 4.0f * X / denom;
	float v = 6.0f * Y / denom;

	/*
	 * Robertson's sign-change algorithm: compute the signed distance
	 * from (u,v) to the tangent line at each locus entry. The CCT lies
	 * between the two entries where the sign changes.
	 */
	float lastD = 0.0f;
	for (size_t i = 0; i < kRobertsonTable.size(); i++) {
		const auto &e = kRobertsonTable[i];
		float d = (v - e.v) - e.t * (u - e.u);

		if (i > 0 && d * lastD <= 0.0f) {
			/* Sign change: interpolate MRD between entries i-1 and i */
			float f = lastD / (lastD - d);
			float mrd = kRobertsonTable[i - 1].mrd +
				    f * (e.mrd - kRobertsonTable[i - 1].mrd);
			if (mrd < 1.0f)
				return 25000.0f;
			return std::clamp(1.0e6f / mrd, 1667.0f, 25000.0f);
		}
		lastD = d;
	}

	/* No sign change found: query point outside table range */
	return 5500.0f;
}

/**
 * \brief Convenience overload: estimate CT directly from stats buffer means
 */
float estimateColorTemperature(const mchp_isc_stat_buffer *stats)
{
	float meanR = calculateChannelMean(stats, kHistChannelR);
	float meanG = (calculateChannelMean(stats, kHistChannelGR) +
		       calculateChannelMean(stats, kHistChannelGB)) /
		      2.0f;
	float meanB = calculateChannelMean(stats, kHistChannelB);
	return estimateColorTemperature(meanR, meanG, meanB, nullptr);
}

/**
 * \brief Estimate scene illuminance in lux
 *
 * Implements the Raspberry Pi lux estimator formula (lux.cpp):
 *   lux = (refExposure/curExposure) * (refGain/curGain) * (curY/refY) * refLux
 */
float estimateLux(float currentY,
		  uint32_t exposureLines,
		  float analogueGain,
		  float lineDurationUs,
		  float refY,
		  float refExposureUs,
		  float refGain,
		  float refLux)
{
	if (refY < 1.0f || currentY < 1.0f || exposureLines == 0 ||
	    analogueGain < 0.001f || lineDurationUs < 0.001f)
		return 400.0f;

	float curExposureUs = static_cast<float>(exposureLines) * lineDurationUs;
	if (curExposureUs < 0.001f)
		return 400.0f;

	float lux = refLux *
		    (refExposureUs / curExposureUs) *
		    (refGain / analogueGain) *
		    (currentY / refY);

	return std::clamp(lux, 1.0f, 100000.0f);
}

} /* namespace ipa::microchip_isc */

} /* namespace libcamera */
