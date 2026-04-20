# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# AWB reference gains calibration module.
# Derives reference_gains_table from grey-card captures at known CCTs.
#
# Data source:
#   Channel means come from IscHistogramMeans vendor metadata (pre-WB,
#   pre-DPC histogram tap via ISC_HIS_CFG_RAR), stored in the .json
#   sidecar as hist_means: {R, GR, GB, B}.  These are pedestal-subtracted
#   net histogram bin values (0-511 scale) — exactly what the IPA AWB
#   algorithm receives at runtime.
#
#   The ISC has no pre-WB DMA path, so calibration cannot use pixel data
#   from JPEG files (which are post-gamma, post-WB, 8-bit).
#
#   The IPA AWB formula (awb.cpp):
#     rGain = meanGR / meanR   (GR is the reference green channel)
#     bGain = meanGR / meanB
#
#   reference_gains_table stores the correct per-CT gains for neutral grey
#   scenes. Used as startup seeds so the first frames have approximately
#   correct white balance before the AWB loop converges.
#
#   Capture requirements:
#     - Neutral grey card or white wall filling the frame (>70% of image)
#     - SINGLE illuminant at the target CCT (no mixed light)
#     - IPA AWB+AGC running and converged (AwbLocked=true in sidecar)
#     - Multiple captures at each CT are averaged for robustness
#
#   Formula uses GR (not (GR+GB)/2) to exactly match IPA awb.cpp.

import logging
import numpy as np

logger = logging.getLogger(__name__)

# Minimum signal floor in net histogram bins. Matches MIN_SNR_BINS in
# recalc.py (8 bins) — the same threshold the IPA uses for dark-scene
# detection. Using a lower value would accept images the runtime rejects.
_MIN_SIGNAL_BINS = 8.0


def calibrate_awb_gains(images):
    """
    Calibrate reference_gains_table from grey-card captures.

    Each image must be a CalibCapture with:
      img.ct   : true CCT (K) from filename tag ct<K> or JSON true_ct
      img.hist : {'R', 'GR', 'GB', 'B'} pre-WB histogram means (0-511)
                 from IscHistogramMeans vendor metadata in the JSON sidecar

    Multiple images at the same CT are averaged before computing gains.
    This reduces noise and compensates for temporal variation in scene
    illumination (e.g. lightbox intensity drift).

    Formula (matches IPA awb.cpp exactly):
      rGain = meanGR / meanR
      bGain = meanGR / meanB

    Returns a sorted list of {'ct': K, 'r': gain_r, 'b': gain_b}
    or None if no valid images found.
    """
    # Collect per-CT measurements
    by_ct = {}  # ct_K -> list of {'R': mean, 'GR': mean, 'GB': mean, 'B': mean}

    for img in images:
        if img.ct is None:
            continue
        if img.hist is None:
            logger.warning(f'{img.name}: no histogram means in sidecar, skipping. '
                           f'Capture with calib.sh to get IscHistogramMeans metadata.')
            continue

        # Reject ColorChecker images — multi-patch scene mean is not neutral grey
        if img.name.lower().startswith('colorchecker') or \
           img.name.lower().startswith('macbeth'):
            logger.warning(f'{img.name}: ColorChecker/Macbeth image — '
                           f'skipping for AWB calibration. Use a neutral 18%% '
                           f'grey card under single-illuminant lighting.')
            continue

        r  = img.hist['R']
        gr = img.hist['GR']
        gb = img.hist['GB']
        b  = img.hist['B']

        if gr < _MIN_SIGNAL_BINS:
            logger.warning(f'{img.name}: GR mean {gr:.1f} < {_MIN_SIGNAL_BINS} bins '
                           f'(too dark for reliable AWB calibration), skipping')
            continue
        if r < _MIN_SIGNAL_BINS or b < _MIN_SIGNAL_BINS:
            logger.warning(f'{img.name}: R={r:.1f} or B={b:.1f} below signal floor, '
                           f'skipping')
            continue

        # Grey-card neutrality check: for a neutral grey scene under a single
        # illuminant, GR and GB should be approximately equal (both are green
        # channels with similar spectral response). A large GR/GB ratio
        # indicates either a non-neutral scene or per-channel BLC error.
        # This is a proxy for scene uniformity since we have only means,
        # not per-pixel std from the histogram.
        if gr > 0 and gb > 0:
            gr_gb_ratio = gr / gb
            if gr_gb_ratio > 1.15 or gr_gb_ratio < 0.87:
                logger.error(f'{img.name}: GR/GB={gr_gb_ratio:.3f} deviates >15%% '
                             f'from 1.0. Scene is not neutral grey or BLC is wrong. '
                             f'Use an 18%% grey card filling >70%% of the frame '
                             f'under a single illuminant. Skipping.')
                continue
            if gr_gb_ratio > 1.08 or gr_gb_ratio < 0.93:
                logger.warning(f'{img.name}: GR/GB={gr_gb_ratio:.3f} deviates >8%% '
                               f'from 1.0. Grey card may not fill enough of the '
                               f'frame, or scene has mixed illumination.')

        ct = int(round(img.ct))
        if ct in by_ct:
            # Check for distinct float values colliding to the same integer CT
            prev_cts = [img2.ct for img2 in images
                        if img2.ct is not None and int(round(img2.ct)) == ct
                        and img2.ct != img.ct]
            if prev_cts:
                logger.debug(f'  CT={img.ct}K rounds to {ct}K (same bucket as '
                             f'{prev_cts[0]:.1f}K) — merged into one CT point.')
        else:
            by_ct[ct] = []
        by_ct[ct].append({'R': r, 'GR': gr, 'GB': gb, 'B': b})
        logger.debug(f'  {img.name}: CT={ct}K  '
                     f'R={r:.2f} GR={gr:.2f} GB={gb:.2f} B={b:.2f}')

    if not by_ct:
        logger.error('No valid AWB gain images found. '
                     'Need images with ct<K> tag in filename and adequate exposure.')
        return None

    # Warn if some CT points had all images rejected.
    # Collect all CT values that were attempted (from all images with a ct tag).
    all_attempted_cts = set()
    for img in images:
        if img.ct is not None and \
           not img.name.lower().startswith(('colorchecker', 'macbeth')):
            all_attempted_cts.add(int(round(img.ct)))
    missed = all_attempted_cts - set(by_ct.keys())
    if missed:
        logger.warning(
            f'CT points {sorted(missed)} had no valid images after quality checks '
            f'(too dark, non-uniform, or insufficient signal). '
            f'Calibration will proceed with {len(by_ct)} CT point(s) only. '
            f'For best AWB accuracy across the full CCT range, re-capture at '
            f'the missing points with correct scene setup.'
        )

    results = []
    for ct in sorted(by_ct.keys()):
        entries = by_ct[ct]
        n = len(entries)

        # Average channel means across all captures at this CT
        avg_r  = float(np.mean([e['R']  for e in entries]))
        avg_gr = float(np.mean([e['GR'] for e in entries]))
        avg_gb = float(np.mean([e['GB'] for e in entries]))
        avg_b  = float(np.mean([e['B']  for e in entries]))

        # Gain from averaged means (ratio of averages — more robust to
        # outliers than average of ratios when n is small).
        r_gain  = round(avg_gr / avg_r,  4)
        b_gain  = round(avg_gr / avg_b,  4)
        gb_gain = round(avg_gr / avg_gb, 4)  # informational

        # Per-frame gain variation (std-dev of per-entry ratios).
        # Note: this describes frame-to-frame consistency, not the
        # uncertainty of r_gain itself (which is ratio of averages).
        r_gains = [e['GR'] / e['R'] for e in entries]
        b_gains = [e['GR'] / e['B'] for e in entries]
        r_std = float(np.std(r_gains)) if n > 1 else 0.0
        b_std = float(np.std(b_gains)) if n > 1 else 0.0

        logger.info(f'  CT={ct}K  n={n}  '
                    f'R={avg_r:.2f} GR={avg_gr:.2f} GB={avg_gb:.2f} B={avg_b:.2f}  '
                    f'rGain={r_gain:.4f}±{r_std:.4f}  '
                    f'bGain={b_gain:.4f}±{b_std:.4f}  '
                    f'gbGain={gb_gain:.4f}')

        # gbGain = GR/GB. For a neutral grey scene this should be ~1.0.
        # A large deviation (>10%) indicates a BLC per-channel imbalance —
        # the GR and GB pedestals differ, typically due to NIR leakage into one
        # green sub-channel, or a BLC run with the wrong lens cover.
        if abs(gb_gain - 1.0) > 0.15:
            logger.error(
                f'  CT={ct}K: gbGain={gb_gain:.4f} deviates >15%% from 1.0. '
                f'GR and GB have different means under neutral grey — strong '
                f'indicator of per-channel BLC error (NIR contamination via '
                f'lens cap, or black_level_gr != black_level_gb in YAML). '
                f'Redo BLC with opaque black tape and verify YAML per-channel BL values.')
        elif abs(gb_gain - 1.0) > 0.10:
            logger.warning(
                f'  CT={ct}K: gbGain={gb_gain:.4f} deviates >10%% from 1.0. '
                f'Check per-channel BLC values in YAML.')

        if r_std > 0.05 or b_std > 0.05:
            logger.error(f'  CT={ct}K: unacceptable gain variance '
                         f'(r_std={r_std:.4f}, b_std={b_std:.4f} > 0.05). '
                         f'This indicates mixed illumination, scene content bias, '
                         f'or AGC not fully converged. Re-capture under a single '
                         f'illuminant with the grey card filling >70%% of frame '
                         f'and AWB/AE both locked before saving.')
        elif r_std > 0.02 or b_std > 0.02:
            logger.warning(f'  CT={ct}K: moderate gain variance '
                           f'(r_std={r_std:.4f}, b_std={b_std:.4f} > 0.02). '
                           f'Increase captures per CT point for better averaging. '
                           f'Check single-illuminant lighting and scene uniformity.')

        results.append({'ct': ct, 'r': r_gain, 'b': b_gain})

    return results
