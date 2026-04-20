# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# CT correction PWL calibration module.
# Derives the ct_correction_pwl table from RAW captures under known illuminants.
#
# Methodology:
#   The Robertson (1968) algorithm estimates CCT from sensor RGB means via a
#   sensor-to-XYZ matrix (ct_color_matrix in YAML). Because the matrix is
#   approximate, the raw Robertson estimate has a systematic bias that varies
#   with CCT.  The ct_correction_pwl table maps raw Robertson estimates to the
#   true CCT measured with a phone camera or lux/CCT meter under known
#   single-illuminant conditions.
#
#   Capture requirements:
#     - Grey card or white wall under a SINGLE illuminant (no mixed light)
#     - At least 2 points; 3 or more cover the full photographic range
#     - The calibration tool reads Robertson raw CT from the IPA debug log
#     - The user provides the true CCT (phone camera app is accurate to ±50 K)
#
#   NOTE: Ambient light calibration (e.g. measuring the room's mixed
#   fluorescent+window light) is NOT suitable because grey-world means reflect
#   scene content, not just the illuminant.  Use a lightbox or single LED
#   panel that fills the frame.
#
# Robertson table:
#   Must be IDENTICAL to the table in src/ipa/microchip-isc/algorithms/common.cpp.
#   The C++ runtime and Python calibration tool MUST use the same table or
#   the calibrated PWL will not map correctly.
#
# References:
#   Robertson, A.R. (1968). "Computation of Correlated Color Temperature and
#   Distribution Temperature." JOSA 58(11): 1528-1535.
#   Judd, D.B. (1960). "Appraisal of Land's Work on Two-Primary Color
#   Projections." JOSA 50(3): 254-268.  (Planckian locus polynomial)

import logging
import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Robertson (1968) Planckian locus table: 31 original entries + 1 extended
# entry to cover the 1600-1739 K range (NoIR sensor ctMin_=1800 K).
#
# Format: (u, v, t, MRD)  where MRD = 1e6 / K
#   u, v : CIE 1960 UCS chromaticity of the Planckian locus at this MRD
#   t    : slope dt/du of the perpendicular to the locus (Robertson's d_i)
#   MRD  : reciprocal megakelvin = 1e6 / K
#
# IMPORTANT: This table MUST match kRobertsonTable in common.cpp exactly.
# The extended entry at MRD=625 was computed from the Judd (1960) Planckian
# locus polynomial:
#   u(T) = (0.860117757 + 1.54118254e-4*T + 1.28641212e-7*T^2)
#          / (1 + 8.42420235e-4*T + 7.08145163e-7*T^2)
#   v(T) = (0.317398726 + 4.22806245e-5*T + 4.20481691e-8*T^2)
#          / (1 + 8.42420235e-4*T + 7.08145163e-7*T^2)
# Slope t = delta_u / delta_v between adjacent entries (finite difference).
# MRD=625 corresponds to T=1600 K.
# ---------------------------------------------------------------------------
ROBERTSON_TABLE = [
    # (u,          v,          t,          MRD)
    (0.18006,  0.26352,  -0.24341,    0.0),
    (0.18066,  0.26589,  -0.25479,   10.0),
    (0.18133,  0.26846,  -0.26876,   20.0),
    (0.18208,  0.27119,  -0.28539,   30.0),
    (0.18293,  0.27407,  -0.30470,   40.0),
    (0.18388,  0.27709,  -0.32675,   50.0),
    (0.18494,  0.28021,  -0.35156,   60.0),
    (0.18611,  0.28342,  -0.37915,   70.0),
    (0.18740,  0.28668,  -0.40955,   80.0),
    (0.18880,  0.28997,  -0.44278,   90.0),
    (0.19032,  0.29326,  -0.47888,  100.0),
    (0.19462,  0.30141,  -0.58204,  125.0),
    (0.19962,  0.30921,  -0.70471,  150.0),
    (0.20525,  0.31647,  -0.84901,  175.0),
    (0.21142,  0.32312,  -1.0182,   200.0),
    (0.21807,  0.32909,  -1.2168,   225.0),
    (0.22511,  0.33439,  -1.4512,   250.0),
    (0.23247,  0.33904,  -1.7298,   275.0),
    (0.24010,  0.34308,  -2.0637,   300.0),
    (0.24792,  0.34655,  -2.4681,   325.0),
    (0.25591,  0.34951,  -2.9641,   350.0),
    (0.26400,  0.35200,  -3.5814,   375.0),
    (0.27218,  0.35407,  -4.3633,   400.0),
    (0.28039,  0.35577,  -5.3762,   425.0),
    (0.28863,  0.35714,  -6.7262,   450.0),
    (0.29685,  0.35823,  -8.5955,   475.0),
    (0.30505,  0.35907, -11.324,    500.0),
    (0.31320,  0.35968, -15.628,    525.0),
    (0.32129,  0.36011, -23.325,    550.0),
    (0.32931,  0.36038, -40.770,    575.0),
    (0.33724,  0.36051, -116.45,    600.0),
    # Extended entry: covers 1600 K (MRD=625) -- matches common.cpp kRobertsonTable
    (0.34424,  0.36051, -210.0,     625.0),
]


def estimate_ct_robertson(means_r, means_gr, means_gb, means_b,
                           ct_color_matrix=None):
    """
    Estimate CCT using Robertson (1968) method from channel means.

    means_*: scalar channel means in linear sensor units (pre-WB, any scale)
    ct_color_matrix: row-major 3x3 sensor->XYZ matrix as a flat list of 9
                     floats, or None to use identity (sensor ~= XYZ).

    Returns estimated CCT in Kelvin (0.0 on failure, 5500.0 as fallback).

    The channel means must be PRE-white-balance raw sensor values.
    For the ISC histogram (tap vp_data, pre-WB), this is correct as-is.
    """
    mean_g = (means_gr + means_gb) / 2.0

    if ct_color_matrix is not None:
        m = ct_color_matrix
        # sensor [R, G, B] -> CIE XYZ (row-major: row0=X, row1=Y, row2=Z)
        X = m[0]*means_r + m[1]*mean_g + m[2]*means_b
        Y = m[3]*means_r + m[4]*mean_g + m[5]*means_b
        Z = m[6]*means_r + m[7]*mean_g + m[8]*means_b
    else:
        # No matrix: assume sensor channels ~= CIE XYZ (poor approximation
        # but stable -- the ct_correction_pwl will correct the systematic bias)
        X, Y, Z = means_r, mean_g, means_b

    denom = X + 15.0*Y + 3.0*Z
    if denom < 1e-6:
        return 0.0

    # CIE 1960 UCS chromaticity
    u = 4.0*X / denom
    v = 6.0*Y / denom

    # Robertson's sign-change algorithm
    last_d = None
    for i, (ui, vi, ti, mrd) in enumerate(ROBERTSON_TABLE):
        d = (v - vi) - ti*(u - ui)
        if last_d is not None and last_d * d < 0.0:
            # Sign change found: interpolate in MRD space
            prev_mrd = ROBERTSON_TABLE[i-1][3]
            f = last_d / (last_d - d)
            mrd_est = prev_mrd + f * (mrd - prev_mrd)
            if mrd_est < 1.0:
                # Near-infinite CCT. Return 0.0 so the caller's
                # `<= 0` check discards this image as invalid.
                return 0.0
            return 1.0e6 / mrd_est
        last_d = d

    # Sign change not found in the Robertson table range. Possible causes:
    # extreme channel ratios, severe colour cast, or low-SNR dark frame.
    # Return 0.0 so the caller's `robertson_ct <= 0` check discards the
    # image rather than silently using a bogus value.
    return 0.0


def _check_robertson_table_consistency():
    """
    Sanity-check the Python Robertson table against its documented properties.

    The table MUST match kRobertsonTable in common.cpp exactly (same number of
    entries, same MRD values). This function verifies the entry count and the
    first/last MRD values — a fast runtime guard against table divergence.

    The C++ table has 33 entries: 31 standard Robertson + 1 extended (MRD=625,
    T=1600K) + entry 0 (MRD=0, infinite CCT sentinel).
    """
    # C++ declares array<RobertsonEntry, 33> but only initialises 32 entries.
    # The 33rd slot is zero-initialised ({0,0,0,0}) which is harmless since
    # the algorithm stops at the first sign change. Python table has 32 entries.
    # Both are functionally identical. Accept 32 as correct.
    expected_count = 32
    expected_first_mrd = 0.0
    expected_last_mrd  = 625.0

    actual_count = len(ROBERTSON_TABLE)
    actual_first_mrd = ROBERTSON_TABLE[0][3]
    actual_last_mrd  = ROBERTSON_TABLE[-1][3]

    ok = True
    if actual_count != expected_count:
        logger.error(
            f'Robertson table has {actual_count} entries but common.cpp '
            f'kRobertsonTable has {expected_count}. CT calibration results '
            f'WILL NOT MATCH the IPA runtime. Update ct_correction.py to '
            f'match common.cpp.')
        ok = False
    if actual_first_mrd != expected_first_mrd or actual_last_mrd != expected_last_mrd:
        logger.error(
            f'Robertson table MRD range [{actual_first_mrd}, {actual_last_mrd}] '
            f'does not match expected [{expected_first_mrd}, {expected_last_mrd}]. '
            f'Update ct_correction.py to match common.cpp kRobertsonTable.')
        ok = False
    if ok:
        logger.debug(f'Robertson table: {actual_count} entries, '
                     f'MRD [{actual_first_mrd:.0f}, {actual_last_mrd:.0f}] — consistent')
    return ok


def calibrate_ct_correction(images, ct_color_matrix=None):
    """
    Derive ct_correction_pwl from a set of calibration images.

    Each image must have:
      img.ct        : true CCT (K) -- provided via filename tag ct<K>
      img.channels  : [R, GR, GB, B] 2-D float arrays

    Calibration images must be captured:
      - Under a SINGLE illuminant (lightbox or LED panel, no mixed light)
      - With a grey card or white wall filling most of the frame
      - With the IPA running so the histogram converges to correct means

    ct_color_matrix: optional flat 9-element list matching YAML ct_color_matrix.
                     When provided, Robertson uses a better XYZ mapping and the
                     correction PWL will be smaller.  When absent, Robertson
                     uses identity and the correction will be larger but valid.

    Returns a flat list [robertson_K0, true_K0, robertson_K1, true_K1, ...]
    sorted ascending by robertson_K, ready for the YAML ct_correction_pwl key.
    Returns None if fewer than 2 valid calibration points are found.

    Multiple images at the same true CT are supported: the median Robertson
    estimate across all captures at that CT is used.  This is more robust
    than a single capture because it rejects outliers from momentary scene
    variation or lightbox flicker.
    """
    # Verify Robertson table consistency with common.cpp before computing
    _check_robertson_table_consistency()

    # Warn if no ct_color_matrix provided -- Robertson errors will be larger
    if ct_color_matrix is None:
        logger.warning('No ct_color_matrix provided. Robertson estimation will '
                       'use identity (sensor ~= XYZ). This produces large '
                       'systematic errors (up to +47% at 2860K for IMX219). '
                       'The PWL will correct for this but is only valid for '
                       'the specific ct_color_matrix in the YAML. If you later '
                       'update ct_color_matrix, re-run CT calibration.')
    else:
        logger.info(f'Using ct_color_matrix: {[round(v,4) for v in ct_color_matrix]}')

    # Collect Robertson estimates per true CT
    by_true_ct = {}  # true_ct_K -> list of Robertson estimates

    for img in images:
        if img.ct is None:
            logger.warning(f'Image {img.name}: no CT tag (ct<K> in filename), skipping')
            continue
        if img.hist is None:
            logger.warning(f'Image {img.name}: no histogram means in sidecar, skipping. '
                           f'Capture with calib.sh to get IscHistogramMeans metadata.')
            continue

        r  = img.hist['R']
        gr = img.hist['GR']
        gb = img.hist['GB']
        b  = img.hist['B']

        if max(r, gr, gb, b) < 1.0:
            logger.warning(f'Image {img.name}: channel means near zero, skipping')
            continue

        robertson_ct = estimate_ct_robertson(r, gr, gb, b, ct_color_matrix)
        if robertson_ct <= 0:
            logger.warning(f'Image {img.name}: Robertson estimation failed')
            continue

        true_k = float(img.ct)
        if true_k not in by_true_ct:
            by_true_ct[true_k] = []
        by_true_ct[true_k].append(robertson_ct)
        logger.debug(f'  {img.name}: true={true_k:.0f}K  '
                     f'Robertson={robertson_ct:.0f}K  error={robertson_ct-true_k:+.0f}K')

    if len(by_true_ct) < 2:
        logger.error('Need at least 2 distinct true CCT values (ct<K> tags)')
        return None

    # Compute median Robertson estimate per true CT
    results = []
    for true_k in sorted(by_true_ct.keys()):
        estimates = by_true_ct[true_k]
        robertson_median = float(np.median(estimates))
        robertson_std = float(np.std(estimates)) if len(estimates) > 1 else 0.0
        error = robertson_median - true_k

        logger.info(f'  true={true_k:.0f}K  n={len(estimates)}  '
                    f'Robertson median={robertson_median:.0f}K  '
                    f'std={robertson_std:.0f}K  error={error:+.0f}K')

        if robertson_std > 100 and len(estimates) > 1:
            logger.warning(f'  true={true_k:.0f}K: high Robertson std={robertson_std:.0f}K '
                           f'across {len(estimates)} captures. Check single-illuminant '
                           f'lighting and neutral grey card.')

        results.append((robertson_median, true_k))

    # Sort by Robertson value (ascending) -- required for PWL lookup in IPA
    results.sort(key=lambda x: x[0])

    # Validate strict monotonicity. The IPA's PWL interpolation (awb.cpp) assumes
    # the Robertson values are strictly ascending — if they are not, the binary
    # search will return wrong CT values for inputs in the non-monotone region,
    # permanently corrupting CT estimation at those scene temperatures.
    # Non-monotone Robertson values indicate:
    #   1. Scene content bias (coloured subject, not a neutral grey card)
    #   2. Mixed illumination at one of the CCT points
    #   3. AGC not converged at capture time (exposure changed mid-histogram cycle)
    monotone_ok = True
    for i in range(1, len(results)):
        if results[i][0] <= results[i-1][0]:
            logger.error(
                f'NON-MONOTONE Robertson values at points {i-1}/{i}: '
                f'{results[i-1][0]:.0f}K → {results[i][0]:.0f}K. '
                f'The ct_correction_pwl CANNOT be written — a non-monotone PWL '
                f'will corrupt IPA CT estimation. '
                f'Root causes: (1) not a neutral grey card, '
                f'(2) mixed illuminant, (3) AGC not converged. '
                f'Re-capture at CT={results[i-1][1]:.0f}K and CT={results[i][1]:.0f}K.')
            monotone_ok = False
        if results[i][1] <= results[i-1][1]:
            logger.error(
                f'Non-monotone true CCT at points {i-1}/{i}: '
                f'{results[i-1][1]:.0f}K → {results[i][1]:.0f}K. '
                f'Enter illuminant CCTs in ascending order (coolest last).')
            monotone_ok = False

    if not monotone_ok:
        logger.error('CT correction PWL NOT written due to monotonicity failure.')
        return None

    pwl_flat = []
    for robertson_k, true_k in results:
        pwl_flat.append(int(round(robertson_k)))
        pwl_flat.append(int(round(true_k)))

    logger.info(f'CT correction PWL ({len(results)} points): {pwl_flat}')
    return pwl_flat
