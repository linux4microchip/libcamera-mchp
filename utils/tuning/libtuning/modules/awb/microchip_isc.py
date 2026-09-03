# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Microchip Technology Inc.
#
# AWB tuning module for the Microchip ISC.
#
# Emits the IPA's Awb.reference_gains_table (rGain = meanGR/meanR,
# bGain = meanGR/meanB) from the located Macbeth ColorChecker's neutral grey
# ramp (patches 3,7,11,15,19,23). The same raw Bayer captures drive the CCM
# solve, so the white balance and the colour matrix describe one identical
# illuminant -- self-consistent by construction, with no separate grey-card
# capture to shoot under matched ambient.
#
# Processed RGB/JPEG captures are post-WB and post-gamma, so they are never used
# for AWB; only the raw Bayer neutral patches feed the gains. When no located
# Macbeth chart is present the module falls back to the authored awb config.

import logging

import numpy as np

import libtuning as lt
from .awb import AWB

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
# locus polynomial. MRD=625 corresponds to T=1600 K.
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
    first/last MRD values -- a fast runtime guard against table divergence.
    """
    # C++ declares array<RobertsonEntry, 33> but only initialises 32 entries.
    # The 33rd slot is zero-initialised ({0,0,0,0}) which is harmless since
    # the algorithm stops at the first sign change. Python table has 32 entries.
    # Both are functionally identical. Accept 32 as correct.
    expected_count = 32
    expected_first_mrd = 0.0
    expected_last_mrd = 625.0

    actual_count = len(ROBERTSON_TABLE)
    actual_first_mrd = ROBERTSON_TABLE[0][3]
    actual_last_mrd = ROBERTSON_TABLE[-1][3]

    ok = True
    if actual_count != expected_count:
        logger.error(
            f'Robertson table has {actual_count} entries but common.cpp '
            f'kRobertsonTable has {expected_count}. CT calibration results '
            f'WILL NOT MATCH the IPA runtime. Update microchip_isc.py to '
            f'match common.cpp.')
        ok = False
    if actual_first_mrd != expected_first_mrd or actual_last_mrd != expected_last_mrd:
        logger.error(
            f'Robertson table MRD range [{actual_first_mrd}, {actual_last_mrd}] '
            f'does not match expected [{expected_first_mrd}, {expected_last_mrd}]. '
            f'Update microchip_isc.py to match common.cpp kRobertsonTable.')
        ok = False
    if ok:
        logger.debug(f'Robertson table: {actual_count} entries, '
                     f'MRD [{actual_first_mrd:.0f}, {actual_last_mrd:.0f}] -- consistent')
    return ok


def calibrate_ct_correction(images, ct_color_matrix=None):
    """
    Robertson-vs-true CT accuracy diagnostic from the located Macbeth captures.

    For each capture carrying a ct<K> tag, estimate CCT with Robertson (1968)
    from the neutral-ramp channel means (the same means used for the AWB gains)
    and compare to the tagged truth. The IPA consumes ct_color_matrix directly
    (not a correction PWL), so this reports how accurately that matrix places
    each illuminant -- it emits no YAML key.

    images: located raw Bayer captures (Image.patches set), each tagged ct<K>.
    ct_color_matrix: optional flat 9-element list matching YAML ct_color_matrix.
                     When provided, Robertson uses the real sensor->XYZ mapping
                     and the reported errors shrink accordingly.

    Returns a flat list [robertson_K0, true_K0, robertson_K1, true_K1, ...]
    sorted ascending by robertson_K, or None if fewer than 2 valid points.
    """
    # Verify Robertson table consistency with common.cpp before computing
    _check_robertson_table_consistency()

    if ct_color_matrix is None:
        logger.warning('No ct_color_matrix provided. Robertson estimation will '
                       'use identity (sensor ~= XYZ). This produces large '
                       'systematic errors (up to +47% at 2860K for IMX219).')
    else:
        logger.info(f'Using ct_color_matrix: {[round(v, 4) for v in ct_color_matrix]}')

    # Collect Robertson estimates per true CT
    by_true_ct = {}  # true_ct_K -> list of Robertson estimates

    for img in images:
        ct_k = _image_ct(img)
        if ct_k is None:
            logger.warning(f'{img.name}: no CT tag (ct<K> in filename), skipping')
            continue

        means = _macbeth_grey_channel_means(img)
        if means is None:
            logger.warning(f'{img.name}: no usable Macbeth neutral patches '
                           f'(all clipped or below black), skipping')
            continue

        r, gr, gb, b = means['R'], means['GR'], means['GB'], means['B']
        if max(r, gr, gb, b) < 1.0:
            logger.warning(f'{img.name}: channel means near zero, skipping')
            continue

        robertson_ct = estimate_ct_robertson(r, gr, gb, b, ct_color_matrix)
        if robertson_ct <= 0:
            logger.warning(f'{img.name}: Robertson estimation failed')
            continue

        true_k = float(ct_k)
        by_true_ct.setdefault(true_k, []).append(robertson_ct)
        logger.debug(f'  {img.name}: true={true_k:.0f}K  '
                     f'Robertson={robertson_ct:.0f}K  error={robertson_ct-true_k:+.0f}K')

    if len(by_true_ct) < 2:
        logger.info('CT accuracy diagnostic needs >= 2 distinct ct<K> tags; '
                    'skipping (calibration itself is unaffected).')
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

    # Sort by Robertson value (ascending)
    results.sort(key=lambda x: x[0])

    pwl_flat = []
    for robertson_k, true_k in results:
        pwl_flat.append(int(round(robertson_k)))
        pwl_flat.append(int(round(true_k)))

    logger.info(f'CT correction diagnostic ({len(results)} points): {pwl_flat}')
    return pwl_flat


# CCT (K) for a located Macbeth capture. A raw libtuning Image carries it in
# .color, with -1 meaning "no ct<K> filename tag". Returns None when untagged.
def _image_ct(image):
    ct = getattr(image, 'ct', None)
    if ct is None:
        ct = getattr(image, 'color', None)
    return None if ct in (None, -1) else ct


# Neutral (greyscale) patch indices of the located Macbeth chart. get_patches()
# documents the bottom row as white=3, then 7, 11, 15, 19, black=23 (columns
# stacked left-to-right). These six are spectrally neutral, so their per-channel
# ratios give the illuminant white balance directly.
_MACBETH_GREY_PATCHES = (3, 7, 11, 15, 19, 23)


def _macbeth_grey_channel_means(image):
    """Summed {R, GR, GB, B} means of the Macbeth neutral ramp, black removed.

    image.patches is [channel][patch][sorted_pixels] (channel order R, GR, GB,
    B), filled by Image.get_patches() once the chart is located. A patch is used
    only when every channel has signal above the pedestal and none is clipped;
    the surviving patches' channel means are summed, so the brightest usable
    neutral (best SNR) dominates without the white patch clipping the result.
    Returns None when no patch qualifies.
    """
    patches = getattr(image, 'patches', None)
    if patches is None:
        return None

    bl = getattr(image, 'blacklevel_16', 0) or 0
    sigbits = getattr(image, 'sigbits', 16)
    white = ((1 << sigbits) - 1) << (16 - sigbits)

    sums = {'R': 0.0, 'GR': 0.0, 'GB': 0.0, 'B': 0.0}
    used = 0
    for p in _MACBETH_GREY_PATCHES:
        chan_means = {}
        keep = True
        for ci, name in enumerate(('R', 'GR', 'GB', 'B')):
            px = patches[ci][p].astype(np.float64)
            # Patch pixels are sorted ascending; the top pixel flags saturation.
            if px[-1] >= white:
                keep = False
                break
            m = float(px.mean()) - bl
            if m <= 0.0:
                keep = False
                break
            chan_means[name] = m
        if not keep:
            continue
        for name in sums:
            sums[name] += chan_means[name]
        used += 1

    return sums if used else None


def macbeth_grey_captures(images):
    """Raw captures with a located Macbeth chart and a CT tag.

    locate_macbeth() (utils.load_images) sets Image.patches only for raw images
    whose chart was found, so a set patches attribute means the neutral ramp is
    available. Untagged charts are skipped -- a gain point needs its CT.
    """
    return [img for img in images
            if getattr(img, 'patches', None) is not None
            and _image_ct(img) is not None]


# Reference ColorChecker sRGB (8-bit, D65 viewing) in the standard chart order
# (dark skin ... black). Img.patches are in the located chart order, which the
# calibration flow aligns to this table's [i::6] interleave, so the same
# reorder is applied before pairing a patch with its reference colour.
_MACBETH_SRGB8 = np.array([
    [116, 81, 67], [199, 147, 129], [91, 122, 156], [90, 108, 64],
    [130, 128, 176], [92, 190, 172], [224, 124, 47], [68, 91, 170],
    [198, 82, 97], [94, 58, 106], [159, 189, 63], [230, 162, 39],
    [35, 63, 147], [67, 149, 74], [180, 49, 57], [238, 198, 20],
    [193, 84, 151], [0, 136, 170], [245, 245, 243], [200, 202, 202],
    [161, 163, 163], [121, 121, 122], [82, 84, 86], [49, 49, 51],
], dtype=np.float64)

# sRGB (linear, D65) -> CIE XYZ. IEC 61966-2-1 primaries; row-major X/Y/Z.
_SRGB_D65_TO_XYZ = np.array([
    [0.4124, 0.3576, 0.1805],
    [0.2126, 0.7152, 0.0722],
    [0.0193, 0.1192, 0.9505],
], dtype=np.float64)


def _srgb8_to_linear(x8):
    """Standard sRGB EOTF: 8-bit encoded sRGB -> linear 0..1."""
    x = x8 / 255.0
    return np.where(x < 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def _reference_xyz_located_order():
    """The 24 reference patch XYZ (D65), reordered to the located chart order."""
    srgb = np.array([_MACBETH_SRGB8[i::6] for i in range(6)]).reshape((24, 3))
    return _srgb8_to_linear(srgb) @ _SRGB_D65_TO_XYZ.T


def _macbeth_sensor_rgb(image):
    """Per-patch black-subtracted linear sensor [R, G, B] for all 24 patches.

    image.patches is [channel][patch][sorted_pixels] (channel order R, GR, GB,
    B). Returns (rgb[24,3], valid[24]) with valid[p] False where any channel of
    patch p is clipped (top sorted pixel at/above white) or sits at/below the
    black pedestal, or None when the chart was not located.
    """
    patches = getattr(image, 'patches', None)
    if patches is None:
        return None

    bl = getattr(image, 'blacklevel_16', 0) or 0
    sigbits = getattr(image, 'sigbits', 16)
    white = ((1 << sigbits) - 1) << (16 - sigbits)

    rgb = np.zeros((24, 3), dtype=np.float64)
    valid = np.ones(24, dtype=bool)
    for p in range(24):
        chan = []
        for ci in range(4):
            px = patches[ci][p].astype(np.float64)
            if px[-1] >= white:
                valid[p] = False
            chan.append(float(px.mean()) - bl)
        r, g, b = chan[0], (chan[1] + chan[2]) / 2.0, chan[3]
        if min(r, g, b) <= 0.0:
            valid[p] = False
        rgb[p] = (r, g, b)
    return rgb, valid


def solve_ct_color_matrix(images):
    """Solve the sensor->XYZ ct_color_matrix from a located Macbeth capture.

    The IPA feeds pre-WB raw channel means through this row-major 3x3 matrix to
    form CIE XYZ, then derives the CIE 1960 (u, v) chromaticity for Robertson
    CCT estimation (awb.cpp / common.cpp). Only chromaticity matters there: u
    and v are ratios (u=4X/D, v=6Y/D), so a single uniform scale of the whole
    matrix cancels. The matrix is therefore fit by unconstrained least squares
    mapping the 24 raw patch RGB to their reference XYZ, then scaled once so the
    Y row sums to 1 for readable numbers.

    This is deliberately NOT a per-row-sum-to-1 solve: normalising each row
    independently rescales the three rows by different factors, which does
    change (X, Y, Z) chromaticity and pulls neutral toward equal-energy white
    (~5455 K) instead of the true illuminant -- a systematic CCT bias of order
    1000 K. A single overall scale is the only normalisation that leaves the
    chromaticity the runtime reads untouched.

    The matrix is a broadband daylight characterisation of the sensor, so the
    capture whose CT tag is closest to D65 (6500 K) conditions it best; untagged
    captures are treated as daylight. Returns (matrix[9], calibration_ct, n_used)
    or None when no chart is located or too few patches survive.
    """
    caps = [img for img in images if getattr(img, 'patches', None) is not None]
    if not caps:
        return None

    def d65_distance(img):
        ct = _image_ct(img)
        return abs((ct if ct else 6500) - 6500)

    img = min(caps, key=d65_distance)
    extracted = _macbeth_sensor_rgb(img)
    if extracted is None:
        return None
    rgb, valid = extracted

    ref_xyz = _reference_xyz_located_order()
    sensor = rgb[valid]
    target = ref_xyz[valid]
    # Need more than the 3 unknowns per row to over-determine the 3x3 fit;
    # require a healthy margin so a few clipped patches cannot skew it.
    if sensor.shape[0] < 6:
        logger.warning(f'{img.name}: only {sensor.shape[0]} usable Macbeth '
                       f'patches; too few to solve ct_color_matrix')
        return None

    # lstsq solves sensor @ X = target, so X (3x3) is the transpose of the
    # row-major sensor->XYZ matrix the runtime applies.
    solution, *_ = np.linalg.lstsq(sensor, target, rcond=None)
    matrix = solution.T

    y_sum = float(matrix[1].sum())
    if abs(y_sum) < 1e-9:
        logger.warning(f'{img.name}: degenerate ct_color_matrix solve (Y row '
                       f'sums to ~0); keeping authored matrix')
        return None
    matrix = matrix / y_sum

    flat = [round(float(v), 4) for v in matrix.flatten()]
    assert len(flat) == 9, f'ct_color_matrix must be 9 floats, got {len(flat)}'
    return flat, _image_ct(img), int(sensor.shape[0])


def calibrate_awb_gains_macbeth(images):
    """reference_gains_table from the Macbeth neutral grey ramp, with per-CT
    inter-frame variance.

    rGain = GR/R, bGain = GR/B (matches the IPA awb.cpp formula), with the
    channel means taken from the ColorChecker's neutral patches in each located
    raw Bayer capture, black level removed. The multiple frames captured at one
    CT (mchp_capture.sh --frames) are averaged for the emitted gain, and the
    spread of their per-frame gains bounds how repeatable that point is.

    Returns (table, variance) where:
      table    : sorted list of {'ct', 'r', 'b'} (the emitted reference gains),
      variance : sorted list of {'ct', 'n', 'r_std', 'b_std', 'r_rel_std',
                 'b_rel_std'} -- the inter-frame gain spread per CT.
    Returns (None, None) when no capture yields a usable neutral ramp.
    """
    by_ct = {}
    for img in images:
        ct_k = _image_ct(img)
        if ct_k is None:
            continue
        means = _macbeth_grey_channel_means(img)
        if means is None:
            logger.warning(f'{img.name}: no usable Macbeth neutral patches '
                           f'(all clipped or below black), skipping')
            continue
        r, gr, gb, b = means['R'], means['GR'], means['GB'], means['B']
        gb_gain = gr / gb if gb > 0 else 0.0
        if abs(gb_gain - 1.0) > 0.15:
            logger.error(f'{img.name}: GR/GB={gb_gain:.3f} deviates >15%% from '
                         f'1.0 across the neutral ramp -- non-neutral lighting '
                         f'or BLC wrong, skipping')
            continue
        ct = int(round(ct_k))
        by_ct.setdefault(ct, []).append((r, gr, gb, b))
        logger.debug(f'  {img.name}: CT={ct}K  R={r:.1f} GR={gr:.1f} '
                     f'GB={gb:.1f} B={b:.1f}')

    if not by_ct:
        logger.error('No valid Macbeth AWB captures (need located ColorChecker '
                     'charts with a ct<K> tag and an unclipped neutral ramp).')
        return None, None

    results = []
    variance = []
    for ct in sorted(by_ct):
        e = by_ct[ct]
        avg_r = float(np.mean([x[0] for x in e]))
        avg_gr = float(np.mean([x[1] for x in e]))
        avg_b = float(np.mean([x[3] for x in e]))
        # Emitted gain: ratio of the averaged channel means (best SNR, and
        # byte-identical to the single-frame result when n == 1).
        r_gain = round(avg_gr / avg_r, 4)
        b_gain = round(avg_gr / avg_b, 4)

        # Inter-frame spread: the per-frame gain of each individual frame, so
        # the std reflects capture repeatability rather than the ratio-of-means.
        per_r = np.array([x[1] / x[0] for x in e if x[0] > 0], dtype=np.float64)
        per_b = np.array([x[1] / x[3] for x in e if x[3] > 0], dtype=np.float64)
        r_std = float(per_r.std()) if per_r.size > 1 else 0.0
        b_std = float(per_b.std()) if per_b.size > 1 else 0.0
        r_mean = float(per_r.mean()) if per_r.size else r_gain
        b_mean = float(per_b.mean()) if per_b.size else b_gain
        r_rel = r_std / r_mean if r_mean > 0 else 0.0
        b_rel = b_std / b_mean if b_mean > 0 else 0.0

        logger.info(f'  CT={ct}K  n={len(e)}  rGain={r_gain:.4f}  '
                    f'bGain={b_gain:.4f}  (Macbeth neutral ramp; '
                    f'inter-frame rel std r={r_rel * 100:.1f}% b={b_rel * 100:.1f}%)')
        results.append({'ct': ct, 'r': r_gain, 'b': b_gain})
        variance.append({'ct': ct, 'n': len(e),
                         'r_std': round(r_std, 5), 'b_std': round(b_std, 5),
                         'r_rel_std': round(r_rel, 5), 'b_rel_std': round(b_rel, 5)})
    return results, variance


def check_awb_curve(table):
    """
    Validate the calibrated reference_gains_table as a colour-temperature curve.

    All findings are logged (not raised) so a marginal set still produces a
    usable file:
      - single illuminant: with only one CT point the IPA extrapolates every
        other colour temperature from it, so those off-CT gains are not
        measured.
      - monotonicity: rGain (GR/R) must rise and bGain (GR/B) must fall as CT
        increases (the Planckian trend). An inversion points to a mislabelled
        ct<K> tag, mixed illumination, or a non-neutral grey card.
      - fit residual: the largest deviation of each gain from a straight line
        in mired space (1e6/CT), the colour-temperature-curve analogue of a
        transverse spread. A large residual means interpolation between the
        measured points is unreliable.
    """
    if not table:
        return

    n = len(table)
    if n < 2:
        logger.warning(
            f'AWB: single illuminant (CT={table[0]["ct"]}K). The '
            f'reference_gains_table has one point, so the IPA extrapolates '
            f'every other colour temperature from it -- those off-CT gains are '
            f'not measured. Capture the Macbeth chart under at least a warm '
            f'(~3000K) and a cool (~6500K) illuminant for a real curve.')
        return

    cts = [e['ct'] for e in table]
    r = [e['r'] for e in table]
    b = [e['b'] for e in table]

    # rGain must rise and bGain must fall as CT increases. Use a small epsilon
    # so rounding noise at the 4th decimal is not flagged as an inversion.
    eps = 1e-4
    r_inv = [(cts[i], cts[i + 1]) for i in range(n - 1) if r[i + 1] < r[i] - eps]
    b_inv = [(cts[i], cts[i + 1]) for i in range(n - 1) if b[i + 1] > b[i] + eps]

    if r_inv:
        logger.warning(f'AWB: rGain (GR/R) falls with CT between {r_inv} -- it '
                       f'should rise as CT increases. Check the ct<K> tags and '
                       f'single-illuminant lighting for those captures.')
    if b_inv:
        logger.warning(f'AWB: bGain (GR/B) rises with CT between {b_inv} -- it '
                       f'should fall as CT increases. Check the ct<K> tags and '
                       f'single-illuminant lighting for those captures.')
    if not r_inv and not b_inv:
        logger.info('AWB: reference_gains_table is monotonic in CT (rGain rises, '
                    'bGain falls) -- consistent with the Planckian trend.')

    # Two points fit a line exactly, so a residual is only meaningful from
    # three CT points up. Gains are close to linear against mired (1e6/CT).
    if n >= 3:
        mired = np.array([1.0e6 / c for c in cts])
        for label, y in (('rGain', np.array(r)), ('bGain', np.array(b))):
            coef = np.polyfit(mired, y, 1)
            resid = np.abs(y - np.polyval(coef, mired))
            max_dev = float(resid.max())
            mean_y = float(np.mean(y))
            rel = max_dev / mean_y if mean_y > 0 else 0.0
            emit = logger.warning if rel > 0.05 else logger.info
            emit(f'AWB: {label} fit residual max={max_dev:.4f} '
                 f'({rel * 100:.1f}% of mean) over {n} CT points.')


def check_awb_variance(variance):
    """
    Report the inter-frame repeatability of each CT's reference gains.

    variance is the per-CT spread returned by calibrate_awb_gains_macbeth: the
    std of the individual frames' GR/R and GR/B gains at one operating point
    (mchp_capture.sh --frames). A large relative std means the sensor was not
    settled or the light flickered while the repeats were shot, so the averaged
    reference gain for that CT is noisy. All findings are logged, not raised.
    A CT captured with a single frame has no spread to report.
    """
    if not variance:
        return

    multi = [v for v in variance if v.get('n', 0) > 1]
    if not multi:
        logger.info('AWB: one frame per CT -- no inter-frame spread to assess '
                    '(capture with --frames >1 for a repeatability bound).')
        return

    # 2% inter-frame relative std is the practical noise floor for a settled
    # sensor on a neutral ramp; above it the averaged gain is unreliable.
    thresh = 0.02
    for v in multi:
        worst = max(v['r_rel_std'], v['b_rel_std'])
        emit = logger.warning if worst > thresh else logger.info
        emit(f'AWB: CT={v["ct"]}K inter-frame rel std '
             f'r={v["r_rel_std"] * 100:.1f}% b={v["b_rel_std"] * 100:.1f}% '
             f'over {v["n"]} frames'
             + (' (noisy -- resettle sensor / steady light)'
                if worst > thresh else ''))


class AWBMicrochipISC(AWB):
    hr_name = 'AWB (Microchip ISC)'
    out_name = 'Awb'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # @brief Validate the optional awb config section.
    #
    # Returning False here is FATAL (Tuner.run aborts the whole run), so this
    # is intentionally lenient: the awb section is entirely optional, and the
    # module falls back to the config values (or nothing) when calibration is
    # not possible. Only a structurally malformed section is rejected.
    def validate_config(self, config: dict) -> bool:
        awb_cfg = config.get('awb')
        if awb_cfg is not None and not isinstance(awb_cfg, dict):
            logger.error('awb config section must be a mapping')
            return False
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        awb_cfg = config.get('awb') or {}

        # The located Macbeth neutral ramp is the single AWB source. It solves
        # the white balance from the same frame as the CCM, so the gains and the
        # colour matrix describe one identical illuminant -- self-consistent by
        # construction, with no separate grey-card capture to shoot under matched
        # ambient. Processed RGB/JPEG frames are post-WB and never used here.
        mac_caps = macbeth_grey_captures(images)
        table, variance = (calibrate_awb_gains_macbeth(mac_caps)
                           if mac_caps else (None, None))

        if not table:
            logger.info('AWB: no usable Macbeth captures; using config values')
            return config.get('awb', None)

        logger.info(f'AWB: reference_gains_table from Macbeth neutral ramp '
                    f'({len(table)} CT point(s))')

        out = dict(awb_cfg)
        out['reference_gains_table'] = table
        # Inter-frame gain spread per CT (diagnostic only): underscore-prefixed
        # so it is stripped from the emitted YAML -- reference_gains_table stays
        # byte-identical -- while the health scorer reads it to grade capture
        # repeatability. Populated only when a CT has >1 frame (M4 --frames).
        if variance:
            out['_awb_variance'] = variance

        # Solve the sensor->XYZ ct_color_matrix from the same located charts.
        # dict(awb_cfg) already carries any authored matrix, so a failed solve
        # keeps the authored value; a successful solve overrides it.
        solved = solve_ct_color_matrix(mac_caps)
        if solved is not None:
            matrix, cal_ct, n_used = solved
            out['ct_color_matrix'] = matrix
            ct_txt = f'{int(cal_ct)}K' if cal_ct else 'untagged (assumed D65)'
            logger.info(f'AWB: solved ct_color_matrix from {n_used}-patch '
                        f'Macbeth at {ct_txt}: {matrix}')
        else:
            logger.info('AWB: ct_color_matrix not solved; keeping authored value')
        ct_matrix = out.get('ct_color_matrix')

        # Diagnostic only: report how accurately the (solved or authored)
        # matrix places each tagged illuminant via Robertson (1968). The IPA
        # consumes ct_color_matrix directly, not a correction PWL, so this
        # emits no key.
        calibrate_ct_correction(mac_caps, ct_matrix)

        check_awb_curve(table)
        check_awb_variance(variance)

        if lt.Debug.Plot in self.debug:
            self._plot_reference_gains(table)

        return out

    # @brief Optional diagnostic plot of the calibrated reference gains.
    #
    # Guarded so the calibration never fails on a host without matplotlib
    # (a display backend, or the package itself, may be absent on a build
    # server). Any error is logged and swallowed -- the plot is a convenience,
    # not a calibration output.
    @staticmethod
    def _plot_reference_gains(table):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            logger.info(f'AWB plot skipped (matplotlib unavailable): {e}')
            return

        try:
            cts = [e['ct'] for e in table]
            fig, ax = plt.subplots()
            ax.plot(cts, [e['r'] for e in table], 'o-', label='rGain (GR/R)')
            ax.plot(cts, [e['b'] for e in table], 's-', label='bGain (GR/B)')
            ax.set_xlabel('Correlated colour temperature (K)')
            ax.set_ylabel('Reference gain')
            ax.set_title('AWB reference_gains_table')
            ax.grid(True)
            ax.legend()
            fig.savefig('awb_reference_gains.png')
            plt.close(fig)
            logger.info('AWB reference gains plotted to awb_reference_gains.png')
        except Exception as e:
            logger.warning(f'AWB plot failed: {e}')
