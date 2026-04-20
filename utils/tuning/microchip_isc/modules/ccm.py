# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# CCM calibration module.
# Derives per-CT 3x3 colour correction matrices from ColorChecker captures.
#
# Methodology:
#   Sensor RAW data is LINEAR (no gamma applied by the ISC pipeline before
#   the CC block). The reference ColorChecker values are sRGB-encoded (gamma).
#   Correct pipeline:
#     1. Normalise measured RAW patch means to white patch (white = 1.0)
#     2. Apply Bradford chromatic adaptation to map reference from D50 to the
#        capture illuminant (so reference and measured are under the same light)
#     3. Linearise the (adapted) sRGB reference through sRGB EOTF
#     4. Solve weighted LSQ: measured_linear @ M = reference_adapted_linear
#     5. No row-normalisation post-hoc (destroys LSQ solution) — use
#        constrained solve with row-sum=1 if neutrality is required
#
# References:
#   - Finlayson & Süsstrunk (2000), "Performance of a chromatic adaptation
#     transform" (Bradford CAT)
#   - Jiang et al. (2013), "What is the Space of Spectral Sensitivity Functions
#     for Digital Color Cameras?"

import logging
import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# X-Rite ColorChecker Classic reference sRGB values (D50 illuminant, 2005)
# Patch order: row-major top-left to bottom-right (24 patches)
# ---------------------------------------------------------------------------
_SRGB_REF_D50 = np.array([
    (115, 82,  68),   #  0  dark_skin
    (194, 150, 130),  #  1  light_skin
    (98,  122, 157),  #  2  blue_sky
    (87,  108, 67),   #  3  foliage
    (133, 128, 177),  #  4  blue_flower
    (103, 189, 170),  #  5  bluish_green
    (214, 126, 44),   #  6  orange
    (80,  91,  166),  #  7  purplish_blue
    (193, 90,  99),   #  8  moderate_red
    (94,  60,  108),  #  9  purple
    (157, 188, 64),   # 10  yellow_green
    (224, 163, 46),   # 11  orange_yellow
    (56,  61,  150),  # 12  blue
    (70,  148, 73),   # 13  green
    (175, 54,  60),   # 14  red
    (231, 199, 31),   # 15  yellow
    (187, 86,  149),  # 16  magenta
    (8,   133, 161),  # 17  cyan
    (243, 243, 243),  # 18  white
    (200, 200, 200),  # 19  neutral_8
    (160, 160, 160),  # 20  neutral_6_5
    (122, 122, 122),  # 21  neutral_5
    (85,  85,  85),   # 22  neutral_3_5
    (52,  52,  52),   # 23  black
], dtype=np.float64) / 255.0

# Grey patch indices (bottom row: white + 5 neutrals)
_GREY_IDX = [18, 19, 20, 21, 22, 23]

# ---------------------------------------------------------------------------
# Bradford chromatic adaptation matrices
# Src: Lindbloom, http://www.brucelindbloom.com/index.html?Eqn_ChromAdapt.html
# ---------------------------------------------------------------------------
_BRADFORD = np.array([
    [ 0.8951,  0.2664, -0.1614],
    [-0.7502,  1.7135,  0.0367],
    [ 0.0389, -0.0685,  1.0296],
], dtype=np.float64)

_BRADFORD_INV = np.linalg.inv(_BRADFORD)

# CIE XYZ of D50 white point (2° observer, normalised Y=1)
_D50_XYZ = np.array([0.96422, 1.00000, 0.82521], dtype=np.float64)

# sRGB primaries to XYZ (D65) — IEC 61966-2-1
_SRGB_TO_XYZ_D65 = np.array([
    [0.4124564, 0.3575761, 0.1804375],
    [0.2126729, 0.7151522, 0.0721750],
    [0.0193339, 0.1191920, 0.9503041],
], dtype=np.float64)

# D65 white point
_D65_XYZ = np.array([0.95047, 1.00000, 1.08883], dtype=np.float64)


def _cct_to_xyz(cct_k):
    """
    Convert CCT in Kelvin to approximate CIE XYZ white point using the
    Kang et al. (2002) chromaticity polynomial + Y=1 normalisation.
    Valid for 1667 K – 25000 K.
    """
    T = float(cct_k)
    if T <= 4000:
        x = (-0.2661239e9 / (T**3) - 0.2343589e6 / (T**2)
              + 0.8776956e3 / T + 0.179910)
    else:
        x = (-3.0258469e9 / (T**3) + 2.1070379e6 / (T**2)
              + 0.2226347e3 / T + 0.240390)

    if T <= 2222:
        y = -1.1063814 * x**3 - 1.34811020 * x**2 + 2.18555832 * x - 0.20219683
    elif T <= 4000:
        y = -0.9549476 * x**3 - 1.37418593 * x**2 + 2.09137015 * x - 0.16748867
    else:
        y =  3.0817580 * x**3 - 5.87338670 * x**2 + 3.75112997 * x - 0.37001483

    # xy -> XYZ with Y=1
    if abs(y) < 1e-9:
        return _D65_XYZ.copy()
    X = x / y
    Z = (1.0 - x - y) / y
    return np.array([X, 1.0, Z], dtype=np.float64)


def _bradford_cat(src_white_xyz, dst_white_xyz):
    """
    Compute 3x3 Bradford chromatic adaptation matrix from src to dst white.
    Applied as: xyz_dst = M @ xyz_src
    """
    cone_src = _BRADFORD @ src_white_xyz
    cone_dst = _BRADFORD @ dst_white_xyz
    scale = np.diag(cone_dst / cone_src)
    return _BRADFORD_INV @ scale @ _BRADFORD


def _adapt_reference_to_illuminant(ref_srgb_d50, illuminant_xyz):
    """
    Chromatically adapt the sRGB/D50 ColorChecker reference to the capture
    illuminant. This puts reference and measured data under the same light
    so the CCM can be solved correctly.

    Steps:
      1. sRGB (D50) -> linear sRGB (D50)
      2. linear sRGB (D50) -> XYZ (D50)  via sRGB primaries adapted to D50
      3. Bradford CAT: XYZ (D50) -> XYZ (illuminant)
      4. XYZ (illuminant) -> linear RGB (illuminant) via inverse of the
         sensor-to-XYZ matrix would be ideal, but since we don't have it
         here we return XYZ directly and the CCM solve handles the mapping

    Returns: Nx3 array of linear values adapted to the capture illuminant
    """
    # Step 1: sRGB gamma -> linear
    def _srgb_to_linear(v):
        return np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4)

    ref_linear_d50 = _srgb_to_linear(ref_srgb_d50)  # Nx3 linear, D50

    # Step 2: linear sRGB (D65) -> XYZ (D65), then adapt D65 -> D50
    # sRGB primaries are defined for D65; the reference values are given for D50.
    # We need XYZ at D50, so use the D50-adapted sRGB matrix.
    # M_sRGB_D65_to_XYZ_D65 × Bradford(D65→D50) = sRGB_to_XYZ_D50
    cat_d65_to_d50 = _bradford_cat(_D65_XYZ, _D50_XYZ)
    srgb_to_xyz_d50 = cat_d65_to_d50 @ _SRGB_TO_XYZ_D65  # 3x3

    ref_xyz_d50 = (srgb_to_xyz_d50 @ ref_linear_d50.T).T  # Nx3 XYZ at D50

    # Step 3: Bradford CAT: D50 -> capture illuminant
    cat_d50_to_illum = _bradford_cat(_D50_XYZ, illuminant_xyz)
    ref_xyz_illum = (cat_d50_to_illum @ ref_xyz_d50.T).T  # Nx3 XYZ at illuminant

    return ref_xyz_illum  # Nx3, linear XYZ under capture illuminant


def solve_ccm(measured_raw, ref_xyz_illum, weights=None):
    """
    Solve for the 3x3 CCM using weighted least squares.

    measured_raw  : Nx3 array of linear sensor RAW values normalised to
                    white patch (so white = [1, 1, 1]).  These are the raw
                    sensor responses BEFORE any CCM is applied.
    ref_xyz_illum : Nx3 array of reference XYZ values adapted to the capture
                    illuminant (output of _adapt_reference_to_illuminant).
    weights       : optional length-N weight vector.

    The CCM M satisfies: measured_raw @ M ≈ ref_xyz_illum
    i.e. M maps sensor RGB to CIE XYZ under the capture illuminant.

    NOTE: This function does NOT row-normalise M.  Row normalisation
    (forcing each row to sum to 1) is an energy-preserving heuristic that
    destroys the ΔE-minimising LSQ solution.  If neutrality (grey-world
    constraint) is needed, use a constrained solve with an explicit
    grey-axis penalty instead of post-hoc normalisation.

    Returns (M 3x3, mean_delta_E_approx).
    """
    A = measured_raw   # Nx3
    B = ref_xyz_illum  # Nx3

    if weights is not None:
        W = np.diag(np.sqrt(weights))  # weighted LSQ via row scaling
        A_w = W @ A
        B_w = W @ B
    else:
        A_w, B_w = A, B

    M, _, _, _ = np.linalg.lstsq(A_w, B_w, rcond=None)  # 3x3

    predicted = A @ M
    diff = predicted - B
    # NOTE: this is a scaled Euclidean distance in XYZ space, NOT CIE ΔE2000.
    # A value of 3.0 here does NOT imply perceptually acceptable ΔE < 3.
    # It is useful for comparing calibration quality across runs only.
    # Actual perceptual quality requires measurement on a display with a
    # proper colour appearance model (CIE ΔE94 or CIEDE2000).
    xyz_rms_scaled = float(np.sqrt((diff ** 2).sum(axis=1)).mean() * 100.0)
    return M, xyz_rms_scaled


# Pre-compute inverse sRGB matrix once at module load time.
# Recomputing np.linalg.inv() inside the per-image loop wastes CPU and
# raises LinAlgError inside the loop if somehow singular.
try:
    _XYZ_D65_TO_SRGB_LINEAR = np.linalg.inv(_SRGB_TO_XYZ_D65)
except np.linalg.LinAlgError as e:
    raise RuntimeError(f'sRGB→XYZ matrix is singular: {e}') from e


def calibrate_ccm(images):
    """
    Calibrate CCM matrices from a list of IscImage objects.

    Each image must have:
      img.macbeth  : True (patch data extracted)
      img.ct       : true CCT in Kelvin (from filename tag ct<K>)
      img.patches  : list of 4 arrays [R_means, GR_means, GB_means, B_means]
                     each of length 24, in ISC histogram units (0–511 after BLC)

    Returns dict: {ct_K: {'matrix': [[...]], 'de': float}}
    The matrix maps sensor RAW RGB to CIE XYZ under the capture illuminant.
    """
    results = {}

    for img in images:
        if not img.macbeth or img.ct is None or img.patches is None:
            logger.warning(f'{img.name}: skipping (no patches or no CT tag)')
            continue

        if len(img.patches) < 4:
            logger.warning(f'{img.name}: not enough patch channels')
            continue

        # Combine GR and GB into a single G channel.
        # Simple average is correct for the grey patches (GR≈GB for neutral).
        # For chromatic patches, GR and GB differ because the sensor rows have
        # slightly different spectral responses. We use the weighted average
        # (equal weight) as the best single-G estimate for the 3-channel CCM.
        # The ISC CCM operates on demosaiced RGB where G is the average of
        # GR and GB pixels — this matches the hardware behaviour.
        G = (img.patches[1] + img.patches[2]) / 2.0
        meas = np.stack([img.patches[0], G, img.patches[3]], axis=1)  # 24x3

        # Sanity check: verify patches have exactly 24 entries
        if meas.shape[0] != 24:
            logger.warning(f'{img.name}: expected 24 patches but got {meas.shape[0]}. '
                           f'Check chart corner coordinates.')
            continue

        # White patch is patch 18. Use the GREEN channel of the white patch
        # as the normalisation reference. Normalising by the RGB mean of white
        # would be biased on a NoIR sensor where R and B contain NIR energy,
        # making white_val artificially high and underscaling R/B in meas_norm.
        # Green is the least NIR-contaminated channel and is the reference
        # used throughout the AWB and AGC algorithms.
        white_val = float(meas[18, 1])  # G channel of white patch (index 1)
        if white_val < 10.0:
            # White patch suspiciously dark — likely wrong corners or bad image
            logger.warning(f'{img.name}: white patch G-channel mean={white_val:.1f} '
                           f'(expected ~200-500 in ISC histogram units). '
                           f'Check chart corners or verify grey card placement.')
            continue

        meas_norm = meas / white_val  # normalised linear RAW [0, 1]

        # Chromatic adaptation: map D50 reference to capture illuminant XYZ
        illuminant_xyz = _cct_to_xyz(img.ct)
        ref_xyz_illum = _adapt_reference_to_illuminant(_SRGB_REF_D50, illuminant_xyz)

        # Convert reference XYZ (at capture illuminant) back to linear sRGB.
        # The ISC CCM hardware maps sensor_RGB → output_RGB, not sensor_RGB → XYZ.
        # Solving measured @ M = ref_xyz would produce a matrix that maps to XYZ,
        # which when applied as an RGB→RGB colour matrix produces wrong colours.
        # Correct target space: linear sRGB under the capture illuminant.
        # _XYZ_D65_TO_SRGB_LINEAR is pre-computed at module load (see above).
        # Our ref_xyz_illum is under the capture illuminant, not D65.
        # Bradford-adapt from capture illuminant to D65 for the sRGB conversion.
        cat_illum_to_d65 = _bradford_cat(illuminant_xyz, _D65_XYZ)
        ref_xyz_d65 = (cat_illum_to_d65 @ ref_xyz_illum.T).T  # Nx3 XYZ at D65
        ref_linear_srgb = (_XYZ_D65_TO_SRGB_LINEAR @ ref_xyz_d65.T).T  # Nx3 linear sRGB
        # Clip negative values (out-of-gamut for sRGB display) — expected for
        # highly saturated Macbeth patches (blue, green, red corners).
        ref_linear_srgb = np.clip(ref_linear_srgb, 0.0, None)

        # Normalise reference to white patch G (same as measured normalisation)
        ref_white_g = float(ref_linear_srgb[18, 1])
        if ref_white_g < 1e-6:
            logger.warning(f'{img.name}: reference white patch G near zero, skipping')
            continue
        ref_norm = ref_linear_srgb / ref_white_g

        # Weights: grey patches (18–23) 3× to enforce grey-axis neutrality
        weights = np.ones(24)
        weights[_GREY_IDX] = 3.0

        M, xyz_err = solve_ccm(meas_norm, ref_norm, weights)

        ccm_flat = [[round(float(M[i, j]), 4) for j in range(3)] for i in range(3)]
        row_sums = [round(sum(row), 4) for row in ccm_flat]

        logger.info(f'CCM at {img.ct:.0f}K  XYZ-RMS-scaled={xyz_err:.3f} '
                    f'(NOT CIE ΔE — see solve_ccm docstring):')
        for row in ccm_flat:
            logger.info(f'  {row}')
        logger.info(f'  Row sums: {row_sums}')
        if any(abs(s - 1.0) > 0.15 for s in row_sums):
            logger.warning(f'{img.name}: CCM row sums deviate >15%% from 1.0 '
                           f'({row_sums}). This indicates a non-energy-preserving '
                           f'matrix — may cause brightness shifts at extreme CTs. '
                           f'Check grey card neutrality and single-illuminant '
                           f'capture conditions.')

        de = xyz_err  # keep variable name for results dict

        results[int(img.ct)] = {
            'matrix': ccm_flat,
            'de': de,
        }

    return results if results else None
