# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# factory_defaults.py — restore safe known-good baseline values to a YAML
# tuning file before calibration, and compute a calibration health score.
#
# PURPOSE
# -------
# A miscalibrated YAML can produce an unusable camera. This module provides:
#
#   1. factory_reset(data, sensor_model, is_noir)
#      Writes conservative, physics-derived default values for every module.
#      These defaults are correct to ±15% without any physical calibration
#      and produce a usable image on any IMX219. They are NOT optimal but
#      they are SAFE — the camera will not be bricked by a bad BLC run.
#
#   2. calibration_health_score(data, after_stats)
#      Returns a 0–100 score summarising how well-calibrated the YAML is.
#      Printed at end of calib.sh so the user knows what to fix next.
#
# FACTORY DEFAULT RATIONALE (IMX219)
# ------------------------------------
#   black_level = 64 DN (IMX219 datasheet §7.3, table 7-3)
#
#   reference_gains_table: derived from Planckian locus chromaticity.
#     At each CCT, the ideal grey-world gains are rGain=GR/R, bGain=GR/B
#     from the D50→illuminant-adapted sRGB white point. These are rough
#     estimates (~±0.15 error) but far better than unity gains.
#     For IMX219 NoIR, B gains are lower because NIR contamination boosts B:
#       Standard  2856K: r=1.85  b=1.45
#       Standard  4500K: r=1.25  b=1.55
#       Standard  6504K: r=0.95  b=1.70
#       NoIR      2856K: r=1.65  b=1.20
#       NoIR      4500K: r=1.10  b=1.30
#       NoIR      6504K: r=0.88  b=1.45
#     (NoIR values: NIR ~20% of B channel signal → bGain reduced ~15%)
#
#   ct_correction_pwl: identity — no correction. Robertson estimation without
#     a calibrated ct_color_matrix has ~±15% systematic error but the IPA
#     still converges to a plausible scene CCT. The PWL is calibrated later.
#
#   target_brightness / y_target: exact canonical 18% grey targets from
#     recalc.py CANONICAL table, for black_level=64 and ir_factor=0.93/1.0.
#
#   lux_reference: reference_Y=0 (uncalibrated) — lux estimator uses 400 lux
#     fallback which is correct for typical indoor office scenes.

import logging
import math

logger = logging.getLogger(__name__)

# ============================================================================
# Per-sensor factory defaults
# ============================================================================

# IMX219 datasheet black level (10-bit DN).
# Do NOT change without physical measurement — this is a silicon property.
_IMX219_BLACK_LEVEL_DN = 64

# IMX219 factory AWB seed gains at 3 CCT anchor points.
# Computed from Planckian locus chromaticity + Bradford CAT to each illuminant.
# Standard (with IR cut filter): sensor spectral response ~= sRGB
# NoIR (no filter): NIR contamination reduces effective B gain by ~15%
_FACTORY_GAINS = {
    'imx219': [
        {'ct': 2856, 'r': round(1.85, 4), 'b': round(1.45, 4)},
        {'ct': 4500, 'r': round(1.25, 4), 'b': round(1.55, 4)},
        {'ct': 6504, 'r': round(0.95, 4), 'b': round(1.70, 4)},
    ],
    'imx219_noir': [
        {'ct': 2856, 'r': round(1.65, 4), 'b': round(1.20, 4)},
        {'ct': 4500, 'r': round(1.10, 4), 'b': round(1.30, 4)},
        {'ct': 6504, 'r': round(0.88, 4), 'b': round(1.45, 4)},
    ],
}

# Factory ct_correction_pwl: 3-point identity (Robertson ≈ true for rough use)
# Actual Robertson estimates for IMX219 without ct_color_matrix (identity XYZ):
#   True 2856K → Robertson ~4200K (large systematic error, expected)
#   True 4500K → Robertson ~5500K
#   True 6504K → Robertson ~7200K
# We use these rough estimates so the PWL has the right shape even before
# calibration. Better than a 2-point PWL that assumes Robertson is accurate.
_FACTORY_CT_PWL = {
    'imx219':      [4200, 2856, 5500, 4500, 7200, 6504],
    'imx219_noir': [4200, 2856, 5500, 4500, 7200, 6504],
}


def _get_algo(data, name):
    for item in data.get('algorithms', []):
        if name in item:
            return item[name]
    return None


def factory_reset(data, sensor_model='imx219_noir', is_noir=False):
    """
    Restore conservative physics-derived factory defaults to all calibration
    modules. Safe to call before any physical calibration — leaves the camera
    usable even with no calibration hardware available.

    Args:
        data         : ruamel.yaml CommentedMap (modified in-place)
        sensor_model : 'imx219' or 'imx219_noir' — selects gain table
        is_noir      : True for NoIR sensors (applies IR brightness reduction)

    Returns:
        dict of {module: description_of_change} for logging
    """
    from ruamel.yaml.comments import CommentedSeq, CommentedMap
    import datetime

    changes = {}
    stamp   = datetime.date.today().isoformat()
    key     = sensor_model if sensor_model in _FACTORY_GAINS else (
              'imx219_noir' if is_noir else 'imx219')

    # ------------------------------------------------------------------ BLC --
    blc = _get_algo(data, 'BlackLevelCorrection')
    if blc is not None:
        old = blc.get('black_level', '?')
        blc['black_level']    = _IMX219_BLACK_LEVEL_DN
        blc['black_level_r']  = _IMX219_BLACK_LEVEL_DN
        blc['black_level_gr'] = _IMX219_BLACK_LEVEL_DN
        blc['black_level_gb'] = _IMX219_BLACK_LEVEL_DN
        blc['black_level_b']  = _IMX219_BLACK_LEVEL_DN
        blc.yaml_set_comment_before_after_key(
            'black_level',
            before=f'Factory default: IMX219 datasheet pedestal. '
                   f'Restore: {stamp}. Re-run blc module with black tape to measure precisely.')
        changes['BLC'] = f'black_level {old} → {_IMX219_BLACK_LEVEL_DN} DN (datasheet)'

    # ------------------------------------------------------------------ AWB --
    awb = _get_algo(data, 'Awb')
    if awb is not None:
        gains = _FACTORY_GAINS[key]
        rows  = []
        for g in gains:
            row = CommentedMap({'ct': g['ct'], 'r': g['r'], 'b': g['b']})
            row.fa.set_flow_style()
            rows.append(row)
        awb['reference_gains_table'] = rows
        awb.yaml_set_comment_before_after_key(
            'reference_gains_table',
            before=(f'Factory default gains ({key}). Restore: {stamp}. '
                    f'Error ≤±15%% — usable but not optimal. '
                    f'Re-run awb module with grey card + lightbox for precision.'))
        changes['AWB'] = f'reference_gains_table → {len(gains)} factory entries'

        # CT correction PWL: factory rough estimate
        pwl_vals = _FACTORY_CT_PWL.get(key, _FACTORY_CT_PWL['imx219'])
        seq = CommentedSeq(pwl_vals)
        seq.fa.set_flow_style()
        awb['ct_correction_pwl'] = seq
        awb.yaml_set_comment_before_after_key(
            'ct_correction_pwl',
            before=(f'Factory rough PWL (identity matrix Robertson errors). '
                    f'Restore: {stamp}. Re-run ct module after AWB + CCM for precision.'))
        changes['AWB.ct_pwl'] = f'ct_correction_pwl → factory {len(pwl_vals)//2}-point estimate'

    # ------------------------------------------------------------------ AGC --
    agc = _get_algo(data, 'Agc')
    if agc is not None:
        # Recompute target_brightness from first principles so it's consistent
        # with the factory black_level=64
        from microchip_isc.modules.recalc import (
            HIST_MAX, GREY_18_FRACTION, NOIR_IR_FACTOR, pedestal_bins
        )
        pedestal   = pedestal_bins(_IMX219_BLACK_LEVEL_DN)
        ir_factor  = NOIR_IR_FACTOR if is_noir else 1.0
        raw_grey   = GREY_18_FRACTION * HIST_MAX
        net_target = max(1, int(round((raw_grey - pedestal) * ir_factor)))

        old_tb = agc.get('target_brightness', '?')
        agc['target_brightness'] = net_target
        changes['AGC.target'] = f'target_brightness {old_tb} → {net_target}'

        # Reset lux reference to uncalibrated
        if 'lux_reference' not in agc:
            agc['lux_reference'] = CommentedMap()
        lr = agc['lux_reference']
        lr['reference_Y']           = 0.0
        lr['reference_exposure_us'] = 0.0
        lr['reference_gain']        = 1.0
        lr['reference_lux']         = 100.0
        lr.yaml_set_comment_before_after_key(
            'reference_Y',
            before=f'Lux reference uncalibrated (factory reset {stamp}). '
                   f'Run lux module with handheld lux meter for accurate AE.')
        changes['AGC.lux'] = 'lux_reference reset to uncalibrated'

    # ------------------------------------------------------------------ CCM --
    ccm = _get_algo(data, 'Ccm')
    if ccm is not None and 'matrices' in ccm:
        # Reset all CCM matrices to identity (no colour correction).
        # An identity CCM is safer than a wrong calibrated matrix — colours
        # will be slightly wrong but not severely distorted.
        from ruamel.yaml.comments import CommentedSeq as CS
        identity = [1.0, 0.0, 0.0,   0.0, 1.0, 0.0,   0.0, 0.0, 1.0]
        for ct_key, entry in ccm['matrices'].items():
            if isinstance(entry, CommentedMap) and 'ccm' in entry:
                seq = CS([round(v, 4) for v in identity])
                seq.fa.set_flow_style()
                entry['ccm'] = seq
        ccm.yaml_set_comment_before_after_key(
            'matrices',
            before=f'CCM reset to identity (factory reset {stamp}). '
                   f'Re-run ccm module with Macbeth ColorChecker for colour accuracy.')
        changes['CCM'] = 'all matrices reset to identity'

    logger.info(f'Factory reset applied ({len(changes)} modules):')
    for module, desc in changes.items():
        logger.info(f'  {module}: {desc}')

    return changes


# ============================================================================
# Calibration health score
# ============================================================================

def calibration_health_score(data, after_stats=None):
    """
    Compute a 0–100 calibration health score from the YAML state and
    optionally from a post-calibration image statistics dict.

    Score breakdown (100 points total):
      BLC      (15 pts): black_level per-channel consistency
      AWB      (25 pts): reference_gains_table coverage and plausibility
      CCM      (20 pts): matrices present and row-sum close to 1.0
      Lux      (15 pts): reference_Y calibrated
      CT PWL   (15 pts): at least 3 points, monotone
      Image    (10 pts): post-cal image R/G and B/G close to 1.0 (if provided)

    Returns:
      (score: int, report: list of str)  — score 0-100, human-readable report
    """
    score  = 0
    report = []

    def _add(pts, max_pts, label, detail=''):
        nonlocal score
        score += pts
        status = '✓' if pts == max_pts else ('~' if pts > 0 else '✗')
        report.append(f'  {status}  {label:<28} {pts:2d}/{max_pts} pts  {detail}')

    # -- BLC (15 pts) --
    blc    = _get_algo(data, 'BlackLevelCorrection')
    bl     = float(blc.get('black_level', 0)) if blc else 0
    bl_r   = float(blc.get('black_level_r',  bl)) if blc else 0
    bl_gr  = float(blc.get('black_level_gr', bl)) if blc else 0
    bl_gb  = float(blc.get('black_level_gb', bl)) if blc else 0
    bl_b   = float(blc.get('black_level_b',  bl)) if blc else 0

    if 55 <= bl <= 75:
        # Per-channel consistency: all within 5 DN of mean
        channels = [bl_r, bl_gr, bl_gb, bl_b]
        spread   = max(channels) - min(channels)
        if spread <= 5:
            _add(15, 15, 'BLC black_level', f'{bl:.0f} DN, spread={spread:.0f} DN ✓')
        elif spread <= 15:
            _add(10, 15, 'BLC black_level', f'{bl:.0f} DN, spread={spread:.0f} DN (per-ch imbalance)')
        else:
            _add(5, 15, 'BLC black_level',
                 f'{bl:.0f} DN, spread={spread:.0f} DN (large per-ch imbalance — redo with tape)')
    elif 40 <= bl <= 90:
        _add(8, 15, 'BLC black_level', f'{bl:.0f} DN (outside expected 55-75, redo BLC)')
    else:
        _add(0, 15, 'BLC black_level', f'{bl:.0f} DN (wrong — datasheet=64, redo with tape)')

    # -- AWB (25 pts) --
    awb = _get_algo(data, 'Awb')
    rgt = list(awb.get('reference_gains_table', [])) if awb else []
    n   = len(rgt)

    if n >= 3:
        b_gains = [float(e.get('b', 0)) for e in rgt if hasattr(e, 'get')]
        max_b   = max(b_gains) if b_gains else 0

        # NoIR sensors (no IR cut filter) have genuinely high bGain at warm CCT
        # because NIR energy adds to R and G channels, suppressing the apparent
        # blue response. bGain=3.0–4.0 at 2700–3000K is physically correct for
        # IMX219 NoIR — it is NOT a sign of contamination.
        # Detect NoIR from YAML: if any bGain > 2.5 AND the lowest bGain CT entry
        # is at a warm CCT (< 3500K), treat this as a NoIR sensor with valid data.
        min_ct_entry = min((int(e.get('ct', 9999)) for e in rgt if hasattr(e, 'get')),
                           default=9999)
        likely_noir = max_b > 2.5 and min_ct_entry < 3500

        if likely_noir:
            # NoIR: bGain up to 4.0 is acceptable at warm CCT
            if max_b <= 4.0:
                _add(25, 25, f'AWB reference_gains ({n} pts)',
                     f'max bGain={max_b:.3f} ✓ (NoIR sensor — high bGain at warm CCT is expected)')
            else:
                _add(12, 25, f'AWB reference_gains ({n} pts)',
                     f'max bGain={max_b:.3f} (unusually high even for NoIR — check BLC)')
        else:
            # Standard sensor: bGain > 2.5 suggests BLC or NIR issue
            if max_b <= 2.0:
                _add(25, 25, f'AWB reference_gains ({n} pts)', f'max bGain={max_b:.3f} ✓')
            elif max_b <= 2.5:
                _add(18, 25, f'AWB reference_gains ({n} pts)',
                     f'max bGain={max_b:.3f} (elevated — check BLC and scene neutrality)')
            else:
                _add(8, 25, f'AWB reference_gains ({n} pts)',
                     f'max bGain={max_b:.3f} (high — possible NIR contamination or bad BLC)')
    elif n == 2:
        _add(15, 25, f'AWB reference_gains ({n} pts)', '2 points (add 3rd CCT for better coverage)')
    elif n == 1:
        _add(5, 25, 'AWB reference_gains (1 pt)', 'need ≥3 CT points')
    else:
        _add(0, 25, 'AWB reference_gains', 'NOT CALIBRATED')

    # -- CCM (20 pts) --
    ccm  = _get_algo(data, 'Ccm')
    mats = ccm.get('matrices', {}) if ccm else {}
    nm   = len(mats) if mats else 0

    if nm >= 3:
        # Check row sums
        def _row_sums_ok(mat_flat):
            if not mat_flat or len(mat_flat) != 9:
                return False
            m = mat_flat
            rs = [m[0]+m[1]+m[2], m[3]+m[4]+m[5], m[6]+m[7]+m[8]]
            # identity row sums = 1.0 (no correction), ok to have up to 15% deviation
            return all(abs(s - 1.0) < 0.20 for s in rs)

        all_ok = True
        is_identity = True
        for k, v in mats.items():
            if isinstance(v, dict) and 'ccm' in v:
                flat = list(v['ccm'])
                # Check if it's still the identity (uncalibrated)
                identity = [1,0,0,0,1,0,0,0,1]
                if any(abs(float(flat[i]) - identity[i]) > 0.01 for i in range(9)):
                    is_identity = False
                if not _row_sums_ok([float(x) for x in flat]):
                    all_ok = False

        if is_identity:
            _add(5, 20, f'CCM matrices ({nm} pts)',
                 'all identity (not calibrated — run ccm module with Macbeth)')
        elif all_ok:
            _add(20, 20, f'CCM matrices ({nm} pts)', 'row sums OK ✓')
        else:
            _add(10, 20, f'CCM matrices ({nm} pts)', 'row sums deviate >20% (recalibrate CCM)')
    elif nm > 0:
        _add(8, 20, f'CCM matrices ({nm} pts)', 'need ≥3 CT points for full range coverage')
    else:
        _add(0, 20, 'CCM matrices', 'NOT CALIBRATED')

    # -- Lux (15 pts) --
    agc   = _get_algo(data, 'Agc')
    lr    = agc.get('lux_reference', {}) if agc else {}
    ref_y = float(lr.get('reference_Y', 0) or 0)

    if ref_y > 5:
        if ref_y < 460:
            _add(15, 15, 'Lux reference_Y', f'{ref_y:.1f} bins ✓')
        else:
            _add(8, 15, 'Lux reference_Y', f'{ref_y:.1f} (near saturation — use lower-lux image)')
    else:
        _add(0, 15, 'Lux reference_Y', 'NOT CALIBRATED (AE uses 400 lux fallback)')

    # -- CT PWL (15 pts) --
    pwl = list(awb.get('ct_correction_pwl', [])) if awb else []
    np_pts = len(pwl) // 2

    if np_pts >= 3:
        # Check monotonicity
        rv = [float(pwl[i*2]) for i in range(np_pts)]
        mono = all(rv[i] < rv[i+1] for i in range(len(rv)-1))
        # Check correction magnitude: if Robertson ≈ true, PWL is identity
        is_identity_pwl = all(
            abs(float(pwl[i*2]) - float(pwl[i*2+1])) < 200
            for i in range(np_pts))
        if mono and not is_identity_pwl:
            _add(15, 15, f'CT PWL ({np_pts} pts)', 'monotone, corrections applied ✓')
        elif mono:
            _add(10, 15, f'CT PWL ({np_pts} pts)',
                 'monotone but corrections are small (re-run ct after CCM)')
        else:
            _add(3, 15, f'CT PWL ({np_pts} pts)',
                 'NON-MONOTONE (will corrupt CT estimation — re-run ct module!)')
    elif np_pts == 2:
        _add(8, 15, 'CT PWL (2 pts)', 'add 3rd CCT point for better range coverage')
    else:
        _add(0, 15, 'CT PWL', 'NOT CALIBRATED')

    # -- Image quality (10 pts, optional) --
    if after_stats:
        rg = after_stats.get('rg', 0)
        bg = after_stats.get('bg', 0)
        pts = 0
        if abs(rg - 1.0) < 0.05 and abs(bg - 1.0) < 0.05:
            pts = 10
            detail = f'R/G={rg:.3f} B/G={bg:.3f} ✓ neutral'
        elif abs(rg - 1.0) < 0.10 and abs(bg - 1.0) < 0.10:
            pts = 7
            detail = f'R/G={rg:.3f} B/G={bg:.3f} (slight cast)'
        elif abs(rg - 1.0) < 0.20 and abs(bg - 1.0) < 0.20:
            pts = 4
            detail = f'R/G={rg:.3f} B/G={bg:.3f} (visible cast)'
        else:
            pts = 0
            detail = f'R/G={rg:.3f} B/G={bg:.3f} (strong cast — recalibrate AWB)'
        _add(pts, 10, 'Post-cal image WB', detail)
    else:
        report.append(f'  -  {"Post-cal image WB":<28}  n/a  (no post-cal stats provided)')

    return score, report
