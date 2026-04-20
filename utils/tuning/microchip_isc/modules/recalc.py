# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# recalc.py -- Recalculate all YAML constants that are derived from the
# sensor black level pedestal.
#
# Context:
#   The ISC histogram taps vp_data (PFE output) which is pre-BLC, pre-WB.
#   The sensor black level pedestal is present in every histogram channel mean.
#   processStats() in the IPA subtracts (black_level / 2) histogram bins before
#   passing means to AWB and AGC algorithms.
#
#   Because of this subtraction, all YAML values that are in absolute histogram
#   bin units must be recomputed when black_level changes.  This module does
#   that automatically so the tool is the single source of truth.
#
# Values recomputed:
#   Agc.target_brightness        : 18% grey target minus pedestal
#   Agc.ae_luminance_floor       : engineering floor (min reliable net signal)
#   Awb.min_scene_luminance      : engineering floor (min SNR for grey-world)
#   Agc.y_target                 : all absolute targets shifted by -pedestal
#
# Values NOT touched here (require live measurement):
#   Agc.lux_reference.reference_Y : must come from lux module with lux meter
#
# Design constants:
#   GREY_18_FRACTION   = 0.18   (18% grey standard target)
#   HIST_MAX           = 511    (ISC histogram max bin index)
#   SENSOR_MAX_DN      = 1023   (10-bit ADC full scale)
#   MIN_SNR_BINS       = 8      (minimum net bins for reliable grey-world)
#   AE_FLOOR_BINS      = 3      (AE stability floor in net bins)
#   NOIR_IR_FACTOR     = 0.93   (NoIR IR offset: ~7% brightness reduction target)

import logging

logger = logging.getLogger(__name__)

# ISC hardware constants
HIST_MAX     = 511    # histogram max bin (512 bins, 0-indexed)
SENSOR_MAX   = 1023   # IMX219 10-bit ADC full scale

# Design targets (independent of pedestal)
GREY_18_FRACTION = 0.18    # 18% grey card reflectance
MIN_SNR_BINS     = 8       # minimum net bins for reliable AWB (SNR ~5:1)
AE_FLOOR_BINS    = 3       # AE stability floor (net bins)
NOIR_IR_FACTOR   = 0.93    # NoIR: 7% brightness reduction vs standard

# Minimum allowable y_target value (normalised).
# Below this the AGC would underexpose so severely that noise dominates.
MIN_Y_TARGET = 0.005


def pedestal_bins(black_level_dn):
    """
    Convert sensor black level in 10-bit DN to histogram bin units.

    ISC histogram: bin_index = pixel_value >> 1
    So pedestal_bins = black_level_dn / 2.0
    """
    return float(black_level_dn) / 2.0


def recalculate(data, black_level=None, is_noir=False):
    """
    Recalculate all histogram-bin-level YAML constants from the sensor
    black level pedestal.

    This function is IDEMPOTENT: it always computes the correct absolute
    value from first principles (18% grey - pedestal), not relative to
    the current YAML value. Running it twice gives the same result.

    Args:
      data         : loaded YAML document (ruamel.yaml CommentedMap)
      black_level  : sensor black level in 10-bit DN.
                     If None, read from BlackLevelCorrection.black_level in YAML.
      is_noir      : True for NoIR sensors (applies IR brightness reduction factor)

    Returns:
      dict of {key: (old_value, new_value)} for all changed values, for reporting.
    """
    changes = {}

    # --- Get black level ---
    if black_level is None:
        blc = _get_algo(data, 'BlackLevelCorrection')
        if blc is None:
            logger.error('No BlackLevelCorrection block in YAML -- cannot recalculate')
            return changes
        black_level = int(blc.get('black_level', 64))

    pedestal = pedestal_bins(black_level)
    ir_factor = NOIR_IR_FACTOR if is_noir else 1.0

    logger.info(f'Recalculating with black_level={black_level} DN '
                f'({pedestal:.1f} histogram bins), '
                f'{"NoIR" if is_noir else "standard"} sensor')

    # --- AGC block ---
    agc = _get_algo(data, 'Agc')
    if agc is not None:

        # target_brightness: 18% grey in net histogram bins.
        # ISC histogram: bin = pixel >> 1, so HIST_MAX = 511 (10-bit max >> 1).
        # 18% grey = 0.18 * 511 = 92.0 raw bins (pre-pedestal).
        # Use HIST_MAX directly, not SENSOR_MAX/2, to avoid the 0.5-bin
        # systematic error from integer rounding of 1023/2 = 511.5.
        raw_grey_bins = GREY_18_FRACTION * HIST_MAX
        net_target = (raw_grey_bins - pedestal) * ir_factor
        new_target = max(1, int(round(net_target)))
        old_target = agc.get('target_brightness', None)
        if old_target != new_target:
            agc['target_brightness'] = new_target
            changes['Agc.target_brightness'] = (old_target, new_target)
            logger.info(f'  target_brightness: {old_target} -> {new_target} '
                        f'(18% grey = {raw_grey_bins:.1f} raw bins, '
                        f'net = {raw_grey_bins - pedestal:.1f}, '
                        f'with IR factor {ir_factor:.2f})')

        # ae_luminance_floor: engineering floor (net bins)
        old_floor = agc.get('ae_luminance_floor', None)
        new_floor = float(AE_FLOOR_BINS)
        if old_floor != new_floor:
            agc['ae_luminance_floor'] = new_floor
            changes['Agc.ae_luminance_floor'] = (old_floor, new_floor)
            logger.info(f'  ae_luminance_floor: {old_floor} -> {new_floor} '
                        f'(engineering floor, {AE_FLOOR_BINS} net bins)')

        if 'y_target' in agc:
            _recalc_y_target(agc, pedestal, ir_factor, changes)

    # --- AWB block ---
    awb = _get_algo(data, 'Awb')
    if awb is not None and 'convergence' in awb:
        conv_awb = awb['convergence']
        old_lsl = conv_awb.get('min_scene_luminance', None)
        new_lsl = float(MIN_SNR_BINS)
        if old_lsl != new_lsl:
            conv_awb['min_scene_luminance'] = new_lsl
            changes['Awb.min_scene_luminance'] = (old_lsl, new_lsl)
            logger.info(f'  Awb.min_scene_luminance: {old_lsl} -> {new_lsl} '
                        f'(SNR floor, {MIN_SNR_BINS} net bins)')

    return changes


def _recalc_y_target(agc, pedestal, ir_factor, changes):
    """
    Recompute every target in the y_target table from the canonical 18%-grey
    brightness curve, adjusted for the current pedestal and IR factor.

    y_target format: flat list [lux0, t0, lux1, t1, ...]
    where t_i is a normalised brightness target: absolute_bins = t_i * HIST_MAX

    CANONICAL TARGETS (independent of pedestal, before any sensor correction):
    These are the intended raw histogram bin targets at each lux level,
    calibrated to produce natural exposure on standard sRGB displays.
    They are NOT the current YAML values -- this function always computes
    from here, making it idempotent regardless of what the YAML currently holds.

    After pedestal subtraction in processStats() the IPA sees net bins.
    The canonical raw bins minus pedestal gives the net target:
      new_t = max(MIN_Y_TARGET, (canonical_bins - pedestal) * ir_factor / HIST_MAX)
    """
    # Canonical raw histogram bin targets keyed by lux value.
    # Matched by lux value from the YAML, not by list position, so the
    # function is robust to YAML tables with different numbers of breakpoints.
    # For lux values not in the table, the nearest neighbour is used.
    # Bins before pedestal subtraction (raw sensor scale):
    #   dark/candle (1-10 lux):   41 bins  (0.08 * 511)
    #   dim indoor (50 lux):      61 bins  (0.12 * 511)
    #   office (100 lux):         77 bins  (0.15 * 511)
    #   bright indoor (300 lux):  92 bins  (0.18 * 511 = 18% grey)
    #   well-lit (500 lux):      102 bins  (0.20 * 511)
    #   outdoor overcast (1k):   128 bins  (0.25 * 511)
    #   outdoor cloudy (3k):     153 bins  (0.30 * 511)
    #   outdoor bright (5k+):    179 bins  (0.35 * 511)
    CANONICAL = {
        1:     41.0,
        10:    41.0,
        50:    61.0,
        100:   77.0,
        300:   92.0,
        500:   102.0,
        1000:  128.0,
        3000:  153.0,
        5000:  179.0,
        20000: 179.0,
    }
    canon_lux_sorted = sorted(CANONICAL.keys())

    def canonical_bins_for(lux):
        """Return canonical bins for lux, using nearest-neighbour lookup."""
        if lux in CANONICAL:
            return CANONICAL[lux]
        # Find nearest lux key
        nearest = min(canon_lux_sorted, key=lambda k: abs(k - lux))
        logger.debug(f'  y_target lux={lux}: not in canonical table, '
                     f'using nearest key {nearest} '
                     f'({CANONICAL[nearest]:.0f} raw bins)')
        return CANONICAL[nearest]

    raw = list(agc['y_target'])
    if len(raw) < 2 or len(raw) % 2 != 0:
        logger.warning(f'y_target has unexpected length {len(raw)}, skipping')
        return

    old_values = raw[:]
    new_values = raw[:]
    changed = False

    # Match by lux value: raw[i*2] is the lux key, raw[i*2+1] is the target
    for i in range(0, len(raw), 2):
        lux_val = float(raw[i])
        t_old = float(raw[i + 1])
        c_bins = canonical_bins_for(lux_val)
        net_bins = (c_bins - pedestal) * ir_factor
        if net_bins <= 0:
            logger.warning(f'  y_target lux={lux_val}: canonical {c_bins:.0f} bins '
                           f'minus pedestal {pedestal:.0f} bins = {net_bins:.1f} <= 0. '
                           f'Pedestal may be too high for this lux point. '
                           f'Using floor value {MIN_Y_TARGET}.')
        t_new = max(MIN_Y_TARGET, net_bins / HIST_MAX)
        t_new = round(t_new, 4)
        new_values[i + 1] = t_new
        if abs(t_new - t_old) > 0.0005:
            changed = True

    if changed:
        # Warn if any target changes by >30% — large changes indicate either
        # a significant pedestal correction (expected after BLC redo) or an
        # accidental recalc run on a manually-tuned table. The canonical table
        # is correct for a standard IMX219 at 64 DN pedestal; large deviations
        # suggest the YAML was manually tuned and recalc will overwrite that work.
        for i in range(0, len(raw), 2):
            t_old = float(old_values[i + 1]) if old_values[i + 1] is not None else 0.0
            t_new = float(new_values[i + 1])
            if t_old > 0 and abs(t_new - t_old) / t_old > 0.30:
                logger.warning(
                    f'  y_target lux={raw[i]}: large change '
                    f'{t_old:.4f} -> {t_new:.4f} '
                    f'({(t_new-t_old)/t_old*100:+.0f}%%). '
                    f'If you have manually tuned this value, recalc is overwriting it. '
                    f'This is expected after a BLC change; otherwise re-check your '
                    f'black_level value in the YAML.')

        # Preserve ruamel flow style if it was a CommentedSeq
        try:
            from ruamel.yaml.comments import CommentedSeq
            seq = CommentedSeq(new_values)
            seq.fa.set_flow_style()
        except Exception:
            seq = new_values
        agc['y_target'] = seq
        changes['Agc.y_target'] = (old_values, new_values)

        # Log the table diff
        logger.info('  y_target recalculated:')
        logger.info(f'    {"lux":>8}  {"old":>7}  {"new":>7}  {"delta_bins":>12}')
        for i in range(0, len(raw), 2):
            lux = raw[i]
            t_old = old_values[i+1]
            t_new = new_values[i+1]
            delta = (t_new - t_old) * HIST_MAX
            logger.info(f'    {lux:>8}  {t_old:>7.4f}  {t_new:>7.4f}  {delta:>+12.1f}')


def _get_algo(data, name):
    for item in data.get('algorithms', []):
        if name in item:
            return item[name]
    return None
