#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# Tuning script for Microchip ISC (SAMA7G5 / SAMA5D2).
# Follows the same offline-only structure as utils/tuning/rkisp1.py and
# utils/raspberrypi/ctt/ctt.py.
#
# Offline workflow:
#
#   1. Deploy the board capture helper once:
#        scp utils/tuning/mchp_capture.sh root@<board>:/tmp/
#
#   2. Capture at each CCT point on the board:
#        /tmp/mchp_capture.sh -c 3200 -l <lux>
#        /tmp/mchp_capture.sh -c 4500 -l <lux>
#        /tmp/mchp_capture.sh -c 6500 -l <lux>
#
#   3. Copy captures to host:
#        scp -r root@<board>:/tmp/calib/ ./captures/
#
#   4. Run calibration tool offline:
#        python3 microchip_isc.py -i ./captures/ -o imx219.yaml
#        python3 microchip_isc.py -i ./captures/ -o imx219.yaml \
#            --modules ct ccm awb lux recalc
#        python3 microchip_isc.py -i ./captures/ -o imx219.yaml --macbeth
#
# Image filename tags (case-insensitive, all optional):
#   ct<K>       True CCT in Kelvin    e.g. ct2860K
#   lux<N>      Scene lux             e.g. lux400
#   exp<N>us    Exposure in us        e.g. exp31200us
#   gain<N>     Analogue gain         e.g. gain1.0

import argparse
import json
import logging
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import coloredlogs

# Add parent dir to path for libtuning imports
sys.path.insert(0, str(Path(__file__).parent))

try:
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedSeq, CommentedMap
except ImportError:
    print('ERROR: ruamel.yaml required: pip3 install ruamel.yaml')
    sys.exit(1)

from microchip_isc.modules.image import IscImage
from microchip_isc.modules.ct_correction import calibrate_ct_correction
from microchip_isc.modules.ccm import calibrate_ccm
from microchip_isc.modules.awb_gains import calibrate_awb_gains
from microchip_isc.modules.lux import calibrate_lux
from microchip_isc.modules.recalc import recalculate
from microchip_isc.modules.factory_defaults import factory_reset, calibration_health_score

coloredlogs.install(level=logging.INFO,
                    fmt='%(name)s %(levelname)s %(message)s')
logger = logging.getLogger('mchp_isc_tuning')


# Mapping from CCT range to YAML matrix key name
_CT_NAME_MAP = [
    (7000, 9999, 'cloudy_daylight'),
    (6000, 6999, 'daylight'),
    (4500, 5999, 'daylight_fluorescent'),
    (3500, 4499, 'fluorescent'),
    (2800, 3499, 'tungsten'),
    (2500, 2799, 'candlelight'),
    (1000, 2499, 'deep_tungsten'),
]


def _ct_key_name(ct):
    for lo, hi, name in _CT_NAME_MAP:
        if lo <= ct <= hi:
            return name
    return f'ct_{ct}K'


def load_yaml(path):
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.width = 120
    with open(path, 'r') as f:
        return yaml.load(f), yaml


def save_yaml(path, data, yaml_inst, backup=True):
    if backup:
        bak = str(path) + f'.bak.{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        shutil.copy2(path, bak)
        logger.info(f'Backup: {bak}')
    with open(path, 'w') as f:
        yaml_inst.dump(data, f)
    logger.info(f'Wrote: {path}')


def get_algo(data, name):
    for item in data.get('algorithms', []):
        if name in item:
            return item[name]
    return None


class CalibCapture:
    """
    One calibration capture: a JPEG visual reference + pre-WB histogram
    channel means from the JSON sidecar.

    This is the primary calibration data object for AWB, CT and lux modules.
    It does NOT hold pixel data — all calibration inputs come from
    IscHistogramMeans metadata (pre-WB, pre-DPC histogram tap).

    Attributes:
        path      : Path to the JPEG file
        name      : filename stem
        ct        : true CCT in K (from filename tag or sidecar true_ct)
        lux       : measured lux (from filename tag or sidecar lux)
        hist      : dict with R, GR, GB, B histogram means (0-511 net bins)
        exposure_us: exposure time in µs (from sidecar)
        gain      : analogue gain (linear, from sidecar)
        awb_locked: True if AWB was locked at capture time
        ae_converged: True if AE was converged at capture time
    """
    def __init__(self, path):
        self.path = Path(path)
        self.name = self.path.stem
        self.ct   = None
        self.lux  = None
        self.hist = None
        self.exposure_us  = None
        self.gain         = None
        self.awb_locked   = False
        self.ae_converged = False

        # Parse filename tags first (ct<K>, lux<N>, exp<N>us, gain<N>)
        ct_m = re.search(r'ct(\d+)K', self.name, re.IGNORECASE)
        if ct_m:
            self.ct = float(ct_m.group(1))
        lux_m = re.search(r'lux(\d+)', self.name, re.IGNORECASE)
        if lux_m:
            self.lux = float(lux_m.group(1))

        # Load JSON sidecar (same stem, .json)
        json_path = self.path.with_suffix('.json')
        if json_path.exists():
            try:
                with open(json_path) as f:
                    meta = json.load(f)
                # Sidecar overrides filename tags only when tag is absent
                if self.ct is None and meta.get('true_ct'):
                    self.ct = float(meta['true_ct'])
                if self.lux is None and meta.get('lux'):
                    self.lux = float(meta['lux'])
                # Histogram means — the core calibration input
                h = meta.get('hist_means')
                if h and all(k in h for k in ('R', 'GR', 'GB', 'B')):
                    self.hist = {
                        'R':  float(h['R']),
                        'GR': float(h['GR']),
                        'GB': float(h['GB']),
                        'B':  float(h['B']),
                    }
                self.exposure_us  = meta.get('exposure_us')
                self.gain         = meta.get('analogue_gain')
                self.awb_locked   = bool(meta.get('awb_locked', False))
                self.ae_converged = bool(meta.get('ae_converged', False))
            except Exception as e:
                logger.warning(f'{self.name}: failed to load sidecar: {e}')
        else:
            logger.warning(f'{self.name}: no JSON sidecar — '
                           f'histogram means unavailable for calibration. '
                           f'Capture images using mchp_capture.sh on the board, then copy to --input.')

    @property
    def valid(self):
        """True if this capture has histogram means and a CT tag."""
        return self.hist is not None and self.ct is not None


def load_images(image_dir, macbeth_mode=False):
    """
    Load calibration images from image_dir.

    JPEG/JPG files → CalibCapture objects (AWB/CT/lux calibration)
    DNG/TIFF/RAW files → IscImage objects (CCM/BLC calibration)

    Returns a mixed list. Calibration modules filter by type using
    isinstance(img, CalibCapture) or isinstance(img, IscImage).
    """
    calib_exts = {'.jpg', '.jpeg'}
    raw_exts   = {'.dng', '.tiff', '.tif', '.raw', '.pgm'}
    all_exts   = calib_exts | raw_exts

    paths = sorted(
        (p for p in Path(image_dir).iterdir() if p.suffix.lower() in all_exts),
        key=lambda p: p.stat().st_mtime
    )

    if not paths:
        logger.error(f'No calibration images found in {image_dir} '
                     f'(expected .jpg from mchp_capture.sh, or .pgm/.dng/.raw for CCM/BLC)')
        return []

    # Warn if images span more than 24 hours -- likely mixing sessions.
    # Use the timestamp from the filename (YYYYMMDD_HHMMSS suffix added by
    # capture_on_board) rather than st_mtime which is reset on file copy.
    if len(paths) > 1:
        from datetime import datetime as _dt
        _ts_re = re.compile(r'(\d{8}_\d{6})')
        timestamps = []
        for p in paths:
            m = _ts_re.search(p.stem)
            if m:
                try:
                    timestamps.append(_dt.strptime(m.group(1), '%Y%m%d_%H%M%S'))
                except ValueError:
                    pass
        if len(timestamps) >= 2:
            span_hours = (max(timestamps) - min(timestamps)).total_seconds() / 3600
            if span_hours > 24:
                logger.warning(
                    f'Images in {image_dir} span {span_hours:.0f} hours '
                    f'(oldest: {paths[0].name}, newest: {paths[-1].name}). '
                    f'Mixing captures from different sessions may corrupt '
                    f'calibration results. Use a fresh directory per session.')

    images = []
    n_calib = 0
    n_raw = 0
    for p in paths:
        try:
            if p.suffix.lower() in calib_exts:
                img = CalibCapture(str(p))
                n_calib += 1
            else:
                img = IscImage(str(p))
                n_raw += 1
            images.append(img)
        except Exception as e:
            logger.warning(f'Failed to load {p.name}: {e}')

    logger.info(f'Loaded {n_calib} JPEG captures + {n_raw} RAW/DNG files '
                f'from {image_dir}')

    # Warn if JPEG captures are missing JSON sidecars (no histogram means)
    missing_sidecar = [img for img in images
                       if isinstance(img, CalibCapture) and img.hist is None]
    if missing_sidecar:
        logger.warning(
            f'{len(missing_sidecar)} JPEG capture(s) have no JSON sidecar '
            f'(missing histogram means): '
            f'{[img.name for img in missing_sidecar[:3]]}{"..." if len(missing_sidecar)>3 else ""}. '
            f'These will be skipped by AWB/CT/lux calibration modules. '
            f'Use mchp_capture.sh on the board — it writes the JSON sidecar automatically.')

    if macbeth_mode:
        logger.info('Macbeth mode: enter chart corners for DNG colorchecker images')
        for img in images:
            if not isinstance(img, IscImage):
                continue
            if not img.name.lower().startswith('colorchecker'):
                continue
            print(f'\n{img.name}  CT={img.ct}K')
            use = input('  Enter Macbeth corners? [y/N] ').strip().lower()
            if use != 'y':
                continue
            corners = []
            for label in ('top-left', 'top-right', 'bottom-left', 'bottom-right'):
                while True:
                    try:
                        val = input(f'    {label} (x,y): ').strip()
                        x, y = map(int, val.split(','))
                        corners.append((x, y))
                        break
                    except Exception:
                        print('    Invalid format')
            if len(corners) == 4:
                img.set_macbeth_patches(corners)

    return images


def run_ct(images, data, yaml_inst, output_path):
    """Calibrate ct_correction_pwl."""
    logger.info('=== CT correction calibration ===')
    awb = get_algo(data, 'Awb')
    if awb is None:
        logger.error('No Awb block in YAML')
        return

    # Get existing ct_color_matrix for Robertson
    ct_matrix = None
    raw_matrix = awb.get('ct_color_matrix')
    if raw_matrix:
        ct_matrix = list(raw_matrix)
    else:
        logger.warning(
            'No ct_color_matrix in YAML. Robertson CT estimation uses identity '
            '(sensor ≈ XYZ), producing up to +47%% systematic error at 2856K '
            'for IMX219. The ct_correction_pwl will compensate but will be '
            'specific to this (identity) Robertson calibration. '
            'IMPORTANT: If you later run CCM calibration, re-run CT calibration '
            'afterwards with the new ct_color_matrix for best PWL accuracy. '
            'Workflow order: blc → ct → awb → ccm → ct (re-run) → recalc.'
        )

    ct_images = [img for img in images if img.ct is not None]
    if len(ct_images) < 2:
        logger.error('Need at least 2 images with ct<K> tag for CT calibration')
        return

    pwl = calibrate_ct_correction(ct_images, ct_matrix)
    if pwl is None:
        return

    seq = CommentedSeq(pwl)
    seq.fa.set_flow_style()
    awb['ct_correction_pwl'] = seq
    awb.yaml_set_comment_before_after_key(
        'ct_correction_pwl',
        before=(f'Robertson CT correction PWL — calibrated '
                f'{datetime.now().strftime("%Y-%m-%d")}.\n'
                f'      # {len(pwl)//2} calibration points.'))
    save_yaml(output_path, data, yaml_inst)


def run_ccm(images, data, yaml_inst, output_path):
    """Calibrate CCM matrices."""
    logger.info('=== CCM calibration ===')
    ccm_block = get_algo(data, 'Ccm')
    if ccm_block is None:
        logger.error('No Ccm block in YAML')
        return

    macbeth_images = [img for img in images if getattr(img, 'macbeth', False)]
    if not macbeth_images:
        logger.error('No images with Macbeth patch data. Run with --macbeth.')
        return

    results = calibrate_ccm(macbeth_images)
    if not results:
        return

    if 'matrices' not in ccm_block:
        ccm_block['matrices'] = CommentedMap()

    for ct_k, result in sorted(results.items()):
        key = _ct_key_name(ct_k)
        flat = [v for row in result['matrix'] for v in row]
        flat_r = [round(v, 4) for v in flat]
        seq = CommentedSeq(flat_r)
        seq.fa.set_flow_style()

        entry = CommentedMap()
        entry['ct'] = ct_k
        entry['ccm'] = seq
        entry.yaml_set_comment_before_after_key(
            'ccm',
            before=(f'Calibrated {datetime.now().strftime("%Y-%m-%d")} '
                    f'mean_dE={result["de"]:.3f}'))
        ccm_block['matrices'][key] = entry

    save_yaml(output_path, data, yaml_inst)


def run_awb(images, data, yaml_inst, output_path):
    """Calibrate AWB reference gains table."""
    logger.info('=== AWB reference gains calibration ===')
    awb = get_algo(data, 'Awb')
    if awb is None:
        logger.error('No Awb block in YAML')
        return

    results = calibrate_awb_gains(images)
    if not results:
        return

    rows = []
    for e in results:
        row = CommentedMap({'ct': e['ct'], 'r': e['r'], 'b': e['b']})
        row.fa.set_flow_style()
        rows.append(row)

    awb['reference_gains_table'] = rows
    awb.yaml_set_comment_before_after_key(
        'reference_gains_table',
        before=(f'Per-CT AWB seed gains -- calibrated '
                f'{datetime.now().strftime("%Y-%m-%d")}.\n'
                f'      # Method: grey card under single-illuminant lightbox, '
                f'{len(results)} CT points.\n'
                f'      # Formula: rGain=meanGR/meanR, bGain=meanGR/meanB '
                f'(matches IPA awb.cpp).'))
    save_yaml(output_path, data, yaml_inst)


def run_lux(images, data, yaml_inst, output_path):
    """Calibrate lux estimator reference."""
    logger.info('=== Lux estimator calibration ===')
    agc = get_algo(data, 'Agc')
    if agc is None:
        logger.error('No Agc block in YAML')
        return

    result = calibrate_lux(images)
    if not result:
        return

    if 'lux_reference' not in agc:
        agc['lux_reference'] = CommentedMap()

    lr = agc['lux_reference']
    lr['reference_Y'] = result['reference_Y']
    lr['reference_exposure_us'] = result['reference_exposure_us']
    lr['reference_gain'] = result['reference_gain']
    lr['reference_lux'] = result['reference_lux']

    save_yaml(output_path, data, yaml_inst)


def run_blc(images, data, yaml_inst, output_path):
    """
    Calibrate black level correction using a 3-tier approach that mirrors
    RkISP1's architecture (camera_sensor_helper.cpp):

      Tier 1 — Sensor datasheet value (primary, most reliable):
        The IMX219 datasheet §7.3 specifies pedestal = 64 DN (10-bit).
        This is a silicon property that does not vary with temperature,
        illumination, or individual unit (typical spread: ±2 DN).
        RPi and RkISP1 both use this approach — no dark frame needed.

      Tier 2 — Dark frame measurement (verification + NIR contamination check):
        A dark frame is captured and measured NOT to get a better value
        (the datasheet is already authoritative), but to DETECT NIR
        contamination — the case where R or B channels read significantly
        higher than GR/GB because light is leaking through the lens cover.
        If contamination is detected, BLC is REFUSED and the user must
        re-cover the lens properly before proceeding.

      Tier 3 — YAML current value (fallback when no SSH capture):
        If no dark images are available, the current YAML value is kept
        and a warning is printed. The user can still run AWB/CT/CCM.

    This approach avoids the EPIPE crash risk of the previous dark frame
    path while correctly catching the NIR contamination failure mode.
    """
    import numpy as np

    # ---- Known sensor datasheet pedestals (10-bit DN) ----
    # Mirrors RkISP1 CameraSensorHelperImx219::blackLevel_ = 4096 (16-bit)
    # = 64 DN in 10-bit. Add other sensors here as needed.
    _SENSOR_DATASHEET_BL = {
        'imx219':      64,   # IMX219 datasheet §7.3
        'imx219_noir': 64,   # Same silicon, NoIR filter doesn't change pedestal
        'ov5647':      16,   # OV5647 datasheet
        'imx477':     256,   # IMX477 datasheet (12-bit sensor, 64 in 12-bit)
    }
    # Detect sensor from output YAML filename stem
    sensor_key = Path(output_path).stem.lower()
    datasheet_bl = _SENSOR_DATASHEET_BL.get(sensor_key, 64)

    logger.info('=== Black level calibration ===')
    logger.info(f'Sensor: {sensor_key}  →  datasheet pedestal = {datasheet_bl} DN')
    logger.info('Approach: datasheet value is authoritative (matches RPi/RkISP1). '
                'Dark frame is used ONLY for NIR contamination detection, '
                'not to compute the black level value.')

    blc = get_algo(data, 'BlackLevelCorrection')
    if blc is None:
        logger.error('No BlackLevelCorrection block in YAML')
        return

    # ---- Tier 2: Dark frame NIR contamination check ----
    dark = [img for img in images
            if (img.lux is not None and img.lux < 0.1)
            or img.name.startswith('dark_')]

    if not dark:
        # No dark frames — use datasheet value with a warning.
        # This matches RkISP1 behaviour when no camera helper exists.
        current = int(blc.get('black_level', datasheet_bl) or datasheet_bl)
        if current == datasheet_bl:
            logger.info(f'No dark frame available. '
                        f'Confirming datasheet value black_level={datasheet_bl} DN.')
        else:
            logger.warning(f'No dark frame available. '
                           f'Current YAML value ({current} DN) differs from '
                           f'datasheet ({datasheet_bl} DN). '
                           f'Using datasheet value. To verify, capture a dark '
                           f'frame with the lens fully covered (black tape).')
        # Write datasheet value and return
        blc['black_level']    = datasheet_bl
        blc['black_level_r']  = datasheet_bl
        blc['black_level_gr'] = datasheet_bl
        blc['black_level_gb'] = datasheet_bl
        blc['black_level_b']  = datasheet_bl
        save_yaml(output_path, data, yaml_inst)
        return

    logger.info(f'Found {len(dark)} dark frame(s) — checking for NIR contamination...')

    bl_r, bl_gr, bl_gb, bl_b = [], [], [], []
    for img in dark:
        if img.channels is None:
            continue
        means = [float(ch.mean()) for ch in img.channels]
        # channels are in histogram bin units (raw_pixel / 2, no BLC subtracted
        # since sidecar has black_level=0). Multiply by 2 to get 10-bit DN.
        bl_r.append(means[0] * 2.0)
        bl_gr.append(means[1] * 2.0)
        bl_gb.append(means[2] * 2.0)
        bl_b.append(means[3] * 2.0)

    if not bl_r:
        logger.error('Dark frames found but had no valid channel data. '
                     'Check image format and sidecar JSON.')
        return

    import numpy as _np
    r  = float(_np.mean(bl_r))
    gr = float(_np.mean(bl_gr))
    gb = float(_np.mean(bl_gb))
    b  = float(_np.mean(bl_b))
    avg = (r + gr + gb + b) / 4.0
    gr_ref = max((gr + gb) / 2.0, 1.0)  # GR/GB average as NIR-safe reference

    logger.info(f'Dark frame means (10-bit DN):  R={r:.1f}  GR={gr:.1f}  '
                f'GB={gb:.1f}  B={b:.1f}  avg={avg:.1f}')
    logger.info(f'Datasheet expected: {datasheet_bl} DN ± 3 DN per channel')

    # ---- Contamination detection ----
    # GR and GB are NIR-insensitive reference channels.
    # R and B on NoIR sensors will read high if NIR leaks through the cover.
    # Threshold: >20% above GR/GB reference = contaminated.
    contaminated = False
    for ch_name, ch_val in [('R', r), ('B', b)]:
        ratio = ch_val / gr_ref
        if ratio > 1.20:
            logger.error(
                f'NIR CONTAMINATION DETECTED: {ch_name}={ch_val:.1f} DN is '
                f'{ratio:.2f}× the GR/GB reference ({gr_ref:.1f} DN). '
                f'Your lens cover transmits near-infrared light (850 nm+). '
                f'This will corrupt AWB B gain and CCM calibration. '
                f'Action: cover lens with BLACK electrical tape (not cello tape, '
                f'not plastic cap — both transmit NIR). '
                f'BLC NOT written. Re-run after fixing the cover.')
            contaminated = True
        elif ratio > 1.10:
            logger.warning(
                f'{ch_name}={ch_val:.1f} DN is {ratio:.2f}× GR/GB reference. '
                f'Marginal NIR leak. Add more layers of dark material.')

    if contaminated:
        logger.error('BLC calibration ABORTED due to NIR contamination. '
                     'Fix lens cover and retry. No changes written to YAML.')
        return

    # ---- Frame sanity check ----
    if avg < 30:
        logger.error(f'Dark frame avg={avg:.1f} DN is below 30. '
                     f'Sensor may not be active or format is wrong. '
                     f'Check: v4l2-ctl format, sensor power, reboot board.')
        return
    if avg > 150:
        logger.error(f'Dark frame avg={avg:.1f} DN exceeds 150. '
                     f'The frame is NOT dark — light is leaking in. '
                     f'Check that the lens is completely sealed.')
        return

    # ---- Compare measurement to datasheet ----
    deviation = abs(avg - datasheet_bl)
    if deviation > 8:
        logger.warning(
            f'Measured avg={avg:.1f} DN deviates {deviation:.1f} DN from '
            f'datasheet ({datasheet_bl} DN). This may indicate '
            f'partial light leakage or a different sensor variant. '
            f'Using datasheet value ({datasheet_bl} DN) as it is more '
            f'reliable than a single dark frame measurement.')
    else:
        logger.info(f'Measured avg={avg:.1f} DN matches datasheet '
                    f'{datasheet_bl} DN (deviation={deviation:.1f} DN) ✓')

    # ---- Tier 1: Write datasheet value (authoritative) ----
    # We always use the datasheet value, not the measured value.
    # Rationale: the datasheet specifies a silicon property to ±1 DN.
    # A single dark frame measurement has ±3 DN RMS noise even with perfect
    # dark conditions, plus any residual thermal noise at room temperature.
    # RPi and RkISP1 both use datasheet values for exactly this reason.
    # The dark frame served its purpose: confirming no NIR contamination.
    blc['black_level']    = datasheet_bl
    blc['black_level_r']  = datasheet_bl
    blc['black_level_gr'] = datasheet_bl
    blc['black_level_gb'] = datasheet_bl
    blc['black_level_b']  = datasheet_bl

    logger.info(f'BLC written: all channels = {datasheet_bl} DN (IMX219 datasheet). '
                f'Dark frame confirmed no NIR contamination ✓')
    logger.info('Note: per-channel BLC values are all equal because the ISC DPC '
                'block on SAMA7G5 applies a single global offset (ISC_DPC_CFG.BLOFST). '
                'Per-channel offsets are only needed on SAMA5D2 (hasDpc=false) '
                'via WB offset registers. Calibrating per-channel differences '
                'requires optical dark columns which IMX219 does not expose.')

    save_yaml(output_path, data, yaml_inst)


# ---------------------------------------------------------------------------
# SSH capture helpers (replaces mchp_isc_calibrate.py)
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Microchip ISC IPA tuning tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Modules and required targets:
  blc     Lens capped, no illumination. Captures via v4l2-ctl (no IPA).
  ct      18% neutral grey card, >70% frame, single illuminant per capture.
          Requires >=2 CCT points. CCT must be lightbox-rated, not IPA estimate.
  awb     Same grey card captures as ct. Can run together: --modules ct awb
  lux     Any scene with known incident illuminance (lux meter required).
          If captured alongside ct/awb, no separate capture needed.
  ccm     Macbeth ColorChecker chart, all 24 patches visible, no glare.
          Captured separately from grey card at same CCT points. Use --macbeth.
  recalc  No capture needed. Recomputes pedestal-derived constants from
          black_level in YAML. Always run after blc.

Filename tags (used in offline mode):
  ct<K>   Lightbox-rated CCT in Kelvin  e.g. ct2856K
  lux<N>  Incident illuminance in lux   e.g. lux400
  grey_card / colorchecker prefix identifies target type

Offline workflow:
  # Capture on board manually, copy files to host, then:
  python3 microchip_isc.py -i ./images/ -o imx219_noir.yaml --noir \\
      --modules ct awb lux recalc

''')

    parser.add_argument('-i', '--input', required=False, default=None,
        help='Directory for RAW calibration images (source or destination). '
             'Not required when using --health or --dry-run.')
    parser.add_argument('-o', '--output', required=True,
        help='YAML tuning file to update')
    parser.add_argument('--modules', nargs='+',
        choices=['ct', 'ccm', 'awb', 'lux', 'blc', 'recalc', 'all'],
        default=['all'],
        help='Calibration modules to run (default: all). '
             'recalc: recompute all pedestal-derived constants from black_level.')
    parser.add_argument('--noir', action='store_true',
        help='Sensor is NoIR (no IR cut filter). Applies 7%% IR brightness '
             'reduction to target_brightness and y_target.')
    parser.add_argument('--macbeth', action='store_true',
        help='Interactively enter ColorChecker chart corners for CCM calibration')
    parser.add_argument('--no-backup', action='store_true',
        help='Do not create .bak backup before writing YAML')
    parser.add_argument('--no-prompt', action='store_true',
        help='Skip interactive preamble and lens-cap prompt (for use from '
             'wrapper scripts that handle user interaction externally)')
    parser.add_argument('--factory-reset', action='store_true',
        help='RECOVERY ONLY: restore physics-derived factory defaults to the '
             'YAML. Use this ONLY when a previous calibration produced garbage '
             'results and you want to return to a known-safe state before '
             're-calibrating. Do NOT use as a routine pre-calibration step — '
             'the standard workflow (mchp_capture.sh + microchip_isc.py) handles starting state '
             'correctly without a reset. '
             'Writes: BLC=64 DN, AWB gains from Planckian locus, '
             'CCM=identity, lux_reference=uncalibrated. '
             'A timestamped backup is always created.')
    parser.add_argument('--health', action='store_true',
        help='Print calibration health score (0–100) for the current YAML '
             'and exit. Does not require --input or --modules.')
    parser.add_argument('--post-cal-image', dest='post_cal_image',
        default=None, metavar='IMAGE',
        help='JPEG image captured after calibration (grey card at reference '
             'CCT) to verify AWB convergence.  R/G and B/G are measured and '
             'scored in the health report.  Use with --health to check the '
             'result after running mchp_capture.sh on the calibrated board.')
    parser.add_argument('--dry-run', action='store_true',
        help='Validate all dependencies (Python packages, '
             'YAML structure) and print a pre-flight check. Does not capture '
             'or modify anything.')

    args = parser.parse_args()

    if not Path(args.output).exists():
        logger.error(f'YAML file not found: {args.output}')
        sys.exit(1)

    # --health: just print score and exit
    if getattr(args, 'health', False):
        data, _ = load_yaml(args.output)
        after_stats = None
        if getattr(args, 'post_cal_image', None):
            try:
                from PIL import Image as _PILImage
                import numpy as _np
                _img = _PILImage.open(args.post_cal_image).convert('RGB')
                _a = _np.array(_img, dtype=_np.float32)
                _r, _g, _b = _a[:,:,0].mean(), _a[:,:,1].mean(), _a[:,:,2].mean()
                if _g > 0:
                    after_stats = {'rg': _r / _g, 'bg': _b / _g}
                    logger.info(
                        f'Post-cal image: R/G={after_stats["rg"]:.3f} '
                        f'B/G={after_stats["bg"]:.3f}  ({args.post_cal_image})')
            except Exception as e:
                logger.warning(f'Could not read post-cal image: {e}')
        score, report = calibration_health_score(data, after_stats)
        grade = ('A' if score >= 90 else 'B' if score >= 75 else
                 'C' if score >= 60 else 'D' if score >= 45 else 'F')
        print(f'\nCalibration Health Score: {score}/100  [{grade}]')
        print(f'YAML: {args.output}\n')
        for line in report:
            print(line)
        print()
        if score < 60:
            print('  Action required: run full calibration sequence.')
            print('  Capture images using mchp_capture.sh, then run microchip_isc.py --input <dir>')
        elif score < 80:
            print('  Good baseline. Run missing modules to reach 9/10.')
        else:
            print('  Well calibrated. Verify visually at each CCT point.')
        print()
        sys.exit(0)

    # --dry-run: validate everything without capturing or writing
    if getattr(args, 'dry_run', False):
        _dry_run_check(args)
        sys.exit(0)

    modules = set(args.modules)
    if 'all' in modules:
        modules = {'ct', 'ccm', 'awb', 'lux', 'blc', 'recalc'}

    # --factory-reset: restore safe defaults before calibration
    if getattr(args, 'factory_reset', False):
        logger.info('=== Factory reset ===')
        data, yaml_inst = load_yaml(args.output)
        sensor_model = Path(args.output).stem  # e.g. 'imx219_noir'
        is_noir = getattr(args, 'noir', False)
        changes = factory_reset(data, sensor_model=sensor_model, is_noir=is_noir)
        if changes:
            save_yaml(args.output, data, yaml_inst, backup=not getattr(args, 'no_backup', False))
            logger.info(f'Factory reset complete ({len(changes)} modules reset)')
        else:
            logger.info('Factory reset: no changes needed')

    logger.info(f'Input:   {args.input}')
    logger.info(f'Output:  {args.output}')
    logger.info(f'Modules: {", ".join(sorted(modules))}')

    # Warn if input directory already contains stale images from a previous
    # session. The timestamp suffix in filenames makes this detectable.
    # Stale images mixed with new captures corrupt calibration by averaging
    # old (possibly wrong) channel means with new ones at the same CT bucket.
    if Path(args.input).is_dir():
        existing = list(Path(args.input).glob('*.raw')) + \
                   list(Path(args.input).glob('*.json'))
        if existing:
            logger.warning(
                f'{args.input} already contains {len(existing)} file(s) from '
                f'a previous session. New captures will be added to the same '
                f'directory and merged with old data during calibration. '
                f'To avoid cross-session contamination, either:\n'
                f'  (a) Delete or move the old files before running, or\n'
                f'  (b) Use a new --input directory for this session.\n'
                f'  Old files: {", ".join(f.name for f in existing[:3])}'
                + (f' ... (+{len(existing)-3} more)' if len(existing) > 3 else ''))

    # Offline mode requires input directory to exist with images
    if not Path(args.input).is_dir():
        logger.error(f'Input directory not found: {args.input}')
        sys.exit(1)

    # Load images from input directory
    images = load_images(args.input, macbeth_mode='ccm' in modules and args.macbeth)
    if not images:
        logger.error('No calibration images found. '
                     'Run mchp_capture.sh on the board and copy captures to --input. See '
                     'DNG/TIFF/.raw files in the input directory.')
        sys.exit(1)

    # Load YAML
    data, yaml_inst = load_yaml(args.output)

    # Run requested modules in dependency order
    if 'blc' in modules:
        run_blc(images, data, yaml_inst, args.output)
        data, yaml_inst = load_yaml(args.output)

    if 'ct' in modules:
        run_ct(images, data, yaml_inst, args.output)
        data, yaml_inst = load_yaml(args.output)

    if 'awb' in modules:
        run_awb(images, data, yaml_inst, args.output)
        data, yaml_inst = load_yaml(args.output)

    if 'ccm' in modules:
        run_ccm(images, data, yaml_inst, args.output)
        data, yaml_inst = load_yaml(args.output)

    if 'lux' in modules:
        run_lux(images, data, yaml_inst, args.output)
        data, yaml_inst = load_yaml(args.output)

    # recalc always runs last: recomputes all constants derived from
    # black_level so they stay consistent with any blc module changes.
    if 'recalc' in modules:
        run_recalc(data, yaml_inst, args.output,
                   is_noir=getattr(args, 'noir', False))

    # Print calibration health score after every run so the user knows
    # what still needs to be calibrated and what the overall status is.
    data_final, _ = load_yaml(args.output)
    score, report = calibration_health_score(data_final)
    grade = ('A' if score >= 90 else 'B' if score >= 75 else
             'C' if score >= 60 else 'D' if score >= 45 else 'F')
    print(f'\n{"="*60}')
    print(f'  Calibration Health Score: {score}/100  [{grade}]')
    print(f'{"="*60}')
    for line in report:
        print(line)
    print()

    logger.info('Done.')


def _dry_run_check(args):
    """
    Pre-flight validation: check all dependencies without capturing or writing.
    Prints a checklist and exits cleanly if everything is OK.
    """
    ok_count = 0
    fail_count = 0

    def chk(label, condition, detail='', fatal=False):
        nonlocal ok_count, fail_count
        if condition:
            print(f'  ✓  {label}  {detail}')
            ok_count += 1
        else:
            sym = '✗' if fatal else '!'
            print(f'  {sym}  {label}  {detail}')
            fail_count += (1 if fatal else 0)

    print('\nPre-flight check:')

    # Python packages
    try:
        from ruamel.yaml import YAML as _Y; chk('ruamel.yaml installed', True)
    except ImportError:
        chk('ruamel.yaml installed', False, '→ pip3 install ruamel.yaml', fatal=True)
    try:
        import numpy as _n; chk(f'numpy installed', True, f'v{_n.__version__}')
    except ImportError:
        chk('numpy installed', False, '→ pip3 install numpy', fatal=True)
    try:
        import coloredlogs as _c; chk('coloredlogs installed', True)
    except ImportError:
        chk('coloredlogs installed', False, '→ pip3 install coloredlogs (non-fatal)')

    # YAML file
    yaml_exists = Path(args.output).exists()
    chk(f'YAML exists: {args.output}', yaml_exists, fatal=True)
    if yaml_exists:
        try:
            data, _ = load_yaml(args.output)
            chk('YAML parseable', True)
        except Exception as e:
            chk('YAML parseable', False, str(e), fatal=True)
            data = None
        if data:
            score, _ = calibration_health_score(data)
            chk(f'Current health score', True, f'{score}/100')

    # Input directory
    if hasattr(args, 'input') and args.input:
        inp = Path(args.input)
        chk(f'Input dir exists: {args.input}', inp.exists())
        if inp.exists():
            raws = list(inp.glob('*.raw'))
            chk(f'RAW files in input', len(raws) > 0, f'{len(raws)} found')

    print(f'\n  {ok_count} checks passed, {fail_count} fatal failures.')
    if fail_count > 0:
        print('  Fix fatal issues before running calibration.')
        sys.exit(1)
    else:
        print('  Pre-flight OK — ready to calibrate.')


def run_recalc(data, yaml_inst, output_path, is_noir=False):
    """
    Recompute all histogram-bin-level YAML constants from black_level.

    This must be the last module to run so it picks up any black_level
    changes made by the blc module.
    """
    logger.info('=== Recalculating pedestal-derived constants ===')
    changes = recalculate(data, is_noir=is_noir)
    if changes:
        for key, (old, new) in changes.items():
            logger.info(f'  {key}: {old} -> {new}')
        save_yaml(output_path, data, yaml_inst)
    else:
        logger.info('  No changes needed.')


if __name__ == '__main__':
    main()
