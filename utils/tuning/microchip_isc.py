#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# Tuning script for Microchip ISC (SAMA7G5 / SAMA5D2).
# Follows the same structure as utils/tuning/rkisp1.py.
#
# Two modes of operation:
#
#   1. Offline batch mode (default):
#      Process pre-captured RAW files from a directory.
#        python3 microchip_isc.py -i ./images/ -o imx219_noir.yaml
#        python3 microchip_isc.py -i images/ -o imx219_noir.yaml \
#            --modules ct ccm awb lux blc
#        python3 microchip_isc.py -i images/ -o imx219_noir.yaml --macbeth
#
#   2. SSH capture mode (--board):
#      Interactively trigger captures on the board via SSH, saving RAW files
#      with proper metadata tags in the filenames.
#        python3 microchip_isc.py --board root@10.40.24.60 --board-pass 123 \
#            -i ./calib_images/ -o imx219_noir.yaml --modules ct awb
#
#      The capture mode:
#        - Connects to the board via SSH
#        - Triggers mchpcam-still (JPEG, full ISC pipeline)
#        - Reads IscHistogramMeans from --metadata JSON (pre-WB histogram tap)
#          Falls back to ISC_AWB:DEBUG log for firmware older than v0.6.1
#        - Saves .jpg with metadata-tagged filenames
#        - Runs the requested calibration modules on the saved images
#        - BLC dark frame: captured with v4l2-ctl directly (no IPA running)
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
                           f'Use calib.sh to capture calibration images.')

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
    raw_exts   = {'.dng', '.tiff', '.tif', '.raw'}
    all_exts   = calib_exts | raw_exts

    paths = sorted(
        (p for p in Path(image_dir).iterdir() if p.suffix.lower() in all_exts),
        key=lambda p: p.stat().st_mtime
    )

    if not paths:
        logger.error(f'No calibration images found in {image_dir} '
                     f'(expected .jpg from calib.sh or .dng/.raw for CCM/BLC)')
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
            f'Use calib.sh to capture — it writes the JSON sidecar automatically.')

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

    macbeth_images = [img for img in images if img.macbeth]
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

def _ssh_run(board, password, cmd, timeout=90):
    """
    Run a command via SSH, return (stdout, stderr, rc).

    Uses Popen + reader threads instead of subprocess.run(capture_output=True).
    With capture_output=True the SSH pipe buffers (~64 KB) fill up when the
    remote side produces verbose debug output (ISC_AWB:DEBUG + ISC_AGC:DEBUG
    = hundreds of lines/second). When the buffer is full the remote process
    blocks writing, communicate() never returns, and the timeout fires as a
    spurious TimeoutExpired even though mchpcam-still is still running fine.

    Streaming with two reader threads drains both pipes continuously so the
    remote process never blocks, regardless of output volume.
    """
    import threading as _th

    if password:
        full = ['sshpass', '-p', password, 'ssh',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10',
                board, cmd]
    else:
        full = ['ssh', '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10', board, cmd]

    proc = subprocess.Popen(full, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)

    stdout_lines, stderr_lines = [], []

    def _drain(stream, buf):
        for line in stream:
            buf.append(line.rstrip('\n'))

    t_o = _th.Thread(target=_drain, args=(proc.stdout, stdout_lines), daemon=True)
    t_e = _th.Thread(target=_drain, args=(proc.stderr, stderr_lines), daemon=True)
    t_o.start(); t_e.start()

    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        t_o.join(timeout=3); t_e.join(timeout=3)
        logger.error(f'SSH command timed out after {timeout}s')
        return '\n'.join(stdout_lines), '\n'.join(stderr_lines), -1

    t_o.join(timeout=5); t_e.join(timeout=5)
    return '\n'.join(stdout_lines), '\n'.join(stderr_lines), proc.returncode


def _ssh_run_progress(board, password, cmd, timeout=180, label='Capturing'):
    """
    Run a long SSH command with a live single-line progress bar on stdout.

    Drains both stdout and stderr via background threads (same approach as
    _ssh_run) while rendering a live status bar every 250ms.

    Progress bar format (single overwriting line, no scroll):
      [████████████░░░░░░░░]  AE  87%  AWB r=1.43 b=1.19  CT=3842K  12s

    AE fill  : currentLuminance / effectiveTarget * 100  (ISC_AGC:DEBUG)
    AWB state: r/b gain values or locked checkmark        (ISC_AWB:DEBUG)
    CT       : Robertson CT estimate                      (informational)

    Returns (stdout_str, stderr_str, rc) -- identical to _ssh_run().
    """
    import threading as _th, time as _time, sys as _sys, re as _re

    if password:
        full = ['sshpass', '-p', password, 'ssh',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10', board, cmd]
    else:
        full = ['ssh', '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=10', board, cmd]

    S = dict(ae_lum=0.0, ae_tgt=0.0, ae_locked=False,
             awb_r=0.0, awb_b=0.0, awb_locked=False,
             ct=0, saved=False, start=_time.time())
    stdout_lines, stderr_lines = [], []

    _re_agc     = _re.compile(r'AGC.*luminance=([0-9.]+).*target=([0-9.]+)')
    _re_lock    = _re.compile(r'AGC stable.*lock=3/3')
    _re_awb     = _re.compile(r'AWB gains:.*R=([0-9.]+).*B=([0-9.]+)')
    _re_ct      = _re.compile(r'ct_smoothed=([0-9]+)')
    _re_ae_app  = _re.compile(r'AE locked\s*:\s*yes', _re.I)
    _re_awb_app = _re.compile(r'AWB locked\s*:\s*yes', _re.I)
    _re_saved   = _re.compile(r'Saved RAW:|Saved JPEG:')

    def _parse(line, is_err):
        (stderr_lines if is_err else stdout_lines).append(line.rstrip('\n'))
        m = _re_agc.search(line)
        if m:
            S['ae_lum'] = float(m.group(1))
            S['ae_tgt'] = float(m.group(2))
        if _re_lock.search(line):    S['ae_locked']  = True
        m = _re_awb.search(line)
        if m:
            S['awb_r'] = float(m.group(1))
            S['awb_b'] = float(m.group(2))
        m = _re_ct.search(line)
        if m: S['ct'] = int(m.group(1))
        if _re_ae_app.search(line):  S['ae_locked']  = True
        if _re_awb_app.search(line): S['awb_locked'] = True
        if _re_saved.search(line):
            S['saved'] = S['ae_locked'] = S['awb_locked'] = True

    def _drain(stream, is_err):
        for line in stream:
            _parse(line, is_err)

    BAR  = 24
    _G   = '\033[32m'; _Y = '\033[33m'; _C = '\033[36m'; _R = '\033[0m'
    _CLR = '\033[K'

    def _fmt_time(secs):
        """Format seconds as M:SS (e.g. 1:07) or s (e.g. 8s) for the bar."""
        s = int(secs)
        if s < 60:
            return f"{s}s"
        return f"{s//60}:{s%60:02d}"

    def _render():
        elapsed = _time.time() - S['start']
        ae_f    = min(S['ae_lum'] / S['ae_tgt'], 1.0) if S['ae_tgt'] > 0 else 0.0
        filled  = int(ae_f * BAR)
        bar     = '\u2588' * filled + '\u2591' * (BAR - filled)

        ae_s  = (f"{_G}AE  \u2713 {_R}" if S['ae_locked']
                 else f"AE {int(ae_f*100):3d}%")
        awb_s = (f"{_G}AWB \u2713 {_R}" if S['awb_locked']
                 else (f"AWB r={S['awb_r']:.2f} b={S['awb_b']:.2f}"
                       if S['awb_r'] > 0 else f"{_Y}AWB  \u2026 {_R}"))
        ct_s  = f"  CT={S['ct']}K" if S['ct'] > 0 else ''

        # ETA only once we have reliable rate data (after 3s, AE in progress)
        eta_s = ''
        if elapsed > 3 and 0.05 < ae_f < 0.99 and not S['ae_locked']:
            rate  = ae_f / elapsed
            if rate > 0:
                eta_s = f"  ETA ~{_fmt_time((1-ae_f)/rate)}"

        # Clock ticks every render call (250ms) — always moving
        clock = _fmt_time(elapsed)

        if S['saved']:
            _sys.stdout.write(
                f"\r  {_G}\u2713{_R}  [{_G}{'█'*BAR}{_R}]"
                f"  {_G}AE \u2713  AWB \u2713{_R}"
                f"  done {clock}{_CLR}")
        else:
            _sys.stdout.write(
                f"\r  {_C}\u00b7{_R}  [{bar}]"
                f"  {ae_s}  {awb_s}{ct_s}  {clock}{eta_s}{_CLR}")
        _sys.stdout.flush()

    _sys.stdout.write(f"\n  {label}\n")
    _render()

    proc = subprocess.Popen(full, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)
    t_o = _th.Thread(target=_drain, args=(proc.stdout, False), daemon=True)
    t_e = _th.Thread(target=_drain, args=(proc.stderr, True),  daemon=True)
    t_o.start(); t_e.start()

    deadline = _time.time() + timeout
    while proc.poll() is None:
        _render()
        _time.sleep(0.25)
        if _time.time() > deadline:
            proc.kill()
            _sys.stdout.write(f"\n  ERROR: capture timed out after {timeout}s\n")
            _sys.stdout.flush()
            t_o.join(timeout=3); t_e.join(timeout=3)
            return '\n'.join(stdout_lines), '\n'.join(stderr_lines), -1

    t_o.join(timeout=5); t_e.join(timeout=5)
    _render()
    _sys.stdout.write('\n')
    _sys.stdout.flush()
    rc = proc.returncode if proc.returncode is not None else -1
    return '\n'.join(stdout_lines), '\n'.join(stderr_lines), rc


def _scp_from_board(board, password, remote, local):
    """Copy a file from the board to the local machine."""
    if password:
        cmd = ['sshpass', '-p', password, 'scp',
               '-o', 'StrictHostKeyChecking=no',
               f'{board}:{remote}', str(local)]
    else:
        cmd = ['scp', '-o', 'StrictHostKeyChecking=no',
               f'{board}:{remote}', str(local)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    return r.returncode == 0


def _parse_awb_debug(log_lines):
    """
    DEPRECATED: AWB calibration data now comes from IscHistogramMeans
    vendor metadata (--metadata --metadata-format=json), not from log
    scraping.  This function is retained only as a reference for
    debugging older firmware that predates IscHistogramMeans (pre v0.6.1).

    Returns None always in production; the caller checks for None and
    emits an error directing the user to update firmware.
    """
    logger.debug('_parse_awb_debug: log scraping is deprecated — '
                 'this path should not be reached with v0.6.1+ firmware.')
    return None


def _parse_agc_debug(log_lines):
    """
    DEPRECATED: AGC calibration data (ExposureTime, AnalogueGain, AeState)
    now comes from frame metadata (--metadata --metadata-format=json).
    This function is retained only as a reference for debugging.

    Matches:
      'AGC stable: luminance=X target=Y lock=3/3'  (preferred)
      'AGC: luminance=X target=Y ... -> exposure=X gain=Y'  (fallback)

    Returns dict: {'luminance': float, 'exposure_lines': int, 'gain': float,
                   'ae_converged': bool}
    or None if not found.
    """
    # Primary: match the stable lock line (post-convergence)
    stable_re = re.compile(
        r'AGC stable:\s+luminance=([0-9.]+)\s+target=[0-9.]+\s+lock=3/3')
    # Secondary: match the running update line which also carries the next exposure
    running_re = re.compile(
        r'AGC:\s+luminance=([0-9.]+).*->\s+exposure=([0-9]+)\s+gain=([0-9.]+)')

    stable_lum_list = []
    agc_list = []
    ae_converged = False

    for line in log_lines:
        ms = stable_re.search(line)
        if ms:
            stable_lum_list.append(float(ms.group(1)))
            ae_converged = True
        mr = running_re.search(line)
        if mr:
            agc_list.append({
                'luminance': float(mr.group(1)),
                'exposure_lines': int(mr.group(2)),
                'gain': float(mr.group(3)),
            })

    if not agc_list:
        return None

    if not ae_converged:
        logger.warning('AE did not reach lock=3/3 before capture end. '
                       'Exposure metadata in sidecar may be mid-ramp. '
                       'Increase --timeout or verify scene brightness.')

    # Use the last 3 running-update lines (these carry the queued sensor values
    # closest to the converged steady state). Matching only 'stable' lines would
    # miss the final applied exposure since stable lines don't carry exposure.
    last = agc_list[-3:]
    return {
        'luminance': sum(d['luminance'] for d in last) / len(last),
        'exposure_lines': int(round(sum(d['exposure_lines'] for d in last) / len(last))),
        'gain': sum(d['gain'] for d in last) / len(last),
        'ae_converged': ae_converged,
    }


def verify_lens_darkness(board, password, width=1920, height=1080, timeout=25):
    """
    Verify that the lens is adequately blocked before BLC calibration.

    Captures a single dark frame at minimum gain/exposure and measures the
    mean pixel level across all 4 Bayer channels. Returns a dict:
      {
        'ok':       bool   — True if all channels are in the pedestal range
        'means':    dict   — {'R': float, 'GR': float, 'GB': float, 'B': float}
        'max_dn':   float  — highest single-pixel value in the frame
        'verdict':  str    — human-readable result
        'nir_leak': bool   — True if B channel significantly higher than GR
      }

    Acceptance criteria (10-bit DN, exposure=50 lines, gain=1×):
      Expected pedestal range : 60–75 DN    (IMX219 datasheet 64 DN ± thermal)
      Max pixel               : < 90 DN     (> 90 means light is leaking in)
      B/GR ratio              : < 1.10      (> 1.10 means NIR leaking via plastic)

    Material notes:
      Black electrical tape : max_dn typically 62–68 DN  — PASS
      Dark paper + cello tape : max_dn 64–75 DN          — usually PASS (verify)
      Plastic lens cap      : max_dn 70–120 DN at B ch   — FAIL (NIR-transparent)
      Hand cover (lit room) : max_dn 90–250 DN           — FAIL
      Hand cover (dark room): max_dn 64–78 DN            — marginal PASS
    """
    logger.info('Verifying lens darkness before BLC capture...')

    frame_bytes = width * height * 2
    cmd = (
        'SENSOR_DEV=$(media-ctl -d /dev/media0 -e "imx219 1-0010" 2>/dev/null | '
        'grep -oP "/dev/v4l-subdev[0-9]+" | head -1); '
        '[ -z "$SENSOR_DEV" ] && SENSOR_DEV=/dev/v4l-subdev3; '
        'v4l2-ctl -d "$SENSOR_DEV" --set-ctrl analogue_gain=0,exposure=50 2>/dev/null; '
        'sleep 0.15; '
        f'v4l2-ctl -d /dev/video0 '
        f'--set-fmt-video=width={width},height={height},pixelformat=RG10 '
        f'--stream-mmap --stream-count=5 --stream-to=/tmp/_dark_verify.raw 2>/dev/null && '
        f'echo VERIFY_OK || echo VERIFY_FAIL'
    )

    stdout, stderr, rc = _ssh_run(board, password, cmd, timeout=timeout)

    if 'VERIFY_FAIL' in stdout or rc != 0:
        # EPIPE: pipeline not reset after previous session — needs reboot
        if 'Broken pipe' in stderr or 'EPIPE' in stderr or 'STREAMON' in stderr:
            return {
                'ok': False,
                'means': {},
                'max_dn': 0,
                'verdict': ('PIPELINE ERROR: ISC is in a broken state (EPIPE). '
                            'Reboot the board and retry: '
                            'this happens after a session crashes mid-stream.'),
                'nir_leak': False,
            }
        return {
            'ok': False, 'means': {}, 'max_dn': 0,
            'verdict': f'Capture failed (rc={rc}): {stderr.strip()[:120]}',
            'nir_leak': False,
        }

    # Pull raw frame to a temp file and analyse in Python
    import tempfile, os
    tmp = tempfile.mktemp(suffix='.raw')
    if not _scp_from_board(board, password, '/tmp/_dark_verify.raw', Path(tmp)):
        return {
            'ok': False, 'means': {}, 'max_dn': 0,
            'verdict': 'SCP failed — cannot pull dark frame for analysis',
            'nir_leak': False,
        }

    import numpy as np
    try:
        data = np.fromfile(tmp, dtype=np.uint16).astype(np.float32)
        os.unlink(tmp)
    except Exception as e:
        return {'ok': False, 'means': {}, 'max_dn': 0,
                'verdict': f'File read failed: {e}', 'nir_leak': False}

    total_px = width * height
    if len(data) < total_px:
        return {'ok': False, 'means': {}, 'max_dn': 0,
                'verdict': f'Frame too small ({len(data)} px < {total_px})',
                'nir_leak': False}

    # Use the last frame (most settled after gain/exposure register write)
    frame = data[-total_px:].reshape(height, width)

    # Extract 4 Bayer channels (RGGB pattern for IMX219)
    R  = frame[0::2, 0::2]   # Red
    GR = frame[0::2, 1::2]   # Green-Red row
    GB = frame[1::2, 0::2]   # Green-Blue row
    B  = frame[1::2, 1::2]   # Blue

    means = {
        'R':  float(R.mean()),
        'GR': float(GR.mean()),
        'GB': float(GB.mean()),
        'B':  float(B.mean()),
    }
    max_dn = float(frame.max())

    # Evaluate
    all_in_pedestal = all(60 <= v <= 80 for v in means.values())
    no_high_pixels  = max_dn < 90
    gr_mean = means['GR']
    b_gr_ratio = means['B'] / gr_mean if gr_mean > 0 else 99
    nir_leak = b_gr_ratio > 1.10

    ok = all_in_pedestal and no_high_pixels and not nir_leak

    if ok:
        verdict = (f'PASS  — all channels in pedestal range '
                   f'[R={means["R"]:.1f} GR={means["GR"]:.1f} '
                   f'GB={means["GB"]:.1f} B={means["B"]:.1f}] DN, '
                   f'max={max_dn:.0f} DN. Safe to proceed with BLC.')
    elif nir_leak and not all_in_pedestal:
        verdict = (f'FAIL: NIR leak detected — B/GR={b_gr_ratio:.2f} (>{1.10:.2f}). '
                   f'B channel mean={means["B"]:.1f} DN >> GR={means["GR"]:.1f} DN. '
                   f'Your cover transmits near-infrared (850 nm+). '
                   f'Use BLACK electrical tape or multiple layers of dark paper. '
                   f'Cello tape alone is transparent to NIR. '
                   f'Add black paper between lens and tape.')
    elif not no_high_pixels:
        verdict = (f'FAIL: Light leaking in — max pixel={max_dn:.0f} DN (must be <90). '
                   f'Check for gaps around the lens barrel. '
                   f'Press cover flat against the lens module with no gaps.')
    else:
        verdict = (f'WARN: Means slightly outside pedestal range '
                   f'[R={means["R"]:.1f} GR={means["GR"]:.1f} '
                   f'GB={means["GB"]:.1f} B={means["B"]:.1f}] DN. '
                   f'Acceptable if all values are 55–85 DN. '
                   f'Proceed with caution or add more layers.')

    logger.info(f'  Darkness check: {verdict}')
    return {
        'ok': ok,
        'means': means,
        'max_dn': max_dn,
        'verdict': verdict,
        'nir_leak': nir_leak,
    }


def _capture_blc_on_board(board, password, output_dir, width=1920, height=1080,
                          timeout=30):
    """
    Capture a dark (lens-capped) frame for BLC calibration using v4l2-ctl.

    Uses v4l2-ctl directly (NOT mchpcam-still) so no IPA is running and
    therefore no AGC gain boost occurs. The IPA would ramp analogue gain to
    maximum (~9x) on a dark frame, making the "dark frame" measure the noise
    floor at high gain rather than the true sensor black level pedestal.

    Sets analogue gain to minimum (1.0) and a fixed short exposure (1ms)
    via V4L2 sensor controls before capture.

    Returns the local path to the captured .raw dark frame, or None on failure.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    remote_raw = '/tmp/blc_dark.raw'
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    local_raw = output_dir / f'dark_lux0_{ts}.raw'

    # Set minimum gain and short exposure on the SENSOR subdevice (/dev/v4l-subdev3),
    # NOT on /dev/video0. The sensor exposure/gain controls belong to the sensor
    # subdevice; setting them on the ISC video node silently fails (unknown control).
    # The subdevice node index may vary — discover it by matching the entity name.
    frame_bytes = width * height * 2
    cmd = (
        # Find sensor subdevice by entity name and store as SENSOR_DEV
        'SENSOR_DEV=$(media-ctl -d /dev/media0 -e "imx219 1-0010" 2>/dev/null | '
        'grep -oP "/dev/v4l-subdev[0-9]+" | head -1); '
        '[ -z "$SENSOR_DEV" ] && SENSOR_DEV=/dev/v4l-subdev3; '
        # Set minimum gain (0 = 1.0×) and short exposure (50 lines ≈ 1ms at 30fps)
        # so the frame is dark enough to measure the true pedestal. Exposure=50
        # is safe: too short to accumulate scene signal through a capped lens.
        'v4l2-ctl -d "$SENSOR_DEV" --set-ctrl analogue_gain=0,exposure=50 2>/dev/null; '
        # Wait 2 frames for the sensor to apply the register write (2-frame lag)
        'sleep 0.1; '
        f'v4l2-ctl -d /dev/video0 '
        f'--set-fmt-video=width={width},height={height},pixelformat=RG10 '
        f'--stream-mmap --stream-count=8 --stream-to=/tmp/_blc_raw8.raw 2>/dev/null; '
        # Take the last 3 frames and average them for noise reduction
        # (vs single frame which has ~3 DN RMS noise at gain=1)
        f'dd if=/tmp/_blc_raw8.raw bs={frame_bytes} skip=5 count=1 '
        f'of={remote_raw} 2>/dev/null; '
        f'rm -f /tmp/_blc_raw8.raw; '
        # Verify file was created and is not empty
        f'test -s {remote_raw} && echo "BLC_CAPTURE_OK" || echo "BLC_CAPTURE_FAIL"'
    )

    logger.info(f'Capturing dark frame via v4l2-ctl on {board}...')
    stdout, stderr, rc = _ssh_run(board, password, cmd, timeout=timeout)

    if rc != 0 or 'BLC_CAPTURE_FAIL' in stdout:
        logger.error(f'v4l2-ctl dark capture failed (rc={rc}): {stderr.strip()[:200]}')
        return None
    if 'BLC_CAPTURE_OK' not in stdout:
        logger.error('BLC capture: output file missing or empty on board.')
        return None

    if not _scp_from_board(board, password, remote_raw, local_raw):
        logger.error(f'Failed to copy dark frame from board')
        return None

    # Write a minimal JSON sidecar so image.py can load it correctly
    # wb_gains all 1.0 (no IPA WB applied), black_level unknown (measuring it)
    sidecar = {
        'width': width, 'height': height,
        'format': 'RG10', 'bayer_pattern': 'RGGB',
        'exposure_us': 1000.0, 'analogue_gain': 1.0,
        'black_level': 0,   # 0 = raw -- no BLC applied for this capture
        'black_level_r': 0, 'black_level_gr': 0,
        'black_level_gb': 0, 'black_level_b': 0,
        'wb_gains_r': 1.0, 'wb_gains_gr': 1.0,
        'wb_gains_gb': 1.0, 'wb_gains_b': 1.0,
        'lux': 0.0,
    }
    json_path = local_raw.with_suffix('.json')
    with open(json_path, 'w') as f:
        json.dump(sidecar, f, indent=2)

    logger.info(f'  Saved dark frame: {local_raw}')
    return local_raw


def capture_on_board(board, password, output_dir, prefix,
                     true_ct=None, true_lux=None,
                     width=1920, height=1080, timeout=90):
    """
    Trigger a calibration capture on the board via SSH.

    Trigger a JPEG calibration capture on the board via SSH.

    Runs mchpcam-still with --metadata --metadata-format=json to get
    IscHistogramMeans (pre-WB histogram channel means) alongside the
    converged AWB/AGC state. Falls back to ISC_AWB:DEBUG log parsing for
    firmware older than v0.6.1.

    Args:
      board      : SSH target, e.g. 'root@10.40.24.60'
      password   : SSH password (or None for key auth)
      output_dir : local directory to save the captured files
      prefix     : filename prefix (e.g. 'grey_card')
      true_ct    : true CCT in Kelvin (from phone camera or lux meter)
      true_lux   : measured scene lux (from handheld meter)
      width/height: capture resolution
      timeout    : SSH command timeout in seconds

    Returns the local path to the saved JPEG, or None on failure.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Capture JPEG (full ISC pipeline active, AWB/AGC converge normally).
    #
    # The ISC has a single DMA output path. SRGGB10 output has the WB block
    # active (bits_pipeline=WB_ENABLE), so it is not true pre-WB raw data.
    # More importantly, without the ISC post-processing pipeline (CFA, gamma,
    # CBC), the AGC luminance target is in the wrong domain and AE never
    # converges — mchpcam-still hangs indefinitely.
    #
    # Calibration channel means are read from the IPA AWB debug log lines
    # (pre-WB histogram tap via ISC_HIS_CFG_RAR), not from pixel data.
    # The JPEG is saved only as a visual reference for the operator.
    remote_jpeg = '/tmp/calib_capture.jpg'
    remote_json = '/tmp/calib_capture.json'

    log_cmd = (''
               'LIBCAMERA_LOG_LEVELS=ISC_AWB:DEBUG,ISC_AGC:DEBUG,ISC_IPA:DEBUG '
               f'mchpcam-still --noir '
               f'--width={width} --height={height} --timeout=200 '
               f'-o {remote_jpeg}')

    ct_str = f'{int(true_ct)}K' if true_ct else '?K'
    stdout, stderr, rc = _ssh_run_progress(
        board, password, log_cmd,
        timeout=180,
        label=f'Capture {ct_str}',
    )

    captured_ok = 'Saved JPEG:' in stdout or 'Saved JPEG:' in stderr
    if not captured_ok:
        logger.error(f'Capture failed (rc={rc}). "Saved JPEG:" not found.')
        logger.error('Check: (1) board alive, (2) mchpcam-still installed, '
                     '(3) sensor connected, (4) scene bright enough for AGC.')
        return None

    # --- Read AWB/AGC data from structured metadata (only valid source) -----
    #
    # mchpcam-still --metadata --metadata-format=json writes one JSON object
    # per captured frame to stdout. We need the LAST object (the saved frame —
    # the converged one) not the first (warmup frame before AWB/AGC converged).
    #
    # IscHistogramMeans [GR, R, GB, B]: pre-WB histogram channel means.
    # This is the authoritative calibration input — the ISC histogram tap
    # (ISC_HIS_CFG_RAR) is before both DPC and WB. Debug log parsing is
    # completely removed: the metadata is a stable versioned interface;
    # log format can change or be suppressed without warning.
    awb = None
    agc = None
    last_meta = None
    for line in stdout.splitlines():
        line = line.strip()
        if not line.startswith('{'):
            continue
        try:
            meta = json.loads(line)
            last_meta = meta      # keep updating — we want the last one
        except json.JSONDecodeError:
            continue

    if last_meta is not None:
        hist = last_meta.get('IscHistogramMeans')
        # IscHistogramMeans array: [GR, R, GB, B] — hardware channel order
        if hist and len(hist) == 4:
            awb = {
                'GR':        float(hist[0]),
                'R':         float(hist[1]),
                'GB':        float(hist[2]),
                'B':         float(hist[3]),
                'awb_locked': bool(last_meta.get('AwbLocked', False)),
                'ct_smoothed': float(last_meta.get('ColourTemperature', 0)),
            }
        else:
            logger.error(
                'IscHistogramMeans not found in metadata JSON. '
                'This firmware does not emit the vendor histogram control. '
                'Rebuild and redeploy libcamera-mchp v0.6.1+ to the board.')
            return None

        exp_us = last_meta.get('ExposureTime')
        gain   = last_meta.get('AnalogueGain')
        if exp_us is not None and gain is not None:
            agc = {
                'exposure_us':  float(exp_us),
                'gain':         float(gain),
                'ae_converged': last_meta.get('AeState') == 'AeStateConverged',
            }
    else:
        logger.error(
            'No JSON metadata found in mchpcam-still stdout. '
            'Check that --metadata --metadata-format=json flags are passed '
            'and that the firmware supports metadata output.')
        return None

    if not awb['awb_locked']:
        logger.warning('AWB was not locked at capture time. '
                       'Channel means may not represent true illuminant. '
                       'Increase --timeout or ensure adequate scene brightness.')

    if agc and not agc['ae_converged']:
        logger.warning('AE had not converged at capture time. '
                       'Exposure/gain values may be mid-ramp.')

    # Build filename tags.
    # CRITICAL: Only use true_ct from the USER (phone/meter measurement).
    # NEVER use IPA's ct_smoothed as the true CCT — it is the value being
    # corrected. Using it as ground truth makes calibration circular.
    tags = []
    if true_ct is not None:
        tags.append(f'ct{int(true_ct)}K')

    if true_lux is not None:
        tags.append(f'lux{int(true_lux)}')

    if agc:
        tags.append(f'exp{round(agc["exposure_us"])}us')
        tags.append(f'gain{agc["gain"]:.2f}')

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    stem = f'{prefix}_{"_".join(tags)}_{ts}' if tags else f'{prefix}_{ts}'

    local_jpeg = output_dir / f'{stem}.jpg'
    local_json  = output_dir / f'{stem}.json'

    # Verify file exists on board before copying
    check_out, _, check_rc = _ssh_run(board, password,
                                      f'test -s {remote_jpeg} && echo "OK"',
                                      timeout=10)
    if check_rc != 0 or 'OK' not in check_out:
        logger.error(f'JPEG file not found or empty on board: {remote_jpeg}')
        return None

    # Copy JPEG from board (visual reference)
    if not _scp_from_board(board, password, remote_jpeg, local_jpeg):
        logger.error(f'Failed to copy {remote_jpeg} from board')
        return None

    # Copy JSON sidecar
    _scp_from_board(board, password, remote_json, local_json)

    # Write a JSON sidecar alongside the JPEG containing all calibration
    # metadata. This is the single source of truth for each capture:
    # - true_ct: ground truth CCT from user (phone/meter) — calibration input
    # - hist_means: pre-WB histogram channel means — calibration input
    # - ipa_ct_smoothed: IPA estimate — informational only, NOT ground truth
    # - exposure_us, gain: for lux calibration and filename tagging
    sidecar = {
        'width':       width,
        'height':      height,
        'format':      'JPEG',
        'true_ct':     true_ct,
        'lux':         true_lux,
    }
    if awb:
        sidecar['hist_means'] = {
            'R':  awb['R'],
            'GR': awb['GR'],
            'GB': awb['GB'],
            'B':  awb['B'],
        }
        sidecar['awb_locked']     = awb['awb_locked']
        sidecar['ipa_ct_smoothed'] = awb['ct_smoothed']
        if true_ct:
            sidecar['ipa_ct_error_before_cal'] = round(awb['ct_smoothed'] - true_ct, 1)
    if agc:
        sidecar['exposure_us']  = agc['exposure_us']
        sidecar['analogue_gain'] = agc['gain']
        sidecar['ae_converged']  = agc['ae_converged']

    try:
        with open(local_json, 'w') as f:
            json.dump(sidecar, f, indent=2)
    except Exception as e:
        logger.warning(f'Could not write JSON sidecar: {e}')

    if awb:
        logger.info(f'  IPA CT smoothed={awb.get("ct_smoothed", 0):.0f}K  '
                    f'means: R={awb["R"]:.1f} GR={awb["GR"]:.1f} '
                    f'GB={awb["GB"]:.1f} B={awb["B"]:.1f}')
        if true_ct:
            err = awb.get('ct_smoothed', 0) - true_ct
            logger.info(f'  CT error before correction: {err:+.0f}K')

    logger.info(f'  Saved: {local_jpeg}')
    return local_jpeg


def run_ssh_capture_session(args, modules):
    """
    Interactive SSH capture session: prompt for each illuminant, trigger
    captures on the board, save RAW+JSON files with metadata filenames.

    Returns the list of captured local paths.
    """
    board = args.board
    password = getattr(args, 'board_pass', None)
    output_dir = Path(args.input)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Check board connectivity
    logger.info(f'Checking board connectivity ({board})...')
    _, _, rc = _ssh_run(board, password,
                        'which mchpcam-still > /dev/null 2>&1', timeout=15)
    if rc != 0:
        logger.error(f'Cannot reach {board} or mchpcam-still not found')
        return []
    logger.info(f'Board reachable: {board}')

    captured = []
    needs_ct = 'ct' in modules or 'awb' in modules or 'ccm' in modules

    no_prompt = getattr(args, 'no_prompt', False)

    if 'blc' in modules:
        if not no_prompt:
            input('Cap the lens. Press Enter when ready: ')
        path = _capture_blc_on_board(board, password, output_dir)
        if path:
            captured.append(path)

    if needs_ct:
        # Determine what target each capture needs based on active modules:
        #   ct / awb : 18% neutral grey card, uniform fill >70% of frame
        #   ccm      : Macbeth ColorChecker chart, all 24 patches visible,
        #              no glare, chart perpendicular to lens axis
        # When both are active, two separate captures are taken per CCT:
        # first the grey card (for ct/awb), then the ColorChecker (for ccm).
        needs_grey     = 'ct' in modules or 'awb' in modules
        needs_macbeth  = 'ccm' in modules

        if not no_prompt:
            targets = []
            if needs_grey:
                targets.append('18% grey card (ct/awb)')
            if needs_macbeth:
                targets.append('Macbeth ColorChecker (ccm)')
            print(f'Captures required per CCT point: {", ".join(targets)}')
            print('Enter lightbox-rated CCT. Type "done" when complete.')

        while True:
            try:
                ct_str = input('CCT [K]: ').strip()
            except EOFError:
                print('\n  EOF on stdin — ending CCT capture loop.')
                break
            if ct_str.lower() == 'done':
                break
            ct_str = ct_str.rstrip('kK')
            try:
                true_ct = float(ct_str)
            except ValueError:
                print('  Invalid CCT. Enter a number (e.g. 4350) or "done".')
                continue

            if true_ct < 1000 or true_ct > 20000:
                print(f'  Warning: {true_ct:.0f}K is outside supported range [1000, 20000].')

            true_lux = None
            if 'lux' in modules:
                try:
                    lux_str = input(f'  Lux at {true_ct:.0f}K (Enter to skip): ').strip()
                except EOFError:
                    lux_str = ''
                if lux_str:
                    try:
                        true_lux = float(lux_str)
                        if true_lux <= 0:
                            print(f'  Lux must be positive. Skipping.')
                            true_lux = None
                    except ValueError:
                        print(f'  Invalid lux value "{lux_str}", skipping.')

            # Grey card capture (ct / awb)
            if needs_grey:
                if not no_prompt:
                    input(f'  Grey card in frame at {true_ct:.0f}K. Press Enter: ')
                path = capture_on_board(board, password, output_dir,
                                        prefix='grey_card',
                                        true_ct=true_ct, true_lux=true_lux,
                                        width=1920, height=1080)
                if path:
                    captured.append(path)
                    print(f'  Saved: {path.name}')

            # ColorChecker capture (ccm) — separate capture at same CCT
            if needs_macbeth:
                if not no_prompt:
                    input(f'  Macbeth ColorChecker in frame at {true_ct:.0f}K. Press Enter: ')
                path = capture_on_board(board, password, output_dir,
                                        prefix='colorchecker',
                                        true_ct=true_ct, true_lux=true_lux,
                                        width=1920, height=1080)
                if path:
                    captured.append(path)
                    print(f'  Saved: {path.name}')

    # Lux capture: needed if no previously-captured image has a non-zero lux tag
    # Check the actual images in the input dir, not just the captured paths,
    # since the user may have pre-existing lux images from a CT capture session.
    existing_lux = [
        p for p in output_dir.iterdir()
        if p.suffix.lower() == '.raw'
        and re.search(r'lux([1-9][0-9]*)', p.stem, re.IGNORECASE)
    ]
    if 'lux' in modules and not existing_lux:
        lux_str = input('Lux reference [lux] (Enter to skip): ').strip()
        if not lux_str:
            logger.info('Lux reference skipped.')
        else:
            try:
                true_lux = float(lux_str)
                path = capture_on_board(board, password, output_dir,
                                        prefix='lux_ref', true_lux=true_lux,
                                        width=1920, height=1080)
                if path:
                    captured.append(path)
            except ValueError:
                logger.warning(f'Invalid lux value "{lux_str}", skipping.')

    return captured


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

SSH capture workflow:
  python3 microchip_isc.py \\
      --board root@<host> --board-pass <pass> \\
      -i ./calib_images/ -o imx219_noir.yaml \\
      --modules blc ct awb lux recalc --noir
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
    parser.add_argument('--board',
        help='SSH target for live capture mode. Accepts any value that SSH '
             'resolves: IP address, hostname, or FQDN. '
             'Example: root@192.168.1.100 or root@boardname. '
             'When set, captures are triggered on the board via SSH and '
             'saved to --input directory before processing.')
    parser.add_argument('--board-pass', dest='board_pass', default=None,
        help='SSH password for --board. Omit to use key-based authentication.')
    parser.add_argument('--factory-reset', action='store_true',
        help='RECOVERY ONLY: restore physics-derived factory defaults to the '
             'YAML. Use this ONLY when a previous calibration produced garbage '
             'results and you want to return to a known-safe state before '
             're-calibrating. Do NOT use as a routine pre-calibration step — '
             'the standard workflow (bash calib.sh) handles starting state '
             'correctly without a reset. '
             'Writes: BLC=64 DN, AWB gains from Planckian locus, '
             'CCM=identity, lux_reference=uncalibrated. '
             'A timestamped backup is always created.')
    parser.add_argument('--health', action='store_true',
        help='Print calibration health score (0–100) for the current YAML '
             'and exit. Does not require --input or --modules.')
    parser.add_argument('--dry-run', action='store_true',
        help='Validate all dependencies (board connectivity, Python packages, '
             'YAML structure) and print a pre-flight check. Does not capture '
             'or modify anything.')

    args = parser.parse_args()

    if not Path(args.output).exists():
        logger.error(f'YAML file not found: {args.output}')
        sys.exit(1)

    # --health: just print score and exit
    if getattr(args, 'health', False):
        data, _ = load_yaml(args.output)
        score, report = calibration_health_score(data)
        grade = ('A' if score >= 90 else 'B' if score >= 75 else
                 'C' if score >= 60 else 'D' if score >= 45 else 'F')
        print(f'\nCalibration Health Score: {score}/100  [{grade}]')
        print(f'YAML: {args.output}\n')
        for line in report:
            print(line)
        print()
        if score < 60:
            print('  Action required: run full calibration sequence.')
            print('  Minimum: bash utils/tuning/calib.sh')
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

    # SSH capture mode: trigger captures on the board first
    if args.board:
        captured = run_ssh_capture_session(args, modules)
        if not captured:
            logger.warning('No images captured via SSH. '
                           'Proceeding with existing images in input directory.')

    # Offline mode requires input directory to exist with images
    if not Path(args.input).is_dir():
        logger.error(f'Input directory not found: {args.input}')
        sys.exit(1)

    # Load images from input directory
    images = load_images(args.input, macbeth_mode='ccm' in modules and args.macbeth)
    if not images:
        logger.error('No calibration images found. '
                     'Use --board to capture from the board, or place '
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

    # Board connectivity
    if getattr(args, 'board', None):
        import subprocess
        pwd = getattr(args, 'board_pass', None) or ''
        cmd = (['sshpass', '-p', pwd] if pwd else []) + \
              ['ssh', '-o', 'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=5',
               args.board, 'which mchpcam-still']
        r = subprocess.run(cmd, capture_output=True, timeout=10)
        chk(f'Board reachable: {args.board}', r.returncode == 0,
            '' if r.returncode == 0 else '→ check BOARD/BOARD_PASS in calib.cfg',
            fatal=True)

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
