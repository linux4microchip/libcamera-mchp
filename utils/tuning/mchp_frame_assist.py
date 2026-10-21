#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Macbeth framing assistant for ISC calibration captures.
#
# Locates a ColorChecker chart with the tuner's own detector and QA gate
# (libtuning.macbeth, libtuning.macbeth_qa) and writes an annotated preview: the
# detected 6x4 grid, a confidence score and a PASS/WARN/FAIL verdict. Sharing
# the calibration code path keeps this verdict and the solver's from diverging.
#
# Modes:
#   --local FILE     analyse an existing capture (.pgm raw, or .jpg/.png)
#   --capture        run a capture on the target over SSH and pull it back
#   --stream PIPE    analyse a live stream from the target (framing only)
#   --watch          repeat the chosen mode on an interval until Ctrl-C
#
# --capture needs key-based SSH to the target and a --capture-cmd that writes a
# raw .pgm (optionally a matching .json sidecar) into its {outdir}. It defaults
# to a single-frame mchpcam-still grab for framing; the full calibration sweep
# (mchp_capture.sh) must run locally on the target, never over SSH, so it is not
# the default here.
#
# --stream takes the receive/decode half of a gst-launch line (up to a raw-video
# pad); the BGR appsink tail is appended here. The target streams with
# libcamerasrc, this host only decodes. No RAW, so framing only, no QA. E.g.
#   target: gst-launch-1.0 libcamerasrc ! video/x-raw,width=1280,height=720 \
#             ! jpegenc ! rtpjpegpay ! udpsink host=<host> port=5000
#   host:   --stream 'udpsrc port=5000 caps=application/x-rtp,encoding-name=JPEG \
#             ! rtpjpegdepay ! jpegdec'
#
# FRAMING (geometry: located, square, confident) is reported for any input; QA
# (clipping, tilt, uneven light) only for a RAW .pgm, n/a for a processed JPEG.

import argparse
import logging
import os
import shlex
import subprocess
import sys
import time
import warnings
from pathlib import Path

import cv2
import numpy as np

# libtuning lives next to this script.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from libtuning.image import Image                         # noqa: E402
from libtuning.macbeth import get_macbeth_chart           # noqa: E402
from libtuning import macbeth_qa                           # noqa: E402

logging.basicConfig(format='%(message)s', level=logging.INFO)
logger = logging.getLogger('frame_assist')

# "Reliable placement" correlation bar (detector floor is 0.60).
_COR_GOOD = 0.75

# Tilt (perspective foreshortening of the located grid) thresholds, shared with
# the QA gate so the framing verdict and the QA verdict agree on squareness.
_TILT_WARN = macbeth_qa._TILT_WARN
_TILT_FAIL = macbeth_qa._TILT_FAIL

_N_COLS = macbeth_qa._N_COLS
_N_ROWS = macbeth_qa._N_ROWS
_N_PATCHES = _N_COLS * _N_ROWS

# Glare / uneven-light floor; QA only WARNs, the live preview lets the operator fix it.
_GLARE_SPREAD = macbeth_qa._SPREAD_WARN

# Exposure bands for the brightest located patch as a fraction of range: below
# _EXP_LOW the dark end sits in noise; above _EXP_HIGH it hits the shoulder;
# _EXP_CLIP is saturation.
_EXP_LOW = 0.55
_EXP_HIGH = 0.95
_EXP_CLIP = 0.98

# A chromatic patch railing a single Bayer channel biases the CCM; fail at this
# fraction even when pooled luma looks unclipped.
_CHAN_CLIP = 0.99

_RAW_SUFFIXES = ('.pgm',)
_IMG_SUFFIXES = ('.jpg', '.jpeg', '.png', '.bmp')

# The remote output dir is wiped with "rm -rf" each capture, so confine it to a
# scratch prefix: an absolute path under one of these, no "..".
_REMOTE_OUTDIR_PREFIXES = ('/tmp/', '/run/', '/var/tmp/', '/root/', '/home/')

# BGR colours (OpenCV order).
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)
_CYAN = (255, 255, 0)
_WHITE = (255, 255, 255)

# ColorChecker Classic is ~1.5:1 (6x4). The guide box marks where to place it:
# centred, square-on, filling most of the frame.
_CHART_ASPECT = 1.5


# --------------------------------------------------------------------------
# Chart location
# --------------------------------------------------------------------------

def _build_ref_data():
    """Load the reference chart used to score a candidate match.

    Mirrors libtuning.macbeth.find_macbeth so the confidence we display is the
    same correlation the tuner computes.
    """
    ref_path = Path(__file__).resolve().parent / 'libtuning' / 'macbeth_ref.pgm'
    ref = cv2.imread(str(ref_path), flags=cv2.IMREAD_GRAYSCALE)
    if ref is None:
        raise FileNotFoundError(f'reference chart not found: {ref_path}')
    ref_w, ref_h = 120, 80
    ref_corns = np.array(((0, 0), (0, ref_h), (ref_w, ref_h), (ref_w, 0)),
                         np.float32)
    return (ref, ref_w, ref_h, ref_corns)


def locate(img, ref_data, cor_good=_COR_GOOD):
    """Locate the chart and return (confidence, fit_coords).

    fit_coords is (outer_corners, patch_centres) in the pixel space of ``img``,
    or None when no chart was found. For an 8-bit image a couple of brightness
    boosts are tried (as the tuner does) to rescue a dark frame; a float Bayer
    average is already contrast-normalised inside the detector, so one pass
    suffices there.

    The chart is expected to fill much of the frame during framing, so the
    tuner's small-chart subselection scan is intentionally skipped -- it is the
    slow part and least useful here (a chart too small to detect at full frame
    is itself the framing problem to report).
    """
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        cor, _, coords, ok = get_macbeth_chart(img, ref_data)

        if img.dtype == np.uint8:
            for boost in (2, 4):
                if cor >= cor_good:
                    break
                cand = get_macbeth_chart(cv2.convertScaleAbs(img, alpha=boost),
                                         ref_data)
                if cand[0] > cor:
                    cor, _, coords, ok = cand

    if not ok:
        return 0.0, None
    return float(cor), coords


def _tilt(centres):
    """Perspective foreshortening of the located centre grid, 0..1.

    Same geometry as macbeth_qa._tilt but computed straight from the located
    centres so it is available for a JPEG too (no Image needed). Returns None if
    the grid is not the expected 24-patch layout.
    """
    if centres is None or len(centres) != _N_PATCHES:
        return None
    pts = np.asarray(centres, dtype=np.float64)

    def g(col, row):
        return pts[col * _N_ROWS + row]

    def dist(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def asym(a, b):
        hi = max(a, b)
        return abs(a - b) / hi if hi > 1e-6 else 0.0

    left_h = dist(g(0, 0), g(0, _N_ROWS - 1))
    right_h = dist(g(_N_COLS - 1, 0), g(_N_COLS - 1, _N_ROWS - 1))
    top_w = dist(g(0, 0), g(_N_COLS - 1, 0))
    bot_w = dist(g(0, _N_ROWS - 1), g(_N_COLS - 1, _N_ROWS - 1))
    return max(asym(left_h, right_h), asym(top_w, bot_w))


# Fraction of the frame the chart should fill. Below _FILL_LOW the patches are
# too small for low-noise per-patch sampling; above _FILL_HIGH the chart risks
# running off an edge (the calibration white/black patches must stay in frame).
_FILL_LOW = 0.35
_FILL_HIGH = 0.92


def _fill_fraction(coords, frame_shape):
    """Chart outer-quad area as a fraction of the frame, 0..1, or None.

    Uses the shoelace area of the located outer quad over the frame area, so it
    tracks how much of the sensor the chart actually covers for framing help.
    """
    if coords is None:
        return None
    quad = np.asarray(coords[0][0], dtype=np.float64).reshape(-1, 2)
    if len(quad) < 3:
        return None
    x, y = quad[:, 0], quad[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    frame = float(frame_shape[0] * frame_shape[1])
    return area / frame if frame > 0 else None


# Focus floor: Laplacian variance over the chart ROI, normalised by mean^2 so it
# is exposure-independent. A sharp chart's patch edges push it well above this;
# a soft/defocused frame collapses toward zero. Conservative first guess -- needs
# one live board frame to confirm it does not false-fail a focused chart.
_FOCUS_MIN = 0.0008


def _focus(gray, coords):
    """Sharpness of the located chart ROI as normalised Laplacian variance.

    Crops the located outer quad from the grayscale plane and returns
    var(Laplacian)/mean^2, or None if the ROI is unusable. Best-effort: a soft
    frame scores low, but an unexpected ROI returns None rather than blocking.
    """
    if gray is None or coords is None:
        return None
    try:
        quad = np.asarray(coords[0][0], dtype=np.float64).reshape(-1, 2)
        if len(quad) < 3:
            return None
        h, w = gray.shape[:2]
        x0 = max(0, int(np.floor(quad[:, 0].min())))
        y0 = max(0, int(np.floor(quad[:, 1].min())))
        x1 = min(w, int(np.ceil(quad[:, 0].max())))
        y1 = min(h, int(np.ceil(quad[:, 1].max())))
        if x1 - x0 < 8 or y1 - y0 < 8:
            return None
        roi = gray[y0:y1, x0:x1].astype(np.float64)
        mean = float(roi.mean())
        if mean <= 1e-9:
            return None
        return float(cv2.Laplacian(roi, cv2.CV_64F).var()) / (mean * mean)
    except Exception as e:
        logger.debug(f'focus check unavailable: {e}')
        return None


# --------------------------------------------------------------------------
# Quality gate (RAW only)
# --------------------------------------------------------------------------

def _qa(image, coords):
    """(verdict_dict, clipped_indices) for a RAW capture, or (None, []).

    Populates image.patches from the located centres, then runs the tuner's QA
    gate for the PASS/WARN/FAIL verdict and separately lists which patches
    clipped (using the same rule the gate counts with) so they can be flagged on
    the overlay. Any geometry that trips the patch sampler degrades to
    (None, []) rather than aborting the preview.
    """
    if image is None or coords is None:
        return None, []
    try:
        image.get_patches(coords[1])
        verdict = macbeth_qa.qa_image(image)
        white = macbeth_qa._white_level(image)
        clipped = [p for p in range(image.patches.shape[1])
                   if any(image.patches[ci][p][-1] >= white for ci in range(4))]
        return verdict, clipped
    except Exception as e:
        logger.debug(f'QA unavailable: {e}')
        return None, []


def _exposure(image):
    """Exposure readiness for a RAW capture, or None.

    Returns {level, band}. level is the brightest located patch's near-peak (p99
    of its pooled Bayer pixels, black-level removed) as a fraction of range;
    band is 'low'/'good'/'high'/'clipped'. Chart-order independent: picks the
    brightest patch rather than a fixed index, so it tracks whichever patch the
    solve needs headroom on. Flags clipped if any patch rails a Bayer channel.
    """
    if image is None or getattr(image, 'patches', None) is None:
        return None
    try:
        n = image.patches.shape[1]
        if n == 0:
            return None
        white = float(macbeth_qa._white_level(image))
        bl = float(getattr(image, 'blacklevel_16', 0) or 0)
        span = max(white - bl, 1.0)
        # p99 per patch, pooled over the 4 Bayer channels; brightest wins.
        peaks = [float(np.percentile(
            np.concatenate([image.patches[ci][p] for ci in range(4)]), 99.0))
            for p in range(n)]
        level = max(0.0, (max(peaks) - bl) / span)
        # Any patch railing a single channel biases the solve.
        chan_rail = white - (1.0 - _CHAN_CLIP) * span
        railed = any(image.patches[ci][p][-1] >= chan_rail
                     for p in range(n) for ci in range(4))
        if railed or level >= _EXP_CLIP:
            band = 'clipped'
        elif level < _EXP_LOW:
            band = 'low'
        elif level > _EXP_HIGH:
            band = 'high'
        else:
            band = 'good'
        return {'level': level, 'band': band}
    except Exception as e:
        logger.debug(f'exposure check unavailable: {e}')
        return None


# Auto-exposure ramp: bisect the shutter on the monotonic shutter<->brightness
# relation until the band is usable. Clamp to a sane window and cap the tries.
_RAMP_MAX_TRIES = 6
_RAMP_SHUTTER_MIN = 500
_RAMP_SHUTTER_MAX = 33000
_RAMP_START = 20000


# Concrete capture-setting hints for the footer, keyed by exposure band. The
# bench runs the IMX219 at unity gain where 29 fps caps the shutter near 34 ms;
# dropping the framerate unlocks a longer exposure without adding read noise.
_EXP_HINT = {
    'low': 'dim: try --framerate 10 --shutter 120000 (keep gain 1.0)',
    'high': 'bright: shorten --shutter or raise --framerate',
    'clipped': 'clipped: lower --shutter (white card is saturating)',
}


# --------------------------------------------------------------------------
# Preview rendering
# --------------------------------------------------------------------------

def _raw_preview(image):
    """A viewable BGR preview from the linear Bayer planes.

    Built at Bayer-channel resolution (half the sensor width/height), which is
    exactly the coordinate space the detector runs in, so located coordinates
    overlay without rescaling. Black level is subtracted and a display gamma is
    applied purely so the operator can see the scene; the preview never feeds
    calibration.
    """
    ch = image.channels
    bl = getattr(image, 'blacklevel_16', 0) or 0
    r = ch[0].astype(np.float64)
    g = (ch[1].astype(np.float64) + ch[2].astype(np.float64)) / 2.0
    b = ch[3].astype(np.float64)

    def norm(x):
        x = np.clip(x - bl, 0, None)
        hi = np.percentile(x, 99.5)
        if hi <= 0:
            hi = 1.0
        x = np.clip(x / hi, 0, 1) ** (1 / 2.2)
        return (x * 255).astype(np.uint8)

    return cv2.merge([norm(b), norm(g), norm(r)])


def _load(path):
    """Return (preview_bgr, detector_input, image_or_None).

    detector_input is what locate() runs on: a float Bayer average for RAW, the
    8-bit BGR frame otherwise. image_or_None is the libtuning Image (RAW only),
    needed for the QA verdict.
    """
    suffix = path.suffix.lower()
    if suffix in _RAW_SUFFIXES:
        image = Image(path)
        if image.channels is None:
            raise ValueError(f'{path.name}: no Bayer data decoded')
        av = np.mean(np.array(image.channels), axis=0) / (2 ** 16)
        return _raw_preview(image), av, image

    if suffix in _IMG_SUFFIXES:
        bgr = cv2.imread(str(path), flags=cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f'{path.name}: could not be read as an image')
        return bgr, bgr, None

    raise ValueError(f'{path.name}: unsupported input (need .pgm or .jpg/.png)')


def _draw_grid(canvas, coords, clipped, colour, lw):
    """Draw the detected outer quad, 6x4 grid lines and patch centres.

    clipped is the list of patch indices whose brightest pixels saturated; those
    centres are ringed in red so the operator sees exactly what to darken.
    """
    outer = np.round(coords[0][0]).astype(np.int32)
    cv2.polylines(canvas, [outer], isClosed=True, color=colour,
                  thickness=lw * 2, lineType=cv2.LINE_AA)

    cen = np.round(coords[1][0]).astype(np.int32)
    if len(cen) != _N_PATCHES:
        return
    grid = cen.reshape(_N_COLS, _N_ROWS, 2)
    for c in range(_N_COLS):
        cv2.polylines(canvas, [np.ascontiguousarray(grid[c])], isClosed=False,
                      color=colour, thickness=lw, lineType=cv2.LINE_AA)
    for r in range(_N_ROWS):
        cv2.polylines(canvas, [np.ascontiguousarray(grid[:, r])],
                      isClosed=False, color=colour, thickness=lw,
                      lineType=cv2.LINE_AA)

    clipped = set(clipped or [])
    for idx, (x, y) in enumerate(cen):
        if idx in clipped:
            cv2.circle(canvas, (int(x), int(y)), lw * 4, _RED, lw + 1,
                       lineType=cv2.LINE_AA)
        else:
            cv2.circle(canvas, (int(x), int(y)), lw * 2, colour, -1,
                       lineType=cv2.LINE_AA)


def _draw_guides(canvas):
    """Draw static framing guides so the chart can be lined up before it locks:
    faint rule-of-thirds plus a centred target box at the chart aspect."""
    h, w = canvas.shape[:2]
    lw = max(1, round(w / 1400.0))
    thirds = canvas.copy()
    for i in (1, 2):
        cv2.line(thirds, (w * i // 3, 0), (w * i // 3, h), _CYAN, lw, cv2.LINE_AA)
        cv2.line(thirds, (0, h * i // 3), (w, h * i // 3), _CYAN, lw, cv2.LINE_AA)
    cv2.addWeighted(thirds, 0.25, canvas, 0.75, 0, canvas)

    bw = int(w * 0.82)
    bh = int(bw / _CHART_ASPECT)
    if bh > h * 0.9:
        bh = int(h * 0.9)
        bw = int(bh * _CHART_ASPECT)
    x0, y0 = (w - bw) // 2, (h - bh) // 2
    x1, y1 = x0 + bw, y0 + bh
    cv2.rectangle(canvas, (x0, y0), (x1, y1), _CYAN, lw, cv2.LINE_AA)
    tick = max(14, bw // 12)
    for cx, cy, dx, dy in ((x0, y0, 1, 1), (x1, y0, -1, 1),
                           (x0, y1, 1, -1), (x1, y1, -1, -1)):
        cv2.line(canvas, (cx, cy), (cx + dx * tick, cy), _CYAN, lw * 2, cv2.LINE_AA)
        cv2.line(canvas, (cx, cy), (cx, cy + dy * tick), _CYAN, lw * 2, cv2.LINE_AA)


def _banner(canvas, lines, colour, fs, lw):
    """Draw a status banner top-left: near-opaque box (legible over bright
    scenes), verdict-coloured headline, white detail lines, colour frame."""
    pad = int(12 * fs)
    heights = [cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, fs, lw)[0][1]
               for t in lines]
    strip_h = sum(heights) + pad * (len(lines) + 1)
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (canvas.shape[1], strip_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.82, canvas, 0.18, 0, canvas)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1] - 1, strip_h),
                  colour, max(2, lw))
    y = pad
    for i, (text, h) in enumerate(zip(lines, heights)):
        y += h
        cv2.putText(canvas, text, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, fs,
                    colour if i == 0 else _WHITE, lw + (i == 0), cv2.LINE_AA)
        y += pad


def _footer(canvas, text, fs, lw):
    """Draw a controls hint bottom-left over an opaque strip."""
    th = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, lw)[0][1]
    pad = int(10 * fs)
    h = canvas.shape[0]
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, h - th - 2 * pad), (canvas.shape[1], h),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.82, canvas, 0.18, 0, canvas)
    cv2.putText(canvas, text, (pad, h - pad), cv2.FONT_HERSHEY_SIMPLEX, fs,
                _WHITE, lw, cv2.LINE_AA)


def _framing_verdict(cor, tilt, fill, exposure, qa=None, is_raw=True,
                     focus=None, cor_good=_COR_GOOD):
    """(colour, headline) for the framing + exposure status.

    Binary green/red by preference: green only when confidence, tilt, fill and
    exposure are all in band and QA passes; otherwise red naming the one fix.
    Measured faults (tilt, fill, exposure, QA) are reported ahead of a bare low
    correlation, since those usually drag the match down and give an actionable
    instruction. Fails closed: once a chart is located, a missing measurement
    means the sampler tripped, which is not a green light. A superset of the QA
    gate: anything QA fails, this fails too, and glare / uneven light QA only
    warns on also blocks here since the operator can fix it live.
    """
    if tilt is None or fill is None:
        return _RED, 'UNSTABLE - re-seat the chart, hold steady'
    if tilt >= _TILT_FAIL:
        return _RED, 'TILTED - face the chart square-on'
    if fill < _FILL_LOW:
        return _RED, 'TOO SMALL - move closer / fill the box'
    if fill > _FILL_HIGH:
        return _RED, 'TOO CLOSE - back off, keep edges in frame'
    # Exposure is RAW-only; JPEG previews carry no linear levels to grade.
    if is_raw and exposure is None:
        return _RED, 'UNSTABLE - re-seat the chart, hold steady'
    if exposure is not None:
        if exposure['band'] == 'clipped':
            return _RED, 'CLIPPED - lower the shutter'
        if exposure['band'] == 'low':
            return _RED, 'TOO DARK - raise the exposure'
        if exposure['band'] == 'high':
            return _RED, 'TOO BRIGHT - ease the exposure'
    # Focus is best-effort, so unlike tilt/fill a missing score does not block.
    if focus is not None and focus < _FOCUS_MIN:
        return _RED, 'OUT OF FOCUS - refocus / steady'
    # Glare / uneven light: steer on QA's spread and neutral-ramp signal.
    if qa is not None:
        if qa.get('spread', 0.0) >= _GLARE_SPREAD:
            return _RED, 'GLARE / UNEVEN LIGHT - kill hotspots, diffuse'
        if qa.get('ramp_inversions', 0):
            return _RED, 'UNEVEN LIGHT - level the lightbox across the chart'
    if tilt >= _TILT_WARN:
        return _RED, 'TILTED - face the chart square-on'
    if qa and qa.get('severity') == 'FAIL':
        return _RED, 'QA FAIL - fix clipping / lighting before shooting'
    if cor < cor_good:
        return _RED, 'WEAK MATCH - square up / steady the chart'
    return _GREEN, 'GOOD - ready to shoot'


def annotate(preview, coords, cor, qa, clipped, is_raw, exposure=None,
             focus=None, cct=None, max_w=1600, controls=None,
             cor_good=_COR_GOOD):
    """Draw the overlay and return (summary, annotated_canvas)."""
    scale = min(1.0, max_w / preview.shape[1]) if preview.shape[1] else 1.0
    if scale < 1.0:
        canvas = cv2.resize(preview, None, fx=scale, fy=scale,
                            interpolation=cv2.INTER_AREA)
        if coords is not None:
            coords = (coords[0] * scale, coords[1] * scale)
    else:
        canvas = preview.copy()

    fs = max(0.55, canvas.shape[1] / 1200.0)
    lw = max(1, round(fs * 1.6))

    if coords is None:
        # Only show the alignment guide while searching; once the chart locks the
        # detected grid is the reference and a second box just clutters the view.
        _draw_guides(canvas)
        _banner(canvas, ['NO CHART FOUND',
                         'move the chart into the cyan box, face-on'],
                _RED, fs, lw)
        summary = 'not located'
    else:
        tilt = _tilt(coords[1][0])
        fill = _fill_fraction(coords, canvas.shape)
        colour, headline = _framing_verdict(cor, tilt, fill, exposure, qa,
                                            is_raw, focus, cor_good)
        _draw_grid(canvas, coords, clipped, colour, lw)

        tilt_txt = 'n/a' if tilt is None else f'{tilt * 100:.0f}%'
        fill_txt = 'n/a' if fill is None else f'{fill * 100:.0f}%'
        exp_txt = ('n/a' if exposure is None
                   else f"{exposure['level'] * 100:.0f}% ({exposure['band']})")
        focus_txt = 'n/a' if focus is None else f'{focus * 1e3:.1f}'
        qa_txt = qa['severity'] if qa else ('n/a (no raw)' if not is_raw else 'n/a')
        glare_txt = 'n/a' if not qa else f"{qa.get('spread', 0.0) * 100:.0f}%"
        cct_txt = f'   CCT {cct}K' if cct else ''
        lines = [headline,
                 f'confidence {cor:.2f}   fill {fill_txt}   tilt {tilt_txt}',
                 f'exposure {exp_txt}   focus {focus_txt}   glare {glare_txt}'
                 f'   QA {qa_txt}{cct_txt}']
        if qa and qa['issues']:
            lines += [f'- {i}' for i in qa['issues']]
        _banner(canvas, lines, colour, fs, lw)
        summary = (f'located cor={cor:.2f} fill={fill_txt} tilt={tilt_txt} '
                   f'exposure={exp_txt} focus={focus_txt} glare={glare_txt} '
                   f'qa={qa_txt}' + (f' cct={cct}K' if cct else ''))

    # Footer: the operator's controls hint, plus a concrete capture-setting fix
    # when the exposure is out of band so dim/bright frames carry their remedy.
    footer = controls or ''
    if exposure is not None and exposure['band'] in _EXP_HINT:
        hint = _EXP_HINT[exposure['band']]
        footer = f'{footer}   |   {hint}' if footer else hint
    if footer:
        _footer(canvas, footer, fs, lw)
    return summary, canvas


# --------------------------------------------------------------------------
# Target capture
# --------------------------------------------------------------------------

# Shared SSH/scp options. BatchMode never prompts (key-based auth only, so an
# unattended --watch cannot hang); accept-new trusts a target's key on first
# connect (writing this host's own ~/.ssh/known_hosts) so a fresh clone does not
# dead-end on host-key verification. Host/user/port/identity/proxy all still
# come from the user's ~/.ssh/config, so --host may be a config alias.
def _ssh_opts(connect_timeout=None):
    opts = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new']
    if connect_timeout is not None:
        opts += ['-o', f'ConnectTimeout={connect_timeout}']
    return opts


def _run(cmd, timeout):
    """Run an ssh/scp command, turning a missing binary or hang into a clear
    error."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout)
    except FileNotFoundError:
        raise RuntimeError(f'{cmd[0]} not found -- install openssh-client')
    except subprocess.TimeoutExpired:
        raise RuntimeError(f'{cmd[0]} timed out after {timeout}s -- check the '
                           'network/ethernet cable and --host')


def _check_reachable(target, connect_timeout):
    """Verify the target answers over SSH, else raise an actionable error.

    ssh exits 255 for any connect/auth failure (no route, refused, timed out,
    key rejected), which is distinct from a remote command's own exit code.
    """
    r = _run(['ssh', *_ssh_opts(connect_timeout), target, 'true'],
             connect_timeout + 5)
    if r.returncode == 0:
        return
    hint = ('check the network/ethernet cable and --host, then run '
            '"--setup-ssh --host <board>" once for passwordless login; '
            'this tool never prompts for a password')
    raise RuntimeError(f'cannot reach {target} over SSH: '
                       f'{r.stderr.strip() or "no response"}\n  {hint}')


def setup_ssh(args):
    """One-time passwordless-SSH bootstrap for a fresh clone.

    Ensures a local ed25519 key exists, copies its public half to the target
    (the single password prompt), then verifies key-based login. After this,
    --capture/--watch and the detached sweep run without a prompt.
    """
    if not args.host:
        raise RuntimeError('--setup-ssh needs --host <board> (and optional --user)')
    target = f'{args.user}@{args.host}' if args.user else args.host

    key = Path.home() / '.ssh' / 'id_ed25519'
    if key.exists():
        logger.info(f'reusing existing key {key}')
    else:
        logger.info(f'no key at {key} -- generating one (ed25519, no passphrase)')
        key.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        if subprocess.run(['ssh-keygen', '-t', 'ed25519', '-N', '',
                           '-f', str(key)]).returncode != 0:
            raise RuntimeError('ssh-keygen failed')

    # Drop any stale host key so a reflashed board or a new board reusing an old
    # IP does not dead-end on a host-key mismatch (accept-new re-trusts below).
    subprocess.run(['ssh-keygen', '-R', args.host], capture_output=True)

    logger.info(f'copying key to {target} -- enter the board password once')
    # Interactive: the prompt must reach the operator, so no capture, no BatchMode.
    if subprocess.run(['ssh-copy-id', '-i', str(key),
                       '-o', 'StrictHostKeyChecking=accept-new',
                       target]).returncode != 0:
        raise RuntimeError('ssh-copy-id failed -- check the password and --host')

    v = _run(['ssh', *_ssh_opts(args.ssh_timeout), target, 'true'],
             args.ssh_timeout + 5)
    if v.returncode != 0:
        raise RuntimeError('key copied but passwordless login still fails: '
                           f'{v.stderr.strip() or "no response"}')
    logger.info(f'passwordless SSH to {target} is ready -- '
                '--capture/--watch and the sweep no longer prompt')


def capture_from_target(args, workdir, shutter=None):
    """Run one capture on the target machine over SSH and pull it back.

    The capture runs in the foreground (ssh blocks until it finishes) because
    the frame is needed before it can be analysed and a single frame is quick;
    if the link drops mid-capture we simply retry on the next --watch tick.
    --capture-cmd is the target-side command (its ``{outdir}`` and ``{shutter}``
    tokens are replaced with the remote output directory and the ramp's shutter
    in us) and must drop a raw .pgm there, optionally with a matching .json
    sidecar. Frames are pulled to fixed local names so a long --watch run does
    not accumulate files. Returns the local .pgm path.
    """
    outdir = args.remote_outdir.strip().rstrip('/')
    # It is wiped with "rm -rf" on the board, so reject anything but a plain
    # absolute path under a scratch prefix.
    if (not outdir.startswith(_REMOTE_OUTDIR_PREFIXES) or '..' in outdir.split('/')
            or any(c in outdir for c in ' \t\n;&|$`')):
        raise RuntimeError(
            f'--remote-outdir {args.remote_outdir!r} must be an absolute scratch '
            f'path (one of {", ".join(_REMOTE_OUTDIR_PREFIXES)}), no "..", '
            'no shell metacharacters (it is wiped with "rm -rf" each capture)')

    workdir.mkdir(parents=True, exist_ok=True)
    target = f'{args.user}@{args.host}'
    _check_reachable(target, args.ssh_timeout)
    q_outdir = shlex.quote(outdir)
    shutter_us = str(int(shutter if shutter is not None else _RAMP_START))
    cmd = (args.capture_cmd.replace('{outdir}', outdir)
                           .replace('{shutter}', shutter_us))
    remote = f'rm -rf {q_outdir} && mkdir -p {q_outdir} && {cmd}'
    logger.info(f'[target] capturing on {target} ...')
    r = _run(['ssh', *_ssh_opts(args.ssh_timeout), target, remote],
             args.timeout)
    if r.returncode == 255:
        _check_reachable(target, args.ssh_timeout)  # raises with the reason
    if r.returncode != 0:
        raise RuntimeError(f'capture command failed on {target} '
                           f'(rc={r.returncode}): '
                           f'{(r.stderr or r.stdout).strip() or "no output"}')

    find = f'ls -t {q_outdir}/*.pgm 2>/dev/null | head -1'
    q = _run(['ssh', *_ssh_opts(args.ssh_timeout), target, find],
             args.ssh_timeout)
    if q.returncode == 255:
        _check_reachable(target, args.ssh_timeout)  # link dropped, not a bad cmd
    newest = q.stdout.strip()
    if not newest:
        raise RuntimeError(f'no raw .pgm produced in {outdir} on {target} '
                           '(check --capture-cmd)')

    # Tag the local name with the illuminant CCT (ct<K>K, as mchp_capture.sh
    # stems do) so a multi-CT session's frames do not overwrite each other;
    # unset keeps the fixed name so --watch behaviour is unchanged.
    stem = f'latest_raw_ct{args.cct}K' if args.cct else 'latest_raw'
    local_pgm = workdir / f'{stem}.pgm'
    remote_json = newest[:-len('.pgm')] + '.json'
    for src, dst, required in ((newest, local_pgm, True),
                               (remote_json, workdir / f'{stem}.json',
                                False)):
        dst.unlink(missing_ok=True)  # avoid pairing a new .pgm with a stale one
        rc = _run(['scp', *_ssh_opts(args.ssh_timeout), f'{target}:{src}',
                   str(dst)], args.ssh_timeout).returncode
        if rc != 0 and required:
            raise RuntimeError(f'failed to pull {src} from {target}')
        if rc != 0:
            logger.debug(f'no sidecar at {src}; QA will use built-in defaults')
            continue
        # A 0-byte pull is a silent half-transfer, not a usable frame.
        if required and dst.stat().st_size == 0:
            raise RuntimeError(f'pulled {src} from {target} but it is empty')
    return local_pgm


def _ramp_step(band, shutter, dark_hi, bright_lo):
    """Next shutter (us) given the current band and the known bracket, or None.

    dark_hi / bright_lo are the highest too-dark and lowest clipped shutters seen
    so far (None until bracketed). With both known the good exposure lies between
    them, so bisect the geometric midpoint; otherwise widen by halving (clipped)
    or doubling (dark) to find the far edge. Clamped; None when it cannot move.
    """
    if band not in ('clipped', 'low'):
        return None
    if dark_hi is not None and bright_lo is not None:
        nxt = (dark_hi * bright_lo) ** 0.5
    elif band == 'clipped':
        nxt = shutter * 0.5
    else:
        nxt = shutter * 2.0
    nxt = int(min(_RAMP_SHUTTER_MAX, max(_RAMP_SHUTTER_MIN, nxt)))
    return None if nxt == shutter else nxt


def capture_well_exposed(args, ref_data, workdir, start_shutter=None):
    """Capture, grading exposure and re-grabbing until the band is usable.

    Ramps the shutter from start_shutter (or --shutter) until _exposure reads
    'good'/'high', the chart is lost, the shutter rails, or the try cap is hit;
    then returns (local_pgm, settled_shutter). A capture-cmd that meters its own
    exposure ignores {shutter}, so the first frame grades usable and the ramp is
    a single grab. Returns the best (closest-to-good) frame if it never settles.
    """
    cor_good = _COR_GOOD
    shutter = int(start_shutter if start_shutter is not None else args.shutter)
    best = None  # (distance-from-good, pgm, shutter); set once a frame is out of band
    dark_hi = bright_lo = None  # bracket: highest too-dark / lowest clipped shutter
    pgm = None
    for _ in range(_RAMP_MAX_TRIES):
        pgm = capture_from_target(args, workdir, shutter)
        _, det_input, image = _load(pgm)
        # _exposure needs image.patches, which only get filled once the chart is
        # located, so run locate + the QA sampler here as run_once does.
        _, coords = locate(det_input, ref_data, cor_good)
        _qa(image, coords)
        exposure = _exposure(image)
        # A chart too dark to locate reads as None; treat it as under-exposed and
        # keep brightening (the try cap bounds a blind ramp).
        band = exposure['band'] if exposure else ('low' if coords is None else None)
        located = '' if coords is not None else ' (not located)'
        logger.info(f'ramp: shutter={shutter}us -> {band or "no chart"}{located}')
        if args.no_exposure_ramp or band in (None, 'good', 'high'):
            return pgm, shutter
        if band == 'clipped':
            bright_lo = shutter if bright_lo is None else min(bright_lo, shutter)
        else:  # low / dark
            dark_hi = shutter if dark_hi is None else max(dark_hi, shutter)
        if exposure is not None:
            # Rank frames by how far the level sits outside the good window.
            level = exposure['level']
            dist = _EXP_LOW - level if band == 'low' else level - _EXP_HIGH
            if best is None or dist < best[0]:
                best = (dist, pgm, shutter)
        nxt = _ramp_step(band, shutter, dark_hi, bright_lo)
        if nxt is None:
            return pgm, shutter
        shutter = nxt
    # Out of tries: best is the closest graded frame, else the last grab.
    logger.info('ramp: exposure did not settle within '
                f'{_RAMP_MAX_TRIES} tries; using the best frame')
    if best is not None:
        return best[1], best[2]
    assert pgm is not None  # _RAMP_MAX_TRIES >= 1, so the loop grabbed once
    return pgm, shutter


# --------------------------------------------------------------------------
# Live stream source
# --------------------------------------------------------------------------

class GstFrameSource:
    """Pull the latest decoded frame off a GStreamer appsink as BGR."""

    def __init__(self, desc):
        try:
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst
        except (ImportError, ValueError) as e:
            raise RuntimeError('--stream needs the GStreamer 1.0 Python '
                               'bindings (python3-gi and gst plugins)') from e
        self._Gst = Gst
        Gst.init(None)
        tail = (' ! videoconvert ! video/x-raw,format=BGR ! appsink '
                'name=fa_sink max-buffers=2 drop=true sync=false')
        try:
            self._pipe = Gst.parse_launch(desc + tail)
        except Exception as e:
            raise RuntimeError(f'bad --stream pipeline: {e}') from e
        self._sink = self._pipe.get_by_name('fa_sink')
        if self._pipe.set_state(Gst.State.PLAYING) == \
                Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('--stream pipeline failed to start; check the '
                               'source and network')

    def grab(self, timeout_s):
        """Latest frame as a BGR ndarray, or None on timeout/EOS."""
        Gst = self._Gst
        sample = self._sink.try_pull_sample(int(timeout_s * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        s = sample.get_caps().get_structure(0)
        w, h = s.get_value('width'), s.get_value('height')
        if not w or not h:
            return None
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            # appsink may pad each row to a stride wider than w*3; slice it off.
            stride = mi.size // h
            rows = np.frombuffer(mi.data, dtype=np.uint8,
                                 count=stride * h).reshape(h, stride)
            return rows[:, :w * 3].reshape(h, w, 3).copy()
        finally:
            buf.unmap(mi)

    def close(self):
        self._pipe.set_state(self._Gst.State.NULL)


# --------------------------------------------------------------------------
# Output: live window or file
# --------------------------------------------------------------------------

_WIN = 'mchp frame assist'
_gui_ok = None  # None until probed, then True/False for the rest of the run


def _probe_gui():
    """Return True only if an OpenCV highgui window can actually open.

    OpenCV's Qt/GTK backend calls abort() -- not a catchable exception -- when
    no display is reachable, so the probe runs in a throwaway subprocess whose
    crash cannot take this process down.
    """
    probe = ('import cv2; cv2.namedWindow("_probe", cv2.WINDOW_NORMAL); '
             'cv2.destroyAllWindows()')
    try:
        return subprocess.run([sys.executable, '-c', probe],
                              capture_output=True, timeout=15).returncode == 0
    except Exception:
        return False


def _show(canvas, wait_ms):
    """Show canvas in a live window.

    Returns True to keep going, False if the user asked to quit (q/Esc or closed
    the window), or None if this host has no usable GUI -- the caller then falls
    back to writing a file. wait_ms=0 blocks until a key is pressed (single
    shot); a positive value is the --watch refresh interval.
    """
    global _gui_ok
    if _gui_ok is None:
        _gui_ok = _probe_gui()
    if not _gui_ok:
        return None
    try:
        cv2.imshow(_WIN, canvas)
        key = cv2.waitKey(max(0, int(wait_ms))) & 0xFF
        if cv2.getWindowProperty(_WIN, cv2.WND_PROP_VISIBLE) < 1:
            return False
    except cv2.error:
        _gui_ok = False
        return None
    return key not in (27, ord('q'))


def _show_blocking(canvas):
    """Show canvas and block until the operator acts: 'refresh' (r/Space),
    'quit' (q/Esc or window closed), or None if this host has no GUI."""
    global _gui_ok
    if _gui_ok is None:
        _gui_ok = _probe_gui()
    if not _gui_ok:
        return None
    try:
        cv2.imshow(_WIN, canvas)
        while True:
            key = cv2.waitKey(100) & 0xFF
            if cv2.getWindowProperty(_WIN, cv2.WND_PROP_VISIBLE) < 1:
                return 'quit'
            if key in (27, ord('q')):
                return 'quit'
            if key in (ord('r'), ord('R'), 32):
                return 'refresh'
    except cv2.error:
        _gui_ok = False
        return None


def _write(path, canvas):
    cv2.imwrite(str(path), canvas)
    return path


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def run_once(args, ref_data, workdir, source=None, controls=None):
    """Capture/load, analyse and annotate one frame; return (located, canvas)."""
    settled = ''
    if source is not None:
        bgr = source.grab(args.stream_timeout)
        if bgr is None:
            raise RuntimeError('no frame from --stream (timeout/EOS); check '
                               'the target pipeline and network')
        name, (preview, det_input, image) = 'stream', (bgr, bgr, None)
    elif args.local:
        name, (preview, det_input, image) = args.local, _load(Path(args.local))
    else:
        # Carry the settled shutter across --watch ticks so a stable scene does
        # not re-ramp from scratch each interval.
        src, shutter = capture_well_exposed(args, ref_data, workdir,
                                            args._ramp_shutter)
        args._ramp_shutter = shutter
        settled = f' [--shutter {shutter}]'
        name, (preview, det_input, image) = src.name, _load(src)
    cor_good = _COR_GOOD
    cor, coords = locate(det_input, ref_data, cor_good)
    qa, clipped = _qa(image, coords)
    exposure = _exposure(image)
    # det_input is a grayscale plane for RAW and BGR for JPEG/stream.
    gray = (det_input if det_input.ndim == 2
            else cv2.cvtColor(det_input, cv2.COLOR_BGR2GRAY))
    focus = _focus(gray, coords)
    cct = getattr(args, 'cct', None)
    summary, canvas = annotate(preview, coords, cor, qa, clipped,
                               image is not None, exposure=exposure,
                               focus=focus, cct=cct, controls=controls,
                               cor_good=cor_good)
    logger.info(f'{name}: {summary}{settled}')
    return coords is not None, canvas


def build_parser():
    p = argparse.ArgumentParser(
        description='Macbeth framing assistant for ISC calibration captures.')
    src = p.add_mutually_exclusive_group()
    src.add_argument('--local', metavar='FILE',
                     help='analyse an existing capture (.pgm/.jpg/.png)')
    src.add_argument('--capture', action='store_true',
                     help='run a capture on the target over SSH (default)')
    src.add_argument('--stream', metavar='PIPELINE',
                     help='analyse a live stream: the receive/decode half of a '
                          'gst-launch line ending at a raw-video pad (framing '
                          'only, always live)')
    p.add_argument('--stream-timeout', type=float, default=5.0,
                   help='seconds to wait for a --stream frame (default 5)')
    p.add_argument('--watch', action='store_true',
                   help='repeat on an interval until q/Esc')
    p.add_argument('--manual', action='store_true',
                   help='live window that refreshes only on demand: adjust the '
                        'chart, press R/Space to re-check, Q/Esc to quit '
                        '(no timed loop)')
    p.add_argument('--interval', type=float, default=3.0,
                   help='seconds between --watch iterations (default 3)')
    p.add_argument('-o', '--output', metavar='PATH',
                   help='write the annotated preview to PATH (single-shot '
                        'defaults to frame_assist.png; --watch shows a live '
                        'window and writes nothing unless PATH is given)')
    disp = p.add_mutually_exclusive_group()
    disp.add_argument('--show', dest='show', action='store_true', default=None,
                      help='pop up a live window (default in --watch)')
    disp.add_argument('--no-show', dest='show', action='store_false',
                      help='never open a window; write to a file instead')

    p.add_argument('--setup-ssh', action='store_true',
                   help='one-time: copy your SSH key to --host for passwordless '
                        'access (one password prompt), then exit')

    b = p.add_argument_group('target capture')
    b.add_argument('--host', help='target machine IP/hostname '
                   '(required for --capture)')
    b.add_argument('--user', default='root', help='target SSH user')
    b.add_argument('--capture-cmd',
                   default='mchpcam-still --raw --width=1920 --height=1080 '
                           '--no-agc --analoggain=1.0 --shutter={shutter} '
                           '-o {outdir}/frame.pgm',
                   help='target-side capture command; {outdir} and {shutter} '
                        'are replaced with --remote-outdir and the ramp shutter '
                        '(us). It must write a raw .pgm there. Defaults to a '
                        'single unity-gain mchpcam-still grab for framing, not '
                        'the calibration sweep (run mchp_capture.sh for that; it '
                        'auto-detaches over SSH)')
    b.add_argument('--shutter', type=int, default=_RAMP_START, metavar='US',
                   help=f'exposure in us; the auto-ramp start point (default '
                        f'{_RAMP_START}). With --no-exposure-ramp it is the '
                        'fixed shutter for a single grab')
    b.add_argument('--no-exposure-ramp', action='store_true',
                   help='take one grab at --shutter instead of ramping until '
                        'the exposure is in band')
    b.add_argument('--remote-outdir', default='/root/frame_assist',
                   help='target-side capture directory')
    b.add_argument('--cct', type=int, metavar='K',
                   help='illuminant colour temperature (Kelvin) of the lightbox; '
                        'tags the pulled frame ct<K>K to match mchp_capture.sh')
    b.add_argument('--ssh-timeout', type=int, default=10,
                   help='SSH connect/transfer timeout (s)')
    b.add_argument('--timeout', type=int, default=120,
                   help='overall capture timeout (s)')
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.setup_ssh:
        try:
            setup_ssh(args)
        except Exception as e:
            logger.error(f'error: {e}')
            return 1
        return 0
    if not args.local and not args.host and not args.stream:
        parser.error('one of --local FILE, --host (capture) or --stream is '
                     'required')
    if args.local and args.host:
        logger.warning('--host is ignored when --local FILE is given')
    args._ramp_shutter = args.shutter  # carried + refined across --watch ticks
    ref_data = _build_ref_data()
    base = Path(args.output).resolve().parent if args.output else Path.cwd()
    workdir = base / '.frame_assist_captures'
    show = args.watch if args.show is None else args.show

    source = None
    if args.stream:
        try:
            source = GstFrameSource(args.stream)
        except Exception as e:
            logger.error(f'error: {e}')
            return 1

    if args.manual:
        controls = '[R]/Space refresh   [Q]/Esc quit'
        logger.info('manual mode: adjust the chart, R/Space to re-check, '
                    'Q/Esc to quit')
        try:
            while True:
                try:
                    _, canvas = run_once(args, ref_data, workdir,
                                         source=source, controls=controls)
                except Exception as e:
                    logger.error(f'error: {e}')
                    time.sleep(1)
                    continue
                action = _show_blocking(canvas)
                if action is None:
                    out = args.output or 'frame_assist.png'
                    logger.info(f'no display available; wrote {_write(out, canvas)}'
                                ' -- manual mode needs a window')
                    break
                if action == 'quit':
                    break
        except KeyboardInterrupt:
            logger.info('stopped.')
        finally:
            cv2.destroyAllWindows()
        return 0

    if not args.watch:
        try:
            _, canvas = run_once(args, ref_data, workdir, source=source)
        except Exception as e:
            logger.error(f'error: {e}')
            return 1
        out = args.output or 'frame_assist.png'
        logger.info(f'wrote {_write(out, canvas)}')
        if show and _show(canvas, 0) is None:
            logger.info('(no display available for --show)')
        cv2.destroyAllWindows()
        return 0

    logger.info('watching (q or Esc to stop) ...')
    try:
        while True:
            canvas = None
            try:
                _, canvas = run_once(args, ref_data, workdir, source=source)
            except Exception as e:
                logger.error(f'error: {e}')
            if canvas is None:
                time.sleep(args.interval)
                continue
            shown = _show(canvas, args.interval * 1000) if show else None
            if shown is False:
                break                       # user asked to quit
            if shown is None:               # no GUI, or --no-show
                dst = args.output or 'frame_assist.png'
                if show:                    # a window was wanted but unavailable
                    show = False
                    logger.info(f'no display available; writing frames to {dst}'
                                ' -- open it in a viewer that auto-refreshes')
                _write(dst, canvas)
                time.sleep(args.interval)
            elif args.output:               # window shown and a path was given
                _write(args.output, canvas)  # waitKey already paced the loop
    except KeyboardInterrupt:
        logger.info('stopped.')
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
