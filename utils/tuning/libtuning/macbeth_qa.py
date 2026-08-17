# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Macbeth capture quality assurance.
#
# A calibration is only as good as the charts it is solved from. The AWB, CCM
# and lux modules trust every located Macbeth capture equally, so a tilted,
# glared or unevenly-lit chart silently biases the emitted tuning. This module
# inspects each located chart -- reusing the sorted per-patch pixels and centre
# geometry that Image.get_patches() already filled -- and flags three capture
# faults before they reach the solvers:
#
#   clipping   any patch whose brightest pixels sit at the sensor maximum, so
#              its mean is biased low (a solve reads a clipped white as darker
#              than it is).
#   tilt       chart not fronto-parallel: perspective foreshortening makes one
#              side of the chart smaller than the other, measured from the
#              located patch-centre grid.
#   non-uniform  illumination gradient across the chart (neutral ramp no longer
#              monotonic) or within a patch (high intra-patch spread).
#
# The verdicts feed the health summary as an advisory 'MacbethQA' stage; they
# never abort the run.

import logging

import numpy as np

logger = logging.getLogger(__name__)

PASS = 'PASS'
WARN = 'WARN'
FAIL = 'FAIL'
_RANK = {PASS: 0, WARN: 1, FAIL: 2}


def _worst(a, b):
    return a if _RANK[a] >= _RANK[b] else b


# Neutral (greyscale) ramp of the located chart, white -> black. get_patches()
# stacks the chart columns left-to-right, so the four-per-column bottom row is
# 3, 7, 11, 15, 19, 23 (white ... black). The bright end (3..15) has the SNR to
# measure intra-patch uniformity; the dark end is noise-dominated and used only
# for the monotonicity check.
_GREY_RAMP = (3, 7, 11, 15, 19, 23)
_GREY_BRIGHT = (3, 7, 11, 15)

# Chart geometry: 6 columns of 4 patches. The located centres are in the same
# column-stacked order, so patch index = column * 4 + row (row 0 = top).
_N_COLS = 6
_N_ROWS = 4

# Perspective foreshortening thresholds: fractional size difference between the
# two sides of the chart. A fronto-parallel chart is ~0; 6% warns, 12% fails.
_TILT_WARN = 0.06
_TILT_FAIL = 0.12

# Intra-patch spread (p85-p15 over the mean) of a bright neutral patch. Even
# illumination leaves only sensor noise; a lighting gradient or glare across the
# patch widens it. Advisory only (noise-sensitive), so WARN, never FAIL.
_SPREAD_WARN = 0.12


def _white_level(image):
    sigbits = getattr(image, 'sigbits', 16)
    return ((1 << sigbits) - 1) << (16 - sigbits)


def _clipping(image, white):
    """Located patches whose brightest pixels reach the sensor maximum.

    Patch pixels are sorted ascending, so the top element flags saturation in
    that channel. A clipped patch mean is biased low and must not feed a solve.
    """
    patches = image.patches
    clipped = [p for p in range(patches.shape[1])
               if any(patches[ci][p][-1] >= white for ci in range(4))]
    return clipped


def _tilt(image):
    """Perspective foreshortening of the located patch-centre grid.

    Returns the larger of the two side-to-side size ratios (left/right column
    height, top/bottom row width). 0 for a fronto-parallel chart. None when the
    centre grid is not the expected 24-patch layout.
    """
    cen = getattr(image, 'cen_coords', None)
    if cen is None or len(cen) != _N_COLS * _N_ROWS:
        return None

    pts = np.array([(float(c[0]), float(c[1])) for c in cen])

    def grid(col, row):
        return pts[col * _N_ROWS + row]

    def dist(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    left_h = dist(grid(0, 0), grid(0, _N_ROWS - 1))
    right_h = dist(grid(_N_COLS - 1, 0), grid(_N_COLS - 1, _N_ROWS - 1))
    top_w = dist(grid(0, 0), grid(_N_COLS - 1, 0))
    bot_w = dist(grid(0, _N_ROWS - 1), grid(_N_COLS - 1, _N_ROWS - 1))

    def asym(a, b):
        hi = max(a, b)
        return abs(a - b) / hi if hi > 1e-6 else 0.0

    return max(asym(left_h, right_h), asym(top_w, bot_w))


def _grey_luma(image, bl):
    """Black-subtracted green luminance of each neutral-ramp patch."""
    patches = image.patches
    luma = {}
    for p in _GREY_RAMP:
        g = (patches[1][p].astype(np.float64).mean() +
             patches[2][p].astype(np.float64).mean()) / 2.0 - bl
        luma[p] = g
    return luma


def _ramp_inversions(luma):
    """Count neutral-ramp steps that fail to darken (white -> black).

    A monotonic ramp confirms even illumination and a correct chart location; an
    inversion points to a gradient across the chart or a misplaced grid.
    """
    seq = [luma[p] for p in _GREY_RAMP]
    return sum(1 for i in range(len(seq) - 1) if seq[i + 1] > seq[i] + 1e-6)


def _intra_patch_spread(image, bl):
    """Worst (p85-p15)/mean over the bright neutral patches, green channel.

    Sorted patch pixels make the robust spread a slice lookup, and the p15/p85
    percentiles ignore a few outlier pixels. Glare or a lighting gradient is
    usually confined to part of the chart, so the worst neutral patch (not the
    median) is the signal -- a single glared patch is still a capture fault.
    """
    patches = image.patches
    worst = 0.0
    for p in _GREY_BRIGHT:
        px = patches[1][p].astype(np.float64)
        n = px.shape[0]
        if n < 8:
            continue
        mean = px.mean() - bl
        if mean <= 0.0:
            continue
        lo = px[int(0.15 * (n - 1))]
        hi = px[int(0.85 * (n - 1))]
        worst = max(worst, float((hi - lo) / mean))
    return worst


def qa_image(image):
    """Quality verdict for one located Macbeth capture.

    Returns a dict {name, severity, issues[], clipped, tilt, ramp_inversions,
    spread} or None when the chart was not located (no patches to inspect).
    """
    if getattr(image, 'patches', None) is None:
        return None

    white = _white_level(image)
    bl = getattr(image, 'blacklevel_16', 0) or 0

    severity = PASS
    issues = []

    # -- Clipping -----------------------------------------------------------
    clipped = _clipping(image, white)
    if len(clipped) >= 3:
        severity = _worst(severity, FAIL)
        issues.append(f'{len(clipped)} clipped patches (lower exposure)')
    elif clipped:
        severity = _worst(severity, WARN)
        issues.append(f'{len(clipped)} clipped patch(es) at {clipped}')

    # -- Tilt / perspective -------------------------------------------------
    tilt = _tilt(image)
    if tilt is None:
        severity = _worst(severity, WARN)
        issues.append('patch grid not 6x4; cannot assess tilt')
    elif tilt >= _TILT_FAIL:
        severity = _worst(severity, FAIL)
        issues.append(f'tilt {tilt * 100:.0f}% (reshoot fronto-parallel)')
    elif tilt >= _TILT_WARN:
        severity = _worst(severity, WARN)
        issues.append(f'tilt {tilt * 100:.0f}%')

    # -- Non-uniform illumination -------------------------------------------
    luma = _grey_luma(image, bl)
    inversions = _ramp_inversions(luma)
    if inversions:
        severity = _worst(severity, WARN)
        issues.append(f'neutral ramp non-monotonic ({inversions} inversion(s); '
                      f'uneven light or misplaced grid)')

    spread = _intra_patch_spread(image, bl)
    if spread >= _SPREAD_WARN:
        severity = _worst(severity, WARN)
        issues.append(f'intra-patch spread {spread * 100:.0f}% (uneven light/glare)')

    return {
        'name': image.name,
        'severity': severity,
        'issues': issues,
        'clipped': len(clipped),
        'tilt': None if tilt is None else round(tilt, 4),
        'ramp_inversions': inversions,
        'spread': round(spread, 4),
    }


def qa_images(images):
    """QA verdicts for every located Macbeth capture, worst-first.

    Returns {'worst': severity, 'images': [verdict, ...]} or None when no chart
    was located (nothing to assess).
    """
    verdicts = []
    for img in images:
        v = qa_image(img)
        if v is not None:
            verdicts.append(v)
    if not verdicts:
        return None
    verdicts.sort(key=lambda v: _RANK[v['severity']], reverse=True)
    worst = PASS
    for v in verdicts:
        worst = _worst(worst, v['severity'])
    return {'worst': worst, 'images': verdicts}
