#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Board-free checks for the Macbeth capture QA (libtuning/macbeth_qa.py). Builds
# synthetic located charts (per-patch sorted pixels + centre grid, exactly what
# Image.get_patches() fills) with clipping, tilt and uneven-light faults injected
# and confirms each is flagged while a clean chart passes.

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from libtuning.macbeth_qa import qa_image, qa_images, PASS, WARN, FAIL

_SIGBITS = 10
_SHIFT = 16 - _SIGBITS
_WHITE = ((1 << _SIGBITS) - 1) << _SHIFT      # 65472
_BLACK = 64 << _SHIFT                         # 4096

# Neutral ramp levels white -> black (16-bit), comfortably below clipping.
_RAMP = {3: 60000, 7: 45000, 11: 32000, 15: 22000, 19: 12000, 23: 6500}


class _StubImage:
    """Minimal stand-in carrying only what qa_image() reads."""

    def __init__(self, name):
        self.name = name
        self.sigbits = _SIGBITS
        self.blacklevel_16 = _BLACK
        self.patches: "np.ndarray | None" = None
        self.cen_coords: "list | None" = None


def _flat_patch(level, n=64):
    """A uniform patch: n identical sorted pixels at `level`."""
    return np.full(n, level, dtype=np.int64)


def _clean_patches():
    """[4][24][n] neutrals on the ramp, colour patches mid-grey, none clipped."""
    patches = np.zeros((4, 24, 64), dtype=np.int64)
    for p in range(24):
        level = _RAMP.get(p, 30000)
        for ci in range(4):
            patches[ci][p] = _flat_patch(level)
    return patches


def _clean_grid():
    """Evenly-spaced 6x4 centre grid, patch index = col*4 + row."""
    cen = []
    for col in range(6):
        for row in range(4):
            cen.append((col * 100 + 50, row * 100 + 50))
    return cen


def _make(name, patches=None, cen=None):
    img = _StubImage(name)
    img.patches = _clean_patches() if patches is None else patches
    img.cen_coords = _clean_grid() if cen is None else cen
    return img


def test_clean():
    v = qa_image(_make('clean_ct5000K.pgm'))
    assert v['severity'] == PASS, v
    assert v['clipped'] == 0 and v['ramp_inversions'] == 0, v
    assert v['tilt'] is not None and v['tilt'] < 0.06, v
    print(f'test_clean: OK ({v["severity"]}, tilt {v["tilt"]}, no issues)')


def test_clipping():
    patches = _clean_patches()
    # Clip the brightest pixels of four mid patches in the red channel.
    for p in (5, 8, 12, 16):
        patches[0][p][-4:] = _WHITE
    v = qa_image(_make('glare_ct5000K.pgm', patches=patches))
    assert v['severity'] == FAIL, v
    assert v['clipped'] == 4, v
    print(f'test_clipping: OK ({v["severity"]}, {v["clipped"]} clipped)')


def test_tilt():
    # Foreshorten the right side: column height shrinks with column index, so
    # the left column is ~25% taller than the right (perspective tilt).
    cen = []
    for col in range(6):
        scale = 1.0 - 0.05 * col          # col0: 1.0 ... col5: 0.75
        for row in range(4):
            cen.append((col * 100 + 50, row * 100 * scale + 50))
    v = qa_image(_make('tilt_ct5000K.pgm', cen=cen))
    assert v['severity'] == FAIL, v
    assert v['tilt'] >= 0.12, v
    print(f'test_tilt: OK ({v["severity"]}, tilt {v["tilt"] * 100:.0f}%)')


def test_ramp_gradient():
    # Swap two neutral-ramp levels so the ramp is no longer monotonic, as an
    # illumination gradient across the chart would produce.
    patches = _clean_patches()
    for ci in range(4):
        patches[ci][11] = _flat_patch(_RAMP[15])   # N6.5 darker than N5
        patches[ci][15] = _flat_patch(_RAMP[11])
    v = qa_image(_make('gradient_ct5000K.pgm', patches=patches))
    assert v['severity'] == WARN, v
    assert v['ramp_inversions'] >= 1, v
    print(f'test_ramp_gradient: OK ({v["severity"]}, '
          f'{v["ramp_inversions"]} inversion(s))')


def test_intra_patch_spread():
    # Widen a bright neutral patch into a strong intra-patch gradient (uneven
    # light / glare across the patch), leaving the ramp order intact.
    patches = _clean_patches()
    spread = np.linspace(_RAMP[7] * 0.6, _RAMP[7] * 1.4, 64).astype(np.int64)
    for ci in range(4):
        patches[ci][7] = np.sort(spread)
    v = qa_image(_make('uneven_ct5000K.pgm', patches=patches))
    assert v['severity'] == WARN, v
    assert v['spread'] >= 0.12, v
    print(f'test_intra_patch_spread: OK ({v["severity"]}, '
          f'spread {v["spread"] * 100:.0f}%)')


def test_aggregate_worst_first():
    clean = _make('clean_ct5000K.pgm')
    bad_patches = _clean_patches()
    for p in (5, 8, 12):
        bad_patches[0][p][-4:] = _WHITE
    bad = _make('glare_ct3000K.pgm', patches=bad_patches)
    agg = qa_images([clean, bad])
    assert agg is not None and agg['worst'] == FAIL, agg
    # Worst capture ranks first.
    assert agg['images'][0]['name'] == 'glare_ct3000K.pgm', agg
    # A dark frame with no located chart contributes nothing.
    assert qa_images([_StubImage('dark.pgm')]) is None
    print(f'test_aggregate_worst_first: OK (worst {agg["worst"]}, '
          f'{len(agg["images"])} charts)')


if __name__ == '__main__':
    test_clean()
    test_clipping()
    test_tilt()
    test_ramp_gradient()
    test_intra_patch_spread()
    test_aggregate_worst_first()
    print('all Macbeth QA tests passed')
