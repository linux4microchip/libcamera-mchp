#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Board-free checks for the Gamma OECF fit (libtuning/modules/gamma). Builds a
# synthetic neutral ramp whose measured input is the reference encoding raised to
# a known gamma, and confirms the module recovers that gamma, that the
# reconstructed LUT tracks the ramp, and that a run with no located chart falls
# back to the authored config verbatim.

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from libtuning.modules.gamma import GammaMicrochipISC
from libtuning.modules.gamma.microchip_isc import (
    _GREY_PATCHES, _GREY_REF_SRGB8, compute_lut)

_SIGBITS = 10
_SHIFT = 16 - _SIGBITS
_WHITE = ((1 << _SIGBITS) - 1) << _SHIFT      # 65472
_BLACK = 64 << _SHIFT                         # 4096


class _StubImage:
    def __init__(self, name, patches):
        self.name = name
        self.sigbits = _SIGBITS
        self.blacklevel_16 = _BLACK
        self.patches = patches


def _ramp_patches(gamma_true, headroom_top=55000):
    """[4][24][n] neutrals whose measured input is ref**gamma_true.

    The module fits y = x**(1/gamma), so seeding x = y**gamma_true must recover
    gamma_true. y is the reference sRGB encoding normalised to the white neutral.
    """
    y = _GREY_REF_SRGB8 / _GREY_REF_SRGB8.max()   # 1.0 .. ~0.2
    x = y ** gamma_true                            # measured linear input
    level = _BLACK + x * (headroom_top - _BLACK)   # 16-bit, below clipping

    patches = np.full((4, 24, 64), 30000, dtype=np.int64)
    for patch, lv in zip(_GREY_PATCHES, level):
        for ci in range(4):
            patches[ci][patch] = np.full(64, int(round(lv)), dtype=np.int64)
    return patches


def _config(gamma=2.2):
    return {'gamma': {'gamma': gamma, 'toe_input': 0.0313,
                      'tone_rolloff_start': 0.85, 'tone_rolloff_slope': 0.35}}


def test_recovers_known_gamma():
    for gamma_true in (1.8, 2.2, 2.4, 2.8):
        img = _StubImage('macbeth_ct5000K.pgm', _ramp_patches(gamma_true))
        out = GammaMicrochipISC().process(_config(2.2), [img], {})
        got = out['gamma']
        assert abs(got - gamma_true) < 0.05, (gamma_true, got)
        # RMS is dominated by the coarse 64-entry LUT quantisation, not fit
        # error; it stays inside the health PASS band (<= 0.06).
        assert out['_oecf_rms'] < 0.05, (gamma_true, out['_oecf_rms'])
        assert out['_oecf_points'] >= 4, out
    print('test_recovers_known_gamma: OK (1.8/2.2/2.4/2.8 recovered within 0.05)')


def test_lut_monotonic():
    lut = compute_lut(2.2, 0.0313, 0.85, 0.35, bipartite=True)
    assert lut[0] == 0, lut[0]
    assert lut[-1] <= 1023, lut[-1]
    assert np.all(np.diff(lut) >= 0), 'LUT is not non-decreasing'
    # output = input**(1/gamma): a higher gamma lifts midtones, so entry 32
    # (~half input) is brighter at gamma 3.0 than at gamma 1.5.
    high = compute_lut(3.0, 0.0313, 0.85, 0.35, bipartite=True)
    low = compute_lut(1.5, 0.0313, 0.85, 0.35, bipartite=True)
    assert high[32] > low[32], (high[32], low[32])
    print(f'test_lut_monotonic: OK (mono, entry32 gamma3.0={high[32]} '
          f'> gamma1.5={low[32]})')


def test_held_scalars_passthrough():
    # toe/rolloff are held at the authored config values, not fitted.
    img = _StubImage('macbeth_ct5000K.pgm', _ramp_patches(2.4))
    cfg = _config(2.2)
    cfg['gamma']['toe_input'] = 0.0469
    cfg['gamma']['tone_rolloff_start'] = 0.9
    out = GammaMicrochipISC().process(cfg, [img], {})
    assert out['toe_input'] == 0.0469, out
    assert out['tone_rolloff_start'] == 0.9, out
    print('test_held_scalars_passthrough: OK (toe/rolloff untouched)')


def test_fallback_no_chart():
    cfg = _config(2.2)
    out = GammaMicrochipISC().process(cfg, [], {})
    # Falls back to the exact config object so the tuner marks it unmeasured.
    assert out is cfg['gamma'], out
    print('test_fallback_no_chart: OK (returns authored config object)')


if __name__ == '__main__':
    test_recovers_known_gamma()
    test_lut_monotonic()
    test_held_scalars_passthrough()
    test_fallback_no_chart()
    print('all Gamma OECF tests passed')
