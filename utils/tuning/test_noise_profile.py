#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Board-free test for the still-path noise-profile calibration.
#
# mchp_capture.sh sweeps analogue gain and shoots --frames aligned repeats of a
# static flat at each gain. calibrate_noise_profile measures, per gain, the
# temporal per-pixel variance across those repeats, fits variance-vs-signal for
# the read (intercept) and shot (slope) terms, and regresses the read term
# against gain to recover the gain exponent. This test asserts:
#
#   recovery      read_noise_var, shot_slope and gain_exponent are recovered from
#                 synthetic flats generated with known injected read+shot noise
#                 that amplifies as gain**(2*exp).
#   byte-identity the per-gain fits / fit-quality diagnostics never reach the
#                 emitted YAML; the tuning file's 'modules:' NoiseReduction block
#                 carries only the three runtime constants and the IPA
#                 'algorithms:' list carries no noise section at all.
#   single-gain   one gain emits the read/shot terms but leaves the exponent at
#                 the runtime default (not measurable from a single point).
#   health        the noise advisory passes on a clean fit and warns on a loose
#                 one, without perturbing the 0-100 score.
#
# Flats are real PGM_SRGGB10 mosaics through the real loader (no stubs), matching
# test_pgm_loader / test_lsc_per_ct. Plain asserts, run directly.

import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from libtuning.image import Image
from libtuning.modules.noise import NoiseMicrochipISC
from libtuning.modules.noise.microchip_isc import calibrate_noise_profile
from microchip_isc_lt import MicrochipYamlOutput
from microchip_isc_health import health_summary

from test_pgm_loader import write_pgm, write_sidecar

_W, _H = 160, 120
_BL10 = 64                     # RAW10 pedestal (matches IMX219 default)

# Injected noise, in the RAW10 measurement domain, at unity gain:
#   var10(signal) = READ10 + SLOPE10 * light10
# The loader shifts RAW10 into 16 bits and the module works in the 8-bit domain
# (>>8), so the recovered constants are the RAW10 ones scaled by the 10->8-bit
# value ratio (1/4): read_var_8 = READ10/16, shot_slope_8 = SLOPE10/4.
_READ10 = 100.0
_SLOPE10 = 3.2
_EXP = 0.7
_EXP_READ_VAR = _READ10 / 16.0     # 6.25
_EXP_SHOT_SLOPE = _SLOPE10 / 4.0   # 0.80


def _light_map():
    """Column ramp of RAW10 light (black-subtracted), ~40..480 DN.

    Capped so that at the top gain the brightest pixels stay several noise sigma
    below the 1023 ceiling: clipping truncates a pixel's temporal variance, which
    would bias the fitted slope/intercept exactly where the injected noise is
    largest.
    """
    x = np.linspace(40.0, 480.0, _W)
    return np.repeat(x[None, :], _H, axis=0)


def _make_frames(tmp, gain, n, rng):
    """n aligned flat repeats at one gain, each with independent injected noise.

    Per pixel the temporal variance across the repeats equals the injected
    var10 = gain**(2*exp) * (READ10 + SLOPE10 * light10), so the module recovers
    the read/shot terms from the spread of the repeats, not from any single
    frame's spatial structure.
    """
    light = _light_map()
    var10 = (gain ** (2.0 * _EXP)) * (_READ10 + _SLOPE10 * light)
    sigma10 = np.sqrt(var10)
    imgs = []
    for f in range(n):
        noise = rng.standard_normal((_H, _W)) * sigma10
        mosaic = np.clip(np.rint(_BL10 + light + noise), 0, 1023).astype(np.uint16)
        p = tmp / f'alsc_ct5000K_gain{gain:g}_g0_f{f}.pgm'
        write_pgm(p, mosaic)
        write_sidecar(p, format='PGM_SRGGB10', width=_W, height=_H,
                      black_level=_BL10, analogue_gain=float(gain))
        img = Image(p)
        img.lsc_only = True
        img.color = 5000
        imgs.append(img)
    return imgs


def _sweep(tmp, gains, n, seed=1):
    rng = np.random.default_rng(seed)
    images = []
    for g in gains:
        images += _make_frames(tmp, g, n, rng)
    return images


def test_recovery(tmp):
    # gains 1..3 (not ..4): the synthetic ramp deliberately spans a wide signal
    # range to exercise the fit, which at the top gain would drive the brightest
    # pixels into clipping (a real flat is exposed to mid-grey, so this is a
    # synthetic artefact, not a module limit). n=12 repeats keep the variance-of-
    # variance low enough that the intercept extrapolation is stable.
    images = _sweep(tmp, (1.0, 2.0, 3.0), n=12)
    profile, per_gain = calibrate_noise_profile(images)

    assert profile is not None, 'no profile from a valid gain sweep'
    assert len(per_gain) == 3, per_gain
    for e in per_gain:
        assert e['r2'] > 0.9, e          # variance-vs-signal is a clean line

    rv = profile['read_noise_var']
    ss = profile['shot_slope']
    ge = profile['gain_exponent']
    assert abs(rv - _EXP_READ_VAR) / _EXP_READ_VAR < 0.30, (rv, _EXP_READ_VAR)
    assert abs(ss - _EXP_SHOT_SLOPE) / _EXP_SHOT_SLOPE < 0.15, (ss, _EXP_SHOT_SLOPE)
    assert abs(ge - _EXP) < 0.15, (ge, _EXP)
    print(f'test_recovery: OK (read_var={rv:.2f}~{_EXP_READ_VAR}, '
          f'shot={ss:.3f}~{_EXP_SHOT_SLOPE}, gainExp={ge:.2f}~{_EXP})')


def test_single_gain_no_exponent(tmp):
    # One gain: the read/shot terms are still measurable, but the gain exponent
    # is not (needs >=2 gains), so it holds the runtime default.
    images = _sweep(tmp, (1.0,), n=12, seed=2)
    profile, per_gain = calibrate_noise_profile(images)
    assert profile is not None and len(per_gain) == 1, per_gain
    assert profile['gain_exponent'] == 0.8, profile['gain_exponent']
    assert profile['gain_exp_r2'] is None, profile
    print('test_single_gain_no_exponent: OK (1 gain => default exponent kept)')


def test_no_repeats_returns_none(tmp):
    # A single frame per gain has no temporal spread to measure -> no profile.
    images = _sweep(tmp, (1.0, 2.0), n=1, seed=3)
    profile, per_gain = calibrate_noise_profile(images)
    assert profile is None and per_gain == [], (profile, per_gain)
    print('test_no_repeats_returns_none: OK (n=1 per gain => nothing emitted)')


def test_yaml_stripped_and_diverted(tmp):
    images = _sweep(tmp, (1.0, 2.0, 3.0), n=12, seed=4)
    out = NoiseMicrochipISC(debug=[]).process({'noise': {}}, images, {})
    assert out is not None and '_per_gain' in out, 'diagnostic missing'

    class _Stub:
        out_name = 'NoiseProfile'
    m = _Stub()
    main = tmp / 'imx219.yaml'
    MicrochipYamlOutput().write(str(main), {m: out}, [m])

    data = main.read_bytes()
    # The noise profile lands in the main file's 'modules:' block keyed
    # NoiseReduction (the key the pipeline handler reads), never in the IPA
    # 'algorithms:' list and never in a sibling file.
    assert b'algorithms:' in data, 'IPA algorithms list missing'
    algos = data.split(b'modules:')[0]
    assert b'NoiseReduction' not in algos and b'NoiseProfile' not in algos, \
        'noise section leaked into the IPA algorithms list'
    assert b'modules:' in data, 'modules block missing from the tuning file'
    assert b'NoiseReduction:' in data, 'NoiseReduction module block missing'
    assert b'read_noise_var' in data and b'gain_exponent' in data, \
        'module block missing the runtime constants'
    assert b'_per_gain' not in data and b'gain_exp_r2' not in data, \
        'diagnostic leaked into the emitted YAML'
    assert not main.with_name('imx219_noise.yaml').exists(), \
        'sibling _noise.yaml must no longer be written'
    print(f'test_yaml_stripped_and_diverted: OK ({len(data)} bytes, noise in '
          'modules block, diagnostics stripped, algorithms list clean)')


def test_health_advisory():
    def section(worst_r2, exp_r2, n_g=3):
        pg = [{'gain': 2.0 ** i, 'read_noise_var': 6.0, 'shot_slope': 0.8,
               'r2': (worst_r2 if i == 0 else 0.99), 'n_px': 10000}
              for i in range(n_g)]
        return {'read_noise_var': 6.25, 'shot_slope': 0.8, 'gain_exponent': 0.7,
                '_per_gain': pg, '_gain_exp_r2': exp_r2, '_ref_gain': 1.0}

    _, report, _ = health_summary({'NoiseProfile': section(0.98, 0.95)},
                                  measured=set())
    adv = [ln for ln in report if 'Noise profile' in ln]
    assert adv and '[ OK ]' in adv[0], adv

    # A loose per-gain line trips the advisory.
    _, report, _ = health_summary({'NoiseProfile': section(0.70, 0.95)},
                                  measured=set())
    adv = [ln for ln in report if 'Noise profile' in ln]
    assert adv and '[WARN]' in adv[0], adv

    # So does a poorly determined gain exponent over a real sweep.
    _, report, _ = health_summary({'NoiseProfile': section(0.98, 0.50)},
                                  measured=set())
    adv = [ln for ln in report if 'Noise profile' in ln]
    assert adv and '[WARN]' in adv[0], adv

    # Advisory carries no points: the 0-100 score is unchanged by its presence.
    s_with, _, _ = health_summary({'NoiseProfile': section(0.98, 0.95)},
                                  measured=set())
    s_without, _, _ = health_summary({}, measured=set())
    assert s_with == s_without, (s_with, s_without)
    print('test_health_advisory: OK (clean=PASS, loose fit/exponent=WARN, '
          'no score weight)')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as d:
        tmp = Path(d)
        test_recovery(tmp)
        test_single_gain_no_exponent(tmp)
        test_no_repeats_returns_none(tmp)
        test_yaml_stripped_and_diverted(tmp)
        test_health_advisory()
    print('all noise-profile tests passed')
