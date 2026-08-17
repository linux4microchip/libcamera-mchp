#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Board-free repeatability harness for the Microchip ISC calibration chain.
#
# Runs the real module chain (BlackLevelCorrection, Awb, Agc, Cbc, Gamma, Ccm,
# LensShadingCorrection) and the real YAML generator on a fixed synthetic
# calibration set, and asserts two properties a calibration tool must hold:
#
#   determinism   the same inputs emit byte-identical YAML every run (no
#                 dictionary-ordering, set-iteration or unseeded-random leak
#                 into the tuning file).
#   stability     a small per-pixel jitter of the captures perturbs the solved
#                 AWB gains and CCM matrices only within a tight tolerance (the
#                 solves are not numerically brittle).
#
# The synthetic charts are built directly (bypassing the fragile chart locator)
# from the ColorChecker reference colours passed through a fixed sensor mixing
# matrix and a per-illuminant white balance, so the AWB/CCM/lux solves have real
# structure to recover. A negative control confirms the byte comparator does
# flag a deliberately perturbed run, so a genuine non-determinism would not pass
# silently.
#
# No pytest/CI in this repo; plain asserts, run directly. (A minimal pytest.ini
# could later collect test_*.py uniformly -- deferred as a project decision.)

import logging
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import libtuning as lt
from libtuning.image import Image
from libtuning.modules.awb import AWBMicrochipISC
from libtuning.modules.agc import AGCMicrochipISC
from libtuning.modules.ccm import CCMMicrochipISC
from libtuning.modules.gamma import GammaMicrochipISC
from libtuning.modules.lsc import LSCMicrochipISC
from libtuning.modules.static import ConfigPassthroughModule
from libtuning.modules.awb.microchip_isc import _srgb8_to_linear

from microchip_isc_lt import MacbethQAModule, MicrochipYamlOutput
from test_pgm_loader import write_pgm, write_sidecar

# The calibration math logs verbosely at INFO; quiet it for the harness.
logging.getLogger().setLevel(logging.ERROR)

_SIGBITS = 10
_SHIFT = 16 - _SIGBITS
_WHITE = ((1 << _SIGBITS) - 1) << _SHIFT
_BLACK = 64 << _SHIFT
_PATCH_PIXELS = 64

# ColorChecker reference (chart order, 8-bit sRGB); reordered [i::6] to the
# located-patch order the solves and ctt_ccm.ccm() both consume.
_M_RGB_CHART = np.array([
    [116, 81, 67], [199, 147, 129], [91, 122, 156], [90, 108, 64],
    [130, 128, 176], [92, 190, 172], [224, 124, 47], [68, 91, 170],
    [198, 82, 97], [94, 58, 106], [159, 189, 63], [230, 162, 39],
    [35, 63, 147], [67, 149, 74], [180, 49, 57], [238, 198, 20],
    [193, 84, 151], [0, 136, 170], [245, 245, 243], [200, 202, 202],
    [161, 163, 163], [121, 121, 122], [82, 84, 86], [49, 49, 51],
], dtype=np.float64)
_M_RGB_LOCATED = np.array([_M_RGB_CHART[i::6] for i in range(6)]).reshape((24, 3))

# A fixed, well-conditioned sensor spectral-mixing matrix (near identity with
# realistic channel crosstalk). CCM should recover roughly its inverse.
_SENSOR_MIX = np.array([
    [0.90, 0.08, 0.02],
    [0.05, 0.88, 0.07],
    [0.02, 0.10, 0.88],
], dtype=np.float64)


class _StubMacbeth:
    """A located-chart stand-in for the module chain (no real image decode)."""

    def __init__(self, name, ct, lux, exposure_us, gain):
        self.name = name
        self.color = ct
        self.lux = lux
        self.lsc_only = False            # a chart, not a flat field
        self.sigbits = _SIGBITS
        self.blacklevel_16 = _BLACK
        self.macbeth = object()          # non-None => solvers accept it
        self.metadata = {'exposure_us': exposure_us, 'analogue_gain': gain}
        self.exposure = exposure_us
        self.againQ8_norm = gain
        self.patches: "np.ndarray | None" = None
        self.cen_coords: "list | None" = None


def _illuminant_gains(ct):
    """Per-channel white-balance imbalance for a CT: warm boosts R, cool B."""
    warm = (6500.0 - ct) / 6500.0          # +ve warm, -ve cool
    return np.array([1.0 + 0.35 * warm, 1.0, 1.0 - 0.30 * warm])


def _chart_patches(ct, rng=None, jitter=0.0):
    """[4][24][N] sensor patches from the reference chart under illuminant ct."""
    lin = _srgb8_to_linear(_M_RGB_LOCATED)          # 24x3, 0..1
    sensor = lin @ _SENSOR_MIX.T                     # apply sensor response
    sensor = sensor * _illuminant_gains(ct)          # apply illuminant WB
    sensor = np.clip(sensor, 0.0, None)

    headroom = _WHITE - _BLACK - 512
    peak = float(sensor.max()) or 1.0
    val16 = _BLACK + sensor / peak * headroom        # 24x3, in [BLACK, ~WHITE)

    patches = np.zeros((4, 24, _PATCH_PIXELS), dtype=np.int64)
    for p in range(24):
        r, g, b = val16[p]
        for ci, level in ((0, r), (1, g), (2, g), (3, b)):
            px = np.full(_PATCH_PIXELS, level, dtype=np.float64)
            if jitter > 0.0 and rng is not None:
                px = px + rng.normal(0.0, jitter * max(level - _BLACK, 1.0),
                                     _PATCH_PIXELS)
            px = np.clip(np.rint(px), 0, _WHITE - 1).astype(np.int64)
            px.sort()
            patches[ci][p] = px
    return patches


def _clean_grid():
    cen = []
    for col in range(6):
        for row in range(4):
            cen.append((col * 100 + 50, row * 100 + 50))
    return cen


def _build_images(tmp, rng=None, jitter=0.0):
    """Synthetic calibration set: three CT charts + one radial LSC flat."""
    specs = [
        ('macbeth_ct2700K_lux800.pgm', 2700, 800, 20000, 1.0),
        ('macbeth_ct4500K_lux1000.pgm', 4500, 1000, 12000, 1.0),
        ('macbeth_ct6500K_lux1200.pgm', 6500, 1200, 8000, 1.0),
    ]
    images = []
    for name, ct, lux, exp_us, gain in specs:
        img = _StubMacbeth(name, ct, lux, exp_us, gain)
        img.patches = _chart_patches(ct, rng, jitter)
        img.cen_coords = _clean_grid()
        images.append(img)

    # A real radial flat field through the real PGM loader for the LSC module.
    w, h = 128, 96
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    rn = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
    rn = np.clip(rn / rn.max(), 0.0, 1.0)
    mosaic = np.clip(np.rint(900.0 * (1.0 - 0.5 * rn ** 2)), 0, 1023).astype(np.uint16)
    p = tmp / 'alsc_5000K_lux1000_raw.pgm'
    write_pgm(p, mosaic)
    write_sidecar(p, format='PGM_SRGGB10', width=w, height=h)
    flat = Image(p)
    flat.lsc_only = True
    flat.color = 5000
    images.append(flat)
    return images


def _config():
    return {
        'general': {},
        'blc': {'black_level': 64, 'black_level_r': 64, 'black_level_gr': 64,
                'black_level_gb': 64, 'black_level_b': 64},
        'cbc': {'brightness': 0, 'contrast': 256, 'saturation': 256},
        'gamma': {'gamma': 2.2, 'toe_input': 0.0313, 'tone_rolloff_start': 0.85},
        'awb': {},
        'agc': {},
        'ccm': {'smoothing_factor': 0.7, 'ct_fallback': 5500,
                'matrices': {}},
        'lsc': {'max_gain': 4.0},
    }


def _make_chain():
    blc = ConfigPassthroughModule('BlackLevelCorrection', 'blc')
    cbc = ConfigPassthroughModule('Cbc', 'cbc')
    gamma = GammaMicrochipISC()
    awb = AWBMicrochipISC(debug=[])
    agc = AGCMicrochipISC(debug=[])
    ccm = CCMMicrochipISC(debug=[])
    lsc = LSCMicrochipISC(debug=[],
                          sector_shape=(16, 12),
                          sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                          sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                          sector_average_function=lt.average.Mean(),
                          smoothing_function=lt.smoothing.MedianBlur(3))
    qa = MacbethQAModule()
    return [blc, awb, agc, cbc, gamma, ccm, lsc, qa]


def _run_chain(tmp, out_path, rng=None, jitter=0.0):
    """Run the full chain + generator once; return (yaml_bytes, outputs)."""
    images = _build_images(tmp, rng, jitter)
    config = _config()
    chain = _make_chain()
    outputs = {}
    for module in chain:
        out = module.process(config, images, outputs)
        if out is not None:
            outputs[module] = out
    MicrochipYamlOutput().write(str(out_path), outputs, chain)
    named = {m.out_name: outputs[m] for m in chain if m in outputs}
    return out_path.read_bytes(), named


def test_determinism(tmp):
    # ctt_ccm writes visualisation PNGs to cwd; contain them in tmp.
    cwd = os.getcwd()
    os.chdir(tmp)
    try:
        runs = [_run_chain(tmp, tmp / f'imx219_run{i}.yaml')[0] for i in range(3)]
    finally:
        os.chdir(cwd)
    assert runs[0] == runs[1] == runs[2], 'chain is non-deterministic across runs'
    assert b'Awb' in runs[0] and b'Ccm' in runs[0], 'expected sections missing'
    # MacbethQA is health-only and must never reach the tuning file.
    assert b'MacbethQA' not in runs[0], 'MacbethQA leaked into the IPA YAML'
    print(f'test_determinism: OK (3 identical runs, {len(runs[0])} bytes)')


def test_stability(tmp):
    cwd = os.getcwd()
    os.chdir(tmp)
    try:
        _, clean = _run_chain(tmp, tmp / 'imx219_clean.yaml')
        rng = np.random.default_rng(20260814)
        _, noisy = _run_chain(tmp, tmp / 'imx219_noisy.yaml', rng=rng, jitter=0.01)
    finally:
        os.chdir(cwd)

    # AWB reference gains: 1% pixel noise must move each gain < 2%.
    c = {e['ct']: e for e in clean['Awb']['reference_gains_table']}
    n = {e['ct']: e for e in noisy['Awb']['reference_gains_table']}
    assert c.keys() == n.keys(), (c.keys(), n.keys())
    for ct in c:
        for k in ('r', 'b'):
            rel = abs(n[ct][k] - c[ct][k]) / max(c[ct][k], 1e-6)
            assert rel < 0.02, f'AWB {k}Gain at {ct}K moved {rel*100:.1f}%'

    # CCM matrix entries: 1% pixel noise must move each element < 0.05 absolute.
    cm = clean['Ccm']['matrices']
    nm = noisy['Ccm']['matrices']
    assert cm.keys() == nm.keys(), (cm.keys(), nm.keys())
    for name in cm:
        for a, b in zip(cm[name]['ccm'], nm[name]['ccm']):
            assert abs(a - b) < 0.05, f'CCM {name} element moved {abs(a-b):.3f}'

    # Gamma exponent: 1% pixel noise must move the fitted gamma < 0.1.
    gc = float(clean['Gamma']['gamma'])
    gn = float(noisy['Gamma']['gamma'])
    assert abs(gc - gn) < 0.1, f'fitted gamma moved {abs(gc - gn):.3f}'

    print(f'test_stability: OK ({len(c)} AWB CT points, {len(cm)} CCM buckets, '
          f'gamma {gc:.2f} within tolerance under 1% jitter)')


def test_negative_control(tmp):
    # Prove the comparator flags a genuinely different run: a real
    # non-determinism would fail test_determinism, not slip through.
    cwd = os.getcwd()
    os.chdir(tmp)
    try:
        base, _ = _run_chain(tmp, tmp / 'imx219_base.yaml')
        rng = np.random.default_rng(1)
        perturbed, _ = _run_chain(tmp, tmp / 'imx219_perturbed.yaml',
                                  rng=rng, jitter=0.2)
    finally:
        os.chdir(cwd)
    assert base != perturbed, 'comparator failed to detect a changed run'
    print('test_negative_control: OK (comparator detects a changed run)')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as d:
        tmp = Path(d)
        test_determinism(tmp)
        test_stability(tmp)
        test_negative_control(tmp)
    print('all repeatability tests passed')
