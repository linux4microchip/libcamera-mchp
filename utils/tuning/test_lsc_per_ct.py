#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Board-free test for the per-CT lens-shading maps and the green-imbalance
# (GEQ) measurement.
#
# mchp_capture.sh now tags flat fields with the illuminant CT, so the LSC module
# can emit one gain-map set per colour temperature (lens colour-shading is
# CT-dependent). It also measures per-sector GR/GB imbalance from the raw Bayer
# greens. This test asserts:
#
#   single-CT     one flat emits the legacy flat 'gains' shape -- byte-identical
#                 to the pre-per-CT output the runtime already parses.
#   multi-CT      several CTs emit a sorted 'per_ct' list, one map set each, and
#                 no top-level 'gains'.
#   geq           an injected GR/GB offset is recovered in the _geq diagnostic;
#                 a balanced flat measures ~0. _geq never reaches the YAML.
#   health        the LSC/GEQ advisory warns when green imbalance exceeds 3%.
#
# Flats are built as real PGM_SRGGB10 mosaics through the real loader (no stubs),
# matching test_pgm_loader.py. Plain asserts, run directly.

import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import libtuning as lt
from libtuning.image import Image
from libtuning.modules.lsc import LSCMicrochipISC
from microchip_isc_lt import MicrochipYamlOutput
from microchip_isc_health import health_summary

from test_pgm_loader import write_pgm, write_sidecar

_W, _H = 128, 96
_NX, _NY = 16, 12


def _falloff_sub():
    """Radial falloff at the Bayer-subsampled resolution (one value per 2x2)."""
    sh, sw = _H // 2, _W // 2
    yy, xx = np.mgrid[0:sh, 0:sw].astype(np.float64)
    cx, cy = (sw - 1) / 2.0, (sh - 1) / 2.0
    rn = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
    rn = np.clip(rn / rn.max(), 0.0, 1.0)
    return 900.0 * (1.0 - 0.5 * rn ** 2)          # centre ~900, corners ~450


def _make_flat(tmp, ct, geq_delta=0.0):
    """A radial flat at CT with an optional GR/GB imbalance (GR up, GB down).

    All four Bayer positions of a 2x2 block share one falloff sample, so GR and
    GB see identical shading -- a balanced flat measures zero imbalance and
    geq_delta is the only GR/GB difference.
    """
    sub = _falloff_sub()
    mosaic = np.zeros((_H, _W), dtype=np.float64)
    # SRGGB10 phase: R=(0::2,0::2) GR=(0::2,1::2) GB=(1::2,0::2) B=(1::2,1::2).
    mosaic[0::2, 0::2] = sub                       # R
    mosaic[0::2, 1::2] = sub * (1.0 + geq_delta)   # GR
    mosaic[1::2, 0::2] = sub * (1.0 - geq_delta)   # GB
    mosaic[1::2, 1::2] = sub                       # B
    mosaic = np.clip(np.rint(mosaic), 0, 1023).astype(np.uint16)

    p = tmp / f'alsc_ct{ct}K_lux1000_raw.pgm'
    write_pgm(p, mosaic)
    write_sidecar(p, format='PGM_SRGGB10', width=_W, height=_H)
    img = Image(p)
    img.lsc_only = True
    img.color = ct
    return img


def _module():
    return LSCMicrochipISC(
        debug=[], sector_shape=(_NX, _NY),
        sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
        sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
        sector_average_function=lt.average.Mean(),
        smoothing_function=lt.smoothing.MedianBlur(3))


def test_single_ct_legacy_shape(tmp):
    out = _module().process({'lsc': {'max_gain': 4.0}}, [_make_flat(tmp, 5000)], {})
    assert 'gains' in out and 'per_ct' not in out, out.keys()
    for ch in ('r', 'g', 'b'):
        assert len(out['gains'][ch]) == _NX * _NY
    print('test_single_ct_legacy_shape: OK (one CT => flat gains, no per_ct)')


def test_multi_ct_per_ct_shape(tmp):
    flats = [_make_flat(tmp, ct) for ct in (3000, 4500, 6500)]
    out = _module().process({'lsc': {'max_gain': 4.0}}, flats, {})
    assert 'per_ct' in out and 'gains' not in out, out.keys()
    cts = [e['ct'] for e in out['per_ct']]
    assert cts == sorted(cts) == [3000, 4500, 6500], cts
    for e in out['per_ct']:
        for ch in ('r', 'g', 'b'):
            assert len(e['gains'][ch]) == _NX * _NY, (e['ct'], ch)
    print(f'test_multi_ct_per_ct_shape: OK ({len(cts)} CT maps, sorted)')


def test_geq_recovered(tmp):
    balanced = _module().process({'lsc': {}}, [_make_flat(tmp, 5000, 0.0)], {})
    skewed = _module().process({'lsc': {}}, [_make_flat(tmp, 5000, 0.05)], {})
    b_dev = balanced['_geq'][0]['max_dev']
    s_dev = skewed['_geq'][0]['max_dev']
    # A 5% per-position GR/GB skew must show up clearly; a balanced flat is flat.
    assert b_dev < 0.02, f'balanced flat shows spurious imbalance {b_dev}'
    assert s_dev > 0.05, f'injected 5% skew not recovered ({s_dev})'
    assert s_dev > b_dev
    print(f'test_geq_recovered: OK (balanced {b_dev*100:.1f}% << skewed '
          f'{s_dev*100:.1f}%)')


def test_geq_stripped_from_yaml(tmp):
    out = _module().process({'lsc': {}}, [_make_flat(tmp, 5000, 0.05)], {})
    assert '_geq' in out, 'diagnostic missing from module output'

    class _Stub:
        out_name = 'LensShadingCorrection'
    m = _Stub()
    main = tmp / 'imx219.yaml'
    MicrochipYamlOutput().write(str(main), {m: out}, [m])
    data = main.read_bytes()
    # LSC lands in the main file's 'modules:' block, not the IPA 'algorithms:'
    # list and not a sibling file.
    assert b'modules:' in data, 'modules block missing from the tuning file'
    assert b'LensShadingCorrection:' in data, 'LSC module block missing'
    assert b'gains' in data, 'gain map missing from the module block'
    assert b'_geq' not in data and b'max_dev' not in data, \
        'GEQ diagnostic leaked into the emitted YAML'
    assert b'per_ct' not in data, 'single-CT block must use the flat shape'
    assert not main.with_name('imx219_lsc.yaml').exists(), \
        'sibling _lsc.yaml must no longer be written'
    print(f'test_geq_stripped_from_yaml: OK ({len(data)} bytes, _geq stripped)')


def test_health_advisory():
    def section(dev):
        return {'grid_width': _NX, 'grid_height': _NY,
                'gains': {c: [1.0] * (_NX * _NY) for c in 'rgb'},
                '_geq': [{'ct': 5000, 'n': 1, 'max_dev': dev, 'mean_dev': dev}]}

    _, report, _ = health_summary({'LensShadingCorrection': section(0.01)},
                                  measured=set())
    adv = [ln for ln in report if 'Lens shading / GEQ' in ln]
    assert adv and '[ OK ]' in adv[0], adv

    _, report, _ = health_summary({'LensShadingCorrection': section(0.08)},
                                   measured=set())
    adv = [ln for ln in report if 'Lens shading / GEQ' in ln]
    assert adv and '[WARN]' in adv[0], adv
    print('test_health_advisory: OK (1% GEQ=PASS, 8% GEQ=WARN)')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as d:
        tmp = Path(d)
        test_single_ct_legacy_shape(tmp)
        test_multi_ct_per_ct_shape(tmp)
        test_geq_recovered(tmp)
        test_geq_stripped_from_yaml(tmp)
        test_health_advisory()
    print('all LSC per-CT / GEQ tests passed')
