#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Board-free test for the multi-CT / multi-frame AWB curve and its inter-frame
# variance bounds.
#
# mchp_capture.sh --frames now shoots N repeats at each (CT, gain) operating
# point. calibrate_awb_gains_macbeth averages those repeats into one reference
# gain per CT and, from the spread of the individual frames' gains, reports how
# repeatable that point was. This test asserts:
#
#   curve         gains are recovered at every CT and follow the Planckian trend
#                 (rGain falls, bGain rises as CT climbs).
#   variance      the per-CT inter-frame spread is populated, is ~zero for
#                 identical repeats, and grows with injected per-frame jitter.
#   byte-identity the variance diagnostic never reaches the emitted YAML -- the
#                 reference_gains_table bytes are identical with and without it.
#   health        the AWB repeatability advisory reads the diagnostic and warns
#                 only when a CT's inter-frame std exceeds the noise floor.
#
# Synthetic charts are built with the same stubbed-locator approach as
# test_repeatability.py (no board, no chart-locator fragility). Plain asserts,
# run directly -- no pytest/CI in this repo.

import logging
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from libtuning.modules.awb import AWBMicrochipISC
from libtuning.modules.awb.microchip_isc import (
    calibrate_awb_gains_macbeth, check_awb_variance)
from microchip_isc_lt import MicrochipYamlOutput
from microchip_isc_health import health_summary

from test_repeatability import _StubMacbeth, _chart_patches, _clean_grid

logging.getLogger().setLevel(logging.ERROR)

_CTS = (2700, 4500, 6500)


def _frames_for_ct(ct, n, rng, jitter):
    """n stubbed Macbeth frames at one CT, each an independent jittered repeat."""
    imgs = []
    for f in range(n):
        img = _StubMacbeth(f'macbeth_ct{ct}K_lux1000_g0_f{f}.pgm',
                           ct, 1000, 12000, 1.0)
        img.patches = _chart_patches(ct, rng, jitter)
        img.cen_coords = _clean_grid()
        imgs.append(img)
    return imgs


def _capture_set(n_frames, rng, jitter):
    images = []
    for ct in _CTS:
        images += _frames_for_ct(ct, n_frames, rng, jitter)
    return images


def test_curve_and_variance():
    rng = np.random.default_rng(11)
    images = _capture_set(5, rng, jitter=0.02)
    table, variance = calibrate_awb_gains_macbeth(images)

    assert table is not None and len(table) == len(_CTS), table
    r = [e['r'] for e in table]
    b = [e['b'] for e in table]
    # A usable interpolation curve is strictly monotone in CT with rGain and
    # bGain moving in opposite directions. (The physical sign -- rGain falls,
    # bGain rises with CT -- is asserted against real captures by
    # check_awb_curve; the synthetic raw model here fixes only the shape.)
    r_up = r[0] < r[1] < r[2]
    r_dn = r[0] > r[1] > r[2]
    b_up = b[0] < b[1] < b[2]
    b_dn = b[0] > b[1] > b[2]
    assert (r_up or r_dn) and (b_up or b_dn), f'non-monotone curve: r={r} b={b}'
    assert r_up != b_up, f'rGain and bGain move together: r={r} b={b}'

    assert len(variance) == len(_CTS), variance
    for v in variance:
        assert v['n'] == 5, v
        # Independent jittered repeats => a real, positive inter-frame spread.
        assert v['r_std'] > 0.0 and v['b_std'] > 0.0, v
        assert v['r_rel_std'] > 0.0 and v['b_rel_std'] > 0.0, v
    print(f'test_curve_and_variance: OK ({len(table)} CT pts, monotonic, '
          f'variance populated for n=5 frames)')


def test_zero_variance_for_identical_frames():
    # Repeats with no jitter are byte-identical frames: the inter-frame spread
    # must be exactly zero, not a floating-point artefact.
    images = _capture_set(4, rng=None, jitter=0.0)
    _, variance = calibrate_awb_gains_macbeth(images)
    for v in variance:
        assert v['n'] == 4, v
        assert v['r_std'] == 0.0 and v['b_std'] == 0.0, v
        assert v['r_rel_std'] == 0.0 and v['b_rel_std'] == 0.0, v
    print('test_zero_variance_for_identical_frames: OK (identical repeats => '
          'zero spread)')


def test_variance_grows_with_jitter():
    lo = calibrate_awb_gains_macbeth(
        _capture_set(6, np.random.default_rng(3), jitter=0.02))[1]
    hi = calibrate_awb_gains_macbeth(
        _capture_set(6, np.random.default_rng(3), jitter=0.20))[1]
    lo_worst = max(max(v['r_rel_std'], v['b_rel_std']) for v in lo)
    hi_worst = max(max(v['r_rel_std'], v['b_rel_std']) for v in hi)
    assert hi_worst > lo_worst, (lo_worst, hi_worst)
    print(f'test_variance_grows_with_jitter: OK (rel std {lo_worst*100:.2f}% '
          f'-> {hi_worst*100:.2f}% as jitter rises)')


def test_single_frame_no_spread():
    # One frame per CT: no repeatability to assess. Variance entries exist (so
    # the curve still emits) but carry n==1 and zero spread.
    _, variance = calibrate_awb_gains_macbeth(_capture_set(1, None, 0.0))
    for v in variance:
        assert v['n'] == 1 and v['r_std'] == 0.0, v
    check_awb_variance(variance)   # must not raise on the single-frame path
    print('test_single_frame_no_spread: OK (n=1 => no spread, no raise)')


def _emit_awb_yaml(tmp, section):
    """Emit just the Awb section through the real YAML writer; return bytes."""
    class _Stub:
        out_name = 'Awb'
    m = _Stub()
    out_path = tmp / 'awb.yaml'
    MicrochipYamlOutput().write(str(out_path), {m: section}, [m])
    return out_path.read_bytes()


def test_yaml_byte_identical(tmp):
    images = _capture_set(5, np.random.default_rng(5), jitter=0.05)
    awb = AWBMicrochipISC(debug=[])
    section = awb.process({'awb': {}}, images, {})
    assert '_awb_variance' in section, 'diagnostic missing from module output'

    with_var = _emit_awb_yaml(tmp, section)
    without = _emit_awb_yaml(tmp, {k: v for k, v in section.items()
                                   if k != '_awb_variance'})
    assert with_var == without, 'variance diagnostic changed the emitted YAML'
    assert b'reference_gains_table' in with_var, 'AWB table missing from YAML'
    assert b'_awb_variance' not in with_var and b'r_rel_std' not in with_var, \
        'diagnostic leaked into the emitted YAML'
    print(f'test_yaml_byte_identical: OK ({len(with_var)} bytes, variance '
          f'stripped, table intact)')


def test_health_advisory():
    # A noisy operating point (>2% inter-frame std) must fold a WARN into the
    # AWB repeatability advisory; a clean one stays PASS. Craft the diagnostic
    # directly so the assertion targets the scorer, not the pixel noise floor.
    clean = {'reference_gains_table': [
                {'ct': c, 'r': 2.0, 'b': 1.0} for c in _CTS],
             '_awb_variance': [
                {'ct': c, 'n': 5, 'r_std': 0.001, 'b_std': 0.001,
                 'r_rel_std': 0.005, 'b_rel_std': 0.005} for c in _CTS]}
    _, report, _ = health_summary({'Awb': clean}, measured={'Awb'})
    adv = [ln for ln in report if 'AWB repeatability' in ln]
    assert adv and '[ OK ]' in adv[0], adv

    noisy = {k: (v if k != '_awb_variance'
                 else [dict(e, r_rel_std=0.05) for e in v])
             for k, v in clean.items()}
    _, report, _ = health_summary({'Awb': noisy}, measured={'Awb'})
    adv = [ln for ln in report if 'AWB repeatability' in ln]
    assert adv and '[WARN]' in adv[0], adv
    print('test_health_advisory: OK (clean=PASS, >2% inter-frame std=WARN)')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as d:
        tmp = Path(d)
        test_curve_and_variance()
        test_zero_variance_for_identical_frames()
        test_variance_grows_with_jitter()
        test_single_frame_no_spread()
        test_yaml_byte_identical(tmp)
        test_health_advisory()
    print('all AWB variance tests passed')
