#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Microchip Technology Inc.
#
# Board-free checks for the PGM Bayer loader in libtuning/image.py. These build
# synthetic PGM captures and exercise the real loader and LSC module (no stubs),
# since the capture helper writes 10-bit Bayer as PGM_SGRBG10 rather than DNG.

import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import libtuning as lt
from libtuning.image import Image
from libtuning.modules.lsc import LSCMicrochipISC


def write_pgm(path, arr, maxval=1023):
    h, w = arr.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n{maxval}\n'.encode())
        f.write(arr.astype('>u2').tobytes())


def write_sidecar(pgm_path, **fields):
    with open(pgm_path.with_suffix('.json'), 'w') as f:
        json.dump(fields, f)


# @brief A PGM mosaic decodes to channels R, GR, GB, B for either Bayer phase
def test_decode(tmp):
    shift = 64  # 1 << (16 - 10)

    def check(fourcc, layout):
        tl, tr, bl, br = layout
        mosaic = np.zeros((4, 4), dtype=np.uint16)
        mosaic[0::2, 0::2] = tl
        mosaic[0::2, 1::2] = tr
        mosaic[1::2, 0::2] = bl
        mosaic[1::2, 1::2] = br

        p = tmp / f'grey_card_ct4500K_lux1000_{fourcc}_raw.pgm'
        write_pgm(p, mosaic)
        write_sidecar(p, format=f'PGM_{fourcc}', width=4, height=4)

        img = Image(p)
        assert (img.w, img.h) == (4, 4), (img.w, img.h)
        assert img.sigbits == 10, img.sigbits
        assert img.blacklevel_16 == 64 << 6, img.blacklevel_16
        return img

    r, gr, gb, b = 200, 100, 400, 300
    expect = [r * shift, gr * shift, gb * shift, b * shift]

    img = check('SRGGB10', (r, gr, gb, b))
    for i, want in enumerate(expect):
        got = img.channels[i]
        assert got.shape == (2, 2), got.shape
        assert np.all(got == want), (i, want, got)

    img = check('SGRBG10', (gr, r, b, gb))
    for i, want in enumerate(expect):
        assert np.all(img.channels[i] == want), (i, want, img.channels[i])

    print('test_decode: OK (RGGB + SGRBG both decode to R/GR/GB/B =', expect, ')')


# @brief LSC gain map end-to-end through the real .pgm loader
#
# A radial-falloff flat field decoded from a PGM must produce a per-channel gain
# map that is ~1.0 at the bright centre and rises toward the dark corners. This
# proves the decoded channels reach lsc/microchip_isc.py and no longer trip the
# "channels is None" guard.
def test_lsc(tmp):
    w, h = 128, 96
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    rn = np.sqrt(((xx - cx) / cx) ** 2 + ((yy - cy) / cy) ** 2)
    rn = np.clip(rn / rn.max(), 0.0, 1.0)
    base = 900.0
    falloff = base * (1.0 - 0.5 * rn ** 2)   # centre ~900, corners ~450
    mosaic = np.clip(np.rint(falloff), 0, 1023).astype(np.uint16)

    p = tmp / 'alsc_5000K_lux1000_raw.pgm'
    write_pgm(p, mosaic)
    write_sidecar(p, format='PGM_SRGGB10', width=w, height=h)

    img = Image(p)
    assert img.channels is not None
    img.lsc_only = True
    img.color = 5000

    lsc = LSCMicrochipISC(debug=[],
                          sector_shape=(16, 12),
                          sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                          sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                          sector_average_function=lt.average.Mean(),
                          smoothing_function=lt.smoothing.MedianBlur(3))

    out = lsc.process({'lsc': {'max_gain': 4.0}}, [img], {})
    assert out is not None, 'LSC returned None for a valid flat field'
    nx, ny = 16, 12
    assert out['grid_width'] == nx and out['grid_height'] == ny
    for ch in ('r', 'g', 'b'):
        assert len(out['gains'][ch]) == nx * ny, (ch, len(out['gains'][ch]))

    g = np.array(out['gains']['g']).reshape(ny, nx)
    centre = float(g[ny // 2, nx // 2])
    corner = float(np.mean([g[0, 0], g[0, -1], g[-1, 0], g[-1, -1]]))
    assert abs(g.min() - 1.0) < 0.05, g.min()
    assert corner > centre + 0.2, (centre, corner)
    assert 1.2 < g.max() <= 4.0, g.max()
    print(f'test_lsc: OK (centre gain {centre:.2f}, corner gain {corner:.2f}, '
          f'peak {g.max():.2f})')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as d:
        tmp = Path(d)
        test_decode(tmp)
        test_lsc(tmp)
    print('all PGM loader tests passed')
