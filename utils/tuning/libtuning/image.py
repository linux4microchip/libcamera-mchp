# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# Container for an image and associated metadata

import binascii
import json
import numpy as np
from pathlib import Path
import re

# pyexiv2 and rawpy are imported lazily inside the raw-decode methods below.
# They are only needed to read a raw Bayer image (DNG/TIFF); a metadata-only
# image carries its measurements in a JSON sidecar and never enters that path,
# so it loads on hosts where those C-extension libraries are unavailable.

import libtuning as lt
import libtuning.utils as utils
import logging

logger = logging.getLogger(__name__)


# Map a four-letter Bayer order (as used by V4L2 fourccs, e.g. SGRBG -> GRBG)
# to an Exif CFAPattern string, so a PGM sidecar can select the reorder tuple
# without carrying Exif tags.
_BAYER_NAME_TO_CFA = {
    'RGGB': '0 1 1 2',
    'GRBG': '1 0 2 1',
    'GBRG': '1 2 0 1',
    'BGGR': '2 1 1 0',
}


# @brief Reorder tuple for an Exif CFAPattern string
#
# The key is the 2x2 Bayer cell row-major in Exif numbering (0=R, 1=G, 2=B); the
# value rearranges the four demosaiced quadrants into R, GR, GB, B order. Kept a
# function (not a module-level dict) because lt.Color resolves only after the
# package finishes importing, and this module is imported partway through that.
def _bayer_order(cfa_pattern):
    return {
        '0 1 1 2': (lt.Color.R, lt.Color.GR, lt.Color.GB, lt.Color.B),
        '1 2 0 1': (lt.Color.GB, lt.Color.B, lt.Color.R, lt.Color.GR),
        '2 1 1 0': (lt.Color.B, lt.Color.GB, lt.Color.GR, lt.Color.R),
        '1 0 2 1': (lt.Color.GR, lt.Color.R, lt.Color.B, lt.Color.GB),
    }[cfa_pattern]


class Image:
    # Image file suffixes carrying a raw Bayer array. DNG/TIFF are decoded via
    # rawpy and py3exiv2; a P5 PGM mosaic is decoded with numpy alone, taking its
    # geometry and Bayer order from the JSON sidecar. Anything else is treated as
    # a metadata-only image (see below).
    raw_suffixes = ('.dng', '.tiff', '.tif', '.pgm')
    _dng_suffixes = ('.dng', '.tiff', '.tif')

    def __init__(self, path: Path):
        self.path = path
        self.lsc_only = False
        self.color = -1
        self.lux = -1
        self.macbeth = None
        self.channels = None
        # Optional per-image metadata sidecar (<stem>.json). Generic: any tuner
        # may attach measurement metadata (illuminant, exposure, sensor
        # statistics) to an image without encoding all of it in the filename.
        self.metadata = {}
        self._load_metadata_sidecar()

        # A metadata-only image (non-raw, e.g. JPEG) carries no Bayer array to
        # demosaic. It exists purely as a carrier for its sidecar measurements,
        # consumed by modules that read Image.metadata. Skip the raw decode
        # path so such images can be loaded on hosts without rawpy/pyexiv2 and
        # so no Macbeth chart is required of them.
        suffix = path.suffix.lower()
        if suffix in Image._dng_suffixes:
            try:
                self._load_metadata_exif()
            except Exception as e:
                logger.error(f'Failed to load metadata from {self.path}: {e}')
                raise e

            try:
                self._read_image_dng()
            except Exception as e:
                logger.error(f'Failed to load image data from {self.path}: {e}')
                raise e
        elif suffix == '.pgm':
            try:
                self._read_image_pgm()
            except Exception as e:
                logger.error(f'Failed to load image data from {self.path}: {e}')
                raise e

    @property
    def name(self):
        return self.path.name

    # @brief Load the optional <stem>.json metadata sidecar, if present
    #
    # The sidecar contents are stored verbatim in self.metadata as a plain
    # dict. Interpretation of the keys is left to the consuming module, so the
    # core loader stays generic (no vendor-specific field names here).
    def _load_metadata_sidecar(self):
        sidecar = self.path.with_suffix('.json')
        if not sidecar.exists():
            return
        try:
            with open(sidecar) as f:
                self.metadata = json.load(f)
        except Exception as e:
            logger.warning(f'{self.path.name}: failed to read metadata '
                           f'sidecar: {e}')

    # May raise KeyError as there are too many to check
    def _load_metadata_exif(self):
        # RawPy doesn't load all the image tags that we need, so we use py3exiv2
        import pyexiv2 as pyexif
        metadata = pyexif.ImageMetadata(str(self.path))
        metadata.read()

        # The DNG and TIFF/EP specifications use different IFDs to store the
        # raw image data and the Exif tags. DNG stores them in a SubIFD and in
        # an Exif IFD respectively (named "SubImage1" and "Photo" by pyexiv2),
        # while TIFF/EP stores them both in IFD0 (name "Image"). Both are used
        # in "DNG" files, with libcamera-apps following the DNG recommendation
        # and applications based on picamera2 following TIFF/EP.
        #
        # This code detects which tags are being used, and therefore extracts the
        # correct values.
        try:
            self.w = metadata['Exif.SubImage1.ImageWidth'].value
            subimage = 'SubImage1'
            photo = 'Photo'
        except KeyError:
            self.w = metadata['Exif.Image.ImageWidth'].value
            subimage = 'Image'
            photo = 'Image'
        self.pad = 0
        self.h = metadata[f'Exif.{subimage}.ImageLength'].value
        white = metadata[f'Exif.{subimage}.WhiteLevel'].value
        self.sigbits = int(white).bit_length()
        self.fmt = (self.sigbits - 4) // 2
        self.exposure = int(metadata[f'Exif.{photo}.ExposureTime'].value * 1000000)
        self.againQ8 = metadata[f'Exif.{photo}.ISOSpeedRatings'].value * 256 / 100
        self.againQ8_norm = self.againQ8 / 256
        self.camName = metadata['Exif.Image.Model'].value
        self.blacklevel = int(metadata[f'Exif.{subimage}.BlackLevel'].value[0])
        self.blacklevel_16 = self.blacklevel << (16 - self.sigbits)

        # Channel order depending on bayer pattern (see _bayer_order).
        # Note: This needs to be in IFD0
        cfa_pattern = metadata[f'Exif.{subimage}.CFAPattern'].value
        self.order = _bayer_order(cfa_pattern)

    def _read_image_dng(self):
        import rawpy as raw
        raw_im = raw.imread(str(self.path))
        raw_data = raw_im.raw_image
        shift = 16 - self.sigbits
        c0 = np.left_shift(raw_data[0::2, 0::2].astype(np.int64), shift)
        c1 = np.left_shift(raw_data[0::2, 1::2].astype(np.int64), shift)
        c2 = np.left_shift(raw_data[1::2, 0::2].astype(np.int64), shift)
        c3 = np.left_shift(raw_data[1::2, 1::2].astype(np.int64), shift)
        self.channels = [c0, c1, c2, c3]
        # Reorder the channels into R, GR, GB, B
        self.channels = [self.channels[i] for i in self.order]

    # @brief Decode a P5 (binary) PGM Bayer mosaic using numpy only
    #
    # The capture helper writes a full-resolution single-plane Bayer mosaic as a
    # PGM, with the geometry, pixel format and black level in the JSON sidecar.
    # Unlike the DNG path this needs neither rawpy nor pyexiv2, so it loads on any
    # host. It fills the same attributes (channels, w, h, sigbits, order, black
    # level, provenance) so the Macbeth and LSC paths treat it like a DNG.
    def _read_image_pgm(self):
        with open(self.path, 'rb') as f:
            if f.readline().strip() != b'P5':
                raise ValueError(f'{self.path.name}: not a binary (P5) PGM')

            # The width, height and maxval tokens may be split across lines and
            # interleaved with '#' comment lines. Read them one whitespace-
            # separated token at a time up to the single byte before the pixels.
            tokens = []
            while len(tokens) < 3:
                ch = f.read(1)
                if ch == b'':
                    raise ValueError(f'{self.path.name}: truncated PGM header')
                if ch == b'#':
                    f.readline()
                    continue
                if ch.isspace():
                    continue
                token = ch
                while True:
                    ch = f.read(1)
                    if ch == b'' or ch.isspace():
                        break
                    token += ch
                tokens.append(int(token))
            w, h, maxval = tokens

            depth = 2 if maxval > 255 else 1
            count = w * h
            buf = f.read(count * depth)

        if len(buf) < count * depth:
            raise ValueError(f'{self.path.name}: truncated PGM pixel data')

        dtype = '>u2' if depth == 2 else '>u1'
        mosaic = np.frombuffer(buf, dtype=dtype, count=count).reshape(h, w)

        self.w = w
        self.h = h
        self.pad = 0
        self.sigbits = int(maxval).bit_length()
        self.fmt = (self.sigbits - 4) // 2

        # Bayer order and black level come from the sidecar; fall back to the
        # IMX219 raw default (RGGB, RAW10 black level 64) when absent. mchpcam
        # forces SRGGB10 for --raw capture (RGGB, no flip), so RGGB is the
        # correct default rather than the sensor's flipped SGRBG phase. The 'S'
        # prefix on a V4L2 fourcc (e.g. SRGGB10) denotes a mosaic; strip it.
        fmt = str(self.metadata.get('format', ''))
        # Strip only the leading "PGM_" tag; keep the rest so a packed fourcc
        # (e.g. PGM_SGRBG10_CSI2P) still resolves its S<RGGB> phase below.
        bayer = fmt.split('_', 1)[1].upper() if '_' in fmt else ''
        if bayer.startswith('S'):
            bayer = bayer[1:]
        cfa = _BAYER_NAME_TO_CFA.get(bayer[:4], _BAYER_NAME_TO_CFA['RGGB'])
        self.order = _bayer_order(cfa)

        self.blacklevel = int(self.metadata.get('black_level', 64))
        self.blacklevel_16 = self.blacklevel << (16 - self.sigbits)

        # Provenance the DNG path also records, so modules that read these
        # attributes behave identically for a PGM capture.
        self.exposure = int(self.metadata.get('exposure_us', 0))
        self.againQ8 = int(round(float(self.metadata.get('analogue_gain', 1.0)) * 256))
        self.againQ8_norm = self.againQ8 / 256
        self.camName = str(self.metadata.get('camera', 'imx219'))

        # Deinterleave the Bayer quadrants and shift to 16-bit exactly as
        # _read_image_dng, then reorder into R, GR, GB, B.
        shift = 16 - self.sigbits
        c0 = np.left_shift(mosaic[0::2, 0::2].astype(np.int64), shift)
        c1 = np.left_shift(mosaic[0::2, 1::2].astype(np.int64), shift)
        c2 = np.left_shift(mosaic[1::2, 0::2].astype(np.int64), shift)
        c3 = np.left_shift(mosaic[1::2, 1::2].astype(np.int64), shift)
        self.channels = [c0, c1, c2, c3]
        self.channels = [self.channels[i] for i in self.order]

    # \todo Move this to macbeth.py
    def get_patches(self, cen_coords, size=16):
        saturated = False

        # Obtain channel widths and heights
        ch_w, ch_h = self.w, self.h
        cen_coords = list(np.array((cen_coords[0])).astype(np.int32))
        self.cen_coords = cen_coords

        # Squares are ordered by stacking macbeth chart columns from left to
        # right. Some useful patch indices:
        #     white = 3
        #     black = 23
        #     'reds' = 9, 10
        #     'blues' = 2, 5, 8, 20, 22
        #     'greens' = 6, 12, 17
        #     greyscale = 3, 7, 11, 15, 19, 23
        all_patches = []
        for ch in self.channels:
            ch_patches = []
            for cen in cen_coords:
                # Macbeth centre is placed at top left of central 2x2 patch to
                # account for rounding. Patch pixels are sorted by pixel
                # brightness so spatial information is lost.
                # Clamp the lower bound: a negative slice start would wrap to the
                # far edge and yield an empty patch (patch[-5] then throws), which
                # a full-frame chart with edge patches would trip.
                y0 = max(cen[1] - 7, 0)
                x0 = max(cen[0] - 7, 0)
                patch = ch[y0:cen[1] + 9, x0:cen[0] + 9].flatten()
                patch.sort()
                if patch[-5] == (2**self.sigbits - 1) * 2**(16 - self.sigbits):
                    saturated = True
                ch_patches.append(patch)

            all_patches.append(ch_patches)

        self.patches = np.array(all_patches)

        return not saturated
