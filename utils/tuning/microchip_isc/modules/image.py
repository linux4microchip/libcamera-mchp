# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# ISC calibration image: loads a RAW DNG/TIFF capture and extracts
# per-channel Bayer data + optional Macbeth chart patch data.

import json
import logging
import re
from pathlib import Path

import numpy as np

try:
    import rawpy
except ImportError:
    rawpy = None

try:
    from PIL import Image as PILImage
except ImportError:
    PILImage = None

logger = logging.getLogger(__name__)


class IscImage:
    """
    Container for one calibration capture.

    Supported input formats:
      .dng / .tiff / .tif  — Adobe DNG or TIFF with Bayer data (rawpy)
      .raw + .json         — v4l2-ctl raw capture (BLC dark frames) with sidecar
      .raw (no .json)      — metadata inferred from filename tags

    Attributes:
        path        : path to the RAW file
        name        : filename stem (used in log messages)
        ct          : true illuminant CCT in K (from filename ct<K> tag or
                      sidecar true_ct field — never from IPA estimate)
        lux         : incident illuminance (lux), from filename or sidecar
        exposure    : exposure time in microseconds
        again       : analogue gain (linear)
        wb_r/gr/gb/b: WB gains applied by the IPA at capture time
        channels    : list of 4 2-D float32 arrays [R, GR, GB, B] in ISC
                      histogram bin units (0-511, pedestal-subtracted, WB-undone)
        patches     : list of 4 arrays of patch means, indexed 0-23
        macbeth     : True if patch data available
    """

    PATCH_NAMES = [
        'dark_skin', 'light_skin', 'blue_sky', 'foliage',
        'blue_flower', 'bluish_green', 'orange', 'purplish_blue',
        'moderate_red', 'purple', 'yellow_green', 'orange_yellow',
        'blue', 'green', 'red', 'yellow', 'magenta', 'cyan',
        'white', 'neutral_8', 'neutral_6_5', 'neutral_5',
        'neutral_3_5', 'black',
    ]

    # X-Rite ColorChecker Classic reference sRGB values (D50, 2005)
    REFERENCE_SRGB = [
        (115, 82,  68),  (194, 150, 130), (98,  122, 157), (87,  108, 67),
        (133, 128, 177), (103, 189, 170), (214, 126, 44),  (80,  91,  166),
        (193, 90,  99),  (94,  60,  108), (157, 188, 64),  (224, 163, 46),
        (56,  61,  150), (70,  148, 73),  (175, 54,  60),  (231, 199, 31),
        (187, 86,  149), (8,   133, 161), (243, 243, 243), (200, 200, 200),
        (160, 160, 160), (122, 122, 122), (85,  85,  85),  (52,  52,  52),
    ]

    GREY_PATCH_INDICES = [18, 19, 20, 21, 22, 23]

    def __init__(self, path: str):
        self.path = Path(path)
        self.name = self.path.stem
        self.channels = None
        self.patches  = None
        self.macbeth  = None

        # Parse metadata from filename tags.
        # Use None sentinel for again so we can distinguish "no tag" from "tag=1.0".
        self.ct       = self._parse_tag('ct',   None, float)
        self.lux      = self._parse_tag('lux',  None, float)
        self.exposure = self._parse_tag('exp',  None, float)  # µs
        self.again    = self._parse_tag('gain', None, float)  # None = not in filename

        # WB gains (populated from sidecar during load)
        self.wb_r  = 1.0
        self.wb_gr = 1.0
        self.wb_gb = 1.0
        self.wb_b  = 1.0

        self._load()

    def _parse_tag(self, tag, default, cast):
        """Parse a tag value from the filename: tag<value>."""
        # Use word-boundary equivalent to avoid matching e.g. 'gain' in 'analogue_gain'
        m = re.search(rf'(?<![a-z]){tag}(\d+\.?\d*)', self.name, re.IGNORECASE)
        if m:
            return cast(m.group(1))
        return default

    def _load(self):
        suffix = self.path.suffix.lower()
        if suffix in ('.dng', '.tiff', '.tif'):
            self._load_rawpy()
        elif suffix == '.raw':
            self._load_mchpcam_raw()
        elif suffix == '.pgm':
            self._load_pgm()
        else:
            raise ValueError(f'Unsupported format: {suffix}')

    def _load_rawpy(self):
        """Load DNG/TIFF via rawpy. Delivers channels in histogram bin units."""
        if rawpy is None:
            raise ImportError('rawpy required: pip3 install rawpy')

        logger.info(f'Loading DNG/TIFF: {self.path}')
        with rawpy.imread(str(self.path)) as raw:
            bayer = raw.raw_image_visible.astype(np.float32)

            # Exposure: use EXIF exposure time (fractions.Fraction → float µs).
            # raw.camera_whitebalance is a WB multiplier, NOT exposure time.
            if self.exposure is None:
                try:
                    exp_s = float(raw.metadata.exposure_time)
                    self.exposure = exp_s * 1e6  # convert to µs
                except Exception:
                    self.exposure = None

            # Analogue gain from EXIF ISO if not in filename
            if self.again is None:
                try:
                    self.again = float(raw.metadata.iso) / 100.0
                except Exception:
                    self.again = 1.0

            pattern   = raw.raw_pattern
            color_map = raw.color_desc.decode()
            logger.debug(f'Bayer: {color_map}  pattern={pattern}')

            self.channels = self._extract_channels(bayer, pattern, color_map)

            # Subtract rawpy black levels. rawpy reports these in the same
            # bit-depth as raw_image_visible (typically 16-bit for DNG).
            bl = raw.black_level_per_channel
            for i, ch in enumerate(self.channels):
                self.channels[i] = np.clip(ch - float(bl[i]), 0.0, None)

            # Normalise to ISC histogram bin units (0-511): divide by 2.
            # The ISC histogram bins pixel values as bin = pixel >> 1 over
            # 10-bit data. DNG rawpy data is typically 14-bit or 16-bit;
            # we scale to 10-bit first then divide by 2.
            raw_max = float(raw.white_level - raw.black_level_per_channel[0])
            for i, ch in enumerate(self.channels):
                # Scale to 10-bit equivalent then to histogram bins
                self.channels[i] = ch / raw_max * 511.0

        logger.info(f'  CT={self.ct}K  lux={self.lux}  '
                    f'exp={self.exposure}us  gain={self.again}')

    def _load_mchpcam_raw(self):
        """
        Load a raw Bayer frame (.raw + optional .json sidecar).

        Used for BLC dark frame captures produced by _capture_blc_on_board()
        (v4l2-ctl --stream-to, no IPA running, WB gains = 1.0) and for
        offline analysis of raw frames from v4l2-ctl.

        Note: mchpcam-still no longer produces .raw files. The ISC DMA path
        has WB gains applied when the IPA is active; those captures are not
        true pre-WB raw and are not suitable for calibration. BLC dark frames
        captured by _capture_blc_on_board() bypass the IPA (no AGC, no WB)
        and write wb_gains all 1.0 in the sidecar, so no WB undo is needed.

        Pipeline note for WB-applied captures (sidecar wb_gains != 1.0):
          1. Subtract per-channel black level (10-bit DN from sidecar).
          2. Divide by 2: bin = pixel >> 1 (512 bins over 10-bit range).
          3. Undo WB gains from sidecar: pre_wb = post_wb / wb_gain.
        """
        json_path = self.path.with_suffix('.json')
        meta = {}
        if json_path.exists():
            with open(json_path) as f:
                meta = json.load(f)
            logger.debug(f'Loaded sidecar: {json_path}')
        else:
            logger.warning(f'{self.name}: no JSON sidecar. WB gains assumed 1.0.')

        width  = meta.get('width')
        height = meta.get('height')
        fmt    = meta.get('format', 'RG10').upper()
        bayer_pattern = meta.get('bayer_pattern', '').upper()

        if width is None or height is None:
            m = re.search(r'(\d+)[x_](\d+)', self.name)
            if m:
                width, height = int(m.group(1)), int(m.group(2))
            else:
                width, height = 1920, 1080
                logger.warning(f'{self.name}: dimensions unknown, assuming {width}x{height}')

        # Metadata: sidecar overrides filename tags, but only when filename
        # tag is absent (None sentinel). Explicit filename tags take priority.
        if self.exposure is None:
            v = meta.get('exposure_us')
            self.exposure = float(v) if v is not None else None
        if self.again is None:
            self.again = float(meta.get('analogue_gain', 1.0))

        # True CCT: filename tag or sidecar true_ct only — never IPA estimate
        if self.ct is None and 'true_ct' in meta:
            self.ct = float(meta['true_ct'])
        if self.ct is None and ('awb_ct_smoothed' in meta or 'ipa_ct_smoothed' in meta):
            ipa_ct = meta.get('ipa_ct_smoothed', meta.get('awb_ct_smoothed', 0))
            logger.warning(f'{self.path.name}: no true_ct in sidecar. '
                           f'IPA estimate ({ipa_ct:.0f}K) will NOT be used as '
                           f'ground truth.')
        if self.lux is None and 'lux' in meta:
            self.lux = float(meta['lux'])

        # Per-channel black levels (10-bit DN)
        bl_default = int(meta.get('black_level', 64))
        if bl_default == 0:
            logger.debug(f'{self.name}: black_level=0 in sidecar (expected for BLC '
                         f'dark frame — no BLC applied to the raw pixels).')
        elif bl_default < 32 or bl_default > 128:
            logger.warning(f'{self.name}: black_level={bl_default} DN outside expected '
                           f'range [32, 128]. Verify the sidecar was written by a recent '
                           f'BLC calibration run. IMX219 datasheet pedestal is 64 DN.')
        bl = {
            'R':  float(meta.get('black_level_r',  bl_default)),
            'GR': float(meta.get('black_level_gr', bl_default)),
            'GB': float(meta.get('black_level_gb', bl_default)),
            'B':  float(meta.get('black_level_b',  bl_default)),
        }

        # WB gains (store as instance attributes for use by lux.py)
        self.wb_r  = float(meta.get('wb_gains_r',  1.0))
        self.wb_gr = float(meta.get('wb_gains_gr', 1.0))
        self.wb_gb = float(meta.get('wb_gains_gb', 1.0))
        self.wb_b  = float(meta.get('wb_gains_b',  1.0))
        wb = {'R': self.wb_r, 'GR': self.wb_gr, 'GB': self.wb_gb, 'B': self.wb_b}

        has_wb = any(v != 1.0 for v in wb.values())
        if not has_wb:
            logger.warning(f'{self.name}: WB gains not in sidecar, assuming 1.0. '
                           f'CT/AWB calibration accuracy reduced.')

        logger.info(f'Loading .raw: {self.path.name}  {width}x{height}  fmt={fmt}')

        raw_bytes = np.frombuffer(self.path.read_bytes(), dtype=np.uint8)

        # Normalise libcamera canonical format names to the ISC shorthand used
        # internally. v4l2-ctl writes 'RG10'; JSON sidecars from other tools
        # may write the libcamera canonical name 'SRGGB10'.
        _LIBCAMERA_TO_ISC = {
            'SRGGB10': 'RG10', 'SGRBG10': 'BA10',
            'SBGGR10': 'BG10', 'SGBRG10': 'GB10',
            'SRGGB8':  'RGGB', 'SGRBG8':  'BA81',
            'SBGGR8':  'BGGR', 'SGBRG8':  'GBRG',
        }
        # Also infer bayer_pattern from libcamera format name if not in sidecar
        _FMT_TO_BAYER = {
            'SRGGB10': 'RGGB', 'SRGGB8': 'RGGB',
            'SGRBG10': 'GRBG', 'SGRBG8': 'GRBG',
            'SBGGR10': 'BGGR', 'SBGGR8': 'BGGR',
            'SGBRG10': 'GBRG', 'SGBRG8': 'GBRG',
        }
        if fmt in _LIBCAMERA_TO_ISC:
            if not bayer_pattern and fmt in _FMT_TO_BAYER:
                bayer_pattern = _FMT_TO_BAYER[fmt]
            fmt = _LIBCAMERA_TO_ISC[fmt]

        if fmt in ('RG10', 'BA10', 'GB10', 'BG10'):
            # 10-bit packed: 4 pixels per 5 bytes (MIPI CSI-2 format)
            # OR 16-bit container (ISC DMA with zero-padded MSB)
            expected_packed = ((width * height + 3) // 4) * 5
            expected_16bit  = width * height * 2
            if len(raw_bytes) == expected_16bit:
                bayer = raw_bytes.view(np.uint16).astype(np.float32)
            elif len(raw_bytes) == expected_packed:
                bayer = self._unpack_mipi10(raw_bytes, width, height)
            else:
                logger.warning(f'{self.name}: unexpected size {len(raw_bytes)} bytes '
                               f'(expected {expected_16bit} 16-bit or {expected_packed} '
                               f'MIPI10-packed). Trying 16-bit view.')
                bayer = raw_bytes.view(np.uint16).astype(np.float32)
        elif fmt in ('RGGB', 'BA81', 'GRBG', 'GBRG', 'BGGR'):
            bayer = raw_bytes.astype(np.float32)
        else:
            raise ValueError(
                f'Unsupported RAW format: {fmt!r}. '
                f'Supported: RG10/BA10/GB10/BG10 (10-bit), SRGGB10/SGRBG10/... '
                f'(libcamera canonical), RGGB/BA81/GRBG/GBRG/BGGR (8-bit)'
            )

        bayer = bayer.reshape(height, width)

        fmt_to_pattern = {
            'RG10': 'RGGB', 'RGGB': 'RGGB',
            'BA10': 'GRBG', 'BA81': 'GRBG',
            'GB10': 'GBRG', 'GBRG': 'GBRG',
            'BG10': 'BGGR',
        }
        if bayer_pattern in ('RGGB', 'GRBG', 'GBRG', 'BGGR'):
            pattern_str = bayer_pattern
        else:
            pattern_str = fmt_to_pattern.get(fmt, 'RGGB')

        # Pattern maps: indices are rawpy-style color codes (0=R,1=G,2=B,3=G)
        # where G at (row,col) with row containing R is GR, other G is GB.
        # Verified for all 4 standard Bayer orderings.
        pattern_map = {
            'RGGB': np.array([[0, 1], [2, 3]]),  # R G / G B  → R(0,0) GR(0,1) GB(1,0) B(1,1)
            'GRBG': np.array([[1, 0], [3, 2]]),  # G R / B G  → GR(0,0) R(0,1) B(1,0) GB(1,1)
            'GBRG': np.array([[1, 2], [0, 3]]),  # G B / R G  → GB(0,0) B(0,1) R(1,0) GR(1,1)
            'BGGR': np.array([[3, 2], [1, 0]]),  # B G / G R  → B(0,0) GB(0,1) GR(1,0) R(1,1)
        }
        np_pattern = pattern_map.get(pattern_str, pattern_map['RGGB'])
        self.channels = self._extract_channels(bayer, np_pattern, pattern_str)

        # Determine whether the DMA output has WB gains applied.
        # On the SAMA7G5 ISC, the WB block sits between the PFE and the CFA.
        # For processed formats (YUYV, RGB565, etc.), WB IS applied before DMA.
        # For raw Bayer formats (RG10 / SRGGB10), the DMA path bypasses WB —
        # the output is pre-WB raw sensor data, identical to the histogram tap.
        # Dividing pre-WB data by WB gains would doubly remove the correction
        # and underestimate R and B channels by 1.3–2.3×, inflating AWB gains.
        #
        # Detection: formats explicitly known to be post-WB:
        #   Any non-Bayer ISC output (YUYV, RGB565, etc.)
        # Formats known to be pre-WB (raw Bayer passthrough):
        #   RG10, BA10, GB10, BG10 (all 10-bit packed Bayer DMA formats)
        # All format strings recognised for raw Bayer captures.
        # 'RG10' is written by _capture_blc_on_board() (v4l2-ctl).
        # 'SRGGB10' is the libcamera canonical name; may appear in sidecars
        # from offline analysis tools. Both represent 10-bit packed Bayer.
        # Alternative Bayer orderings are included for completeness.
        _RAW_BAYER_FMTS = {
            'RG10', 'BA10', 'GB10', 'BG10',          # ISC DMA shorthand
            'SRGGB10', 'SGRBG10', 'SBGGR10', 'SGBRG10',  # libcamera canonical
            'SRGGB8',  'SGRBG8',  'SBGGR8',  'SGBRG8',   # 8-bit variants
            'RGGB', 'BA81', 'GRBG', 'GBRG', 'BGGR',  # rawpy pattern strings
        }
        is_raw_bayer = fmt.upper() in _RAW_BAYER_FMTS
        if is_raw_bayer and has_wb:
            logger.info(f'{self.name}: raw Bayer DMA — WB undo skipped '
                        f'(pre-WB data; gains stored for lux reference only)')

        ch_names = ['R', 'GR', 'GB', 'B']
        for i, name in enumerate(ch_names):
            ch = self.channels[i]
            # Step 1: subtract black level pedestal (10-bit DN)
            ch = np.clip(ch - bl[name], 0.0, None)
            # Step 2: convert to histogram bin units (bin = pixel >> 1)
            ch = ch / 2.0
            # Step 3: undo IPA WB gain applied to DMA output.
            # Skip for raw Bayer formats where DMA is pre-WB (see above).
            if not is_raw_bayer:
                wg = wb[name]
                if wg > 0.01:
                    ch = ch / wg
                elif wg > 0.0:
                    logger.warning(f'{self.name}: WB gain {name}={wg:.4f} near zero, '
                                   f'skipping WB undo for this channel.')
            self.channels[i] = ch

        logger.info(f'  CT={self.ct}K  lux={self.lux}  '
                    f'exp={self.exposure}us  gain={self.again}')
        means = [float(ch.mean()) for ch in self.channels]
        logger.debug(f'  Channel means (hist bins, pre-WB): '
                     f'R={means[0]:.2f} GR={means[1]:.2f} '
                     f'GB={means[2]:.2f} B={means[3]:.2f}')

    def _load_pgm(self):
        """
        Load a 16-bit binary PGM produced by mchpcam-still --raw.

        Format produced by savePgm():
          - P5 header: "P5\\n<width> <height>\\n1023\\n"
          - Pixel data: width × height × uint16 big-endian, 10-bit in [0,1023]
          - Bayer pattern: RGGB  (IMX219 default, no flip, first pixel = R)

        The ISC DMA path keeps WB_ENABLE active even for Bayer output
        (bits_pipeline = WB_ENABLE, confirmed in kernel driver).  mchpcam-still
        --raw forces unity WB gains (512 = 1.0× in ISC Q9 scale), making the
        WB block a no-op.  The PGM pixels are therefore equivalent to true
        pre-WB sensor data — no WB undo is needed.

        Channel layout (GRBG 2×2 superpixel, top-left origin):
          row 0: Gr B  Gr B  ...
          row 1: R  Gb R  Gb ...
          → channels = [R, GR, GB, B]  (matches IscImage convention)
        """
        raw_bytes = self.path.read_bytes()

        # --- Parse PGM header ---
        # Header is ASCII lines before the binary data.
        # Lines: "P5", "<W> <H>", "<maxval>", then binary data.
        pos = 0
        lines = []
        while len(lines) < 3:
            end = raw_bytes.index(b'\n', pos)
            line = raw_bytes[pos:end].decode('ascii').strip()
            pos = end + 1
            # Skip PGM comment lines (rare but spec-legal)
            if line.startswith('#'):
                continue
            lines.append(line)

        if lines[0] != 'P5':
            raise ValueError(f'{self.path.name}: not a P5 PGM (got {lines[0]!r})')

        width, height = map(int, lines[1].split())
        maxval = int(lines[2])
        if maxval != 1023:
            logger.warning(f'{self.path.name}: PGM maxval={maxval}, expected 1023 '
                           f'for 10-bit SGRBG10. Scaling will be incorrect.')

        # --- Load pixel data ---
        pixel_bytes = raw_bytes[pos:]
        expected = width * height * 2
        if len(pixel_bytes) != expected:
            raise ValueError(f'{self.path.name}: expected {expected} pixel bytes, '
                             f'got {len(pixel_bytes)}')

        # Big-endian uint16 → float32
        bayer = np.frombuffer(pixel_bytes, dtype='>u2').astype(np.float32)
        bayer = bayer.reshape(height, width)

        # --- Load JSON sidecar ---
        json_path = self.path.with_suffix('.json')
        meta = {}
        if json_path.exists():
            with open(json_path) as f:
                meta = json.load(f)
            logger.debug(f'Loaded sidecar: {json_path.name}')
        else:
            logger.warning(f'{self.name}: no JSON sidecar.')

        # Populate metadata from sidecar (filename tags take priority)
        if self.exposure is None:
            v = meta.get('exposure_us')
            self.exposure = float(v) if v is not None else None
        if self.again is None:
            self.again = float(meta.get('analogue_gain', 1.0))
        if self.ct is None and 'true_ct' in meta:
            self.ct = float(meta['true_ct'])
        if self.lux is None and 'lux' in meta:
            self.lux = float(meta['lux'])

        # WB gains are unity by construction (--raw forces 512/512 = 1.0)
        # Store them so downstream code that reads wb_r etc. sees 1.0.
        self.wb_r = self.wb_gr = self.wb_gb = self.wb_b = 1.0

        logger.info(f'Loading PGM: {self.path.name}  {width}x{height}  '
                    f'SRGGB10 RGGB (unity WB, pre-WB equivalent)')

        # --- Extract 4 Bayer channels from RGGB superpixel ---
        # IMX219 default output: SRGGB10_1X10 (RGGB, no flip).
        # Confirmed via kernel imx219_mbus_formats[0] = MEDIA_BUS_FMT_SRGGB10_1X10
        # and media-ctl -p showing SRGGB10_1X10 on all pipeline pads.
        #
        # RGGB layout:
        #   even rows: R(col0)  Gr(col1) R(col2)  Gr(col3) ...
        #   odd  rows: Gb(col0) B(col1)  Gb(col2) B(col3)  ...
        #
        # Convert pixel values to histogram bin units: bin = pixel >> 1
        # (512 bins cover the 10-bit range 0..1023; each bin spans 2 DN)
        r  = bayer[0::2, 0::2] / 2.0   # even row, even col
        gr = bayer[0::2, 1::2] / 2.0   # even row, odd  col
        gb = bayer[1::2, 0::2] / 2.0   # odd  row, even col
        b  = bayer[1::2, 1::2] / 2.0   # odd  row, odd  col

        # channels order: [R, GR, GB, B]  (IscImage convention)
        self.channels = [r, gr, gb, b]

        means = [float(ch.mean()) for ch in self.channels]
        logger.info(f'  CT={self.ct}K  lux={self.lux}  '
                    f'exp={self.exposure}us  gain={self.again}')
        logger.debug(f'  Channel means (hist bins, pre-WB): '
                     f'R={means[0]:.2f} GR={means[1]:.2f} '
                     f'GB={means[2]:.2f} B={means[3]:.2f}')

    @staticmethod
    def _unpack_mipi10(raw_bytes, width, height):
        """Unpack MIPI CSI-2 10-bit packed data to float32 array.
        Format: 4 pixels in 5 bytes. n_groups = ceil(W*H / 4).
        """
        n_groups = (width * height + 3) // 4
        out = np.zeros(height * width, dtype=np.float32)
        for g in range(n_groups):
            base = g * 5
            if base + 5 > len(raw_bytes):
                # Incomplete last group — partial frame or truncated file
                logger.warning(f'MIPI10 unpack: truncated at group {g} '
                               f'(base={base}, len={len(raw_bytes)})')
                break
            b = raw_bytes[base:base+5]
            out[g*4+0] = (b[0] << 2) | (b[4] & 0x03)
            out[g*4+1] = (b[1] << 2) | ((b[4] >> 2) & 0x03)
            out[g*4+2] = (b[2] << 2) | ((b[4] >> 4) & 0x03)
            out[g*4+3] = (b[3] << 2) | ((b[4] >> 6) & 0x03)
        return np.clip(out, 0.0, 1023.0)

    @staticmethod
    def _extract_channels(bayer, pattern, color_desc):
        """
        Extract [R, GR, GB, B] 2-D channel arrays from a Bayer mosaic.

        GR = green in the same row as red.
        GB = green in the same row as blue.
        Identification: the first G encountered in row-major scan of the
        2×2 pattern is GR if that row also contains R, else GB.
        """
        h, w = bayer.shape
        ch_map = {}  # name -> (row_offset, col_offset)
        seen_g = 0

        # First pass: locate R and B to determine which row each is in
        r_row = None
        for r in range(2):
            for c in range(2):
                idx = pattern[r, c]
                letter = color_desc[idx] if idx < len(color_desc) else '?'
                if letter == 'R':
                    r_row = r
                    break

        for r in range(2):
            for c in range(2):
                idx = pattern[r, c]
                letter = color_desc[idx] if idx < len(color_desc) else '?'
                if letter == 'G':
                    # GR if this green is in the same row as red, else GB
                    if r_row is not None and r == r_row:
                        key = 'GR'
                    elif seen_g == 0:
                        # No R found yet (e.g. pure G-B pattern); use scan order
                        key = 'GR'
                    else:
                        key = 'GB'
                    seen_g += 1
                else:
                    key = letter  # 'R' or 'B'
                ch_map[key] = (r, c)

        channels = {}
        for name, (ro, co) in ch_map.items():
            channels[name] = bayer[ro::2, co::2]

        return [channels['R'], channels['GR'], channels['GB'], channels['B']]

    def set_macbeth_patches(self, corners):
        """
        Extract 24 ColorChecker patch means from 4 corner pixel coordinates.
        corners: [(tl_x, tl_y), (tr_x, tr_y), (bl_x, bl_y), (br_x, br_y)]
        """
        tl, tr, bl, br = corners
        n_cols, n_rows = 6, 4
        patch_means = [[], [], [], []]

        # Channels are half-resolution (2×2 Bayer → 1 sample per channel)
        scale = 0.5

        # Adaptive patch sampling radius: use 15% of the patch size so the
        # window scales correctly from 640×480 to 3264×2464 captures.
        # At 1920×1080 with chart at 40% FOV: patch ~120×90 px → radius ~13 px.
        # At 640×480: patch ~40×30 px → radius ~5 px (minimum for SNR).
        # At 3264×2464: patch ~200×150 px → radius ~22 px (avoids patch edges).
        # Channel arrays are half-resolution (one sample per 2×2 Bayer block),
        # so full-res corners are scaled by 0.5 before computing the centroid.
        chart_w_half = (tr[0] - tl[0]) * 0.5 * scale   # chart width in channel px
        chart_h_half = (bl[1] - tl[1]) * 0.5 * scale   # chart height in channel px
        patch_w_half = chart_w_half / n_cols
        patch_h_half = chart_h_half / n_rows
        # radius = 30% of half-patch-size, clamped to [3, 30] channel pixels
        radius = int(round(max(3, min(30, 0.30 * min(patch_w_half, patch_h_half)))))

        for row in range(n_rows):
            for col in range(n_cols):
                fx = (col + 0.5) / n_cols
                fy = (row + 0.5) / n_rows
                top_x = tl[0] + fx * (tr[0] - tl[0])
                top_y = tl[1] + fx * (tr[1] - tl[1])
                bot_x = bl[0] + fx * (br[0] - bl[0])
                bot_y = bl[1] + fx * (br[1] - bl[1])
                cx = int((top_x + fy * (bot_x - top_x)) * scale)
                cy = int((top_y + fy * (bot_y - top_y)) * scale)

                for i, ch in enumerate(self.channels or []):
                    h, w = ch.shape
                    r0 = max(0, cy - radius)
                    r1 = min(h, cy + radius)
                    c0 = max(0, cx - radius)
                    c1 = min(w, cx + radius)
                    if r1 > r0 and c1 > c0:
                        patch_means[i].append(float(ch[r0:r1, c0:c1].mean()))
                    else:
                        logger.warning(f'Patch ({row},{col}) at ({cx},{cy}) is '
                                       f'outside channel bounds ({w}x{h}). '
                                       f'Check chart corners are within the image.')
                        patch_means[i].append(float('nan'))

        self.patches = [np.array(pm) for pm in patch_means]
        self.macbeth = True
        logger.info(f'  Extracted 24 ColorChecker patches from {self.name}')

    @property
    def grey_patch_means(self):
        """Return per-channel means of the 6 neutral grey patches."""
        if self.patches is None:
            return None
        idx = self.GREY_PATCH_INDICES
        return {
            'R':  np.mean([self.patches[0][i] for i in idx]),
            'GR': np.mean([self.patches[1][i] for i in idx]),
            'GB': np.mean([self.patches[2][i] for i in idx]),
            'B':  np.mean([self.patches[3][i] for i in idx]),
        }
