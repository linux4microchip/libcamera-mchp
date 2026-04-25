# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# validation.py -- Pre-calibration image quality validation.
#
# Each validator returns a ValidationResult (PASS / WARN / FAIL).
# FAIL blocks the calibration module from running.
# WARN lets it run but prints a diagnostic.
#
# Validators:
#   validate_grey_card(images)        for ct / awb / lux modules
#   validate_colorchecker(images)     for ccm module
#   validate_blc(images)              for blc module
#   validate_lux(images)              for lux module
#   run_validation(module, images)    dispatches to the right validator
#
# Checks performed per module:
#
#  grey_card (ct/awb/lux):
#    [1] At least one image with JSON sidecar (histogram means present)
#    [2] Gain == 1.0  (pedestal not amplified)
#    [3] Not overexposed  (histogram mean < 90% of kHistMax=511)
#    [4] Not underexposed (histogram mean > 2 net bins above pedestal)
#    [5] GR/GB channel balance  (|GR-GB|/GR < 15% → neutral scene)
#    [6] B channel not floored  (net B > 1 bin — warns if NoIR issue)
#    [7] Scene is neutral  (chroma imbalance < 40% → not a colour chart)
#    [8] Consistent CT across images (warn if >800K spread — mixed sessions)
#
#  colorchecker (ccm):
#    [1] At least one PGM with patch data extracted
#    [2] Gain close to 1.0
#    [3] White patch (index 18) is brightest among grey patches
#    [4] Grey ramp monotonically decreasing  (patch 18→23)
#    [5] Patch variance high  (colour patches are distinguishable)
#    [6] No patch saturated  (any patch mean > 480/511 bins)
#    [7] No patch floored    (any patch mean < 2 net bins)
#    [8] White/black contrast ratio > 4×
#
#  blc:
#    [1] At least one dark frame
#    [2] All channel means very low  (< 40 bins — lens is capped)
#    [3] No B-channel NIR leak       (B/GR < 1.1)
#
#  lux:
#    [1] At least one image has a lux tag
#    [2] Exposure and gain present in sidecar
#    [3] Gain close to 1.0 for best SNR

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

logger = logging.getLogger(__name__)

# ISC hardware constants (must match common.h)
HIST_MAX    = 511.0
PEDESTAL_DN = 64       # IMX219 datasheet
PEDESTAL_BINS = PEDESTAL_DN / 2.0   # = 32


# ---------------------------------------------------------------------------
# Result types
# ---------------------------------------------------------------------------

class Severity(Enum):
    PASS = 'PASS'
    WARN = 'WARN'
    FAIL = 'FAIL'


@dataclass
class Check:
    name:     str
    severity: Severity
    message:  str
    detail:   str = ''


@dataclass
class ValidationResult:
    module:  str
    checks:  List[Check] = field(default_factory=list)

    @property
    def passed(self):
        return all(c.severity != Severity.FAIL for c in self.checks)

    @property
    def worst(self):
        if any(c.severity == Severity.FAIL for c in self.checks):
            return Severity.FAIL
        if any(c.severity == Severity.WARN for c in self.checks):
            return Severity.WARN
        return Severity.PASS

    def add(self, name, severity, message, detail=''):
        self.checks.append(Check(name, severity, message, detail))

    def print_report(self):
        icons = {Severity.PASS: '[PASS]', Severity.WARN: '[WARN]', Severity.FAIL: '[FAIL]'}
        print(f'\n  Validation: {self.module}  ({self.worst.value})')
        for c in self.checks:
            if c.severity == Severity.PASS:
                continue   # only print non-trivial checks
            icon = icons[c.severity]
            print(f'    {icon}  {c.name}: {c.message}')
            if c.detail:
                print(f'           {c.detail}')
        if self.worst == Severity.PASS:
            print('    [PASS]  All checks passed')
        print()


# ---------------------------------------------------------------------------
# Helper: extract histogram means from a list of images
# ---------------------------------------------------------------------------

def _grey_images(images):
    """Return CalibCapture images that have JSON sidecar histogram means."""
    try:
        from microchip_isc.modules.image import CalibCapture
    except ImportError:
        return []
    return [img for img in images
            if isinstance(img, CalibCapture) and img.hist is not None]


def _pgm_images(images):
    """Return IscImage objects that have patch data extracted."""
    try:
        from microchip_isc.modules.image import IscImage
    except ImportError:
        return []
    return [img for img in images
            if isinstance(img, IscImage) and img.macbeth]


def _net(raw, gain=1.0):
    """Convert raw histogram bin to net (pedestal-subtracted) value."""
    ped = PEDESTAL_BINS * max(1.0, gain)
    return max(0.0, raw - ped)


# ---------------------------------------------------------------------------
# Grey card validator  (ct / awb / lux)
# ---------------------------------------------------------------------------

def validate_grey_card(images):
    """
    Validate images for grey-card calibration (ct / awb / lux modules).
    """
    r = ValidationResult('grey_card')
    valid = _grey_images(images)

    # [1] Images with sidecar
    if not valid:
        r.add('sidecar', Severity.FAIL,
              'No images with histogram metadata found.',
              'Use calib.sh to capture -- it writes the JSON sidecar automatically.\n'
              '           Standalone JPEG captures without --metadata are not usable.')
        return r
    r.add('sidecar', Severity.PASS,
          f'{len(valid)} image(s) with histogram sidecar')

    for img in valid:
        name  = img.name
        hist  = img.hist    # dict with GR, R, GB, B keys
        gain  = img.gain if img.gain else 1.0
        ct    = img.ct   if img.ct   else 0

        raw_GR = float(hist.get('GR', 0))
        raw_R  = float(hist.get('R',  0))
        raw_GB = float(hist.get('GB', 0))
        raw_B  = float(hist.get('B',  0))

        net_GR = _net(raw_GR, gain)
        net_R  = _net(raw_R,  gain)
        net_GB = _net(raw_GB, gain)
        net_B  = _net(raw_B,  gain)
        mean_net = (net_GR + net_R + net_GB + net_B) / 4.0

        # [2] Gain
        if gain > 1.5:
            r.add(f'{name}:gain', Severity.FAIL,
                  f'Analogue gain = {gain:.1f}x (need 1.0x for calibration)',
                  'Increase scene brightness (more lux) so AGC settles at gain=1.\n'
                  '           Minimum recommended: 500 lux for IMX219.')
        elif gain > 1.05:
            r.add(f'{name}:gain', Severity.WARN,
                  f'Analogue gain = {gain:.2f}x (slightly above 1.0)',
                  'Results usable but accuracy reduced. Increase lightbox brightness.')
        else:
            r.add(f'{name}:gain', Severity.PASS, f'gain={gain:.2f}x')

        # [3] Overexposure
        if raw_GR > HIST_MAX * 0.92:
            r.add(f'{name}:exposure', Severity.FAIL,
                  f'Histogram saturated: raw_GR={raw_GR:.0f} bins (>{HIST_MAX*0.92:.0f})',
                  'Reduce lightbox intensity or add ND filter.')
        elif raw_GR > HIST_MAX * 0.80:
            r.add(f'{name}:exposure', Severity.WARN,
                  f'Close to saturation: raw_GR={raw_GR:.0f} bins',
                  'Reduce lightbox intensity slightly.')
        else:
            r.add(f'{name}:exposure', Severity.PASS, f'raw_GR={raw_GR:.0f} bins')

        # [4] Underexposure
        if net_GR < 3.0:
            r.add(f'{name}:underexpose', Severity.FAIL,
                  f'Scene too dark: net_GR={net_GR:.1f} bins (<3)',
                  'Increase lightbox intensity. Minimum: ~500 lux for gain=1.')
        elif net_GR < 8.0:
            r.add(f'{name}:underexpose', Severity.WARN,
                  f'Low signal: net_GR={net_GR:.1f} bins (<8)',
                  'AWB SNR reduced. Increase brightness if possible.')

        # [5] GR/GB balance (neutral grey check)
        if net_GR > 0 and net_GB > 0:
            imbalance = abs(net_GR - net_GB) / max(net_GR, net_GB)
            if imbalance > 0.15:
                r.add(f'{name}:gr_gb_balance', Severity.WARN,
                      f'GR/GB imbalance: {imbalance*100:.0f}% '
                      f'(GR={net_GR:.1f}, GB={net_GB:.1f})',
                      'Scene may not be neutral grey. Use 18% grey card.\n'
                      '           Sensor row-to-row variation above 15% is unusual -- check capture.')
            else:
                r.add(f'{name}:gr_gb_balance', Severity.PASS,
                      f'GR/GB imbalance={imbalance*100:.1f}%')

        # [6] B channel floor (NoIR without IR cut)
        if net_B < 0.5 and net_GR > 5.0:
            r.add(f'{name}:b_channel', Severity.WARN,
                  f'B channel at pedestal floor (net_B={net_B:.1f})',
                  'NoIR sensor without IR cut filter: B is depleted by IR.\n'
                  '           AWB bGain will be high. lux_reference calibration still valid.\n'
                  '           CCM calibration not recommended for this sensor+illuminant.')
        elif net_B < 2.0 and net_GR > 5.0:
            r.add(f'{name}:b_channel', Severity.WARN,
                  f'B channel very low (net_B={net_B:.1f})',
                  'Possible IR contamination or wrong illuminant for this sensor.')

        # [7] Scene neutrality (not pointing at colour chart or coloured wall)
        if net_GR > 2.0 and net_R > 0.5 and net_B > 0.5:
            chroma = max(
                abs(net_R  - net_GR) / max(net_GR, 0.1),
                abs(net_GB - net_GR) / max(net_GR, 0.1),
                abs(net_B  - net_GR) / max(net_GR, 0.1),
            )
            if chroma > 0.60:
                r.add(f'{name}:neutrality', Severity.FAIL,
                      f'Scene is NOT neutral grey (chroma={chroma*100:.0f}%)',
                      f'net: R={net_R:.1f} GR={net_GR:.1f} GB={net_GB:.1f} B={net_B:.1f}\n'
                      '           Use 18% grey card or Spyder Checkr grey ramp side.\n'
                      '           Do NOT use the colour checker side for ct/awb/lux.')
            elif chroma > 0.30:
                r.add(f'{name}:neutrality', Severity.WARN,
                      f'Scene has mild colour cast (chroma={chroma*100:.0f}%)',
                      f'net: R={net_R:.1f} GR={net_GR:.1f} GB={net_GB:.1f} B={net_B:.1f}\n'
                      '           AWB will compensate but accuracy is reduced.')
            else:
                r.add(f'{name}:neutrality', Severity.PASS,
                      f'Scene neutral (chroma={chroma*100:.1f}%)')

    # [8] CT consistency across images
    cts = [img.ct for img in valid if img.ct]
    if len(cts) >= 2:
        spread = max(cts) - min(cts)
        if spread > 800:
            r.add('ct_spread', Severity.WARN,
                  f'CT spread {spread:.0f}K across images ({min(cts):.0f}K – {max(cts):.0f}K)',
                  'Images from different sessions or different lightbox settings?\n'
                  '           Use --clean to start fresh if mixing old and new captures.')

    return r


# ---------------------------------------------------------------------------
# ColorChecker validator  (ccm)
# ---------------------------------------------------------------------------

def validate_colorchecker(images):
    """
    Validate ColorChecker images for CCM calibration.
    Called after set_macbeth_patches() so img.patches is populated.
    """
    r = ValidationResult('colorchecker_ccm')
    pgm_imgs = _pgm_images(images)

    # [1] At least one image with patch data
    if not pgm_imgs:
        # Check if there are PGM images without patches (corners not picked)
        try:
            from microchip_isc.modules.image import IscImage
            raw_imgs = [img for img in images
                        if isinstance(img, IscImage)
                        and img.name.lower().startswith('colorchecker')]
        except ImportError:
            raw_imgs = []

        if raw_imgs:
            r.add('patch_data', Severity.FAIL,
                  f'{len(raw_imgs)} colorchecker image(s) found but no corners picked.',
                  'Run with --macbeth and select corners when prompted.')
        else:
            r.add('patch_data', Severity.FAIL,
                  'No colorchecker*.pgm image found in calibration directory.',
                  'Capture with: /tmp/mchp_capture.sh -t colorchecker -c <CT> --raw')
        return r

    r.add('patch_data', Severity.PASS,
          f'{len(pgm_imgs)} image(s) with 24-patch data')

    for img in pgm_imgs:
        name = img.name
        gain = img.gain if img.gain else 1.0

        # patches[0]=R, patches[1]=GR, patches[2]=GB, patches[3]=B (index by channel)
        # patch order: row-major 6x4, index 0-23
        R_means  = img.patches[0]
        GR_means = img.patches[1]
        GB_means = img.patches[2]
        B_means  = img.patches[3]
        G_means  = (GR_means + GB_means) / 2.0

        # Use G channel as luminance proxy (most neutral, least NoIR-affected)
        lum = 0.299 * R_means + 0.587 * G_means + 0.114 * B_means

        # [2] Gain
        if gain > 1.5:
            r.add(f'{name}:gain', Severity.WARN,
                  f'Analogue gain={gain:.1f}x  (prefer 1.0x for CCM)',
                  'High gain introduces noise into per-patch means.')
        else:
            r.add(f'{name}:gain', Severity.PASS, f'gain={gain:.2f}x')

        # [3] White patch (index 18) is brightest grey patch
        grey_lum  = [lum[i] for i in [18, 19, 20, 21, 22, 23]]
        white_lum = lum[18]
        black_lum = lum[23]

        if any(lum[i] > white_lum for i in [19, 20, 21, 22, 23]):
            brightest_grey = max(range(18, 24), key=lambda i: lum[i])
            r.add(f'{name}:white_patch', Severity.WARN,
                  f'White patch (18) is NOT the brightest grey '
                  f'(patch {brightest_grey} is brighter: {lum[brightest_grey]:.1f} vs {white_lum:.1f})',
                  'Possibly wrong corner order (TL/TR/BL/BR swapped?) or\n'
                  '           card rotated 180 degrees. Re-pick corners.')
        else:
            r.add(f'{name}:white_patch', Severity.PASS,
                  f'White patch luminance={white_lum:.1f} bins')

        # [4] Grey ramp monotonically decreasing (18=white → 23=black)
        grey_ok = all(grey_lum[i] >= grey_lum[i+1] for i in range(len(grey_lum)-1))
        if not grey_ok:
            violations = [(i, grey_lum[i], grey_lum[i+1])
                          for i in range(len(grey_lum)-1)
                          if grey_lum[i] < grey_lum[i+1]]
            r.add(f'{name}:grey_ramp', Severity.WARN,
                  f'Grey ramp not monotonically decreasing',
                  f'Patches {[(18+v[0], 18+v[0]+1) for v in violations]} are inverted.\n'
                  '           This indicates wrong corner order or glare on grey patches.\n'
                  '           Re-pick corners carefully (TL=white, BR=black).')
        else:
            r.add(f'{name}:grey_ramp', Severity.PASS,
                  'Grey ramp monotonically decreasing (18=white → 23=black)')

        # [5] Patch variance (colour chart vs grey card check)
        lum_std  = float(lum.std())
        lum_mean = float(lum.mean())
        cv = lum_std / max(lum_mean, 1.0)   # coefficient of variation

        if cv < 0.10:
            r.add(f'{name}:patch_variance', Severity.FAIL,
                  f'Very low patch variance (CV={cv*100:.0f}%, std={lum_std:.1f})',
                  'Image looks like a grey card, NOT a colour checker.\n'
                  '           Flip to the colour checker side (24 coloured patches).\n'
                  '           The grey ramp side is for ct/awb/lux, NOT ccm.')
        elif cv < 0.20:
            r.add(f'{name}:patch_variance', Severity.WARN,
                  f'Low patch variance (CV={cv*100:.0f}%)',
                  'Colour patches not strongly saturated. Check card orientation\n'
                  '           and that lightbox illumination is even.')
        else:
            r.add(f'{name}:patch_variance', Severity.PASS,
                  f'Patch variance CV={cv*100:.0f}%  (colour patches distinguishable)')

        # [6] No patches saturated (glare check)
        sat_idx = [i for i in range(24) if lum[i] > HIST_MAX * 0.92]
        if sat_idx:
            r.add(f'{name}:saturation', Severity.FAIL,
                  f'{len(sat_idx)} patch(es) saturated: {sat_idx}',
                  'Glare on patches or image overexposed.\n'
                  '           Reduce lightbox intensity, tilt card slightly to eliminate glare.\n'
                  '           Check from camera angle before capturing.')
        else:
            r.add(f'{name}:saturation', Severity.PASS,
                  f'No patches saturated (max={max(lum):.0f} bins)')

        # [7] No patches floored (underexposure or corner error)
        floor_idx = [i for i in range(24) if G_means[i] < 2.0]
        if floor_idx:
            r.add(f'{name}:floor', Severity.FAIL,
                  f'{len(floor_idx)} patch(es) at noise floor: {floor_idx}',
                  'Patches outside image bounds or image severely underexposed.\n'
                  '           Re-pick corners ensuring all 24 patches are within the frame.')
        elif any(float(v) < 5.0 for v in G_means):
            low_idx = [i for i in range(24) if G_means[i] < 5.0]
            r.add(f'{name}:floor', Severity.WARN,
                  f'{len(low_idx)} dark patch(es) near noise floor: {low_idx}',
                  'Black patches (22-23) near floor is normal. Others: check corners.')
        else:
            r.add(f'{name}:floor', Severity.PASS, 'All patches above noise floor')

        # [8] White/black contrast ratio
        if black_lum > 1.0:
            ratio = white_lum / black_lum
            if ratio < 4.0:
                r.add(f'{name}:contrast', Severity.WARN,
                      f'White/black contrast ratio = {ratio:.1f}x (expect >4x)',
                      'Low contrast may indicate wrong card, wrong corners, or low lux.\n'
                      '           Check: patch 18=white, patch 23=black.')
            else:
                r.add(f'{name}:contrast', Severity.PASS,
                      f'White/black contrast = {ratio:.1f}x')

    return r


# ---------------------------------------------------------------------------
# BLC validator
# ---------------------------------------------------------------------------

def validate_blc(images):
    """
    Validate dark frame images for BLC calibration.
    """
    r = ValidationResult('blc')
    try:
        from microchip_isc.modules.image import IscImage
        dark_imgs = [img for img in images
                     if isinstance(img, IscImage)
                     and img.name.lower().startswith('blc_dark')]
    except ImportError:
        dark_imgs = []

    if not dark_imgs:
        r.add('dark_frame', Severity.FAIL,
              'No blc_dark*.pgm frame found.',
              'Cap the lens completely (not just a plastic cap -- use black tape\n'
              '           or a solid metal cap). Capture with:\n'
              '           /tmp/mchp_capture.sh --blc')
        return r

    r.add('dark_frame', Severity.PASS, f'{len(dark_imgs)} dark frame(s) found')

    for img in dark_imgs:
        name = img.name
        if img.channels is None:
            r.add(f'{name}:channels', Severity.FAIL,
                  'RAW channel data not loaded. PGM file may be corrupt.')
            continue

        means = [float(ch.mean()) for ch in img.channels]
        GR_mean, R_mean, GB_mean, B_mean = means

        # [2] All channels low (lens is capped)
        max_mean = max(means)
        if max_mean > 40.0:
            r.add(f'{name}:darkness', Severity.FAIL,
                  f'Dark frame not dark enough (max channel mean={max_mean:.1f} bins)',
                  'Lens may not be fully capped. Use opaque black tape.\n'
                  '           Do NOT use a translucent or thin plastic cap --\n'
                  '           even small light leaks inflate the pedestal measurement.')
        elif max_mean > 25.0:
            r.add(f'{name}:darkness', Severity.WARN,
                  f'Dark frame slightly bright (max={max_mean:.1f} bins)',
                  'Check for light leaks around lens cap edges.')
        else:
            r.add(f'{name}:darkness', Severity.PASS,
                  f'All channels dark (max mean={max_mean:.1f} bins)')

        # [3] B channel NIR check
        if GR_mean > 0.5:
            b_gr_ratio = B_mean / GR_mean
            if b_gr_ratio > 1.10:
                r.add(f'{name}:nir_leak', Severity.FAIL,
                      f'NIR contamination detected: B/GR={b_gr_ratio:.2f} (>1.10)',
                      'IR light leaking through the lens cap (plastic cap glows in IR).\n'
                      '           Use black electrical tape or metal cap to block IR.\n'
                      '           BLC values calibrated from this frame will be wrong.')
            else:
                r.add(f'{name}:nir_leak', Severity.PASS,
                      f'No NIR leak: B/GR={b_gr_ratio:.2f}')

    return r


# ---------------------------------------------------------------------------
# Lux validator
# ---------------------------------------------------------------------------

def validate_lux(images):
    """
    Validate images for lux reference calibration.
    """
    r = ValidationResult('lux')
    valid = _grey_images(images)
    lux_imgs = [img for img in valid
                if img.lux is not None and img.lux >= 1.0]

    # [1] At least one lux-tagged image
    if not lux_imgs:
        r.add('lux_tag', Severity.FAIL,
              'No images with measured lux value found.',
              'Images need lux<N> filename tag (e.g. grey_card_ct5000K_lux2370_...).\n'
              '           Use calib.sh which prompts for the handheld lux meter reading.\n'
              '           Without lux, AGC will use a 400 lux fallback (inaccurate).')
        return r

    r.add('lux_tag', Severity.PASS,
          f'{len(lux_imgs)} image(s) with measured lux')

    for img in lux_imgs:
        name = img.name
        gain = img.gain if img.gain else 1.0
        lux  = img.lux

        # [2] Exposure present
        if not img.exposure_us or img.exposure_us <= 0:
            r.add(f'{name}:exposure', Severity.FAIL,
                  'ExposureTime missing from JSON sidecar.',
                  'Lux calibration requires exposure time.\n'
                  '           Re-capture with mchpcam-still --metadata --metadata-format=json.')
        else:
            r.add(f'{name}:exposure', Severity.PASS,
                  f'exposure={img.exposure_us:.0f}us')

        # [3] Gain close to 1.0 for best accuracy
        if gain > 2.0:
            r.add(f'{name}:gain', Severity.WARN,
                  f'gain={gain:.1f}x for lux calibration (prefer 1.0x)',
                  f'At {lux:.0f} lux the sensor needs gain={gain:.1f}x.\n'
                  '           Lux calibration still works but accuracy is lower.\n'
                  '           Use a brighter scene (>1000 lux) for best results.')
        else:
            r.add(f'{name}:gain', Severity.PASS, f'gain={gain:.2f}x')

        # [4] Reasonable lux value
        if lux < 50:
            r.add(f'{name}:lux_level', Severity.WARN,
                  f'lux={lux:.0f} is very low (recommend >=100)',
                  'Low lux increases gain noise in the lux estimate.\n'
                  '           Use a brighter reference image if available.')
        elif lux > 100000:
            r.add(f'{name}:lux_level', Severity.WARN,
                  f'lux={lux:.0f} is very high (>100k)',
                  'Sensor may be overexposing at this illuminance.')

    return r


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------

def run_validation(module, images):
    """
    Run the appropriate validator for the given calibration module name.

    Args:
        module : one of 'ct', 'awb', 'lux', 'ccm', 'blc'
        images : list of CalibCapture / IscImage objects

    Returns:
        ValidationResult — call .passed to check if calibration should proceed.
    """
    grey_modules = {'ct', 'awb'}
    if module in grey_modules:
        result = validate_grey_card(images)
    elif module == 'lux':
        result = validate_lux(images)
        # Also run grey card checks for lux (needs neutral scene)
        grey_r = validate_grey_card(images)
        # Merge neutrality and gain checks only
        for c in grey_r.checks:
            if 'neutrality' in c.name or 'gain' in c.name:
                result.checks.append(c)
    elif module == 'ccm':
        result = validate_colorchecker(images)
    elif module == 'blc':
        result = validate_blc(images)
    else:
        result = ValidationResult(module)
        result.add('module', Severity.PASS, f'No validator for module: {module}')

    result.print_report()
    return result
