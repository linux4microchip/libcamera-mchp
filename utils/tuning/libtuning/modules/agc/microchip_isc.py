# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Microchip Technology Inc.
#
# AGC tuning module for the Microchip ISC.
#
# Emits the IPA's Agc.lux_reference calibration point (reference_Y,
# reference_exposure_us, reference_gain, reference_lux) so the runtime can
# estimate scene lux from the live exposure/gain. reference_Y is a pre-WB
# BT.601 luminance on the ISC 0-511 histogram scale -- the exact quantity
# calculateChannelMean() feeds the estimator.
#
# Source: the located Macbeth chart's Neutral-5 (~18% grey) patch gives
# reference_Y directly, so AGC exposure calibration folds into the single chart
# capture already used for AWB and CCM -- no separate grey-card shot under
# matched ambient. The Neutral-5 patch and a frame-filling 18% grey card yield
# the same reference_Y for one illuminant, keeping target_brightness / y_target
# (calibrated against 18% grey) valid. The capture needs a lux<N> tag and
# exposure_us in the sidecar; absent a located chart the module falls back to
# the authored agc config.

import logging

import numpy as np

from .agc import AGC

logger = logging.getLogger(__name__)


# The 512-bin ISC histogram spans the full 16-bit-scaled pixel range, so
# bin = value16 >> 7. Dividing a 16-bit channel mean by this factor puts it on
# the same 0-511 scale as calculateChannelMean() in common.cpp, matching what a
# grey-card histogram capture reports.
_HIST_16BIT_PER_BIN = 128.0

# Neutral 5 (~19% reflectance) of the located ColorChecker greyscale ramp
# (white=3, N8=7, N6.5=11, N5=15, N3.5=19, black=23 in get_patches() order).
# Its luminance equals what an 18% grey card filling the frame would produce --
# the reference target_brightness and the y_target PWL are anchored to -- so it
# is a drop-in replacement for the grey-card lux reference.
_MACBETH_MIDGREY_PATCH = 15


def _macbeth_reference_y(image):
    """Pre-WB BT.601 luminance of the Neutral-5 patch, on the 0-511 scale.

    Mirrors the ISC runtime luminance (microchip_isc_ipa.cpp): raw pre-WB
    channel means, black level removed, BT.601 weighted -- no white-balance
    gains, because the ISC histogram taps pre-WB and the lux estimator consumes
    that raw luminance directly. Returns None when the patch is clipped or sits
    at/below black level.
    """
    patches = getattr(image, 'patches', None)
    if patches is None:
        return None

    bl = getattr(image, 'blacklevel_16', 0) or 0
    sigbits = getattr(image, 'sigbits', 16)
    white = ((1 << sigbits) - 1) << (16 - sigbits)

    means = {}
    for ci, name in enumerate(('R', 'GR', 'GB', 'B')):
        px = patches[ci][_MACBETH_MIDGREY_PATCH].astype(np.float64)
        # Patch pixels are sorted ascending; the top pixel flags saturation.
        if px[-1] >= white:
            return None
        net = float(px.mean()) - bl
        if net <= 0.0:
            return None
        means[name] = net / _HIST_16BIT_PER_BIN

    mean_g = (means['GR'] + means['GB']) / 2.0
    return 0.299 * means['R'] + 0.587 * mean_g + 0.114 * means['B']


def _macbeth_lux_meta(image):
    """(lux, exposure_us, gain) for a Macbeth capture (filename tag / sidecar).

    Filename lux<N> tag wins over the sidecar (Image.lux defaults to -1);
    exposure and analogue gain come from the JSON sidecar.
    """
    meta = getattr(image, 'metadata', None) or {}
    lux = getattr(image, 'lux', -1)
    lux = lux if lux not in (None, -1) else meta.get('lux')
    return lux, meta.get('exposure_us'), meta.get('analogue_gain')


def macbeth_lux_captures(images):
    """Located Macbeth captures carrying a lux tag and an exposure value.

    locate_macbeth() sets Image.patches only for raw images whose chart was
    found, so a set patches attribute means the neutral ramp is available.
    """
    caps = []
    for img in images:
        if getattr(img, 'patches', None) is None:
            continue
        lux, exp_us, _ = _macbeth_lux_meta(img)
        if lux is None or lux < 1.0 or exp_us is None or exp_us <= 0:
            continue
        caps.append(img)
    return caps


def calibrate_lux_macbeth(images):
    """lux_reference from a Macbeth ColorChecker capture.

    reference_Y is a pre-WB 0-511 luminance read from the chart's Neutral-5
    patch. Selects the highest-lux capture (best SNR, least gain noise). Returns
    a dict with keys reference_Y, reference_exposure_us, reference_gain,
    reference_lux, or None.
    """
    valid = []
    for img in images:
        ref_y = _macbeth_reference_y(img)
        if ref_y is None:
            logger.warning(f'{img.name}: Neutral-5 patch clipped or below '
                           f'black level, skipping for lux')
            continue
        lux, exp_us, gain = _macbeth_lux_meta(img)
        valid.append((img, ref_y, lux, exp_us, gain))

    if not valid:
        logger.error('No valid Macbeth lux captures (need a located chart with '
                     'a lux<N> tag and exposure_us in the sidecar).')
        return None

    img, ref_y, lux, exp_us, gain = max(valid, key=lambda e: e[2])
    if len(valid) > 1:
        logger.info(f'Multiple Macbeth lux captures; using highest lux: {img.name}')
    logger.info(f'Selected {img.name} for lux calibration (lux={lux})')

    if lux < 50:
        logger.warning(f'lux={lux} is low -- recommend >= 100 lux for reliable '
                       f'calibration. High gain at low lux increases noise.')
    if ref_y > 460.0:
        logger.warning(f'reference_Y={ref_y:.1f} is near histogram maximum '
                       f'(511); the Neutral-5 patch may be saturating.')

    gain = float(gain) if gain is not None else 1.0
    logger.info(f'  reference_Y={ref_y:.2f} (0-511, Neutral-5 patch)  '
                f'exposure={exp_us:.1f}us  gain={gain:.4f}  lux={lux}')
    return {
        'reference_Y': round(float(ref_y), 2),
        'reference_exposure_us': round(float(exp_us), 1),
        'reference_gain': round(gain, 4),
        'reference_lux': float(lux),
    }


class AGCMicrochipISC(AGC):
    hr_name = 'AGC (Microchip ISC)'
    out_name = 'Agc'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # Lenient: returning False aborts the whole run, and the agc section is
    # optional. Reject only a structurally malformed section.
    def validate_config(self, config: dict) -> bool:
        agc_cfg = config.get('agc')
        if agc_cfg is not None and not isinstance(agc_cfg, dict):
            logger.error('agc config section must be a mapping')
            return False
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        agc_cfg = config.get('agc') or {}

        mac_caps = macbeth_lux_captures(images)
        ref = calibrate_lux_macbeth(mac_caps) if mac_caps else None

        if not ref:
            logger.info('AGC: no usable Macbeth lux captures; using config values')
            return config.get('agc', None)

        logger.info('AGC: lux_reference from Macbeth Neutral-5 patch')
        out = dict(agc_cfg)
        out['lux_reference'] = ref
        return out
