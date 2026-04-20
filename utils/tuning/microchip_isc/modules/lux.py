# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# Lux estimator calibration module.
# Derives lux_reference from images captured under known illuminance.
#
# Methodology (mirrors RPi lux.cpp, BSD-2-Clause):
#   lux = (refExposure / curExposure) * (refGain / curGain)
#         * (curY / refY) * refLux
#
#   The reference point (refY, refExposure, refGain, refLux) is calibrated
#   from one image captured under a known lux level, with the AGC converged.
#
#   reference_Y is the BT.601-weighted luminance computed from the ISC
#   histogram channel means. The histogram is 512 bins (0-511 range):
#     - Bin 0   = pixel value 0   (darkest)
#     - Bin 511 = pixel value 511 (brightest, ~10-bit >> 1)
#   Channel means from calculateChannelMean() in common.cpp are in 0-511.
#   So reference_Y must be in the SAME 0-511 range.
#
#   CRITICAL: Do NOT scale to 65535 or 1023. The IPA computes currentY from
#   the histogram directly in 0-511 units. Using a different scale for the
#   reference would corrupt the lux estimate by a constant factor.
#
# Calibration advice:
#   - Use the image with the highest measured lux in your set (best SNR,
#     least gain noise). If you only have 100-400 lux, use the brightest.
#   - "Closest to 1000 lux" is ideal but not mandatory.
#   - The lux meter should face the camera (incident light measurement),
#     not face the scene (reflected light).

import logging

logger = logging.getLogger(__name__)


def calibrate_lux(images):
    """
    Calibrate lux_reference from images with known lux values.

    Each image must have:
      img.lux       : measured scene illuminance in lux (from filename lux<N>)
      img.channels  : [R, GR, GB, B] 2-D arrays in ISC histogram units (0-511)
      img.exposure  : exposure time in microseconds
      img.again     : analogue gain (linear)

    reference_Y is computed in ISC histogram units (0-511), matching what
    calculateChannelMean() returns in common.cpp. Do NOT normalise to 16-bit
    or 10-bit range -- the runtime formula uses the same 0-511 scale.

    Selects the image with the highest lux level (best SNR, least gain noise).
    If multiple images have the same lux, uses the first one.

    Returns dict with keys: reference_Y, reference_exposure_us,
                            reference_gain, reference_lux
    Returns None if no valid calibration images are found.
    """
    valid = []
    for img in images:
        # lux must be a positive measured value (>= 1.0 lux minimum)
        if img.lux is None or img.lux < 1.0:
            continue
        if img.hist is None:
            logger.warning(f'{img.name}: no histogram means in sidecar, skipping')
            continue
        if img.exposure_us is None or img.exposure_us <= 0:
            logger.warning(f'{img.name}: no exposure_us — '
                           f'lux calibration requires exposure time. '
                           f'Use calib.sh to capture images (mchpcam-still --metadata '
                           f'embeds ExposureTime in the JSON output).')
            continue
        valid.append(img)

    if not valid:
        logger.error('No valid lux calibration images found. '
                     'Need lux<N> tag (N >= 1), exposure in JSON sidecar, '
                     'and channel data.')
        return None

    # Select image with highest lux (best SNR, least gain noise)
    max_lux = max(i.lux for i in valid)
    candidates = [i for i in valid if i.lux == max_lux]
    img = candidates[0]
    if len(candidates) > 1:
        logger.info(f'Multiple images at lux={max_lux}, using first: {img.name}')
    logger.info(f'Selected {img.name} for lux calibration (lux={img.lux})')

    if img.lux < 50:
        logger.warning(f'lux={img.lux} is low — recommend >= 100 lux for reliable '
                       f'calibration. High gain at low lux increases noise.')

    # Read histogram means from the JSON sidecar (pre-WB, pre-DPC tap)
    r  = img.hist['R']
    gr = img.hist['GR']
    gb = img.hist['GB']
    b  = img.hist['B']

    # BT.601 luma from pre-WB histogram means.
    # Matches common.cpp: Y = 0.299*R + 0.587*((GR+GB)/2) + 0.114*B
    mean_g = (gr + gb) / 2.0
    ref_y = 0.299 * r + 0.587 * mean_g + 0.114 * b

    # Near-saturation check
    if ref_y > 460.0:
        logger.warning(f'reference_Y={ref_y:.1f} is near histogram maximum (511). '
                       f'The reference image may be saturating. '
                       f'Use a lower-lux reference if available.')
    if ref_y < 5.0:
        logger.warning(f'reference_Y={ref_y:.2f} is very low. '
                       f'Check that AGC was converged and the scene was adequately lit.')

    # Green channel consistency check
    if gr > 0 and abs(gr - gb) / max(gr, gb) > 0.1:
        logger.warning(f'GR={gr:.1f} and GB={gb:.1f} differ by more than 10%%. '
                       f'Scene may not be neutral grey, or BL subtraction is off. '
                       f'Lux reference_Y accuracy may be reduced.')

    logger.info(f'  reference_Y={ref_y:.2f} (0-511 histogram units)  '
                f'exposure={img.exposure_us:.1f}us  '
                f'gain={img.gain:.4f}  lux={img.lux}')

    return {
        'reference_Y': round(float(ref_y), 2),
        'reference_exposure_us': round(float(img.exposure_us), 1),
        'reference_gain': round(float(img.gain), 4),
        'reference_lux': float(img.lux),
    }
