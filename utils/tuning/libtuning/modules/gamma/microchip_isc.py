# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Gamma tone-curve tuning module for the Microchip ISC.
#
# Fits the base gamma exponent from the located Macbeth neutral ramp instead of
# authoring it by hand. The runtime (gamma.cpp computeLut()) always synthesises
# the 64-entry tone LUT analytically from a handful of scalars, so the tuning
# task is to recover those scalars, not to tabulate a curve.
#
# What the ramp can and cannot constrain:
#   gamma   the power-law exponent -- well constrained. The six neutral patches
#           (located indices 3,7,11,15,19,23 = white..black) sample the curve
#           across its mid and upper range, so a log-log regression of measured
#           input vs the reference sRGB encoding recovers the exponent directly.
#   toe_input / tone_rolloff_start / tone_rolloff_slope -- NOT recoverable from a
#           standard neutral ramp. Normalised to the brightest neutral the ramp
#           lives in [~0.03, 1.0]: the linear toe breakpoint (~0.0313) sits below
#           the darkest usable patch and the highlight shoulder (>=0.5) is sampled
#           by the single white point. Both are scene/hardware choices, not ramp
#           measurements, so they are held at their authored config values -- the
#           tool never fabricates a fit the data does not support.
#
# The measured OECF is: x = black-subtracted green sensor mean of each neutral,
# normalised to the brightest usable neutral; y = that neutral's reference sRGB
# green (the desired display encoding), normalised the same way. Fitting gamma so
# the analytic curve maps x->y makes the recovered exponent absorb any real
# sensor non-linearity rather than assuming a perfectly linear response.
#
# No runtime or YAML change: the emitted 'gamma' scalar is already parsed by
# gamma.cpp init(). When no Macbeth chart is located the module falls back to the
# authored gamma config, so a full regeneration still reproduces the tuning file.

import logging

import numpy as np

from libtuning.modules.module import Module

logger = logging.getLogger(__name__)


# Located ColorChecker greyscale ramp, white -> black (get_patches() column-
# stacked order). Matches _MACBETH_GREY_PATCHES in the AWB module.
_GREY_PATCHES = (3, 7, 11, 15, 19, 23)

# Reference green sRGB (8-bit, D65) of those neutrals, white -> black, taken from
# the standard ColorChecker chart order reordered [i::6] to the located order
# (the same interleave the AWB/CCM solves use). These are the desired display
# encoding of each neutral: the tone curve should map the measured linear input
# of a patch onto its reference-encoded value.
_GREY_REF_SRGB8 = np.array([243.0, 202.0, 163.0, 121.0, 84.0, 49.0])

# Runtime clamp ranges (gamma.cpp init()). The emitted scalar must land inside
# the same range or tool and runtime disagree.
_GAMMA_MIN, _GAMMA_MAX = 0.5, 5.0

# gamma.cpp LUT geometry: 64 entries over the 10-bit input range.
_LUT_ENTRIES = 64
_INPUT_MAX = 1023.0
_LUT_STEP = 16.0


def _srgb8_to_linear(x8):
    """Standard sRGB EOTF: 8-bit encoded sRGB -> linear 0..1."""
    x = x8 / 255.0
    return np.where(x < 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def _neutral_ramp_oecf(image):
    """Measured OECF of one located chart's neutral ramp.

    Returns (x, y, weight) arrays over the usable neutrals, normalised so the
    brightest usable patch is (1, 1), or None when fewer than two neutrals are
    usable. x is the black-subtracted green sensor mean (linear); y is the
    reference sRGB green of the same patch (the desired encoded output); weight
    is the reference luminance, so bright high-SNR patches dominate the fit and
    noise-limited dark patches contribute little.
    """
    patches = getattr(image, 'patches', None)
    if patches is None:
        return None

    bl = getattr(image, 'blacklevel_16', 0) or 0
    sigbits = getattr(image, 'sigbits', 16)
    white = ((1 << sigbits) - 1) << (16 - sigbits)

    xs, ys = [], []
    for patch, ref8 in zip(_GREY_PATCHES, _GREY_REF_SRGB8):
        gr = patches[1][patch].astype(np.float64)
        gb = patches[2][patch].astype(np.float64)
        # Sorted ascending: a top pixel at white flags a clipped patch whose
        # mean is biased low -- drop it rather than fit to it.
        if gr[-1] >= white or gb[-1] >= white:
            continue
        g = (gr.mean() + gb.mean()) / 2.0 - bl
        if g <= 0.0:
            continue
        xs.append(g)
        ys.append(ref8)

    if len(xs) < 2:
        return None

    x = np.array(xs)
    y = np.array(ys)
    x /= x.max()
    y /= y.max()
    # Reference luminance as the fit weight (linear reflectance of the patch).
    weight = _srgb8_to_linear(y * 255.0)
    return x, y, weight


def _fit_gamma(x, y, weight):
    """Recover the power-law exponent from an OECF ramp by log-log regression.

    y = x**(1/gamma) is linear in log space with slope 1/gamma. The anchor point
    (x=1) carries no slope information (log 1 = 0) and is skipped. Returns the
    clamped gamma, or None when fewer than two off-anchor points survive.
    """
    mask = (x > 1e-4) & (x < 1.0 - 1e-6) & (y > 1e-4)
    lx = np.log(x[mask])
    ly = np.log(y[mask])
    w = weight[mask]
    if lx.size < 2:
        return None

    # Weighted least-squares slope of ly = slope * lx (line through the origin in
    # log space, since (log 1, log 1) = (0, 0) anchors both ends).
    slope = float(np.sum(w * lx * ly) / np.sum(w * lx * lx))
    if slope <= 1e-6:
        return None
    return float(np.clip(1.0 / slope, _GAMMA_MIN, _GAMMA_MAX))


def compute_lut(gamma, toe_input, rolloff_start, rolloff_slope, bipartite):
    """Reconstruct the runtime 64-entry tone LUT (mirror of gamma.cpp).

    Pure power-law with an optional C1 highlight shoulder above rolloff_start and
    an optional linear toe below toe_input (bipartite mode). The histogram-CDF
    blend is omitted: it is a per-frame runtime adaptation, not a tuning scalar.
    Returns a length-64 int array of 10-bit output codes.
    """
    inv = 1.0 / gamma
    y_at_toe = toe_input ** inv
    toe_slope = (y_at_toe / toe_input) if toe_input > 0.0 else 1.0

    lut = np.zeros(_LUT_ENTRIES, dtype=np.int64)
    for i in range(_LUT_ENTRIES):
        x = (i * _LUT_STEP) / _INPUT_MAX

        if rolloff_start < 1.0 and x > rolloff_start:
            y_at_shoulder = rolloff_start ** inv
            slope_at_shoulder = inv * rolloff_start ** (inv - 1.0)
            target_slope = slope_at_shoulder * rolloff_slope
            y_shoulder = y_at_shoulder + target_slope * (x - rolloff_start)
            x = min(max(y_shoulder, 0.0), 1.0) ** gamma

        if x == 0.0:
            y = 0.0
        elif bipartite and x < toe_input:
            y = toe_slope * x
        else:
            y = x ** inv

        lut[i] = int(min(max(round(y * _INPUT_MAX), 0.0), _INPUT_MAX))
    return lut


def _oecf_rms(lut, x, y):
    """RMS error between the reconstructed LUT and the measured OECF points.

    Each normalised input x is looked up in the coarse 64-entry LUT (nearest
    entry, exactly as the input range maps to entries in hardware) and the
    normalised output compared to the reference encoding y.
    """
    idx = np.clip(np.round(x * _INPUT_MAX / _LUT_STEP).astype(int),
                  0, _LUT_ENTRIES - 1)
    out = lut[idx].astype(np.float64) / lut.max()
    return float(np.sqrt(np.mean((out - y) ** 2)))


def _located_charts(images):
    return [img for img in images if getattr(img, 'patches', None) is not None]


class GammaMicrochipISC(Module):
    hr_name = 'Gamma (Microchip ISC)'
    out_name = 'Gamma'
    type = 'gamma'

    # Lenient: the gamma section is authored, so reject only a malformed mapping
    # rather than aborting the whole run.
    def validate_config(self, config: dict) -> bool:
        gamma_cfg = config.get('gamma')
        if gamma_cfg is not None and not isinstance(gamma_cfg, dict):
            logger.error('gamma config section must be a mapping')
            return False
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        gamma_cfg = config.get('gamma') or {}

        # Pool the neutral ramps of every located chart: gamma is illuminant-
        # independent, so more points tighten the exponent.
        xs, ys, ws = [], [], []
        used = 0
        for img in _located_charts(images):
            ramp = _neutral_ramp_oecf(img)
            if ramp is None:
                continue
            x, y, w = ramp
            xs.append(x)
            ys.append(y)
            ws.append(w)
            used += 1

        if not xs:
            logger.info('Gamma: no usable Macbeth neutral ramp; using config values')
            return config.get('gamma', None)

        x = np.concatenate(xs)
        y = np.concatenate(ys)
        w = np.concatenate(ws)
        fitted = _fit_gamma(x, y, w)
        if fitted is None:
            logger.info('Gamma: neutral ramp too sparse to fit; using config values')
            return config.get('gamma', None)

        # Held (not fitted) shoulder/toe scalars: the ramp does not sample them.
        toe_input = float(gamma_cfg.get('toe_input', 0.0313))
        rolloff_start = float(gamma_cfg.get('tone_rolloff_start', 0.85))
        rolloff_slope = float(gamma_cfg.get('tone_rolloff_slope', 0.35))

        # Reconstruct the runtime LUT with the fitted exponent and check it tracks
        # the measured ramp. Bipartite toe matches the SAMA7G5 tuning target; it
        # only affects inputs below the darkest usable neutral, so it does not
        # move the RMS, but reconstructing exactly what the runtime builds keeps
        # the diagnostic honest.
        lut = compute_lut(fitted, toe_input, rolloff_start, rolloff_slope,
                          bipartite=True)
        rms = _oecf_rms(lut, x, y)

        cfg_gamma = float(gamma_cfg.get('gamma', 2.2))
        logger.info(f'Gamma: fitted gamma={fitted:.3f} from {used} chart(s), '
                    f'{x.size} ramp points (config {cfg_gamma:.3f}); '
                    f'OECF RMS={rms:.4f}')
        logger.info('Gamma: toe_input/tone_rolloff held at authored values '
                    '(neutral ramp does not sample the toe or shoulder)')

        out = dict(gamma_cfg)
        out['gamma'] = round(fitted, 4)
        # Underscore-prefixed diagnostics for the health score; stripped from the
        # emitted YAML by MicrochipYamlOutput.
        out['_oecf_rms'] = round(rms, 4)
        out['_oecf_points'] = int(x.size)
        out['_gamma_fit'] = round(fitted, 4)
        out['_gamma_config'] = round(cfg_gamma, 4)
        return out
