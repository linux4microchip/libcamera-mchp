# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Noise-profile calibration for the Microchip ISC still path.
#
# The still-path denoiser (isc_postproc.cpp) sizes its bilateral range sigma from
# a sensor noise model: variance = read_noise_var + shot_slope * signal, with the
# whole noise amplitude scaling by analogue_gain ** gain_exponent. Those three
# scalars are hardcoded defaults in the C++; this module measures them from a
# flat-field gain sweep so the denoiser smooths dark/high-gain pixels by the real
# noise the sensor produces rather than a guess.
#
# Method (per-gain photon transfer, temporal): mchp_capture.sh --frames shoots N
# aligned repeats of a static flat at each analogue gain. Across those repeats the
# per-pixel temporal variance IS the noise at that pixel's signal level -- fixed
# spatial shading and fixed-pattern offsets cancel, unlike a single-frame spatial
# variance. Binning pixels by signal and fitting variance-vs-signal gives the read
# (intercept) and shot (slope) terms at each gain; regressing the read term against
# gain in log-log recovers the gain exponent. The read/shot terms are then
# normalised to unity gain so the emitted triple drops straight into the runtime's
# three constants.
#
# The measurement is done in the 8-bit domain the denoiser operates in (the raw
# 16-bit Bayer planes divided by 256), black level removed, so read_noise_var and
# shot_slope come out directly in runtime units. NR is a still-path CPU
# post-process, not an IPA algorithm, so the profile is emitted to a sibling
# <model>_noise.yaml (like the lens-shading map) and scored as a health advisory.

import logging

import numpy as np

from libtuning.modules.module import Module

logger = logging.getLogger(__name__)

# Runtime defaults (isc_postproc.cpp kReadNoiseVar / kShotSlope / kGainExp). Used
# only to report how far a measured profile moved from the built-in guess.
_DEFAULT_READ_VAR = 4.0
_DEFAULT_SHOT_SLOPE = 0.5
_DEFAULT_GAIN_EXP = 0.8


def _image_gain(image):
    """Applied analogue gain of a capture (1.0 when the sidecar omits it)."""
    g = getattr(image, 'againQ8_norm', None)
    if g is None:
        g = float(getattr(image, 'metadata', {}).get('analogue_gain', 1.0))
    return float(g)


def _fit_variance_vs_signal(signal, tvar, n_bins=12, min_per_bin=64):
    """Fit temporal variance = read_var + shot_slope * signal.

    signal/tvar are flat per-pixel arrays (8-bit domain, black removed). Pixels
    are grouped into equal-count signal bins and their mean variance fitted
    against mean signal -- averaging many pixels per bin tames the per-pixel
    variance-of-variance so the line is stable. Returns (read_var, shot_slope,
    r2, n_px) or None when the signal range or pixel count is too thin to fit.
    """
    signal = np.asarray(signal, dtype=np.float64)
    tvar = np.asarray(tvar, dtype=np.float64)
    if signal.size < n_bins * min_per_bin:
        n_bins = max(2, signal.size // min_per_bin)
    if signal.size < 2 * min_per_bin:
        return None

    order = np.argsort(signal)
    signal = signal[order]
    tvar = tvar[order]
    edges = np.linspace(0, signal.size, n_bins + 1, dtype=int)

    xs, ys = [], []
    for i in range(n_bins):
        lo, hi = edges[i], edges[i + 1]
        if hi - lo < min_per_bin:
            continue
        xs.append(float(signal[lo:hi].mean()))
        ys.append(float(tvar[lo:hi].mean()))
    if len(xs) < 2 or (max(xs) - min(xs)) < 1.0:
        return None

    xs = np.array(xs)
    ys = np.array(ys)
    shot_slope, read_var = np.polyfit(xs, ys, 1)
    pred = shot_slope * xs + read_var
    ss_res = float(np.sum((ys - pred) ** 2))
    ss_tot = float(np.sum((ys - ys.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
    # A negative intercept/slope is an artefact of a short signal range or a
    # noisy fit, not a physical value; clamp to zero so the runtime never sizes
    # a negative sigma from it.
    return max(read_var, 0.0), max(shot_slope, 0.0), r2, int(signal.size)


def _collect_points_per_gain(images):
    """Per-gain (signal, temporal-variance) point clouds from flat repeats.

    Flats are grouped by (CT, gain): only aligned repeats of one static scene
    give a meaningful temporal variance, so each group needs >=2 frames. The
    per-pixel points from every group at one gain (all four Bayer channels, all
    CTs) are pooled -- they are all photosites of the same sensor, and pooling
    widens the signal range the line is fitted over. Returns {gain: (signal[],
    tvar[])}.
    """
    groups = {}
    for image in images:
        if not getattr(image, 'lsc_only', False):
            continue
        if getattr(image, 'channels', None) is None:
            continue
        ct = int(round(float(getattr(image, 'color', 0) or 0)))
        gain = round(_image_gain(image), 3)
        groups.setdefault((ct, gain), []).append(image)

    per_gain = {}
    for (ct, gain), group in groups.items():
        if len(group) < 2:
            continue
        bl8 = float(getattr(group[0], 'blacklevel_16', 0) or 0) / 256.0
        sig_parts, var_parts = [], []
        for ch in range(len(group[0].channels)):
            stack = np.stack([im.channels[ch].astype(np.float64) / 256.0
                              for im in group])
            signal = stack.mean(axis=0) - bl8
            tvar = stack.var(axis=0, ddof=1)
            mask = signal > 1.0            # keep pixels that carry light
            sig_parts.append(signal[mask].ravel())
            var_parts.append(tvar[mask].ravel())
        if not sig_parts:
            continue
        s = np.concatenate(sig_parts)
        v = np.concatenate(var_parts)
        if gain in per_gain:
            ps, pv = per_gain[gain]
            per_gain[gain] = (np.concatenate([ps, s]),
                              np.concatenate([pv, v]))
        else:
            per_gain[gain] = (s, v)
    return per_gain


def calibrate_noise_profile(images):
    """Read-noise variance, shot slope and gain exponent from a flat gain sweep.

    Returns (profile, per_gain) where:
      profile  : {'read_noise_var', 'shot_slope', 'gain_exponent'} at unity gain
                 (the runtime's three constants), plus 'gain_exp_r2', 'ref_gain',
                 or None when no gain fitted.
      per_gain : sorted list of {'gain', 'read_noise_var', 'shot_slope', 'r2',
                 'n_px'} -- the per-gain fits, for the diagnostic/health path.
    Returns (None, []) when no usable flat-field repeats are present.
    """
    per_gain_points = _collect_points_per_gain(images)
    if not per_gain_points:
        logger.info('Noise: no flat-field repeats (need >=2 frames per gain at '
                    'one CT, tagged alsc); skipping noise profile.')
        return None, []

    per_gain = []
    for gain in sorted(per_gain_points):
        signal, tvar = per_gain_points[gain]
        fit = _fit_variance_vs_signal(signal, tvar)
        if fit is None:
            logger.warning(f'Noise: gain {gain:.2f} flat has too thin a signal '
                           f'range to fit variance-vs-signal; skipping this gain.')
            continue
        read_var, shot_slope, r2, n_px = fit
        per_gain.append({'gain': round(gain, 3),
                         'read_noise_var': round(read_var, 5),
                         'shot_slope': round(shot_slope, 6),
                         'r2': round(r2, 4), 'n_px': n_px})
        logger.info(f'  gain {gain:.2f}: read_var={read_var:.3f} '
                    f'shot_slope={shot_slope:.4f} R2={r2:.3f} (n={n_px} px)')

    if not per_gain:
        return None, []

    gains = np.array([e['gain'] for e in per_gain])
    read_vars = np.array([e['read_noise_var'] for e in per_gain])
    shot_slopes = np.array([e['shot_slope'] for e in per_gain])

    # Reference operating point: the fitted gain closest to unity. Its read/shot
    # terms are extrapolated to exactly gain=1 so the emitted triple maps 1:1
    # onto the runtime constants (defined at unity gain).
    ref_i = int(np.argmin(np.abs(gains - 1.0)))
    ref_gain = float(gains[ref_i])

    # Gain exponent: the runtime scales the whole noise amplitude by
    # gain**gain_exponent, i.e. variance by gain**(2*gain_exponent). The read
    # (intercept) term is the cleanest gain-scaling probe, so regress
    # log(read_var) on log(gain): slope = 2 * gain_exponent. Needs >=2 gains and
    # a positive read term at each.
    gain_exp = _DEFAULT_GAIN_EXP
    gain_exp_r2 = None
    usable = (gains > 0) & (read_vars > 1e-6)
    if usable.sum() >= 2:
        lg = np.log(gains[usable])
        lv = np.log(read_vars[usable])
        slope, intercept = np.polyfit(lg, lv, 1)
        gain_exp = float(slope / 2.0)
        pred = slope * lg + intercept
        ss_res = float(np.sum((lv - pred) ** 2))
        ss_tot = float(np.sum((lv - lv.mean()) ** 2))
        gain_exp_r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
        # Clamp to a physically sane band: read noise grows with gain (>=0) and
        # no faster than the signal it rides on (<=1 in amplitude).
        gain_exp = float(np.clip(gain_exp, 0.0, 1.0))
    else:
        logger.info('Noise: single usable gain; keeping the runtime default '
                    f'gain exponent {_DEFAULT_GAIN_EXP} (sweep >=2 gains to '
                    f'measure it).')

    # Normalise read/shot to unity gain: divide out gain**(2*gain_exp). Averaging
    # the normalised terms across gains uses every operating point rather than a
    # single reference frame.
    scale = gains ** (2.0 * gain_exp)
    read_var_1 = float(np.mean(read_vars / scale))
    shot_slope_1 = float(np.mean(shot_slopes / scale))

    profile = {
        'read_noise_var': round(read_var_1, 5),
        'shot_slope': round(shot_slope_1, 6),
        'gain_exponent': round(gain_exp, 4),
        'gain_exp_r2': round(gain_exp_r2, 4) if gain_exp_r2 is not None else None,
        'ref_gain': round(ref_gain, 3),
    }
    return profile, per_gain


def check_noise_profile(profile, per_gain):
    """Log how trustworthy the fitted noise profile is. Never raises.

    Flags a loose variance-vs-signal fit (low per-gain R2) or a poorly
    determined gain exponent (few gains / low log-log R2), so the operator knows
    whether to trust the emitted profile or widen the gain sweep.
    """
    if not profile or not per_gain:
        return

    worst_r2 = min(e['r2'] for e in per_gain)
    if worst_r2 < 0.9:
        logger.warning(f'Noise: weakest per-gain variance-vs-signal fit R2='
                       f'{worst_r2:.2f} (<0.90). The flat field may be too '
                       f'uniform (narrow signal range) or the repeats too few; '
                       f'the shot slope is then poorly determined.')

    exp_r2 = profile.get('gain_exp_r2')
    if len(per_gain) < 2:
        logger.info('Noise: gain exponent not measured (single gain); the '
                    'runtime default was kept.')
    elif exp_r2 is not None and exp_r2 < 0.8:
        logger.warning(f'Noise: gain-exponent log-log fit R2={exp_r2:.2f} '
                       f'(<0.80) over {len(per_gain)} gains -- read noise does '
                       f'not scale cleanly with gain; treat the exponent as '
                       f'approximate.')

    logger.info(f'Noise: profile read_noise_var={profile["read_noise_var"]:.3f} '
                f'shot_slope={profile["shot_slope"]:.4f} '
                f'gain_exponent={profile["gain_exponent"]:.3f} '
                f'(runtime defaults {_DEFAULT_READ_VAR}/{_DEFAULT_SHOT_SLOPE}/'
                f'{_DEFAULT_GAIN_EXP})')


class NoiseMicrochipISC(Module):
    hr_name = 'Noise (Microchip ISC)'
    out_name = 'NoiseProfile'
    type = 'noise'

    def __init__(self, **kwargs):
        self.debug = kwargs.get('debug', [])

    # Lenient like the other optional stages: a malformed section aborts the
    # run, but an absent one just means "no authored noise config", and the
    # module emits nothing when there is no flat-field sweep to measure.
    def validate_config(self, config: dict) -> bool:
        noise_cfg = config.get('noise')
        if noise_cfg is not None and not isinstance(noise_cfg, dict):
            logger.error('noise config section must be a mapping')
            return False
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        profile, per_gain = calibrate_noise_profile(images)
        if not profile:
            logger.info('Noise: no noise profile emitted; the still-path '
                        'denoiser uses its built-in defaults.')
            return None

        check_noise_profile(profile, per_gain)

        # Emit only the three runtime constants; the per-gain fits and fit
        # quality ride along as underscore-prefixed diagnostics (stripped from
        # the YAML, read by the health summary).
        out = {
            'read_noise_var': profile['read_noise_var'],
            'shot_slope': profile['shot_slope'],
            'gain_exponent': profile['gain_exponent'],
            '_per_gain': per_gain,
            '_gain_exp_r2': profile['gain_exp_r2'],
            '_ref_gain': profile['ref_gain'],
        }
        logger.info(f'Noise: profile from {len(per_gain)} gain(s) '
                    f'(still-path denoiser parameters; not an IPA algorithm)')
        return out
