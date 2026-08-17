# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Microchip Technology Inc.
#
# Lens-shading calibration for the Microchip ISC still path.
#
# The ISC has no hardware lens-shading block: every ISC gain stage (WB, CC,
# GAM, CBC) applies frame-global coefficients, and there is no register that
# accepts a spatially-varying gain map. Vignetting therefore cannot be
# corrected in hardware, and it is not affordable per-pixel at video rate on
# the single Cortex-A7. It can, however, be recovered on a single still as a
# CPU post-process.
#
# This module measures the lens falloff from a uniformly-lit flat-field capture
# (filename tagged 'alsc') and emits a per-channel gain map under
# LensShadingCorrection. The map is consumed by the mchpcam still-path software
# corrector, NOT by the IPA. The falloff is measured on the raw Bayer channels
# (the physically correct place), while the corrector applies it to the final
# demosaiced image, so luminance vignetting is recovered well and residual
# corner colour shift only partially.

import logging

import numpy as np

import libtuning as lt
from .lsc import LSC

logger = logging.getLogger(__name__)


class LSCMicrochipISC(LSC):
    hr_name = 'LSC (Microchip ISC)'
    out_name = 'LensShadingCorrection'

    # Lenient: the lsc config section is optional and calibration falls back to
    # emitting nothing when no flat-field capture is supplied. Reject only a
    # structurally malformed section (returning False aborts the whole run).
    def validate_config(self, config: dict) -> bool:
        lsc_cfg = config.get('lsc')
        if lsc_cfg is not None and not isinstance(lsc_cfg, dict):
            logger.error('lsc config section must be a mapping')
            return False
        return True

    # The base _get_grid is reused unchanged: it grids the whole Bayer plane.
    # Its img_w/2, img_h/2 is not a radial-symmetry quadrant assumption; each
    # image.channels[] plane is already half-width/half-height (the Bayer
    # subsample), so w/2, h/2 are the plane dimensions and the grid spans the
    # full frame. The still-path corrector likewise makes no symmetry
    # assumption and interpolates the grid across the whole image, so no
    # override is needed.

    # @brief Per-channel gain map for one flat-field image.
    #
    # Each channel is normalised to its own brightest sector (gain 1.0 there,
    # >1.0 where the lens is darker), so the map corrects spatial falloff only
    # and leaves the global channel balance to WB. The two green planes are
    # averaged into a single G map for the demosaiced RGB corrector.
    def _channel_maps(self, image):
        green = (image.channels[lt.Color.GR].astype(np.float64) +
                 image.channels[lt.Color.GB].astype(np.float64)) / 2.0
        r, _ = self._lsc_single_channel(image.channels[lt.Color.R], image)
        g, _ = self._lsc_single_channel(green, image)
        b, _ = self._lsc_single_channel(image.channels[lt.Color.B], image)
        return r, g, b

    # @brief Per-sector green-imbalance (GR/GB) ratio for one flat-field image.
    #
    # A perfectly balanced sensor reads GR == GB on a neutral flat; a real one
    # drifts (channel-dependent crosstalk, microlens asymmetry), which appears
    # after demosaic as a green maze. This measures the ratio per sector so the
    # calibration can report where and how far the greens diverge. Black level
    # is removed first so the ratio is of light, not pedestal.
    def _geq_ratio(self, image):
        bl = image.blacklevel_16
        gr = np.maximum(self._get_grid(
            image.channels[lt.Color.GR].astype(np.float64),
            image.w, image.h) - bl, 0.1)
        gb = np.maximum(self._get_grid(
            image.channels[lt.Color.GB].astype(np.float64),
            image.w, image.h) - bl, 0.1)
        return gr / gb

    def _average_maps(self, maps, max_gain):
        """Average per-channel sector maps and clamp; return (gains, peak)."""
        gains = {}
        peak = 0.0
        for ch in ('r', 'g', 'b'):
            avg = np.mean(np.array(maps[ch]), axis=0)
            peak = max(peak, float(avg.max()))
            avg = np.clip(avg, 1.0, max_gain)
            gains[ch] = [round(float(v), 4) for v in avg.flatten()]
        return gains, peak

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        lsc_cfg = config.get('lsc') or {}
        # Corner gains blow up when the flat field is near black at the edges
        # (amplifying noise). Cap them and warn rather than emit a map that
        # would over-brighten noisy corners.
        max_gain = float(lsc_cfg.get('max_gain', 4.0))

        # Group flats by their illuminant CT. A single group reproduces the
        # legacy averaged map exactly; multiple groups emit one map per CT so
        # the still-path corrector can pick the map matching the scene's colour
        # temperature (lens colour-shading is CT-dependent).
        by_ct = {}
        for image in self._enumerate_lsc_images(images):
            if getattr(image, 'channels', None) is None:
                logger.warning(f'{image.name}: flat-field capture has no raw '
                               f'Bayer data (needs a DNG/TIFF), skipping')
                continue
            ct = int(round(float(getattr(image, 'color', 0) or 0)))
            by_ct.setdefault(ct, []).append(image)

        if not by_ct:
            logger.info('LSC: no flat-field (alsc) captures; skipping lens-shading '
                        'map. The still-path corrector runs only when a map is '
                        'present.')
            return None

        nx, ny = self.sector_shape
        out = {
            'grid_width': nx,
            'grid_height': ny,
        }

        per_ct = []
        geq = []
        peak = 0.0
        for ct in sorted(by_ct):
            group = by_ct[ct]
            maps = {'r': [], 'g': [], 'b': []}
            ratios = []
            for image in group:
                r, g, b = self._channel_maps(image)
                maps['r'].append(r)
                maps['g'].append(g)
                maps['b'].append(b)
                ratios.append(self._geq_ratio(image))
            gains, gpeak = self._average_maps(maps, max_gain)
            peak = max(peak, gpeak)
            per_ct.append({'ct': ct, 'gains': gains})

            ratio = np.mean(np.array(ratios), axis=0)
            dev = np.abs(ratio - 1.0)
            geq.append({'ct': ct, 'n': len(group),
                        'max_dev': round(float(dev.max()), 5),
                        'mean_dev': round(float(dev.mean()), 5)})

        # Byte-identical single-CT path: emit the flat top-level 'gains' the
        # runtime already parses. Only a genuine multi-CT calibration emits the
        # 'per_ct' shape (a runtime reader change gates behind its presence).
        if len(per_ct) == 1:
            out['gains'] = per_ct[0]['gains']
        else:
            out['per_ct'] = per_ct

        # Green-imbalance measurement (diagnostic only): underscore-prefixed so
        # it is stripped from the emitted YAML. A runtime green-equalisation
        # apply is a separate step -- the ISC demosaics in hardware, so the
        # post-processor sees no Bayer greens to equalise; this quantifies the
        # imbalance and lets health flag it.
        out['_geq'] = geq

        if peak > max_gain:
            logger.warning(f'LSC: peak corner gain {peak:.2f} exceeds max_gain '
                           f'{max_gain:.2f} and was clamped. Corners of the flat '
                           f'field are near black; brighten the flat field or '
                           f'raise lsc.max_gain.')
        worst_geq = max((e['max_dev'] for e in geq), default=0.0)
        n_flats = sum(len(v) for v in by_ct.values())
        logger.info(f'LSC: lens-shading map(s) from {n_flats} flat-field '
                    f'capture(s) across {len(by_ct)} CT(s), {nx}x{ny} sectors, '
                    f'peak gain {min(peak, max_gain):.2f}, worst green imbalance '
                    f'{worst_geq * 100:.1f}% (still-path software corrector; '
                    f'not an IPA algorithm)')

        if lt.Debug.Plot in self.debug:
            plot_gains = (out['gains'] if 'gains' in out
                          else per_ct[0]['gains'])
            self._plot_map(plot_gains, nx, ny)

        return out

    # @brief Optional heat-map of the luminance gain, guarded so calibration
    #        never fails on a host without matplotlib.
    @staticmethod
    def _plot_map(gains, nx, ny):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            logger.info(f'LSC plot skipped (matplotlib unavailable): {e}')
            return
        try:
            g = np.array(gains['g']).reshape(ny, nx)
            fig, ax = plt.subplots()
            im = ax.imshow(g, origin='upper')
            fig.colorbar(im, ax=ax, label='G gain')
            ax.set_title('LSC luminance gain map')
            fig.savefig('lsc_gain_map.png')
            plt.close(fig)
            logger.info('LSC gain map plotted to lsc_gain_map.png')
        except Exception as e:
            logger.warning(f'LSC plot failed: {e}')
