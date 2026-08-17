# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Microchip Technology Inc.
#
# CCM tuning module for the Microchip ISC.
#
# Reuses libtuning's least-squares CCM solve (the base CCM class, a genuine
# Macbeth-chart calibration) and emits the Microchip IPA layout:
#   algorithms[].Ccm.matrices: {name: {ct, ccm[9]}}
# where name is the illuminant bucket the IPA interpolates between.

import logging

from .ccm import CCM

logger = logging.getLogger(__name__)


class CCMMicrochipISC(CCM):
    hr_name = 'CCM (Microchip ISC)'
    out_name = 'Ccm'

    # Name the CT buckets the IPA interpolates between. These names and CTs
    # must match the authored 'matrices' in microchip-isc-config.yaml (and thus
    # imx219.yaml), so a calibrated run and a fallback run key the same buckets.
    _CT_NAMES = [
        (2500, 'candlelight'),
        (3000, 'tungsten'),
        (4000, 'fluorescent'),
        (5000, 'daylight_fluorescent'),
        (6500, 'daylight'),
        (7500, 'cloudy_daylight'),
    ]

    def _name_for_ct(self, ct):
        return min(self._CT_NAMES, key=lambda e: abs(e[0] - ct))[1]

    def validate_config(self, config: dict) -> bool:
        ccm_cfg = config.get('ccm')
        if ccm_cfg is not None and not isinstance(ccm_cfg, dict):
            logger.error('ccm config section must be a mapping')
            return False
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        ccms = self.do_calibration(images)
        # do_calibration returns None on a solve error and an empty list when no
        # Macbeth images are present; both mean nothing was calibrated. Fall back
        # to the authored ccm section so a full regeneration still reproduces a
        # complete tuning file. Emitting an empty matrices map instead would drop
        # the colour-correction matrices the IPA relies on.
        if not ccms:
            logger.info('CCM: no Macbeth captures; using config values')
            return config.get('ccm', None)

        matrices = {}
        de_avgs = []
        de_maxs = []
        for entry in ccms:
            ct = int(round(entry['ct']))
            flat = [round(float(v), 4) for row in entry['ccm'] for v in row] \
                if hasattr(entry['ccm'][0], '__len__') else \
                [round(float(v), 4) for v in entry['ccm']]
            name = self._name_for_ct(ct)
            if name in matrices:
                logger.warning(f'CCM: {ct}K and {matrices[name]["ct"]}K both map '
                               f'to the "{name}" bucket; keeping the later {ct}K '
                               f'matrix. Space calibration CTs further apart '
                               f'(buckets: {[n for _, n in self._CT_NAMES]}).')
            matrices[name] = {'ct': ct, 'ccm': flat}
            if 'delta_e_avg' in entry:
                de_avgs.append(float(entry['delta_e_avg']))
                de_maxs.append(float(entry['delta_e_max']))
                logger.info(f'CCM: {ct}K CIEDE2000 avg={entry["delta_e_avg"]:.2f} '
                            f'max={entry["delta_e_max"]:.2f}')

        # Static defaults the IPA also reads under Ccm; keep them tunable via
        # the input config rather than hardcoded here.
        section = config.get('ccm', {})
        out = {
            'smoothing_factor': section.get('smoothing_factor', 0.7),
            'ct_fallback': section.get('ct_fallback', 5500),
            'ct_snap': section.get('ct_snap',
                                   {'snap_threshold': 1500,
                                    'fast_threshold': 500,
                                    'fast_factor': 0.5}),
            'matrices': matrices,
        }
        # Underscore-prefixed keys are tool-internal diagnostics for the health
        # score; MicrochipYamlOutput strips them so the tuning file stays clean.
        if de_avgs:
            out['_delta_e_avg'] = round(float(sum(de_avgs) / len(de_avgs)), 3)
            out['_delta_e_max'] = round(float(max(de_maxs)), 3)
        return out
