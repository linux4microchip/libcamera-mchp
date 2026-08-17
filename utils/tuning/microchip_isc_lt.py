#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# microchip_isc_lt.py - libtuning-based tuning generator for the Microchip ISC
#
# Mainline-style replacement for the bespoke microchip_isc.py: composes
# libtuning modules (like utils/tuning/rkisp1.py) so calibration is a single
# command with automatic Macbeth-chart detection, and the tuning file is
# generated data consumed by the IPA (no magic numbers in the IPA .cpp).
#
# All the Microchip calibration math lives in the libtuning module tree
# (libtuning/modules/{awb,agc,ccm}/microchip_isc.py) as hardware subclasses of
# the base AWB/AGC/CCM modules, exactly as rkisp1 does. This script only
# composes them.
#
# Usage:
#   ./microchip_isc_lt.py -i <calib-images-dir> -o imx219.yaml -c config.yaml
#
# Calibration input set (see utils/tuning/README):
#   - dark frame (lens capped)            -> BlackLevelCorrection
#   - flat field (uniform), alsc-tagged   -> 'modules:' LensShadingCorrection
#   - Macbeth under known illuminant(s)   -> Awb, Agc (lux), Ccm

import logging
import sys

import coloredlogs
import libtuning as lt
from libtuning.generators import YamlOutput
from libtuning.macbeth_qa import qa_images
from libtuning.modules.agc import AGCMicrochipISC
from libtuning.modules.awb import AWBMicrochipISC
from libtuning.modules.ccm import CCMMicrochipISC
from libtuning.modules.gamma import GammaMicrochipISC
from libtuning.modules.lsc import LSCMicrochipISC
from libtuning.modules.module import Module
from libtuning.modules.noise import NoiseMicrochipISC
from libtuning.modules.static import ConfigPassthroughModule
from libtuning.parsers import YamlParser

from microchip_isc_health import log_health_summary

logger = logging.getLogger(__name__)


# Tuner that prints a calibration health summary after a successful
# run. The base Tuner stores each module's emitted section in self.output
# (keyed by module instance); we remap it to {out_name: section} and score it.
#
# AWB/AGC/CCM return the *same* config object they were handed when no captures
# are available (config.get('awb'/'agc'/'ccm')), so an identity comparison tells
# us which stages were measured this run versus fell back to authored values.
# The health score marks fallback stages so a no-capture regeneration cannot
# read as fully calibrated.
class MicrochipTuner(lt.Tuner):
    _MEASURED_KEYS = {'Awb': 'awb', 'Agc': 'agc', 'Ccm': 'ccm', 'Gamma': 'gamma'}

    def run(self, argv):
        rc = super().run(argv)
        if rc == 0:
            sections = {m.out_name: self.output[m]
                        for m in self.output_order if m in self.output}
            measured = set()
            for out_name, cfg_key in self._MEASURED_KEYS.items():
                sect = sections.get(out_name)
                if sect is not None and sect is not self.config.get(cfg_key):
                    measured.add(out_name)
            log_health_summary(sections, measured)
        return rc


# The still-path lens-shading map is not an IPA algorithm. The IPA loads every
# entry under the tuning file's 'algorithms:' list by name and would fail on an
# unknown LensShadingCorrection algorithm. This generator keeps the 'algorithms:'
# list to algorithms the IPA implements and diverts the LSC section to the
# tuning file's top-level 'modules:' list, which the ISC pipeline handler reads
# directly for its still-path corrector.
# Macbeth capture quality assurance. Not an IPA algorithm: it inspects the
# located charts (clipping, tilt, uneven illumination) and returns a per-capture
# verdict the health summary scores as an advisory 'MacbethQA' stage. Returns
# None when no chart was located, so the section is simply absent then.
class MacbethQAModule(Module):
    hr_name = 'Macbeth QA'
    out_name = 'MacbethQA'
    type = 'macbeth_qa'

    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        return qa_images(images)


class MicrochipYamlOutput(YamlOutput):
    # Still-path sections are not IPA algorithms and must never appear in the
    # tuning file's 'algorithms:' list. Lens-shading and noise data instead go
    # into a top-level 'modules:' list carried in the same tuning file (mirroring
    # the dw100 dewarp module block in rkisp1). The ISC pipeline handler re-opens
    # the tuning file and reads these blocks for its CPU still-path corrector and
    # denoiser; the IPA never sees them. out_name -> module block key (the key
    # the pipeline handler looks up).
    _MODULE_BLOCKS = {
        'LensShadingCorrection': 'LensShadingCorrection',
        'NoiseProfile': 'NoiseReduction',
    }

    # Sections that must never be written into the tuning file's 'algorithms:'
    # list. The still-path sections go to the 'modules:' block (above); MacbethQA
    # is a health-only diagnostic and is dropped entirely.
    _NON_IPA_OUT_NAMES = set(_MODULE_BLOCKS) | {'MacbethQA'}

    # Tool-internal diagnostics (e.g. CCM CIEDE2000) are carried on the emitted
    # sections under underscore-prefixed keys so the health score can read them.
    # They are not IPA tuning parameters, so strip them before writing the file.
    @classmethod
    def _strip_diagnostics(cls, value):
        if isinstance(value, dict):
            return {k: cls._strip_diagnostics(v) for k, v in value.items()
                    if not (isinstance(k, str) and k.startswith('_'))}
        if isinstance(value, list):
            return [cls._strip_diagnostics(v) for v in value]
        return value

    def write(self, output_file, output_dict, output_order):
        # Copy with diagnostics stripped; the original dicts (with diagnostics)
        # remain in the tuner's output for the post-write health summary.
        output_dict = {m: self._strip_diagnostics(sect)
                       for m, sect in output_dict.items()}
        ipa_order = [m for m in output_order
                     if getattr(m, 'out_name', None) not in self._NON_IPA_OUT_NAMES]
        super().write(output_file, output_dict, ipa_order)

        # Append the still-path tuning data as a top-level 'modules:' list. Keep
        # it in the same file (not sibling files) so one artefact fully describes
        # a sensor. Emitted only when a module actually produced data, so a
        # capture set without flat-fields leaves the file with no 'modules:'
        # block and the runtime keeps its built-in defaults (byte-identical to
        # the pre-calibration path).
        module_lines = []
        for out_name, block_key in self._MODULE_BLOCKS.items():
            mod = next((m for m in output_order
                        if getattr(m, 'out_name', None) == out_name
                        and m in output_dict), None)
            if mod is None or not output_dict[mod]:
                continue
            module_lines.append(f'  - {block_key}:')
            module_lines += [f'      {line}'
                             for line in self._stringify_dict(output_dict[mod])]

        if not module_lines:
            return

        with open(output_file, 'a', encoding='utf-8') as f:
            f.write('modules:\n')
            for line in module_lines:
                f.write(f'{line}\n')
        logger.info('Still-path tuning data written to the "modules:" block of '
                    f'{output_file} (consumed by the ISC pipeline handler, not '
                    'the IPA)')


coloredlogs.install(level=logging.INFO,
                    fmt='%(name)s %(levelname)s %(message)s')


# AWB, AGC (lux) and CCM all derive from the same located Macbeth captures.
# libtuning's core loader (libtuning/image.py + utils.py) locates the chart and
# fills Image.patches; AWB takes the neutral-ramp channel ratios for the gains,
# AGC reads the Neutral-5 patch for the lux reference_Y, and CCM runs the
# standard least-squares solve over all 24 patches. One chart capture per
# illuminant feeds all three, so the gains, exposure reference and colour
# matrix describe one identical illuminant.
awb = AWBMicrochipISC(debug=[lt.Debug.Plot])
agc = AGCMicrochipISC(debug=[lt.Debug.Plot])
ccm = CCMMicrochipISC(debug=[lt.Debug.Plot])

# BlackLevelCorrection and Cbc are authored (not calibrated from captures):
# their values live in the input config and are emitted verbatim, so a full
# regeneration reproduces the complete tuning file. See the config-key sections
# in microchip-isc-config.yaml.
blc = ConfigPassthroughModule('BlackLevelCorrection', 'blc')
cbc = ConfigPassthroughModule('Cbc', 'cbc')

# Gamma fits its base exponent from the located Macbeth neutral ramp (measured
# OECF vs the reference sRGB encoding); the toe and highlight-shoulder scalars,
# which the ramp cannot sample, are held at their authored config values. With
# no located chart it falls back to the authored gamma config verbatim.
gamma = GammaMicrochipISC()

# Still-path lens-shading corrector map. Built from a uniformly-lit flat-field
# capture whose filename carries the 'alsc' tag; the per-Bayer-channel falloff
# is measured on the raw plane and emitted to the tuning file's 'modules:' block
# as LensShadingCorrection. The ISC has no hardware shading block, so this is
# applied by the ISC pipeline handler as a CPU still-path post-process, not by
# the IPA. A run with no flat-field capture omits the map (the module returns
# nothing).
lsc = LSCMicrochipISC(debug=[lt.Debug.Plot],
                      sector_shape=(16, 12),
                      sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                      sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                      sector_average_function=lt.average.Mean(),
                      smoothing_function=lt.smoothing.MedianBlur(3))

# Still-path denoiser noise profile. Read-noise variance, shot-noise slope and
# gain exponent measured from a flat-field gain sweep (mchp_capture.sh gain axis
# + --frames repeats), emitted to the tuning file's 'modules:' block as
# NoiseReduction. Like the lens-shading map it is applied by the ISC pipeline
# handler's still path, not the IPA; a run without a flat-field sweep omits it
# (the module returns nothing) and the denoiser keeps its built-in defaults.
noise = NoiseMicrochipISC(debug=[lt.Debug.Plot])

# Capture-quality gate over the located Macbeth charts. Emits no IPA section;
# its verdict is scored by the health summary and dropped from the tuning file.
qa = MacbethQAModule()

tuner = MicrochipTuner('MicrochipISC')
tuner.add([blc, awb, agc, cbc, gamma, ccm, lsc, noise, qa])
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(MicrochipYamlOutput())

# Keep the IPA's section order in the output. LSC and the noise profile are
# diverted to the tuning file's 'modules:' block (still path, not IPA) and
# MacbethQA is a health-only diagnostic; the generator keeps all three out of
# the emitted 'algorithms:' list.
tuner.set_output_order([blc, awb, agc, cbc, gamma, ccm, lsc, noise, qa])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
