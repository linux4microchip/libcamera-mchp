# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Ideas on Board
#
# Module implementation for static data

from .module import Module


# This module can be used in cases where the tuning file should contain
# static data.
class StaticModule(Module):
    def __init__(self, out_name: str, output: dict = {}):
        super().__init__()
        self.out_name = out_name
        self.hr_name = f'Static {out_name}'
        self.type = f'static_{out_name}'
        self.output = output

    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        return self.output


# Like StaticModule, but the section is taken from the input config at run time
# (config[config_key]) rather than fixed at construction. Used for tuning
# sections that are authored in the config and not calibrated from captures, so
# a full regeneration reproduces them verbatim. Returning None when the key is
# absent makes the generator skip the section (rather than emit an empty one).
class ConfigPassthroughModule(Module):
    def __init__(self, out_name: str, config_key: str):
        super().__init__()
        self.out_name = out_name
        self.hr_name = f'Passthrough {out_name}'
        self.type = f'passthrough_{out_name}'
        self.config_key = config_key

    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        return config.get(self.config_key)
