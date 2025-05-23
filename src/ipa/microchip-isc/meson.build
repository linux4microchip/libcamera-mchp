# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_microchip_isc'  # Changed to match the interface name

microchip_isc_sources = files([
    'microchip_isc_ipa.cpp',     # Add the IPA interface implementation
    'algorithms/awb.cpp',
    'algorithms/agc.cpp',
    'algorithms/blc.cpp',
    'algorithms/common.cpp',
    'algorithms/ccm.cpp',
])

mod = shared_module(ipa_name,
                    [microchip_isc_sources, libcamera_generated_ipa_headers],
                    name_prefix : '',
                    include_directories : [ipa_includes],
                    dependencies : [libcamera_private, libipa_dep],
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@'],
                  install : false,
                  build_by_default : true)
endif

ipa_names += ipa_name
