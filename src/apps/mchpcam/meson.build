# SPDX-License-Identifier: CC0-1.0

opt_mchpcam = get_option('mchpcam')
if not opt_mchpcam.allowed()
    subdir_done()
endif

mchpcam_still_sources = files([
    'mchpcam-still.cpp',
    'mchpcam_common.cpp',
])

libjpeg = dependency('libjpeg', required : true)
libpng  = dependency('libpng',  required : true)

executable('mchpcam-still', mchpcam_still_sources,
           dependencies : [
               libcamera_public,
               libcamera_private,
               libjpeg,
               libpng,
           ],
           install : true)
