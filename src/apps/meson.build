# SPDX-License-Identifier: CC0-1.0

opt_cam = get_option('cam')
opt_lc_compliance = get_option('lc-compliance')

# libevent is needed by cam and lc-compliance. As they are both feature options,
# they can't be combined with simple boolean logic.
libevent = dependency('libevent_pthreads', required : opt_cam)
if not libevent.found()
    libevent = dependency('libevent_pthreads', required : opt_lc_compliance)
endif

libtiff = dependency('libtiff-4', required : false)

subdir('common')

subdir('lc-compliance')

subdir('cam')
subdir('qcam')

subdir('ipa-verify')

# meson.build support for mchpcam applications
opt_mchpcam = get_option('mchpcam')
mchpcam_still_enabled = false

if not opt_mchpcam.disabled()
    libjpeg = dependency('libjpeg', required : false)
    libpng = dependency('libpng', required : false)
    glib_dep = dependency('glib-2.0', required: false)
    gstreamer_dep = dependency('gstreamer-1.0', required: false)
    gstreamer_app_dep = dependency('gstreamer-app-1.0', required: false)
    gstreamer_video_dep = dependency('gstreamer-video-1.0', required: false)

    if libjpeg.found() and libpng.found()
        mchpcam_still_enabled = true
    endif

    if opt_mchpcam.enabled()
        if not mchpcam_still_enabled
            error('mchpcam-still is enabled but libjpeg or libpng is missing')
        endif
    else
        if not mchpcam_still_enabled
            message('libjpeg or libpng for mchpcam-still is missing, skipping build')
        endif
    endif
endif

if mchpcam_still_enabled or mchpcam_vid_enabled
    subdir('mchpcam')
    summary({
        'mchpcam-still application': mchpcam_still_enabled,
    }, section : 'Mchpcam Applications')
endif
