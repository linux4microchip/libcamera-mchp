libcamera_deps = dependency('libcamera', required: true)

mchpcam_common_sources = files('mchpcam_common.cpp')
mchpcam_still_sources = files('mchpcam-still.cpp')

mchpcam_common_deps = [
    libcamera_deps,
]

mchpcam_still_deps = [
    mchpcam_common_deps,
    libjpeg,
    libpng,
]

if mchpcam_still_enabled
    executable('mchpcam-still',
        [mchpcam_common_sources, mchpcam_still_sources],
        dependencies : mchpcam_still_deps,
            include_directories: include_directories(join_paths(meson.build_root(), 'include')),
        install : true
    )
endif
