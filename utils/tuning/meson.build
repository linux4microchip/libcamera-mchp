# SPDX-License-Identifier: CC0-1.0

py3 = import('python').find_installation('python3', required : false)

if not py3.found()
    subdir_done()
endif

tuning_tests = [
    'test_awb_variance.py',
    'test_gamma_oecf.py',
    'test_lsc_per_ct.py',
    'test_macbeth_qa.py',
    'test_noise_profile.py',
    'test_pgm_loader.py',
    'test_repeatability.py',
]

foreach t : tuning_tests
    test(t.replace('.py', '').replace('test_', 'tuning-'),
         py3,
         args : files(t),
         workdir : meson.current_source_dir(),
         suite : 'tuning')
endforeach
