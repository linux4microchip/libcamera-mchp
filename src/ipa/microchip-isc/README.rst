.. SPDX-License-Identifier: CC-BY-SA-4.0

.. _microchip-isc-calibration:

Microchip ISC IMX219 calibration
================================

This document describes how to calibrate the IMX219 tuning file for the
Microchip ISC IPA. The tuning files in the tree are a baseline and are not
calibrated for an individual camera. A camera that has not been calibrated
reports an unreliable colour temperature and shows a colour cast.

.. note::
   The Microchip ISC IPA and its calibration tooling are work in progress.
   If in doubt, ask on the mailing list.

The colour temperature estimate drives both the white balance gains and the
colour matrix. It depends on two entries in the tuning file, ``ct_color_matrix``
and ``ct_correction_pwl``. The shipped matrix is approximate and the correction
curve is absent until you calibrate, so calibration is required for accurate
colour.

The standard sensor uses ``imx219.yaml``. The NoIR variant has no IR-cut filter
and uses ``imx219_noir.yaml``, calibrated from its own captures.

Requirements
------------

White balance and colour temperature, the minimum:

- An 18% neutral grey card.
- A light source with a known, rated colour temperature, used at two or more
  points such as 2850 K, 4500 K and 6500 K. A small lightbox or a CCT-rated LED
  panel works.
- A handheld lux meter is optional, and only needed to calibrate the lux
  estimator.

Colour matrix (CCM) accuracy also needs an X-Rite or Calibrite ColorChecker
(24-patch Macbeth chart), evenly lit with no glare.

Set up the host tools
---------------------

The calibration tool runs on the host. Install its dependencies in a virtual
environment, not the system Python::

    python3 -m venv ~/calib-venv
    . ~/calib-venv/bin/activate
    pip install -r utils/tuning/microchip_isc/requirements.txt

This covers grey-card AWB, CT and lux calibration. CCM from RAW captures also
needs ``rawpy``, left commented out in the requirements file as it needs the
libraw headers.

Activate the environment in each new shell before running the tool::

    . ~/calib-venv/bin/activate

Check that the tool and its dependencies load without capturing anything::

    python3 utils/tuning/microchip_isc_lt.py --help

Capture on the board
--------------------

Copy the capture helper to the board and run it on the board, over the serial
console or a local terminal. Do not run it over SSH, as a disconnect can leave
the sensor in an unrecoverable state.

::

    scp utils/tuning/mchp_capture.sh root@<board>:/tmp/

Capture a grey card at each rated colour temperature. The ``-c`` value is the
lightbox-rated temperature in Kelvin, not the IPA estimate::

    /tmp/mchp_capture.sh -t grey_card -c 2850
    /tmp/mchp_capture.sh -t grey_card -c 4500
    /tmp/mchp_capture.sh -t grey_card -c 6500

Add ``-l <lux>`` if you have a meter reading, and ``--noir`` for the NoIR
sensor. Each run writes a JPEG and a JSON sidecar to ``/tmp/calib``. The sidecar
carries the pre-white-balance histogram channel means that the calibration
modules read, so the JPEG alone is not enough.

For the colour matrix, capture the ColorChecker chart separately at the same
temperatures, as a 10-bit Bayer frame::

    /tmp/mchp_capture.sh -t colorchecker -c 4500 --raw

Keep the filenames the helper generates. The tool parses the ``ct<K>`` and
``lux<N>`` tags from them.

Copy the captures to the host
-----------------------------

Copy the images and their sidecars into a fresh directory for each session.
Mixing captures from different sessions corrupts the averaged channel means::

    scp root@<board>:/tmp/calib/*.jpg root@<board>:/tmp/calib/*.json ./captures/

Run the calibration
-------------------

Run the tool offline against the captures. One command runs every stage (black
level, AWB, CT, lux, CBC, gamma and CCM); the config file supplies the authored
fallbacks and the ColorChecker detection settings::

    python3 utils/tuning/microchip_isc_lt.py \
        -i ./captures/ -o src/ipa/microchip-isc/data/imx219.yaml \
        -c utils/tuning/microchip-isc-config.yaml

Stages with no matching capture fall back to the values in the config, so a run
with only grey-card images still produces a complete tuning file. The tool
writes the tuning file in place, so keep it under version control to track
changes.

The colour matrix is solved automatically once ColorChecker captures are in the
input directory. Detection is controlled by the ``general.macbeth`` section of
the config (``small: 1`` when the chart fills less than a fifth of the frame,
``show: 1`` to display the located chart). No separate command is needed, the
CCM stage runs as part of the same invocation.

If a uniformly-lit flat-field capture tagged ``alsc`` is present, the tool also
writes a lens-shading gain map into the tuning file's top-level ``modules:``
list (a flat-field gain sweep likewise adds a noise profile there). The ISC
pipeline handler reads these blocks and applies them on the still path as a CPU
post-process, since the ISC has no hardware lens-shading or noise-reduction
block. They are not IPA algorithms and never appear in the ``algorithms:`` list.

For the NoIR sensor there is no separate flag. Point ``-o`` at
``imx219_noir.yaml`` and use captures taken with the NoIR module::

    python3 utils/tuning/microchip_isc_lt.py \
        -i ./captures/ -o src/ipa/microchip-isc/data/imx219_noir.yaml \
        -c utils/tuning/microchip-isc-config.yaml

Verify the result
-----------------

A calibration health summary prints automatically at the end of every run. It
scores each stage out of 100 and flags any stage that fell back to authored
config values instead of measured captures, so you can see straight away which
stages have real calibration behind them. To confirm the result on hardware,
deploy the tuning file (below) and capture a grey card again, then compare the
R/G and B/G ratios against a neutral target.

Deploy and re-test
------------------

Copy the tuning file to the board and capture again::

    scp src/ipa/microchip-isc/data/imx219.yaml \
        root@<board>:/usr/share/libcamera/ipa/microchip-isc/imx219.yaml

Recovery and troubleshooting
----------------------------

To return to a known state before re-calibrating, restore the tuning file from
version control. Running the tool with a config but no captures also rewrites it
to the authored fallbacks, since every stage with no matching capture falls back
to the config values.

Missing JSON sidecar
    A plain ``mchpcam-still`` capture has no histogram metadata. Capture with
    ``mchp_capture.sh``, which writes the sidecar. Images without a sidecar are
    skipped by the AWB, CT and lux modules.

NIR contamination on dark frames
    During black level calibration the tool refuses to write if a dark frame
    reads high on R or B relative to the green channels. Cover the lens with
    black electrical tape, not a plastic cap which passes near-infrared, and
    capture again.

Mixed sessions
    The tool warns when the input directory holds images from more than one
    session. Use a fresh directory for each run.
