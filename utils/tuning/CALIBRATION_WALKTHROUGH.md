<!-- SPDX-License-Identifier: GPL-2.0-or-later -->

# Microchip ISC / IMX219 — lightbox calibration walkthrough

Step-by-step procedure to capture the calibration image set the libtuning
generator (`microchip_isc_lt.py`) turns into an IPA tuning file
(`imx219.yaml` / `imx219_noir.yaml`) plus the sibling still-path LSC map
(`<output>_lsc.yaml`).

Everything here runs **on the board** through `mchp_capture.sh`. The tuning
generator runs **on the host** afterwards. The board is driven over SSH: the
sweep auto-detaches so a disconnect cannot lock the sensor (see Step 0 and
§4.3), and framing is live over SSH via `mchp_frame_assist.py`.

---

## 1. What each capture feeds

| Capture | Target | Mode | Feeds |
|---|---|---|---|
| Dark frame | Lens capped | RAW | Black-level floor (BlackLevelCorrection) |
| Flat field | Defocused uniform white | RAW (`alsc`) | Still-path LSC map (`_lsc.yaml`) |
| Grey card | Neutral 18 % grey | JPEG | AWB `reference_gains_table`, CT curve, AGC/lux |
| Macbeth | ColorChecker Classic | RAW | CCM, per-CT colour matrices |

The grey-card and Macbeth captures are repeated at **four illuminants**:

| CCT | Standard illuminant | Notes |
|---|---|---|
| 2856 K | CIE A (tungsten) | Warm anchor |
| 4000 K | — | Mid / cool-white |
| 5000 K | D50 | Print/daylight anchor |
| 6500 K | D65 | Daylight anchor |

---

## 2. Why unity gain, why non-clipped

- **Unity analogue gain (1.0×).** Every calibration frame is captured at the
  same low-noise operating point. Gain driven up by AGC adds shot/read noise
  that biases the CCM and reference-gain fits. `mchp_capture.sh` defaults to
  `--gain=1.0`, which locks the sensor with `--gain-mode=manual
  --analoggain=1.0` and lets AE reach the brightness target using **exposure
  only**. The lightbox must therefore be bright enough to hit ~18 % grey at
  unity gain within the exposure ceiling.
- **Non-clipped.** A clipped highlight is a saturated sensor well — its true
  value is unknown, so any patch touching it corrupts the colour fit. The
  script now inspects the **full per-channel histogram** (`hist_bins`), not
  just the mean, and FAILs a capture when more than 0.2 % (grey/flat) or
  1.0 % (Macbeth white patch) of samples land in the top raw bins.

---

## 3. Equipment

- Lightbox / integrating source with a dialable/known CCT covering 2856–6500 K
  (or discrete D50/D65/A + a 4000 K source). Record the **rated CCT** — it is
  ground truth; the IPA's own CT estimate is never used for calibration.
- Neutral 18 % grey card.
- X-Rite ColorChecker Classic (24-patch).
- Handheld lux meter (optional but recommended — feeds the AGC lux curve).
- Lens cap (dark frame) and a way to **defocus** for the flat field
  (diffuser over the lens, or rack focus fully off a uniform white field).
- Tripod / fixed mount; the sensor must not move within a CCT set.

---

## 0. One-time SSH setup (per host, per board)

Passwordless SSH is set up once from the calibration host:
```bash
python3 utils/tuning/mchp_frame_assist.py --setup-ssh --host <board>
```
Enter the board password when prompted (once). This copies your SSH key so all
later framing, capture and file-copy steps run without a prompt. Re-run it for a
**new board**, or whenever the board's IP changes to one you have used before
(it clears the stale host key). If the board rootfs is not persistent across
reboots the copied key may not survive — re-run `--setup-ssh` after a reboot.

The board IP is not fixed anywhere: every command takes `--host <ip>` (framing)
or `root@<ip>` (capture/copy), so a changed IP just means a different value.

---

## 4. Pre-flight

1. **Confirm the sensor module.** The tuning splits NoIR vs IR-cut. Use
   `--noir` for a NoIR (no IR-cut-filter) module and drop it for an IR-cut
   module. Mixing the two invalidates the colour fit.
2. **Deploy the app + script to the board:**
   ```bash
   scp build/src/apps/mchpcam/mchpcam-still root@<board>:/usr/bin/
   scp utils/tuning/mchp_capture.sh          root@<board>:/tmp/
   ```
3. **SSH is fine — the sweep is disconnect-safe.** Over SSH `mchp_capture.sh`
   ignores SIGHUP and re-execs itself detached into its own session, logging to
   `<outdir>/capture_<ts>.log`; a dropped or killed SSH session can no longer
   interrupt a capture mid-stream. `tail -f` the log to watch, and wait for the
   `<outdir>/.capture_done` marker. Pass `--foreground` to keep output on the
   terminal (SIGHUP is still ignored). The serial console works too, but is no
   longer required.
4. **Warm up.** Let the lightbox stabilise (several minutes) before the first
   capture at each CCT.

---

## 5. Capture sequence

Work in one directory per run (default `/tmp/calib`). Do the CCT-independent
captures once, then the CCT sweeps.

Frame the chart live before each capture and adjust until the banner is green:
```bash
python3 utils/tuning/mchp_frame_assist.py --watch --show --host <board>
```
Then trigger the sweep over SSH — it detaches automatically:
```bash
ssh root@<board> /tmp/mchp_capture.sh -t colorchecker -c 5000 --raw --noir
```
`tail -f <outdir>/capture_<ts>.log` to watch; collect the files when
`<outdir>/.capture_done` appears.

### 5a. Dark frame (black level) — once

Cap the lens completely.
```sh
/tmp/mchp_capture.sh -t flat_field -c 5000 --noir   # lens CAPPED; RAW forced
```
(The `-t flat_field` path forces RAW; a fully-capped lens gives the black
floor. `-c` is required syntactically — the value is irrelevant for a dark
frame.) Rename or tag the file as the dark frame if your generator config
distinguishes it; otherwise keep it separate from the defocused flat field.

### 5b. Flat field (LSC) — once per illuminant you calibrate LSC at

Defocus onto a uniform, evenly-lit white field filling the frame.
```sh
/tmp/mchp_capture.sh -t flat_field -c 5000 --noir
```
Check the validator: luminance in range, **no clipping** on any channel. Even
illumination matters more than exact CCT here — vignetting is what's measured.

### 5c. Grey-card CCT sweep (AWB / CT / lux)

For each CCT, set the lightbox, let it settle, then:
```sh
for CCT in 2856 4000 5000 6500; do
    echo "Set lightbox to ${CCT}K, press Enter when ready"; read _
    /tmp/mchp_capture.sh -t grey_card -c $CCT -l <lux> --noir
done
```
Replace `<lux>` with the handheld reading (omit `-l` if you have no meter).
Each capture must report **AWB locked**, **AE converged**, unity gain, and no
clipping. Re-capture any that FAIL.

### 5d. Macbeth CCT sweep (CCM) — RAW

Fill the frame with the ColorChecker, evenly lit, no glare on the patches.
```sh
for CCT in 2856 4000 5000 6500; do
    echo "Set lightbox to ${CCT}K, press Enter when ready"; read _
    /tmp/mchp_capture.sh -t colorchecker -c $CCT -l <lux> --raw --noir
done
```
RAW gives the tool linear per-patch values. Watch the white/light-grey
patches for clipping — that is the most common Macbeth failure; reduce
intensity or add an ND filter until all four channels pass.

---

## 6. Validation (what to trust)

`mchp_capture.sh` prints a PASS/WARN/FAIL block per capture:

- **FAIL Luminance … (too dark / clipping)** — mean out of [15 %, 85 %].
- **FAIL `<ch>` clipping: X %** — per-bin saturation over limit. Hard stop:
  reduce light / add ND. This is the non-clipped gate.
- **FAIL Analogue gain … != requested** — the manual gain lock didn't take;
  the build doesn't honour `--gain-mode=manual`. Fix the build, not the light.
- **WARN AWB not locked / AE not converged** — extend warmup (`--timeout 90`)
  and re-capture; required for grey-card AWB calibration.
- **FAIL GR/GB, R/GR, B/GR out of range** — non-neutral card, mixed
  illuminant, or a black-level error. Verify the scene before proceeding.

Only proceed to the generator once every capture in the set reports OK.

---

## 7. Copy to host and generate the tuning file

```bash
scp root@<board>:/tmp/calib/*.pgm  /path/to/calib_images/
scp root@<board>:/tmp/calib/*.jpg  /path/to/calib_images/
scp root@<board>:/tmp/calib/*.json /path/to/calib_images/

python3 utils/tuning/microchip_isc_lt.py \
    -i /path/to/calib_images/ \
    -o src/ipa/microchip-isc/data/imx219_noir.yaml \
    -c utils/tuning/microchip-isc-config.yaml
```
Use `imx219.yaml` (drop `--noir` throughout) for an IR-cut module. The
generator auto-detects the Macbeth chart, diverts the LSC grid into
`imx219_noir_lsc.yaml`, and writes the AWB/AGC/CCM sections into the tuning
file.

### Merge, not overwrite

When `-o` points at an existing tuning file the generator **merges** in place:
it round-trips the file and replaces only the measured blocks, leaving every
authored block and comment byte-identical. A no-capture run is a no-op. Only a
first-time (absent) target is written whole.

**Measured (regenerated on each run):** `Awb.reference_gains_table`,
`Awb.ct_color_matrix`, `Agc.lux_reference`, `Ccm.matrices`, and the
`modules.LensShadingCorrection` grid. A block is touched only when its module
actually recomputed it. An unsolved stage leaves the authored value alone.

**Authored (never touched by the tuner):** the tone/contrast "look" —
`Gamma`, `modules.AutoLevels`, `Cbc`, and the `Agc` metering knobs
(`scene_ev_bias`, `highlight_p85_*`, `zone_metering_strength`, `ev_default`).
These are hand-tuned design constants, not physical measurements. Do not try to
"calibrate" them. The Gamma OECF fit is emitted as a health advisory only and
does not propagate into the shipped `gamma:`.

The noise profile is fit and scored in the health summary but **not written**:
there is no runtime noise-reduction consumer yet, so the writer is parked.

---

## 8. Troubleshooting

- **Sensor stuck** — unlikely now the sweep is disconnect-safe, but if it
  happens power-cycle the board; then see the parked "kernel AWB capture reset"
  note in `CLAUDE.md`.
- **Everything clips at unity gain** — the box is too bright for the exposure
  floor; add an ND filter rather than raising gain.
- **Can't reach 18 % grey at unity gain** — box too dim; raise intensity or
  extend `--timeout`. Do **not** switch to `--gain=auto` for calibration
  unless you accept the added noise (it is offered only as an escape hatch).
- **NoIR warm-CCT blue looks impossibly low** — expected; NIR contaminates
  R/G and suppresses apparent blue. The validator relaxes NoIR thresholds
  accordingly.
