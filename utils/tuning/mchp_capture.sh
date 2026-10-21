#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
#
# mchp_capture.sh — Calibration image capture for Microchip ISC IPA tuning
#
# Runs on the board. Over SSH it auto-detaches and logs to a file so a dropped
# connection cannot kill a capture mid-stream. Pass --foreground to opt out.
#
# After capture, copy the output files to the host:
#   scp root@<board-ip>:/tmp/calib/*.jpg /path/to/calib_images/
#   scp root@<board-ip>:/tmp/calib/*.json /path/to/calib_images/
# Then run:
#   python3 utils/tuning/microchip_isc_lt.py \
#       -i /path/to/calib_images/ \
#       -o src/ipa/microchip-isc/data/imx219_noir.yaml \
#       -c utils/tuning/microchip-isc-config.yaml
#
# Usage:
#   mchp_capture.sh [OPTIONS]
#
# Options:
#   -t, --target <type>   Target: grey_card (default) | colorchecker | flat_field
#                         colorchecker feeds AWB, CT, lux and CCM; flat_field
#                         feeds LSC and forces --raw; grey_card is a neutrality
#                         and exposure check, not a tuning input.
#   -c, --cct <K>         Lightbox-rated colour temperature in Kelvin (required)
#   -l, --lux <N>         Incident illuminance from handheld lux meter (optional)
#   -g, --gain <x|auto>   Analogue gain lock (default: 1.0 = unity). Calibration
#                         runs at a fixed low-noise point; AE reaches target via
#                         exposure only. Pass "auto" to let AGC pick the gain.
#       --gain-sweep "L"  Space-separated analogue-gain list for noise-profile
#                         calibration (e.g. "1 2 4 8"). Overrides --gain: the
#                         point is captured at each gain so the noise tool sees
#                         variance rise with gain. Best with -t flat_field on a
#                         dim target (so high gains do not clip).
#   -o, --outdir <dir>    Output directory (default: /tmp/calib)
#   -r, --resolution WxH  Capture resolution (default: 1920x1080)
#   -n, --frames <N>      Frames per (CCT, gain) point (default: 3). Each frame
#                         is saved individually so the noise tool can measure
#                         temporal per-pixel variance across the N frames. Frame
#                         0 lets AE settle; the repeats lock its exposure so all
#                         N sit at one operating point (noise, not AE jitter).
#       --no-awb          Disable AWB (use for colorchecker captures)
#       --noir            Use NoIR tuning file (imx219_noir.yaml)
#       --raw             Capture 10-bit Bayer PGM instead of JPEG.
#                         Forces unity WB (512) — AE converges normally.
#                         Use for CCM calibration (better accuracy than JPEG).
#       --dry-run         Print the capture plan (filenames + mchpcam-still
#                         commands) without touching the camera. Use to verify a
#                         gain × frame sweep before committing board time.
#       --foreground      Keep output on the terminal instead of auto-detaching
#                         over SSH. SIGHUP is ignored either way.
#   -h, --help            Show this help
#
# Filename convention (parsed by microchip_isc_lt.py):
#   The loader keys off the ct<K>K / lux<N> / alsc tokens; gain, exposure and the
#   _gN_fM sweep indices ride in the tokens for humans and in the JSON sidecar for
#   the tools (gain_index, frame_index).
#   grey_card_ct2850K_lux2869_exp4500us_gain1.00_g0_f00_20260420_182043.jpg
#   colorchecker_ct4450K_lux2939_exp4200us_gain1.00_g0_f00_20260420_183012_raw.pgm
#   flat_field_alsc_ct5000K_exp4200us_gain2.00_g1_f02_20260420_184501_raw.pgm
#
# Examples:
#   # Grey card at 2856K (JPEG — neutrality/exposure check, unity gain):
#   mchp_capture.sh -t grey_card -c 2856 -l 2869 --noir
#
#   # Macbeth chart at 5000K RAW PGM (AWB, CT, lux and CCM; unity WB):
#   mchp_capture.sh -t colorchecker -c 5000 -l 2939 --raw --noir
#
#   # Uniformly-lit flat field (forces RAW — for still-path LSC calibration):
#   mchp_capture.sh -t flat_field -c 5000 --noir
#
#   # Standard 4-point CCT sweep (Illuminant A, ~4000K, D50, D65):
#   for CCT in 2856 4000 5000 6500; do
#       echo "Set lightbox to ${CCT}K, press Enter when ready"
#       read _
#       mchp_capture.sh -t colorchecker -c $CCT --raw --noir
#   done
#
#   # Per-CT flat-field gain sweep (noise profile — 5 frames at each gain):
#   for CCT in 2856 4000 5000 6500; do
#       echo "Set lightbox to ${CCT}K, press Enter when ready"
#       read _
#       mchp_capture.sh -t flat_field -c $CCT --gain-sweep "1 2 4 8" -n 5 --noir
#   done
#
# See src/ipa/microchip-isc/README.rst for the full lightbox procedure.
# =============================================================================

set -e

# Ignore SIGHUP so a dropped terminal cannot kill a capture mid-stream, which is
# what locks the sensor. Children inherit the ignored disposition.
trap '' HUP

# ── Defaults ──────────────────────────────────────────────────────────────────
TARGET="grey_card"
CCT=""
LUX=""
# Analogue gain lock for calibration. Unity (1.0) is the default so every
# calibration frame is captured at the same low-noise operating point; AE
# reaches the brightness target using exposure only. "auto" opts out.
GAIN="1.0"
# Space-separated analogue-gain sweep for noise-profile calibration. When set it
# overrides --gain: the point is captured at each listed gain, so the noise tool
# sees variance rise with gain. Empty = single-gain capture at --gain.
GAIN_SWEEP=""
OUTDIR="/tmp/calib"
WIDTH=1920
HEIGHT=1080
NO_AWB=0
NOIR=0
RAW=0
TIMEOUT=60
# Number of frames to capture per (CCT, gain) point. Each frame is saved
# individually (frame_index in name and sidecar) so the noise tool can measure
# temporal per-pixel variance; the AWB/CCM/lux solves use any one frame.
# Recommended: 3 for routine calibration, 5+ for a noise sweep.
N_FRAMES=3
# Print the capture plan (filenames + mchpcam-still commands) without touching
# the camera. Use to verify a sweep before committing board time.
DRY_RUN=0
# Keep output on the terminal instead of auto-detaching over SSH.
FOREGROUND=0
# Back-compat no-op: SSH capture is safe by default now (auto-detached).
ALLOW_SSH=0

# ── Argument parsing ──────────────────────────────────────────────────────────
# Save argv for the SSH re-exec below; the parse loop consumes "$@".
ORIG_ARGS=
for _a in "$@"; do
    ORIG_ARGS="$ORIG_ARGS '$(printf '%s' "$_a" | sed "s/'/'\\\\''/g")'"
done

while [ $# -gt 0 ]; do
    case "$1" in
        -t|--target)    TARGET="$2";  shift 2 ;;
        -c|--cct)       CCT="$2";     shift 2 ;;
        -l|--lux)       LUX="$2";     shift 2 ;;
        -g|--gain)      GAIN="$2";    shift 2 ;;
        --gain-sweep)   GAIN_SWEEP="$2"; shift 2 ;;
        -o|--outdir)    OUTDIR="$2";  shift 2 ;;
        -r|--resolution)
            WIDTH="${2%%x*}"
            HEIGHT="${2##*x}"
            shift 2 ;;
        -n|--frames)    N_FRAMES="$2"; shift 2 ;;
        --no-awb)       NO_AWB=1;     shift   ;;
        --noir)         NOIR=1;       shift   ;;
        --raw)          RAW=1;        shift   ;;
        --dry-run)      DRY_RUN=1;    shift   ;;
        --foreground)   FOREGROUND=1; shift   ;;
        --allow-ssh)    ALLOW_SSH=1;  shift   ;;
        --timeout)      TIMEOUT="$2"; shift 2 ;;
        -h|--help)
            sed -n '3,/^# =\{10,\}/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1 ;;
    esac
done

# Sweep temp files on any exit and, when detached, drop the done-marker the
# watcher polls for (written on success or failure so it never waits forever).
# Installed before validation so an early failure still releases the watcher.
_cleanup() {
    rm -f "$OUTDIR"/_calib_tmp_* "$OUTDIR"/_calib_stdout_* 2>/dev/null || true
    [ -n "$MCHP_DONE" ] && : > "$MCHP_DONE" 2>/dev/null || true
}
trap _cleanup EXIT INT TERM

# ── Validate ──────────────────────────────────────────────────────────────────
if [ -z "$CCT" ]; then
    echo "ERROR: --cct <K> is required (lightbox-rated colour temperature)" >&2
    echo "  Example: mchp_capture.sh -c 4450 -l 2939 --noir" >&2
    exit 1
fi

case "$TARGET" in
    grey_card|colorchecker|flat_field) ;;
    *)
        echo "ERROR: --target must be grey_card, colorchecker or flat_field" >&2
        exit 1 ;;
esac

# The still-path LSC measures per-Bayer-channel falloff on the raw plane, which
# a colour-processed JPEG no longer carries. A flat-field capture is therefore
# always raw.
[ "$TARGET" = "flat_field" ] && RAW=1

# Integer check for CCT
if ! echo "$CCT" | grep -qE '^[0-9]+$'; then
    echo "ERROR: --cct must be a positive integer, got: $CCT" >&2
    exit 1
fi

# Gain must be a positive number or the literal "auto"
if [ "$GAIN" != "auto" ] && ! echo "$GAIN" | grep -qE '^[0-9]+(\.[0-9]+)?$'; then
    echo "ERROR: --gain must be a positive number or 'auto', got: $GAIN" >&2
    exit 1
fi

# Frame count must be a positive integer.
if ! echo "$N_FRAMES" | grep -qE '^[0-9]+$' || [ "$N_FRAMES" -lt 1 ]; then
    echo "ERROR: --frames must be a positive integer, got: $N_FRAMES" >&2
    exit 1
fi

# Build the gain axis. --gain-sweep (a space-separated list) overrides --gain;
# each listed gain must be a positive number. Without a sweep the axis is the
# single --gain lock, so the common single-point capture path is unchanged.
if [ -n "$GAIN_SWEEP" ]; then
    for g in $GAIN_SWEEP; do
        if ! echo "$g" | grep -qE '^[0-9]+(\.[0-9]+)?$'; then
            echo "ERROR: --gain-sweep entries must be positive numbers, got: $g" >&2
            exit 1
        fi
    done
    GAIN_LIST="$GAIN_SWEEP"
else
    GAIN_LIST="$GAIN"
fi

# ── Detach over SSH ───────────────────────────────────────────────────────────
# Args are validated above, so a bad invocation fails here on the terminal
# rather than silently in the detached logfile. Over SSH, re-exec in a new
# session logging to a file so neither a disconnect (SIGHUP) nor a killed
# session (SIGKILL) can reach the capture mid-stream. --foreground opts out;
# MCHP_DETACHED breaks the re-exec loop; a dry-run never detaches.
if [ -z "$MCHP_DETACHED" ] && [ "$FOREGROUND" -eq 0 ] && [ "$DRY_RUN" -eq 0 ] && \
   { [ -n "$SSH_CONNECTION" ] || [ -n "$SSH_CLIENT" ] || [ -n "$SSH_TTY" ]; }; then
    mkdir -p "$OUTDIR"
    _log="$OUTDIR/capture_$(date +%Y%m%d_%H%M%S).log"
    _done="$OUTDIR/.capture_done"
    rm -f "$_done"
    echo "SSH detected — detaching so a disconnect cannot lock the sensor."
    echo "  log   : $_log"
    echo "  watch : tail -f $_log"
    echo "  done  : $_done appears when finished"
    echo "  (pass --foreground to keep output on this terminal)"
    if command -v setsid > /dev/null 2>&1; then
        eval "MCHP_DETACHED=1 MCHP_DONE=\"\$_done\" setsid sh \"\$0\" $ORIG_ARGS > \"\$_log\" 2>&1 &"
    else
        eval "MCHP_DETACHED=1 MCHP_DONE=\"\$_done\" nohup sh \"\$0\" $ORIG_ARGS > \"\$_log\" 2>&1 &"
    fi
    echo "  pid   : $!"
    exit 0
fi

# ── Check prerequisites ───────────────────────────────────────────────────────
# A dry-run only prints the plan, so it needs neither the app nor the camera.
if [ "$DRY_RUN" -eq 0 ]; then
    if ! command -v mchpcam-still > /dev/null 2>&1; then
        echo "ERROR: mchpcam-still not found" >&2
        echo "  Deploy libcamera-mchp to the board first." >&2
        exit 1
    fi

    if ! ls /dev/video0 /dev/media0 > /dev/null 2>&1; then
        echo "ERROR: No camera devices found (/dev/video0, /dev/media0)" >&2
        echo "  Check: dmesg | grep -E 'imx219|isc|EREMOTEIO'" >&2
        exit 1
    fi
fi

# ── Prepare output directory ──────────────────────────────────────────────────
mkdir -p "$OUTDIR"

# ── Invariant filename tags ───────────────────────────────────────────────────
# CT and lux are fixed for the whole invocation; exposure, gain and the _gN_fM
# sweep indices are per-frame and appended inside capture_one.
CT_TAG="ct${CCT}K"
LUX_TAG=""
[ -n "$LUX" ] && LUX_TAG="_lux${LUX}"

# ── capture_one: capture, name and validate one frame ─────────────────────────
# Args: req_gain gain_index frame_index total_gains total_frames do_validate
#       [lock_exp_us]
# Captures a single frame at the requested analogue-gain lock, parses the
# metadata block from mchpcam-still's stdout, writes the calibration sidecar
# (with gain_index/frame_index and a per-channel noise_stats block), and — on the
# representative frame of each gain — runs the capture-quality validation. With
# --dry-run it prints the planned command and filename and returns without
# touching the camera.
#
# lock_exp_us (optional): when set, the shutter is fixed to this exposure (µs)
# instead of letting AE re-converge. The loop passes frame 0's AE-settled value
# to the sibling repeats so every frame at a gain point shares one operating
# point — temporal variance then measures sensor noise, not AE jitter. On
# return, the measured exposure is left in the global LAST_EXP_US.
capture_one() {
    req_gain="$1"
    gain_index="$2"
    frame_index="$3"
    total_gains="$4"
    total_frames="$5"
    do_validate="$6"
    lock_exp_us="${7:-}"

    # Per-frame timestamp: provenance plus filename uniqueness inside a sweep
    # (the _gN_fM tokens guarantee uniqueness even at sub-second capture rates).
    TS=$(date +%Y%m%d_%H%M%S)

    raw_tok=""
    ext="jpg"
    if [ "$RAW" -eq 1 ]; then
        raw_tok="_raw"
        ext="pgm"
    fi

    # Temp capture path, unique per (gain,frame) so a fast sweep never collides.
    TMP_OUT="${OUTDIR}/_calib_tmp_g${gain_index}_f${frame_index}.${ext}"

    # ── Build the mchpcam-still command for this frame's gain lock ────────────
    MCHP_CMD="mchpcam-still"
    [ "$NOIR" -eq 1 ] && MCHP_CMD="$MCHP_CMD --noir"
    [ "$RAW"  -eq 1 ] && MCHP_CMD="$MCHP_CMD --raw"
    MCHP_CMD="$MCHP_CMD --width=$WIDTH --height=$HEIGHT"
    MCHP_CMD="$MCHP_CMD --timeout=$TIMEOUT"
    # Lock analogue gain for calibration (unity by default). Manual gain mode
    # fixes the gain knob while AE keeps adjusting exposure to reach the target.
    if [ "$req_gain" != "auto" ]; then
        MCHP_CMD="$MCHP_CMD --gain-mode=manual --analoggain=$req_gain"
    fi
    # Sibling repeats reuse frame 0's converged exposure so the whole gain point
    # sits at one operating point (manual shutter also engages manual exposure
    # mode). Frame 0 has no lock and lets AE settle.
    if [ -n "$lock_exp_us" ] && [ "$lock_exp_us" -gt 0 ] 2>/dev/null; then
        MCHP_CMD="$MCHP_CMD --shutter=$lock_exp_us"
    fi
    # IscHistogramBins (512-bin full histogram per channel) rides alongside
    # IscHistogramMeans unconditionally — no special flag needed.
    MCHP_CMD="$MCHP_CMD --metadata --metadata-format=json"
    [ "$NO_AWB" -eq 1 ] && MCHP_CMD="$MCHP_CMD --no-awb"
    MCHP_CMD="$MCHP_CMD -o $TMP_OUT"

    echo ""
    echo "[gain ${req_gain} ($((gain_index + 1))/${total_gains})  frame $((frame_index + 1))/${total_frames}]  ${TARGET} at ${CCT}K${LUX_TAG:+ lux=${LUX}}"

    if [ "$TARGET" = "flat_field" ]; then
        stem_base="flat_field_alsc_${CT_TAG}${LUX_TAG}"
    else
        stem_base="${TARGET}_${CT_TAG}${LUX_TAG}"
    fi

    if [ "$DRY_RUN" -eq 1 ]; then
        echo "  would run : $MCHP_CMD"
        echo "  would save: ${stem_base}_exp<us>us_gain${req_gain}_g${gain_index}_f${frame_index}_${TS}${raw_tok}.${ext} (+ .json)"
        return 0
    fi

    STDOUT_TMP="${OUTDIR}/_calib_stdout_g${gain_index}_f${frame_index}.txt"

    # Run capture — local process, no SSH, no timeout risk.
    # Show output live via tee while keeping a copy for metadata parsing, and
    # recover mchpcam-still's own exit status through the pipe. PIPESTATUS is a
    # bashism; /bin/sh on the board is busybox ash, where it expands to nothing.
    # fd4 carries tee's live output to the terminal; fd3 carries the producer's
    # exit code out of the pipeline into the command substitution.
    set +e
    exec 4>&1
    CAPTURE_RC=$({ { sh -c "$MCHP_CMD" 2>&1; echo $? >&3; } | tee "$STDOUT_TMP" >&4; } 3>&1)
    exec 4>&-
    set -e

    SAVED_MSG="Saved JPEG"
    [ "$RAW" -eq 1 ] && SAVED_MSG="Saved PGM"
    if [ "$CAPTURE_RC" -ne 0 ] || ! grep -q "$SAVED_MSG" "$STDOUT_TMP"; then
        echo ""
        echo "ERROR: capture failed (exit code $CAPTURE_RC)" >&2
        echo "  Check: dmesg | grep -E 'imx219|stream|EREMOTEIO'" >&2
        rm -f "$STDOUT_TMP" "$TMP_OUT"
        return 1
    fi

    # ── Parse metadata from captured stdout ───────────────────────────────────
    # Use Python to robustly parse the multi-line JSON block from stdout.
    # Shell grep on multi-line JSON is fragile; Python handles it correctly.
    SIDECAR_DATA=$(python3 - "$STDOUT_TMP" << 'PYEOF'
import sys, json, re

stdout = open(sys.argv[1]).read()

# Extract last complete JSON block between { and } at column 0
json_buf = []
in_json  = False
last_meta = None
for line in stdout.splitlines():
    stripped = line.strip()
    if not in_json and stripped == '{':
        in_json  = True
        json_buf = [line]
    elif in_json:
        json_buf.append(line)
        if stripped == '}':
            in_json = False
            try:
                last_meta = json.loads('\n'.join(json_buf))
            except json.JSONDecodeError:
                pass
            json_buf = []

if last_meta is None:
    print('{}')
    sys.exit(0)

out = {
    'exposure_us':    last_meta.get('ExposureTime',      0),
    'analogue_gain':  last_meta.get('AnalogueGain',      1.0),
    'awb_locked':     last_meta.get('AwbLocked',         False),
    'ae_converged':   int(last_meta.get('AeState', 0)) == 2,
    'ipa_ct_smoothed': last_meta.get('ColourTemperature', 0),
}

# Negotiated pixel format from the app's config summary. The raw Bayer order
# rides on hflip (SRGGB10 with no flip, SGRBG10 with hflip=1), so the loader
# must read the real format rather than assume one.
fmt = re.search(r'Camera configured:\s+(\S+)', stdout)
out['pixel_format'] = fmt.group(1) if fmt else ''

# IscHistogramMeans: [GR, R, GB, B]
means = last_meta.get('IscHistogramMeans')
if means and len(means) == 4:
    out['hist_means'] = {'GR': means[0], 'R': means[1], 'GB': means[2], 'B': means[3]}

# IscHistogramBins: 2048 int32 values [GR[0..511], R[0..511], GB[0..511], B[0..511]]
# Store as 4 separate arrays for easier access by the calibration tool.
bins_flat = last_meta.get('IscHistogramBins')
if bins_flat and len(bins_flat) == 2048:
    out['hist_bins'] = {
        'GR': bins_flat[0:512],
        'R':  bins_flat[512:1024],
        'GB': bins_flat[1024:1536],
        'B':  bins_flat[1536:2048],
    }

print(json.dumps(out))
PYEOF
)
    rm -f "$STDOUT_TMP"

    # Parse key values for filename tagging (measured, not requested).
    EXP_US=$(echo "$SIDECAR_DATA" | python3 -c "import sys,json; d=json.load(sys.stdin); print(int(d.get('exposure_us',0)))")
    # Publish the measured exposure so the loop can lock sibling repeats to it.
    LAST_EXP_US="$EXP_US"
    ACT_GAIN=$(echo "$SIDECAR_DATA" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('analogue_gain',1.0))")

    # Build final filename. A flat field carries an 'alsc' token so the tuner's
    # loader selects it as LSC-only and skips Macbeth detection. The _gN_fM
    # tokens tag the sweep position for humans; tools read gain_index/frame_index
    # from the sidecar.
    STEM="$stem_base"
    [ -n "$EXP_US" ]   && STEM="${STEM}_exp${EXP_US}us"
    [ -n "$ACT_GAIN" ] && STEM="${STEM}_gain${ACT_GAIN}"
    STEM="${STEM}_g${gain_index}_f${frame_index}_${TS}${raw_tok}"

    FINAL_OUT="${OUTDIR}/${STEM}.${ext}"
    FINAL_JSON="${OUTDIR}/${STEM}.json"

    mv "$TMP_OUT" "$FINAL_OUT"

    # Write the calibration sidecar JSON (this is what microchip_isc_lt.py reads)
    # CRITICAL: true_ct is always from the user (lightbox dial), NEVER from IPA.
    # Merge capture-time metadata (from SIDECAR_DATA) with provenance fields.
    python3 - "$FINAL_JSON" "$CCT" "${LUX:-0}" "$WIDTH" "$HEIGHT" "$SIDECAR_DATA" "$RAW" "$gain_index" "$frame_index" << 'PYEOF'
import sys, json

out_path    = sys.argv[1]
true_ct     = float(sys.argv[2])
lux         = float(sys.argv[3])
width       = int(sys.argv[4])
height      = int(sys.argv[5])
meta_json   = sys.argv[6]
is_raw      = sys.argv[7] == '1' if len(sys.argv) > 7 else False
gain_index  = int(sys.argv[8]) if len(sys.argv) > 8 else 0
frame_index = int(sys.argv[9]) if len(sys.argv) > 9 else 0

try:
    meta = json.loads(meta_json)
except Exception:
    meta = {}

# Bayer fourcc as the app negotiated it (parsed from its config summary);
# default to SRGGB10, the --raw forced format, if the summary was absent.
pixel_format = meta.get('pixel_format') or 'SRGGB10'

sidecar = {
    'width':            width,
    'height':           height,
    'format':           f'PGM_{pixel_format}' if is_raw else 'JPEG',
    'raw':              is_raw,
    # Ground truth from operator — NEVER from IPA estimate
    'true_ct':          true_ct,
    'lux':              lux,
    'exposure_us':      meta.get('exposure_us',    0),
    'analogue_gain':    meta.get('analogue_gain',  1.0),
    'awb_locked':       meta.get('awb_locked',     False),
    'ae_converged':     meta.get('ae_converged',   False),
    'ipa_ct_smoothed':  meta.get('ipa_ct_smoothed', 0),
    # Sweep coordinates: which gain step and which repeat this frame is. The
    # noise tool groups frames by (true_ct, gain_index) and measures temporal
    # variance across the frame_index repeats; the AWB/CCM/lux solves ignore
    # them and use any one frame.
    'gain_index':       gain_index,
    'frame_index':      frame_index,
}

# Per-channel histogram means (4 floats — fast path for calibration tool)
if 'hist_means' in meta:
    sidecar['hist_means'] = meta['hist_means']

# Full histogram bins (4 × 512 uint32 — precision path)
# hist_bins['GR'][0..511], ['R'][0..511], ['GB'][0..511], ['B'][0..511]
if 'hist_bins' in meta:
    sidecar['hist_bins'] = meta['hist_bins']

# Per-channel quick-look noise stats from the histogram: mean and variance in
# bin units (0..511, each bin = raw16 >> 7, i.e. pixel10 >> 1). This is a
# WHOLE-FRAME statistic, so on a flat field it also carries spatial shading —
# the noise tool uses it as a cross-check and derives the real read/shot terms
# from temporal per-pixel variance across the frame_index repeats.
#
# The raw bins carry the sensor pedestal (IMX219 RAW10 black level 64 = 32 bins);
# subtract it from the mean so 'mean' is the net signal on the same scale as
# hist_means (a shot-noise var-vs-signal fit reads mean as signal). Variance is
# invariant to the constant offset, so it is left as-is.
PEDESTAL_BINS = 64 >> 1
def _chan_stats(counts):
    n = 0
    s1 = 0.0
    s2 = 0.0
    for i, c in enumerate(counts):
        n += c
        s1 += i * c
        s2 += (i * i) * c
    if n <= 0:
        return None
    mean = s1 / n
    var = s2 / n - mean * mean
    return {'mean': mean - PEDESTAL_BINS, 'var': var if var > 0.0 else 0.0,
            'n': n, 'pedestal_bins': PEDESTAL_BINS}

if 'hist_bins' in meta:
    ns = {}
    for ch in ('GR', 'R', 'GB', 'B'):
        st = _chan_stats(meta['hist_bins'].get(ch, []))
        if st is not None:
            ns[ch] = st
    if ns:
        sidecar['noise_stats'] = ns

with open(out_path, 'w') as f:
    json.dump(sidecar, f, indent=2)
print(f'Sidecar: hist_means={bool("hist_means" in sidecar)} '
      f'hist_bins={bool("hist_bins" in sidecar)} '
      f'noise_stats={bool("noise_stats" in sidecar)}')
PYEOF

    if [ "$do_validate" -ne 1 ]; then
        echo "  saved frame $((frame_index + 1))/${total_frames} at gain ${req_gain} (validated on frame 1 of this gain)"
        echo "  -> $FINAL_OUT"
        return 0
    fi

    # ── Post-capture validation ───────────────────────────────────────────────
    # Run on-board quality check on the representative frame of each gain.
    # For grey_card:   verify R/G and B/G channel ratios are within ±30% of 1.0
    #                  and mean luminance is in [15%, 85%] of histogram range.
    # For colorchecker: verify frame is not clipped or completely dark.
    # Uses histogram means from the JSON sidecar — no image decoding needed.
    python3 - "$FINAL_JSON" "$TARGET" "$CCT" "$NOIR" "$req_gain" << 'VALEOF'
import sys, json, math

json_path = sys.argv[1]
target    = sys.argv[2]
cct       = int(sys.argv[3])
is_noir   = sys.argv[4] == '1' if len(sys.argv) > 4 else False
req_gain  = sys.argv[5] if len(sys.argv) > 5 else '1.0'

try:
    with open(json_path) as f:
        s = json.load(f)
except Exception as e:
    print(f'  WARN: could not read sidecar for validation: {e}')
    sys.exit(0)

# ── Exposure / AE convergence ──────────────────────────────────────────────
ae_ok = bool(s.get('ae_converged', False))
awb_ok = bool(s.get('awb_locked', False))
if not ae_ok:
    print('  WARN: AE not converged at capture time. '
          'Re-capture with a longer warmup (--timeout 90).')
if not awb_ok and target == 'grey_card':
    print('  WARN: AWB not locked at capture time. '
          'Re-capture; AWB stability is required for reference_gains calibration.')

# ── Histogram means check ──────────────────────────────────────────────────
hist = s.get('hist_means')
if not hist:
    print('  WARN: No histogram means in sidecar. '
          'Validation skipped. Ensure mchpcam-still --metadata is set.')
    sys.exit(0)

r  = float(hist.get('R',  0))
gr = float(hist.get('GR', 0))
gb = float(hist.get('GB', 0))
b  = float(hist.get('B',  0))
mean_lum = 0.299 * r + 0.587 * ((gr + gb) / 2) + 0.114 * b

PASS = '\033[32mPASS\033[0m'
WARN = '\033[33mWARN\033[0m'
FAIL = '\033[31mFAIL\033[0m'
ok = True

# Luminance check: [15, 85]% of 511 bins
lum_lo, lum_hi = 511 * 0.15, 511 * 0.85
if mean_lum < lum_lo:
    print(f'  {FAIL} Luminance {mean_lum:.1f} bins < {lum_lo:.0f} (too dark). '
          f'Increase lightbox intensity or extend exposure (--timeout 90).')
    ok = False
elif mean_lum > lum_hi:
    print(f'  {FAIL} Luminance {mean_lum:.1f} bins > {lum_hi:.0f} (clipping). '
          f'Reduce lightbox intensity or add ND filter.')
    ok = False
else:
    print(f'  {PASS} Luminance {mean_lum:.1f} bins  (range 0-511)')

# ── Gain-lock check ──────────────────────────────────────────────────────────
# Each frame must sit at its requested gain: a lock that did not take (AGC still
# driving the gain) invalidates that sweep point. For a gain sweep this also
# confirms the sweep axis is real (each step lands where it was asked to).
if req_gain != 'auto':
    actual_gain = float(s.get('analogue_gain', 0) or 0)
    want_gain = float(req_gain)
    if abs(actual_gain - want_gain) > 0.05:
        print(f'  {FAIL} Analogue gain {actual_gain:.2f}x != requested '
              f'{want_gain:.2f}x. Manual gain lock did not take — verify '
              f'mchpcam-still --gain-mode=manual is honoured on this build.')
        ok = False
    else:
        print(f'  {PASS} Analogue gain {actual_gain:.2f}x (locked)')

# ── Per-bin highlight-clipping check (the real non-clipped gate) ─────────────
# The mean can sit mid-range while a chart's white patch or a bright corner
# clips. Inspect the full per-channel histogram: the fraction of samples in the
# top bins is the direct saturation measure. Bins are pre-WB raw values (0..511)
# so the top bin is genuine sensor saturation, independent of WB/CCM.
bins = s.get('hist_bins')
if bins:
    CLIP_BIN = 505  # samples in bins >= 505 (of 0..511) count as clipped
    # A Macbeth white patch legitimately occupies a few percent of the frame;
    # grey/flat fields should be essentially clip-free.
    clip_limit = 0.010 if target == 'colorchecker' else 0.002
    for ch in ('GR', 'R', 'GB', 'B'):
        cbins = bins.get(ch) or []
        total = sum(cbins)
        if total <= 0:
            continue
        clipped = sum(cbins[CLIP_BIN:]) / total
        if clipped > clip_limit:
            print(f'  {FAIL} {ch} clipping: {clipped * 100:.2f}% of samples in '
                  f'bins >= {CLIP_BIN} (limit {clip_limit * 100:.2f}%). '
                  f'Reduce lightbox intensity or add an ND filter.')
            ok = False
        else:
            print(f'  {PASS} {ch} clip {clipped * 100:.2f}% '
                  f'(<= {clip_limit * 100:.2f}%)')
else:
    print(f'  {WARN} No histogram bins in sidecar — per-bin clip check skipped '
          f'(mean-based luminance check still applied).')

if target == 'grey_card':
    # Grey-card flatness checks.
    # Pre-WB histogram — gains not applied yet — ratios vary by CCT.
    # NoIR sensors (no IR cut filter) have NIR energy adding to all channels
    # but especially R, suppressing B/GR at warm CCT. Thresholds are relaxed
    # for NoIR to avoid false positives on physically correct measurements.
    if gr > 1e-3:
        r_ratio  = r  / gr
        b_ratio  = b  / gr
        gb_ratio = gb / gr

        # GR/GB check: both green channels, same spectral response.
        # This is the most reliable BLC health indicator — NIR affects GR
        # and GB nearly equally so GR/GB ≈ 1.0 even on NoIR sensors.
        if abs(gb_ratio - 1.0) > 0.12:
            print(f'  {FAIL} GR/GB={1/gb_ratio:.3f} deviates >{12}% from 1.0 '
                  f'(GR={gr:.1f} GB={gb:.1f}). '
                  f'Likely per-channel BLC error or non-neutral scene.')
            ok = False
        else:
            print(f'  {PASS} GR/GB={1/gb_ratio:.3f}  (should be ~1.0)')

        # R/GR: plausible range for all sensors and CCTs
        if r_ratio < 0.3 or r_ratio > 3.5:
            print(f'  {FAIL} R/GR={r_ratio:.3f} out of plausible range [0.3, 3.5]. '
                  f'Verify grey card is neutral and scene has single illuminant.')
            ok = False
        else:
            print(f'  {PASS} R/GR={r_ratio:.3f}')

        # B/GR: NoIR sensors have low B at warm CCT due to NIR contamination
        # in R and G channels. Standard lower bound 0.4 is a false positive.
        # NoIR lower bound: 0.15 (covers bGain up to ~6.5× which is physically
        # extreme but possible at 2700K on a sensor with no IR cut filter).
        b_lo = 0.15 if is_noir else 0.4
        if b_ratio < b_lo or b_ratio > 2.5:
            print(f'  {FAIL} B/GR={b_ratio:.3f} out of plausible range [{b_lo}, 2.5].')
            ok = False
        else:
            print(f'  {PASS} B/GR={b_ratio:.3f}')

        # CCT sanity: warm → R/GR > 1 (even on NoIR, R is elevated at warm CCT)
        # Cool → B/GR rising. For NoIR the CCT threshold for B/GR is lower
        # because NIR suppresses the apparent blue channel even under cool light.
        if cct < 3500 and r_ratio < 0.7:
            print(f'  {WARN} At {cct}K (warm) expected R/GR > 0.7 but got {r_ratio:.3f}. '
                  f'Check lightbox CCT setting.')
        # NoIR: B/GR > 0.5 at cool CCT is acceptable (vs 0.8 for standard sensor)
        cool_b_threshold = 0.5 if is_noir else 0.8
        if cct > 5500 and b_ratio < cool_b_threshold:
            print(f'  {WARN} At {cct}K (cool) expected B/GR > {cool_b_threshold} '
                  f'but got {b_ratio:.3f}.'
                  + (' (NoIR — NIR suppresses blue channel)' if is_noir else
                     ' Check lightbox CCT setting.'))

if ok:
    print(f'  Capture quality: OK  (CT={cct}K target={target})')
else:
    print(f'  Capture quality: FAIL — review warnings above before proceeding.')
    print(f'  Re-capture: /tmp/mchp_capture.sh -t {target} -c {cct} [options]')
VALEOF

    echo "  -> $FINAL_OUT"
    echo "     $FINAL_JSON"
}

# ── Run the capture matrix: gains × frames ────────────────────────────────────
# Count the gain axis for progress display.
NGAINS=0
for _g in $GAIN_LIST; do
    NGAINS=$((NGAINS + 1))
done

echo ""
if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY RUN — no camera access. Capture plan for ${TARGET} at ${CCT}K:"
else
    echo "Capturing ${TARGET} at ${CCT}K${LUX_TAG:+ (lux=${LUX})}"
fi
echo "  gain axis     : ${GAIN_LIST}"
echo "  frames / gain : ${N_FRAMES}"
echo "  total frames  : $((NGAINS * N_FRAMES))"

gain_index=0
for g in $GAIN_LIST; do
    frame_index=0
    # Cleared each gain point: frame 0 runs AE-auto and republishes it, so a
    # failed frame 0 never leaks the previous gain's exposure into the repeats.
    LAST_EXP_US=""
    while [ "$frame_index" -lt "$N_FRAMES" ]; do
        # Validate only the first frame of each gain: the remaining repeats are
        # siblings for temporal noise and share the same operating point.
        do_validate=0
        [ "$frame_index" -eq 0 ] && do_validate=1
        # Frame 0 lets AE settle (no lock); siblings reuse its exposure so the
        # temporal variance reflects sensor noise rather than AE re-convergence.
        lock_exp=""
        [ "$frame_index" -ne 0 ] && lock_exp="$LAST_EXP_US"
        capture_one "$g" "$gain_index" "$frame_index" "$NGAINS" "$N_FRAMES" "$do_validate" "$lock_exp"
        frame_index=$((frame_index + 1))
    done
    gain_index=$((gain_index + 1))
done

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY RUN complete — no files written. Drop --dry-run to capture."
else
    echo "Captured $((NGAINS * N_FRAMES)) frame(s) to ${OUTDIR}"
    echo ""
    echo "When all CCT points are captured, copy to host:"
    echo "  scp 'root@<board-ip>:${OUTDIR}/*.jpg' 'root@<board-ip>:${OUTDIR}/*.pgm' 'root@<board-ip>:${OUTDIR}/*.json' /path/to/calib_images/"
fi
echo ""
