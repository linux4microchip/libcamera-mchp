#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
#
# mchp_capture.sh — Calibration image capture for Microchip ISC IPA tuning
#
# Run this script DIRECTLY ON THE BOARD (via serial console or local terminal).
# Do NOT run it over SSH — the SSH timeout/disconnect path causes mchpcam-still
# to receive SIGKILL or SIGHUP, leaving the sensor in an unrecoverable state.
#
# After capture, copy the output files to the host:
#   scp root@<board-ip>:/tmp/calib/*.jpg /path/to/calib_images/
#   scp root@<board-ip>:/tmp/calib/*.json /path/to/calib_images/
# Then run:
#   python3 utils/tuning/microchip_isc.py \
#       -i /path/to/calib_images/ \
#       -o src/ipa/microchip-isc/data/imx219_noir.yaml \
#       --modules ct awb lux ccm recalc --noir
#
# Usage:
#   mchp_capture.sh [OPTIONS]
#
# Options:
#   -t, --target <type>   Target: grey_card (default) | colorchecker
#   -c, --cct <K>         Lightbox-rated colour temperature in Kelvin (required)
#   -l, --lux <N>         Incident illuminance from handheld lux meter (optional)
#   -o, --outdir <dir>    Output directory (default: /tmp/calib)
#   -r, --resolution WxH  Capture resolution (default: 1920x1080)
#       --no-awb          Disable AWB (use for colorchecker captures)
#       --noir            Use NoIR tuning file (imx219_noir.yaml)
#   -h, --help            Show this help
#
# Filename convention (parsed by microchip_isc.py):
#   grey_card_ct2850K_lux2869_exp4500us_gain1.00_20260420_182043.jpg
#   colorchecker_ct4450K_lux2939_exp4200us_gain1.00_20260420_183012.jpg
#
# Examples:
#   # Grey card at 2850K, lux meter reads 2869:
#   mchp_capture.sh -t grey_card -c 2850 -l 2869 --noir
#
#   # Macbeth chart at 4450K (AWB disabled):
#   mchp_capture.sh -t colorchecker -c 4450 -l 2939 --no-awb --noir
#
#   # All three CCT points in sequence:
#   for CCT in 2850 4450 6400; do
#       echo "Set lightbox to ${CCT}K, press Enter when ready"
#       read _
#       mchp_capture.sh -t grey_card -c $CCT --noir
#   done
# =============================================================================

set -e

# ── Defaults ──────────────────────────────────────────────────────────────────
TARGET="grey_card"
CCT=""
LUX=""
OUTDIR="/tmp/calib"
WIDTH=1920
HEIGHT=1080
NO_AWB=0
NOIR=0
TIMEOUT=60
# Number of frames to capture per CCT point.
# Multiple frames are averaged by the calibration tool for better SNR.
# Recommended: 3 for routine calibration, 5 for production.
N_FRAMES=3

# ── Argument parsing ──────────────────────────────────────────────────────────
while [ $# -gt 0 ]; do
    case "$1" in
        -t|--target)    TARGET="$2";  shift 2 ;;
        -c|--cct)       CCT="$2";     shift 2 ;;
        -l|--lux)       LUX="$2";     shift 2 ;;
        -o|--outdir)    OUTDIR="$2";  shift 2 ;;
        -r|--resolution)
            WIDTH="${2%%x*}"
            HEIGHT="${2##*x}"
            shift 2 ;;
        -n|--frames)    N_FRAMES="$2"; shift 2 ;;
        --no-awb)       NO_AWB=1;     shift   ;;
        --noir)         NOIR=1;       shift   ;;
        --timeout)      TIMEOUT="$2"; shift 2 ;;
        -h|--help)
            sed -n '3,50p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1 ;;
    esac
done

# ── Validate ──────────────────────────────────────────────────────────────────
if [ -z "$CCT" ]; then
    echo "ERROR: --cct <K> is required (lightbox-rated colour temperature)" >&2
    echo "  Example: mchp_capture.sh -c 4450 -l 2939 --noir" >&2
    exit 1
fi

case "$TARGET" in
    grey_card|colorchecker) ;;
    *)
        echo "ERROR: --target must be grey_card or colorchecker" >&2
        exit 1 ;;
esac

# Integer check for CCT
if ! echo "$CCT" | grep -qE '^[0-9]+$'; then
    echo "ERROR: --cct must be a positive integer, got: $CCT" >&2
    exit 1
fi

# ── Check prerequisites ───────────────────────────────────────────────────────
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

# ── Prepare output directory ──────────────────────────────────────────────────
mkdir -p "$OUTDIR"

# ── Build filename stem ───────────────────────────────────────────────────────
TS=$(date +%Y%m%d_%H%M%S)
CT_TAG="ct${CCT}K"
LUX_TAG=""
[ -n "$LUX" ] && LUX_TAG="_lux${LUX}"

# Temporary capture path (metadata not yet known)
REMOTE_JPEG="${OUTDIR}/_calib_tmp.jpg"
REMOTE_META="${OUTDIR}/_calib_tmp.json"

# ── Build mchpcam-still command ───────────────────────────────────────────────
MCHP_CMD="mchpcam-still"
[ "$NOIR" -eq 1 ] && MCHP_CMD="$MCHP_CMD --noir"
MCHP_CMD="$MCHP_CMD --width=$WIDTH --height=$HEIGHT"
MCHP_CMD="$MCHP_CMD --timeout=$TIMEOUT"
# IscHistogramBins (512-bin full histogram per channel) is emitted
# unconditionally alongside IscHistogramMeans — no special flag needed.
MCHP_CMD="$MCHP_CMD --metadata --metadata-format=json"
[ "$NO_AWB" -eq 1 ] && MCHP_CMD="$MCHP_CMD --no-awb"
MCHP_CMD="$MCHP_CMD -o $REMOTE_JPEG"

# ── Capture N_FRAMES frames ───────────────────────────────────────────────────
# Single capture per CCT point — simple and reliable.
# N_FRAMES > 1 is not used (burst mode had timeout issues with --capture=N).
# The calibration tool averages across multiple CCT captures instead.
echo ""
echo "Capturing: $TARGET at ${CCT}K${LUX_TAG:+ lux=${LUX}}"
echo ""

STDOUT_TMP="${OUTDIR}/_calib_stdout.txt"

# Run capture — local process, no SSH, no timeout risk.
# Redirect stdout to temp file AND show on terminal via tee.
# Use sh -c to avoid set -e propagating through the pipe.
set +e
sh -c "$MCHP_CMD" 2>&1 | tee "$STDOUT_TMP"
CAPTURE_RC=${PIPESTATUS[0]:-$?}
set -e

if [ "$CAPTURE_RC" -ne 0 ] || ! grep -q "Saved JPEG" "$STDOUT_TMP"; then
    echo ""
    echo "ERROR: capture failed (exit code $CAPTURE_RC)" >&2
    echo "  Check: dmesg | grep -E 'imx219|stream|EREMOTEIO'" >&2
    rm -f "$STDOUT_TMP"
    exit 1
fi

# ── Parse metadata from captured stdout ──────────────────────────────────────
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

# Parse key values for filename tagging
EXP_US=$(echo "$SIDECAR_DATA" | python3 -c "import sys,json; d=json.load(sys.stdin); print(int(d.get('exposure_us',0)))")
GAIN=$(echo "$SIDECAR_DATA"   | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('analogue_gain',1.0))")

# Build final filename
STEM="${TARGET}_${CT_TAG}${LUX_TAG}"
[ -n "$EXP_US" ] && STEM="${STEM}_exp${EXP_US}us"
[ -n "$GAIN"   ] && STEM="${STEM}_gain${GAIN}"
STEM="${STEM}_${TS}"

FINAL_JPEG="${OUTDIR}/${STEM}.jpg"
FINAL_JSON="${OUTDIR}/${STEM}.json"

# ── Move captured files to final names ───────────────────────────────────────
mv "$REMOTE_JPEG" "$FINAL_JPEG"

# Write the calibration sidecar JSON (this is what microchip_isc.py reads)
# CRITICAL: true_ct is always from the user (lightbox dial), NEVER from IPA.
# Merge capture-time metadata (from SIDECAR_DATA) with provenance fields.
python3 - "$FINAL_JSON" "$CCT" "${LUX:-0}" "$WIDTH" "$HEIGHT" "$SIDECAR_DATA" << 'PYEOF'
import sys, json

out_path  = sys.argv[1]
true_ct   = float(sys.argv[2])
lux       = float(sys.argv[3])
width     = int(sys.argv[4])
height    = int(sys.argv[5])
meta_json = sys.argv[6]

try:
    meta = json.loads(meta_json)
except Exception:
    meta = {}

sidecar = {
    'width':            width,
    'height':           height,
    'format':           'JPEG',
    # Ground truth from operator — NEVER from IPA estimate
    'true_ct':          true_ct,
    'lux':              lux,
    'exposure_us':      meta.get('exposure_us',    0),
    'analogue_gain':    meta.get('analogue_gain',  1.0),
    'awb_locked':       meta.get('awb_locked',     False),
    'ae_converged':     meta.get('ae_converged',   False),
    'ipa_ct_smoothed':  meta.get('ipa_ct_smoothed', 0),
}

# Per-channel histogram means (4 floats — fast path for calibration tool)
if 'hist_means' in meta:
    sidecar['hist_means'] = meta['hist_means']

# Full histogram bins (4 × 512 uint32 — precision path)
# hist_bins['GR'][0..511], ['R'][0..511], ['GB'][0..511], ['B'][0..511]
# Used by awb_gains.py for percentile-based gain computation and
# by ccm.py for patch spectral statistics without requiring RAW DMA.
if 'hist_bins' in meta:
    sidecar['hist_bins'] = meta['hist_bins']

with open(out_path, 'w') as f:
    json.dump(sidecar, f, indent=2)
print(f'Sidecar: hist_means={bool("hist_means" in sidecar)} '
      f'hist_bins={bool("hist_bins" in sidecar)}')
PYEOF

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "Saved: $FINAL_JPEG"
echo "       $FINAL_JSON"
echo ""
echo "When all CCT points are captured, copy to host:"
echo "  scp root@<board-ip>:${OUTDIR}/*.jpg root@<board-ip>:${OUTDIR}/*.json /path/to/calib_images/"
echo ""
