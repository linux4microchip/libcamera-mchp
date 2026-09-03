#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2026, Microchip Technology Inc.
#
# Macbeth framing assistant for ISC calibration captures.
#
# Locates a ColorChecker chart with the tuner's own detector and QA gate
# (libtuning.macbeth, libtuning.macbeth_qa) and writes an annotated preview: the
# detected 6x4 grid, a confidence score and a PASS/WARN/FAIL verdict. Sharing
# the calibration code path keeps this verdict and the solver's from diverging.
#
# Modes:
#   --local FILE     analyse an existing capture (.pgm raw, or .jpg/.png)
#   --capture        run a capture on the target over SSH and pull it back
#   --stream PIPE    analyse a live stream from the target (framing only)
#   --watch          repeat the chosen mode on an interval until Ctrl-C
#
# --capture needs key-based SSH to the target and a --capture-cmd that writes a
# raw .pgm (optionally a matching .json sidecar) into its {outdir}. It defaults
# to a single-frame mchpcam-still grab for framing; the full calibration sweep
# (mchp_capture.sh) must run locally on the target, never over SSH, so it is not
# the default here.
#
# --stream takes the receive/decode half of a gst-launch line (up to a raw-video
# pad); the BGR appsink tail is appended here. The target streams with
# libcamerasrc, this host only decodes. No RAW, so framing only, no QA. E.g.
#   target: gst-launch-1.0 libcamerasrc ! video/x-raw,width=1280,height=720 \
#             ! jpegenc ! rtpjpegpay ! udpsink host=<host> port=5000
#   host:   --stream 'udpsrc port=5000 caps=application/x-rtp,encoding-name=JPEG \
#             ! rtpjpegdepay ! jpegdec'
#
# FRAMING (geometry: located, square, confident) is reported for any input; QA
# (clipping, tilt, uneven light) only for a RAW .pgm, n/a for a processed JPEG.

import argparse
import logging
import os
import subprocess
import sys
import time
import warnings
from pathlib import Path

import cv2
import numpy as np

# libtuning lives next to this script.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from libtuning.image import Image                         # noqa: E402
from libtuning.macbeth import get_macbeth_chart           # noqa: E402
from libtuning import macbeth_qa                           # noqa: E402

logging.basicConfig(format='%(message)s', level=logging.INFO)
logger = logging.getLogger('frame_assist')

# The detector correlates a warped candidate against a fixed reference. >=0.75
# is the tuner's "reliable placement" bar; below its 0.60 accept floor the
# detector returns no match at all. We mirror the 0.75 bar for the framing
# verdict so our advice matches what the solver will do with the frame.
_COR_GOOD = 0.75

# Tilt (perspective foreshortening of the located grid) thresholds, shared with
# the QA gate so the framing verdict and the QA verdict agree on squareness.
_TILT_WARN = macbeth_qa._TILT_WARN
_TILT_FAIL = macbeth_qa._TILT_FAIL

_N_COLS = macbeth_qa._N_COLS
_N_ROWS = macbeth_qa._N_ROWS
_N_PATCHES = _N_COLS * _N_ROWS

_RAW_SUFFIXES = ('.pgm',)
_IMG_SUFFIXES = ('.jpg', '.jpeg', '.png', '.bmp')

# BGR colours (OpenCV order).
_GREEN = (0, 255, 0)
_AMBER = (0, 200, 255)
_RED = (0, 0, 255)


# --------------------------------------------------------------------------
# Chart location
# --------------------------------------------------------------------------

def _build_ref_data():
    """Load the reference chart used to score a candidate match.

    Mirrors libtuning.macbeth.find_macbeth so the confidence we display is the
    same correlation the tuner computes.
    """
    ref_path = Path(__file__).resolve().parent / 'libtuning' / 'macbeth_ref.pgm'
    ref = cv2.imread(str(ref_path), flags=cv2.IMREAD_GRAYSCALE)
    if ref is None:
        raise FileNotFoundError(f'reference chart not found: {ref_path}')
    ref_w, ref_h = 120, 80
    ref_corns = np.array(((0, 0), (0, ref_h), (ref_w, ref_h), (ref_w, 0)),
                         np.float32)
    return (ref, ref_w, ref_h, ref_corns)


def locate(img, ref_data):
    """Locate the chart and return (confidence, fit_coords).

    fit_coords is (outer_corners, patch_centres) in the pixel space of ``img``,
    or None when no chart was found. For an 8-bit image a couple of brightness
    boosts are tried (as the tuner does) to rescue a dark frame; a float Bayer
    average is already contrast-normalised inside the detector, so one pass
    suffices there.

    The chart is expected to fill much of the frame during framing, so the
    tuner's small-chart subselection scan is intentionally skipped -- it is the
    slow part and least useful here (a chart too small to detect at full frame
    is itself the framing problem to report).
    """
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        cor, _, coords, ok = get_macbeth_chart(img, ref_data)

        if img.dtype == np.uint8:
            for boost in (2, 4):
                if cor >= _COR_GOOD:
                    break
                cand = get_macbeth_chart(cv2.convertScaleAbs(img, alpha=boost),
                                         ref_data)
                if cand[0] > cor:
                    cor, _, coords, ok = cand

    if not ok:
        return 0.0, None
    return float(cor), coords


def _tilt(centres):
    """Perspective foreshortening of the located centre grid, 0..1.

    Same geometry as macbeth_qa._tilt but computed straight from the located
    centres so it is available for a JPEG too (no Image needed). Returns None if
    the grid is not the expected 24-patch layout.
    """
    if centres is None or len(centres) != _N_PATCHES:
        return None
    pts = np.asarray(centres, dtype=np.float64)

    def g(col, row):
        return pts[col * _N_ROWS + row]

    def dist(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def asym(a, b):
        hi = max(a, b)
        return abs(a - b) / hi if hi > 1e-6 else 0.0

    left_h = dist(g(0, 0), g(0, _N_ROWS - 1))
    right_h = dist(g(_N_COLS - 1, 0), g(_N_COLS - 1, _N_ROWS - 1))
    top_w = dist(g(0, 0), g(_N_COLS - 1, 0))
    bot_w = dist(g(0, _N_ROWS - 1), g(_N_COLS - 1, _N_ROWS - 1))
    return max(asym(left_h, right_h), asym(top_w, bot_w))


# --------------------------------------------------------------------------
# Quality gate (RAW only)
# --------------------------------------------------------------------------

def _qa(image, coords):
    """(verdict_dict, clipped_indices) for a RAW capture, or (None, []).

    Populates image.patches from the located centres, then runs the tuner's QA
    gate for the PASS/WARN/FAIL verdict and separately lists which patches
    clipped (using the same rule the gate counts with) so they can be flagged on
    the overlay. Any geometry that trips the patch sampler degrades to
    (None, []) rather than aborting the preview.
    """
    if image is None or coords is None:
        return None, []
    try:
        image.get_patches(coords[1])
        verdict = macbeth_qa.qa_image(image)
        white = macbeth_qa._white_level(image)
        clipped = [p for p in range(image.patches.shape[1])
                   if any(image.patches[ci][p][-1] >= white for ci in range(4))]
        return verdict, clipped
    except Exception as e:
        logger.debug(f'QA unavailable: {e}')
        return None, []


# --------------------------------------------------------------------------
# Preview rendering
# --------------------------------------------------------------------------

def _raw_preview(image):
    """A viewable BGR preview from the linear Bayer planes.

    Built at Bayer-channel resolution (half the sensor width/height), which is
    exactly the coordinate space the detector runs in, so located coordinates
    overlay without rescaling. Black level is subtracted and a display gamma is
    applied purely so the operator can see the scene; the preview never feeds
    calibration.
    """
    ch = image.channels
    bl = getattr(image, 'blacklevel_16', 0) or 0
    r = ch[0].astype(np.float64)
    g = (ch[1].astype(np.float64) + ch[2].astype(np.float64)) / 2.0
    b = ch[3].astype(np.float64)

    def norm(x):
        x = np.clip(x - bl, 0, None)
        hi = np.percentile(x, 99.5)
        if hi <= 0:
            hi = 1.0
        x = np.clip(x / hi, 0, 1) ** (1 / 2.2)
        return (x * 255).astype(np.uint8)

    return cv2.merge([norm(b), norm(g), norm(r)])


def _load(path):
    """Return (preview_bgr, detector_input, image_or_None).

    detector_input is what locate() runs on: a float Bayer average for RAW, the
    8-bit BGR frame otherwise. image_or_None is the libtuning Image (RAW only),
    needed for the QA verdict.
    """
    suffix = path.suffix.lower()
    if suffix in _RAW_SUFFIXES:
        image = Image(path)
        if image.channels is None:
            raise ValueError(f'{path.name}: no Bayer data decoded')
        av = np.mean(np.array(image.channels), axis=0) / (2 ** 16)
        return _raw_preview(image), av, image

    if suffix in _IMG_SUFFIXES:
        bgr = cv2.imread(str(path), flags=cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f'{path.name}: could not be read as an image')
        return bgr, bgr, None

    raise ValueError(f'{path.name}: unsupported input (need .pgm or .jpg/.png)')


def _draw_grid(canvas, coords, clipped, colour, lw):
    """Draw the detected outer quad, 6x4 grid lines and patch centres.

    clipped is the list of patch indices whose brightest pixels saturated; those
    centres are ringed in red so the operator sees exactly what to darken.
    """
    outer = np.round(coords[0][0]).astype(np.int32)
    cv2.polylines(canvas, [outer], isClosed=True, color=colour,
                  thickness=lw * 2, lineType=cv2.LINE_AA)

    cen = np.round(coords[1][0]).astype(np.int32)
    if len(cen) != _N_PATCHES:
        return
    grid = cen.reshape(_N_COLS, _N_ROWS, 2)
    for c in range(_N_COLS):
        cv2.polylines(canvas, [np.ascontiguousarray(grid[c])], isClosed=False,
                      color=colour, thickness=lw, lineType=cv2.LINE_AA)
    for r in range(_N_ROWS):
        cv2.polylines(canvas, [np.ascontiguousarray(grid[:, r])],
                      isClosed=False, color=colour, thickness=lw,
                      lineType=cv2.LINE_AA)

    clipped = set(clipped or [])
    for idx, (x, y) in enumerate(cen):
        if idx in clipped:
            cv2.circle(canvas, (int(x), int(y)), lw * 4, _RED, lw + 1,
                       lineType=cv2.LINE_AA)
        else:
            cv2.circle(canvas, (int(x), int(y)), lw * 2, colour, -1,
                       lineType=cv2.LINE_AA)


def _banner(canvas, lines, colour, fs, lw):
    """Draw a status banner (list of text lines) top-left over a dim strip."""
    pad = int(10 * fs)
    heights = [cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, fs, lw)[0][1]
               for t in lines]
    strip_h = sum(heights) + pad * (len(lines) + 1)
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (canvas.shape[1], strip_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)
    y = pad
    for text, h in zip(lines, heights):
        y += h
        cv2.putText(canvas, text, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, fs,
                    colour, lw, cv2.LINE_AA)
        y += pad


def _framing_verdict(cor, tilt):
    """(colour, headline) for the geometry-only framing status."""
    if cor >= _COR_GOOD and tilt is not None and tilt < _TILT_WARN:
        return _GREEN, 'FRAMING GOOD - shoot the set'
    if tilt is not None and tilt >= _TILT_FAIL:
        return _RED, 'FRAMING POOR - chart tilted, shoot face-on'
    return _AMBER, 'FRAMING WEAK - adjust position/angle'


def annotate(preview, coords, cor, qa, clipped, is_raw, max_w=1600):
    """Draw the overlay and return (summary, annotated_canvas)."""
    scale = min(1.0, max_w / preview.shape[1]) if preview.shape[1] else 1.0
    if scale < 1.0:
        canvas = cv2.resize(preview, None, fx=scale, fy=scale,
                            interpolation=cv2.INTER_AREA)
        if coords is not None:
            coords = (coords[0] * scale, coords[1] * scale)
    else:
        canvas = preview.copy()

    fs = max(0.5, canvas.shape[1] / 1400.0)
    lw = max(1, round(fs * 1.6))

    if coords is None:
        _banner(canvas, ['CHART NOT LOCATED',
                         'reposition: fill frame, face-on, even light'],
                _RED, fs, lw)
        return 'not located', canvas

    tilt = _tilt(coords[1][0])
    colour, headline = _framing_verdict(cor, tilt)
    _draw_grid(canvas, coords, clipped, colour, lw)

    tilt_txt = 'n/a' if tilt is None else f'{tilt * 100:.0f}%'
    qa_txt = qa['severity'] if qa else ('n/a (JPEG)' if not is_raw else 'n/a')
    lines = [headline,
             f'confidence {cor:.2f}   tilt {tilt_txt}   QA {qa_txt}']
    if qa and qa['issues']:
        lines += [f'- {i}' for i in qa['issues']]
    _banner(canvas, lines, colour, fs, lw)

    return f'located cor={cor:.2f} tilt={tilt_txt} qa={qa_txt}', canvas


# --------------------------------------------------------------------------
# Target capture
# --------------------------------------------------------------------------

# Shared SSH/scp options. BatchMode never prompts (key-based auth only, so an
# unattended --watch cannot hang); accept-new trusts a target's key on first
# connect (writing this host's own ~/.ssh/known_hosts) so a fresh clone does not
# dead-end on host-key verification. Host/user/port/identity/proxy all still
# come from the user's ~/.ssh/config, so --host may be a config alias.
def _ssh_opts(connect_timeout=None):
    opts = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new']
    if connect_timeout is not None:
        opts += ['-o', f'ConnectTimeout={connect_timeout}']
    return opts


def _run(cmd, timeout):
    """Run an ssh/scp command, turning a missing binary or hang into a clear
    error."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout)
    except FileNotFoundError:
        raise RuntimeError(f'{cmd[0]} not found -- install openssh-client')
    except subprocess.TimeoutExpired:
        raise RuntimeError(f'{cmd[0]} timed out after {timeout}s -- check the '
                           'network/ethernet cable and --host')


def _check_reachable(target, connect_timeout):
    """Verify the target answers over SSH, else raise an actionable error.

    ssh exits 255 for any connect/auth failure (no route, refused, timed out,
    key rejected), which is distinct from a remote command's own exit code.
    """
    r = _run(['ssh', *_ssh_opts(connect_timeout), target, 'true'],
             connect_timeout + 5)
    if r.returncode == 0:
        return
    hint = ('check the network/ethernet cable, --host, and key-based login '
            f'("ssh-copy-id {target}"); this tool never prompts for a password')
    raise RuntimeError(f'cannot reach {target} over SSH: '
                       f'{r.stderr.strip() or "no response"}\n  {hint}')


def capture_from_target(args, workdir):
    """Run one capture on the target machine over SSH and pull it back.

    The capture runs in the foreground (ssh blocks until it finishes) because
    the frame is needed before it can be analysed and a single frame is quick;
    if the link drops mid-capture we simply retry on the next --watch tick.
    --capture-cmd is the target-side command (its ``{outdir}`` is replaced with
    the remote output directory) and must drop a raw .pgm there, optionally with
    a matching .json sidecar. Frames are pulled to fixed local names so a long
    --watch run does not accumulate files. Returns the local .pgm path.
    """
    outdir = args.remote_outdir.strip().rstrip('/')
    if not outdir:
        raise RuntimeError('--remote-outdir must be a real path, not empty '
                           'or "/" (it is wiped with "rm -rf" each capture)')

    workdir.mkdir(parents=True, exist_ok=True)
    target = f'{args.user}@{args.host}'
    _check_reachable(target, args.ssh_timeout)
    cmd = args.capture_cmd.replace('{outdir}', outdir)
    remote = f'rm -rf {outdir} && mkdir -p {outdir} && {cmd}'
    logger.info(f'[target] capturing on {target} ...')
    r = _run(['ssh', *_ssh_opts(args.ssh_timeout), target, remote],
             args.timeout)
    if r.returncode == 255:
        _check_reachable(target, args.ssh_timeout)  # raises with the reason
    if r.returncode != 0:
        raise RuntimeError(f'capture command failed on {target} '
                           f'(rc={r.returncode}): '
                           f'{(r.stderr or r.stdout).strip() or "no output"}')

    find = f'ls -t {outdir}/*.pgm 2>/dev/null | head -1'
    q = _run(['ssh', *_ssh_opts(args.ssh_timeout), target, find],
             args.ssh_timeout)
    if q.returncode == 255:
        _check_reachable(target, args.ssh_timeout)  # link dropped, not a bad cmd
    newest = q.stdout.strip()
    if not newest:
        raise RuntimeError(f'no raw .pgm produced in {outdir} on {target} '
                           '(check --capture-cmd)')

    local_pgm = workdir / 'latest_raw.pgm'
    remote_json = newest[:-len('.pgm')] + '.json'
    for src, dst, required in ((newest, local_pgm, True),
                               (remote_json, workdir / 'latest_raw.json',
                                False)):
        dst.unlink(missing_ok=True)  # avoid pairing a new .pgm with a stale one
        rc = _run(['scp', *_ssh_opts(args.ssh_timeout), f'{target}:{src}',
                   str(dst)], args.ssh_timeout).returncode
        if rc != 0 and required:
            raise RuntimeError(f'failed to pull {src} from {target}')
        if rc != 0:
            logger.debug(f'no sidecar at {src}; QA will use built-in defaults')
    return local_pgm


# --------------------------------------------------------------------------
# Live stream source
# --------------------------------------------------------------------------

class GstFrameSource:
    """Pull the latest decoded frame off a GStreamer appsink as BGR."""

    def __init__(self, desc):
        try:
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst
        except (ImportError, ValueError) as e:
            raise RuntimeError('--stream needs the GStreamer 1.0 Python '
                               'bindings (python3-gi and gst plugins)') from e
        self._Gst = Gst
        Gst.init(None)
        tail = (' ! videoconvert ! video/x-raw,format=BGR ! appsink '
                'name=fa_sink max-buffers=2 drop=true sync=false')
        try:
            self._pipe = Gst.parse_launch(desc + tail)
        except Exception as e:
            raise RuntimeError(f'bad --stream pipeline: {e}') from e
        self._sink = self._pipe.get_by_name('fa_sink')
        if self._pipe.set_state(Gst.State.PLAYING) == \
                Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('--stream pipeline failed to start; check the '
                               'source and network')

    def grab(self, timeout_s):
        """Latest frame as a BGR ndarray, or None on timeout/EOS."""
        Gst = self._Gst
        sample = self._sink.try_pull_sample(int(timeout_s * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        s = sample.get_caps().get_structure(0)
        w, h = s.get_value('width'), s.get_value('height')
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            # appsink may pad each row to a stride wider than w*3; slice it off.
            stride = mi.size // h
            rows = np.frombuffer(mi.data, dtype=np.uint8,
                                 count=stride * h).reshape(h, stride)
            return rows[:, :w * 3].reshape(h, w, 3).copy()
        finally:
            buf.unmap(mi)

    def close(self):
        self._pipe.set_state(self._Gst.State.NULL)


# --------------------------------------------------------------------------
# Output: live window or file
# --------------------------------------------------------------------------

_WIN = 'mchp frame assist'
_gui_ok = None  # None until probed, then True/False for the rest of the run


def _probe_gui():
    """Return True only if an OpenCV highgui window can actually open.

    OpenCV's Qt/GTK backend calls abort() -- not a catchable exception -- when
    no display is reachable, so the probe runs in a throwaway subprocess whose
    crash cannot take this process down.
    """
    probe = ('import cv2; cv2.namedWindow("_probe", cv2.WINDOW_NORMAL); '
             'cv2.destroyAllWindows()')
    try:
        return subprocess.run([sys.executable, '-c', probe],
                              capture_output=True, timeout=15).returncode == 0
    except Exception:
        return False


def _show(canvas, wait_ms):
    """Show canvas in a live window.

    Returns True to keep going, False if the user asked to quit (q/Esc or closed
    the window), or None if this host has no usable GUI -- the caller then falls
    back to writing a file. wait_ms=0 blocks until a key is pressed (single
    shot); a positive value is the --watch refresh interval.
    """
    global _gui_ok
    if _gui_ok is None:
        _gui_ok = _probe_gui()
    if not _gui_ok:
        return None
    try:
        cv2.imshow(_WIN, canvas)
        key = cv2.waitKey(max(0, int(wait_ms))) & 0xFF
        if cv2.getWindowProperty(_WIN, cv2.WND_PROP_VISIBLE) < 1:
            return False
    except cv2.error:
        _gui_ok = False
        return None
    return key not in (27, ord('q'))


def _write(path, canvas):
    cv2.imwrite(str(path), canvas)
    return path


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def run_once(args, ref_data, workdir, source=None):
    """Capture/load, analyse and annotate one frame; return (located, canvas)."""
    if source is not None:
        bgr = source.grab(args.stream_timeout)
        if bgr is None:
            raise RuntimeError('no frame from --stream (timeout/EOS); check '
                               'the target pipeline and network')
        name, (preview, det_input, image) = 'stream', (bgr, bgr, None)
    elif args.local:
        name, (preview, det_input, image) = args.local, _load(Path(args.local))
    else:
        src = capture_from_target(args, workdir)
        name, (preview, det_input, image) = src.name, _load(src)
    cor, coords = locate(det_input, ref_data)
    qa, clipped = _qa(image, coords)
    summary, canvas = annotate(preview, coords, cor, qa, clipped,
                               image is not None)
    logger.info(f'{name}: {summary}')
    return coords is not None, canvas


def build_parser():
    p = argparse.ArgumentParser(
        description='Macbeth framing assistant for ISC calibration captures.')
    src = p.add_mutually_exclusive_group()
    src.add_argument('--local', metavar='FILE',
                     help='analyse an existing capture (.pgm/.jpg/.png)')
    src.add_argument('--capture', action='store_true',
                     help='run a capture on the target over SSH (default)')
    src.add_argument('--stream', metavar='PIPELINE',
                     help='analyse a live stream: the receive/decode half of a '
                          'gst-launch line ending at a raw-video pad (framing '
                          'only, always live)')
    p.add_argument('--stream-timeout', type=float, default=5.0,
                   help='seconds to wait for a --stream frame (default 5)')
    p.add_argument('--watch', action='store_true',
                   help='repeat on an interval until q/Esc')
    p.add_argument('--interval', type=float, default=3.0,
                   help='seconds between --watch iterations (default 3)')
    p.add_argument('-o', '--output', metavar='PATH',
                   help='write the annotated preview to PATH (single-shot '
                        'defaults to frame_assist.png; --watch shows a live '
                        'window and writes nothing unless PATH is given)')
    disp = p.add_mutually_exclusive_group()
    disp.add_argument('--show', dest='show', action='store_true', default=None,
                      help='pop up a live window (default in --watch)')
    disp.add_argument('--no-show', dest='show', action='store_false',
                      help='never open a window; write to a file instead')

    b = p.add_argument_group('target capture')
    b.add_argument('--host', help='target machine IP/hostname '
                   '(required for --capture)')
    b.add_argument('--user', default='root', help='target SSH user')
    b.add_argument('--capture-cmd',
                   default='mchpcam-still --raw --width=1920 --height=1080 '
                           '-o {outdir}/frame.pgm',
                   help='target-side capture command; {outdir} is replaced '
                        'with --remote-outdir and it must write a raw .pgm '
                        'there. Defaults to a single-frame mchpcam-still grab '
                        '(framing only) — deliberately not the calibration '
                        'sweep, which must not run over SSH (mchp_capture.sh '
                        'refuses it; pass --allow-ssh there only if you must)')
    b.add_argument('--remote-outdir', default='/root/frame_assist',
                   help='target-side capture directory')
    b.add_argument('--ssh-timeout', type=int, default=10,
                   help='SSH connect/transfer timeout (s)')
    b.add_argument('--timeout', type=int, default=120,
                   help='overall capture timeout (s)')
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    if not args.local and not args.host:
        parser.error('--host is required for --capture (or pass --local FILE)')
    if args.local and args.host:
        logger.warning('--host is ignored when --local FILE is given')
    ref_data = _build_ref_data()
    base = Path(args.output).resolve().parent if args.output else Path.cwd()
    workdir = base / '.frame_assist_captures'
    show = args.watch if args.show is None else args.show

    if not args.watch:
        try:
            _, canvas = run_once(args, ref_data, workdir)
        except Exception as e:
            logger.error(f'error: {e}')
            return 1
        out = args.output or 'frame_assist.png'
        logger.info(f'wrote {_write(out, canvas)}')
        if show and _show(canvas, 0) is None:
            logger.info('(no display available for --show)')
        cv2.destroyAllWindows()
        return 0

    logger.info('watching (q or Esc to stop) ...')
    try:
        while True:
            canvas = None
            try:
                _, canvas = run_once(args, ref_data, workdir)
            except Exception as e:
                logger.error(f'error: {e}')
            if canvas is None:
                time.sleep(args.interval)
                continue
            shown = _show(canvas, args.interval * 1000) if show else None
            if shown is False:
                break                       # user asked to quit
            if shown is None:               # no GUI, or --no-show
                dst = args.output or 'frame_assist.png'
                if show:                    # a window was wanted but unavailable
                    show = False
                    logger.info(f'no display available; writing frames to {dst}'
                                ' -- open it in a viewer that auto-refreshes')
                _write(dst, canvas)
                time.sleep(args.interval)
            elif args.output:               # window shown and a path was given
                _write(args.output, canvas)  # waitKey already paced the loop
    except KeyboardInterrupt:
        logger.info('stopped.')
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
