# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2024 Microchip Technology Inc.
#
# corner_picker.py -- Interactive ColorChecker corner selection.
#
# Provides two entry points:
#   pick_corners(image_path)  -- try matplotlib GUI, fall back to text input
#   annotate_patches(image_path, corners, output_path)  -- save annotated image
#
# Corner order (matches image.py set_macbeth_patches):
#   click 1 → top-left     (TL)
#   click 2 → top-right    (TR)
#   click 3 → bottom-left  (BL)
#   click 4 → bottom-right (BR)
#
# The 24 ColorChecker Classic patches are arranged in a 6-column x 4-row grid:
#
#   TL ─────────────────────── TR
#    │ [ 0][ 1][ 2][ 3][ 4][ 5] │  row 0: dark_skin → bluish_green
#    │ [ 6][ 7][ 8][ 9][10][11] │  row 1: orange → orange_yellow
#    │ [12][13][14][15][16][17] │  row 2: blue → cyan
#    │ [18][19][20][21][22][23] │  row 3: white → black  (grey ramp)
#   BL ─────────────────────── BR
#
# Patch 18 = white reference  (used for CCM normalisation)
# Patches 18-23 = neutral grey ramp  (3x weight in CCM solve)

import logging
import os
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

logger = logging.getLogger(__name__)

# X-Rite ColorChecker Classic sRGB reference values (D50, row-major)
_PATCH_SRGB = [
    (115, 82,  68),   #  0  dark_skin
    (194, 150, 130),  #  1  light_skin
    (98,  122, 157),  #  2  blue_sky
    (87,  108, 67),   #  3  foliage
    (133, 128, 177),  #  4  blue_flower
    (103, 189, 170),  #  5  bluish_green
    (214, 126, 44),   #  6  orange
    (80,  91,  166),  #  7  purplish_blue
    (193, 90,  99),   #  8  moderate_red
    (94,  60,  108),  #  9  purple
    (157, 188, 64),   # 10  yellow_green
    (224, 163, 46),   # 11  orange_yellow
    (56,  61,  150),  # 12  blue
    (70,  148, 73),   # 13  green
    (175, 54,  60),   # 14  red
    (231, 199, 31),   # 15  yellow
    (187, 86,  149),  # 16  magenta
    (8,   133, 161),  # 17  cyan
    (243, 243, 243),  # 18  white
    (200, 200, 200),  # 19  neutral_8
    (160, 160, 160),  # 20  neutral_6_5
    (122, 122, 122),  # 21  neutral_5
    (85,  85,  85),   # 22  neutral_3_5
    (52,  52,  52),   # 23  black
]

_PATCH_NAMES = [
    'dark skin', 'light skin', 'blue sky', 'foliage',
    'blue flower', 'bluish green', 'orange', 'purplish blue',
    'moderate red', 'purple', 'yellow green', 'orange yellow',
    'blue', 'green', 'red', 'yellow', 'magenta', 'cyan',
    'white', 'neutral 8', 'neutral 6.5', 'neutral 5',
    'neutral 3.5', 'black',
]

_GREY_IDX = {18, 19, 20, 21, 22, 23}
_N_COLS, _N_ROWS = 6, 4


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _compute_patch_centres(corners):
    """
    Compute 24 patch centre pixel coordinates from 4 corner points.
    corners: [(tl_x,tl_y), (tr_x,tr_y), (bl_x,bl_y), (br_x,br_y)]
    Returns list of 24 (x, y) float tuples, row-major.
    """
    tl, tr, bl, br = corners
    centres = []
    for row in range(_N_ROWS):
        for col in range(_N_COLS):
            fx = (col + 0.5) / _N_COLS
            fy = (row + 0.5) / _N_ROWS
            top_x = tl[0] + fx * (tr[0] - tl[0])
            top_y = tl[1] + fx * (tr[1] - tl[1])
            bot_x = bl[0] + fx * (br[0] - bl[0])
            bot_y = bl[1] + fx * (br[1] - bl[1])
            cx = top_x + fy * (bot_x - top_x)
            cy = top_y + fy * (bot_y - top_y)
            centres.append((cx, cy))
    return centres


def _patch_radius(corners):
    """
    Compute sampling radius (pixels) scaled to patch size.
    Matches image.py set_macbeth_patches() radius calculation.
    """
    tl, tr, bl, _ = corners
    chart_w = abs(tr[0] - tl[0])
    chart_h = abs(bl[1] - tl[1])
    pw = chart_w / _N_COLS
    ph = chart_h / _N_ROWS
    return max(8, min(60, int(0.30 * min(pw, ph))))


# ---------------------------------------------------------------------------
# Text-based fallback
# ---------------------------------------------------------------------------

def _text_input_corners():
    """
    Prompt the user to type 4 corner coordinates as 'x,y'.
    Returns list of 4 (int,int) tuples or None if cancelled.
    """
    print()
    print('  Enter the pixel coordinates of the 4 corners of the')
    print('  ColorChecker patch grid (not the card edge):')
    print()
    print('    TL ─── TR        Click order: top-left first,')
    print('    │  grid │         then top-right, bottom-left,')
    print('    BL ─── BR         bottom-right.')
    print()
    labels = [
        ('top-left  ', '(TL) — patch 0  dark skin'),
        ('top-right ', '(TR) — patch 5  bluish green'),
        ('bottom-left ', '(BL) — patch 18 white'),
        ('bottom-right', '(BR) — patch 23 black'),
    ]
    corners = []
    for label, hint in labels:
        while True:
            try:
                raw = input(f'    {label} {hint}: ').strip()
                if raw.lower() in ('q', 'quit', 'cancel'):
                    return None
                x, y = map(int, raw.split(','))
                corners.append((x, y))
                break
            except (ValueError, KeyboardInterrupt):
                print('    Invalid format — enter as x,y  (e.g. 142,89)')
    return corners


# ---------------------------------------------------------------------------
# Matplotlib interactive picker
# ---------------------------------------------------------------------------

def _matplotlib_pick(image_path):
    """
    Open image in a matplotlib window. User clicks 4 corners in order:
    TL → TR → BL → BR.

    Returns list of 4 (int,int) tuples or None if cancelled / display unavailable.
    """
    try:
        import matplotlib
        # Use non-interactive backend if DISPLAY is not set (headless)
        display = os.environ.get('DISPLAY', '')
        wayland = os.environ.get('WAYLAND_DISPLAY', '')
        if not display and not wayland:
            logger.debug('No DISPLAY/WAYLAND_DISPLAY — skipping matplotlib GUI')
            return None
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        logger.debug('matplotlib not available — using text input')
        return None
    except Exception as e:
        logger.debug(f'matplotlib init failed ({e}) — using text input')
        return None

    try:
        img_pil = Image.open(image_path).convert('RGB')
    except Exception as e:
        logger.warning(f'Cannot open image for display: {e}')
        return None

    fig, ax = plt.subplots(figsize=(14, 9))
    ax.imshow(np.array(img_pil))
    ax.set_title(
        'ColorChecker Corner Picker\n'
        'Click 4 corners: TL → TR → BL → BR  |  '
        'Right-click to undo last  |  Close window to cancel',
        fontsize=11, pad=8
    )

    # Corner labels shown on image as user clicks
    _order  = ['TL (top-left)', 'TR (top-right)',
               'BL (bottom-left)', 'BR (bottom-right)']
    _colours = ['#00e676', '#00b0ff', '#ff9100', '#e040fb']
    _markers = []   # (scatter, annotation) objects
    _pts     = []   # collected (x, y) pairs

    status_text = ax.text(
        0.01, 0.02,
        f'Click {_order[0]}  (patch 0 - dark skin corner)',
        transform=ax.transAxes,
        fontsize=10, color='white',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#333333', alpha=0.85)
    )

    def _update_status():
        n = len(_pts)
        if n < 4:
            status_text.set_text(
                f'Click {_order[n]}  '
                f'({4 - n} corner(s) remaining)'
            )
        else:
            status_text.set_text(
                'All 4 corners selected.  Close window to confirm or '
                'right-click to redo.'
            )
        fig.canvas.draw_idle()

    def _draw_grid(pts):
        """Draw the 6x4 patch grid as an overlay."""
        for artist in getattr(_draw_grid, '_artists', []):
            try:
                artist.remove()
            except Exception:
                pass
        _draw_grid._artists = []
        if len(pts) < 4:
            return
        centres = _compute_patch_centres(pts)
        rad = _patch_radius(pts)
        for idx, (cx, cy) in enumerate(centres):
            colour = '#%02x%02x%02x' % _PATCH_SRGB[idx]
            sq = mpatches.Rectangle(
                (cx - rad, cy - rad), 2*rad, 2*rad,
                linewidth=1.5, edgecolor='white', facecolor=colour, alpha=0.55
            )
            ax.add_patch(sq)
            is_grey = idx in _GREY_IDX
            txt = ax.text(
                cx, cy, str(idx),
                ha='center', va='center', fontsize=7,
                color='white' if idx in _GREY_IDX or idx >= 12 else 'black',
                fontweight='bold' if is_grey else 'normal'
            )
            _draw_grid._artists.extend([sq, txt])
        fig.canvas.draw_idle()

    def _on_click(event):
        if event.inaxes != ax:
            return
        if event.button == 3:   # right-click → undo
            if _pts:
                _pts.pop()
                sc, ann = _markers.pop()
                sc.remove()
                ann.remove()
                _draw_grid(_pts)
                _update_status()
            return
        if event.button != 1:
            return
        if len(_pts) >= 4:
            return
        x, y = int(round(event.xdata)), int(round(event.ydata))
        _pts.append((x, y))
        n = len(_pts) - 1
        sc = ax.scatter(x, y, s=120, c=_colours[n],
                        marker='+', linewidths=2.5, zorder=5)
        ann = ax.annotate(
            _order[n], (x, y),
            xytext=(10, -14), textcoords='offset points',
            fontsize=9, color=_colours[n], fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.6)
        )
        _markers.append((sc, ann))
        _draw_grid(_pts)
        _update_status()

    fig.canvas.mpl_connect('button_press_event', _on_click)
    plt.tight_layout()
    plt.show()

    if len(_pts) == 4:
        return _pts
    logger.info('Corner picker closed without 4 corners — falling back to text')
    return None


# ---------------------------------------------------------------------------
# Annotated output image
# ---------------------------------------------------------------------------

def annotate_patches(image_path, corners, output_path):
    """
    Draw corner markers, patch grid, patch numbers and colour swatches
    on the image. Saves to output_path as JPEG.

    Always works (no display needed) — uses PIL only.
    """
    try:
        img = Image.open(image_path).convert('RGB')
    except Exception as e:
        logger.warning(f'annotate_patches: cannot open {image_path}: {e}')
        return

    draw = ImageDraw.Draw(img, 'RGBA')
    w, h = img.size

    centres = _compute_patch_centres(corners)
    rad     = _patch_radius(corners)

    # ── patch boxes with reference colour fill ────────────────────────────
    for idx, (cx, cy) in enumerate(centres):
        cx, cy = int(cx), int(cy)
        r, g, b = _PATCH_SRGB[idx]
        is_grey = idx in _GREY_IDX

        # filled box in reference colour (semi-transparent)
        draw.rectangle(
            [cx - rad, cy - rad, cx + rad, cy + rad],
            fill=(r, g, b, 120),
            outline=(255, 255, 255, 220),
        )

        # patch index label
        lum = 0.299*r + 0.587*g + 0.114*b
        txt_col = (255, 255, 255) if lum < 128 else (0, 0, 0)
        draw.text((cx - 6, cy - 6), str(idx), fill=txt_col)

        # thicker border for grey patches (CCM reference)
        if is_grey:
            draw.rectangle(
                [cx - rad - 2, cy - rad - 2, cx + rad + 2, cy + rad + 2],
                outline=(255, 220, 0, 255),
            )

    # ── corner markers ────────────────────────────────────────────────────
    corner_labels = ['TL', 'TR', 'BL', 'BR']
    corner_colours = [(0, 230, 118), (0, 176, 255), (255, 145, 0), (224, 64, 251)]
    for (cx, cy), label, col in zip(corners, corner_labels, corner_colours):
        cx, cy = int(cx), int(cy)
        sz = 14
        draw.line([(cx - sz, cy), (cx + sz, cy)], fill=col, width=3)
        draw.line([(cx, cy - sz), (cx, cy + sz)], fill=col, width=3)
        draw.text((cx + sz + 2, cy - 8), label, fill=col)

    # ── border lines between chart rows ──────────────────────────────────
    tl, tr, bl, br = corners
    for row in range(1, _N_ROWS):
        fy = row / _N_ROWS
        lx = tl[0] + fy * (bl[0] - tl[0])
        ly = tl[1] + fy * (bl[1] - tl[1])
        rx = tr[0] + fy * (br[0] - tr[0])
        ry = tr[1] + fy * (br[1] - tr[1])
        draw.line([(int(lx), int(ly)), (int(rx), int(ry))],
                  fill=(255, 255, 255, 100), width=1)

    # ── info banner ───────────────────────────────────────────────────────
    banner_h = 38
    draw.rectangle([0, 0, w, banner_h], fill=(20, 20, 20, 210))
    draw.text(
        (8, 6),
        f'CCM patch grid  |  24 patches detected  |  '
        f'yellow border = grey ramp (CCM reference weight x3)  |  '
        f'corners: TL{corners[0]} TR{corners[1]} BL{corners[2]} BR{corners[3]}',
        fill=(200, 200, 200)
    )

    try:
        img.save(output_path, quality=92)
        logger.info(f'Annotated patch grid saved: {output_path}')
    except Exception as e:
        logger.warning(f'annotate_patches: save failed: {e}')


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def pick_corners(image_path, save_annotated=None):
    """
    Interactive ColorChecker corner picker.

    Tries matplotlib GUI first (if DISPLAY is available).
    Falls back to text-based input if no display or matplotlib is missing.

    Args:
        image_path    : path to the captured image (JPEG or PGM)
        save_annotated: if set, saves annotated patch-grid image to this path

    Returns:
        List of 4 (x, y) tuples [(tl),(tr),(bl),(br)], or None if cancelled.
    """
    # For PGM files, try to find the corresponding JPEG for display
    display_path = image_path
    p = Path(image_path)
    if p.suffix.lower() == '.pgm':
        jpg_candidate = p.with_suffix('.jpg')
        if not jpg_candidate.exists():
            # Try stripping _raw suffix
            stem = p.stem
            if stem.endswith('_raw'):
                jpg_candidate = p.with_name(stem[:-4] + '.jpg')
        if jpg_candidate.exists():
            display_path = str(jpg_candidate)
            logger.info(f'Using JPEG preview for display: {jpg_candidate.name}')
        else:
            logger.info('No JPEG preview found — displaying raw PGM (Bayer pattern visible)')

    logger.info(f'Corner picker: {Path(image_path).name}')

    # Try GUI first
    corners = _matplotlib_pick(display_path)

    # Fall back to text
    if corners is None:
        logger.info('Using text-based corner input')
        corners = _text_input_corners()

    if corners is None:
        logger.warning('Corner selection cancelled')
        return None

    if len(corners) != 4:
        logger.error(f'Expected 4 corners, got {len(corners)}')
        return None

    logger.info(f'Corners: TL={corners[0]} TR={corners[1]} '
                f'BL={corners[2]} BR={corners[3]}')

    # Save annotated image
    out_path = save_annotated
    if out_path is None:
        # Default: same dir as image, with _patches suffix
        out_path = str(p.with_name(p.stem + '_patches.jpg'))
    annotate_patches(display_path, corners, out_path)

    return corners
