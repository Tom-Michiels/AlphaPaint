#!/usr/bin/env python3
"""Photograph the sheet in a grid and report which parts are still empty.

Moves the gantry over the paper with the pen up, takes one picture per grid
point and measures how much ink each picture contains. The result is a map of
the sheet and the emptiest spot, which is where a calibration ladder or a new
drawing can go.

Note the camera sits a few centimetres away from the pen tip, so a spot that
looks empty through the camera is empty *around* that machine position, not
exactly under the pen. Keep a margin, or measure the offset first with
explore.py camera.

Run under the system python3 (it has OpenCV):
    python3 scan_sheet.py --x0 421 --y0 414 --x1 779 --y1 648
"""

import argparse
import json
import os
import sys
import time

from plotter_api import Plotter, PlotterError

DATA_DIR = os.path.expanduser('~/alphapaint-exploration')


def ink_fraction(path, min_contrast=25, min_saturation=60):
    """Share of pixels that look like ink rather than paper.

    Compares every pixel with a heavily blurred copy of the picture instead of
    with a fixed brightness: a shadow (from the gantry in the sun, say) is a
    slow gradient and cancels out, while a pen line is a sharp dark step and
    survives. Colour counts too, because a shadow is grey and the pens are not.
    """
    import cv2
    import numpy as np

    image = cv2.imread(path)
    if image is None:
        return None
    height, width = image.shape[:2]
    patch = image[height // 4: 3 * height // 4, width // 4: 3 * width // 4]
    grey = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY).astype(np.int16)
    background = cv2.GaussianBlur(grey.astype(np.uint8), (0, 0), 25).astype(np.int16)
    darker_than_surroundings = (background - grey) > min_contrast
    saturation = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[:, :, 1]
    return float(np.mean(darker_than_surroundings | (saturation > min_saturation)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x0', type=float, default=421.0)
    parser.add_argument('--y0', type=float, default=414.0)
    parser.add_argument('--x1', type=float, default=779.0)
    parser.add_argument('--y1', type=float, default=648.0)
    parser.add_argument('--cols', type=int, default=5)
    parser.add_argument('--rows', type=int, default=4)
    parser.add_argument('--z', type=float, default=60.0)
    parser.add_argument('--settle', type=float, default=0.6)
    parser.add_argument('--contrast', type=int, default=25,
                        help='how much darker than its surroundings a pixel must be')
    args = parser.parse_args()

    os.makedirs(DATA_DIR, exist_ok=True)
    samples = []

    p = Plotter()
    p.take_control()
    status = p.status()
    if not status['homed']:
        print("homing first...")
        p.home()
    if not status['camera']['device']:
        print("no camera found", file=sys.stderr)
        return 1
    p.pen_up()

    print(f"scanning {args.cols}x{args.rows} over X {args.x0:.0f}..{args.x1:.0f}, "
          f"Y {args.y0:.0f}..{args.y1:.0f}")
    for row in range(args.rows):
        y = args.y0 + (args.y1 - args.y0) * row / max(args.rows - 1, 1)
        for col in range(args.cols):
            x = args.x0 + (args.x1 - args.x0) * col / max(args.cols - 1, 1)
            p.move(x=x, y=y, z=args.z)
            time.sleep(args.settle)
            photo = p.photo(f'sheet-r{row}c{col}')
            samples.append({'row': row, 'col': col, 'x': x, 'y': y,
                            'photo': photo['path']})
            print(f"  ({x:6.1f}, {y:6.1f})  {photo['path']}")

    print("\nmeasuring ink per picture")
    for sample in samples:
        sample['ink'] = ink_fraction(sample['photo'], args.contrast)
        shown = 'n/a' if sample['ink'] is None else f"{sample['ink']:6.2%}"
        print(f"  ({sample['x']:6.1f}, {sample['y']:6.1f})  {shown}")

    print("\nsheet map (top row = high Y); . is empty, # is drawn on")
    for row in reversed(range(args.rows)):
        line = []
        for col in range(args.cols):
            sample = next(s for s in samples if s['row'] == row and s['col'] == col)
            ink = sample['ink'] or 0.0
            line.append('.' if ink < 0.01 else (':' if ink < 0.05 else ('o' if ink < 0.15 else '#')))
        y = args.y0 + (args.y1 - args.y0) * row / max(args.rows - 1, 1)
        print(f"  Y={y:6.1f}  " + ' '.join(line))
    print("            " + ' '.join(
        f"{args.x0 + (args.x1 - args.x0) * c / max(args.cols - 1, 1):.0f}"[:3]
        for c in range(args.cols)))

    clean = sorted((s for s in samples if s['ink'] is not None), key=lambda s: s['ink'])
    best = clean[0]
    print(f"\nemptiest spot: X={best['x']:.1f} Y={best['y']:.1f} "
          f"({best['ink']:.2%} ink) - {best['photo']}")

    with open(os.path.join(DATA_DIR, 'sheet-scan.json'), 'w') as f:
        json.dump({'created': time.time(), 'samples': samples}, f, indent=2)
    print(f"written: {os.path.join(DATA_DIR, 'sheet-scan.json')}")
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except PlotterError as e:
        print(f"machine refused: {e}", file=sys.stderr)
        sys.exit(1)
