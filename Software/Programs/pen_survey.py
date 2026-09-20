#!/usr/bin/env python3
"""What does each pen look like, and how deep does it have to go?

Draws one column of test strokes per pen - the same short line at a ladder of
Z heights - photographs each column and measures, per height, how wide the
line is and what colour the ink is. That is everything a drawing program needs
to pick a pen and a pen height: the Z where the pen starts to mark, the Z
where it draws a thin line, the Z where it draws a fat one, and the colour to
plan with.

The strokes run along X and the ladder runs along Y, so one photograph covers
a whole column (the camera sees about 57 mm across X and 103 mm across Y).

    python3 pen_survey.py --pens 0,1,2,3,4          # draw and measure
    python3 pen_survey.py --measure-only            # re-measure the photos

Run under the system python3; measuring needs OpenCV.
"""

import argparse
import json
import os
import sys
import time

from plotter_api import Plotter, PlotterError

DATA = os.path.expanduser('~/alphapaint-exploration')
PX_X, PX_Y = 12.2, 12.4                 # camera pixels per mm
CAM_OFF_X, CAM_OFF_Y = -0.2, 41.4       # where the camera looks, from the pen


def colour_name(b, g, r):
    """A rough name for an ink colour, enough to talk about the pens."""
    import colorsys
    h, l, s = colorsys.rgb_to_hls(r / 255, g / 255, b / 255)
    if l < 0.18:
        return 'zwart'
    if s < 0.18:
        return 'grijs'
    hue = h * 360
    for limit, name in ((15, 'rood'), (45, 'oranje'), (70, 'geel'), (160, 'groen'),
                        (200, 'turkoois'), (255, 'blauw'), (290, 'paars'),
                        (340, 'roze'), (361, 'rood')):
        if hue < limit:
            return name
    return 'onbekend'


def measure(path, strokes, length_mm):
    """Per stroke in the photo: line width in mm and the ink colour."""
    import cv2
    import numpy as np

    img = cv2.imread(path)
    if img is None:
        return None
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.int16)
    background = cv2.GaussianBlur(grey.astype(np.uint8), (0, 0), 31).astype(np.int16)
    saturation = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:, :, 1].astype(np.int16)
    base_saturation = cv2.GaussianBlur(saturation.astype(np.uint8), (0, 0), 31).astype(np.int16)
    ink = (((background - grey) > 18) | ((saturation - base_saturation) > 25)).astype(np.uint8)
    ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    height, width = ink.shape
    results = []
    for stroke in strokes:
        # A stroke runs along machine X, which is the image's y direction, at
        # one machine Y, which is one band of image columns.
        centre = int(width / 2 + (stroke['y'] - stroke['cam_y']) * PX_Y)
        half = int(0.5 * stroke['spacing'] * PX_Y)
        lo, hi = max(0, centre - half), min(width, centre + half)
        band = ink[:, lo:hi]
        pixels = int(band.sum())
        rows = int((band.sum(axis=1) > 0).sum())
        if pixels < 20 or rows < 10:
            results.append({**stroke, 'width_mm': 0.0, 'colour': None, 'rgb': None})
            continue
        width_mm = pixels / rows / PX_Y
        mask = np.zeros(ink.shape, np.uint8)
        mask[:, lo:hi] = band
        b, g, r = (int(v) for v in cv2.mean(img, mask=mask)[:3])
        results.append({**stroke, 'width_mm': round(width_mm, 2),
                        'rgb': [r, g, b], 'colour': colour_name(b, g, r),
                        'covered_mm': round(rows / PX_X, 1), 'expected_mm': length_mm})
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pens', default='0,1,2,3,4')
    parser.add_argument('--z', default='2.0,1.6,1.2,0.9,0.6,0.4,0.2,0.0',
                        help='pen heights to try, high (no contact) to low (fat line)')
    parser.add_argument('--x0', type=float, default=650.0, help='first pen column')
    parser.add_argument('--column-spacing', type=float, default=40.0)
    parser.add_argument('--y0', type=float, default=110.0, help='first rung')
    parser.add_argument('--spacing', type=float, default=12.0, help='mm between rungs')
    parser.add_argument('--length', type=float, default=18.0)
    parser.add_argument('--feed', type=int, default=2000)
    parser.add_argument('--measure-only', action='store_true')
    parser.add_argument('--photos', default=None,
                        help='where the column photos are (default: /var/log/alphapaint-photos)')
    args = parser.parse_args()

    pens = [int(v) for v in args.pens.split(',')]
    heights = [float(v) for v in args.z.split(',')]
    os.makedirs(DATA, exist_ok=True)
    plan = []
    for column, pen in enumerate(pens):
        x = args.x0 + column * args.column_spacing
        cam_y = args.y0 + (len(heights) - 1) * args.spacing / 2
        strokes = [{'z': z, 'x': x, 'y': args.y0 + rung * args.spacing,
                    'spacing': args.spacing, 'cam_y': cam_y}
                   for rung, z in enumerate(heights)]
        plan.append({'pen': pen, 'x': x, 'cam_x': x + args.length / 2, 'cam_y': cam_y,
                     'photo': os.path.join(args.photos or '/var/log/alphapaint-photos',
                                           f'survey-pen{pen}.jpg'),
                     'strokes': strokes})

    if not args.measure_only:
        with Plotter() as p:
            if not p.status()['homed']:
                p.home()
            for column in plan:
                print(f"pen {column['pen']}: column at X={column['x']:.0f}")
                p.pickup_pen(column['pen'])
                for stroke in column['strokes']:
                    p.move(x=stroke['x'], y=stroke['y'])
                    p.pen_down(z=stroke['z'])
                    p.move(x=stroke['x'] + args.length, y=stroke['y'],
                           draw=True, feed=args.feed)
                    p.move(z=8.0, draw=True)
                p.pen_up()
                p.return_pen(column['pen'])
                p.move(x=column['cam_x'] - CAM_OFF_X, y=column['cam_y'] - CAM_OFF_Y, z=60.0)
                time.sleep(0.8)
                try:
                    p.photo(f"survey-pen{column['pen']}")
                except PlotterError as e:
                    # The strokes are on the paper either way; losing the camera
                    # must not cost the whole run. Re-measure with --measure-only.
                    print(f"  no photo of this column ({e}); the strokes are drawn",
                          file=sys.stderr)

    print(f"\n{'pen':>3} {'Z':>5} {'width':>8} {'drawn':>7}  colour")
    survey = []
    for column in plan:
        results = measure(column['photo'], column['strokes'], args.length)
        if results is None:
            print(f"{column['pen']:3d}  no photo at {column['photo']}")
            continue
        for row in results:
            shown = f"{row['width_mm']:.2f} mm" if row['width_mm'] else '   -'
            drawn = f"{row.get('covered_mm', 0):.0f} mm" if row['width_mm'] else '  -'
            print(f"{column['pen']:3d} {row['z']:5.2f} {shown:>8} {drawn:>7}  "
                  f"{row['colour'] or ''} {row['rgb'] or ''}")
        marking = [r for r in results if r['width_mm'] > 0]
        survey.append({'pen': column['pen'], 'photo': column['photo'],
                       'rungs': results,
                       'first_marks_at_z': marking[0]['z'] if marking else None,
                       'thin_z': min((r for r in marking), key=lambda r: r['width_mm'])['z']
                       if marking else None,
                       'fat_z': max((r for r in marking), key=lambda r: r['width_mm'])['z']
                       if marking else None,
                       'rgb': marking[-1]['rgb'] if marking else None,
                       'colour': marking[-1]['colour'] if marking else None})
        print()
    with open(os.path.join(DATA, 'pen-survey.json'), 'w') as f:
        json.dump({'created': time.time(), 'pens': survey}, f, indent=2)
    print(f"written: {os.path.join(DATA, 'pen-survey.json')}")
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except PlotterError as e:
        print(f"machine refused: {e}", file=sys.stderr)
        sys.exit(1)
