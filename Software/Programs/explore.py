#!/usr/bin/env python3
"""Let the machine find out what it is working with, step by step.

Three procedures, each writing what it learned to a JSON file so the next one
can build on it. Run them with the machine in view: they move the gantry and
put a pen on the paper.

  scan      Sweep the table diagonally from home, photographing as it goes.
            Produces a manifest of (machine position, photo) pairs.
  paper     Work out the paper edges from those photos (needs OpenCV).
  pen-depth For one pen, draw short test strokes at decreasing Z and photograph
            each, so we can see at which height it starts marking.
  camera    Draw a cross, then look at it, to measure how far the camera sits
            from the pen tip.

Together these give what a painting program needs: where the paper is, how deep
each pen must go, and how to translate what the camera sees into machine
coordinates.

Usage: explore.py scan|paper|pen-depth|camera [options]
Run under the system python3 (it has OpenCV): python3 explore.py paper
"""

import argparse
import json
import os
import sys
import time
from typing import Dict, List

from plotter_api import Plotter, PlotterError

DATA_DIR = os.path.expanduser('~/alphapaint-exploration')


def save(name: str, data: Dict) -> str:
    os.makedirs(DATA_DIR, exist_ok=True)
    path = os.path.join(DATA_DIR, name)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"written: {path}")
    return path


def load(name: str) -> Dict:
    with open(os.path.join(DATA_DIR, name)) as f:
        return json.load(f)


# ---------------------------------------------------------------- scanning

def scan(args):
    """Move diagonally across the table, photographing along the way."""
    with Plotter() as p:
        status = p.status()
        if not status['homed']:
            print("homing first...")
            p.home()
            status = p.status()
        limits = status['limits']
        if not status['camera']['device']:
            print("No camera found - plug in the gantry camera first.", file=sys.stderr)
            return 1

        x_max = min(args.max_x, limits['X'][1])
        y_max = min(args.max_y, limits['Y'][1])
        steps = args.steps
        print(f"scanning the diagonal from (0,0) to ({x_max:.0f},{y_max:.0f}) "
              f"in {steps} steps, camera height Z={args.z}")

        p.pen_up()
        samples: List[Dict] = []
        for index in range(steps + 1):
            fraction = index / steps
            x = fraction * x_max
            y = fraction * y_max
            p.move(x=x, y=y, z=args.z)
            time.sleep(args.settle)
            photo = p.photo(f'scan-{index:03d}')
            samples.append({'index': index, 'x': x, 'y': y, 'z': args.z,
                            'photo': photo['path']})
            print(f"  {index:3d}/{steps}  ({x:7.1f}, {y:7.1f})  {photo['path']}")

        save('scan.json', {'created': time.time(), 'z': args.z,
                           'x_max': x_max, 'y_max': y_max, 'samples': samples})
    return 0


# ------------------------------------------------------------ paper edges

def paper(args):
    """Guess the paper rectangle from the scan photos.

    Simple first pass: the paper is the bright area in the middle of the
    picture. For every photo we measure how much of the centre is bright, so
    the positions where the camera was over paper stand out from the rest.
    """
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("OpenCV is needed: run this with the system python3", file=sys.stderr)
        return 1

    scan_data = load('scan.json')
    results = []
    for sample in scan_data['samples']:
        image = cv2.imread(sample['photo'], cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"  cannot read {sample['photo']}")
            continue
        height, width = image.shape
        patch = image[height // 3: 2 * height // 3, width // 3: 2 * width // 3]
        bright = float((patch > args.threshold).mean())
        results.append({**sample, 'bright_fraction': bright,
                        'mean': float(patch.mean())})
        print(f"  ({sample['x']:7.1f}, {sample['y']:7.1f})  bright {bright:5.1%}  "
              f"mean {patch.mean():5.1f}")

    over_paper = [r for r in results if r['bright_fraction'] >= args.min_bright]
    if over_paper:
        summary = {
            'x_min': min(r['x'] for r in over_paper),
            'x_max': max(r['x'] for r in over_paper),
            'y_min': min(r['y'] for r in over_paper),
            'y_max': max(r['y'] for r in over_paper),
            'samples_over_paper': len(over_paper),
        }
        print(f"\npaper seen between X {summary['x_min']:.0f}..{summary['x_max']:.0f} "
              f"and Y {summary['y_min']:.0f}..{summary['y_max']:.0f} "
              f"(from {len(over_paper)} of {len(results)} photos along one diagonal)")
        print("note: one diagonal only bounds the paper roughly; scan a grid for the real edges")
    else:
        summary = {}
        print("\nno photo looked like paper - adjust --threshold/--min-bright or the lighting")
    save('paper.json', {'created': time.time(), 'threshold': args.threshold,
                        'samples': results, 'summary': summary})
    return 0


# --------------------------------------------------------------- pen depth

def pen_depth(args):
    """Draw short strokes at decreasing Z to find where the pen starts marking."""
    with Plotter() as p:
        status = p.status()
        if not status['homed']:
            p.home()
        if args.pick:
            print(f"picking up pen {args.pen}")
            p.pickup_pen(args.pen)

        x, y = args.x, args.y
        strokes = []
        z = args.z_start
        index = 0
        while z >= args.z_end - 1e-9:
            y_stroke = y + index * args.spacing
            print(f"  stroke {index}: Z={z:.2f} at Y={y_stroke:.1f}")
            p.move(x=x, y=y_stroke)
            p.pen_down(z=z)
            p.move(x=x + args.length, y=y_stroke, draw=True, feed=args.feed)
            p.pen_up()
            strokes.append({'index': index, 'z': round(z, 3), 'x0': x,
                            'x1': x + args.length, 'y': y_stroke})
            z -= args.step
            index += 1

        # Photograph the whole ladder of strokes from above
        p.move(x=x + args.length / 2, y=y + (index - 1) * args.spacing / 2, z=args.camera_z)
        time.sleep(args.settle)
        photo = p.photo(f'pen{args.pen}-depth')
        print(f"photo of the test strokes: {photo['path']}")
        if args.pick:
            p.return_pen(args.pen)

        save(f'pen{args.pen}-depth.json',
             {'created': time.time(), 'pen': args.pen, 'strokes': strokes,
              'photo': photo['path'],
              'note': 'look at the photo: the first Z that leaves a line is the '
                      'contact height; set it with /api/pen/z'})
    return 0


# ------------------------------------------------------- camera vs pen tip

def camera(args):
    """Draw a cross, then centre the camera on where it should be.

    The difference between where the cross lands in the picture and the centre
    of the picture is the camera-to-pen offset, in pixels; combined with a
    known stroke length it also gives the mm-per-pixel scale.
    """
    with Plotter() as p:
        status = p.status()
        if not status['homed']:
            p.home()
        if args.pick:
            p.pickup_pen(args.pen)

        x, y = args.x, args.y
        arm = args.size / 2
        print(f"drawing a {args.size} mm cross at ({x}, {y})")
        p.move(x=x - arm, y=y)
        p.pen_down(z=args.z)
        p.move(x=x + arm, y=y, draw=True, feed=args.feed)
        p.pen_up()
        p.move(x=x, y=y - arm)
        p.pen_down(z=args.z)
        p.move(x=x, y=y + arm, draw=True, feed=args.feed)
        p.pen_up()

        # Look at it with the camera at the same machine position
        p.move(x=x, y=y, z=args.camera_z)
        time.sleep(args.settle)
        photo = p.photo('camera-offset')
        print(f"photo: {photo['path']}")
        if args.pick:
            p.return_pen(args.pen)

        save('camera.json', {
            'created': time.time(), 'cross_at': {'x': x, 'y': y},
            'cross_size_mm': args.size, 'camera_z': args.camera_z,
            'photo': photo['path'],
            'note': 'find the cross centre in the photo; its offset from the '
                    'image centre is where the camera looks relative to the pen, '
                    'and the arm length in pixels gives mm per pixel'})
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    s = sub.add_parser('scan', help='diagonal sweep with photos')
    s.add_argument('--max-x', type=float, default=900.0)
    s.add_argument('--max-y', type=float, default=900.0)
    s.add_argument('--steps', type=int, default=20)
    s.add_argument('--z', type=float, default=60.0)
    s.add_argument('--settle', type=float, default=0.4)
    s.set_defaults(func=scan)

    s = sub.add_parser('paper', help='find the paper in the scan photos')
    s.add_argument('--threshold', type=int, default=170, help='grey value counted as bright')
    s.add_argument('--min-bright', type=float, default=0.8, help='fraction of the centre')
    s.set_defaults(func=paper)

    s = sub.add_parser('pen-depth', help='find the height where a pen marks')
    s.add_argument('--pen', type=int, default=0)
    s.add_argument('--pick', action='store_true', help='pick the pen up first')
    s.add_argument('--x', type=float, default=450.0)
    s.add_argument('--y', type=float, default=450.0)
    s.add_argument('--z-start', type=float, default=3.0)
    s.add_argument('--z-end', type=float, default=-0.5)
    s.add_argument('--step', type=float, default=0.25)
    s.add_argument('--length', type=float, default=20.0)
    s.add_argument('--spacing', type=float, default=5.0)
    s.add_argument('--feed', type=int, default=2000)
    s.add_argument('--camera-z', type=float, default=60.0)
    s.add_argument('--settle', type=float, default=0.4)
    s.set_defaults(func=pen_depth)

    s = sub.add_parser('camera', help='measure the camera offset from the pen')
    s.add_argument('--pen', type=int, default=0)
    s.add_argument('--pick', action='store_true')
    s.add_argument('--x', type=float, default=450.0)
    s.add_argument('--y', type=float, default=450.0)
    s.add_argument('--z', type=float, default=0.5)
    s.add_argument('--size', type=float, default=20.0)
    s.add_argument('--feed', type=int, default=2000)
    s.add_argument('--camera-z', type=float, default=60.0)
    s.add_argument('--settle', type=float, default=0.4)
    s.set_defaults(func=camera)

    args = parser.parse_args()
    try:
        return args.func(args)
    except PlotterError as e:
        print(f"machine refused: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
