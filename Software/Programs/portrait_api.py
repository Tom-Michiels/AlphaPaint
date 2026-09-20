#!/usr/bin/env python3
"""Draw a photograph as coloured scribbles with a handful of pens.

The same greedy search as `ekster_api.py`, but the canvas is in colour and the
strokes are drawn with the real ink colours measured by `pen_survey.py`. Ink
is modelled the way a marker behaves: a stroke multiplies what is underneath
by its own colour, so a second pen over a first one darkens rather than
replaces, and white paper takes the pen colour as it is.

One pen is drawn at a time, in the order given, so the machine makes one pen
change per pen and not one per stroke. Drawing light before dark gives the
pens a chance to mix on the paper.

    python3 portrait_api.py plan photo.jpg --pens 2,1,0 --out ~/alphapaint-exploration/portrait
    python3 portrait_api.py draw ~/alphapaint-exploration/portrait.json

Planning needs the system python3 (OpenCV); drawing needs only the standard
library.
"""

import argparse
import json
import math
import os
import random
import sys
import time

from ekster_api import arc_points, order_strokes, path_length, to_machine
from plotter_api import Plotter, PlotterError

DATA = os.path.expanduser('~/alphapaint-exploration')
SEGMENT_LENGTH = (6.0, 22.0)
SEGMENT_TURN = (-1.2, 1.2)
SEGMENT_CANDIDATES = 24
START_CANDIDATES = 300
SCRIBBLE_MAX_SEGMENTS = 7


def load_pens(path=None):
    """Pen colours and heights as measured by pen_survey.py."""
    path = path or os.path.join(DATA, 'pen-survey.json')
    with open(path) as f:
        survey = json.load(f)
    return {entry['pen']: entry for entry in survey['pens']}


def stroke_coverage(shape, points, thickness):
    """How much ink each pixel of the patch gets, 0..1, anti-aliased."""
    import cv2
    import numpy as np

    layer = np.zeros(shape, np.uint8)
    pts = np.array([[int(round(x)), int(round(y))] for x, y in points],
                   np.int32).reshape((-1, 1, 2))
    cv2.polylines(layer, [pts], False, 255, thickness, cv2.LINE_AA)
    return layer.astype(np.float32) / 255.0


def score(canvas, target, points, ink, thickness, strength):
    """Improvement from laying this stroke down, and the patch it leaves."""
    import numpy as np

    height, width = canvas.shape[:2]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    pad = thickness + 2
    x0, x1 = int(min(xs)) - pad, int(max(xs)) + pad + 1
    y0, y1 = int(min(ys)) - pad, int(max(ys)) + pad + 1
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        return None
    patch = canvas[y0:y1, x0:x1]
    alpha = stroke_coverage(patch.shape[:2], [(x - x0, y - y0) for x, y in points],
                            thickness)[:, :, None] * strength
    # A marker multiplies: paper * ink, so a second pen darkens the first.
    drawn = patch * (1 - alpha) + patch * (ink / 255.0) * alpha
    wanted = target[y0:y1, x0:x1]
    before = patch - wanted
    after = drawn - wanted
    gain = float((before * before).sum() - (after * after).sum())
    return gain, (x0, y0, x1, y1), drawn


def demand(canvas, target, ink):
    """Per pixel: how much this pen would help. Used to aim the scribbles."""
    import numpy as np

    would_be = canvas * (ink / 255.0)
    before = np.abs(canvas - target).sum(axis=2)
    after = np.abs(would_be - target).sum(axis=2)
    return before - after


def plan_pen(canvas, target, ink, scribbles, thickness, strength, label):
    import numpy as np

    strokes = []
    height, width = canvas.shape[:2]
    started = time.time()
    for index in range(scribbles):
        want = demand(canvas, target, ink)
        xs = np.random.randint(0, width, START_CANDIDATES)
        ys = np.random.randint(0, height, START_CANDIDATES)
        best_start = int(np.argmax(want[ys, xs]))
        if want[ys[best_start], xs[best_start]] <= 0:
            break                        # this pen has nothing left to add
        x, y = float(xs[best_start]), float(ys[best_start])
        theta = random.uniform(0, 2 * math.pi)
        points = [(x, y)]
        for _ in range(SCRIBBLE_MAX_SEGMENTS):
            best = None
            for _ in range(SEGMENT_CANDIDATES):
                length = random.uniform(*SEGMENT_LENGTH)
                turn = random.uniform(*SEGMENT_TURN)
                candidate, new_theta = arc_points(x, y, theta, length, turn)
                result = score(canvas, target, candidate, ink, thickness, strength)
                if result is None:
                    continue
                if best is None or result[0] > best[0]:
                    best = (result[0], candidate, new_theta, result[1], result[2])
            if best is None or best[0] <= 0:
                break
            _, candidate, theta, (bx0, by0, bx1, by1), drawn = best
            canvas[by0:by1, bx0:bx1] = drawn
            points.extend(candidate[1:])
            x, y = candidate[-1]
        if len(points) > 1:
            strokes.append(points)
        if (index + 1) % 50 == 0:
            print(f"  {label}: {index + 1:4d} tried, {len(strokes)} kept, "
                  f"{time.time() - started:4.0f}s", file=sys.stderr)
    return strokes


def parse_pen(spec):
    """'0:4,42,93:2.2:0.5' -> pen, ink colour, line width in mm, pen-down Z."""
    parts = spec.split(':')
    if len(parts) < 3:
        raise SystemExit(f"pen spec needs pen:r,g,b:width_mm[:z], got {spec!r}")
    pen = int(parts[0])
    r, g, b = (int(v) for v in parts[1].split(','))
    return {'pen': pen, 'rgb': [r, g, b], 'width_mm': float(parts[2]),
            'z': float(parts[3]) if len(parts) > 3 else None}


def command_plan(args):
    import cv2
    import numpy as np

    random.seed(args.seed)
    np.random.seed(args.seed)

    source = cv2.imread(args.image)
    if source is None:
        raise SystemExit(f"cannot read {args.image}")

    area = tuple(float(v) for v in args.area.split(','))
    area = (area[0] + args.margin, area[1] + args.margin,
            area[2] - args.margin, area[3] - args.margin)
    area_w, area_h = area[2] - area[0], area[3] - area[1]

    # Work in real millimetres: the planning pixel is a fixed fraction of a
    # millimetre, so a pen's measured width becomes a thickness in pixels and
    # the preview shows what the pen can actually do.
    height, width = source.shape[:2]
    scale = min(area_w / width, area_h / height)
    pixels = (max(1, int(width * scale / args.mm_per_px)),
              max(1, int(height * scale / args.mm_per_px)))
    print(f"drawing {width * scale:.0f} x {height * scale:.0f} mm, planning at "
          f"{args.mm_per_px} mm per pixel ({pixels[0]} x {pixels[1]})", file=sys.stderr)

    target = cv2.resize(source, pixels, interpolation=cv2.INTER_AREA).astype(np.float32)
    if args.contrast != 1.0:
        target = np.clip((target - 128) * args.contrast + 128, 0, 255)
    if args.white_point < 255:
        # Anything lighter than the white point is paper: no pen goes there.
        # Without this the greedy scribbles over every mid-tone, and a face
        # turns into a filled blob instead of a drawing.
        target = np.clip(target / args.white_point, 0, 1) * 255
    if args.lighten:
        target = target * (1 - args.lighten) + 255 * args.lighten

    budgets = [int(v) for v in str(args.scribbles).split(',')]
    canvas = np.full(target.shape, 255.0, np.float32)
    layers = []
    for index, spec in enumerate(args.pen):
        entry = parse_pen(spec)
        thickness = max(1, int(round(entry['width_mm'] / args.mm_per_px)))
        r, g, b = entry['rgb']
        ink = np.array([b, g, r], np.float32)
        print(f"pen {entry['pen']}: rgb{tuple(entry['rgb'])}, {entry['width_mm']} mm "
              f"= {thickness} px", file=sys.stderr)
        budget = budgets[min(index, len(budgets) - 1)]
        strokes = plan_pen(canvas, target, ink, budget, thickness,
                           args.strength, f"pen {entry['pen']}")
        entry['thickness_px'] = thickness
        entry['strokes'] = strokes
        layers.append(entry)

    shape = target.shape[:2]
    for layer in layers:
        layer['strokes_mm'] = [[list(p) for p in stroke]
                               for stroke in order_strokes(to_machine(layer['strokes'],
                                                                      shape, area))]
        del layer['strokes']

    preview = os.path.expanduser(args.out) + '.png'
    cv2.imwrite(preview, np.hstack([np.clip(canvas, 0, 255).astype(np.uint8),
                                    np.clip(target, 0, 255).astype(np.uint8)]))
    plan_file = os.path.expanduser(args.out) + '.json'
    with open(plan_file, 'w') as f:
        json.dump({'created': time.time(), 'image': args.image, 'area': list(area),
                   'mm_per_px': args.mm_per_px,
                   'pixel_size': [shape[1], shape[0]], 'layers': layers}, f)
    total = 0.0
    for layer in layers:
        length = path_length([[tuple(p) for p in s] for s in layer['strokes_mm']])
        total += length
        print(f"pen {layer['pen']} rgb{tuple(layer['rgb'])}: "
              f"{len(layer['strokes_mm'])} strokes, {length / 1000:.1f} m")
    print(f"{total / 1000:.1f} m of line in total")
    print(f"preview: {preview}")
    print(f"plan:    {plan_file}")
    return 0


def command_draw(args):
    with open(args.plan) as f:
        data = json.load(f)
    layers = data['layers']
    for layer in layers:
        strokes = [[tuple(p) for p in s] for s in layer['strokes_mm']]
        print(f"pen {layer['pen']}: {len(strokes)} strokes, "
              f"{path_length(strokes) / 1000:.1f} m")
    if args.dry_run:
        return 0

    with Plotter() as p:
        if not p.status()['homed']:
            p.home()
        started = time.time()
        for index, layer in enumerate(layers):
            if index < args.start_layer:
                continue
            strokes = [[tuple(p) for p in s] for s in layer['strokes_mm']]
            z = args.pen_z if args.pen_z is not None else layer.get('z')
            print(f"pen {layer['pen']} at Z={z}, {len(strokes)} strokes")
            p.pickup_pen(layer['pen'])
            if z is not None:
                p.set_pen_z(float(z))
            for number, stroke in enumerate(strokes):
                p.move(x=stroke[0][0], y=stroke[0][1], wait=False)
                p.pen_down()
                for x, y in stroke[1:]:
                    p.move(x=x, y=y, draw=True, feed=args.feed, wait=False)
                p.move(z=args.hop, draw=True)
                if (number + 1) % 50 == 0:
                    print(f"    {number + 1}/{len(strokes)}, "
                          f"{(time.time() - started) / 60:.1f} min")
            p.pen_up()
            p.return_pen(layer['pen'])
        print(f"done in {(time.time() - started) / 60:.1f} min")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    p_plan = sub.add_parser('plan')
    p_plan.add_argument('image')
    p_plan.add_argument('--pen', action='append', required=True,
                        help='pen:r,g,b:width_mm[:z], repeat in draw order, light first')
    p_plan.add_argument('--mm-per-px', type=float, default=0.5)
    p_plan.add_argument('--scribbles', default='400',
                        help='attempts per pen, one number or one per pen')
    p_plan.add_argument('--white-point', type=float, default=255.0,
                        help='tones lighter than this stay bare paper')
    p_plan.add_argument('--strength', type=float, default=0.85,
                        help='how much ink one pass lays down, 0..1')
    p_plan.add_argument('--lighten', type=float, default=0.10)
    p_plan.add_argument('--contrast', type=float, default=1.0)
    p_plan.add_argument('--seed', type=int, default=3)
    p_plan.add_argument('--margin', type=float, default=12.0)
    p_plan.add_argument('--area', default='421,414,779,648')
    p_plan.add_argument('--out', default=os.path.join(DATA, 'portrait'))
    p_plan.set_defaults(func=command_plan)

    p_draw = sub.add_parser('draw')
    p_draw.add_argument('plan')
    p_draw.add_argument('--feed', type=int, default=3000)
    p_draw.add_argument('--hop', type=float, default=6.0)
    p_draw.add_argument('--pen-z', type=float, default=None)
    p_draw.add_argument('--start-layer', type=int, default=0)
    p_draw.add_argument('--dry-run', action='store_true')
    p_draw.set_defaults(func=command_draw)

    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except PlotterError as e:
        print(f"machine refused: {e}", file=sys.stderr)
        sys.exit(1)
