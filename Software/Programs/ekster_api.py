#!/usr/bin/env python3
"""Draw the magpie as scribbles, through the remote API, with a single pen.

Same idea as `ekster.py`: approximate a photograph with curved pen strokes,
each one chosen greedily because it makes the drawing look more like the
target. Two differences, both of them the reason this file exists:

  * it plans first and draws afterwards. `plan` writes the strokes to a JSON
    file and a preview PNG, so the drawing can be judged before the machine
    moves at all - a run takes many minutes and a sheet of paper.
  * it goes through the remote API (`plotter_api.Plotter`), not the external
    program protocol, and uses one pen for the whole drawing.

The greedy search is the same but local: a candidate stroke is scored only
over the pixels it touches, which is what makes planning a few hundred
scribbles take a minute instead of an hour.

    python3 ekster_api.py plan --scribbles 320 --out ~/alphapaint-exploration/ekster
    python3 ekster_api.py draw ~/alphapaint-exploration/ekster.json --pen 0

Run under the system python3 for `plan` (it has OpenCV); `draw` needs only the
standard library.
"""

import argparse
import json
import math
import os
import random
import sys
import time

from plotter_api import Plotter, PlotterError

IMAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ekster.png')

# The sheet as the console operator marked it out, minus a margin.
SHEET = (421.0, 414.0, 779.0, 648.0)

SEGMENT_LENGTH = (10.0, 34.0)       # pixels
SEGMENT_TURN = (-1.0, 1.0)          # radians over the whole segment
SEGMENT_CANDIDATES = 24
START_CANDIDATES = 200
SCRIBBLE_MAX_SEGMENTS = 9
THICKNESS = 1                      # pen width in pixels, see --max-dim


# ----------------------------------------------------------------- planning

def arc_points(x, y, theta, length, turn, step=3.0):
    """Points along one segment, plus the tangent at its end.

    A nearly straight segment is two points; a curved one is flattened into
    chords of a few pixels, because the API draws lines.
    """
    if abs(turn) < 0.02:
        end = (x + length * math.cos(theta), y + length * math.sin(theta))
        return [(x, y), end], theta
    radius = length / abs(turn)
    sign = 1.0 if turn > 0 else -1.0
    cx = x - sign * radius * math.sin(theta)
    cy = y + sign * radius * math.cos(theta)
    start_angle = theta - sign * math.pi / 2
    count = max(2, int(min(length / step, abs(turn) / 0.4) + 1))
    points = []
    for i in range(count + 1):
        angle = start_angle + turn * i / count
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points, theta + turn


def score(canvas, target, points, thickness):
    """How much this stroke improves the drawing, and the canvas it leaves.

    Only the rectangle the stroke touches is rendered and compared, so the
    cost of a candidate is its own size and not the size of the drawing.
    """
    import cv2
    import numpy as np

    height, width = canvas.shape
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    pad = thickness + 2
    x0, x1 = int(min(xs)) - pad, int(max(xs)) + pad + 1
    y0, y1 = int(min(ys)) - pad, int(max(ys)) + pad + 1
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        return None                      # would run off the paper
    patch = canvas[y0:y1, x0:x1]
    layer = np.full(patch.shape, 255, dtype=np.uint8)
    shifted = np.array([[int(round(px - x0)), int(round(py - y0))] for px, py in points],
                       dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(layer, [shifted], False, 0, thickness, cv2.LINE_AA)
    drawn = np.minimum(patch, layer)
    wanted = target[y0:y1, x0:x1].astype(np.int32)
    before = patch.astype(np.int32) - wanted
    after = drawn.astype(np.int32) - wanted
    gain = int((before * before).sum() - (after * after).sum())
    return gain, (x0, y0, x1, y1), drawn


def pick_start(canvas, target, count):
    """A random spot where the drawing is furthest from the target."""
    import numpy as np

    height, width = canvas.shape
    xs = np.random.randint(0, width, count)
    ys = np.random.randint(0, height, count)
    error = np.abs(canvas[ys, xs].astype(np.int32) - target[ys, xs].astype(np.int32))
    best = int(np.argmax(error))
    return float(xs[best]), float(ys[best])


def plan(image_path, max_dim, scribbles, seed, thickness=THICKNESS, lighten=0.0):
    """Greedily build a list of strokes; returns them in pixel coordinates."""
    import cv2
    import numpy as np

    random.seed(seed)
    np.random.seed(seed)

    source = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if source is None:
        raise SystemExit(f"cannot read {image_path}")
    height, width = source.shape
    if width >= height:
        size = (max_dim, max(1, int(max_dim * height / width)))
    else:
        size = (max(1, int(max_dim * width / height)), max_dim)
    target = cv2.resize(source, size, interpolation=cv2.INTER_AREA)
    if lighten:
        # Keep the darkest tone above pure black so the blacks stay a dense
        # scribble instead of a soaked-through blob of ink.
        target = (target.astype(np.float32) * (1 - lighten) + 255 * lighten).astype(np.uint8)
    canvas = np.full(target.shape[:2], 255, dtype=np.uint8)

    strokes = []
    started = time.time()
    for index in range(scribbles):
        x, y = pick_start(canvas, target, START_CANDIDATES)
        theta = random.uniform(0, 2 * math.pi)
        points = [(x, y)]
        for _ in range(SCRIBBLE_MAX_SEGMENTS):
            best = None
            for _ in range(SEGMENT_CANDIDATES):
                length = random.uniform(*SEGMENT_LENGTH)
                turn = random.uniform(*SEGMENT_TURN)
                candidate, new_theta = arc_points(x, y, theta, length, turn)
                result = score(canvas, target, candidate, thickness)
                if result is None:
                    continue
                gain = result[0]
                if best is None or gain > best[0]:
                    best = (gain, candidate, new_theta, result[1], result[2])
            if best is None or best[0] <= 0:
                break                      # nothing here makes it better
            _, candidate, theta, (bx0, by0, bx1, by1), drawn = best
            canvas[by0:by1, bx0:bx1] = drawn
            points.extend(candidate[1:])
            x, y = candidate[-1]
        if len(points) > 1:
            strokes.append(points)
        if (index + 1) % 25 == 0:
            done = np.abs(canvas.astype(np.int32) - target.astype(np.int32)).mean()
            print(f"  {index + 1:4d} scribbles, {len(strokes)} kept, "
                  f"mean error {done:5.1f}, {time.time() - started:4.0f}s", file=sys.stderr)
    return strokes, canvas, target


# ------------------------------------------------------------------ drawing

def to_machine(strokes, shape, area):
    """Pixels to machine millimetres, centred in the area, aspect preserved."""
    height, width = shape
    x0, y0, x1, y1 = area
    scale = min((x1 - x0) / width, (y1 - y0) / height)
    off_x = x0 + ((x1 - x0) - width * scale) / 2
    off_y = y0 + ((y1 - y0) - height * scale) / 2
    drawn_height = height * scale
    return [[(off_x + px * scale, off_y + drawn_height - py * scale) for px, py in stroke]
            for stroke in strokes]


def path_length(strokes):
    total = 0.0
    for stroke in strokes:
        for (ax, ay), (bx, by) in zip(stroke, stroke[1:]):
            total += math.hypot(bx - ax, by - ay)
    return total


def order_strokes(strokes):
    """Nearest-neighbour ordering so the pen travels less between strokes."""
    remaining = list(strokes)
    ordered = []
    here = (0.0, 0.0)
    while remaining:
        best = min(range(len(remaining)),
                   key=lambda i: math.hypot(remaining[i][0][0] - here[0],
                                            remaining[i][0][1] - here[1]))
        stroke = remaining.pop(best)
        ordered.append(stroke)
        here = stroke[-1]
    return ordered


def draw(plan_path, pen, feed, pen_z, start_at, dry_run, hop=6.0):
    with open(plan_path) as f:
        data = json.load(f)
    strokes = [[tuple(point) for point in stroke] for stroke in data['strokes_mm']]
    print(f"{len(strokes)} strokes, {path_length(strokes) / 1000:.1f} m of line, "
          f"area X {data['area'][0]:.0f}..{data['area'][2]:.0f} "
          f"Y {data['area'][1]:.0f}..{data['area'][3]:.0f}")
    if dry_run:
        return 0

    with Plotter() as p:
        status = p.status()
        if not status['homed']:
            print("homing first...")
            p.home()
        if pen_z is not None:
            p.set_pen_z(pen_z)
        if start_at == 0:
            print(f"picking up pen {pen}")
            p.pickup_pen(pen)

        started = time.time()
        for index, stroke in enumerate(strokes):
            if index < start_at:
                continue
            p.move(x=stroke[0][0], y=stroke[0][1], wait=False)
            p.pen_down()
            for x, y in stroke[1:]:
                p.move(x=x, y=y, draw=True, feed=feed, wait=False)
            # A low hop instead of a full pen up: Z runs at 2400 mm/min, so
            # going to 60 and back costs three seconds on every stroke - more
            # than the stroke itself takes to draw.
            p.move(z=hop, draw=True)
            if (index + 1) % 20 == 0:
                elapsed = time.time() - started
                per = elapsed / (index + 1 - start_at)
                print(f"  stroke {index + 1}/{len(strokes)}, {elapsed / 60:.1f} min, "
                      f"{per * (len(strokes) - index - 1) / 60:.1f} min to go")

        p.pen_up()
        print("putting the pen back")
        p.return_pen(pen)
        print(f"done in {(time.time() - started) / 60:.1f} min")
    return 0


# --------------------------------------------------------------------- main

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    p_plan = sub.add_parser('plan', help='work out the strokes and preview them')
    p_plan.add_argument('--image', default=IMAGE_PATH)
    p_plan.add_argument('--scribbles', type=int, default=320)
    p_plan.add_argument('--max-dim', type=int, default=700, help='planning resolution in pixels')
    p_plan.add_argument('--seed', type=int, default=7)
    p_plan.add_argument('--thickness', type=int, default=THICKNESS,
                        help='pen width in planning pixels')
    p_plan.add_argument('--lighten', type=float, default=0.0,
                        help='0..1, lift the blacks so they stay open scribble')
    p_plan.add_argument('--margin', type=float, default=12.0, help='mm kept free around the drawing')
    p_plan.add_argument('--area', default=None,
                        help='X0,Y0,X1,Y1 in machine mm (default: the marked sheet)')
    p_plan.add_argument('--out', default=os.path.expanduser('~/alphapaint-exploration/ekster'))

    p_draw = sub.add_parser('draw', help='play a plan back on the machine')
    p_draw.add_argument('plan')
    p_draw.add_argument('--pen', type=int, default=0)
    p_draw.add_argument('--feed', type=int, default=3000)
    p_draw.add_argument('--pen-z', type=float, default=None)
    p_draw.add_argument('--start-at', type=int, default=0,
                        help='resume at this stroke (the pen is assumed to be held)')
    p_draw.add_argument('--hop', type=float, default=6.0,
                        help='mm the pen lifts between strokes')
    p_draw.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    if args.command == 'draw':
        return draw(args.plan, args.pen, args.feed, args.pen_z, args.start_at,
                    args.dry_run, args.hop)

    import cv2
    import numpy as np

    if args.area:
        area = tuple(float(v) for v in args.area.split(','))
    else:
        area = SHEET
    area = (area[0] + args.margin, area[1] + args.margin,
            area[2] - args.margin, area[3] - args.margin)

    strokes, canvas, target = plan(args.image, args.max_dim, args.scribbles, args.seed,
                                   args.thickness, args.lighten)
    machine = order_strokes(to_machine(strokes, canvas.shape, area))

    preview = os.path.expanduser(args.out) + '.png'
    side = np.hstack([canvas, target])
    cv2.imwrite(preview, side)
    plan_file = os.path.expanduser(args.out) + '.json'
    with open(plan_file, 'w') as f:
        json.dump({'created': time.time(), 'image': args.image, 'seed': args.seed,
                   'scribbles': args.scribbles, 'area': list(area),
                   'pixel_size': [canvas.shape[1], canvas.shape[0]],
                   'strokes_mm': [[list(point) for point in stroke] for stroke in machine]},
                  f)
    print(f"{len(machine)} strokes, {sum(len(s) for s in machine)} points, "
          f"{path_length(machine) / 1000:.2f} m of line")
    print(f"preview: {preview}")
    print(f"plan:    {plan_file}")
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except PlotterError as e:
        print(f"machine refused: {e}", file=sys.stderr)
        sys.exit(1)
