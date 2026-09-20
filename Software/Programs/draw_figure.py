#!/usr/bin/env python3
"""Draw a stick figure through the remote API - a small example of using it.

Works in machine coordinates, centred on the sheet as it was last marked out
with the console (B=(421,414), C=(779,648)). Homes first, takes a pen from the
changer, draws, puts the pen back and hands control to the console again.

Usage: draw_figure.py [--pen 0] [--x 600] [--y 530] [--scale 1.0]
"""

import argparse
import math
import sys

from plotter_api import Plotter, PlotterError


def polyline(p, points, feed):
    """Pen down at the first point, draw through the rest, pen up."""
    p.move(x=points[0][0], y=points[0][1])
    p.pen_down()
    for x, y in points[1:]:
        p.move(x=x, y=y, draw=True, feed=feed)
    p.pen_up()


def circle(centre, radius, segments=32):
    cx, cy = centre
    return [(cx + radius * math.cos(2 * math.pi * i / segments),
             cy + radius * math.sin(2 * math.pi * i / segments))
            for i in range(segments + 1)]


def arc(centre, radius, start_deg, end_deg, segments=12):
    cx, cy = centre
    points = []
    for i in range(segments + 1):
        angle = math.radians(start_deg + (end_deg - start_deg) * i / segments)
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pen', type=int, default=0)
    parser.add_argument('--x', type=float, default=600.0, help='centre on the sheet')
    parser.add_argument('--y', type=float, default=530.0)
    parser.add_argument('--scale', type=float, default=1.0)
    parser.add_argument('--feed', type=int, default=3000)
    args = parser.parse_args()

    s = args.scale
    cx, cy = args.x, args.y

    def at(dx, dy):
        return (cx + dx * s, cy + dy * s)

    head_centre = at(0, 55)
    head_radius = 22 * s

    strokes = [
        circle(head_centre, head_radius),                    # head
        [at(0, 33), at(0, -25)],                             # body
        [at(-42, 15), at(42, 15)],                           # arms
        [at(0, -25), at(-28, -72)],                          # left leg
        [at(0, -25), at(28, -72)],                           # right leg
        [at(-8, 62), at(-8, 59)],                            # left eye
        [at(8, 62), at(8, 59)],                              # right eye
        arc(at(0, 55), 12 * s, 215, 325),                    # smile
    ]

    with Plotter() as p:                      # takes control, releases on exit
        status = p.status()
        print(f"state: {status['state']}, homed: {status['homed']}")
        print("homing...")
        status = p.home()
        print(f"homed at {status['position']}")

        print(f"picking up pen {args.pen}")
        p.pickup_pen(args.pen)

        for index, points in enumerate(strokes, 1):
            print(f"  stroke {index}/{len(strokes)} ({len(points)} points)")
            polyline(p, points, args.feed)

        print("putting the pen back")
        p.return_pen(args.pen)
        print(f"done, at {p.position()}")
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except PlotterError as e:
        print(f"machine refused: {e}", file=sys.stderr)
        sys.exit(1)
