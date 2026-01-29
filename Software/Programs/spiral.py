#!/usr/bin/env python3
"""
Spiral Drawing Program for AlphaPaint.

Draws an Archimedean spiral from the center of the canvas outward.
"""

import math
from alphapaint import AlphaPaint


def main():
    with AlphaPaint() as ap:
        canvas = ap.canvas
        cx = canvas.center_x
        cy = canvas.center_y

        # Spiral parameters
        num_turns = 5
        max_radius = canvas.min_dimension / 2 * 0.9
        points_per_turn = 36
        total_points = num_turns * points_per_turn

        # Start at center
        ap.move_to(cx, cy)
        ap.pen_down()

        for i in range(1, total_points + 1):
            t = i / total_points
            angle = t * num_turns * 2 * math.pi
            radius = t * max_radius

            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)

            ap.draw_to(x, y, wait=False)

        ap.pen_up()
        ap.flush()


if __name__ == "__main__":
    main()
