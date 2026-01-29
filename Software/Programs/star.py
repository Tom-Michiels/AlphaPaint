#!/usr/bin/env python3
"""
Star Drawing Program for AlphaPaint.

Draws a multi-pointed star with inner pentagram in the center of the canvas.
"""

import math
from alphapaint import AlphaPaint


def main():
    with AlphaPaint() as ap:
        canvas = ap.canvas
        cx = canvas.center_x
        cy = canvas.center_y

        # Star parameters
        num_points = 5
        outer_radius = canvas.min_dimension / 2 * 0.9
        inner_radius = outer_radius * 0.4

        # Calculate all vertices (alternating outer and inner)
        vertices = []
        for i in range(num_points * 2):
            angle = -math.pi / 2 + (i * math.pi / num_points)
            radius = outer_radius if i % 2 == 0 else inner_radius
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            vertices.append((x, y))

        # Draw the star outline
        ap.move_to(vertices[0][0], vertices[0][1])
        ap.pen_down()
        for x, y in vertices[1:]:
            ap.draw_to(x, y, wait=False)
        ap.draw_to(vertices[0][0], vertices[0][1], wait=False)
        ap.pen_up()

        ap.flush()

        # Draw inner pentagram (connect every other outer vertex)
        outer_vertices = [vertices[i] for i in range(0, len(vertices), 2)]
        for i in range(num_points):
            start = outer_vertices[i]
            end = outer_vertices[(i + 2) % num_points]
            ap.move_to(start[0], start[1])
            ap.pen_down()
            ap.draw_to(end[0], end[1], wait=False)
            ap.pen_up()

        ap.flush()


if __name__ == "__main__":
    main()
