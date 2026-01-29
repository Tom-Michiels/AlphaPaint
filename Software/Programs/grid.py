#!/usr/bin/env python3
"""
Grid Drawing Program for AlphaPaint.

Draws a grid pattern across the canvas.
"""

from alphapaint import AlphaPaint


def main():
    with AlphaPaint() as ap:
        canvas = ap.canvas

        # Grid parameters
        num_lines_x = 10  # Vertical lines
        num_lines_y = 10  # Horizontal lines
        margin = 5  # mm margin from edges

        x0 = margin
        y0 = margin
        x1 = canvas.width - margin
        y1 = canvas.height - margin
        w = x1 - x0
        h = y1 - y0

        # Draw vertical lines (alternating direction for efficiency)
        col_spacing = w / (num_lines_x - 1)
        for i in range(num_lines_x):
            lx = x0 + i * col_spacing
            if i % 2 == 0:
                ap.move_to(lx, y0)
            else:
                ap.move_to(lx, y1)
            ap.pen_down()
            if i % 2 == 0:
                ap.draw_to(lx, y1, wait=False)
            else:
                ap.draw_to(lx, y0, wait=False)
            ap.pen_up()

        ap.flush()

        # Draw horizontal lines (alternating direction for efficiency)
        row_spacing = h / (num_lines_y - 1)
        for i in range(num_lines_y):
            ly = y0 + i * row_spacing
            if i % 2 == 0:
                ap.move_to(x0, ly)
            else:
                ap.move_to(x1, ly)
            ap.pen_down()
            if i % 2 == 0:
                ap.draw_to(x1, ly, wait=False)
            else:
                ap.draw_to(x0, ly, wait=False)
            ap.pen_up()

        ap.flush()


if __name__ == "__main__":
    main()
