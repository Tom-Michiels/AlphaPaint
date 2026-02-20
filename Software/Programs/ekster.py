#!/usr/bin/env python3
import cv2
import numpy as np
import math
import os
import random
import sys

from alphapaint import AlphaPaint

#######################################
# Configuration
#######################################

PIXEL_MAX_DIM = 800

# Pen configuratie
NUM_PENS = 5
MIN_STROKES_PER_PEN = 1
MAX_STROKES_PER_PEN = 10

SCRIBBLE_MIN_SEGMENTS = 1
SCRIBBLE_MAX_SEGMENTS = 9
IMPROVEMENT_THRESHOLD = 0.0001

SEGMENT_LENGTH_MIN = 10
SEGMENT_LENGTH_MAX = 34
SEGMENT_DELTA_MIN = -1
SEGMENT_DELTA_MAX = 1

STROKE_THICKNESS = 1
COLORS = [[0, 0, 0]]

SEGMENT_CANDIDATES = 20
START_CANDIDATES = 100

IMAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ekster.png")

#######################################
# Utility Functions
#######################################

def fit_image_on_canvas(target_img, canvas_width, canvas_height):
    """Scale target_img (grayscale) preserving aspect ratio and center it on a white background."""
    h, w = target_img.shape[:2]
    scale = min(canvas_width / w, canvas_height / h)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(target_img, (new_w, new_h))
    canvas_img = np.full((canvas_height, canvas_width), 255, dtype=np.uint8)
    x_offset = (canvas_width - new_w) // 2
    y_offset = (canvas_height - new_h) // 2
    canvas_img[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
    return canvas_img

def draw_curve(canvas, x0, y0, theta, L, delta, color, thickness):
    """
    Draw a curve segment (or a straight line if delta is nearly zero) onto a temporary stroke layer,
    blend it with the canvas (via per-pixel minimum), and return the new canvas along with the
    endpoint (x1, y1), new tangent (theta+delta) and arc_info (if an arc was drawn, else None).
    """
    stroke_layer = np.full(canvas.shape, 255, dtype=np.uint8)
    arc_info = None
    if abs(delta) < 1e-6:
        x1 = x0 + L * math.cos(theta)
        y1 = y0 + L * math.sin(theta)
        pt0 = (int(round(x0)), int(round(y0)))
        pt1 = (int(round(x1)), int(round(y1)))
        cv2.line(stroke_layer, pt0, pt1, color=color, thickness=thickness, lineType=cv2.LINE_AA)
    else:
        r = L / abs(delta)
        if delta > 0:
            cx = x0 - r * math.sin(theta)
            cy = y0 + r * math.cos(theta)
        else:
            cx = x0 + r * math.sin(theta)
            cy = y0 - r * math.cos(theta)
        arc_info = {"cx": cx, "cy": cy, "r": r, "delta": delta}
        sign = 1 if delta > 0 else -1
        start_angle = theta - sign * math.pi / 2
        end_angle = start_angle + delta

        # Calculate exact endpoint before rounding
        x1 = cx + r * math.cos(end_angle)
        y1 = cy + r * math.sin(end_angle)

        # Assert using exact endpoint coordinates
        assert(abs(math.hypot(x1 - cx, y1 - cy) - r) < 1e-6)

        # Draw the polyline with rounded points for display
        num_points = max(int(L), 2)
        angles = np.linspace(start_angle, end_angle, num_points)
        pts = []
        for a in angles:
            x = cx + r * math.cos(a)
            y = cy + r * math.sin(a)
            pts.append([int(round(x)), int(round(y))])
        pts = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(stroke_layer, [pts], isClosed=False, color=color, thickness=thickness, lineType=cv2.LINE_AA)

    new_canvas = np.minimum(canvas, stroke_layer)
    new_theta = theta + delta
    return new_canvas, x1, y1, new_theta, arc_info

def compute_error(canvas, target_gray):
    """Compute error as the sum of squared differences of Gaussian-blurred images."""
    canvas_gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    blurred_canvas = cv2.GaussianBlur(canvas_gray, (5, 5), 0)
    blurred_target = cv2.GaussianBlur(target_gray, (1, 1), 0)
    diff = blurred_canvas.astype(np.int32) - blurred_target.astype(np.int32)
    return np.sum(diff * diff)

def select_next_start(canvas, target_gray, num_candidates):
    """Select the canvas coordinate with the highest local error (after blurring) from a random sample."""
    canvas_gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    blurred_canvas = cv2.GaussianBlur(canvas_gray, (3, 3), 0)
    blurred_target = cv2.GaussianBlur(target_gray, (3, 3), 0)
    error_map = np.abs(blurred_canvas.astype(np.int32) - blurred_target.astype(np.int32))
    best_error = -1
    best_coord = (None, None)
    h, w = error_map.shape
    for _ in range(num_candidates):
        x = random.randint(0, w - 1)
        y = random.randint(0, h - 1)
        if error_map[y, x] > best_error:
            best_error = error_map[y, x]
            best_coord = (x, y)
    return float(best_coord[0]), float(best_coord[1])

#######################################
# Main Program
#######################################

def main():
    with AlphaPaint() as ap:
        # Get plotter canvas dimensions (mm).
        ap_canvas = ap.canvas
        ap_w = ap_canvas.width
        ap_h = ap_canvas.height

        # Load and fit target image.
        target_img = cv2.imread(IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
        if target_img is None:
            print("Error loading image.", file=sys.stderr)
            return

        # Compute pixel canvas dimensions from image aspect ratio.
        img_h, img_w = target_img.shape[:2]
        if img_w >= img_h:
            pixel_width = PIXEL_MAX_DIM
            pixel_height = int(PIXEL_MAX_DIM * img_h / img_w)
        else:
            pixel_height = PIXEL_MAX_DIM
            pixel_width = int(PIXEL_MAX_DIM * img_w / img_h)

        target_gray = fit_image_on_canvas(target_img, pixel_width, pixel_height)

        # Compute pixel-to-mm scale (preserve aspect ratio, center on canvas).
        scale = min(ap_w / pixel_width, ap_h / pixel_height)
        x_offset = (ap_w - pixel_width * scale) / 2
        y_offset = (ap_h - pixel_height * scale) / 2

        def pixel_to_canvas(x, y):
            """Convert pixel coordinates to AlphaPaint canvas coordinates (mm)."""
            cx = x * scale + x_offset
            cy = ap_h - (y * scale + y_offset)  # Invert Y
            return cx, cy

        # Create white canvas (color).
        canvas = np.full((pixel_height, pixel_width, 3), 255, dtype=np.uint8)
        try:
            cv2.namedWindow("Canvas", cv2.WINDOW_NORMAL)
            show_gui = True
        except cv2.error:
            show_gui = False

        total_segments = 0
        pen_round = 0

        while True:
            # Pick up pen voor deze ronde
            pen_index = pen_round % NUM_PENS
            strokes_this_pen = random.randint(MIN_STROKES_PER_PEN, MAX_STROKES_PER_PEN)
            print(f"=== Pen {pen_index}, planning {strokes_this_pen} strokes ===", file=sys.stderr)
            ap.pickup_pen(pen_index)

            strokes_done = 0
            while strokes_done < strokes_this_pen:
                # Choose new starting point.
                start_x, start_y = select_next_start(canvas, target_gray, num_candidates=START_CANDIDATES)
                current_theta = random.uniform(0, 2*math.pi)
                current_x, current_y = start_x, start_y
                scribble_color = tuple(int(c) for c in random.choice(COLORS))

                scribble_start_error = compute_error(canvas, target_gray)
                scribble_count = 0

                while True:
                    current_error = compute_error(canvas, target_gray)
                    best_error = None
                    best_canvas = None
                    best_endpoint = (None, None)
                    best_new_theta = None
                    best_arc = None
                    for _ in range(SEGMENT_CANDIDATES):
                        L = random.uniform(SEGMENT_LENGTH_MIN, SEGMENT_LENGTH_MAX)
                        delta = random.uniform(SEGMENT_DELTA_MIN, SEGMENT_DELTA_MAX)
                        candidate_canvas, candidate_x, candidate_y, candidate_new_theta, arc_info = draw_curve(
                            canvas, current_x, current_y, current_theta, L, delta, scribble_color, STROKE_THICKNESS)
                        err = compute_error(candidate_canvas, target_gray)
                        if best_error is None or err < best_error:
                            best_error = err
                            best_canvas = candidate_canvas
                            best_endpoint = (candidate_x, candidate_y)
                            best_new_theta = candidate_new_theta
                            best_arc = arc_info

                    # If no improvement found, skip this scribble entirely
                    if best_error >= current_error:
                        break

                    # On first successful segment, move to start and lower pen
                    if scribble_count == 0:
                        sx, sy = pixel_to_canvas(start_x, start_y)
                        ap.move_to(sx, sy)
                        ap.pen_down()

                    canvas = best_canvas
                    prev_x, prev_y = current_x, current_y
                    current_x, current_y = best_endpoint
                    current_theta = best_new_theta
                    scribble_count += 1
                    total_segments += 1

                    if show_gui:
                        cv2.imshow("Canvas", canvas)
                    print(f"Segment {total_segments:06d}, Scribble segments: {scribble_count}, Error: {best_error}", file=sys.stderr)

                    # Send segment to plotter
                    cx, cy = pixel_to_canvas(current_x, current_y)
                    if best_arc is not None:
                        # I, J = offset from current plotter position to arc center
                        i_mm = (best_arc["cx"] - prev_x) * scale
                        j_mm = -(best_arc["cy"] - prev_y) * scale  # Invert Y

                        # Validate arc geometry before sending
                        # Radius from I,J offset
                        radius_ij = math.hypot(i_mm, j_mm)
                        # Expected endpoint relative to center
                        px, py = pixel_to_canvas(prev_x, prev_y)
                        center_x = px + i_mm
                        center_y = py + j_mm
                        # Distance from center to endpoint
                        radius_end = math.hypot(cx - center_x, cy - center_y)

                        # Check if radii match (within 0.5mm tolerance)
                        radius_error = abs(radius_ij - radius_end)
                        if radius_error < 0.5 and radius_ij > 0.1:
                            # Arc is valid - Y inversion flips CW/CCW direction
                            clockwise = (best_arc["delta"] > 0)  # Inverted due to Y-axis flip
                            ap.draw_arc(cx, cy, i_mm, j_mm, clockwise=clockwise)
                        else:
                            # Arc invalid, fall back to line
                            ap.draw_to(cx, cy)
                    else:
                        ap.draw_to(cx, cy)

                    if show_gui and cv2.waitKey(1) & 0xFF == 27:
                        break

                    new_err = compute_error(canvas, target_gray)
                    improvement = scribble_start_error - new_err
                    if scribble_count >= SCRIBBLE_MIN_SEGMENTS and improvement < scribble_start_error * IMPROVEMENT_THRESHOLD:
                        break
                    if scribble_count >= SCRIBBLE_MAX_SEGMENTS:
                        break

                # Lift pen after scribble (fast for quick repositioning within same pen)
                ap.pen_up_fast()
                strokes_done += 1

            # Full pen up before returning pen
            ap.pen_up()
            ap.return_pen(pen_index)
            pen_round += 1

        if show_gui:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
