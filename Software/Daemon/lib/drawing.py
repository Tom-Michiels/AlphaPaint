"""Drawing functions for AlphaPaint."""

import math
import logging
from typing import Tuple, Dict
from .fluidnc import FluidNCHandler


logger = logging.getLogger(__name__)


def draw_line(
    fluidnc: FluidNCHandler,
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    pen_z: float,
    machine_limits: Dict[str, Tuple[float, float]],
    draw_feedrate: int = 500,
    pen_feedrate: int = 100
) -> bool:
    """
    Draw a straight line from p1 to p2.

    Args:
        fluidnc: FluidNC handler instance
        p1: Start point (x, y) in mm
        p2: End point (x, y) in mm
        pen_z: Pen down Z position in mm
        machine_limits: Machine axis limits
        draw_feedrate: Drawing speed in mm/min
        pen_feedrate: Pen lift/lower speed in mm/min

    Returns:
        True if drawing successful, False on error
    """
    logger.info(f"Drawing line from {p1} to {p2}")

    # Get maximum Z from machine limits
    max_z = machine_limits['Z'][1]

    try:
        # Ensure absolute mode
        if not fluidnc.send_gcode("G90"):
            return False

        # Lift pen to maximum height
        if not fluidnc.send_gcode(f"G0 Z{max_z:.2f}"):
            return False

        # Move to start point
        if not fluidnc.send_gcode(f"G0 X{p1[0]:.2f} Y{p1[1]:.2f}"):
            return False

        # Lower pen to drawing height
        if not fluidnc.send_gcode(f"G1 Z{pen_z:.2f} F{pen_feedrate}"):
            return False

        # Draw line to end point
        if not fluidnc.send_gcode(f"G1 X{p2[0]:.2f} Y{p2[1]:.2f} F{draw_feedrate}"):
            return False

        # Lift pen to maximum height
        if not fluidnc.send_gcode(f"G0 Z{max_z:.2f}"):
            return False

        logger.info("Line drawing complete")
        return True

    except Exception as e:
        logger.error(f"Error drawing line: {e}")
        return False


def draw_ellipse(
    fluidnc: FluidNCHandler,
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    pen_z: float,
    machine_limits: Dict[str, Tuple[float, float]],
    draw_feedrate: int = 500,
    pen_feedrate: int = 100,
    num_segments: int = 36
) -> bool:
    """
    Draw an ellipse inscribed in rectangle defined by p1 and p2.

    Args:
        fluidnc: FluidNC handler instance
        p1: First corner point (x, y) in mm
        p2: Second corner point (x, y) in mm
        pen_z: Pen down Z position in mm
        machine_limits: Machine axis limits
        draw_feedrate: Drawing speed in mm/min
        pen_feedrate: Pen lift/lower speed in mm/min
        num_segments: Number of line segments for ellipse approximation

    Returns:
        True if drawing successful, False on error
    """
    logger.info(f"Drawing ellipse from {p1} to {p2}")

    # Calculate ellipse parameters
    center_x = (p1[0] + p2[0]) / 2
    center_y = (p1[1] + p2[1]) / 2
    radius_x = abs(p2[0] - p1[0]) / 2
    radius_y = abs(p2[1] - p1[1]) / 2

    # Get maximum Z from machine limits
    max_z = machine_limits['Z'][1]

    try:
        # Ensure absolute mode
        if not fluidnc.send_gcode("G90"):
            return False

        # Lift pen to maximum height
        if not fluidnc.send_gcode(f"G0 Z{max_z:.2f}"):
            return False

        # Move to ellipse start (t=0, rightmost point)
        start_x = center_x + radius_x
        start_y = center_y
        if not fluidnc.send_gcode(f"G0 X{start_x:.2f} Y{start_y:.2f}"):
            return False

        # Lower pen to drawing height
        if not fluidnc.send_gcode(f"G1 Z{pen_z:.2f} F{pen_feedrate}"):
            return False

        # Check if it's a perfect circle (within tolerance)
        tolerance = 0.01  # 0.01mm tolerance
        if abs(radius_x - radius_y) < tolerance:
            # Use G2 arc command for perfect circle (more efficient)
            logger.info("Drawing perfect circle using G2 arc")
            if not fluidnc.send_gcode(
                f"G2 X{start_x:.2f} Y{start_y:.2f} I{-radius_x:.2f} J0 F{draw_feedrate}"
            ):
                return False
        else:
            # Draw ellipse with line segments
            logger.info(f"Drawing ellipse with {num_segments} segments")
            for i in range(1, num_segments + 1):
                t = (i / num_segments) * 2 * math.pi
                x = center_x + radius_x * math.cos(t)
                y = center_y + radius_y * math.sin(t)

                if not fluidnc.send_gcode(f"G1 X{x:.2f} Y{y:.2f} F{draw_feedrate}"):
                    return False

        # Lift pen to maximum height
        if not fluidnc.send_gcode(f"G0 Z{max_z:.2f}"):
            return False

        logger.info("Ellipse drawing complete")
        return True

    except Exception as e:
        logger.error(f"Error drawing ellipse: {e}")
        return False
