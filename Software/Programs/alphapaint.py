"""
AlphaPaint Python API Library.

Provides the communication layer and basic drawing primitives for
AlphaPaint drawing programs. Shape logic belongs in the programs themselves.

Communicates with the AlphaPaint daemon via JSON over stdin/stdout.
"""

import json
import sys
from typing import Any, Dict, Optional, Tuple


class AlphaPaintError(Exception):
    """Exception raised when the AlphaPaint API returns an error."""

    def __init__(self, message: str, code: int = 0):
        super().__init__(message)
        self.code = code


class Canvas:
    """Represents the drawable canvas area."""

    def __init__(self, origin: Tuple[float, float], size: Tuple[float, float]):
        self.origin = origin
        self.size = size
        self.width = size[0]
        self.height = size[1]
        self.center_x = size[0] / 2
        self.center_y = size[1] / 2

    @property
    def center(self) -> Tuple[float, float]:
        return (self.center_x, self.center_y)

    @property
    def min_dimension(self) -> float:
        return min(self.width, self.height)


class AlphaPaint:
    """
    AlphaPaint drawing API.

    Wraps the daemon's JSON protocol with a clean Python interface.
    Use as a context manager to ensure done() is always called.

    Example:
        from alphapaint import AlphaPaint

        with AlphaPaint() as ap:
            ap.move_to(10, 10)
            ap.pen_down()
            ap.draw_to(50, 10)
            ap.draw_to(50, 50)
            ap.pen_up()
    """

    def __init__(self):
        self._request_id = 0
        self._canvas: Optional[Canvas] = None
        self._pen_is_down = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.done()
        except Exception:
            pass
        return False

    # -- Communication --------------------------------------------------------

    def _call(self, method: str, params: Optional[Dict] = None) -> Any:
        """Send a JSON-RPC request to the daemon and return the result."""
        self._request_id += 1
        request = {
            "id": self._request_id,
            "method": method,
            "params": params or {}
        }

        print(json.dumps(request), flush=True)

        response_line = sys.stdin.readline()
        if not response_line:
            sys.exit(1)

        response = json.loads(response_line)

        if "event" in response:
            if response["event"] == "interrupted":
                sys.exit(0)

        if response.get("error"):
            error = response["error"]
            raise AlphaPaintError(error.get("message", "Unknown error"), error.get("code", 0))

        return response.get("result")

    # -- Queries --------------------------------------------------------------

    @property
    def canvas(self) -> Canvas:
        """Get canvas info (cached after first call)."""
        if self._canvas is None:
            result = self._call("query_canvas")
            self._canvas = Canvas(
                origin=tuple(result["origin"]),
                size=tuple(result["size"])
            )
        return self._canvas

    def query_machine(self) -> Dict:
        """Get machine info including limits and feedrates."""
        return self._call("query_machine")

    def query_position(self) -> Dict:
        """Get current position in all coordinate systems."""
        return self._call("query_position")

    # -- Pen control ----------------------------------------------------------

    def pen_up(self) -> None:
        """Lift the pen to safe height."""
        self._call("pen_up")
        self._pen_is_down = False

    def pen_down(self) -> None:
        """Lower the pen to drawing height."""
        self._call("pen_down")
        self._pen_is_down = True

    @property
    def pen_is_down(self) -> bool:
        return self._pen_is_down

    # -- Canvas coordinate drawing --------------------------------------------

    def move_to(self, x: float, y: float, wait: bool = True) -> None:
        """Rapid move to (x, y) in canvas coordinates (mm from canvas origin)."""
        self._call("canvas_move_to", {"x": x, "y": y, "wait": wait})

    def draw_to(self, x: float, y: float, feedrate: Optional[float] = None, wait: bool = True) -> None:
        """Draw a line to (x, y) in canvas coordinates."""
        params = {"x": x, "y": y, "wait": wait}
        if feedrate is not None:
            params["feedrate"] = feedrate
        self._call("canvas_draw_to", params)

    def draw_arc(self, x: float, y: float, i: float, j: float,
                 clockwise: bool = True, feedrate: Optional[float] = None, wait: bool = True) -> None:
        """
        Draw an arc to (x, y) in canvas coordinates.

        i, j are the offsets from the current position to the arc center.
        """
        params = {"x": x, "y": y, "i": i, "j": j, "clockwise": clockwise, "wait": wait}
        if feedrate is not None:
            params["feedrate"] = feedrate
        self._call("canvas_draw_arc", params)

    # -- Normalized coordinate drawing (0-1) ----------------------------------

    def move_to_normalized(self, x: float, y: float, wait: bool = True) -> None:
        """Rapid move using normalized coordinates (0=left/bottom, 1=right/top)."""
        self._call("normalized_move_to", {"x": x, "y": y, "wait": wait})

    def draw_to_normalized(self, x: float, y: float, feedrate: Optional[float] = None, wait: bool = True) -> None:
        """Draw using normalized coordinates (0-1 range)."""
        params = {"x": x, "y": y, "wait": wait}
        if feedrate is not None:
            params["feedrate"] = feedrate
        self._call("normalized_draw_to", params)

    # -- Machine coordinate drawing -------------------------------------------

    def move_to_machine(self, x: Optional[float] = None, y: Optional[float] = None,
                        z: Optional[float] = None, wait: bool = True) -> None:
        """Rapid move in machine coordinates."""
        params = {"wait": wait}
        if x is not None:
            params["x"] = x
        if y is not None:
            params["y"] = y
        if z is not None:
            params["z"] = z
        self._call("move_to", params)

    def draw_to_machine(self, x: Optional[float] = None, y: Optional[float] = None,
                        z: Optional[float] = None, feedrate: Optional[float] = None,
                        wait: bool = True) -> None:
        """Draw in machine coordinates."""
        params = {"wait": wait}
        if x is not None:
            params["x"] = x
        if y is not None:
            params["y"] = y
        if z is not None:
            params["z"] = z
        if feedrate is not None:
            params["feedrate"] = feedrate
        self._call("draw_to", params)

    # -- Control --------------------------------------------------------------

    def set_feedrate(self, feedrate: float) -> None:
        """Set the default drawing feedrate (mm/min)."""
        self._call("set_feedrate", {"feedrate": feedrate})

    def flush(self) -> None:
        """Wait for all queued commands to complete."""
        self._call("flush")

    def done(self, lift_pen: bool = True) -> None:
        """Signal that the program is complete."""
        self._call("done", {"lift_pen": lift_pen})
