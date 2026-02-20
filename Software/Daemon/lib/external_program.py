"""External Program Handler for AlphaPaint Daemon.

Manages communication with external drawing programs via stdin/stdout JSON protocol.
"""

import json
import logging
import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from .fluidnc import FluidNCHandler


class ExternalProgramHandler:
    """Handles communication with external drawing programs."""

    # Error codes
    ERR_LIMIT_EXCEEDED = 100
    ERR_CANVAS_UNDEFINED = 101
    ERR_INVALID_PARAMS = 102
    ERR_GCODE_ERROR = 103
    ERR_COMMUNICATION_ERROR = 104
    ERR_INVALID_METHOD = 105
    ERR_INTERRUPTED = 106

    def __init__(
        self,
        fluidnc: FluidNCHandler,
        config: Dict,
        canvas_origin: Tuple[float, float],
        canvas_size: Tuple[float, float],
        pen_z: float,
        machine_limits: Dict[str, Tuple[float, float]],
        on_complete: Optional[Callable[[bool], None]] = None
    ):
        """
        Initialize external program handler.

        Args:
            fluidnc: FluidNC handler instance
            config: Configuration dictionary
            canvas_origin: Bottom-left corner of canvas (min X, min Y)
            canvas_size: Canvas dimensions (width, height)
            pen_z: Pen contact Z height
            machine_limits: Machine axis limits
            on_complete: Callback when program completes (success: bool)
        """
        self.fluidnc = fluidnc
        self.config = config
        self.canvas_origin = canvas_origin
        self.canvas_size = canvas_size
        self.pen_z = pen_z
        self.machine_limits = machine_limits
        self.on_complete = on_complete
        self.logger = logging.getLogger(__name__)

        # State
        self.process: Optional[subprocess.Popen] = None
        self.current_feedrate = config['machine']['draw_feedrate']
        self._pen_is_down = False
        self._pen_lock = threading.Lock()  # Protects pen_is_down state
        self._running = False
        self._interrupted = False
        self._done_called = False  # True when program calls 'done' API

        # Thread for reading program output
        self._read_thread: Optional[threading.Thread] = None

    @property
    def pen_is_down(self) -> bool:
        """Thread-safe getter for pen state."""
        with self._pen_lock:
            return self._pen_is_down

    @pen_is_down.setter
    def pen_is_down(self, value: bool):
        """Thread-safe setter for pen state."""
        with self._pen_lock:
            self._pen_is_down = value

    def start(self, command: str, args: List[str] = None, timeout: int = 0) -> bool:
        """
        Start external program.

        Args:
            command: Path to executable
            args: Command line arguments
            timeout: Maximum runtime in seconds (0 = unlimited)

        Returns:
            True if started successfully
        """
        if self._running:
            self.logger.warning("External program already running")
            return False

        args = args or []

        try:
            self.process = subprocess.Popen(
                [command] + args,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1  # Line buffered
            )
            self._running = True
            self._interrupted = False
            self._done_called = False
            self.pen_is_down = False

            # Start read thread
            self._read_thread = threading.Thread(
                target=self._communication_loop,
                args=(timeout,),
                daemon=True
            )
            self._read_thread.start()

            # Start stderr logging thread
            self._stderr_thread = threading.Thread(
                target=self._stderr_loop,
                daemon=True
            )
            self._stderr_thread.start()

            self.logger.info(f"Started external program: {command}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to start external program: {e}")
            return False

    def interrupt(self) -> bool:
        """
        Interrupt running program.

        Sends interrupt event, waits for graceful exit, then forces termination.

        Returns:
            True if program was interrupted
        """
        if not self._running or not self.process:
            return False

        self._interrupted = True
        self.logger.info("Interrupting external program")

        # Send interrupt event
        try:
            self._send_event("interrupted", {"reason": "user_cancel"})
        except Exception:
            pass

        # Wait for graceful exit (2 seconds)
        try:
            self.process.wait(timeout=2.0)
            self.logger.info("External program exited gracefully")
        except subprocess.TimeoutExpired:
            # Send SIGTERM
            self.logger.info("Sending SIGTERM to external program")
            self.process.terminate()

            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                # Force kill
                self.logger.info("Sending SIGKILL to external program")
                self.process.kill()
                self.process.wait()

        self._running = False

        # Lift pen for safety
        self._emergency_pen_lift()

        return True

    def _emergency_pen_lift(self):
        """Lift pen to safe height."""
        max_z = self.machine_limits['Z'][1]
        self.fluidnc.send_gcode(f"G0 Z{max_z:.2f}", wait_ok=True, timeout=5.0)
        self.pen_is_down = False

    def _stderr_loop(self):
        """Read and log stderr from external program."""
        try:
            while self._running and self.process and self.process.poll() is None:
                line = self.process.stderr.readline()
                if not line:
                    break
                line = line.rstrip('\n\r')
                if line:
                    self.logger.info(f"Program stderr: {line}")
        except Exception as e:
            self.logger.error(f"Error reading stderr: {e}")

    def _communication_loop(self, timeout: int):
        """
        Main communication loop with external program.

        Args:
            timeout: Maximum runtime in seconds (0 = unlimited)
        """
        start_time = time.time()

        try:
            while self._running and self.process and self.process.poll() is None:
                # Check timeout
                if timeout > 0 and (time.time() - start_time) > timeout:
                    self.logger.warning("External program timeout")
                    self.interrupt()
                    break

                # Read request from program stdout
                try:
                    line = self.process.stdout.readline()
                    if not line:
                        # EOF - program closed stdout
                        break

                    line = line.strip()
                    if not line:
                        continue

                    # Parse JSON request
                    try:
                        request = json.loads(line)
                        self._handle_request(request)
                    except json.JSONDecodeError as e:
                        self.logger.error(f"Invalid JSON from program: {e}")

                except Exception as e:
                    self.logger.error(f"Error reading from program: {e}")
                    break

        finally:
            self._running = False

            # Ensure pen is up
            if self.pen_is_down:
                self._emergency_pen_lift()

            # Wait for process to exit and get return code
            if self.process:
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.logger.warning("Process did not exit after done, terminating")
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()

            # Call completion callback
            if self.on_complete:
                # Success if: done was called, not interrupted, and process exited cleanly
                success = self._done_called and not self._interrupted and (
                    self.process is None or self.process.returncode == 0
                )
                self.on_complete(success)

    def _handle_request(self, request: Dict):
        """
        Process incoming API request.

        Args:
            request: Parsed JSON request
        """
        request_id = request.get('id', 0)
        method = request.get('method', '')
        params = request.get('params', {})

        # Method dispatch table
        handlers = {
            'query_machine': self._api_query_machine,
            'query_canvas': self._api_query_canvas,
            'query_position': self._api_query_position,
            'pen_up': self._api_pen_up,
            'pen_up_fast': self._api_pen_up_fast,
            'pen_down': self._api_pen_down,
            'move_to': self._api_move_to,
            'draw_to': self._api_draw_to,
            'draw_arc': self._api_draw_arc,
            'canvas_move_to': self._api_canvas_move_to,
            'canvas_draw_to': self._api_canvas_draw_to,
            'canvas_draw_arc': self._api_canvas_draw_arc,
            'normalized_move_to': self._api_normalized_move_to,
            'normalized_draw_to': self._api_normalized_draw_to,
            'set_feedrate': self._api_set_feedrate,
            'flush': self._api_flush,
            'done': self._api_done,
        }

        handler = handlers.get(method)
        if handler:
            try:
                result = handler(params)
                self._send_response(request_id, result=result)
            except Exception as e:
                self.logger.error(f"Error handling {method}: {e}")
                self._send_response(
                    request_id,
                    error={'code': self.ERR_GCODE_ERROR, 'message': str(e)}
                )
        else:
            self._send_response(
                request_id,
                error={'code': self.ERR_INVALID_METHOD, 'message': f"Unknown method: {method}"}
            )

    def _send_response(self, request_id: int, result: Any = None, error: Dict = None):
        """Send response to program."""
        response = {
            'id': request_id,
            'result': result,
            'error': error
        }
        self._write_line(json.dumps(response))

    def _send_event(self, event_type: str, data: Dict = None):
        """Send unsolicited event to program."""
        event = {'event': event_type}
        if data:
            event.update(data)
        self._write_line(json.dumps(event))

    def _write_line(self, line: str):
        """Write line to program stdin."""
        if self.process and self.process.stdin:
            try:
                self.process.stdin.write(line + '\n')
                self.process.stdin.flush()
            except Exception as e:
                self.logger.error(f"Error writing to program: {e}")

    # Coordinate transforms

    def _canvas_to_machine(self, x: float, y: float) -> Tuple[float, float]:
        """Convert canvas coordinates to machine coordinates."""
        return (
            self.canvas_origin[0] + x,
            self.canvas_origin[1] + y
        )

    def _normalized_to_machine(self, x: float, y: float) -> Tuple[float, float]:
        """Convert normalized (0-1) coordinates to machine coordinates."""
        return (
            self.canvas_origin[0] + x * self.canvas_size[0],
            self.canvas_origin[1] + y * self.canvas_size[1]
        )

    def _machine_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        """Convert machine coordinates to canvas coordinates."""
        return (
            x - self.canvas_origin[0],
            y - self.canvas_origin[1]
        )

    def _validate_machine_position(self, x: float = None, y: float = None, z: float = None) -> Tuple[bool, str]:
        """Check if position is within machine limits."""
        if x is not None:
            min_x, max_x = self.machine_limits['X']
            if x < min_x or x > max_x:
                return False, f"X position {x} out of bounds [{min_x}, {max_x}]"
        if y is not None:
            min_y, max_y = self.machine_limits['Y']
            if y < min_y or y > max_y:
                return False, f"Y position {y} out of bounds [{min_y}, {max_y}]"
        if z is not None:
            min_z, max_z = self.machine_limits['Z']
            if z < min_z or z > max_z:
                return False, f"Z position {z} out of bounds [{min_z}, {max_z}]"
        return True, None

    def _get_current_position(self) -> Dict[str, float]:
        """Get current machine position from auto-report cache.

        Uses cached position from auto-report to avoid disrupting the serial
        command/response stream. Falls back to explicit query only when
        no cached position exists.
        """
        cached = self.fluidnc.get_cached_status(max_age=1.0)
        if cached and 'position' in cached:
            return cached['position']

        # Fallback: only happens if auto-report not yet active
        self.logger.warning("No cached position available, falling back to explicit status query")
        status = self.fluidnc.get_status()
        if status and 'position' in status:
            return status['position']
        return {'X': 0.0, 'Y': 0.0, 'Z': 0.0}

    # API method implementations

    def _api_query_machine(self, params: Dict) -> Dict:
        """Query full machine information."""
        pos = self._get_current_position()
        return {
            'limits': {
                'x': list(self.machine_limits['X']),
                'y': list(self.machine_limits['Y']),
                'z': list(self.machine_limits['Z']),
            },
            'position': {'x': pos['X'], 'y': pos['Y'], 'z': pos['Z']},
            'pen_z': self.pen_z,
            'feedrates': {
                'draw': self.config['machine']['draw_feedrate'],
                'rapid': self.config['machine']['jog_feedrate'],
                'pen': self.config['machine']['pen_lift_feedrate'],
            }
        }

    def _api_query_canvas(self, params: Dict) -> Dict:
        """Query canvas rectangle definition."""
        return {
            'origin': list(self.canvas_origin),
            'size': list(self.canvas_size),
            'corners': {
                'bottom_left': list(self.canvas_origin),
                'top_right': [
                    self.canvas_origin[0] + self.canvas_size[0],
                    self.canvas_origin[1] + self.canvas_size[1]
                ]
            }
        }

    def _api_query_position(self, params: Dict) -> Dict:
        """Query current pen position in all coordinate systems."""
        pos = self._get_current_position()
        canvas_x, canvas_y = self._machine_to_canvas(pos['X'], pos['Y'])

        return {
            'machine': {'x': pos['X'], 'y': pos['Y'], 'z': pos['Z']},
            'canvas': {'x': canvas_x, 'y': canvas_y},
            'canvas_normalized': {
                'x': canvas_x / self.canvas_size[0] if self.canvas_size[0] > 0 else 0,
                'y': canvas_y / self.canvas_size[1] if self.canvas_size[1] > 0 else 0,
            },
            'pen_down': self.pen_is_down
        }

    def _api_pen_up(self, params: Dict) -> Dict:
        """Lift pen to safe height."""
        max_z = self.machine_limits['Z'][1]
        success = self.fluidnc.send_gcode(
            f"G0 Z{max_z:.2f}",
            wait_ok=True,
            timeout=5.0
        )
        if success:
            self.pen_is_down = False
        return {'success': success, 'z': max_z}

    def _api_pen_down(self, params: Dict) -> Dict:
        """Lower pen to drawing height."""
        feedrate = self.config['machine']['pen_lift_feedrate']
        success = self.fluidnc.send_gcode(
            f"G1 Z{self.pen_z:.2f} F{feedrate}",
            wait_ok=True,
            timeout=5.0
        )
        if success:
            self.pen_is_down = True
        return {'success': success, 'z': self.pen_z}

    def _api_pen_up_fast(self, params: Dict) -> Dict:
        """Lift pen 8mm above paper for quick repositioning."""
        target_z = self.pen_z + 8.0
        # Clamp to max Z if needed
        max_z = self.machine_limits['Z'][1]
        if target_z > max_z:
            target_z = max_z
        success = self.fluidnc.send_gcode(
            f"G0 Z{target_z:.2f}",
            wait_ok=True,
            timeout=5.0
        )
        if success:
            self.pen_is_down = False
        return {'success': success, 'z': target_z}

    def _api_move_to(self, params: Dict) -> Dict:
        """Rapid move in machine coordinates."""
        x = params.get('x')
        y = params.get('y')
        z = params.get('z')
        wait = params.get('wait', True)  # Wait for completion by default

        if x is None and y is None and z is None:
            raise ValueError("At least one axis must be specified")

        valid, error = self._validate_machine_position(x, y, z)
        if not valid:
            raise ValueError(error)

        # Build G0 command
        cmd_parts = ["G0"]
        if x is not None:
            cmd_parts.append(f"X{x:.2f}")
        if y is not None:
            cmd_parts.append(f"Y{y:.2f}")
        if z is not None:
            cmd_parts.append(f"Z{z:.2f}")

        success = self.fluidnc.send_gcode(" ".join(cmd_parts), wait_ok=wait)

        if wait and success:
            pos = self._get_current_position()
            return {
                'success': success,
                'position': {'x': pos['X'], 'y': pos['Y'], 'z': pos['Z']}
            }
        elif wait:
            return {'success': False}
        else:
            return {'success': success, 'queued': True}

    def _api_draw_to(self, params: Dict) -> Dict:
        """Linear interpolation move in machine coordinates."""
        x = params.get('x')
        y = params.get('y')
        z = params.get('z')
        feedrate = params.get('feedrate', self.current_feedrate)
        wait = params.get('wait', True)  # Wait for completion by default

        if x is None and y is None and z is None:
            raise ValueError("At least one axis must be specified")

        valid, error = self._validate_machine_position(x, y, z)
        if not valid:
            raise ValueError(error)

        # Build G1 command
        cmd_parts = ["G1"]
        if x is not None:
            cmd_parts.append(f"X{x:.2f}")
        if y is not None:
            cmd_parts.append(f"Y{y:.2f}")
        if z is not None:
            cmd_parts.append(f"Z{z:.2f}")
        cmd_parts.append(f"F{feedrate}")

        success = self.fluidnc.send_gcode(" ".join(cmd_parts), wait_ok=wait)

        if wait and success:
            pos = self._get_current_position()
            return {
                'success': success,
                'position': {'x': pos['X'], 'y': pos['Y'], 'z': pos['Z']}
            }
        elif wait:
            return {'success': False}
        else:
            return {'success': success, 'queued': True}

    def _api_draw_arc(self, params: Dict) -> Dict:
        """Arc interpolation in machine coordinates."""
        x = params.get('x')
        y = params.get('y')
        i = params.get('i')
        j = params.get('j')
        clockwise = params.get('clockwise', True)
        feedrate = params.get('feedrate', self.current_feedrate)
        wait = params.get('wait', True)  # Wait for completion by default

        if x is None or y is None or i is None or j is None:
            raise ValueError("x, y, i, j are all required for arc")

        valid, error = self._validate_machine_position(x, y)
        if not valid:
            raise ValueError(error)

        # G2 = clockwise, G3 = counter-clockwise
        cmd = "G2" if clockwise else "G3"
        gcode = f"{cmd} X{x:.2f} Y{y:.2f} I{i:.2f} J{j:.2f} F{feedrate}"

        success = self.fluidnc.send_gcode(gcode, wait_ok=wait)

        if wait and success:
            pos = self._get_current_position()
            return {
                'success': success,
                'position': {'x': pos['X'], 'y': pos['Y'], 'z': pos['Z']}
            }
        elif wait:
            return {'success': False}
        else:
            return {'success': success, 'queued': True}

    def _api_canvas_move_to(self, params: Dict) -> Dict:
        """Rapid move in canvas coordinates."""
        x = params.get('x', 0)
        y = params.get('y', 0)
        wait = params.get('wait', True)
        mx, my = self._canvas_to_machine(x, y)
        return self._api_move_to({'x': mx, 'y': my, 'wait': wait})

    def _api_canvas_draw_to(self, params: Dict) -> Dict:
        """Draw in canvas coordinates."""
        x = params.get('x', 0)
        y = params.get('y', 0)
        feedrate = params.get('feedrate', self.current_feedrate)
        wait = params.get('wait', True)
        mx, my = self._canvas_to_machine(x, y)
        return self._api_draw_to({'x': mx, 'y': my, 'feedrate': feedrate, 'wait': wait})

    def _api_canvas_draw_arc(self, params: Dict) -> Dict:
        """Arc in canvas coordinates."""
        x = params.get('x')
        y = params.get('y')
        i = params.get('i')
        j = params.get('j')
        clockwise = params.get('clockwise', True)
        feedrate = params.get('feedrate', self.current_feedrate)
        wait = params.get('wait', True)

        if x is None or y is None or i is None or j is None:
            raise ValueError("x, y, i, j are all required for arc")

        mx, my = self._canvas_to_machine(x, y)
        # i and j are offsets, not absolute positions, so they stay the same
        return self._api_draw_arc({
            'x': mx, 'y': my, 'i': i, 'j': j,
            'clockwise': clockwise, 'feedrate': feedrate, 'wait': wait
        })

    def _api_normalized_move_to(self, params: Dict) -> Dict:
        """Rapid move using normalized (0-1) coordinates."""
        x = params.get('x', 0)
        y = params.get('y', 0)
        wait = params.get('wait', True)
        mx, my = self._normalized_to_machine(x, y)
        return self._api_move_to({'x': mx, 'y': my, 'wait': wait})

    def _api_normalized_draw_to(self, params: Dict) -> Dict:
        """Draw using normalized (0-1) coordinates."""
        x = params.get('x', 0)
        y = params.get('y', 0)
        feedrate = params.get('feedrate', self.current_feedrate)
        wait = params.get('wait', True)
        mx, my = self._normalized_to_machine(x, y)
        return self._api_draw_to({'x': mx, 'y': my, 'feedrate': feedrate, 'wait': wait})

    def _api_set_feedrate(self, params: Dict) -> Dict:
        """Set default drawing feedrate."""
        feedrate = params.get('feedrate')
        if feedrate is None or feedrate <= 0:
            raise ValueError("feedrate must be a positive number")
        self.current_feedrate = feedrate
        return {'feedrate': self.current_feedrate}

    def _api_flush(self, params: Dict) -> Dict:
        """
        Wait for all queued commands to complete.

        This sends a G4 P0 (dwell for 0 seconds) which forces FluidNC to
        complete all buffered commands before acknowledging.
        """
        # G4 P0 is a "dwell" command that waits for the motion buffer to empty
        # before returning "ok"
        success = self.fluidnc.send_gcode("G4 P0", wait_ok=True, timeout=60.0)

        pos = self._get_current_position()
        return {
            'success': success,
            'position': {'x': pos['X'], 'y': pos['Y'], 'z': pos['Z']}
        }

    def _api_done(self, params: Dict) -> Dict:
        """Signal program completion."""
        lift_pen = params.get('lift_pen', True)
        wait_timeout = params.get('timeout', 60.0)

        # If interrupted, skip the wait - just return quickly
        if self._interrupted:
            self.logger.info("Program was interrupted, skipping wait_idle")
            self._done_called = True
            return {'success': True}

        # Wait for all motion to complete before lifting pen
        self.logger.info("Waiting for FluidNC to complete all motion...")
        if not self.fluidnc.wait_idle(timeout=wait_timeout):
            self.logger.warning("Timeout waiting for FluidNC idle before done")

        if lift_pen and self.pen_is_down:
            self._api_pen_up({})

        # Wait again after pen lift to ensure it's complete
        self.fluidnc.wait_idle(timeout=5.0)

        # Mark that done was called - program should exit after receiving response
        self._done_called = True
        self.logger.info("External program done, FluidNC is idle")

        return {'success': True}
