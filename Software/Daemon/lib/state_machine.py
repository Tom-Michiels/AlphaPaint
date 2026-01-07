"""State Machine for AlphaPaint Daemon."""

import logging
import time
from typing import Dict, Tuple, Optional
from .console import ConsoleHandler
from .fluidnc import FluidNCHandler
from .drawing import draw_line, draw_ellipse


class StateMachine:
    """Manages the state machine and workflow for AlphaPaint."""

    # State constants
    STATE_STARTUP = "STARTUP"
    STATE_NOT_HOMED = "NOT_HOMED"
    STATE_HOMING = "HOMING"
    STATE_HOMED = "HOMED"
    STATE_CANVAS_SETUP = "CANVAS_SETUP"
    STATE_READY = "READY"
    STATE_DRAWING = "DRAWING"
    STATE_ERROR = "ERROR"

    def __init__(
        self,
        console: ConsoleHandler,
        fluidnc: FluidNCHandler,
        config: Dict
    ):
        """
        Initialize state machine.

        Args:
            console: Console handler instance
            fluidnc: FluidNC handler instance
            config: Configuration dictionary
        """
        self.console = console
        self.fluidnc = fluidnc
        self.config = config
        self.logger = logging.getLogger(__name__)

        # State variables
        self.state = self.STATE_STARTUP
        self.current_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.machine_limits = {
            'X': (0.0, 300.0),
            'Y': (-150.0, 150.0),
            'Z': (0.0, 50.0)
        }

        # Canvas definition
        self.point_B: Optional[Tuple[float, float]] = None
        self.point_C: Optional[Tuple[float, float]] = None
        self.pen_Z = config['machine']['pen_z_default']

        # Flags
        self.homed = False
        self.console_mode = "PASSIVE"
        self.active_axis = "X"
        self.precision_mode = False

        # Register Console callbacks
        self._register_callbacks()

    def _register_callbacks(self):
        """Register callbacks for Console messages."""
        self.console.on_message('BTN', self._on_button)
        self.console.on_message('POS', self._on_position)
        self.console.on_message('AXIS', self._on_axis)
        self.console.on_message('PRECISION', self._on_precision)
        self.console.on_message('STATUS', self._on_status)

    def start(self):
        """Start the state machine."""
        self.logger.info("State machine starting")
        self.transition(self.STATE_NOT_HOMED)

    def transition(self, new_state: str):
        """
        Transition to new state.

        Args:
            new_state: New state name
        """
        old_state = self.state
        self.state = new_state
        self.logger.info(f"State transition: {old_state} → {new_state}")

        # State entry actions
        if new_state == self.STATE_NOT_HOMED:
            self._enter_not_homed()
        elif new_state == self.STATE_CANVAS_SETUP:
            self._enter_canvas_setup()
        elif new_state == self.STATE_READY:
            self._enter_ready()

    def _enter_not_homed(self):
        """Enter NOT_HOMED state."""
        self.console.set_led('A', 'BLINK')  # Slow blink
        self.console.set_led('B', 'OFF')
        self.console.set_led('C', 'OFF')
        self.console.set_led('D', 'OFF')
        self.console.set_led('E', 'OFF')
        self.console.set_led('F', 'OFF')
        self.console.set_mode('PASSIVE')
        self.console_mode = 'PASSIVE'

    def _enter_canvas_setup(self):
        """Enter CANVAS_SETUP state."""
        self.console.set_led('A', 'ON')
        self.console.set_led('B', 'BLINK')  # Slow blink
        self.console.set_led('C', 'BLINK')  # Slow blink
        self.console.set_led('D', 'ON')     # Default pen Z active
        self.console.set_led('E', 'OFF')
        self.console.set_led('F', 'OFF')

    def _enter_ready(self):
        """Enter READY state."""
        self.console.set_led('A', 'ON')
        self.console.set_led('B', 'ON')
        self.console.set_led('C', 'ON')
        self.console.set_led('D', 'ON')
        self.console.set_led('E', 'BLINK')  # Slow blink - drawing available
        self.console.set_led('F', 'BLINK')  # Slow blink - drawing available

    # ========== Button Handlers ==========

    def _on_button(self, button: str, action: str):
        """
        Handle button press from Console.

        Args:
            button: Button identifier (A-G)
            action: SHORT or LONG
        """
        self.logger.info(f"Button {button} {action} in state {self.state}")

        # Button A - Homing
        if button == 'A':
            if action == 'SHORT':
                self._on_button_A_short()
            elif action == 'LONG':
                self._on_button_A_long()

        # Button B - Canvas corner 1
        elif button == 'B' and action == 'SHORT':
            self._on_button_B()

        # Button C - Canvas corner 2
        elif button == 'C' and action == 'SHORT':
            self._on_button_C()

        # Button D - Pen Z position
        elif button == 'D' and action == 'SHORT':
            self._on_button_D()

        # Button E - Draw line
        elif button == 'E' and action == 'SHORT':
            self._on_button_E()

        # Button F - Draw ellipse
        elif button == 'F' and action == 'SHORT':
            self._on_button_F()

    def _on_button_A_short(self):
        """Handle button A short press - start homing."""
        if self.state == self.STATE_NOT_HOMED:
            self._start_homing_sequence()

    def _on_button_A_long(self):
        """Handle button A long press - re-home from any state."""
        self.logger.info("Re-homing requested")

        # Cancel any active jog
        if self.console_mode == "ACTIVE":
            self.fluidnc.cancel_jog()

        # Reset state
        self.homed = False
        self.point_B = None
        self.point_C = None
        self.pen_Z = self.config['machine']['pen_z_default']

        # Go to NOT_HOMED
        self.transition(self.STATE_NOT_HOMED)

    def _start_homing_sequence(self):
        """Start homing sequence."""
        self.logger.info("Starting homing sequence")
        self.transition(self.STATE_HOMING)
        self.console.set_led('A', 'FAST_BLINK')

        # Execute homing in background thread to avoid blocking
        import threading
        threading.Thread(target=self._execute_homing, daemon=True).start()

    def _execute_homing(self):
        """Execute homing (runs in background thread)."""
        try:
            # Send home command to FluidNC
            if not self.fluidnc.home():
                self.logger.error("Homing failed")
                self.transition(self.STATE_ERROR)
                self._error_blink_all()
                return

            # Read machine limits
            self.machine_limits = self.fluidnc.get_limits()

            # Get current position
            status = self.fluidnc.get_status()
            if status:
                self.current_pos = status['position']

            # Send position to Console
            self.console.set_position('X', self.current_pos['X'])
            self.console.set_position('Y', self.current_pos['Y'])
            self.console.set_position('Z', self.current_pos['Z'])

            # Send limits to Console
            for axis in ['X', 'Y', 'Z']:
                min_val, max_val = self.machine_limits[axis]
                self.console.set_limit(axis, min_val, max_val)

            # Enable ACTIVE mode
            self.console.set_mode('ACTIVE')
            self.console_mode = 'ACTIVE'

            # Update flags
            self.homed = True

            # Transition to CANVAS_SETUP
            self.transition(self.STATE_CANVAS_SETUP)

            self.logger.info("Homing complete")

        except Exception as e:
            self.logger.error(f"Error during homing: {e}")
            self.transition(self.STATE_ERROR)
            self._error_blink_all()

    def _on_button_B(self):
        """Handle button B - set canvas corner 1."""
        if self.state not in [self.STATE_CANVAS_SETUP, self.STATE_READY]:
            return

        # Store current X, Y position
        self.point_B = (self.current_pos['X'], self.current_pos['Y'])
        self.logger.info(f"Point B set to {self.point_B}")

        # Visual feedback
        self.console.set_led('B', 'OFF')
        time.sleep(0.1)
        self.console.set_led('B', 'ON')

        # Check if ready
        self._check_ready()

    def _on_button_C(self):
        """Handle button C - set canvas corner 2."""
        if self.state not in [self.STATE_CANVAS_SETUP, self.STATE_READY]:
            return

        # Store current X, Y position
        self.point_C = (self.current_pos['X'], self.current_pos['Y'])
        self.logger.info(f"Point C set to {self.point_C}")

        # Visual feedback
        self.console.set_led('C', 'OFF')
        time.sleep(0.1)
        self.console.set_led('C', 'ON')

        # Check if ready
        self._check_ready()

    def _on_button_D(self):
        """Handle button D - set pen Z position."""
        if self.state not in [self.STATE_CANVAS_SETUP, self.STATE_READY]:
            return

        # Store current Z position
        self.pen_Z = self.current_pos['Z']
        self.logger.info(f"Pen Z position set to {self.pen_Z:.2f}")

        # Visual feedback
        self.console.set_led('D', 'OFF')
        time.sleep(0.1)
        self.console.set_led('D', 'ON')

    def _check_ready(self):
        """Check if both canvas points are defined and transition to READY."""
        if self.point_B is not None and self.point_C is not None:
            if self.state != self.STATE_READY:
                self.transition(self.STATE_READY)
                self.logger.info(f"Canvas defined: B={self.point_B}, C={self.point_C}")

    def _on_button_E(self):
        """Handle button E - draw line."""
        if self.state != self.STATE_READY:
            return

        if self.point_B is None or self.point_C is None:
            self.logger.warning("Cannot draw line: canvas not defined")
            self._error_blink_all()
            return

        self.logger.info("Drawing line")
        self.transition(self.STATE_DRAWING)
        self.console.set_led('E', 'FAST_BLINK')
        self.console.set_mode('PASSIVE')
        self.console_mode = 'PASSIVE'

        # Execute drawing in background thread
        import threading
        threading.Thread(target=self._execute_line_drawing, daemon=True).start()

    def _execute_line_drawing(self):
        """Execute line drawing (runs in background thread)."""
        try:
            success = draw_line(
                self.fluidnc,
                self.point_B,
                self.point_C,
                self.pen_Z,
                self.machine_limits,
                draw_feedrate=self.config['machine']['draw_feedrate'],
                pen_feedrate=self.config['machine']['pen_lift_feedrate']
            )

            if success:
                self.logger.info("Line drawing complete")
                self.transition(self.STATE_READY)
                self.console.set_mode('ACTIVE')
                self.console_mode = 'ACTIVE'
            else:
                self.logger.error("Line drawing failed")
                self.transition(self.STATE_ERROR)
                self._error_blink_all()

        except Exception as e:
            self.logger.error(f"Error during line drawing: {e}")
            self.transition(self.STATE_ERROR)
            self._error_blink_all()

    def _on_button_F(self):
        """Handle button F - draw ellipse."""
        if self.state != self.STATE_READY:
            return

        if self.point_B is None or self.point_C is None:
            self.logger.warning("Cannot draw ellipse: canvas not defined")
            self._error_blink_all()
            return

        self.logger.info("Drawing ellipse")
        self.transition(self.STATE_DRAWING)
        self.console.set_led('F', 'FAST_BLINK')
        self.console.set_mode('PASSIVE')
        self.console_mode = 'PASSIVE'

        # Execute drawing in background thread
        import threading
        threading.Thread(target=self._execute_ellipse_drawing, daemon=True).start()

    def _execute_ellipse_drawing(self):
        """Execute ellipse drawing (runs in background thread)."""
        try:
            success = draw_ellipse(
                self.fluidnc,
                self.point_B,
                self.point_C,
                self.pen_Z,
                self.machine_limits,
                draw_feedrate=self.config['machine']['draw_feedrate'],
                pen_feedrate=self.config['machine']['pen_lift_feedrate'],
                num_segments=self.config['drawing']['ellipse_segments']
            )

            if success:
                self.logger.info("Ellipse drawing complete")
                self.transition(self.STATE_READY)
                self.console.set_mode('ACTIVE')
                self.console_mode = 'ACTIVE'
            else:
                self.logger.error("Ellipse drawing failed")
                self.transition(self.STATE_ERROR)
                self._error_blink_all()

        except Exception as e:
            self.logger.error(f"Error during ellipse drawing: {e}")
            self.transition(self.STATE_ERROR)
            self._error_blink_all()

    # ========== Position/Axis Handlers ==========

    def _on_position(self, axis: str, value_str: str):
        """
        Handle position update from Console (Active mode).

        Args:
            axis: Axis (X, Y, or Z)
            value_str: Position value as string
        """
        if self.console_mode != "ACTIVE":
            return

        try:
            value = float(value_str)

            # Forward to FluidNC as jog command
            jog_feedrate = self.config['machine']['jog_feedrate']
            self.fluidnc.jog(axis, value, jog_feedrate)

            # Update internal state
            self.current_pos[axis] = value

        except ValueError:
            self.logger.error(f"Invalid position value: {value_str}")

    def _on_axis(self, axis: str, action: str):
        """
        Handle axis selection from Console.

        Args:
            axis: Axis (X, Y, or Z)
            action: SELECTED or other
        """
        if action == "SELECTED":
            self.active_axis = axis
            self.logger.debug(f"Active axis: {axis}")

    def _on_precision(self, axis: str, state: str):
        """
        Handle precision mode toggle.

        Args:
            axis: Axis (X, Y, or Z)
            state: ON or OFF
        """
        self.precision_mode = (state == "ON")
        self.logger.debug(f"Precision mode {axis}: {state}")

    def _on_status(self, status: str):
        """
        Handle status message from Console.

        Args:
            status: Status string
        """
        self.logger.debug(f"Console status: {status}")

    # ========== Helper Functions ==========

    def _error_blink_all(self):
        """Blink all LEDs to indicate error."""
        for led in ['A', 'B', 'C', 'D', 'E', 'F']:
            self.console.set_led(led, 'FAST_BLINK')
