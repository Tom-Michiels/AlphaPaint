"""State Machine for AlphaPaint Daemon."""

import logging
import threading
import time
from typing import Dict, Tuple, Optional
from .console import ConsoleHandler
from .fluidnc import FluidNCHandler
from .external_program import ExternalProgramHandler


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
    STATE_EXTERNAL_PROGRAM = "EXTERNAL_PROGRAM"
    STATE_ERROR = "ERROR"

    # States in which the machine position is trusted (homed and verified)
    HOMED_STATES = ("CANVAS_SETUP", "READY", "DRAWING", "EXTERNAL_PROGRAM")

    DEFAULT_LIMITS = {'X': (0.0, 300.0), 'Y': (-150.0, 150.0), 'Z': (0.0, 50.0)}
    # Maximum deviation (mm) between reported MPos and the configured home
    # position after homing before the result is rejected.
    HOME_POSITION_TOLERANCE = 0.5

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

        # Callbacks arrive from the console thread, the FluidNC read thread,
        # the homing thread and the external-program thread. All state
        # changes go through this lock.
        self._lock = threading.RLock()
        # Set while a machine-position loss is being handled, so repeated
        # alarm reports do not trigger repeated recoveries.
        self._position_lost_pending = False

        # Expected machine position right after homing (FluidNC mpos_mm)
        home = config.get('machine', {}).get('home_position') or {}
        self.home_position = {
            'X': float(home.get('X', 0.0)),
            'Y': float(home.get('Y', 0.0)),
            'Z': float(home.get('Z', 60.0)),
        }

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

        # External program handler
        self.external_handler: Optional[ExternalProgramHandler] = None
        self.active_external_button: Optional[str] = None

        # Register callbacks
        self._register_callbacks()

    def _register_callbacks(self):
        """Register callbacks for Console and FluidNC messages."""
        self._register_console_callbacks()

        # FluidNC status callback for continuous position updates
        self.fluidnc.on_status(self._on_fluidnc_status)
        # Anything that invalidates the machine position
        self.fluidnc.on_alarm(self._on_fluidnc_alarm)
        self.fluidnc.on_unexpected_reset(self._on_fluidnc_unexpected_reset)

    def _register_console_callbacks(self):
        self.console.on_message('BTN', self._on_button)
        self.console.on_message('POS', self._on_position)
        self.console.on_message('AXIS', self._on_axis)
        self.console.on_message('PRECISION', self._on_precision)
        self.console.on_message('STATUS', self._on_status)

    def replace_console(self, console: ConsoleHandler):
        """Attach a reconnected console without touching FluidNC.

        The console may have rebooted, so its display, limits, LEDs and mode
        are rebuilt from the daemon's state.
        """
        with self._lock:
            self.console = console
            self._register_console_callbacks()
            self.console.reset()
            if self.homed:
                for axis in ('X', 'Y', 'Z'):
                    min_val, max_val = self.machine_limits[axis]
                    self.console.set_limit(axis, min_val, max_val)
                    self.console.set_position(axis, self.current_pos[axis])
            # Re-run the entry action of the current state for LEDs and mode
            if self.state == self.STATE_NOT_HOMED:
                self._enter_not_homed()
            elif self.state == self.STATE_CANVAS_SETUP:
                self._enter_canvas_setup()
                self._set_console_mode('ACTIVE')
            elif self.state == self.STATE_READY:
                self._enter_ready()
                self._set_console_mode('ACTIVE')
            elif self.state == self.STATE_EXTERNAL_PROGRAM and self.active_external_button:
                self._enter_external_program(self.active_external_button, transition=False)
            elif self.state == self.STATE_ERROR:
                self._error_blink_all()
            self.logger.info(f"Console re-attached in state {self.state}")

    def _set_console_mode(self, mode: str):
        self.console.set_mode(mode)
        self.console_mode = mode

    def start(self):
        """Start the state machine."""
        self.logger.info("State machine starting")

        # Reset console to clear any old state (positions, limits, canvas)
        self.console.reset()

        self.transition(self.STATE_NOT_HOMED)

    def stop(self):
        """Stop the state machine and clean up resources."""
        self.logger.info("State machine stopping")

        # Stop any external program
        if self.external_handler:
            try:
                self.external_handler.interrupt()
            except Exception as e:
                self.logger.debug(f"Error stopping external program: {e}")
            self.external_handler = None
            self.active_external_button = None

        # Stop FluidNC background thread
        if self.fluidnc:
            try:
                self.fluidnc.stop()
            except Exception as e:
                self.logger.debug(f"Error stopping FluidNC: {e}")

    def transition(self, new_state: str):
        """
        Transition to new state.

        Args:
            new_state: New state name
        """
        with self._lock:
            self._transition_locked(new_state)

    def _transition_locked(self, new_state: str):
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
        self.console.set_led('G', 'OFF')
        self._set_console_mode('PASSIVE')

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

        # Set LEDs for external programs (E, F, G) based on config
        for button, config_key in [('E', 'button_e'), ('F', 'button_f'), ('G', 'button_g')]:
            if self._is_external_program_available(config_key):
                self.console.set_led(button, 'BLINK')  # Slow blink - program available
            else:
                self.console.set_led(button, 'OFF')

    def _is_external_program_available(self, config_key: str) -> bool:
        """
        Check if an external program is configured and available.

        Args:
            config_key: Config key like 'button_e', 'button_f', 'button_g'

        Returns:
            True if program is enabled and command exists
        """
        ext_programs = self.config.get('external_programs')
        if not ext_programs:
            return False

        ext_config = ext_programs.get(config_key)
        if not ext_config:
            return False

        enabled = ext_config.get('enabled', False)
        command = ext_config.get('command', '')

        return enabled and command

    # ========== Button Handlers ==========

    def _on_button(self, button: str, action: str):
        """
        Handle button press from Console.

        Args:
            button: Button identifier (A-G)
            action: SHORT or LONG
        """
        self.logger.info(f"Button {button} {action} in state {self.state}")
        with self._lock:
            self._on_button_locked(button, action)

    def _on_button_locked(self, button: str, action: str):
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

        # Button E - External program 1 or draw line
        elif button == 'E' and action == 'SHORT':
            self._on_button_E()

        # Button F - External program 2 or draw ellipse
        elif button == 'F' and action == 'SHORT':
            self._on_button_F()

        # Button G - External program 3
        elif button == 'G' and action == 'SHORT':
            self._on_button_G()

    def _on_button_A_short(self):
        """Handle button A short press - start homing."""
        if self.state == self.STATE_NOT_HOMED:
            self._start_homing_sequence()

    def _on_button_A_long(self):
        """Handle button A long press - re-home from any state."""
        if self.state == self.STATE_HOMING:
            self.logger.info("Re-homing ignored: homing already in progress")
            return
        self.logger.info("Re-homing requested")

        # If external program is running, interrupt it first. interrupt()
        # stops the machine with feed hold + reset, keeping its position.
        if self.external_handler:
            self.logger.info("Interrupting external program for re-homing")
            self.external_handler.interrupt()
            self.external_handler = None
            self.active_external_button = None

        # Reset state
        self.homed = False
        self.point_B = None
        self.point_C = None
        self.pen_Z = self.config['machine']['pen_z_default']

        # Start homing immediately. The homing thread brings the machine to a
        # controlled stop first; the pen lifts as the first homing cycle (Z).
        self._start_homing_sequence()

    def _start_homing_sequence(self):
        """Start homing sequence. Caller must hold the lock."""
        if self.state == self.STATE_HOMING:
            self.logger.info("Homing already in progress")
            return
        self.logger.info("Starting homing sequence")
        self._transition_locked(self.STATE_HOMING)
        self.homed = False
        # No handwheel jogs while homing: they would queue up in FluidNC and
        # run right after homing towards a stale target.
        self._set_console_mode('PASSIVE')
        self.console.set_led('A', 'FAST_BLINK')

        # Execute homing in background thread to avoid blocking
        threading.Thread(target=self._execute_homing, daemon=True).start()

    def _homing_failed(self, reason: str):
        self.logger.error(f"Homing failed: {reason}")
        with self._lock:
            self.homed = False
            self._transition_locked(self.STATE_ERROR)
            self._error_blink_all()

    def _execute_homing(self):
        """Execute homing (runs in background thread)."""
        try:
            # Direct serial access during homing: stop the read thread
            self.fluidnc.stop()

            # Bring any motion to a controlled stop and flush queued commands.
            # (A plain Ctrl-X while moving makes FluidNC lose its position.)
            self.fluidnc.stop_motion()
            time.sleep(0.5)

            if not self.fluidnc.home():
                self._homing_failed("FluidNC did not complete $H")
                return

            # Verify the result: older firmware answers "ok" even when the
            # cycle ended in an alarm, e.g. a limit switch that was missed.
            time.sleep(0.3)
            status = self.fluidnc.get_status()
            if not status:
                self._homing_failed("no status after homing")
                return
            if status['state'] != 'Idle':
                self._homing_failed(f"FluidNC state is {status['state']} after homing")
                return
            pos = status['position']
            off = {a: abs(pos[a] - self.home_position[a]) for a in ('X', 'Y', 'Z')}
            if max(off.values()) > self.HOME_POSITION_TOLERANCE:
                self._homing_failed(f"position {pos} is not the home position {self.home_position}")
                return

            # Read machine limits (with retry logic)
            limits = self.fluidnc.get_limits()
            if limits == self.DEFAULT_LIMITS:
                # Never run with guessed limits: the console and the
                # external programs trust them as the machine boundaries.
                self._homing_failed("could not read machine limits from FluidNC")
                return

            # Start the read thread first so the "ok"s of the auto-report
            # setup are matched to their commands (no orphaned oks).
            self.fluidnc.start()
            self.fluidnc.enable_auto_report(100)

            with self._lock:
                if self.state != self.STATE_HOMING:
                    self.logger.warning(f"Homing finished but state is {self.state}; ignoring result")
                    return
                self.machine_limits = limits
                self.current_pos = dict(pos)

                for axis in ['X', 'Y', 'Z']:
                    self.console.set_position(axis, self.current_pos[axis])
                    min_val, max_val = self.machine_limits[axis]
                    self.console.set_limit(axis, min_val, max_val)

                self._position_lost_pending = False
                self.homed = True
                self._set_console_mode('ACTIVE')
                self._transition_locked(self.STATE_CANVAS_SETUP)

            self.logger.info(f"Homing complete and verified at {pos}")

        except Exception as e:
            self.logger.error(f"Error during homing: {e}", exc_info=True)
            self._homing_failed(str(e))

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
        """Handle button E - external program 1."""
        if self.state != self.STATE_READY:
            return

        if self.point_B is None or self.point_C is None:
            self.logger.warning("Cannot run program: canvas not defined")
            self._error_blink_all()
            return

        self._try_start_external_program('E', 'button_e')

    def _on_button_F(self):
        """Handle button F - external program 2."""
        if self.state != self.STATE_READY:
            return

        if self.point_B is None or self.point_C is None:
            self.logger.warning("Cannot run program: canvas not defined")
            self._error_blink_all()
            return

        self._try_start_external_program('F', 'button_f')

    def _on_button_G(self):
        """Handle button G - external program 3."""
        if self.state != self.STATE_READY:
            return

        if self.point_B is None or self.point_C is None:
            self.logger.warning("Cannot run program: canvas not defined")
            self._error_blink_all()
            return

        self._try_start_external_program('G', 'button_g')

    def _try_start_external_program(self, button: str, config_key: str):
        """
        Try to start an external program for the given button.
        Logs extensive debug info if program is not configured.
        """
        # Log all config info for debugging
        self.logger.info(f"Button {button} pressed - checking external program config")
        self.logger.info(f"Config keys present: {list(self.config.keys())}")

        ext_programs = self.config.get('external_programs')
        if ext_programs is None:
            self.logger.error(f"Button {button}: 'external_programs' section MISSING from config!")
            self.logger.error(f"Full config: {self.config}")
            self._error_blink_all()
            return

        self.logger.info(f"external_programs keys: {list(ext_programs.keys())}")

        ext_config = ext_programs.get(config_key)
        if ext_config is None:
            self.logger.error(f"Button {button}: '{config_key}' section MISSING from external_programs!")
            self.logger.error(f"external_programs content: {ext_programs}")
            self._error_blink_all()
            return

        self.logger.info(f"{config_key} config: {ext_config}")

        enabled = ext_config.get('enabled', False)
        command = ext_config.get('command', '')

        self.logger.info(f"enabled={enabled} (type={type(enabled).__name__})")
        self.logger.info(f"command='{command}' (type={type(command).__name__})")

        if not enabled:
            self.logger.error(f"Button {button}: external program is DISABLED (enabled={enabled})")
            self._error_blink_all()
            return

        if not command:
            self.logger.error(f"Button {button}: external program has NO COMMAND (command='{command}')")
            self._error_blink_all()
            return

        # All checks passed - start the program
        self._start_external_program(button, ext_config)

    # ========== External Program Handling ==========

    def _start_external_program(self, button: str, ext_config: Dict):
        """
        Start an external drawing program.

        Args:
            button: Button identifier (E, F, or G)
            ext_config: External program configuration dict
        """
        command = ext_config.get('command', '')
        args = ext_config.get('args', [])
        timeout = ext_config.get('timeout', 0)
        name = ext_config.get('name', f'Program {button}')

        self.logger.info(f"Starting external program '{name}': {command}")

        # Calculate canvas origin and size from points B and C
        min_x = min(self.point_B[0], self.point_C[0])
        min_y = min(self.point_B[1], self.point_C[1])
        max_x = max(self.point_B[0], self.point_C[0])
        max_y = max(self.point_B[1], self.point_C[1])

        canvas_origin = (min_x, min_y)
        canvas_size = (max_x - min_x, max_y - min_y)

        # Create handler
        self.external_handler = ExternalProgramHandler(
            fluidnc=self.fluidnc,
            config=self.config,
            canvas_origin=canvas_origin,
            canvas_size=canvas_size,
            pen_z=self.pen_Z,
            machine_limits=self.machine_limits,
            on_complete=self._on_external_program_complete
        )

        # Start program
        if self.external_handler.start(command, args, timeout):
            self.active_external_button = button
            self._enter_external_program(button)
        else:
            self.logger.error(f"Failed to start external program: {command}")
            self.external_handler = None
            self._error_blink_all()

    def _enter_external_program(self, button: str, transition: bool = True):
        """
        Enter EXTERNAL_PROGRAM state.

        Args:
            button: Button that triggered the program (E, F, or G)
            transition: False when only refreshing the console
        """
        if transition:
            self._transition_locked(self.STATE_EXTERNAL_PROGRAM)

        # All LEDs off except active button (fast blink)
        for led in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
            self.console.set_led(led, 'OFF')
        self.console.set_led(button, 'FAST_BLINK')

        # Set console to passive mode
        self._set_console_mode('PASSIVE')

    def _on_external_program_complete(self, success: bool):
        """
        Callback when external program completes.

        Args:
            success: True if program completed successfully
        """
        self.logger.info(f"External program completed (success={success})")
        with self._lock:
            # If we're no longer in EXTERNAL_PROGRAM state (e.g., re-homing was
            # triggered), the state machine has already moved on
            if self.state != self.STATE_EXTERNAL_PROGRAM:
                self.logger.info("State already changed, skipping completion transition")
                return

            self.external_handler = None
            self.active_external_button = None

            if success:
                self._transition_locked(self.STATE_READY)
                self._set_console_mode('ACTIVE')
            else:
                self._transition_locked(self.STATE_ERROR)
                self._error_blink_all()

    # ========== Position/Axis Handlers ==========

    def _on_position(self, axis: str, value_str: str):
        """
        Handle position update from Console (Active mode).

        Args:
            axis: Axis (X, Y, or Z)
            value_str: Position value as string
        """
        with self._lock:
            if self.console_mode != "ACTIVE" or not self.homed:
                return
            if self.state not in (self.STATE_CANVAS_SETUP, self.STATE_READY):
                return
            if axis not in self.machine_limits:
                self.logger.error(f"Invalid jog axis: {axis!r}")
                return

            try:
                value = float(value_str)
            except ValueError:
                self.logger.error(f"Invalid position value: {value_str!r}")
                return

            # Never forward a target outside the machine, whatever the console
            # sent (corrupted line, stale limits after a console reboot, ...)
            min_val, max_val = self.machine_limits[axis]
            if not (min_val <= value <= max_val):
                clamped = min(max(value, min_val), max_val)
                self.logger.warning(f"Jog target {axis}{value} outside [{min_val}, {max_val}], "
                                    f"clamped to {clamped}")
                value = clamped

            # Forward to FluidNC as jog command
            jog_feedrate = self.config['machine']['jog_feedrate']
            self.fluidnc.jog(axis, value, jog_feedrate)

    def _on_axis(self, axis: str, action: str, *extra):
        """
        Handle axis selection from Console.

        Args:
            axis: Axis (X, Y, or Z)
            action: SELECTED, or PRECISION (for precision mode toggle)
            *extra: Additional arguments (e.g., ON/OFF for precision mode)
        """
        if action == "SELECTED":
            self.active_axis = axis
            self.logger.debug(f"Active axis: {axis}")
        elif action == "PRECISION" and extra:
            # Handle AXIS:X:PRECISION:ON/OFF format
            self.precision_mode = (extra[0] == "ON")
            self.logger.debug(f"Precision mode {axis}: {extra[0]}")

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

    def _on_fluidnc_status(self, status: Dict):
        """
        Callback for FluidNC status updates (called automatically at 10Hz).

        Args:
            status: Status dict with 'state' and 'position' keys
        """
        if status.get('state') in ('Alarm', 'Critical') and self.state in self.HOMED_STATES:
            # Backup for a missed alarm message
            self._position_lost("FluidNC reports Alarm state")

        if 'position' in status:
            pos = status['position']
            # Update internal position
            self.current_pos = pos
            # Only update console displays in PASSIVE mode
            # In ACTIVE mode, the Console is the position master (encoder control)
            if self.console_mode == 'PASSIVE':
                self.console.set_position('X', pos['X'])
                self.console.set_position('Y', pos['Y'])
                self.console.set_position('Z', pos['Z'])

    def _on_fluidnc_alarm(self, text: str):
        """FluidNC raised an alarm (hard/soft limit, reset in motion, ...)."""
        if self.state == self.STATE_HOMING:
            return  # The homing thread evaluates the outcome itself
        if self.state in self.HOMED_STATES:
            self._position_lost(f"FluidNC alarm: {text}")

    def _on_fluidnc_unexpected_reset(self):
        """FluidNC rebooted or was reset by someone else."""
        if self.state == self.STATE_HOMING:
            return
        self._position_lost("FluidNC restarted unexpectedly")

    def _position_lost(self, reason: str):
        """The machine position can no longer be trusted: stop and re-home.

        Called from the FluidNC read thread, so the actual work runs in a
        separate thread (it needs the read thread to talk to FluidNC).
        """
        with self._lock:
            if self._position_lost_pending:
                return
            self._position_lost_pending = True
        self.logger.error(f"Machine position lost: {reason} - homing required")
        threading.Thread(target=self._handle_position_lost, daemon=True).start()

    def _handle_position_lost(self):
        with self._lock:
            handler = self.external_handler
            self.external_handler = None
            self.active_external_button = None
            self.homed = False
            self.point_B = None
            self.point_C = None
        if handler:
            try:
                handler.interrupt()
            except Exception as e:
                self.logger.error(f"Error interrupting external program: {e}")
        with self._lock:
            if self.state != self.STATE_HOMING:
                self._transition_locked(self.STATE_NOT_HOMED)

    def _error_blink_all(self):
        """Blink all LEDs to indicate error."""
        for led in ['A', 'B', 'C', 'D', 'E', 'F']:
            self.console.set_led(led, 'FAST_BLINK')
