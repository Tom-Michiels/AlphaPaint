"""FluidNC Handler for CNC controller communication."""

import serial
import threading
import logging
import time
import re
import queue
from typing import Optional, Dict, Tuple, Callable


class FluidNCHandler:
    """Handles communication with FluidNC CNC controller."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize FluidNC handler.

        Args:
            port: Serial port path (e.g., /dev/ttyUSB1)
            baudrate: Serial baudrate (default: 115200)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.logger = logging.getLogger(__name__)
        self._serial_lock = threading.Lock()  # Protects all serial operations

        # Background read thread
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._response_queue: queue.Queue = queue.Queue()
        self._status_callback: Optional[Callable[[Dict], None]] = None

        # Buffer management for flow control (character-counting protocol)
        self._buffer_size = 128  # FluidNC/Grbl RX buffer size
        self._buffer_used = 0  # Bytes currently in buffer
        self._pending_commands: list = []  # Track sent command lengths
        self._buffer_lock = threading.Lock()
        self._buffer_space_event = threading.Event()
        self._buffer_space_event.set()  # Initially have space

    def connect(self) -> bool:
        """
        Connect to FluidNC serial port.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Wait for FluidNC to initialize
            self.logger.info(f"Connected to FluidNC on {self.port}")

            # Send soft reset to clear any stuck state
            self.serial.write(b'\x18')
            self.logger.info("Soft reset sent to clear any stuck state")
            time.sleep(2.0)  # Wait for reset to complete
            self.serial.reset_input_buffer()  # Clear any startup messages

            # Disable auto-report to prevent flooding
            self.serial.write(b'$Report/Interval=0\n')
            time.sleep(0.1)
            self.serial.reset_input_buffer()

            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to FluidNC: {e}")
            return False

    def disconnect(self):
        """Disconnect from FluidNC."""
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Disconnected from FluidNC")

    def start(self):
        """Start background read thread for continuous status updates."""
        if self._running:
            return

        # Reset buffer tracking
        with self._buffer_lock:
            self._buffer_used = 0
            self._pending_commands.clear()
            self._buffer_space_event.set()

        # Clear response queue
        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except queue.Empty:
                break

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        self.logger.info("FluidNC read thread started")

    def stop(self):
        """Stop background read thread."""
        if not self._running:
            return

        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        self.logger.info("FluidNC read thread stopped")

    def on_status(self, callback: Callable[[Dict], None]):
        """
        Register callback for status updates.

        Args:
            callback: Function receiving status dict with 'state' and 'position' keys
        """
        self._status_callback = callback

    def enable_auto_report(self, interval_ms: int = 100) -> bool:
        """
        Enable automatic status reporting from FluidNC.

        Args:
            interval_ms: Report interval in milliseconds

        Returns:
            True if command sent successfully
        """
        return self.send(f"$Report/Interval={interval_ms}")

    def _read_loop(self):
        """Background thread that reads from FluidNC and dispatches messages."""
        while self._running:
            try:
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.logger.debug(f"FluidNC RX: {line}")
                        self._dispatch_message(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.logger.error(f"Error in FluidNC read loop: {e}")
                time.sleep(0.1)

    def _dispatch_message(self, line: str):
        """
        Dispatch incoming message to appropriate handler.

        Args:
            line: Raw line from FluidNC
        """
        if line.startswith('<'):
            # Status report - call callback
            status = self._parse_status(line)
            if status and self._status_callback:
                try:
                    self._status_callback(status)
                except Exception as e:
                    self.logger.error(f"Error in status callback: {e}")
        elif line.startswith('ok') or line.startswith('error:'):
            # Command acknowledged - free buffer space
            self._on_command_ack()
            # Command response - put in queue for waiting commands
            self._response_queue.put(line)
        # Other messages (info, welcome, etc.) are just logged

    def _on_command_ack(self):
        """Called when FluidNC acknowledges a command (ok or error)."""
        with self._buffer_lock:
            if self._pending_commands:
                cmd_len = self._pending_commands.pop(0)
                self._buffer_used -= cmd_len
                if self._buffer_used < 0:
                    self._buffer_used = 0  # Safety
                self.logger.debug(f"Buffer freed {cmd_len} bytes, now {self._buffer_used}/{self._buffer_size}")
            # Signal that there's buffer space available
            if self._buffer_used < self._buffer_size - 50:  # Leave some margin
                self._buffer_space_event.set()

    def _send_unlocked(self, command: str) -> bool:
        """
        Send raw command to FluidNC (internal, no lock).

        Args:
            command: Command string (will add newline if not present)

        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            self.logger.warning("Cannot send: FluidNC not connected")
            return False

        if not command.endswith('\n'):
            command += '\n'

        try:
            self.serial.write(command.encode('utf-8'))
            self.logger.debug(f"FluidNC TX: {command.strip()}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to send to FluidNC: {e}")
            return False

    def send(self, command: str) -> bool:
        """
        Send raw command to FluidNC (without waiting for response).
        Thread-safe.

        Args:
            command: Command string (will add newline if not present)

        Returns:
            True if sent successfully, False otherwise
        """
        with self._serial_lock:
            return self._send_unlocked(command)

    def send_gcode(self, gcode: str, wait_ok: bool = True, timeout: float = 10.0) -> bool:
        """
        Send G-code command with proper flow control.
        Thread-safe: uses buffer management to prevent overflow.

        Args:
            gcode: G-code command
            wait_ok: Wait for 'ok' response before returning
            timeout: Timeout in seconds for waiting

        Returns:
            True if successful, False on error
        """
        # Calculate command length (including newline)
        cmd_with_newline = gcode if gcode.endswith('\n') else gcode + '\n'
        cmd_len = len(cmd_with_newline)

        # Wait for buffer space if needed (flow control)
        if self._running:
            start_time = time.time()
            while True:
                with self._buffer_lock:
                    if self._buffer_used + cmd_len <= self._buffer_size:
                        # Have space - track this command
                        self._buffer_used += cmd_len
                        self._pending_commands.append(cmd_len)
                        self.logger.debug(f"Buffer: +{cmd_len} bytes, now {self._buffer_used}/{self._buffer_size}")
                        if self._buffer_used >= self._buffer_size - 50:
                            self._buffer_space_event.clear()
                        break

                # No space - wait for acknowledgment
                if time.time() - start_time > timeout:
                    self.logger.error(f"Timeout waiting for buffer space to send: {gcode}")
                    return False

                # Wait for buffer space event with short timeout
                self._buffer_space_event.wait(timeout=0.1)

        # Send the command
        with self._serial_lock:
            if not self._send_unlocked(gcode):
                # Failed to send - remove from tracking
                if self._running:
                    with self._buffer_lock:
                        if self._pending_commands and self._pending_commands[-1] == cmd_len:
                            self._pending_commands.pop()
                            self._buffer_used -= cmd_len
                return False

        if not wait_ok:
            return True

        # Wait for 'ok' or 'error' response
        if self._running:
            # Background thread is handling reads - use queue
            try:
                response = self._response_queue.get(timeout=timeout)
                if response.startswith('ok'):
                    return True
                elif response.startswith('error:'):
                    self.logger.error(f"FluidNC error: {response}")
                    return False
            except queue.Empty:
                self.logger.error(f"Timeout waiting for response to: {gcode}")
                return False
        else:
            # No background thread - read directly (no flow control)
            with self._serial_lock:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    response = self._readline_unlocked()
                    if response:
                        self.logger.debug(f"FluidNC RX: {response}")

                        if response.startswith('ok'):
                            return True
                        elif response.startswith('error:'):
                            self.logger.error(f"FluidNC error: {response}")
                            return False

                    time.sleep(0.01)

                self.logger.error(f"Timeout waiting for response to: {gcode}")
                return False

        return False

    def _readline_unlocked(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read a line from FluidNC (internal, no lock).

        Args:
            timeout: Override default timeout

        Returns:
            Line string (stripped) or None if nothing available
        """
        if not self.serial or not self.serial.is_open:
            return None

        old_timeout = self.serial.timeout
        if timeout is not None:
            self.serial.timeout = timeout

        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                return line if line else None
        except Exception as e:
            self.logger.error(f"Error reading from FluidNC: {e}")
        finally:
            if timeout is not None:
                self.serial.timeout = old_timeout

        return None

    def readline(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read a line from FluidNC.
        Thread-safe.

        Args:
            timeout: Override default timeout

        Returns:
            Line string (stripped) or None if nothing available
        """
        with self._serial_lock:
            return self._readline_unlocked(timeout)

    def home(self) -> bool:
        """
        Execute homing sequence.

        Returns:
            True if homing successful, False otherwise
        """
        self.logger.info("Starting homing sequence")
        return self.send_gcode("$H", wait_ok=True, timeout=30.0)

    def get_limits(self) -> Dict[str, Tuple[float, float]]:
        """
        Query machine limits from FluidNC settings.
        Thread-safe: acquires serial lock for entire operation.

        Returns:
            Dictionary with axis limits: {'X': (min, max), ...}
        """
        # Default limits
        limits = {
            'X': (0.0, 300.0),
            'Y': (-150.0, 150.0),
            'Z': (0.0, 50.0)
        }

        with self._serial_lock:
            # Clear any pending data before sending $$
            if self.serial:
                self.serial.reset_input_buffer()

            # Send $$ to get settings
            if not self._send_unlocked("$$"):
                return limits

            # Read settings until 'ok' or timeout
            start_time = time.time()
            settings = {}

            while time.time() - start_time < 5.0:  # Increased timeout
                line = self._readline_unlocked(timeout=0.1)
                if line:
                    self.logger.debug(f"get_limits received: {line}")
                    # Check for end of settings
                    if line.startswith('ok'):
                        break
                    # Parse setting line: $130=300.000
                    match = re.match(r'\$(\d+)=([\d.-]+)', line)
                    if match:
                        setting_num = int(match.group(1))
                        value = float(match.group(2))
                        settings[setting_num] = value

            if not settings:
                self.logger.warning("get_limits: No settings received from FluidNC")

        # Extract limits from settings (outside lock - just local data)
        # $130, $131, $132 = max travel for X, Y, Z
        if 130 in settings:
            limits['X'] = (0.0, settings[130])
        if 131 in settings:
            limits['Y'] = (0.0, settings[131])
        if 132 in settings:
            limits['Z'] = (0.0, settings[132])

        self.logger.info(f"Machine limits: {limits}")
        return limits

    def get_status(self) -> Optional[Dict]:
        """
        Query current status and position.
        Thread-safe: acquires serial lock for entire operation.

        Returns:
            Dictionary with state and position, or None on error
        """
        with self._serial_lock:
            if not self._send_unlocked("?"):
                return None

            # Wait for status response: <Idle|MPos:10.00,20.00,5.00|...>
            start_time = time.time()
            while time.time() - start_time < 0.2:  # 200ms is plenty for status response
                line = self._readline_unlocked(timeout=0.1)
                if line and line.startswith('<'):
                    return self._parse_status(line)

            return None

    def _parse_status(self, status_line: str) -> Optional[Dict]:
        """
        Parse FluidNC status response.

        Example: <Idle|MPos:10.00,20.00,5.00|FS:0,0>

        Returns:
            Dictionary with 'state' and 'position' keys
        """
        try:
            # Remove < and >
            status_line = status_line.strip('<>')

            # Split by |
            parts = status_line.split('|')

            result = {
                'state': parts[0],
                'position': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
            }

            # Find MPos or WPos
            for part in parts:
                if part.startswith('MPos:') or part.startswith('WPos:'):
                    coords = part.split(':')[1].split(',')
                    if len(coords) >= 3:
                        result['position']['X'] = float(coords[0])
                        result['position']['Y'] = float(coords[1])
                        result['position']['Z'] = float(coords[2])

            return result

        except Exception as e:
            self.logger.error(f"Error parsing status: {e}")
            return None

    def jog(self, axis: str, position: float, feedrate: int = 1000) -> bool:
        """
        Send jog command (absolute positioning).

        Args:
            axis: Axis to jog (X, Y, or Z)
            position: Target position in mm
            feedrate: Jog speed in mm/min

        Returns:
            True if command sent successfully
        """
        command = f"$J=G90 G21 {axis}{position:.2f} F{feedrate}"
        return self.send(command)

    def cancel_jog(self) -> bool:
        """
        Cancel any active jog command.
        Thread-safe.

        Returns:
            True if cancel sent successfully
        """
        with self._serial_lock:
            # Send Jog Cancel character (0x85)
            try:
                if self.serial and self.serial.is_open:
                    self.serial.write(b'\x85')
                    self.logger.info("Jog cancelled")
                    return True
            except Exception as e:
                self.logger.error(f"Error cancelling jog: {e}")

            return False

    def soft_reset(self) -> bool:
        """
        Send soft reset (Ctrl-X).
        Thread-safe.

        Returns:
            True if reset sent successfully
        """
        with self._serial_lock:
            try:
                if self.serial and self.serial.is_open:
                    self.serial.write(b'\x18')
                    self.logger.info("Soft reset sent")
                    time.sleep(2.0)  # Wait for reset

                    # Reset buffer tracking after reset
                    with self._buffer_lock:
                        self._buffer_used = 0
                        self._pending_commands.clear()
                        self._buffer_space_event.set()

                    return True
            except Exception as e:
                self.logger.error(f"Error sending soft reset: {e}")

            return False

    def identify(self) -> bool:
        """
        Identify if this device is a FluidNC controller.
        Thread-safe.

        Returns:
            True if device is FluidNC, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            return False

        with self._serial_lock:
            try:
                # Clear input buffer
                self.serial.reset_input_buffer()

                # Send version query
                self._send_unlocked("$I")

                # Wait for response
                start_time = time.time()
                while time.time() - start_time < 2.0:
                    line = self._readline_unlocked(timeout=0.1)
                    if line:
                        self.logger.debug(f"FluidNC ID response: {line}")

                        # Look for FluidNC/Grbl version string
                        if '[VER:' in line or '[MSG:' in line or 'Grbl' in line:
                            return True

                        # Also check for status response
                        if line.startswith('<'):
                            return True

                # Try status query as alternative
                self._send_unlocked("?")
                time.sleep(0.5)

                while self.serial.in_waiting:
                    line = self._readline_unlocked(timeout=0.1)
                    if line and line.startswith('<'):
                        return True

                return False

            except Exception as e:
                self.logger.error(f"Error identifying FluidNC: {e}")
            return False

    def wait_idle(self, timeout: float = 60.0, poll_interval: float = 0.1) -> bool:
        """
        Wait until FluidNC is in Idle state (all motion complete).

        Args:
            timeout: Maximum time to wait in seconds
            poll_interval: Time between status checks in seconds

        Returns:
            True if Idle state reached, False on timeout
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            status = self.get_status()
            if status and status.get('state') == 'Idle':
                return True
            time.sleep(poll_interval)

        self.logger.warning(f"Timeout waiting for Idle state after {timeout}s")
        return False
