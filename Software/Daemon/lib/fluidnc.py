"""FluidNC Handler for CNC controller communication."""

import serial
import threading
import logging
import time
import re
from typing import Optional, Dict, Tuple


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
        self.response_queue = []
        self.response_lock = threading.Lock()

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
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to FluidNC: {e}")
            return False

    def disconnect(self):
        """Disconnect from FluidNC."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Disconnected from FluidNC")

    def send(self, command: str) -> bool:
        """
        Send raw command to FluidNC (without waiting for response).

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

    def send_gcode(self, gcode: str, wait_ok: bool = True, timeout: float = 10.0) -> bool:
        """
        Send G-code command and optionally wait for 'ok' response.

        Args:
            gcode: G-code command
            wait_ok: Wait for 'ok' response
            timeout: Timeout in seconds for waiting

        Returns:
            True if successful, False on error
        """
        if not self.send(gcode):
            return False

        if not wait_ok:
            return True

        # Wait for 'ok' or 'error' response
        start_time = time.time()
        while time.time() - start_time < timeout:
            response = self.readline()
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

    def readline(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read a line from FluidNC.

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

        Returns:
            Dictionary with axis limits: {'X': (min, max), ...}
        """
        # Default limits
        limits = {
            'X': (0.0, 300.0),
            'Y': (-150.0, 150.0),
            'Z': (0.0, 50.0)
        }

        # Send $$ to get settings
        if not self.send("$$"):
            return limits

        # Read settings for ~2 seconds
        start_time = time.time()
        settings = {}

        while time.time() - start_time < 2.0:
            line = self.readline(timeout=0.1)
            if line:
                # Parse setting line: $130=300.000
                match = re.match(r'\$(\d+)=([\d.-]+)', line)
                if match:
                    setting_num = int(match.group(1))
                    value = float(match.group(2))
                    settings[setting_num] = value

        # Extract limits from settings
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

        Returns:
            Dictionary with state and position, or None on error
        """
        if not self.send("?"):
            return None

        # Wait for status response: <Idle|MPos:10.00,20.00,5.00|...>
        start_time = time.time()
        while time.time() - start_time < 1.0:
            line = self.readline(timeout=0.1)
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

        Returns:
            True if cancel sent successfully
        """
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

        Returns:
            True if reset sent successfully
        """
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(b'\x18')
                self.logger.info("Soft reset sent")
                time.sleep(2.0)  # Wait for reset
                return True
        except Exception as e:
            self.logger.error(f"Error sending soft reset: {e}")

        return False

    def identify(self) -> bool:
        """
        Identify if this device is a FluidNC controller.

        Returns:
            True if device is FluidNC, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            return False

        try:
            # Clear input buffer
            self.serial.reset_input_buffer()

            # Send version query
            self.send("$I")

            # Wait for response
            start_time = time.time()
            while time.time() - start_time < 2.0:
                line = self.readline(timeout=0.1)
                if line:
                    self.logger.debug(f"FluidNC ID response: {line}")

                    # Look for FluidNC/Grbl version string
                    if '[VER:' in line or '[MSG:' in line or 'Grbl' in line:
                        return True

                    # Also check for status response
                    if line.startswith('<'):
                        return True

            # Try status query as alternative
            self.send("?")
            time.sleep(0.5)

            while self.serial.in_waiting:
                line = self.readline(timeout=0.1)
                if line and line.startswith('<'):
                    return True

            return False

        except Exception as e:
            self.logger.error(f"Error identifying FluidNC: {e}")
            return False
