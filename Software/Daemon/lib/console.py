"""Console Handler for AlphaPaint Console communication."""

import serial
import threading
import logging
import time
from typing import Callable, Dict, Optional


class ConsoleHandler:
    """Handles communication with the AlphaPaint Console (ESP32)."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize Console handler.

        Args:
            port: Serial port path (e.g., /dev/ttyUSB0)
            baudrate: Serial baudrate (default: 115200)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.callbacks: Dict[str, Callable] = {}
        self.running = False
        self.read_thread: Optional[threading.Thread] = None
        self.logger = logging.getLogger(__name__)

        # Connection monitoring
        self._connected = False
        self._disconnect_callback: Optional[Callable[[], None]] = None
        self._consecutive_errors = 0
        self._max_consecutive_errors = 5  # Trigger disconnect after this many errors

    def connect(self) -> bool:
        """
        Connect to Console serial port.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout
            )
            self._connected = True
            self._consecutive_errors = 0
            self.logger.info(f"Connected to Console on {self.port}")
            return True
        except Exception as e:
            self._connected = False
            self.logger.error(f"Failed to connect to Console: {e}")
            return False

    def disconnect(self):
        """Disconnect from Console."""
        self._connected = False
        self.stop()
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass  # Ignore errors during close
            self.logger.info("Disconnected from Console")

    def is_connected(self) -> bool:
        """Check if console is connected and healthy."""
        return self._connected and self.serial is not None and self.serial.is_open

    def on_disconnect(self, callback: Callable[[], None]):
        """
        Register callback for when connection is lost.

        Args:
            callback: Function to call when connection is lost
        """
        self._disconnect_callback = callback

    def _handle_connection_lost(self):
        """Handle loss of connection to console."""
        if not self._connected:
            return  # Already handled

        self._connected = False
        self.logger.error("Connection to Console lost!")

        # Call disconnect callback if registered
        if self._disconnect_callback:
            try:
                self._disconnect_callback()
            except Exception as e:
                self.logger.error(f"Error in disconnect callback: {e}")

    def send(self, message: str) -> bool:
        """
        Send message to Console.

        Args:
            message: Message string (will add newline if not present)

        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            self.logger.warning("Cannot send: Console not connected")
            return False

        if not message.endswith('\n'):
            message += '\n'

        try:
            self.serial.write(message.encode('utf-8'))
            self.logger.debug(f"Console TX: {message.strip()}")
            self._consecutive_errors = 0  # Reset error count on success
            return True
        except serial.SerialException as e:
            self.logger.error(f"Serial error sending to Console: {e}")
            self._handle_connection_lost()
            return False
        except OSError as e:
            self.logger.error(f"OS error sending to Console: {e}")
            self._handle_connection_lost()
            return False
        except Exception as e:
            self.logger.error(f"Failed to send to Console: {e}")
            self._consecutive_errors += 1
            if self._consecutive_errors >= self._max_consecutive_errors:
                self._handle_connection_lost()
            return False

    def set_led(self, led: str, state: str):
        """
        Set LED state on Console.

        Args:
            led: LED identifier (A-G)
            state: LED state (ON, OFF, BLINK, FAST_BLINK)
        """
        self.send(f"LED:{led}:{state}")

    def set_mode(self, mode: str):
        """
        Set Console mode.

        Args:
            mode: ACTIVE or PASSIVE
        """
        self.send(f"MODE:{mode}")

    def set_position(self, axis: str, value: float):
        """
        Set position display (used in Passive mode).

        Args:
            axis: X, Y, or Z
            value: Position value in mm
        """
        self.send(f"POS:{axis}:{value:.2f}")

    def set_limit(self, axis: str, min_val: float, max_val: float):
        """
        Set axis limits on Console.

        Args:
            axis: X, Y, or Z
            min_val: Minimum value
            max_val: Maximum value
        """
        self.send(f"LIMIT:{axis}:{min_val:.2f}:{max_val:.2f}")

    def reset(self):
        """
        Reset Console to initial state.

        This puts the console in a known startup state by:
        - Setting mode to PASSIVE
        - Turning off all LEDs
        - Resetting limits to large defaults (will be overwritten after homing)
        """
        # Set passive mode
        self.set_mode('PASSIVE')

        # Turn off all LEDs
        for led in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
            self.set_led(led, 'OFF')

        # Reset limits to large defaults (matching console firmware defaults)
        # These will be overwritten with real values after homing
        for axis in ['X', 'Y', 'Z']:
            self.set_limit(axis, -9999.99, 9999.99)

        self.logger.info("Console reset: mode=PASSIVE, LEDs off, limits reset")

    def on_message(self, message_type: str, callback: Callable):
        """
        Register callback for incoming message type.

        Args:
            message_type: Message type (e.g., 'BTN', 'POS', 'STATUS')
            callback: Callback function(args...)
        """
        self.callbacks[message_type] = callback

    def start(self):
        """Start background thread for reading Console messages."""
        if self.running:
            return

        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        self.logger.info("Console read thread started")

    def stop(self):
        """Stop background read thread."""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
            self.read_thread = None

    def _read_loop(self):
        """Background thread for reading Console messages."""
        while self.running:
            try:
                if not self.serial or not self.serial.is_open:
                    self.logger.warning("Serial port closed, exiting read loop")
                    self._handle_connection_lost()
                    break

                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.logger.debug(f"Console RX: {line}")
                        self._parse_message(line)
                    self._consecutive_errors = 0  # Reset on successful read
                else:
                    time.sleep(0.01)  # Small delay to avoid busy waiting

            except serial.SerialException as e:
                self.logger.error(f"Serial error reading from Console: {e}")
                self._handle_connection_lost()
                break

            except OSError as e:
                # Common when USB device is unplugged
                self.logger.error(f"OS error reading from Console: {e}")
                self._handle_connection_lost()
                break

            except Exception as e:
                self.logger.error(f"Error reading from Console: {e}")
                self._consecutive_errors += 1
                if self._consecutive_errors >= self._max_consecutive_errors:
                    self._handle_connection_lost()
                    break
                time.sleep(0.1)

    def _parse_message(self, message: str):
        """
        Parse incoming message and call appropriate callback.

        Message format: TYPE:ARG1:ARG2:...

        Examples:
            BTN:A:SHORT
            BTN:B:LONG
            POS:X:123.45
            STATUS:HOMED
            AXIS:X:SELECTED
            PRECISION:X:ON
        """
        # Skip ESP-IDF log messages (contain ANSI codes or start with log level indicators)
        # These look like: [0;32mI (12345) TAG: message
        if message.startswith('[0;') or message.startswith('\x1b['):
            return  # Skip ANSI-colored ESP-IDF log output

        parts = message.split(':')
        if len(parts) < 2:
            # Not a protocol message - silently ignore
            return

        msg_type = parts[0]
        args = parts[1:]

        # Call registered callback if exists
        if msg_type in self.callbacks:
            try:
                self.callbacks[msg_type](*args)
            except Exception as e:
                self.logger.error(f"Error in callback for {msg_type}: {e}")
                self.logger.error(f"Message was: {message}")

    def identify(self) -> bool:
        """
        Identify if this device is the AlphaPaint Console.

        Returns:
            True if device is Console, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            return False

        try:
            # Clear input buffer
            self.serial.reset_input_buffer()

            # Send ID query (or any command to trigger a response)
            self.send("ID?")

            # Wait for response
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    self.logger.debug(f"Console ID response: {line}")

                    # Check for Console identification string
                    if "CONSOLE:ALPHAPAINT" in line:
                        return True

                    # Also accept STATUS message as confirmation
                    if line.startswith("STATUS:"):
                        return True

                    # Detect Console by its debug output pattern (ESP-IDF logging)
                    # e.g. "I (12345) CONSOLE: RX: ..." or "CONSOLE: TX: ..."
                    if "CONSOLE:" in line and ("RX:" in line or "TX:" in line):
                        return True

                time.sleep(0.1)

            return False

        except Exception as e:
            self.logger.error(f"Error identifying Console: {e}")
            return False
