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
            self.logger.info(f"Connected to Console on {self.port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to Console: {e}")
            return False

    def disconnect(self):
        """Disconnect from Console."""
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Disconnected from Console")

    def send(self, message: str):
        """
        Send message to Console.

        Args:
            message: Message string (will add newline if not present)
        """
        if not self.serial or not self.serial.is_open:
            self.logger.warning("Cannot send: Console not connected")
            return

        if not message.endswith('\n'):
            message += '\n'

        try:
            self.serial.write(message.encode('utf-8'))
            self.logger.debug(f"Console TX: {message.strip()}")
        except Exception as e:
            self.logger.error(f"Failed to send to Console: {e}")

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
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.logger.debug(f"Console RX: {line}")
                        self._parse_message(line)
                else:
                    time.sleep(0.01)  # Small delay to avoid busy waiting
            except Exception as e:
                self.logger.error(f"Error reading from Console: {e}")
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
        parts = message.split(':')
        if len(parts) < 2:
            self.logger.warning(f"Invalid message format: {message}")
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
        else:
            self.logger.debug(f"No callback registered for message type: {msg_type}")

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
