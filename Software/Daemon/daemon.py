#!/usr/bin/env python3
"""
AlphaPaint Daemon - Main Entry Point

This daemon runs on the Raspberry Pi and coordinates communication between
the AlphaPaint Console (ESP32) and the FluidNC CNC controller.
"""

import sys
import time
import logging
import signal
import glob
import yaml
from pathlib import Path
from typing import Optional, Tuple

from lib import ConsoleHandler, FluidNCHandler, StateMachine


class AlphaPaintDaemon:
    """Main daemon class."""

    def __init__(self, config_path: str = "config.yaml"):
        """
        Initialize daemon.

        Args:
            config_path: Path to configuration file
        """
        self.config = self._load_config(config_path)
        self.console: Optional[ConsoleHandler] = None
        self.fluidnc: Optional[FluidNCHandler] = None
        self.state_machine: Optional[StateMachine] = None
        self.running = False
        self.logger = logging.getLogger(__name__)

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _load_config(self, config_path: str) -> dict:
        """
        Load configuration from YAML file.

        Args:
            config_path: Path to config file

        Returns:
            Configuration dictionary
        """
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                print(f"Configuration loaded from {config_path}")
                return config
        except FileNotFoundError:
            print(f"Warning: Config file {config_path} not found, using defaults")
            return self._default_config()
        except Exception as e:
            print(f"Error loading config: {e}")
            return self._default_config()

    def _default_config(self) -> dict:
        """
        Get default configuration.

        Returns:
            Default configuration dictionary
        """
        return {
            'serial': {
                'baud_rate': 115200,
                'timeout': 1.0,
                'reconnect_delay': 5.0
            },
            'machine': {
                'pen_z_default': 0.5,
                'jog_feedrate': 1000,
                'draw_feedrate': 500,
                'pen_lift_feedrate': 100
            },
            'drawing': {
                'ellipse_segments': 36,
                'min_position_delta': 0.01
            },
            'logging': {
                'level': 'INFO',
                'file': '/var/log/alphapaint-daemon.log'
            }
        }

    def _setup_logging(self):
        """Setup logging configuration."""
        log_level = getattr(logging, self.config['logging']['level'].upper())
        log_file = self.config['logging'].get('file')

        # Configure root logger
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        # Add file handler if specified
        if log_file:
            try:
                file_handler = logging.FileHandler(log_file)
                file_handler.setFormatter(
                    logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s')
                )
                logging.getLogger().addHandler(file_handler)
                self.logger.info(f"Logging to file: {log_file}")
            except Exception as e:
                self.logger.warning(f"Could not setup file logging: {e}")

    def _signal_handler(self, signum, frame):
        """
        Handle shutdown signals.

        Args:
            signum: Signal number
            frame: Stack frame
        """
        self.logger.info(f"Received signal {signum}, shutting down...")
        self.running = False

    def _scan_serial_ports(self) -> list:
        """
        Scan for available serial ports.

        Returns:
            List of serial port paths
        """
        ports = []

        # Common serial port patterns
        patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/ttyS*',
            '/dev/tty.usb*'  # macOS
        ]

        for pattern in patterns:
            ports.extend(glob.glob(pattern))

        self.logger.info(f"Found {len(ports)} serial port(s): {ports}")
        return sorted(ports)

    def _identify_device(self, port: str) -> Optional[str]:
        """
        Identify device on serial port.

        Args:
            port: Serial port path

        Returns:
            'CONSOLE' or 'FLUIDNC' or None if unknown
        """
        self.logger.info(f"Identifying device on {port}...")

        # Try Console
        try:
            console = ConsoleHandler(
                port,
                self.config['serial']['baud_rate'],
                self.config['serial']['timeout']
            )
            if console.connect():
                if console.identify():
                    console.disconnect()
                    self.logger.info(f"  → Console detected on {port}")
                    return 'CONSOLE'
                console.disconnect()
        except Exception as e:
            self.logger.debug(f"Console identification failed on {port}: {e}")

        # Try FluidNC
        try:
            fluidnc = FluidNCHandler(
                port,
                self.config['serial']['baud_rate'],
                self.config['serial']['timeout']
            )
            if fluidnc.connect():
                if fluidnc.identify():
                    fluidnc.disconnect()
                    self.logger.info(f"  → FluidNC detected on {port}")
                    return 'FLUIDNC'
                fluidnc.disconnect()
        except Exception as e:
            self.logger.debug(f"FluidNC identification failed on {port}: {e}")

        self.logger.info(f"  → Unknown device on {port}")
        return None

    def _scan_and_connect(self) -> Tuple[Optional[str], Optional[str]]:
        """
        Scan serial ports and identify Console and FluidNC.

        Returns:
            Tuple of (console_port, fluidnc_port) or (None, None) if not found
        """
        self.logger.info("Scanning for devices...")

        ports = self._scan_serial_ports()
        if not ports:
            self.logger.error("No serial ports found")
            return None, None

        console_port = None
        fluidnc_port = None

        for port in ports:
            device_type = self._identify_device(port)

            if device_type == 'CONSOLE':
                console_port = port
            elif device_type == 'FLUIDNC':
                fluidnc_port = port

            # Stop if we found both
            if console_port and fluidnc_port:
                break

        if console_port and fluidnc_port:
            self.logger.info(f"Devices found - Console: {console_port}, FluidNC: {fluidnc_port}")
            return console_port, fluidnc_port
        else:
            missing = []
            if not console_port:
                missing.append("Console")
            if not fluidnc_port:
                missing.append("FluidNC")
            self.logger.error(f"Could not find: {', '.join(missing)}")
            return None, None

    def run(self):
        """Main daemon loop."""
        self._setup_logging()
        self.logger.info("="*60)
        self.logger.info("AlphaPaint Daemon Starting")
        self.logger.info("="*60)

        self.running = True

        while self.running:
            try:
                # Scan and connect to devices
                console_port, fluidnc_port = self._scan_and_connect()

                if not console_port or not fluidnc_port:
                    self.logger.warning(
                        f"Retrying in {self.config['serial']['reconnect_delay']} seconds..."
                    )
                    time.sleep(self.config['serial']['reconnect_delay'])
                    continue

                # Connect to Console
                self.console = ConsoleHandler(
                    console_port,
                    self.config['serial']['baud_rate'],
                    self.config['serial']['timeout']
                )
                if not self.console.connect():
                    self.logger.error("Failed to connect to Console")
                    time.sleep(self.config['serial']['reconnect_delay'])
                    continue

                # Connect to FluidNC
                self.fluidnc = FluidNCHandler(
                    fluidnc_port,
                    self.config['serial']['baud_rate'],
                    self.config['serial']['timeout']
                )
                if not self.fluidnc.connect():
                    self.logger.error("Failed to connect to FluidNC")
                    self.console.disconnect()
                    time.sleep(self.config['serial']['reconnect_delay'])
                    continue

                # Create state machine
                self.state_machine = StateMachine(
                    self.console,
                    self.fluidnc,
                    self.config
                )

                # Start Console read thread
                self.console.start()

                # Start state machine
                self.state_machine.start()

                self.logger.info("Daemon running - devices connected")

                # Main loop - just keep running
                while self.running:
                    time.sleep(0.1)

                    # TODO: Add connection monitoring
                    # Check if devices are still connected, reconnect if needed

            except KeyboardInterrupt:
                self.logger.info("Keyboard interrupt received")
                break

            except Exception as e:
                self.logger.error(f"Error in main loop: {e}", exc_info=True)
                time.sleep(self.config['serial']['reconnect_delay'])

            finally:
                # Cleanup
                if self.console:
                    self.console.disconnect()
                    self.console = None
                if self.fluidnc:
                    self.fluidnc.disconnect()
                    self.fluidnc = None
                self.state_machine = None

        self.logger.info("AlphaPaint Daemon stopped")


def main():
    """Main entry point."""
    # Get config path from command line or use default
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"

    # Create and run daemon
    daemon = AlphaPaintDaemon(config_path)
    daemon.run()


if __name__ == "__main__":
    main()
