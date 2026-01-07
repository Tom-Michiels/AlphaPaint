#!/usr/bin/env python3
"""
AlphaPaint Console Test Stub

Simulates the Raspberry Pi's interaction with the console firmware.
Provides an interactive menu to test all console functionality:
- Homing sequence
- Mode switching (PASSIVE/ACTIVE)
- Position updates
- LED control
- Limit configuration
- Button event monitoring

Usage:
    python3 console_test_stub.py [--port /dev/ttyUSB0] [--baud 115200]
"""

import serial
import threading
import time
import sys
import argparse
from typing import Optional

class ConsoleTestStub:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None

        # State tracking
        self.console_state = "UNKNOWN"
        self.console_mode = "UNKNOWN"
        self.positions = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self.limits = {
            "X": {"min": -999.99, "max": 999.99},
            "Y": {"min": -999.99, "max": 999.99},
            "Z": {"min": -999.99, "max": 999.99}
        }

    def connect(self):
        """Connect to the console via UART"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")

            # Start RX thread
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()

            # Wait for identification string
            time.sleep(1)

            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from the console"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✓ Disconnected")

    def _rx_loop(self):
        """Background thread to receive and process messages from console"""
        buffer = ""

        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._process_rx_message(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"\n✗ RX error: {e}")
                break

    def _process_rx_message(self, message: str):
        """Process incoming message from console"""
        print(f"\n← {message}")

        parts = message.split(':')
        if len(parts) < 2:
            return

        cmd = parts[0]

        # Parse different message types
        if cmd == "CONSOLE":
            # CONSOLE:ALPHAPAINT:V1.1
            print(f"  Console identified: {':'.join(parts[1:])}")

        elif cmd == "STATUS":
            # STATUS:NOT_HOMED or STATUS:HOMED
            self.console_state = parts[1]
            print(f"  Console state: {self.console_state}")

        elif cmd == "BTN":
            # BTN:X:SHORT or BTN:A:LONG
            button = parts[1]
            press_type = parts[2]
            print(f"  Button {button} pressed: {press_type}")

        elif cmd == "AXIS":
            # AXIS:X:SELECT or AXIS:Z:PRECISION:ON
            axis = parts[1]
            event = ':'.join(parts[2:])
            print(f"  Axis {axis} event: {event}")

        elif cmd == "POS":
            # POS:X:123.45
            axis = parts[1]
            value = float(parts[2])
            self.positions[axis] = value
            print(f"  Position {axis}: {value:.2f}")

        elif cmd == "LIMIT":
            # LIMIT:X:-100.00:200.00
            if len(parts) >= 4:
                axis = parts[1]
                min_val = float(parts[2])
                max_val = float(parts[3])
                self.limits[axis] = {"min": min_val, "max": max_val}
                print(f"  Limits {axis}: {min_val:.2f} to {max_val:.2f}")

        elif cmd == "ERROR":
            # ERROR:message
            error = ':'.join(parts[1:])
            print(f"  ⚠ ERROR: {error}")

    def send_command(self, command: str):
        """Send command to console"""
        if not self.serial or not self.serial.is_open:
            print("✗ Not connected")
            return False

        try:
            message = command + '\n'
            self.serial.write(message.encode('utf-8'))
            print(f"→ {command}")
            time.sleep(0.05)  # Small delay for command processing
            return True
        except Exception as e:
            print(f"✗ Send error: {e}")
            return False

    def perform_homing(self):
        """Simulate homing sequence"""
        print("\n=== Simulating Homing Sequence ===")
        print("Sending initial positions (this will trigger HOMED state)...")

        # Send initial positions - this transitions console from NOT_HOMED to HOMED
        self.send_command("POS:X:0.00")
        time.sleep(0.1)
        self.send_command("POS:Y:0.00")
        time.sleep(0.1)
        self.send_command("POS:Z:0.00")
        time.sleep(0.2)

        print("✓ Homing complete - console should now be in HOMED state")

    def set_mode(self, mode: str):
        """Set operating mode (PASSIVE or ACTIVE)"""
        if mode.upper() not in ["PASSIVE", "ACTIVE"]:
            print("✗ Invalid mode. Use PASSIVE or ACTIVE")
            return

        self.send_command(f"MODE:{mode.upper()}")
        self.console_mode = mode.upper()

    def set_position(self, axis: str, value: float):
        """Set axis position"""
        if axis.upper() not in ["X", "Y", "Z"]:
            print("✗ Invalid axis. Use X, Y, or Z")
            return

        self.send_command(f"POS:{axis.upper()}:{value:.2f}")
        # Update local state - console doesn't echo POS commands back
        self.positions[axis.upper()] = value

    def set_led(self, led: str, state: str):
        """Set LED state"""
        if led.upper() not in list("ABCDEFG"):
            print("✗ Invalid LED. Use A-G")
            return

        if state.upper() not in ["OFF", "ON", "BLINK", "FAST_BLINK"]:
            print("✗ Invalid state. Use OFF, ON, BLINK, or FAST_BLINK")
            return

        self.send_command(f"LED:{led.upper()}:{state.upper()}")

    def set_limits(self, axis: str, min_val: float, max_val: float):
        """Set axis limits"""
        if axis.upper() not in ["X", "Y", "Z"]:
            print("✗ Invalid axis. Use X, Y, or Z")
            return

        if min_val >= max_val:
            print("✗ Min must be less than max")
            return

        self.send_command(f"LIMIT:{axis.upper()}:{min_val:.2f}:{max_val:.2f}")

    def query_limits(self, axis: str):
        """Query axis limits"""
        if axis.upper() not in ["X", "Y", "Z"]:
            print("✗ Invalid axis. Use X, Y, or Z")
            return

        self.send_command(f"LIMIT:{axis.upper()}?")

    def interactive_menu(self):
        """Display interactive menu for testing"""
        print("\n" + "="*60)
        print("   AlphaPaint Console Test Stub - Interactive Menu")
        print("="*60)

        while True:
            print("\n--- Main Menu ---")
            print("1. Perform Homing Sequence")
            print("2. Set Mode (PASSIVE/ACTIVE)")
            print("3. Set Position")
            print("4. Control LED")
            print("5. Configure Limits")
            print("6. Query Limits")
            print("7. Run Automated Test Sequence")
            print("8. Display Current State")
            print("9. Send Custom Command")
            print("0. Exit")

            choice = input("\nSelect option: ").strip()

            if choice == "1":
                self.perform_homing()

            elif choice == "2":
                mode = input("Enter mode (PASSIVE/ACTIVE): ").strip()
                self.set_mode(mode)

            elif choice == "3":
                axis = input("Enter axis (X/Y/Z): ").strip()
                try:
                    value = float(input("Enter position: ").strip())
                    self.set_position(axis, value)
                except ValueError:
                    print("✗ Invalid number")

            elif choice == "4":
                led = input("Enter LED (A-G): ").strip()
                state = input("Enter state (OFF/ON/BLINK/FAST_BLINK): ").strip()
                self.set_led(led, state)

            elif choice == "5":
                axis = input("Enter axis (X/Y/Z): ").strip()
                try:
                    min_val = float(input("Enter min limit: ").strip())
                    max_val = float(input("Enter max limit: ").strip())
                    self.set_limits(axis, min_val, max_val)
                except ValueError:
                    print("✗ Invalid number")

            elif choice == "6":
                axis = input("Enter axis (X/Y/Z): ").strip()
                self.query_limits(axis)

            elif choice == "7":
                self.run_automated_test()

            elif choice == "8":
                self.display_state()

            elif choice == "9":
                cmd = input("Enter command: ").strip()
                self.send_command(cmd)

            elif choice == "0":
                print("\nExiting...")
                break

            else:
                print("✗ Invalid option")

    def display_state(self):
        """Display current state"""
        print("\n" + "="*60)
        print("   Current State")
        print("="*60)
        print(f"Console State: {self.console_state}")
        print(f"Operating Mode: {self.console_mode}")
        print(f"\nPositions:")
        print(f"  X: {self.positions['X']:8.2f}")
        print(f"  Y: {self.positions['Y']:8.2f}")
        print(f"  Z: {self.positions['Z']:8.2f}")
        print(f"\nLimits:")
        for axis in ['X', 'Y', 'Z']:
            print(f"  {axis}: {self.limits[axis]['min']:8.2f} to {self.limits[axis]['max']:8.2f}")
        print("="*60)

    def run_automated_test(self):
        """Run automated test sequence"""
        print("\n" + "="*60)
        print("   Automated Test Sequence")
        print("="*60)

        print("\n[1] Testing Homing...")
        self.perform_homing()
        time.sleep(1)

        print("\n[2] Testing Mode Switch to ACTIVE...")
        self.set_mode("ACTIVE")
        time.sleep(0.5)

        print("\n[3] Testing Position Updates...")
        positions = [
            ("X", 10.50),
            ("Y", 20.75),
            ("Z", -5.25),
            ("X", 15.00),
        ]
        for axis, value in positions:
            self.set_position(axis, value)
            time.sleep(0.3)

        print("\n[4] Testing LED Control...")
        led_tests = [
            ("A", "ON"),
            ("B", "BLINK"),
            ("C", "ON"),
            ("D", "FAST_BLINK"),
        ]
        for led, state in led_tests:
            self.set_led(led, state)
            time.sleep(0.3)

        print("\n[5] Testing Limit Configuration...")
        self.set_limits("X", -100.0, 200.0)
        time.sleep(0.3)
        self.set_limits("Y", -150.0, 150.0)
        time.sleep(0.3)
        self.set_limits("Z", -50.0, 100.0)
        time.sleep(0.3)

        print("\n[6] Querying Limits...")
        for axis in ['X', 'Y', 'Z']:
            self.query_limits(axis)
            time.sleep(0.3)

        print("\n[7] Testing Mode Switch to PASSIVE...")
        self.set_mode("PASSIVE")
        time.sleep(0.5)

        print("\n[8] Testing Position Updates in PASSIVE mode...")
        self.set_position("X", 50.00)
        time.sleep(0.3)
        self.set_position("Y", 75.00)
        time.sleep(0.3)

        print("\n[9] Turning off LEDs...")
        for led in "ABCD":
            self.set_led(led, "OFF")
            time.sleep(0.2)

        print("\n✓ Automated test sequence complete!")
        print("="*60)


def main():
    parser = argparse.ArgumentParser(
        description="AlphaPaint Console Test Stub - Simulates Raspberry Pi interaction"
    )
    parser.add_argument(
        "--port", "-p",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )

    args = parser.parse_args()

    print("="*60)
    print("   AlphaPaint Console Test Stub")
    print("="*60)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print("="*60)

    stub = ConsoleTestStub(args.port, args.baud)

    if not stub.connect():
        print("\n✗ Failed to connect. Check:")
        print("  - Console is powered on")
        print("  - USB cable is connected")
        print(f"  - Port {args.port} is correct (try: ls /dev/tty*)")
        print("  - You have permissions (try: sudo usermod -a -G dialout $USER)")
        return 1

    try:
        stub.interactive_menu()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        stub.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
