#!/usr/bin/env python3
"""Quick FluidNC probe: auto-detect the controller's serial port and dump
identification ($I), settings ($$), and a few real-time status reports (?).

Run with the daemon stopped so the port isn't busy:
    sudo systemctl stop alphapaint-daemon
    python3 test_fluidnc.py
"""
import glob
import sys
import time

import serial


BAUD = 115200
TIMEOUT = 1.0


def open_port(port):
    s = serial.Serial(port, BAUD, timeout=TIMEOUT)
    time.sleep(2.0)  # ESP32/Arduino reset on DTR
    s.reset_input_buffer()
    return s


def drain(s, duration=0.5):
    """Read everything available for `duration` seconds, return list of lines."""
    lines = []
    end = time.time() + duration
    while time.time() < end:
        line = s.readline().decode(errors='replace').strip()
        if line:
            lines.append(line)
            end = time.time() + duration  # extend while data flows
    return lines


def is_fluidnc(port):
    try:
        with open_port(port) as s:
            s.write(b"$I\n")
            for line in drain(s, 1.5):
                if '[VER:' in line or 'Grbl' in line or line.startswith('<'):
                    return True
    except Exception as e:
        print(f"  {port}: error {e}")
    return False


def find_fluidnc():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    print(f"Scanning {len(ports)} port(s): {ports}")
    for p in ports:
        print(f"  probing {p}...")
        if is_fluidnc(p):
            print(f"  -> FluidNC on {p}")
            return p
    return None


def query(s, cmd, duration=1.0, label=None):
    print(f"\n=== {label or cmd} ===")
    s.reset_input_buffer()
    s.write(f"{cmd}\n".encode())
    for line in drain(s, duration):
        print(line)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_fluidnc()
    if not port:
        print("FluidNC not found")
        sys.exit(1)

    with open_port(port) as s:
        # Drain any boot banner
        banner = drain(s, 0.5)
        if banner:
            print("=== boot banner ===")
            for l in banner:
                print(l)

        query(s, "$I", 1.5, "build info ($I)")
        query(s, "?", 0.5, "status report (?)")
        query(s, "$G", 0.5, "parser state ($G)")
        query(s, "$#", 0.5, "ngc parameters ($#)")
        query(s, "$$", 2.0, "settings ($$)")

        print("\n=== streaming status for 3s ===")
        end = time.time() + 3.0
        while time.time() < end:
            s.write(b"?")
            time.sleep(0.2)
            for line in drain(s, 0.05):
                print(line)


if __name__ == '__main__':
    main()
