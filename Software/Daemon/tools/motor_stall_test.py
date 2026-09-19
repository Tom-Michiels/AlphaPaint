#!/usr/bin/env python3
"""Move one axis back and forth near the origin while reading the TMC drivers.

One motor intermittently stops turning, which on CoreXY shows up as diagonal
motion. This runs the machine in a small area far from the pen changer and asks
FluidNC for the driver status ($MS) about once a second *while it is moving* -
open-load and short flags only mean something under load. Afterwards it homes
the tested axis again and reports how far the machine had drifted.

The daemon must be stopped (it owns the serial port). Ctrl-C stops the machine
with a feed hold.

Usage: motor_stall_test.py [PORT] [--axis X|Y] [--seconds 90]
"""

import argparse
import re
import sys
import time

import serial

# Test area: well away from the pen holder at X 700-836
CENTER_X = 100.0
SWEEP_MIN = 30.0
SWEEP_MAX = 180.0
FEED = 6000          # mm/min, the rapid rate the machine uses in practice
SAFE_Z = 60.0        # pen up

FLAG_RE = re.compile(r'(open_load a:(\w) b:(\w)|temp_warn:(\w)|temp_shutdown:(\w)'
                     r'|short_gnd a:(\w) b:(\w)|driver_was_reset:(\w))')


class Machine:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.5)
        self.serial.reset_input_buffer()
        self.lines = []

    def send(self, line):
        self.serial.write(line.encode() + b'\n')

    def realtime(self, char):
        self.serial.write(char)

    def read(self, seconds=0.0):
        """Collect output for a while; returns the new lines."""
        deadline = time.time() + seconds
        new = []
        while True:
            raw = self.serial.readline()
            if raw:
                text = raw.decode(errors='replace').strip()
                if text:
                    new.append(text)
                    self.lines.append(text)
            elif time.time() >= deadline:
                break
        return new

    def status(self):
        self.realtime(b'?')
        deadline = time.time() + 1.0
        while time.time() < deadline:
            for line in self.read(0.1):
                if line.startswith('<'):
                    return line
        return None

    def wait_for_ok(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self.read(0.2):
                if line.startswith('ok'):
                    return True
                if line.startswith('error:'):
                    print(f"  ! {line}")
                    return False
        return False

    def position(self):
        status = self.status()
        if not status:
            return None
        match = re.search(r'MPos:([-\d.]+),([-\d.]+),([-\d.]+)', status)
        return tuple(float(v) for v in match.groups()) if match else None


def report_flags(lines):
    """Print only driver lines that show something other than 'all clear'."""
    interesting = []
    for line in lines:
        if 'motor:' in line or 'Axis   ' in line:
            if ('open_load a:Y' in line or 'open_load b:Y' in line or 'temp_warn:Y' in line
                    or 'temp_shutdown:Y' in line or 'short_gnd a:Y' in line
                    or 'short_gnd b:Y' in line or 'driver_was_reset:Y' in line
                    or 'no usable answer' in line):
                interesting.append(line)
    return interesting


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--axis', default='Y', choices=['X', 'Y'])
    parser.add_argument('--seconds', type=float, default=90.0)
    args = parser.parse_args()

    m = Machine(args.port)
    # Silence any auto-report left over from an earlier run first, otherwise
    # the replies below arrive buried in a stream of status lines.
    m.send('$Report/Interval=0')
    time.sleep(0.5)
    m.serial.reset_input_buffer()
    print(f"Connected to {args.port}; state: {m.status()}")

    print("Homing (needed before anything may move)...")
    m.send('$H')
    if not m.wait_for_ok(120):
        print("Homing failed - see the output above. Stopping.")
        return 1
    print(f"Homed: {m.status()}")

    # 10 status reports a second: the drift measurement below needs a dense
    # stream, '?' polling alone samples far too coarsely.
    m.send('$Report/Interval=100')
    m.wait_for_ok(5)

    print(f"Moving to the test area (X={CENTER_X}, {args.axis} sweeps "
          f"{SWEEP_MIN}-{SWEEP_MAX} mm, pen up)")
    m.send(f'G0 Z{SAFE_Z}')
    m.wait_for_ok(10)
    m.send(f'G0 X{CENTER_X} Y{SWEEP_MIN}')
    m.wait_for_ok(30)

    other = CENTER_X if args.axis == 'Y' else SWEEP_MIN
    print(f"Running for {args.seconds:.0f}s. Watch the machine; Ctrl-C stops it.")
    start = time.time()
    flags_seen = []
    sweeps = 0
    try:
        while time.time() - start < args.seconds:
            # Queue a few sweeps so the machine keeps moving while we ask $MS
            for target in (SWEEP_MAX, SWEEP_MIN):
                if args.axis == 'Y':
                    m.send(f'G1 X{other} Y{target} F{FEED}')
                else:
                    m.send(f'G1 X{target} Y{other} F{FEED}')
                sweeps += 1
            m.send('$MS')
            time.sleep(1.0)
            lines = m.read(0.3)
            for line in report_flags(lines):
                stamp = time.time() - start
                print(f"  [{stamp:5.1f}s] {line}")
                flags_seen.append(line)
            status = m.status()
            if status and ('Alarm' in status or 'Critical' in status):
                print(f"  ! Machine stopped: {status}")
                break
    except KeyboardInterrupt:
        print("\nInterrupted - feed hold")
        m.realtime(b'!')
        time.sleep(1.0)
        m.realtime(b'\x18')
        return 1

    # Let the queue run out before measuring
    m.send('G4 P0')
    m.wait_for_ok(60)
    time.sleep(0.5)
    m.serial.reset_input_buffer()
    before = m.position()
    print(f"\nDone after {sweeps} sweeps. Position now: {before}")

    print(f"Re-homing {args.axis} to measure the drift...")
    m.serial.reset_input_buffer()
    m.send(f'$H{args.axis}')
    drift = None
    done = False
    deadline = time.time() + 120
    axis_index = {'X': 0, 'Y': 1}[args.axis]
    pos_re = re.compile(r'MPos:([-\d.]+),([-\d.]+),([-\d.]+)')
    while time.time() < deadline and not done:
        for line in m.read(0.05):
            if line.startswith('ok'):
                done = True
            elif line.startswith('error:'):
                print(f"  ! {line}")
                done = True
            else:
                match = pos_re.search(line)
                if match:
                    value = float(match.groups()[axis_index])
                    drift = value if drift is None else min(drift, value)

    print("\n=== RESULT ===")
    if drift is not None:
        print(f"Lowest {args.axis} reached while homing: {drift:+.1f} mm "
              f"(around -2 mm is normal; much lower means the machine lost steps)")
    if flags_seen:
        print(f"Driver flags seen while moving: {len(flags_seen)}")
        for line in flags_seen[-10:]:
            print(f"  {line}")
    else:
        print("No driver fault flags while moving.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
