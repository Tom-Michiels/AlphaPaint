#!/usr/bin/env python3
"""Drive one CoreXY motor at a time and catch drift on every single stroke.

On CoreXY, motor A moves with X+Y and motor B with X-Y, so a 45 degree
diagonal turns exactly one motor while the other stands still:

  --motor a : (0,0) <-> (150,150)    only the X+Y motor turns
  --motor b : (0,150) <-> (150,0)    only the X-Y motor turns

Both strokes end on a corner where an axis sits at machine zero. The limit
switch is one pull-off distance (2 mm) beyond that, so a healthy machine never
touches it. The moment the machine loses steps towards the switch, the status
report shows "Pn:" - so every stroke is its own drift test, instead of having
to wait for a whole drawing to go wrong.

The daemon must be stopped (it owns the serial port). Ctrl-C stops the machine
with a feed hold.

Usage: diagonal_drift_test.py [PORT] [--motor a|b] [--strokes 40] [--feed 6000]
"""

import argparse
import re
import sys
import time

import serial

SAFE_Z = 60.0        # pen up
SPAN = 150.0         # length of the diagonal in each axis

STATUS_RE = re.compile(r'<(?P<state>[A-Za-z]+)[^|]*\|MPos:(?P<x>[-\d.]+),(?P<y>[-\d.]+),(?P<z>[-\d.]+)'
                       r'(?P<rest>[^>]*)>')
PIN_RE = re.compile(r'\|Pn:([^|>]+)')


class Machine:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(0.5)
        self.send('$Report/Interval=0')
        time.sleep(0.5)
        self.serial.reset_input_buffer()
        self.state = None
        self.pos = (0.0, 0.0, 0.0)
        self.pins = ''

    def send(self, line):
        self.serial.write(line.encode() + b'\n')

    def realtime(self, char):
        self.serial.write(char)

    def poll(self, seconds=0.0):
        """Read serial output, tracking status; returns the other lines."""
        deadline = time.time() + seconds
        other = []
        while True:
            raw = self.serial.readline()
            if raw:
                line = raw.decode(errors='replace').strip()
                if not line:
                    continue
                match = STATUS_RE.search(line)
                if match:
                    self.state = match.group('state')
                    self.pos = (float(match.group('x')), float(match.group('y')),
                                float(match.group('z')))
                    pins = PIN_RE.search(match.group('rest'))
                    self.pins = pins.group(1) if pins else ''
                else:
                    other.append(line)
            elif time.time() >= deadline:
                return other

    def status(self, timeout=2.0):
        """Ask for a status report and wait for it (auto-report may be off)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.realtime(b'?')
            self.poll(0.3)
            if self.state:
                return self.state
        return None

    def wait_ok(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self.poll(0.2):
                if line.startswith('ok'):
                    return True
                if line.startswith('error:'):
                    print(f"  ! {line}")
                    return False
        return False

    def wait_idle(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.poll(0.2)
            if self.state == 'Idle':
                return True
            if self.state in ('Alarm', 'Critical'):
                return False
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--motor', default='a', choices=['a', 'b'])
    parser.add_argument('--strokes', type=int, default=40)
    parser.add_argument('--feed', type=int, default=6000)
    args = parser.parse_args()

    # Motor a moves along X+Y, motor b along X-Y
    if args.motor == 'a':
        corners = [(0.0, 0.0), (SPAN, SPAN)]
    else:
        corners = [(0.0, SPAN), (SPAN, 0.0)]

    m = Machine(args.port)
    m.status()
    print(f"Connected to {args.port}; state: {m.state} at {m.pos}"
          + (f", switch {m.pins} pressed" if m.pins else ""))

    print("Homing...")
    m.send('$H')
    if not m.wait_ok(180):
        m.status()
        print(f"Homing failed (state {m.state}, switches '{m.pins}') - stopping.")
        return 1
    m.send('$Report/Interval=100')
    m.wait_ok(5)
    m.poll(0.5)
    print(f"Homed: {m.pos}")

    print(f"Pen up and moving to the first corner {corners[0]}")
    m.send(f'G0 Z{SAFE_Z}')
    m.wait_ok(10)
    m.send(f'G0 X{corners[0][0]:.3f} Y{corners[0][1]:.3f}')
    m.wait_ok(30)
    m.wait_idle(30)

    print(f"Motor {args.motor.upper()}: {args.strokes} strokes between "
          f"{corners[0]} and {corners[1]} at F{args.feed}.")
    print("A switch touching means the machine lost steps towards it.\n")

    touches = []
    start = time.time()
    try:
        for stroke in range(1, args.strokes + 1):
            target = corners[stroke % 2]
            m.send(f'G1 X{target[0]:.3f} Y{target[1]:.3f} F{args.feed}')
            if stroke % 5 == 0:
                m.send('$MS')

            # Follow this stroke until the machine is back at a standstill
            deadline = time.time() + 60
            reported = False
            while time.time() < deadline:
                for line in m.poll(0.05):
                    if 'temp_shutdown:Y' in line or 'temp_warn:Y' in line:
                        print(f"  [stroke {stroke:3d}] {line}")
                    elif line.startswith('ALARM') or 'ALARM' in line:
                        print(f"  [stroke {stroke:3d}] {line}")
                if m.pins and not reported:
                    reported = True
                    touches.append((stroke, m.pins, m.pos))
                    print(f"  [stroke {stroke:3d}] LIMIT SWITCH {m.pins} touched at "
                          f"X={m.pos[0]:.2f} Y={m.pos[1]:.2f}  <-- drift")
                if m.state in ('Alarm', 'Critical'):
                    print(f"  [stroke {stroke:3d}] machine stopped in {m.state} at "
                          f"X={m.pos[0]:.2f} Y={m.pos[1]:.2f}")
                    raise SystemExit(0)
                if m.state == 'Idle':
                    break
            if stroke % 10 == 0:
                print(f"  ... {stroke} strokes, {time.time() - start:.0f}s, "
                      f"no switch touched" if not touches else
                      f"  ... {stroke} strokes, {len(touches)} touches so far")
    except KeyboardInterrupt:
        print("\nInterrupted - feed hold")
        m.realtime(b'!')
        time.sleep(1.0)
        m.realtime(b'\x18')
        return 1
    except SystemExit:
        pass

    print("\n=== RESULT ===")
    print(f"Motor {args.motor.upper()}, feed {args.feed}, {len(touches)} switch touches")
    for stroke, pins, pos in touches[:20]:
        print(f"  stroke {stroke:3d}: {pins} at X={pos[0]:.2f} Y={pos[1]:.2f}")
    if not touches:
        print("  No drift big enough to reach a switch (more than ~2 mm).")
    return 0


if __name__ == '__main__':
    sys.exit(main())
