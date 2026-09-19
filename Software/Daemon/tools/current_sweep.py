#!/usr/bin/env python3
"""Find the motor current where the machine neither overheats nor loses steps.

For each current it sets run_amps at runtime ($axes/.../run_amps + $MI),
sweeps an axis back and forth for a while, watches the TMC drivers for
over-temperature, and then re-homes that axis to measure how far the machine
drifted. Between runs it lets the drivers cool down with the motors disabled.

Too much current: the driver reaches its 143 C shutdown, that motor stops and
on CoreXY the machine runs off diagonally. Too little: the motors lack torque
and lose steps anyway. The sweet spot is the current with no thermal shutdown
and no drift.

Runs unattended: it stops the whole sweep as soon as the machine ends up
against a limit switch, because freeing it needs a hand.

Usage: current_sweep.py [PORT] [--currents 1.0,1.1,...] [--seconds 120]
"""

import argparse
import re
import subprocess
import sys
import time

import serial

CENTER_X = 100.0
SWEEP_MIN = 30.0
SWEEP_MAX = 180.0
SAFE_Z = 60.0
AXIS = 'Y'

STATUS_RE = re.compile(r'<(?P<state>[A-Za-z]+)[^|]*\|MPos:(?P<x>[-\d.]+),(?P<y>[-\d.]+),(?P<z>[-\d.]+)'
                       r'(?P<rest>[^>]*)>')
PIN_RE = re.compile(r'\|Pn:([^|>]+)')


def log(message):
    print(f"{time.strftime('%H:%M:%S')}  {message}", flush=True)


class Machine:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(0.5)
        self.state = None
        self.pos = (0.0, 0.0, 0.0)
        self.pins = ''
        self.send('$Report/Interval=0')
        time.sleep(0.5)
        self.serial.reset_input_buffer()

    def send(self, line):
        self.serial.write(line.encode() + b'\n')

    def poll(self, seconds=0.0):
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

    def status(self, timeout=3.0):
        deadline = time.time() + timeout
        self.state = None
        while time.time() < deadline:
            self.serial.write(b'?')
            self.poll(0.3)
            if self.state:
                return self.state
        return None

    def command(self, line, timeout=10.0):
        """Send a line and wait for ok/error."""
        self.send(line)
        deadline = time.time() + timeout
        while time.time() < deadline:
            for out in self.poll(0.2):
                if out.startswith('ok'):
                    return True
                if out.startswith('error:'):
                    log(f"    ! {line} -> {out}")
                    return False
        log(f"    ! {line} -> no answer")
        return False


def set_current(m, amps):
    ok = True
    for axis in ('x', 'y'):
        ok &= m.command(f'$axes/{axis}/motor0/tmc_2209/run_amps={amps:.3f}')
    ok &= m.command('$MI', timeout=20)   # re-apply the driver configuration
    return ok


def home(m, timeout=180):
    m.send('$H')
    deadline = time.time() + timeout
    while time.time() < deadline:
        for out in m.poll(0.2):
            if out.startswith('ok'):
                return True
            if out.startswith('error:'):
                log(f"    ! homing -> {out}")
                return False
    return False


def run_one(m, amps, seconds):
    log(f"--- {amps:.1f} A ---")
    if not set_current(m, amps):
        return {'amps': amps, 'error': 'could not set current'}

    if not home(m):
        m.status()
        return {'amps': amps, 'error': f'homing failed (state {m.state}, switch {m.pins!r})'}

    m.command('$Report/Interval=100', timeout=5)
    m.command(f'G0 Z{SAFE_Z}')
    m.command(f'G0 X{CENTER_X} Y{SWEEP_MIN}', timeout=30)

    warned = shutdown = False
    start = time.time()
    stroke = 0
    while time.time() - start < seconds:
        for target in (SWEEP_MAX, SWEEP_MIN):
            m.send(f'G1 X{CENTER_X} Y{target} F6000')
            stroke += 1
        m.send('$MS')
        time.sleep(1.0)
        for line in m.poll(0.3):
            if 'temp_shutdown:Y' in line:
                shutdown = True
            elif 'temp_warn:Y' in line:
                warned = True
        if m.state in ('Alarm', 'Critical'):
            return {'amps': amps, 'error': f'machine stopped in {m.state} at {m.pos}',
                    'warned': warned, 'shutdown': shutdown}

    m.command('G4 P0', timeout=90)

    # Re-home the axis and watch where the switch actually trips: that is how
    # far the machine drifted (about -2 mm is the normal offset).
    m.serial.reset_input_buffer()
    m.send(f'$H{AXIS}')
    drift = None
    index = {'X': 0, 'Y': 1}[AXIS]
    deadline = time.time() + 180
    while time.time() < deadline:
        done = False
        for out in m.poll(0.05):
            if out.startswith('ok'):
                done = True
            elif out.startswith('error:'):
                log(f"    ! re-homing -> {out}")
                done = True
        value = m.pos[index]
        drift = value if drift is None else min(drift, value)
        if done:
            break

    return {'amps': amps, 'strokes': stroke, 'warned': warned,
            'shutdown': shutdown, 'drift': drift}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--currents', default='1.0,1.1,1.2,1.3,1.4,1.5')
    parser.add_argument('--seconds', type=float, default=120.0)
    parser.add_argument('--cooldown', type=float, default=180.0)
    parser.add_argument('--restore', type=float, default=1.5)
    args = parser.parse_args()

    currents = [float(v) for v in args.currents.split(',')]
    log(f"Sweep over {currents} A, {args.seconds:.0f}s each, "
        f"{args.cooldown:.0f}s cooldown in between")

    m = Machine(args.port)
    log(f"Machine state: {m.status()} at {m.pos}"
        + (f", switch {m.pins} pressed" if m.pins else ""))

    results = []
    for index, amps in enumerate(currents):
        if index:
            log(f"Cooling down for {args.cooldown:.0f}s with the motors off")
            m.command('$MD', timeout=5)    # disable motors so they cool
            time.sleep(args.cooldown)
        result = run_one(m, amps, args.seconds)
        results.append(result)
        if 'error' in result:
            log(f"    {amps:.1f} A: {result['error']}")
            log("Stopping the sweep - the machine needs a hand before it can move again.")
            break
        log(f"    {amps:.1f} A: drift {result['drift']:+.1f} mm, "
            f"temp warning {'yes' if result['warned'] else 'no'}, "
            f"shutdown {'YES' if result['shutdown'] else 'no'} "
            f"({result['strokes']} strokes)")

    log("")
    log("=== SUMMARY ===")
    log(f"{'current':>8}  {'drift':>8}  {'temp warn':>9}  {'shutdown':>8}")
    for r in results:
        if 'error' in r:
            log(f"{r['amps']:7.1f}A  {r['error']}")
        else:
            log(f"{r['amps']:7.1f}A  {r['drift']:+7.1f}mm  "
                f"{'yes' if r['warned'] else 'no':>9}  {'YES' if r['shutdown'] else 'no':>8}")
    good = [r for r in results if 'error' not in r and not r['shutdown']
            and r['drift'] is not None and abs(r['drift'] + 2) < 3]
    if good:
        log(f"Best: {max(r['amps'] for r in good):.1f} A "
            f"(no thermal shutdown, drift within a few mm)")
    else:
        log("No current in this range was free of both problems - the drivers "
            "need cooling.")

    log(f"Restoring run_amps to {args.restore:.1f} A")
    set_current(m, args.restore)
    m.serial.close()

    log("Starting the daemon again")
    subprocess.run(['sudo', 'systemctl', 'start', 'alphapaint-daemon'], check=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
