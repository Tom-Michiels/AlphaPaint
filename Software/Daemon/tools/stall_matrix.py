#!/usr/bin/env python3
"""Map where the machine starts losing steps: motor current against speed.

Tom's test pattern, one cycle:

  1. stand just above the Y limit switch (MPos Y=0, the switch trips ~2 mm lower)
  2. move away from it FAST - this is where a stall happens
  3. come back SLOWLY to Y=0 - a machine that lost steps now presses the switch

So every cycle checks itself, and the position at which the switch trips says
how far the machine drifted. Hard limits are switched off for the duration:
otherwise the first touch alarms the machine against its own switch, and
freeing it needs a hand. They go back on at the end.

The grid is currents x speeds; each cell runs a number of cycles and the
drivers cool down in between. Results also go to a CSV next to the log.

The daemon must be stopped (it owns the serial port).

Usage: stall_matrix.py [PORT] [--currents 1.1,1.2,1.3,1.4]
                       [--feeds 3000,4500,6000,7500,9000] [--cycles 8]
"""

import argparse
import csv
import re
import subprocess
import sys
import time

import serial

CENTER_X = 100.0
NEAR_Y = 0.0         # closest we may command; the switch is ~2 mm below this
FAR_Y = 180.0
RETURN_FEED = 2000   # slow leg: should never stall by itself
SAFE_Z = 60.0

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
        self.temp_warn = False
        self.temp_shutdown = False
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
                    if 'temp_shutdown:Y' in line:
                        self.temp_shutdown = True
                    if 'temp_warn:Y' in line:
                        self.temp_warn = True
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

    def command(self, line, timeout=15.0):
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

    def wait_idle(self, timeout=90.0):
        """Wait until motion stops; returns the lines seen meanwhile."""
        deadline = time.time() + timeout
        seen = []
        while time.time() < deadline:
            seen += self.poll(0.05)
            if self.state == 'Idle':
                return seen
            if self.state in ('Alarm', 'Critical'):
                return seen
        return seen

    def home(self, axis='', timeout=180):
        self.serial.reset_input_buffer()
        self.send(f'$H{axis}')
        deadline = time.time() + timeout
        while time.time() < deadline:
            for out in self.poll(0.2):
                if out.startswith('ok'):
                    return True
                if out.startswith('error:'):
                    log(f"    ! homing{axis} -> {out}")
                    return False
        log(f"    ! homing{axis} timed out")
        return False


def set_config(m, path, value):
    return m.command(f'${path}={value}')


def wait_switch_clear(m, timeout=10.0):
    """Wait until the Y switch reads released (it is still pressed right after
    homing, which would otherwise count as a drift touch)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.status(1.0)
        if not m.pins or 'Y' not in m.pins:
            return True
    return False


def run_cell(m, amps, feed, cycles):
    """One grid cell: cycles of fast-away / slow-back, counting switch touches."""
    m.temp_warn = m.temp_shutdown = False
    touches = []

    if not m.home('Y'):
        return {'error': 'homing failed before the cell'}
    m.command(f'G0 X{CENTER_X} Y{NEAR_Y}', timeout=60)
    m.wait_idle()
    if not wait_switch_clear(m):
        return {'error': 'Y switch stays pressed after homing'}

    for cycle in range(1, cycles + 1):
        m.send(f'G1 X{CENTER_X} Y{FAR_Y} F{feed}')       # fast away
        m.send(f'G1 X{CENTER_X} Y{NEAR_Y} F{RETURN_FEED}')  # slow back
        m.send('$MS')

        touched = False
        m.pins = ''          # only count touches made during this cycle
        deadline = time.time() + 120
        while time.time() < deadline:
            m.poll(0.05)
            if m.pins and 'Y' in m.pins and not touched:
                touched = True
                touches.append((cycle, m.pos[1]))
                log(f"      cycle {cycle}: switch tripped at Y={m.pos[1]:+.2f} "
                    f"-> lost about {m.pos[1] + 2:.1f} mm")
            if m.state in ('Alarm', 'Critical'):
                return {'error': f'machine stopped in {m.state} at {m.pos}',
                        'touches': touches}
            if m.state == 'Idle':
                break
        if touched:
            # Re-zero so the next cycle measures fresh drift again
            if not m.home('Y'):
                return {'error': 'homing failed after a switch touch', 'touches': touches}
            m.command(f'G0 X{CENTER_X} Y{NEAR_Y}', timeout=60)
            m.wait_idle()
            if not wait_switch_clear(m):
                return {'error': 'Y switch stays pressed after homing', 'touches': touches}

    return {'touches': touches, 'cycles': cycles,
            'warn': m.temp_warn, 'shutdown': m.temp_shutdown}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--currents', default='1.1,1.2,1.3,1.4')
    parser.add_argument('--feeds', default='3000,4500,6000,7500,9000')
    parser.add_argument('--cycles', type=int, default=8)
    parser.add_argument('--cooldown', type=float, default=60.0)
    parser.add_argument('--restore-current', type=float, default=1.5)
    parser.add_argument('--csv', default='/home/tom/stall-matrix.csv')
    args = parser.parse_args()

    currents = [float(v) for v in args.currents.split(',')]
    feeds = [int(v) for v in args.feeds.split(',')]
    log(f"Grid: {currents} A x {feeds} mm/min, {args.cycles} cycles per cell")
    log(f"Pattern: fast to Y={FAR_Y:.0f}, slow back to Y={NEAR_Y:.0f} at F{RETURN_FEED}")

    m = Machine(args.port)
    log(f"Machine: {m.status()} at {m.pos}" + (f", switch {m.pins}" if m.pins else ""))

    # Hard limits off: a touch must be a measurement, not an alarm that leaves
    # the machine stuck against its own switch. Raise max_rate so the feed
    # rates below are what actually happens.
    for axis in ('x', 'y'):
        set_config(m, f'axes/{axis}/motor0/hard_limits', 'false')
        set_config(m, f'axes/{axis}/max_rate_mm_per_min', max(feeds) + 1000)
    m.command('$MI', timeout=20)

    if not m.home():
        log("Initial homing failed - stopping.")
        return 1
    m.command('$Report/Interval=100', timeout=5)
    m.command(f'G0 Z{SAFE_Z}')

    rows = []
    stop = False
    for amps in currents:
        if stop:
            break
        for axis in ('x', 'y'):
            set_config(m, f'axes/{axis}/motor0/tmc_2209/run_amps', f'{amps:.3f}')
        m.command('$MI', timeout=20)
        for feed in feeds:
            log(f"--- {amps:.1f} A, F{feed} ---")
            result = run_cell(m, amps, feed, args.cycles)
            row = {'amps': amps, 'feed': feed,
                   'touches': len(result.get('touches', [])),
                   'first_touch_cycle': result['touches'][0][0] if result.get('touches') else '',
                   'worst_mm': (f"{min(t[1] for t in result['touches']) + 2:.1f}"
                                if result.get('touches') else ''),
                   'temp_warn': result.get('warn', ''), 'temp_shutdown': result.get('shutdown', ''),
                   'error': result.get('error', '')}
            rows.append(row)
            if result.get('error'):
                log(f"    {amps:.1f} A F{feed}: {result['error']}")
                log("Stopping: the machine needs a hand before it can move again.")
                stop = True
                break
            log(f"    {amps:.1f} A F{feed}: {row['touches']}/{args.cycles} cycles lost steps"
                + (f", worst {row['worst_mm']} mm" if row['worst_mm'] else "")
                + (", TEMP SHUTDOWN" if result.get('shutdown') else
                   (", temp warning" if result.get('warn') else "")))
            with open(args.csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
            if args.cooldown:
                m.command('$MD', timeout=5)
                time.sleep(args.cooldown)

    log("")
    log("=== GRID (cycles that lost steps, out of %d) ===" % args.cycles)
    header = "   A \\ F  " + "".join(f"{feed:>8}" for feed in feeds)
    log(header)
    for amps in currents:
        cells = []
        for feed in feeds:
            row = next((r for r in rows if r['amps'] == amps and r['feed'] == feed), None)
            if row is None:
                cells.append("     -")
            elif row['error']:
                cells.append("    ERR")
            else:
                mark = '!' if row['temp_shutdown'] else ''
                cells.append(f"{row['touches']:>7}{mark}")
        log(f"  {amps:.1f}A    " + "".join(f"{c:>8}" for c in cells))
    log("('!' marks a thermal shutdown; lower is better, 0 is clean)")
    log(f"CSV: {args.csv}")

    log("Restoring hard limits, max rate and current")
    for axis in ('x', 'y'):
        set_config(m, f'axes/{axis}/motor0/hard_limits', 'true')
        set_config(m, f'axes/{axis}/max_rate_mm_per_min', '6000')
        set_config(m, f'axes/{axis}/motor0/tmc_2209/run_amps', f'{args.restore_current:.3f}')
    m.command('$MI', timeout=20)
    m.serial.close()

    log("Starting the daemon again")
    subprocess.run(['sudo', 'systemctl', 'start', 'alphapaint-daemon'], check=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
