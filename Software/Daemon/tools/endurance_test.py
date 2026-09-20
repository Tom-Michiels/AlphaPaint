#!/usr/bin/env python3
"""Run at one current for a long time and see whether it stays trustworthy.

Works in blocks: a few minutes of continuous fast motion, then a homing cycle
to measure drift, then the next block. Per block it records

  * how much of the time the driver sat above its 120 C warning threshold
    (the only temperature reading a TMC2209 gives us), which tells us when
    the machine has reached thermal steady state
  * whether a thermal shutdown happened
  * the drift measured by homing: where the switch tripped compared with the
    expected -2 mm, so a few mm either way is normal
  * any limit switch touched during the block itself (the near end of every
    stroke sits 2 mm from the switch, so drift shows up immediately)

Hard limits are off while this runs and restored at the end, together with
max_rate, the current and the daemon.

Usage: endurance_test.py [PORT] [--amps 1.2] [--minutes 60] [--block 5]
"""

import argparse
import csv
import subprocess
import sys
import time

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from stall_matrix import (CENTER_X, FAR_Y, NEAR_Y, SAFE_Z, Machine,  # noqa: E402
                          log, set_config, wait_switch_clear)

NORMAL_TRIP = -2.0      # where the switch trips on a machine that did not drift


def measure_drift(m, axis='Y'):
    """Home one axis and return where the switch actually tripped."""
    index = {'X': 0, 'Y': 1}[axis]
    m.serial.reset_input_buffer()
    m.send(f'$H{axis}')
    lowest = None
    deadline = time.time() + 180
    while time.time() < deadline:
        done = False
        for out in m.poll(0.05):
            if out.startswith('ok'):
                done = True
            elif out.startswith('error:'):
                log(f"      ! homing -> {out}")
                return None
        if m.state in ('Home', 'Homing'):
            value = m.pos[index]
            lowest = value if lowest is None else min(lowest, value)
        if done:
            return lowest
    return None


def work_block(m, feed, seconds):
    """Move continuously; report temperature duty and any switch touch."""
    m.temp_warn = m.temp_shutdown = False
    m.pins = ''
    polls = warm_polls = 0
    touch = None
    shutdown = False
    queued = 0
    start = time.time()

    while time.time() - start < seconds:
        while queued < 3:
            m.send(f'G1 X{CENTER_X} Y{FAR_Y} F{feed}')
            m.send(f'G1 X{CENTER_X} Y{NEAR_Y} F{feed}')
            queued += 1
        m.temp_warn = False
        m.send('$MS')
        m.poll(2.0)
        polls += 1
        if m.temp_warn:
            warm_polls += 1
        if m.temp_shutdown and not shutdown:
            shutdown = True
            log(f"      THERMAL SHUTDOWN after {(time.time() - start) / 60:.1f} min "
                f"of this block")
        if m.pins and 'Y' in m.pins and touch is None:
            touch = m.pos[1]
            log(f"      switch tripped mid-block at Y={touch:+.2f} - lost steps")
            break
        if m.state in ('Alarm', 'Critical'):
            log(f"      machine stopped in {m.state} at {m.pos}")
            break
        if m.state == 'Idle':
            queued = 0

    m.serial.write(b'!')
    time.sleep(1.0)
    m.serial.write(b'\x18')
    time.sleep(2.0)
    m.serial.reset_input_buffer()
    return {'warm_fraction': (warm_polls / polls) if polls else 0.0,
            'shutdown': shutdown, 'touch': touch}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--amps', type=float, default=1.2)
    parser.add_argument('--feed', type=int, default=6000)
    parser.add_argument('--minutes', type=float, default=60.0, help='total run time')
    parser.add_argument('--block', type=float, default=5.0, help='minutes between drift checks')
    parser.add_argument('--restore-current', type=float, default=1.5)
    parser.add_argument('--csv', default='/home/tom/endurance.csv')
    args = parser.parse_args()

    log(f"Endurance run at {args.amps:.1f} A, F{args.feed}, {args.minutes:.0f} min "
        f"in blocks of {args.block:.0f} min with a homing check after each")

    m = Machine(args.port)
    log(f"Machine: {m.status()} at {m.pos}")

    for axis in ('x', 'y'):
        set_config(m, f'axes/{axis}/motor0/hard_limits', 'false')
        set_config(m, f'axes/{axis}/max_rate_mm_per_min', args.feed + 1000)
        set_config(m, f'axes/{axis}/motor0/tmc_2209/run_amps', f'{args.amps:.3f}')
    m.command('$MI', timeout=20)

    if not m.home():
        log("Initial homing failed - stopping.")
        return 1
    m.command('$Report/Interval=100', timeout=5)
    m.command(f'G0 Z{SAFE_Z}')

    rows = []
    start = time.time()
    block = 0
    stop_reason = 'time limit reached'
    while (time.time() - start) / 60 < args.minutes:
        block += 1
        m.command(f'G0 X{CENTER_X} Y{NEAR_Y}', timeout=60)
        m.wait_idle()
        if not wait_switch_clear(m):
            stop_reason = 'Y switch stayed pressed'
            break

        log(f"--- block {block} ({(time.time() - start) / 60:.0f} min in) ---")
        result = work_block(m, args.feed, args.block * 60)
        drift = measure_drift(m)
        offset = None if drift is None else drift - NORMAL_TRIP
        log(f"    block {block}: above 120 C for {result['warm_fraction']:5.1%} of the time, "
            f"drift {('%+.1f mm' % offset) if offset is not None else 'not measured'}"
            + (", THERMAL SHUTDOWN" if result['shutdown'] else ""))

        rows.append({'block': block,
                     'minutes': round((time.time() - start) / 60, 1),
                     'warm_fraction': round(result['warm_fraction'], 3),
                     'shutdown': result['shutdown'],
                     'drift_mm': '' if offset is None else round(offset, 2),
                     'mid_block_touch': '' if result['touch'] is None else round(result['touch'], 2)})
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        if result['touch'] is not None:
            stop_reason = 'lost steps during a block'
            break
        if result['shutdown']:
            stop_reason = 'thermal shutdown'
            break
        if offset is not None and abs(offset) > 3.0:
            stop_reason = f'drift of {offset:+.1f} mm measured by homing'
            break

    log("")
    log(f"=== {args.amps:.1f} A, stopped because: {stop_reason} ===")
    log(f"{'block':>6} {'minutes':>8} {'>120C':>8} {'drift mm':>9} {'shutdown':>9}")
    for row in rows:
        log(f"{row['block']:6d} {row['minutes']:8.1f} {row['warm_fraction']:7.1%} "
            f"{str(row['drift_mm']):>9} {'YES' if row['shutdown'] else '-':>9}")
    warm = [r['warm_fraction'] for r in rows]
    if len(warm) >= 3 and max(warm[-3:]) - min(warm[-3:]) < 0.1:
        log(f"Temperature looks settled: the last three blocks sat above 120 C for "
            f"{min(warm[-3:]):.0%}-{max(warm[-3:]):.0%} of the time.")
    drifts = [r['drift_mm'] for r in rows if r['drift_mm'] != '']
    if drifts:
        log(f"Drift over the whole run stayed between {min(drifts):+.1f} and "
            f"{max(drifts):+.1f} mm.")
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
