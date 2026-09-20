#!/usr/bin/env python3
"""How long can the machine work before a driver gives up?

For each motor current it drives an axis back and forth without pause - both
legs fast, so the drivers heat up the way they do during a real drawing - and
records when things happen:

  * first over-temperature warning   (>120 C)
  * first thermal shutdown           (143 C: that motor stops turning)
  * first lost step                  (the near end of every stroke sits 2 mm
                                      from the limit switch, so a machine that
                                      drifted towards it presses the switch)

A run ends at the first lost step, at the first shutdown (optional) or after
the time limit. Between currents the motors are switched off until the
temperature warning clears again, so each run starts cold.

The result is a working time per current: "at 1.5 A the first drop-out comes
after three minutes, at 1.2 A nothing happens in twenty" - which is what you
need to know for a drawing that takes half an hour.

Hard limits are off while this runs (a touch must be a measurement, not an
alarm that leaves the machine stuck against its own switch) and restored at
the end, together with max_rate, the current and the daemon.

Usage: heat_soak.py [PORT] [--currents 1.2,1.3,1.4,1.5] [--minutes 15]
"""

import argparse
import csv
import subprocess
import sys
import time

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from stall_matrix import (CENTER_X, FAR_Y, NEAR_Y, SAFE_Z, Machine,  # noqa: E402
                          log, set_config, wait_switch_clear)


def cool_down(m, max_seconds, settle=45.0):
    """Motors off until the temperature warning clears, then a little longer."""
    log(f"    cooling down (max {max_seconds / 60:.0f} min)")
    m.command('$MD', timeout=5)
    deadline = time.time() + max_seconds
    cleared_at = None
    while time.time() < deadline:
        m.temp_warn = False
        m.command('$MS', timeout=10)
        m.poll(0.5)
        if not m.temp_warn:
            if cleared_at is None:
                cleared_at = time.time()
                log(f"    temperature warning gone after "
                    f"{cleared_at - (deadline - max_seconds):.0f}s")
            if time.time() - cleared_at >= settle:
                return True
        else:
            cleared_at = None
        time.sleep(5)
    log("    still warm when the cooldown ran out")
    return False


def soak(m, amps, feed, minutes, stop_on_shutdown):
    """Run one current until something gives or the time is up."""
    for axis in ('x', 'y'):
        set_config(m, f'axes/{axis}/motor0/tmc_2209/run_amps', f'{amps:.3f}')
    m.command('$MI', timeout=20)

    if not m.home('Y'):
        return {'amps': amps, 'error': 'homing failed'}
    m.command(f'G0 X{CENTER_X} Y{NEAR_Y}', timeout=60)
    m.wait_idle()
    if not wait_switch_clear(m):
        return {'amps': amps, 'error': 'Y switch stays pressed after homing'}

    m.temp_warn = m.temp_shutdown = False
    m.pins = ''
    start = time.time()
    limit = start + minutes * 60
    first_warn = first_shutdown = first_loss = None
    cycles = 0
    queued = 0

    log(f"    running at F{feed}, up to {minutes:.0f} min")
    while time.time() < limit:
        # Keep a couple of strokes queued so the machine never pauses
        while queued < 3:
            m.send(f'G1 X{CENTER_X} Y{FAR_Y} F{feed}')
            m.send(f'G1 X{CENTER_X} Y{NEAR_Y} F{feed}')
            queued += 1
            cycles += 1
        m.send('$MS')
        m.poll(2.0)
        elapsed = time.time() - start

        if m.temp_warn and first_warn is None:
            first_warn = elapsed
            log(f"      {elapsed / 60:5.1f} min: over-temperature warning")
        if m.temp_shutdown and first_shutdown is None:
            first_shutdown = elapsed
            log(f"      {elapsed / 60:5.1f} min: THERMAL SHUTDOWN - a motor stops here")
            if stop_on_shutdown:
                break
        if m.pins and 'Y' in m.pins and first_loss is None:
            first_loss = elapsed
            log(f"      {elapsed / 60:5.1f} min: LOST STEPS (switch tripped at "
                f"Y={m.pos[1]:+.2f})")
            break
        if m.state in ('Alarm', 'Critical'):
            log(f"      machine stopped in {m.state} at {m.pos}")
            break
        if m.state == 'Idle':
            queued = 0      # the queue ran out; refill it

    m.serial.write(b'!')    # feed hold
    time.sleep(1.0)
    m.serial.write(b'\x18')  # and flush what is left
    time.sleep(2.0)
    m.serial.reset_input_buffer()

    elapsed = time.time() - start
    return {'amps': amps, 'feed': feed, 'minutes_run': round(elapsed / 60, 1),
            'cycles': cycles,
            'warn_min': round(first_warn / 60, 1) if first_warn else '',
            'shutdown_min': round(first_shutdown / 60, 1) if first_shutdown else '',
            'lost_steps_min': round(first_loss / 60, 1) if first_loss else ''}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default='/dev/ttyUSB0')
    parser.add_argument('--currents', default='1.2,1.3,1.4,1.5')
    parser.add_argument('--feed', type=int, default=6000)
    parser.add_argument('--minutes', type=float, default=15.0)
    parser.add_argument('--cooldown-minutes', type=float, default=10.0)
    parser.add_argument('--keep-running-after-shutdown', action='store_true',
                        help='do not stop at the first thermal shutdown')
    parser.add_argument('--restore-current', type=float, default=1.5)
    parser.add_argument('--csv', default='/home/tom/heat-soak.csv')
    args = parser.parse_args()

    currents = [float(v) for v in args.currents.split(',')]
    log(f"Heat soak at F{args.feed}, {currents} A, up to {args.minutes:.0f} min each")

    m = Machine(args.port)
    log(f"Machine: {m.status()} at {m.pos}")

    for axis in ('x', 'y'):
        set_config(m, f'axes/{axis}/motor0/hard_limits', 'false')
        set_config(m, f'axes/{axis}/max_rate_mm_per_min', args.feed + 1000)
    m.command('$MI', timeout=20)
    if not m.home():
        log("Initial homing failed - stopping.")
        return 1
    m.command('$Report/Interval=100', timeout=5)
    m.command(f'G0 Z{SAFE_Z}')

    rows = []
    for index, amps in enumerate(currents):
        if index:
            cool_down(m, args.cooldown_minutes * 60)
        log(f"--- {amps:.1f} A ---")
        row = soak(m, amps, args.feed, args.minutes,
                   not args.keep_running_after_shutdown)
        rows.append(row)
        if row.get('error'):
            log(f"    {amps:.1f} A: {row['error']} - stopping")
            break
        log(f"    {amps:.1f} A: ran {row['minutes_run']} min, "
            f"warning at {row['warn_min'] or '-'}, shutdown at {row['shutdown_min'] or '-'}, "
            f"lost steps at {row['lost_steps_min'] or '-'} (minutes)")
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    log("")
    log("=== WORKING TIME PER CURRENT (minutes) ===")
    log(f"{'current':>8} {'ran':>6} {'warning':>9} {'shutdown':>9} {'lost steps':>11}")
    for row in rows:
        if row.get('error'):
            log(f"{row['amps']:7.1f}A  {row['error']}")
        else:
            log(f"{row['amps']:7.1f}A {row['minutes_run']:6.1f} {str(row['warn_min'] or '-'):>9} "
                f"{str(row['shutdown_min'] or '-'):>9} {str(row['lost_steps_min'] or '-'):>11}")
    log("An empty column means it never happened within the time limit.")
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
