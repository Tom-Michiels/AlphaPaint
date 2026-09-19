#!/usr/bin/env python3
"""
Post-mortem analysis for the FluidNC position-trace CSV.

Default path: /var/log/alphapaint-position-trace.csv
Override with the first positional argument.

Reports:
  - Discontinuity events (idle drift, end-of-motion mismatch, soft reset, ...)
  - State transition timeline
  - Top-N samples by expected/reported delta
  - Run summary (sample count, time covered, motion vs idle ratio)

Usage:
    ./analyze_position_trace.py                       # default file
    ./analyze_position_trace.py /tmp/run.csv          # custom file
    ./analyze_position_trace.py --tail 1000           # last 1000 samples only
    ./analyze_position_trace.py --since 2026-05-03T14 # samples from that ISO time onwards
    ./analyze_position_trace.py --top 30              # show 30 worst deltas
"""

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

DEFAULT_PATH = '/var/log/alphapaint-position-trace.csv'

EVENT_KINDS = {'STATE', 'SOFT_RESET', 'UNLOCK', 'WCS', 'MODE',
               'IDLE_DRIFT', 'END_MISMATCH', 'CMD', 'INIT'}


def load_rows(path, tail=None, since=None):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if since and row.get('iso_time', '') < since:
                continue
            rows.append(row)
    if tail:
        rows = rows[-tail:]
    return rows


def to_float(s, default=None):
    if s is None or s == '':
        return default
    try:
        return float(s)
    except ValueError:
        return default


def report_events(rows):
    print('\n=== EVENTS ===')
    events = [r for r in rows if r['kind'] in EVENT_KINDS - {'INIT'}]
    if not events:
        print('  (none)')
        return
    counter = Counter(r['kind'] for r in events)
    print('  Counts: ' + ', '.join(f'{k}={v}' for k, v in counter.most_common()))
    print()
    # Show every event with timestamp + context
    for r in events:
        cmd = r.get('last_cmd', '').strip('"')
        mpos = f"({r.get('mpos_x','')},{r.get('mpos_y','')},{r.get('mpos_z','')})"
        ev = r.get('event', '')
        print(f"  {r['iso_time']:23} {r['kind']:12} state={r.get('state','')!s:8} "
              f"mpos={mpos:24} {ev}  last_cmd={cmd!r}")


def report_state_timeline(rows):
    print('\n=== STATE TIMELINE ===')
    transitions = [r for r in rows if r['kind'] == 'STATE']
    if not transitions:
        print('  (no transitions recorded)')
        return
    for r in transitions:
        print(f"  {r['iso_time']:23} {r.get('event',''):20} "
              f"mpos=({r.get('mpos_x','')},{r.get('mpos_y','')},{r.get('mpos_z','')})")


def report_top_deltas(rows, top_n):
    print(f'\n=== TOP {top_n} SAMPLES BY |MPOS - EXPECTED| (xy) ===')
    samples = [(to_float(r.get('delta_xy'), 0), r) for r in rows
               if r['kind'] == 'S' and r.get('delta_xy')]
    samples.sort(key=lambda x: x[0], reverse=True)
    if not samples:
        print('  (no samples with delta data)')
        return
    print(f"  {'time':23} {'state':6} {'delta_xy':>9} {'delta_z':>9} "
          f"{'mpos':>26} {'expected':>26} last_cmd")
    for delta, r in samples[:top_n]:
        cmd = r.get('last_cmd', '').strip('"')
        mpos = f"({r.get('mpos_x','')},{r.get('mpos_y','')},{r.get('mpos_z','')})"
        exp = f"({r.get('exp_x','')},{r.get('exp_y','')},{r.get('exp_z','')})"
        print(f"  {r['iso_time']:23} {r.get('state','')[:6]:6} "
              f"{delta:9.3f} {to_float(r.get('delta_z'),0):9.3f} "
              f"{mpos:>26} {exp:>26} {cmd}")


def report_summary(rows):
    print('\n=== SUMMARY ===')
    if not rows:
        print('  (empty trace)')
        return
    samples = [r for r in rows if r['kind'] == 'S']
    states = Counter(r.get('state', '') for r in samples)
    print(f"  Total rows         : {len(rows)}")
    print(f"  Status samples     : {len(samples)}")
    print(f"  Time span          : {rows[0]['iso_time']}  ->  {rows[-1]['iso_time']}")
    if rows[0].get('mono') and rows[-1].get('mono'):
        try:
            span = float(rows[-1]['mono']) - float(rows[0]['mono'])
            print(f"  Monotonic duration : {span:.1f}s")
            if samples and span > 0:
                print(f"  Sample rate        : {len(samples)/span:.1f} Hz")
        except ValueError:
            pass
    print('  State distribution :')
    for state, n in states.most_common():
        print(f"    {state or '?':10} {n:6}  ({100*n/len(samples):.1f}%)")
    # Counts of event kinds
    event_counts = Counter(r['kind'] for r in rows
                           if r['kind'] != 'S')
    if event_counts:
        print('  Event kinds        :')
        for kind, n in event_counts.most_common():
            print(f'    {kind:14} {n}')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('path', nargs='?', default=DEFAULT_PATH,
                   help=f'CSV trace file (default: {DEFAULT_PATH})')
    p.add_argument('--tail', type=int, default=None,
                   help='Only analyze the last N rows')
    p.add_argument('--since', default=None,
                   help='Only rows with iso_time >= this prefix '
                        '(e.g. 2026-05-03T14)')
    p.add_argument('--top', type=int, default=20,
                   help='Show top-N samples by delta (default 20)')
    args = p.parse_args()

    if not Path(args.path).exists():
        print(f"Trace file not found: {args.path}", file=sys.stderr)
        sys.exit(1)

    rows = load_rows(args.path, tail=args.tail, since=args.since)
    print(f"Loaded {len(rows)} rows from {args.path}")

    report_summary(rows)
    report_events(rows)
    report_state_timeline(rows)
    report_top_deltas(rows, args.top)


if __name__ == '__main__':
    main()
