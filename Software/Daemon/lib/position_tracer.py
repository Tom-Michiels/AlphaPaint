"""
Position tracer for FluidNC debugging.

Tracks the position FluidNC *should* be at (computed from outgoing G-code)
versus the position it *reports* (from auto-report MPos), so we can spot
discrepancies that hint at lost steps, soft resets clearing state, planner
faults, or coordinate-system surprises.

Three layers of output:

1. Real-time WARNING log (always on when tracer is enabled):
   - State transitions to Alarm/Door/Hold/Sleep
   - Soft reset, $H, $X, G92, G10
   - End-of-motion mismatches (Run -> Idle but reported MPos != expected)
   - Idle-state MPos drift (position changed without a motion command)

2. CSV trace file (opt-in via config: debugging.position_trace_file):
   - One row per status auto-report (~10 Hz)
   - Plus event rows (state changes, commands, discontinuities)
   - Designed for post-mortem analysis with tools/analyze_position_trace.py

3. Counters via get_stats() for periodic health reporting.

Thread safety: all public methods take the internal lock. note_outgoing()
is called from the main thread (send_gcode), note_status() and the event
hooks are called from the FluidNC read thread.
"""

import logging
import re
import threading
import time
from typing import Dict, Optional


# Tolerance for "MPos == expected" comparison. Below this we treat the
# difference as floating-point noise (FluidNC reports 3 decimals).
POS_EQUAL_EPSILON = 0.02

# Mismatch threshold at end of motion (Run -> Idle): if reported and
# expected differ by more than this, we log a warning. 0.5 mm is well
# above any rounding/coordinate-system noise but small enough to catch
# real step loss on a 300x300 mm canvas.
END_OF_MOTION_MISMATCH_MM = 0.5

# Drift threshold while idle (no command in flight): MPos shouldn't move
# at all when nothing is being commanded. Anything above measurement
# noise is a real event worth investigating.
IDLE_DRIFT_MM = 0.05

# How long after the last outgoing command we still consider the machine
# "potentially moving" even if state hasn't ticked to Run yet. Avoids
# false idle-drift warnings during the gap between sending and the next
# auto-report arriving.
COMMAND_SETTLE_S = 0.5


# Strip comments (parens and semicolons) before parsing.
_COMMENT_RE = re.compile(r'\(.*?\)|;.*')
# Strip line numbers like "N100 ".
_LINENO_RE = re.compile(r'^N\d+\s*', re.IGNORECASE)
# Match G-code words: a letter followed by a signed number.
_WORD_RE = re.compile(r'([A-Za-z])\s*([-+]?\d*\.?\d+)')


def _parse_words(line: str) -> list:
    """Return [(letter, value_str), ...] for a single G-code line."""
    line = _COMMENT_RE.sub('', line)
    line = _LINENO_RE.sub('', line.strip())
    return [(letter.upper(), value) for letter, value in _WORD_RE.findall(line)]


class PositionTracer:
    """Tracks expected vs reported FluidNC position."""

    # Default homing endpoints from the FluidNC config (mpos_mm).
    # If the actual config differs, this is wrong on the very first
    # post-home sample but self-corrects from the next status report.
    DEFAULT_HOME_MPOS = {'X': 0.0, 'Y': 0.0, 'Z': 60.0}

    def __init__(self, trace_file: Optional[str] = None):
        """
        Args:
            trace_file: Path to CSV trace file, or None to disable CSV output.
                        Events are still logged to the main logger regardless.
        """
        self.logger = logging.getLogger(__name__)
        self._lock = threading.Lock()

        # Modal G-code state for parsing.
        self._absolute_mode = True   # G90 default
        self._mm_mode = True          # G21 default

        # Position state. None means "unknown" — we don't compare or warn
        # until we have a baseline (after first status or first $H).
        self._expected: Optional[Dict[str, float]] = None
        self._reported: Optional[Dict[str, float]] = None
        self._reported_state: Optional[str] = None
        self._prev_state: Optional[str] = None
        self._prev_reported: Optional[Dict[str, float]] = None

        # Last command sent (most recent) — for trace context.
        self._last_cmd: str = ''
        self._last_cmd_time: float = 0.0
        # When the most recent motion command was sent. Used to suppress
        # false idle-drift warnings during the settle window between
        # send and first status report reflecting it.
        self._last_motion_cmd_time: float = 0.0
        # Counter so analysis can detect missing samples.
        self._sample_seq: int = 0

        # Stats for periodic health reporting.
        self._stats = {
            'discontinuities': 0,
            'end_of_motion_mismatches': 0,
            'idle_drifts': 0,
            'state_transitions': 0,
            'soft_resets': 0,
            'unlocks': 0,
            'homings': 0,
            'wcs_changes': 0,
            'alarm_events': 0,
        }

        # CSV writer — append mode so daemon restarts don't lose history.
        self._trace_file = trace_file
        self._trace_fh = None
        if trace_file:
            try:
                # Buffering=1 = line-buffered, flushed on each \n.
                self._trace_fh = open(trace_file, 'a', buffering=1)
                # Write header only if the file is empty (fresh file).
                if self._trace_fh.tell() == 0:
                    self._trace_fh.write(
                        'iso_time,mono,seq,kind,state,'
                        'mpos_x,mpos_y,mpos_z,'
                        'exp_x,exp_y,exp_z,'
                        'delta_xy,delta_z,event,last_cmd\n'
                    )
                self.logger.info(f"Position trace CSV: {trace_file}")
            except Exception as e:
                self.logger.warning(f"Cannot open trace file {trace_file}: {e}")
                self._trace_fh = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def close(self):
        """Close trace file. Safe to call multiple times."""
        with self._lock:
            if self._trace_fh:
                try:
                    self._trace_fh.close()
                except Exception:
                    pass
                self._trace_fh = None

    def get_stats(self) -> Dict[str, int]:
        with self._lock:
            return dict(self._stats)

    def get_expected(self) -> Optional[Dict[str, float]]:
        """Return current expected MPos (copy), or None if unknown."""
        with self._lock:
            return dict(self._expected) if self._expected else None

    def note_outgoing(self, gcode: str):
        """
        Called for every command sent to FluidNC (including those that
        don't produce motion). Updates expected MPos and modal state.
        """
        if not gcode:
            return

        cmd = gcode.strip()
        with self._lock:
            self._last_cmd = cmd
            self._last_cmd_time = time.time()

            # Detect special $-commands first. They carry no G-code words
            # ("$H" has no number), so check them before the empty-words exit.
            cmd_upper = cmd.upper()
            if re.fullmatch(r'\$H[XYZ]*', cmd_upper):
                self._handle_homing(cmd_upper[2:].strip())
                self._write_event('CMD', f'homing: {cmd}')
                return
            if cmd_upper.startswith('$X'):
                self._stats['unlocks'] += 1
                self.logger.warning(
                    f"$X unlock sent (expected={self._fmt_pos(self._expected)}, "
                    f"reported={self._fmt_pos(self._reported)}) — "
                    f"position state is now suspect until next $H")
                self._write_event('UNLOCK', f'cmd={cmd}')
                return
            if cmd_upper.startswith('$J='):
                # Strip "$J=" prefix, parse the rest as gcode.
                self._handle_motion_words(_parse_words(cmd[3:]),
                                          jog_local_mode=True)
                self._last_motion_cmd_time = self._last_cmd_time
                return

            words = _parse_words(cmd)
            if not words:
                return

            # Detect modal/coordinate changes from G-words.
            absolute_save = self._absolute_mode
            for letter, value in words:
                if letter != 'G':
                    continue
                gnum = self._gnum(value)
                if gnum == 90:
                    self._absolute_mode = True
                elif gnum == 91:
                    self._absolute_mode = False
                elif gnum == 20:
                    self._mm_mode = False
                    self.logger.warning("G20 (inch mode) sent — "
                                        "tracer assumes mm; expected position will be wrong")
                elif gnum == 21:
                    self._mm_mode = True
                elif gnum == 92:
                    self._stats['wcs_changes'] += 1
                    self.logger.warning(f"G92 sent (work-offset change): {cmd} "
                                        f"— MPos unaffected, WPos shifted")
                    self._write_event('WCS', f'g92: {cmd}')
                elif gnum == 10:
                    self._stats['wcs_changes'] += 1
                    self.logger.warning(f"G10 sent (WCS change): {cmd} "
                                        f"— MPos unaffected")
                    self._write_event('WCS', f'g10: {cmd}')

            # Motion commands: G0/G1/G2/G3.
            has_motion = any(letter == 'G' and self._gnum(val) in (0, 1, 2, 3)
                             for letter, val in words)
            if has_motion:
                self._handle_motion_words(words, jog_local_mode=False)
                self._last_motion_cmd_time = self._last_cmd_time
            elif any(letter in ('X', 'Y', 'Z') for letter, _ in words):
                # Bare "X10 Y20" reuses the current modal motion (G0/G1).
                # We don't track the modal motion explicitly — assume it
                # produces motion either way.
                self._handle_motion_words(words, jog_local_mode=False)
                self._last_motion_cmd_time = self._last_cmd_time

            # If absolute mode changed, leave it set (modal).
            if absolute_save != self._absolute_mode:
                self._write_event('MODE',
                                  f'absolute={self._absolute_mode}')

    def note_status(self, state: Optional[str], mpos: Optional[Dict[str, float]]):
        """
        Called from the FluidNC read thread for every status report.
        Compares reported MPos with expected, detects discontinuities,
        logs state transitions.
        """
        if mpos is None:
            return
        now = time.time()
        with self._lock:
            self._sample_seq += 1
            prev_reported = self._reported
            prev_state = self._reported_state

            self._reported = dict(mpos)
            self._reported_state = state

            # Initialize expected from first reported position if unknown.
            if self._expected is None:
                self._expected = dict(mpos)
                self._write_sample('INIT', state, mpos, '')
                return

            # State transition detection.
            event_msg = ''
            if state and state != prev_state:
                self._stats['state_transitions'] += 1
                event_msg = f'{prev_state}->{state}'
                self._write_event('STATE', event_msg)

                # Loud states get a WARNING in the main log.
                if state in ('Alarm', 'Door', 'Hold', 'Sleep', 'Check'):
                    self._stats['alarm_events'] += 1
                    self.logger.warning(
                        f"FluidNC state -> {state} "
                        f"(prev={prev_state}, mpos={self._fmt_pos(mpos)}, "
                        f"expected={self._fmt_pos(self._expected)}, "
                        f"last_cmd={self._last_cmd!r})")

                # End-of-motion check on Run -> Idle.
                if prev_state == 'Run' and state == 'Idle':
                    self._check_end_of_motion(mpos)

            # Idle-drift detection: state Idle, was Idle, no recent motion
            # command, but MPos changed.
            if (state == 'Idle' and prev_state == 'Idle'
                    and prev_reported is not None
                    and (now - self._last_motion_cmd_time) > COMMAND_SETTLE_S):
                drift = self._delta_xy(mpos, prev_reported)
                drift_z = abs(mpos.get('Z', 0) - prev_reported.get('Z', 0))
                if drift > IDLE_DRIFT_MM or drift_z > IDLE_DRIFT_MM:
                    self._stats['idle_drifts'] += 1
                    self.logger.warning(
                        f"Idle-drift detected: MPos moved "
                        f"{self._fmt_pos(prev_reported)} -> {self._fmt_pos(mpos)} "
                        f"(dxy={drift:.3f}mm, dz={drift_z:.3f}mm) "
                        f"with no command in flight — possible step loss, "
                        f"EMI, or external state change")
                    self._write_event('IDLE_DRIFT',
                                      f'dxy={drift:.3f},dz={drift_z:.3f}')

            self._write_sample('S', state, mpos, event_msg)
            self._prev_state = prev_state
            self._prev_reported = prev_reported

    def note_soft_reset(self):
        """Called when a soft reset (Ctrl-X) is sent. Position becomes unknown."""
        with self._lock:
            self._stats['soft_resets'] += 1
            self.logger.warning(
                f"Soft reset (Ctrl-X) sent — expected position cleared "
                f"(was {self._fmt_pos(self._expected)}, "
                f"reported {self._fmt_pos(self._reported)})")
            self._expected = None
            self._absolute_mode = True
            self._mm_mode = True
            self._write_event('SOFT_RESET', '')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _gnum(self, value: str) -> int:
        """Parse a G-word value to an integer (G01 -> 1, G90 -> 90)."""
        try:
            return int(float(value))
        except ValueError:
            return -1

    def _handle_homing(self, axes: str = ''):
        """$H homes all axes; $HY (etc.) only the listed ones."""
        self._stats['homings'] += 1
        homed = [a for a in axes if a in self.DEFAULT_HOME_MPOS]
        if not homed or self._expected is None:
            self._expected = dict(self.DEFAULT_HOME_MPOS)
        else:
            for axis in homed:
                self._expected[axis] = self.DEFAULT_HOME_MPOS[axis]
        self.logger.info(
            f"$H homing — expected MPos reset to "
            f"{self._fmt_pos(self._expected)}")

    def _handle_motion_words(self, words, jog_local_mode: bool):
        """Update expected position from a motion command's words."""
        if self._expected is None:
            # No baseline yet — can't compute relative moves.
            return

        # Detect local G90/G91 inside this command (e.g. "$J=G91 X10").
        local_absolute = self._absolute_mode
        for letter, value in words:
            if letter == 'G':
                gnum = self._gnum(value)
                if gnum == 90:
                    local_absolute = True
                elif gnum == 91:
                    local_absolute = False

        for letter, value in words:
            if letter not in ('X', 'Y', 'Z'):
                continue
            try:
                v = float(value)
            except ValueError:
                continue
            if local_absolute:
                self._expected[letter] = v
            else:
                self._expected[letter] = self._expected.get(letter, 0.0) + v

        # For arcs (G2/G3) the endpoint is X/Y as parsed above; I/J are
        # offsets to the center and don't change the endpoint, so we
        # don't need special handling for them here.

    def _check_end_of_motion(self, mpos: Dict[str, float]):
        """At Run->Idle, expected should equal reported within tolerance."""
        if self._expected is None:
            return
        delta_xy = self._delta_xy(mpos, self._expected)
        delta_z = abs(mpos.get('Z', 0) - self._expected.get('Z', 0))
        if delta_xy > END_OF_MOTION_MISMATCH_MM or delta_z > END_OF_MOTION_MISMATCH_MM:
            self._stats['end_of_motion_mismatches'] += 1
            self.logger.warning(
                f"End-of-motion mismatch: expected "
                f"{self._fmt_pos(self._expected)}, reported "
                f"{self._fmt_pos(mpos)} "
                f"(dxy={delta_xy:.3f}mm, dz={delta_z:.3f}mm), "
                f"last_cmd={self._last_cmd!r}")
            self._write_event('END_MISMATCH',
                              f'dxy={delta_xy:.3f},dz={delta_z:.3f}')

    @staticmethod
    def _delta_xy(a: Dict[str, float], b: Dict[str, float]) -> float:
        dx = a.get('X', 0.0) - b.get('X', 0.0)
        dy = a.get('Y', 0.0) - b.get('Y', 0.0)
        return (dx * dx + dy * dy) ** 0.5

    @staticmethod
    def _fmt_pos(p: Optional[Dict[str, float]]) -> str:
        if p is None:
            return '?'
        return f"({p.get('X', 0):.3f},{p.get('Y', 0):.3f},{p.get('Z', 0):.3f})"

    def _write_sample(self, kind: str, state: Optional[str],
                      mpos: Dict[str, float], event: str):
        """Write a status-sample row. Caller must hold _lock."""
        if not self._trace_fh:
            return
        delta_xy = ''
        delta_z = ''
        if self._expected is not None:
            delta_xy = f'{self._delta_xy(mpos, self._expected):.3f}'
            delta_z = f'{abs(mpos.get("Z", 0) - self._expected.get("Z", 0)):.3f}'
        self._write_row(kind, state, mpos, self._expected, delta_xy, delta_z, event)

    def _write_event(self, kind: str, event: str):
        """Write an event row (no fresh sample). Caller must hold _lock."""
        if not self._trace_fh:
            return
        self._write_row(kind, self._reported_state, self._reported,
                        self._expected, '', '', event)

    def _write_row(self, kind, state, mpos, expected,
                   delta_xy, delta_z, event):
        try:
            iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime()) \
                + f'.{int((time.time() % 1) * 1000):03d}'
            mono = f'{time.monotonic():.3f}'
            mx = f'{mpos.get("X", 0):.3f}' if mpos else ''
            my = f'{mpos.get("Y", 0):.3f}' if mpos else ''
            mz = f'{mpos.get("Z", 0):.3f}' if mpos else ''
            ex = f'{expected.get("X", 0):.3f}' if expected else ''
            ey = f'{expected.get("Y", 0):.3f}' if expected else ''
            ez = f'{expected.get("Z", 0):.3f}' if expected else ''
            # Quote the last_cmd field — it can contain commas.
            last_cmd = self._last_cmd.replace('"', "'")
            self._trace_fh.write(
                f'{iso},{mono},{self._sample_seq},{kind},{state or ""},'
                f'{mx},{my},{mz},{ex},{ey},{ez},'
                f'{delta_xy},{delta_z},{event},"{last_cmd}"\n'
            )
        except Exception as e:
            # Don't let trace failures kill the read thread.
            self.logger.debug(f"trace write failed: {e}")
