"""FluidNC Handler for CNC controller communication."""

import serial
import threading
import logging
import time
import re
import collections
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple, Callable


@dataclass
class PendingCommand:
    """Tracks a command awaiting response from FluidNC."""
    gcode: str
    cmd_len: int
    response_event: threading.Event = field(default_factory=threading.Event)
    response: Optional[str] = None
    timestamp: float = field(default_factory=time.time)
    sent_timestamp: float = 0.0  # When bytes actually went on the wire


class FluidNCHandler:
    """Handles communication with FluidNC CNC controller."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize FluidNC handler.

        Args:
            port: Serial port path (e.g., /dev/ttyUSB1)
            baudrate: Serial baudrate (default: 115200)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.logger = logging.getLogger(__name__)
        self._serial_lock = threading.Lock()  # Protects all serial operations

        # Background read thread
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._status_callback: Optional[Callable[[Dict], None]] = None

        # Per-command response tracking (replaces shared queue)
        self._pending_lock = threading.Lock()
        self._pending_deque: collections.deque = collections.deque()
        # Count of late "ok" responses expected after lost-ok recovery.
        # When we synthesize an "ok", the real one may still arrive later.
        # This counter tells _deliver_response to discard those late arrivals
        # instead of delivering them to the next command in the FIFO.
        self._late_ok_expected = 0

        # Buffer management for flow control (character-counting protocol)
        self._buffer_size = 128  # FluidNC/Grbl RX buffer size
        self._buffer_used = 0  # Bytes currently in buffer
        self._buffer_lock = threading.Lock()
        self._buffer_space_event = threading.Event()
        self._buffer_space_event.set()  # Initially have space

        # Status request mechanism for thread-safe get_status/get_limits
        self._status_request = threading.Event()
        self._status_response: Optional[Dict] = None
        self._status_ready = threading.Event()
        self._limits_request = threading.Event()
        self._limits_response: Optional[Dict] = None
        self._limits_ready = threading.Event()

        # Cached position from auto-report status updates
        self._cached_position: Optional[Dict[str, float]] = None
        self._cached_position_time: float = 0.0
        self._cached_state: Optional[str] = None

        # Instrumentation counters
        self._stats = {
            'ok_delivered': 0,
            'ok_orphaned': 0,
            'ok_late_discarded': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'status_queries_explicit': 0,
            'buffer_desyncs': 0,
            'command_timeouts': 0,
            'buffer_timeout_leaks_fixed': 0,
            'ok_recovered': 0,
            'max_buffer_drift': 0,      # Largest single correction by auto-report
        }
        # Snapshot of stats at last periodic log, for delta reporting
        self._stats_snapshot: Dict[str, int] = {}

    def connect(self) -> bool:
        """
        Connect to FluidNC serial port.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Wait for FluidNC to initialize
            self.logger.info(f"Connected to FluidNC on {self.port}")

            # Full reset sequence to clear any stuck state
            self.serial.write(b'\x18')  # Ctrl-X soft reset
            self.logger.info("Soft reset sent to clear any stuck state")
            time.sleep(2.0)  # Wait for reset to complete
            self.serial.reset_input_buffer()  # Clear startup messages

            # Unlock if in ALARM state (e.g. after previous crash)
            self.serial.write(b'$X\n')
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.logger.info("Alarm unlock ($X) sent")

            # Disable auto-report to prevent flooding
            self.serial.write(b'$Report/Interval=0\n')
            time.sleep(0.1)
            self.serial.reset_input_buffer()

            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to FluidNC: {e}")
            return False

    def disconnect(self):
        """Disconnect from FluidNC."""
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.logger.info("Disconnected from FluidNC")

    def start(self):
        """Start background read thread for continuous status updates."""
        if self._running:
            return

        # Reset buffer tracking
        with self._buffer_lock:
            self._buffer_used = 0
            self._buffer_space_event.set()
            self.logger.info(f"Buffer tracking reset: 0/{self._buffer_size} bytes")

        # Clear pending commands
        with self._pending_lock:
            old_count = len(self._pending_deque)
            self._pending_deque.clear()
            self._late_ok_expected = 0
            if old_count > 0:
                self.logger.warning(f"Cleared {old_count} pending commands on start")

        # Reset cached position
        self._cached_position = None
        self._cached_position_time = 0.0
        self._cached_state = None

        # Reset status request state
        self._status_request.clear()
        self._status_ready.clear()
        self._limits_request.clear()
        self._limits_ready.clear()

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        self.logger.info("FluidNC read thread started")

    def stop(self):
        """Stop background read thread and clean up pending commands."""
        if not self._running:
            return

        self._running = False

        # Wake up any waiting status/limits requests
        self._status_ready.set()
        self._limits_ready.set()

        # Wake up all pending command waiters with shutdown error
        with self._pending_lock:
            for pending in self._pending_deque:
                pending.response = "error:shutdown"
                pending.response_event.set()
            self._pending_deque.clear()
            self._late_ok_expected = 0

        # Wake up buffer waiters
        self._buffer_space_event.set()

        if self._read_thread:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        self.logger.info("FluidNC read thread stopped")

    def on_status(self, callback: Callable[[Dict], None]):
        """
        Register callback for status updates.

        Args:
            callback: Function receiving status dict with 'state' and 'position' keys
        """
        self._status_callback = callback

    def enable_auto_report(self, interval_ms: int = 100) -> bool:
        """
        Enable automatic status reporting from FluidNC.

        Args:
            interval_ms: Report interval in milliseconds

        Returns:
            True if command sent successfully
        """
        # Configure status report to include buffer data (Bf:)
        # $10=3 means: MPos (1) + Buffer data (2) = 3
        self.send_gcode("$10=3", wait_ok=False)
        time.sleep(0.05)
        return self.send_gcode(f"$Report/Interval={interval_ms}", wait_ok=False)

    def _read_loop(self):
        """Background thread that reads from FluidNC and dispatches messages."""
        last_stale_check = time.time()
        last_buffer_log = time.time()
        STALE_CHECK_INTERVAL = 5.0  # Check every 5 seconds
        STALE_TIMEOUT = 30.0  # Commands older than 30 seconds are stale
        BUFFER_LOG_INTERVAL = 10.0  # Log buffer state every 10 seconds

        while self._running:
            try:
                # Check for status request from main thread
                if self._status_request.is_set():
                    self._handle_status_request()
                    continue

                # Check for limits request from main thread
                if self._limits_request.is_set():
                    self._handle_limits_request()
                    continue

                # Normal read loop
                if self.serial and self.serial.is_open and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.logger.debug(f"FluidNC RX: '{line}' (len={len(line)})")
                        self._dispatch_message(line)
                else:
                    time.sleep(0.01)

                # Periodic stale command check
                if time.time() - last_stale_check > STALE_CHECK_INTERVAL:
                    last_stale_check = time.time()
                    self._check_stale_commands(STALE_TIMEOUT)

                # Periodic health reporting
                if time.time() - last_buffer_log > BUFFER_LOG_INTERVAL:
                    last_buffer_log = time.time()
                    self._log_periodic_health()

            except Exception as e:
                self.logger.error(f"Error in FluidNC read loop: {e}")
                time.sleep(0.1)

    def _log_periodic_health(self):
        """Log periodic health summary with delta reporting."""
        with self._buffer_lock:
            buffer_used = self._buffer_used
        with self._pending_lock:
            pending_count = len(self._pending_deque)
            late_ok = self._late_ok_expected
            now_health = time.time()
            pending_info = [
                f"{p.gcode[:20]}({now_health - p.sent_timestamp:.1f}s)"
                if p.sent_timestamp > 0 else p.gcode[:20]
                for p in self._pending_deque
            ]

        # Compute deltas since last report
        snap = self._stats_snapshot
        deltas = {}
        problems = []
        for key in ('ok_recovered', 'ok_late_discarded', 'ok_orphaned', 'command_timeouts'):
            cur = self._stats.get(key, 0)
            prev = snap.get(key, 0)
            d = cur - prev
            if d > 0:
                deltas[key] = d

        if deltas.get('ok_recovered', 0) > 0:
            problems.append(f"lost_oks={deltas['ok_recovered']}")
        if deltas.get('command_timeouts', 0) > 0:
            problems.append(f"timeouts={deltas['command_timeouts']}")
        if deltas.get('ok_orphaned', 0) > 0:
            problems.append(f"orphans={deltas['ok_orphaned']}")

        # Save snapshot for next delta
        self._stats_snapshot = dict(self._stats)

        # Log problems at WARNING, otherwise only when active
        if problems:
            self.logger.warning(
                f"Serial health: {', '.join(problems)} "
                f"(buf={buffer_used}/{self._buffer_size}, "
                f"pending={pending_count}: {pending_info}, late_ok_due={late_ok}, "
                f"totals: recovered={self._stats['ok_recovered']}, "
                f"late_discarded={self._stats['ok_late_discarded']}, "
                f"orphaned={self._stats['ok_orphaned']})")
        elif pending_count > 0 or buffer_used > 0:
            self.logger.info(
                f"Buf: {buffer_used}/{self._buffer_size}, "
                f"{pending_count} pending: {pending_info}, late_ok_due={late_ok}")

    def _drain_and_dispatch_unlocked(self):
        """Read and dispatch all currently buffered serial data.

        Routes 'ok' and 'error:' responses to _deliver_response() so they
        are not lost. Must be called with _serial_lock held.
        """
        drained = 0
        while self.serial and self.serial.in_waiting:
            line = self._readline_unlocked(timeout=0.05)
            if line:
                drained += 1
                self.logger.debug(f"Drained: {line}")
                self._dispatch_message(line)
            else:
                break
        if drained > 0:
            self.logger.debug(f"Drained {drained} lines from serial buffer")

    def _handle_status_request(self):
        """Handle a status query request from the main thread."""
        self._status_request.clear()

        with self._serial_lock:
            # Drain and dispatch buffered data (preserves ok responses)
            self._drain_and_dispatch_unlocked()

            if not self._send_unlocked("?"):
                self._status_response = None
                self._status_ready.set()
                return

            # Wait for status response
            start_time = time.time()
            while time.time() - start_time < 0.5:
                line = self._readline_unlocked(timeout=0.1)
                if not line:
                    continue
                # Route non-status responses properly
                if not line.startswith('<'):
                    self._dispatch_message(line)
                    continue
                if line.startswith('<'):
                    self._status_response = self._parse_status(line)
                    self._status_ready.set()
                    return

            self._status_response = None
            self._status_ready.set()

    def _handle_limits_request(self):
        """Handle a limits query request from the main thread."""
        self._limits_request.clear()

        # Default limits
        limits = {
            'X': (0.0, 300.0),
            'Y': (-150.0, 150.0),
            'Z': (0.0, 50.0)
        }

        with self._serial_lock:
            # Drain and dispatch buffered data (preserves ok responses)
            self._drain_and_dispatch_unlocked()

            if not self._send_unlocked("$$"):
                self._limits_response = limits
                self._limits_ready.set()
                return

            # Read settings until 'ok' or timeout
            start_time = time.time()
            settings = {}

            while time.time() - start_time < 5.0:
                line = self._readline_unlocked(timeout=0.1)
                if line:
                    self.logger.debug(f"get_limits received: {line}")
                    if line.startswith('ok'):
                        break
                    match = re.match(r'\$(\d+)=([\d.-]+)', line)
                    if match:
                        setting_num = int(match.group(1))
                        value = float(match.group(2))
                        settings[setting_num] = value

        # Extract limits from settings
        if 130 in settings:
            limits['X'] = (0.0, settings[130])
        if 131 in settings:
            limits['Y'] = (0.0, settings[131])
        if 132 in settings:
            limits['Z'] = (0.0, settings[132])

        self._limits_response = limits
        self._limits_ready.set()

    def _check_stale_commands(self, timeout: float):
        """Check for commands that have been pending too long."""
        now = time.time()
        with self._pending_lock:
            stale = []
            for pending in self._pending_deque:
                if (now - pending.timestamp) > timeout:
                    stale.append(pending)

            for pending in stale:
                self.logger.warning(f"Stale command timeout ({timeout}s): {pending.gcode}")

                # Update buffer tracking
                with self._buffer_lock:
                    self._buffer_used -= pending.cmd_len
                    if self._buffer_used < 0:
                        self._buffer_used = 0
                    if self._buffer_used < self._buffer_size - 50:
                        self._buffer_space_event.set()

                # Deliver timeout error to waiting caller
                pending.response = "error:stale_timeout"
                pending.response_event.set()
                self._pending_deque.remove(pending)

    def _dispatch_message(self, line: str):
        """
        Dispatch incoming message to appropriate handler.

        Args:
            line: Raw line from FluidNC
        """
        if line.startswith('<'):
            # Status report - call callback and sync buffer tracking
            self.logger.debug(f"Dispatch: status report")
            status = self._parse_status(line)
            if status:
                # Cache position from auto-report for non-blocking queries
                if 'position' in status:
                    self._cached_position = status['position']
                    self._cached_position_time = time.time()
                if 'state' in status:
                    self._cached_state = status['state']

                # Sync buffer tracking with real FluidNC buffer state
                if 'buffer' in status:
                    self._sync_buffer_from_status(status['buffer'])

                if self._status_callback:
                    try:
                        self._status_callback(status)
                    except Exception as e:
                        self.logger.error(f"Error in status callback: {e}")
        elif line.startswith('ok') or line.startswith('error:'):
            # Command acknowledged - deliver to waiting caller
            self.logger.debug(f"Dispatch: command response '{line}'")
            self._deliver_response(line)
        else:
            # Other messages (info, welcome, etc.)
            self.logger.debug(f"Dispatch: other message '{line}'")

    def _sync_buffer_from_status(self, buffer_info: Dict):
        """
        Snap buffer tracking to ground truth from auto-report.

        Every 100ms FluidNC tells us exactly how full its RX buffer is.
        We always trust that, adjusted only for bytes we sent so recently
        that they may not be reflected in this report yet (grace period).

        This makes sustained desync impossible — any drift from ok-based
        tracking between reports is corrected within one auto-report cycle.

        Also recovers lost "ok" responses: if FluidNC's buffer is empty
        but we have old pending commands, the "ok" was lost in serial
        transit (UART TX collision with auto-report status lines).

        Args:
            buffer_info: Dict with 'planner_available' and 'rx_available' keys
        """
        rx_available = buffer_info.get('rx_available', 0)
        real_buffer_used = self._buffer_size - rx_available

        now = time.time()
        GRACE_PERIOD = 0.15  # 150ms — generous for 115200 baud

        with self._pending_lock:
            # Bytes sent within the grace period: the auto-report was
            # generated by FluidNC before these bytes arrived, so
            # rx_available doesn't account for them. We must add them
            # back to avoid thinking there's more space than there is.
            grace_bytes = sum(
                p.cmd_len for p in self._pending_deque
                if p.sent_timestamp > 0 and (now - p.sent_timestamp) < GRACE_PERIOD
            )

            corrected = real_buffer_used + grace_bytes

            with self._buffer_lock:
                old_used = self._buffer_used
                self._buffer_used = max(0, min(corrected, self._buffer_size))
                drift = old_used - self._buffer_used  # positive = we were too high

                if drift != 0:
                    abs_drift = abs(drift)
                    if abs_drift > self._stats['max_buffer_drift']:
                        self._stats['max_buffer_drift'] = abs_drift
                    self.logger.debug(
                        f"Buffer sync: {old_used} -> {self._buffer_used} "
                        f"(real={real_buffer_used}, grace={grace_bytes})")

                if self._buffer_used < self._buffer_size - 50:
                    self._buffer_space_event.set()
                else:
                    self._buffer_space_event.clear()

            # Recover lost "ok" responses.
            #
            # In Grbl/FluidNC, "ok" means "I parsed this command into
            # my planner" — NOT "motion complete". If the planner buffer
            # is full (planner_available == 0), FluidNC delays the "ok"
            # until a planner slot opens — this is NORMAL and not a lost ok.
            #
            # We only recover when BOTH conditions are met:
            #   1. RX buffer is empty (command was read from serial)
            #   2. Planner has free slots (no reason for ok to be delayed)
            # If both are true and we still haven't received ok after 5s,
            # it was genuinely lost on the serial line.
            planner_available = buffer_info.get('planner_available', 0)
            LOST_OK_THRESHOLD = 5.0
            if rx_available >= self._buffer_size and planner_available > 0:
                while self._pending_deque:
                    oldest = self._pending_deque[0]
                    if oldest.sent_timestamp <= 0:
                        break
                    age = now - oldest.sent_timestamp
                    if age > LOST_OK_THRESHOLD:
                        stale = self._pending_deque.popleft()
                        self.logger.warning(
                            f"Recovering lost 'ok' for '{stale.gcode}' "
                            f"(age={age:.1f}s since send, FluidNC buffer empty, "
                            f"planner={planner_available})")
                        stale.response = 'ok'
                        stale.response_event.set()
                        self._stats['ok_recovered'] += 1
                        # The real "ok" may still arrive later. Track it
                        # so _deliver_response discards it instead of
                        # misdelivering to the next command in the FIFO.
                        self._late_ok_expected += 1
                    else:
                        break

    def _deliver_response(self, response: str):
        """
        Deliver response to the oldest pending command (FIFO order).
        Also updates buffer tracking.

        Args:
            response: The 'ok' or 'error:...' response from FluidNC
        """
        with self._pending_lock:
            # Check if this is a late "ok" from a command we already
            # recovered via synthetic ok. Discard it to prevent FIFO
            # mismatch — the buffer accounting was already handled during
            # recovery.
            if self._late_ok_expected > 0 and response.startswith('ok'):
                self._late_ok_expected -= 1
                self._stats['ok_late_discarded'] += 1
                self.logger.debug(f"Discarding late 'ok' (expected after recovery, "
                                 f"{self._late_ok_expected} more expected)")
                return

            if self._pending_deque:
                pending = self._pending_deque.popleft()
                cmd_len = pending.cmd_len
                cmd_gcode = pending.gcode

                # Update buffer tracking
                with self._buffer_lock:
                    old_used = self._buffer_used
                    self._buffer_used -= cmd_len
                    if self._buffer_used < 0:
                        self._buffer_used = 0  # Safety
                    self.logger.debug(f"Response '{response}' for cmd '{cmd_gcode}': "
                                    f"buffer {old_used} -> {self._buffer_used}/{self._buffer_size}")
                    if self._buffer_used < self._buffer_size - 50:
                        self._buffer_space_event.set()
                        self.logger.debug("Buffer space event set")

                # Deliver response to waiting caller
                pending.response = response
                pending.response_event.set()
                self._stats['ok_delivered'] += 1
                self.logger.debug(f"Response event set for cmd '{cmd_gcode}'")
            else:
                self._stats['ok_orphaned'] += 1
                self.logger.warning(f"Received response with no pending command: {response}")

    def _send_unlocked(self, command: str) -> bool:
        """
        Send raw command to FluidNC (internal, no lock).

        Args:
            command: Command string (will add newline if not present)

        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            self.logger.warning("Cannot send: FluidNC not connected")
            return False

        if not command.endswith('\n'):
            command += '\n'

        try:
            self.serial.write(command.encode('utf-8'))
            self.logger.debug(f"FluidNC TX: {command.strip()}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to send to FluidNC: {e}")
            return False

    def send(self, command: str) -> bool:
        """
        Send raw command to FluidNC (without waiting for response).
        Thread-safe.

        Args:
            command: Command string (will add newline if not present)

        Returns:
            True if sent successfully, False otherwise
        """
        with self._serial_lock:
            return self._send_unlocked(command)

    def send_gcode(self, gcode: str, wait_ok: bool = True, timeout: float = 10.0) -> bool:
        """
        Send G-code command with proper flow control.
        Thread-safe: uses per-command tracking to ensure correct response delivery.

        Args:
            gcode: G-code command
            wait_ok: Wait for 'ok' response before returning
            timeout: Timeout in seconds for waiting

        Returns:
            True if successful, False on error
        """
        # Calculate command length (including newline)
        cmd_with_newline = gcode if gcode.endswith('\n') else gcode + '\n'
        cmd_len = len(cmd_with_newline)

        # Create pending command tracker
        pending = PendingCommand(gcode=gcode, cmd_len=cmd_len)

        # Wait for buffer space if needed (flow control)
        # Read-only check: do NOT increment _buffer_used here.
        # The increment happens atomically with the serial write below.
        if self._running:
            start_time = time.time()
            wait_logged = False
            while True:
                with self._buffer_lock:
                    if self._buffer_used + cmd_len <= self._buffer_size:
                        break  # Space available - proceed to send
                    else:
                        if not wait_logged:
                            self.logger.warning(f"Buffer full ({self._buffer_used}/{self._buffer_size}), waiting to send: {gcode}")
                            wait_logged = True

                # No space - wait for acknowledgment
                if time.time() - start_time > timeout:
                    with self._buffer_lock:
                        self.logger.error(f"Timeout waiting for buffer space to send: {gcode} "
                                        f"(buffer={self._buffer_used}/{self._buffer_size})")
                    with self._pending_lock:
                        pending_cmds = [p.gcode for p in self._pending_deque]
                        self.logger.error(f"Pending commands at timeout: {pending_cmds}")
                    return False

                # Wait for buffer space event with short timeout
                self._buffer_space_event.wait(timeout=0.1)

        # Atomic: send command + update buffer tracking + register pending.
        # All under _serial_lock so auto-report sync cannot see an incremented
        # buffer before the command is actually on the wire.
        with self._serial_lock:
            if not self._send_unlocked(gcode):
                return False  # Nothing to clean up - no state modified yet

            # Command is on the wire - register in pending FIRST (so grace
            # period check in _sync_buffer_from_status can find it), then
            # increment buffer tracking.
            pending.sent_timestamp = time.time()
            with self._pending_lock:
                self._pending_deque.append(pending)
            with self._buffer_lock:
                old_used = self._buffer_used
                self._buffer_used += cmd_len
                self.logger.debug(f"Buffer space OK for '{gcode}': {old_used} + {cmd_len} = {self._buffer_used}/{self._buffer_size}")
                if self._buffer_used >= self._buffer_size - 50:
                    self._buffer_space_event.clear()

        if not wait_ok:
            return True

        # Wait for response on this specific command.
        #
        # FluidNC delays "ok" while the planner is full — this is normal
        # for arc commands that decompose into many short segments.
        # We only time out if the machine is idle (nothing to process)
        # and we STILL haven't received ok.
        if self._running:
            deadline = time.time() + timeout
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                if pending.response_event.wait(timeout=min(remaining, 0.5)):
                    # Got a response
                    response = pending.response
                    if response and response.startswith('ok'):
                        return True
                    elif response and response.startswith('error:'):
                        self.logger.error(f"FluidNC error: {response}")
                        return False
                    else:
                        self.logger.error(f"Unexpected response to {gcode}: {response}")
                        return False

                # No response yet — if machine is actively running,
                # the ok is legitimately delayed (planner processing).
                # Reset deadline to avoid timing out during normal work.
                if self._cached_state == 'Run':
                    deadline = time.time() + timeout

            self.logger.error(f"Timeout waiting for response to: {gcode}")
            self._stats['command_timeouts'] += 1
            # Remove from pending and expect a late ok.
            # If we leave it, the next command's ok gets delivered to
            # this stale entry (FIFO), cascading into complete desync.
            with self._pending_lock:
                try:
                    self._pending_deque.remove(pending)
                    self._late_ok_expected += 1
                except ValueError:
                    pass  # Already removed by recovery
            return False
        else:
            # No background thread - read directly (no flow control)
            # Remove from pending deque since we're handling response directly
            with self._pending_lock:
                try:
                    self._pending_deque.remove(pending)
                except ValueError:
                    pass

            with self._serial_lock:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    response = self._readline_unlocked()
                    if response:
                        self.logger.debug(f"FluidNC RX: {response}")

                        if response.startswith('ok'):
                            return True
                        elif response.startswith('error:'):
                            self.logger.error(f"FluidNC error: {response}")
                            return False

                    time.sleep(0.01)

                self.logger.error(f"Timeout waiting for response to: {gcode}")
                return False

        return False

    def _readline_unlocked(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read a line from FluidNC (internal, no lock).

        Args:
            timeout: Override default timeout

        Returns:
            Line string (stripped) or None if nothing available
        """
        if not self.serial or not self.serial.is_open:
            return None

        old_timeout = self.serial.timeout
        if timeout is not None:
            self.serial.timeout = timeout

        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                return line if line else None
        except Exception as e:
            self.logger.error(f"Error reading from FluidNC: {e}")
        finally:
            if timeout is not None:
                self.serial.timeout = old_timeout

        return None

    def readline(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Read a line from FluidNC.
        Thread-safe.

        Args:
            timeout: Override default timeout

        Returns:
            Line string (stripped) or None if nothing available
        """
        with self._serial_lock:
            return self._readline_unlocked(timeout)

    def home(self) -> bool:
        """
        Execute homing sequence.

        Returns:
            True if homing successful, False otherwise
        """
        self.logger.info("Starting homing sequence")
        return self.send_gcode("$H", wait_ok=True, timeout=60.0)

    def get_limits(self) -> Dict[str, Tuple[float, float]]:
        """
        Query machine limits from FluidNC settings.
        Thread-safe: uses read thread when running, direct read otherwise.

        Returns:
            Dictionary with axis limits: {'X': (min, max), ...}
        """
        # Default limits
        default_limits = {
            'X': (0.0, 300.0),
            'Y': (-150.0, 150.0),
            'Z': (0.0, 50.0)
        }

        if self._running:
            # Use the read thread to handle the query (avoids serial race)
            self._limits_response = None
            self._limits_ready.clear()
            self._limits_request.set()

            # Wait for response from read thread
            if self._limits_ready.wait(timeout=6.0):
                result = self._limits_response if self._limits_response else default_limits
                self.logger.info(f"Machine limits: {result}")
                return result
            else:
                self.logger.warning("get_limits: Timeout waiting for read thread")
                return default_limits
        else:
            # Direct read when no background thread
            with self._serial_lock:
                # Wait for FluidNC to settle after homing
                time.sleep(0.5)

                if self.serial:
                    self.serial.reset_input_buffer()

                # Retry mechanism for reliability
                settings = {}
                for attempt in range(3):
                    if not self._send_unlocked("$$"):
                        time.sleep(0.3)
                        continue

                    start_time = time.time()

                    while time.time() - start_time < 5.0:
                        line = self._readline_unlocked(timeout=0.1)
                        if line:
                            self.logger.debug(f"get_limits received: {line}")
                            if line.startswith('ok'):
                                break
                            match = re.match(r'\$(\d+)=([\d.-]+)', line)
                            if match:
                                setting_num = int(match.group(1))
                                value = float(match.group(2))
                                settings[setting_num] = value

                    if settings:
                        break  # Got settings, exit retry loop

                    self.logger.warning(f"get_limits attempt {attempt + 1}/3: No settings received, retrying...")
                    time.sleep(0.5)
                    if self.serial:
                        self.serial.reset_input_buffer()

                if not settings:
                    self.logger.warning("get_limits: No settings received from FluidNC after 3 attempts")

            # Extract limits from settings
            limits = dict(default_limits)
            if 130 in settings:
                limits['X'] = (0.0, settings[130])
            if 131 in settings:
                limits['Y'] = (0.0, settings[131])
            if 132 in settings:
                limits['Z'] = (0.0, settings[132])

            self.logger.info(f"Machine limits: {limits}")
            return limits

    def get_cached_status(self, max_age: float = 0.5) -> Optional[Dict]:
        """
        Return the most recent status from auto-report without querying FluidNC.

        This avoids disrupting the serial command/response stream.
        Returns None if no status has been received or if the cached data is
        older than max_age seconds.

        Args:
            max_age: Maximum age in seconds for cached data (default 0.5s)

        Returns:
            Dictionary with 'state' and 'position' keys, or None
        """
        if self._cached_position is None:
            self._stats['cache_misses'] += 1
            return None

        age = time.time() - self._cached_position_time
        if age > max_age:
            self._stats['cache_misses'] += 1
            return None

        self._stats['cache_hits'] += 1
        return {
            'state': self._cached_state,
            'position': dict(self._cached_position)
        }

    def get_stats(self) -> Dict[str, int]:
        """Return instrumentation counters for debugging."""
        return dict(self._stats)

    def get_status(self) -> Optional[Dict]:
        """
        Query current status and position.
        Thread-safe: uses read thread when running, direct read otherwise.

        Returns:
            Dictionary with state and position, or None on error
        """
        self._stats['status_queries_explicit'] += 1
        if self._running:
            # Use the read thread to handle the query (avoids serial race)
            self._status_response = None
            self._status_ready.clear()
            self._status_request.set()

            # Wait for response from read thread
            if self._status_ready.wait(timeout=1.0):
                return self._status_response
            else:
                self.logger.warning("get_status: Timeout waiting for read thread")
                return None
        else:
            # Direct read when no background thread
            with self._serial_lock:
                if self.serial:
                    self.serial.reset_input_buffer()

                if not self._send_unlocked("?"):
                    return None

                start_time = time.time()
                while time.time() - start_time < 0.2:
                    line = self._readline_unlocked(timeout=0.1)
                    if line and line.startswith('<'):
                        return self._parse_status(line)

                return None

    def _parse_status(self, status_line: str) -> Optional[Dict]:
        """
        Parse FluidNC status response.

        Example: <Idle|MPos:10.00,20.00,5.00|Bf:15,120|FS:0,0>

        Returns:
            Dictionary with 'state', 'position', and optionally 'buffer' keys
        """
        try:
            # Remove < and >
            status_line = status_line.strip('<>')

            # Split by |
            parts = status_line.split('|')

            result = {
                'state': parts[0],
                'position': {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
            }

            # Parse all fields
            for part in parts:
                if part.startswith('MPos:') or part.startswith('WPos:'):
                    coords = part.split(':')[1].split(',')
                    if len(coords) >= 3:
                        result['position']['X'] = float(coords[0])
                        result['position']['Y'] = float(coords[1])
                        result['position']['Z'] = float(coords[2])
                elif part.startswith('Bf:'):
                    # Buffer status: Bf:planner_blocks,rx_chars_available
                    bf_values = part.split(':')[1].split(',')
                    if len(bf_values) >= 2:
                        result['buffer'] = {
                            'planner_available': int(bf_values[0]),
                            'rx_available': int(bf_values[1])
                        }

            return result

        except Exception as e:
            self.logger.error(f"Error parsing status: {e}")
            return None

    def jog(self, axis: str, position: float, feedrate: int = 1000) -> bool:
        """
        Send jog command (absolute positioning).

        Args:
            axis: Axis to jog (X, Y, or Z)
            position: Target position in mm
            feedrate: Jog speed in mm/min

        Returns:
            True if command sent successfully
        """
        command = f"$J=G90 G21 {axis}{position:.2f} F{feedrate}"
        # Use send_gcode with wait_ok=False for proper buffer tracking
        # without blocking on response (jog needs to be fast)
        return self.send_gcode(command, wait_ok=False, timeout=1.0)

    def cancel_jog(self) -> bool:
        """
        Cancel any active jog command.
        Thread-safe.

        Returns:
            True if cancel sent successfully
        """
        with self._serial_lock:
            # Send Jog Cancel character (0x85)
            try:
                if self.serial and self.serial.is_open:
                    self.serial.write(b'\x85')
                    self.logger.info("Jog cancelled")
                    return True
            except Exception as e:
                self.logger.error(f"Error cancelling jog: {e}")

            return False

    def soft_reset(self) -> bool:
        """
        Send soft reset (Ctrl-X) and clear all pending state.
        Thread-safe.

        Returns:
            True if reset sent successfully
        """
        with self._serial_lock:
            try:
                if self.serial and self.serial.is_open:
                    self.serial.write(b'\x18')
                    self.logger.info("Soft reset sent")
                    time.sleep(0.3)  # Wait for reset (FluidNC resets within ~100ms)

                    # Clear all pending commands with reset error
                    with self._pending_lock:
                        for pending in self._pending_deque:
                            pending.response = "error:reset"
                            pending.response_event.set()
                        self._pending_deque.clear()
                        self._late_ok_expected = 0

                    # Reset buffer tracking
                    with self._buffer_lock:
                        self._buffer_used = 0
                        self._buffer_space_event.set()

                    return True
            except Exception as e:
                self.logger.error(f"Error sending soft reset: {e}")

            return False

    def identify(self) -> bool:
        """
        Identify if this device is a FluidNC controller.
        Thread-safe.

        Returns:
            True if device is FluidNC, False otherwise
        """
        if not self.serial or not self.serial.is_open:
            return False

        with self._serial_lock:
            try:
                # Clear input buffer
                self.serial.reset_input_buffer()

                # Send version query
                self._send_unlocked("$I")

                # Wait for response - look for version info, consume until 'ok'
                start_time = time.time()
                found_fluidnc = False
                while time.time() - start_time < 2.0:
                    line = self._readline_unlocked(timeout=0.1)
                    if line:
                        self.logger.debug(f"FluidNC ID response: {line}")

                        # 'ok' marks end of $I response
                        if line.startswith('ok'):
                            break

                        # Look for FluidNC/Grbl version string
                        if '[VER:' in line or '[MSG:' in line or 'Grbl' in line:
                            found_fluidnc = True

                        # Also check for status response
                        if line.startswith('<'):
                            found_fluidnc = True

                if found_fluidnc:
                    return True

                # Try status query as alternative
                self._send_unlocked("?")
                time.sleep(0.5)

                while self.serial.in_waiting:
                    line = self._readline_unlocked(timeout=0.1)
                    if line and line.startswith('<'):
                        return True

                return False

            except Exception as e:
                self.logger.error(f"Error identifying FluidNC: {e}")
            return False

    def wait_idle(self, timeout: float = 60.0, poll_interval: float = 0.1) -> bool:
        """
        Wait until FluidNC is in Idle state (all motion complete).

        Args:
            timeout: Maximum time to wait in seconds
            poll_interval: Time between status checks in seconds

        Returns:
            True if Idle state reached, False on timeout
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            status = self.get_status()
            if status and status.get('state') == 'Idle':
                return True
            time.sleep(poll_interval)

        self.logger.warning(f"Timeout waiting for Idle state after {timeout}s")
        return False
