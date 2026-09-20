"""Machine operations for software that drives the plotter directly.

This is the layer under the remote API: homing, moves, pen handling, the pen
changer and the gantry camera, in machine coordinates and with the same safety
rules the rest of the daemon uses (limits are checked, a rejected move raises,
and a move only counts as done once FluidNC has actually executed it).

Canvas coordinates are available when the console operator has set the canvas
corners, but software in control normally works in machine coordinates and
finds the paper itself.
"""

import logging
import os
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple


class MachineError(Exception):
    """Raised when the machine cannot carry out a request."""


class MachineController:
    """Motion, pen and camera operations for external software."""

    def __init__(self, state_machine, config: Dict):
        self.state_machine = state_machine
        self.fluidnc = state_machine.fluidnc
        self.config = config
        self.logger = logging.getLogger(__name__)
        self._lock = threading.Lock()   # one caller at a time moves the machine
        self._pen_index: Optional[int] = None
        self._pen_is_down = False

        machine = config.get('machine', {})
        self.pen_z = float(machine.get('pen_z_default', 0.5))
        self.draw_feedrate = int(machine.get('draw_feedrate', 3000))
        self.rapid_feedrate = int(machine.get('jog_feedrate', 6000))

        tool = config.get('toolchanger', {})
        self.slot_count = int(tool.get('slots', 5))
        self.slot_first_x = float(tool.get('first_x', 700.0))
        self.slot_spacing = float(tool.get('spacing', 34.0))
        self.slot_y = float(tool.get('y', 12.0))
        self.slot_z = float(tool.get('z', 13.0))
        self.slot_y_safe = float(tool.get('y_safe', 55.0))
        self.slot_feedrate = int(tool.get('approach_feedrate', 1000))

        camera = config.get('camera', {})
        self.camera_device = camera.get('device', 'auto')
        self.photo_dir = camera.get('photo_dir', '/var/log/alphapaint-photos')
        self.camera_warmup_frames = int(camera.get('warmup_frames', 4))
        # v4l2 controls applied before every shot. Auto exposure blows out a
        # white sheet completely (mean grey 255), which hides pen lines.
        self.camera_controls = camera.get('controls') or {}

    # ---------------------------------------------------------------- status

    def status(self) -> Dict:
        sm = self.state_machine
        status = self.fluidnc.get_cached_status(max_age=1.0) or self.fluidnc.get_status()
        position = status.get('position') if status else None
        return {
            'state': sm.state,
            'machine_state': status.get('state') if status else None,
            'homed': sm.homed,
            'position': position,
            'limits': {axis: list(values) for axis, values in sm.machine_limits.items()},
            'pen': {'index': self._pen_index, 'down': self._pen_is_down, 'z': self.pen_z},
            'canvas': ({'origin': list(sm.point_B), 'corner': list(sm.point_C)}
                       if sm.point_B and sm.point_C else None),
            'camera': {'device': self.find_camera(), 'photo_dir': self.photo_dir},
        }

    def require_homed(self):
        if not self.state_machine.homed:
            raise MachineError("machine is not homed")

    # ---------------------------------------------------------------- motion

    def home(self) -> Dict:
        """Run a full homing cycle and wait for the result."""
        with self._lock:
            self.state_machine.request_homing()
            deadline = time.time() + 180
            while time.time() < deadline:
                state = self.state_machine.state
                if state in (self.state_machine.STATE_CANVAS_SETUP,
                             self.state_machine.STATE_READY,
                             self.state_machine.STATE_REMOTE):
                    self._pen_is_down = False
                    return self.status()
                if state == self.state_machine.STATE_ERROR:
                    raise MachineError("homing failed, see the log")
                time.sleep(0.2)
            raise MachineError("homing did not finish in time")

    def _send(self, gcode: str, timeout: float = 60.0):
        if not self.fluidnc.send_gcode(gcode, wait_ok=True, timeout=timeout):
            raise MachineError(f"FluidNC rejected or did not acknowledge: {gcode}")

    def _validate(self, x=None, y=None, z=None):
        limits = self.state_machine.machine_limits
        for axis, value in (('X', x), ('Y', y), ('Z', z)):
            if value is None:
                continue
            low, high = limits[axis]
            if not (low <= value <= high):
                raise MachineError(f"{axis}={value} is outside the machine limits "
                                   f"[{low}, {high}]")

    def sync(self) -> Dict[str, float]:
        """Wait until everything sent so far has really been executed."""
        self._send("G4 P0", timeout=120)
        status = self.fluidnc.get_status()
        if not status or 'position' not in status:
            raise MachineError("no position from FluidNC")
        return status['position']

    def move_to(self, x=None, y=None, z=None, feed: Optional[int] = None,
                draw: bool = False, wait: bool = True) -> Dict:
        """Move in machine coordinates. draw=True uses G1 at the feed rate."""
        self.require_homed()
        self._validate(x, y, z)
        if x is None and y is None and z is None:
            raise MachineError("no target given")
        with self._lock:
            parts = ["G1" if draw else "G0"]
            if x is not None:
                parts.append(f"X{x:.3f}")
            if y is not None:
                parts.append(f"Y{y:.3f}")
            if z is not None:
                parts.append(f"Z{z:.3f}")
            if draw:
                parts.append(f"F{feed or self.draw_feedrate}")
            self._send(" ".join(parts))
            if not wait:
                return {'queued': True}
            position = self.sync()
            return {'position': position}

    def stop(self) -> Dict:
        """Stop motion without losing position and drop anything queued."""
        held = self.fluidnc.stop_motion()
        self._pen_is_down = False
        return {'controlled_stop': held}

    # ------------------------------------------------------------------- pen

    def pen_up(self, wait: bool = True) -> Dict:
        z = self.state_machine.machine_limits['Z'][1]
        result = self.move_to(z=z, wait=wait)
        self._pen_is_down = False
        return result

    def pen_down(self, z: Optional[float] = None, feed: Optional[int] = None) -> Dict:
        """Lower the pen to its drawing height (or an explicit Z)."""
        target = self.pen_z if z is None else float(z)
        result = self.move_to(z=target, feed=feed or self.config['machine']['pen_lift_feedrate'],
                              draw=True)
        self._pen_is_down = True
        return result

    def set_pen_z(self, z: float) -> Dict:
        """Set the height at which the pen touches the paper."""
        self._validate(z=z)
        self.pen_z = float(z)
        return {'pen_z': self.pen_z}

    # ------------------------------------------------------------ toolchanger

    def slot_position(self, index: int) -> Tuple[float, float, float]:
        if not 0 <= index < self.slot_count:
            raise MachineError(f"pen index {index} outside 0..{self.slot_count - 1}")
        return (self.slot_first_x + index * self.slot_spacing, self.slot_y, self.slot_z)

    def _check_at(self, x: float, y: float, z: float, tolerance: float = 0.5):
        position = self.sync()
        for axis, expected in (('X', x), ('Y', y), ('Z', z)):
            if abs(position[axis] - expected) > tolerance:
                raise MachineError(
                    f"machine is at {axis}={position[axis]:.2f} instead of {expected:.2f}; "
                    f"pen change aborted to avoid a crash")

    def pickup_pen(self, index: int) -> Dict:
        """Take a pen out of its slot."""
        self.require_homed()
        pen_x, pen_y, pen_z = self.slot_position(index)
        z_max = self.state_machine.machine_limits['Z'][1]
        self.pen_up()
        self.move_to(y=pen_y + self.slot_y_safe)
        self.move_to(x=pen_x, y=pen_y + self.slot_y_safe, z=pen_z)
        self._check_at(pen_x, pen_y + self.slot_y_safe, pen_z)
        self.move_to(y=pen_y, feed=self.slot_feedrate, draw=True)   # magnet clicks on
        self.move_to(z=z_max)
        self.move_to(y=pen_y + self.slot_y_safe)
        self._pen_index = index
        self._pen_is_down = False
        return {'pen': index}

    def return_pen(self, index: Optional[int] = None) -> Dict:
        """Put the pen back in its slot."""
        self.require_homed()
        index = self._pen_index if index is None else index
        if index is None:
            raise MachineError("no pen is held")
        pen_x, pen_y, pen_z = self.slot_position(index)
        z_max = self.state_machine.machine_limits['Z'][1]
        self.pen_up()
        self.move_to(x=pen_x, y=pen_y + self.slot_y_safe, z=z_max)
        self._check_at(pen_x, pen_y + self.slot_y_safe, z_max)
        self.move_to(y=pen_y, feed=self.slot_feedrate, draw=True)
        self.move_to(z=pen_z, feed=self.slot_feedrate, draw=True)
        self.move_to(z=0, feed=self.slot_feedrate, draw=True)       # release
        self.move_to(y=pen_y + self.slot_y_safe)
        self.move_to(z=pen_z)
        self._pen_index = None
        return {'pen': None}

    # ---------------------------------------------------------------- camera

    def find_camera(self) -> Optional[str]:
        """Return the USB camera device, or None when none is connected."""
        if self.camera_device != 'auto':
            return self.camera_device if os.path.exists(self.camera_device) else None
        try:
            listing = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True,
                                     text=True, timeout=5).stdout
        except Exception:
            listing = ''
        device = None
        usb_block = False
        for line in listing.splitlines():
            if line and not line.startswith('\t'):
                usb_block = 'usb' in line.lower()
            elif usb_block and line.strip().startswith('/dev/video'):
                device = line.strip()
                break
        return device

    def _apply_camera_controls(self, device: str):
        """Fix exposure and friends; auto exposure ruins pictures of paper."""
        if not self.camera_controls:
            return
        settings = ','.join(f"{key}={value}" for key, value in self.camera_controls.items())
        try:
            subprocess.run(['v4l2-ctl', '-d', device, '-c', settings],
                           capture_output=True, text=True, timeout=5)
        except Exception as e:
            self.logger.debug(f"Could not set camera controls: {e}")

    def photo(self, name: Optional[str] = None) -> Dict:
        """Take a picture with the gantry camera and return its path.

        Uses ffmpeg so the daemon needs no image libraries of its own; the
        analysis scripts read the file afterwards.
        """
        device = self.find_camera()
        if not device:
            raise MachineError("no USB camera found (is it plugged in?)")
        self._apply_camera_controls(device)
        os.makedirs(self.photo_dir, exist_ok=True)
        name = name or time.strftime('photo-%Y%m%d-%H%M%S')
        path = os.path.join(self.photo_dir, f"{name}.jpg")
        position = self.fluidnc.get_cached_status(max_age=1.0)
        command = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'v4l2',
                   '-i', device, '-frames:v', '1', path]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=20)
        except Exception as e:
            raise MachineError(f"camera capture failed: {e}")
        if result.returncode != 0 or not os.path.exists(path):
            raise MachineError(f"camera capture failed: {result.stderr.strip()[:200]}")
        return {'path': path, 'device': device,
                'position': position.get('position') if position else None}
