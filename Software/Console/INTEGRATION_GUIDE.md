# AlphaPaint Console - Integration Guide v1.1

## Document Purpose

This guide is for integrators connecting the AlphaPaint Console to a Raspberry Pi controller. It describes the physical connection, device identification, and the UART communication protocol.

**Version 1.1 adds:**
- NOT_HOMED state requirement
- LIMIT commands for axis boundaries
- Mandatory homing sequence before ACTIVE mode

---

## 1. Physical Connection

### 1.1 Hardware Setup

```
┌─────────────────┐         ┌──────────────────┐         ┌──────────────┐
│  AlphaPaint     │         │   Raspberry Pi   │         │   FluidNC    │
│   Console       │  USB    │   Controller     │  USB    │  Controller  │
│   (ESP32)       ├────────►│                  ├────────►│              │
└─────────────────┘         └──────────────────┘         └──────────────┘
```

### 1.2 USB Connection
- **Cable**: Standard USB Type-A to Micro-USB or USB-C (depending on ESP32 board)
- **Driver**: CP2102/CH340 USB-to-UART (automatic on modern Linux)
- **Power**: Console is powered via USB (5V, ~200mA typical)

### 1.3 Device Detection on Raspberry Pi

When connected, the console appears as a serial device:
```bash
# List USB serial devices
ls -l /dev/ttyUSB*
# or
ls -l /dev/ttyACM*

# View device information
udevadm info -a -n /dev/ttyUSB0 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}'
```

---

## 2. Device Identification

### 2.1 Automatic Identification

The console sends an identification string immediately after connecting or resetting:

```
CONSOLE:ALPHAPAINT:V1.1\n
STATUS:NOT_HOMED\n
```

**Characteristics:**
- Sent within 2 seconds of power-up
- Always the first messages from the console
- Single lines terminated with `\n`
- Version 1.1 indicates support for NOT_HOMED state and LIMIT commands

### 2.2 Identification Procedure

**Python Example:**
```python
import serial
import time

def identify_console(port, timeout=3):
    """
    Identify if a serial port is the AlphaPaint console.

    Returns: (is_console, version) tuple
    """
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)  # Wait for potential bootup message

        # Clear any existing data
        ser.reset_input_buffer()

        # Send identification query (optional)
        ser.write(b"ID?\n")

        # Wait for response
        start = time.time()
        while time.time() - start < timeout:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith('CONSOLE:ALPHAPAINT'):
                    version = line.split(':')[2] if len(line.split(':')) > 2 else 'UNKNOWN'
                    return (True, version)
        return (False, None)
    except Exception as e:
        print(f"Error checking {port}: {e}")
        return (False, None)

# Scan all serial ports
for port in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0']:
    is_console, version = identify_console(port)
    if is_console:
        print(f"AlphaPaint Console found on {port}, version {version}")
```

### 2.3 udev Rules (Linux)

Create persistent device naming using udev rules:

**File: `/etc/udev/rules.d/99-alphapaint.rules`**
```bash
# AlphaPaint Console - ESP32
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="alphapaint-console"

# FluidNC Controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="fluidnc-controller"
```

Then reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now access console via `/dev/alphapaint-console`.

---

## 3. UART Protocol Specification

### 3.1 Serial Port Settings

| Parameter | Value |
|-----------|-------|
| Baud Rate | 115200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| Line Ending | `\n` (LF) |

### 3.2 Message Format

**General Structure:**
```
<CATEGORY>:<PARAMETER1>:<PARAMETER2>:...\n
```

- All messages are ASCII text
- Fields separated by colon `:`
- Terminated with newline `\n`
- Case-sensitive
- Maximum message length: 64 characters

### 3.3 Message Categories

#### 3.3.1 Console → Pi (Outgoing)

| Category | Description |
|----------|-------------|
| `CONSOLE` | Identification and version |
| `STATUS` | Homing and operational status |
| `BTN` | Button press events |
| `AXIS` | Axis selection and mode changes |
| `POS` | Position updates (Active mode) |
| `LIMIT` | Limit query responses |
| `ERROR` | Error messages |

#### 3.3.2 Pi → Console (Incoming)

| Category | Description |
|----------|-------------|
| `MODE` | Set operating mode |
| `LED` | Control LED states |
| `POS` | Set position (Passive mode) |
| `LIMIT` | Set/query axis limits |
| `DISPLAY` | Display control |
| `ID` | Query identification |
| `STATUS` | Query status |

---

## 4. Command Reference

### 4.1 Console → Pi Commands

#### 4.1.1 Identification and Status
```
CONSOLE:ALPHAPAINT:V1.1      # Sent on startup
STATUS:NOT_HOMED             # Initial state after power-on
STATUS:HOMED                 # Sent after first POS command received
```

#### 4.1.2 Button Events

**Format:**
```
BTN:<button>:<action>
```

**Parameters:**
- `<button>`: `A`, `B`, `C`, `D`, `E`, `F`, `G`, `X`, `Y`, `Z`
- `<action>`: `SHORT`, `LONG`

**Examples:**
```
BTN:A:SHORT          # Button A short press
BTN:A:LONG           # Button A long press (≥500ms)
BTN:X:SHORT          # Button X short press (in Passive or NOT_HOMED)
BTN:Z:LONG           # Button Z long press
```

**Timing:**
- `SHORT`: Sent on button release if held < 500ms
- `LONG`: Sent immediately when 500ms threshold is reached (while still pressed)

#### 4.1.3 Axis Events (Active Mode - HOMED)

**Format:**
```
AXIS:<axis>:<event>:<value>
```

**Events:**

```
AXIS:X:SELECT               # X axis selected as active
AXIS:Y:SELECT               # Y axis selected
AXIS:Z:SELECT               # Z axis selected

AXIS:X:PRECISION:ON         # X axis precision mode enabled
AXIS:X:PRECISION:OFF        # X axis precision mode disabled
```

#### 4.1.4 Position Updates (Active Mode)

**Format:**
```
POS:<axis>:<value>
```

**Parameters:**
- `<axis>`: `X`, `Y`, `Z`
- `<value>`: Floating point with 2 decimal places, range -9999.99 to 9999.99

**Examples:**
```
POS:X:123.45            # X position is 123.45
POS:Y:-42.10            # Y position is -42.10
POS:Z:0.00              # Z position is 0.00
```

**Transmission:**
- Sent only when value changes
- Maximum rate: 20 Hz
- Always 2 decimal places
- Position clamped to limits if limit reached

#### 4.1.5 Limit Query Response

**Format:**
```
LIMIT:<axis>:<min>:<max>
```

**Examples:**
```
LIMIT:X:0.00:300.00         # Response to LIMIT:X? query
LIMIT:Y:-150.00:150.00
```

#### 4.1.6 Error Messages

**Format:**
```
ERROR:<type>:<details>
```

**Examples:**
```
ERROR:INVALID_CMD:LED:X:BLAH        # Invalid command received
ERROR:VALUE_RANGE:POS:X:99999       # Value out of range
ERROR:NOT_HOMED:MODE:ACTIVE         # Cannot enter ACTIVE when not homed
ERROR:INVALID_LIMIT:X:300:0         # Min > Max
ERROR:I2C_FAILURE                   # Hardware communication error
```

---

### 4.2 Pi → Console Commands

#### 4.2.1 Mode Control

**Set Operating Mode:**
```
MODE:<mode>
```

**Parameters:**
- `<mode>`: `ACTIVE` or `PASSIVE`

**Examples:**
```
MODE:ACTIVE             # Enable handwheel and local control (requires HOMED)
MODE:PASSIVE            # Disable handwheel, Pi controls positions
```

**Response:**
```
ERROR:NOT_HOMED:MODE:ACTIVE  # If attempting ACTIVE when NOT_HOMED
ERROR:INVALID_MODE:<mode>     # If mode is not ACTIVE or PASSIVE
```

**Note:** MODE commands do not send acknowledgment responses on success. The console silently changes mode. Use STATUS? query to verify current mode.

**Important:** MODE:ACTIVE is only allowed after console is HOMED (has received at least one POS command).

#### 4.2.2 LED Control

**Format:**
```
LED:<led>:<state>
```

**Parameters:**
- `<led>`: `A`, `B`, `C`, `D`, `E`, `F`, `G`
- `<state>`: `OFF`, `ON`, `BLINK`, `FAST_BLINK`

**Examples:**
```
LED:A:ON                # Turn LED A on
LED:B:OFF               # Turn LED B off
LED:C:BLINK             # LED C blink (300ms)
LED:D:FAST_BLINK        # LED D fast blink (150ms)
```

**Blink Rates:**
- `BLINK`: 500ms on/off cycle (slow blink)
- `FAST_BLINK`: 150ms on/off cycle (fast blink)

**Response:**
```
# No response on success - LED state changes silently
ERROR:INVALID_CMD:LED:<led>:<state>  # If command is invalid
```

**Note:** LED commands do not send acknowledgment responses on success.

#### 4.2.3 Position Control (Passive Mode)

**Format:**
```
POS:<axis>:<value>
```

**Parameters:**
- `<axis>`: `X`, `Y`, `Z`
- `<value>`: Floating point, -9999.99 to 9999.99

**Examples:**
```
POS:X:250.50            # Set X display to 250.50
POS:Y:-10.25            # Set Y display to -10.25
POS:Z:0.00              # Set Z display to 0.00
```

**Response:**
```
STATUS:HOMED            # Sent on first POS command if was NOT_HOMED
ERROR:MODE:ACTIVE       # If in ACTIVE mode (console controls position)
ERROR:VALUE_RANGE:POS:<axis>:<value>  # If value out of range
```

**Note:** POS commands do not send acknowledgment responses on success. Position updates are applied silently.

**Special Behavior:**
- **First POS command** after power-on transitions console from NOT_HOMED → HOMED state
- Displays stop blinking "------" and show numeric values
- After HOMED, MODE:ACTIVE becomes allowed

**Notes:**
- Works in both PASSIVE mode and NOT_HOMED state
- In ACTIVE mode, returns `ERROR:MODE:ACTIVE` (console controls position)

#### 4.2.4 Axis Limit Control

**Set Axis Limits:**
```
LIMIT:<axis>:<min>:<max>
```

**Parameters:**
- `<axis>`: `X`, `Y`, `Z`
- `<min>`: Minimum value (-9999.99 to 9999.99)
- `<max>`: Maximum value (-9999.99 to 9999.99)

**Examples:**
```
LIMIT:X:0.00:300.00          # X axis range: 0 to 300mm
LIMIT:Y:-150.00:150.00       # Y axis range: -150 to +150mm
LIMIT:Z:-50.00:0.00          # Z axis range: -50 to 0mm
```

**Response:**
```
ERROR:INVALID_LIMIT:<axis>:<min>:<max>  # If min > max
ERROR:VALUE_RANGE:LIMIT:<axis>:<value>  # If value out of display range
```

**Note:** LIMIT set commands do not send acknowledgment responses on success. Limits are applied silently.

**Notes:**
- Limits are enforced in ACTIVE mode during handwheel operation
- Position will not increment beyond min/max values
- Setting limits does not affect PASSIVE mode (Pi can display any value)
- Default limits on startup: -9999.99 to 9999.99 (full range)
- Limits persist until power cycle or new LIMIT command

**Query Current Limits:**
```
LIMIT:<axis>?
```

**Response:**
```
LIMIT:X:0.00:300.00
```

#### 4.2.5 Display Control

**Set Brightness:**
```
DISPLAY:BRIGHTNESS:<level>
```
- `<level>`: 0-7 (0=dim, 7=bright)

**Examples:**
```
DISPLAY:BRIGHTNESS:4    # Set medium brightness
```

**Response:**
```
# No response - brightness changes silently
```

**Note:** DISPLAY commands do not send acknowledgment responses.

#### 4.2.6 Query Commands

**Request Identification:**
```
ID?
```

**Response:**
```
CONSOLE:ALPHAPAINT:V1.1
```

**Request Current Status:**
```
STATUS?
```

**Response:**
```
STATUS:MODE:ACTIVE:AXIS:X:HOMED:POS:X:123.45:Y:-42.10:Z:0.00
# or
STATUS:MODE:PASSIVE:NOT_HOMED
```

---

## 5. Protocol Examples

### 5.1 Startup and Homing Sequence (MANDATORY)

```
# Console boots up in NOT_HOMED state
[Console] → CONSOLE:ALPHAPAINT:V1.1
[Console] → STATUS:NOT_HOMED

# All displays show blinking "------"
# Console is in PASSIVE mode and cannot switch to ACTIVE

# Pi attempts to set ACTIVE mode (will fail because NOT_HOMED)
[Pi] → MODE:ACTIVE
[Console] → ERROR:NOT_HOMED:MODE:ACTIVE

# ===== Pi performs homing sequence via FluidNC =====
# ... FluidNC homing process ...
# ... FluidNC reports final position ...

# Pi sends FIRST position update (this transitions to HOMED)
[Pi] → POS:X:0.00
[Console] → STATUS:HOMED          # Console is now homed!

# Displays stop blinking, show "0.00"

# Pi sends remaining axes
[Pi] → POS:Y:0.00
# (no response)

[Pi] → POS:Z:0.00
# (no response)

# Pi configures axis limits based on machine configuration
[Pi] → LIMIT:X:0.00:300.00
# (no response - applied silently)

[Pi] → LIMIT:Y:-150.00:150.00
# (no response)

[Pi] → LIMIT:Z:-50.00:0.00
# (no response)

# NOW ACTIVE mode is allowed
[Pi] → MODE:ACTIVE
# (no response - mode changed silently)

# User can now jog with handwheel (within limits)
```

**CRITICAL:** Pi must send at least one `POS` command before attempting `MODE:ACTIVE`. This ensures the machine has been homed and positions are known.

### 5.2 Jogging in Active Mode (with Limits)

```
# Console is in ACTIVE mode, X axis has limits 0.00 to 300.00

# User selects X axis
[Console] → AXIS:X:SELECT

# User rotates handwheel clockwise
[Console] → POS:X:0.00
[Console] → POS:X:1.00
[Console] → POS:X:2.00
...
[Console] → POS:X:298.00
[Console] → POS:X:299.00
[Console] → POS:X:300.00

# User continues rotating but position is clamped at limit
# No more POS messages sent (value didn't change)

# User rotates counter-clockwise
[Console] → POS:X:299.00
[Console] → POS:X:298.00
...

# User enables precision mode (long press X)
[Console] → AXIS:X:PRECISION:ON

# User rotates handwheel
[Console] → POS:X:298.01
[Console] → POS:X:298.02
...

# User disables precision mode (short press X)
[Console] → AXIS:X:PRECISION:OFF
```

### 5.3 CNC Operation in Passive Mode

```
# Pi switches to passive mode for CNC job
[Pi] → MODE:PASSIVE
# (no response - mode changed silently)

# Pi updates positions from FluidNC feedback during job
[Pi] → POS:X:125.50
# (no response - position updated silently)

[Pi] → POS:Y:89.23
# (no response)

[Pi] → POS:Z:-5.10
# (no response)

# User presses X button (has no effect on display, but sends event)
[Console] → BTN:X:SHORT

# Pi could respond by pausing job and entering active mode
[Pi] → MODE:ACTIVE
# (no response - mode changed silently)

# User can now jog manually
```

### 5.4 Limit Query Example

```
# Pi queries current limits
[Pi] → LIMIT:X?
[Console] → LIMIT:X:0.00:300.00

[Pi] → LIMIT:Y?
[Console] → LIMIT:Y:-150.00:150.00

[Pi] → LIMIT:Z?
[Console] → LIMIT:Z:-50.00:0.00
```

---

## 6. Python Integration Library

### 6.1 Basic Console Class

```python
import serial
import threading
import time
from typing import Callable, Optional

class AlphaPaintConsole:
    """Python interface for AlphaPaint Console v1.1."""

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.serial = None
        self.running = False
        self.callbacks = {}
        self.mode = "UNKNOWN"
        self.homed = False
        self.limits = {'X': (-9999.99, 9999.99), 
                       'Y': (-9999.99, 9999.99), 
                       'Z': (-9999.99, 9999.99)}

    def connect(self) -> bool:
        """Connect to console."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1
            )
            time.sleep(2)  # Wait for bootup
            self.running = True

            # Start receive thread
            self.rx_thread = threading.Thread(target=self._receive_loop)
            self.rx_thread.daemon = True
            self.rx_thread.start()

            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from console."""
        self.running = False
        if self.serial:
            self.serial.close()

    def _receive_loop(self):
        """Background thread to receive messages."""
        while self.running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8').strip()
                    self._process_message(line)
            except Exception as e:
                print(f"Receive error: {e}")
                time.sleep(0.1)

    def _process_message(self, msg: str):
        """Process incoming message."""
        print(f"RX: {msg}")

        parts = msg.split(':')
        if not parts:
            return

        category = parts[0]

        # Call registered callbacks
        if category in self.callbacks:
            self.callbacks[category](parts)

        # Handle specific messages
        if msg.startswith('CONSOLE:ALPHAPAINT'):
            print(f"Console identified: {msg}")
        elif msg.startswith('STATUS:NOT_HOMED'):
            self.homed = False
            print("Console is NOT_HOMED - displays show '------'")
        elif msg.startswith('STATUS:HOMED'):
            self.homed = True
            print("Console is now HOMED")
        elif msg.startswith('MODE:'):
            self.mode = parts[1]
        elif msg.startswith('BTN:'):
            button = parts[1]
            action = parts[2]
            self._on_button(button, action)
        elif msg.startswith('POS:'):
            axis = parts[1]
            value = float(parts[2])
            self._on_position(axis, value)
        elif msg.startswith('LIMIT:') and len(parts) == 4:
            axis = parts[1]
            min_val = float(parts[2])
            max_val = float(parts[3])
            self.limits[axis] = (min_val, max_val)
        elif msg.startswith('ERROR:'):
            print(f"ERROR from console: {msg}")

    def _send(self, cmd: str):
        """Send command to console."""
        if self.serial:
            msg = cmd + '\n'
            print(f"TX: {cmd}")
            self.serial.write(msg.encode('utf-8'))

    # Public API

    def set_mode(self, mode: str):
        """Set operating mode: ACTIVE or PASSIVE."""
        if mode == "ACTIVE" and not self.homed:
            print("WARNING: Console is NOT_HOMED, ACTIVE mode will be rejected")
        self._send(f"MODE:{mode}")

    def set_led(self, led: str, state: str):
        """
        Set LED state.
        led: A-G
        state: OFF, ON, BLINK, FAST_BLINK
        """
        self._send(f"LED:{led}:{state}")

    def set_position(self, axis: str, value: float):
        """Set position display (Passive mode or NOT_HOMED state)."""
        self._send(f"POS:{axis}:{value:.2f}")

    def set_limit(self, axis: str, min_val: float, max_val: float):
        """Set axis limits."""
        if min_val >= max_val:
            print(f"ERROR: Invalid limit range {min_val} >= {max_val}")
            return
        self._send(f"LIMIT:{axis}:{min_val:.2f}:{max_val:.2f}")

    def get_limit(self, axis: str):
        """Query axis limits."""
        self._send(f"LIMIT:{axis}?")

    def set_brightness(self, level: int):
        """Set display brightness (0-7)."""
        self._send(f"DISPLAY:BRIGHTNESS:{level}")

    def request_status(self):
        """Request current status."""
        self._send("STATUS?")

    def home_sequence(self, x: float, y: float, z: float):
        """
        Complete homing sequence.
        Call this after FluidNC homing is complete.
        """
        print("Starting homing sequence...")
        self.set_position('X', x)
        time.sleep(0.1)
        self.set_position('Y', y)
        time.sleep(0.1)
        self.set_position('Z', z)
        time.sleep(0.2)
        
        if self.homed:
            print("Console is now HOMED")
        else:
            print("WARNING: Console did not report HOMED status")

    def on(self, category: str, callback: Callable):
        """Register callback for message category."""
        self.callbacks[category] = callback

    def _on_button(self, button: str, action: str):
        """Override to handle button events."""
        print(f"Button {button} {action}")

    def _on_position(self, axis: str, value: float):
        """Override to handle position updates."""
        print(f"Position {axis} = {value:.2f}")


# Example usage
if __name__ == "__main__":
    console = AlphaPaintConsole("/dev/alphapaint-console")

    if console.connect():
        print("Connected to AlphaPaint Console")
        time.sleep(2)  # Wait for startup messages

        # Console starts in NOT_HOMED state
        # Perform homing sequence (simulated)
        print("\n=== SIMULATING HOMING ===")
        console.home_sequence(0.0, 0.0, 0.0)
        time.sleep(0.5)

        # Set machine limits
        print("\n=== SETTING LIMITS ===")
        console.set_limit("X", 0.0, 300.0)
        console.set_limit("Y", -150.0, 150.0)
        console.set_limit("Z", -50.0, 0.0)
        time.sleep(0.5)

        # Now can enter ACTIVE mode
        print("\n=== ENTERING ACTIVE MODE ===")
        console.set_mode("ACTIVE")
        time.sleep(0.5)

        # Control LEDs
        console.set_led("A", "ON")
        console.set_led("B", "BLINK")

        # Wait for user input
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

        console.disconnect()
```

### 6.2 Integration with FluidNC (Complete Example)

```python
import serial
import threading
import re
from alphapaint_console import AlphaPaintConsole

class PlotterController:
    """Main controller integrating Console and FluidNC."""

    def __init__(self, console_port: str, fluidnc_port: str):
        self.console = AlphaPaintConsole(console_port)
        self.fluidnc = serial.Serial(fluidnc_port, 115200)

        self.current_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        self.machine_limits = {'X': (0, 300), 'Y': (-150, 150), 'Z': (-50, 0)}
        self.jog_mode = False
        self.homing_complete = False

    def start(self):
        """Start controller."""
        # Connect to console
        if not self.console.connect():
            raise Exception("Failed to connect to console")

        # Override console callbacks
        self.console._on_button = self.handle_button
        self.console._on_position = self.handle_position

        # Console starts in NOT_HOMED state with "------" blinking
        # Cannot enter ACTIVE mode yet

        # Start FluidNC status polling
        self.start_fluidnc_polling()

    def perform_homing(self):
        """Perform homing sequence via FluidNC."""
        print("Starting FluidNC homing...")
        
        # Send home command to FluidNC
        self.fluidnc.write(b"$H\n")
        
        # Wait for homing to complete (monitor FluidNC responses)
        # ... parse FluidNC status messages ...
        
        # After homing, FluidNC reports position (usually 0,0,0 or machine max)
        home_x, home_y, home_z = 0.0, 0.0, 0.0  # Get from FluidNC
        
        # Send positions to console (this transitions from NOT_HOMED → HOMED)
        print("Sending home positions to console...")
        self.console.home_sequence(home_x, home_y, home_z)
        
        # Configure limits on console
        print("Setting console limits...")
        for axis, (min_val, max_val) in self.machine_limits.items():
            self.console.set_limit(axis, min_val, max_val)
        
        self.homing_complete = True
        self.current_pos = {'X': home_x, 'Y': home_y, 'Z': home_z}
        
        print("Homing complete! Console is now HOMED.")
        print("User can now switch to ACTIVE mode for jogging.")

    def handle_button(self, button: str, action: str):
        """Handle button press from console."""
        if button in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
            # Programmable buttons
            if action == 'SHORT':
                self.execute_macro(button)
            elif action == 'LONG':
                self.start_continuous_action(button)

        elif button in ['X', 'Y', 'Z']:
            # Axis buttons - toggle jog mode
            if action == 'SHORT':
                if not self.homing_complete:
                    print("Cannot jog - machine not homed yet")
                    self.console.set_led('A', 'FAST_BLINK')  # Warning
                else:
                    self.toggle_jog_mode()

    def handle_position(self, axis: str, value: float):
        """Handle position update from console (Active mode)."""
        if self.jog_mode:
            # Send jog command to FluidNC
            delta = value - self.current_pos[axis]
            if abs(delta) > 0.001:  # Threshold to avoid tiny movements
                self.jog_axis(axis, value)
                self.current_pos[axis] = value

    def jog_axis(self, axis: str, position: float):
        """Send jog command to FluidNC (absolute positioning)."""
        cmd = f"G90 G0 {axis}{position:.2f}\n"
        self.fluidnc.write(cmd.encode())
        print(f"Jogging {axis} to {position:.2f}")

    def toggle_jog_mode(self):
        """Toggle between jog and passive mode."""
        self.jog_mode = not self.jog_mode

        if self.jog_mode:
            self.console.set_mode("ACTIVE")
            self.console.set_led("A", "ON")  # Indicate jog mode
            print("JOG MODE: User can control with handwheel")
        else:
            self.console.set_mode("PASSIVE")
            self.console.set_led("A", "OFF")
            print("PASSIVE MODE: CNC operations only")

    def update_console_from_fluidnc(self, x: float, y: float, z: float):
        """Update console display from FluidNC position (Passive mode)."""
        if not self.jog_mode:  # Only update in passive mode
            self.console.set_position("X", x)
            self.console.set_position("Y", y)
            self.console.set_position("Z", z)
            self.current_pos = {'X': x, 'Y': y, 'Z': z}

    def execute_macro(self, button: str):
        """Execute macro for button A-G."""
        macros = {
            'A': self.perform_homing,  # Re-home
            'B': self.zero_xyz,
            'C': self.goto_origin,
            # Add more macros...
        }

        if button in macros:
            macros[button]()

    def zero_xyz(self):
        """Zero current position."""
        self.fluidnc.write(b"G92 X0 Y0 Z0\n")
        self.console.set_position('X', 0)
        self.console.set_position('Y', 0)
        self.console.set_position('Z', 0)
        self.current_pos = {'X': 0, 'Y': 0, 'Z': 0}

    def goto_origin(self):
        """Go to 0,0,0."""
        self.fluidnc.write(b"G90 G0 X0 Y0 Z0\n")


# Main application
if __name__ == "__main__":
    controller = PlotterController(
        console_port="/dev/alphapaint-console",
        fluidnc_port="/dev/fluidnc-controller"
    )
    
    controller.start()
    
    # MANDATORY: Home the machine before allowing user control
    print("\n" + "="*50)
    print("HOMING REQUIRED BEFORE OPERATION")
    print("="*50)
    input("Press Enter to start homing sequence...")
    
    controller.perform_homing()
    
    print("\n" + "="*50)
    print("SYSTEM READY")
    print("Console displays show position")
    print("Press axis button to enable jogging")
    print("="*50)
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        controller.console.disconnect()
```

---

## 7. Troubleshooting

### 7.1 Console Not Detected

**Problem:** Serial device not appearing

**Solutions:**
1. Check USB cable connection
2. Verify ESP32 is powered (LED on board)
3. Check driver installation: `lsusb` should show CP210x or CH340 device
4. Try different USB port
5. Check permissions: `sudo usermod -a -G dialout $USER` (logout/login required)

### 7.2 Stuck in NOT_HOMED State

**Problem:** Console displays show "------" blinking, cannot enter ACTIVE mode

**Solutions:**
1. Send at least one `POS` command: `POS:X:0.00`
2. Check that Pi has completed FluidNC homing
3. Verify UART communication is working (send `ID?`)
4. Check for `STATUS:HOMED` message after sending POS command
5. Reset console if needed (power cycle)

### 7.3 MODE:ACTIVE Rejected

**Problem:** Receive `ERROR:NOT_HOMED:MODE:ACTIVE`

**Solution:**
- Console must be HOMED first
- Send position updates via `POS` commands
- Wait for `STATUS:HOMED` response
- Then MODE:ACTIVE will work

### 7.4 Position Exceeds Limits

**Problem:** Handwheel stops moving but should go further

**Solutions:**
1. Check current limits: send `LIMIT:X?`
2. Adjust limits if needed: `LIMIT:X:0.00:500.00`
3. Limits only enforced in ACTIVE mode (expected behavior)
4. Verify position hasn't actually reached limit

### 7.5 Communication Errors

**Problem:** Garbled messages or no response

**Solutions:**
1. Verify baud rate is 115200
2. Check for other processes accessing the port: `lsof | grep ttyUSB`
3. Reset console (unplug/replug USB)
4. Check for EMI (move away from motors/power supplies)

---

## 8. Testing Procedure

### 8.1 Basic Connectivity Test

```bash
# 1. Connect console via USB
# 2. Identify serial port
ls -l /dev/ttyUSB*

# 3. Test with screen or minicom
screen /dev/ttyUSB0 115200

# 4. Look for identification messages
# Should see:
#   CONSOLE:ALPHAPAINT:V1.1
#   STATUS:NOT_HOMED

# 5. Send test commands
POS:X:0.00
# Should see: STATUS:HOMED (first POS command only)

MODE:ACTIVE
# (no response - mode changed silently, now that it's homed)

LIMIT:X:0:300
# (no response - limits applied silently)

LED:A:ON
LED:A:BLINK
LED:A:OFF
# (no responses - LED states change silently)

# Exit screen: Ctrl-A, then K
```

### 8.2 Python Quick Test

```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

# Should receive ID string and NOT_HOMED status
while ser.in_waiting:
    print(ser.readline().decode().strip())

# Test homing sequence
ser.write(b"POS:X:0.00\n")
time.sleep(0.1)
if ser.in_waiting:
    print(ser.readline().decode().strip())  # Should see STATUS:HOMED

# Test limit setting
ser.write(b"LIMIT:X:0.00:300.00\n")
time.sleep(0.1)
# (no response expected - applied silently)

# Test mode change (now should work)
ser.write(b"MODE:ACTIVE\n")
time.sleep(0.1)
# (no response expected - mode changed silently)

ser.close()
```

---

## 9. Safety Considerations

### 9.1 Mandatory Homing
- Console REQUIRES homing before manual control (ACTIVE mode)
- Prevents accidental movement with unknown position
- "------" display is visual indicator that homing is needed
- Pi must complete FluidNC homing and send positions

### 9.2 Limit Protection
- Set appropriate limits based on machine physical constraints
- Limits enforced in ACTIVE mode (jogging)
- Passive mode bypasses limits (Pi is trusted)
- Review and update limits when changing machine configuration

### 9.3 Emergency Stop
- Console does NOT have emergency stop capability
- Emergency stop must be implemented at FluidNC/Raspberry Pi level
- Consider dedicating button G for emergency stop function
- E-stop should cut power to motors, not rely on software

### 9.4 Power Loss
- Console loses all state on power loss
- Returns to NOT_HOMED state
- All limits reset to defaults
- Pi must re-home and reconfigure after power cycle

### 9.5 Communication Loss
- Console continues operating in last known mode
- ACTIVE mode: Handwheel still functional (limits enforced)
- PASSIVE mode: Display frozen until communication restored
- Implement watchdog/heartbeat in Pi application

---

## 10. Support and Resources

### 10.1 Documentation
- Full specification: `SPECIFICATION.md`
- Update summary: `SPEC_v1.1_SUMMARY.md`
- Source code: `main/console_test.c` (to be implemented)

### 10.2 Common Issues
- See troubleshooting section above
- Check GitHub issues: [repository link]

### 10.3 Contact
- For integration support: [contact info]
- For hardware issues: [contact info]

---

## Appendix A: Complete Command Reference Table

| Direction | Command | Parameters | Description | Response |
|-----------|---------|------------|-------------|----------|
| Console→Pi | `CONSOLE:ALPHAPAINT:V1.1` | - | Identification on startup | - |
| Console→Pi | `STATUS:NOT_HOMED` | - | Console not homed (startup) | - |
| Console→Pi | `STATUS:HOMED` | - | Console homed (after first POS) | - |
| Console→Pi | `BTN:<btn>:<act>` | btn: A-Z, act: SHORT/LONG | Button press event | - |
| Console→Pi | `AXIS:<axis>:SELECT` | axis: X/Y/Z | Axis selected (Active) | - |
| Console→Pi | `AXIS:<axis>:PRECISION:<state>` | state: ON/OFF | Precision mode changed | - |
| Console→Pi | `POS:<axis>:<val>` | axis: X/Y/Z, val: float | Position update (Active) | - |
| Console→Pi | `LIMIT:<axis>:<min>:<max>` | axis: X/Y/Z, min/max: float | Limit query response | - |
| Console→Pi | `ERROR:<type>:<msg>` | type, msg: strings | Error message | - |
| Pi→Console | `MODE:<mode>` | mode: ACTIVE/PASSIVE | Set operating mode | Silent (or ERROR) |
| Pi→Console | `LED:<led>:<state>` | led: A-G, state: OFF/ON/BLINK/FAST_BLINK | Set LED state | Silent (or ERROR) |
| Pi→Console | `POS:<axis>:<val>` | axis: X/Y/Z, val: float | Set position (Passive/NOT_HOMED) | Silent or STATUS:HOMED (or ERROR) |
| Pi→Console | `LIMIT:<axis>:<min>:<max>` | axis: X/Y/Z, min/max: float | Set axis limits | Silent (or ERROR) |
| Pi→Console | `LIMIT:<axis>?` | axis: X/Y/Z | Query axis limits | `LIMIT:<axis>:<min>:<max>` |
| Pi→Console | `DISPLAY:BRIGHTNESS:<lev>` | lev: 0-7 | Set brightness | Silent |
| Pi→Console | `ID?` | - | Query identification | `CONSOLE:ALPHAPAINT:V1.1` |
| Pi→Console | `STATUS?` | - | Query full status | Status string |

**Note:** Most commands do not send acknowledgment responses on success. Only errors, queries, and specific state transitions (like NOT_HOMED→HOMED) generate responses.

---

## Appendix B: State Transition Diagram

```
                    POWER-ON
                       │
                       v
              ┌────────────────┐
              │   NOT_HOMED    │
              │                │
              │ Display: "---" │
              │  (blinking)    │
              │ Mode: PASSIVE  │
              │   (forced)     │
              └────────┬───────┘
                       │
            First POS command from Pi
            (any axis: X, Y, or Z)
                       │
                       v
              ┌────────────────┐
              │     HOMED      │
              │                │
              │ Display: 0.00  │
              │ Mode: PASSIVE  │
              │ (can switch)   │
              └────────┬───────┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
 MODE:PASSIVE   MODE:ACTIVE    MODE:PASSIVE
        │              │              │
        v              v              v
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│   PASSIVE    │  │   ACTIVE     │  │   PASSIVE    │
│              │◄─┤              ├─►│              │
│ Pi controls  │  │ Handwheel on │  │ Pi controls  │
│ Limits: none │  │ Limits: ON   │  │ Limits: none │
└──────────────┘  └──────────────┘  └──────────────┘
```

**Notes:**
- Cannot transition to ACTIVE until HOMED
- MODE:ACTIVE before homing returns ERROR:NOT_HOMED
- First POS command (any axis) triggers HOMED state
- Limits only enforced in ACTIVE mode

---

**Document Version:** 1.1  
**Last Updated:** 2026-01-05  
**Author:** AlphaPaint Team

**Changelog:**
- v1.1 (2026-01-05): Added NOT_HOMED state, LIMIT commands, mandatory homing sequence, expanded examples
- v1.0 (2026-01-04): Initial release
