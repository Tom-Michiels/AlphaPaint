# AlphaPaint Daemon - Specification v1.0

## Document Purpose

This specification describes the AlphaPaint daemon software that runs on the Raspberry Pi. The daemon coordinates communication between the AlphaPaint Console (ESP32 control panel) and the FluidNC CNC controller to provide an integrated plotting system with simplified user interaction.

**Key Features:**
- Automatic device detection and connection
- Guided homing and canvas setup workflow
- Simple button-based drawing operations
- Handwheel jogging integration
- Safety and error handling

---

## 1. System Overview

### 1.1 Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Raspberry Pi (Linux)                       │
│                                                         │
│  ┌───────────────────────────────────────────────────┐ │
│  │         AlphaPaint Daemon (Python)                │ │
│  │                                                   │ │
│  │  ┌──────────────┐         ┌──────────────┐      │ │
│  │  │   Console    │         │   FluidNC    │      │ │
│  │  │   Handler    │         │   Handler    │      │ │
│  │  └──────┬───────┘         └──────┬───────┘      │ │
│  │         │                        │              │ │
│  │  ┌──────┴────────────────────────┴───────┐      │ │
│  │  │     State Machine & Logic             │      │ │
│  │  └───────────────────────────────────────┘      │ │
│  └───────────┬─────────────────────┬───────────────┘ │
│              │                     │                  │
└──────────────┼─────────────────────┼──────────────────┘
               │                     │
        USB/UART                USB/UART
               │                     │
      ┌────────┴────────┐   ┌────────┴────────┐
      │  AlphaPaint     │   │    FluidNC      │
      │   Console       │   │   Controller    │
      │   (ESP32)       │   │                 │
      └─────────────────┘   └─────────────────┘
```

### 1.2 Daemon Responsibilities

1. **Device Management**
   - Scan and identify USB serial devices
   - Maintain persistent connections
   - Handle reconnection on failure

2. **Workflow Orchestration**
   - Guide user through homing sequence
   - Coordinate canvas corner definition
   - Execute drawing operations

3. **Communication Bridge**
   - Translate Console button events to CNC commands
   - Forward handwheel positions as jog commands
   - Update Console display with CNC position feedback

4. **Safety & Error Handling**
   - Enforce state machine transitions
   - Validate coordinates and limits
   - Provide visual feedback via LEDs

---

## 2. Startup and Device Detection

### 2.1 Auto-Start Configuration

The daemon should be configured to start automatically on Raspberry Pi boot using systemd.

**Service File:** `/etc/systemd/system/alphapaint-daemon.service`
```ini
[Unit]
Description=AlphaPaint Daemon
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/alphapaint
ExecStart=/usr/bin/python3 /home/pi/alphapaint/daemon.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 2.2 USB Device Scanning

On startup, the daemon must:

1. **Scan all serial devices**
   - Check `/dev/ttyUSB*` and `/dev/ttyACM*`
   - Open each port at 115200 baud

2. **Identify Console**
   - Look for identification string: `CONSOLE:ALPHAPAINT:V1.1`
   - Console sends this automatically on startup or reset
   - May send `ID?` query to trigger response
   - Confirm with `STATUS:NOT_HOMED` message

3. **Identify FluidNC**
   - Look for Grbl-style responses: `Grbl`, `ok`, or `<...>` status messages
   - May send `$I` (version query) or `?` (status query)
   - FluidNC typically responds with version string like `[VER:...]`

4. **Establish Connections**
   - Once both devices identified, establish persistent connections
   - Log device paths and identities
   - Begin protocol handlers for each device

**Pseudocode:**
```python
def scan_and_connect():
    devices = list_serial_ports()  # /dev/ttyUSB*, /dev/ttyACM*

    console_port = None
    fluidnc_port = None

    for port in devices:
        device_type = identify_device(port)

        if device_type == "CONSOLE":
            console_port = port
        elif device_type == "FLUIDNC":
            fluidnc_port = port

    if console_port and fluidnc_port:
        console = ConsoleHandler(console_port)
        fluidnc = FluidNCHandler(fluidnc_port)
        return (console, fluidnc)
    else:
        log_error("Could not find both devices")
        retry_after_delay()
```

### 2.3 Connection Monitoring

- Monitor both connections for disconnection
- If either device disconnects, enter safe state
- Attempt automatic reconnection
- Re-identify devices on reconnection (ports may swap)

---

## 3. State Machine

### 3.1 State Definitions

| State | Description | LED Pattern | Allowed Actions |
|-------|-------------|-------------|-----------------|
| `STARTUP` | Daemon initializing, scanning devices | All LEDs off | None |
| `NOT_HOMED` | Console connected but machine not homed | A: slow blink (500ms) | Press A to home<br>Long press A: re-home |
| `HOMING` | Homing sequence in progress | A: fast blink (150ms) | Wait<br>Long press A: re-home |
| `HOMED` | Homing complete, ready for setup | A: solid ON | Long press A: re-home |
| `CANVAS_SETUP` | Defining canvas corners B & C | B,C: slow blink<br>D: solid ON | Press B, C (required)<br>Press D (optional)<br>Long press A: re-home |
| `READY` | Canvas defined, drawing available | A,B,C,D: solid ON<br>E,F: slow blink | Press E, F to draw<br>Use handwheel to jog<br>Long press A: re-home |
| `DRAWING` | Executing drawing operation | Active button: fast blink | Wait (buttons disabled)<br>Long press A: re-home |
| `ERROR` | Error state | All LEDs: fast blink | Long press A: recover |

### 3.2 State Transition Diagram

```
    STARTUP
       │
       ├─ Console & FluidNC detected
       │
       v
  NOT_HOMED ◄──────────────────────┐
       │                           │
       ├─ Button A pressed         │
       │                           │
       v                           │
    HOMING                         │
       │                           │
       ├─ Homing complete          │
       │                           │
       v                           │
    HOMED                          │
       │                           │
       ├─ B or C pressed (1st)     │
       │                           │
       v                           │
  CANVAS_SETUP                     │
       │                           │
       ├─ B AND C pressed          │ Button A LONG press
       │                           │ (any state)
       v                           │
    READY ◄─────────┐              │
       │            │              │
       ├─ E or F    │              │
       │  pressed   │              │
       │            │              │
       v            │              │
    DRAWING         │              │
       │            │              │
       ├─ Complete ─┘              │
       │                           │
       ├─ Error ───────────────────┘
       v
    ERROR
```

### 3.3 State Variables

The daemon maintains the following state:

```python
class DaemonState:
    current_state: str = "STARTUP"

    # Position tracking
    current_pos: dict = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
    machine_limits: dict = {'X': (0, 300), 'Y': (-150, 150), 'Z': (0, 50)}

    # Canvas definition
    point_B: tuple = None  # (x, y)
    point_C: tuple = None  # (x, y)
    pen_Z: float = 0.5     # Default Z contact position (mm above Z=0)

    # Flags
    homed: bool = False
    console_mode: str = "PASSIVE"

    # Active axis for jogging
    active_axis: str = "X"
    precision_mode: bool = False
```

---

## 4. Button Functions and Workflow

### 4.1 Button A - Homing

**Purpose:** Home the machine and establish coordinate system.

**Behavior:**

1. **Initial State (NOT_HOMED)**
   - LED A: slow blink (500ms on/off)
   - User presses button A (short press)

2. **Send Homing Command**
   - Daemon → FluidNC: `$H\n` (Grbl homing command)
   - LED A: fast blink (150ms on/off)
   - State: `HOMING`

3. **Monitor Homing Progress**
   - FluidNC sends status updates during homing
   - Parse for completion or error
   - Typical response on completion: `ok` followed by position report

4. **Homing Complete**
   - FluidNC reports final position (usually max limits or 0,0,0)
   - Daemon reads machine limits from FluidNC:
     - Send `$$` to get settings
     - Parse `$130`, `$131`, `$132` for max travel distances
     - Or parse from status response if available

5. **Configure Console**
   - Send first position to Console: `POS:X:0.00`, `POS:Y:0.00`, `POS:Z:0.00`
   - Console responds with `STATUS:HOMED`
   - Set Console limits: `LIMIT:X:<min>:<max>`, etc.
   - Set Console mode: `MODE:ACTIVE`
   - LED A: solid ON
   - State: `HOMED` → `CANVAS_SETUP`

6. **Enable Canvas Setup**
   - LEDs B, C: slow blink (500ms) - indicates canvas corners can be set
   - LED D: solid ON - indicates pen Z has default value (0.5mm), can be overridden by pressing D

**Re-homing (Any State):**
- User long-presses button A (≥500ms)
- Console sends `BTN:A:LONG`
- Daemon resets all state variables
- Returns to `NOT_HOMED` state
- Repeat homing sequence

**Implementation:**
```python
def on_button_A_short():
    if state == "NOT_HOMED":
        start_homing_sequence()

def on_button_A_long():
    # Re-home from any state
    reset_state()
    state = "NOT_HOMED"
    set_led('A', 'BLINK')  # A slow blink
    set_led('B', 'OFF')
    set_led('C', 'OFF')
    set_led('D', 'OFF')
    set_led('E', 'OFF')
    set_led('F', 'OFF')

def start_homing_sequence():
    state = "HOMING"
    set_led('A', 'FAST_BLINK')
    fluidnc.send("$H\n")

def on_homing_complete(position):
    # Read machine limits from FluidNC settings
    limits = fluidnc.get_limits()

    # Send positions to Console
    console.send(f"POS:X:{position['X']:.2f}\n")
    console.send(f"POS:Y:{position['Y']:.2f}\n")
    console.send(f"POS:Z:{position['Z']:.2f}\n")

    # Set Console limits
    console.send(f"LIMIT:X:{limits['X'][0]:.2f}:{limits['X'][1]:.2f}\n")
    console.send(f"LIMIT:Y:{limits['Y'][0]:.2f}:{limits['Y'][1]:.2f}\n")
    console.send(f"LIMIT:Z:{limits['Z'][0]:.2f}:{limits['Z'][1]:.2f}\n")

    # Enable ACTIVE mode (handwheel control)
    console.send("MODE:ACTIVE\n")

    # Update LEDs
    set_led('A', 'ON')
    set_led('B', 'BLINK')  # Canvas corner 1
    set_led('C', 'BLINK')  # Canvas corner 2
    set_led('D', 'ON')     # Pen Z has default value

    state = "CANVAS_SETUP"
    current_pos = position
    homed = True
```

---

### 4.2 Button B - Canvas Corner 1

**Purpose:** Define first corner of canvas rectangle.

**Behavior:**

1. **First Press**
   - State: `CANVAS_SETUP` or `READY`
   - User positions handwheel to desired corner location
   - User presses button B (short press)
   - Console sends `BTN:B:SHORT`

2. **Store Position**
   - Daemon stores current X, Y position as `point_B`
   - Z position is ignored (only X, Y matter for canvas)

3. **Visual Feedback**
   - LED B: momentarily OFF (100ms) then solid ON
   - Indicates point stored

4. **Subsequent Presses**
   - User can press B again to update point_B
   - Same feedback: LED B blinks OFF briefly then back ON

5. **State Update**
   - If both `point_B` and `point_C` are defined:
     - State → `READY`
     - LEDs E, F: slow blink (indicates drawing functions available)

**Implementation:**
```python
def on_button_B_short():
    if state in ["CANVAS_SETUP", "READY"]:
        # Store current X, Y position
        point_B = (current_pos['X'], current_pos['Y'])

        # Visual feedback
        set_led('B', 'OFF')
        time.sleep(0.1)
        set_led('B', 'ON')

        # Check if ready
        if point_B and point_C:
            state = "READY"
            set_led('E', 'BLINK')
            set_led('F', 'BLINK')
            log(f"Canvas defined: B={point_B}, C={point_C}")
```

---

### 4.3 Button C - Canvas Corner 2

**Purpose:** Define second corner of canvas rectangle.

**Behavior:**

Identical to Button B, but stores position as `point_C`.

1. User positions to desired corner
2. Press button C (short press)
3. Store current X, Y as `point_C`
4. LED C: blink OFF (100ms) then solid ON
5. If both points defined → State = `READY`, enable E and F

**Implementation:**
```python
def on_button_C_short():
    if state in ["CANVAS_SETUP", "READY"]:
        point_C = (current_pos['X'], current_pos['Y'])

        set_led('C', 'OFF')
        time.sleep(0.1)
        set_led('C', 'ON')

        if point_B and point_C:
            state = "READY"
            set_led('E', 'BLINK')
            set_led('F', 'BLINK')
            log(f"Canvas defined: B={point_B}, C={point_C}")
```

---

### 4.4 Button D - Pen Z Position

**Purpose:** Define Z-coordinate where pen contacts paper.

**Default Value:** 0.5 mm (pen down position for drawing)

**Z-Axis Coordinate System:**
- Z=0: Home position (bottom/reference)
- Z=max (e.g., 50mm): Maximum height - pen fully raised for travel moves
- pen_Z (default 0.5mm): Height where pen touches paper for drawing

**Behavior:**

1. **First Press**
   - State: `CANVAS_SETUP` or `READY`
   - User jogs Z-axis to position where pen just touches paper
   - User presses button D (short press)
   - Console sends `BTN:D:SHORT`

2. **Store Z Position**
   - Daemon stores current Z position as `pen_Z`
   - This becomes the "pen down" Z coordinate for drawing

3. **Visual Feedback**
   - LED D: momentarily OFF (100ms) then solid ON

4. **Subsequent Presses**
   - User can press D again to update `pen_Z`
   - Same feedback pattern

**Note:**
- Button D is **optional** - a default value of 0.5mm is already set after homing
- LED D is solid ON after homing to indicate the default value is active
- Button D can be pressed at any time after homing to override the default
- If never pressed, the default value of 0.5mm is used for all drawing operations

**Implementation:**
```python
def on_button_D_short():
    if state in ["CANVAS_SETUP", "READY"]:
        pen_Z = current_pos['Z']

        set_led('D', 'OFF')
        time.sleep(0.1)
        set_led('D', 'ON')

        log(f"Pen Z position set to {pen_Z:.2f}")
```

---

### 4.5 Button E - Draw Line

**Purpose:** Draw a straight line from point B to point C.

**Requirements:**
- State must be `READY`
- Both `point_B` and `point_C` must be defined

**Behavior:**

1. **Trigger**
   - User presses button E (short press)
   - Console sends `BTN:E:SHORT`

2. **Pre-checks**
   - Verify `point_B` and `point_C` are defined
   - If not: blink all LEDs (error indication), return

3. **State Transition**
   - State → `DRAWING`
   - LED E: fast blink (150ms)
   - Set Console mode: `MODE:PASSIVE`

4. **Drawing Sequence**

   a. **Lift Pen (Maximum Z)**
   ```gcode
   G90          ; Absolute positioning
   G0 Z<max_Z>  ; Lift to maximum Z height (pen fully raised)
   ```

   b. **Move to Point B**
   ```gcode
   G0 X<Bx> Y<By>
   ```

   c. **Lower Pen**
   ```gcode
   G1 Z<pen_Z> F100   ; Lower to pen contact Z at 100mm/min (default 0.5mm)
   ```

   d. **Draw Line to Point C**
   ```gcode
   G1 X<Cx> Y<Cy> F500   ; Linear move at 500mm/min drawing speed
   ```

   e. **Lift Pen**
   ```gcode
   G0 Z<max_Z>  ; Lift to maximum height
   ```

   f. **Return to Start**
   ```gcode
   G0 X<Bx> Y<By>   ; Optional: return to B
   ```

5. **Monitor Execution**
   - Send G-code commands to FluidNC
   - Monitor `ok` responses
   - Update Console display with current position during execution

6. **Completion**
   - State → `READY`
   - LED E: solid ON (back to previous state)
   - Set Console mode: `MODE:ACTIVE`

**Error Handling:**
- If FluidNC reports error during drawing:
  - State → `ERROR`
  - All LEDs: fast blink
  - Log error message

**Implementation:**
```python
def on_button_E_short():
    if state != "READY":
        return

    if not point_B or not point_C:
        error_blink_all_leds()
        return

    state = "DRAWING"
    set_led('E', 'FAST_BLINK')
    console.send("MODE:PASSIVE\n")

    try:
        draw_line(point_B, point_C, pen_Z)
        state = "READY"
        set_led('E', 'BLINK')
        console.send("MODE:ACTIVE\n")
    except Exception as e:
        state = "ERROR"
        log(f"Drawing error: {e}")
        error_blink_all_leds()

def draw_line(p1, p2, pen_z):
    """Draw line from p1 to p2 using G-code."""
    # Get maximum Z from machine limits
    max_z = machine_limits['Z'][1]  # e.g., 50mm

    # Lift pen to maximum height
    fluidnc.send_gcode(f"G90")
    fluidnc.send_gcode(f"G0 Z{max_z:.2f}")

    # Move to start point
    fluidnc.send_gcode(f"G0 X{p1[0]:.2f} Y{p1[1]:.2f}")

    # Lower pen to drawing height
    fluidnc.send_gcode(f"G1 Z{pen_z:.2f} F100")

    # Draw line
    fluidnc.send_gcode(f"G1 X{p2[0]:.2f} Y{p2[1]:.2f} F500")

    # Lift pen to maximum height
    fluidnc.send_gcode(f"G0 Z{max_z:.2f}")

    # Return to start (optional)
    # fluidnc.send_gcode(f"G0 X{p1[0]:.2f} Y{p1[1]:.2f}")
```

---

### 4.6 Button F - Draw Ellipse

**Purpose:** Draw an ellipse inscribed in the rectangle defined by points B and C.

**Requirements:**
- State must be `READY`
- Both `point_B` and `point_C` must be defined

**Behavior:**

Similar to Button E, but draws an ellipse instead of a line.

1. **Trigger**
   - User presses button F (short press)
   - Console sends `BTN:F:SHORT`

2. **State Transition**
   - State → `DRAWING`
   - LED F: fast blink (150ms)
   - Set Console mode: `MODE:PASSIVE`

3. **Ellipse Calculation**

   Given points B = (x1, y1) and C = (x2, y2):

   ```
   center_x = (x1 + x2) / 2
   center_y = (y1 + y2) / 2
   radius_x = |x2 - x1| / 2
   radius_y = |y2 - y1| / 2
   ```

   Parametric ellipse equation:
   ```
   x(t) = center_x + radius_x * cos(t)
   y(t) = center_y + radius_y * sin(t)

   where t ∈ [0, 2π]
   ```

4. **Drawing Sequence**

   a. **Lift Pen**
   ```gcode
   G90
   G0 Z<max_Z>  ; Lift to maximum height
   ```

   b. **Move to Ellipse Start** (t=0, rightmost point)
   ```gcode
   G0 X<center_x + radius_x> Y<center_y>
   ```

   c. **Lower Pen**
   ```gcode
   G1 Z<pen_Z> F100  ; Lower to drawing height (default 0.5mm)
   ```

   d. **Draw Ellipse**

   **Note:** G-code G2/G3 arc commands only support perfect circles. For true ellipses (radius_x ≠ radius_y), we approximate with line segments.

   **Special case - Perfect Circle:** If radius_x == radius_y, use G2 for efficiency:
   ```gcode
   G2 X<start_x> Y<start_y> I<-radius> J0 F500  ; Full circle CW
   ```

   **General case - Ellipse:** Approximate with line segments (angular step ~10 degrees):
   ```python
   num_segments = 36  # 10 degree steps
   for i in range(num_segments + 1):
       t = (i / num_segments) * 2 * math.pi
       x = center_x + radius_x * math.cos(t)
       y = center_y + radius_y * math.sin(t)

       # Send G-code
       G1 X<x> Y<y> F500
   ```

   e. **Lift Pen**
   ```gcode
   G0 Z<max_Z>  ; Lift to maximum height
   ```

5. **Completion**
   - State → `READY`
   - LED F: slow blink
   - Set Console mode: `MODE:ACTIVE`

**Implementation:**
```python
def on_button_F_short():
    if state != "READY":
        return

    if not point_B or not point_C:
        error_blink_all_leds()
        return

    state = "DRAWING"
    set_led('F', 'FAST_BLINK')
    console.send("MODE:PASSIVE\n")

    try:
        draw_ellipse(point_B, point_C, pen_Z)
        state = "READY"
        set_led('F', 'BLINK')
        console.send("MODE:ACTIVE\n")
    except Exception as e:
        state = "ERROR"
        log(f"Drawing error: {e}")
        error_blink_all_leds()

def draw_ellipse(p1, p2, pen_z):
    """Draw ellipse inscribed in rectangle defined by p1 and p2."""
    import math

    # Calculate ellipse parameters
    center_x = (p1[0] + p2[0]) / 2
    center_y = (p1[1] + p2[1]) / 2
    radius_x = abs(p2[0] - p1[0]) / 2
    radius_y = abs(p2[1] - p1[1]) / 2

    # Get maximum Z from machine limits
    max_z = machine_limits['Z'][1]  # e.g., 50mm

    # Lift pen to maximum height
    fluidnc.send_gcode("G90")
    fluidnc.send_gcode(f"G0 Z{max_z:.2f}")

    # Move to ellipse start (t=0, rightmost point)
    start_x = center_x + radius_x
    start_y = center_y
    fluidnc.send_gcode(f"G0 X{start_x:.2f} Y{start_y:.2f}")

    # Lower pen to drawing height
    fluidnc.send_gcode(f"G1 Z{pen_z:.2f} F100")

    # Check if it's a perfect circle
    tolerance = 0.01  # 0.01mm tolerance
    if abs(radius_x - radius_y) < tolerance:
        # Use G2 arc command for perfect circle (more efficient)
        fluidnc.send_gcode(f"G2 X{start_x:.2f} Y{start_y:.2f} I{-radius_x:.2f} J0 F500")
    else:
        # Draw ellipse with line segments
        num_segments = 36  # 10 degree angular resolution
        for i in range(1, num_segments + 1):
            t = (i / num_segments) * 2 * math.pi
            x = center_x + radius_x * math.cos(t)
            y = center_y + radius_y * math.sin(t)
            fluidnc.send_gcode(f"G1 X{x:.2f} Y{y:.2f} F500")

    # Lift pen to maximum height
    fluidnc.send_gcode(f"G0 Z{max_z:.2f}")
```

---

### 4.7 Button G - Reserved

**Status:** Reserved for future use (e.g., emergency stop, additional drawing function).

**Current Behavior:**
- No action assigned
- LED G remains off

---

## 5. Handwheel Jogging Integration

### 5.1 Active Mode Jogging

When Console is in `ACTIVE` mode (state = `HOMED`, `CANVAS_SETUP`, or `READY`):

1. **Console Controls Position**
   - User rotates handwheel
   - User selects axis (X, Y, or Z button)
   - Console sends position updates: `POS:X:123.45`

2. **Daemon Forwards to FluidNC**
   - Receive `POS:<axis>:<value>` from Console
   - Send jog command to FluidNC using `$J` command

   **FluidNC $J Jog Command:**
   ```gcode
   $J=G90 G21 <axis><value> F<feedrate>
   ```

   Where:
   - `G90`: Absolute positioning mode
   - `G21`: Metric units (mm)
   - `F<feedrate>`: Jog speed in mm/min (from config, e.g., 1000)

   Example:
   ```
   Console → Daemon: POS:X:150.50
   Daemon → FluidNC: $J=G90 G21 X150.50 F1000
   ```

   **Benefits of $J over G0:**
   - Can be cancelled immediately (safety)
   - Doesn't affect G-code parser state
   - Specifically designed for real-time jogging
   - Automatically clears planner buffer on new jog command

3. **Rate Limiting**
   - Console sends position updates at maximum 20 Hz
   - Daemon may optionally throttle to reduce FluidNC command queue load
   - Debounce small changes (e.g., < 0.01mm) to avoid jitter
   - Each new `$J` command cancels the previous one (no queue buildup)

4. **Update Internal State**
   - Daemon tracks `current_pos` based on Console updates
   - Use FluidNC status reports (`?` query) to verify actual position periodically

**Implementation:**
```python
def on_console_position_update(axis, value):
    """Handle position update from Console (Active mode)."""
    if console_mode == "ACTIVE":
        # Forward to FluidNC as $J jog command
        jog_feedrate = config['machine']['jog_feedrate']  # e.g., 1000 mm/min
        fluidnc.send(f"$J=G90 G21 {axis}{value:.2f} F{jog_feedrate}\n")

        # Update internal state
        current_pos[axis] = value

def on_console_axis_select(axis):
    """Handle axis selection from Console."""
    active_axis = axis
    log(f"Active axis: {axis}")

def on_console_precision_mode(axis, enabled):
    """Handle precision mode toggle."""
    precision_mode = enabled
    log(f"Precision mode {axis}: {enabled}")
```

### 5.2 Cancelling Jog Commands

**Important:** Jog commands should be cancelled in the following situations:

1. **User initiates re-homing** (long press A)
   - Send `0x85` (Jog Cancel) to FluidNC
   - Or send Ctrl-X (`\x18`) for soft reset

2. **Emergency stop** (if implemented)
   - Send Ctrl-X (`\x18`) for immediate stop

**Implementation:**
```python
def cancel_jog():
    """Cancel any active jog command."""
    fluidnc.send("\x85")  # Jog Cancel character
    # Or: fluidnc.send("\x18")  # Soft reset (more aggressive)

def on_button_A_long():
    """Re-home from any state."""
    # Cancel any active jog
    if console_mode == "ACTIVE":
        cancel_jog()

    # Reset state and return to NOT_HOMED
    reset_state()
    state = "NOT_HOMED"
    # ... (rest of re-homing logic)
```

### 5.3 Passive Mode Display Updates

When Console is in `PASSIVE` mode (state = `DRAWING` or during CNC operations):

1. **FluidNC Controls Position**
   - FluidNC executes G-code commands
   - Daemon queries FluidNC status: `?`
   - FluidNC responds with position: `<Idle|MPos:10.00,20.00,5.00|...>`

2. **Update Console Display**
   - Parse position from FluidNC status
   - Send to Console: `POS:X:10.00`, `POS:Y:20.00`, `POS:Z:5.00`

3. **Polling Rate**
   - Query FluidNC status at 10 Hz
   - Send Console updates only when position changes

**Implementation:**
```python
def poll_fluidnc_status():
    """Periodically query FluidNC position (Passive mode)."""
    while True:
        if console_mode == "PASSIVE":
            # Query FluidNC
            fluidnc.send("?\n")

            # Parse response (example: <Idle|MPos:10.00,20.00,5.00|...>)
            response = fluidnc.readline()
            pos = parse_fluidnc_position(response)

            # Update Console if changed
            if pos != current_pos:
                console.send(f"POS:X:{pos['X']:.2f}\n")
                console.send(f"POS:Y:{pos['Y']:.2f}\n")
                console.send(f"POS:Z:{pos['Z']:.2f}\n")
                current_pos = pos

        time.sleep(0.1)  # 10 Hz
```

---

## 6. FluidNC Communication

### 6.1 FluidNC Protocol Overview

FluidNC implements the Grbl protocol, a standard for CNC control.

**Key Commands:**

| Command | Description | Response |
|---------|-------------|----------|
| `$H\n` | Home all axes | `ok` when complete |
| `$$\n` | Print settings | List of `$<num>=<value>` |
| `$I\n` | Print version info | `[VER:...]` and `[OPT:...]` |
| `?\n` | Status query | `<State\|MPos:x,y,z\|...>` |
| `$J=G90 X10 F1000\n` | Jog command (absolute) | `ok` |
| `$J=G91 X1.0 F100\n` | Jog command (relative) | `ok` |
| `G90 G0 X10 Y20\n` | Absolute rapid move | `ok` |
| `G91 G1 X1.0 F100\n` | Incremental feed move | `ok` |
| `M3 S1000\n` | Spindle on (if used) | `ok` |
| `M5\n` | Spindle off | `ok` |

**Response Types:**
- `ok` - Command executed successfully
- `error:<code>` - Command failed
- `<...>` - Status report
- `[MSG:...]` - Informational message
- `[GC:...]` - G-code parser state

### 6.2 Reading Machine Limits

After homing, query machine limits:

```python
def get_fluidnc_limits():
    """Query FluidNC for machine limits."""
    fluidnc.send("$$\n")

    limits = {'X': (0, 300), 'Y': (-150, 150), 'Z': (-50, 0)}  # Defaults

    # Parse responses
    # $130=300.000  (X max travel)
    # $131=300.000  (Y max travel)
    # $132=50.000   (Z max travel)

    while True:
        line = fluidnc.readline()
        if line.startswith('ok'):
            break

        if line.startswith('$130='):
            limits['X'] = (0, float(line.split('=')[1]))
        elif line.startswith('$131='):
            limits['Y'] = (0, float(line.split('=')[1]))
        elif line.startswith('$132='):
            limits['Z'] = (-float(line.split('=')[1]), 0)

    return limits
```

**Note:** Limit interpretation depends on machine configuration. Adjust parsing logic based on actual FluidNC setup (e.g., some machines use negative coordinates).

### 6.3 Sending G-code Sequences

When sending multi-line G-code (e.g., drawing operations):

1. **Send command and wait for `ok`**
   ```python
   def send_gcode(cmd):
       fluidnc.send(cmd + "\n")

       # Wait for response
       while True:
           response = fluidnc.readline()

           if response.startswith('ok'):
               return True
           elif response.startswith('error'):
               raise Exception(f"G-code error: {response}")
   ```

2. **Update Console display during execution**
   - While waiting for `ok`, periodically query status (`?`)
   - Update Console with current position

3. **Handle errors**
   - If `error:` response received, abort sequence
   - Transition to `ERROR` state
   - Log error details

---

## 7. Error Handling and Safety

### 7.1 Error Conditions

| Error | Detection | Response |
|-------|-----------|----------|
| Console disconnected | Serial read timeout | Enter safe state, attempt reconnect |
| FluidNC disconnected | Serial read timeout | Enter safe state, attempt reconnect |
| Homing failure | `error:` from FluidNC | State → `ERROR`, blink all LEDs |
| Drawing failure | `error:` during G-code | State → `ERROR`, blink all LEDs |
| Invalid state transition | Button press in wrong state | Ignore, optionally blink LED |
| Canvas not defined | E or F pressed without B, C | Blink all LEDs (error indication) |
| Limit exceeded | Position beyond machine limits | FluidNC will reject, handle error |

### 7.2 Recovery Procedures

**From ERROR State:**
- User long-presses button A
- Daemon resets to `NOT_HOMED`
- User must re-home

**Connection Loss:**
- Daemon continuously attempts reconnection
- When both devices reconnected, reset to `NOT_HOMED`
- Visual indication on Console (all LEDs fast blink during disconnect)

### 7.3 Safety Features

1. **Mandatory Homing**
   - Console cannot enter ACTIVE mode until homed
   - Prevents uncontrolled movement with unknown position

2. **Limit Enforcement**
   - Console enforces limits in ACTIVE mode (handwheel)
   - FluidNC enforces machine limits (hard limits via settings)

3. **Pen Lift Before Moves**
   - Always lift pen to maximum Z height before rapid XY moves
   - Maximum Z is read from machine limits after homing
   - Prevents dragging pen across work surface

4. **Mode Consistency**
   - Console in PASSIVE during drawing (prevents handwheel interference)
   - Console in ACTIVE during jogging (prevents display lag)

---

## 8. Configuration and Settings

### 8.1 Daemon Configuration File

**File:** `/home/pi/alphapaint/config.yaml`

```yaml
# AlphaPaint Daemon Configuration

serial:
  baud_rate: 115200
  timeout: 1.0
  reconnect_delay: 5.0

machine:
  pen_z_default: 0.5        # Default pen contact Z (mm) - height where pen touches paper
  jog_feedrate: 1000        # Jog speed (mm/min)
  draw_feedrate: 500        # Drawing speed (mm/min)
  pen_lift_feedrate: 100    # Pen Z movement speed (mm/min)
  # Note: Safe Z height is dynamically read from machine_limits['Z'][1]

drawing:
  ellipse_segments: 36      # Number of segments for ellipse approximation
  min_position_delta: 0.01  # Minimum position change to send jog (mm)

logging:
  level: INFO               # DEBUG, INFO, WARNING, ERROR
  file: /var/log/alphapaint-daemon.log
  max_size: 10485760        # 10 MB
  backup_count: 5

debugging:
  enable_console_echo: false  # Echo Console messages to log
  enable_fluidnc_echo: false  # Echo FluidNC messages to log
```

### 8.2 Loading Configuration

```python
import yaml

def load_config():
    with open('/home/pi/alphapaint/config.yaml', 'r') as f:
        return yaml.safe_load(f)

config = load_config()
pen_z_default = config['machine']['pen_z_default']  # 0.5mm
# Safe Z height is read from machine_limits after homing
```

---

## 9. Logging and Debugging

### 9.1 Log Levels

- **DEBUG:** Low-level protocol messages, all serial I/O
- **INFO:** State transitions, button events, drawing operations
- **WARNING:** Recoverable errors, retries
- **ERROR:** Unrecoverable errors, connection loss

### 9.2 Log Format

```
2026-01-05 14:30:15 [INFO] Daemon started
2026-01-05 14:30:16 [INFO] Console detected on /dev/ttyUSB0 (V1.1)
2026-01-05 14:30:16 [INFO] FluidNC detected on /dev/ttyUSB1
2026-01-05 14:30:17 [INFO] State: STARTUP → NOT_HOMED
2026-01-05 14:30:20 [INFO] Button A pressed (short)
2026-01-05 14:30:20 [INFO] State: NOT_HOMED → HOMING
2026-01-05 14:30:25 [INFO] Homing complete: X=0.00, Y=0.00, Z=0.00
2026-01-05 14:30:25 [INFO] State: HOMING → CANVAS_SETUP
2026-01-05 14:30:30 [INFO] Button B pressed: Point B set to (50.00, 50.00)
2026-01-05 14:30:35 [INFO] Button C pressed: Point C set to (200.00, 150.00)
2026-01-05 14:30:35 [INFO] State: CANVAS_SETUP → READY
2026-01-05 14:30:40 [INFO] Button E pressed: Drawing line
2026-01-05 14:30:40 [INFO] State: READY → DRAWING
2026-01-05 14:30:50 [INFO] Drawing complete
2026-01-05 14:30:50 [INFO] State: DRAWING → READY
```

### 9.3 Debug Output

When `enable_console_echo` or `enable_fluidnc_echo` is enabled:

```
2026-01-05 14:30:16 [DEBUG] Console RX: CONSOLE:ALPHAPAINT:V1.1
2026-01-05 14:30:16 [DEBUG] Console RX: STATUS:NOT_HOMED
2026-01-05 14:30:17 [DEBUG] Console TX: LED:A:BLINK
2026-01-05 14:30:20 [DEBUG] Console RX: BTN:A:SHORT
2026-01-05 14:30:20 [DEBUG] FluidNC TX: $H
2026-01-05 14:30:25 [DEBUG] FluidNC RX: ok
```

---

## 10. Implementation Architecture

### 10.1 Module Structure

```
alphapaint-daemon/
├── daemon.py              # Main entry point
├── config.yaml            # Configuration file
├── requirements.txt       # Python dependencies
├── lib/
│   ├── __init__.py
│   ├── console.py         # Console handler class
│   ├── fluidnc.py         # FluidNC handler class
│   ├── state_machine.py   # State machine logic
│   ├── drawing.py         # Drawing functions (line, ellipse)
│   └── utils.py           # Utility functions
└── tests/
    ├── test_console.py
    ├── test_fluidnc.py
    └── test_drawing.py
```

### 10.2 Main Classes

**ConsoleHandler:**
```python
class ConsoleHandler:
    def __init__(self, port):
        self.port = port
        self.serial = serial.Serial(port, 115200, timeout=1)
        self.callbacks = {}

    def send(self, message):
        """Send message to Console."""
        pass

    def set_led(self, led, state):
        """Set LED state."""
        pass

    def set_mode(self, mode):
        """Set Console mode (ACTIVE/PASSIVE)."""
        pass

    def set_position(self, axis, value):
        """Set position display (Passive mode)."""
        pass

    def on_message(self, callback):
        """Register callback for incoming messages."""
        pass
```

**FluidNCHandler:**
```python
class FluidNCHandler:
    def __init__(self, port):
        self.port = port
        self.serial = serial.Serial(port, 115200, timeout=1)

    def send_gcode(self, command):
        """Send G-code command and wait for OK."""
        pass

    def home(self):
        """Execute homing sequence."""
        pass

    def get_limits(self):
        """Query machine limits."""
        pass

    def get_status(self):
        """Query current status and position."""
        pass

    def jog(self, axis, position):
        """Send jog command (absolute positioning)."""
        pass
```

**StateMachine:**
```python
class StateMachine:
    def __init__(self, console, fluidnc):
        self.state = "STARTUP"
        self.console = console
        self.fluidnc = fluidnc
        self.point_B = None
        self.point_C = None
        self.pen_Z = -0.5
        self.current_pos = {'X': 0, 'Y': 0, 'Z': 0}

    def on_button_press(self, button, action):
        """Handle button event from Console."""
        pass

    def on_position_update(self, axis, value):
        """Handle position update from Console (Active mode)."""
        pass

    def transition(self, new_state):
        """Transition to new state with logging."""
        pass
```

**Drawing Functions:**
```python
def draw_line(fluidnc, p1, p2, pen_z, safe_z=5.0):
    """Draw line from p1 to p2."""
    pass

def draw_ellipse(fluidnc, p1, p2, pen_z, safe_z=5.0, segments=36):
    """Draw ellipse inscribed in rectangle p1-p2."""
    pass
```

### 10.3 Main Loop

```python
def main():
    # Load configuration
    config = load_config()

    # Setup logging
    setup_logging(config)

    # Scan and connect to devices
    console, fluidnc = scan_and_connect()

    # Initialize state machine
    state_machine = StateMachine(console, fluidnc)

    # Register callbacks
    console.on_message('BTN', state_machine.on_button_press)
    console.on_message('POS', state_machine.on_position_update)
    console.on_message('AXIS', state_machine.on_axis_event)

    # Start background threads
    start_fluidnc_polling_thread(fluidnc, console, state_machine)

    # Main event loop
    try:
        while True:
            # Process Console messages
            console.process_messages()

            # Process FluidNC messages
            fluidnc.process_messages()

            time.sleep(0.01)  # 100 Hz loop

    except KeyboardInterrupt:
        log("Daemon shutting down")

    finally:
        console.disconnect()
        fluidnc.disconnect()

if __name__ == "__main__":
    main()
```

---

## 11. Testing and Validation

### 11.1 Unit Tests

- **Console Communication:** Test message parsing, LED control, mode switching
- **FluidNC Communication:** Test G-code sending, status parsing, limit reading
- **Drawing Functions:** Test line and ellipse G-code generation
- **State Machine:** Test state transitions, button handling

### 11.2 Integration Tests

1. **Device Detection**
   - Connect Console and FluidNC
   - Verify automatic detection
   - Swap USB ports, verify re-detection

2. **Homing Sequence**
   - Start daemon
   - Press button A
   - Verify FluidNC homing executed
   - Verify Console updated with position and limits

3. **Canvas Setup**
   - Move to corner positions
   - Press B and C
   - Verify LED feedback
   - Verify state transition to READY

4. **Drawing Operations**
   - Press E (line drawing)
   - Verify G-code sent to FluidNC
   - Verify pen movement
   - Press F (ellipse drawing)
   - Verify correct ellipse shape

5. **Jogging**
   - Rotate handwheel
   - Verify FluidNC receives jog commands
   - Verify machine moves correctly

6. **Error Recovery**
   - Disconnect FluidNC during drawing
   - Verify error state
   - Reconnect and long-press A
   - Verify recovery

### 11.3 Acceptance Criteria

- [ ] Daemon auto-starts on Raspberry Pi boot
- [ ] Console and FluidNC automatically detected within 10 seconds
- [ ] Homing sequence completes successfully
- [ ] Canvas points B and C can be set and updated
- [ ] Line drawing (button E) executes correctly
- [ ] Ellipse drawing (button F) executes correctly
- [ ] Handwheel jogging works smoothly in all axes
- [ ] LED feedback matches specification
- [ ] Re-homing (long press A) works from any state
- [ ] Error handling and recovery functional
- [ ] No unintended machine movements

---

## 12. Future Enhancements

### 12.1 Potential Features

1. **Button G Functions**
   - Emergency stop
   - Pen test (lower and lift pen at current position)
   - Additional drawing patterns (circle, rectangle, spiral)

2. **Advanced Drawing**
   - Load G-code files from USB stick
   - Draw text (bitmap font rendering)
   - Pattern fill (hatching)

3. **Canvas Management**
   - Multiple saved canvas presets
   - Auto-calibration with touch probe

4. **Networking**
   - Web interface for configuration
   - Remote monitoring
   - Upload drawings via web/SSH

5. **Feedback**
   - Display messages on Console (if display is character-capable)
   - Audio feedback (beeper) for errors

---

## 13. Appendix A: Message Flow Examples

### 13.1 Complete Startup and Drawing Session

```
# Daemon starts
[Daemon] Scan USB devices
[Daemon] → Console: ID?
[Console] → Daemon: CONSOLE:ALPHAPAINT:V1.1
[Console] → Daemon: STATUS:NOT_HOMED
[Daemon] → FluidNC: $I
[FluidNC] → Daemon: [VER:3.7...]
[Daemon] Devices identified

# Initial state
[Daemon] → Console: LED:A:BLINK (slow)
[State: NOT_HOMED]

# User presses A (home)
[Console] → Daemon: BTN:A:SHORT
[Daemon] → Console: LED:A:FAST_BLINK
[Daemon] → FluidNC: $H
[FluidNC] Executes homing...
[FluidNC] → Daemon: ok
[Daemon] → FluidNC: $$
[FluidNC] → Daemon: (settings including limits)
[Daemon] → Console: POS:X:0.00
[Console] → Daemon: STATUS:HOMED
[Daemon] → Console: POS:Y:0.00
[Daemon] → Console: POS:Z:0.00
[Daemon] → Console: LIMIT:X:0.00:300.00
[Daemon] → Console: LIMIT:Y:-150.00:150.00
[Daemon] → Console: LIMIT:Z:-50.00:0.00
[Daemon] → Console: MODE:ACTIVE
[Daemon] → Console: LED:A:ON
[Daemon] → Console: LED:B:BLINK
[Daemon] → Console: LED:C:BLINK
[Daemon] → Console: LED:D:ON
[State: CANVAS_SETUP]

# User jogs to corner 1 and presses B
[Console] → Daemon: POS:X:50.00
[Daemon] → FluidNC: $J=G90 G21 X50.00 F1000
[Console] → Daemon: POS:Y:50.00
[Daemon] → FluidNC: $J=G90 G21 Y50.00 F1000
[Console] → Daemon: BTN:B:SHORT
[Daemon] Store point_B = (50, 50)
[Daemon] → Console: LED:B:OFF
[Daemon] (wait 100ms)
[Daemon] → Console: LED:B:ON

# User jogs to corner 2 and presses C
[Console] → Daemon: POS:X:200.00
[Daemon] → FluidNC: $J=G90 G21 X200.00 F1000
[Console] → Daemon: POS:Y:150.00
[Daemon] → FluidNC: $J=G90 G21 Y150.00 F1000
[Console] → Daemon: BTN:C:SHORT
[Daemon] Store point_C = (200, 150)
[Daemon] → Console: LED:C:OFF
[Daemon] (wait 100ms)
[Daemon] → Console: LED:C:ON
[Daemon] → Console: LED:E:BLINK
[Daemon] → Console: LED:F:BLINK
[State: READY]

# User presses E (draw line)
[Console] → Daemon: BTN:E:SHORT
[Daemon] → Console: LED:E:FAST_BLINK
[Daemon] → Console: MODE:PASSIVE
[Daemon] → FluidNC: G90
[FluidNC] → Daemon: ok
[Daemon] → FluidNC: G0 Z5.00
[FluidNC] → Daemon: ok
[Daemon] → FluidNC: G0 X50.00 Y50.00
[FluidNC] → Daemon: ok
[Daemon] → FluidNC: G1 Z-0.50 F100
[FluidNC] → Daemon: ok
[Daemon] → FluidNC: G1 X200.00 Y150.00 F500
[FluidNC] → Daemon: ok
[Daemon] → FluidNC: G0 Z5.00
[FluidNC] → Daemon: ok
[Daemon] → Console: LED:E:BLINK
[Daemon] → Console: MODE:ACTIVE
[State: READY]
```

---

## 14. Appendix B: G-code Reference

### 14.1 Essential G-code Commands

| Command | Description | Example |
|---------|-------------|---------|
| `G90` | Absolute positioning mode | `G90` |
| `G91` | Relative positioning mode | `G91` |
| `G0 X Y Z` | Rapid move (non-cutting) | `G0 X10 Y20 Z5` |
| `G1 X Y Z F` | Linear move (cutting) at feedrate | `G1 X100 Y50 F500` |
| `G2 X Y I J F` | Clockwise arc | `G2 X10 Y10 I5 J0 F300` |
| `G3 X Y I J F` | Counter-clockwise arc | `G3 X10 Y10 I5 J0 F300` |
| `G28` | Go to home position | `G28` |
| `M3 S` | Spindle on (CW) at speed | `M3 S1000` |
| `M5` | Spindle off | `M5` |

### 14.2 Typical Drawing Sequence

```gcode
G90              ; Absolute mode
G0 Z50.00        ; Lift to maximum Z height (pen fully raised)
G0 X50 Y50       ; Rapid to start position
G1 Z0.50 F100    ; Lower pen to drawing height (pen touches paper)
G1 X100 Y50 F500 ; Draw to point
G1 X100 Y100     ; Draw to point
G1 X50 Y100      ; Draw to point
G1 X50 Y50       ; Draw to start (close shape)
G0 Z50.00        ; Lift pen to maximum height
```

**Note:** Maximum Z height (50mm in this example) is read from machine limits. Pen drawing height (0.5mm) is configurable via button D.

---

## 15. Document Information

**Version:** 1.0
**Date:** 2026-01-05
**Author:** AlphaPaint Team

**Revision History:**
- v1.0 (2026-01-05): Initial specification

**Related Documents:**
- [Console Integration Guide](../Console/INTEGRATION_GUIDE.md)
- [Console Specification](../Console/SPECIFICATION.md)

**Contact:**
For questions or clarifications, contact the AlphaPaint development team.

---

**END OF SPECIFICATION**
