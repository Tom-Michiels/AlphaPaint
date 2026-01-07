# AlphaPaint Console Test Stub

Python test stub that simulates the Raspberry Pi's interaction with the AlphaPaint console firmware. Use this to test the console hardware and firmware without needing the full CNC system.

## Features

- **Interactive Menu**: Easy-to-use command interface
- **Homing Simulation**: Trigger the NOT_HOMED → HOMED state transition
- **Mode Switching**: Test PASSIVE and ACTIVE modes
- **Position Updates**: Send position commands to update displays
- **LED Control**: Control all 7 LEDs (A-G) with different states
- **Limit Configuration**: Set and query axis limits
- **Automated Test Sequence**: Run comprehensive test of all features
- **Real-time Monitoring**: See all console responses and button presses

## Requirements

- Python 3.6 or higher
- pyserial library

## Installation

1. Install Python dependencies:
```bash
cd test_stub
pip3 install -r requirements.txt
```

2. Find your serial port:
```bash
# On macOS:
ls /dev/tty.usb*

# On Linux:
ls /dev/ttyUSB*

# On Windows:
# Check Device Manager for COM ports
```

3. Set permissions (Linux/macOS):
```bash
# Add yourself to dialout group (Linux)
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect

# Or use sudo to run the script:
sudo python3 console_test_stub.py
```

## Usage

### Basic Usage

```bash
python3 console_test_stub.py --port /dev/ttyUSB0
```

### Command Line Options

```bash
python3 console_test_stub.py [options]

Options:
  --port, -p PORT    Serial port (default: /dev/ttyUSB0)
  --baud, -b BAUD    Baud rate (default: 115200)
  --help, -h         Show help message
```

### Examples

```bash
# macOS with USB-UART adapter
python3 console_test_stub.py --port /dev/tty.usbserial-0001

# Linux
python3 console_test_stub.py --port /dev/ttyUSB0

# Windows
python3 console_test_stub.py --port COM3

# Custom baud rate (if needed for debugging)
python3 console_test_stub.py --port /dev/ttyUSB0 --baud 9600
```

## Interactive Menu

Once connected, you'll see an interactive menu:

```
--- Main Menu ---
1. Perform Homing Sequence
2. Set Mode (PASSIVE/ACTIVE)
3. Set Position
4. Control LED
5. Configure Limits
6. Query Limits
7. Run Automated Test Sequence
8. Display Current State
9. Send Custom Command
0. Exit
```

### Menu Options Explained

#### 1. Perform Homing Sequence
Simulates the homing process by sending initial position commands. This transitions the console from NOT_HOMED to HOMED state.

**What happens:**
- Console displays stop showing "------"
- Numeric displays show 0.00 for all axes
- Console can now switch to ACTIVE mode
- Handwheel becomes functional in ACTIVE mode

#### 2. Set Mode (PASSIVE/ACTIVE)
Switch between operating modes:
- **PASSIVE**: Display-only mode, handwheel disabled
- **ACTIVE**: Local control mode, handwheel enabled (only works when HOMED)

#### 3. Set Position
Update the position of any axis (X, Y, or Z). The display will update to show the new value.

**Example:**
```
Enter axis (X/Y/Z): X
Enter position: 123.45
```

#### 4. Control LED
Control LEDs A-G with different states:
- **OFF**: LED off
- **ON**: LED constantly on
- **BLINK**: LED blinks at 300ms interval
- **FAST_BLINK**: LED blinks at 150ms interval

#### 5. Configure Limits
Set minimum and maximum limits for each axis. These limits are enforced when using the handwheel in ACTIVE mode.

**Example:**
```
Enter axis (X/Y/Z): X
Enter min limit: -100.0
Enter max limit: 200.0
```

#### 6. Query Limits
Query the current limit settings for an axis. The console will respond with the configured min/max values.

#### 7. Run Automated Test Sequence
Runs a comprehensive test that exercises all console features:
- Homing
- Mode switching
- Position updates
- LED control
- Limit configuration
- Limit queries

This is useful for quick verification that everything is working.

#### 8. Display Current State
Shows the current known state of the console:
- Homing state (NOT_HOMED/HOMED)
- Operating mode (PASSIVE/ACTIVE)
- Current positions for all axes
- Current limit settings

#### 9. Send Custom Command
Send a raw command to the console for advanced testing.

**Examples:**
```
MODE:ACTIVE
POS:X:50.00
LED:A:ON
LIMIT:X:-100.00:200.00
LIMIT:Y?
```

## Testing Workflow

### First Time Setup Test

1. Flash the console firmware to your ESP32
2. Connect console to computer via USB
3. Run the test stub
4. Select option 7 (Automated Test Sequence)
5. Verify:
   - All displays show numeric values
   - LEDs respond to commands
   - Button presses appear in console output

### Manual Testing Workflow

1. **Start in NOT_HOMED state**
   - All displays should show blinking "------"
   - Try pressing buttons, should see BTN events

2. **Perform homing (option 1)**
   - Displays should show 0.00
   - Console state becomes HOMED

3. **Switch to ACTIVE mode (option 2)**
   - Enter "ACTIVE"
   - X LED should light up (default active axis)

4. **Test handwheel**
   - Turn handwheel
   - Active axis display should update
   - Position updates appear in terminal

5. **Test axis selection**
   - Press Y button on console
   - Y LED should light up
   - Turn handwheel, Y position changes

6. **Test precision mode**
   - Long press active axis button (hold >500ms)
   - Last 2 digits of display should blink
   - Handwheel now moves in 0.01 increments

7. **Test LED buttons**
   - Short press A button: LED A toggles
   - Long press A button: LED A enters blink mode

8. **Test limits (option 5)**
   - Set X limits: -50.0 to 100.0
   - Try jogging with handwheel beyond limits
   - Should stop at limit boundaries

9. **Switch to PASSIVE (option 2)**
   - Enter "PASSIVE"
   - All axis LEDs turn off
   - Handwheel disabled
   - Can still update positions via commands (option 3)

## Console Response Messages

The test stub monitors and displays all messages from the console:

### Startup Messages
```
← CONSOLE:ALPHAPAINT:V1.1
← STATUS:NOT_HOMED
```

### Button Events
```
← BTN:A:SHORT         # Short press on button A
← BTN:B:LONG          # Long press on button B
← BTN:X:SHORT         # Axis button pressed
```

### Axis Events
```
← AXIS:X:SELECT       # X axis selected
← AXIS:Y:PRECISION:ON # Y axis precision mode enabled
← AXIS:Z:PRECISION:OFF
```

### Position Updates
```
← POS:X:123.45        # Position sent from handwheel movement
```

### Status Changes
```
← STATUS:HOMED        # Console transitioned to HOMED state
```

### Limit Responses
```
← LIMIT:X:-100.00:200.00  # Response to limit query
```

### Error Messages
```
← ERROR:NOT_HOMED:MODE:ACTIVE    # Tried to go ACTIVE before homing
← ERROR:INVALID_CMD              # Malformed command
← ERROR:INVALID_AXIS             # Invalid axis specified
```

## Troubleshooting

### Cannot connect to serial port

**macOS:**
- Check port name: `ls /dev/tty.usb*`
- No permission needed usually

**Linux:**
```bash
# Check port
ls /dev/ttyUSB*

# Check permissions
ls -l /dev/ttyUSB0

# Add yourself to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Or run with sudo
sudo python3 console_test_stub.py
```

**Windows:**
- Check Device Manager for COM port number
- Use format: `--port COM3`

### Console not responding

1. Check that firmware is flashed: `idf.py flash monitor`
2. Verify baud rate matches (115200)
3. Press ESP32 reset button
4. Check USB cable (use data cable, not charge-only)

### Displays showing wrong values

- Ensure you've performed homing first (option 1)
- Check that positions are in valid range (-9999.99 to 9999.99)
- Verify decimal point is in correct position (format: XXXX.XX)

### Handwheel not working

- Console must be in HOMED state (perform homing first)
- Console must be in ACTIVE mode
- Check encoder wiring (pins 34 and 39)
- Try reversing encoder A/B connections if direction is wrong

### LEDs not responding

- Test with option 4 (Control LED)
- Check I2C connections (SDA=21, SCL=22)
- Verify I2C expander address (0x38)
- Run `i2cdetect -y 1` on Linux to check I2C devices

## Protocol Reference

### Commands (PC → Console)

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| MODE | `MODE:<mode>` | Set operating mode | `MODE:ACTIVE` |
| POS | `POS:<axis>:<value>` | Set position | `POS:X:123.45` |
| LED | `LED:<led>:<state>` | Control LED | `LED:A:ON` |
| LIMIT | `LIMIT:<axis>:<min>:<max>` | Set limits | `LIMIT:X:-100.00:200.00` |
| LIMIT? | `LIMIT:<axis>?` | Query limits | `LIMIT:X?` |

### Events (Console → PC)

| Event | Format | Description | Example |
|-------|--------|-------------|---------|
| CONSOLE | `CONSOLE:<name>:<version>` | Identification | `CONSOLE:ALPHAPAINT:V1.1` |
| STATUS | `STATUS:<state>` | State change | `STATUS:HOMED` |
| BTN | `BTN:<button>:<type>` | Button press | `BTN:A:SHORT` |
| AXIS | `AXIS:<axis>:<event>` | Axis event | `AXIS:X:SELECT` |
| POS | `POS:<axis>:<value>` | Position update | `POS:Y:50.25` |
| LIMIT | `LIMIT:<axis>:<min>:<max>` | Limit response | `LIMIT:Z:-50.00:100.00` |
| ERROR | `ERROR:<message>` | Error message | `ERROR:NOT_HOMED` |

## License

Part of the AlphaPaint Console project.
