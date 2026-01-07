# AlphaPaint Daemon

The AlphaPaint daemon runs on the Raspberry Pi and coordinates communication between the AlphaPaint Console (ESP32) and the FluidNC CNC controller.

## Features

- Automatic device detection (Console and FluidNC)
- State machine for guided workflow
- Homing sequence with LED feedback
- Canvas corner definition (B and C points)
- Pen Z position calibration (D point)
- Line drawing (button E)
- Ellipse drawing (button F) with automatic circle detection
- Real-time jogging with $J commands
- Error handling and recovery

## Installation

### Prerequisites

- Raspberry Pi (tested on Pi 3/4)
- Python 3.7 or higher
- Two USB serial devices (Console and FluidNC)

### Install Dependencies

```bash
cd /home/pi/alphapaint
pip3 install -r requirements.txt
```

### Configuration

Edit `config.yaml` to adjust settings:

```yaml
machine:
  pen_z_default: 0.5        # Default pen contact height (mm)
  jog_feedrate: 1000        # Jogging speed (mm/min)
  draw_feedrate: 500        # Drawing speed (mm/min)
  pen_lift_feedrate: 100    # Pen Z movement speed (mm/min)

drawing:
  ellipse_segments: 36      # Segments for ellipse approximation

logging:
  level: INFO               # DEBUG, INFO, WARNING, ERROR
  file: /var/log/alphapaint-daemon.log
```

## Running

### Manual Start

```bash
cd /home/pi/alphapaint
python3 daemon.py config.yaml
```

### Auto-Start on Boot (systemd)

1. Copy service file:
```bash
sudo cp alphapaint-daemon.service /etc/systemd/system/
```

2. Enable and start service:
```bash
sudo systemctl enable alphapaint-daemon
sudo systemctl start alphapaint-daemon
```

3. Check status:
```bash
sudo systemctl status alphapaint-daemon
```

4. View logs:
```bash
sudo journalctl -u alphapaint-daemon -f
```

## Usage Workflow

1. **Power on** - Daemon starts automatically and detects devices
2. **Press A (short)** - Start homing sequence
   - LED A blinks fast during homing
   - LED A solid when homing complete
3. **Jog to corner 1** - Use handwheel to position
4. **Press B** - Store first canvas corner
5. **Jog to corner 2** - Move to opposite corner
6. **Press C** - Store second canvas corner
   - LEDs E and F start blinking (drawing ready)
7. **Optional: Set pen Z** - Jog Z to pen contact position, press D
8. **Press E** - Draw line from B to C
9. **Press F** - Draw ellipse inscribed in B-C rectangle

### Re-homing

**Long press A** from any state to re-home the machine.

## Architecture

```
AlphaPaint Daemon
├── daemon.py              # Main entry point
├── config.yaml           # Configuration
├── lib/
│   ├── __init__.py
│   ├── console.py        # Console handler
│   ├── fluidnc.py        # FluidNC handler
│   ├── state_machine.py  # State machine logic
│   └── drawing.py        # Drawing functions
└── requirements.txt      # Python dependencies
```

## State Machine

```
STARTUP → NOT_HOMED → HOMING → HOMED → CANVAS_SETUP → READY → DRAWING
                ↑                                                  ↓
                └──────────────── (Long press A) ─────────────────┘
```

## Troubleshooting

### Devices not detected

Check connected USB devices:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Test serial port permissions:
```bash
sudo usermod -a -G dialout pi
# Logout and login again
```

### Daemon not starting

Check service status:
```bash
sudo systemctl status alphapaint-daemon
```

View detailed logs:
```bash
sudo journalctl -u alphapaint-daemon -n 50
```

### Homing fails

- Check FluidNC is properly configured
- Verify machine limits in FluidNC settings (`$$`)
- Check limit switches are working

### Drawing errors

- Ensure canvas points B and C are defined
- Verify pen Z position is correct
- Check machine is homed before drawing

## Development

### Running in debug mode

```bash
# Edit config.yaml
logging:
  level: DEBUG

# Run daemon
python3 daemon.py config.yaml
```

### Testing without hardware

You can test the daemon logic without actual hardware by using virtual serial ports or by modifying the device identification functions.

## License

See project root for license information.

## Related Documentation

- [Console Specification](../Console/SPECIFICATION.md)
- [Console Integration Guide](../Console/INTEGRATION_GUIDE.md)
- [Daemon Specification](SPECIFICATION.md)
