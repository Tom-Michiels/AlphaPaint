# AlphaPaint Console - Software Specification v1.1

## 1. Overview

The AlphaPaint Console is an ESP32-based control interface for the AlphaPaint plotter system. It connects to a Raspberry Pi controller via USB (UART) and provides:

- 7 programmable buttons (A-G) with LED feedback
- 3-axis positioning interface (X, Y, Z) with 6-digit displays
- Rotary encoder (handwheel) for precision adjustment
- Bidirectional communication with Raspberry Pi
- Position limit enforcement for safe operation
- Homing state management

## 2. Hardware Components

### 2.1 Input Devices
- **Buttons A-G**: 7 programmable buttons with corresponding LEDs (I2C PCF8574A)
- **Buttons X, Y, Z**: 3 axis selection buttons with individual LEDs (GPIO)
- **Handwheel**: Quadrature rotary encoder on GPIO34/39 (hardware PCNT)

### 2.2 Output Devices
- **LEDs A-G**: I2C LED expander (PCF8574A at address 0x38)
- **LEDs X, Y, Z**: Direct GPIO control (active LOW)
- **Displays X, Y, Z**: Three 6-digit TM1637 displays showing position values

### 2.3 Communication
- **USB-UART**: Primary communication with Raspberry Pi
  - Baud rate: 115200
  - Data bits: 8
  - Parity: None
  - Stop bits: 1
  - Flow control: None

## 3. Operating States and Modes

### 3.1 Homing States

#### 3.1.1 NOT_HOMED State (Initial)
On power-up, the console starts in NOT_HOMED state:

- **Display**: All three displays show blinking "------" (300ms interval)
- **Mode**: Forced to PASSIVE mode
- **Handwheel**: Disabled
- **X/Y/Z Buttons**: Send button events but no local control
- **Active Mode Blocked**: Cannot switch to ACTIVE until homed
- **Exit Condition**: First `POS` command from Pi sets HOMED state

**Purpose:** Indicates machine position is unknown until Raspberry Pi performs homing sequence via FluidNC and sends initial positions.

#### 3.1.2 HOMED State
After receiving first position update from Pi:

- **Display**: Shows numeric position values with decimal point
- **Mode**: Can switch between PASSIVE and ACTIVE modes
- **Position Known**: True machine coordinates established
- **Persistent**: Remains homed until power cycle

### 3.2 Operating Modes (HOMED state only)

#### 3.2.1 Active Mode
In Active Mode, the console operates autonomously (requires HOMED state):

- **Axis Selection**: X/Y/Z buttons select the active axis
- **Handwheel Control**: Rotary encoder adjusts active axis value
- **Precision Mode**:
  - Long press (≥500ms) on active axis button enables precision mode
  - Normal: 1.00 units per click
  - Precision: 0.01 units per click (last 2 digits blink at 150ms)
  - Short press exits precision mode
- **Value Range**: Constrained by axis limits (set via LIMIT command)
- **Limit Enforcement**: Handwheel cannot move position beyond min/max limits
  - Position clamped to limit if reached
  - No overshoot beyond limits
- **Updates to Pi**: Position changes are transmitted to Raspberry Pi

#### 3.2.2 Passive Mode
In Passive Mode, the console acts as a display only:

- **Handwheel**: Disabled (no effect)
- **X/Y/Z Buttons**: Send button press events but don't change active axis
- **Position Display**: Updated by Raspberry Pi commands only
- **No Precision Mode**: Position control disabled
- **No Limit Enforcement**: Pi can set any display value (limits only enforced in Active mode)

## 4. Button Behavior

### 4.1 Buttons A-G

#### Short Press (< 500ms)
- **Action**: Send `BTN:<letter>:SHORT` command to Pi
- **LED Control**: No local LED state change (Pi controls LEDs via LED commands)

#### Long Press (≥ 500ms)
- **Action**: Send `BTN:<letter>:LONG` immediately when threshold reached
- **LED Control**: No local LED state change (Pi controls LEDs via LED commands)
- **Release**: No additional command on release

**Note:** The console does not toggle LED states locally on button press. All LED control is via UART commands from the Pi. This allows the Pi to implement custom button behaviors and LED feedback patterns.

### 4.2 Buttons X, Y, Z (Active Mode - HOMED)

#### Short Press (< 500ms)
- **If not active axis**: Set as active axis, send `AXIS:<axis>:SELECT`
- **If active axis and precision OFF**: Send `AXIS:<axis>:SELECT`
- **If active axis and precision ON**: Disable precision mode, send `AXIS:<axis>:PRECISION:OFF`

#### Long Press (≥ 500ms)
- **If active axis**: Enable precision mode, send `AXIS:<axis>:PRECISION:ON`
- **Release**: No additional command

### 4.3 Buttons X, Y, Z (Passive Mode or NOT_HOMED)

#### Any Press
- Send button event to Pi: `BTN:X:SHORT` or `BTN:X:LONG`
- No local state change
- No handwheel control
- No axis switching

## 5. Display Format

### 5.1 6-Digit Display Layout

#### Normal Operation (HOMED state)
```
XXXX.XX
```
- Decimal point always before last 2 digits
- Leading zero suppression (except shows at least "0.00")
- Range: -9999.99 to 9999.99
- Negative values show minus sign in leftmost position

#### NOT_HOMED State
```
------
```
- All 6 positions show minus character (segment pattern: 0x40)
- Blinks at 300ms interval
- Decimal point not shown
- Indicates machine position is unknown
- Displayed on all three axes simultaneously

### 5.2 Precision Mode Indication
- Last 2 digits (after decimal) blink at 150ms rate
- Only on active axis
- Only in Active mode (HOMED state)

### 5.3 Active Axis Indication
- Corresponding axis LED (X/Y/Z) illuminated
- Only one axis LED active at a time
- Only in Active mode

## 6. LED States

### 6.1 LEDs A-G
Four possible states:
1. **OFF**: LED is off
2. **ON**: LED is continuously on
3. **BLINK**: LED blinks at 500ms interval (slow blink)
4. **FAST_BLINK**: LED blinks at 150ms interval (fast blink)

### 6.2 LEDs X, Y, Z
Two states:
1. **OFF**: Axis is not active (or in Passive mode)
2. **ON**: Axis is currently active (Active mode only)

## 7. Axis Limits

### 7.1 Limit Storage
- Each axis has independent min and max limits
- Stored internally as signed 32-bit integers (units × 100)
- Example: limit 0.00 to 300.00 stored as 0 to 30000
- Default on startup: -999999 to 999999 (-9999.99 to 9999.99)

### 7.2 Limit Setting
- Set via `LIMIT:<axis>:<min>:<max>` command from Pi
- Can be queried via `LIMIT:<axis>?` command
- No limit checking when setting (Pi is trusted)
- Limits persist until power cycle or new LIMIT command

### 7.3 Limit Enforcement
- **Active Mode Only**: Limits enforced during handwheel operation
- **Before Increment**: Check if new value would exceed limit
  - If new_value < min_limit: clamp to min_limit
  - If new_value > max_limit: clamp to max_limit
- **Passive Mode**: No enforcement, Pi can set any display value
- **Clamping Behavior**: Position stops at limit, cannot go beyond

### 7.4 Use Cases
- Prevent jogging beyond machine physical limits
- Software limit switches
- Work envelope definition
- Safety boundaries

## 8. Value Storage and Transmission

### 8.1 Internal Storage
- Values stored as signed 32-bit integers (units × 100)
- Example: 12.34 stored as 1234
- Range: -999999 to 999999 (internal), -9999.99 to 9999.99 (display)
- Limits stored separately per axis (min/max as int32_t × 100)
- Default limits: -999999 to 999999 (full display range)

### 8.2 Handwheel Encoding
- Hardware PCNT quadrature decoder (4x resolution)
- 4 hardware counts per detent
- Normal mode: ±100 units per click (±1.00 displayed)
- Precision mode: ±1 unit per click (±0.01 displayed)
- **Limit Enforcement**:
  - Before applying increment, check if new value would exceed limits
  - If would exceed min: clamp to min value
  - If would exceed max: clamp to max value
  - Only enforced in ACTIVE mode

## 9. State Machine

### 9.1 Console States
```
┌──────────────┐
│     INIT     │
└──────┬───────┘
       │
       v
┌──────────────┐
│  NOT_HOMED   │  Display: "------" blinking (300ms)
│  (PASSIVE)   │  Cannot enter ACTIVE mode
└──────┬───────┘
       │ First POS:<axis>:<value> command from Pi
       │ (any axis triggers HOMED state)
       v
┌──────────────┐
│    HOMED     │  Display: numeric values
│  (PASSIVE)   │  Can switch between modes
└──────┬───────┘
       │
       │  MODE:ACTIVE command allowed
       v
┌─────────────┐     MODE:PASSIVE    ┌──────────────┐
│   ACTIVE    │◄────────────────────►│   PASSIVE    │
│    MODE     │     MODE:ACTIVE      │     MODE     │
│  (HOMED)    │                      │   (HOMED)    │
└─────────────┘                      └──────────────┘
```

**State Descriptions:**
- **NOT_HOMED**: Initial state after power-on
  - Displays blink "------" on all axes
  - Forced PASSIVE mode
  - MODE:ACTIVE command returns ERROR:NOT_HOMED
  - Handwheel disabled
  - Waiting for Pi to send positions after homing

- **HOMED/PASSIVE**: Position known, Pi controls display
  - Can switch to ACTIVE mode
  - Displays show numeric values
  - Handwheel disabled
  - Pi updates positions via POS commands

- **HOMED/ACTIVE**: Position known, handwheel controls position
  - Local axis selection and control
  - Limits enforced
  - Position updates sent to Pi
  - Can switch back to PASSIVE

### 9.2 Axis States (Active Mode Only)
```
┌──────────────┐
│  AXIS:IDLE   │
│ (Not Active) │
└──────┬───────┘
       │ X/Y/Z Short Press
       v
┌──────────────┐     Long Press     ┌──────────────┐
│ AXIS:ACTIVE  │─────────────────→  │ PRECISION    │
│ (Normal)     │                     │    MODE      │
│              │ ←───────────────    │              │
└──────────────┘   Short Press       └──────────────┘
```

## 10. Initialization Sequence

### 10.1 Power-On
1. Initialize hardware peripherals (I2C, GPIO, PCNT, UART)
2. Set state to **NOT_HOMED**
3. Set all displays to blinking "------" (300ms interval)
4. Turn off all LEDs (A-G, X, Y, Z)
5. Set mode to **PASSIVE** (forced until homed)
6. Set active axis to X (for when ACTIVE mode becomes available)
7. Set default limits for all axes: -9999.99 to 9999.99
8. Clear all position values to 0 (internal, not displayed)
9. Send identification string: `CONSOLE:ALPHAPAINT:V1.1`
10. Send status: `STATUS:NOT_HOMED`
11. Enter main loop

### 10.2 Startup Display Test (Optional)
Can be enabled for hardware verification:
- Brief display of "888888" on all displays (1 second)
- Flash all LEDs (3 times, 300ms interval)
- Then enter NOT_HOMED state with "------" blinking

### 10.3 Homing Transition
Triggered by first `POS` command from Pi:
1. Receive `POS:<axis>:<value>` command
2. If in NOT_HOMED state:
   - Set state to HOMED
   - Stop "------" blinking
   - Display received position value
   - Send `STATUS:HOMED`
   - MODE:ACTIVE now allowed (if commanded)
3. Update internal position value
4. Update display

## 11. Error Handling

### 11.1 UART Communication Errors
- **Receive buffer overflow**: Discard oldest data, log warning
- **Invalid command format**: Send `ERROR:INVALID_CMD:<command>`
- **Checksum mismatch** (if implemented): Send `ERROR:CHECKSUM`

### 11.2 Hardware Errors
- **I2C communication failure**: Retry 3 times, then disable LED updates, send `ERROR:I2C_FAILURE`
- **Display communication failure**: Continue operation, log error
- **PCNT overflow**: Reset counter, send `ERROR:PCNT_OVERFLOW`

### 11.3 State/Mode Errors
- **MODE:ACTIVE when NOT_HOMED**: Send `ERROR:NOT_HOMED:MODE:ACTIVE`
- **Invalid mode**: Send `ERROR:INVALID_MODE:<mode>`

### 11.4 Value Range Errors
- **Position overflow**: Clamp to ±9999.99 (display limit)
- **Invalid value from Pi**: Send `ERROR:VALUE_RANGE:<value>`
- **Limit beyond range**: Send `ERROR:VALUE_RANGE:LIMIT:<axis>:<value>`
- **Invalid limit (min > max)**: Send `ERROR:INVALID_LIMIT:<axis>:<min>:<max>`

## 12. Performance Requirements

### 12.1 Response Times
- Button press detection: < 50ms
- UART command processing: < 10ms
- Display update: < 100ms
- LED update: < 10ms
- State transition (NOT_HOMED → HOMED): < 50ms

### 12.2 Update Rates
- Main loop: 50ms (20 Hz)
- Display blink (precision): 150ms toggle
- LED blink (BLINK mode): 500ms toggle
- LED fast blink (FAST_BLINK mode): 150ms toggle
- NOT_HOMED display blink: 300ms toggle
- Position transmission: On change only (max 20 Hz, throttled to 50ms minimum interval)

### 12.3 UART Throughput
- Expected: < 100 messages/second
- Buffer size: 256 bytes (RX), 256 bytes (TX)

## 13. Memory Requirements

### 13.1 RAM Usage (Estimated)
- Global variables: ~250 bytes (added limit storage)
- UART buffers: 512 bytes
- FreeRTOS stack: ~4KB
- Total: < 5KB

### 13.2 Flash Usage (Estimated)
- Application code: ~60KB (added state management)
- ESP-IDF libraries: ~200KB
- Total: < 300KB (plenty of room on ESP32)

## 14. Configuration Parameters

### 14.1 Compile-Time Constants
```c
#define LONG_PRESS_TIME_MS        500    // Long press threshold
#define DISPLAY_BLINK_MS          150    // Fast blink rate (FAST_BLINK, precision mode)
#define LED_BLINK_MS              500    // Slow blink rate (BLINK mode)
#define NOT_HOMED_BLINK_MS        300    // NOT_HOMED display blink rate
#define UART_BAUD_RATE            115200 // Serial communication
#define MAIN_LOOP_DELAY_MS        50     // Main loop period
#define POSITION_UPDATE_RATE_MS   50     // Throttle limit for position updates
#define ENCODER_COUNTS_PER_CLICK  4      // PCNT quadrature resolution
#define NORMAL_INCREMENT          100    // Normal mode step (1.00)
#define PRECISION_INCREMENT       1      // Precision mode step (0.01)
#define DEFAULT_LIMIT_MIN         -999999 // Default min limit (internal units)
#define DEFAULT_LIMIT_MAX         999999  // Default max limit (internal units)
#define DISPLAY_LIMIT_MIN         -999999 // Display min (-9999.99)
#define DISPLAY_LIMIT_MAX         999999  // Display max (9999.99)
```

### 14.2 Runtime Configuration
Currently all parameters are compile-time. Future versions could support:
- Adjustable blink rates via UART commands
- Configurable long press threshold
- Custom button mappings
- Persistent limit storage (EEPROM/NVS)

## 15. Testing and Validation

### 15.1 Unit Tests Required
- Button press detection (short/long)
- UART command parsing (including LIMIT commands)
- Value range checking and limit enforcement
- Display formatting (numeric and "------")
- LED state management
- State transitions (NOT_HOMED → HOMED)
- Mode switching validation

### 15.2 Integration Tests
- Full button sequence testing
- Handwheel accuracy over range
- Handwheel limit enforcement (should stop at limits)
- Mode switching (Active ↔ Passive)
- Homing sequence (NOT_HOMED → HOMED)
- Pi communication protocol
- Concurrent operations (multiple buttons, encoder, UART)
- Limit setting and querying

### 15.3 Acceptance Criteria
- All buttons respond within 50ms
- Position accuracy: ±0.01 units
- Handwheel respects limits (cannot exceed)
- "------" display on startup
- Cannot enter ACTIVE mode until homed
- No lost UART messages under normal load
- Stable operation for 24+ hours
- Clean display updates (no flicker)

## 16. Future Enhancements

### 16.1 Potential Features
- [ ] EEPROM/NVS storage of limits and last position
- [ ] Configurable button actions
- [ ] Multiple axis profiles
- [ ] Display brightness control via UART
- [ ] Sleep/power saving mode
- [ ] Emergency stop button support
- [ ] Multi-language display support
- [ ] Diagnostic mode
- [ ] Soft limits warning (before hard limit)
- [ ] Feed rate override control

### 16.2 Protocol Extensions
- [ ] Binary protocol option (higher efficiency)
- [ ] Command acknowledgment/handshaking (optional CRC)
- [ ] Firmware update over UART
- [ ] Status query commands
- [ ] Event subscriptions
- [ ] Home position storage (G28/G30 equivalents)

## 17. Safety Considerations

### 17.1 Limit Protection
- Limits prevent jogging beyond defined boundaries
- Software limits complement hardware limit switches
- Pi should set appropriate limits based on machine configuration
- Limits only enforced in ACTIVE mode (Pi trusted in PASSIVE)

### 17.2 Homing Requirement
- Machine must be homed before manual control (ACTIVE mode)
- Prevents accidental movement before position is known
- Pi responsible for homing sequence via FluidNC

### 17.3 Communication Loss
- Console continues in last mode if UART connection lost
- ACTIVE mode: Handwheel still functional (limits enforced)
- PASSIVE mode: Display frozen
- No automatic fallback to prevent unexpected behavior

## 18. Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-04 | Initial | First specification |
| 1.1 | 2026-01-05 | Update | Added NOT_HOMED state, axis limits (LIMIT command), homing requirement, expanded error handling |

---

**End of Specification Document**
