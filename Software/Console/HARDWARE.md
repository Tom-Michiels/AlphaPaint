# AlphaPaint Console - Hardware Specification and Programming Guide

## Document Overview

This document provides complete hardware specifications and programming details for the AlphaPaint Console. It combines pin assignments, electrical characteristics, initialization procedures, and code examples extracted from the working test implementation.

**Version:** 1.0  
**Target Hardware:** ESP32 38-pin Development Board  
**Firmware Framework:** ESP-IDF

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [Component Details](#2-component-details)
3. [Pin Configuration](#3-pin-configuration)
4. [GPIO Summary Table](#4-gpio-summary-table)
5. [Programming Guide](#5-programming-guide)
6. [Hardware PCNT for Handwheel](#6-hardware-pcnt-for-handwheel)
7. [TM1637 Display Programming](#7-tm1637-display-programming)
8. [I2C LED Expander](#8-i2c-led-expander)
9. [Button Debouncing](#9-button-debouncing)
10. [Testing and Validation](#10-testing-and-validation)

---

## 1. Hardware Overview

### 1.1 System Architecture

The AlphaPaint Console is an ESP32-based control interface providing:

- **Input**: 1 rotary encoder + 10 pushbuttons
- **Output**: 3 × 6-digit displays + 10 LEDs
- **Communication**: USB-UART to Raspberry Pi controller

```
┌─────────────────────────────────────────┐
│           ESP32 38-Pin Board            │
│                                         │
│  ┌────────┐  ┌──────┐  ┌────────────┐  │
│  │ PCNT   │  │ I2C  │  │ TM1637 ×3  │  │
│  │Encoder │  │LEDs  │  │ Displays   │  │
│  └────────┘  └──────┘  └────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │   GPIO Buttons (×10)             │  │
│  │   GPIO LEDs (×3)                 │  │
│  └──────────────────────────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │   UART ──► Raspberry Pi          │  │
│  └──────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

### 1.2 Key Components

| Component | Quantity | Interface | Purpose |
|-----------|----------|-----------|---------|
| ESP32 (38-pin) | 1 | - | Main microcontroller |
| Rotary Encoder | 1 | GPIO (PCNT) | Handwheel input |
| Push Buttons | 10 | GPIO | User input (X/Y/Z + A-G) |
| PCF8574A | 1 | I2C (0x38) | LED driver (A-G) |
| TM1637 Display | 3 | 2-wire serial | Position display (X/Y/Z) |
| Axis LEDs | 3 | GPIO | Active axis indicator |

---

## 2. Component Details

### 2.1 Microcontroller: ESP32 38-Pin Development Board

**Specifications:**
- Dual-core Xtensa LX6 @ 240 MHz
- 520 KB SRAM
- WiFi + Bluetooth (not used in this application)
- 34 GPIO pins (some input-only)
- Hardware peripherals: I2C, UART, PCNT (Pulse Counter)

**Power Requirements:**
- Supply: 5V via USB
- Current: ~200 mA typical, 500 mA max
- 3.3V GPIO logic levels

### 2.2 Rotary Encoder (Handwheel)

**Type:** Quadrature encoder (mechanical with detents)

**Electrical:**
- 2 channels (A and B)
- Pull-up resistors required (10kΩ recommended)
- Output: Open collector / switch to ground

**Mechanical:**
- Typical: 20-24 pulses per revolution
- Detents: Mechanical click per position
- Quadrature encoding: 4 states per detent

**Programming:**
- Uses ESP32 hardware PCNT (Pulse Counter) peripheral
- Automatic quadrature decoding
- No software polling needed
- 4x resolution (4 counts per physical detent)

### 2.3 Push Buttons

**Type:** Tactile momentary switches

**Electrical:**
- Active LOW (pressed = GND, released = 3.3V)
- Internal pull-ups available on most GPIOs
- External pull-up (10kΩ) required for GPIO 34-39 (input-only pins)

**Debouncing:**
- Hardware: Optional 100nF capacitor across switch
- Software: Implemented via state machine (see Button Debouncing section)

### 2.4 PCF8574A I2C GPIO Expander

**Purpose:** Drive 7 LEDs (A-G)

**Specifications:**
- 8-bit I/O expander
- I2C address: 0x38 (A2=1, A1=1, A0=0)
- Max current per pin: 25 mA
- Logic: Active LOW for LEDs (0=ON, 1=OFF)

**LED Mapping (Reversed):**
```
PCF8574A Bit:  7   6   5   4   3   2   1   0
LED:          N/C  A   B   C   D   E   F   G
```

### 2.5 TM1637 6-Digit 7-Segment Display

**Purpose:** Position readout for X, Y, Z axes

**Specifications:**
- 6 digits per module
- Built-in controller + LED driver
- 2-wire interface (DIO + CLK)
- Brightness control: 8 levels
- Decimal point support

**Display Configuration:**
- 3 independent modules (X, Y, Z)
- Each has dedicated DIO and CLK pins
- Display format: `XXXX.XX` (4 digits + 2 decimals)

**Segment Encoding:**
- Common cathode display
- Bit 7: Decimal point
- Bits 6-0: Segments (A-G)

---

## 3. Pin Configuration

### 3.1 Handwheel Encoder

| Signal | GPIO | Direction | Pull-up | Notes |
|--------|------|-----------|---------|-------|
| Encoder A | 34 | Input | External | Input-only GPIO |
| Encoder B | 39 | Input | External | Input-only GPIO |

**Configuration Code:**
```c
#define HANDWHEEL_A_PIN 34
#define HANDWHEEL_B_PIN 39

// GPIO 34 and 39 are input-only, require external pull-ups
// No GPIO configuration needed - PCNT driver handles this
```

### 3.2 Push Buttons

| Button | GPIO | Pull-up | Notes |
|--------|------|---------|-------|
| X | 23 | Internal | - |
| Y | 26 | Internal | - |
| Z | 25 | Internal | - |
| A | 33 | Internal | - |
| B | 0  | Internal | ⚠️ Strapping pin - don't press during boot |
| C | 12 | Internal | ⚠️ Strapping pin - must be LOW for 3.3V flash |
| D | 13 | Internal | - |
| E | 14 | Internal | - |
| F | 36 | External | Input-only GPIO |
| G | 27 | Internal | - |

**Configuration Code:**
```c
// Button F (GPIO36) - input-only, needs external pull-up
gpio_config_t io_conf_input_only = {
    .pin_bit_mask = (1ULL << 36),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,  // Must use external
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
};
gpio_config(&io_conf_input_only);

// All other buttons - can use internal pull-ups
gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << 23) | (1ULL << 26) | (1ULL << 25) |
                    (1ULL << 33) | (1ULL << 0)  | (1ULL << 12) |
                    (1ULL << 13) | (1ULL << 14) | (1ULL << 27),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
};
gpio_config(&io_conf);
```

**Reading Button State:**
```c
int button_state = gpio_get_level(GPIO_PIN);
// Returns: 1 (released/HIGH), 0 (pressed/LOW)
```

### 3.3 Axis LEDs (X, Y, Z)

| LED | GPIO | Connection | Logic |
|-----|------|------------|-------|
| X | 16 | Cathode | Active LOW |
| Y | 4  | Cathode | Active LOW |
| Z | 2  | Cathode | Active LOW |

**Configuration Code:**
```c
#define LED_X_PIN 16
#define LED_Y_PIN 4
#define LED_Z_PIN 2

gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << LED_X_PIN) | (1ULL << LED_Y_PIN) | (1ULL << LED_Z_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
};
gpio_config(&io_conf);

// Turn OFF (Active LOW, so set HIGH)
gpio_set_level(LED_X_PIN, 1);
gpio_set_level(LED_Y_PIN, 1);
gpio_set_level(LED_Z_PIN, 1);

// Turn ON (Active LOW, so set LOW)
gpio_set_level(LED_X_PIN, 0);  // X LED ON
```

### 3.4 I2C GPIO Expander (LEDs A-G)

| Signal | GPIO | Function |
|--------|------|----------|
| SDA | 21 | I2C Data |
| SCL | 22 | I2C Clock |

**I2C Configuration:**
- Address: 0x38
- Speed: 100 kHz (standard mode)
- Pull-ups: Internal enabled (4.7kΩ external recommended)

**Configuration Code:**
```c
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_EXPANDER_ADDR 0x38

i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA_PIN,
    .scl_io_num = I2C_SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
};

i2c_param_config(I2C_MASTER_NUM, &conf);
i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
```

**LED Control:**
```c
// LED mapping: A=bit6, B=bit5, C=bit4, D=bit3, E=bit2, F=bit1, G=bit0
// Active LOW: 0=ON, 1=OFF

// Example: Turn on LED A, turn off all others
uint8_t pattern = 0xFF & ~(1 << 6);  // Clear bit 6 (LED A)

i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, (I2C_EXPANDER_ADDR << 1) | I2C_MASTER_WRITE, true);
i2c_master_write_byte(cmd, pattern, true);
i2c_master_stop(cmd);
i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
i2c_cmd_link_delete(cmd);
```

### 3.5 TM1637 Displays (X, Y, Z)

| Display | DIO GPIO | CLK GPIO |
|---------|----------|----------|
| X-axis | 18 | 19 |
| Y-axis | 15 | 5  |
| Z-axis | 17 | 32 |

**Configuration Code:**
```c
#define DISPLAY_X_DIO_PIN 18
#define DISPLAY_X_CLK_PIN 19
#define DISPLAY_Y_DIO_PIN 15
#define DISPLAY_Y_CLK_PIN 5
#define DISPLAY_Z_DIO_PIN 17
#define DISPLAY_Z_CLK_PIN 32

gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << DISPLAY_X_DIO_PIN) | (1ULL << DISPLAY_X_CLK_PIN) |
                    (1ULL << DISPLAY_Y_DIO_PIN) | (1ULL << DISPLAY_Y_CLK_PIN) |
                    (1ULL << DISPLAY_Z_DIO_PIN) | (1ULL << DISPLAY_Z_CLK_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
};
gpio_config(&io_conf);

// Set all pins HIGH initially
gpio_set_level(DISPLAY_X_DIO_PIN, 1);
gpio_set_level(DISPLAY_X_CLK_PIN, 1);
// ... etc
```

---

## 4. GPIO Summary Table

| GPIO | Function | Dir | Config | Notes |
|------|----------|-----|--------|-------|
| 0  | Button B | In  | Pull-up, Active LOW | ⚠️ Strapping pin |
| 2  | Z LED    | Out | Active LOW | - |
| 4  | Y LED    | Out | Active LOW | - |
| 5  | Y Display CLK | Out | TM1637 | - |
| 12 | Button C | In  | Pull-up, Active LOW | ⚠️ Strapping pin |
| 13 | Button D | In  | Pull-up, Active LOW | - |
| 14 | Button E | In  | Pull-up, Active LOW | - |
| 15 | Y Display DIO | I/O | TM1637 | Bidirectional |
| 16 | X LED    | Out | Active LOW | - |
| 17 | Z Display DIO | I/O | TM1637 | Bidirectional |
| 18 | X Display DIO | I/O | TM1637 | Bidirectional |
| 19 | X Display CLK | Out | TM1637 | - |
| 21 | I2C SDA  | I/O | I2C | Open-drain |
| 22 | I2C SCL  | Out | I2C | Open-drain |
| 23 | Button X | In  | Pull-up, Active LOW | - |
| 25 | Button Z | In  | Pull-up, Active LOW | - |
| 26 | Button Y | In  | Pull-up, Active LOW | - |
| 27 | Button G | In  | Pull-up, Active LOW | - |
| 32 | Z Display CLK | Out | TM1637 | - |
| 33 | Button A | In  | Pull-up, Active LOW | - |
| 34 | Encoder A | In | PCNT | ⚠️ Input-only, ext pull-up |
| 36 | Button F | In  | External pull-up | ⚠️ Input-only |
| 39 | Encoder B | In | PCNT | ⚠️ Input-only, ext pull-up |

**Strapping Pins Warning:**
- **GPIO 0**: Must be HIGH during boot (normal operation). Do not press Button B during reset.
- **GPIO 12**: Must be LOW during boot for 3.3V flash. Button C should not be pressed during reset.

---

## 5. Programming Guide

### 5.1 Initialization Sequence

Recommended initialization order:

```c
void app_main(void) {
    // 1. Initialize handwheel (PCNT)
    init_handwheel();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 2. Initialize buttons
    init_buttons();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 3. Initialize axis LEDs
    init_axis_leds();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 4. Initialize I2C and LED expander
    init_i2c_leds();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 5. Initialize TM1637 displays
    init_displays();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 6. Start main application
    interactive_mode();
}
```

### 5.2 Complete Initialization Functions

See sections 6-8 for detailed initialization code for each peripheral.

---

## 6. Hardware PCNT for Handwheel

### 6.1 PCNT Overview

The ESP32 Pulse Counter (PCNT) peripheral provides hardware quadrature decoding:

- **Channels**: 2 channels per unit (0 and 1)
- **Resolution**: Up to 4x quadrature decoding
- **Range**: -32768 to +32767 counts
- **Speed**: Can track very fast encoders (>1 kHz)

### 6.2 Quadrature Decoding

Quadrature encoding provides direction and relative position:

```
      ┌───┐   ┌───┐   ┌───┐
  A   │   │   │   │   │   │
  ────┘   └───┘   └───┘   └───
    ┌───┐   ┌───┐   ┌───┐
  B │   │   │   │   │   │
  ──┘   └───┘   └───┘   └─────
    
  Clockwise (CW): A leads B
  Counter-CW (CCW): B leads A
```

### 6.3 PCNT Configuration

```c
#define PCNT_UNIT PCNT_UNIT_0
#define HANDWHEEL_A_PIN 34
#define HANDWHEEL_B_PIN 39

void init_handwheel(void) {
    // Configure PCNT Channel 0
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = HANDWHEEL_A_PIN,
        .ctrl_gpio_num = HANDWHEEL_B_PIN,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,       // Count up on rising edge when ctrl=1
        .neg_mode = PCNT_COUNT_DEC,       // Count down on falling edge when ctrl=1
        .lctrl_mode = PCNT_MODE_REVERSE,  // Reverse when ctrl=0
        .hctrl_mode = PCNT_MODE_KEEP,     // Keep when ctrl=1
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    pcnt_unit_config(&pcnt_config);
    
    // Configure PCNT Channel 1 (swap A and B for full quadrature)
    pcnt_config.pulse_gpio_num = HANDWHEEL_B_PIN;
    pcnt_config.ctrl_gpio_num = HANDWHEEL_A_PIN;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    pcnt_unit_config(&pcnt_config);
    
    // Start counting
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}
```

### 6.4 Reading PCNT Value

```c
void update_encoder(void) {
    int16_t pcnt_value;
    pcnt_get_counter_value(PCNT_UNIT, &pcnt_value);
    
    // Calculate delta from last read
    // Quadrature gives 4 counts per detent, so divide by 4
    int delta = (pcnt_value - last_pcnt_value) / 4;
    
    if (delta != 0) {
        position += delta;
        last_pcnt_value += delta * 4;  // Update to nearest multiple of 4
        
        // Update display, etc.
    }
}
```

**Polling Rate:** 20-50 ms recommended (hardware counts in background)

---

## 7. TM1637 Display Programming

### 7.1 TM1637 Protocol

The TM1637 uses a 2-wire synchronous serial protocol (similar to I2C but proprietary):

**Timing:**
- Clock frequency: ~20 kHz typical
- Bit delay: 50 μs
- Clock pulse: 5 μs

**Commands:**
- `0x40`: Data command (auto-increment address)
- `0x80`: Display control (brightness + on/off)
- `0xC0`: Address command (set start address)

### 7.2 Segment Encoding

```c
// Segment mapping (GFEDCBA, bit 7 = decimal point)
//       A
//      ---
//   F |   | B
//      -G-
//   E |   | C
//      ---
//       D    DP

static const uint8_t digit_to_segment[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71  // F
};

// Decimal point: OR with 0x80
// Minus sign: 0x40
```

### 7.3 TM1637 Protocol Functions

```c
#define TM1637_BIT_DELAY 50
#define TM1637_CLK_DELAY 5

void tm1637_start(int dio_pin, int clk_pin) {
    gpio_set_level(dio_pin, 1);
    gpio_set_level(clk_pin, 1);
    ets_delay_us(TM1637_BIT_DELAY);
    gpio_set_level(dio_pin, 0);
    ets_delay_us(TM1637_BIT_DELAY);
}

void tm1637_stop(int dio_pin, int clk_pin) {
    gpio_set_level(clk_pin, 0);
    ets_delay_us(TM1637_BIT_DELAY);
    gpio_set_level(dio_pin, 0);
    ets_delay_us(TM1637_BIT_DELAY);
    gpio_set_level(clk_pin, 1);
    ets_delay_us(TM1637_BIT_DELAY);
    gpio_set_level(dio_pin, 1);
    ets_delay_us(TM1637_BIT_DELAY);
}

void tm1637_write_byte(int dio_pin, int clk_pin, uint8_t data) {
    gpio_set_direction(dio_pin, GPIO_MODE_OUTPUT);
    
    for (int i = 0; i < 8; i++) {
        gpio_set_level(clk_pin, 0);
        ets_delay_us(TM1637_CLK_DELAY);
        
        gpio_set_level(dio_pin, (data >> i) & 0x01);
        ets_delay_us(TM1637_CLK_DELAY);
        
        gpio_set_level(clk_pin, 1);
        ets_delay_us(TM1637_CLK_DELAY);
    }
    
    // Wait for ACK
    gpio_set_level(clk_pin, 0);
    gpio_set_direction(dio_pin, GPIO_MODE_INPUT);
    ets_delay_us(TM1637_CLK_DELAY);
    
    gpio_set_level(clk_pin, 1);
    ets_delay_us(TM1637_CLK_DELAY);
    
    gpio_set_level(clk_pin, 0);
    gpio_set_direction(dio_pin, GPIO_MODE_OUTPUT);
}
```

### 7.4 Display Number Function

```c
void tm1637_display_number(int dio_pin, int clk_pin, int32_t number) {
    uint8_t digits[6] = {0};
    bool negative = false;
    
    if (number < 0) {
        negative = true;
        number = -number;
    }
    
    // Convert to digits
    for (int i = 5; i >= 0; i--) {
        digits[i] = number % 10;
        number /= 10;
    }
    
    // Set data write mode
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x40);  // Auto-increment
    tm1637_stop(dio_pin, clk_pin);
    
    // Write digits (NOTE: display order is reversed in groups of 3)
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0xC0);  // Start address 0
    
    int display_order[6] = {2, 1, 0, 5, 4, 3};  // Reversed groups
    
    for (int pos = 0; pos < 6; pos++) {
        int i = display_order[pos];
        uint8_t segment_data = digit_to_segment[digits[i]];
        
        // Add decimal point at position 3 (before last 2 digits)
        if (i == 3) {
            segment_data |= 0x80;
        }
        
        // Add minus sign if negative
        if (i == 0 && negative) {
            segment_data = 0x40;
        }
        
        tm1637_write_byte(dio_pin, clk_pin, segment_data);
    }
    
    tm1637_stop(dio_pin, clk_pin);
    
    // Set brightness
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x88 | 4);  // Brightness level 4
    tm1637_stop(dio_pin, clk_pin);
}
```

**Note:** The TM1637 6-digit displays have unusual addressing - digits are reversed in groups of 3. See `display_order` array.

---

## 8. I2C LED Expander

### 8.1 PCF8574A Device

The PCF8574A is an 8-bit I/O expander with:
- I2C address: 0x38 (when A2=1, A1=1, A0=0)
- Quasi-bidirectional I/O pins
- Weak pull-ups on all pins
- Max 25 mA per pin

### 8.2 LED Control Pattern

```c
// LED mapping (reversed)
// Bit:   7    6    5    4    3    2    1    0
// LED:  N/C   A    B    C    D    E    F    G

void set_led_state(int led_index, bool on) {
    // led_index: 0=A, 1=B, ..., 6=G
    // on: true=ON, false=OFF
    
    uint8_t pattern = 0xFF;  // All OFF (Active LOW)
    
    if (on) {
        int bit = 6 - led_index;  // Reverse mapping
        pattern &= ~(1 << bit);   // Clear bit to turn ON
    }
    
    // Write to I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, pattern, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
```

### 8.3 Multiple LED Control

```c
void update_all_leds(bool led_states[7]) {
    uint8_t pattern = 0xFF;
    
    for (int i = 0; i < 7; i++) {
        if (led_states[i]) {
            int bit = 6 - i;
            pattern &= ~(1 << bit);
        }
    }
    
    // Single I2C transaction for all LEDs
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x38 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, pattern, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
```

---

## 9. Button Debouncing

### 9.1 Software Debouncing Strategy

Implemented using state machine with time tracking:

```c
typedef struct {
    int pin;
    int last_state;
    uint32_t press_time;
    bool long_press_triggered;
} button_state_t;

button_state_t buttons[10];
const uint32_t LONG_PRESS_TIME_MS = 500;

void poll_buttons(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < 10; i++) {
        int state = gpio_get_level(buttons[i].pin);
        
        // Button pressed (HIGH → LOW)
        if (state == 0 && buttons[i].last_state == 1) {
            buttons[i].press_time = current_time;
            buttons[i].long_press_triggered = false;
        }
        
        // Button held (check for long press)
        else if (state == 0 && buttons[i].last_state == 0) {
            if (!buttons[i].long_press_triggered &&
                (current_time - buttons[i].press_time) >= LONG_PRESS_TIME_MS) {
                buttons[i].long_press_triggered = true;
                handle_long_press(i);
            }
        }
        
        // Button released (LOW → HIGH)
        else if (state == 1 && buttons[i].last_state == 0) {
            if (!buttons[i].long_press_triggered) {
                handle_short_press(i);
            }
        }
        
        buttons[i].last_state = state;
    }
}
```

**Polling Rate:** 50 ms recommended (matches main loop)

### 9.2 Hardware Debouncing (Optional)

Add 100 nF capacitor in parallel with each button:

```
   VCC (3.3V)
      │
     ┌┴┐ 10kΩ
     └┬┘
      ├────────── To GPIO
      │
    ──┴──  100nF
    ──┬──
      │
    [Button]
      │
     GND
```

---

## 10. Testing and Validation

### 10.1 Hardware Checkout Procedure

**Step 1: Visual Inspection**
- Check solder joints
- Verify no shorts between pins
- Check component orientation

**Step 2: Power-On Test**
- Connect USB
- Verify 3.3V rail stable
- Check ESP32 board LED

**Step 3: I/O Test Program**

Use the provided test program in `main/console_test.c`:

```bash
./build.sh
./flash.sh
```

**Expected Behavior:**
1. All displays show "888888" for 1 second
2. Displays clear
3. Interactive mode starts

**Step 4: Component Tests**

| Test | Procedure | Expected Result |
|------|-----------|-----------------|
| Buttons | Press each button A-Z | Log message for each press |
| Handwheel | Rotate encoder | Position increments/decrements |
| Axis LEDs | Press X/Y/Z buttons | Corresponding LED turns on |
| A-G LEDs | Press A-G buttons | LED toggles on/off |
| Displays | Rotate handwheel | Numbers update on active axis |

### 10.2 Common Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Display blank | Wrong pin assignment | Check DIO/CLK pins |
| Display garbled | Timing issue | Verify ets_delay_us() works |
| I2C LEDs not working | Wrong address | Scan I2C bus (code includes scanner) |
| Encoder unreliable | No pull-ups | Add 10kΩ external pull-ups |
| Button stuck | Strapping pin held | Don't press GPIO0/12 during boot |
| Random resets | Power supply | Use good USB cable, check current |

### 10.3 I2C Bus Scan

Built into test program:

```c
for (int addr = 0x03; addr < 0x78; addr++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Found I2C device at 0x%02X", addr);
    }
}
```

Expected result: `Found I2C device at 0x38`

---

## Appendix A: Schematic Reference

### Pin Grouping

**Input Only (require external pull-ups):**
- GPIO 34, 36, 39

**Strapping Pins (avoid during boot):**
- GPIO 0 (must be HIGH)
- GPIO 12 (must be LOW for 3.3V flash)

**Safe for any use:**
- GPIO 2, 4, 5, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33

---

## Appendix B: Code Repository Structure

```
Software/
├── main/
│   └── console_test.c       # Complete working test program
├── HARDWARE.md              # This document
├── SPECIFICATION.md         # Software protocol spec
├── INTEGRATION_GUIDE.md     # Integration guide for Pi
└── build.sh                 # Build script
```

---

**Document Version:** 1.0  
**Last Updated:** 2026-01-05  
**Author:** AlphaPaint Team

---

**End of Hardware Specification**
