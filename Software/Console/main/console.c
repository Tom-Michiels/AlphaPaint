/*
 * AlphaPaint Console Firmware v1.2
 *
 * Production firmware implementing the full specification:
 * - Homing state management (NOT_HOMED/HOMED)
 * - Operating modes (PASSIVE/ACTIVE)
 * - Axis limits with enforcement
 * - UART communication protocol
 * - Button handling with short/long press
 * - Display management with blinking states
 *
 * Hardware drivers preserved from console_test_reference.c
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/pcnt.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "rom/ets_sys.h"  // For ets_delay_us

static const char *TAG = "CONSOLE";

// ============================================================================
// HARDWARE CONFIGURATION (from HARDWARE.md)
// ============================================================================

// PCNT unit for handwheel
#define PCNT_UNIT PCNT_UNIT_0

// Pin Definitions
#define HANDWHEEL_A_PIN     34
#define HANDWHEEL_B_PIN     39

// Button Pins (Active LOW with pull-up)
#define BTN_X_PIN           23
#define BTN_Y_PIN           26
#define BTN_Z_PIN           25
#define BTN_A_PIN           33
#define BTN_B_PIN           0
#define BTN_C_PIN           12
#define BTN_D_PIN           13
#define BTN_E_PIN           14
#define BTN_F_PIN           36
#define BTN_G_PIN           27

// Axis LED Pins (Active LOW)
#define LED_X_PIN           16
#define LED_Y_PIN           4
#define LED_Z_PIN           2

// I2C Configuration
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_EXPANDER_ADDR   0x38

// TM1637 Display Pins
#define DISPLAY_X_DIO_PIN   18
#define DISPLAY_X_CLK_PIN   19
#define DISPLAY_Y_DIO_PIN   15
#define DISPLAY_Y_CLK_PIN   5
#define DISPLAY_Z_DIO_PIN   17
#define DISPLAY_Z_CLK_PIN   32

// UART Configuration
#define UART_NUM            UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       1024  // Increased from 256 for fast handwheel updates

// ============================================================================
// TIMING CONSTANTS (from SPECIFICATION.md)
// ============================================================================

#define LONG_PRESS_TIME_MS        500
#define DISPLAY_BLINK_MS          150  // Fast blink for FAST_BLINK and precision mode
#define LED_BLINK_MS              500  // Slow blink for BLINK mode (was 300ms)
#define NOT_HOMED_BLINK_MS        300
#define POSITION_UPDATE_RATE_MS   50   // Rate limit for UART position updates
#define MAIN_LOOP_TICKS           1    // One RTOS tick (10 ms at 100 Hz); lets the idle task run
#define BUTTON_DEBOUNCE_MS        30   // Button level must be stable this long
#define PCNT_RECENTER_LIMIT       16000 // Re-center the encoder counter well before +-32767
#define ENCODER_COUNTS_PER_CLICK  4
#define NORMAL_INCREMENT          100
#define PRECISION_INCREMENT       1
#define DEFAULT_LIMIT_MIN         -999999
#define DEFAULT_LIMIT_MAX         999999

// TM1637 timing
#define TM1637_BIT_DELAY    50
#define TM1637_CLK_DELAY    5

// ============================================================================
// STATE DEFINITIONS
// ============================================================================

typedef enum {
    HOMING_STATE_NOT_HOMED,
    HOMING_STATE_HOMED
} homing_state_t;

typedef enum {
    MODE_PASSIVE,
    MODE_ACTIVE
} operating_mode_t;

typedef enum {
    LED_STATE_OFF,
    LED_STATE_ON,
    LED_STATE_BLINK,
    LED_STATE_FAST_BLINK
} led_state_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================

// System state
static homing_state_t homing_state = HOMING_STATE_NOT_HOMED;
static operating_mode_t operating_mode = MODE_PASSIVE;

// Axis data
static int active_axis = 0;  // 0=X, 1=Y, 2=Z
static int32_t axis_positions[3] = {0, 0, 0};  // Stored as units × 100
static int32_t axis_limits_min[3] = {DEFAULT_LIMIT_MIN, DEFAULT_LIMIT_MIN, DEFAULT_LIMIT_MIN};
static int32_t axis_limits_max[3] = {DEFAULT_LIMIT_MAX, DEFAULT_LIMIT_MAX, DEFAULT_LIMIT_MAX};
static bool precision_mode[3] = {false, false, false};

// LED states (A-G)
static led_state_t led_states[7] = {LED_STATE_OFF, LED_STATE_OFF, LED_STATE_OFF,
                                     LED_STATE_OFF, LED_STATE_OFF, LED_STATE_OFF, LED_STATE_OFF};

// Encoder state
static int16_t last_pcnt_value = 0;

// Position update throttling
static uint32_t last_position_sent_time = 0;
static bool pending_position_update = false;
static int pending_position_axis = 0;

// Button state tracking
typedef struct {
    int pin;
    int last_state;            // Debounced level
    int last_raw;              // Raw level at the previous poll
    uint32_t raw_change_time;  // When the raw level last changed
    uint32_t press_time;
    bool long_press_triggered;
} button_state_t;

static button_state_t buttons[10];

// Timing state
static uint32_t last_display_blink_time = 0;
static bool display_blink_state = false;
static uint32_t last_led_blink_time = 0;
static bool led_blink_state = false;
static uint32_t last_not_homed_blink_time = 0;
static bool not_homed_blink_state = false;

// ============================================================================
// TM1637 DISPLAY DRIVER (from console_test_reference.c)
// ============================================================================

// Digit encoding for 0-9, A-F
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

void tm1637_set_brightness(int dio_pin, int clk_pin, uint8_t brightness) {
    uint8_t cmd = 0x80;
    if (brightness < 8) {
        cmd |= 0x08 | (brightness & 0x07);
    }

    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, cmd);
    tm1637_stop(dio_pin, clk_pin);
}

// Display "------" for NOT_HOMED state
void tm1637_display_dashes(int dio_pin, int clk_pin, bool visible) {
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x40);
    tm1637_stop(dio_pin, clk_pin);

    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0xC0);

    uint8_t segment_data = visible ? 0x40 : 0x00;  // 0x40 = minus sign
    for (int i = 0; i < 6; i++) {
        tm1637_write_byte(dio_pin, clk_pin, segment_data);
    }

    tm1637_stop(dio_pin, clk_pin);
    tm1637_set_brightness(dio_pin, clk_pin, 4);
}

// Display number with decimal point and optional blinking
void tm1637_display_number_decimal(int dio_pin, int clk_pin, int32_t number,
                                   bool leading_zero, int decimal_pos, bool blink_last_two) {
    uint8_t digits[6] = {0, 0, 0, 0, 0, 0};
    bool negative = false;

    if (number < 0) {
        negative = true;
        number = -number;
    }

    // Convert number to digits
    for (int i = 5; i >= 0; i--) {
        digits[i] = number % 10;
        number /= 10;
    }

    // Set data write mode
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x40);
    tm1637_stop(dio_pin, clk_pin);

    // Write digits - reversed in groups of 3
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0xC0);

    int display_order[6] = {2, 1, 0, 5, 4, 3};

    // Find first non-zero digit
    int first_nonzero = 5;
    if (!leading_zero) {
        for (int i = 0; i < 6; i++) {
            if (digits[i] != 0) {
                first_nonzero = i;
                break;
            }
        }
        if (decimal_pos >= 0 && first_nonzero > decimal_pos) {
            first_nonzero = decimal_pos;
        }
    }

    for (int pos = 0; pos < 6; pos++) {
        int i = display_order[pos];
        uint8_t segment_data = 0;

        // Check if blinking should hide these digits
        bool hide_for_blink = blink_last_two && (i == 4 || i == 5);

        if (hide_for_blink) {
            segment_data = 0x00;
        } else if (i == 0 && negative) {
            segment_data = 0x40;  // Minus sign
        } else if (leading_zero || i >= first_nonzero) {
            segment_data = digit_to_segment[digits[i]];
            if (decimal_pos >= 0 && i == decimal_pos) {
                segment_data |= 0x80;  // Add decimal point
            }
        } else {
            segment_data = 0x00;  // Blank
        }

        tm1637_write_byte(dio_pin, clk_pin, segment_data);
    }

    tm1637_stop(dio_pin, clk_pin);
    tm1637_set_brightness(dio_pin, clk_pin, 4);
}

void tm1637_clear(int dio_pin, int clk_pin) {
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x40);
    tm1637_stop(dio_pin, clk_pin);

    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0xC0);
    for (int i = 0; i < 6; i++) {
        tm1637_write_byte(dio_pin, clk_pin, 0x00);
    }
    tm1637_stop(dio_pin, clk_pin);
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void init_handwheel(void) {
    ESP_LOGI(TAG, "Initializing handwheel encoder with PCNT...");

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = HANDWHEEL_A_PIN,
        .ctrl_gpio_num = HANDWHEEL_B_PIN,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    pcnt_config.pulse_gpio_num = HANDWHEEL_B_PIN;
    pcnt_config.ctrl_gpio_num = HANDWHEEL_A_PIN;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT));

    ESP_LOGI(TAG, "Handwheel initialized");
}

void init_buttons(void) {
    ESP_LOGI(TAG, "Initializing buttons...");

    // GPIO 36 (F button) - input only, needs external pull-up
    gpio_config_t io_conf_input_only = {
        .pin_bit_mask = (1ULL << BTN_F_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf_input_only);

    // Other buttons with internal pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_X_PIN) | (1ULL << BTN_Y_PIN) |
                       (1ULL << BTN_Z_PIN) | (1ULL << BTN_A_PIN) |
                       (1ULL << BTN_B_PIN) | (1ULL << BTN_C_PIN) |
                       (1ULL << BTN_D_PIN) | (1ULL << BTN_E_PIN) |
                       (1ULL << BTN_G_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // Initialize button state array
    int button_pins[] = {BTN_X_PIN, BTN_Y_PIN, BTN_Z_PIN, BTN_A_PIN, BTN_B_PIN,
                         BTN_C_PIN, BTN_D_PIN, BTN_E_PIN, BTN_F_PIN, BTN_G_PIN};
    for (int i = 0; i < 10; i++) {
        buttons[i].pin = button_pins[i];
        buttons[i].last_state = gpio_get_level(button_pins[i]);
        buttons[i].last_raw = buttons[i].last_state;
        buttons[i].raw_change_time = 0;
        buttons[i].press_time = 0;
        buttons[i].long_press_triggered = false;
    }

    ESP_LOGI(TAG, "Buttons initialized");
}

void init_axis_leds(void) {
    ESP_LOGI(TAG, "Initializing axis LEDs...");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_X_PIN) | (1ULL << LED_Y_PIN) | (1ULL << LED_Z_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // Turn all LEDs off (Active LOW)
    gpio_set_level(LED_X_PIN, 1);
    gpio_set_level(LED_Y_PIN, 1);
    gpio_set_level(LED_Z_PIN, 1);

    ESP_LOGI(TAG, "Axis LEDs initialized");
}

void init_i2c_leds(void) {
    ESP_LOGI(TAG, "Initializing I2C LED expander...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // Turn all LEDs off (Active LOW, 0xFF = all off)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_EXPANDER_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "I2C LED expander initialized");
}

void init_displays(void) {
    ESP_LOGI(TAG, "Initializing TM1637 displays...");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DISPLAY_X_DIO_PIN) | (1ULL << DISPLAY_X_CLK_PIN) |
                       (1ULL << DISPLAY_Y_DIO_PIN) | (1ULL << DISPLAY_Y_CLK_PIN) |
                       (1ULL << DISPLAY_Z_DIO_PIN) | (1ULL << DISPLAY_Z_CLK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    // Set all pins high initially
    gpio_set_level(DISPLAY_X_DIO_PIN, 1);
    gpio_set_level(DISPLAY_X_CLK_PIN, 1);
    gpio_set_level(DISPLAY_Y_DIO_PIN, 1);
    gpio_set_level(DISPLAY_Y_CLK_PIN, 1);
    gpio_set_level(DISPLAY_Z_DIO_PIN, 1);
    gpio_set_level(DISPLAY_Z_CLK_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Clear all displays
    tm1637_clear(DISPLAY_X_DIO_PIN, DISPLAY_X_CLK_PIN);
    tm1637_clear(DISPLAY_Y_DIO_PIN, DISPLAY_Y_CLK_PIN);
    tm1637_clear(DISPLAY_Z_DIO_PIN, DISPLAY_Z_CLK_PIN);

    ESP_LOGI(TAG, "Displays initialized");
}

void init_uart(void) {
    ESP_LOGI(TAG, "Initializing UART...");

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));

    ESP_LOGI(TAG, "UART initialized at %d baud", UART_BAUD_RATE);
}

// ============================================================================
// UART COMMUNICATION
// ============================================================================

void uart_send_string(const char *str) {
    uart_write_bytes(UART_NUM, str, strlen(str));
    uart_write_bytes(UART_NUM, "\n", 1);
}

void uart_send_button_event(const char *button, const char *type) {
    char buf[32];
    snprintf(buf, sizeof(buf), "BTN:%s:%s", button, type);
    uart_send_string(buf);
    ESP_LOGI(TAG, "TX: %s", buf);
}

void uart_send_axis_event(const char *axis, const char *event) {
    char buf[32];
    snprintf(buf, sizeof(buf), "AXIS:%s:%s", axis, event);
    uart_send_string(buf);
    ESP_LOGI(TAG, "TX: %s", buf);
}

// Format a value stored as hundredths. Keeps the sign for -0.99..-0.01,
// which "%d.%02d" of value/100 would lose.
static void format_centi(char *buf, size_t len, int32_t value) {
    int32_t a = value < 0 ? -value : value;
    snprintf(buf, len, "%s%d.%02d", value < 0 ? "-" : "", (int)(a / 100), (int)(a % 100));
}

// Parse "123.45" into hundredths, rounding instead of truncating
static int32_t parse_centi(const char *str) {
    return (int32_t)lroundf(atof(str) * 100.0f);
}

void uart_send_position(const char *axis, int32_t value) {
    char num[16];
    char buf[32];
    format_centi(num, sizeof(num), value);
    snprintf(buf, sizeof(buf), "POS:%s:%s", axis, num);
    uart_send_string(buf);
}

void uart_send_status(const char *status) {
    char buf[32];
    snprintf(buf, sizeof(buf), "STATUS:%s", status);
    uart_send_string(buf);
    ESP_LOGI(TAG, "TX: %s", buf);
}

void uart_send_error(const char *error) {
    char buf[64];
    snprintf(buf, sizeof(buf), "ERROR:%s", error);
    uart_send_string(buf);
    ESP_LOGI(TAG, "TX: %s", buf);
}

void uart_send_limit_response(const char *axis, int32_t min, int32_t max) {
    char min_str[16], max_str[16];
    char buf[64];
    format_centi(min_str, sizeof(min_str), min);
    format_centi(max_str, sizeof(max_str), max);
    snprintf(buf, sizeof(buf), "LIMIT:%s:%s:%s", axis, min_str, max_str);
    uart_send_string(buf);
    ESP_LOGI(TAG, "TX: %s", buf);
}

// Forward declarations
void update_display(int axis);
void update_axis_leds(void);
void send_pending_position_update(uint32_t current_time);

// Parse incoming UART commands
void process_uart_command(const char *cmd) {
    char cmd_copy[128];
    strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';

    ESP_LOGI(TAG, "RX: %s", cmd_copy);

    // Tokenize command
    char *token = strtok(cmd_copy, ":");
    if (token == NULL) {
        uart_send_error("INVALID_CMD");
        return;
    }

    // Identification query from the Pi
    if (strcmp(token, "ID?") == 0) {
        uart_send_string("CONSOLE:ALPHAPAINT:V1.2");
    }
    // MODE command
    else if (strcmp(token, "MODE") == 0) {
        token = strtok(NULL, ":");
        if (token == NULL) {
            uart_send_error("INVALID_CMD:MODE");
            return;
        }

        if (strcmp(token, "ACTIVE") == 0) {
            if (homing_state == HOMING_STATE_NOT_HOMED) {
                uart_send_error("NOT_HOMED:MODE:ACTIVE");
            } else {
                operating_mode = MODE_ACTIVE;
                update_axis_leds();  // Update LEDs when entering ACTIVE mode
                ESP_LOGI(TAG, "Mode changed to ACTIVE");
            }
        } else if (strcmp(token, "PASSIVE") == 0) {
            operating_mode = MODE_PASSIVE;
            update_axis_leds();  // Turn off axis LEDs when entering PASSIVE mode
            ESP_LOGI(TAG, "Mode changed to PASSIVE");
        } else {
            uart_send_error("INVALID_MODE");
        }
    }
    // POS command
    else if (strcmp(token, "POS") == 0) {
        char *axis_str = strtok(NULL, ":");
        char *value_str = strtok(NULL, ":");

        if (axis_str == NULL || value_str == NULL) {
            uart_send_error("INVALID_CMD:POS");
            return;
        }

        // Parse value (format: 123.45 or -123.45)
        int32_t value = parse_centi(value_str);

        // Determine which axis
        int axis_index = -1;
        if (strcmp(axis_str, "X") == 0) axis_index = 0;
        else if (strcmp(axis_str, "Y") == 0) axis_index = 1;
        else if (strcmp(axis_str, "Z") == 0) axis_index = 2;
        else {
            uart_send_error("INVALID_AXIS");
            return;
        }

        // First position command transitions from NOT_HOMED to HOMED
        if (homing_state == HOMING_STATE_NOT_HOMED) {
            homing_state = HOMING_STATE_HOMED;
            uart_send_status("HOMED");
            ESP_LOGI(TAG, "Homing complete - state changed to HOMED");
        }

        axis_positions[axis_index] = value;
        update_display(axis_index);  // Update display when position changes
        ESP_LOGD(TAG, "Position %s set to %ld", axis_str, (long)value);
    }
    // LED command
    else if (strcmp(token, "LED") == 0) {
        char *led_str = strtok(NULL, ":");
        char *state_str = strtok(NULL, ":");

        if (led_str == NULL || state_str == NULL) {
            uart_send_error("INVALID_CMD:LED");
            return;
        }

        // Parse LED (A-G)
        if (strlen(led_str) != 1 || led_str[0] < 'A' || led_str[0] > 'G') {
            uart_send_error("INVALID_LED");
            return;
        }
        int led_index = led_str[0] - 'A';

        // Parse state
        if (strcmp(state_str, "OFF") == 0) {
            led_states[led_index] = LED_STATE_OFF;
        } else if (strcmp(state_str, "ON") == 0) {
            led_states[led_index] = LED_STATE_ON;
        } else if (strcmp(state_str, "BLINK") == 0) {
            led_states[led_index] = LED_STATE_BLINK;
        } else if (strcmp(state_str, "FAST_BLINK") == 0) {
            led_states[led_index] = LED_STATE_FAST_BLINK;
        } else {
            uart_send_error("INVALID_LED_STATE");
            return;
        }

        ESP_LOGI(TAG, "LED %c set to %s", led_str[0], state_str);
    }
    // LIMIT command
    else if (strcmp(token, "LIMIT") == 0) {
        char *axis_str = strtok(NULL, ":");

        if (axis_str == NULL) {
            uart_send_error("INVALID_CMD:LIMIT");
            return;
        }

        // Determine axis
        int axis_index = -1;
        if (strcmp(axis_str, "X") == 0) axis_index = 0;
        else if (strcmp(axis_str, "Y") == 0) axis_index = 1;
        else if (strcmp(axis_str, "Z") == 0) axis_index = 2;
        else if (strcmp(axis_str, "X?") == 0) axis_index = 0;
        else if (strcmp(axis_str, "Y?") == 0) axis_index = 1;
        else if (strcmp(axis_str, "Z?") == 0) axis_index = 2;
        else {
            uart_send_error("INVALID_AXIS");
            return;
        }

        // Check if query
        if (strchr(axis_str, '?') != NULL) {
            const char *axis_name = (axis_index == 0) ? "X" : (axis_index == 1) ? "Y" : "Z";
            uart_send_limit_response(axis_name, axis_limits_min[axis_index], axis_limits_max[axis_index]);
            return;
        }

        // Parse min and max values
        char *min_str = strtok(NULL, ":");
        char *max_str = strtok(NULL, ":");

        if (min_str == NULL || max_str == NULL) {
            uart_send_error("INVALID_CMD:LIMIT");
            return;
        }

        int32_t min_val = parse_centi(min_str);
        int32_t max_val = parse_centi(max_str);

        // Validate
        if (min_val > max_val) {
            uart_send_error("INVALID_LIMIT");
            return;
        }

        axis_limits_min[axis_index] = min_val;
        axis_limits_max[axis_index] = max_val;

        const char *axis_name = (axis_index == 0) ? "X" : (axis_index == 1) ? "Y" : "Z";
        ESP_LOGD(TAG, "Limits %s set to %ld : %ld", axis_name, (long)min_val, (long)max_val);
    }
    else {
        uart_send_error("UNKNOWN_CMD");
    }
}

// Check for incoming UART data - process all available bytes
void check_uart_rx(void) {
    static char rx_buffer[128];
    static int rx_pos = 0;

    uint8_t data;
    // Read all available bytes in the UART buffer
    while (uart_read_bytes(UART_NUM, &data, 1, 0) > 0) {
        if (data == '\n' || data == '\r') {
            if (rx_pos > 0) {
                rx_buffer[rx_pos] = '\0';
                process_uart_command(rx_buffer);
                rx_pos = 0;
            }
        } else if (rx_pos < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_pos++] = data;
        }
    }
}

// ============================================================================
// DISPLAY MANAGEMENT
// ============================================================================

void update_display(int axis) {
    int dio_pin, clk_pin;

    if (axis == 0) {
        dio_pin = DISPLAY_X_DIO_PIN;
        clk_pin = DISPLAY_X_CLK_PIN;
    } else if (axis == 1) {
        dio_pin = DISPLAY_Y_DIO_PIN;
        clk_pin = DISPLAY_Y_CLK_PIN;
    } else {
        dio_pin = DISPLAY_Z_DIO_PIN;
        clk_pin = DISPLAY_Z_CLK_PIN;
    }

    // NOT_HOMED: show blinking dashes
    if (homing_state == HOMING_STATE_NOT_HOMED) {
        tm1637_display_dashes(dio_pin, clk_pin, not_homed_blink_state);
    }
    // HOMED: show numeric value
    else {
        bool should_blink = precision_mode[axis] && (axis == active_axis) &&
                           (operating_mode == MODE_ACTIVE) && !display_blink_state;
        tm1637_display_number_decimal(dio_pin, clk_pin, axis_positions[axis], false, 3, should_blink);
    }
}

void update_all_displays(void) {
    update_display(0);
    update_display(1);
    update_display(2);
}

// ============================================================================
// LED MANAGEMENT
// ============================================================================

void update_i2c_leds(void) {
    // Build pattern: LEDs reversed (A=bit6, B=bit5...G=bit0)
    // Active LOW: 0=ON, 1=OFF
    uint8_t pattern = 0xFF;

    for (int i = 0; i < 7; i++) {
        bool should_be_on = false;

        if (led_states[i] == LED_STATE_ON) {
            should_be_on = true;
        } else if (led_states[i] == LED_STATE_BLINK && led_blink_state) {
            should_be_on = true;
        } else if (led_states[i] == LED_STATE_FAST_BLINK && display_blink_state) {
            should_be_on = true;
        }

        if (should_be_on) {
            int bit = 6 - i;
            pattern &= ~(1 << bit);
        }
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_EXPANDER_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, pattern, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void update_axis_leds(void) {
    // Turn all off
    gpio_set_level(LED_X_PIN, 1);
    gpio_set_level(LED_Y_PIN, 1);
    gpio_set_level(LED_Z_PIN, 1);

    // Turn on active axis LED only in ACTIVE mode
    if (operating_mode == MODE_ACTIVE) {
        if (active_axis == 0) gpio_set_level(LED_X_PIN, 0);
        else if (active_axis == 1) gpio_set_level(LED_Y_PIN, 0);
        else if (active_axis == 2) gpio_set_level(LED_Z_PIN, 0);
    }
}

// ============================================================================
// HANDWHEEL ENCODER
// ============================================================================

void update_encoder(void) {
    int16_t pcnt_value;
    ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT, &pcnt_value));

    // Keep the hardware counter away from its limits: PCNT jumps back to 0
    // at +-32767, which looked like a huge turn and jogged to an axis limit.
    if (pcnt_value > PCNT_RECENTER_LIMIT || pcnt_value < -PCNT_RECENTER_LIMIT) {
        int remainder = pcnt_value - last_pcnt_value;  // not yet consumed counts
        pcnt_counter_clear(PCNT_UNIT);
        pcnt_value = 0;
        last_pcnt_value = -remainder;
    }

    // Only process encoder in ACTIVE mode and HOMED state. Otherwise discard
    // the turns: they used to pile up and were applied all at once when
    // ACTIVE mode started, jogging the machine by surprise.
    if (operating_mode != MODE_ACTIVE || homing_state != HOMING_STATE_HOMED) {
        last_pcnt_value = pcnt_value;
        return;
    }

    int delta = (pcnt_value - last_pcnt_value) / ENCODER_COUNTS_PER_CLICK;

    if (delta != 0) {
        int increment = precision_mode[active_axis] ? PRECISION_INCREMENT : NORMAL_INCREMENT;
        int32_t new_position = axis_positions[active_axis] + (delta * increment);

        // Enforce limits
        if (new_position < axis_limits_min[active_axis]) {
            new_position = axis_limits_min[active_axis];
        } else if (new_position > axis_limits_max[active_axis]) {
            new_position = axis_limits_max[active_axis];
        }

        axis_positions[active_axis] = new_position;
        last_pcnt_value += delta * ENCODER_COUNTS_PER_CLICK;

        // Update display immediately (no throttling for local display)
        update_display(active_axis);

        // Mark that we have a pending position to send
        // This ensures the final position is always sent
        pending_position_update = true;
        pending_position_axis = active_axis;
    }
}

void send_pending_position_update(uint32_t current_time) {
    // Check if we have a pending update and enough time has passed
    if (pending_position_update) {
        if (current_time - last_position_sent_time >= POSITION_UPDATE_RATE_MS) {
            // Send the current position
            const char *axis_name = (pending_position_axis == 0) ? "X" :
                                   (pending_position_axis == 1) ? "Y" : "Z";
            uart_send_position(axis_name, axis_positions[pending_position_axis]);

            last_position_sent_time = current_time;
            pending_position_update = false;
        }
        // Note: if time hasn't passed yet, pending_position_update stays true
        // so the final position will be sent in the next iteration
    }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handle_button_press(int button_index, bool is_long_press) {
    // Button mapping: 0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C, 6=D, 7=E, 8=F, 9=G

    // X/Y/Z buttons
    if (button_index >= 0 && button_index <= 2) {
        int axis = button_index;
        const char *axis_name = (axis == 0) ? "X" : (axis == 1) ? "Y" : "Z";

        // In ACTIVE mode and HOMED
        if (operating_mode == MODE_ACTIVE && homing_state == HOMING_STATE_HOMED) {
            if (is_long_press) {
                // Long press on active axis: enable precision mode
                if (axis == active_axis) {
                    precision_mode[axis] = true;
                    uart_send_axis_event(axis_name, "PRECISION:ON");
                    ESP_LOGI(TAG, "%s precision mode: ON", axis_name);
                }
            } else {
                // Short press
                if (axis == active_axis && precision_mode[axis]) {
                    // Turn off precision mode
                    precision_mode[axis] = false;
                    update_display(axis);  // Update display to show all digits
                    uart_send_axis_event(axis_name, "PRECISION:OFF");
                    ESP_LOGI(TAG, "%s precision mode: OFF", axis_name);
                } else {
                    // Select axis
                    active_axis = axis;
                    uart_send_axis_event(axis_name, "SELECT");
                    update_axis_leds();
                    ESP_LOGI(TAG, "Active axis: %s", axis_name);
                }
            }
        }
        // In PASSIVE mode or NOT_HOMED: just send button event
        else {
            uart_send_button_event(axis_name, is_long_press ? "LONG" : "SHORT");
        }
    }
    // A-G buttons - only send events, no local control
    else if (button_index >= 3 && button_index <= 9) {
        int led_index = button_index - 3;
        char led_name[2] = {'A' + led_index, '\0'};

        // Just send button event to Pi - LED control is via UART only
        uart_send_button_event(led_name, is_long_press ? "LONG" : "SHORT");
        ESP_LOGI(TAG, "Button %c: %s", led_name[0], is_long_press ? "LONG" : "SHORT");
    }
}

void poll_buttons(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int i = 0; i < 10; i++) {
        // Debounce: a contact bounce used to register a short press on the
        // way down, turning one long press on A into SHORT + LONG (two
        // concurrent homing requests on the Pi).
        int raw = gpio_get_level(buttons[i].pin);
        if (raw != buttons[i].last_raw) {
            buttons[i].last_raw = raw;
            buttons[i].raw_change_time = current_time;
        }
        int state = buttons[i].last_state;
        if (raw != buttons[i].last_state &&
            (current_time - buttons[i].raw_change_time) >= BUTTON_DEBOUNCE_MS) {
            state = raw;
        }

        // Button pressed (HIGH → LOW)
        if (state == 0 && buttons[i].last_state == 1) {
            buttons[i].press_time = current_time;
            buttons[i].long_press_triggered = false;
        }
        // Button held
        else if (state == 0 && buttons[i].last_state == 0) {
            if (!buttons[i].long_press_triggered &&
                (current_time - buttons[i].press_time) >= LONG_PRESS_TIME_MS) {
                buttons[i].long_press_triggered = true;
                handle_button_press(i, true);
            }
        }
        // Button released (LOW → HIGH)
        else if (state == 1 && buttons[i].last_state == 0) {
            if (!buttons[i].long_press_triggered) {
                handle_button_press(i, false);
            }
        }

        buttons[i].last_state = state;
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "AlphaPaint Console Firmware v1.2");
    ESP_LOGI(TAG, "========================================");

    // Initialize hardware
    init_handwheel();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_buttons();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_axis_leds();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_i2c_leds();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_displays();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_uart();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Send identification
    uart_send_string("CONSOLE:ALPHAPAINT:V1.2");
    uart_send_status("NOT_HOMED");

    // Initialize displays to show dashes (NOT_HOMED state)
    update_all_displays();

    ESP_LOGI(TAG, "Initialization complete");
    ESP_LOGI(TAG, "State: NOT_HOMED, Mode: PASSIVE");
    ESP_LOGI(TAG, "Waiting for homing from Raspberry Pi...");

    // From here on only warnings: ESP_LOG writes bypass the UART driver and
    // could land in the middle of a protocol message on the same UART.
    esp_log_level_set("*", ESP_LOG_WARN);

    // Main loop
    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Update display blink (150ms for precision mode and FAST_BLINK LEDs)
        if (current_time - last_display_blink_time >= DISPLAY_BLINK_MS) {
            display_blink_state = !display_blink_state;
            last_display_blink_time = current_time;

            // Update active axis display if in precision mode
            if (operating_mode == MODE_ACTIVE && precision_mode[active_axis]) {
                update_display(active_axis);
            }

            // Update I2C LEDs for FAST_BLINK state
            update_i2c_leds();
        }

        // Update LED blink (300ms)
        if (current_time - last_led_blink_time >= LED_BLINK_MS) {
            led_blink_state = !led_blink_state;
            last_led_blink_time = current_time;
            update_i2c_leds();
        }

        // Update NOT_HOMED display blink (300ms)
        if (homing_state == HOMING_STATE_NOT_HOMED) {
            if (current_time - last_not_homed_blink_time >= NOT_HOMED_BLINK_MS) {
                not_homed_blink_state = !not_homed_blink_state;
                last_not_homed_blink_time = current_time;
                update_all_displays();
            }
        }

        // Poll buttons
        poll_buttons();

        // Update encoder
        update_encoder();

        // Send pending position updates (rate limited)
        send_pending_position_update(current_time);

        // Check UART
        check_uart_rx();

        // Main loop delay
        vTaskDelay(MAIN_LOOP_TICKS);
    }
}
