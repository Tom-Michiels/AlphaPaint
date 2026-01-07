#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/pcnt.h"
#include "esp_log.h"

static const char *TAG = "CONSOLE_TEST";

// PCNT unit number for handwheel
#define PCNT_UNIT PCNT_UNIT_0

// Pin Definitions
#define HANDWHEEL_A_PIN     34
#define HANDWHEEL_B_PIN     39

// Button Pins (Active LOW with pull-up) - CORRECTED MAPPING!
#define BTN_X_PIN           23  // Was BTN_G (X and G swapped)
#define BTN_Y_PIN           26  // Was BTN_D (Y and D swapped)
#define BTN_Z_PIN           25  // Was BTN_C (Z and C swapped)
#define BTN_A_PIN           33  // Was BTN_B (A and B swapped)
#define BTN_B_PIN           0   // GPIO0 - strapping pin, safe with NO button
#define BTN_C_PIN           12  // GPIO12 - strapping pin, requires LOW for 3.3V flash
#define BTN_D_PIN           13  // Fixed: F→D mapping
#define BTN_E_PIN           14  // Fixed: C→E mapping
#define BTN_F_PIN           36  // Fixed: D→F mapping
#define BTN_G_PIN           27  // Was BTN_E (G and E swapped)

// Axis LED Pins (Active LOW) - CORRECTED!
#define LED_X_PIN           16  // Swapped with Y
#define LED_Y_PIN           4   // Swapped with X
#define LED_Z_PIN           2 

// I2C Configuration for LED expander
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_EXPANDER_ADDR   0x38  // PCF8574A address (found via I2C scan)

// 7-Segment Display Pins - CORRECTED!
#define DISPLAY_X_DIO_PIN   18  // X Display DIO
#define DISPLAY_X_CLK_PIN   19  // X Display CLK
#define DISPLAY_Y_DIO_PIN   15  // Y Display DIO (was swapped with Z)
#define DISPLAY_Y_CLK_PIN   5   // Y Display CLK
#define DISPLAY_Z_DIO_PIN   17  // Z Display DIO (was swapped with Y)
#define DISPLAY_Z_CLK_PIN   32  // Z Display CLK

// Function Prototypes
void init_handwheel(void);
void init_buttons(void);
void init_axis_leds(void);
void init_i2c_leds(void);
void init_displays(void);
void interactive_mode(void);
void set_led_state(int led_index, bool on);
void set_led_blink_mode(int led_index, bool blink);
void update_leds(void);
void set_active_axis(int axis);
void update_encoder(void);
void update_display(int axis);
void tm1637_display_number_decimal(int dio_pin, int clk_pin, int32_t number, bool leading_zero, int decimal_pos, bool blink_last_two);

// Initialize handwheel encoder inputs using hardware PCNT
void init_handwheel(void) {
    ESP_LOGI(TAG, "Initializing handwheel encoder with PCNT...");

    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = HANDWHEEL_A_PIN,
        .ctrl_gpio_num = HANDWHEEL_B_PIN,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,   // Count up on positive edge when ctrl=1
        .neg_mode = PCNT_COUNT_DEC,   // Count down on negative edge when ctrl=1
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse when ctrl=0
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep when ctrl=1
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    // Configure channel 1 (swap A and B for full quadrature)
    pcnt_config.pulse_gpio_num = HANDWHEEL_B_PIN;
    pcnt_config.ctrl_gpio_num = HANDWHEEL_A_PIN;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.pos_mode = PCNT_COUNT_DEC;
    pcnt_config.neg_mode = PCNT_COUNT_INC;
    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    // Initialize and start counting
    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT));

    ESP_LOGI(TAG, "Handwheel PCNT initialized on GPIO%d (A) and GPIO%d (B)", HANDWHEEL_A_PIN, HANDWHEEL_B_PIN);
}

// Initialize all button inputs
void init_buttons(void) {
    ESP_LOGI(TAG, "Initializing buttons...");

    // Note: GPIO 34-39 are input only and don't have software pull-up/pull-down
    // They require external pull-up resistors

    // Configure GPIO 36 (input-only pin) - button F now
    gpio_config_t io_conf_input_only = {
        .pin_bit_mask = (1ULL << BTN_F_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // Must use external pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_input_only);

    // Configure other button pins with internal pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_X_PIN) | (1ULL << BTN_Y_PIN) |
                       (1ULL << BTN_Z_PIN) | (1ULL << BTN_A_PIN) |
                       (1ULL << BTN_B_PIN) | (1ULL << BTN_C_PIN) |
                       (1ULL << BTN_D_PIN) | (1ULL << BTN_E_PIN) |
                       (1ULL << BTN_G_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Buttons initialized (Active LOW)");
}

// Initialize axis LEDs (X, Y, Z)
void init_axis_leds(void) {
    ESP_LOGI(TAG, "Initializing axis LEDs...");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_X_PIN) | (1ULL << LED_Y_PIN) | (1ULL << LED_Z_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Turn all LEDs off (Active LOW, so set HIGH)
    gpio_set_level(LED_X_PIN, 1);
    gpio_set_level(LED_Y_PIN, 1);
    gpio_set_level(LED_Z_PIN, 1);

    ESP_LOGI(TAG, "Axis LEDs initialized (Active LOW)");
}

// Initialize I2C and LED expander
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

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return;
    }

    // Turn all LEDs off (Active LOW, so write 0xFF)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_EXPANDER_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xFF, true);  // All LEDs off
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C LED expander initialized at address 0x%02X", I2C_EXPANDER_ADDR);
    } else {
        ESP_LOGW(TAG, "I2C LED expander communication failed: %s", esp_err_to_name(err));
        ESP_LOGI(TAG, "Scanning I2C bus for devices...");

        // Scan I2C bus to find devices
        int devices_found = 0;
        for (int addr = 0x03; addr < 0x78; addr++) {
            i2c_cmd_handle_t scan_cmd = i2c_cmd_link_create();
            i2c_master_start(scan_cmd);
            i2c_master_write_byte(scan_cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(scan_cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, scan_cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(scan_cmd);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Found I2C device at address 0x%02X", addr);
                devices_found++;
            }
        }

        if (devices_found == 0) {
            ESP_LOGW(TAG, "No I2C devices found. Check wiring!");
        }
    }
}

// TM1637 timing constants (microseconds)
#define TM1637_BIT_DELAY    50
#define TM1637_CLK_DELAY    5

// TM1637 Commands
#define TM1637_CMD_DATA     0x40  // Data command
#define TM1637_CMD_DISPLAY  0x80  // Display control command
#define TM1637_CMD_ADDRESS  0xC0  // Address command

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

// TM1637 Protocol Functions
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
    // Configure DIO as output
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
    // brightness: 0 (dim) to 7 (bright), or 8 for off
    uint8_t cmd = TM1637_CMD_DISPLAY;
    if (brightness < 8) {
        cmd |= 0x08 | (brightness & 0x07);  // Display ON + brightness
    }

    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, cmd);
    tm1637_stop(dio_pin, clk_pin);
}

void tm1637_display_number(int dio_pin, int clk_pin, int32_t number, bool leading_zero) {
    tm1637_display_number_decimal(dio_pin, clk_pin, number, leading_zero, -1, false);
}

void tm1637_display_number_decimal(int dio_pin, int clk_pin, int32_t number, bool leading_zero, int decimal_pos, bool blink_last_two) {
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

    // Set data write mode with auto-increment
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x40);  // Data command, auto increment address
    tm1637_stop(dio_pin, clk_pin);

    // Write digits starting at address 0 - REVERSED IN GROUPS OF 3
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0xC0);  // Address command, start at 0

    // Display shows groups reversed: to show 123456, send digits[2,1,0,5,4,3]
    int display_order[6] = {2, 1, 0, 5, 4, 3};

    // Find first non-zero digit for leading zero suppression
    int first_nonzero = 5;  // Default to rightmost digit
    if (!leading_zero) {
        for (int i = 0; i < 6; i++) {
            if (digits[i] != 0) {
                first_nonzero = i;
                break;
            }
        }
        // If we have a decimal point, always show at least the digits after it
        if (decimal_pos >= 0 && first_nonzero > decimal_pos) {
            first_nonzero = decimal_pos;  // Show from decimal position onwards
        }
    }

    for (int pos = 0; pos < 6; pos++) {
        int i = display_order[pos];
        uint8_t segment_data = 0;

        // Check if this digit should be hidden due to blinking
        bool hide_for_blink = blink_last_two && (i == 4 || i == 5);

        if (hide_for_blink) {
            segment_data = 0x00;  // Blank during blink
        } else if (i == 0 && negative) {
            segment_data = 0x40;  // Minus sign
        } else if (leading_zero || i >= first_nonzero) {
            segment_data = digit_to_segment[digits[i]];
            // Add decimal point if needed (bit 7)
            if (decimal_pos >= 0 && i == decimal_pos) {
                segment_data |= 0x80;
            }
        } else {
            segment_data = 0x00;  // Blank (leading zero suppression)
        }

        tm1637_write_byte(dio_pin, clk_pin, segment_data);
    }

    tm1637_stop(dio_pin, clk_pin);

    // Turn on display with brightness
    tm1637_set_brightness(dio_pin, clk_pin, 2);
}

void tm1637_clear(int dio_pin, int clk_pin) {
    // Set data write mode with auto-increment
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0x40);
    tm1637_stop(dio_pin, clk_pin);

    // Write zeros to all addresses
    tm1637_start(dio_pin, clk_pin);
    tm1637_write_byte(dio_pin, clk_pin, 0xC0);  // Start at address 0
    for (int i = 0; i < 6; i++) {
        tm1637_write_byte(dio_pin, clk_pin, 0x00);
    }
    tm1637_stop(dio_pin, clk_pin);
}

// Initialize TM1637 displays (X, Y, Z - each has dedicated DIO and CLK)
void init_displays(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TM1637 Display Initialization");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "X Display: DIO=GPIO%d, CLK=GPIO%d", DISPLAY_X_DIO_PIN, DISPLAY_X_CLK_PIN);
    ESP_LOGI(TAG, "Y Display: DIO=GPIO%d, CLK=GPIO%d", DISPLAY_Y_DIO_PIN, DISPLAY_Y_CLK_PIN);
    ESP_LOGI(TAG, "Z Display: DIO=GPIO%d, CLK=GPIO%d", DISPLAY_Z_DIO_PIN, DISPLAY_Z_CLK_PIN);

    // Configure all display pins - three separate DIO+CLK pairs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DISPLAY_X_DIO_PIN) |
                       (1ULL << DISPLAY_X_CLK_PIN) |
                       (1ULL << DISPLAY_Y_DIO_PIN) |
                       (1ULL << DISPLAY_Y_CLK_PIN) |
                       (1ULL << DISPLAY_Z_DIO_PIN) |
                       (1ULL << DISPLAY_Z_CLK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "GPIO configured");

    // Set all pins high initially
    gpio_set_level(DISPLAY_X_DIO_PIN, 1);
    gpio_set_level(DISPLAY_X_CLK_PIN, 1);
    gpio_set_level(DISPLAY_Y_DIO_PIN, 1);
    gpio_set_level(DISPLAY_Y_CLK_PIN, 1);
    gpio_set_level(DISPLAY_Z_DIO_PIN, 1);
    gpio_set_level(DISPLAY_Z_CLK_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test GPIO toggling for Y and Z to verify they work
    ESP_LOGI(TAG, "Testing Y display GPIO toggling...");
    for (int i = 0; i < 5; i++) {
        gpio_set_level(DISPLAY_Y_DIO_PIN, 0);
        gpio_set_level(DISPLAY_Y_CLK_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(DISPLAY_Y_DIO_PIN, 1);
        gpio_set_level(DISPLAY_Y_CLK_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Testing Z display GPIO toggling...");
    for (int i = 0; i < 5; i++) {
        gpio_set_level(DISPLAY_Z_DIO_PIN, 0);
        gpio_set_level(DISPLAY_Z_CLK_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(DISPLAY_Z_DIO_PIN, 1);
        gpio_set_level(DISPLAY_Z_CLK_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "GPIO toggle test complete");

    // Initialize all three displays with 888888
    ESP_LOGI(TAG, "Initializing X display with 888888...");
    tm1637_display_number(DISPLAY_X_DIO_PIN, DISPLAY_X_CLK_PIN, 888888, true);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Initializing Y display with 888888...");
    ESP_LOGI(TAG, "  Y DIO=GPIO%d, CLK=GPIO%d", DISPLAY_Y_DIO_PIN, DISPLAY_Y_CLK_PIN);
    ESP_LOGI(TAG, "  Y pins (DIO=%d, CLK=%d)", gpio_get_level(DISPLAY_Y_DIO_PIN), gpio_get_level(DISPLAY_Y_CLK_PIN));
    tm1637_set_brightness(DISPLAY_Y_DIO_PIN, DISPLAY_Y_CLK_PIN, 7);
    vTaskDelay(pdMS_TO_TICKS(100));
    tm1637_display_number(DISPLAY_Y_DIO_PIN, DISPLAY_Y_CLK_PIN, 888888, true);
    ESP_LOGI(TAG, "  Y display command sent");
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Initializing Z display with 888888...");
    ESP_LOGI(TAG, "  Z DIO=GPIO%d, CLK=GPIO%d", DISPLAY_Z_DIO_PIN, DISPLAY_Z_CLK_PIN);
    ESP_LOGI(TAG, "  Z pins (DIO=%d, CLK=%d)", gpio_get_level(DISPLAY_Z_DIO_PIN), gpio_get_level(DISPLAY_Z_CLK_PIN));
    tm1637_set_brightness(DISPLAY_Z_DIO_PIN, DISPLAY_Z_CLK_PIN, 7);
    vTaskDelay(pdMS_TO_TICKS(100));
    tm1637_display_number(DISPLAY_Z_DIO_PIN, DISPLAY_Z_CLK_PIN, 888888, true);
    ESP_LOGI(TAG, "  Z display command sent");
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "All three displays initialized");
    ESP_LOGI(TAG, "Verify:");
    ESP_LOGI(TAG, "  X Display should show 888888");
    ESP_LOGI(TAG, "  Y Display should show 888888");
    ESP_LOGI(TAG, "  Z Display should show 888888");
    ESP_LOGI(TAG, "========================================");
}

// Global state variables
static int active_axis = 0;  // 0=X, 1=Y, 2=Z
static int32_t axis_values[3] = {0, 0, 0};  // X, Y, Z values (stored as whole units * 100)
static bool led_states[7] = {false, false, false, false, false, false, false};  // A-G LED states (ON/OFF)
static bool led_blink_mode[7] = {false, false, false, false, false, false, false};  // A-G LED blink mode
static int last_pcnt_value = 0;
static bool precision_mode[3] = {false, false, false};  // Precision mode per axis
static uint32_t last_blink_time = 0;
static bool blink_state = false;
static uint32_t last_led_blink_time = 0;
static bool led_blink_state = false;

// Update LED hardware based on states and blink modes
void update_leds(void) {
    // Build I2C pattern: LEDs are reversed (A=bit6, B=bit5...G=bit0)
    // Active LOW, so 0=ON, 1=OFF
    uint8_t pattern = 0xFF;
    for (int i = 0; i < 7; i++) {
        bool should_be_on = led_states[i];

        // If in blink mode, apply blink state
        if (led_blink_mode[i] && !led_blink_state) {
            should_be_on = false;  // Turn off during blink
        }

        if (should_be_on) {
            int bit = 6 - i;  // Reverse mapping
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

// Set LED state for A-G LEDs
void set_led_state(int led_index, bool on) {
    if (led_index < 0 || led_index >= 7) return;
    led_states[led_index] = on;
    update_leds();
}

// Set LED blink mode for A-G LEDs
void set_led_blink_mode(int led_index, bool blink) {
    if (led_index < 0 || led_index >= 7) return;
    led_blink_mode[led_index] = blink;
    if (blink) {
        led_states[led_index] = true;  // Ensure LED is on when entering blink mode
    }
    update_leds();
}

// Set active axis and update axis LEDs
void set_active_axis(int axis) {
    if (axis < 0 || axis >= 3) return;

    active_axis = axis;

    // Turn all axis LEDs off first (Active LOW, so HIGH=OFF)
    gpio_set_level(LED_X_PIN, 1);
    gpio_set_level(LED_Y_PIN, 1);
    gpio_set_level(LED_Z_PIN, 1);

    // Turn on the active axis LED (Active LOW, so LOW=ON)
    if (axis == 0) {
        gpio_set_level(LED_X_PIN, 0);
        ESP_LOGI(TAG, "Active axis: X");
    } else if (axis == 1) {
        gpio_set_level(LED_Y_PIN, 0);
        ESP_LOGI(TAG, "Active axis: Y");
    } else if (axis == 2) {
        gpio_set_level(LED_Z_PIN, 0);
        ESP_LOGI(TAG, "Active axis: Z");
    }
}

// Update display for a specific axis
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

    // Show with decimal point before last 2 digits (position 3)
    // In precision mode, blink last two digits if on active axis
    bool should_blink = precision_mode[axis] && (axis == active_axis) && !blink_state;
    tm1637_display_number_decimal(dio_pin, clk_pin, axis_values[axis], false, 3, should_blink);
}

// Read hardware PCNT and update active axis value
void update_encoder(void) {
    int16_t pcnt_value;
    ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT, &pcnt_value));

    // Calculate delta from last read
    // Quadrature encoding gives 4 counts per detent, so divide by 4
    int delta = (pcnt_value - last_pcnt_value) / 4;

    if (delta != 0) {
        // In precision mode, increment by 1 (0.01 units)
        // In normal mode, increment by 100 (1.00 units)
        int increment = precision_mode[active_axis] ? 1 : 100;
        axis_values[active_axis] += delta * increment;
        last_pcnt_value += delta * 4;  // Update to nearest multiple of 4

        // Update the corresponding display
        update_display(active_axis);

        const char* axis_name = (active_axis == 0) ? "X" : (active_axis == 1) ? "Y" : "Z";
        ESP_LOGI(TAG, "%s = %d.%02d (delta=%d, precision=%s)",
                 axis_name,
                 (int)axis_values[active_axis] / 100,
                 (int)abs(axis_values[active_axis]) % 100,
                 delta,
                 precision_mode[active_axis] ? "ON" : "OFF");
    }
}

// Interactive mode main loop
void interactive_mode(void) {
    ESP_LOGI(TAG, "=== INTERACTIVE MODE ===");
    ESP_LOGI(TAG, "Buttons A-G: Toggle LEDs A-G");
    ESP_LOGI(TAG, "Buttons X/Y/Z: Select active axis");
    ESP_LOGI(TAG, "Handwheel: Adjust active axis value");
    ESP_LOGI(TAG, "========================================");

    // Button definitions
    const struct {
        int pin;
        const char* name;
        int index;
    } buttons[] = {
        {BTN_X_PIN, "X", -1},
        {BTN_Y_PIN, "Y", -2},
        {BTN_Z_PIN, "Z", -3},
        {BTN_A_PIN, "A", 0},
        {BTN_B_PIN, "B", 1},
        {BTN_C_PIN, "C", 2},
        {BTN_D_PIN, "D", 3},
        {BTN_E_PIN, "E", 4},
        {BTN_F_PIN, "F", 5},
        {BTN_G_PIN, "G", 6}
    };

    // Initialize button states and timing
    int last_button_states[10];
    uint32_t button_press_time[10] = {0};
    bool long_press_triggered[10] = {false};
    const uint32_t LONG_PRESS_TIME_MS = 500;  // 500ms for long press

    for (int i = 0; i < 10; i++) {
        last_button_states[i] = gpio_get_level(buttons[i].pin);
    }

    // Initialize PCNT counter
    pcnt_counter_clear(PCNT_UNIT);
    last_pcnt_value = 0;

    // Set initial active axis
    set_active_axis(0);

    // Initialize all displays to 0 with decimal point
    update_display(0);
    update_display(1);
    update_display(2);

    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Update display blink state (toggle every 150ms for faster blinking)
        if (current_time - last_blink_time > 150) {
            blink_state = !blink_state;
            last_blink_time = current_time;

            // Update display of active axis if in precision mode
            if (precision_mode[active_axis]) {
                update_display(active_axis);
            }
        }

        // Update LED blink state (toggle every 300ms)
        if (current_time - last_led_blink_time > 300) {
            led_blink_state = !led_blink_state;
            last_led_blink_time = current_time;

            // Update LEDs if any are in blink mode
            bool any_blinking = false;
            for (int i = 0; i < 7; i++) {
                if (led_blink_mode[i]) {
                    any_blinking = true;
                    break;
                }
            }
            if (any_blinking) {
                update_leds();
            }
        }

        // Check all buttons
        for (int i = 0; i < 10; i++) {
            int state = gpio_get_level(buttons[i].pin);

            // Detect button press (transition from HIGH to LOW)
            if (state == 0 && last_button_states[i] == 1) {
                button_press_time[i] = current_time;
                long_press_triggered[i] = false;
            }
            // Check for long press while button is held
            else if (state == 0 && last_button_states[i] == 0) {
                if (!long_press_triggered[i] &&
                    (current_time - button_press_time[i]) >= LONG_PRESS_TIME_MS) {
                    long_press_triggered[i] = true;

                    // Handle long press for X/Y/Z buttons (enable precision mode)
                    if (buttons[i].index == -1 && active_axis == 0) {
                        precision_mode[0] = true;
                        ESP_LOGI(TAG, "X precision mode: ON");
                        update_display(0);
                    } else if (buttons[i].index == -2 && active_axis == 1) {
                        precision_mode[1] = true;
                        ESP_LOGI(TAG, "Y precision mode: ON");
                        update_display(1);
                    } else if (buttons[i].index == -3 && active_axis == 2) {
                        precision_mode[2] = true;
                        ESP_LOGI(TAG, "Z precision mode: ON");
                        update_display(2);
                    } else if (buttons[i].index >= 0 && buttons[i].index < 7) {
                        // A-G button - long press enables blink mode
                        int led_idx = buttons[i].index;
                        set_led_blink_mode(led_idx, true);
                        ESP_LOGI(TAG, "LED %c: BLINK MODE ON", 'A' + led_idx);
                    }
                }
            }
            // Detect button release (transition from LOW to HIGH)
            else if (state == 1 && last_button_states[i] == 0) {
                // Only handle short press if long press wasn't triggered
                if (!long_press_triggered[i]) {
                    if (buttons[i].index == -1) {
                        // X button - short press
                        if (active_axis == 0 && precision_mode[0]) {
                            // Turn off precision mode if already in it
                            precision_mode[0] = false;
                            ESP_LOGI(TAG, "X precision mode: OFF");
                            update_display(0);
                        } else {
                            // Set active axis
                            set_active_axis(0);
                            update_display(0);
                        }
                    } else if (buttons[i].index == -2) {
                        // Y button - short press
                        if (active_axis == 1 && precision_mode[1]) {
                            precision_mode[1] = false;
                            ESP_LOGI(TAG, "Y precision mode: OFF");
                            update_display(1);
                        } else {
                            set_active_axis(1);
                            update_display(1);
                        }
                    } else if (buttons[i].index == -3) {
                        // Z button - short press
                        if (active_axis == 2 && precision_mode[2]) {
                            precision_mode[2] = false;
                            ESP_LOGI(TAG, "Z precision mode: OFF");
                            update_display(2);
                        } else {
                            set_active_axis(2);
                            update_display(2);
                        }
                    } else {
                        // A-G buttons: short press
                        int led_idx = buttons[i].index;
                        if (led_blink_mode[led_idx]) {
                            // If in blink mode, turn it off
                            set_led_blink_mode(led_idx, false);
                            set_led_state(led_idx, false);  // Turn LED off
                            ESP_LOGI(TAG, "LED %c: BLINK MODE OFF", 'A' + led_idx);
                        } else {
                            // Normal toggle
                            set_led_state(led_idx, !led_states[led_idx]);
                            ESP_LOGI(TAG, "LED %c: %s", 'A' + led_idx, led_states[led_idx] ? "ON" : "OFF");
                        }
                    }
                }
            }

            last_button_states[i] = state;
        }

        // Check handwheel encoder using hardware PCNT
        update_encoder();

        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms poll rate (hardware counts in background)
    }
}

// Main application
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "AlphaPaint Console Interactive Program");
    ESP_LOGI(TAG, "========================================");

    // Initialize all I/O
    ESP_LOGI(TAG, "Initializing all I/O...\n");

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

    ESP_LOGI(TAG, "\nInitialization complete!\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run interactive mode
    interactive_mode();
}
