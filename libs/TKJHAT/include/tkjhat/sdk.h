/*
Version 0.8

MIT License

Copyright (c) 2025 , Raisul Islam, Iván Sánchez Milara

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/**
 * @file tkjhat/sdk.h
 * \author Raisul Islam
 * \author Iván Sánchez Milara
 * @brief Public API for the JTKJ HAT SDK (Raspberry Pi Pico / Pico 2).
 *
 * \par Contributors
 * Emil Kelhala
 * 
 * @version 0.8
 *
 * @details
 * This SDK provides simple functions to use the main hardware on the JTKJ HAT:
 *
 * - **User inputs**
 *   - Two buttons: SW1 (GPIO 2), SW2 (GPIO 22)
 *
 * - **Indicators / output**
 *   - One red LED (GPIO 14)
 *   - One RGB LED (GPIO 18:R, 19:G, 20:B) driven by PWM
 *   - Buzzer (GPIO 17)
 *   - 0.96" SSD1306 I2C OLED display (addr 0x3C)
 *
 * - **Sensors**
 *   - VEML6030 ambient light sensor (I2C addr 0x10)
 *   - HDC2021 temperature & humidity sensor (I2C addr 0x40)
 *   - ICM-42670 IMU: 3-axis accelerometer + 3-axis gyroscope (I2C addr 0x69 / 0x69 alt autodetect)
 *   - PDM MEMS microphone (DATA GPIO 16, CLK GPIO 15) via PIO + OpenPDM2PCM
 *
 * - **I2C defaults**
 *   - SDA GPIO 12, SCL GPIO 13
 *
 * The API aims to be minimal and easy to use in teaching settings. Most functions are
 * non-blocking and configure the required pins/peripherals for basic operation. For more
 * advanced control (e.g., custom I2C transfers), helper functions are also provided.
 *
 * ### Typical usage
 * @code
 * #include <tkjhat/sdk.h>
 *
 * int main(void) {
 *     stdio_init_all();
 *
 *     init_hat_sdk();          // put board in a known state (e.g., RGB off) and start default i2c
 *     init_red_led();          // optional: prepare red LED
 *     init_rgb_led();          // optional: enable RGB PWM
 *
 *     init_display();          // SSD1306 ready to use
 *     write_text("Hello HAT"); // draw text
 *
 *     ICM42670_init();                         // IMU WHO_AM_I + basic setup
 *     ICM42670_startAccel(100, 4);            // 100 Hz, ±4 g
 *     ICM42670_startGyro(100, 250);           // 100 Hz, ±250 dps
 *     ICM42670_enable_accel_gyro_ln_mode();   // low-noise mode
 *
 *     // ...
 * }
 * @endcode
 *
 * ### Notes
 * - `init_hat_sdk()` only prepares the board state (e.g., turns RGB LED off) and initialize default I2C to connect the peripherals
 *    It does **not** initialize peripherals; call the per-device init functions you need.
 *
 * ### Dependencies
 * - Raspberry Pi Pico SDK (GPIO, PWM, I2C, PIO, DMA where applicable)
 * - (Optional) FreeRTOS if used elsewhere in the application
 * - OpenPDM2PCM (bundled) for microphone PCM conversion
 * - pico-ssd1306 (bundled) for OLED display support - https://github.com/daschr/pico-ssd1306
 *
 * @copyright
 * MIT License — see the header for full text.
 */

#ifndef SDK_H
#define SDK_H


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include <hardware/i2c.h>

#include "pdm_microphone.h"   // pdm_samples_ready_handler_t
#include "pins.h"



/* =========================
 *  VEML6030
 * ========================= */
#define VEML6030_I2C_ADDR                       0x10
#define VEML6030_CONFIG_REG                     0x00
#define VEML6030_ALS_REG                        0x04

/* =========================
 *  HDC2021
 * ========================= */
#define HDC2021_I2C_ADDRESS                     0x40
#define HDC2021_TEMP_LOW                        0x00
#define HDC2021_TEMP_HIGH                       0x01
#define HDC2021_HUMIDITY_LOW                    0x02
#define HDC2021_HUMIDITY_HIGH                   0x03
#define HDC2021_CONFIG                          0x0E
#define HDC2021_MEASUREMENT_CONFIG              0x0F
#define HDC2021_TEMP_THR_L                      0x13
#define HDC2021_TEMP_THR_H                      0x14
#define HDC2021_HUMID_THR_L                     0x15
#define HDC2021_HUMID_THR_H                     0x16

/* =========================
 *  SSD1306
 * ========================= */

 #define SSD1306_I2C_ADDRESS                    0x3C

 /* =========================
 *  MEMS MICROPHONE
 * ========================= */

# define MEMS_SAMPLING_FREQUENCY                8000
# define MEMS_BUFFER_SIZE                       256

/* =========================
 *  ICM42670
 * ========================= */
#define ICM42670_I2C_ADDRESS                    0x69
#define ICM42670_I2C_ADDRESS_ALT                0x69
#define ICM42670_REG_WHO_AM_I                   0x75
#define ICM42670_WHO_AM_I_RESPONSE              0x67
#define ICM42670_INT_CONFIG                     0x06
#define ICM42670_INT1_CONFIG_VALUE              0x02
#define ICM42670_MAX_READ_LENGTH                256
#define ICM42670_MAX_WRITE_LENGTH               256

#define ICM42670_ACCEL_CONFIG0_REG              0x21
#define ICM42670_PWR_MGMT0_REG                  0x1F

// Accel FSR encodings
#define ICM42670_ACCEL_FSR_2G                   0x03
#define ICM42670_ACCEL_FSR_4G                   0x02
#define ICM42670_ACCEL_FSR_8G                   0x01
#define ICM42670_ACCEL_FSR_16G                  0x00
#define ICM42670_ACCEL_FSR_DEFAULT              4
#define ICM42670_GYRO_FSR_DEFAULT               250

// Accel ODR encodings (LN mode)
#define ICM42670_ACCEL_ODR_25HZ                 0x0B
#define ICM42670_ACCEL_ODR_50HZ                 0x0A
#define ICM42670_ACCEL_ODR_100HZ                0x09
#define ICM42670_ACCEL_ODR_200HZ                0x08
#define ICM42670_ACCEL_ODR_400HZ                0x07
#define ICM42670_ACCEL_ODR_800HZ                0x06
#define ICM42670_ACCEL_ODR_1600HZ               0x05
#define ICM42670_ACCEL_ODR_DEFAULT              100
#define ICM42670_GYRO_ODR_DEFAULT               100

// Accel LN mode in PWR_MGMT0
/* note: ICM42670_PWR_MGMT0_REG also defined above */
#define ICM42670_ACCEL_MODE_LN                  0x03
#define ICM42670_GYRO_CONFIG0_REG               0x20
#define ICM42670_REG_SIGNAL_PATH_RESET          0x02
#define ICM42670_RESET_CONFIG_BITS              0x10

// Gyro FSR encodings
#define ICM42670_GYRO_FSR_250DPS                0x03
#define ICM42670_GYRO_FSR_500DPS                0x02
#define ICM42670_GYRO_FSR_1000DPS               0x01
#define ICM42670_GYRO_FSR_2000DPS               0x00

// Gyro ODR encodings
#define ICM42670_GYRO_ODR_25HZ                  0x0B
#define ICM42670_GYRO_ODR_50HZ                  0x0A
#define ICM42670_GYRO_ODR_100HZ                 0x09
#define ICM42670_GYRO_ODR_200HZ                 0x08
#define ICM42670_GYRO_ODR_400HZ                 0x07
#define ICM42670_GYRO_ODR_800HZ                 0x06
#define ICM42670_GYRO_ODR_1600HZ                0x05

#define ICM42670_GYRO_MODE_LN                   0x0C
#define ICM42670_SENSOR_DATA_START_REG          0x09

/* =========================
 *  Public function prototypes
 * ========================= */

/**
 * @brief Initialize the HAT SDK.
 *
 * This function performs the initial setup of the HAT board.
 * Starts the default I2C for communicating with devices.
 * It does not initialize any peripherals.
 */
void init_hat_sdk(void);


/* =========================
 *  BUTTONS / SWITCHES
 * ========================= */

/**
 * @brief Initialize switch SW1 (GPIO 2).
 *
 * Configures SW1 as a digital input with no pull resistors enabled. It uses hardware pull-down resistors.
 * 
 * Use @c gpio_get(SW1_PIN) to read its state directly.
 *
 * @note SW1 is active-high (pressed = 1, released = 0). 
 */
void init_sw1(void);

/**
 * @brief Initializes button 1 (same as SW1).
 *
 * This function is a wrapper for @ref init_sw1, performing the same initialization
 * as the SW1 switch setup. It configures GPIO 2 as a digital input using the 
 * hardware pull-down resistor.
 *
 * @see init_sw1()
 */
void init_button1(void);

/**
 * @brief Initialize switch SW2 (GPIO 22).
 *
 * Configures SW2 as a digital input with no pull resistors enabled. It uses hardware pull-down resistors.
 * Use @c gpio_get(SW2_PIN) to read its state directly.
 *
 * @note SW2 is active-hig (pressed = 1, released = 0) 
 */
void init_sw2(void);

/**
 * @brief Initializes button2 (same as SW2).
 *
 * This function is a wrapper for @ref init_sw2, performing the same initialization
 * as the SW2 switch setup. It configures GPIO 22 as a digital input using the 
 * hardware pull-down resistor.
 *
 * @see init_sw2()
 */
void init_button2(void);


/* =========================
 *  LEDs
 * ========================= */

/**
 * @brief Initialize the onboard LED.
 *
 * Configures the onboard LED pin as an output.  
 * On the JTKJ HAT the onboard LED is red (GPIO 14).
 */
void init_led(void);

/**
 * @brief Initialize the red LED.
 *
 * Configures the red LED pin (GPIO 14) as an output.  
 * On this board, the red LED is the same as the onboard LED.
 */
void init_red_led(void);

/**
 * @brief Toggle the onboard LED state.
 *
 * Reads the current state of the onboard LED and switches it
 * (ON → OFF, OFF → ON).
 */
void toggle_led(void);

/**
 * @brief Toggle the red LED state.
 *
 * Reads the current state of the red LED and switches it
 * (ON → OFF, OFF → ON).  
 * On this board, the red LED is the same as the onboard LED.
 */
void toggle_red_led(void);

/**
 * @brief Set the onboard LED state explicitly.
 *
 * @param status Set to @c true to turn the LED on, @c false to turn it off.
 */
void set_led_status(bool status);

/**
 * @brief Set the red LED state explicitly.
 *
 * @param status Set to @c true to turn the LED on, @c false to turn it off.  
 * On this board, the red LED is the same as the onboard LED.
 */
void set_red_led_status(bool status);

/**
 * @brief Blink the onboard LED a given number of times.
 *
 * Toggles the onboard LED on/off with a fixed delay (~120 ms) 
 * between transitions. Leaves the LED turned OFF at the end.
 *
 * @param n Number of times to blink.
 */
void blink_led(int n);

/**
 * @brief Blink the red LED a given number of times.
 *
 * Toggles the red LED on/off with a fixed delay (~120 ms) 
 * between transitions. Leaves the LED turned OFF at the end.  
 * On this board, the red LED is the same as the onboard LED.
 *
 * @param n Number of times to blink.
 */
void blink_red_led(int n);


/**
 * @brief Initialize the RGB LED (GPIO 18:R, 19:G, 20:B).
 *
 * Configures the RGB LED pins as PWM outputs and enables their
 * PWM slices. With the default 16-bit TOP (65535) and clkdiv = 4,
 * the PWM frequency is: 480 Hz.
 *
 * @note After initialization, you can set colors using ::rgb_led_write().
 *       Call ::stop_rgb_led() to release the pins.
 */
void init_rgb_led(void);

/**
 * @brief Set the RGB LED color.
 *
 * Writes PWM duty cycles to the RGB LED channels to produce the
 * requested color. The LED is wired as common-anode, so inputs are
 * inverted internally (0 = full on, 255 = off). A simple gamma
 * correction is applied by squaring the 8-bit values.
 *
 * @param r Red intensity   (0–255, 0 = full on, 255 = off)
 * @param g Green intensity (0–255, 0 = full on, 255 = off)
 * @param b Blue intensity  (0–255, 0 = full on, 255 = off)
 */
void rgb_led_write(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Stop and release the RGB LED pins.
 *
 * Disables the PWM slices driving the RGB LED and returns the pins
 * to inputs (Hi-Z). After this call, the RGB LED is fully off and
 * the pins are available for other use.
 */
void stop_rgb_led(void);


/* =========================
 *  BUZZER
 * ========================= */

/**
 * @brief Initialize the buzzer (GPIO 17).
 *
 * Configures the buzzer pin as a digital output.  
 * After this call, the buzzer can be controlled with
 * ::buzzer_play_tone() or ::buzzer_turn_off().
 */
void init_buzzer(void);

/**
 * @brief Play a tone on the buzzer.
 *
 * Generates a square wave at the requested frequency for the
 * specified duration by toggling the buzzer pin.
 *
 * @param frequency     Tone frequency in Hz.
 * @param duration_ms   Duration of the tone in milliseconds.
 *
 * @note This implementation is blocking: the CPU is kept busy
 *       toggling the pin for the entire duration.
 */
void buzzer_play_tone(uint32_t frequency, uint32_t duration_ms);

/**
 * @brief Turn the buzzer off.
 *
 * Drives the buzzer pin low, silencing any ongoing tone.
 */
void buzzer_turn_off(void);

/**
 * @brief Deinitialize the buzzer.
 *
 * Releases the buzzer pin (GPIO 17) so it can be reused for
 * other purposes.
 */
void deinit_buzzer(void);



/* =========================
 *  PDM MEMS MICROPHONE
 * ========================= */

// Uses https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico/tree/main
// Uses pio to read pdm data and OpenPDM2PCM library to transform PDM to PCM
/**
 * @brief Initialize the PDM MEMS microphone.
 *
 * Configures the microphone using PIO to capture PDM data and the
 * OpenPDM2PCM library to transform it into PCM samples.
 *
 * Implementation is based on the official
 * [Arm Developer Ecosystem microphone library for Pico]
 * (https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico/tree/main).
 *
 * Default parameters:
 * - Data pin: GPIO 16
 * - Clock pin: GPIO 15
 * - Sample rate: 16 kHz
 * - Buffer size: 256 samples
 *
 * @return 0 on success, negative value on error.
 */
int init_pdm_microphone(void);

/**
 * @brief Start microphone sampling.
 *
 * Begins continuous capture of PCM samples from the microphone
 * at 16 kHz with a buffer size of 256 samples.
 *
 * @return 0 on success, negative value on error.
 */
int init_microphone_sampling(void);

/**
 * @brief Stop microphone sampling.
 *
 * Halts the ongoing microphone capture.  
 * Call again later with ::init_microphone_sampling() to resume.
 */
void end_microphone_sampling(void);

/**
 * @brief Register a callback for new microphone samples.
 *
 * Sets the function that will be invoked when new samples are
 * available in the buffer.
 *
 * @param handler Callback of type ::pdm_samples_ready_handler_t.
 */
void pdm_microphone_set_callback(pdm_samples_ready_handler_t handler);

/**
 * @brief Retrieve PCM samples from the microphone buffer.
 *
 * Copies up to @p samples 16-bit values into the provided buffer.
 *
 * @param buffer  Destination buffer for PCM samples.
 * @param samples Number of samples to read.
 * @return The number of samples actually read, or negative on error.
 *
 * @note This function is called inside the sample-ready callback. Otherwise, might not work. 
 */
int get_microphone_samples(int16_t *buffer, size_t samples);



/**
 * @brief Initialize an I²C instance with explicit pins.
 *
 * Configures @c i2c_default for Fast-mode (400 kHz), sets @p sda_pin and
 * @p scl_pin to I²C function, and enables pull-ups on both lines.
 *
 * @param sda_pin GPIO to use for SDA (e.g., @ref DEFAULT_I2C_SDA_PIN).
 * @param scl_pin GPIO to use for SCL (e.g., @ref DEFAULT_I2C_SCL_PIN).
 */
void init_i2c(uint sda_pin, uint scl_pin);

/**
 * @brief Initialize the default I²C instance using board default pins.
 *
 * Convenience wrapper that calls:
 * @code
 * init_i2c(DEFAULT_I2C_SDA_PIN, DEFAULT_I2C_SCL_PIN);
 * @endcode
 *
 * Defaults (from @ref pins.h):
 * - SDA = @ref DEFAULT_I2C_SDA_PIN  (12)
 * - SCL = @ref DEFAULT_I2C_SCL_PIN  (13)
 *
 * Connected devices on the JTKJ HAT (default bus):
 * - SSD1306 OLED display               (0x3C)
 * - VEML6030 ambient light sensor      (0x10)
 * - HDC2021 temperature/humidity       (0x40)
 * - ICM-42670 IMU (accel + gyro)       (0x69)
 *
 * @post @c i2c_default is ready at 400 kHz with pull-ups enabled.
 */
void init_i2c_default(void);

/**
 * @brief Write data to an I²C device.
 *
 * Writes a buffer of bytes to the specified I²C address.
 *
 * Typical usage: writing a configuration value to a sensor register.
 *
 * @code
 * // Example: write 0x01 into configuration register 0x0E of HDC2021
 * uint8_t data[2] = {0x0E, 0x01};  // [register, value]
 * bool ok = i2c_write(HDC2021_I2C_ADDRESS, data, sizeof(data), false);
 * @endcode
 *
 * @param addr   7-bit I²C device address.
 * @param src    Pointer to data buffer to transmit.
 * @param len    Number of bytes to write.
 * @param nostop If true, the transfer does not send a STOP
 *               condition (repeated start).
 *
 * @return @c true if all bytes were written, @c false otherwise.
 */
bool i2c_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop);

/**
 * @brief Read data from an I²C device.
 *
 * Reads a buffer of bytes from the specified I²C address.
 *
 * Typical usage: reading consecutive registers by first writing
 * the register address, then reading back the data.
 *
 * @code
 * // Example: read 2 bytes of temperature from HDC2021 (low+high registers)
 * uint8_t reg = HDC2021_TEMP_LOW;   // starting register
 * i2c_write(HDC2021_I2C_ADDRESS, &reg, 1, true);   // write register, keep bus active
 *
 * uint8_t data[2] = {0};
 * bool ok = i2c_read(HDC2021_I2C_ADDRESS, data, 2, false);
 *
 * uint16_t raw = ((uint16_t)data[1] << 8) | data[0]; // combine MSB+LSB
 * float temp_c = (raw * 165.0f / 65536.0f) - 40.0f;  // convert to Celsius
 * @endcode
 *
 * @param addr   7-bit I²C device address.
 * @param dst    Pointer to destination buffer.
 * @param len    Number of bytes to read.
 * @param nostop If true, the transfer does not send a STOP
 *               condition (repeated start).
 *
 * @return @c true if all bytes were read, @c false otherwise.
 */
bool i2c_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop);


/* =========================
 *  DISPLAY SSD1306
 * ========================= */
// Datasheet can be found at: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
// Library used can be found at: https://github.com/daschr/pico-ssd1306https://github.com/daschr/pico-ssd1306

/**
 * @defgroup Display SSD1306 (I²C addr 0x3C)
 * @brief Convenience API for the 128×64 SSD1306 I²C OLED (addr 0x3C).
 *
 * Uses the bundled pico-ssd1306 library to draw text and simple shapes.
 *
 * @see https://github.com/daschr/pico-ssd1306
 * @see SSD1306 datasheet: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 * @{
 */

/**
 * @brief Initialize the SSD1306 OLED (I²C addr 0x3C).
 *
 * Sets up an internal @c ssd1306_t instance (128×64), powers the panel on,
 * and clears the off-screen buffer.
 *
 * @pre Call @c init_i2c_default before this function.
 *
 * @post The display is powered on and ready to draw.
 */
void init_display(void);

/**
 * @brief Write a text string centered-ish on the display.
 *
 * Draws @p text at a predefined position with a larger font scale (2),
 * then updates the panel.
 *
 * @param text Null-terminated C string. Ignored if @c NULL.
 *
 * @note This helper calls @c ssd1306_show() internally.
 * @see write_text_xy()
 */
void write_text(const char *text);

/**
 * @brief Write a text string starting at (x0, y0).
 *
 * Renders @p text into the off-screen buffer at position (x0,y0) with
 * font scale 1 and then updates the panel.
 *
 * Coordinate system: origin (0,0) = top-left; X→right, Y→down.
 *
 * @param x0  Start X in pixels (values < 0 are clamped to 0).
 * @param y0  Start Y in pixels (values < 0 are clamped to 0).
 * @param text Null-terminated C string. Ignored if @c NULL.
 *
 * @note Calls @c ssd1306_show() internally.
 */
void write_text_xy(int16_t x0, int16_t y0, const char *text);

/**
 * @brief Set the text cursor for subsequent text rendering.
 *
 * Changes the logical text origin to (x0,y0) for text-drawing helpers
 * that use a cursor-based workflow.
 *
 * @param x0 Cursor X in pixels.
 * @param y0 Cursor Y in pixels.
 *
 * @note If you only use @ref write_text_xy, you may not need this.
 */
void set_text_cursor (int16_t x0, int16_t y0);

/**
 * @brief Draw a circle centered at (x0, y0).
 *
 * Uses a midpoint (Bresenham) algorithm to render an outline or a filled disk
 * into the off-screen buffer and then updates the panel.
 *
 * @param x0   Center X.
 * @param y0   Center Y.
 * @param r    Radius in pixels (>= 0).
 * @param fill If @c true, draws a filled disk; otherwise, only the outline.
 *
 * @post Calls @c ssd1306_show() once at the end.
 * @complexity O(r)
 */
void draw_circle(int16_t x0, int16_t y0, int16_t r, bool fill);

/**
 * @brief Draw a line from (x0, y0) to (x1, y1).
 *
 * Renders the line into the off-screen buffer and updates the panel.
 *
 * @param x0 Start X.
 * @param y0 Start Y.
 * @param x1 End X.
 * @param y1 End Y.
 *
 * @note Calls @c ssd1306_show() internally.
 */
void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1);

/**
 * @brief Draw a rectangle at (x, y) with width @p w and height @p h.
 *
 * Draws either a filled or an outline rectangle into the off-screen buffer
 * and updates the panel.
 *
 * @param x  Top-left X.
 * @param y  Top-left Y.
 * @param w  Width in pixels.
 * @param h  Height in pixels.
 * @param fill If @c true, filled rectangle; otherwise, outline only.
 *
 * @note Calls @c ssd1306_show() internally.
 */
void draw_square(uint32_t x, uint32_t y, uint32_t w, uint32_t h, bool fill);

/**
 * @brief Clear the display.
 *
 * Clears the off-screen buffer and updates the panel (screen goes blank).
 */
void clear_display(void);

/**
 * @brief Power off the OLED panel.
 *
 * Sends panel power-off; the internal buffer remains allocated in RAM.
 * Call @ref init_display to power the panel on again and reinitialize.
 */
void stop_display(void);

/** @} */ // end of group Display



/* =========================
 *  LIGHT SENSOR VEML6030
 * =========================  */
// Useful info at: https://learn.sparkfun.com/tutorials/qwiic-ambient-light-sensor-veml6030-hookup-guide/all#arduino-library
// Programming application: https://www.vishay.com/docs/84367/designingveml6030.pdf
// Datasheet: https://www.vishay.com/docs/84366/veml6030.pdf

/**
 * @defgroup VEML6030 Ambient light sensor (VEML6030, I²C addr 0x10)
 * @brief API to configure and read the Vishay VEML6030 light sensor.
 *
 * The VEML6030 is a high-sensitivity ambient light sensor with a 16-bit
 * dynamic range. On the JTKJ HAT it is connected via the default I²C bus
 * at address 0x10.
 *
 * @see SparkFun guide: https://learn.sparkfun.com/tutorials/qwiic-ambient-light-sensor-veml6030-hookup-guide/all#arduino-library
 * @see Application note: https://www.vishay.com/docs/84367/designingveml6030.pdf
 * @see Datasheet: https://www.vishay.com/docs/84366/veml6030.pdf
 * @{
 */

/**
 * @brief Initialize the VEML6030 light sensor.
 *
 * Configures the sensor with default parameters:
 * - Gain: 1/8
 * - Integration time: 100 ms
 * - Power: ON
 * - Interrupts: disabled
 *
 * @note Call @c init_i2c_default before this function.
 */
void init_veml6030(void);

/**
 * @brief Read the current light level in lux.
 *
 * Fetches the raw ALS (ambient light sensing) output and applies the
 * recommended conversion factor. For high-light conditions, a polynomial
 * correction is applied (per Vishay app note).
 *
 * @return Ambient light level in lux.
 *
 * @note Ensure that sufficient time has passed since the last sample
 *       (> integration time, i.e. 100 ms).
 */
uint32_t veml6030_read_light(void);

/**
 * @brief Power down the VEML6030.
 *
 * Places the sensor into a low-power OFF mode by setting the power bit.
 *
 * @note Call @ref veml6030_init again to power it back on and reconfigure.
 */
void veml6030_stop(void);

/** @} */ // end of group VEML6030



/* ===============================================
 *  TEMPERATURE AND HUMIDITY SENSOR HDC 2021
 * ===============================================  */
// Temperature & Humidity sensor related function
// Datasheet: https://www.ti.com/lit/ds/symlink/hdc2021.pdf?ts=1757522824481&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FHDC2021
// Usage: https://www.ti.com/lit/ug/snau250/snau250.pdf?ts=1757438909914

/**
 * @defgroup HDC2021 Temperature & Humidity sensor (HDC2021, I²C addr 0x40)
 * @brief API for the Texas Instruments HDC2021 digital temperature
 * and humidity sensor.
 *
 * On the JTKJ HAT, the HDC2021 is connected via the default I²C bus
 * at address 0x40. It continuously measures temperature and humidity
 * at 1 Hz with 14-bit resolution.
 *
 * Default thresholds configured by ::hdc2021_init():
 * - Temperature low:  -30 °C
 * - Temperature high: +50 °C
 * - Humidity low:       0 %
 * - Humidity high:    100 %
 *
 * These thresholds correspond to the sensor’s alert mechanism.
 * However, **interrupt/alert handling is not yet implemented
 * in this SDK**. Threshold functions are provided for completeness
 * but are not required in normal polling mode.
 *
 * For simple use (polling values), it is sufficient to call
 * ::hdc2021_init(), then use ::hdc2021_read_temperature() and
 * ::hdc2021_read_humidity().
 *
 * @see Datasheet: https://www.ti.com/lit/ds/symlink/hdc2021.pdf?ts=1757522824481&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FHDC2021
 * @see Usage Guide: https://www.ti.com/lit/ug/snau250/snau250.pdf?ts=1757438909914
 * @{
 */

/**
 * @brief Initialize the HDC2021 sensor.
 *
 * Performs a soft reset and configures the device with default
 * thresholds and continuous measurement mode:
 * - 1 Hz sampling
 * - 14-bit resolution for both temperature and humidity
 * - Thresholds: -30 °C low, +50 °C high, 0 % low, 100 % high
 * 
 * @note Call @c init_i2c_default before this function.
 * 
 */
void init_hdc2021_(void);

/**
 * @brief Power down the HDC2021 sensor.
 *
 * Stops continuous measurements and puts the sensor in a low-power mode.
 */
void stop_hdc2021(void);

/**
 * @brief Set low temperature threshold for alerts.
 *
 * @param temp Threshold in °C (-40 to +125).
 */
void hdc2021_set_low_temp_threshold(float temp);

/**
 * @brief Set high temperature threshold for alerts.
 *
 * @param temp Threshold in °C (-40 to +125).
 */
void hdc2021_set_high_temp_threshold(float temp);

/**
 * @brief Set high humidity threshold for alerts.
 *
 * @param humid Threshold in %RH (0–100).
 */
void hdc2021_set_high_humidity_threshold(float humid);

/**
 * @brief Set low humidity threshold for alerts.
 *
 * @param humid Threshold in %RH (0–100).
 */
void hdc2021_set_low_humidity_threshold(float humid);

/**
 * @brief Read temperature in Celsius.
 *
 * @return Temperature as a floating-point value in °C.
 *
 * @note Default measurement rate is 1 Hz, resolution 14 bits.
 */
float hdc2021_read_temperature(void);

/**
 * @brief Read relative humidity in %.
 *
 * @return Relative humidity as a floating-point value in percent (0–100).
 *
 * @note Default measurement rate is 1 Hz, resolution 14 bits.
 */
float hdc2021_read_humidity(void);

/** @} */ // end of group HDC2021


/* =========================
 *  ICM-42670 (IMU)
 * ========================= */
// Datasheet: https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf

/**
 * @defgroup ICM42670 IMU (ICM-42670-P, I²C addr 0x69)
 * @brief API for the TDK InvenSense ICM-42670 6-axis accelerometer + gyroscope.
 *
 * ### Recommended defaults
 * - Accelerometer: ±@ref ICM42670_ACCEL_FSR_DEFAULT g,
 *                  @ref ICM42670_ACCEL_ODR_DEFAULT Hz
 *   (typically: ±4 g @ 100 Hz)
 * - Gyroscope:    ±@ref ICM42670_GYRO_FSR_DEFAULT dps,
 *                  @ref ICM42670_GYRO_ODR_DEFAULT Hz
 *   (typically: ±250 dps @ 100 Hz)
 *
 * ### Modes
 * - This SDK supports **Low-Noise (LN) mode** (higher precision, higher power).
 * - Other modes (LP/ULP/hybrid) are not yet implemented in this SDK.
 *
 * @see Datasheet: https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf
 * @{
 */

/**
 * @brief Initialize the IMU.
 *
 * Performs a soft reset and check the connection.
 *
 * @return 0 on success, negative value on error.
 * 
 * @note Call @c init_i2c_default before this function.
 */
int init_ICM42670(void);

/**
 * @brief Start accelerometer measurements.
 *
 * Configures the accelerometer with the requested ODR and FSR.
 *
 * @param odr_hz Desired output data rate in Hz (e.g., @ref ICM42670_ACCEL_ODR_DEFAULT).
 * @param fsr_g  Full-scale range in g (2, 4, 8, 16; e.g., @ref ICM42670_ACCEL_FSR_DEFAULT).
 *
 * @return 0 on success, negative value on error.
 */
int ICM42670_startAccel(uint16_t odr_hz, uint16_t fsr_g);

/**
 * @brief Start gyroscope measurements.
 *
 * Configures the gyroscope with the requested ODR and FSR.
 *
 * @param odr_hz  Desired output data rate in Hz (e.g., @ref ICM42670_GYRO_ODR_DEFAULT).
 * @param fsr_dps Full-scale range in degrees/second (250, 500, 1000, 2000; e.g., @ref ICM42670_GYRO_FSR_DEFAULT).
 *
 * @return 0 on success, negative value on error.
 */
int ICM42670_startGyro(uint16_t odr_hz, uint16_t fsr_dps);

/**
 * @brief Enable low-noise (LN) mode for both accelerometer and gyroscope.
 *
 * @return 0 on success, negative value on error.
 */
int ICM42670_enable_accel_gyro_ln_mode(void);

/**
 * @brief Start IMU with SDK default settings and enable LN mode.
 *
 * Calls:
 * - ::ICM42670_startAccel(@ref ICM42670_ACCEL_ODR_DEFAULT,
 *                         @ref ICM42670_ACCEL_FSR_DEFAULT)
 * - ::ICM42670_startGyro (@ref ICM42670_GYRO_ODR_DEFAULT,
 *                         @ref ICM42670_GYRO_FSR_DEFAULT)
 * - ::ICM42670_enable_accel_gyro_ln_mode()
 *
 * @pre Call ::ICM42670_init() successfully before this function.
 *
 * @return 0 on success, negative error code from the first failing call.
 */
int ICM42670_start_with_default_values(void);

/**
 * @brief Read accelerometer, gyroscope, and temperature data.
 *
 * Units:
 * - Acceleration (@p ax, @p ay, @p az) in **g** (where 1 g ≈ 9.81 m/s²).
 *   At rest, the axis aligned with gravity will read about **±1 g**
 * - Angular rate (@p gx, @p gy, @p gz) in **degrees/second (dps)**.
 * - Temperature (@p t) in **°C**.
 *
 * @param ax Pointer to store accel X (g).
 * @param ay Pointer to store accel Y (g).
 * @param az Pointer to store accel Z (g).
 * @param gx Pointer to store gyro X (dps).
 * @param gy Pointer to store gyro Y (dps).
 * @param gz Pointer to store gyro Z (dps).
 * @param t  Pointer to store temperature (°C).
 *
 * @return 0 on success, negative value on error.
 */
int ICM42670_read_sensor_data(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz,
                              float *t);

/** @} */ // end of group ICM42670


#endif /* SDK_H */
