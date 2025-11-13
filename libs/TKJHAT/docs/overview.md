# TKJHAT SDK Overview

The TKJHAT is an educational add-on board for the Raspberry Pi Pico / Pico 2 to be used within in the course Computer Systems and Introduction to Computer Systems in University of Oulu.  
This SDK provides a minimal C API for working with the onboard sensors and actuators.

---

## Peripherals connected to GPIO pins

| Device                 | Interface             | Pin name(s)                      | Notes |
|------------------------|----------------------|----------------------------------|-------|
| Red LED                | GPIO 14              | `RED_LED_PIN` / `LED1`           | Onboard indicator LED (also referred to as “onboard LED”) |
| RGB LED                | GPIO 18:R, 19:G, 20:B| `RGB_LED_R`, `RGB_LED_G`, `RGB_LED_B` | Common-anode LED, driven via PWM |
| Buzzer                 | GPIO 17              | `BUZZER_PIN`                     | Digital output, simple square-wave control |
| PDM MEMS Microphone    | GPIO 16 (DATA), GPIO 15 (CLK) | `PDM_DATA`, `PDM_CLK` | Uses PIO + [Arm Developer Pico microphone library](https://github.com/ArmDeveloperEcosystem/microphone-library-for-pico/tree/main) |


---

## Peripherals connected to I²C

The default I²C bus uses SDA = GPIO 12 (*DEFAULT_I2C_SDA_PIN*) and SCL = GPIO 13 (DEFAULT_I2C_SCL_PIN ).  

| Sensor        | Interface | Address | Datasheet |
|---------------|-----------|---------|-----------|
| Ambient light sensor (VEML6030) | I²C | VEML6030_I2C_ADDR (0x10) | [Vishay VEML6030](https://www.vishay.com/docs/84366/veml6030.pdf) |
| Temperature & humidity (HDC2021) | I²C | HDC2021_I2C_ADDRESS (0x40) | [TI HDC2021](https://www.ti.com/lit/ds/symlink/hdc2021.pdf) |
| IMU (ICM-42670, accel+gyro) | I²C | ICM42670_I2C_ADDRESS (0x69) | [TDK ICM-42670](https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf) |
| OLED display (SSD1306) | I²C  | SSD1306_I2C_ADDRESS (0x3C) | [SSD1306 datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf) |


---

## Notes

- The SDK is intended for teaching: APIs are simplified, and defaults (e.g. 100 Hz ODR, ±4 g accelerometer) are chosen to be practical.  

---

## Authors

- Raisul Islam  
- Iván Sánchez Milara

## Contributors
- Emil Kelhala - Help in initial testing. 

© 2025 — University of Oulu, released under the MIT License.
