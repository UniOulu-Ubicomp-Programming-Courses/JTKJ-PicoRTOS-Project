


/**
 * @file tkjhat/pins.h
 * \author Iván Sánchez Milara
 * @brief Macros including the pins of the HAT.
 *
 * 
 * @version 0.83
 */
 

/* =========================
 *  Board / pin macros
 * ========================= */

/**
 * @defgroup board_pins Board and pin definitions
 * @brief Default GPIO assignments for peripherals on the JTKJ HAT.
 *
 * @details
 * These macros define the GPIO pin numbers for peripherals and interfaces
 * used by the SDK. They can be referenced directly or overridden at compile time
 * if custom wiring is used.
 *
 * **Default pinout:**
 * | Function | Macro | GPIO | Notes |
 * |-----------|--------|------|-------|
 * | I²C SDA   | @ref DEFAULT_I2C_SDA_PIN | 12 | Default I²C data |
 * | I²C SCL   | @ref DEFAULT_I2C_SCL_PIN | 13 | Default I²C clock |
 * | UART0     | @ref DEFAULT_UART_0      | 0  | Primary UART port |
 * | UART1     | @ref DEFAULT_UART_1      | 1  | Secondary UART port |
 * | SW1       | @ref SW1_PIN / @ref BUTTON1 | 2  | User button 1 |
 * | SW2       | @ref SW2_PIN / @ref BUTTON2 | 22 | User button 2 |
 * | Red LED   | @ref RED_LED_PIN / @ref LED1 | 14 | Onboard LED |
 * | RGB LED R | @ref RGB_LED_R | 18 | RGB red channel |
 * | RGB LED G | @ref RGB_LED_G | 19 | RGB green channel |
 * | RGB LED B | @ref RGB_LED_B | 20 | RGB blue channel |
 * | Buzzer    | @ref BUZZER_PIN | 17 | Active buzzer |
 * | PDM DATA  | @ref PDM_DATA | 16 | Microphone data line |
 * | PDM CLK   | @ref PDM_CLK | 15 | Microphone clock line |
 * | VEML6030 Interrupt | @ref VEML6030_INTERRUPT | 9  | Light sensor INT pin |
 * | HDC2021 Interrupt  | @ref HDC2021_INTERRUPT | 21 | Temp/humidity INT pin |
 * | ICM42670 Interrupt | @ref ICM42670_INT | 6  | IMU INT1 pin |
 *
 * @note These pin numbers correspond to the Raspberry Pi Pico GPIO numbers.
 * @{
 */

/** @name I²C Bus (default)
 *  Pins used for the I²C bus connected to sensors and the display.
 *  @{
 */
#define DEFAULT_I2C_SDA_PIN   12  /**< I²C data pin (SDA) */
#define DEFAULT_I2C_SCL_PIN   13  /**< I²C clock pin (SCL) */
/** @} */

/** @name UART interfaces
 *  @{
 */
#define DEFAULT_UART_0        0   /**< UART0 identifier */
#define DEFAULT_UART_1        1   /**< UART1 identifier */
/** @} */

/** @name Buttons / Switches
 *  @{
 */
#define SW1_PIN               2   /**< SW1 button pin (GPIO 2) */
#define SW2_PIN               22  /**< SW2 button pin (GPIO 22) */
#define BUTTON1               SW1_PIN  /**< Alias for SW1 button */
#define BUTTON2               SW2_PIN  /**< Alias for SW2 button */
/** @} */

/** @name LEDs
 *  @{
 */
#define RED_LED_PIN           14  /**< Onboard red LED pin (GPIO 14) */
#define LED1                  RED_LED_PIN /**< Alias for red LED */
/** @} */

/** @name RGB LED (common-anode)
 *  @{
 */
#define RGB_LED_R             18  /**< RGB LED red channel (GPIO 18) */
#define RGB_LED_G             19  /**< RGB LED green channel (GPIO 19) */
#define RGB_LED_B             20  /**< RGB LED blue channel (GPIO 20) */
/** @} */

/** @name Audio
 *  @{
 */
#define BUZZER_PIN            17  /**< Buzzer control pin (GPIO 17) */
/** @} */

/** @name PDM Microphone
 *  @{
 */
#define PDM_DATA              16  /**< Microphone data input (GPIO 16) */
#define PDM_CLK               15  /**< Microphone clock output (GPIO 15) */
/** @} */

/** @name Interrupts from Sensors
 *  @{
 */
#define VEML6030_INTERRUPT    9   /**< Ambient light sensor interrupt pin */
#define HDC2021_INTERRUPT     21  /**< Temperature & humidity sensor interrupt pin */
#define ICM42670_INT          6   /**< IMU interrupt pin */
/** @} */


/** @} */ /* end of group board_pins */
