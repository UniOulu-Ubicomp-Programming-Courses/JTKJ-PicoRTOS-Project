/*
 * hat_app / main.c
 *
 * Course: Computer Systems
 * Project: JTKJ – PicoRTOS Project (Tier 1)
 * Device: Raspberry Pi Pico W + JTKJ Hat
 *
 * Authors:
 *   Tatu Kari 
 *   Elias Peltokorpi
 *   Eemil Holma
 * 
 * Friend A: Input Task
 * - Implemented IMU tilt detection for dot/dash.
 * - Coded button press logic.
 * - Tested motion recognition.

 * Eemil: Encoder Task
 * - Implemented encoder logic converting events into Morse output.
 * - Managed queue communication.
 * - Formatted messages according to protocol.

 * Tatu: Integration + Support
 * - Built system setup (tasks, queues, init).
 * - Wrote message buffer, reset logic, debug tools.
 * - Helped with IMU thresholds and button logic.
 * - Ensured the whole project compiles, runs, and follows Tier 1 rules.

*
 * Description:
 *   This program reads motions from the IMU sensor and button presses
 *   from the JTKJ Hat and converts them into Morse code. The Morse
 *   code is sent over USB (stdio) to the Serial Client program.
 *
 *   - IMU movement -> DOT (.) or DASH (-)
 *   - BUTTON1 press -> spaces and end of message
 *   - Encoder task formats events into Morse protocol:
 *         one space between letters,
 *         two spaces between words,
 *         two spaces + '\n' at end of message.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tkjhat/sdk.h"

#define TX_BUF_LEN             256
#define MORSE_Q_LEN            32

// Morse events

typedef enum {
    EV_DOT,         // '.'
    EV_DASH,        // '-'
    EV_GAP_LETTER,  // space between letters
    EV_GAP_WORD,    // space between words
    EV_END_MSG      // end of message ("  \n")
} morse_event_t;

// Global queue for Morse events
static QueueHandle_t g_morse_q = NULL;

// Forward declarations of tasks
static void status_task(void *arg);
static void input_task(void *arg);
static void encoder_task(void *arg);

// Helper to get current time in ms
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// status_task: simple status task, prints once per second

static void status_task(void *arg) {
    (void)arg;

    // Wait until USB is connected
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    printf("hat_app started\n");

    while (1) {
        printf("status: running...\n");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

// input_task: IMU + BUTTON1 -> Morse events
//
//  - IMU:
//     X-axis tilt -> EV_DOT
//     Y-axis tilt -> EV_DASH
//
//  - BUTTON1:
//      1st press   -> EV_GAP_LETTER
//      2nd press  -> EV_GAP_WORD
//      3rd press    -> EV_END_MSG

static void input_task(void *arg)
{
    (void)arg;

    // Settings
    const uint32_t SAMPLE_MS       = 20;    // IMU sampling period
    const float    MOVE_THRESHOLD  = 0.35f; // movement starts
    const float    STILL_THRESHOLD = 0.15f; // movement ends
    const uint32_t MOVEMENT_DELAY_MS = 300; // min delay between movements

    // IMU variables
    float ax, ay, az, gx, gy, gz, t;

    // Button variables
    bool btn_prev          = false;
    bool btn_now           = false;
    int button_press_count = 0;        // Count button presses

     // Tilt tracking variables
    bool x_tilted = false;  // Whether the device was tilted along X-axis
    bool y_tilted = false;  // Whether the device was tilted along Y-axis
    uint32_t last_movement_time = 0; // Last time when movement was detected

    // Message storage (Morse code buffer)
    char morse_message[512];  // Buffer to store the Morse code message
    int morse_msg_index = 0;  // Index to keep track of where we are in the buffer

    // Init JTKJ Hat
    init_hat_sdk();

    // Init IMU (ICM-42670P)
    if (init_ICM42670() == 0) {
        printf("IMU init OK\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("IMU start default values failed\n");
        }
    } else {
        printf("IMU init FAILED\n");
    }

    // Init BUTTON1 (from TKJHAT)
    init_sw1();
    gpio_pull_up(BUTTON1);   // active-low

    while (1) {
        uint32_t now = now_ms();

        // ---- 1) Read IMU and detect tilt for dot/dash ----
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            // Calculate total acceleration magnitude (normalize to 1g)
            float amag = sqrtf(ax * ax + ay * ay + az * az);
            // How much extra acceleration is above gravity (1g)
            float extra = fabsf(amag - 1.0f);

            // Check if the device is stationary (ax ~ 0, ay ~ 0, az ~ 1)
            bool is_stationary = (fabsf(ax) < 0.1f && fabsf(ay) < 0.1f && fabsf(az - 1.0f) < 0.1f);

            // Check if enough time has passed since last movement
            bool movement_cooldown_expired = (now - last_movement_time) > MOVEMENT_DELAY_MS;

            // If stationary, only handle button presses for space and don't print dot/dash
        if (is_stationary) {
                // Reset tilt flags if stationary
                x_tilted = false;
                y_tilted = false;
            } else {
                // If the device is not stationary, process the tilt for dot or dash
                if (fabsf(ax) > MOVE_THRESHOLD && !x_tilted) {
                    // If there's significant tilt along the X-axis, print a dot
                    morse_event_t ev = EV_DOT;
                    xQueueSend(g_morse_q, &ev, portMAX_DELAY);

                    x_tilted = true;  // Prevent printing the same event repeatedly
                    button_press_count = 1; // Reset button press count on dot

                } else if (fabsf(ay) > MOVE_THRESHOLD && !y_tilted) {
                    // If there's significant tilt along the Y-axis, print a dash
                    morse_event_t ev = EV_DASH;
                    xQueueSend(g_morse_q, &ev, portMAX_DELAY);

                    y_tilted = true;  // Prevent printing the same event repeatedly
                    button_press_count = 1; // Reset button press count on dash

                }
            }
        }

        // Read BUTTON1 and detect gaps/end of message
        btn_now = (gpio_get(BUTTON1) == 0);  // active-low

        // Button pressed down
        if (btn_now && !btn_prev) {
            button_press_count++;  // Increment press count on each press
        }

        // Button released (now we process the press count)
        if (!btn_now && btn_prev) {
            if (button_press_count == 1) {
                // Single press: send letter gap event
                morse_event_t ev = EV_GAP_LETTER;
                xQueueSend(g_morse_q, &ev, portMAX_DELAY);

            } else if (button_press_count == 2) {
                // Double press: send word gap event  
                morse_event_t ev = EV_GAP_WORD;
                xQueueSend(g_morse_q, &ev, portMAX_DELAY);

            } else if (button_press_count == 3) {
                // Triple press: send end message event
                morse_event_t ev = EV_END_MSG;
                xQueueSend(g_morse_q, &ev, portMAX_DELAY);

                // Reset press count for next sequence
                button_press_count = 0;
            }
        }

        btn_prev = btn_now;

        // Sleep a bit before reading the next sample
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}



// encoder_task: converts events to Morse string and prints to USB
//
// Rules:
//   EV_DOT  -> '.'
//   EV_DASH -> '-'
//   EV_GAP_LETTER -> one space ' '
//   EV_GAP_WORD   -> two spaces "  "
//   EV_END_MSG    -> ensure string ends with "  \n" and send it

static void encoder_task(void *arg)
{
    (void)arg;

    char   out[TX_BUF_LEN];
    size_t len = 0;

    // initialize buffer
    memset(out, 0, sizeof(out));

    while (1) {
        morse_event_t ev;

        if (xQueueReceive(g_morse_q, &ev, portMAX_DELAY) == pdTRUE) {

            switch (ev) {
            case EV_DOT:
                if (len + 1 < TX_BUF_LEN) {
                    out[len++] = '.';
                    printf("X-axis tilt -> DOT\n");
                }
                break;

            case EV_DASH:
                if (len + 1 < TX_BUF_LEN) {
                    out[len++] = '-';
                    printf("Y-axis tilt -> DASH\n");
                }
                break;

            case EV_GAP_LETTER:
                if (len && out[len - 1] != ' ' && len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                    printf("Button pressed once: Added letter gap\n");
                }
                break;

            case EV_GAP_WORD:
                if (len && out[len - 1] != ' ' && len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                }
                if (len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                    printf("Button pressed twice: Added word gap\n");
                }
                break;

            case EV_END_MSG:
                // ensure we end with "  \n"
                if (len && out[len - 1] != ' ' && len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                }
                if (len + 2 < TX_BUF_LEN) {
                    out[len++] = ' ';
                    out[len++] = '\n';
                }

                printf("\n=== MESSAGE COMPLETE ===\n");
                printf("Message: %.*s", (int)len, out);
                printf("Sending to Serial Client...\n");

                // send to Serial Client
                fflush(stdout);

                printf("Message sent and reset.\n\n");

                // reset buffer
                len = 0;
                memset(out, 0, sizeof(out));
                break;
            }
        }
    }
}


// main: initializes everything and starts FreeRTOS scheduler

int main(void)
{
    // Initialize stdio over USB
    stdio_init_all();

    // Small delay so USB is ready
    sleep_ms(200);

    // Create Morse event queue
    g_morse_q = xQueueCreate(MORSE_Q_LEN, sizeof(morse_event_t));
    if (g_morse_q == NULL) {
        // Failed to create queue, stop here
        while (1) {
            sleep_ms(1000);
        }
    }

    // Create tasks
    xTaskCreate(status_task,  "status",  1024, NULL, 1, NULL);
    xTaskCreate(input_task,   "input",   2048, NULL, 2, NULL);
    xTaskCreate(encoder_task, "encoder", 2048, NULL, 2, NULL);

    // Start scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (1) {
    }

    return 0;
}
