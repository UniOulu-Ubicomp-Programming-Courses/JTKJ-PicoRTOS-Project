/*
 * morse_project.c
 *
 * Course: JTKJ – PicoRTOS Project
 * Board:  Raspberry Pi Pico W + JTKJ Hat
 *
 * Overview
 * --------
 * This program sends Morse-coded messages over USB to the Serial Client.
 * It is structured with FreeRTOS tasks so that different team members
 * can work independently:
 *
 * - Person A:        RTOS setup, queues, main() – already mostly done.
 * - Person B:        Implement INPUT task:
 *                      - Read button OR IMU
 *                      - Detect DOT / DASH based on duration
 *                      - Detect letter / word gaps
 *                      - Push events into g_morse_q
 * - Person C:        Implement ENCODER task:
 *                      - Convert events into ".- " string
 *                      - Follow the course Morse protocol
 *                      - Print to USB (printf) so Serial Client can decode
 *
 * Morse Protocol
 * -----------------------------------
 * - Each character is encoded as dots '.' and dashes '-'
 * - Characters separated by ONE space ' '
 * - Words separated by TWO spaces "  "
 * - The message ENDS with TWO spaces and a newline: "  \n"
 *
 * NOTE: This file DOES NOT translate Morse ↔ letters.
 *       It only sends/receives dots, dashes and spaces.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tkjhat/sdk.h"

// ==================== Morse config (shared) ====================
// These are timing constants used for both sending and interpreting
// Morse. All times are in milliseconds.

#define DOT_MS                 150                // Duration of a DOT
#define DASH_MS                (3 * DOT_MS)       // Duration of a DASH
#define INTRA_SYMBOL_GAP_MS    (1 * DOT_MS)       // Gap inside a character (between dot/dash)
#define INTER_LETTER_GAP_MS    (3 * DOT_MS)       // Gap between characters -> ONE space
#define INTER_WORD_GAP_MS      (7 * DOT_MS)       // Gap between words     -> TWO spaces

// Maximum length of one outgoing Morse message (including spaces + '\n')
#define TX_BUF_LEN             256

// Max number of Morse events queued at once
#define MORSE_Q_LEN            32

// ==================== Shared event type & queue ====================
//
// This enum describes the "Morse events" that the INPUT task produces
// and the ENCODER task consumes.

typedef enum {
    EV_DOT,         // A single DOT    ('.')
    EV_DASH,        // A single DASH   ('-')
    EV_GAP_LETTER,  // Gap between characters  -> ONE space ' '
    EV_GAP_WORD,    // Gap between words       -> TWO spaces "  "
    EV_END_MSG      // End of message          -> "  \n"
} morse_event_t;

// Global queue handle accessible from all tasks.
// Person B: you only SEND events to this queue.
// Person C: you only RECEIVE events from this queue.
static QueueHandle_t g_morse_q;

// Forward declarations of tasks so each person knows where to work.
static void diag_task(void *arg);    // Simple "I'm alive" debug task (Person A)
static void input_task(void *arg);   // INPUT (Person B)
static void encoder_task(void *arg); // ENCODER (Person C)

// Small time helper using the Pico SDK
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ===================================================================
// ==================== Person A: RTOS / main ========================
// ===================================================================
//
// Person A responsibilities here:
//  - Initialize stdio (USB), JTKJ Hat SDK.
//  - Create the global queue g_morse_q.
//  - Create all FreeRTOS tasks (diag, input, encoder).
//  - Start the scheduler.
//
// After this is working, Person B and C can focus on their tasks
// without touching main() or the CMake configuration.

// Simple debug task: prints once per second so we know RTOS is running.
static void diag_task(void *arg) {
    (void)arg;
    for (;;) {
        printf("__rtos alive__\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    // Initialize USB stdio (printf -> USB Serial).
    stdio_init_all();

    // Initialize the JTKJ Hat SDK.
    // This prepares I/O, I2C, IMU, etc. Even if you don't use all sensors,
    init_hat_sdk();

    // Create Morse event queue.
    //  - Producer: input_task  (Person B)
    //  - Consumer: encoder_task (Person C)
    g_morse_q = xQueueCreate(MORSE_Q_LEN, sizeof(morse_event_t));
    if (g_morse_q == NULL) {
        printf("Queue create failed\n");
        while (1) { tight_loop_contents(); }
    }

    // Create tasks.
    //
    // diag_task   : debug prints every 1s.
    // input_task  : PERSON B - Read input (button/IMU) and generate events.
    // encoder_task: PERSON C - Convert events to Morse string and print to USB.
    //
    // Stack sizes and priorities are chosen conservatively.
    // They can be adjusted later if needed.
    xTaskCreate(diag_task,    "diag",    1024, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(input_task,   "input",   2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(encoder_task, "encoder", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);

    // Start the FreeRTOS scheduler. From this point on, tasks run.
    vTaskStartScheduler();

    // If the scheduler ever returns, something went wrong.
    while (1) { }
}

// ===================================================================
// ==================== Person B: INPUT TASK =========================
// ===================================================================
//
// Person B's job: REPLACE the demo pattern below with real detection.
//
// Requirements:
//   - Use either a BUTTON or IMU data from JTKJ Hat.
//   - Measure how long a "gesture" or button press lasts.
//     * Short duration  -> EV_DOT
//     * Long duration   -> EV_DASH
//   - Measure silence (no movement/press) time between actions.
//     * >= INTER_LETTER_GAP_MS -> EV_GAP_LETTER
//     * >= INTER_WORD_GAP_MS   -> EV_GAP_WORD
//   - When user finishes a message (e.g. special gesture or long idle):
//     * Send EV_END_MSG to tell encoder to output "  \n".
//
// Helper to send an event safely:
//   xQueueSend(g_morse_q, &ev, portMAX_DELAY);

static void input_task(void *arg)
{
    (void)arg;

    // === TEMPORARY DEMO IMPLEMENTATION =================================
    // This part is ONLY for testing the encoder + Serial Client pipeline.
    // It sends a fake pattern ".-  " repeatedly.
    //
    // Person B: Once you're ready, DELETE this whole loop body, and
    // replace it with your actual button/IMU-based recognition.
    // ===================================================================
    for (;;) {
        // Example pattern: ".-  " + end message
        // EV_DOT  -> '.'
        // EV_DASH -> '-'
        // EV_GAP_WORD -> "  " (two spaces)
        // EV_END_MSG -> forces encoder to append final "  \n"
        morse_event_t demo[] = { EV_DOT, EV_DASH, EV_GAP_WORD, EV_END_MSG };

        for (size_t i = 0; i < sizeof(demo)/sizeof(demo[0]); ++i) {
            xQueueSend(g_morse_q, &demo[i], portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(100)); // small delay between events
        }

        // Wait before sending the next demo message.
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// ===================================================================
// ==================== Person C: ENCODER TASK =======================
// ===================================================================
//
// Person C's job: Take the events and build the outgoing Morse string.
//
// Specification (course protocol):
//   - DOT    -> '.'
//   - DASH   -> '-'
//   - Between characters   -> ONE space ' '      (EV_GAP_LETTER)
//   - Between words        -> TWO spaces "  "    (EV_GAP_WORD)
//   - End of message       -> TWO spaces + '\n'  (EV_END_MSG)
//
// Example final string for "aasi on":
//   .- .- ... ..  --- -.  \n
//
// This task does NOT convert dots/dashes back into letters.
// The PC Serial Client will handle that.

static void encoder_task(void *arg)
{
    (void)arg;

    char   out[TX_BUF_LEN] = {0};   // output buffer
    size_t len             = 0;     // current length used

    for (;;) {
        morse_event_t ev;

        // Wait for next event from input_task
        if (xQueueReceive(g_morse_q, &ev, portMAX_DELAY) == pdTRUE) {
            switch (ev) {

            case EV_DOT:
                if (len + 1 < TX_BUF_LEN) {
                    out[len++] = '.';
                }
                break;

            case EV_DASH:
                if (len + 1 < TX_BUF_LEN) {
                    out[len++] = '-';
                }
                break;

            case EV_GAP_LETTER:
                // Only add one space if last char is not already a space
                if (len && out[len - 1] != ' ' && len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                }
                break;

            case EV_GAP_WORD:
                // Ensure exactly two spaces between words
                if (len && out[len - 1] != ' ' && len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                }
                if (len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                }
                break;

            case EV_END_MSG:
                // Finish current message with "  \n"
                // 1) Ensure we end on a space
                if (len && out[len - 1] != ' ' && len + 1 < TX_BUF_LEN) {
                    out[len++] = ' ';
                }
                // 2) Add one more space and newline
                if (len + 2 < TX_BUF_LEN) {
                    out[len++] = ' ';
                    out[len++] = '\n';
                }

                // Send the whole Morse string via USB
                // to the Serial Client. It will interpret
                // the dots/dashes/spaces and show the text.
                printf("%.*s", (int)len, out);
                fflush(stdout);

                // Reset buffer for the next message
                len    = 0;
                out[0] = '\0';
                break;
            } // switch
        } // if xQueueReceive
    } // for(;;)
}
