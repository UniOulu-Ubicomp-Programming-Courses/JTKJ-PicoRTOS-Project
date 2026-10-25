/*

Version 0.80

MIT License

Copyright (c) 2025 Iván Sánchez Milara

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

#include <string.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <tusb.h>
#include "usbSerialDebug/helper.h"

static SemaphoreHandle_t g_log_mtx;
static const TickType_t wait = pdMS_TO_TICKS(5);

static inline bool cdc0_ready(void) {
    return tud_mounted() && tud_cdc_n_connected(0);
}

bool usb_serial_init(void) {
    g_log_mtx = xSemaphoreCreateMutex();
    return g_log_mtx != NULL;
}

void usb_serial_flush(void) {
    if (!cdc0_ready() || g_log_mtx == NULL) return;

    // Skip this flush if another logger call owns the mutex.
    if (xSemaphoreTake(g_log_mtx, 0) == pdTRUE) {
        tud_cdc_n_write_flush(0);
        xSemaphoreGive(g_log_mtx);
    }
}

bool usb_serial_connected(void) {
    return cdc0_ready();
}

// Application-task helper. Do not call from ISRs or TinyUSB callbacks.
int usb_serial_print(const char *s) {
    if (!s) return -1;
    if (!cdc0_ready() || g_log_mtx == NULL) return 0;
    if (xSemaphoreTake(g_log_mtx, wait) != pdTRUE) return 0;

    size_t n = strlen(s);
    const size_t initial = n;
    unsigned retries = 0;

    while (n > 0) {
        if (!cdc0_ready()) {
            xSemaphoreGive(g_log_mtx);
            return 0;
        }

        uint32_t available = tud_cdc_n_write_available(0);
        uint32_t chunk = n < available ? (uint32_t)n : available;
        uint32_t written = 0;

        if (chunk > 0) {
            written = tud_cdc_n_write(0, s, chunk);
            s += written;
            n -= written;
        }
        tud_cdc_n_write_flush(0);

        if (n > 0 && written == 0) {
            // Give the USB task time to send, but do not wait forever.
            if (retries >= 10) {
                xSemaphoreGive(g_log_mtx);
                return 0;
            }
            retries++;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    xSemaphoreGive(g_log_mtx);
    return (int)(initial - n);
}
