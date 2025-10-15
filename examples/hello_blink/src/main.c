/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

 //NOTE: THIS PROGRAM ONLY WORKS WITH PICOW
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 1000
#endif

// Perform initialisation
// Should use the WiFi driver to control the LED. 
int pico_led_init(void) {
    return cyw43_arch_init();
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}

int main() {
    // Init the LED
    pico_led_init();
    // Initalize stdout so we can printout
    stdio_init_all();
    // Sending messages to terminal.
    printf("Initiating\n");
   
    while (true) {
        //Turn the led 0n
        pico_set_led(true);
        // Wait 1 s
        sleep_ms(LED_DELAY_MS);
        // Turn the led off
        pico_set_led(false);
        // Wait one second
        sleep_ms(LED_DELAY_MS);
        // Communicate the tick
        printf("Tick\n");
    }
}
