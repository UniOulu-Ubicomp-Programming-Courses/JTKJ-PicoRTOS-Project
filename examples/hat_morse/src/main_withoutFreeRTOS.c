

#include <time.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <string.h>
#include <tusb.h>
#include <pico/stdio.h>

#define BUFFER_SIZE     30
#define TEMP_MIN        0
#define TEMP_MAX        40
#define LUX_MIN         100
#define LUX_MAX         1500
#define CDC_INTERFACE   1

static int rand_in_range(int min, int max) {
    return min + rand() % (max - min + 1);
}

void custom_cdc_task(void);

int main (void) {
    // Initialize TinyUSB first
    tusb_init();
    //Initialize all USB (including stdio as CDC0)
    stdio_init_all();
    while (!tud_cdc_n_connected(0) || !tud_cdc_n_connected(1)){
        tud_task();
        sleep_ms(10);
    }

    //sleep_ms(200);

    //Generating random seed
    srand((unsigned)time(NULL));

    printf("=== Dual CDC Example Started ===\n");
    printf("CDC0: Debug output (this interface)\n");
    printf("CDC1: Sending serial data\n");

    char buf[BUFFER_SIZE]; 
    while (1){
        // Process TinyUSB tasks (this must be called regularly)
        tud_task();

        //Generated random numbers
        int temp = rand_in_range(TEMP_MIN, TEMP_MAX);
        int lux  = rand_in_range(LUX_MIN,  LUX_MAX);

        //Send them using 
        snprintf(buf, BUFFER_SIZE,"temp:%d, light:%d\n", temp, lux);
        printf("[DEBUG] Generated : %s", buf);
        snprintf(buf, BUFFER_SIZE,"%d, %d\n", temp, lux);
        tud_cdc_n_write(CDC_INTERFACE,buf,strlen(buf));
        tud_cdc_n_write_flush(CDC_INTERFACE);
        tud_task();
        sleep_ms(1000);
    }
}

// callback when data is received on a CDC interface
void tud_cdc_rx_cb(uint8_t itf){   
    // allocate buffer for the data in the stack
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    printf("RX CDC %d\n", itf);

    // read the available data 
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    // check if the data was received on the second cdc interface
    if (itf == 1) {
        // process the received data
        buf[count] = 0; // null-terminate the string
        // now echo data back to the console on CDC 0
        printf("Received on CDC 1: %s\n", buf);

        // and echo back OK on CDC 1
        tud_cdc_n_write(itf, (uint8_t const *) "OK\n", 3);
        tud_cdc_n_write_flush(itf);
        }
}