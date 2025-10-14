#include <time.h>
#include <stdlib.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include <tusb.h>
#include "usbSerialDebug/helper.h"

#define BUFFER_SIZE     30
#define TEMP_MIN        0
#define TEMP_MAX        40
#define LUX_MIN         100
#define LUX_MAX         1500
#define CDC_ITF_TX      1


/*#if   CFG_TUSB_OS == OPT_OS_FREERTOS
#pragma message "TinyUSB: FreeRTOS"
#elif CFG_TUSB_OS == OPT_OS_PICO
#pragma message("TinyUSB: OS PICO")
#elif CFG_TUSB_OS == OPT_OS_NONE
#pragma message("TinyUSB: sin RTOS")
#endif*/

#if CFG_TUSB_OS != OPT_OS_FREERTOS
#error "This should be using FREERTOS but the CFG_TUSB_OS is not OPT_OS_FREERTOS"
#endif

static int rand_in_range(int min, int max) {
    return min + rand() % (max - min + 1);
}

// ---- Task generating sensor data----
static void sensorTask (void *arg){
    char buf[BUFFER_SIZE]; 

    while (!tud_mounted() || !tud_cdc_n_connected(1)){
            vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (usb_serial_connected()) {
        usb_serial_print("=== Dual CDC Example Started ===\n");
        usb_serial_print("CDC0: Debug output (this interface)\n");
        usb_serial_print("CDC1: Sending serial data\n");
    }
    usb_serial_flush();

    while (1) {
        //Generated random numbers
        int temp = rand_in_range(TEMP_MIN, TEMP_MAX);
        int lux  = rand_in_range(LUX_MIN,  LUX_MAX);
        
        if(tud_cdc_n_connected(CDC_ITF_TX)){
            //Send them using tucdc_n_write to the ACM1
            snprintf(buf, BUFFER_SIZE,"%d, %d\n", temp, lux);
            tud_cdc_n_write(CDC_ITF_TX,buf,strlen(buf));
            tud_cdc_n_write_flush(CDC_ITF_TX);
            
        }
        //Send also the debug log to the ACM0
        if (usb_serial_connected()) {
            snprintf(buf, BUFFER_SIZE,"temp:%d, light:%d\n", temp, lux);
            usb_serial_print(buf);
            usb_serial_flush();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

// ---- Task running USB stack ----
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}

int main (void) {
    //Generating random seed
    srand((unsigned)time(NULL));

    // Create tasks
    TaskHandle_t hUsb   = NULL;
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    xTaskCreate(sensorTask, "app", 1024, NULL, 2, NULL);

    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    
    // VERY IMPORTANT, THIS SHOULD GO JUST BEFORE vTaskStartSheduler
    // WITHOUT ANY DELAYS. OTHERWISE, THE TinyUSB stack wont recognize
    // the device.
    // Initialize TinyUSB 
    tusb_init();
    //Initialize helper library to write in CDC0)
    usb_serial_init();
    vTaskStartScheduler();

}

// callback when data is received on a CDC interface
void tud_cdc_rx_cb(uint8_t itf){   
    // allocate buffer for the data in the stack
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE+1];

    //printf("RX CDC %d\n", itf);

    // read the available data 
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    // check if the data was received on the second cdc interface
    if (itf == 1) {
        // process the received data
        buf[count] = '\n'; 
        buf[count+1] = 0;// null-terminate the string
        // now echo data back to the console on CDC 0
        usb_serial_print("\nReceived on CDC 1:");
        usb_serial_print(buf);

        // and echo back OK on CDC 1
        tud_cdc_n_write(itf, (uint8_t const *) "OK\n", 3);
        tud_cdc_n_write_flush(itf);
        }
}