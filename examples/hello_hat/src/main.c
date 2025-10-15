
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

static void leds_task(void *arg) {
    (void)arg;

    // Initialize LED
    init_led();
    printf("Initializing on board led\n");

    while (1) {
        toggle_led();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void display_task(void *arg) {
    (void)arg;

    init_display();
    printf("Initializing display\n");
    static int counter = 0; 

    while(1) {
        
        clear_display();
        char buf[5]; //Store a number of maximum 5 figures 
        sprintf(buf,"%d",counter++);
        write_text(buf);
        vTaskDelay(pdMS_TO_TICKS(4000));
    }

}

static void buzzer_task(void *arg) {
    (void)arg;

    //Initialize the buzzer
    init_buzzer();
    printf("Initializing buzzer\n");

    while(1){
        buzzer_play_tone (440, 500);
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}


int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    // Specially for debugging
    while (!stdio_usb_connected()){
        sleep_ms(10);
    } 
    init_hat_sdk();
    sleep_ms(400); 

    TaskHandle_t ledsTaskHandle, displayTaskHandle, buzzerTaskHandle = NULL;
   
    // Create the single blinky/printf task
    // (en) We create a task
    BaseType_t result = xTaskCreate(leds_task,       // (en) Task function
                "led",              // (en) Name of the task 
                1024,               // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &ledsTaskHandle);   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Led Task created failed\n");
        return 0;
    }

    result = xTaskCreate(buzzer_task,   // (en) Task function
            "buzzer",                   // (en) Name of the task 
                1024,                   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                   // (en) Arguments of the task 
                2,                      // (en) Priority of this task
                &buzzerTaskHandle);     // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Buzzer Task created failed\n");
        return 0;
    }
    result = xTaskCreate(display_task,   // (en) Task function
            "display",                   // (en) Name of the task 
                1024,                    // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                    // (en) Arguments of the task 
                2,                       // (en) Priority of this task
                &displayTaskHandle);     // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Display Task created failed\n");
        return 0;
    }
    // Start the scheduler (never returns)
    vTaskStartScheduler();



    
    return 0;
}

