
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 

//Add here necessary states
enum state { IDLE=1 };
enum state programState = IDLE;

struct SensorData{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float t;
};

static void sensor_task(void *arg){
    struct SensorData data;
    init_ICM42670();
    ICM42670_start_with_default_values();

    for(;;){
        printf("sensortask");
        while (programState == IDLE){
        ICM42670_read_sensor_data(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz, &data.t);
        printf("t: %.2f, a: %.2f, %.2f, %.2f,  g: %.2f, %.2f, %.2f\n", data.t, data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    TaskHandle_t myExampleTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task,       // (en) Task function
                "sensor",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &myExampleTask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Example Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

