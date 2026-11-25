
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 
#define BUFFER_SIZE 50

//Add here necessary states
enum state { IDLE=1, SENDING};
enum state programState = IDLE;

struct SensorData{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float t;
} data;

static void button_handler(uint gpio, uint32_t eventMask){
    if (gpio == BUTTON2 && eventMask == GPIO_IRQ_EDGE_FALL)
        programState = SENDING;

}

static void sensor_task(void *arg)
{
    init_ICM42670();
    ICM42670_start_with_default_values();

    for (;;)
    {
        if (programState == IDLE)
        {
            rgb_led_write(0,100,0);
            ICM42670_read_sensor_data(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz, &data.t);
            // printf("t: %.2f, a: %.2f, %.2f, %.2f,  g: %.2f, %.2f, %.2f\n", data.t, data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void morse_task(void *arg){
    const float MARGIN_OF_ERROR = 0.3;
    char buffer[BUFFER_SIZE] = "";

    for (;;)
    {
        if (programState == SENDING)
        {
            rgb_led_write(0, 0, 100);
            if (data.az > 1 - MARGIN_OF_ERROR)
            {
                sprintf(buffer, strcat(buffer, " "));
                if (strlen(buffer) >= 2 && buffer[strlen(buffer) - 2] == ' ')
                {
                    sprintf(buffer, strcat(buffer, "\n"));
                    fwrite(buffer, 1, strlen(buffer), stdout);
                    stdio_flush();
                    sprintf(buffer, "");
                }
            }
            else if (data.ax > 1 - MARGIN_OF_ERROR)
                sprintf(buffer, strcat(buffer, "-"));
            else if (data.ax < 1 - MARGIN_OF_ERROR)
                sprintf(buffer, strcat(buffer, "."));
            programState = IDLE;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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

    TaskHandle_t sensorTask = NULL;
    TaskHandle_t morseTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task,       // (en) Task function
                "sensor",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &sensorTask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("sensortask creation failed\n");
        return 0;
    }
    result = xTaskCreate(morse_task,       // (en) Task function
                "morse",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &morseTask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("morsetask creation failed\n");
        return 0;
    }
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, button_handler);

    init_rgb_led();

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

