
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
enum sensormode { IMU=1, LIGHT};
enum sensormode SensorMode = IMU;

//Global datastructure to hold accelerometer data
struct SensorData{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float t;
} data;

//interrupt handler function
static void button_handler(uint gpio, uint32_t eventMask){
    if (gpio == BUTTON1 && eventMask == GPIO_IRQ_EDGE_FALL) //if interrupt came from button 1 and the event is press then change state
        programState = SENDING;

}

static void IMU_Task(void *arg)
{
    // initializes acclerometer
    init_ICM42670();
    ICM42670_start_with_default_values();

    for (;;)
    {
        if (programState == IDLE && SensorMode == IMU)
        {
            //gets data from sensor and reads it into "data"
            ICM42670_read_sensor_data(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz, &data.t);
            // printf("t: %.2f, a: %.2f, %.2f, %.2f,  g: %.2f, %.2f, %.2f\n", data.t, data.ax, data.ay, data.az, data.gx, data.gy, data.gz);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


uint32_t lightdata = 0; //global variable for light data in lux

static void light_task(void *arg)
{
    init_veml6030(); //initializes the light sensor
    for (;;)
    {
        if (programState == IDLE && SensorMode == LIGHT)
        {
            //gets sensor data and reads it into LightData
            lightdata = veml6030_read_light();
        }
    vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// task for reading data and contructing/sending morse message based off of it
static void morse_task(void *arg){
    const float MARGIN_OF_ERROR = 0.8; //cant always hold the device at perfect angle, 80% there should be enough
    char buffer[BUFFER_SIZE] = ""; //initialize buffer so it wont contain garbage
    init_red_led();

    for (;;)
    {
        if (programState == SENDING)
        {
            // based on data; append a symbol at the end of the buffer
            if (data.ay < -1 * MARGIN_OF_ERROR)
            {
                blink_red_led(1);
                sprintf(buffer, strcat(buffer, "  \n"));
                fwrite(buffer, 1, strlen(buffer), stdout); //write the data and flush it just in case
                stdio_flush();
                sprintf(buffer, ""); //reset buffer back to empty
            }
            else if (data.az >= 1 * MARGIN_OF_ERROR)
            {
                blink_red_led(1);
                sprintf(buffer, strcat(buffer, " "));
            }
            else if (data.ax >= 1 * MARGIN_OF_ERROR)
            {
                blink_red_led(1);
                sprintf(buffer, strcat(buffer, "-"));
            }
            else if (data.ax <= -1 * MARGIN_OF_ERROR)
            {
                blink_red_led(1);
                sprintf(buffer, strcat(buffer, "."));
            }
            programState = IDLE;
        }
        vTaskDelay(pdMS_TO_TICKS(300)); //slightly longer delay to avoid double press
    }
}

int main()
{
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    TaskHandle_t IMUtask = NULL;
    TaskHandle_t morseTask = NULL;
    TaskHandle_t lighttask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(IMU_Task,       // (en) Task function
                "IMU",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &IMUtask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("light_task creation failed\n");
        return 0;
    }

    
    result = xTaskCreate(light_task,       // (en) Task function
                "light",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &lighttask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("light_task creation failed\n");
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

    // setting a new interrupt for button 1, event click
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, button_handler);


    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}
