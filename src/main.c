
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
enum state { IDLE=1, SENDING, RECEIVING};
volatile enum state programState = IDLE;
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
    if (gpio == BUTTON1 && eventMask == GPIO_IRQ_EDGE_FALL && programState == IDLE) //if interrupt came from button 1 and the event is press then change state
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

static void show_morse(char message[])
{
    clear_display();
    write_text_xy(0,0, "Morse Code:");

    int length = strlen(message);

    for (int i = 0; i < length; i+=20)
    {
        char line[21]; //20 characters + null terminator
        strncpy(line, message + i, 20);
        line[20] = '\0'; // Ensure null termination
        write_text_xy(0, (i/20) * 10 + 10, line); // Display each line with a vertical offset
    }
    
    for (int i = 0; i < length; i++)
    {
        if (message[i] == '.')
        {
            set_red_led_status(true);
            vTaskDelay(pdMS_TO_TICKS(200)); //dot duration
            set_red_led_status(false);
            vTaskDelay(pdMS_TO_TICKS(200)); //pause between symbols
        }
        else if (message[i] == '-')
        {
            set_red_led_status(true);
            vTaskDelay(pdMS_TO_TICKS(600)); //dash duration
            set_red_led_status(false);
            vTaskDelay(pdMS_TO_TICKS(200)); //pause between symbols
        }
        else if (message[i] == ' ')
        {
            if (message[i+1] == ' ') //check if next character is also a space
            {
                vTaskDelay(pdMS_TO_TICKS(1200)); //pause between words
                i++; //skip the next space since we already handled it
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(400)); //pause between letters
            }
        }
    }
}

static void receive_task(void *arg){
    char message[BUFFER_SIZE] = ""; //initialize buffer so it wont contain garbage
    int index = 0; //index for buffer
    
    for (;;)
    {
        if (programState != SENDING)
        {
            int c = getchar_timeout_us(0); //non-blocking read
            
            if (c != PICO_ERROR_TIMEOUT) //if there is data to read
            {
                if (c == '\r')  //ignore carriage return
                {
                    continue;
                }
                else if (c == '\n')
                {
                    if (index >= 2 && message[index-1] == ' ' && message[index-2] == ' ')
                    { message[index-2] = '\0'; //replace the last two spaces with null terminator to end the string
                      show_morse(message); }
                    
                    index = 0; //reset index for next message
                    programState = IDLE; //reset state to idle after receiving a message
                }
            else if (index < BUFFER_SIZE - 1) //check to avoid buffer overflow
                {
                message[index] = (char)c; //store the character in the buffer
                index++;
                programState = RECEIVING; //set state to receiving when data is being received
                }
            }
                    
        }

        vTaskDelay(pdMS_TO_TICKS(20)); //slightly longer delay to avoid double press
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
    init_display();
    init_led();

    TaskHandle_t IMUtask = NULL;
    TaskHandle_t morseTask = NULL;
    TaskHandle_t lighttask = NULL;
    TaskHandle_t receiveTask = NULL;
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

    result = xTaskCreate(receive_task,       // (en) Task function
                "receive",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &receiveTask);    // (en) A handle to control the execution of this task
    
    if (result != pdPASS) {
        printf("receive_task creation failed\n");
        return 0;
    }

    // setting a new interrupt for button 1, event click
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, button_handler);


    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}
