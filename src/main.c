#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"
#include "translatemorse.h"


// Default stack size for the tasks. It can be reduced to 1024 if task is not using a lot of memory
#define DEFAULT_STACK_SIZE 2048 
// Buffer size for sending morse data to serial client
#define OUTPUT_BUFFER_SIZE 128
// Buffer size for receiving morse data from serial client
#define INPUT_BUFFER_SIZE 256

//Program states. IMU_task and receive_task run continuously on IDLE
enum state { IDLE=1, BUTTON1_PRESSED, BUTTON2_PRESSED, MORSE_READY};
volatile enum state programState = IDLE;

// Character buffer for the current IMU position
static char ch;
// Character array for the received data from serial client
static char line[INPUT_BUFFER_SIZE];

// IMU task
static void IMU_task(void *pvParameters) {
    (void)pvParameters;
    float ax, ay, az, gx, gy, gz, t;
    
    // Setting up the sensor
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }

    // Read sensor data when IDLE and change value of ch depending on the tilting of the device's x-axis
    while (1) {
        if (programState == IDLE) {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            if ((ax >= 0.7) && (ax < 1.1)) {
                ch = ' ';
            }
            else if ((ax < 0.7) && (ax > -0.7)) {
                ch = '.';
            }
            else if ((ax <= -0.7) && (ax > -1.1)) {
                ch = '-';
            }

        } else {
            printf("Failed to read imu data\n");
        }
    }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Interrupt handler for the buttons. Changes programState according to button presses
static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio  == BUTTON1) {
        programState = BUTTON1_PRESSED;
    }
    else if (gpio == BUTTON2) {
        programState = BUTTON2_PRESSED;
    }
}

// Task for updating and sending the morse string to serial client when buttons are pressed
static void print_task(void *arg){
    (void)arg;
    static char text[OUTPUT_BUFFER_SIZE];
    static int i = 0;
    
    while(1){
        // button 1 updates string buffer
        if (programState == BUTTON1_PRESSED) {
            if (i < (OUTPUT_BUFFER_SIZE - 1)) // To prevent overflow
            {
                text[i++] = ch;
            }
            programState = IDLE;
        }
        // button 2 sends the morse string to serial client
        else if (programState == BUTTON2_PRESSED) {
            text[i] = '\0';
            for (int j = 0; text[j] != '\0'; j++) {
                printf("%c", text[j]);
            }
            printf("  \n");
            i = 0;
            programState = IDLE;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// Task for buzzing and displaying the received morse code. Runs when programState is MORSE_READY
static void displaybuzzer_task(void *arg) {
    (void)arg;
    int time = 100; // Duration of buzzer sound in (ms)

    while(1){
        if (programState == MORSE_READY) {
            // Function for decoding and displaying received morse from serial client. (From translatemorse.h)
            display_morse(line, sizeof(line));

            // Loop for sounding the received string in morse
            for(int i = 0; line[i] != '\0'; i++) {
                if (line[i] == '.') {
                    buzzer_play_tone(700, time);
                    vTaskDelay(pdMS_TO_TICKS(time));
                }
                else if (line[i] == '-') {
                    buzzer_play_tone(800, (time * 3)); // A dash is 3 times longer than a dot
                    vTaskDelay(pdMS_TO_TICKS(time));
                }
                else if ((line[i] == ' ') && (line[i + 1] == ' ')) {
                    vTaskDelay(pdMS_TO_TICKS(time * 10)); // Two consecutive spaces includes a longer pause
                }
                else if (line[i] == ' ') {
                    vTaskDelay(pdMS_TO_TICKS(time * 2));
                }

            }
            programState = IDLE;
        }
            vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task for polling and handling the received morse string and handing it to displaybuzzer_task
static void receive_task(void *arg){
    (void)arg;
    size_t index = 0;

    while (1) {
        if (programState == IDLE) {
            int c = getchar_timeout_us(0); // Take one char per time and store it in line array, until reeceived the \n
            if (c != PICO_ERROR_TIMEOUT) { // I have received a character
                if (c == '\r') continue; // Ignore CR, wait for LF
                if (c == '\n') {
                    // Terminate and process the collected line
                    line[index] = '\0'; 
                    index = 0;
                    programState = MORSE_READY; // Change programState to MORSE_READY for the displaybuzzer_task
                    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
                }
                else if(index < INPUT_BUFFER_SIZE - 1) {
                    line[index++] = (char)c;
                }
                else { // Overflow: process the previous string and restart the buffer with the new character
                    line[INPUT_BUFFER_SIZE - 1] = '\0';
                    index = 0; 
                    programState = MORSE_READY;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    line[index++] = (char)c; 
                }
            }
            else {
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait until the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 
    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of USB and hat is done
    init_button1();
    init_button2();
    init_buzzer();
    init_display();

    // Interrupt handlers for buttons 1 and 2 
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL, true);
    printf("Start accelerometer\n\n");

    TaskHandle_t hImuTask, hPrintTask, hReceiveTask, hDisplayBuzzerTask = NULL;

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(
                IMU_task,     
                "IMUtask",              
                DEFAULT_STACK_SIZE, 
                NULL,               
                2,                  
                &hImuTask);    

    BaseType_t result2 = xTaskCreate(
                print_task,
                "print",              
                DEFAULT_STACK_SIZE,   
                NULL,                 
                2,                    
                &hPrintTask);         

    BaseType_t result3 = xTaskCreate(
                displaybuzzer_task, 
                "displaybuzzer",                   
                1024,                   
                NULL,                   
                2,                      
                &hDisplayBuzzerTask);         

    BaseType_t result4 = xTaskCreate(
                receive_task,
                "receive",             
                DEFAULT_STACK_SIZE,   
                NULL,                 
                2,                    
                &hReceiveTask);         
    
    // Check that tasks were created successfully
    if(result != pdPASS) {
        printf("imu Task creation failed\n");
        return 0;
    }
    else if (result2 != pdPASS) {
        printf("print Task creation failed\n");
        return 0;
    }
    else if (result3 != pdPASS) {
        printf("displaybuzzer Task creation failed\n");
        return 0;
    }
    else if (result4 != pdPASS) {
        printf("receive Task creation failed\n");
        return 0;
    }

    // Start the scheduler
    vTaskStartScheduler();

    // Never reach this line
    return 0;
}