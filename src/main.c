#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"
#include "translatemorse.h"


// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 
#define OUTPUT_BUFFER_SIZE 128
#define INPUT_BUFFER_SIZE 256

//Add here necessary states
enum state { IDLE=1, BUTTON1_PRESSED, BUTTON2_PRESSED, BEEP_READY};
volatile enum state programState = IDLE;

static char ch;
static char line[INPUT_BUFFER_SIZE];

static void ImuTask(void *pvParameters) {
    (void)pvParameters;
    float ax, ay, az, gx, gy, gz, t;

    
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);*/
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop. 
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

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio  == BUTTON1) {
        programState = BUTTON1_PRESSED;
    }
    else if (gpio == BUTTON2) {
        programState = BUTTON2_PRESSED;
    }
}

static void print_task(void *arg){
    (void)arg;
    static char text[OUTPUT_BUFFER_SIZE];
    static int i = 0;
    
    while(1){

        if (programState == BUTTON1_PRESSED) {
            if (i < (OUTPUT_BUFFER_SIZE - 1)) {
                text[i++] = ch;
            }
            programState = IDLE;
        }
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

static void buzzerdisplay_task(void *arg) {
    (void)arg;
    int unit = 100;

    while(1){
        if (programState == BEEP_READY) {
        display_morse(line, sizeof(line));
        for(int i = 0; line[i] != '\0'; i++) {
            if (line[i] == '.') {
                buzzer_play_tone(700, unit);
                vTaskDelay(pdMS_TO_TICKS(unit));
            }
            else if (line[i] == '-') {
                buzzer_play_tone(800, (unit * 3));
                vTaskDelay(pdMS_TO_TICKS(unit));
            }
            else if ((line[i] == ' ') && (line[i + 1] == ' ')) {
                vTaskDelay(pdMS_TO_TICKS(unit * 10));
            }
            else if (line[i] == ' ') {
                vTaskDelay(pdMS_TO_TICKS(unit * 2));
            }

        }
        programState = IDLE;
    }
        vTaskDelay(pdMS_TO_TICKS(100));
}
}

static void receive_task(void *arg){
    (void)arg;
    size_t index = 0;
    
    while (1){
        if (programState == IDLE) {
        //OPTION 1
        // Using getchar_timeout_us https://www.raspberrypi.com/documentation/pico-sdk/runtime.html#group_pico_stdio_1ga5d24f1a711eba3e0084b6310f6478c1a
        // take one char per time and store it in line array, until reeceived the \n
        // The application should instead play a sound, or blink a LED.

        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT){// I have received a character
            if (c == '\r') continue; // ignore CR, wait for LF if (ch == '\n') { line[len] = '\0';
            if (c == '\n'){
                // terminate and process the collected line
                line[index] = '\0'; 
                index = 0;
                programState = BEEP_READY;
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
            }
            else if(index < INPUT_BUFFER_SIZE - 1){
                line[index++] = (char)c;
            }
            else { //Overflow: print and restart the buffer with the new character. 
                line[INPUT_BUFFER_SIZE - 1] = '\0';
                index = 0; 
                programState = BEEP_READY;
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
    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    init_button1();
    init_button2();
    init_buzzer();
    init_display();
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL, true);
    printf("Start accelerometer\n\n");

    TaskHandle_t hImuTask, hPrintTask, hReceiveTask, hBuzzerDisplayTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(
                ImuTask,       // (en) Task function
                "IMUtask",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &hImuTask);    // (en) A handle to control the execution of this task


    BaseType_t result2 = xTaskCreate(
                print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hPrintTask);         // (en) A handle to control the execution of this task

    BaseType_t result3 = xTaskCreate(
                buzzerdisplay_task,   // (en) Task function
                "buzzerdisplay",                   // (en) Name of the task 
                1024,                   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                   // (en) Arguments of the task 
                2,                      // (en) Priority of this task
                &hBuzzerDisplayTask);     // (en) A handle to control the execution of this task    

    BaseType_t result4 = xTaskCreate(
                receive_task,  // (en) Task function
                "receive",             // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hReceiveTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("imu Task creation failed\n");
        return 0;
    }
    else if (result2 != pdPASS) {
        printf("print Task creation failed\n");
        return 0;
    }
    else if (result3 != pdPASS) {
        printf("buzzerdisplay Task creation failed\n");
        return 0;
    }
    else if (result4 != pdPASS) {
        printf("receive Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}
