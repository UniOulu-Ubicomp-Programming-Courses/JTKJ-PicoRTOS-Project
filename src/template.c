
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"


// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 
#define TEXT_BUFFER_SIZE 20

//Add here necessary states
enum state { IDLE=1 };
enum state programState = IDLE;

float ax, ay, az, gx, gy, gz, t;
static char *str[2];
static char *text[TEXT_BUFFER_SIZE];
int i = 0;
static volatile uint8_t button1_pressed, button2_pressed;

static void ImuTask(void *pvParameters) {
    (void)pvParameters;
    
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
    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            
            if ((ax >= 0.7) && (ax < 1.1)) {
                str[0] = " ";
            }
            else if ((ax < 0.7) && (ax > -0.7)) {
                str[0] = ".";
            }
            else if ((ax <= -0.7) && (ax > -1.1)) {
                str[0] = "-";
            }
            /*
            else if ((ax >= -0.9) && (ax < -1.1)) {
                str[0] = "\n";
            }
            
            else if (((ay >= 0.9) && (ay < 1.1)) && ((ax >= -0.1) && (ax < 0.1)) && ((az >= -0.1) && (az < 0.1))) {
                str[0] = "\n";
            }
            */


        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio  == BUTTON1) {
        button1_pressed = true;
    }
        
    else if (gpio == BUTTON2) {
        button2_pressed = true;
    }
}

static void print_task(void *arg){
    (void)arg;
    
    while(1){

        if (button1_pressed) {
            text[i] = str[0];
            i++;
            button1_pressed = false;

        }
        if (button2_pressed) {
            text[i] = NULL;
            for (int j = 0; text[j] != NULL; j++) {
                printf("%s", text[j]);
            }
            printf("  \n");
            i = 0;
            button2_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
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
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL, true);
    printf("Start accelerometer\n\n");

    TaskHandle_t hImuTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(
                ImuTask,       // (en) Task function
                "IMUtask",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &hImuTask);    // (en) A handle to control the execution of this task

    TaskHandle_t hPrintTask;

    BaseType_t result2 = xTaskCreate(
                print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hPrintTask);         // (en) A handle to control the execution of this task


    if(result != pdPASS) {
        printf("imu Task creation failed\n");
        return 0;
    }
    else if (result2 != pdPASS) {
        printf("print Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}
