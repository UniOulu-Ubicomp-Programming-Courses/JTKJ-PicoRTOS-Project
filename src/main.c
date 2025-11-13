
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Exercise 4. Include the libraries necessaries to use the usb-serial-debug, and tinyusb
// Tehtävä 4 . Lisää usb-serial-debugin ja tinyusbin käyttämiseen tarvittavat kirjastot.



#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
#define DEFAULT_I2C_SDA_PIN  12
#define DEFAULT_I2C_SCL_PIN  13
#define DATASIZE 5
#define GYROTHRESHOLD 0.5

enum state {WAITING=1, DATA_READY};
enum state programState = WAITING;

float normal_IMUGyro[3];
float saved_IMUGyro[DATASIZE][3];
int saveData_it = 0;
char *morseMessage = "";

static void btn_fxn(uint gpio, uint32_t eventMask) {
    toggle_red_led();
    if (gpio == BUTTON1) {
        programState = DATA_READY;
    }

    else if (gpio == BUTTON2) {
        // printf("%s", morseMessage);
    }
}

static void sensor_task(void *arg){
    (void)arg;
    for(;;){
        // Collect data when programState == WAITING
        if(programState == WAITING) {
            float ax, ay, az, gx, gy, gz, t;
            if(ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                float all_IMUData[7] = {ax, ay, az, gx, gy, gz, t};
                // Filter and normalize gyroscope absolute values
                for(int i = 3; i < 6; i++) {
                    normal_IMUGyro[i-3] = (fabs(all_IMUData[i])) / (ICM42670_GYRO_FSR_DEFAULT);
                }
                // Save the last n = DATASIZE gyro values into a matrix
                // If the last n = DATASIZE gyro values have been saved, reset saveData_it iterator to 0
                if(saveData_it < DATASIZE){
                    for(int i = 0; i < 3; i++) {
                        saved_IMUGyro[saveData_it][i] = normal_IMUGyro[i];
                    }
                    saveData_it++;
                }
                else {
                    saveData_it = 0;
                }
            }
        }   
        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void print_task(void *arg){
    (void)arg;
    while(1){
        // Print data when programState = DATA_READY
        if(programState == DATA_READY) {
            float maxVal = 0;
            int maxVal_i = 0;
            int maxVal_j = 0;
            printf("\n[ Gx    Gy    Gz ]");

            // Loop through saved_IMUGyro
            for(int i = 0; i < DATASIZE; i++) {
                printf("\n[%.2f, %.2f, %.2f]", saved_IMUGyro[i][0], saved_IMUGyro[i][1], saved_IMUGyro[i][2]);
                
                // Check which axis has the largest value
                for(int j = 0; j < 3; j++) {
                    if(saved_IMUGyro[i][j] > GYROTHRESHOLD && saved_IMUGyro[i][j] > maxVal) {
                        maxVal = saved_IMUGyro[i][j];
                        maxVal_i = i;
                        maxVal_j = j;
                    }
                }
            }

            // Check that maxVal has been given and then print dot, dash or space
            if(maxVal > GYROTHRESHOLD) {
                // Label the detected axis
                char *axis;
                switch(maxVal_j) {
                    case 0:
                        axis = "Gx";
                        break;
                    case 1:
                        axis = "Gy";
                        break;
                    case 2:
                        axis = "Gz";
                        break;
                }
                printf("\nLARGEST SAVED GYRO DATA OF AXIS (%s) -- ", axis);
                switch(maxVal_j) {
                    case 0:
                        // morseMessage += '.';
                        printf("[.]");
                        break;
                    case 1:
                        // morseMessage += '-';
                        printf("[-]");
                        break;
                    case 2:
                        // morseMessage += ' ';
                        printf("[SPACE]");
                        break;
                }
            }
            printf("\n");
            programState = WAITING;
        }
        
        // Exercise 4. Use the usb_serial_print() instead of printf or similar in the previous line.
        //             Check the rest of the code that you do not have printf (substitute them by usb_serial_print())
        //             Use the TinyUSB library to send data through the other serial port (CDC 1).
        //             You can use the functions at https://github.com/hathach/tinyusb/blob/master/src/class/cdc/cdc_device.h
        //             You can find an example at hello_dual_cdc
        //             The data written using this should be provided using csv
        //             timestamp, luminance
        // Tehtävä 4. Käytä usb_serial_print()-funktiota printf:n tai vastaavien sijaan edellisellä rivillä.
        //            Tarkista myös muu koodi ja varmista, ettei siinä ole printf-kutsuja
        //            (korvaa ne usb_serial_print()-funktiolla).
        //            Käytä TinyUSB-kirjastoa datan lähettämiseen toisen sarjaportin (CDC 1) kautta.
        //            Voit käyttää funktioita: https://github.com/hathach/tinyusb/blob/master/src/class/cdc/cdc_device.h
        //            Esimerkki löytyy hello_dual_cdc-projektista.
        //            Tällä menetelmällä kirjoitettu data tulee antaa CSV-muodossa:
        //            timestamp, luminance

        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// static void saveTask (void *arg){
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }


// Exercise 4: Uncomment the following line to activate the TinyUSB library.  
// Tehtävä 4:  Poista seuraavan rivin kommentointi aktivoidaksesi TinyUSB-kirjaston. 

/*
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}*/

int main() {



    // Exercise 4: Comment the statement stdio_init_all(); 
    //             Instead, add AT THE END OF MAIN (before vTaskStartScheduler();) adequate statements to enable the TinyUSB library and the usb-serial-debug.
    //             You can see hello_dual_cdc for help
    //             In CMakeLists.txt add the cfg-dual-usbcdc
    //             In CMakeLists.txt deactivate pico_enable_stdio_usb
    // Tehtävä 4:  Kommentoi lause stdio_init_all();
    //             Sen sijaan lisää MAIN LOPPUUN (ennen vTaskStartScheduler();) tarvittavat komennot aktivoidaksesi TinyUSB-kirjaston ja usb-serial-debugin.
    //             Voit katsoa apua esimerkistä hello_dual_cdc.
    //             Lisää CMakeLists.txt-tiedostoon cfg-dual-usbcdc
    //             Poista CMakeLists.txt-tiedostosta käytöstä pico_enable_stdio_usb

    stdio_init_all();

    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    init_button1();
    init_button2();
    init_red_led();
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, btn_fxn);

    init_ICM42670();
    ICM42670_start_with_default_values();

    
    
    TaskHandle_t hSensorTask, hPrintTask, hUSB = NULL;

    // Exercise 4: Uncomment this xTaskCreate to create the task that enables dual USB communication.
    // Tehtävä 4: Poista tämän xTaskCreate-rivin kommentointi luodaksesi tehtävän,
    // joka mahdollistaa kaksikanavaisen USB-viestinnän.

    /*
    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    */


    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hSensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    result = xTaskCreate(print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hPrintTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

