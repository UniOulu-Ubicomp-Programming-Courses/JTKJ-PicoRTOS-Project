#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <tusb.h>

#include "usbSerialDebug/helper.h"
#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_DATA 1

#define MAX_MSG_LENGTH 64
#define DEBUG_MSG_LENGTH 64

enum state
{
    // IDLE-tilassa sensoria luetaan ja etsitään uutta liikettä.
    IDLE = 1,
    // SENDING on varattu tunnistetun merkin lähettämistä varten.
    SENDING = 2,
    // PLAY_BACK on varattu tallennetun datan toistamiseen.
    PLAY_BACK = 3,
    // SUCCESS kertoo, kun viesti on lähetetty
    SUCCESS = 4
};

enum state programState = IDLE;

char debugBuffer[DEBUG_MSG_LENGTH]; // vain tarpeellinen jos halutaan formatointia usb_serial_printiin

char rxBuffer[MAX_MSG_LENGTH] = "\0";
char txBuffer[MAX_MSG_LENGTH];
int msgIndex = 0;

typedef struct
{
    uint32_t frequency;
    uint32_t length; // millisekuntia
} Note;

// Super Mario 1-Up -ääni
const Note kuittausmelodia[] = {
    {659, 133},  // E5
    {784, 133},  // G5
    {1319, 133}, // E6
    {1046, 133}, // C6
    {1175, 133}, // D6
    {1568, 133}  // G6
};

// Suojaa tilamuutujan ja I2C-väylän käyttöä useiden tehtävien välillä.
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t i2cMutex;

const char *get_state_string(enum state s)
{
    switch (s)
    {
    case IDLE:
        return "IDLE";
    case SENDING:
        return "SENDING";
    case PLAY_BACK:
        return "PLAYBACK";
    case SUCCESS:
        return "SUCCESS";
    }
    return "UNKNOWN";
}

/// @brief UI-taski, joka hallinnoi käyttöliittymää, näyttöä, led-valoa ja summeria.
/// @param pvParameters
void ui_task(void *pvParameters)
{
    int len_kuittausmelodia = sizeof(kuittausmelodia) / sizeof(kuittausmelodia[0]);

    enum state current_state = IDLE;
    enum state previous_state = IDLE;

    // Alustetaan led ja summeri
    init_buzzer();
    init_led();

    // Alustetaan näyttö
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    init_display();

    clear_display();
    write_text_xy(0, 0, get_state_string(current_state));
    write_text_xy(0, 14, rxBuffer);
    xSemaphoreGive(i2cMutex);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(50));

        xSemaphoreTake(stateMutex, portMAX_DELAY);
        current_state = programState;
        xSemaphoreGive(stateMutex);

        if (current_state != previous_state)
        {
            // Päivitetään näyttö
            xSemaphoreTake(i2cMutex, portMAX_DELAY);
            clear_display();
            write_text_xy(0, 0, get_state_string(current_state));
            write_text_xy(0, 14, rxBuffer);
            xSemaphoreGive(i2cMutex);

            if (current_state == PLAY_BACK)
            {
                // Toistetaan vastaanotettu viesti ledillä ja summerilla.
                for (int i = 0; i < sizeof(rxBuffer) / sizeof(rxBuffer[0]); i++)
                {
                    char c = rxBuffer[i];

                    // Lopetetaan toisto, jos tulee tyhjä merkki vastaan
                    if (c == '\0')
                    {
                        break;
                    }

                    // Piste
                    if (c == '.')
                    {
                        set_led_status(true);
                        buzzer_play_tone(641, 100);
                        set_led_status(false);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    // Viiva
                    else if (c == '-')
                    {
                        set_led_status(true);
                        buzzer_play_tone(641, 300);
                        set_led_status(false);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    // Välilyönti
                    else if (c == ' ')
                    {
                        vTaskDelay(pdMS_TO_TICKS(300));
                    }
                }

                // Palautetaan tila IDLE:ksi
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                programState = IDLE;
                xSemaphoreGive(stateMutex);
            }

            else if (current_state == SUCCESS)
            {
                // Käydään läpi kuittausmelodian kaikki nuotit
                for (int i = 0; i < len_kuittausmelodia; i++)
                {
                    buzzer_play_tone(kuittausmelodia[i].frequency, kuittausmelodia[i].length - 20);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }

                // Muutetaan tila IDLE:ksi
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                programState = IDLE;
                current_state = IDLE;
                xSemaphoreGive(stateMutex);

                // Päivitetään näyttö
                xSemaphoreTake(i2cMutex, portMAX_DELAY);
                clear_display();
                write_text_xy(0, 0, get_state_string(current_state));
                write_text_xy(0, 16, rxBuffer);
                xSemaphoreGive(i2cMutex);
            }
            previous_state = current_state;
        }
    }
}

/// @brief Sensoritaski, joka hakee kiihtyvyys- ja gyroskooppiarvot ja tarkistaa liikkeen.
/// @param arg
static void sensor_task(void *arg)
{
    (void)arg;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    enum state current_state = programState;
    xSemaphoreGive(stateMutex);

    // Edellisiä ja suodatettuja kiihtyvyysarvoja käytetään liikkeen havaitsemiseen.
    float previous_ax, previous_ay;
    float filtered_ax, filtered_ay;
    float previous_gy, filtered_gy;

    const float accelerometer_threshold = 0.2f;
    const float accelerometer_alpha = 0.4f;
    const float gyroscope_threshold = 100.0f;
    const float gyroscope_alpha = 0.4f;

    bool is_moving = false;

    // Y-akselin liike tuottaa pisteen, X-akselin liike viivan
    // ja riittävän voimakas Y-akselin pyöräytys välilyönnin.
    char detection_symbol;

    // Liikkeen on oltava rauhallinen usean näytteen ajan ennen merkin hyväksymistä.
    uint8_t debounce_counter = 0;
    uint8_t rotation_counter = 0;

    // Alustetaan IMU ja otetaan kiihtyvyysanturi sekä gyroskooppi käyttöön.
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (init_ICM42670() != 0)
    {
        usb_serial_print((char const *)"Failed to initialize ICM42670\n\0");
        usb_serial_flush();
        xSemaphoreGive(i2cMutex);
        vTaskDelete(NULL);
        return;
    }

    if (ICM42670_start_with_default_values() != 0)
    {
        usb_serial_print((char const *)"Failed to start ICM42670\n\0");
        usb_serial_flush();
        xSemaphoreGive(i2cMutex);
        vTaskDelete(NULL);
        return;
    }

    // Ensimmäinen näyte alustaa vertailu- ja suodatusarvot.
    if (ICM42670_read_sensor_data(&previous_ax, &previous_ay, NULL,
                                  NULL, &previous_gy, NULL, NULL) != 0)
    {
        usb_serial_print((char const *)"Failed to read initial ICM42670 data\n\0");
        usb_serial_flush();
        xSemaphoreGive(i2cMutex);
        vTaskDelete(NULL);
        return;
    }
    xSemaphoreGive(i2cMutex);

    // Säilyttää tasaisen 50 ms näytteenottovälin.
    TickType_t last_wake = xTaskGetTickCount();

    filtered_ax = previous_ax;
    filtered_ay = previous_ay;
    filtered_gy = previous_gy;

    while (1)
    {
        // Suojataan tilan luku ja muutos muilta tehtäviltä.
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        current_state = programState;
        xSemaphoreGive(stateMutex);

        if (current_state == IDLE)
        {
            float current_ax, current_ay;
            float current_gy;

            xSemaphoreTake(i2cMutex, portMAX_DELAY);
            if (ICM42670_read_sensor_data(&current_ax, &current_ay, NULL, NULL, &current_gy, NULL, NULL) != 0)
            {
                usb_serial_print((char const *)"Failed to read ICM42670 data\n\0");
                usb_serial_flush();
                xSemaphoreGive(i2cMutex);
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
                continue;
            }
            xSemaphoreGive(i2cMutex);

            // Suodatus vähentää mittauskohinaa ennen liikkeen tunnistamista.
            filtered_gy = gyroscope_alpha * current_gy + (1 - gyroscope_alpha) * previous_gy;
            float rotation_speed = fabs(filtered_gy);

            filtered_ax = accelerometer_alpha * current_ax + (1 - accelerometer_alpha) * previous_ax;
            filtered_ay = accelerometer_alpha * current_ay + (1 - accelerometer_alpha) * previous_ay;
            float delta_ax = fabs(filtered_ax - previous_ax);
            float delta_ay = fabs(filtered_ay - previous_ay);

            if (!is_moving)
            {
                // Tunnistetaan pyöräytys ennen pisteen tai viivan tunnistamista.
                if (rotation_speed > gyroscope_threshold)
                {
                    rotation_counter++;

                    if (rotation_counter >= 3)
                    {
                        detection_symbol = ' ';
                        is_moving = true;

                        snprintf(debugBuffer, DEBUG_MSG_LENGTH, "Movement detected: '%c'\n", detection_symbol);
                        usb_serial_print(debugBuffer);
                        usb_serial_flush();

                        rotation_counter = 0;
                    }
                }
                else if (delta_ay > accelerometer_threshold && delta_ay > delta_ax)
                {
                    detection_symbol = '.';
                    is_moving = true;

                    snprintf(debugBuffer, DEBUG_MSG_LENGTH, "Movement detected: '%c'\n", detection_symbol);
                    usb_serial_print(debugBuffer);
                    usb_serial_flush();

                    rotation_counter = 0;
                    debounce_counter = 0;
                }
                else if (delta_ax > accelerometer_threshold)
                {
                    detection_symbol = '-';
                    is_moving = true;

                    snprintf(debugBuffer, DEBUG_MSG_LENGTH, "Movement detected: '%c'\n", detection_symbol);
                    usb_serial_print(debugBuffer);
                    usb_serial_flush();

                    rotation_counter = 0;
                    debounce_counter = 0;
                }
                else
                {
                    rotation_counter = 0;
                }
            }
            else if (is_moving)
            {
                // Lähetetään merkki vasta, kun laite on ollut rauhallinen 250 ms ajan.
                if (delta_ax < accelerometer_threshold && delta_ay < accelerometer_threshold && rotation_speed < gyroscope_threshold)
                {
                    debounce_counter++;
                    if (debounce_counter >= 25)
                    {
                        // Lähetetään merkki, kun laite on ollut rauhallinen tarpeeksi kauan.
                        xSemaphoreTake(stateMutex, portMAX_DELAY);
                        if (msgIndex < MAX_MSG_LENGTH - 4)
                        {
                            txBuffer[msgIndex++] = detection_symbol;
                            is_moving = false;

                            if (msgIndex >= MAX_MSG_LENGTH - 4)
                            {
                                programState = SENDING; // Vaihdetaan tilaksi SENDING, jotta viesti lähetetään
                                xSemaphoreGive(stateMutex);
                                usb_serial_print((char const *)"Transmit buffer full\n\0");
                                usb_serial_flush();
                            }
                            else
                            {
                                xSemaphoreGive(stateMutex);
                            }
                        }
                        else
                        {
                            xSemaphoreGive(stateMutex);
                        }

                        rotation_counter = 0;
                        debounce_counter = 0;
                    }
                }
                else
                {
                    rotation_counter = 0;
                    debounce_counter = 0;
                }
            }
            // Käytetään suodatettua näytettä seuraavan kierroksen vertailuarvona.
            previous_ax = filtered_ax;
            previous_ay = filtered_ay;
            previous_gy = filtered_gy;
        }
        else
        {
            is_moving = false;
            rotation_counter = 0;
            debounce_counter = 0;
        }
        // Odotetaan seuraavaan näytteenottohetkeen.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

/// @brief Painikkeen keskeytyskäsittelijä.
/// @param gpio
/// @param events
void button_callback(uint gpio, uint32_t events)
{
    programState = SENDING;
}

/// @brief USB-taski, joka hoitaa USB-pinon.
/// @param arg
void usb_task(void *arg)
{
    (void)arg;
    while (1)
    {
        tud_task();
        // Ei vTaskDelay
    }
}

/// @brief Lähettää viesti bufferin sisällön.
/// @param arg
void transmit_task(void *arg)
{
    (void)arg;

    while (1)
    {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        if (programState == SENDING && msgIndex != 0)
        {
            txBuffer[msgIndex++] = ' ';
            txBuffer[msgIndex++] = ' ';
            txBuffer[msgIndex++] = '\n'; // Lisää välit ja rivinvaihto viestin loppuun
            tud_cdc_n_write(CDC_ITF_DATA, txBuffer, sizeof(char) * msgIndex);
            tud_cdc_n_write_flush(CDC_ITF_DATA);
            // Resetoi viestin indeksi seuraavaa viestiä varten
            msgIndex = 0;
            programState = SUCCESS;
        }
        else if (programState == SENDING && msgIndex == 0)
        {
            programState = IDLE;
        }
        xSemaphoreGive(stateMutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/// @brief USB-vastaanottokäsittelijä.
/// @param itf
void tud_cdc_rx_cb(uint8_t itf)
{
    char tempBuffer[4];
    static uint8_t rxIndex = 0;
    // Vastaan otetaan itf:stä riippumatta, jotta bufferit tyhjenee.
    uint32_t count = tud_cdc_n_read(itf, tempBuffer, sizeof(tempBuffer));
    
    // Käsitellään vain CDC_ITF_DATA-liitännän kautta tuleva data.
    if (itf == CDC_ITF_DATA)
    {
        if (rxIndex < MAX_MSG_LENGTH - 1 && count != 3) //vain lopetus viesti 3 pitkä
        {
            rxBuffer[rxIndex++] = tempBuffer[0];
        }
        else 
        {
            rxBuffer[rxIndex] = '\0'; // Lopetetaan merkkijono
            rxIndex = 0; // Resetoi indeksi seuraavaa viestiä varten
            // Vahidetaan tilaksi PLAY_BACK, jotta ui_task voi käsitellä vastaanotetun datan.
            programState = PLAY_BACK;
        }
    }
}

int main()
{
    // Alustaa HAT-laitteen ja sen oletusarvoisen I2C-väylän.
    init_hat_sdk();
    sleep_ms(300);

    // Alustaa painikkeen ja määrittää sen keskeytyskäsittelijän.
    gpio_init(BUTTON1);
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, button_callback);

    // Alustetaan USB yhteys ennen debug viestejä
    tusb_init();
    usb_serial_init();

    // Mutex estää samanaikaiset ristiriitaiset programState ja I2C -muutokset.
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL)
    {
        usb_serial_print((char const *)"Failed to create state mutex\n\0");
        usb_serial_flush();
        return 0;
    }

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL)
    {
        usb_serial_print((char const *)"Failed to create I2C mutex\n\0");
        usb_serial_flush();
        return 0;
    }

    // Luodaan RTOS tehtävät ja tarkistetaan että ne onnistui
    BaseType_t uiTaskResult = xTaskCreate(
        ui_task,
        "ui",
        DEFAULT_STACK_SIZE,
        NULL,
        1,
        NULL);
    if (uiTaskResult != pdPASS)
    {
        usb_serial_print((char const *)"UI task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t sensorTaskResult = xTaskCreate(
        sensor_task,
        "sensor",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        NULL);
    if (sensorTaskResult != pdPASS)
    {
        usb_serial_print((char const *)"Sensor task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t usbTaskResult = xTaskCreate(
        usb_task,
        "usb",
        DEFAULT_STACK_SIZE,
        NULL,
        3, // Suurempi prioriteetti kuin muilla tehtävillä
        NULL);
    if (usbTaskResult != pdPASS)
    {
        usb_serial_print((char const *)"USB task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t transmitTaskResult = xTaskCreate(
        transmit_task,
        "transmit",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        NULL);
    if (transmitTaskResult != pdPASS)
    {
        usb_serial_print((char const *)"Transmit task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    // Käynnistyksen jälkeen FreeRTOS Scheduler hallitsee tehtävien ajoitusta.
    vTaskStartScheduler();

    // Schedulerin ei normaalisti pitäisi palata tähän.
    return 0;
}
