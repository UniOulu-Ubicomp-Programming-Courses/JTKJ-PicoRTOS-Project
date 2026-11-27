#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
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

#define IMU_SAMPLE_PERIOD_MS 10

#define IMU_ACCEL_FILTER_ALPHA 0.4f
#define IMU_ACCEL_THRESHOLD_G 0.2f

#define IMU_GYRO_FILTER_ALPHA 0.4f
#define IMU_GYRO_THRESHOLD_DPS 600.0f

#define IMU_ROTATION_CONFIRM_SAMPLES 5
#define IMU_QUIET_CONFIRM_SAMPLES 50

enum program_state
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

enum program_state program_state = IDLE;

char rx_buffer[MAX_MSG_LENGTH] = "\0";
char tx_buffer[MAX_MSG_LENGTH];
int msg_index = 0;

typedef struct
{
    uint32_t frequency;
    uint32_t length; // millisekuntia
} note_t;

// Super Mario 1-Up -ääni
const note_t OK_melody[] = {
    {659, 133},  // E5
    {784, 133},  // G5
    {1319, 133}, // E6
    {1046, 133}, // C6
    {1175, 133}, // D6
    {1568, 133}  // G6
};

// Suojaa tilamuutujan ja I2C-väylän käyttöä useiden tehtävien välillä.
SemaphoreHandle_t state_mutex;
SemaphoreHandle_t i2c_mutex;

// Aapo Saari
/// @brief Lisää IMU-merkin viestipuskuriin.
/// @param symbol Lisättävä merkki ('.', '-' tai ' ').
/// @return true, jos merkki lisättiin onnistuneesti, false jos puskuri on täynnä tai ohjelma ei ole IDLE-tilassa.
static bool append_imu_symbol(char symbol)
{
    bool appended = false;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (program_state == IDLE)
    {
        if (msg_index < MAX_MSG_LENGTH - 4)
        {
            tx_buffer[msg_index++] = symbol;
            appended = true;
            if (msg_index >= MAX_MSG_LENGTH - 4)
            {
                program_state = SENDING;
            }
        }
        else
        {
            program_state = SENDING;
        }
    }
    xSemaphoreGive(state_mutex);

    return appended;
}

// Konsta Turpeinen
/// @brief Palauttaa tilan merkkijonona.
/// @param s Tila, jonka merkkijono halutaan.
/// @return Merkkijono, joka kuvaa tilaa.
const char *get_state_string(enum program_state s)
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

// Konsta Turpeinen
/// @brief UI-taski, joka hallinnoi käyttöliittymää, näyttöä, led-valoa ja summeria.
/// @param parameters
void ui_task(void *parameters)
{
    int OK_melody_length = sizeof(OK_melody) / sizeof(OK_melody[0]);

    enum program_state current_state = IDLE;
    enum program_state previous_state = IDLE;

    // Alustetaan led ja summeri
    init_buzzer();
    init_led();

    // Alustetaan näyttö
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    init_display();

    clear_display();
    write_text_xy(0, 0, get_state_string(current_state));
    write_text_xy(0, 14, rx_buffer);
    xSemaphoreGive(i2c_mutex);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(50));

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        current_state = program_state;
        xSemaphoreGive(state_mutex);

        if (current_state != previous_state)
        {
            // Päivitetään näyttö
            xSemaphoreTake(i2c_mutex, portMAX_DELAY);
            clear_display();
            write_text_xy(0, 0, get_state_string(current_state));
            write_text_xy(0, 14, rx_buffer);
            xSemaphoreGive(i2c_mutex);

            if (current_state == PLAY_BACK)
            {
                // Toistetaan vastaanotettu viesti ledillä ja summerilla.
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                for (int i = 0; i < sizeof(rx_buffer) / sizeof(rx_buffer[0]); i++)
                {
                    char c = rx_buffer[i];

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
                program_state = IDLE;
                xSemaphoreGive(state_mutex);
            }

            else if (current_state == SUCCESS)
            {
                // Käydään läpi kuittausmelodian kaikki nuotit
                for (int i = 0; i < OK_melody_length; i++)
                {
                    buzzer_play_tone(OK_melody[i].frequency, OK_melody[i].length - 20);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }

                // Muutetaan tila IDLE:ksi
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                program_state = IDLE;
                current_state = IDLE;
                xSemaphoreGive(state_mutex);

                // Päivitetään näyttö
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                clear_display();
                write_text_xy(0, 0, get_state_string(current_state));
                write_text_xy(0, 16, rx_buffer);
                xSemaphoreGive(i2c_mutex);
            }
            previous_state = current_state;
        }
    }
}

// Aapo Saari
/// @brief Alustaa IMU:n ja käsittelee sensorinäytteitä toistuvassa tunnistussilmukassa.
/// @param parameters FreeRTOS-tehtävän käynnistysparametri, jota tämä tehtävä ei käytä.
static void sensor_task(void *parameters)
{
    (void)parameters;

    // Alustetaan sensori I2C-lukon suojassa ja lopetetaan tehtävä, jos käynnistys epäonnistuu.
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    if (init_ICM42670() != 0)
    {
        usb_serial_print("ICM42670-sensorin alustus epäonnistui\n");
        usb_serial_flush();
        xSemaphoreGive(i2c_mutex);
        vTaskDelete(NULL);
        return;
    }

    int start_result = ICM42670_enable_accel_gyro_ln_mode();
    if (start_result == 0)
    {
        start_result = ICM42670_startAccel(
            ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
    }
    if (start_result == 0)
    {
        start_result = ICM42670_startGyro(
            ICM42670_GYRO_ODR_DEFAULT, 2000);
    }
    if (start_result != 0)
    {
        usb_serial_print("ICM42670-sensorin käynnistys epäonnistui\n");
        usb_serial_flush();
        xSemaphoreGive(i2c_mutex);
        vTaskDelete(NULL);
        return;
    }
    xSemaphoreGive(i2c_mutex);

    TickType_t last_wake = xTaskGetTickCount();

    // Säilytetään suodattimien vertailuarvot ja tunnistuksen tila näytteiden välillä.
    float previous_accel_x = 0.0f;
    float previous_accel_z = 0.0f;
    float previous_gyro_y = 0.0f;

    bool filter_initialized = false;
    bool is_moving = false;

    char detected_symbol = '\0';

    uint8_t debounce_counter = 0;
    uint8_t rotation_counter = 0;

    // Käsitellään näytteet tasaisin aikavälein koko tehtävän elinkaaren ajan.
    while (1)
    {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        enum program_state current_state = program_state;
        xSemaphoreGive(state_mutex);

        // Keskeytetään eleen tunnistus sovelluksen muiden tilojen ajaksi.
        if (current_state != IDLE)
        {
            is_moving = false;
            detected_symbol = '\0';
            debounce_counter = 0;
            rotation_counter = 0;
            filter_initialized = false;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
            continue;
        }

        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float temperature;

        // Luetaan sensorin kaikki tulosarvot I2C-lukon aikana.
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        int read_result = ICM42670_read_sensor_data(
            &accel_x, &accel_y, &accel_z,
            &gyro_x, &gyro_y, &gyro_z, &temperature);
        xSemaphoreGive(i2c_mutex);

        if (read_result != 0)
        {
            // Lukuhäiriön jälkeen hylätään keskeneräinen ele ja alustetaan suodatus uudelleen.
            usb_serial_print("ICM42670-sensorin lukeminen epäonnistui\n");
            usb_serial_flush();
            is_moving = false;
            detected_symbol = '\0';
            debounce_counter = 0;
            rotation_counter = 0;
            filter_initialized = false;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
            continue;
        }

        if (!filter_initialized)
        {
            // Ensimmäinen näyte asettaa suodattimien lähtötason, eikä sitä tulkita liikkeeksi.
            previous_accel_x = accel_x;
            previous_accel_z = accel_z;
            previous_gyro_y = gyro_y;
            filter_initialized = true;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
            continue;
        }

        // Tasataan mittauskohinaa ja lasketaan kiihtyvyyden muutos edelliseen suodatettuun näytteeseen.
        const float filtered_accel_x =
            IMU_ACCEL_FILTER_ALPHA * accel_x +
            (1.0f - IMU_ACCEL_FILTER_ALPHA) * previous_accel_x;

        const float filtered_accel_z =
            IMU_ACCEL_FILTER_ALPHA * accel_z +
            (1.0f - IMU_ACCEL_FILTER_ALPHA) * previous_accel_z;

        const float filtered_gyro_y =
            IMU_GYRO_FILTER_ALPHA * gyro_y +
            (1.0f - IMU_GYRO_FILTER_ALPHA) * previous_gyro_y;

        const float delta_accel_x = fabsf(filtered_accel_x - previous_accel_x);
        const float delta_accel_z = fabsf(filtered_accel_z - previous_accel_z);
        const float rotation_speed = fabsf(filtered_gyro_y);

        // Uuden eleen alussa gyro tunnistaa pyörähdyksen ja kiihtyvyys akseliliikkeen.
        if (!is_moving)
        {
            // Pyörähdys tarkistetaan ensin, jotta välilyönti ei sekoitu akseliliikkeeseen.
            if (rotation_speed > IMU_GYRO_THRESHOLD_DPS)
            {
                rotation_counter++;
                if (rotation_counter >= IMU_ROTATION_CONFIRM_SAMPLES)
                {
                    detected_symbol = ' ';
                    is_moving = true;
                    rotation_counter = 0;
                    debounce_counter = 0;
                }
            }
            else if (delta_accel_z > IMU_ACCEL_THRESHOLD_G)
            {
                detected_symbol = '.';
                is_moving = true;
                rotation_counter = 0;
                debounce_counter = 0;
            }
            else if (delta_accel_x > IMU_ACCEL_THRESHOLD_G)
            {
                detected_symbol = '-';
                is_moving = true;
                rotation_counter = 0;
                debounce_counter = 0;
            }
            else
            {
                rotation_counter = 0;
            }
        }
        else if (delta_accel_x < IMU_ACCEL_THRESHOLD_G &&
                 delta_accel_z < IMU_ACCEL_THRESHOLD_G &&
                 rotation_speed < IMU_GYRO_THRESHOLD_DPS)
        {
            debounce_counter++;
            if (debounce_counter >= IMU_QUIET_CONFIRM_SAMPLES)
            {
                // Lisätään merkki puskuriin vasta riittävän pitkän rauhallisen jakson jälkeen.
                if (append_imu_symbol(detected_symbol))
                {
                    if (detected_symbol == ' ')
                    {
                        buzzer_play_tone(1282, 300);
                    }
                    else if (detected_symbol == '.')
                    {
                        buzzer_play_tone(641, 100);
                    }
                    else if (detected_symbol == '-')
                    {
                        buzzer_play_tone(641, 300);
                    }
                }
                is_moving = false;
                detected_symbol = '\0';
                debounce_counter = 0;
            }
        }
        else
        {
            debounce_counter = 0;
        }

        // Käytetään tämän kierroksen suodatettuja arvoja seuraavan näytteen vertailutasona.
        previous_accel_x = filtered_accel_x;
        previous_accel_z = filtered_accel_z;
        previous_gyro_y = filtered_gyro_y;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}

// Meeko Röning
/// @brief Painikkeen keskeytyskäsittelijä.
/// @param gpio
/// @param events
void button_callback(uint gpio, uint32_t events)
{
    program_state = SENDING;
}

// Meeko Röning
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

// Meeko Röning
/// @brief Lähettää viestipuskurin sisällön.
/// @param arg
void transmit_task(void *arg)
{
    (void)arg;

    while (1)
    {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (program_state == SENDING && msg_index != 0)
        {
            tx_buffer[msg_index++] = ' ';
            tx_buffer[msg_index++] = ' ';
            tx_buffer[msg_index++] = '\n'; // Lisää välit ja rivinvaihto viestin loppuun
            tud_cdc_n_write(CDC_ITF_DATA, tx_buffer, sizeof(char) * msg_index);
            tud_cdc_n_write_flush(CDC_ITF_DATA);
            // Resetoi viestin indeksi seuraavaa viestiä varten
            msg_index = 0;
            program_state = SUCCESS;
        }
        else if (program_state == SENDING && msg_index == 0)
        {
            program_state = IDLE;
        }
        xSemaphoreGive(state_mutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Meeko Röning
/// @brief USB-vastaanottokäsittelijä.
/// @param itf
void tud_cdc_rx_cb(uint8_t itf)
{
    char temp_buffer[4];
    static uint8_t rx_index = 0;
    // Vastaan otetaan itf:stä riippumatta, jotta bufferit tyhjenee.
    uint32_t count = tud_cdc_n_read(itf, temp_buffer, sizeof(temp_buffer));

    // Käsitellään vain CDC_ITF_DATA-liitännän kautta tuleva data.
    if (itf == CDC_ITF_DATA)
    {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (rx_index < MAX_MSG_LENGTH - 1 && count != 3) // vain lopetusviesti 3 pitkä
        {
            rx_buffer[rx_index++] = temp_buffer[0];
        }
        else
        {
            rx_buffer[rx_index] = '\0'; // Lopetetaan merkkijono
            rx_index = 0;               // Resetoi indeksi seuraavaa viestiä varten

            // Vahidetaan tilaksi PLAY_BACK, jotta ui_task voi käsitellä vastaanotetun datan.
            program_state = PLAY_BACK;
        }
        xSemaphoreGive(state_mutex);
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

    // Mutex estää samanaikaiset ristiriitaiset program_state ja I2C -muutokset.
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL)
    {
        usb_serial_print((char const *)"Tilalukon luonti epäonnistui\n\0");
        usb_serial_flush();
        return 0;
    }

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL)
    {
        usb_serial_print((char const *)"I2C-lukon luonti epäonnistui\n\0");
        usb_serial_flush();
        return 0;
    }

    // Luodaan RTOS tehtävät ja tarkistetaan että ne onnistui
    BaseType_t ui_task_result = xTaskCreate(
        ui_task,
        "ui",
        DEFAULT_STACK_SIZE,
        NULL,
        1,
        NULL);
    if (ui_task_result != pdPASS)
    {
        usb_serial_print((char const *)"Käyttöliittymätehtävän luonti epäonnistui\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t sensor_task_result = xTaskCreate(
        sensor_task,
        "sensor",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        NULL);
    if (sensor_task_result != pdPASS)
    {
        usb_serial_print((char const *)"Sensoritehtävän luonti epäonnistui\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t usb_task_result = xTaskCreate(
        usb_task,
        "usb",
        DEFAULT_STACK_SIZE,
        NULL,
        3, // Suurempi prioriteetti kuin muilla tehtävillä
        NULL);
    if (usb_task_result != pdPASS)
    {
        usb_serial_print((char const *)"USB-tehtävän luonti epäonnistui\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t transmit_task_result = xTaskCreate(
        transmit_task,
        "transmit",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        NULL);
    if (transmit_task_result != pdPASS)
    {
        usb_serial_print((char const *)"Lähetystehtävän luonti epäonnistui\n\0");
        usb_serial_flush();
        return 0;
    }

    // Käynnistyksen jälkeen FreeRTOS Scheduler hallitsee tehtävien ajoitusta.
    vTaskStartScheduler();

    // Schedulerin ei normaalisti pitäisi palata tähän.
    return 0;
}
