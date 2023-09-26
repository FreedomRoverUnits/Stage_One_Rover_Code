#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "Encoder.h"

void app_main(void)
{
    // Declare the PCNT unit
    pcnt_unit_handle_t pcnt_unit_motor_A = NULL;
    pcnt_unit_handle_t pcnt_unit_motor_B = NULL;

    // Initialize PCNT units for motors
    create_pcnt_for_motors(&pcnt_unit_motor_A, &pcnt_unit_motor_B);

    // Declare channels/gpios to edit counters
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_channel_handle_t pcnt_chan_b = NULL;

    // Initialize channels
    create_pcnt_channels_for_motors(&pcnt_unit_motor_A, &pcnt_unit_motor_B, &pcnt_chan_a, &pcnt_chan_b);

    // Start PCNT units
    start_pcnt_for_motors(&pcnt_unit_motor_A, &pcnt_unit_motor_B);

    // Here is where the counts printed to monitor
    int pulse_count_A = 0;
    int pulse_count_B = 0;
    int ten_sec_mark = 0;
    while(1){/*
        if(ten_sec_mark == 10){
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_motor_A));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_motor_B));
            ten_sec_mark = 0;
        }
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_motor_A, &pulse_count_A));
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_motor_B, &pulse_count_B));
        ESP_LOGI(TAG, "Pulse count motor A: %d Pulse count motor B: %d", pulse_count_A, pulse_count_B);
        ten_sec_mark++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        */
       ESP_LOGI(TAG, "RPM: %f", getRPM(&pcnt_unit_motor_A));
       vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
