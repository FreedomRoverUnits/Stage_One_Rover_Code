#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

static const char *TAG = "Testing Motor Control";

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

#define MOTOR_A_ENCODER 0
#define MOTOR_B_ENCODER 2

void app_main(void)
{
    // Here we will create the counting unit, this will
    // be used to keep track of the rototary enocoder counts
    ESP_LOGI(TAG, "Create pcnt unit for Motor A");
    
    // This creates the setting for the unit counter
    // Essentially setting a limit to the range the counter can be
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    
    // Declare the PCNT unit
    pcnt_unit_handle_t pcnt_unit_motor_A = NULL;
   
    // Create PCNT unit with unit_config
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_motor_A));

    // Repeating the same step for Motor B
    ESP_LOGI(TAG, "Create pcnt unit for Motor B");
    pcnt_unit_handle_t pcnt_unit_motor_B = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_motor_B));

    // Next the glitch filter is going to be placed
    // this just makes sure we only read stable signals
    // similar to push button debouncing
    ESP_LOGI(TAG, "Setting Glitch Filters");
        pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_motor_A, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_motor_B, &filter_config));

    // Next is setting up the actual gpio channel that will increment
    // or decrement the counter
    ESP_LOGI(TAG, "Creating pcnt channels");

    // Here is the configuration for the channel
    // We are only using the rising edge from the encoder
    // to count the pulses for determing the RPM, which is
    // why we set level to -1
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = MOTOR_A_ENCODER,
        .level_gpio_num = -1,
    };

    // Create the channel/gpio and tie it to unit counter
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_motor_A, &chan_a_config, &pcnt_chan_a));

    // Repeat for motor B encoder
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = MOTOR_B_ENCODER,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_motor_B, &chan_b_config, &pcnt_chan_b));

    // This section focuses on setting the logic/reaction 
    // to when the gpio/channel recieves a rising edge
    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");

    // On the rising edge (the second arg to pcnt_channel_set_edge_action)
    // the counter will increment while on the falling edge (third arg)
    // the counter will remain the same
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    // Even though we are not using level, the setting will be set to
    // keep the count going in the same direction, counting up to 1000,
    // just to be safe
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // Finally here is where the counters enabled, cleared to start at
    // zero, and started
    
    // Enable pcnt units for motor A and motor B
    ESP_LOGI(TAG, "enable pcnt units");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_motor_A));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_motor_B));

    // Clear the counter incase there is data left over
    ESP_LOGI(TAG, "clear pcnt units");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_motor_A));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_motor_B));

    // Start counters
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_motor_A));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_motor_B));

    // Here is where the counts printed to monitor
    int pulse_count_A = 0;
    int pulse_count_B = 0;
    int ten_sec_mark = 0;
    while(1){
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

    }
}
