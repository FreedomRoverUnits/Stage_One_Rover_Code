#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "pid_ctrl.h"
#include "Encoder.h"
#include "PWM.h"


void app_main(void)
{   //---------------------- Encoder setup (start) -------------------------------//
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
    //---------------------- Encoder setup (end) -------------------------------//

    //---------------------- MCPWM setup (start) -------------------------------//
    //1. mcpwm gpio initialization
    mcpwm_gpio_initialize_both_wheels();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings - Left Wheel
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings - Right Wheel
    //---------------------- MCPWM setup (end) -------------------------------//

    //---------------------- PID setup (start) -------------------------------//
    
    //---------------------- PID setup (end) -------------------------------//


    while(1){
        // if(ten_sec_mark == 10){
        //     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_motor_A));
        //     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_motor_B));
        //     ten_sec_mark = 0;
        // }
        // ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_motor_A, &pulse_count_A));
        // ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_motor_B, &pulse_count_B));
        // ESP_LOGI(TAG, "Pulse count motor A: %d Pulse count motor B: %d", pulse_count_A, pulse_count_B);
        // ten_sec_mark++;
        // vTaskDelay(pdMS_TO_TICKS(1000));

    }
}
