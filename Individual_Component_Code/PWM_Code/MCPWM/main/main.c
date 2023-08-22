#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "PWM.h"

void app_main(void)
{
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
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
    
    while (1) {

        // brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        //TODO
        
    }
}