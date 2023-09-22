#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "PWM.h"

void app_main(void)
{
    setup_rover_wheels();

    while (1) {
        //This Test that the rover will move forward and backward
        rover_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90, MCPWM_TIMER_1,90);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        rover_stop(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_TIMER_1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        rover_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 100, MCPWM_TIMER_1,100);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        rover_stop(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_TIMER_1);
        vTaskDelay(500 / portTICK_PERIOD_MS);      
        
    }
}