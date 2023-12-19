// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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