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
#include "PID.h"

void app_main(void)
{
    PID pid1;

    PID_initialize(&pid1, 1.099, 10.0, 1,2,3);

    printf("pid1.min_val_ = %.4f\n", pid1.min_val_);
    printf("pid1.max_val_ = %.4f\n", pid1.max_val_);
    printf("pid1.kp_ = %.4f\n", pid1.kp_);
    printf("pid1.ki_ = %.4f\n", pid1.ki_);
    printf("pid1.kd_ = %.4f\n", pid1.kd_);
    printf("pid1.integral_ = %.4f\n", pid1.integral_);
    printf("pid1.derivative_ = %.4f\n", pid1.derivative_);
    printf("pid1.prev_error_ = %.4f\n", pid1.prev_error_);

    updateConstants(&pid1, 5,6,7);

    printf("pid1.kp_ = %.4f\n", pid1.kp_);
    printf("pid1.ki_ = %.4f\n", pid1.ki_);
    printf("pid1.kd_ = %.4f\n", pid1.kd_);

    compute(&pid1, 1.0, 4.0);

    printf("pid1.min_val_ = %.4f\n", pid1.min_val_);
    printf("pid1.max_val_ = %.4f\n", pid1.max_val_);
    printf("pid1.kp_ = %.4f\n", pid1.kp_);
    printf("pid1.ki_ = %.4f\n", pid1.ki_);
    printf("pid1.kd_ = %.4f\n", pid1.kd_);
    printf("pid1.integral_ = %.4f\n", pid1.integral_);
    printf("pid1.derivative_ = %.4f\n", pid1.derivative_);
    printf("pid1.prev_error_ = %.4f\n", pid1.prev_error_);

    while (1) {
        
        
    }
}