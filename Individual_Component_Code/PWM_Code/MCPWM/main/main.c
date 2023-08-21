// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/mcpwm_prelude.h"
// #include "esp_log.h"
// #include "sdkconfig.h"
// #include "bdc_motor.h"
// #include "esp_timer.h"

// static const char *TAG = "example";

// #define PWM_GPIO_A 22
// #define PWM_GPIO_B 23
// #define PWM_RESOLUTION_HZ 1 //1Hz, 1 sec per tick// 1000 // 1KHz, 1ms per tick
// #define PWM_PERIOD 30 // 30 ticks, 1/2 minute   //1000 //1000 ticks, 1 second
// #define PWM_HEAVY_DUTY 10 // 10 seconds of high

// static uint8_t gpio_level_A = 0;
// static uint8_t gpio_level_B = 0;

// static void configure_led(void)
// {
//     ESP_LOGI(TAG, "ESP32 configured to rotate motors! just set gpio 34 and 35");
//     gpio_reset_pin(PWM_GPIO_A);
//     gpio_reset_pin(PWM_GPIO_B);
//     /* Set the GPIO as a push/pull output */
//     gpio_set_direction(PWM_GPIO_A, GPIO_MODE_OUTPUT);
//     gpio_set_direction(PWM_GPIO_B, GPIO_MODE_OUTPUT);
// }

// static void configure_PWM(void)
// {
//     ESP_LOGI(TAG, "Create timer and operator");
//     mcpwm_timer_handle_t timer = NULL;
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = PWM_RESOLUTION_HZ,
//         .period_ticks = PWM_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

//     mcpwm_oper_handle_t oper = NULL;
//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // operator must be in the same group to the timer
//     };
//     ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

//     ESP_LOGI(TAG, "Connect timer and operator");
//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

//     ESP_LOGI(TAG, "Create comparator and generator from the operator");
//     mcpwm_cmpr_handle_t comparator = NULL;
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

//     mcpwm_gen_handle_t generator = NULL;
//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = PWM_GPIO_A,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

//     // set the initial compare value, so that the servo will spin to the center position
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, PWM_HEAVY_DUTY));

//     ESP_LOGI(TAG, "Set generator action on timer and compare event");
//     // go high on counter empty
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
//                     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     // go low on compare threshold
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
//                     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

//     ESP_LOGI(TAG, "Enable and start timer");
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
// }
// static void pwm_motor(void)
// {
//     ESP_LOGI(TAG, "Create DC motor");
//     bdc_motor_config_t motor_config = {
//         .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
//         .pwma_gpio_num = PWM_GPIO_A,
//         .pwmb_gpio_num = PWM_GPIO_B,
//     };
//     bdc_motor_mcpwm_config_t mcpwm_config = {
//         .group_id = 0,
//         .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
//     };
//     //bdc_motor_handle_t motor = NULL;
//     //ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
//     //motor_ctrl_ctx.motor = motor;


// }


// static void counter_clockwise(void)
// {
//     gpio_set_level(PWM_GPIO_A,0);
//     gpio_set_level(PWM_GPIO_B,1);
// }

// static void clockwise(void)
// {
//     gpio_set_level(PWM_GPIO_A,1);
//     gpio_set_level(PWM_GPIO_B,0);
// }

// void app_main(void)
// {
//     /* Configure the peripheral according to the LED type */
//     //configure_led();
//     configure_PWM();
//     int32_t heavy_duty = 0;
//     int32_t counter = 0;
//     while (1) {
//         /*clockwise();
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         counter_clockwise();
//         vTaskDelay(1000 / portTICK_PERIOD_MS);*/
//         // if(counter >= 10000)
//         // {
//         //     if(heavy_duty >= 1000)
//         //     {
//         //         heavy_duty = 0;
//         //     }
//         //     else
//         //     {
//         //         heavy_duty += 100;
//         //     }
//         //     counter = 0;
//         // }
//         // counter +=1;


//     }
// }
/* brushed dc motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}