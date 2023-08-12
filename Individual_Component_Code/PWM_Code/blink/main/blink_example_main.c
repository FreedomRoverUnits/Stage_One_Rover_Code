/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2
#define PWM_GPIO_A 22
#define PWM_GPIO_B 23

static uint8_t gpio_level_A = 0;
static uint8_t gpio_level_B = 0;

static uint8_t s_led_state = 0;

static void configure_led(void)
{
    ESP_LOGI(TAG, "ESP32 configured to rotate motors! just set gpio 34 and 35");
    gpio_reset_pin(PWM_GPIO_A);
    gpio_reset_pin(PWM_GPIO_B);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PWM_GPIO_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PWM_GPIO_B, GPIO_MODE_OUTPUT);
}

static void counter_clockwise(void)
{
    gpio_set_level(PWM_GPIO_A,0);
    gpio_set_level(PWM_GPIO_B,1);
}

static void clockwise(void)
{
    gpio_set_level(PWM_GPIO_A,1);
    gpio_set_level(PWM_GPIO_B,0);
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();

    while (1) {
        /*ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        * Toggle the LED state *
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);*/
        
        clockwise();
        vTaskDelay(500 / portTICK_PERIOD_MS);
        counter_clockwise();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
