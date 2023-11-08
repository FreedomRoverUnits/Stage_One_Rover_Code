#ifndef OUR_LED_H
#define OUR_LED_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define BLINK_GPIO 27 // GPIO pin for the LED (change to your desired GPIO pin)

void led_setup(void);
void led_blink(int n_times);
void led_turn_on(void);
void led_turn_off(void);

#endif