#include "blink.h"

void led_setup() {
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void led_blink(int n_times) {
    for(int i=0; i<n_times; i++)
    {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
     }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
}
