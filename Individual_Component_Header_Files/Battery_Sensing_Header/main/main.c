#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "blink.h"
#include "Battery_Sense.h"

void app_main(void)
{
    setup_adc();

    led_setup();

    while(1){
        // If check battery returns true, battery is too low
        if(check_battery()){
            led_turn_on();
        }
        else{
            led_turn_off();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

