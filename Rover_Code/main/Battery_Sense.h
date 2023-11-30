#ifndef OUR_BATTERY_SENSE
#define OUR_BATTERY_SENSE

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

const static char *TAG_ADC = "ADC";

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

void setup_adc(void);
bool check_battery(void);
void tear_down_adc(void);
void example_adc_calibration_deinit(adc_cali_handle_t handle);
bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

#endif