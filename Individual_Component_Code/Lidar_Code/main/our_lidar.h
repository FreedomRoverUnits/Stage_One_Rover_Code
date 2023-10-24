#ifndef OUR_LIDAR_H
#define OUR_LIDAR_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "math.h"

static const char *Lidar_TAG = "UART Lidar test";

#define BUF_SIZE_U2 (2520)
unsigned char sync[2];
unsigned char frame[BUF_SIZE_U2]; 
int ready = 0;
int num_of_times_it_made_it = 0;
int did_not_work=0;
int num_reset = 0;
const uart_port_t uart_port_numb = UART_NUM_2;

void uart_setup();
int uart_read(uint8_t* data);
int uart_write(char* str);

#endif