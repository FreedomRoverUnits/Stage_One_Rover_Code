#ifndef OUR_LIDAR_H
#define OUR_LIDAR_H

///code from Matthew Hogan link: https://medium.com/@Matthew_Hogan/interesting-electronic-components-1-hls-lfcd2-640d897f9fc9
//refactored by Brittney, Misi and Jordy
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "math.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h> 

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rosidl_runtime_c/string_functions.h>

static const char *TAG_LIDAR = "UART Lidar test";

#define BUF_SIZE_U2 (2520)

void uart_setup();
int uart_read(uint8_t* data);
int uart_write(char* str);
bool poll_lidar(sensor_msgs__msg__LaserScan * lidar_msg_);
void RCL_Lidar_setup();
void lidar_setup(sensor_msgs__msg__LaserScan * lidar_msg_);

#endif