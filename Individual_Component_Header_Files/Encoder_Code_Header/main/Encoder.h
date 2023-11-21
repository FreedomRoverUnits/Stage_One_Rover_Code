#pragma once
#ifndef OUR_ENCODER_H
#define OUR_ENCODER_H

#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "Testing Motor Control";

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  -1000

#define MOTOR_A_ENCODER 14
#define MOTOR_B_ENCODER 25

extern unsigned long prev_update_time_test;
extern unsigned long pre_encoder_ticks_test;

void create_pcnt_for_motors(pcnt_unit_handle_t* motor_A, pcnt_unit_handle_t* motor_B);
void create_pcnt_channels_for_motors(pcnt_unit_handle_t* motor_A, pcnt_unit_handle_t* motor_B, 
                                        pcnt_channel_handle_t* channel_A, pcnt_channel_handle_t* channel_B);
void start_pcnt_for_motors(pcnt_unit_handle_t* motor_A, pcnt_unit_handle_t* motor_B);
float getRPM_motor1(pcnt_unit_handle_t * motor_enc);
float getRPM_motor2(pcnt_unit_handle_t * motor_enc);

#endif