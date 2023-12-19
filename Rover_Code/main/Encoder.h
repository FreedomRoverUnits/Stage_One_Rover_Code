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
#pragma once
#ifndef OUR_ENCODER_H
#define OUR_ENCODER_H

#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG_encoder = "Testing Motor Control";

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
void setup_both_encoders(pcnt_unit_handle_t * pcnt_unit_motor_A, pcnt_unit_handle_t * pcnt_unit_motor_B,
                        pcnt_channel_handle_t * pcnt_chan_a, pcnt_channel_handle_t * pcnt_chan_b);
                        
#endif