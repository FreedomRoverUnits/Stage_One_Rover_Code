#ifndef OUR_PWM_H
#define OUR_PWM_H

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

static const char *TAG = "Testing PWM/Speed Motor Control";

#define LEFT_WHEEL_R 15
#define LEFT_WHEEL_B 4

#define RIGHT_WHEEL_R 19
#define RIGHT_WHEEL_B 18

//motor control
void mcpwm_gpio_initialize_both_wheels(void);
void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);
void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle);
void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num);

//full rovor body control
void rover_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num_left_wheel, mcpwm_timer_t timer_num_right_wheel);
void rover_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num_left_wheel, float duty_cycle_left_wheel, mcpwm_timer_t timer_num_right_wheel, float duty_cycle_right_wheel);
void rover_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num_left_wheel, float duty_cycle_left_wheel, mcpwm_timer_t timer_num_right_wheel, float duty_cycle_right_wheel);
void rover_right_turn(void); //place holder for turning right TODO
void rover_left_turn(void); //place holder for turning left TODO

//setup for rover wheels
void setup_rover_wheels(void);


#endif