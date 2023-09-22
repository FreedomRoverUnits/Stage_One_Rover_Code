// Copyright (c) 2021 Juan Miguel Jimeno
//
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

#include "PID.h"

void PID_initialize(PID* pid, float min_val, float max_val, float kp, float ki, float kd){
    pid->min_val_ = min_val;
    pid->max_val_ = max_val;
    pid->kp_ = kp;
    pid->ki_ = ki;
    pid->kd_ = kd;
}

double compute(PID* pid, float setpoint, float measured_value)
{
    double error;
    double pid_value;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    pid->integral_ += error;
    pid->derivative_ = error - pid->prev_error_;

    if(setpoint == 0 && error == 0)
    {
        pid->integral_ = 0;
        pid->derivative_ = 0;
    }

    pid_value = (pid->kp_ * error) + (pid->ki_ * pid->integral_) + (pid->kd_ * pid->derivative_);
    pid->prev_error_ = error;

    return constrain(pid_value, pid->min_val_, pid->max_val_);
}

void updateConstants(PID* pid, float kp, float ki, float kd)
{
    pid->kp_ = kp;
    pid->ki_ = ki;
    pid->kd_ = kd;
}
double constrain(double pid_value, float min_val, float max_val){

    if(pid_value < min_val)
        return min_val;
    else if(pid_value > max_val)
        return max_val;
    else 
        return pid_value;
}
