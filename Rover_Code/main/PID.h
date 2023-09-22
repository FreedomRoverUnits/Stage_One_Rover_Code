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

#ifndef OUR_PID_H
#define OUR_PID_H

typedef struct PID_{
    float min_val_;
    float max_val_;
    float kp_;
    float ki_;
    float kd_;
    double integral_;
    double derivative_;
    double prev_error_; 
}PID;

void PID_initialize(PID* pid,float min_val, float max_val, float kp, float ki, float kd);
double constrain(double pid_value, float min_val, float max_val);
double compute_pid(PID* pid, float setpoint, float measured_value);
void updateConstants(PID* pid, float kp, float ki, float kd);

#endif
