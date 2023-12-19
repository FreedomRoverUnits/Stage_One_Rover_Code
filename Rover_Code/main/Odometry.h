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
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

typedef struct L_Odometry{
    nav_msgs__msg__Odometry odom_msg_;
    float x_pos_;
    float y_pos_;
    float heading_;
}Odometry;

void Odometry_Constructor(Odometry * our_odo);
void update(Odometry * our_odo, float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
nav_msgs__msg__Odometry getData(Odometry * our_odo);
void euler_to_quat(float roll, float pitch, float yaw, float* q);

#endif