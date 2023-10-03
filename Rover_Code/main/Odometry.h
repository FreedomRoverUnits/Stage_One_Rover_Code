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