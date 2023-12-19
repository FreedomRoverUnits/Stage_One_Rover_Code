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
#include "Odometry.h"
#include "math.h"
void Odometry_Constructor(Odometry * our_odo){
    our_odo->x_pos_ = 0.0;
    our_odo->y_pos_ = 0.0;
    our_odo->heading_ = 0.0;
    rosidl_runtime_c__String__init(&(our_odo->odom_msg_.header.frame_id));
    our_odo->odom_msg_.header.frame_id = micro_ros_string_utilities_set(our_odo->odom_msg_.header.frame_id, "odom");
    rosidl_runtime_c__String__init(&(our_odo->odom_msg_.child_frame_id));
    our_odo->odom_msg_.child_frame_id = micro_ros_string_utilities_set(our_odo->odom_msg_.child_frame_id, "base_footprint");
}

void update(Odometry * our_odo, float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z){
    float delta_heading = angular_vel_z * vel_dt; //radians
    float cos_h = cos(our_odo->heading_);
    float sin_h = sin(our_odo->heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; //m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; //m

    //calculate current position of the robot
    our_odo->x_pos_ += delta_x;
    our_odo->y_pos_ += delta_y;
    our_odo->heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, our_odo->heading_, q);

    //robot's position in x,y, and z
    our_odo->odom_msg_.pose.pose.position.x = our_odo->x_pos_;
    our_odo->odom_msg_.pose.pose.position.y = our_odo->y_pos_;
    our_odo->odom_msg_.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    our_odo->odom_msg_.pose.pose.orientation.x = (double) q[1];
    our_odo->odom_msg_.pose.pose.orientation.y = (double) q[2];
    our_odo->odom_msg_.pose.pose.orientation.z = (double) q[3];
    our_odo->odom_msg_.pose.pose.orientation.w = (double) q[0];

    our_odo->odom_msg_.pose.covariance[0] = 0.05;
    our_odo->odom_msg_.pose.covariance[7] = 0.05;
    our_odo->odom_msg_.pose.covariance[35] = 0.05;

    //linear speed from encoders
    our_odo->odom_msg_.twist.twist.linear.x = linear_vel_x;
    our_odo->odom_msg_.twist.twist.linear.y = linear_vel_y;
    our_odo->odom_msg_.twist.twist.linear.z = 0.0;

    //angular speed from encoders
    our_odo->odom_msg_.twist.twist.angular.x = 0.0;
    our_odo->odom_msg_.twist.twist.angular.y = 0.0;
    our_odo->odom_msg_.twist.twist.angular.z = angular_vel_z;

    our_odo->odom_msg_.twist.covariance[0] = 0.0001;
    our_odo->odom_msg_.twist.covariance[7] = 0.0001;
    our_odo->odom_msg_.twist.covariance[35] = 0.0001;
}

nav_msgs__msg__Odometry getData(Odometry * our_odo){
    return our_odo->odom_msg_;
}

void euler_to_quat(float roll, float pitch, float yaw, float* q){
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}