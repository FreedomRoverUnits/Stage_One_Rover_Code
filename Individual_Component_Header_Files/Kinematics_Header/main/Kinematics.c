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
#include "Kinematics.h"

void Kinematics_Constructor(Kinematics * our_kino, base robot_base, int motor_max_rpm, float max_rpm_ratio,
                   float motor_operating_voltage, float motor_power_max_voltage,
                   float wheel_diameter, float wheels_y_distance){

    our_kino->base_platform_ = robot_base;
    our_kino->wheels_y_distance_ = wheels_y_distance;
    our_kino->wheel_circumference_ = M_PI * wheel_diameter;
    our_kino->total_wheels_ = getTotalWheels(robot_base);

    //If else if to mimic arduino constrain
    if(motor_power_max_voltage < 0){
        motor_power_max_voltage = 0;
    }
    else if(motor_power_max_voltage > motor_operating_voltage){
        motor_power_max_voltage = motor_operating_voltage;
    }
    else{
        motor_power_max_voltage = motor_power_max_voltage;
    }

    our_kino->max_rpm_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

R_rpm calculateRPM(Kinematics * our_kino, float linear_x, float linear_y, float angular_z){
    float tangential_vel = angular_z * (our_kino->wheels_y_distance_ / 2.0);

    //convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60.0;
    float linear_vel_y_mins = linear_y * 60.0;
    //convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / our_kino->wheel_circumference_;
    float y_rpm = linear_vel_y_mins / our_kino->wheel_circumference_;
    float tan_rpm = tangential_vel_mins / our_kino->wheel_circumference_;

    float a_x_rpm = fabs(x_rpm);
    float a_y_rpm = fabs(y_rpm);
    float a_tan_rpm = fabs(tan_rpm);

    float xy_sum = a_x_rpm + a_y_rpm;
    float xtan_sum = a_x_rpm + a_tan_rpm;

    //calculate the scale value how much each target velocity
    //must be scaled down in such cases where the total required RPM
    //is more than the motor's max RPM
    //this is to ensure that the required motion is achieved just with slower speed
    if(xy_sum >= our_kino->max_rpm_ && angular_z == 0)
    {
        float vel_scaler = our_kino->max_rpm_ / xy_sum;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }
    else if(xtan_sum >= our_kino->max_rpm_ && linear_y == 0)
    {
        float vel_scaler = our_kino->max_rpm_ / xtan_sum;

        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    R_rpm rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    //rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);
    if(rpm.motor1 < -(our_kino->max_rpm_)){
        rpm.motor1 = -(our_kino->max_rpm_);
    }
    else if(rpm.motor1 > our_kino->max_rpm_){
        rpm.motor1 = our_kino->max_rpm_;
    }
    else{
        rpm.motor1 = rpm.motor1;
    }

    //front-right motor
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    //rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);
    if(rpm.motor2 < -(our_kino->max_rpm_)){
        rpm.motor2 = -(our_kino->max_rpm_);
    }
    else if(rpm.motor2 > our_kino->max_rpm_){
        rpm.motor2 = our_kino->max_rpm_;
    }
    else{
        rpm.motor2 = rpm.motor2;
    }

    //rear-left motor
    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    //rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);
    if(rpm.motor3 < -(our_kino->max_rpm_)){
        rpm.motor3 = -(our_kino->max_rpm_);
    }
    else if(rpm.motor3 > our_kino->max_rpm_){
        rpm.motor3 = our_kino->max_rpm_;
    }
    else{
        rpm.motor3 = rpm.motor3;
    }

    //rear-right motor
    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    //rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
    if(rpm.motor4 < -(our_kino->max_rpm_)){
        rpm.motor4 = -(our_kino->max_rpm_);
    }
    else if(rpm.motor4 > our_kino->max_rpm_){
        rpm.motor4 = our_kino->max_rpm_;
    }
    else{
        rpm.motor4 = rpm.motor4;
    }

    return rpm;
}

R_rpm getRPM(Kinematics * our_kino, float linear_x, float linear_y, float angular_z){
    if(our_kino->base_platform_ == DIFFERENTIAL_DRIVE || our_kino->base_platform_ == SKID_STEER){
        linear_y = 0;
    }

    return calculateRPM(our_kino, linear_x, linear_y, angular_z);
}

R_velocities getVelocities(Kinematics * our_kino, float rpm1, float rpm2, float rpm3, float rpm4){
    R_velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    if(our_kino->base_platform_ == DIFFERENTIAL_DRIVE)
    {
        rpm3 = 0.0;
        rpm4 = 0.0;
    }

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / our_kino->total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * our_kino->wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / our_kino->total_wheels_) / 60.0; // RPM
    if(our_kino->base_platform_ == MECANUM)
        vel.linear_y = average_rps_y * our_kino->wheel_circumference_; // m/s
    else
        vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / our_kino->total_wheels_) / 60.0;
    vel.angular_z =  (average_rps_a * our_kino->wheel_circumference_) / (our_kino->wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

int getTotalWheels(base robot_base){
    switch(robot_base)
    {
        case DIFFERENTIAL_DRIVE:    return 2;
        case SKID_STEER:            return 4;
        case MECANUM:               return 4;
        default:                    return 2;
    }
}

float getMaxRPM(Kinematics * our_kino){
    return our_kino->max_rpm_;
}