#ifndef KINEMATICS_H
#define KINEMATICS_H

typedef enum L_base {DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM}base;

typedef struct L_rpm
{
    float motor1;
    float motor2;
    float motor3;
    float motor4;
}R_rpm;

typedef struct L_velocities
{
    float linear_x;
    float linear_y;
    float angular_z;
}R_velocities;

typedef struct L_pwm
{
    int motor1;
    int motor2;
    int motor3;
    int motor4;
}R_pwm;

typedef struct L_Kinematics{
    R_pwm pwm;
    R_velocities velocities;
    R_rpm rpm;
    float max_rpm_;
    float wheels_y_distance_;
    float pwm_res_;
    float wheel_circumference_;
    int total_wheels_;
}Kinematics;

void Kinematics_Constructor(Kinematics * our_kino, base robot_base, int motor_max_rpm, float max_rpm_ratio,
                   float motor_operating_voltage, float motor_power_max_voltage,
                   float wheel_diameter, float wheels_y_distance);

R_velocities getVelocities(Kinematics * our_kino, float rpm1, float rpm2, float rpm3, float rpm4);
R_rpm getRPM(Kinematics * our_kino, float linear_x, float linear_y, float angular_z);
float getMaxRPM(Kinematics * our_kino);
R_rpm calculateRPM(Kinematics * our_kino, float linear_x, float linear_y, float angular_z);
int getTotalWheels(base robot_base);
#endif