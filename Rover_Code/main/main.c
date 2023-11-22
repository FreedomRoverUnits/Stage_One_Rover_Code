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
//#include <Arduino.h>
//#include <micro_ros_platformio.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// Micro Ros libraries
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h> //????
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include "esp_timer.h"

//#include "config.h"
#include "PWM.h"
#include "Kinematics.h"
#include "PID.h"
#include "Odometry.h"
#include "OUR_IMU.h"
#include "Encoder.h"
#include "blink.h"
#include "Battery_Sense.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = millis_time();} \
  if (millis_time() - init > MS) { X; init = millis_time();} \
} while (0)

//////////////////////// Defines From Lino Config ////////////////////////////////////////
#define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
#define K_P 0.6                             // P constant
#define K_I 0.8                             // I constant
#define K_D 0.5                             // D constant
#define MOTOR_MAX_RPM 140                   // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 144000              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 144000              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 144000              // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 144000              // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.152                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.271            // distance between left and right wheels


rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

bool battery_cutoff = false;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Our Encoder for the motors
pcnt_unit_handle_t pcnt_unit_motor_1 = NULL;
pcnt_unit_handle_t pcnt_unit_motor_2 = NULL;
pcnt_channel_handle_t pcnt_chan_1 = NULL;
pcnt_channel_handle_t pcnt_chan_2 = NULL;

PID motor1_pid, motor2_pid;

Kinematics kinematics;

Odometry odometry;

bool destroyEntities(void);
bool createEntities(void);
void moveBase(void);
void publishData(void);
int64_t millis_time(void);
struct timespec getTime(void);
void rclErrorLoop(void);
void syncTime(void);

static const char *TAG_ERROR = "Debugging Test";

void micro_ros_main_loop(void * arg){
        state = AGENT_AVAILABLE;
        while (1) {
        switch (state) 
        {
            case WAITING_AGENT:
                ESP_LOGI(TAG_ERROR, "wait");
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                ESP_LOGI(TAG_ERROR, "looking for agent");
                state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) 
                {
                    ESP_LOGI(TAG_ERROR, "agent not found");
                    destroyEntities();
                }
                break;
            case AGENT_CONNECTED:
                //ESP_LOGI(TAG_ERROR, "found agent");
                //EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                state = AGENT_CONNECTED;
                if (state == AGENT_CONNECTED) 
                {
                    //ESP_LOGI(TAG_ERROR, "connecting to agent, spin");
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                }
                break;
            case AGENT_DISCONNECTED:
                ESP_LOGI(TAG_ERROR, "destroy");
                destroyEntities();
                state = WAITING_AGENT;
                break;
            default:
                ESP_LOGI(TAG_ERROR, "why");
                break;
        }
    }
}


void app_main(void)
{
    //set up led
    led_setup();

    //setup ADC for battery
    setup_adc();

    //setting up Encoders
    setup_both_encoders(&pcnt_unit_motor_1, &pcnt_unit_motor_2, &pcnt_chan_1, &pcnt_chan_2);
    
    //setups both wheels
    setup_rover_wheels();

    //setup pid for both wheels
    PID_initialize(&motor1_pid,0,100,K_P,K_I,K_D); //left
    PID_initialize(&motor2_pid,0,100,K_P,K_I,K_D); //right

    //setup Kinematics 
    Kinematics_Constructor(&kinematics, LINO_BASE, MOTOR_MAX_RPM, MAX_RPM_RATIO, MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE, WHEEL_DIAMETER, LR_WHEELS_DISTANCE);

    //setup IMU
    setup_imu();

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    //ESP_LOGI(TAG_ERROR, "transport");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    //ESP_LOGI(TAG_ERROR, "transport2");
    #endif

    state = WAITING_AGENT;

    xTaskCreate(micro_ros_main_loop, "uros_task", 3000, NULL, 5, NULL);
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase(); //one thing todo in this
       publishData();
    }
}

void twistCallback(const void * msgin) 
{
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis_time();
}

bool createEntities()
{

    allocator = rcl_get_default_allocator();
    
    //Added in by us
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    //RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    //Ends here
    
    //create init_options (Needs to be edit to match line 54 in int32 pub example)
    //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));

    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));

    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), //TODO : double check how this works
        "imu/data"
    ));

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    // create timer for actuating the motors at 50 Hz (1000/20)
    //const unsigned int control_timeout = 20;
    
    const unsigned int control_timeout = 1000;
    
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));

    executor = rclc_executor_get_zero_initialized_executor();

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    //digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rclc_support_fini(&support));

    //digitalWrite(LED_PIN, HIGH);
    
    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    rover_stop(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_TIMER_1); //both wheels stop
}

void moveBase()
{
    //ESP_LOGI(TAG_ERROR, "Twist: linear x: %lf linear y: %lf angular z: %lf", twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
    // brake if there's no command received, or when it's only the first command sent
    //if(((millis_time()- prev_cmd_time) >= 200)) 
    if(((millis_time() - prev_cmd_time) >= 1000))
    {
        //ESP_LOGI(TAG_ERROR, "Millis: %lli pre_cmd_time %lu", millis_time(),  prev_cmd_time);
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        // digitalWrite(LED_PIN, HIGH);
    }

    //Put battery check Here
    if(check_battery()){
        fullStop();
        led_turn_on();
        ESP_LOGI(TAG_ERROR, "Battery just ooofffed");
    }

    //ESP_LOGI(TAG_ERROR, "Twist: linear x: %lf linear y: %lf angular z: %lf", twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);

    // get the required rpm for each motor based on required velocities, and base used
    R_rpm req_rpm = getRPM(&kinematics,
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    //ESP_LOGI(TAG_ERROR, "motor1: %f motor2: %f motor3: %f motor4: %f", req_rpm.motor1, req_rpm.motor2, req_rpm.motor3, req_rpm.motor4);

    // get the current speed of each motor
    float current_rpm1 = getRPM_motor1(&pcnt_unit_motor_1); //TODO: tuesday!!1!!!! //Get rpm might overflow
    float current_rpm2 = getRPM_motor2(&pcnt_unit_motor_2);
    float current_rpm3 = 0.0; // Not using these guys make sure we can pass 0.0
    float current_rpm4 = 0.0;

    //ESP_LOGI(TAG_ERROR, "current rpm1: %f current rpm2; %f", current_rpm1, current_rpm2);
    //ESP_LOGI(TAG_ERROR, "Duty Cycle from pid: %lf", compute_pid(&motor1_pid, req_rpm.motor1, current_rpm1));
    //ESP_LOGI(TAG_ERROR, "Duty Cycle from pid: %lf", compute_pid(&motor2_pid, req_rpm.motor2, current_rpm2));

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    spin_dir_left(MCPWM_UNIT_0,MCPWM_TIMER_0,compute_pid(&motor1_pid, req_rpm.motor1, current_rpm1));
    spin_dir_right(MCPWM_UNIT_0,MCPWM_TIMER_1,compute_pid(&motor2_pid, req_rpm.motor2, current_rpm2));


    R_velocities current_vel = getVelocities(&kinematics,
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    //ESP_LOGI(TAG_ERROR, "Current rpm1: %f Current rpm2: %f Current rpm3: %f Current rpm4: %f", current_rpm1, current_rpm2, current_rpm3, current_rpm4);

    unsigned long now = millis_time();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    
    update(&odometry,
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

void publishData()
{
    odom_msg = getData(&odometry);
    imu_msg = getIMUData(); // imu method call

    //calculate_IMU_error();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis_time();

    //RCCHECK(rmw_uros_sync_session(10));
    rmw_uros_sync_session(10);

    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 

    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;

}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis_time() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop() 
{
    while(true)
    {
        led_blink(2);
    }
}

int64_t millis_time() 
{
    return ((esp_timer_get_time())/1000);
}