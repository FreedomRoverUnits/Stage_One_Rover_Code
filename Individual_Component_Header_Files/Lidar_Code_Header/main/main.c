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
# include "our_lidar.h"
#include "esp_timer.h"

rcl_publisher_t lidar_publisher;
sensor_msgs__msg__LaserScan lidar_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

unsigned long long time_offset = 0;


int64_t millis_time() 
{
    return ((esp_timer_get_time())/1000);
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing angle increment: %f\n", (float) lidar_msg.angle_increment);
		printf("Publishing angle min: %f\n", (float) lidar_msg.angle_min);
		printf("Publishing angle max: %f\n", (float) lidar_msg.angle_max);
		printf("Publishing range min: %f\n", (float) lidar_msg.range_min);
		printf("Publishing range max: %f\n", (float) lidar_msg.range_max);

        for(int i = 0; i<360 ; i++)
        {
		    printf("Publishing range %d: %f\n", i, (float) lidar_msg.ranges.data[i]);
		    printf("Publishing intensities %d: %f\n", i, (float) lidar_msg.intensities.data[i]);
        }
		RCSOFTCHECK(rcl_publish(&lidar_publisher, &lidar_msg, NULL));
	}
}


void micro_ros_task_lidar (){
    ESP_LOGI(TAG_LIDAR, "Inside micro ros for lidar");

    //-------------------- setup of node, and publisher
    allocator = rcl_get_default_allocator();
	// rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    ESP_LOGI(TAG_LIDAR, "IP address: %s", CONFIG_MICRO_ROS_AGENT_IP);
    ESP_LOGI(TAG_LIDAR, "Port number: %s", CONFIG_MICRO_ROS_AGENT_PORT);
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

    ESP_LOGI(TAG_LIDAR, "create init_options");

	// create init_options
	//RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    // bool value_rclc = false;
    // if (RCL_RET_ERROR == rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)){
    //     value_rclc = true;
    // }
    // ESP_LOGI(TAG_LIDAR, "result for rclc: %d ", value_rclc); //next best thing to do to debug is cross-reference with linorobot and main.c
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    ESP_LOGI(TAG_LIDAR, "create node"); //next best thing to do to debug is cross-reference with linorobot and main.c

	// create node
	// rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "scan", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&lidar_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
		"rover_publisher_lidar"));

    // create timer, never calls the callback because we never timeout
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	// rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while(1) {

        poll_lidar(&lidar_msg);

    	struct timespec time_stamp = getTime();

		lidar_msg.header.stamp.sec = time_stamp.tv_sec;
    	lidar_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    	RCSOFTCHECK(rcl_publish(&lidar_publisher, &lidar_msg, NULL));
        //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(250/portTICK_PERIOD_MS); //this is how often it is getting called // 250ms
    }

    // free resources
	RCCHECK(rcl_publisher_fini(&lidar_publisher, &node));
	RCCHECK(rcl_node_fini(&node));
	RCCHECK(rcl_timer_fini(&timer));
	
	// RCCHECK(rclc_executor_fini());

  	vTaskDelete(NULL);
}

void app_main(void){
    lidar_setup(&lidar_msg);

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

	//micro_ros_task_lidar();
    xTaskCreate(micro_ros_task_lidar, "uros_task_lidar", 10000, NULL, 5, NULL);

	// free(&(lidar_msg.ranges.data));
	// free(&(lidar_msg.intensities.data));
    //xTaskCreate(micro_ros_task_lidar, "uros_task", 10000, NULL, 5, NULL);
}