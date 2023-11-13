///code from Matthew Hogan link: https://medium.com/@Matthew_Hogan/interesting-electronic-components-1-hls-lfcd2-640d897f9fc9
//refactored by Brittney, Misi and Jordy
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "math.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
// #include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h> 

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rosidl_runtime_c/string_functions.h>

static const char *TAG = "UART Lidar test";

#define BUF_SIZE_U2 (2520)
unsigned char sync[2];
float * ranges_val[360];
int num_of_times_it_made_it = 0;
int num_reset = 0;
const uart_port_t uart_port_numb = UART_NUM_2;
sensor_msgs__msg__LaserScan lidar_msg_;

uint16_t rpms; // derived from the rpm bytes in lfcd packet (lidar)
//--------------------------------------

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;

void uart_setup(){
    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_LOGI(TAG, "created uart config");
    //vTaskDelay(10);
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (2520); //TODO: Change this
    ESP_LOGI(TAG, "created uart buffer size");
    //vTaskDelay(10);

    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "created intr alloc flag variable");
    //vTaskDelay(10);    

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_port_numb, uart_buffer_size*2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_set_mode(uart_port_numb,UART_MODE_UART));
    ESP_LOGI(TAG, "Uart driver install");
    //vTaskDelay(10); 
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_port_numb, &uart_config));
    //uart_param_config(uart_port_numb, &uart_config);

    ESP_LOGI(TAG, "Setup config");
    //vTaskDelay(10);

    // Set UART2 pins(TX: IO17, RX: IO16, RTS: -1, CTS: -1)
    ESP_ERROR_CHECK(uart_set_pin(uart_port_numb, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "Setup pins");
    //vTaskDelay(10);
}

int uart_read(uint8_t *data){
    return uart_read_bytes(uart_port_numb, data, (BUF_SIZE_U2), 50); 

}

int uart_write(char* str){
    return uart_write_bytes(uart_port_numb, (const char*)str, sizeof(str));
}

void poll_lidar(){
    int ready = 0;
    unsigned char frame[BUF_SIZE_U2]; 
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE_U2);
    int index;
    int did_not_work=0;
    int error_tx, error_tx_end;
    char* test_str = "b";
    char* test_str_end = "e";
    bool successful_scan = false;
    uint8_t good_sets = 0;
    uint32_t motor_speed = 0;
    rpms = 0;
    size_t uart_read_size;


    while(!successful_scan){
        uart_read_size = uart_read(data);
            
        if(uart_read_size > 0){
            sync[0] = data[0];
            sync[1] = data[1];
        }
        
        //First capture the start byte of a frame:
        //Once start byte captured, read remaining frame into array:
        if (sync[0] == 0xFA && sync[1] == 0xA0) {
            frame[0] = 0xFA;
            frame[1] = 0xA0;
            for (int v = 2; v <= 2520; v++) {
                frame[v] = data[v]; //copying the data from data array to frame
            }
            ready = 1;
        }  
        
        //Once frame captured, extract range/angle and convert to x/y:
        if (ready == 1) {
            // lidar_msg_.data->angle_increment = (2.0*M_PI/360.0);
            // lidar_msg_.data->angle_min = 0.0;
            // lidar_msg_.data->angle_max = 2.0 * M_PI - lidar_msg_.angle_increment;
            // lidar_msg_.data->range_min = 0.12;
            // lidar_msg_.data->range_max = 3.5;
            // lidar_msg_.size = 360;
            lidar_msg_.angle_increment = (2.0*M_PI/360.0);
            lidar_msg_.angle_min = 0.0;
            lidar_msg_.angle_max = 2.0 * M_PI - lidar_msg_.angle_increment;
            lidar_msg_.range_min = 0.12;
            lidar_msg_.range_max = 3.5;
            
            for (uint16_t i = 0; i < 2520; i = i + 42) {
                if (frame[i] == 0xFA && frame[(i + 1)] == 0xA0 + (i / 42)) {                    
                    good_sets++;
                    motor_speed += (frame[i+3] << 8) + frame[i+2]; //accumlate count for avg. time increment
                    for (uint16_t j = i + 4; j < i + 40; j = j + 6) {
                    
                        index = 6*(i/42) + (j-4-i)/6;

                        uint8_t rangeA = frame[j + 2];
                        uint8_t rangeB = frame[j + 3];
                        uint8_t rangeC = frame[j];
                        uint8_t rangeD = frame[j+1];

                        uint16_t range = (rangeB << 8) + rangeA;
                        uint16_t intensity = (rangeD << 8) + rangeC; 

                        lidar_msg_.ranges.data[359-index] = range/1000.0;
                        lidar_msg_.intensities.data[359-index] = intensity;

                        // if (index != 0 && range != 0) {
                        //         float Radians = (index * M_PI) / 180;
                        //         float x = range * cos(Radians);
                        //         float y = range * sin(Radians);
                        //         // printf("%f , %f \n", x, y);
                        //         ESP_LOGI(TAG, "x,y: %f , %f \n", x,y);
                        //         //vTaskDelay(10);
                        //     }
                    }

                }
            }
            
            rpms = motor_speed / good_sets / 10;
            lidar_msg_.time_increment = (float)(1.0 / (rpms*6));
            lidar_msg_.scan_time = lidar_msg_.time_increment * 360;

            ready = 0;
            num_of_times_it_made_it++;
            successful_scan = true;
            //vTaskDelay(1);
        }  
        else {
            sync[0] = 0;
            sync[1] = 0;
            did_not_work++;
            }

        if(did_not_work >= 5 )
        {
            did_not_work =0;
            num_reset++;
            error_tx_end = uart_write(test_str_end);
            ESP_LOGI(TAG, "Number of resets of lidar: %d\n Current value of conversions: %d", num_reset, num_of_times_it_made_it);
            vTaskDelay(5);                      
            error_tx = uart_write(test_str);
        }
    }
    //return lidar_msg_;
}

void RCL_setup(){
    
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing angle increment: %f\n", (float) lidar_msg_.angle_increment);
		printf("Publishing angle min: %f\n", (float) lidar_msg_.angle_min);
		printf("Publishing angle max: %f\n", (float) lidar_msg_.angle_max);
		printf("Publishing range min: %f\n", (float) lidar_msg_.range_min);
		printf("Publishing range max: %f\n", (float) lidar_msg_.range_max);

        for(int i = 0; i<360 ; i++)
        {
		    printf("Publishing range %d: %f\n", i, (float) lidar_msg_.ranges.data[i]);
		    printf("Publishing range %d: %f\n", i, (float) lidar_msg_.intensities.data[i]);

        }

		RCSOFTCHECK(rcl_publish(&publisher, &lidar_msg_, NULL));
		//lidar_msg_.data++;
	}
}

void micro_ros_task ( void * arg){
    //-------------------- setup of node, and publisher
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

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

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
		"freertos_int32_publisher"));

    // create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	//msg.data = 0;

    while(1) {

        poll_lidar();

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		//usleep(10000);
        vTaskDelay(100);
    }

    // free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void) {

    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Setup is starting");

    uart_setup();

    ESP_LOGI(TAG, "Setup is done");
    //vTaskDelay(10);
    // char* test_str = "b";
    // char* test_str_end = "e";
    // int error_tx, error_tx_end;
    // int index;
    //error_tx = uart_write_bytes(uart_port_numb, (const char*)test_str, sizeof(test_str));
    //ESP_ERROR_CHECK(uart_wait_tx_done(uart_port_numb,100));
    //error_tx = uart_write(test_str_end);
    char* start_lidar = "b";
    uart_write(start_lidar);


    ESP_LOGI(TAG, "%s, this to start sending data from lidar",start_lidar);

    rosidl_runtime_c__String__init(&(lidar_msg_.header.frame_id));
    lidar_msg_.header.frame_id = micro_ros_string_utilities_set(lidar_msg_.header.frame_id, "laser");


    //sets the lidar message
    lidar_msg_.ranges.capacity = 360;
    lidar_msg_.ranges.size = 360;
    lidar_msg_.ranges.data = (float*) malloc(lidar_msg_.ranges.capacity * sizeof(float));

    lidar_msg_.intensities.capacity = 360;
    lidar_msg_.intensities.size = 360;
    lidar_msg_.intensities.data = (float*) malloc(lidar_msg_.ranges.capacity * sizeof(float));

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);

        
}