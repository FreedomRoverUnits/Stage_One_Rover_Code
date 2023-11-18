#include "our_lidar.h"

// publisher and lidar message
// rcl_publisher_t lidar_publisher;
// sensor_msgs__msg__LaserScan lidar_msg_;

//UART 
const uart_port_t uart_port_numb = UART_NUM_2;

//Lidar messages
unsigned char sync[2];
uint16_t rpms; // derived from the rpm bytes in lfcd packet (lidar)

//Counter for debugging purposes, when receiving messages from lidar to mcu.
int num_of_times_it_made_it = 0;
int num_reset = 0;


void uart_setup(){
    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_LOGI(TAG_LIDAR, "created uart config");

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (2520); //TODO: Change this
    ESP_LOGI(TAG_LIDAR, "created uart buffer size");


    int intr_alloc_flags = 0;
    ESP_LOGI(TAG_LIDAR, "created intr alloc flag variable");


    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_port_numb, uart_buffer_size*2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_set_mode(uart_port_numb,UART_MODE_UART));
    ESP_LOGI(TAG_LIDAR, "Uart driver install");

    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_port_numb, &uart_config));

    ESP_LOGI(TAG_LIDAR, "Setup config");

    // Set UART2 pins(TX: IO17, RX: IO16, RTS: -1, CTS: -1)
    ESP_ERROR_CHECK(uart_set_pin(uart_port_numb, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG_LIDAR, "Setup pins");
}

int uart_read(uint8_t *data){
    return uart_read_bytes(uart_port_numb, data, (BUF_SIZE_U2), 50); 
}

int uart_write(char* str){
    return uart_write_bytes(uart_port_numb, (const char*)str, sizeof(str));
}

void poll_lidar(sensor_msgs__msg__LaserScan * lidar_msg_){
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
            lidar_msg_->angle_increment = (2.0*M_PI/360.0);
            lidar_msg_->angle_min = 0.0;
            lidar_msg_->angle_max = 2.0 * M_PI - lidar_msg_->angle_increment;
            lidar_msg_->range_min = 0.12;
            lidar_msg_->range_max = 3.5;
            
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

                        lidar_msg_->ranges.data[359-index] = range/1000.0;
                        lidar_msg_->intensities.data[359-index] = intensity;
                    }

                }
            }
            
            rpms = motor_speed / good_sets / 10;
            lidar_msg_->time_increment = (float)(1.0 / (rpms*6));
            lidar_msg_->scan_time = lidar_msg_->time_increment * 360;

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
            ESP_LOGI(TAG_LIDAR, "Number of resets of lidar: %d\n Current value of conversions: %d", num_reset, num_of_times_it_made_it);
            vTaskDelay(5);                      
            error_tx = uart_write(test_str);
        }
    }
}

void RCL_setup(){
    
}

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time, sensor_msgs__msg__LaserScan * lidar_msg_)
// {
// 	RCLC_UNUSED(last_call_time);
// 	if (timer != NULL) {
// 		printf("Publishing angle increment: %f\n", (float) lidar_msg_->angle_increment);
// 		printf("Publishing angle min: %f\n", (float) lidar_msg_->angle_min);
// 		printf("Publishing angle max: %f\n", (float) lidar_msg_->angle_max);
// 		printf("Publishing range min: %f\n", (float) lidar_msg_->range_min);
// 		printf("Publishing range max: %f\n", (float) lidar_msg_->range_max);

//         for(int i = 0; i<360 ; i++)
//         {
// 		    printf("Publishing range %d: %f\n", i, (float) lidar_msg_->ranges.data[i]);
// 		    printf("Publishing range %d: %f\n", i, (float) lidar_msg_->intensities.data[i]);

//         }

// 		RCSOFTCHECK(rcl_publish(&lidar_publisher, &lidar_msg_, NULL));
// 		//lidar_msg_->data++;
// 	}
// }

/*
void micro_ros_task ( void * arg){
    ESP_LOGI(TAG_LIDAR, "Inside micro ros for lidar");

    //-------------------- setup of node, and publisher
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

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
    bool value_rclc = false;
    if (RCL_RET_ERROR == rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator)){
        value_rclc = true;
    }
    ESP_LOGI(TAG_LIDAR, "result for rclc: %d ", value_rclc); //next best thing to do to debug is cross-reference with linorobot and main.c
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    ESP_LOGI(TAG_LIDAR, "create node"); //next best thing to do to debug is cross-reference with linorobot and main.c

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "/scan/laser_scan", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&lidar_publisher,
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

    while(1) {

        poll_lidar();

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(100); //this is how often it is getting called
    }

    // free resources
	RCCHECK(rcl_publisher_fini(&lidar_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}*/

void lidar_setup(sensor_msgs__msg__LaserScan * lidar_msg_){
    esp_log_level_set(TAG_LIDAR, ESP_LOG_INFO);

    ESP_LOGI(TAG_LIDAR, "Setup is starting");

    uart_setup();

    ESP_LOGI(TAG_LIDAR, "Setup is done");
    
    char* start_lidar = "b";
    uart_write(start_lidar);

    ESP_LOGI(TAG_LIDAR, "%s, this to start sending data from lidar",start_lidar);

    rosidl_runtime_c__String__init(&(lidar_msg_->header.frame_id));
    lidar_msg_->header.frame_id = micro_ros_string_utilities_set(lidar_msg_->header.frame_id, "laser");

    //sets the lidar message
    lidar_msg_->ranges.capacity = 360;
    lidar_msg_->ranges.size = 360;
    lidar_msg_->ranges.data = (float*) malloc(lidar_msg_->ranges.capacity * sizeof(float));

    lidar_msg_->intensities.capacity = 360;
    lidar_msg_->intensities.size = 360;
    lidar_msg_->intensities.data = (float*) malloc(lidar_msg_->ranges.capacity * sizeof(float));
}