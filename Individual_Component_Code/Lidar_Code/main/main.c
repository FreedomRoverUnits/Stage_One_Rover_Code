// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "our_lidar.h"

void app_main(void) {
    esp_log_level_set(Lidar_TAG, ESP_LOG_INFO);

    ESP_LOGI(Lidar_TAG, "Setup is starting");
     
    uart_setup();

    size_t uart_read_size;
    
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE_U2);
    ESP_LOGI(Lidar_TAG, "Setup is done");
     
    char* start_lidar = "b";
    char* stop_lidar = "e";
    int err_code_write_start_lidar, err_code_write_stop_lidar;
    //err_code_write_start_lidar = uart_write_bytes(uart_port_numb, (const char*)start_lidar, sizeof(start_lidar));
    //ESP_ERROR_CHECK(uart_wait_tx_done(uart_port_numb,100));
    err_code_write_start_lidar = uart_write(stop_lidar);
    
    ESP_LOGI(Lidar_TAG, "%s, this to start sending data from lidar, this is the size of 'b': %d",start_lidar, err_code_write_start_lidar);


    while(1) {
        
        //uart_read_size = uart_read_bytes(uart_port_numb, data, (BUF_SIZE_U2), 50); 
        uart_read_size = uart_read(data);
        //ESP_LOGI(Lidar_TAG, "the size of the bytes read: %d", size_of_uart0);
        
        if(uart_read_size > 0){
            sync[0] = data[0];
            sync[1] = data[1];
            //ESP_LOGI(Lidar_TAG, "Inside the 1st if statement.\n this is what is inside data[0]: %04x\n this is what is inside data[1]: %04x\n", data[0], data[1] );
             
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
            //ESP_LOGI(Lidar_TAG, "Inside the 2nd if statement\n");
             
        }  
        
        //Once frame captured, extract range/angle and convert to x/y:
        if (ready == 1) {
            //ESP_LOGI(Lidar_TAG, "Inside the 3rd if statement\n");
            //vTaskDelay(5);
            for (int i = 0; i < 2520; i = i + 42) {
                if (frame[i] == 0xFA && frame[(i + 1)] == 0xA0 + (i / 42)) {
                    //ESP_LOGI(Lidar_TAG, "Inside the frame calculation\n");
                    
                    for (int j = i + 4; j < i + 40; j = j + 6) {
                    int rangeA = frame[j + 2];
                    int rangeB = frame[j + 3];
                    int Degrees = 6 * (i / 42) + (j - 4 - i) / 6;
                    int range = (rangeB << 8) + rangeA;
                    if (Degrees != 0 && range != 0) {
                            float Radians = (Degrees * M_PI) / 180;
                            float x = range * cos(Radians);
                            float y = range * sin(Radians);
                            //ESP_LOGI(Lidar_TAG, "x,y: %f , %f \n", x,y);
                            //vTaskDelay(10);
                        }
                    }
                }
            }

            ready = 0;
            num_of_times_it_made_it++;
            //ESP_LOGI(Lidar_TAG, "Number of times inside the convers: %d\n", num_of_times_it_made_it);
            //ESP_LOGI(Lidar_TAG, "at the end of the 3rd if statement\n");
            vTaskDelay(1);
        }  
        else {
            sync[0] = 0;
            sync[1] = 0;
            //ESP_LOGI(Lidar_TAG, "Inside the else statement\n");
             
            did_not_work++;
            }

        if(did_not_work >= 5 )
        {
            did_not_work =0;
            num_reset++;
            //err_code_write_stop_lidar = uart_write_bytes(uart_port_numb, (const char*)stop_lidar, sizeof(stop_lidar));
            err_code_write_stop_lidar = uart_write(stop_lidar);
            ESP_LOGI(Lidar_TAG, "Number of resets of lidar: %d\n Current value of conversions: %d", num_reset, num_of_times_it_made_it);
            vTaskDelay(5);                      
            //err_code_write_start_lidar = uart_write_bytes(uart_port_numb, (const char*)start_lidar, sizeof(start_lidar));
            err_code_write_start_lidar = uart_write(start_lidar);
        }
            
         
        
    }
}