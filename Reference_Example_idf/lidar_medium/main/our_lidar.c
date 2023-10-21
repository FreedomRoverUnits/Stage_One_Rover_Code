///code from Matthew Hogan link: https://medium.com/@Matthew_Hogan/interesting-electronic-components-1-hls-lfcd2-640d897f9fc9
//refactored by Brittney, Misi and Jordy
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "UART Lidar test";
//ESP_LOGI(TAG, "Setup is starting");


//byte Frame [2520];
#define BUF_SIZE_U2 (2520)
unsigned char sync[2];
unsigned char frame[BUF_SIZE_U2]; //TODO we can get rid of this :)
//byte Sync[2];
int ready = 0;
int num_of_times_it_made_it = 0;
int did_not_work=0;
int num_reset = 0;

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Setup is starting");
    vTaskDelay(10);

    const uart_port_t uart_port_numb = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_LOGI(TAG, "created uart config");
    vTaskDelay(10);
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (2520); //TODO: Change this
    ESP_LOGI(TAG, "created uart buffer size");
    vTaskDelay(10);

    // QueueHandle_t uart_queue;
    // ESP_LOGI(TAG, "created uart queue");
    // vTaskDelay(10);

    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "created intr alloc flag variable");
    vTaskDelay(10);    

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_port_numb, uart_buffer_size*2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_set_mode(uart_port_numb,UART_MODE_UART));
    ESP_LOGI(TAG, "Uart driver install");
    vTaskDelay(10); 
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_port_numb, &uart_config));
    //uart_param_config(uart_port_numb, &uart_config);

    ESP_LOGI(TAG, "Setup config");
    vTaskDelay(10);

    // Set UART pins(TX: IO10, RX: IO9, RTS: -1, CTS: -1)
    ESP_ERROR_CHECK(uart_set_pin(uart_port_numb, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "Setup pins");
    vTaskDelay(10);

    

    size_t size_of_uart0;
    

  //Serial.begin(9600);
//   Serial1.begin(230400);
//   Serial1.write('b');
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE_U2);
    ESP_LOGI(TAG, "Setup is done");
    vTaskDelay(10);
    char* test_str = "b";
    char* test_str_end = "e";
    int error_tx, error_tx_end;
    error_tx = uart_write_bytes(uart_port_numb, (const char*)test_str, sizeof(test_str));
    ESP_ERROR_CHECK(uart_wait_tx_done(uart_port_numb,100));
    ESP_LOGI(TAG, "%s, is there an error with sending data : %d",test_str, error_tx);


    while(1) {
        // if (Serial.available()) {
        //     Serial1.write(Serial.read());
        // }  
        // ESP_LOGI(TAG, "Number of times inside the convers: %d\n", num_of_times_it_made_it);
        //vTaskDelay(1);
        
        //ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port_numb, &size_of_uart0));

        
        size_of_uart0 = uart_read_bytes(uart_port_numb, data, (BUF_SIZE_U2), 50); //TODO: change 100 to something else
        //ESP_LOGI(TAG, "the size of the bytes read: %d", size_of_uart0);
        
        if(size_of_uart0 > 0){
            sync[0] = data[0];
            // if(sync[0] == 0xFA) {
            //     while (size_of_uart0 <= 1) { //TODO do we need this?
            //     size_of_uart0 = uart_read_bytes(uart_port_numb, data, size_of_uart0, 100); //TODO: change 100 to something else   
            //     }
            // }
            sync[1] = data[1];
            //ESP_LOGI(TAG, "Inside the 1st if statement.\n this is what is inside data[0]: %04x\n this is what is inside data[1]: %04x\n", data[0], data[1] );
            //vTaskDelay(10);
        }
        //First capture the start byte of a frame:
          //Once start byte captured, read remaining frame into array:
        if (sync[0] == 0xFA && sync[1] == 0xA0) {
            frame[0] = 0xFA;
            frame[1] = 0xA0;
            for (int v = 2; v <= 2520; v++) {
            // while (!Serial1.available()) {
            //     Serial1.write(Serial.read());
            // }
                frame[v] = data[v]; //copying the data from data array to frame
            }
            ready = 1;
            //ESP_LOGI(TAG, "Inside the 2nd if statement\n");
            //vTaskDelay(10);
        }  //Once frame captured, extract range/angle and convert to x/y:
        if (ready == 1) {
            //ESP_LOGI(TAG, "Inside the 3rd if statement\n");
            //vTaskDelay(5);
            for (int i = 0; i < 2520; i = i + 42) {
            if (frame[i] == 0xFA && frame[(i + 1)] == 0xA0 + (i / 42)) {
                //ESP_LOGI(TAG, "Inside the frame calculation\n");
                
                for (int j = i + 4; j < i + 40; j = j + 6) {
                int rangeA = frame[j + 2];
                int rangeB = frame[j + 3];
                int Degrees = 6 * (i / 42) + (j - 4 - i) / 6;
                int range = (rangeB << 8) + rangeA;
                if (Degrees != 0 && range != 0) {
                    float Radians = (Degrees * M_PI) / 180;
                    float x = range * cos(Radians);
                    float y = range * sin(Radians);
                    //printf("%f , %f \n", x, y);
                    //ESP_LOGI(TAG, "x,y: %f , %f \n", x,y);
                    //vTaskDelay(10);

                }
                }
            }
            }
            ready = 0;
            num_of_times_it_made_it++;
            //ESP_LOGI(TAG, "Number of times inside the convers: %d\n", num_of_times_it_made_it);
            
            //ESP_LOGI(TAG, "at the end of the 3rd if statement\n");
            //vTaskDelay(10);
        }  
        else {
            sync[0] = 0;
            sync[1] = 0;
            //ESP_LOGI(TAG, "Inside the else statement\n");
            //vTaskDelay(10);
            did_not_work++;
            }

        if(did_not_work >= 5 )
        {
            did_not_work =0;
            num_reset++;
            error_tx_end = uart_write_bytes(uart_port_numb, (const char*)test_str_end, sizeof(test_str_end));
            ESP_LOGI(TAG, "Number of resets of lidar: %d\n Current value of conversions: %d", num_reset, num_of_times_it_made_it);
            vTaskDelay(5);
            error_tx = uart_write_bytes(uart_port_numb, (const char*)test_str, sizeof(test_str));
            
        }
            
        //vTaskDelay(10);
        
    }
}