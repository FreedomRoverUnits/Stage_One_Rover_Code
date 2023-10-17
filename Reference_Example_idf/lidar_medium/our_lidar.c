///code from Matthew Hogan link: https://medium.com/@Matthew_Hogan/interesting-electronic-components-1-hls-lfcd2-640d897f9fc9
//refactored by Brittney, Misi and Jordy
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"


//byte Frame [2520];
#define BUF_SIZE (1024)
uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

byte Sync[2];
int Ready = 0;

void app_main(void) {
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins(TX: IO1, RX: IO3, RTS: -1, CTS: -1)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024); //TODO: Change this
    QueueHandle_t uart_queue;
    int intr_alloc_flags = 0;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, intr_alloc_flags));

    size_t size_of_uart0;
  //Serial.begin(9600);
  //Serial1.begin(230400);
  //Serial1.write('b');

    while(1) {
        // if (Serial.available()) {
        //     //Serial1.write(Serial.read());
        // }  
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, &size_of_uart0));

        if(size_of_uart0 > 0){
            size_of_uart0 = uart_read_bytes(uart_num, data, size_of_uart0, 100); //TODO: change 100 to something else
            //need to added Serial1.write
        }



        //First capture the start byte of a frame:
        if (//Serial1.available()) {
            Sync[0] = //Serial1.read();
            if (Sync[0] == 0xFA) {
            while (!//Serial1.available()) {
                //Serial1.write(Serial.read());
            }
            Sync[1] = //Serial1.read();
            }
        }  //Once start byte captured, read remaining frame into array:
        if (Sync[0] == 0xFA && Sync [1] == 0xA0) {
            Frame[0] = 0xFA;
            Frame[1] = 0xA0;
            for (int v = 2; v <= 2520; v++) {
            while (!//Serial1.available()) {
                //Serial1.write(Serial.read());
            }
            Frame[v] = //Serial1.read();
            }
            Ready = 1;
        }  //Once frame captured, extract range/angle and convert to x/y:
        if (Ready == 1) {
            for (int i = 0; i < 2520; i = i + 42) {
            if (Frame[i] == 0xFA && Frame[i + 1] == 0xA0 + (i / 42)) {
                for (int j = i + 4; j < i + 40; j = j + 6) {
                int rangeA = Frame[j + 2];
                int rangeB = Frame[j + 3];
                int Degrees = 6 * (i / 42) + (j - 4 - i) / 6;
                int range = (rangeB << 8) + rangeA;
                if (Degrees != 0 && range != 0) {
                    float Radians = (Degrees * PI) / 180;
                    float x = range * cos(Radians);
                    float y = range * sin(Radians);
                    Serial.print(x);
                    Serial.print(",");
                    Serial.println(y);
                }
                }
            }
            }
            Ready = 0;
        }  
        else {
            Sync[0] = 0;
            Sync [1] = 0;
            }
    }
}