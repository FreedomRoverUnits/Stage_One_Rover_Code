#include <stdio.h>
#include "OUR_IMU.h"
//#include "esp_timer.h"

//float elaspedTime, previousTime;
//float currentTime = 0.0;

float elaspedTime = 0.0;

void app_main(void)
{
    float main_yaw, main_pitch, main_roll;
    main_yaw = 0.0;
    main_pitch = 0.0;
    main_roll = 0.0;
    
    uint8_t data[2];
    ESP_ERROR_CHECK(our_i2c_master_init());
    ESP_LOGI(TAG_IMU, "I2C initialized successfully");

    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 0));

    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG_IMU, "WHO_AM_I = %X", data[0]);

    calculate_IMU_error();

    
    while(1){
        //previousTime = currentTime;
        //currentTime = (float)(esp_timer_get_time());
        //elaspedTime = (currentTime - previousTime) / 1000000;
        calculate_Angles(elaspedTime, &main_roll, &main_pitch, &main_yaw);

        ESP_LOGI(TAG_IMU, "Roll: %f Pitch: %f Yaw: %f", main_roll, main_pitch, main_yaw);
        //vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}
