#ifndef OUR_IMU_H
#define OUR_IMU_H

#include "esp_log.h"
#include "driver/i2c.h"
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

static const char *TAG_IMU = "IMU_I2C";

#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_ACCEL_XOUT                  0x3B
#define MPU6050_ACCEL_YOUT                  0x3D
#define MPU6050_ACCEL_ZOUT                  0x3F
#define MPU6050_GYRO_XOUT                   0x43
#define MPU6050_GYRO_YOUT                   0x45
#define MPU6050_GYRO_ZOUT                   0x47

esp_err_t our_i2c_master_init(void);
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data);
void calculate_IMU_error(void);
void calculate_Angles(int64_t elapsed_time, float* ret_roll, float* ret_pitch, float* ret_yaw);
geometry_msgs__msg__Vector3 readAccelerometer();
geometry_msgs__msg__Vector3 readGyroscope();
sensor_msgs__msg__Imu getIMUData();
void setup_imu(void);
#endif