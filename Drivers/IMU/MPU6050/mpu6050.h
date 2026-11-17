#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* MPU6050 I2C address */
#define MPU6050_ADDR_LOW       0x68
#define MPU6050_ADDR_HIGH      0x69

#define MPU6050_WHO_AM_I_VAL   0x68

/* Register addresses */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_PIN_CFG  0x37
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_INT_STATUS   0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_DATA_LENGTH      14

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t temperature;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

} MPU6050_RawData_t;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float temperature_c;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

} MPU6050_Data_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;

    uint16_t dev_address;

    uint8_t buffer[MPU6050_DATA_LENGTH];

    volatile uint8_t transfer_busy;
    volatile uint8_t data_ready;
    volatile uint8_t error;

} MPU6050_Handle_t;

/* Initialization */
HAL_StatusTypeDef MPU6050_Init(
    MPU6050_Handle_t *imu,
    I2C_HandleTypeDef *hi2c,
    uint8_t address
);

/* Non-blocking DMA acquisition */
HAL_StatusTypeDef MPU6050_Read_DMA(MPU6050_Handle_t *imu);

/* Called from HAL I2C callbacks */
void MPU6050_RxCompleteCallback(MPU6050_Handle_t *imu);
void MPU6050_ErrorCallback(MPU6050_Handle_t *imu);

/* Convert received bytes into measurements */
void MPU6050_ParseData(
    const MPU6050_Handle_t *imu,
    MPU6050_RawData_t *raw,
    MPU6050_Data_t *data
);

#endif
