#pragma once
#include "main.h"
#define MPU6050_ADDR_LOW 0x68
typedef struct{float accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,temperature_c;}MPU6050_Data_t;
typedef struct{int x;}MPU6050_RawData_t;
typedef struct{I2C_HandleTypeDef*hi2c;volatile uint8_t transfer_busy,data_ready;}MPU6050_Handle_t;
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t*,I2C_HandleTypeDef*,uint8_t);
HAL_StatusTypeDef MPU6050_Read_DMA(MPU6050_Handle_t*);
void MPU6050_ParseData(MPU6050_Handle_t*,MPU6050_RawData_t*,MPU6050_Data_t*);
void MPU6050_RxCompleteCallback(MPU6050_Handle_t*);void MPU6050_ErrorCallback(MPU6050_Handle_t*);
