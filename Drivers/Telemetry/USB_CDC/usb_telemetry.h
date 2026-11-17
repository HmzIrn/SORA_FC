
#ifndef USB_TELEMETRY_H
#define USB_TELEMETRY_H

#include "mpu6050.h"
#include <stdint.h>
#include "mpu6050.h"
#include "imu_calibration.h"

#define TELEMETRY_INTERVAL_MS  10U  // 100 Hz

void USB_Telemetry_SendCalibrationStatus(
    IMU_CalibrationStatus_t status,
    IMU_CalibrationStage_t stage,
    uint8_t face_index
);

void USB_Telemetry_SendIMU(const MPU6050_Data_t *imu);

typedef struct
{
    uint32_t timestamp_ms;

    MPU6050_Data_t imu;

} Telemetry_Data_t;

/* Initialize telemetry module */
void USB_Telemetry_Init(void);

/* Attempt to transmit an IMU telemetry packet */
void USB_Telemetry_SendIMU(const MPU6050_Data_t *imu);

/* Send a complete telemetry snapshot */
void USB_Telemetry_Send(const Telemetry_Data_t *data);

#endif
