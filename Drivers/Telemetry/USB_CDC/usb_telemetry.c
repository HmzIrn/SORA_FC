
#include "usb_telemetry.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdint.h>

void USB_Telemetry_Init(void)
{
    // USB initialization is handled by MX_USB_DEVICE_Init().
}

void USB_Telemetry_SendCalibrationStatus(
    IMU_CalibrationStatus_t status,
    IMU_CalibrationStage_t stage,
    uint8_t face_index)
{
    static uint8_t previous_valid = 0;
    static IMU_CalibrationStatus_t previous_status;
    static IMU_CalibrationStage_t previous_stage;
    static uint8_t previous_face;

    char buffer[128];
    const char *message = NULL;
    int len;

    static const char *face_names[IMU_ACCEL_FACE_COUNT] =
    {
        "POS_X", "NEG_X", "POS_Y",
        "NEG_Y", "POS_Z", "NEG_Z"
    };

    const char *face = (face_index < IMU_ACCEL_FACE_COUNT)
                       ? face_names[face_index]
                       : "UNKNOWN";

    if (previous_valid &&
        status == previous_status &&
        stage == previous_stage &&
        face_index == previous_face)
    {
        return;
    }

    switch (status)
    {
        case IMU_CAL_STATUS_IDLE:
            message = "CAL,IDLE,Calibration not started\r\n";
            break;

        case IMU_CAL_STATUS_COMPLETE:
            message = "CAL,COMPLETE,Calibration successful\r\n";
            break;

        case IMU_CAL_STATUS_ERROR:
            message = "CAL,ERROR,Calibration failed\r\n";
            break;

        case IMU_CAL_STATUS_COLLECTING:
            switch (stage)
            {
                case IMU_CAL_STAGE_STATIC:
                    message =
                        "[CAL:GYRO]: Keep the board completely stationary\r\n";
                    break;

                case IMU_CAL_STAGE_WAIT_ACCEL_FACE:
                    len = snprintf(
                        buffer, sizeof(buffer),
                        "[CAL:WAIT_FACE]: %s,Orient this face upward and press the button\r\n",
                        face
                    );
                    message = NULL;
                    break;

                case IMU_CAL_STAGE_ACCEL_FACE:
                    len = snprintf(
                        buffer, sizeof(buffer),
                        "[CAL:CAPTURING]: %s,Keep the board stationary\r\n",
                        face
                    );
                    message = NULL;
                    break;

                default:
                    message = "[CAL:UNKNOWN]: Check calibration state\r\n";
                    break;
            }
            break;

        default:
            message = "[CAL:UNKNOWN]: Check calibration state\r\n";
            break;
    }

    if (message != NULL)
    {
        len = snprintf(buffer, sizeof(buffer), "%s", message);
    }

    if (len <= 0 || len >= (int)sizeof(buffer))
        return;

    if (CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)len) == USBD_OK)
    {
        previous_status = status;
        previous_stage = stage;
        previous_face = face_index;
        previous_valid = 1;
    }
}

void USB_Telemetry_SendIMU(const MPU6050_Data_t *imu)
{
    char buffer[128];

    if (imu == NULL)
        return;

    int32_t ax = (int32_t)(imu->accel_x_g * 1000.0f);
    int32_t ay = (int32_t)(imu->accel_y_g * 1000.0f);
    int32_t az = (int32_t)(imu->accel_z_g * 1000.0f);

    int32_t gx = (int32_t)(imu->gyro_x_dps * 1000.0f);
    int32_t gy = (int32_t)(imu->gyro_y_dps * 1000.0f);
    int32_t gz = (int32_t)(imu->gyro_z_dps * 1000.0f);

    int32_t temp = (int32_t)(imu->temperature_c * 1.0f);

    int len = snprintf(
        buffer,
        sizeof(buffer),
        "DATA,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)HAL_GetTick(),
        (long)ax, (long)ay, (long)az,
        (long)gx, (long)gy, (long)gz,
        (long)temp
    );

    if (len <= 0 || len >= (int)sizeof(buffer))
        return;

    CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)len);
}
