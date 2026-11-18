#include "usb_telemetry.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>

/*
 * CDC_Transmit_FS() does not copy the buffer: the USB peripheral keeps
 * reading it after the call returns (multi-packet transfers are fed from
 * the FIFO-empty interrupt). The buffer must therefore be static and must
 * not be rewritten while a transfer is in flight, so every sender checks
 * cdc_tx_idle() BEFORE formatting into it.
 */
#define TX_BUFFER_SIZE  384U

static char tx_buffer[TX_BUFFER_SIZE];

static uint8_t cdc_tx_idle(void)
{
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    return (hcdc != NULL) && (hcdc->TxState == 0U);
}

/* Round-to-nearest conversion to a scaled integer; non-finite -> 0. */
static int32_t scaled(float value, float scale)
{
    const float v = value * scale;

    if (!isfinite(v))
        return 0;

    return (int32_t)(v + ((v >= 0.0f) ? 0.5f : -0.5f));
}

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

    const char *message = NULL;
    int len = 0;

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

    /* Retry on a later call if the previous transfer is still running. */
    if (!cdc_tx_idle())
        return;

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
                        tx_buffer, sizeof(tx_buffer),
                        "[CAL:WAIT_FACE]: %s,Orient this face upward and press the button\r\n",
                        face
                    );
                    break;

                case IMU_CAL_STAGE_ACCEL_FACE:
                    len = snprintf(
                        tx_buffer, sizeof(tx_buffer),
                        "[CAL:CAPTURING]: %s,Keep the board stationary\r\n",
                        face
                    );
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
        len = snprintf(tx_buffer, sizeof(tx_buffer), "%s", message);
    }

    if (len <= 0 || len >= (int)sizeof(tx_buffer))
        return;

    if (CDC_Transmit_FS((uint8_t *)tx_buffer, (uint16_t)len) == USBD_OK)
    {
        previous_status = status;
        previous_stage = stage;
        previous_face = face_index;
        previous_valid = 1;
    }
}

/* Format the DATA line (unchanged wire format) at buf; returns length or <0. */
static int format_data_line(char *buf, size_t size,
                            uint32_t tick, const MPU6050_Data_t *imu)
{
    const int32_t ax = (int32_t)(imu->accel_x_g * 1000.0f);
    const int32_t ay = (int32_t)(imu->accel_y_g * 1000.0f);
    const int32_t az = (int32_t)(imu->accel_z_g * 1000.0f);

    const int32_t gx = (int32_t)(imu->gyro_x_dps * 1000.0f);
    const int32_t gy = (int32_t)(imu->gyro_y_dps * 1000.0f);
    const int32_t gz = (int32_t)(imu->gyro_z_dps * 1000.0f);

    const int32_t temp = (int32_t)(imu->temperature_c * 1.0f);

    return snprintf(
        buf,
        size,
        "DATA,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)tick,
        (long)ax, (long)ay, (long)az,
        (long)gx, (long)gy, (long)gz,
        (long)temp
    );
}

void USB_Telemetry_SendIMU(const MPU6050_Data_t *imu)
{
    if (imu == NULL || !cdc_tx_idle())
        return;

    const int len = format_data_line(tx_buffer, sizeof(tx_buffer),
                                     HAL_GetTick(), imu);

    if (len <= 0 || len >= (int)sizeof(tx_buffer))
        return;

    CDC_Transmit_FS((uint8_t *)tx_buffer, (uint16_t)len);
}

uint8_t USB_Telemetry_Send(const Telemetry_Data_t *data)
{
    if (data == NULL || !cdc_tx_idle())
        return 0U;

    int len = format_data_line(tx_buffer, sizeof(tx_buffer),
                               data->timestamp_ms, &data->imu);

    if (len <= 0 || len >= (int)sizeof(tx_buffer))
        return 0U;

    const MotorMixer_Output_t *m = &data->motors;

    const unsigned int flags =
        (m->active ? 1U : 0U) | (m->saturated ? 2U : 0U);

    const int len2 = snprintf(
        &tx_buffer[len],
        sizeof(tx_buffer) - (size_t)len,
        "FC,%lu,%u,%u,"
        "%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,"
        "%u,%u,"
        "%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,"
        "%ld,"
        "%ld,%ld,%ld\r\n",
        (unsigned long)data->timestamp_ms,
        (unsigned int)data->filter_state,
        (unsigned int)data->filter_event,
        /* roll, pitch, yaw [mdeg] */
        (long)scaled(data->euler_rad[0], MEKF_RAD_TO_DEG * 1000.0f),
        (long)scaled(data->euler_rad[1], MEKF_RAD_TO_DEG * 1000.0f),
        (long)scaled(data->euler_rad[2], MEKF_RAD_TO_DEG * 1000.0f),
        /* quaternion [1e-4] */
        (long)scaled(data->q[0], 10000.0f),
        (long)scaled(data->q[1], 10000.0f),
        (long)scaled(data->q[2], 10000.0f),
        (long)scaled(data->q[3], 10000.0f),
        /* gyro bias [mdps] */
        (long)scaled(data->gyro_bias_rads[0], MEKF_RAD_TO_DEG * 1000.0f),
        (long)scaled(data->gyro_bias_rads[1], MEKF_RAD_TO_DEG * 1000.0f),
        (long)scaled(data->gyro_bias_rads[2], MEKF_RAD_TO_DEG * 1000.0f),
        /* mixer status / flags */
        (unsigned int)m->status,
        flags,
        /* thrust [permille] */
        (long)scaled(m->thrust[0], 1000.0f),
        (long)scaled(m->thrust[1], 1000.0f),
        (long)scaled(m->thrust[2], 1000.0f),
        (long)scaled(m->thrust[3], 1000.0f),
        /* pwm [us] */
        (long)scaled(m->pwm_us[0], 1.0f),
        (long)scaled(m->pwm_us[1], 1.0f),
        (long)scaled(m->pwm_us[2], 1.0f),
        (long)scaled(m->pwm_us[3], 1.0f),
        /* collective, torque commands [permille] */
        (long)scaled(m->collective, 1000.0f),
        (long)scaled(m->torque[0], 1000.0f),
        (long)scaled(m->torque[1], 1000.0f),
        (long)scaled(m->torque[2], 1000.0f)
    );

    if (len2 <= 0 || (len + len2) >= (int)sizeof(tx_buffer))
        return 0U;

    return (CDC_Transmit_FS((uint8_t *)tx_buffer,
                            (uint16_t)(len + len2)) == USBD_OK) ? 1U : 0U;
}
