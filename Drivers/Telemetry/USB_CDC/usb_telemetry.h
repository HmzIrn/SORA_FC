#ifndef USB_TELEMETRY_H
#define USB_TELEMETRY_H

#include <stdint.h>

#include "mpu6050.h"
#include "imu_calibration.h"
#include "mekf.h"
#include "motor_mixer.h"

#define TELEMETRY_INTERVAL_MS  10U  // 100 Hz

/*
 * Complete telemetry snapshot: calibrated IMU, attitude filter and the
 * computed motor response.
 */
typedef struct
{
    uint32_t timestamp_ms;

    /* Calibrated IMU measurement (sensor frame). */
    MPU6050_Data_t imu;

    /* Attitude filter (FRD body, NED world). */
    MEKF_State_t filter_state;
    MEKF_Event_t filter_event;
    float q[4];                 /* body -> world, [w,x,y,z]              */
    float euler_rad[3];         /* roll, pitch, yaw                      */
    float gyro_bias_rads[3];    /* residual bias estimated by the filter */

    /* Motor mixer output (computed only, not necessarily applied). */
    MotorMixer_Output_t motors;
} Telemetry_Data_t;

/* Initialize telemetry module */
void USB_Telemetry_Init(void);

void USB_Telemetry_SendCalibrationStatus(
    IMU_CalibrationStatus_t status,
    IMU_CalibrationStage_t stage,
    uint8_t face_index
);

/* Transmit a DATA line only (IMU). Kept for IMU-only use. */
void USB_Telemetry_SendIMU(const MPU6050_Data_t *imu);

/*
 * Transmit one packet containing two lines in a single USB transfer:
 *
 *   DATA,tick,Ax,Ay,Az,Gx,Gy,Gz,Temp           (unchanged, dash.py compatible)
 *   FC,<fields below>
 *
 * FC fields, in order (all integers, no floating-point printf needed):
 *   1  tick [ms]
 *   2  filter state       0 = CALIBRATING, 1 = RUNNING
 *   3  filter event       MEKF_Event_t
 *   4  roll   [mdeg]
 *   5  pitch  [mdeg]
 *   6  yaw    [mdeg]
 *   7  qw     [1e-4]
 *   8  qx     [1e-4]
 *   9  qy     [1e-4]
 *   10 qz     [1e-4]
 *   11 gyro bias x [mdps]
 *   12 gyro bias y [mdps]
 *   13 gyro bias z [mdps]
 *   14 mixer status       MotorMixer_Status_t
 *   15 mixer flags        bit0 = active, bit1 = saturated
 *   16 thrust M1 [permille of max thrust]   (M1 FR, CCW)
 *   17 thrust M2 [permille]                 (M2 RL, CCW)
 *   18 thrust M3 [permille]                 (M3 FL, CW)
 *   19 thrust M4 [permille]                 (M4 RR, CW)
 *   20 pwm M1 [us equivalent, 1000..2000]
 *   21 pwm M2
 *   22 pwm M3
 *   23 pwm M4
 *   24 collective [permille]
 *   25 roll  torque command [permille]
 *   26 pitch torque command [permille]
 *   27 yaw   torque command [permille]
 *
 * If the USB endpoint is still busy with the previous transfer the packet
 * is dropped (returns 0). Returns 1 when queued.
 */
uint8_t USB_Telemetry_Send(const Telemetry_Data_t *data);

#endif
