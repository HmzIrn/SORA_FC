#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CAL_GRAVITY_G                 (1.0f)
#define IMU_CAL_DEFAULT_GYRO_SAMPLES      (2000U)
#define IMU_CAL_DEFAULT_ACCEL_SAMPLES     (2000U)

/* Calibration lifecycle. */
typedef enum
{
    IMU_CAL_STATUS_IDLE = 0,
    IMU_CAL_STATUS_COLLECTING,
    IMU_CAL_STATUS_COMPLETE,
    IMU_CAL_STATUS_ERROR
} IMU_CalibrationStatus_t;

/* Current calibration stage. */
typedef enum
{
    IMU_CAL_STAGE_STATIC = 0,
    IMU_CAL_STAGE_WAIT_ACCEL_FACE,
    IMU_CAL_STAGE_ACCEL_FACE,
    IMU_CAL_STAGE_COMPLETE,
    IMU_CAL_STAGE_ERROR
} IMU_CalibrationStage_t;

/* Physical orientation presented during six-position calibration. */
typedef enum
{
    IMU_ACCEL_FACE_POS_X = 0,
    IMU_ACCEL_FACE_NEG_X,
    IMU_ACCEL_FACE_POS_Y,
    IMU_ACCEL_FACE_NEG_Y,
    IMU_ACCEL_FACE_POS_Z,
    IMU_ACCEL_FACE_NEG_Z,
    IMU_ACCEL_FACE_COUNT
} IMU_AccelCalibrationFace_t;

/* Final calibration values and useful measurement statistics. */
typedef struct
{
    /* Final gyro bias uses static-stage and all six face captures. */
    float gyro_bias_dps[3];
    float gyro_noise_stddev_dps[3];
    uint32_t gyro_sample_count;

    /*
     * Static-stage accelerometer statistics.
     * These are diagnostic values; they are NOT the final accel offsets.
     */
    float static_accel_mean_g[3];
    float static_accel_noise_stddev_g[3];
    uint32_t static_accel_sample_count;

    /* Six-position accelerometer calibration. */
    float accel_offset_g[3];
    float accel_scale[3];
    float accel_noise_stddev_g[3];
    uint32_t accel_samples_per_face;
} IMU_CalibrationResult_t;

/* Working state; initialize with IMU_Calibration_Init(). */
typedef struct
{
    IMU_CalibrationStatus_t status;
    IMU_CalibrationStage_t stage;

    uint32_t gyro_target_samples;
    uint32_t accel_target_samples_per_face;
    float gravity_g;

    /* Combined gyro Welford accumulator: static + all six face captures. */
    uint32_t gyro_sample_count;
    float gyro_mean[3];
    float gyro_m2[3];

    /* Static-stage accelerometer Welford accumulator. */
    uint32_t static_accel_sample_count;
    float static_accel_mean[3];
    float static_accel_m2[3];

    /* Current face accelerometer Welford accumulator. */
    uint32_t accel_sample_count;
    float accel_mean[3];
    float accel_m2[3];

    /* Current face gyro statistics, retained for per-face diagnostics. */
    uint32_t face_gyro_sample_count;
    float face_gyro_mean[3];
    float face_gyro_m2[3];

    /* Per-face accelerometer means and M2 values. */
    float face_mean[IMU_ACCEL_FACE_COUNT][3];
    float face_m2[IMU_ACCEL_FACE_COUNT][3];

    uint8_t completed_faces_mask;
    IMU_AccelCalibrationFace_t active_face;
    bool active_face_complete;

    IMU_CalibrationResult_t result;
} IMU_Calibration_t;

void IMU_Calibration_Init(IMU_Calibration_t *cal,
                          uint32_t static_samples,
                          uint32_t accel_samples_per_face,
                          float gravity_g);

void IMU_Calibration_Start(IMU_Calibration_t *cal);

bool IMU_Calibration_SetAccelFace(IMU_Calibration_t *cal,
                                  IMU_AccelCalibrationFace_t face);

bool IMU_Calibration_Update(IMU_Calibration_t *cal,
                            float gx_dps, float gy_dps, float gz_dps,
                            float ax_g, float ay_g, float az_g);

IMU_CalibrationStatus_t
IMU_Calibration_GetStatus(const IMU_Calibration_t *cal);

IMU_CalibrationStage_t
IMU_Calibration_GetStage(const IMU_Calibration_t *cal);

bool IMU_Calibration_GetResult(const IMU_Calibration_t *cal,
                               IMU_CalibrationResult_t *result);

void IMU_Calibration_Apply(const IMU_CalibrationResult_t *cal,
                           const float gyro_raw_dps[3],
                           const float accel_raw_g[3],
                           float gyro_corrected_dps[3],
                           float accel_corrected_g[3]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_H */
