#include "imu_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ACCEL_ALL_FACES_MASK ((uint8_t)0x3FU)

static bool all_finite(const float values[3])
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!isfinite(values[i]))
            return false;
    }

    return true;
}

/*
 * Welford's online algorithm.
 * count must be incremented before this function is called.
 */
static void welford_update(uint32_t count,
                           const float sample[3],
                           float mean[3],
                           float m2[3])
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        const float delta = sample[i] - mean[i];
        mean[i] += delta / (float)count;

        const float delta2 = sample[i] - mean[i];
        m2[i] += delta * delta2;
    }
}

static void reset_gyro_accumulator(IMU_Calibration_t *cal)
{
    cal->gyro_sample_count = 0U;
    memset(cal->gyro_mean, 0, sizeof(cal->gyro_mean));
    memset(cal->gyro_m2, 0, sizeof(cal->gyro_m2));
}

static void reset_static_accel_accumulator(IMU_Calibration_t *cal)
{
    cal->static_accel_sample_count = 0U;
    memset(cal->static_accel_mean, 0, sizeof(cal->static_accel_mean));
    memset(cal->static_accel_m2, 0, sizeof(cal->static_accel_m2));
}

static void reset_face_accumulators(IMU_Calibration_t *cal)
{
    cal->accel_sample_count = 0U;
    memset(cal->accel_mean, 0, sizeof(cal->accel_mean));
    memset(cal->accel_m2, 0, sizeof(cal->accel_m2));

    cal->face_gyro_sample_count = 0U;
    memset(cal->face_gyro_mean, 0, sizeof(cal->face_gyro_mean));
    memset(cal->face_gyro_m2, 0, sizeof(cal->face_gyro_m2));

    cal->active_face_complete = false;
}

static void calibration_fail(IMU_Calibration_t *cal)
{
    cal->status = IMU_CAL_STATUS_ERROR;
    cal->stage = IMU_CAL_STAGE_ERROR;
}

static bool calculate_stddev(const float m2[3],
                             uint32_t sample_count,
                             float stddev[3])
{
    if (sample_count < 2U)
        return false;

    const float denominator = (float)(sample_count - 1U);

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        const float variance = m2[axis] / denominator;

        if (!isfinite(variance) || variance < 0.0f)
            return false;

        stddev[axis] = sqrtf(variance);

        if (!isfinite(stddev[axis]))
            return false;
    }

    return true;
}

static bool finalize_static_accelerometer(IMU_Calibration_t *cal)
{
    if (cal->static_accel_sample_count < 2U)
        return false;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        cal->result.static_accel_mean_g[axis] =
            cal->static_accel_mean[axis];
    }

    if (!calculate_stddev(cal->static_accel_m2,
                          cal->static_accel_sample_count,
                          cal->result.static_accel_noise_stddev_g))
    {
        return false;
    }

    cal->result.static_accel_sample_count =
        cal->static_accel_sample_count;

    return true;
}

static bool finalize_gyro(IMU_Calibration_t *cal)
{
    if (cal->gyro_sample_count < 2U)
        return false;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        cal->result.gyro_bias_dps[axis] = cal->gyro_mean[axis];
    }

    if (!calculate_stddev(cal->gyro_m2,
                          cal->gyro_sample_count,
                          cal->result.gyro_noise_stddev_dps))
    {
        return false;
    }

    cal->result.gyro_sample_count = cal->gyro_sample_count;
    return true;
}

static bool finalize_accelerometer(IMU_Calibration_t *cal)
{
    static const uint8_t positive_face[3] =
    {
        IMU_ACCEL_FACE_POS_X,
        IMU_ACCEL_FACE_POS_Y,
        IMU_ACCEL_FACE_POS_Z
    };

    static const uint8_t negative_face[3] =
    {
        IMU_ACCEL_FACE_NEG_X,
        IMU_ACCEL_FACE_NEG_Y,
        IMU_ACCEL_FACE_NEG_Z
    };

    const float min_span = 0.5f * cal->gravity_g;
    const uint32_t dof =
        (uint32_t)IMU_ACCEL_FACE_COUNT *
        (cal->accel_target_samples_per_face - 1U);

    if (dof == 0U)
        return false;

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        const float positive =
            cal->face_mean[positive_face[axis]][axis];
        const float negative =
            cal->face_mean[negative_face[axis]][axis];
        const float span = positive - negative;

        /* Positive face must read higher than negative face on this axis. */
        if (!isfinite(span) || span < min_span)
            return false;

        const float offset = 0.5f * (positive + negative);
        const float scale = (2.0f * cal->gravity_g) / span;

        /* Broad limits catch incorrect face selection, units, or captures. */
        if (!isfinite(offset) || !isfinite(scale) ||
            scale < 0.5f || scale > 2.0f ||
            fabsf(offset) > cal->gravity_g)
        {
            return false;
        }

        cal->result.accel_offset_g[axis] = offset;
        cal->result.accel_scale[axis] = scale;

        float total_m2 = 0.0f;

        for (uint8_t face = 0U;
             face < (uint8_t)IMU_ACCEL_FACE_COUNT;
             face++)
        {
            total_m2 += cal->face_m2[face][axis];
        }

        const float variance = total_m2 / (float)dof;

        if (!isfinite(variance) || variance < 0.0f)
            return false;

        cal->result.accel_noise_stddev_g[axis] = sqrtf(variance);

        if (!isfinite(cal->result.accel_noise_stddev_g[axis]))
            return false;
    }

    cal->result.accel_samples_per_face =
        cal->accel_target_samples_per_face;

    return true;
}

void IMU_Calibration_Init(IMU_Calibration_t *cal,
                          uint32_t static_samples,
                          uint32_t accel_samples_per_face,
                          float gravity_g)
{
    if (cal == NULL)
        return;

    memset(cal, 0, sizeof(*cal));

    cal->gyro_target_samples =
        (static_samples >= 2U)
        ? static_samples
        : IMU_CAL_DEFAULT_GYRO_SAMPLES;

    cal->accel_target_samples_per_face =
        (accel_samples_per_face >= 2U)
        ? accel_samples_per_face
        : IMU_CAL_DEFAULT_ACCEL_SAMPLES;

    cal->gravity_g =
        (isfinite(gravity_g) && gravity_g > 0.0f)
        ? gravity_g
        : IMU_CAL_GRAVITY_G;

    cal->active_face = IMU_ACCEL_FACE_COUNT;
    cal->status = IMU_CAL_STATUS_IDLE;
    cal->stage = IMU_CAL_STAGE_STATIC;
}

void IMU_Calibration_Start(IMU_Calibration_t *cal)
{
    if (cal == NULL)
        return;

    reset_gyro_accumulator(cal);
    reset_static_accel_accumulator(cal);
    reset_face_accumulators(cal);

    cal->completed_faces_mask = 0U;
    cal->active_face = IMU_ACCEL_FACE_COUNT;

    memset(cal->face_mean, 0, sizeof(cal->face_mean));
    memset(cal->face_m2, 0, sizeof(cal->face_m2));
    memset(&cal->result, 0, sizeof(cal->result));

    cal->status = IMU_CAL_STATUS_COLLECTING;
    cal->stage = IMU_CAL_STAGE_STATIC;
}

/* Collect gyro and accelerometer samples simultaneously while stationary. */
static bool update_static(IMU_Calibration_t *cal,
                          const float gyro_sample[3],
                          const float accel_sample[3])
{
    cal->gyro_sample_count++;
    welford_update(cal->gyro_sample_count,
                   gyro_sample,
                   cal->gyro_mean,
                   cal->gyro_m2);

    cal->static_accel_sample_count++;
    welford_update(cal->static_accel_sample_count,
                   accel_sample,
                   cal->static_accel_mean,
                   cal->static_accel_m2);

    if (cal->gyro_sample_count < cal->gyro_target_samples)
        return false;

    if (!finalize_static_accelerometer(cal))
    {
        calibration_fail(cal);
        return true;
    }

    /*
     * Static-stage measurements are retained in the global gyro accumulator.
     * The static accelerometer mean is diagnostic only; six faces determine
     * the actual per-axis accelerometer offset and scale.
     */
    cal->stage = IMU_CAL_STAGE_WAIT_ACCEL_FACE;
    return true;
}

bool IMU_Calibration_SetAccelFace(IMU_Calibration_t *cal,
                                  IMU_AccelCalibrationFace_t face)
{
    if (cal == NULL ||
        cal->status != IMU_CAL_STATUS_COLLECTING ||
        cal->stage != IMU_CAL_STAGE_WAIT_ACCEL_FACE ||
        (unsigned int)face >= (unsigned int)IMU_ACCEL_FACE_COUNT)
    {
        return false;
    }

    /* Do not allow a face to be captured twice. */
    if ((cal->completed_faces_mask &
         (uint8_t)(1U << (unsigned int)face)) != 0U)
    {
        return false;
    }

    cal->active_face = face;
    reset_face_accumulators(cal);
    cal->stage = IMU_CAL_STAGE_ACCEL_FACE;

    return true;
}

/*
 * Each face capture contributes to:
 *  - the global gyro accumulator (final bias/noise),
 *  - a per-face gyro accumulator (available for diagnostics),
 *  - the current face accelerometer accumulator.
 */
static bool update_face(IMU_Calibration_t *cal,
                        const float gyro_sample[3],
                        const float accel_sample[3])
{
    cal->gyro_sample_count++;
    welford_update(cal->gyro_sample_count,
                   gyro_sample,
                   cal->gyro_mean,
                   cal->gyro_m2);

    cal->face_gyro_sample_count++;
    welford_update(cal->face_gyro_sample_count,
                   gyro_sample,
                   cal->face_gyro_mean,
                   cal->face_gyro_m2);

    cal->accel_sample_count++;
    welford_update(cal->accel_sample_count,
                   accel_sample,
                   cal->accel_mean,
                   cal->accel_m2);

    if (cal->accel_sample_count <
        cal->accel_target_samples_per_face)
    {
        return false;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        cal->face_mean[cal->active_face][axis] =
            cal->accel_mean[axis];

        cal->face_m2[cal->active_face][axis] =
            cal->accel_m2[axis];
    }

    cal->completed_faces_mask |=
        (uint8_t)(1U << (unsigned int)cal->active_face);

    cal->active_face_complete = true;
    cal->active_face = IMU_ACCEL_FACE_COUNT;

    if (cal->completed_faces_mask == ACCEL_ALL_FACES_MASK)
    {
        if (!finalize_gyro(cal) || !finalize_accelerometer(cal))
        {
            calibration_fail(cal);
            return true;
        }

        cal->stage = IMU_CAL_STAGE_COMPLETE;
        cal->status = IMU_CAL_STATUS_COMPLETE;
        return true;
    }

    cal->stage = IMU_CAL_STAGE_WAIT_ACCEL_FACE;
    return true;
}

bool IMU_Calibration_Update(IMU_Calibration_t *cal,
                            float gx_dps, float gy_dps, float gz_dps,
                            float ax_g, float ay_g, float az_g)
{
    if (cal == NULL ||
        cal->status != IMU_CAL_STATUS_COLLECTING)
    {
        return false;
    }

    const float gyro_sample[3] =
    {
        gx_dps, gy_dps, gz_dps
    };

    const float accel_sample[3] =
    {
        ax_g, ay_g, az_g
    };

    if (!all_finite(gyro_sample) || !all_finite(accel_sample))
        return false;

    if (cal->stage == IMU_CAL_STAGE_STATIC)
    {
        return update_static(cal, gyro_sample, accel_sample);
    }

    if (cal->stage == IMU_CAL_STAGE_ACCEL_FACE)
    {
        return update_face(cal, gyro_sample, accel_sample);
    }

    /* No samples are accumulated while waiting for the next face selection. */
    return false;
}

IMU_CalibrationStatus_t
IMU_Calibration_GetStatus(const IMU_Calibration_t *cal)
{
    if (cal == NULL)
        return IMU_CAL_STATUS_ERROR;

    return cal->status;
}

IMU_CalibrationStage_t
IMU_Calibration_GetStage(const IMU_Calibration_t *cal)
{
    if (cal == NULL)
        return IMU_CAL_STAGE_ERROR;

    return cal->stage;
}

bool IMU_Calibration_GetResult(const IMU_Calibration_t *cal,
                               IMU_CalibrationResult_t *result)
{
    if (cal == NULL ||
        result == NULL ||
        cal->status != IMU_CAL_STATUS_COMPLETE)
    {
        return false;
    }

    *result = cal->result;
    return true;
}

void IMU_Calibration_Apply(const IMU_CalibrationResult_t *cal,
                           const float gyro_raw_dps[3],
                           const float accel_raw_g[3],
                           float gyro_corrected_dps[3],
                           float accel_corrected_g[3])
{
    if (cal == NULL ||
        gyro_raw_dps == NULL ||
        accel_raw_g == NULL ||
        gyro_corrected_dps == NULL ||
        accel_corrected_g == NULL)
    {
        return;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        gyro_corrected_dps[axis] =
            gyro_raw_dps[axis] - cal->gyro_bias_dps[axis];

        accel_corrected_g[axis] =
            (accel_raw_g[axis] - cal->accel_offset_g[axis]) *
            cal->accel_scale[axis];
    }
}
