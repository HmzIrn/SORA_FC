#ifndef MEKF_H
#define MEKF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Multiplicative Extended Kalman Filter (attitude + gyro bias).
 * Ported from dash.py. Single precision throughout (STM32F401 FPU).
 *
 * CONVENTIONS
 *  - Body frame : FRD (x forward, y right, z down).
 *  - World frame: NED. Gravity = +z_w.
 *  - Quaternion : Hamilton, [w, x, y, z], rotates BODY -> WORLD.
 *  - Level and at rest, an FRD accelerometer reads [0, 0, -1 g].
 *  - Inputs are expected in BODY (FRD) axes: map the sensor frame to FRD
 *    before calling MEKF_Process().
 *  - Units: gyro in rad/s, accelerometer in g, dt in seconds.
 *    (IMU_Calibration works in deg/s: multiply by MEKF_DEG_TO_RAD.)
 *
 * Yaw is unobservable without a magnetometer and WILL DRIFT.
 * Noise parameters are starting values and must be tuned on hardware.
 */

#define MEKF_DEG_TO_RAD   (0.017453292519943295f)
#define MEKF_RAD_TO_DEG   (57.29577951308232f)

#define MEKF_STATE_DIM    (6U)   /* [dtheta(3), dbg(3)] */

typedef enum
{
    MEKF_STATE_CALIBRATING = 0,
    MEKF_STATE_RUNNING
} MEKF_State_t;

/* Result / status of the most recent call (replaces dash.py's msg strings). */
typedef enum
{
    MEKF_EVENT_NONE = 0,
    MEKF_EVENT_CALIBRATING,
    MEKF_EVENT_CALIBRATION_RESTART,           /* motion detected, restarting */
    MEKF_EVENT_RUNNING,
    MEKF_EVENT_RUNNING_CALIBRATED_WITH_MOTION,
    MEKF_EVENT_ACCEL_DOWNWEIGHTED,            /* update applied, R inflated  */
    MEKF_EVENT_ACCEL_INVALID,                 /* |a| ~ 0, update skipped     */
    MEKF_EVENT_ACCEL_REJECTED,                /* |a| far from gravity        */
    MEKF_EVENT_ACCEL_SINGULAR,                /* S not invertible            */
    MEKF_EVENT_ACCEL_INNOVATION_REJECTED,
    MEKF_EVENT_DATA_GAP,
    MEKF_EVENT_ATTITUDE_RESET,
    MEKF_EVENT_INVALID_SAMPLE,
    MEKF_EVENT_NUMERICAL_RESET
} MEKF_Event_t;

typedef struct
{
    float calib_duration_s;     /* stationary gyro-bias / gravity calibration time */
    float calib_max_gyro_std;   /* rad/s: more motion than this restarts calibration */
    float gyro_noise;           /* rad/s/sqrt(Hz), angular random walk              */
    float gyro_bias_rw;         /* rad/s/sqrt(s),  bias random walk                 */
    float accel_noise;          /* std of the unit gravity-direction measurement    */
    float init_att_std;         /* rad, after accelerometer initialisation          */
    float init_bias_std;        /* rad/s                                            */
    float gravity;              /* expected |a| in the units fed to the filter (g)  */
    float accel_soft_tol;       /* relative |a| error where R starts to inflate     */
    float accel_reject_tol;     /* relative |a| error above which update is skipped */
    float max_cov;              /* clamp for covariance diagonal                    */
} MEKF_Config_t;

/* Working state; initialize with MEKF_Init(). */
typedef struct
{
    MEKF_Config_t cfg;
    MEKF_State_t  state;
    MEKF_Event_t  event;

    float q[4];                              /* body -> world                */
    float bg[3];                             /* gyro bias [rad/s]            */
    float P[MEKF_STATE_DIM][MEKF_STATE_DIM]; /* error-state covariance       */
    float q_prev[4];                         /* sign-continuity reference    */

    uint32_t rejected;
    uint32_t updates;
    uint32_t recoveries;

    float last_acc[3];
    bool  has_last_acc;

    /* Stationary calibration accumulators. */
    bool     calib_bias_only;
    uint32_t calib_count;
    float    calib_elapsed_s;
    float    calib_sum_gyro[3];
    float    calib_sum_gyro_sq[3];
    float    calib_sum_acc[3];
    uint8_t  calib_tries;
} MEKF_t;

void MEKF_GetDefaultConfig(MEKF_Config_t *cfg);

/* cfg may be NULL (defaults). Invalid fields fall back to defaults.
 * Starts the built-in stationary calibration (same as MEKF_Reinitialize). */
void MEKF_Init(MEKF_t *mekf, const MEKF_Config_t *cfg);

/* Identity attitude, zero bias, restart full stationary calibration. */
void MEKF_Reinitialize(MEKF_t *mekf);

/*
 * Skip the built-in calibration (e.g. use IMU_Calibration results):
 * gyro bias in rad/s, accelerometer (already corrected, FRD) in g.
 * Attitude is levelled from the accelerometer with yaw = 0.
 * Returns false (state unchanged) on invalid input.
 */
bool MEKF_StartWithKnownBias(MEKF_t *mekf,
                             const float gyro_bias_rads[3],
                             const float accel_g[3]);

/* Re-estimate gyro bias only; attitude is kept. Vehicle must be stationary. */
void MEKF_RecalibrateBias(MEKF_t *mekf);

/* Re-level to the last accelerometer reading with yaw = 0; bias is kept. */
void MEKF_ResetAttitude(MEKF_t *mekf);

/*
 * Main entry, call once per IMU sample. gyro [rad/s], acc [g], FRD axes,
 * dt [s] > 0. Returns (and stores in mekf->event) what happened.
 */
MEKF_Event_t MEKF_Process(MEKF_t *mekf,
                          const float gyro_rads[3],
                          const float acc_g[3],
                          float dt_s);

/* Add process noise for a data gap instead of propagating on a stale dt. */
void MEKF_InflateCovariance(MEKF_t *mekf, float gap_s);

MEKF_State_t MEKF_GetState(const MEKF_t *mekf);
MEKF_Event_t MEKF_GetEvent(const MEKF_t *mekf);
bool         MEKF_IsRunning(const MEKF_t *mekf);

/* 0.0 .. 1.0 while calibrating, 1.0 when running. */
float MEKF_GetCalibrationProgress(const MEKF_t *mekf);

void MEKF_GetQuaternion(const MEKF_t *mekf, float q[4]);
void MEKF_GetGyroBias(const MEKF_t *mekf, float bg_rads[3]);

/* Aerospace ZYX Euler angles in radians (display / logging only). */
void MEKF_GetEuler(const MEKF_t *mekf,
                   float *roll_rad, float *pitch_rad, float *yaw_rad);

#ifdef __cplusplus
}
#endif

#endif /* MEKF_H */
