#include "mekf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define N MEKF_STATE_DIM

/* Defaults (match dash.py MEKFConfig). */
#define DEF_CALIB_DURATION_S    (2.0f)
#define DEF_CALIB_MAX_GYRO_STD  (0.15f)
#define DEF_GYRO_NOISE          (0.01f)
#define DEF_GYRO_BIAS_RW        (1e-4f)
#define DEF_ACCEL_NOISE         (0.05f)
#define DEF_INIT_ATT_STD        (0.05f)
#define DEF_INIT_BIAS_STD       (0.01f)
#define DEF_GRAVITY             (1.0f)
#define DEF_ACCEL_SOFT_TOL      (0.05f)
#define DEF_ACCEL_REJECT_TOL    (0.25f)
#define DEF_MAX_COV             (10.0f)

#define CALIB_MAX_RESTARTS      (3U)
#define MAX_GAP_S               (0.5f)
#define MAX_INNOVATION_RAD      (0.8f)
#define MIN_COV_DIAG            (1e-12f)

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static bool all_finite(const float *v, uint8_t n)
{
    for (uint8_t i = 0U; i < n; i++)
    {
        if (!isfinite(v[i]))
            return false;
    }

    return true;
}

static bool positive_finite(float v)
{
    return isfinite(v) && v > 0.0f;
}

static void quat_identity(float q[4])
{
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
}

static void quat_normalize(float q[4])
{
    const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

    if (!isfinite(n2) || n2 < 1e-24f)
    {
        quat_identity(q);
        return;
    }

    const float inv = 1.0f / sqrtf(n2);

    for (uint8_t i = 0U; i < 4U; i++)
        q[i] *= inv;
}

/* out = a (x) b. out must not alias a or b. */
static void quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

/* Exponential map: rotation vector [rad] -> unit quaternion. */
static void quat_from_rotvec(const float v[3], float q[4])
{
    const float th2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

    if (th2 < 1e-12f)
    {
        q[0] = 1.0f;
        q[1] = 0.5f * v[0];
        q[2] = 0.5f * v[1];
        q[3] = 0.5f * v[2];
        quat_normalize(q);
        return;
    }

    const float th = sqrtf(th2);
    const float s = sinf(0.5f * th) / th;

    q[0] = cosf(0.5f * th);
    q[1] = s * v[0];
    q[2] = s * v[1];
    q[3] = s * v[2];
}

/* Body -> world rotation matrix of a unit quaternion. */
static void quat_to_rot(const float q[4], float R[3][3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];

    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] = 2.0f * (x * y - w * z);
    R[0][2] = 2.0f * (x * z + w * y);
    R[1][0] = 2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] = 2.0f * (y * z - w * x);
    R[2][0] = 2.0f * (x * z - w * y);
    R[2][1] = 2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static void skew(const float v[3], float S[3][3])
{
    S[0][0] = 0.0f;   S[0][1] = -v[2];  S[0][2] = v[1];
    S[1][0] = v[2];   S[1][1] = 0.0f;   S[1][2] = -v[0];
    S[2][0] = -v[1];  S[2][1] = v[0];   S[2][2] = 0.0f;
}

/*
 * Rodrigues: rotation vector -> rotation matrix.
 * Series for small angles (also avoids 1 - cos(x) cancellation in float).
 */
static void rotvec_to_rot(const float v[3], float R[3][3])
{
    const float th2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    float K[3][3];
    float a, b;

    skew(v, K);

    if (th2 < 1e-3f)
    {
        a = 1.0f - th2 / 6.0f + th2 * th2 / 120.0f;   /* sin(th)/th       */
        b = 0.5f - th2 / 24.0f + th2 * th2 / 720.0f;  /* (1-cos(th))/th^2 */
    }
    else
    {
        const float th = sqrtf(th2);
        a = sinf(th) / th;
        b = (1.0f - cosf(th)) / th2;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            float kk = 0.0f;

            for (uint8_t k = 0U; k < 3U; k++)
                kk += K[i][k] * K[k][j];

            R[i][j] = ((i == j) ? 1.0f : 0.0f) + a * K[i][j] + b * kk;
        }
    }
}

static void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
    const float cr = cosf(0.5f * roll),  sr = sinf(0.5f * roll);
    const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
    const float cy = cosf(0.5f * yaw),   sy = sinf(0.5f * yaw);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    quat_normalize(q);
}

/* Level attitude (yaw = 0) from a specific-force vector in FRD body axes. */
static void quat_from_accel(const float a_in[3], float q[4])
{
    const float n2 = a_in[0] * a_in[0] + a_in[1] * a_in[1] + a_in[2] * a_in[2];

    if (!isfinite(n2) || n2 < 1e-18f)
    {
        quat_identity(q);
        return;
    }

    const float inv = 1.0f / sqrtf(n2);
    const float ax = a_in[0] * inv;
    const float ay = a_in[1] * inv;
    const float az = a_in[2] * inv;

    float sp = ax;
    if (sp > 1.0f)  sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;

    euler_to_quat(atan2f(-ay, -az), asinf(sp), 0.0f, q);
}

/* ------------------------------------------------------------------------ */
/* 6x6 matrix helpers                                                       */
/* ------------------------------------------------------------------------ */

/* C = A * B. C must not alias A or B. */
static void mat6_mul(float A[N][N], float B[N][N], float C[N][N])
{
    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < N; j++)
        {
            float sum = 0.0f;

            for (uint8_t k = 0U; k < N; k++)
                sum += A[i][k] * B[k][j];

            C[i][j] = sum;
        }
    }
}

/* C = A * B^T. C must not alias A or B. */
static void mat6_mul_bt(float A[N][N], float B[N][N], float C[N][N])
{
    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < N; j++)
        {
            float sum = 0.0f;

            for (uint8_t k = 0U; k < N; k++)
                sum += A[i][k] * B[j][k];

            C[i][j] = sum;
        }
    }
}

/* P <- M P M^T */
static void congruence(float P[N][N], float M[N][N])
{
    float tmp[N][N];

    mat6_mul(M, P, tmp);
    mat6_mul_bt(tmp, M, P);
}

static void mat6_identity(float M[N][N])
{
    memset(M, 0, sizeof(float) * N * N);

    for (uint8_t i = 0U; i < N; i++)
        M[i][i] = 1.0f;
}

/*
 * Inverse of a symmetric positive-definite 3x3 via Cholesky.
 * Returns false if S is not positive definite.
 */
static bool invert_spd3(float S[3][3], float inv[3][3])
{
    if (!(S[0][0] > 0.0f))
        return false;

    const float l00 = sqrtf(S[0][0]);
    const float l10 = S[1][0] / l00;
    const float l20 = S[2][0] / l00;

    const float d1 = S[1][1] - l10 * l10;
    if (!(d1 > 0.0f))
        return false;

    const float l11 = sqrtf(d1);
    const float l21 = (S[2][1] - l20 * l10) / l11;

    const float d2 = S[2][2] - l20 * l20 - l21 * l21;
    if (!(d2 > 0.0f))
        return false;

    const float l22 = sqrtf(d2);

    /* Linv (lower triangular). */
    float li[3][3] = {{0.0f}};
    li[0][0] = 1.0f / l00;
    li[1][1] = 1.0f / l11;
    li[2][2] = 1.0f / l22;
    li[1][0] = -l10 * li[0][0] * li[1][1];
    li[2][1] = -l21 * li[1][1] * li[2][2];
    li[2][0] = -(l20 * li[0][0] + l21 * li[1][0]) * li[2][2];

    /* S^-1 = Linv^T * Linv */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            float sum = 0.0f;

            for (uint8_t k = 0U; k < 3U; k++)
                sum += li[k][i] * li[k][j];

            inv[i][j] = sum;
        }
    }

    return true;
}

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */

void MEKF_GetDefaultConfig(MEKF_Config_t *cfg)
{
    if (cfg == NULL)
        return;

    cfg->calib_duration_s   = DEF_CALIB_DURATION_S;
    cfg->calib_max_gyro_std = DEF_CALIB_MAX_GYRO_STD;
    cfg->gyro_noise         = DEF_GYRO_NOISE;
    cfg->gyro_bias_rw       = DEF_GYRO_BIAS_RW;
    cfg->accel_noise        = DEF_ACCEL_NOISE;
    cfg->init_att_std       = DEF_INIT_ATT_STD;
    cfg->init_bias_std      = DEF_INIT_BIAS_STD;
    cfg->gravity            = DEF_GRAVITY;
    cfg->accel_soft_tol     = DEF_ACCEL_SOFT_TOL;
    cfg->accel_reject_tol   = DEF_ACCEL_REJECT_TOL;
    cfg->max_cov            = DEF_MAX_COV;
}

static void sanitize_config(MEKF_Config_t *cfg)
{
    MEKF_Config_t def;
    MEKF_GetDefaultConfig(&def);

    if (!positive_finite(cfg->calib_duration_s))   cfg->calib_duration_s   = def.calib_duration_s;
    if (!positive_finite(cfg->calib_max_gyro_std)) cfg->calib_max_gyro_std = def.calib_max_gyro_std;
    if (!positive_finite(cfg->gyro_noise))         cfg->gyro_noise         = def.gyro_noise;
    if (!positive_finite(cfg->gyro_bias_rw))       cfg->gyro_bias_rw       = def.gyro_bias_rw;
    if (!positive_finite(cfg->accel_noise))        cfg->accel_noise        = def.accel_noise;
    if (!positive_finite(cfg->init_att_std))       cfg->init_att_std       = def.init_att_std;
    if (!positive_finite(cfg->init_bias_std))      cfg->init_bias_std      = def.init_bias_std;
    if (!positive_finite(cfg->gravity))            cfg->gravity            = def.gravity;
    if (!positive_finite(cfg->accel_soft_tol))     cfg->accel_soft_tol     = def.accel_soft_tol;
    if (!positive_finite(cfg->accel_reject_tol))   cfg->accel_reject_tol   = def.accel_reject_tol;
    if (!positive_finite(cfg->max_cov))            cfg->max_cov            = def.max_cov;
}

/* ------------------------------------------------------------------------ */
/* Covariance helpers                                                       */
/* ------------------------------------------------------------------------ */

static void set_cov_diag(MEKF_t *m, float att_var, float bias_var)
{
    memset(m->P, 0, sizeof(m->P));

    for (uint8_t i = 0U; i < 3U; i++)
    {
        m->P[i][i] = att_var;
        m->P[i + 3U][i + 3U] = bias_var;
    }
}

/* P += Q(dt): gyro angular random walk + bias random walk (exact integral). */
static void add_process_noise(MEKF_t *m, float dt)
{
    const float sg2 = m->cfg.gyro_noise * m->cfg.gyro_noise;
    const float sb2 = m->cfg.gyro_bias_rw * m->cfg.gyro_bias_rw;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        m->P[i][i]           += sg2 * dt + sb2 * dt3 / 3.0f;
        m->P[i][i + 3U]      += -sb2 * dt2 / 2.0f;
        m->P[i + 3U][i]      += -sb2 * dt2 / 2.0f;
        m->P[i + 3U][i + 3U] += sb2 * dt;
    }
}

static void recover(MEKF_t *m)
{
    m->recoveries++;

    if (!all_finite(m->q, 4U))
        quat_identity(m->q);

    if (!all_finite(m->bg, 3U))
        memset(m->bg, 0, sizeof(m->bg));

    set_cov_diag(m, 0.1f * 0.1f, m->cfg.init_bias_std * m->cfg.init_bias_std);
    m->event = MEKF_EVENT_NUMERICAL_RESET;
}

static void sanitize_cov(MEKF_t *m)
{
    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < N; j++)
        {
            if (!isfinite(m->P[i][j]))
            {
                recover(m);
                return;
            }
        }
    }

    /* Symmetrize. */
    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = (uint8_t)(i + 1U); j < N; j++)
        {
            const float avg = 0.5f * (m->P[i][j] + m->P[j][i]);
            m->P[i][j] = avg;
            m->P[j][i] = avg;
        }
    }

    for (uint8_t i = 0U; i < N; i++)
    {
        if (m->P[i][i] < 0.0f)
        {
            recover(m);
            return;
        }
    }

    for (uint8_t i = 0U; i < N; i++)
    {
        float d = m->P[i][i];

        if (d < MIN_COV_DIAG)        d = MIN_COV_DIAG;
        if (d > m->cfg.max_cov)      d = m->cfg.max_cov;

        m->P[i][i] = d;
    }
}

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                */
/* ------------------------------------------------------------------------ */

static void start_calibration(MEKF_t *m, bool bias_only)
{
    m->calib_bias_only = bias_only;
    m->calib_count = 0U;
    m->calib_elapsed_s = 0.0f;
    m->calib_tries = 0U;
    memset(m->calib_sum_gyro, 0, sizeof(m->calib_sum_gyro));
    memset(m->calib_sum_gyro_sq, 0, sizeof(m->calib_sum_gyro_sq));
    memset(m->calib_sum_acc, 0, sizeof(m->calib_sum_acc));
    m->state = MEKF_STATE_CALIBRATING;
}

void MEKF_Init(MEKF_t *mekf, const MEKF_Config_t *cfg)
{
    if (mekf == NULL)
        return;

    memset(mekf, 0, sizeof(*mekf));

    if (cfg != NULL)
        mekf->cfg = *cfg;
    else
        MEKF_GetDefaultConfig(&mekf->cfg);

    sanitize_config(&mekf->cfg);
    MEKF_Reinitialize(mekf);
}

void MEKF_SetConfig(MEKF_t *mekf, const MEKF_Config_t *cfg)
{
    if (mekf == NULL || cfg == NULL)
        return;

    MEKF_Config_t c = *cfg;

    sanitize_config(&c);
    mekf->cfg = c;
}

void MEKF_SetGyroBias(MEKF_t *mekf, const float bg_rads[3])
{
    if (mekf == NULL || bg_rads == NULL || !all_finite(bg_rads, 3U))
        return;

    memcpy(mekf->bg, bg_rads, sizeof(mekf->bg));
}

void MEKF_Reinitialize(MEKF_t *mekf)
{
    if (mekf == NULL)
        return;

    quat_identity(mekf->q);
    memset(mekf->bg, 0, sizeof(mekf->bg));

    /* Unknown attitude until calibration completes. */
    set_cov_diag(mekf, 0.5f * 0.5f,
                 mekf->cfg.init_bias_std * mekf->cfg.init_bias_std);

    mekf->rejected = 0U;
    mekf->updates = 0U;
    mekf->recoveries = 0U;
    mekf->has_last_acc = false;
    memset(mekf->last_acc, 0, sizeof(mekf->last_acc));

    start_calibration(mekf, false);
    quat_identity(mekf->q_prev);
    mekf->event = MEKF_EVENT_CALIBRATING;
}

bool MEKF_StartWithKnownBias(MEKF_t *mekf,
                             const float gyro_bias_rads[3],
                             const float accel_g[3])
{
    if (mekf == NULL || gyro_bias_rads == NULL || accel_g == NULL)
        return false;

    if (!all_finite(gyro_bias_rads, 3U) || !all_finite(accel_g, 3U))
        return false;

    const float n2 = accel_g[0] * accel_g[0] +
                     accel_g[1] * accel_g[1] +
                     accel_g[2] * accel_g[2];

    if (n2 < 1e-12f)
        return false;

    memcpy(mekf->bg, gyro_bias_rads, sizeof(mekf->bg));
    quat_from_accel(accel_g, mekf->q);
    memcpy(mekf->q_prev, mekf->q, sizeof(mekf->q));
    memcpy(mekf->last_acc, accel_g, sizeof(mekf->last_acc));
    mekf->has_last_acc = true;

    set_cov_diag(mekf,
                 mekf->cfg.init_att_std * mekf->cfg.init_att_std,
                 mekf->cfg.init_bias_std * mekf->cfg.init_bias_std);

    mekf->state = MEKF_STATE_RUNNING;
    mekf->event = MEKF_EVENT_RUNNING;
    return true;
}

void MEKF_RecalibrateBias(MEKF_t *mekf)
{
    if (mekf == NULL)
        return;

    start_calibration(mekf, true);
    mekf->event = MEKF_EVENT_CALIBRATING;
}

void MEKF_ResetAttitude(MEKF_t *mekf)
{
    if (mekf == NULL)
        return;

    if (mekf->has_last_acc)
        quat_from_accel(mekf->last_acc, mekf->q);
    else
        quat_identity(mekf->q);

    const float att_var = mekf->cfg.init_att_std * mekf->cfg.init_att_std;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            mekf->P[i][j] = (i == j) ? att_var : 0.0f;
            mekf->P[i][j + 3U] = 0.0f;
            mekf->P[j + 3U][i] = 0.0f;
        }
    }

    memcpy(mekf->q_prev, mekf->q, sizeof(mekf->q));
    mekf->event = MEKF_EVENT_ATTITUDE_RESET;
}

/* ------------------------------------------------------------------------ */
/* Calibration                                                              */
/* ------------------------------------------------------------------------ */

static void calibrate(MEKF_t *m,
                      const float gyro[3],
                      const float acc[3],
                      float dt)
{
    m->calib_count++;
    m->calib_elapsed_s += dt;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        m->calib_sum_gyro[i]    += gyro[i];
        m->calib_sum_gyro_sq[i] += gyro[i] * gyro[i];
        m->calib_sum_acc[i]     += acc[i];
    }

    if (m->calib_elapsed_s < m->cfg.calib_duration_s)
    {
        m->event = MEKF_EVENT_CALIBRATING;
        return;
    }

    const float inv_n = 1.0f / (float)m->calib_count;
    float mean_gyro[3];
    float mean_acc[3];
    float max_std = 0.0f;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        mean_gyro[i] = m->calib_sum_gyro[i] * inv_n;
        mean_acc[i]  = m->calib_sum_acc[i] * inv_n;

        float var = m->calib_sum_gyro_sq[i] * inv_n -
                    mean_gyro[i] * mean_gyro[i];
        if (var < 0.0f)
            var = 0.0f;

        const float s = sqrtf(var);
        if (s > max_std)
            max_std = s;
    }

    const float acc_norm2 = mean_acc[0] * mean_acc[0] +
                            mean_acc[1] * mean_acc[1] +
                            mean_acc[2] * mean_acc[2];

    const bool moving = (max_std > m->cfg.calib_max_gyro_std) ||
                        (acc_norm2 < 1e-12f);

    if (moving && m->calib_tries < CALIB_MAX_RESTARTS)
    {
        m->calib_tries++;
        m->calib_count = 0U;
        m->calib_elapsed_s = 0.0f;
        memset(m->calib_sum_gyro, 0, sizeof(m->calib_sum_gyro));
        memset(m->calib_sum_gyro_sq, 0, sizeof(m->calib_sum_gyro_sq));
        memset(m->calib_sum_acc, 0, sizeof(m->calib_sum_acc));
        m->event = MEKF_EVENT_CALIBRATION_RESTART;
        return;
    }

    memcpy(m->bg, mean_gyro, sizeof(m->bg));

    if (!m->calib_bias_only)
    {
        quat_from_accel(mean_acc, m->q);
        memcpy(m->q_prev, m->q, sizeof(m->q));
        set_cov_diag(m,
                     m->cfg.init_att_std * m->cfg.init_att_std,
                     m->cfg.init_bias_std * m->cfg.init_bias_std);
    }
    else
    {
        const float bias_var = m->cfg.init_bias_std * m->cfg.init_bias_std;

        for (uint8_t i = 0U; i < 3U; i++)
        {
            for (uint8_t j = 0U; j < 3U; j++)
            {
                m->P[i + 3U][j + 3U] = (i == j) ? bias_var : 0.0f;
                m->P[i][j + 3U] = 0.0f;
                m->P[j + 3U][i] = 0.0f;
            }
        }
    }

    m->state = MEKF_STATE_RUNNING;
    m->event = moving ? MEKF_EVENT_RUNNING_CALIBRATED_WITH_MOTION
                      : MEKF_EVENT_RUNNING;
}

/* ------------------------------------------------------------------------ */
/* Prediction                                                               */
/* ------------------------------------------------------------------------ */

static void predict(MEKF_t *m, const float gyro[3], float dt)
{
    float th[3];
    float dq[4];
    float qn[4];

    for (uint8_t i = 0U; i < 3U; i++)
        th[i] = (gyro[i] - m->bg[i]) * dt;

    quat_from_rotvec(th, dq);
    quat_mul(m->q, dq, qn);
    memcpy(m->q, qn, sizeof(qn));
    quat_normalize(m->q);

    /* F = [[exp(-[w dt]x), -I dt], [0, I]] */
    float Rth[3][3];
    float F[N][N];

    rotvec_to_rot(th, Rth);
    mat6_identity(F);

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
            F[i][j] = Rth[j][i];            /* transpose */

        F[i][i + 3U] = -dt;
    }

    congruence(m->P, F);
    add_process_noise(m, dt);
    sanitize_cov(m);
}

/* ------------------------------------------------------------------------ */
/* Accelerometer update                                                     */
/* ------------------------------------------------------------------------ */

static void update_accel(MEKF_t *m, const float acc[3])
{
    const MEKF_Config_t *c = &m->cfg;

    const float n2 = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];

    if (!isfinite(n2) || n2 < 1e-12f)
    {
        m->event = MEKF_EVENT_ACCEL_INVALID;
        return;
    }

    const float n = sqrtf(n2);
    const float dev = fabsf(n - c->gravity) / c->gravity;

    if (dev > c->accel_reject_tol)
    {
        m->rejected++;
        m->event = MEKF_EVENT_ACCEL_REJECTED;
        return;
    }

    /* Expected unit specific force in body axes: h = -R^T e3 = -R[2][:]. */
    float R[3][3];
    float h[3];
    float y[3];
    float Sk[3][3];

    quat_to_rot(m->q, R);

    for (uint8_t i = 0U; i < 3U; i++)
    {
        h[i] = -R[2][i];
        y[i] = acc[i] / n - h[i];
    }

    skew(h, Sk);

    /* Measurement noise (isotropic), inflated for dynamic acceleration. */
    const float ratio = dev / c->accel_soft_tol;
    const float r = c->accel_noise * c->accel_noise * (1.0f + ratio * ratio);

    /* H = [Sk 0].  PHt = P H^T (6x3),  S = H P H^T + r I (3x3). */
    float PHt[N][3];
    float S[3][3];

    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            float sum = 0.0f;

            for (uint8_t k = 0U; k < 3U; k++)
                sum += m->P[i][k] * Sk[j][k];

            PHt[i][j] = sum;
        }
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            float sum = 0.0f;

            for (uint8_t k = 0U; k < 3U; k++)
                sum += Sk[i][k] * PHt[k][j];

            S[i][j] = sum + ((i == j) ? r : 0.0f);
        }
    }

    float Sinv[3][3];

    if (!invert_spd3(S, Sinv))
    {
        m->event = MEKF_EVENT_ACCEL_SINGULAR;
        return;
    }

    /* K = PHt * S^-1 (6x3). */
    float K[N][3];

    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            float sum = 0.0f;

            for (uint8_t k = 0U; k < 3U; k++)
                sum += PHt[i][k] * Sinv[k][j];

            K[i][j] = sum;
        }
    }

    /* dx = K y */
    float dx[N];

    for (uint8_t i = 0U; i < N; i++)
        dx[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];

    const float dth_norm2 = dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2];

    if (!all_finite(dx, N) ||
        dth_norm2 > (MAX_INNOVATION_RAD * MAX_INNOVATION_RAD))
    {
        m->rejected++;
        m->event = MEKF_EVENT_ACCEL_INNOVATION_REJECTED;
        return;
    }

    /* Joseph form: P = (I-KH) P (I-KH)^T + K Rn K^T. */
    float IKH[N][N];

    mat6_identity(IKH);

    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
        {
            float kh = 0.0f;

            for (uint8_t k = 0U; k < 3U; k++)
                kh += K[i][k] * Sk[k][j];

            IKH[i][j] -= kh;
        }
    }

    congruence(m->P, IKH);

    for (uint8_t i = 0U; i < N; i++)
    {
        for (uint8_t j = 0U; j < N; j++)
        {
            m->P[i][j] += r * (K[i][0] * K[j][0] +
                               K[i][1] * K[j][1] +
                               K[i][2] * K[j][2]);
        }
    }

    /* Error injection: q <- q (x) exp(dtheta), bg += dbg. */
    float dq[4];
    float qn[4];

    quat_from_rotvec(dx, dq);
    quat_mul(m->q, dq, qn);
    memcpy(m->q, qn, sizeof(qn));
    quat_normalize(m->q);

    for (uint8_t i = 0U; i < 3U; i++)
        m->bg[i] += dx[i + 3U];

    /* Covariance reset: G = diag(I - 0.5 [dtheta]x, I). */
    float G[N][N];
    float Sd[3][3];

    mat6_identity(G);
    skew(dx, Sd);

    for (uint8_t i = 0U; i < 3U; i++)
    {
        for (uint8_t j = 0U; j < 3U; j++)
            G[i][j] -= 0.5f * Sd[i][j];
    }

    congruence(m->P, G);
    sanitize_cov(m);

    m->updates++;
    m->event = (dev < c->accel_soft_tol) ? MEKF_EVENT_RUNNING
                                         : MEKF_EVENT_ACCEL_DOWNWEIGHTED;
}

/* ------------------------------------------------------------------------ */
/* Public processing                                                        */
/* ------------------------------------------------------------------------ */

static void finish(MEKF_t *m)
{
    if (!all_finite(m->q, 4U) || !all_finite(m->bg, 3U))
    {
        recover(m);
        return;
    }

    quat_normalize(m->q);

    /* Sign continuity. */
    const float dot = m->q_prev[0] * m->q[0] + m->q_prev[1] * m->q[1] +
                      m->q_prev[2] * m->q[2] + m->q_prev[3] * m->q[3];

    if (dot < 0.0f)
    {
        for (uint8_t i = 0U; i < 4U; i++)
            m->q[i] = -m->q[i];
    }

    memcpy(m->q_prev, m->q, sizeof(m->q));
}

MEKF_Event_t MEKF_Process(MEKF_t *mekf,
                          const float gyro_rads[3],
                          const float acc_g[3],
                          float dt_s)
{
    if (mekf == NULL)
        return MEKF_EVENT_INVALID_SAMPLE;

    if (gyro_rads == NULL || acc_g == NULL ||
        !all_finite(gyro_rads, 3U) || !all_finite(acc_g, 3U) ||
        !isfinite(dt_s) || dt_s <= 0.0f)
    {
        mekf->event = MEKF_EVENT_INVALID_SAMPLE;
        return mekf->event;
    }

    memcpy(mekf->last_acc, acc_g, sizeof(mekf->last_acc));
    mekf->has_last_acc = true;

    if (mekf->state == MEKF_STATE_CALIBRATING)
    {
        calibrate(mekf, gyro_rads, acc_g, dt_s);
        return mekf->event;
    }

    predict(mekf, gyro_rads, dt_s);
    update_accel(mekf, acc_g);
    finish(mekf);

    return mekf->event;
}

void MEKF_InflateCovariance(MEKF_t *mekf, float gap_s)
{
    if (mekf == NULL || mekf->state != MEKF_STATE_RUNNING ||
        !isfinite(gap_s) || gap_s <= 0.0f)
    {
        return;
    }

    add_process_noise(mekf, (gap_s < MAX_GAP_S) ? gap_s : MAX_GAP_S);
    sanitize_cov(mekf);

    if (mekf->event != MEKF_EVENT_NUMERICAL_RESET)
        mekf->event = MEKF_EVENT_DATA_GAP;
}

/* ------------------------------------------------------------------------ */
/* Getters                                                                  */
/* ------------------------------------------------------------------------ */

MEKF_State_t MEKF_GetState(const MEKF_t *mekf)
{
    return (mekf == NULL) ? MEKF_STATE_CALIBRATING : mekf->state;
}

MEKF_Event_t MEKF_GetEvent(const MEKF_t *mekf)
{
    return (mekf == NULL) ? MEKF_EVENT_INVALID_SAMPLE : mekf->event;
}

bool MEKF_IsRunning(const MEKF_t *mekf)
{
    return (mekf != NULL) && (mekf->state == MEKF_STATE_RUNNING);
}

float MEKF_GetCalibrationProgress(const MEKF_t *mekf)
{
    if (mekf == NULL)
        return 0.0f;

    if (mekf->state == MEKF_STATE_RUNNING)
        return 1.0f;

    const float p = mekf->calib_elapsed_s / mekf->cfg.calib_duration_s;

    return (p > 1.0f) ? 1.0f : p;
}

void MEKF_GetQuaternion(const MEKF_t *mekf, float q[4])
{
    if (mekf == NULL || q == NULL)
        return;

    memcpy(q, mekf->q, sizeof(mekf->q));
}

void MEKF_GetGyroBias(const MEKF_t *mekf, float bg_rads[3])
{
    if (mekf == NULL || bg_rads == NULL)
        return;

    memcpy(bg_rads, mekf->bg, sizeof(mekf->bg));
}

void MEKF_GetEuler(const MEKF_t *mekf,
                   float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    if (mekf == NULL)
        return;

    const float w = mekf->q[0], x = mekf->q[1], y = mekf->q[2], z = mekf->q[3];

    float sp = 2.0f * (w * y - z * x);
    if (sp > 1.0f)  sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;

    if (roll_rad != NULL)
        *roll_rad = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

    if (pitch_rad != NULL)
        *pitch_rad = asinf(sp);

    if (yaw_rad != NULL)
        *yaw_rad = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
