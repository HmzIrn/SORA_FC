#include "motor_mixer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define NUM_MOTORS  MOTOR_MIXER_NUM_MOTORS

#define PWM_MIN_US          (1000.0f)
#define PWM_SPAN_US         (1000.0f)
#define INTEGRATE_ERR_LIMIT (0.15f)   /* integrate only when |e| < this (rad) */

/*
 * Columns = [roll, pitch, yaw] command -> per-motor thrust delta.
 * Derived from the layout in motor_mixer.h:
 *   row = [-y_sign, +x_sign, +spin]
 */
static const float MIX_MATRIX[NUM_MOTORS][3] =
{
    { -1.0f, +1.0f, +1.0f },   /* M1 FR, CCW */
    { +1.0f, -1.0f, +1.0f },   /* M2 RL, CCW */
    { +1.0f, +1.0f, -1.0f },   /* M3 FL, CW  */
    { -1.0f, -1.0f, -1.0f }    /* M4 RR, CW  */
};

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

static bool non_negative_finite(float v)
{
    return isfinite(v) && v >= 0.0f;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* out = a (x) b. out must not alias a or b. */
static void quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void clear_output(MotorMixer_Output_t *o)
{
    memset(o, 0, sizeof(*o));

    for (uint8_t i = 0U; i < NUM_MOTORS; i++)
        o->pwm_us[i] = PWM_MIN_US;

    o->status = MOTOR_MIXER_STATUS_DISABLED;
}

static void finish_output(const MotorMixer_t *m, MotorMixer_Output_t *o)
{
    o->active = true;

    for (uint8_t i = 0U; i < NUM_MOTORS; i++)
    {
        o->speed[i]  = sqrtf(clampf(o->thrust[i], 0.0f, 1.0f));
        o->rpm[i]    = o->speed[i] * m->cfg.max_rpm;
        o->pwm_us[i] = PWM_MIN_US + PWM_SPAN_US * o->speed[i];
    }
}

void MotorMixer_GetDefaultConfig(MotorMixer_Config_t *cfg)
{
    if (cfg == NULL)
        return;

    cfg->twr          = 2.0f;
    cfg->max_rpm      = 12000.0f;
    cfg->idle_thrust  = 0.03f;
    cfg->kp_rp        = 0.30f;
    cfg->ki_rp        = 0.03f;
    cfg->kd_rp        = 0.02f;
    cfg->kp_yaw       = 0.10f;
    cfg->ki_yaw       = 0.01f;
    cfg->kd_yaw       = 0.05f;
    cfg->i_limit      = 0.10f;
    cfg->use_integral = false;
    cfg->min_cos_tilt = 0.5f;
    cfg->max_tilt_deg = 60.0f;
    cfg->trim[0]      = 0.0f;
    cfg->trim[1]      = 0.0f;
    cfg->trim[2]      = 0.0f;
}

static void sanitize_config(MotorMixer_Config_t *cfg)
{
    MotorMixer_Config_t def;
    MotorMixer_GetDefaultConfig(&def);

    if (!positive_finite(cfg->twr))           cfg->twr          = def.twr;
    if (!positive_finite(cfg->max_rpm))       cfg->max_rpm      = def.max_rpm;
    if (!isfinite(cfg->idle_thrust) ||
        cfg->idle_thrust < 0.0f || cfg->idle_thrust >= 1.0f)
                                              cfg->idle_thrust  = def.idle_thrust;
    if (!non_negative_finite(cfg->kp_rp))     cfg->kp_rp        = def.kp_rp;
    if (!non_negative_finite(cfg->ki_rp))     cfg->ki_rp        = def.ki_rp;
    if (!non_negative_finite(cfg->kd_rp))     cfg->kd_rp        = def.kd_rp;
    if (!non_negative_finite(cfg->kp_yaw))    cfg->kp_yaw       = def.kp_yaw;
    if (!non_negative_finite(cfg->ki_yaw))    cfg->ki_yaw       = def.ki_yaw;
    if (!non_negative_finite(cfg->kd_yaw))    cfg->kd_yaw       = def.kd_yaw;
    if (!non_negative_finite(cfg->i_limit))   cfg->i_limit      = def.i_limit;
    if (!positive_finite(cfg->min_cos_tilt))  cfg->min_cos_tilt = def.min_cos_tilt;
    if (!positive_finite(cfg->max_tilt_deg))  cfg->max_tilt_deg = def.max_tilt_deg;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!isfinite(cfg->trim[i]) || fabsf(cfg->trim[i]) > 0.5f)
            cfg->trim[i] = 0.0f;
    }
}

void MotorMixer_SetConfig(MotorMixer_t *mixer, const MotorMixer_Config_t *cfg)
{
    if (mixer == NULL || cfg == NULL)
        return;

    MotorMixer_Config_t c = *cfg;

    sanitize_config(&c);
    mixer->cfg = c;
}

void MotorMixer_Init(MotorMixer_t *mixer, const MotorMixer_Config_t *cfg)
{
    if (mixer == NULL)
        return;

    memset(mixer, 0, sizeof(*mixer));

    if (cfg != NULL)
        mixer->cfg = *cfg;
    else
        MotorMixer_GetDefaultConfig(&mixer->cfg);

    sanitize_config(&mixer->cfg);
    mixer->yaw_sp = 0.0f;
    MotorMixer_Reset(mixer);
}

void MotorMixer_Reset(MotorMixer_t *mixer)
{
    if (mixer == NULL)
        return;

    memset(mixer->integrator, 0, sizeof(mixer->integrator));
    clear_output(&mixer->out);
}

void MotorMixer_SetYawHold(MotorMixer_t *mixer, float yaw_rad)
{
    if (mixer == NULL || !isfinite(yaw_rad))
        return;

    mixer->yaw_sp = yaw_rad;
}

float MotorMixer_GetHoverThrust(const MotorMixer_t *mixer)
{
    if (mixer == NULL)
        return 0.0f;

    return 1.0f / ((mixer->cfg.twr > 1.0f) ? mixer->cfg.twr : 1.0f);
}

const MotorMixer_Output_t *MotorMixer_Update(MotorMixer_t *mixer,
                                              const float q[4],
                                              const float omega[3],
                                              float dt,
                                              bool enabled,
                                              bool filter_ready)
{
    static MotorMixer_Output_t null_out;

    if (mixer == NULL)
    {
        clear_output(&null_out);
        return &null_out;
    }

    const MotorMixer_Config_t *c = &mixer->cfg;
    MotorMixer_Output_t *o = &mixer->out;

    clear_output(o);

    if (!enabled)
    {
        o->status = MOTOR_MIXER_STATUS_DISABLED;
        return o;
    }

    if (!filter_ready)
    {
        o->status = MOTOR_MIXER_STATUS_WAITING_FOR_FILTER;
        return o;
    }

    if (q == NULL || omega == NULL ||
        !all_finite(q, 4U) || !all_finite(omega, 3U) ||
        !isfinite(dt) || dt < 0.0f)
    {
        o->status = MOTOR_MIXER_STATUS_INVALID_INPUT;
        return o;
    }

    /* cos(tilt) of body z vs world down = R[2][2]. */
    const float cos_t = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    const float f_min = c->idle_thrust;
    const float f_max = 1.0f;

    if (cos_t < cosf(c->max_tilt_deg * 0.017453292519943295f))
    {
        for (uint8_t i = 0U; i < NUM_MOTORS; i++)
            o->thrust[i] = f_min;

        o->status = MOTOR_MIXER_STATUS_TILT_LIMIT;
        finish_output(mixer, o);
        return o;
    }

    /* Desired attitude: level, captured yaw. q_des = Rz(yaw_sp). */
    const float q_des[4] =
    {
        cosf(0.5f * mixer->yaw_sp), 0.0f, 0.0f, sinf(0.5f * mixer->yaw_sp)
    };

    /* Body-frame error: q^-1 (x) q_des, shortest path. */
    const float q_conj[4] = { q[0], -q[1], -q[2], -q[3] };
    float qe[4];
    quat_mul(q_conj, q_des, qe);

    const float sign = (qe[0] >= 0.0f) ? 1.0f : -1.0f;
    const float e[3] = { 2.0f * qe[1] * sign,
                         2.0f * qe[2] * sign,
                         2.0f * qe[3] * sign };

    const float kp[3] = { c->kp_rp, c->kp_rp, c->kp_yaw };
    const float ki[3] = { c->ki_rp, c->ki_rp, c->ki_yaw };
    const float kd[3] = { c->kd_rp, c->kd_rp, c->kd_yaw };

    if (!c->use_integral)
        memset(mixer->integrator, 0, sizeof(mixer->integrator));

    float u[3];

    for (uint8_t i = 0U; i < 3U; i++)
        u[i] = kp[i] * e[i] + ki[i] * mixer->integrator[i] - kd[i] * omega[i] +
               c->trim[i];

    float d[NUM_MOTORS];
    float d_min, d_max;

    for (uint8_t i = 0U; i < NUM_MOTORS; i++)
    {
        d[i] = MIX_MATRIX[i][0] * u[0] +
               MIX_MATRIX[i][1] * u[1] +
               MIX_MATRIX[i][2] * u[2];
    }

    d_min = d[0];
    d_max = d[0];

    for (uint8_t i = 1U; i < NUM_MOTORS; i++)
    {
        if (d[i] < d_min) d_min = d[i];
        if (d[i] > d_max) d_max = d[i];
    }

    const float hover = MotorMixer_GetHoverThrust(mixer);
    float base = hover / ((cos_t > c->min_cos_tilt) ? cos_t : c->min_cos_tilt);
    bool sat = false;

    /* Not enough range: scale attitude commands (attitude has priority). */
    const float span = d_max - d_min;

    if (span > (f_max - f_min))
    {
        const float k = (f_max - f_min) / span;

        for (uint8_t i = 0U; i < NUM_MOTORS; i++)
            d[i] *= k;

        for (uint8_t i = 0U; i < 3U; i++)
            u[i] *= k;

        d_min *= k;
        d_max *= k;
        sat = true;
    }

    /* Shift collective so every motor stays inside [f_min, f_max]. */
    const float lo = f_min - d_min;
    const float hi = f_max - d_max;

    if (base < lo || base > hi)
    {
        base = clampf(base, lo, hi);
        sat = true;
    }

    for (uint8_t i = 0U; i < NUM_MOTORS; i++)
        o->thrust[i] = clampf(base + d[i], f_min, f_max);

    /* Integrate only when not saturated and near level. */
    if (!sat && dt > 0.0f)
    {
        for (uint8_t i = 0U; i < 3U; i++)
        {
            const float lim = c->i_limit / ((ki[i] > 1e-9f) ? ki[i] : 1e-9f);
            const float err = (fabsf(e[i]) < INTEGRATE_ERR_LIMIT) ? e[i] : 0.0f;

            mixer->integrator[i] =
                clampf(mixer->integrator[i] + err * dt, -lim, lim);
        }
    }

    o->collective = base;
    memcpy(o->torque, u, sizeof(o->torque));
    o->saturated = sat;
    o->status = sat ? MOTOR_MIXER_STATUS_SATURATED
                    : MOTOR_MIXER_STATUS_HOVERING;

    finish_output(mixer, o);
    return o;
}
