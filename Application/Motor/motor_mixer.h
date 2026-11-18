#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Hover controller + quad-X motor mixer. Ported from dash.py.
 *
 * Holds level attitude and a captured yaw using PD(+I) control on the
 * attitude-error quaternion (body frame), then mixes roll/pitch/yaw commands
 * into four motor thrusts. Output only: arming, motor cut-off and the actual
 * timer/DShot writes are the caller's responsibility.
 *
 * Frames/conventions are the same as mekf.h: FRD body, NED world,
 * Hamilton quaternion [w, x, y, z] body -> world.
 *
 * Motor layout (PX4 numbering), looking from above, nose up the page:
 *
 *        front
 *     M3(CW)  M1(CCW)
 *     M2(CCW) M4(CW)
 *        rear
 *
 * Array index = motor number - 1: [0]=M1 FR, [1]=M2 RL, [2]=M3 FL, [3]=M4 RR.
 * Map these to your timer channels in your board code.
 *
 * Mixing (roll,pitch,yaw) -> per-motor thrust delta:
 *   M1: -roll +pitch +yaw     M3: +roll +pitch -yaw
 *   M2: +roll -pitch +yaw     M4: -roll -pitch -yaw
 *   roll  + = right wing down, pitch + = nose up, yaw + = nose right.
 *
 * Thrust domain: hover thrust per motor = 1/TWR. Collective is tilt
 * compensated (vertical thrust = weight). Saturation is resolved by shifting
 * the collective first, then scaling the attitude commands (attitude has
 * priority). Motor speed = sqrt(thrust fraction) since thrust ~ omega^2.
 */

#define MOTOR_MIXER_NUM_MOTORS   (4U)

typedef enum
{
    MOTOR_MIXER_STATUS_DISABLED = 0,
    MOTOR_MIXER_STATUS_WAITING_FOR_FILTER,
    MOTOR_MIXER_STATUS_INVALID_INPUT,
    MOTOR_MIXER_STATUS_TILT_LIMIT,     /* beyond max tilt: outputs idle */
    MOTOR_MIXER_STATUS_HOVERING,
    MOTOR_MIXER_STATUS_SATURATED
} MotorMixer_Status_t;

typedef struct
{
    float twr;            /* max total thrust / weight (hover = 1/TWR per motor) */
    float max_rpm;        /* for the RPM read-out only                           */
    float idle_thrust;    /* minimum thrust fraction per motor (spin-up floor)   */
    float kp_rp;          /* thrust fraction per rad            (roll/pitch)     */
    float ki_rp;
    float kd_rp;          /* thrust fraction per rad/s                           */
    float kp_yaw;
    float ki_yaw;
    float kd_yaw;
    float i_limit;        /* integrator clamp (thrust fraction)                  */
    bool  use_integral;   /* OFF for bench tests, ON for flight                  */
    float min_cos_tilt;   /* tilt compensation limit                             */
    float max_tilt_deg;   /* beyond this the mixer outputs idle (fail-safe)      */
} MotorMixer_Config_t;

typedef struct
{
    float thrust[MOTOR_MIXER_NUM_MOTORS];   /* fraction of max thrust, 0..1       */
    float speed[MOTOR_MIXER_NUM_MOTORS];    /* sqrt(thrust), 0..1                 */
    float rpm[MOTOR_MIXER_NUM_MOTORS];
    float pwm_us[MOTOR_MIXER_NUM_MOTORS];   /* 1000..2000 us equivalent           */
    float collective;                       /* thrust-domain base value           */
    float torque[3];                        /* roll, pitch, yaw commands          */
    bool  saturated;
    bool  active;                           /* false = outputs are not valid      */
    MotorMixer_Status_t status;
} MotorMixer_Output_t;

typedef struct
{
    MotorMixer_Config_t cfg;
    float yaw_sp;                 /* [rad] */
    float integrator[3];
    MotorMixer_Output_t out;
} MotorMixer_t;

void MotorMixer_GetDefaultConfig(MotorMixer_Config_t *cfg);

/* cfg may be NULL (defaults). Invalid fields fall back to defaults. */
void MotorMixer_Init(MotorMixer_t *mixer, const MotorMixer_Config_t *cfg);

/* Clears integrator and output. Yaw set-point is kept. */
void MotorMixer_Reset(MotorMixer_t *mixer);

void  MotorMixer_SetYawHold(MotorMixer_t *mixer, float yaw_rad);
float MotorMixer_GetHoverThrust(const MotorMixer_t *mixer);

/*
 * q     : attitude quaternion [w,x,y,z] (body -> world), e.g. MEKF_GetQuaternion
 * omega : body rates in rad/s with gyro bias removed (gyro - mekf.bg)
 * dt    : seconds since the previous call
 *
 * When enabled is false or filter_ready is false, or the input is invalid,
 * the returned output has active == false and zero thrust (pwm_us = 1000).
 * Returns a pointer to mixer->out (never NULL for a non-NULL mixer).
 */
const MotorMixer_Output_t *MotorMixer_Update(MotorMixer_t *mixer,
                                              const float q[4],
                                              const float omega[3],
                                              float dt,
                                              bool enabled,
                                              bool filter_ready);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MIXER_H */
