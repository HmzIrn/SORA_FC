#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "imu_calibration.h"
#include "mekf.h"
#include "motor_mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Text command interface over USB CDC.
 *
 * Lines end with CR and/or LF. Case-insensitive. Separators: space, comma,
 * '=' or tab, so "SET mix.kp_rp 0.35", "set,mix.kp_rp,0.35" and
 * "set mix.kp_rp=0.35" are the same command.
 *
 *   HELP                      list commands
 *   STATUS                    calibration / filter / storage state
 *   LIST [prefix]             all parameters (or those starting with prefix)
 *   GET <name>                read one parameter
 *   SET <name> <value>        change ONE parameter live (not saved yet)
 *   SAVE                      write all current parameters to flash
 *   LOAD                      reload the stored parameters (drops unsaved edits)
 *   CAL FULL                  full IMU calibration (button per face), then
 *                             MEKF bias calibration, then auto-save
 *   CAL BIAS                  MEKF gyro-bias calibration only (~2 s, board
 *                             still), then auto-save
 *   CLEAR                     erase stored config (next boot recalibrates)
 *   REBOOT                    reset the MCU
 *
 * Replies: "OK,..." or "ERR,<reason>,...". LIST sends "PARAM,..." lines then
 * "OK,LIST,<count>". Asynchronous notices start with "CFG," / "CAL," /
 * "WARN,". Parameter groups: cal.* (accel/gyro calibration), mekf.* (filter
 * tuning + residual gyro bias), mix.* (PID, limits, trims).
 *
 * FRAMES: cal.gyro_bias_*, cal.accel_offset_*, cal.accel_scale_* are in the SENSOR
 * frame (applied to the raw IMU values). mekf.bias_* is the filter's residual gyro
 * bias and is in the BODY frame (FRD), i.e. after the SENSOR_TO_BODY mapping in
 * main.c - with the default diag(1,-1,-1) mapping the y and z signs are flipped
 * relative to the sensor frame. Units: bias in deg/s, offsets in g.
 *
 * Integration: call USB_Command_Feed() from CDC_Receive_FS() and
 * USB_Command_Process() once per main-loop iteration.
 */

typedef enum
{
    USB_PARAM_FLOAT = 0,
    USB_PARAM_BOOL
} USB_ParamType_t;

/* Extra action required after a parameter changed. */
typedef enum
{
    USB_PARAM_APPLY_NONE = 0,
    USB_PARAM_APPLY_MEKF_BIAS   /* push mekf bias [dps] into the live filter */
} USB_ParamApply_t;

typedef struct
{
    const char      *name;
    void            *ptr;       /* float* or bool* */
    USB_ParamType_t  type;
    float            min;
    float            max;
    USB_ParamApply_t apply;
} USB_Param_t;

/* Callbacks into the application (any may be NULL -> "ERR,UNSUPPORTED"). */
typedef struct
{
    bool (*start_full_calibration)(void);
    bool (*start_bias_calibration)(void);
    bool (*save)(void);                 /* gather + write to flash          */
    bool (*load)(void);                 /* read flash + apply to live state */
    bool (*clear)(void);                /* erase stored config              */
    void (*reboot)(void);
    bool (*busy)(void);                 /* true while calibrating: SET/SAVE/LOAD refused */
    void (*param_changed)(const USB_Param_t *param);
    int  (*format_status)(char *buf, size_t size);
} USB_Command_Hooks_t;

/* Live objects the parameter table points into. */
typedef struct
{
    IMU_CalibrationResult_t *imu_result;
    MEKF_Config_t           *mekf_cfg;
    float                   *mekf_bias_dps;   /* float[3] */
    MotorMixer_Config_t     *mixer_cfg;
    USB_Command_Hooks_t      hooks;
} USB_Command_Bindings_t;

void USB_Command_Init(const USB_Command_Bindings_t *bindings);

/* Safe to call from the USB interrupt (CDC_Receive_FS). Only fills a ring. */
void USB_Command_Feed(const uint8_t *data, uint32_t len);

/* Call every main-loop iteration: parses commands, sends queued replies. */
void USB_Command_Process(void);

/* Queue one text line (CRLF appended) for transmission. Dropped if full. */
void USB_Command_Notify(const char *line);

/* True when a parameter was changed with SET and not saved/loaded since. */
bool USB_Command_IsDirty(void);
void USB_Command_ClearDirty(void);

/* Number of parameters in the table (for tests / diagnostics). */
uint32_t USB_Command_ParamCount(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMMAND_H */
