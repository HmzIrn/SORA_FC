#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_calibration.h"
#include "mekf.h"
#include "motor_mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Persistent configuration in on-chip flash (STM32F401 has no EEPROM).
 *
 * The LAST flash sector is used as an append-only log of fixed-size records.
 * Every save programs the next free 256-byte slot; the sector is only
 * erased when all slots are used up (128 KB sector -> 512 saves per erase),
 * so saves are fast (~1 ms) and wear is spread out. On load, the valid
 * record (magic, version, size, CRC-32) with the highest sequence number
 * wins, so a power loss in the middle of a save simply falls back to the
 * previous record.
 *
 * SAFETY / SETUP
 *  - The sector is chosen from the flash-size register (128K: sector 4,
 *    256K: 5, 384K: 6, 512K: 7). Make sure firmware never reaches it;
 *    ideally shrink FLASH LENGTH in the linker script by that sector so
 *    the linker enforces it. At run time the store refuses to operate if
 *    the firmware image (_sidata + .data size) overlaps the sector.
 *  - Override with CONFIG_STORE_SECTOR_NUMBER / _ADDR / _SIZE if needed.
 *  - Erasing a sector stalls the CPU for ~1-2 s, but that only happens once
 *    every 512 saves (and on ConfigStore_Erase()). Do not save in flight.
 *  - Changing any struct below changes its size; old records are then
 *    ignored (CONFIG_STORE_VERSION mismatch or size mismatch) and the
 *    device falls back to a fresh calibration. Bump the version when you
 *    change the meaning of a field.
 */

#define CONFIG_STORE_VERSION      (1U)
#define CONFIG_STORE_RECORD_SIZE  (256U)

/* Everything that is persisted. */
typedef struct
{
    IMU_CalibrationResult_t imu;    /* gyro bias, accel offset/scale + stats  */
    MEKF_Config_t           mekf;   /* filter tuning                          */
    MotorMixer_Config_t     mixer;  /* PID gains, limits, trims               */
    float mekf_bias_dps[3];         /* residual gyro bias from MEKF bias cal  */
} Config_Payload_t;

typedef struct
{
    bool     available;    /* sector located and safe to use                  */
    bool     valid;        /* at least one valid record present               */
    uint32_t sequence;     /* sequence number of the newest valid record      */
    uint32_t slot;         /* slot index of the newest valid record           */
    uint32_t slots_used;   /* slots consumed since the last erase             */
    uint32_t slot_count;   /* total slots in the sector                       */
} ConfigStore_Info_t;

/* False when no usable sector (unknown flash size / image overlaps). */
bool ConfigStore_IsAvailable(void);

/* Newest valid record -> *payload. False if none (blank/corrupt/old). */
bool ConfigStore_Load(Config_Payload_t *payload);

/* Append a new record. False on flash error or verify mismatch. */
bool ConfigStore_Save(const Config_Payload_t *payload);

/* Erase the whole sector (all stored configuration is lost). */
bool ConfigStore_Erase(void);

void ConfigStore_GetInfo(ConfigStore_Info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STORE_H */
