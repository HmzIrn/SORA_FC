/*
 * fw_sim - the real flight-controller firmware (main.c main loop, usb_command, usb_telemetry,
 * config_store, mekf, motor_mixer, imu_calibration) compiled for the host and wired to
 *   stdin  = USB CDC OUT  (bytes the host sends; goes into USB_Command_Feed like CDC_Receive_FS does)
 *   stdout = USB CDC IN   (what CDC_Transmit_FS sends to the host)
 * A simulated IMU (with offsets, scale errors and gyro bias) and a simulated user (who orients the six
 * faces and presses the button when asked) close the loop, so the firmware behaves as on the bench.
 * Flash is a file (FW_SIM_FLASH) and survives "reboots" (NVIC_SystemReset re-executes the process).
 *
 * Extra simulator-only control lines on stdin (never reach the firmware):
 *     !tilt <x_deg> <y_deg>        pose the board is held in after calibration
 *     !gbias <x> <y> <z>           true gyro bias of the simulated sensor [dps]
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include "usbd_cdc_if.h"

#define main fw_main
#include "main.c"
#undef main

#define SIM_BASE 0x08060000UL
#define SIM_SIZE 2048U
uint8_t sim_flash[SIM_SIZE];
static char **g_argv;
static const char *flash_path(void) { const char *p = getenv("FW_SIM_FLASH"); return p ? p : "fw_flash.bin"; }
static void flash_dump(void) { FILE *f = fopen(flash_path(), "wb"); if (f) { fwrite(sim_flash, 1, SIM_SIZE, f); fclose(f); } }
static void flash_load(void) { memset(sim_flash, 0xFF, SIM_SIZE); FILE *f = fopen(flash_path(), "rb"); if (f) { if (fread(sim_flash, 1, SIM_SIZE, f) != SIM_SIZE) memset(sim_flash, 0xFF, SIM_SIZE); fclose(f); } }

I2C_HandleTypeDef hi2c1; GPIO_TypeDef gb, gc; GPIO_TypeDef *GPIOB = &gb, *GPIOC = &gc;
static uint32_t g_tick; uint32_t HAL_GetTick(void) { return g_tick; }
void HAL_Init(void) {} void HAL_Delay(uint32_t ms) { g_tick += ms; }
void NVIC_SystemReset(void) { flash_dump(); execv("/proc/self/exe", g_argv); _exit(1); }
void __disable_irq(void) {}
void HAL_GPIO_WritePin(GPIO_TypeDef *a, uint16_t b, GPIO_PinState c) { (void)a; (void)b; (void)c; }
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *a) { (void)a; return HAL_OK; }
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *a, uint32_t b) { (void)a; (void)b; return HAL_OK; }
uint32_t SystemCoreClock = 84000000; static CoreDebug_t cd; static DWT_t dw;
CoreDebug_t *CoreDebug = &cd; DWT_t *DWT = &dw;
USBD_HandleTypeDef hUsbDeviceFS; static USBD_CDC_HandleTypeDef hc;
void MX_USB_DEVICE_Init(void) {} void MX_GPIO_Init(void) {} void MX_DMA_Init(void) {} void MX_I2C1_Init(void) {}
HAL_StatusTypeDef MPU6050_Init(MPU6050_Handle_t *a, I2C_HandleTypeDef *b, uint8_t c) { (void)a; (void)b; (void)c; return HAL_OK; }
HAL_StatusTypeDef MPU6050_Read_DMA(MPU6050_Handle_t *a) { (void)a; return HAL_OK; }
void MPU6050_RxCompleteCallback(MPU6050_Handle_t *a) { (void)a; }
void MPU6050_ErrorCallback(MPU6050_Handle_t *a) { (void)a; }

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { flash_dump(); return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t t, uint32_t a, uint64_t d)
{
    (void)t; uint32_t off = a - (uint32_t)SIM_BASE, cur, nv = (uint32_t)d;
    memcpy(&cur, &sim_flash[off], 4);
    if ((cur & nv) != nv) return HAL_ERROR;        /* programming over non-erased flash */
    cur &= nv; memcpy(&sim_flash[off], &cur, 4); return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *x) { (void)e; (void)x; memset(sim_flash, 0xFF, SIM_SIZE); return HAL_OK; }

uint8_t CDC_Transmit_FS(uint8_t *b, uint16_t l)
{
    size_t off = 0; while (off < l) { ssize_t n = write(1, b + off, l - off); if (n <= 0) _exit(0); off += (size_t)n; }
    return USBD_OK;
}

/* ---------------------------------------------------------------- simulated IMU + user */
static float TRUE_GYRO_BIAS[3] = { 0.55f, -0.32f, 0.21f };
static const float TRUE_ACC_OFF[3] = { 0.020f, -0.015f, 0.030f };
static const float TRUE_ACC_SCL[3] = { 1.012f, 0.991f, 1.020f };
static uint32_t rng = 12345u;
static float urand(void) { rng = rng * 1664525u + 1013904223u; return ((rng >> 8) & 0xFFFFFF) / 16777216.0f; }
static float nrand(void) { float u1 = urand() + 1e-7f, u2 = urand(); return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2); }
static float pose[3] = { 0, 0, 1 }, tilt_pose[3] = { 0.1736f, -0.0872f, 0.9809f };
static int pressing = 0, press_left = 0, settle = 0;
static const float FACE_POSE[6][3] = { { 1,0,0 }, { -1,0,0 }, { 0,1,0 }, { 0,-1,0 }, { 0,0,1 }, { 0,0,-1 } };
static MPU6050_Data_t cur;
static char inbuf[256]; static size_t inlen;
static long sleep_us = 200;

static void sim_command(char *line)
{
    float a, b, c;
    if (sscanf(line, "!tilt %f %f", &a, &b) == 2)
    {
        float ax = sinf(a * 0.0174532925f), ay = sinf(b * 0.0174532925f);
        tilt_pose[0] = ax; tilt_pose[1] = ay; tilt_pose[2] = sqrtf(1.0f - ax * ax - ay * ay);
    }
    else if (sscanf(line, "!gbias %f %f %f", &a, &b, &c) == 3) { TRUE_GYRO_BIAS[0] = a; TRUE_GYRO_BIAS[1] = b; TRUE_GYRO_BIAS[2] = c; }
}

static void poll_stdin(void)
{
    char tmp[128]; ssize_t n = read(0, tmp, sizeof tmp);
    if (n == 0) { flash_dump(); _exit(0); }
    for (ssize_t i = 0; i < n; i++)
    {
        char ch = tmp[i];
        if (inlen < sizeof inbuf - 2) inbuf[inlen++] = ch;
        if (ch == '\n')
        {
            inbuf[inlen] = 0;
            if (inbuf[0] == '!') sim_command(inbuf);
            else USB_Command_Feed((const uint8_t *)inbuf, (uint32_t)inlen);   /* what CDC_Receive_FS does */
            inlen = 0;
        }
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port; (void)pin;
    g_tick += 2; dw.CYCCNT += 168000;
    IMU_CalibrationStage_t st = IMU_Calibration_GetStage(&imu_cal);
    if (!imu_calibration_ready && IMU_Calibration_GetStatus(&imu_cal) == IMU_CAL_STATUS_COLLECTING)
    {
        if (st == IMU_CAL_STAGE_STATIC) { pose[0] = 0; pose[1] = 0; pose[2] = 1; }
        else if (st == IMU_CAL_STAGE_WAIT_ACCEL_FACE)
        {
            if (next_face < 6) memcpy(pose, FACE_POSE[next_face], sizeof pose);
            if (!pressing && ++settle >= 100) { pressing = 1; press_left = 40; settle = 0; }
        }
        else if (st == IMU_CAL_STAGE_ACCEL_FACE) { pressing = 0; settle = 0; }
    }
    else if (imu_calibration_ready) memcpy(pose, tilt_pose, sizeof pose);
    if (pressing && --press_left <= 0) pressing = 0;
    float *d[6] = { &cur.accel_x_g, &cur.accel_y_g, &cur.accel_z_g, &cur.gyro_x_dps, &cur.gyro_y_dps, &cur.gyro_z_dps };
    for (int i = 0; i < 3; i++)
    {
        *d[i] = pose[i] * TRUE_ACC_SCL[i] + TRUE_ACC_OFF[i] + 0.002f * nrand();
        *d[3 + i] = TRUE_GYRO_BIAS[i] + 0.03f * nrand();
    }
    cur.temperature_c = 27.0f;
    hmpu.data_ready = 1;
    hUsbDeviceFS.pClassData = &hc; hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED; hc.TxState = 0;
    poll_stdin();
    if (sleep_us > 0) usleep((useconds_t)sleep_us);
    return pressing ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
void MPU6050_ParseData(MPU6050_Handle_t *a, MPU6050_RawData_t *b, MPU6050_Data_t *c) { (void)a; (void)b; *c = cur; }

int main(int argc, char **argv)
{
    (void)argc; g_argv = argv;
    const char *s = getenv("FW_SIM_SLEEP_US"); if (s) sleep_us = atol(s);
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    flash_load();
    fw_main();
    return 0;
}
