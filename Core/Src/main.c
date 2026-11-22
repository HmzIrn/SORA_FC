/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "imu_calibration.h"
#include "usb_telemetry.h"
#include "mekf.h"
#include "motor_mixer.h"
#include "config_store.h"
#include "usb_command.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    CONFIG_SOURCE_NONE = 0,      /* not calibrated yet                    */
    CONFIG_SOURCE_FLASH,         /* loaded from flash at boot             */
    CONFIG_SOURCE_CALIBRATED     /* calibrated in this session            */
} ConfigSource_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAL_BUTTON_PORT GPIOB
#define CAL_BUTTON_PIN  GPIO_PIN_0

/* Samples further apart than this are treated as a data gap (seconds). */
#define ATTITUDE_MAX_DT_S   (0.1f)

/* Compute the motor mixer from the attitude estimate (1) or hold it off (0).
 * The mixer output is only calculated and telemetered; nothing here drives
 * the motors. */
#define MIXER_ENABLED       (1)

/* Boot with stored data: average this much live accelerometer data to level
 * the attitude filter before it starts. */
#define BOOT_AVG_MIN_MS       (200U)
#define BOOT_AVG_MIN_SAMPLES  (20U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_Handle_t hmpu;

volatile uint8_t mpu_drdy_flag = 0;

MPU6050_RawData_t mpu_raw;
MPU6050_Data_t mpu_data;       // Raw parsed measurements
MPU6050_Data_t mpu_corrected;  // Calibrated measurements

IMU_Calibration_t imu_cal;
IMU_CalibrationResult_t imu_cal_result;

float gyro_corrected_dps[3];
float accel_corrected_g[3];

uint8_t imu_calibration_ready = 0;

static uint8_t button_previous = 1;
static uint8_t button_stable = 1;
static uint32_t button_last_change = 0;

static uint8_t next_face = 0;
static uint8_t selected_face = 0;

/* Attitude estimation + motor mixing */
MEKF_t ekf;
MotorMixer_t mixer;
Telemetry_Data_t telemetry;

/* Persistent configuration / calibration sequencing */
static ConfigSource_t config_source = CONFIG_SOURCE_NONE;
static float    mekf_bias_dps[3];            /* residual gyro bias (persisted) */
static uint8_t  save_after_mekf_cal = 0;     /* capture bias + save when MEKF done */
static uint8_t  mekf_cal_bias_only = 0;
static uint8_t  boot_avg_pending = 0;
static uint32_t boot_avg_count = 0;
static uint32_t boot_avg_start_ms = 0;
static float    boot_avg_sum[3];

static uint8_t  att_timebase_valid = 0;
static uint32_t att_last_cycle = 0;
static uint32_t att_last_tick = 0;

/*
 * Sensor frame -> body frame (FRD: x forward, y right, z down).
 * body = SENSOR_TO_BODY * sensor.
 * Default: sensor x forward, y left, z up (accelerometer reads +1 g on Z
 * when level), which is diag(1,-1,-1). CHANGE THIS to match how the board
 * is mounted in the airframe.
 */
static const float SENSOR_TO_BODY[3][3] =
{
    {  1.0f,  0.0f,  0.0f },
    {  0.0f, -1.0f,  0.0f },
    {  0.0f,  0.0f, -1.0f }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void StatusLED_Update(void);
static void CalibrationButton_Update(void);
static void SensorToBody(const float s[3], float b[3]);
static void Attitude_StartTimebase(void);
static void Config_ApplyValues(const Config_Payload_t *p);
static bool Config_Save(void);
static bool Calibration_IsBusy(void);
static bool Calibration_StartFull(void);
static bool Calibration_StartBias(void);
static void Calibration_OnImuComplete(void);
static void Calibration_OnMekfComplete(void);
static void Attitude_BootAverage(const float accel_g[3]);
static bool Hook_Save(void);
static bool Hook_Load(void);
static bool Hook_Clear(void);
static void Hook_Reboot(void);
static void Hook_ParamChanged(const USB_Param_t *param);
static int  Hook_FormatStatus(char *buf, size_t size);
static void Attitude_Update(const float gyro_dps[3], const float accel_g[3]);
static void Telemetry_Fill(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == hmpu.hi2c) {
        MPU6050_RxCompleteCallback(&hmpu);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == hmpu.hi2c) {
        MPU6050_ErrorCallback(&hmpu);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        mpu_drdy_flag = 1;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  if (MPU6050_Init(&hmpu, &hi2c1, MPU6050_ADDR_LOW) != HAL_OK) {
      Error_Handler();
  }
  IMU_Calibration_Init(
      &imu_cal,
      IMU_CAL_DEFAULT_GYRO_SAMPLES,
      IMU_CAL_DEFAULT_ACCEL_SAMPLES,
      IMU_CAL_GRAVITY_G
  );

  USB_Telemetry_Init();

  /* Free-running cycle counter used for the filter's dt. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Defaults from dash.py; tune on hardware. */
  MEKF_Init(&ekf, NULL);
  MotorMixer_Init(&mixer, NULL);

  /* Command interface (parameter table points at the live objects). */
  {
      const USB_Command_Bindings_t bindings =
      {
          .imu_result    = &imu_cal_result,
          .mekf_cfg      = &ekf.cfg,
          .mekf_bias_dps = mekf_bias_dps,
          .mixer_cfg     = &mixer.cfg,
          .hooks =
          {
              .start_full_calibration = Calibration_StartFull,
              .start_bias_calibration = Calibration_StartBias,
              .save                   = Hook_Save,
              .load                   = Hook_Load,
              .clear                  = Hook_Clear,
              .reboot                 = Hook_Reboot,
              .busy                   = Calibration_IsBusy,
              .param_changed          = Hook_ParamChanged,
              .format_status          = Hook_FormatStatus
          }
      };

      USB_Command_Init(&bindings);
  }

  /*
   * Stored calibration found -> use it (no six-face procedure).
   * Otherwise run the full calibration with the board stationary.
   */
  {
      Config_Payload_t stored;

      if (ConfigStore_Load(&stored) &&
          IMU_Calibration_LoadResult(&imu_cal, &stored.imu))
      {
          Config_ApplyValues(&stored);
          imu_calibration_ready = 1;
          config_source = CONFIG_SOURCE_FLASH;
          boot_avg_pending = 1;
          boot_avg_count = 0;
          USB_Command_Notify("CFG,LOADED,using stored calibration; send CAL FULL to recalibrate");
      }
      else
      {
          /* Start after sensor warm-up, with the board stationary. */
          IMU_Calibration_Start(&imu_cal);
          USB_Command_Notify("CFG,EMPTY,no stored calibration; starting full calibration");
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  static uint32_t last_telemetry_ms = 0;

  while (1)
  {
      /* MPU6050 data-ready interrupt */
      if (mpu_drdy_flag && !hmpu.transfer_busy)
      {
          mpu_drdy_flag = 0;

          if (MPU6050_Read_DMA(&hmpu) != HAL_OK)
          {
              // Handle acquisition start failure.
          }
      }

      /* Process completed DMA transfer */
      if (hmpu.data_ready)
      {
          hmpu.data_ready = 0;

          MPU6050_ParseData(&hmpu, &mpu_raw, &mpu_data);

          /* Calibration phase */
          if (!imu_calibration_ready)
          {
              bool calibration_event = IMU_Calibration_Update(
                  &imu_cal,
                  mpu_data.gyro_x_dps,
                  mpu_data.gyro_y_dps,
                  mpu_data.gyro_z_dps,
                  mpu_data.accel_x_g,
                  mpu_data.accel_y_g,
                  mpu_data.accel_z_g
              );

              if (calibration_event &&
                  IMU_Calibration_GetStatus(&imu_cal) ==
                  IMU_CAL_STATUS_COMPLETE)
              {
                  if (IMU_Calibration_GetResult(
                          &imu_cal,
                          &imu_cal_result))
                  {
                      imu_calibration_ready = 1;
                      Calibration_OnImuComplete();
                  }
              }
          }

          /* Prepare corrected measurement object */
          if (imu_calibration_ready)
          {
              const float gyro_raw[3] =
              {
                  mpu_data.gyro_x_dps,
                  mpu_data.gyro_y_dps,
                  mpu_data.gyro_z_dps
              };

              const float accel_raw[3] =
              {
                  mpu_data.accel_x_g,
                  mpu_data.accel_y_g,
                  mpu_data.accel_z_g
              };

              IMU_Calibration_Apply(
                  &imu_cal_result,
                  gyro_raw,
                  accel_raw,
                  gyro_corrected_dps,
                  accel_corrected_g
              );

              // Copy the original measurement structure.
              // Temperature and other fields remain unchanged.
              mpu_corrected = mpu_data;

              // Replace sensor measurements with calibrated values.
              mpu_corrected.gyro_x_dps = gyro_corrected_dps[0];
              mpu_corrected.gyro_y_dps = gyro_corrected_dps[1];
              mpu_corrected.gyro_z_dps = gyro_corrected_dps[2];

              mpu_corrected.accel_x_g = accel_corrected_g[0];
              mpu_corrected.accel_y_g = accel_corrected_g[1];
              mpu_corrected.accel_z_g = accel_corrected_g[2];

              /* Attitude estimate and motor response for this sample */
              Attitude_Update(gyro_corrected_dps, accel_corrected_g);
          }

          /* Send corrected IMU data at configured telemetry rate */
          uint32_t now = HAL_GetTick();

          if (imu_calibration_ready &&
              (uint32_t)(now - last_telemetry_ms) >=
              TELEMETRY_INTERVAL_MS)
          {
              last_telemetry_ms = now;

              Telemetry_Fill(now);
              USB_Telemetry_Send(&telemetry);
          }
      }

      /* Button handling must run independently of DMA completion */
      CalibrationButton_Update();

      /* USB command interface (parse commands, send queued replies) */
      USB_Command_Process();

      /* Calibration status LED */
      StatusLED_Update();

      /* Calibration instructions over USB */
      IMU_CalibrationStage_t stage =
          IMU_Calibration_GetStage(&imu_cal);

      uint8_t face_index =
          (stage == IMU_CAL_STAGE_ACCEL_FACE)
          ? selected_face
          : next_face;

      USB_Telemetry_SendCalibrationStatus(
          IMU_Calibration_GetStatus(&imu_cal),
          stage,
          face_index
      );

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void SensorToBody(const float s[3], float b[3])
{
    for (uint8_t i = 0U; i < 3U; i++)
    {
        b[i] = SENSOR_TO_BODY[i][0] * s[0] +
               SENSOR_TO_BODY[i][1] * s[1] +
               SENSOR_TO_BODY[i][2] * s[2];
    }
}

static void Attitude_StartTimebase(void)
{
    att_timebase_valid = 0;
}

/* ------------------------------------------------------------------------ */
/* Persistent configuration                                                 */
/* ------------------------------------------------------------------------ */

static void Config_Gather(Config_Payload_t *p)
{
    memset(p, 0, sizeof(*p));

    p->imu = imu_cal_result;
    p->mekf = ekf.cfg;
    p->mixer = mixer.cfg;
    memcpy(p->mekf_bias_dps, mekf_bias_dps, sizeof(mekf_bias_dps));
}

/* Push stored values into the live objects (no state machine changes). */
static void Config_ApplyValues(const Config_Payload_t *p)
{
    float bg[3];

    imu_cal_result = p->imu;
    MEKF_SetConfig(&ekf, &p->mekf);
    MotorMixer_SetConfig(&mixer, &p->mixer);
    memcpy(mekf_bias_dps, p->mekf_bias_dps, sizeof(mekf_bias_dps));

    for (uint8_t i = 0U; i < 3U; i++)
        bg[i] = mekf_bias_dps[i] * MEKF_DEG_TO_RAD;

    MEKF_SetGyroBias(&ekf, bg);
}

static bool Config_Save(void)
{
    Config_Payload_t p;
    ConfigStore_Info_t info;
    char msg[64];

    Config_Gather(&p);

    if (!ConfigStore_Save(&p))
    {
        USB_Command_Notify("ERR,SAVE_FAILED,flash write failed or storage unavailable");
        return false;
    }

    USB_Command_ClearDirty();
    ConfigStore_GetInfo(&info);
    snprintf(msg, sizeof(msg), "CFG,SAVED,seq=%lu", (unsigned long)info.sequence);
    USB_Command_Notify(msg);

    return true;
}

/* ------------------------------------------------------------------------ */
/* Calibration sequencing                                                   */
/*                                                                          */
/*  CAL FULL / first boot:  IMU six-face calibration (button)               */
/*                          -> MEKF stationary calibration (bias + level)   */
/*                          -> bias captured, everything saved to flash     */
/*  CAL BIAS:               MEKF bias-only calibration -> saved             */
/*  Boot with stored data:  average ~0.2 s of live accel to level the       */
/*                          filter, start with the stored bias              */
/* ------------------------------------------------------------------------ */

static bool Calibration_IsBusy(void)
{
    return (!imu_calibration_ready) ||
           (IMU_Calibration_GetStatus(&imu_cal) == IMU_CAL_STATUS_COLLECTING) ||
           save_after_mekf_cal ||
           boot_avg_pending;
}

static bool Calibration_StartFull(void)
{
    imu_calibration_ready = 0;
    boot_avg_pending = 0;
    save_after_mekf_cal = 0;
    mekf_cal_bias_only = 0;
    next_face = 0;
    selected_face = 0;
    config_source = CONFIG_SOURCE_NONE;

    IMU_Calibration_Init(&imu_cal,
                         IMU_CAL_DEFAULT_GYRO_SAMPLES,
                         IMU_CAL_DEFAULT_ACCEL_SAMPLES,
                         IMU_CAL_GRAVITY_G);
    IMU_Calibration_Start(&imu_cal);

    MotorMixer_Reset(&mixer);
    return true;
}

static bool Calibration_StartBias(void)
{
    if (!imu_calibration_ready || boot_avg_pending ||
        IMU_Calibration_GetStatus(&imu_cal) != IMU_CAL_STATUS_COMPLETE)
    {
        return false;
    }

    mekf_cal_bias_only = 1;
    save_after_mekf_cal = 1;

    MEKF_RecalibrateBias(&ekf);
    MotorMixer_Reset(&mixer);
    Attitude_StartTimebase();
    return true;
}

/* IMU six-face calibration finished: now level + bias-calibrate the filter. */
static void Calibration_OnImuComplete(void)
{
    config_source = CONFIG_SOURCE_CALIBRATED;
    mekf_cal_bias_only = 0;
    save_after_mekf_cal = 1;
    memset(mekf_bias_dps, 0, sizeof(mekf_bias_dps));

    MEKF_Reinitialize(&ekf);
    MotorMixer_Reset(&mixer);
    MotorMixer_SetYawHold(&mixer, 0.0f);
    Attitude_StartTimebase();

    USB_Command_Notify("CAL,MEKF,Place the board level and keep it still");
}

/* The filter finished its stationary calibration: capture bias, save. */
static void Calibration_OnMekfComplete(void)
{
    const bool with_motion =
        (MEKF_GetEvent(&ekf) == MEKF_EVENT_RUNNING_CALIBRATED_WITH_MOTION);

    save_after_mekf_cal = 0;

    if (mekf_cal_bias_only && with_motion)
    {
        float old_bg[3];

        for (uint8_t i = 0U; i < 3U; i++)
            old_bg[i] = mekf_bias_dps[i] * MEKF_DEG_TO_RAD;

        MEKF_SetGyroBias(&ekf, old_bg);
        USB_Command_Notify("ERR,CAL_BIAS,motion detected; previous bias kept. Retry with the board still");
        return;
    }

    float bg[3];

    MEKF_GetGyroBias(&ekf, bg);

    for (uint8_t i = 0U; i < 3U; i++)
        mekf_bias_dps[i] = bg[i] * MEKF_RAD_TO_DEG;

    if (with_motion)
        USB_Command_Notify("WARN,CAL,motion during the bias estimate; run CAL BIAS when stationary");

    (void)Config_Save();
}

/* Boot with stored data: level the filter from live accelerometer data. */
static void Attitude_BootAverage(const float accel_g[3])
{
    float accel_body[3];
    float mean[3];
    float bias_rads[3];

    SensorToBody(accel_g, accel_body);

    const uint32_t now = HAL_GetTick();

    if (boot_avg_count == 0U)
    {
        boot_avg_start_ms = now;
        memset(boot_avg_sum, 0, sizeof(boot_avg_sum));
    }

    for (uint8_t i = 0U; i < 3U; i++)
        boot_avg_sum[i] += accel_body[i];

    boot_avg_count++;

    if ((uint32_t)(now - boot_avg_start_ms) < BOOT_AVG_MIN_MS ||
        boot_avg_count < BOOT_AVG_MIN_SAMPLES)
    {
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        mean[i] = boot_avg_sum[i] / (float)boot_avg_count;
        bias_rads[i] = mekf_bias_dps[i] * MEKF_DEG_TO_RAD;
    }

    boot_avg_pending = 0;

    if (!MEKF_StartWithKnownBias(&ekf, bias_rads, mean))
    {
        /* Fall back to the filter's own stationary calibration. */
        MEKF_Reinitialize(&ekf);
    }

    MotorMixer_Reset(&mixer);
    MotorMixer_SetYawHold(&mixer, 0.0f);
    Attitude_StartTimebase();
}

/* ------------------------------------------------------------------------ */
/* Command hooks                                                            */
/* ------------------------------------------------------------------------ */

static bool Hook_Save(void)
{
    return Config_Save();
}

static bool Hook_Load(void)
{
    Config_Payload_t p;

    if (!ConfigStore_Load(&p))
        return false;

    Config_ApplyValues(&p);
    return true;
}

static bool Hook_Clear(void)
{
    return ConfigStore_Erase();
}

static void Hook_Reboot(void)
{
    HAL_Delay(50U);   /* let the reply leave the USB FIFO */
    NVIC_SystemReset();
}

static void Hook_ParamChanged(const USB_Param_t *param)
{
    if (param->apply == USB_PARAM_APPLY_MEKF_BIAS)
    {
        float bg[3];

        for (uint8_t i = 0U; i < 3U; i++)
            bg[i] = mekf_bias_dps[i] * MEKF_DEG_TO_RAD;

        MEKF_SetGyroBias(&ekf, bg);
    }
}

static int Hook_FormatStatus(char *buf, size_t size)
{
    static const char *const status_names[] =
        { "idle", "collecting", "complete", "error" };
    static const char *const source_names[] =
        { "none", "flash", "calibrated" };

    ConfigStore_Info_t info;
    const unsigned int st = (unsigned int)IMU_Calibration_GetStatus(&imu_cal);

    ConfigStore_GetInfo(&info);

    return snprintf(buf, size,
        "STATUS,imu_cal=%s,source=%s,mekf=%s,dirty=%u,store=%s,seq=%lu,slots=%lu/%lu",
        (st < 4U) ? status_names[st] : "?",
        source_names[(unsigned int)config_source],
        MEKF_IsRunning(&ekf) ? "running" : "calibrating",
        USB_Command_IsDirty() ? 1U : 0U,
        !info.available ? "unavailable" : (info.valid ? "ok" : "empty"),
        (unsigned long)info.sequence,
        (unsigned long)info.slots_used,
        (unsigned long)info.slot_count);
}

/*
 * Run the MEKF and the motor mixer on one calibrated sample.
 * Inputs are in the sensor frame: gyro in deg/s, accel in g.
 */
static void Attitude_Update(const float gyro_dps[3], const float accel_g[3])
{
    /* Boot with stored calibration: level the filter first. */
    if (boot_avg_pending)
    {
        Attitude_BootAverage(accel_g);
        return;
    }

    const uint32_t cycles = DWT->CYCCNT;
    const uint32_t tick = HAL_GetTick();

    if (!att_timebase_valid)
    {
        att_last_cycle = cycles;
        att_last_tick = tick;
        att_timebase_valid = 1;
        return;
    }

    const float dt_s =
        (float)(uint32_t)(cycles - att_last_cycle) / (float)SystemCoreClock;
    const float tick_gap_s =
        (float)(uint32_t)(tick - att_last_tick) * 0.001f;

    att_last_cycle = cycles;
    att_last_tick = tick;

    /* Data gap: add process noise instead of propagating on a stale dt. */
    if (dt_s > ATTITUDE_MAX_DT_S || tick_gap_s > ATTITUDE_MAX_DT_S)
    {
        MEKF_InflateCovariance(&ekf,
                               (dt_s > tick_gap_s) ? dt_s : tick_gap_s);
        MotorMixer_Reset(&mixer);
        return;
    }

    if (dt_s <= 0.0f)
        return;

    float gyro_sensor_rads[3];
    float gyro_body[3];
    float accel_body[3];

    for (uint8_t i = 0U; i < 3U; i++)
        gyro_sensor_rads[i] = gyro_dps[i] * MEKF_DEG_TO_RAD;

    SensorToBody(gyro_sensor_rads, gyro_body);
    SensorToBody(accel_g, accel_body);

    (void)MEKF_Process(&ekf, gyro_body, accel_body, dt_s);

    /* Stationary MEKF calibration finished: store bias + configuration. */
    if (save_after_mekf_cal && MEKF_IsRunning(&ekf))
        Calibration_OnMekfComplete();

    /* Body rates with the filter's residual bias removed. */
    float omega[3];

    for (uint8_t i = 0U; i < 3U; i++)
        omega[i] = gyro_body[i] - ekf.bg[i];

    (void)MotorMixer_Update(&mixer,
                            ekf.q,
                            omega,
                            dt_s,
                            (MIXER_ENABLED != 0),
                            MEKF_IsRunning(&ekf));
}

static void Telemetry_Fill(uint32_t now)
{
    telemetry.timestamp_ms = now;
    telemetry.imu = mpu_corrected;

    telemetry.filter_state = MEKF_GetState(&ekf);
    telemetry.filter_event = MEKF_GetEvent(&ekf);

    MEKF_GetQuaternion(&ekf, telemetry.q);
    MEKF_GetEuler(&ekf,
                  &telemetry.euler_rad[0],
                  &telemetry.euler_rad[1],
                  &telemetry.euler_rad[2]);
    MEKF_GetGyroBias(&ekf, telemetry.gyro_bias_rads);

    telemetry.motors = mixer.out;
}


static void StatusLED_Update(void)
{
    static uint32_t last_toggle = 0;
    static uint8_t led_state = 0;

    uint32_t now = HAL_GetTick();

    IMU_CalibrationStatus_t status =
        IMU_Calibration_GetStatus(&imu_cal);

    IMU_CalibrationStage_t stage =
        IMU_Calibration_GetStage(&imu_cal);

    uint32_t interval = 0;
    uint8_t waiting_for_face = 0;

    switch (status)
    {
        case IMU_CAL_STATUS_IDLE:
            led_state = 0;
            break;

        case IMU_CAL_STATUS_COMPLETE:
            led_state = 1;
            break;

        case IMU_CAL_STATUS_ERROR:
            interval = 100;
            break;

        case IMU_CAL_STATUS_COLLECTING:
            switch (stage)
            {
                case IMU_CAL_STAGE_STATIC:
                    interval = 250;
                    break;

                case IMU_CAL_STAGE_WAIT_ACCEL_FACE:
                    waiting_for_face = 1;
                    break;

                case IMU_CAL_STAGE_ACCEL_FACE:
                    interval = 100;
                    break;

                case IMU_CAL_STAGE_COMPLETE:
                    led_state = 1;
                    break;

                case IMU_CAL_STAGE_ERROR:
                    interval = 100;
                    break;

                default:
                    led_state = 0;
                    break;
            }
            break;

        default:
            led_state = 0;
            break;
    }

    if (waiting_for_face)
    {
        // Short ON pulse every 1 second.
        led_state = ((now % 1000U) < 100U);
    }
    else if (interval > 0)
    {
        if ((uint32_t)(now - last_toggle) >= interval)
        {
            last_toggle = now;
            led_state = !led_state;
        }
    }
    else
    {
        last_toggle = now;
    }

    // PC13 is active-low.
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_13,
        led_state ? GPIO_PIN_RESET : GPIO_PIN_SET
    );
}

static void CalibrationButton_Update(void)
{
    uint8_t current =
        HAL_GPIO_ReadPin(CAL_BUTTON_PORT, CAL_BUTTON_PIN);

    uint32_t now = HAL_GetTick();

    if (current != button_previous)
    {
        button_previous = current;
        button_last_change = now;
    }

    // 30 ms debounce
    if ((uint32_t)(now - button_last_change) >= 30)
    {
        if (current != button_stable)
        {
            button_stable = current;

            // Falling edge: button pressed
            if (button_stable == GPIO_PIN_RESET)
            {
                if (IMU_Calibration_GetStage(&imu_cal) ==
                    IMU_CAL_STAGE_WAIT_ACCEL_FACE)
                {

                	if (next_face < IMU_ACCEL_FACE_COUNT)
                	{
                	    if (IMU_Calibration_SetAccelFace(
                	            &imu_cal,
                	            (IMU_AccelCalibrationFace_t)next_face))
                	    {
                	        selected_face = next_face;
                	        next_face++;
                	    }
                	}
                }
            }
        }
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
