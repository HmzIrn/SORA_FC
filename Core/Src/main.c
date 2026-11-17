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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAL_BUTTON_PORT GPIOB
#define CAL_BUTTON_PIN  GPIO_PIN_0

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void StatusLED_Update(void);
static void CalibrationButton_Update(void);
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

  /* Start after sensor warm-up, with the board stationary. */
  IMU_Calibration_Start(&imu_cal);
  USB_Telemetry_Init();
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
          }

          /* Send corrected IMU data at configured telemetry rate */
          uint32_t now = HAL_GetTick();

          if (imu_calibration_ready &&
              (uint32_t)(now - last_telemetry_ms) >=
              TELEMETRY_INTERVAL_MS)
          {
              last_telemetry_ms = now;

              USB_Telemetry_SendIMU(&mpu_corrected);
          }
      }

      /* Button handling must run independently of DMA completion */
      CalibrationButton_Update();

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
