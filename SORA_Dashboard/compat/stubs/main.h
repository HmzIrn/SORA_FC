#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef enum {HAL_OK=0,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT}HAL_StatusTypeDef;
typedef struct{int x;}I2C_HandleTypeDef; extern I2C_HandleTypeDef hi2c1;
typedef struct{int x;}GPIO_TypeDef; extern GPIO_TypeDef *GPIOB,*GPIOC;
#define GPIO_PIN_0 1
#define GPIO_PIN_13 2
typedef enum{GPIO_PIN_RESET=0,GPIO_PIN_SET}GPIO_PinState;
uint32_t HAL_GetTick(void); void HAL_Init(void); void Error_Handler(void); void HAL_Delay(uint32_t);
void NVIC_SystemReset(void); void __disable_irq(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*,uint16_t); void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
typedef struct{uint32_t OscillatorType,HSEState;struct{uint32_t PLLState,PLLSource,PLLM,PLLN,PLLP,PLLQ;}PLL;}RCC_OscInitTypeDef;
typedef struct{uint32_t ClockType,SYSCLKSource,AHBCLKDivider,APB1CLKDivider,APB2CLKDivider;}RCC_ClkInitTypeDef;
#define RCC_OSCILLATORTYPE_HSE 1
#define RCC_HSE_ON 1
#define RCC_PLL_ON 1
#define RCC_PLLSOURCE_HSE 1
#define RCC_PLLP_DIV4 1
#define RCC_CLOCKTYPE_HCLK 1
#define RCC_CLOCKTYPE_SYSCLK 1
#define RCC_CLOCKTYPE_PCLK1 1
#define RCC_CLOCKTYPE_PCLK2 1
#define RCC_SYSCLKSOURCE_PLLCLK 1
#define RCC_SYSCLK_DIV1 1
#define RCC_HCLK_DIV2 1
#define RCC_HCLK_DIV1 1
#define FLASH_LATENCY_2 2
#define PWR_REGULATOR_VOLTAGE_SCALE2 2
#define __HAL_RCC_PWR_CLK_ENABLE()
#define __HAL_PWR_VOLTAGESCALING_CONFIG(x)
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef*); HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef*,uint32_t);
extern uint32_t SystemCoreClock;
typedef struct{uint32_t DEMCR;}CoreDebug_t; typedef struct{uint32_t CYCCNT,CTRL;}DWT_t;
extern CoreDebug_t *CoreDebug; extern DWT_t *DWT;
#define CoreDebug_DEMCR_TRCENA_Msk 1
#define DWT_CTRL_CYCCNTENA_Msk 1
/* flash HAL */
typedef struct{uint32_t TypeErase,Banks,Sector,NbSectors,VoltageRange;}FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0
#define FLASH_VOLTAGE_RANGE_3 2
#define FLASH_TYPEPROGRAM_WORD 2
#define FLASH_FLAG_EOP 1
#define FLASH_FLAG_OPERR 2
#define FLASH_FLAG_WRPERR 4
#define FLASH_FLAG_PGAERR 8
#define FLASH_FLAG_PGPERR 16
#define FLASH_FLAG_PGSERR 32
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)(x))
HAL_StatusTypeDef HAL_FLASH_Unlock(void); HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,uint32_t address,uint64_t data);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef*,uint32_t*);
