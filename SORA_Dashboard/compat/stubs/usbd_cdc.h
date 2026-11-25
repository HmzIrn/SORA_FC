#pragma once
#include <stdint.h>
#define USBD_STATE_CONFIGURED 3
typedef struct{uint32_t TxState;}USBD_CDC_HandleTypeDef;
typedef struct{void*pClassData; uint8_t dev_state;}USBD_HandleTypeDef;
