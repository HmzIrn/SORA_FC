#pragma once
#include "usbd_cdc.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
void MX_USB_DEVICE_Init(void);
void MX_GPIO_Init(void);void MX_DMA_Init(void);void MX_I2C1_Init(void);
