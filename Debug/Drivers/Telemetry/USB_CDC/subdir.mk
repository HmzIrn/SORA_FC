################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Telemetry/USB_CDC/usb_telemetry.c 

OBJS += \
./Drivers/Telemetry/USB_CDC/usb_telemetry.o 

C_DEPS += \
./Drivers/Telemetry/USB_CDC/usb_telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Telemetry/USB_CDC/%.o Drivers/Telemetry/USB_CDC/%.su Drivers/Telemetry/USB_CDC/%.cyclo: ../Drivers/Telemetry/USB_CDC/%.c Drivers/Telemetry/USB_CDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/IMU/MPU6050" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/Telemetry/USB_CDC" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Calibration" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Telemetry-2f-USB_CDC

clean-Drivers-2f-Telemetry-2f-USB_CDC:
	-$(RM) ./Drivers/Telemetry/USB_CDC/usb_telemetry.cyclo ./Drivers/Telemetry/USB_CDC/usb_telemetry.d ./Drivers/Telemetry/USB_CDC/usb_telemetry.o ./Drivers/Telemetry/USB_CDC/usb_telemetry.su

.PHONY: clean-Drivers-2f-Telemetry-2f-USB_CDC

