################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/MEKF/mekf.c 

OBJS += \
./Application/MEKF/mekf.o 

C_DEPS += \
./Application/MEKF/mekf.d 


# Each subdirectory must supply rules for building sources it contributes
Application/MEKF/%.o Application/MEKF/%.su Application/MEKF/%.cyclo: ../Application/MEKF/%.c Application/MEKF/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/IMU/MPU6050" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/Telemetry/USB_CDC" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Calibration" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Motor" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/MEKF" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-MEKF

clean-Application-2f-MEKF:
	-$(RM) ./Application/MEKF/mekf.cyclo ./Application/MEKF/mekf.d ./Application/MEKF/mekf.o ./Application/MEKF/mekf.su

.PHONY: clean-Application-2f-MEKF

