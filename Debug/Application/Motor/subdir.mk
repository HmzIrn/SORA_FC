################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Motor/motor_mixer.c 

OBJS += \
./Application/Motor/motor_mixer.o 

C_DEPS += \
./Application/Motor/motor_mixer.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Motor/%.o Application/Motor/%.su Application/Motor/%.cyclo: ../Application/Motor/%.c Application/Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/IMU/MPU6050" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/Telemetry/USB_CDC" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Calibration" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Motor" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/MEKF" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Motor

clean-Application-2f-Motor:
	-$(RM) ./Application/Motor/motor_mixer.cyclo ./Application/Motor/motor_mixer.d ./Application/Motor/motor_mixer.o ./Application/Motor/motor_mixer.su

.PHONY: clean-Application-2f-Motor

