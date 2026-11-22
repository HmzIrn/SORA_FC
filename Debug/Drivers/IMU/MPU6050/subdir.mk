################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU/MPU6050/mpu6050.c 

OBJS += \
./Drivers/IMU/MPU6050/mpu6050.o 

C_DEPS += \
./Drivers/IMU/MPU6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU/MPU6050/%.o Drivers/IMU/MPU6050/%.su Drivers/IMU/MPU6050/%.cyclo: ../Drivers/IMU/MPU6050/%.c Drivers/IMU/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/IMU/MPU6050" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Drivers/Telemetry/USB_CDC" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Calibration" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Motor" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/MEKF" -I"C:/Users/hmz_i/SEM-5/XX/SORA_FC/Application/Config" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IMU-2f-MPU6050

clean-Drivers-2f-IMU-2f-MPU6050:
	-$(RM) ./Drivers/IMU/MPU6050/mpu6050.cyclo ./Drivers/IMU/MPU6050/mpu6050.d ./Drivers/IMU/MPU6050/mpu6050.o ./Drivers/IMU/MPU6050/mpu6050.su

.PHONY: clean-Drivers-2f-IMU-2f-MPU6050

