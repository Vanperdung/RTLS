################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/User/led/led.c 

OBJS += \
./Core/User/led/led.o 

C_DEPS += \
./Core/User/led/led.d 


# Each subdirectory must supply rules for building sources it contributes
Core/User/led/%.o Core/User/led/%.su: ../Core/User/led/%.c Core/User/led/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/User -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-User-2f-led

clean-Core-2f-User-2f-led:
	-$(RM) ./Core/User/led/led.d ./Core/User/led/led.o ./Core/User/led/led.su

.PHONY: clean-Core-2f-User-2f-led

