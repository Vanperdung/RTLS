################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/User/decawave/dw_api.c \
../Core/User/decawave/dw_func.c \
../Core/User/decawave/dw_spi.c 

OBJS += \
./Core/User/decawave/dw_api.o \
./Core/User/decawave/dw_func.o \
./Core/User/decawave/dw_spi.o 

C_DEPS += \
./Core/User/decawave/dw_api.d \
./Core/User/decawave/dw_func.d \
./Core/User/decawave/dw_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/User/decawave/%.o Core/User/decawave/%.su: ../Core/User/decawave/%.c Core/User/decawave/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/User -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-User-2f-decawave

clean-Core-2f-User-2f-decawave:
	-$(RM) ./Core/User/decawave/dw_api.d ./Core/User/decawave/dw_api.o ./Core/User/decawave/dw_api.su ./Core/User/decawave/dw_func.d ./Core/User/decawave/dw_func.o ./Core/User/decawave/dw_func.su ./Core/User/decawave/dw_spi.d ./Core/User/decawave/dw_spi.o ./Core/User/decawave/dw_spi.su

.PHONY: clean-Core-2f-User-2f-decawave

