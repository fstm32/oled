################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libs/fonts.c \
../Core/Libs/ssd1306.c 

OBJS += \
./Core/Libs/fonts.o \
./Core/Libs/ssd1306.o 

C_DEPS += \
./Core/Libs/fonts.d \
./Core/Libs/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libs/%.o Core/Libs/%.su: ../Core/Libs/%.c Core/Libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Libs

clean-Core-2f-Libs:
	-$(RM) ./Core/Libs/fonts.d ./Core/Libs/fonts.o ./Core/Libs/fonts.su ./Core/Libs/ssd1306.d ./Core/Libs/ssd1306.o ./Core/Libs/ssd1306.su

.PHONY: clean-Core-2f-Libs

