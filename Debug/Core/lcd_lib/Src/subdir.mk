################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lcd_lib/Src/i2c_lcd.c 

OBJS += \
./Core/lcd_lib/Src/i2c_lcd.o 

C_DEPS += \
./Core/lcd_lib/Src/i2c_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lcd_lib/Src/%.o Core/lcd_lib/Src/%.su Core/lcd_lib/Src/%.cyclo: ../Core/lcd_lib/Src/%.c Core/lcd_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/yedeklemeler/STM32 Workspace/sut_olcum__1903/Core/lcd_lib/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-lcd_lib-2f-Src

clean-Core-2f-lcd_lib-2f-Src:
	-$(RM) ./Core/lcd_lib/Src/i2c_lcd.cyclo ./Core/lcd_lib/Src/i2c_lcd.d ./Core/lcd_lib/Src/i2c_lcd.o ./Core/lcd_lib/Src/i2c_lcd.su

.PHONY: clean-Core-2f-lcd_lib-2f-Src

