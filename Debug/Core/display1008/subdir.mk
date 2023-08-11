################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/display1008/fonts.c 

CPP_SRCS += \
../Core/display1008/SSD1306.cpp \
../Core/display1008/SSD1306I2C.cpp 

C_DEPS += \
./Core/display1008/fonts.d 

OBJS += \
./Core/display1008/SSD1306.o \
./Core/display1008/SSD1306I2C.o \
./Core/display1008/fonts.o 

CPP_DEPS += \
./Core/display1008/SSD1306.d \
./Core/display1008/SSD1306I2C.d 


# Each subdirectory must supply rules for building sources it contributes
Core/display1008/%.o Core/display1008/%.su Core/display1008/%.cyclo: ../Core/display1008/%.cpp Core/display1008/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/display1008/%.o Core/display1008/%.su Core/display1008/%.cyclo: ../Core/display1008/%.c Core/display1008/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-display1008

clean-Core-2f-display1008:
	-$(RM) ./Core/display1008/SSD1306.cyclo ./Core/display1008/SSD1306.d ./Core/display1008/SSD1306.o ./Core/display1008/SSD1306.su ./Core/display1008/SSD1306I2C.cyclo ./Core/display1008/SSD1306I2C.d ./Core/display1008/SSD1306I2C.o ./Core/display1008/SSD1306I2C.su ./Core/display1008/fonts.cyclo ./Core/display1008/fonts.d ./Core/display1008/fonts.o ./Core/display1008/fonts.su

.PHONY: clean-Core-2f-display1008

