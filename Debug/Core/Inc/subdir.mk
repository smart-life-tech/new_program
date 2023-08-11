################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/fonts.c 

CPP_SRCS += \
../Core/Inc/SSD1306.cpp \
../Core/Inc/SSD1306I2C.cpp 

C_DEPS += \
./Core/Inc/fonts.d 

OBJS += \
./Core/Inc/SSD1306.o \
./Core/Inc/SSD1306I2C.o \
./Core/Inc/fonts.o 

CPP_DEPS += \
./Core/Inc/SSD1306.d \
./Core/Inc/SSD1306I2C.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.cpp Core/Inc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/SSD1306.cyclo ./Core/Inc/SSD1306.d ./Core/Inc/SSD1306.o ./Core/Inc/SSD1306.su ./Core/Inc/SSD1306I2C.cyclo ./Core/Inc/SSD1306I2C.d ./Core/Inc/SSD1306I2C.o ./Core/Inc/SSD1306I2C.su ./Core/Inc/fonts.cyclo ./Core/Inc/fonts.d ./Core/Inc/fonts.o ./Core/Inc/fonts.su

.PHONY: clean-Core-2f-Inc

