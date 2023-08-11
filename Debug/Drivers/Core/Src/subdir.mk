################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Core/Src/system_stm32f4xx.c 

C_DEPS += \
./Drivers/Core/Src/system_stm32f4xx.d 

OBJS += \
./Drivers/Core/Src/system_stm32f4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Core/Src/%.o Drivers/Core/Src/%.su Drivers/Core/Src/%.cyclo: ../Drivers/Core/Src/%.c Drivers/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Core-2f-Src

clean-Drivers-2f-Core-2f-Src:
	-$(RM) ./Drivers/Core/Src/system_stm32f4xx.cyclo ./Drivers/Core/Src/system_stm32f4xx.d ./Drivers/Core/Src/system_stm32f4xx.o ./Drivers/Core/Src/system_stm32f4xx.su

.PHONY: clean-Drivers-2f-Core-2f-Src

