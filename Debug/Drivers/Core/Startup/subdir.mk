################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/Core/Startup/startup_stm32f446retx.s 

S_DEPS += \
./Drivers/Core/Startup/startup_stm32f446retx.d 

OBJS += \
./Drivers/Core/Startup/startup_stm32f446retx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Core/Startup/%.o: ../Drivers/Core/Startup/%.s Drivers/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Drivers-2f-Core-2f-Startup

clean-Drivers-2f-Core-2f-Startup:
	-$(RM) ./Drivers/Core/Startup/startup_stm32f446retx.d ./Drivers/Core/Startup/startup_stm32f446retx.o

.PHONY: clean-Drivers-2f-Core-2f-Startup

