################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Lib/CPDevVM/src/vm_arduino.cpp 

OBJS += \
./Core/Lib/CPDevVM/src/vm_arduino.o 

CPP_DEPS += \
./Core/Lib/CPDevVM/src/vm_arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/CPDevVM/src/%.o Core/Lib/CPDevVM/src/%.su Core/Lib/CPDevVM/src/%.cyclo: ../Core/Lib/CPDevVM/src/%.cpp Core/Lib/CPDevVM/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-CPDevVM-2f-src

clean-Core-2f-Lib-2f-CPDevVM-2f-src:
	-$(RM) ./Core/Lib/CPDevVM/src/vm_arduino.cyclo ./Core/Lib/CPDevVM/src/vm_arduino.d ./Core/Lib/CPDevVM/src/vm_arduino.o ./Core/Lib/CPDevVM/src/vm_arduino.su

.PHONY: clean-Core-2f-Lib-2f-CPDevVM-2f-src

