################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.cpp 

OBJS += \
./Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.o 

CPP_DEPS += \
./Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/CPDevVM/src/VM/vmfunc/%.o Core/Lib/CPDevVM/src/VM/vmfunc/%.su Core/Lib/CPDevVM/src/VM/vmfunc/%.cyclo: ../Core/Lib/CPDevVM/src/VM/vmfunc/%.cpp Core/Lib/CPDevVM/src/VM/vmfunc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-CPDevVM-2f-src-2f-VM-2f-vmfunc

clean-Core-2f-Lib-2f-CPDevVM-2f-src-2f-VM-2f-vmfunc:
	-$(RM) ./Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.cyclo ./Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.d ./Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.o ./Core/Lib/CPDevVM/src/VM/vmfunc/vm_data_access.su

.PHONY: clean-Core-2f-Lib-2f-CPDevVM-2f-src-2f-VM-2f-vmfunc

