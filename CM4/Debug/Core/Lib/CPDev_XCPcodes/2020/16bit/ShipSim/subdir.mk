################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.c 

C_DEPS += \
./Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.d 

OBJS += \
./Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/%.o Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/%.su Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/%.cyclo: ../Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/%.c Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H747xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-CPDev_XCPcodes-2f-2020-2f-16bit-2f-ShipSim

clean-Core-2f-Lib-2f-CPDev_XCPcodes-2f-2020-2f-16bit-2f-ShipSim:
	-$(RM) ./Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.cyclo ./Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.d ./Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.o ./Core/Lib/CPDev_XCPcodes/2020/16bit/ShipSim/ShipSim.su

.PHONY: clean-Core-2f-Lib-2f-CPDev_XCPcodes-2f-2020-2f-16bit-2f-ShipSim

