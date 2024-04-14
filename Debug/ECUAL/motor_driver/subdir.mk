################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/motor_driver/motor_driver.c 

C_DEPS += \
./ECUAL/motor_driver/motor_driver.d 

OBJS += \
./ECUAL/motor_driver/motor_driver.o 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/motor_driver/%.o ECUAL/motor_driver/%.su ECUAL/motor_driver/%.cyclo: ../ECUAL/motor_driver/%.c ECUAL/motor_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECUAL-2f-motor_driver

clean-ECUAL-2f-motor_driver:
	-$(RM) ./ECUAL/motor_driver/motor_driver.cyclo ./ECUAL/motor_driver/motor_driver.d ./ECUAL/motor_driver/motor_driver.o ./ECUAL/motor_driver/motor_driver.su

.PHONY: clean-ECUAL-2f-motor_driver

