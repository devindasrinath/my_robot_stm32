################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/pid/pid_v1.c 

C_DEPS += \
./ECUAL/pid/pid_v1.d 

OBJS += \
./ECUAL/pid/pid_v1.o 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/pid/%.o ECUAL/pid/%.su ECUAL/pid/%.cyclo: ../ECUAL/pid/%.c ECUAL/pid/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECUAL-2f-pid

clean-ECUAL-2f-pid:
	-$(RM) ./ECUAL/pid/pid_v1.cyclo ./ECUAL/pid/pid_v1.d ./ECUAL/pid/pid_v1.o ./ECUAL/pid/pid_v1.su

.PHONY: clean-ECUAL-2f-pid

