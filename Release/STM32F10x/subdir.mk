################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c 

OBJS += \
./STM32F10x/system_stm32f1xx.o 

C_DEPS += \
./STM32F10x/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
STM32F10x/system_stm32f1xx.o: /home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-non-eabi-gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


