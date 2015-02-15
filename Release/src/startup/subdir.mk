################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../src/startup/startup_stm32f10x_hd.S 

OBJS += \
./src/startup/startup_stm32f10x_hd.o 

S_UPPER_DEPS += \
./src/startup/startup_stm32f10x_hd.d 


# Each subdirectory must supply rules for building sources it contributes
src/startup/%.o: ../src/startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -Wa,-adhlns="$@.lst" -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


