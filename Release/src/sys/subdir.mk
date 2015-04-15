################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/sys/itoa.c \
../src/sys/printf.c 

OBJS += \
./src/sys/itoa.o \
./src/sys/printf.o 

C_DEPS += \
./src/sys/itoa.d \
./src/sys/printf.d 


# Each subdirectory must supply rules for building sources it contributes
src/sys/%.o: ../src/sys/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/arm-none-eabi/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include-fixed" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Device\ST\STM32F10x\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32_USB-FS-Device_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32F10x_StdPeriph_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\src" -I"C:\Users\Chris\workspace\Firmware-master\src\VCP\inc" -O2 -ffunction-sections -fdata-sections -Wall -Wextra -std=gnu99 -Wa,-adhlns="$@.lst" -c -fsingle-precision-constant -Wstrict-prototypes -fverbose-asm -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


