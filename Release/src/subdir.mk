################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/adc.c \
../src/comio.c \
../src/commhandler.c \
../src/config.c \
../src/eeprom.c \
../src/engine.c \
../src/fasttrig.c \
../src/gyro.c \
../src/i2c.c \
../src/main.c \
../src/newlib_stubs.c \
../src/pins.c \
../src/pwm.c \
../src/rc.c \
../src/reboot.c \
../src/ringbuffer.c \
../src/stm32_it.c \
../src/stopwatch.c \
../src/system_stm32f10x.c \
../src/systick.c \
../src/usart.c \
../src/usb.c \
../src/utils.c 

OBJS += \
./src/adc.o \
./src/comio.o \
./src/commhandler.o \
./src/config.o \
./src/eeprom.o \
./src/engine.o \
./src/fasttrig.o \
./src/gyro.o \
./src/i2c.o \
./src/main.o \
./src/newlib_stubs.o \
./src/pins.o \
./src/pwm.o \
./src/rc.o \
./src/reboot.o \
./src/ringbuffer.o \
./src/stm32_it.o \
./src/stopwatch.o \
./src/system_stm32f10x.o \
./src/systick.o \
./src/usart.o \
./src/usb.o \
./src/utils.o 

C_DEPS += \
./src/adc.d \
./src/comio.d \
./src/commhandler.d \
./src/config.d \
./src/eeprom.d \
./src/engine.d \
./src/fasttrig.d \
./src/gyro.d \
./src/i2c.d \
./src/main.d \
./src/newlib_stubs.d \
./src/pins.d \
./src/pwm.d \
./src/rc.d \
./src/reboot.d \
./src/ringbuffer.d \
./src/stm32_it.d \
./src/stopwatch.d \
./src/system_stm32f10x.d \
./src/systick.d \
./src/usart.d \
./src/usb.d \
./src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/arm-none-eabi/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include-fixed" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Device\ST\STM32F10x\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32_USB-FS-Device_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32F10x_StdPeriph_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\src" -I"C:\Users\Chris\workspace\Firmware-master\src\VCP\inc" -O2 -ffunction-sections -fdata-sections -Wall -Wextra -std=gnu99 -Wa,-adhlns="$@.lst" -c -fsingle-precision-constant -Wstrict-prototypes -fverbose-asm -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


