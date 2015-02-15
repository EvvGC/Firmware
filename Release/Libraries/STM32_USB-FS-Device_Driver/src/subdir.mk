################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/STM32_USB-FS-Device_Driver/src/usb_core.c \
../Libraries/STM32_USB-FS-Device_Driver/src/usb_init.c \
../Libraries/STM32_USB-FS-Device_Driver/src/usb_int.c \
../Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.c \
../Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.c \
../Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.c 

OBJS += \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_core.o \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_init.o \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_int.o \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.o \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.o \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.o 

C_DEPS += \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_core.d \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_init.d \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_int.d \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.d \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.d \
./Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/STM32_USB-FS-Device_Driver/src/%.o: ../Libraries/STM32_USB-FS-Device_Driver/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/arm-none-eabi/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include-fixed" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Device\ST\STM32F10x\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32_USB-FS-Device_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32F10x_StdPeriph_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\src" -I"C:\Users\Chris\workspace\Firmware-master\src\VCP\inc" -O2 -ffunction-sections -fdata-sections -Wall -Wextra -std=gnu99 -Wa,-adhlns="$@.lst" -c -fsingle-precision-constant -Wstrict-prototypes -fverbose-asm -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


