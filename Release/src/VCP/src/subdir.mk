################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/VCP/src/hw_config.c \
../src/VCP/src/usb_desc.c \
../src/VCP/src/usb_endp.c \
../src/VCP/src/usb_istr.c \
../src/VCP/src/usb_prop.c \
../src/VCP/src/usb_pwr.c 

OBJS += \
./src/VCP/src/hw_config.o \
./src/VCP/src/usb_desc.o \
./src/VCP/src/usb_endp.o \
./src/VCP/src/usb_istr.o \
./src/VCP/src/usb_prop.o \
./src/VCP/src/usb_pwr.o 

C_DEPS += \
./src/VCP/src/hw_config.d \
./src/VCP/src/usb_desc.d \
./src/VCP/src/usb_endp.d \
./src/VCP/src/usb_istr.d \
./src/VCP/src/usb_prop.d \
./src/VCP/src/usb_pwr.d 


# Each subdirectory must supply rules for building sources it contributes
src/VCP/src/%.o: ../src/VCP/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/arm-none-eabi/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include" -I"C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4/lib/gcc/arm-none-eabi/4.8.3/include-fixed" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\CMSIS\Device\ST\STM32F10x\Include" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32_USB-FS-Device_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\Libraries\STM32F10x_StdPeriph_Driver\inc" -I"C:\Users\Chris\workspace\Firmware-master\src" -I"C:\Users\Chris\workspace\Firmware-master\src\VCP\inc" -O2 -ffunction-sections -fdata-sections -Wall -Wextra -std=gnu99 -Wa,-adhlns="$@.lst" -c -fsingle-precision-constant -Wstrict-prototypes -fverbose-asm -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


