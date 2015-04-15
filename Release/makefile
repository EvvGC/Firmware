################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/sys/subdir.mk
-include src/startup/subdir.mk
-include src/VCP/src/subdir.mk
-include src/subdir.mk
-include Libraries/STM32_USB-FS-Device_Driver/src/subdir.mk
-include Libraries/STM32F10x_StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Firmware-master.hex \

SECONDARY_SIZE += \
Firmware-master.siz \


# All Target
all: Firmware-master.elf secondary-outputs

# Tool invocations
Firmware-master.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc -T"stm32_flash.ld" -nostartfiles -Xlinker --gc-sections -L"C:/Users/Chris/workspace/Firmware-master" -L"C:/Users/Chris/workspace/Firmware-master/Libraries/CMSIS/lib/GCC" -Wl,-Map,"Firmware-master.map" -mcpu=cortex-m3 -mthumb -o "Firmware-master.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Firmware-master.hex: Firmware-master.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "Firmware-master.elf"  "Firmware-master.hex"
	@echo 'Finished building: $@'
	@echo ' '

Firmware-master.siz: Firmware-master.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley "Firmware-master.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(S_UPPER_DEPS) Firmware-master.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
