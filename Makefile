#=============================================================================#
# ARM makefile
#
# author: Freddie Chopin, http://www.freddiechopin.info/
# last change: 2012-01-07
#
# this makefile is based strongly on many examples found in the network
#=============================================================================#

#=============================================================================#
# toolchain configuration
#=============================================================================#

TOOLCHAIN = arm-none-eabi-

CXX = $(TOOLCHAIN)g++
CC = $(TOOLCHAIN)gcc
AS = $(TOOLCHAIN)gcc -x assembler-with-cpp
OBJCOPY = $(TOOLCHAIN)objcopy
OBJDUMP = $(TOOLCHAIN)objdump
SIZE = $(TOOLCHAIN)size
RM = rm -f

#=============================================================================#
# project configuration
#=============================================================================#

# project name
PROJECT = STM32Gimbal
PROJECT_USB = $(PROJECT).USB

# core type
CORE = cortex-m3

# linker script
LD_SCRIPT = stm32_flash.ld
LD_USB_SCRIPT = stm32_flash_usb.ld

# output folder (absolute or relative path, leave empty for in-tree compilation)
OUT_DIR = out

# C++ definitions (e.g. "-Dsymbol_with_value=0xDEAD -Dsymbol_without_value")
CXX_DEFS = -DSTM32F10X_HD

# C definitions
C_DEFS = -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER
#C_DEFS += -DDISABLE_PA10
#C_DEFS += -DUSB_DISC_DEV=GPIOD -DUSB_DISC_PIN=GPIO_Pin_11 -DUSB_DISC_RCC=RCC_APB2Periph_GPIOA

# ASM definitions
AS_DEFS =

# include directories (absolute or relative paths to additional folders with
# headers, current folder is always included)
INC_DIRS = src Libraries \
	Libraries/CMSIS/Include \
	Libraries/CMSIS/Device/ST/STM32F10x/Include \
	Libraries/STM32F10x_StdPeriph_Driver/inc \
	Libraries/STM32_USB-FS-Device_Driver/inc \
	src/VCP/inc

# library directories (absolute or relative paths to additional folders with
# libraries)
LIB_DIRS =

# libraries (additional libraries for linking, e.g. "-lm -lsome_name" to link
# math library libm.a and libsome_name.a)
LIBS =

# additional directories with source files (absolute or relative paths to
# folders with source files, current folder is always included)
SRCS_DIRS = Libraries/STM32F10x_StdPeriph_Driver/src src src/startup src/sys \
	Libraries/STM32_USB-FS-Device_Driver/src \
	src/VCP/src

# extension of C++ files
CXX_EXT = cpp

# wildcard for C++ source files (all files with CXX_EXT extension found in
# current folder and SRCS_DIRS folders will be compiled and linked)
CXX_SRCS = $(wildcard $(patsubst %, %/*.$(CXX_EXT), . $(SRCS_DIRS)))

# extension of C files
C_EXT = c

# wildcard for C source files (all files with C_EXT extension found in current
# folder and SRCS_DIRS folders will be compiled and linked)
C_SRCS = $(wildcard $(patsubst %, %/*.$(C_EXT), . $(SRCS_DIRS)))

# extension of ASM files
AS_EXT = S

# wildcard for ASM source files (all files with AS_EXT extension found in
# current folder and SRCS_DIRS folders will be compiled and linked)
AS_SRCS = $(wildcard $(patsubst %, %/*.$(AS_EXT), . $(SRCS_DIRS)))

# optimization flags ("-O0" - no optimization, "-O1" - optimize, "-O2" -
# optimize even more, "-Os" - optimize for size or "-O3" - optimize yet more) 
OPTIMIZATION = -O2

# set to 1 to optimize size by removing unused code and data during link phase
REMOVE_UNUSED = 1

# set to 1 to compile and link additional code required for C++
USES_CXX = 0

# define warning options here
CXX_WARNINGS = -Wall -Wextra
C_WARNINGS = -Wall -Wstrict-prototypes -Wextra

# C++ language standard ("c++98", "gnu++98" - default, "c++0x", "gnu++0x")
CXX_STD = gnu++98

# C language standard ("c89" / "iso9899:1990", "iso9899:199409",
# "c99" / "iso9899:1999", "gnu89" - default, "gnu99")
C_STD = gnu99

#=============================================================================#
# set the VPATH according to SRCS_DIRS
#=============================================================================#

VPATH = $(SRCS_DIRS)

#=============================================================================#
# when using output folder, append trailing slash to its name
#=============================================================================#

ifeq ($(strip $(OUT_DIR)), )
	OUT_DIR_F =
else
	OUT_DIR_F = $(strip $(OUT_DIR))/
endif

#=============================================================================#
# various compilation flags
#=============================================================================#

# core flags
CORE_FLAGS = -mcpu=$(CORE) -mthumb

# flags for C++ compiler
#CXX_FLAGS = -std=$(CXX_STD) -g -ggdb3 -fno-rtti -fno-exceptions -fverbose-asm -Wa,-ahlms=$(OUT_DIR_F)$(notdir $(<:.$(CXX_EXT)=.lst))
CXX_FLAGS = -std=$(CXX_STD) -fno-rtti -fno-exceptions -fverbose-asm -Wa,-ahlms=$(OUT_DIR_F)$(notdir $(<:.$(CXX_EXT)=.lst))

# flags for C compiler
#C_FLAGS = -std=$(C_STD) -g -ggdb3 -fverbose-asm -Wa,-ahlms=$(OUT_DIR_F)$(notdir $(<:.$(C_EXT)=.lst))
C_FLAGS = -std=$(C_STD) -fverbose-asm -Wa,-ahlms=$(OUT_DIR_F)$(notdir $(<:.$(C_EXT)=.lst))
C_FLAGS += -fsingle-precision-constant

# flags for assembler
#AS_FLAGS = -g -ggdb3 -Wa,-amhls=$(OUT_DIR_F)$(notdir $(<:.$(AS_EXT)=.lst))
AS_FLAGS = -Wa,-amhls=$(OUT_DIR_F)$(notdir $(<:.$(AS_EXT)=.lst))

# flags for linker
LD_FLAGS = -T$(LD_SCRIPT) -g -Wl,-Map=$(OUT_DIR_F)$(PROJECT).map,--cref,--no-warn-mismatch
LD_USB_FLAGS = -T$(LD_USB_SCRIPT) -g -Wl,-Map=$(OUT_DIR_F)$(PROJECT_USB).map,--cref,--no-warn-mismatch

# process option for removing unused code
ifeq ($(REMOVE_UNUSED), 1)
	LD_FLAGS += -Wl,--gc-sections
	LD_USB_FLAGS += -Wl,--gc-sections
	OPTIMIZATION += -ffunction-sections -fdata-sections
endif

# if __USES_CXX is defined for ASM then code for global/static constructors /
# destructors is compiled; if -nostartfiles option for linker is added then C++
# initialization / finalization code is not linked
ifeq ($(USES_CXX), 1)
	AS_DEFS += -D__USES_CXX
else
	LD_FLAGS += -nostartfiles
	LD_USB_FLAGS += -nostartfiles
endif

#=============================================================================#
# do some formatting
#=============================================================================#

CXX_OBJS = $(addprefix $(OUT_DIR_F), $(notdir $(CXX_SRCS:.$(CXX_EXT)=.o)))
C_OBJS = $(addprefix $(OUT_DIR_F), $(notdir $(C_SRCS:.$(C_EXT)=.o)))
AS_OBJS = $(addprefix $(OUT_DIR_F), $(notdir $(AS_SRCS:.$(AS_EXT)=.o)))
OBJS = $(AS_OBJS) $(C_OBJS) $(CXX_OBJS) $(USER_OBJS)
DEPS = $(OBJS:.o=.d)
INC_DIRS_F = -I. $(patsubst %, -I%, $(INC_DIRS))
LIB_DIRS_F = $(patsubst %, -L%, $(LIB_DIRS))

ELF = $(OUT_DIR_F)$(PROJECT).elf
HEX = $(OUT_DIR_F)$(PROJECT).hex
BIN = $(OUT_DIR_F)$(PROJECT).bin
LSS = $(OUT_DIR_F)$(PROJECT).lss
DMP = $(OUT_DIR_F)$(PROJECT).dmp

USBELF = $(OUT_DIR_F)$(PROJECT_USB).elf
USBHEX = $(OUT_DIR_F)$(PROJECT_USB).hex
USBBIN = $(OUT_DIR_F)$(PROJECT_USB).bin
USBLSS = $(OUT_DIR_F)$(PROJECT_USB).lss
USBDMP = $(OUT_DIR_F)$(PROJECT_USB).dmp


# format final flags for tools, request dependancies for C++, C and asm
CXX_FLAGS_F = $(CORE_FLAGS) $(OPTIMIZATION) $(CXX_WARNINGS) $(CXX_FLAGS)  $(CXX_DEFS) -MD -MP -MF $(OUT_DIR_F)$(@F:.o=.d) $(INC_DIRS_F)
C_FLAGS_F = $(CORE_FLAGS) $(OPTIMIZATION) $(C_WARNINGS) $(C_FLAGS) $(C_DEFS) -MD -MP -MF $(OUT_DIR_F)$(@F:.o=.d) $(INC_DIRS_F)
AS_FLAGS_F = $(CORE_FLAGS) $(AS_FLAGS) $(AS_DEFS) -MD -MP -MF $(OUT_DIR_F)$(@F:.o=.d) $(INC_DIRS_F)
LD_FLAGS_F = $(CORE_FLAGS) $(LD_FLAGS) $(LIB_DIRS_F)
LD_USB_FLAGS_F = $(CORE_FLAGS) $(LD_USB_FLAGS) $(LIB_DIRS_F)

#contents of output directory
GENERATED = $(wildcard $(patsubst %, $(OUT_DIR_F)*.%, bin d dmp elf hex lss lst map o))

#=============================================================================#
# make all
#=============================================================================#

#all : make_output_dir $(ELF) $(LSS) $(DMP) $(HEX) $(BIN) print_size
all: make_output_dir $(ELF) $(LSS) $(DMP) $(HEX) $(BIN) $(USBELF) $(USBLSS) $(USBBIN) print_size \
#					$(USBELF) $(USBLSS) $(USBBIN

# make object files dependent on Makefile
#$(OBJS) : Makefile
# make .elf file dependent on linker script
$(ELF) : $(LD_SCRIPT)
$(USBELF) : $(LD_USB_SCRIPT)

#-----------------------------------------------------------------------------#
# linking - objects -> elf
#-----------------------------------------------------------------------------#

$(ELF) : $(OBJS)  $(LD_SCRIPT)
	@echo 'Linking target: $(ELF)'
	$(CXX) $(LD_FLAGS_F) $(OBJS) $(LIBS) -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# linking - objects -> elf, USB version
#-----------------------------------------------------------------------------#

$(USBELF) : $(ELF) $(OBJS) $(LD_USB_SCRIPT)
	@echo 'Linking target: $(USBELF)'
	$(CXX) $(LD_USB_FLAGS_F) $(OBJS) $(LIBS) -o $@
	@echo ' '
	
#-----------------------------------------------------------------------------#
# compiling - C++ source -> objects
#-----------------------------------------------------------------------------#

$(OUT_DIR_F)%.o : %.$(CXX_EXT)
	@echo 'Compiling file: $<'
	$(CXX) -c $(CXX_FLAGS_F) $< -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# compiling - C source -> objects
#-----------------------------------------------------------------------------#

$(OUT_DIR_F)%.o : %.$(C_EXT)
	@echo 'Compiling file: $<'
	$(CC) -c $(C_FLAGS_F) $< -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# assembling - ASM source -> objects
#-----------------------------------------------------------------------------#

$(OUT_DIR_F)%.o : %.$(AS_EXT)
	@echo 'Assembling file: $<'
	$(AS) -c $(AS_FLAGS_F) $< -o $@
	@echo ' '

#-----------------------------------------------------------------------------#
# memory images - elf -> hex, elf -> bin
#-----------------------------------------------------------------------------#

$(HEX) : $(ELF)
	@echo 'Creating IHEX image: $(HEX)'
	$(OBJCOPY) -O ihex $< $@
	@echo ' '

$(BIN) : $(ELF)
	@echo 'Creating binary image: $(BIN)'
	$(OBJCOPY) -O binary $< $@
	@echo ' '
	
$(USBBIN) : $(USBELF)
	@echo 'Creating binary image: $(USBBIN)'
	$(OBJCOPY) -O binary $< $@
	@echo ' '	

#-----------------------------------------------------------------------------#
# memory dump - elf -> dmp
#-----------------------------------------------------------------------------#

$(DMP) : $(ELF)
	@echo 'Creating memory dump: $(DMP)'
	$(OBJDUMP) -x --syms $< > $@
	@echo ' '

#-----------------------------------------------------------------------------#
# extended listing - elf -> lss
#-----------------------------------------------------------------------------#

$(LSS) : $(ELF)
	@echo 'Creating extended listing: $(LSS)'
	$(OBJDUMP) -S $< > $@
	@echo ' '

	
$(USBLSS) : $(USBELF)
	@echo 'Creating extended listing: $(USBLSS)'
	$(OBJDUMP) -S $< > $@
	@echo ' '	
#-----------------------------------------------------------------------------#
# print the size of the objects and the .elf file
#-----------------------------------------------------------------------------#

print_size: $(OBJS) $(ELF)
	-#@echo 'Size of modules:'
	-#$(SIZE) -B -t --common $(OBJS) $(USER_OBJS)
	-#@echo ' '
	@echo 'Size of target .elf file:'
	$(SIZE) -B $(ELF)
	@echo ' '

#-----------------------------------------------------------------------------#
# create the desired output directory
#-----------------------------------------------------------------------------#

make_output_dir :
	$(shell mkdir $(OUT_DIR_F) 2>/dev/null)

#=============================================================================#
# make clean
#=============================================================================#

clean:
ifeq ($(strip $(OUT_DIR_F)), )
	@echo 'Removing all generated output files'
else
	@echo 'Removing all generated output files from output directory: $(OUT_DIR_F)'
endif
ifneq ($(strip $(GENERATED)), )
	$(RM) $(GENERATED)
else
	@echo 'Nothing to remove...'
endif

#=============================================================================#
# global exports
#=============================================================================#

.PHONY: all clean dependents

.SECONDARY:

# include dependancy files
#-include $(DEPS)
