##########################################################################################################################
# File automatically-generated by STM32forVSCode
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = stm32c0116


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/adc.c \
Core/Src/dma.c \
Core/Src/gpio.c \
Core/Src/i2c.c \
Core/Src/spi.c \
Core/Src/stm32c0xx_hal_msp.c \
Core/Src/stm32c0xx_it.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system_stm32c0xx.c \
Core/Src/tim.c \
Core/Src/usart.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_adc.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_adc_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_cortex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_dma.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_dma_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_exti.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_flash.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_flash_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_gpio.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_i2c.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_i2c_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_pwr.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_pwr_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_rcc.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_rcc_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_spi.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_spi_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_tim.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_tim_ex.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_uart.c \
Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal_uart_ex.c


CPP_SOURCES = \
Core/Src/Application.cpp \
Libs/src/Drivers/LED_RGB.cpp \
Libs/src/Drivers/RTC/DS3231.cpp \
Libs/src/MCU/ST/STM32C011F6/ADC.cpp \
Libs/src/MCU/ST/STM32C011F6/GPIO.cpp \
Libs/src/MCU/ST/STM32C011F6/I2CController.cpp \
Libs/src/MCU/ST/STM32C011F6/PWM.cpp \
Libs/src/MCU/ST/STM32C011F6/SPIController.cpp \
Libs/src/String.cpp \
build/main.cpp


# ASM sources
ASM_SOURCES =  \
startup_stm32c011xx.s



#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
POSTFIX = "
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
GCC_PATH="/opt/gcc-arm-none-eabi/bin
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++$(POSTFIX)
CC = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX)
AS = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX) -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy$(POSTFIX)
SZ = $(GCC_PATH)/$(PREFIX)size$(POSTFIX)
else
CXX = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0plus

# fpu
FPU = 

# float-abi
FLOAT-ABI = 

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32C011xx \
-DUSE_HAL_DRIVER


# CXX defines
CXX_DEFS =  \
-DNDEBUG \
-DSTM32C011xx \
-DUSE_HAL_DRIVER


# AS includes
AS_INCLUDES = \

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32C0xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32C0xx_HAL_Driver/Inc \
-IDrivers/STM32C0xx_HAL_Driver/Inc/Legacy \
-ILibs/include



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(CXX_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -feliminate-unused-debug-types

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -ggdb
CXXFLAGS += -g -gdwarf -ggdb
endif

# Add additional flags
CFLAGS += -Wall -fdata-sections -ffunction-sections -flto 
ASFLAGS += -Wall -fdata-sections -ffunction-sections 
CXXFLAGS += -Wall -Wextra -Wno-missing-field-initializers -Wno-register -fdata-sections -ffunction-sections -flto -fmessage-length=0 -fno-builtin -fno-exceptions -fno-rtti -std=c++20 

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32C011F6Ux_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = \


# Additional LD Flags from config file
ADDITIONALLDFLAGS = -flto -specs=nano.specs 

LDFLAGS = $(MCU) $(ADDITIONALLDFLAGS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of cpp program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of C objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
UPPER_CASE_ASM_SOURCES = $(filter %.S,$(ASM_SOURCES))
LOWER_CASE_ASM_SOURCES = $(filter %.s,$(ASM_SOURCES))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(UPPER_CASE_ASM_SOURCES:.S=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(LOWER_CASE_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cxx STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cxx=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c STM32Make.make | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) STM32Make.make
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	"/usr/bin/openocd" -f ./openocd.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

#######################################
# erase
#######################################
erase: $(BUILD_DIR)/$(TARGET).elf
	"/usr/bin/openocd" -f ./openocd.cfg -c "init; reset halt; stm32f0x mass_erase 0; exit"

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# custom makefile rules
#######################################


	
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***