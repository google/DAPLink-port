#
# Copyright (c) 2014-2015 0xc0170
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This project was exported via the project generator.  More information https://github.com/project-generator/project_generator

CPU = cortex-m7

# toolchain specific
TOOLCHAIN = arm-none-eabi-
TOOLCHAIN_BINPATH =
CC = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)gcc
CXX = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)g++
AS = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)gcc
LD = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)gcc
AR = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)ar
CPP = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)cpp

SIZE = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)size
OBJCOPY = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)objcopy
OBJDUMP = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)objdump
NM = $(TOOLCHAIN_BINPATH)$(TOOLCHAIN)nm

# application specific
INSTRUCTION_MODE = thumb
TARGET = stm32h743ii_if

TARGET_EXT = .elf
# Executables also produce hex and binary outputs.
ALL_TARGET_OUT_FILES = $(TARGET_OUT) $(TARGET_HEX) $(TARGET_BIN)


TARGET_OUT = $(OBJ_FOLDER)$(TARGET)$(TARGET_EXT)
TARGET_HEX = $(OBJ_FOLDER)$(TARGET).hex
TARGET_BIN = $(OBJ_FOLDER)$(TARGET).bin

LINKER_EXT = .ld

LD_SCRIPT_IN = stm32h743ii_if.ld
LD_SCRIPT = $(OUT_DIR)/$(TARGET).generated$(LINKER_EXT)


CC_SYMBOLS =  -DDAPLINK_VERSION=254  -DHID_ENDPOINT  -DMSC_ENDPOINT  -DCDC_ENDPOINT  -DWEBUSB_INTERFACE  -DWINUSB_INTERFACE  -DDRAG_N_DROP_SUPPORT  -DDAPLINK_IF  -DDAPLINK_BUILD_KEY=0x9B939E8F  -DOS_TICK=10000  -DINTERFACE_STM32H743  -DUSE_HAL_DRIVER  -DSTM32H743xx  -DDAPLINK_HIC_ID=0x97969940  -D__packed=__packed  -DDAPLINK_NO_ASSERT_FILENAMES  -DOS_CLOCK=64000000
ASM_SYMBOLS = $(CC_SYMBOLS)

LIBS =  -lm  -lgcc  -lc  -lnosys


LIB_PATHS = -L/usr/lib/arm-none-eabi/newlib/thumb/


# directories
INC_DIRS =  ../../../source/usb  ../../../source/daplink/cmsis-dap  ../../../source/daplink/drag-n-drop  ../../../source/daplink/usb2uart  ../../../source/daplink/settings  ../../../source/daplink  ../../../source/daplink/interface  ../../../source/cmsis-core  ../../../source/hic_hal  ../../../source/target  ../../../source/rtos/Include  ../../../source/rtos/RTX/Include  ../../../source/hic_hal/stm32/stm32h743ii  ../../../source/hic_hal/stm32/stm32h743ii/cmsis  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Inc  ../../../source/hic_hal/stm32/stm32h743ii/STCubeGenerated/Inc

OUT_DIR = build

INC_DIRS_F = -I. $(patsubst %, -I%, $(INC_DIRS))

SRC_DIRS =  ../../../source/usb  ../../../source/usb/hid  ../../../source/usb/msc  ../../../source/usb/cdc  ../../../source/usb/webusb  ../../../source/usb/winusb  ../../../source/daplink/cmsis-dap  ../../../source/daplink/drag-n-drop  ../../../source/daplink/usb2uart  ../../../source/daplink/settings  ../../../source/daplink  ../../../source/daplink/interface  ../../../source/target  ../../../source/rtos/RTX/Config  ../../../source/rtos/RTX/Source  ../../../source/rtos/Source  ../../../source/hic_hal/stm32/stm32h743ii  ../../../source/hic_hal/stm32/stm32h743ii/cmsis  ../../../source/hic_hal/stm32/stm32h743ii/gccarm  ../../../source/hic_hal/stm32/stm32h743ii/armcc  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src  ../../../source/hic_hal/stm32/stm32h743ii/STCubeGenerated/Src  ../../../source/family/freescale  ../../../source/family/nordic  ../../../source/family/realtek  ../../../source/family/ti  ../../../source/family/toshiba  ../../../source/family/wiznet  ../../../source/family/renesas  ../../../source/rtos/RTX/Source/GCC

ifeq ($(strip $(OUT_DIR)), )
	OBJ_FOLDER =
else
	OBJ_FOLDER = $(strip $(OUT_DIR))/
endif

# Flags
COMMON_FLAGS =  -Os  -g3  -Werror  -ffunction-sections  -fdata-sections  -std=gnu99  -fshort-wchar  -fno-common  -ffreestanding  -Wa,-adln=$@.lst  -fverbose-asm  -mno-unaligned-access
COMMON_FLAGS +=  -mcpu=$(CPU) -m$(INSTRUCTION_MODE) -MMD -MP $(CC_SYMBOLS)

C_FLAGS  =
CXX_FLAGS  =
ASM_FLAGS  =

CFLAGS = $(C_FLAGS) $(INC_DIRS_F) -c $(CC_SYMBOLS)
CXXFLAGS = $(CXX_FLAGS) $(INC_DIRS_F) -c $(CC_SYMBOLS)
ASFLAGS = $(ASM_FLAGS) $(INC_DIRS_F) -c $(ASM_SYMBOLS)
NMFLAGS = -n -f posix -C -l

# Linker options
LD_OPTIONS +=  -Wl,--no-wchar-size-warning  --specs=nano.specs  --specs=nosys.specs  -Wl,--gc-sections  -Wl,--defsym=__stack_size__=0x200  -Wl,--defsym=__heap_size__=0x0
LD_OPTIONS +=  -mcpu=$(CPU) -m$(INSTRUCTION_MODE) -Wl,-Map=$(OBJ_FOLDER)$(TARGET).map,--cref -T$(LD_SCRIPT)

# Flags to run only preprocessor
CPP_FLAGS = -E


# Flags to preprocess the linker script
LD_CPP_FLAGS = -x c -P


OBJCPFLAGS = -O ihex

ARFLAGS = cr

ifeq ($(OS),Windows_NT)
	RM = rmdir /s /q
else
	RM = rm -rf
endif

C_SRCS :=  ../../../source/daplink/HardFault_Handler.c  ../../../source/daplink/circ_buf.c  ../../../source/daplink/cmsis-dap/DAP.c  ../../../source/daplink/cmsis-dap/DAP_queue.c  ../../../source/daplink/cmsis-dap/DAP_vendor.c  ../../../source/daplink/cmsis-dap/JTAG_DP.c  ../../../source/daplink/cmsis-dap/SWO.c  ../../../source/daplink/cmsis-dap/SW_DP.c  ../../../source/daplink/cortex_m.c  ../../../source/daplink/crc16.c  ../../../source/daplink/crc32.c  ../../../source/daplink/drag-n-drop/file_stream.c  ../../../source/daplink/drag-n-drop/flash_decoder.c  ../../../source/daplink/drag-n-drop/flash_intf.c  ../../../source/daplink/drag-n-drop/flash_manager.c  ../../../source/daplink/drag-n-drop/iap_flash_intf.c  ../../../source/daplink/drag-n-drop/intelhex.c  ../../../source/daplink/drag-n-drop/vfs_manager.c  ../../../source/daplink/drag-n-drop/vfs_user.c  ../../../source/daplink/drag-n-drop/virtual_fs.c  ../../../source/daplink/error.c  ../../../source/daplink/flash_hal.c  ../../../source/daplink/info.c  ../../../source/daplink/interface/bootloader_update.c  ../../../source/daplink/interface/daplink.c  ../../../source/daplink/interface/main.c  ../../../source/daplink/interface/swd_host.c  ../../../source/daplink/interface/swd_host_ca.c  ../../../source/daplink/interface/target_flash.c  ../../../source/daplink/sdk_stub.c  ../../../source/daplink/settings/settings.c  ../../../source/daplink/settings/settings_rom.c  ../../../source/daplink/usb2uart/usbd_user_cdc_acm.c  ../../../source/daplink/util.c  ../../../source/daplink/validation.c  ../../../source/family/freescale/target_reset_Kseries.c  ../../../source/family/freescale/target_reset_Lseries.c  ../../../source/family/freescale/target_reset_mimxrt.c  ../../../source/family/freescale/target_reset_rapid_iot.c  ../../../source/family/nordic/target_reset_nrf51.c  ../../../source/family/nordic/target_reset_nrf52.c  ../../../source/family/realtek/target_reset_realtek_rtl8195am.c  ../../../source/family/renesas/target_reset_rza.c  ../../../source/family/ti/target_reset_ti.c  ../../../source/family/toshiba/target_reset_tz.c  ../../../source/family/wiznet/target_reset_wiznet.c  ../../../source/hic_hal/stm32/stm32h743ii/STCubeGenerated/Src/main_743ii.c  ../../../source/hic_hal/stm32/stm32h743ii/STCubeGenerated/Src/stm32h7xx_hal_msp.c  ../../../source/hic_hal/stm32/stm32h743ii/STCubeGenerated/Src/stm32h7xx_it.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma2d.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart_ex.c  ../../../source/hic_hal/stm32/stm32h743ii/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c  ../../../source/hic_hal/stm32/stm32h743ii/cmsis/system_stm32h7xx.c  ../../../source/hic_hal/stm32/stm32h743ii/flash.c  ../../../source/hic_hal/stm32/stm32h743ii/gpio.c  ../../../source/hic_hal/stm32/stm32h743ii/read_uid.c  ../../../source/hic_hal/stm32/stm32h743ii/sdk.c  ../../../source/hic_hal/stm32/stm32h743ii/uart.c  ../../../source/hic_hal/stm32/stm32h743ii/usb_config.c  ../../../source/hic_hal/stm32/stm32h743ii/usbd_STM32F4xx_HS.c  ../../../source/rtos/RTX/Config/RTX_Config.c  ../../../source/rtos/RTX/Source/rtx_delay.c  ../../../source/rtos/RTX/Source/rtx_kernel.c  ../../../source/rtos/RTX/Source/rtx_lib.c  ../../../source/rtos/RTX/Source/rtx_memory.c  ../../../source/rtos/RTX/Source/rtx_mempool.c  ../../../source/rtos/RTX/Source/rtx_msgqueue.c  ../../../source/rtos/RTX/Source/rtx_mutex.c  ../../../source/rtos/RTX/Source/rtx_system.c  ../../../source/rtos/RTX/Source/rtx_thread.c  ../../../source/rtos/RTX/Source/rtx_timer.c  ../../../source/rtos/Source/os_systick.c  ../../../source/target/target_board.c  ../../../source/target/target_family.c  ../../../source/usb/cdc/usbd_cdc_acm.c  ../../../source/usb/cdc/usbd_core_cdc.c  ../../../source/usb/hid/usbd_core_hid.c  ../../../source/usb/hid/usbd_hid.c  ../../../source/usb/hid/usbd_user_hid.c  ../../../source/usb/msc/usbd_core_msc.c  ../../../source/usb/msc/usbd_msc.c  ../../../source/usb/usbd_core.c  ../../../source/usb/webusb/usbd_core_webusb.c  ../../../source/usb/winusb/usbd_core_winusb.c
C_OBJS := $(patsubst %.c,$(OBJ_FOLDER)%.o,$(notdir $(C_SRCS)))

CPP_SRCS :=
CPP_OBJS := $(patsubst %.cpp,$(OBJ_FOLDER)%.o,$(notdir $(CPP_SRCS)))

S_SRCS := ../../../source/rtos/RTX/Source/GCC/irq_cm3.S
S_SRCS += ../../../source/hic_hal/stm32/stm32h743ii/gccarm/startup_stm32h743xx.s
S_OBJS = $(patsubst %.s,$(OBJ_FOLDER)%.o,$(filter %.s,$(notdir $(S_SRCS))))
S_OBJS += $(patsubst %.S,$(OBJ_FOLDER)%.o,$(filter %.S,$(notdir $(S_SRCS))))

$(info $$S_SRCS is [${S_SRCS}])
$(info $$S_OBJS is [${S_OBJS}])

O_OBJS :=

ALL_OBJS := $(C_OBJS) \
	$(CPP_OBJS) \
	$(S_OBJS) \
	$(O_OBJS)

VPATH := $(SRC_DIRS)

#-------------------------------------------------------------------------------
# Logging options
#-------------------------------------------------------------------------------

# Enable color output by default.
USE_COLOR ?= 1

# Normally, commands in recipes are prefixed with '@' so the command itself
# is not echoed by make. But if VERBOSE is defined (set to anything non-empty),
# then the '@' is removed from recipes. The 'at' variable is used to control
# this.
ifeq "$(VERBOSE)" "1"
at :=
else
at := @
endif

# These colors must be printed with the printf command. echo won't handle the
# escape sequences.
color_default = \033[00m
color_bold = \033[01m
color_red = \033[31m
color_green = \033[32m
color_yellow = \033[33m
color_blue = \033[34m
color_magenta = \033[35m
color_cyan = \033[36m
color_orange = \033[38;5;172m
color_light_blue = \033[38;5;039m
color_gray = \033[38;5;008m
color_purple = \033[38;5;097m

ifeq "$(USE_COLOR)" "1"
color_build := $(color_light_blue)
color_c := $(color_green)
color_cxx := $(color_green)
color_cpp := $(color_orange)
color_asm := $(color_magenta)
color_ar := $(color_yellow)
color_link := $(color_cyan)
color_convert := $(color_gray)
endif

# Used in printmessage if the color args are not present.
color_ :=

# Use in recipes to print color messages if printing to a terminal. If
# USE_COLOR is not set to 1, this reverts to a simple uncolorized printf.
# A newline is added to the end of the printed message.
#
# Arguments:
#  1 - name of the color variable (see above), minus the "color_" prefix
#  2 - first colorized part of the message
#  3 - first uncolorized part of the message
#  4 - color name for second colorized message
#  5 - second colorized message
#  6 - second uncolorized part of the message
#  7 - uncolorized prefix on the whole line; this is last because it is expected to be used rarely
#
# All arguments are optional.
#
# Use like:
#  $(call printmessage,cyan,Building, remainder of the message...)
ifeq "$(USE_COLOR)" "1"
define printmessage
if [ -t 1 ]; then printf "$(7)$(color_$(1))$(2)$(color_default)$(3)$(color_$(4))$(5)$(color_default)$(6)\n" ; \
else printf "$(7)$(2)$(3)$(5)$(6)\n" ; fi
endef
else
define printmessage
printf "$(7)$(2)$(3)$(5)$(6)\n"
endef
endif

#-------------------------------------------------------------------------------
# Recipes
#-------------------------------------------------------------------------------

# Compile C sources.
$(OBJ_FOLDER)%.o : %.c
	@$(call printmessage,c,Compiling, $<)
	$(at)$(CC) $(CFLAGS) $(COMMON_FLAGS) $< -o $@

# Compile C++ sources.
$(OBJ_FOLDER)%.o : %.cpp
	@$(call printmessage,cxx,Compiling, $<)
	$(at)$(CXX) $(CXXFLAGS) $(COMMON_FLAGS) $< -o $@

# Preprocess and assemble .S sources.
$(OBJ_FOLDER)%.o : %.S
	@$(call printmessage,asm,Assembling, $<)
	$(at)$(AS) $(ASFLAGS) $(COMMON_FLAGS) $< -o $@

# Assemble .s sources.
$(OBJ_FOLDER)%.o : %.s
	@$(call printmessage,asm,Assembling, $<)
	$(at)$(AS) $(ASFLAGS) $(COMMON_FLAGS) $< -o $@

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------

PRE_BUILD_SCRIPT := $(shell ../../../tools/pre_build_script.sh &&  true)

all: $(ALL_TARGET_OUT_FILES)
	$(at) ../../../tools/post_build_script_gcc.sh &&  true

# Make the build directory an order-only prerequisite for everything that goes in it.
$(ALL_TARGET_OUT_FILES) $(ALL_OBJS) $(LD_SCRIPT): | $(OUT_DIR)

$(OUT_DIR):
ifeq ($(OS),Windows_NT)
	$(at)-mkdir $(OUT_DIR)
else
	$(at)$(shell mkdir $(OBJ_FOLDER) 2>/dev/null)
endif



# Tool invocations

$(LD_SCRIPT): $(LD_SCRIPT_IN)
	@$(call printmessage,cpp,Preprocessing, $<)
	$(at)$(CPP) $(CPP_FLAGS) $(LD_CPP_FLAGS) $(INC_DIRS_F) $(CC_SYMBOLS) -MMD $< -o $@


$(TARGET_OUT): $(LD_SCRIPT) $(C_OBJS) $(CPP_OBJS) $(S_OBJS)
	@$(call printmessage,link,Linking, $@)
	$(at)$(LD) $(LIB_PATHS) -o $@ $(CPP_OBJS) $(C_OBJS) $(S_OBJS) $(O_OBJS) $(LIBS) $(LD_OPTIONS)
	$(at)$(SIZE) --totals $(TARGET_OUT)
	$(at)-$(NM) $(NMFLAGS) $(TARGET_OUT) > $(OBJ_FOLDER)$(TARGET)-symbol-table.txt

$(TARGET_HEX): $(TARGET_OUT)
	@$(call printmessage,convert,Converting, $@)
	$(at)@$(OBJCOPY) -O ihex $(TARGET_OUT)  $(TARGET_HEX)

$(TARGET_BIN): $(TARGET_OUT)
	@$(call printmessage,convert,Converting, $@)
	$(at)@$(OBJCOPY) -O binary $(TARGET_OUT)  $(TARGET_BIN)



# Other Targets
clean:
	@echo 'Removing entire out directory'
	$(at)$(RM) $(OBJ_FOLDER)* $(OBJ_FOLDER)
	@echo ' '

help:
	@echo "Useful targets:"
	@echo " - all (default)"
	@echo " - clean"
	@echo " - help"
	@echo
	@echo "Options:"
	@echo " - VERBOSE={0|1} to show full command lines."
	@echo " - USE_COLOR={0|1} to override color output."

.PHONY: clean help

# Include dependencies
-include $(ALL_OBJS:.o=.d)


# Include the linker script dependencies
-include $(LD_SCRIPT:.ld=.d)
