###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
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
 #
 ##############################################################################

ifeq "$(CMSIS_ROOT)" ""
$(error CMSIS_ROOT must be specified)
endif

# The build directory
ifeq "$(BUILD_DIR)" ""
BUILD_DIR=$(CURDIR)/build
endif

ifeq "$(STARTUPFILE)" ""
STARTUPFILE=startup_max32665.S
endif

ifeq "$(LINKERFILE)" ""
LINKERFILE=$(CMSIS_ROOT)/Device/Maxim/MAX32665/Source/GCC/max32665.ld
endif

ifeq "$(ENTRY)" ""
ENTRY=Reset_Handler
endif

# Default is 560kB, use 448kB if ECC is enabled
ifeq "$(SRAM_SIZE)" ""
SRAM_SIZE=0x8C000
endif

# Default TARGET_REVISION
# "A1" in ASCII
ifeq "$(TARGET_REV)" ""
TARGET_REV=0x4131
endif

# Add target specific CMSIS source files
ifneq (${MAKECMDGOALS},lib)
ifneq (${MAKECMDGOALS},scpa)
SRCS += ${STARTUPFILE}
endif
SRCS += backup_max32665.S
SRCS += heap.c
SRCS += system_max32665.c
endif

# Add target specific CMSIS source directories
VPATH+=$(CMSIS_ROOT)/Device/Maxim/MAX32665/Source/GCC
VPATH+=$(CMSIS_ROOT)/Device/Maxim/MAX32665/Source

# Add target specific CMSIS include directories
IPATH+=$(CMSIS_ROOT)/Device/Maxim/MAX32665/Include

# Add CMSIS Core files
CMSIS_VER ?= 5.9.0
IPATH+=$(CMSIS_ROOT)/$(CMSIS_VER)/Core/Include

# Add directory with linker include file
LIBPATH+=$(CMSIS_ROOT)/Device/Maxim/MAX32665/Source/GCC

# Include the rules and goals for building
include $(CMSIS_ROOT)/Device/Maxim/GCC/gcc.mk

# Include rules for flashing
include $(CMSIS_ROOT)/../../Tools/Flash/flash.mk
