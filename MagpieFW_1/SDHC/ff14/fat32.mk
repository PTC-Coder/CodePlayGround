###############################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by
 # Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc.
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

################################################################################
# This file can be included in a project makefile to build the library for the 
# project.
################################################################################

$(warning Warning: Building with FatFS R0.14.  The MSDK will update the default FatFS version to R0.15 next release.  See https://github.com/analogdevicesinc/msdk/pull/720)

ifeq "$(FAT32_DRIVER_DIR)" ""
$(error FAT32_DRIVER_DIR must be specified")
endif

# Specify the build directory if not defined by the project
ifeq "$(SDHC_DIR)" ""
FAT32_DRIVER_BUILD_DIR=$(CURDIR)/build/Fat32Driver
else
FAT32_DRIVER_BUILD_DIR=$(SDHC_DIR)/Fat32Driver
endif

# Export paths needed by the fat32 driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
FAT32_DRIVER_BUILD_DIR := ${abspath ${FAT32_DRIVER_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export PERIPH_DRIVER_DIR := ${abspath ${PERIPH_DRIVER_DIR}}

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS
export MXC_OPTIMIZE_CFLAGS
export USE_NATIVE_SDHC

# Add to library list
LIBS += ${FAT32_DRIVER_BUILD_DIR}/FAT32.a

# Add to include directory list
IPATH += ${FAT32_DRIVER_DIR}/Source

# Add rule to build the Driver Library
${FAT32_DRIVER_BUILD_DIR}/FAT32.a: $(PROJECTMK)
	$(MAKE) -C ${FAT32_DRIVER_DIR} lib BUILD_DIR=${FAT32_DRIVER_BUILD_DIR} BOARD=${BOARD}

distclean:
		$(MAKE) -C ${SDHC_DRIVER_DIR} clean
