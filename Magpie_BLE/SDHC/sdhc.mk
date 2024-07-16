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

ifeq "$(SDHC_DRIVER_DIR)" ""
$(error SDHC_DRIVER_DIR must be specified")
endif

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
SDHC_DRIVER_BUILD_DIR=$(CURDIR)/build/SDHCDriver
else
SDHC_DRIVER_BUILD_DIR=$(BUILD_DIR)/SDHCDriver
endif

# Export paths needed by the SDHC driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
SDHC_DRIVER_BUILD_DIR := ${abspath ${SDHC_DRIVER_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export PERIPH_DRIVER_DIR := ${abspath ${PERIPH_DRIVER_DIR}}
export MISC_DRIVERS_DIR=$(LIBS_DIR)/MiscDrivers

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS
export MXC_OPTIMIZE_CFLAGS
export BOARD_DIR
export USE_NATIVE_SDHC
export EXTERNAL_FLASH
# Add to library list
LIBS += ${SDHC_DRIVER_BUILD_DIR}/sdhc.a

# Add to include directory list
IPATH += ${SDHC_DRIVER_DIR}/Include

# Add rule to build the Driver Library
${SDHC_DRIVER_BUILD_DIR}/sdhc.a: $(PROJECTMK)
	$(MAKE) -C ${SDHC_DRIVER_DIR} lib BUILD_DIR=${SDHC_DRIVER_BUILD_DIR} BOARD=${BOARD} PROJECTMK=$(PROJECTMK)

distclean:
	$(MAKE) -C ${SDHC_DRIVER_DIR} clean
