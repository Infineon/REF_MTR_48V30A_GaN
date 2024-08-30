################################################################################
# \file bwc.mk
#
# \brief
# Backwards-compatibility variables
#
################################################################################
# \copyright
# Copyright 2018-2023 Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0
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
################################################################################

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

##########################
# Tool paths
##########################

#
# Special handling for GCC
# 	app makefile - CY_COMPILER_PATH (if selected toolchain is GCC)
# 	startex.mk - CY_COMPILER_DIR
# 	app makefile and again in main.mk if not set - CY_COMPILER_GCC_ARM_DIR
#
ifneq ($(CY_COMPILER_GCC_ARM_DIR),)
CY_INTERNAL_GCC_PATH=$(CY_COMPILER_GCC_ARM_DIR)
CY_USE_CUSTOM_GCC=true
else
CY_INTERNAL_GCC_PATH=$(CY_COMPILER_DIR)
endif

ifeq ($(TOOLCHAIN),GCC_ARM)
ifneq ($(CY_COMPILER_PATH),)
CY_INTERNAL_GCC_PATH=$(CY_COMPILER_PATH)
CY_USE_CUSTOM_GCC=true
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_gcc_BASE:=$(CY_INTERNAL_GCC_PATH)
else
ifneq ($(CY_TOOL_gcc_BASE_ABS),)
CY_INTERNAL_TOOL_gcc_BASE:=$(CY_TOOL_gcc_BASE_ABS)
else
CY_INTERNAL_TOOL_gcc_BASE:=$(CY_INTERNAL_GCC_PATH)
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_arm-none-eabi-gcc_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-gcc
else
ifneq ($(CY_TOOL_arm-none-eabi-gcc_EXE_ABS),)
CY_INTERNAL_TOOL_arm-none-eabi-gcc_EXE:=$(CY_TOOL_arm-none-eabi-gcc_EXE_ABS)
else
CY_INTERNAL_TOOL_arm-none-eabi-gcc_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-gcc
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_arm-none-eabi-g++_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-g++
else
ifneq ($(CY_TOOL_arm-none-eabi-g++_EXE_ABS),)
CY_INTERNAL_TOOL_arm-none-eabi-g++_EXE:=$(CY_TOOL_arm-none-eabi-g++_EXE_ABS)
else
CY_INTERNAL_TOOL_arm-none-eabi-g++_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-g++
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_arm-none-eabi-ar_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-ar
else
ifneq ($(CY_TOOL_arm-none-eabi-ar_EXE_ABS),)
CY_INTERNAL_TOOL_arm-none-eabi-ar_EXE:=$(CY_TOOL_arm-none-eabi-ar_EXE_ABS)
else
CY_INTERNAL_TOOL_arm-none-eabi-ar_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-ar
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_arm-none-eabi-gdb_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-gdb
else
ifneq ($(CY_TOOL_arm-none-eabi-gdb_EXE_ABS),)
CY_INTERNAL_TOOL_arm-none-eabi-gdb_EXE:=$(CY_TOOL_arm-none-eabi-gdb_EXE_ABS)
else
CY_INTERNAL_TOOL_arm-none-eabi-gdb_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-gdb
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_arm-none-eabi-objcopy_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-objcopy
else
ifneq ($(CY_TOOL_arm-none-eabi-objcopy_EXE_ABS),)
CY_INTERNAL_TOOL_arm-none-eabi-objcopy_EXE:=$(CY_TOOL_arm-none-eabi-objcopy_EXE_ABS)
else
CY_INTERNAL_TOOL_arm-none-eabi-objcopy_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-objcopy
endif
endif

ifeq ($(CY_USE_CUSTOM_GCC),true)
CY_INTERNAL_TOOL_arm-none-eabi-readelf_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-readelf
else
ifneq ($(CY_TOOL_arm-none-eabi-readelf_EXE_ABS),)
CY_INTERNAL_TOOL_arm-none-eabi-readelf_EXE:=$(CY_TOOL_arm-none-eabi-readelf_EXE_ABS)
else
CY_INTERNAL_TOOL_arm-none-eabi-readelf_EXE:=$(CY_INTERNAL_GCC_PATH)/bin/arm-none-eabi-readelf
endif
endif

##########################
# Eclipse launch configs
##########################

CY_ECLIPSE_GDB=$${cy_tools_path:CY_TOOL_arm-none-eabi-gdb_EXE}

# Special case to account for IDE 2.1 + tools 2.2 (or later)
ifeq ($(CY_MAKE_IDE),eclipse)
ifeq ($(CY_MAKE_IDE_VERSION),)
CY_ECLIPSE_GDB=$${cy_tools_path:gcc\}/bin/arm-none-eabi-gdb
endif
endif

##########################
# Pre-post build
##########################

ifneq ($(CY_BSP_PREBUILD),)
bsp_prebuild: _mtb_build__legacy_bsp_prebuild
_mtb_build__legacy_bsp_prebuild: _mtb_build_prebuild_mkdirs
	$(CY_BSP_PREBUILD)
endif

ifneq ($(PREBUILD),)
project_prebuild: _mtb_build__legacy_project_prebuild
_mtb_build__legacy_project_prebuild: bsp_prebuild
	$(PREBUILD)
endif

ifneq ($(CY_BSP_POSTBUILD),)
bsp_postbuild: _mtb_build__legacy_bsp_postbuild
_mtb_build__legacy_bsp_postbuild: recipe_postbuild
	$(CY_BSP_POSTBUILD)
endif

ifneq ($(POSTBUILD),)
project_postbuild: _mtb_build__legacy_project_postbuild
_mtb_build__legacy_project_postbuild: bsp_postbuild
	$(POSTBUILD)
endif

##########################
# Bug fix
##########################
# Bug fixes for issues found in MTB 3.0 that will fixed in MTB 3.1.
# These are ordinarily defined in tools-make there are redefined here to fix a bug related to cdb file generation.
# Remove at the next major version release.

#
# Writes to file
# $(1) : File to write
# $(2) : String
#
ifeq ($(MTB_FILE_TYPE),file)
mtb__file_write=$(file >$1,$2)
else
mtb__file_write=$(shell echo '$(subst ','"'"',$2)' >$1)
endif

#
# Appends to file
# $(1) : File to write
# $(2) : String
#
ifeq ($(MTB_FILE_TYPE),file)
mtb__file_append=$(file >>$1,$2)
else
mtb__file_append=$(shell echo '$(subst ','"'"',$2)' >>$1)
endif
