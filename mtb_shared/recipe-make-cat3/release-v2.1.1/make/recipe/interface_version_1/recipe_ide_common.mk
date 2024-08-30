################################################################################
# \file recipe_ide_common.mk
#
# \brief
# This make file defines the IDE export variables and target.
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


# Set the output file paths
_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE:=$(notdir $(MTB_TOOLS__OUTPUT_BASE_DIR))/$(TARGET)/$(CONFIG)
_MTB_RECIPE__IDE_BUILD_APPLICATION_PATH_RELATIVE:=$(notdir $(MTB_TOOLS__PRJ_DIR))/$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)
_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE:=$(patsubst $(call mtb__path_normalize,$(MTB_TOOLS__PRJ_DIR)/../)/%,%,$(_MTB_RECIPE__APP_HEX_FILE))


MTB_RECIPE__IDE_RECIPE_DATA_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/ide_recipe_data.temp
MTB_RECIPE__IDE_RECIPE_METADATA_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/ide_recipe_metadata.temp
_MTB_RECIPE__ECLIPSE_OPENOCD_SVD_PATH:=$${cy_prj_path}/$(DEVICE_$(DEVICE)_SVD)
# Eclipse
ifeq ($(filter eclipse,$(MAKECMDGOALS)),eclipse)

ifneq ($(CY_BUILD_LOCATION),)
_MTB_RECIPE__ECLIPSE_SYM_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
else
_MTB_RECIPE__ECLIPSE_SYM_FILE:=$${cy_prj_path}/$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$${cy_prj_path}/$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
endif

ifeq ($(MTB_TYPE),PROJECT)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$${cy_prj_path}/../$(_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE)
endif

eclipse_textdata_file_common:
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__OPENOCD_CFG&&=$(_MTB_RECIPE__OPENOCD_DEVICE_CFG))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__OPENOCD_CHIP&&=$(_MTB_RECIPE__OPENOCD_CHIP_NAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__APPNAME&&=$(CY_IDE_PRJNAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__SVD_PATH&&=$(_MTB_RECIPE__ECLIPSE_OPENOCD_SVD_PATH))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__SYM_FILE&&=$(_MTB_RECIPE__ECLIPSE_SYM_FILE))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__PROG_FILE&&=$(_MTB_RECIPE__ECLIPSE_PROG_FILE))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__ECLIPSE_GDB&&=$(CY_ECLIPSE_GDB))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&MTB_APPLICATION_NAME&&=$(_MTB_ECLIPSE_APPLICATION_NAME))

eclipse_textdata_file : eclipse_textdata_file_common
endif

# VSCode
ifeq ($(filter vscode,$(MAKECMDGOALS)),vscode)
_MTB_RECIPE__GCC_BASE_DIR:=$(subst $(MTB_TOOLS__TOOLS_DIR)/,,$(MTB_TOOLCHAIN_GCC_ARM__BASE_DIR))
_MTB_RECIPE__GCC_VERSION:=$(shell $(MTB_TOOLCHAIN_GCC_ARM__CC) -dumpversion)
_MTB_RECIPE__OPENOCD_EXE_DIR_RELATIVE:=$(CY_TOOL_openocd_EXE)
_MTB_RECIPE__OPENOCD_SCRIPTS_DIR_RELATIVE:=$(CY_TOOL_openocd_scripts_SCRIPT)

ifneq ($(CY_BUILD_LOCATION),)
_MTB_RECIPE__ELF_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__ELF_FILE_APPLICATION:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__HEX_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
else
_MTB_RECIPE__ELF_FILE:=./$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__ELF_FILE_APPLICATION:=./$(_MTB_RECIPE__IDE_BUILD_APPLICATION_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__HEX_FILE:=./$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
endif

_MTB_RECIPE__HEX_FILE_APPLICATION:=./$(_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE)
ifeq ($(MTB_TYPE),PROJECT)
_MTB_RECIPE__HEX_FILE:=../$(_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE)
endif

# This must set with = instead of :=
_MTB_RECIPE__C_FLAGS=$(subst $(MTB__SPACE),\"$(MTB__COMMA)$(MTB__NEWLINE_MARKER)\",$(strip $(MTB_RECIPE__CFLAGS)))

ifeq ($(MTB_RECIPE__ATTACH_SERVER_TYPE),)
MTB_RECIPE__ATTACH_SERVER_TYPE=openocd
endif

_MTB_VSCODE_MODUS_SHELL_RELATIVE:=$(CY_TOOL_modus-shell_BASE)

_MTB_VSCODE_SVD_PATH:=$(DEVICE_$(DEVICE)_SVD)
_MTB_VSCODE_APPLICATION_SVD_PATH=$(patsubst ../%,%,$(_MTB_VSCODE_SVD_PATH))

$(MTB_RECIPE__IDE_RECIPE_DATA_FILE):
	$(MTB__NOISE)echo "s|&&_MTB_RECIPE__ELF_FILE&&|$(_MTB_RECIPE__ELF_FILE)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__HEX_FILE&&|$(_MTB_RECIPE__HEX_FILE)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__ELF_FILE_APPLICATION&&|$(_MTB_RECIPE__ELF_FILE_APPLICATION)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__HEX_FILE_APPLICATION&&|$(_MTB_RECIPE__HEX_FILE_APPLICATION)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__OPEN_OCD_FILE&&|$(_MTB_RECIPE__OPENOCD_DEVICE_CFG)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__SVD_FILE_NAME&&|$(_MTB_VSCODE_SVD_PATH)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__APPLICATION_SVD_FILE_NAME&&|$(_MTB_VSCODE_APPLICATION_SVD_PATH)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__MTB_PATH&&|$(CY_TOOLS_DIR)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__TOOL_CHAIN_DIRECTORY&&|$(subst ",,$(CY_CROSSPATH))|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__C_FLAGS&&|$(_MTB_RECIPE__C_FLAGS)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__GCC_VERSION&&|$(_MTB_RECIPE__GCC_VERSION)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__OPENOCD_EXE_DIR_RELATIVE&&|$(_MTB_RECIPE__OPENOCD_EXE_DIR_RELATIVE)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__OPENOCD_SCRIPTS_DIR_RELATIVE&&|$(_MTB_RECIPE__OPENOCD_SCRIPTS_DIR_RELATIVE)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__DEVICE_ATTACH&&|$(_MTB_RECIPE__JLINK_DEVICE_CFG_ATTACH)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__MODUS_SHELL_BASE&&|$(_MTB_VSCODE_MODUS_SHELL_RELATIVE)|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__ATTACH_SERVER_TYPE&&|$(MTB_RECIPE__ATTACH_SERVER_TYPE)|g;" >> $@;
ifeq ($(CY_USE_CUSTOM_GCC),true)
	$(MTB__NOISE)echo "s|&&_MTB_RECIPE__GCC_BIN_DIR&&|$(MTB_TOOLCHAIN_GCC_ARM__BASE_DIR)/bin|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__GCC_DIRECTORY&&|$(MTB_TOOLCHAIN_GCC_ARM__BASE_DIR)|g;" >> $@;
else
	$(MTB__NOISE)echo "s|&&_MTB_RECIPE__GCC_BIN_DIR&&|$$\{config:modustoolbox.toolsPath\}/$(_MTB_RECIPE__GCC_BASE_DIR)/bin|g;" >> $@;\
	echo "s|&&_MTB_RECIPE__GCC_DIRECTORY&&|$$\{config:modustoolbox.toolsPath\}/$(_MTB_RECIPE__GCC_BASE_DIR)|g;" >> $@;
endif

$(MTB_RECIPE__IDE_RECIPE_DATA_FILE) : $(MTB_RECIPE__IDE_RECIPE_DATA_FILE)_vscode
.PHONY: $(MTB_RECIPE__IDE_RECIPE_DATA_FILE)_vscode
endif
