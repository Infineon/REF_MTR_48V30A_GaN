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

##############################################
# VSCode Eclipse
##############################################
_MTB_RECIPE__IDE_TEMPLATE_DIR:=$(MTB_TOOLS__RECIPE_DIR)/make/scripts/interface_version_2
_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE:=$(notdir $(MTB_TOOLS__OUTPUT_BASE_DIR))/$(TARGET)/$(CONFIG)
_MTB_RECIPE__IDE_BUILD_APPLICATION_PATH_RELATIVE:=$(notdir $(MTB_TOOLS__PRJ_DIR))/$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)
_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE:=$(patsubst $(call mtb__path_normalize,$(MTB_TOOLS__PRJ_DIR)/../)/%,%,$(_MTB_RECIPE__APP_HEX_FILE))
_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/recipe_ide_commom_text_data.txt

##############################################
# VSCode
##############################################
_MTB_RECIPE__VSCODE_GCC_BASE_DIR:=$(subst $(MTB_TOOLS__TOOLS_DIR)/,$${config:modustoolbox.toolsPath}/,$(MTB_TOOLCHAIN_GCC_ARM__BASE_DIR))
_MTB_RECIPE__VSCODE_HEX_FILE_APPLICATION:=$(_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE)
ifneq ($(CY_BUILD_LOCATION),)
_MTB_RECIPE__VSCODE_ELF_FILE_APPLICATION:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__VSCODE_ELF_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
ifeq ($(MTB_TYPE),PROJECT)
_MTB_RECIPE__VSCODE_HEX_FILE:=$(_MTB_RECIPE__APP_HEX_FILE)
else
_MTB_RECIPE__VSCODE_HEX_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
endif
else #($(CY_BUILD_LOCATION),)
_MTB_RECIPE__VSCODE_ELF_FILE_APPLICATION:=$(_MTB_RECIPE__IDE_BUILD_APPLICATION_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__VSCODE_ELF_FILE:=./$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
ifeq ($(MTB_TYPE),PROJECT)
_MTB_RECIPE__VSCODE_HEX_FILE:=../$(_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE)
else
_MTB_RECIPE__VSCODE_HEX_FILE:=./$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
endif
endif #($(CY_BUILD_LOCATION),)
ifeq ($(MTB_RECIPE__ATTACH_SERVER_TYPE),)
MTB_RECIPE__ATTACH_SERVER_TYPE=openocd
endif
_MTB_RECIPE__VSCODE_SVD_PATH:=$(DEVICE_$(DEVICE)_SVD)
_MTB_RECIPE__VSCODE_APPLICATION_SVD_PATH=$(patsubst ../%,%,$(_MTB_RECIPE__VSCODE_SVD_PATH))

vscode_generate: recipe_vscode_common_text_replacement_data_file
vscode_generate: MTB_CORE__EXPORT_CMDLINE += -textdata $(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE)

recipe_vscode_common_text_replacement_data_file:
	$(call mtb__file_write,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__ELF_FILE&&=$(_MTB_RECIPE__VSCODE_ELF_FILE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__HEX_FILE&&=$(_MTB_RECIPE__VSCODE_HEX_FILE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__ELF_FILE_APPLICATION&&=$(_MTB_RECIPE__VSCODE_ELF_FILE_APPLICATION))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__HEX_FILE_APPLICATION&&=$(_MTB_RECIPE__VSCODE_HEX_FILE_APPLICATION))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__OPEN_OCD_FILE&&=$(_MTB_RECIPE__OPENOCD_DEVICE_CFG))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__OPENOCD_CHIP&&=$(_MTB_RECIPE__OPENOCD_CHIP_NAME))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__SVD_FILE_NAME&&=$(_MTB_RECIPE__VSCODE_SVD_PATH))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__APPLICATION_SVD_FILE_NAME&&=$(_MTB_RECIPE__VSCODE_APPLICATION_SVD_PATH))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__MTB_PATH&&=$(CY_TOOLS_DIR))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__OPENOCD_EXE_DIR_RELATIVE&&=$(CY_TOOL_openocd_EXE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__OPENOCD_SCRIPTS_DIR_RELATIVE&&=$(CY_TOOL_openocd_scripts_SCRIPT))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__DEVICE_ATTACH&&=$(_MTB_RECIPE__JLINK_DEVICE_CFG_ATTACH))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__MODUS_SHELL_BASE&&=$(CY_TOOL_modus-shell_BASE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__ATTACH_SERVER_TYPE&&=$(MTB_RECIPE__ATTACH_SERVER_TYPE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__GCC_BIN_DIR&&=$(_MTB_RECIPE__VSCODE_GCC_BASE_DIR)/bin)

.PHONY: recipe_vscode_common_text_replacement_data_file

##############################################
# Eclipse
##############################################
_MTB_RECIPE__ECLIPSE_OPENOCD_SVD_PATH:=$${cy_prj_path}/$(DEVICE_$(DEVICE)_SVD)
ifneq ($(CY_BUILD_LOCATION),)
_MTB_RECIPE__ECLIPSE_SYM_FILE:=$(_MTB_RECIPE__TARG_FILE)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$(_MTB_RECIPE__PROG_FILE)
ifeq ($(MTB_TYPE),PROJECT)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$(_MTB_RECIPE__APP_HEX_FILE)
endif
else
_MTB_RECIPE__ECLIPSE_SYM_FILE:=$${cy_prj_path}/$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$${cy_prj_path}/$(_MTB_RECIPE__IDE_BUILD_PATH_RELATIVE)/$(APPNAME).$(MTB_RECIPE__SUFFIX_PROGRAM)
ifeq ($(MTB_TYPE),PROJECT)
_MTB_RECIPE__ECLIPSE_PROG_FILE:=$${cy_prj_path}/../$(_MTB_RECIPE__IDE_COMBINED_HEX_RELATIVE)
endif
endif

# JLink path
ifneq (,$(MTB_JLINK_DIR))
ifneq (,$(MTB_CORE__JLINK_GDB_EXE))
_MTB_RECIPE__ECLIPSE_JLINK_EXE:=$(MTB_CORE__JLINK_GDB_EXE)
else
_MTB_RECIPE__ECLIPSE_JLINK_EXE:=$${jlink_path}/$${jlink_gdbserver}
endif
else
_MTB_RECIPE__ECLIPSE_JLINK_EXE:=$${jlink_path}/$${jlink_gdbserver}
endif

# GDB path
_MTB_RECIPE__ECLIPSE_GDB=$${cy_tools_path:CY_TOOL_arm-none-eabi-gdb_EXE}

# If a custom name needs to be provided for the IDE environment it can be specified by
# CY_IDE_PRJNAME. If CY_IDE_PRJNAME was not set on the command line, use APPNAME as the
# default. CY_IDE_PRJNAME can be important in some environments like eclipse where the
# name used within the project is not necessarily what the user created. This can happen
# in Eclipse if there is already a project with the desired name. In this case Eclipse
# will create its own name. That name must still be used for launch configurations instead
# of the name the user actually gave. It can also be necessary when there are multiple
# applications that get created for a single design. In either case we allow a custom name
# to be provided. If one is not provided, we will fallback to the default APPNAME.
ifeq ($(CY_IDE_PRJNAME),)
CY_IDE_PRJNAME=$(APPNAME)
_MTB_RECIPE__ECLIPSE_APPLICATION_NAME:=$(patsubst "%",%,$(MTB_APPLICATION_NAME))
else
# in a multi-core application, CY_IDE_PRJNAME is name selected in the project-creator and should only apply to the project
_MTB_RECIPE__ECLIPSE_APPLICATION_NAME:=$(CY_IDE_PRJNAME)
endif

ifeq ($(MTB_TYPE),PROJECT)
ifneq ($(MTB_APPLICATION_SUBPROJECTS),)
_MTB_RECIPE__ECLIPSE_PROJECT_NAME=$(_MTB_RECIPE__ECLIPSE_APPLICATION_NAME).$(APPNAME)
endif
else #($(MTB_TYPE),PROJECT)
_MTB_RECIPE__ECLIPSE_PROJECT_NAME=$(CY_IDE_PRJNAME)
endif #($(MTB_TYPE),PROJECT)

eclipse_generate: recipe_eclipse_common_text_replacement_data_file
eclipse_generate: MTB_CORE__EXPORT_CMDLINE += -textdata $(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE)

recipe_eclipse_common_text_replacement_data_file:
	$(call mtb__file_write,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__OPENOCD_CFG&&=$(_MTB_RECIPE__OPENOCD_DEVICE_CFG))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__OPENOCD_CHIP&&=$(_MTB_RECIPE__OPENOCD_CHIP_NAME))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__SVD_PATH&&=$(_MTB_RECIPE__ECLIPSE_OPENOCD_SVD_PATH))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__SYM_FILE&&=$(_MTB_RECIPE__ECLIPSE_SYM_FILE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__PROG_FILE&&=$(_MTB_RECIPE__ECLIPSE_PROG_FILE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__ECLIPSE_GDB&&=$(_MTB_RECIPE__ECLIPSE_GDB))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__ECLIPSE_JLINK_EXE&&=$(_MTB_RECIPE__ECLIPSE_JLINK_EXE))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__APP_NAME&&=$(_MTB_RECIPE__ECLIPSE_APPLICATION_NAME))
	$(call mtb__file_append,$(_MTB_RECIPE__IDE_COMMON_TEXT_DATA_FILE),&&_MTB_RECIPE__PRJ_NAME&&=$(_MTB_RECIPE__ECLIPSE_PROJECT_NAME))

.PHONY: recipe_eclipse_common_text_replacement_data_file

##############################################
# EW
##############################################
_MTB_RECIPE__IDE_DFP_DATA_FILE=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/recipe_ide_dfp_data.txt
ewarm8: MTB_CORE__EXPORT_CMDLINE+= -dfp_data $(_MTB_RECIPE__IDE_DFP_DATA_FILE)
ewarm8: recipe_ewarm_dfp_data_file

recipe_ewarm_dfp_data_file:
	$(call mtb__file_write,$(_MTB_RECIPE__IDE_DFP_DATA_FILE),DEVICE=$(DEVICE)$(_MTB_RECIPE__IAR_CORE_SUFFIX))

.PHONY: recipe_ewarm_dfp_data_file

