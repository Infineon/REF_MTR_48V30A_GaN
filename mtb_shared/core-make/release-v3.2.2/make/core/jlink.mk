################################################################################
# \file jlink.mk
#
# \brief
# JLink Path handling
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

# default install path on windows
_MTB_CORE__JLINK_DEFAULT_GDB_WINDOWS:=C:/Program Files/SEGGER/JLink/JLinkGDBServerCL.exe
# default installp path on macos
_MTB_CORE__JLINK_DEFAULT_GDB_OSX:=/Applications/SEGGER/JLink/JLinkGDBServerCLExe
# There is no default install path on linux, just tgz archive.
_MTB_CORE__JLINK_DEFAULT_GDB_LINUX:=JLinkGDBServerCLExe

ifneq (,$(MTB_JLINK_DIR))
# if MTB_JLINK_DIR is set, look for the JLinkGDBServerCL.exe (windows) or JLinkGDBServerCLExe (unix) there
MTB_CORE__JLINK_GDB_EXE:=$(wildcard $(call mtb_core__escaped_path,$(MTB_JLINK_DIR))/JLinkGDBServerCL.exe)
ifeq (,$(MTB_CORE__JLINK_GDB_EXE))
MTB_CORE__JLINK_GDB_EXE:=$(wildcard $(call mtb_core__escaped_path,$(MTB_JLINK_DIR))/JLinkGDBServerCLExe)
endif
MTB_CORE__JLINK_EXE:=$(wildcard $(call mtb_core__escaped_path,$(MTB_JLINK_DIR))/JLink.exe)
ifeq (,$(MTB_CORE__JLINK_EXE))
MTB_CORE__JLINK_EXE:=$(wildcard $(call mtb_core__escaped_path,$(MTB_JLINK_DIR))/JLinkExe)
endif
else #(,$(JLINK_DIR))
# if MTB_JLINK_DIR is not set, look for it in the user PATH env var.
MTB_CORE__JLINK_GDB_EXE:=$(call mtb__get_file_path,,JLinkGDBServerCL.exe)
ifeq (,$(MTB_CORE__JLINK_GDB_EXE))
MTB_CORE__JLINK_GDB_EXE:=$(call mtb__get_file_path,,JLinkGDBServerCLExe)
endif
MTB_CORE__JLINK_EXE:=$(call mtb__get_file_path,,JLink.exe)
ifeq (,$(MTB_CORE__JLINK_EXE))
MTB_CORE__JLINK_EXE:=$(call mtb__get_file_path,,JLinkExe)
endif
endif #(,$(JLINK_DIR))

# If JLink executable is not found, it will be set to empty.
