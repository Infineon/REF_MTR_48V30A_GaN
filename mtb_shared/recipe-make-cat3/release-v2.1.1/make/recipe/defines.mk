################################################################################
# \file defines.mk
#
# \brief
# Defines, needed for the XMC build recipe.
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

include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/defines_common.mk


################################################################################
# General
################################################################################
_MTB_RECIPE__DEFAULT_PROGRAM_INTERFACE:=JLink
_MTB_RECIPE__PROGRAM_INTERFACE_SUPPORTED:=JLink
#
# Compactibility interface for this recipe make
#
MTB_RECIPE__INTERFACE_VERSION:=2

#
# List the supported toolchains
#
CY_SUPPORTED_TOOLCHAINS:=GCC_ARM IAR

#
# Family specifics
#
ifeq (XMC1100,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC1
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc1100
_MTB_RECIPE__XMC_SERIES:=XMC1100

else ifeq (XMC1200,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC1
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc1200
_MTB_RECIPE__XMC_SERIES:=XMC1200

else ifeq (XMC1300,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC1
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc1300
_MTB_RECIPE__XMC_SERIES:=XMC1300

else ifeq (XMC1400,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC1
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc1400
_MTB_RECIPE__XMC_SERIES:=XMC1400

else ifneq (,$(findstring $(_MTB_RECIPE__DEVICE_DIE),XMC4100 XMC4104 XMC4108))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4100
_MTB_RECIPE__XMC_SERIES:=XMC4100

else ifeq (XMC4200,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4200
_MTB_RECIPE__XMC_SERIES:=XMC4200

else ifeq (XMC4300,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4300
_MTB_RECIPE__XMC_SERIES:=XMC4300

else ifeq (XMC4400,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4400
_MTB_RECIPE__XMC_SERIES:=XMC4400

else ifneq (,$(findstring $(_MTB_RECIPE__DEVICE_DIE),XMC4500 XMC4502 XMC4504))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4500
_MTB_RECIPE__XMC_SERIES:=XMC4500

else ifeq (XMC4700,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4700
_MTB_RECIPE__XMC_SERIES:=XMC4700

else ifeq (XMC4800,$(_MTB_RECIPE__DEVICE_DIE))
_MTB_RECIPE__XMC_ARCH:=XMC4
_MTB_RECIPE__OPENOCD_CHIP_NAME:=xmc4800
_MTB_RECIPE__XMC_SERIES:=XMC4800

else
$(call mtb__error,Incorrect part number $(DEVICE). Check DEVICE variable.)
endif

#
# Architecture specifics
#
ifeq ($(_MTB_RECIPE__XMC_ARCH),XMC1)
_MTB_RECIPE__START_FLASH:=0x10001000
_MTB_RECIPE__OPENOCD_DEVICE_CFG:=xmc1xxx.cfg
_MTB_RECIPE__JLINK_DEVICE_CFG_ATTACH:=$(DEVICE)
else ifeq ($(_MTB_RECIPE__XMC_ARCH),XMC4)
_MTB_RECIPE__START_FLASH:=0x0C000000
_MTB_RECIPE__OPENOCD_DEVICE_CFG:=xmc4xxx.cfg
_MTB_RECIPE__JLINK_DEVICE_CFG_ATTACH:=Cortex-M4
endif

# Add the series name to the standard components list
# to enable auto-discovery of CMSIS startup templates
MTB_RECIPE__COMPONENT+=$(_MTB_RECIPE__DEVICE_DIE)


################################################################################
# Tools specifics
################################################################################

ifneq (,$(findstring $(_MTB_RECIPE__DEVICE_DIE),XMC1100 XMC1200 XMC1300 XMC1400))
MTB_RECIPE__SIM_URL_RAW="https://design.infineon.com/tinaui/designer.php?path=EXAMPLESROOT%7CINFINEON%7CApplications%7CIndustrial%7C&file=mcu_$(_MTB_RECIPE__XMC_SERIES)_Boot_Kit_MTB_v2.tsc"

ifeq ($(OS),Windows_NT)
# escape the & with ^&
MTB_RECIPE__SIM_URL=$(subst &,^&,$(MTB_RECIPE__SIM_URL_RAW))
else
MTB_RECIPE__SIM_URL=$(MTB_RECIPE__SIM_URL_RAW)
endif
MTB_RECIPE__SIM_GEN_SUPPORTED=1
endif
