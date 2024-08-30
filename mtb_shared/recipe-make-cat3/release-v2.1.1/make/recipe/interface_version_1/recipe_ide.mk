################################################################################
# \file recipe_ide.mk
#
# \brief
# This make file defines the IDE export variables and target.
#
################################################################################
# \copyright
# Copyright 2022-2023 Cypress Semiconductor Corporation
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
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/interface_version_1/recipe_ide_common.mk

################################################################################
# IDE specifics
################################################################################

MTB_RECIPE__IDE_SUPPORTED:=eclipse vscode uvision5 ewarm8

ifeq ($(filter vscode,$(MAKECMDGOALS)),vscode)
$(MTB_RECIPE__IDE_RECIPE_DATA_FILE)_vscode:
	$(MTB__NOISE)echo "s|&&DEVICE&&|$(DEVICE)|g;" > $(MTB_RECIPE__IDE_RECIPE_DATA_FILE);\
	echo "s|&&_MTB_RECIPE__OPENOCD_CHIP&&|$(_MTB_RECIPE__OPENOCD_CHIP_NAME)|g;" >> $(MTB_RECIPE__IDE_RECIPE_DATA_FILE);
ifeq ($(MTB_RECIPE__CORE),CM0)
	$(MTB__NOISE)echo "s|&&_MTB_RECIPE__APP_ENTRY_POINT&&|main|g;" >> $(MTB_RECIPE__IDE_RECIPE_DATA_FILE);
else
	$(MTB__NOISE)echo "s|&&_MTB_RECIPE__APP_ENTRY_POINT&&||g;" >> $(MTB_RECIPE__IDE_RECIPE_DATA_FILE);
endif
endif

ifeq ($(filter eclipse,$(MAKECMDGOALS)),eclipse)
eclipse_textdata_file:
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__JLINK_CFG_PROGRAM&&=$(DEVICE))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__JLINK_CFG_DEBUG&&=$(DEVICE))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),&&_MTB_RECIPE__JLINK_CFG_ATTACH&&=$(_MTB_RECIPE__JLINK_DEVICE_CFG_ATTACH))

_MTB_ECLIPSE_TEMPLATE_RECIPE_SEARCH:=$(MTB_TOOLS__RECIPE_DIR)/make/scripts/interface_version_1/eclipse
_MTB_ECLIPSE_TEMPLATE_RECIPE_APP_SEARCH:=$(MTB_TOOLS__RECIPE_DIR)/make/scripts/interface_version_1/eclipse/Application

eclipse_recipe_metadata_file:
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),RECIPE_TEMPLATE=$(_MTB_ECLIPSE_TEMPLATE_RECIPE_SEARCH))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),RECIPE_APP_TEMPLATE=$(_MTB_ECLIPSE_TEMPLATE_RECIPE_APP_SEARCH))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),PROJECT_UUID=&&PROJECT_UUID&&)

endif

ewarm8_recipe_data_file:
	$(call mtb__file_write,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),$(DEVICE))

ewarm8: ewarm8_recipe_data_file

ifeq ($(_MTB_RECIPE__XMC_ARCH),XMC1)
_MTB_RECIPE__CMSIS_ARCH_NAME:=XMC1000_DFP
else ifeq ($(_MTB_RECIPE__XMC_ARCH),XMC4)
_MTB_RECIPE__CMSIS_ARCH_NAME:=XMC4000_DFP
endif
_MTB_RECIPE__CMSIS_VENDOR_NAME:=Infineon
_MTB_RECIPE__CMSIS_VENDOR_ID:=7
# pName is optional in the DFP and cprj. But they must match.
# pName is not specified in the XMC dfp, so we have to also not specify in the generated cprj file.
# If we specified the pName, the generated cprj file will not work.
_MTB_RECIPE__CMSIS_PNAME:=

_MTB_RECIPE__CMSIS_LDFLAGS:=

uvision5_recipe_data_file:
	$(call mtb__file_write,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),$(_MTB_RECIPE__CMSIS_ARCH_NAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),$(_MTB_RECIPE__CMSIS_VENDOR_NAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),$(_MTB_RECIPE__CMSIS_VENDOR_ID))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),$(_MTB_RECIPE__CMSIS_PNAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_DATA_FILE),$(_MTB_RECIPE__CMSIS_LDFLAGS))

uvision5: uvision5_recipe_data_file