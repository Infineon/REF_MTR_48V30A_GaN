################################################################################
# \file defines_common.mk
#
# \brief
# Common variables and targets for defines.mk
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

################################################################################
# BSP
################################################################################

ifneq (1,$(words $(DEVICE_$(DEVICE)_CORES)))
_MTB_RECIPE__IS_MULTI_CORE_DEVICE:=true
endif

ifneq (,$(filter StandardSecure,$(DEVICE_$(DEVICE)_FEATURES)))
_MTB_RECIPE__IS_SECURE_DEVICE:=true
endif
ifneq (,$(filter SecureBoot,$(DEVICE_$(DEVICE)_FEATURES)))
_MTB_RECIPE__IS_SECURE_DEVICE:=true
endif

_MTB_RECIPE__DEVICE_DIE:=$(DEVICE_$(DEVICE)_DIE)

_MTB_RECIPE__DEVICE_FLASH_KB:=$(DEVICE_$(DEVICE)_FLASH_KB)

################################################################################
# Paths
################################################################################

ifeq ($(MTB_TYPE),PROJECT)
# Application build output is one level up relative to the project directory
_MTB_RECIPE__APP_HEX_DIR:=../build
_MTB_RECIPE__PRJ_HEX_DIR:=$(_MTB_RECIPE__APP_HEX_DIR)/project_hex

# Override the hex path for qprogram_proj target
ifneq ($(MTB_APPLICATION_SUBPROJECTS),)
_MTB_RECIPE__APP_HEX_FILE:=$(call mtb__path_normalize,$(_MTB_RECIPE__APP_HEX_DIR)/app_combined.$(MTB_RECIPE__SUFFIX_PROGRAM))
endif
endif
