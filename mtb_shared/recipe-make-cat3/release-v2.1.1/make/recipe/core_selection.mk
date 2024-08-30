################################################################################
# \file core_selection.mk
#
# \brief
# Determine which MCU core is being targeted.
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

# 
# CORE
#   - The type of ARM core used by the application.
#   - May be set by user in Makefile or by a BSP.
#   - If not set, assume CM0P.
#   - Valid COREs are determined by the selected toolchain. 
#     Currently this includes: CM0, CM0P, CM4, and.
#
#
# Core specifics
#
ifneq (1,$(words $(DEVICE_$(DEVICE)_CORES)))
$(call mtb__error,Incorrect cores: "$(DEVICE_$(DEVICE)_CORES)". Check DEVICE_$(DEVICE)_CORES variable.)
endif

MTB_RECIPE__CORE_NAME:=$(patsubst CORE_NAME_%,%,$(DEVICE_$(DEVICE)_CORES))
MTB_RECIPE__CORE:=$(patsubst %_0,%,$(MTB_RECIPE__CORE_NAME))

COMPONENTS+=$(MTB_RECIPE__CORE) $(MTB_RECIPE__CORE_NAME)
