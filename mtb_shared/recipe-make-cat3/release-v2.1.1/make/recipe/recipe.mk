################################################################################
# \file recipe.mk
#
# \brief
# Set up a set of defines, includes, software components, linker script, 
# Pre and Post build steps and call a macro to create a specific ELF file.
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

include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/recipe_common.mk

# Aclang arguments must match the symbols in the PDL makefile
_MTB_RECIPE__ACLANG_POSTBUILD=\
	$(MTB_TOOLS__RECIPE_DIR)/make/scripts/m2bin \
	--verbose --vect $(VECT_BASE_CM0P) --text $(TEXT_BASE_CM0P) --data $(RAM_BASE_CM0P) --size $(TEXT_SIZE_CM0P)\
	$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).mach_o\
	$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).bin

progtool:
	$(call mtb__error,make progtool is not supported for the current device.)

.PHONY: progtool
