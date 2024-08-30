################################################################################
# \file build.mk
#
# \brief
# Performs the compilation and linking steps.
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
# Defines
################################################################################

# Define application target
CY_PREBUILD_TARGET=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).$(MTB_RECIPE__SUFFIX_TARGET)

################################################################################
# Targets
################################################################################

_mtb_print_start_build_message:
	@:
	$(info Initializing build: $(APPNAME)$(LIBNAME) $(CONFIG) $(TARGET) $(TOOLCHAIN))

prebuild $(_MTB_CORE__QBUILD_MK_FILE)|: _mtb_print_start_build_message

$(_MTB_CORE__FORCEBUILD_MK_FILE): prebuild

_mtb_build_prebuild_mkdirs:
	$(MTB__NOISE)echo; \
	mkdir -p $(MTB_TOOLS__OUTPUT_CONFIG_DIR) $(MTB__SILENT_OUTPUT)

recipe_prebuild: _mtb_build_prebuild_mkdirs

bsp_prebuild: recipe_prebuild

project_prebuild: bsp_prebuild

_mtb_build_prebuild_postprint: project_prebuild
	$(MTB__NOISE)echo "Prebuild operations complete"

#
# Top-level prebuild dependency
#
prebuild: _mtb_build_prebuild_postprint

.PHONY: prebuild recipe_prebuild project_prebuild bsp_prebuild qprebuild _mtb_print_start_build_message
.PHONY: _mtb_build_prebuild_mkdirs _mtb_build_prebuild_postprint
