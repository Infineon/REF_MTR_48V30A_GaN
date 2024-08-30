################################################################################
# \file ide.mk
#
# \brief
# IDE-specific targets and variables
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
# IDE common
################################################################################

#
# Print information before file generation
#
CY_IDE_preprint:
	$(info )
	$(info ==============================================================================)
	$(info = Generating IDE files =)
	$(info ==============================================================================)

$(MTB_TOOLS__OUTPUT_CONFIG_DIR):
	$(MTB__NOISE)mkdir -p $(MTB_TOOLS__OUTPUT_CONFIG_DIR);

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
_MTB_ECLIPSE_APPLICATION_NAME=$(patsubst "%",%,$(MTB_APPLICATION_NAME))
else
# in a multi-core application, CY_IDE_PRJNAME is name selected in the project-creator and should only apply to the project
_MTB_ECLIPSE_APPLICATION_NAME=$(CY_IDE_PRJNAME)
endif

# Shared repo vars
_MTB_CORE__IDE_SHARED=$(patsubst %/,%,$(CY_GETLIBS_SHARED_PATH))/$(CY_GETLIBS_SHARED_NAME)

# DEFINES
_MTB_CORE__IDE_DEFINES=$(patsubst -D%,%,$(MTB_RECIPE__DEFINES))

# INCLUDES
_MTB_CORE__IDE_INCLUDES=$(patsubst -I%,%,$(MTB_RECIPE__INCLUDES))

# SOURCES
_MTB_CORE__IDE_SOURCES=$(MTB_RECIPE__SOURCE) $(CY_RECIPE_GENERATED) $(SOURCES)
_MTB_CORE__IDE_SOURCES_C=$(filter %.$(MTB_RECIPE__SUFFIX_C),$(_MTB_CORE__IDE_SOURCES))
_MTB_CORE__IDE_SOURCES_CPP=$(filter %.$(MTB_RECIPE__SUFFIX_CPP),$(_MTB_CORE__IDE_SOURCES))
_MTB_CORE__IDE_SOURCES_CXX=$(filter %.$(MTB_RECIPE__SUFFIX_CXX),$(_MTB_CORE__IDE_SOURCES))
_MTB_CORE__IDE_SOURCES_CC=$(filter %.$(MTB_RECIPE__SUFFIX_CC),$(_MTB_CORE__IDE_SOURCES))
_MTB_CORE__IDE_SOURCES_s=$(filter %.$(MTB_RECIPE__SUFFIX_s),$(_MTB_CORE__IDE_SOURCES))
_MTB_CORE__IDE_SOURCES_S=$(filter %.$(MTB_RECIPE__SUFFIX_S),$(_MTB_CORE__IDE_SOURCES))

# HEADERS
_MTB_CORE__IDE_HEADERS=$(_MTB_CORE__SEACH_APP_HEADERS)

# LIBS
_MTB_CORE__IDE_LIBS=$(MTB_RECIPE__LIBS)

_MTB_CORE__IDE_PREBUILD_MSG=Note: Building the application runs "make prebuild". You may want to include that as part of the project prebuild steps.

_MTB_CORE__IDE_POSTBUILD_MSG=Note: Building the application runs "make postbuild". You may want to include their content as part of the project postbuild steps.

ifeq ($(strip $(filter 3 2 1,$(MTB__MAKE_MAJOR_VER))),)
# --output-sync argument is only supported on GNU make-4.0 or newer
_MTB_CORE__IDE_OUTPUT_SYNC=--output-sync
endif

################################################################################
# Eclipse
################################################################################

ifeq ($(filter eclipse,$(MAKECMDGOALS)),eclipse)
include $(MTB_TOOLS__CORE_DIR)/make/core/interface_version_1/eclipse_export.mk
endif # ifeq ($(filter eclipse,$(MAKECMDGOALS)),eclipse)

################################################################################
# IAR
################################################################################

ifeq ($(filter ewarm8,$(MAKECMDGOALS)),ewarm8)
include $(MTB_TOOLS__CORE_DIR)/make/core/interface_version_1/iar_export.mk
endif #ifeq ($(filter ewarm8,$(MAKECMDGOALS)),ewarm8)


################################################################################
# CMSIS Project Description files (*.cpdsc and *.gpdsc)
################################################################################

ifeq ($(filter uvision5,$(MAKECMDGOALS)),uvision5)
include $(MTB_TOOLS__CORE_DIR)/make/core/interface_version_1/cmsis_export.mk
endif # ifeq ($(filter uvision5,$(MAKECMDGOALS)),uvision5)


################################################################################
# VSCode
################################################################################

ifeq ($(filter vscode,$(MAKECMDGOALS)),vscode)
include $(MTB_TOOLS__CORE_DIR)/make/core/interface_version_1/vscode_export.mk
endif

#
# Identify the phony targets
#
.PHONY: CY_IDE_preprint $(MTB_TOOLS__OUTPUT_CONFIG_DIR)
