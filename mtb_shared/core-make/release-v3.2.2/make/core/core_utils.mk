################################################################################
# \file core_utils.mk
#
# \brief
# Global utilities used across the application recipes and BSPs
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
# Components
################################################################################

#
# VFP-specific component
#
ifeq ($(VFP_SELECT),hardfp)
CY_COMPONENT_VFP:=HARDFP
else
CY_COMPONENT_VFP:=SOFTFP
endif

MTB_CORE__FULL_COMPONENT_LIST=$(sort $(MTB_RECIPE__CORE) $(MTB_RECIPE__CORE_NAME) $(CY_COMPONENT_VFP) $(COMPONENTS) $(TOOLCHAIN) $(TARGET) $(CONFIG) $(MTB_RECIPE__COMPONENT) $(DEVICE_COMPONENTS) $(BSP_COMPONENTS))

_MTB_CORE__FULL_SEARCH_ROOTS=$(strip $(SEARCH) $(SEARCH_MTB_MK))

################################################################################
# Macros
################################################################################

#
# Prints for bypassing TARGET/DEVICE checks
# $(1) : String to print
#
ifneq (,$(filter build build_proj qbuild qbuild_proj program program_proj debug,$(MAKECMDGOALS)))
_MTB_CORE__FAIL_ON_ERROR:=true
endif
ifeq ($(_MTB_CORE__FAIL_ON_ERROR),true)
mtb__error=$(error $(1))
else
mtb__error=$(info WARNING: $(1))
endif

#
# Get unquoted path with escaped spaces
# $(1) : path for which quotes and escapes should be removed but spaces should be escaped
#
mtb_core__escaped_path=$(subst $(MTB__OPEN_PAREN),\$(MTB__OPEN_PAREN),$(subst $(MTB__CLOSE_PAREN),\$(MTB__CLOSE_PAREN),$(subst $(MTB__SPACE),\$(MTB__SPACE),$(1))))


# escape " and \ for json
mtb_core__json_escaped_string=$(subst ",\",$(subst \,\\,$(strip $1)))

#
# Prints the warning and creates a variable to hold that warning (for printing later)
# Note that this doesn't use the $(warning) function as that adds the line number (not useful for end user)
# $(1) : Message ID
# $(2) : String to print
#
define CY_MACRO_WARNING
$(info )
$(info $(2))
CY_WARNING_$(1)=$(2)
endef

#
# Prints the info and creates a variable to hold that info (for printing later)
# $(1) : Message ID
# $(2) : String to print
#
define CY_MACRO_INFO
$(info )
$(info $(2))
CY_INFO_$(1)=$(2)
endef

################################################################################
# Misc.
################################################################################

# Create a maker that can be used by a replace operation to insert a newline
MTB__NEWLINE_MARKER:=__!__

################################################################################
# Utility targets
################################################################################

bsp:
	@:
	$(error Make bsp target is no longer supported. Use BSP assistant tool instead.)

update_bsp:
	@:
	$(error Make bsp target is no longer supported. Use BSP assistant tool instead.)


################################################################################
# Test/debug targets
################################################################################

CY_TOOLS_LIST+=bash git find ls cp mkdir rm cat sed awk perl file whereis

check:
	@:
	$(info )
	$(foreach tool,$(CY_TOOLS_LIST),$(if $(shell which $(tool)),\
		$(info SUCCESS: "$(tool)" found in PATH),$(info FAILED : "$(tool)" was not found in PATH)$(info )))
	$(info )
	$(info Tools check complete.)
	$(info )

get_env_info:
	$(MTB__NOISE)echo;\
	echo "make location :" $$(which make);\
	echo "make version  :" $(MAKE_VERSION);\
	echo "git location  :" $$(which git);\
	echo "git version   :" $$(git --version);\
	echo "git remote    :";\
	git remote -v;\
	echo "git rev-parse :" $$(git rev-parse HEAD)

printlibs:

# Defined in recipe's program.mk
progtool:

# Empty libs on purpose. May be defined by the application
shared_libs:

ifeq ($(CY_PROTOCOL),)
MTB_CORE__CY_PROTOCOL_VERSION:=2
else
MTB_CORE__CY_PROTOCOL_VERSION:=$(CY_PROTOCOL)
endif
MTB_CORE__SUPPORTED_PROTOCAL_VERSIONS=1

ifeq ($(MTB_QUERY),)
# undefined MTB_QUERY. Use the latest
MTB_CORE__MTB_QUERY=$(lastword $(MTB_CORE__SUPPORTED_PROTOCAL_VERSIONS))
# MTB_QUERY version is supported
else
ifeq ($(filter $(MTB_QUERY),$(MTB_CORE__SUPPORTED_PROTOCAL_VERSIONS)),$(MTB_QUERY))
MTB_CORE__MTB_QUERY=$(MTB_QUERY)
else
# MTB_QUERY is newer than max supported version. Use the latest
MTB_CORE__MTB_QUERY=$(lastword $(MTB_CORE__SUPPORTED_PROTOCAL_VERSIONS))
$(warning Requested MTB_QUERY version is newer than is supported.)
endif
endif

# CY_PROTOCOl=2, MTB_QUERY=1. Supports ModusToolbox 3.0
get_app_info_2_1:
	@:
	$(MTB__NOISE)cat $(_MTB_CORE__GET_APP_INFO_DATA_FILE)

get_app_info: get_app_info_$(MTB_CORE__CY_PROTOCOL_VERSION)_$(MTB_CORE__MTB_QUERY)
	@:

.PHONY: get_app_info get_app_info_2_1

#
# Identify the phony targets
#
.PHONY: bsp update_bsp check get_env_info get_app_info printlibs shared_libs
