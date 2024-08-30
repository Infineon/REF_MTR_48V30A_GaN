################################################################################
# \file search.mk
#
# \brief
# Parses the data from cyqbuild.mk to generate useful build variables
#
################################################################################
# \copyright
# Copyright 2021-2023 Cypress Semiconductor Corporation
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


# Fix a bug related to CONFIG_ directory not being properly filtered.
#
# Filters for configurations
# $(1) : List of files of a certain file type
# $(2) : Filter string
#
_MTB_CORE__MACRO_MATCH_CONFIGURATION=$(strip $(foreach item,$(1),$(if $(findstring $(2),/$(item)/),$(item),)))
_MTB_CORE__MACRO_REMOVE_CONFIGURATION=$(strip $(foreach item,$(1),$(if $(findstring $(2),/$(item)),,$(item))))
_MTB_CORE__MACRO_FILTER_CONFIGURATION=$(call _MTB_CORE__MACRO_REMOVE_CONFIGURATION,$(1),/$(strip $(2))_)\
							$(call _MTB_CORE__MACRO_MATCH_CONFIGURATION,$(1),/$(strip $(2))_$($(strip $(2)))/)\
							$(strip $(if $(findstring -,$($(strip $(2)))),\
							$(call _MTB_CORE__MACRO_MATCH_CONFIGURATION,$(1),/$(strip $(2))_$(subst -,_,$($(strip $(2))))/),\
							))

CY_SEARCH_ALL_FILES:=$(call _MTB_CORE__MACRO_FILTER_CONFIGURATION,$(CY_SEARCH_ALL_FILES),CONFIG)
CY_SEARCH_ALL_INCLUDES:=$(call _MTB_CORE__MACRO_FILTER_CONFIGURATION,$(CY_SEARCH_ALL_INCLUDES),CONFIG)

_MTB_CORE__SEARCH_SOURCE_EXT=$(foreach ext,$(MTB_RECIPE__SUFFIX_C) $(MTB_RECIPE__SUFFIX_S) $(MTB_RECIPE__SUFFIX_s) $(MTB_RECIPE__SUFFIX_C) $(MTB_RECIPE__SUFFIX_CPP) $(MTB_RECIPE__SUFFIX_CXX) $(MTB_RECIPE__SUFFIX_CC),.$(ext))
_MTB_CORE__SEARCH_LIB_EXT=$(foreach ext,$(MTB_RECIPE__SUFFIX_O) $(MTB_RECIPE__SUFFIX_A),.$(ext))
_MTB_CORE__SEARCH_HEADER_EXT=$(foreach ext,$(MTB_RECIPE__SUFFIX_H) $(MTB_RECIPE__SUFFIX_HPP),.$(ext))

MTB_CORE__SEARCH_APP_SOURCE:=$(sort $(filter $(foreach ext,$(_MTB_CORE__SEARCH_SOURCE_EXT),%$(ext)),$(CY_SEARCH_ALL_FILES)))
MTB_CORE__SEARCH_APP_LIBS:=$(sort $(filter $(foreach ext,$(_MTB_CORE__SEARCH_LIB_EXT),%$(ext)),$(CY_SEARCH_ALL_FILES)))
MTB_CORE__SEARCH_APP_INCLUDES:=$(CY_SEARCH_ALL_INCLUDES)
_MTB_CORE__SEACH_APP_HEADERS:=$(sort $(filter $(foreach ext, $(_MTB_CORE__SEARCH_HEADER_EXT),%$(ext)),$(CY_SEARCH_ALL_FILES)))
_MTB_CORE__SEARCH_EXT_SOURCE_ASSET=$(sort $(filter ../%,$(MTB_CORE__SEARCH_APP_SOURCE)))
