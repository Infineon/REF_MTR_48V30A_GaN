################################################################################
# \file get_app_info.mk
#
# \brief
# Print data required by mtbquery API to file.
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

_MTB_CORE__SUPPORTED_TOOL_IDS:=$(DEVICE_TOOL_IDS) $(CY_SUPPORTED_TOOL_TYPES)

# a temp file, so that get_app_info_data.mk only is modified if it has changed.
_MTB_CORE__GET_APP_INFO_TEMP_FILE:=$(MTB_TOOLS__OUTPUT_BASE_DIR)/get_app_info.temp
$(info $(shell mkdir -p $(MTB_TOOLS__OUTPUT_BASE_DIR)))
$(call mtb__file_write,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_MPN_LIST=$(MPN_LIST))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_DEVICE_LIST=$(DEVICE_LIST))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_DEVICE=$(DEVICE))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_SEARCH=$(MTB_TOOLS__SEARCH))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_TOOLCHAIN=$(TOOLCHAIN))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_TARGET=$(TARGET))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_CONFIG=$(CONFIG))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_APP_NAME=$(APPNAME)$(LIBNAME))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_COMPONENTS=$(MTB_CORE__FULL_COMPONENT_LIST))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_DISABLED_COMPONENTS=$(DISABLE_COMPONENTS))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_ADDITIONAL_DEVICES=$(ADDITIONAL_DEVICES))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_LIBS=$(CY_GETLIBS_PATH))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_DEPS=$(CY_GETLIBS_DEPS_PATH))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_WKS_SHARED_NAME=$(CY_GETLIBS_SHARED_NAME))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_WKS_SHARED_DIR=$(CY_GETLIBS_SHARED_PATH))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_FLOW_VERSION=$(FLOW_VERSION))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_QUERY=$(MTB_CORE__MTB_QUERY))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_TOOLS_DIR=$(MTB_TOOLS__TOOLS_DIR))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_DEVICE_PROGRAM_IDS=$(_MTB_CORE__SUPPORTED_TOOL_IDS))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_BSP_TOOL_TYPES=$(_MTB_CORE__SUPPORTED_TOOL_ID))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_MW_TOOL_TYPES=)
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_IGNORE=$(strip $(CY_IGNORE) $(MTB_TOOLS__OUTPUT_BASE_DIR)))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_TYPE=$(MTB_TYPE))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_CORE_TYPE=$(MTB_RECIPE__CORE))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_CORE_NAME=$(MTB_RECIPE__CORE_NAME))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_BUILD_SUPPORT=$(MTB_BUILD_SUPPORT))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_CACHE_DIR=$(MTB_TOOLS__CACHE_DIR))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_OFFLINE_DIR=$(MTB_TOOLS__OFFLINE_DIR))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_GLOBAL_DIR=$(MTB_TOOLS__GLOBAL_DIR))
$(call mtb__file_append,$(_MTB_CORE__GET_APP_INFO_TEMP_FILE),MTB_APP_PATH=$(MTB_TOOLS__REL_PRJ_PATH))

# If we have new "app info" replace the old info with the temporary info
$(info $(shell cmp -s "$(_MTB_CORE__GET_APP_INFO_TEMP_FILE)" "$(_MTB_CORE__GET_APP_INFO_DATA_FILE)" || cp -f "$(_MTB_CORE__GET_APP_INFO_TEMP_FILE)" "$(_MTB_CORE__GET_APP_INFO_DATA_FILE)"))
