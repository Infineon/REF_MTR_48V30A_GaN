################################################################################
# \file search.mk
#
# \brief
# Performs create cyqbuild.mk file by calling mtbsearch
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

# the data file that get_app_info generates
_MTB_CORE__GET_APP_INFO_DATA_FILE:=$(MTB_TOOLS__OUTPUT_BASE_DIR)/get_app_info.txt

_MTB_CORE__QBUILD_MK_FILE=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/cyqbuild.mk
_MTB_CORE__FORCEBUILD_MK_FILE=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/cyforcebuild.mk

_MTB_CORE__CODE_GEN_FLAG:=$(if $(SKIP_CODE_GEN),,--generate)

# arguments for mtbsearch
_MTB_CORE__SEARCH_CMD=$(CY_TOOL_mtbsearch_EXE_ABS) --project $(MTB_TOOLS__PRJ_DIR) @$(_MTB_CORE__GET_APP_INFO_DATA_FILE) @MTB_TOOLS_DIR=$(MTB_TOOLS__TOOLS_DIR) $(_MTB_CORE__CODE_GEN_FLAG) -o $(_MTB_CORE__QBUILD_MK_FILE)

# generate the cyqbuild.mk file
$(_MTB_CORE__FORCEBUILD_MK_FILE) $(_MTB_CORE__QBUILD_MK_FILE): $(_MTB_CORE__GET_APP_INFO_DATA_FILE)
	$(info )
	$(info Auto-discovery in progress...)
	$(MTB__NOISE)mkdir -p $(MTB_TOOLS__OUTPUT_CONFIG_DIR)
	$(MTB__NOISE)$(_MTB_CORE__SEARCH_CMD)
	$(MTB__NOISE)echo Auto-discovery complete

.PHONY: $(_MTB_CORE__FORCEBUILD_MK_FILE)
