################################################################################
# \file config.mk
#
# \brief
# Configurator-related routines
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
# online simulator launch
################################################################################

##########################
# online-simulator
##########################

# CY_ALL_TOOLS_DIRS wildcard/filter-out tools that don't exist on disk. Provide an absolute path to these tools so that they won't be filtered out
ifeq ($(OS),Windows_NT)
_MTB_CORE__SIM_CMD=$(subst \,/,${COMSPEC})
 _MTB_CORE__SIM_ARGS=/c start
else
ifneq ($(findstring Darwin,$(shell uname)),)
_MTB_CORE__SIM_CMD=/usr/bin/open
_MTB_CORE__SIM_ARGS=
else
_MTB_CORE__SIM_CMD=/usr/bin/xdg-open
_MTB_CORE__SIM_ARGS=
endif
endif


online_simulator:
ifeq ($(MTB_RECIPE__SIM_URL),)
	$(error $(MTB__NEWLINE)Infineon simulator not supported for the current device)
else
	$(if $(wildcard $(_MTB_CORE__SIM_CMD)),,$(error $(_MTB_CORE__SIM_CMD) not found. The online simulator be accessed through the following URL: $(MTB_RECIPE__SIM_URL_RAW)))
	$(info $(MTB__NEWLINE)Opening the Infineon online simulator $(MTB_RECIPE__SIM_URL_RAW))
	$(MTB__NOISE) $(_MTB_CORE__SIM_CMD) $(_MTB_CORE__SIM_ARGS) $(MTB_RECIPE__SIM_URL) $(MTB__JOB_BACKGROUND)
endif

.PHONY: online_simulator
