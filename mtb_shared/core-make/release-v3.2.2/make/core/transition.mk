################################################################################
# \file transition.mk
#
# \brief
# Perform device transition for device that support secure modes.
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

device_transition_default:
	$(error Device transitions are unnecessary and unsupported for $(DEVICE).)
# Transition the device from one mode to another
device_transition: $(if $(BSP_DEVICE_TRANSITION_TARGET),$(BSP_DEVICE_TRANSITION_TARGET),$(if $(RECIPE_DEVICE_TRANSITION_TARGET),$(RECIPE_DEVICE_TRANSITION_TARGET),device_transition_default))

.PHONY: device_transition_default device_transition
