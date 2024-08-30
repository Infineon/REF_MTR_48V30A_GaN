################################################################################
# \file eclipse_export.mk
#
# \brief
# IDE-specific targets and variables
#
################################################################################
# \copyright
# Copyright 2022-2023 Cypress Semiconductor Corporation
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

ifeq ($(CY_IDE_PRJNAME),)
CY_MESSAGE_prjname=WARNING: No value set for CY_IDE_PRJNAME. APPNAME "$(APPNAME)" will be used instead.\
This may cause launch configurations to not show up if the name Eclipse uses for the project differs.
$(eval $(call CY_MACRO_WARNING,CY_MESSAGE_prjname,$(CY_MESSAGE_prjname)))
endif

_MTB_CORE__ECLIPSE_TEMPLATE_PATH=$(MTB_TOOLS__CORE_DIR)/make/scripts/interface_version_1/eclipse

# Source files outside of the project directory
_MTB_CORE__ECLIPSE_SOURCES_INTERNAL:=$(filter-out $(MTB_TOOLS__PRJ_DIR)/%, $(abspath $(SOURCES)))
_MTB_CORE__ECLIPSE_INCLUDES_INTERNAL:=$(filter-out $(MTB_TOOLS__PRJ_DIR)/%, $(abspath $(INCLUDES)))

_MTB_CORE__ECLIPSE_SOURCES:=$(call mtb__path_normalize,$(_MTB_CORE__ECLIPSE_SOURCES_INTERNAL))
_MTB_CORE__ECLIPSE_INCLUDES:=$(call mtb__path_normalize,$(_MTB_CORE__ECLIPSE_INCLUDES_INTERNAL))

ifeq ($(MTB_TYPE),PROJECT)
ifneq ($(MTB_APPLICATION_SUBPROJECTS),)
_MTB_ECLIPSE_PROJECT_NAME=$(_MTB_ECLIPSE_APPLICATION_NAME).$(APPNAME)
endif
else #($(MTB_TYPE),PROJECT)
_MTB_ECLIPSE_PROJECT_NAME=$(CY_IDE_PRJNAME)
endif #($(MTB_TYPE),PROJECT)


################################################################################
# eclipse targets
################################################################################

ifeq ($(MTB_CORE__APPLICATION_BOOTSTRAP),true)
# Need to force the other cores in multi-core to not skip first stage.
eclipse_application_bootstrap:
	$(MTB__NOISE)$(MAKE) -C .. eclipse CY_SECONDSTAGE=

eclipse: eclipse_application_bootstrap
else
# Whether the recipe support eclipse export


ifeq ($(LIBNAME),)
eclipse: eclipse_generate
ifeq ($(findstring eclipse,$(MTB_RECIPE__IDE_SUPPORTED)),)
	$(call mtb__error,Unable to proceed. Export is not supported for this device)
endif
ifeq ($(CY_TOOL_mtbideexport_EXE_ABS),)
	$(call mtb__error,Unable to proceed. Eclipse export executable not found)
endif
else #($(LIBNAME),)
eclipse:
	@:
endif

# The the export export executable
eclipse_generate: CY_IDE_preprint $(MTB_TOOLS__OUTPUT_CONFIG_DIR) eclipse_metadata_file eclipse_textdata_file
	$(MTB__NOISE)$(CY_TOOL_mtbideexport_EXE_ABS) -ide eclipse -metadata $(MTB_RECIPE__IDE_RECIPE_METADATA_FILE) -textdata $(MTB_RECIPE__IDE_RECIPE_DATA_FILE)
	$(MTB__NOISE)rm -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $(MTB_RECIPE__IDE_RECIPE_METADATA_FILE)


eclipse_metadata_file: eclipse_recipe_metadata_file
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),MTB_APPLICATION_SUBPROJECTS=$(MTB_APPLICATION_SUBPROJECTS))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),APPNAME=$(_MTB_ECLIPSE_PROJECT_NAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),MTB_APPLICATION_NAME=$(_MTB_ECLIPSE_APPLICATION_NAME))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),EXTERNAL_SOURCES=$(_MTB_CORE__ECLIPSE_SOURCES))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),EXTERNAL_INCLUDES=$(strip $(_MTB_CORE__ECLIPSE_INCLUDES)))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),SEARCH_DIRS=$(_MTB_CORE__FULL_SEARCH_ROOTS))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),MTB_SHARED_SEARCH=$(_MTB_CORE__IDE_SHARED))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),PROJECT_TEMPLATE=$(_MTB_CORE__ECLIPSE_TEMPLATE_PATH))
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),PROTOCOL_VERSION=1)
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),APPLICATION_UUID=&&APPLICATION_UUID&&)
	$(call mtb__file_append,$(MTB_RECIPE__IDE_RECIPE_METADATA_FILE),REGENERATE_PROJECT=$(MTB_MAKE_ECLIPSE_FORCE_REGENERATE_PROJECT))

endif #($(_MTB_CORE__ECLIPSE_CALL_APPLICATION_ECLIPSE),true)

.PHONY: eclipse eclipse_metadata_file eclipse_textdata_file eclipse_application_bootstrap
