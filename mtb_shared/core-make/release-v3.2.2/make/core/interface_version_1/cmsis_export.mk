################################################################################
# \file cmsis_export.mk
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

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif


_MTB_CORE__CMSIS_BUILD_DATA_FILE=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/cmsisdata.temp
_MTB_CORE__CMSIS_CPDSC=$(MTB_TOOLS__PRJ_DIR)/$(CY_IDE_PRJNAME).cpdsc
_MTB_CORE__CMSIS_GPDSC=$(MTB_TOOLS__PRJ_DIR)/$(CY_IDE_PRJNAME).gpdsc
_MTB_CORE__CMSIS_CPRJ=$(MTB_TOOLS__PRJ_DIR)/$(CY_IDE_PRJNAME).cprj
_MTB_CORE__CMSIS_CYIGNORE_PATH=$(MTB_TOOLS__PRJ_DIR)/.cyignore
_MTB_CORE__CMSIS_TEMPLATE_PATH=$(MTB_TOOLS__CORE_DIR)/make/scripts/interface_version_1/cmsis

# All paths are expected to be relative of the Makefile(Project Directory)
_MTB_CORE__CMSIS_DEFINES=$(foreach onedef,$(_MTB_CORE__IDE_DEFINES),\"$(onedef)\",)
_MTB_CORE__CMSIS_INCLUDES=$(foreach onedef,$(_MTB_CORE__IDE_INCLUDES),"$(onedef)",)
_MTB_CORE__CMSIS_SOURCES_C_CPP=$(foreach onedef,$(_MTB_CORE__IDE_SOURCES_C) $(_MTB_CORE__IDE_SOURCES_CPP) $(_MTB_CORE__IDE_SOURCES_CXX) $(_MTB_CORE__IDE_SOURCES_CC),"$(onedef)",)
_MTB_CORE__CMSIS_SOURCES_s_S=$(foreach onedef,$(_MTB_CORE__IDE_SOURCES_s) $(_MTB_CORE__IDE_SOURCES_S),"$(onedef)",)
_MTB_CORE__CMSIS_HEADERS=$(foreach onedef,$(_MTB_CORE__IDE_HEADERS),"$(onedef)",)
_MTB_CORE__CMSIS_LIBS=$(foreach onedef,$(_MTB_CORE__IDE_LIBS),"$(onedef)",)
_MTB_CORE__CMSIS_SEARCHES=$(foreach onedef,$(_MTB_CORE__FULL_SEARCH_ROOTS),"$(onedef)",)

ifeq ($(TOOLCHAIN), GCC_ARM)
_MTB_CORE__CMSIS_MESSAGE_uvision_gcc=WARNING: GCC support in Keil uVision is experimental. To use ARM Compiler 6, run: make uvision5 TOOLCHAIN=ARM.
$(eval $(call CY_MACRO_WARNING,_MTB_CORE__CMSIS_MESSAGE_uvision_gcc,$(_MTB_CORE__CMSIS_MESSAGE_uvision_gcc)))
else ifneq ($(TOOLCHAIN), ARM)
$(call mtb__error,Unable to proceed. TOOLCHAIN must be set to ARM. Use TOOLCHAIN=ARM on the command line or edit the Makefile)
endif

uvision5_build_data_file:
	$(call mtb__file_write,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(CY_IDE_PRJNAME))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(DEVICE))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(MTB_RECIPE__LINKER_SCRIPT))
	$(shell echo $(_MTB_CORE__CMSIS_DEFINES) >> $(_MTB_CORE__CMSIS_BUILD_DATA_FILE))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__CMSIS_INCLUDES))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__CMSIS_SOURCES_C_CPP))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__CMSIS_SOURCES_s_S))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__CMSIS_HEADERS))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__CMSIS_LIBS))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__CMSIS_SEARCHES))
	$(call mtb__file_append,$(_MTB_CORE__CMSIS_BUILD_DATA_FILE),$(_MTB_CORE__IDE_SHARED))

uvision5: uvision5_build_data_file CY_IDE_preprint $(_MTB_CORE__QBUILD_MK_FILE) $(MTB_TOOLS__OUTPUT_CONFIG_DIR)
ifeq ($(findstring uvision5,$(MTB_RECIPE__IDE_SUPPORTED)),)
	$(call mtb__error,Unable to proceed. Export is not supported for this device)
endif
	$(MTB__NOISE)echo
	$(MTB__NOISE)$(CY_PYTHON_PATH) $(_MTB_CORE__CMSIS_TEMPLATE_PATH)/cmsis_export.py -build_data $(_MTB_CORE__CMSIS_BUILD_DATA_FILE) -recipe_data $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) -cpdsc $(_MTB_CORE__CMSIS_CPDSC) -gpdsc $(_MTB_CORE__CMSIS_GPDSC) -cprj $(_MTB_CORE__CMSIS_CPRJ)
	$(MTB__NOISE)rm -rf $(_MTB_CORE__CMSIS_BUILD_DATA_FILE);\
	echo Keil uVision version \<\= 5.29: double-click the .cpdsc file. The .gpdsc file is loaded automatically.;\
	echo Keil uVision version \>\= 5.30: double-click the .cprj file. The .gpdsc file is loaded automatically.;\
	echo;\
	echo $(_MTB_CORE__IDE_PREBUILD_MSG);\
	echo $(_MTB_CORE__IDE_POSTBUILD_MSG)
ifeq ($(TOOLCHAIN), GCC_ARM)
	$(MTB__NOISE)echo;\
	echo To switch the project to use GCC toolchain, open Project - Manage - Project Items - Folders/Extensions, and set GCC prefix and path.
endif
	$(MTB__NOISE)if  [ ! -f $(_MTB_CORE__CMSIS_CYIGNORE_PATH) ] || ! grep -q "$(CY_IDE_PRJNAME)_build" $(_MTB_CORE__CMSIS_CYIGNORE_PATH) || ! grep -q "RTE" $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
	then \
		echo;\
		echo Note: Added Keil uVision5 generated folders \"$(CY_IDE_PRJNAME)_build\", \"$(CY_IDE_PRJNAME)_Listings\", \"$(CY_IDE_PRJNAME)_Object\" and \"RTE\" to $(_MTB_CORE__CMSIS_CYIGNORE_PATH) file. \
		For custom output directories, manually add them to the $(_MTB_CORE__CMSIS_CYIGNORE_PATH) file to exclude them from auto-discovery.; \
		echo >> $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
		echo "# Automatically added by uvision5 make target" >> $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
		echo "$(CY_IDE_PRJNAME)_build" >> $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
		echo "$(CY_IDE_PRJNAME)_Listings" >> $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
		echo "$(CY_IDE_PRJNAME)_Objects" >> $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
		echo "RTE" >> $(_MTB_CORE__CMSIS_CYIGNORE_PATH);\
	fi;\
	echo;

.PHONY: uvision5 uvision5_build_data_file
