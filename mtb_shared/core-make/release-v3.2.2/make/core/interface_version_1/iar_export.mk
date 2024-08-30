################################################################################
# \file iar_export.mk
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

_MTB_CORE__IAR_BUILD_DATA_FILE:=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/iar_build_data.temp
_MTB_CORE__IAR_OUTFILE:=$(CY_IDE_PRJNAME).ipcf
_MTB_CORE__IAR_CYIGNORE_PATH:=$(MTB_TOOLS__PRJ_DIR)/.cyignore
_MTB_CORE__IAR_TEMPLATE_PATH:=$(MTB_TOOLS__CORE_DIR)/make/scripts/interface_version_1/iar

# Note: All paths are expected to be relative of the Makefile(Project Directory)
# the defines need to be sorted. IAR will throw an error if there are duplicate asm defines.
_MTB_CORE__IAR_DEFINES=$(foreach onedef,$(_MTB_CORE__IDE_DEFINES),\"$(onedef)\",)
_MTB_CORE__IAR_INCLUDES:=$(foreach onedef,$(_MTB_CORE__IDE_INCLUDES),"$(onedef)",)
_MTB_CORE__IAR_SOURCES_C_CPP:=$(foreach onedef,$(_MTB_CORE__IDE_SOURCES_C) $(_MTB_CORE__IDE_SOURCES_CPP) $(_MTB_CORE__IDE_SOURCES_CXX) $(_MTB_CORE__IDE_SOURCES_CC),"$(onedef)",)
_MTB_CORE__IAR_SOURCES_s_S:=$(foreach onedef,$(_MTB_CORE__IDE_SOURCES_s) $(_MTB_CORE__IDE_SOURCES_S),"$(onedef)",)
_MTB_CORE__IAR_HEADERS:=$(foreach onedef,$(_MTB_CORE__IDE_HEADERS),"$(onedef)",)
_MTB_CORE__IAR_LIBS:=$(foreach onedef,$(_MTB_CORE__IDE_LIBS),"$(onedef)",)
_MTB_CORE__IAR_SEARCHES:=$(foreach onedef,$(_MTB_CORE__FULL_SEARCH_ROOTS),"$(onedef)",)

ewarm8_build_data_file:
	$(call mtb__file_write,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(CY_IDE_PRJNAME))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(MTB_RECIPE__CORE))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(MTB_RECIPE__LINKER_SCRIPT))
	$(shell echo $(_MTB_CORE__IAR_DEFINES) >> $(_MTB_CORE__IAR_BUILD_DATA_FILE))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IAR_INCLUDES))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IAR_SOURCES_C_CPP))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IAR_SOURCES_s_S))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IAR_HEADERS))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IAR_LIBS))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IAR_SEARCHES))
	$(call mtb__file_append,$(_MTB_CORE__IAR_BUILD_DATA_FILE),$(_MTB_CORE__IDE_SHARED))

ewarm8: ewarm8_build_data_file CY_IDE_preprint $(_MTB_CORE__QBUILD_MK_FILE) $(MTB_TOOLS__OUTPUT_CONFIG_DIR)
ifneq ($(TOOLCHAIN), IAR)
	$(call mtb__error,Unable to proceed. TOOLCHAIN must be set to IAR. Use TOOLCHAIN=IAR on the command line, or edit the Makefile)
endif
ifeq ($(findstring ewarm8,$(MTB_RECIPE__IDE_SUPPORTED)),)
	$(call mtb__error,Unable to proceed. Export is not supported for this device)
endif
	$(MTB__NOISE)echo
	$(MTB__NOISE)$(CY_PYTHON_PATH) $(_MTB_CORE__IAR_TEMPLATE_PATH)/iar_export.py -build_data $(_MTB_CORE__IAR_BUILD_DATA_FILE) -recipe_data $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) -o $(MTB_TOOLS__PRJ_DIR)/$(_MTB_CORE__IAR_OUTFILE)
	$(MTB__NOISE)rm -rf $(_MTB_CORE__IAR_BUILD_DATA_FILE);\
	rm -rf $(MTB_RECIPE__IDE_RECIPE_DATA_FILE);\
	echo;\
	echo "Instructions:";\
	echo "1. Open IAR EW for Arm 8.x";\
	echo "2. Project->Create New Project...->Empty project";\
	echo "3. Finish creating the new empty project";\
	echo "4. Project->Add Project Connection...";\
	echo "5. Navigate to the app directory and open the .ipcf";\
	echo ;\
	echo "The following flags will be automatically added to the IAR ewarm project:";\
	echo "C Compiler Flags: $(MTB_RECIPE__CFLAGS)";\
	echo "C++ Compiler Flags: $(MTB_RECIPE__CXXFLAGS)";\
	echo "Assembler Flags: $(MTB_RECIPE__ASFLAGS)";\
	echo "Linker Flags: $(MTB_RECIPE__LDFLAGS)";\
	echo;\
	echo "To add additional build options: See Project->Options->C/C++ Compiler->Extra Options, Project->Options->Assembler->Extra Options, and Project->Options->Linker->Extra Options.";\
	echo;
ifneq ($(CFLAGS)$(CXXFLAGS)$(ASFLAGS)$(LDFLAGS),)
	$(MTB__NOISE)echo -e "\033[31mThe following Flags are not automatically added to the IAR ewarm project and must be added manually:\e[0m";
endif
ifneq ($(CFLAGS),)
	$(MTB__NOISE)echo -e "\033[31mC Compiler Flags: $(CFLAGS)\e[0m";
endif
ifneq ($(CXXFLAGS),)
	$(MTB__NOISE)echo -e "\033[31mC++ Compiler Flags: $(CXXFLAGS)\e[0m";
endif
ifneq ($(ASFLAGS),)
	$(MTB__NOISE)echo -e "\033[31mAssembler Flags: $(ASFLAGS)\e[0m";
endif
ifneq ($(LDFLAGS),)
	$(MTB__NOISE)echo -e "\033[31mLinker Flags: $(LDFLAGS)\e[0m";
endif
	$(MTB__NOISE)echo;\
	echo $(_MTB_CORE__IDE_PREBUILD_MSG);\
	echo $(_MTB_CORE__IDE_POSTBUILD_MSG);\
	echo "Some applications require additional customization to be functional in IAR EWARM environment. Check ModusToolbox user guide section 'Export IAR EWARM' for more details"
ifneq (,$(filter MW_ABSTRACTION_RTOS,$(COMPONENTS)))
# Note: If the RTOS-specific flags set in IAR.mk are modified, this section should be updated to reflect the changes.
	$(MTB__NOISE)echo;\
	echo "WARNING: Since FreeRTOS is enabled for this project, the compiler and linker settings must be manually updated in IAR EW.";\
	echo "Option 1: Set the project options";\
	echo "    1. Project->Options->General Options->Library Configuration";\
	echo "    2. Set the \"Library:\" dropdown to \"Full\"";\
	echo "    3. Check the box for \"Enable thread support in library\"";\
	echo "    4. Click \"OK\"";\
	echo "Option 2: Set the compiler and linker flags";\
	echo "    1. Project->Options->C/C++ Compiler->Extra Options";\
	echo "    2. Check the box for \"Use command line options\"";\
	echo "    3. Enter \"--dlib_config=full\" in the text box";\
	echo "    4. Project->Options->Linker->Extra Options";\
	echo "    5. Check the box for \"Use command line options\"";\
	echo "    6. Enter \"--threaded_lib\" in the text box";\
	echo "    7. Click \"OK\"";\
	echo;
endif
	$(MTB__NOISE)if  [ ! -f $(_MTB_CORE__IAR_CYIGNORE_PATH) ] || ! grep -q 'Debug' $(_MTB_CORE__IAR_CYIGNORE_PATH) || ! grep -q 'Release' $(_MTB_CORE__IAR_CYIGNORE_PATH);\
	then \
		echo;\
		echo Note: Added default IAR-EW output folders \"Debug\" and \"Release\" to $(_MTB_CORE__IAR_CYIGNORE_PATH) file. \
		For custom IAR output directories, manually add them to the $(_MTB_CORE__IAR_CYIGNORE_PATH) file to exclude them from auto-discovery.; \
		echo >> $(_MTB_CORE__IAR_CYIGNORE_PATH);\
		echo "# Automatically added by ewarm8 make target" >> $(_MTB_CORE__IAR_CYIGNORE_PATH);\
		echo "Debug" >> $(_MTB_CORE__IAR_CYIGNORE_PATH);\
		echo "Release" >> $(_MTB_CORE__IAR_CYIGNORE_PATH);\
		echo;\
	fi;

.PHONY: ewarm8 ewarm8_build_data_file
