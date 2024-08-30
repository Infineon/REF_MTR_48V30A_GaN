################################################################################
# \file vscode_export.mk
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

################################################################################
# vscode export defines
################################################################################

CY_VSCODE_OUT_PATH=$(MTB_TOOLS__PRJ_DIR)/.vscode
CY_VSCODE_APPLICATION_OUT_PATH=$(MTB_TOOLS__PRJ_DIR)/../.vscode
# temporary directory for perform processing
CY_VSCODE_OUT_TEMPLATE_PATH=$(CY_VSCODE_OUT_PATH)/cytemplates
_MTB_CORE__VSCODE_BACKUP_PATH=$(CY_VSCODE_OUT_PATH)/backup
_MTB_CORE__VSCODE_APPLICATION_BACKUP_PATH=$(MTB_TOOLS__PRJ_DIR)/../.vscode/backup
# The location for the template files in core-make
_MTB_CORE__VSCODE_TEMPLATE_PATH=$(MTB_TOOLS__CORE_DIR)/make/scripts/interface_version_1/vscode
# Only include if using separate core-make and recipe-make
ifneq ($(MTB_TOOLS__RECIPE_DIR),$(MTB_TOOLS__CORE_DIR))
# The location for the template files in recipe-make
CY_VSCODE_TEMPLATE_RECIPE_PATH=$(MTB_TOOLS__RECIPE_DIR)/make/scripts/interface_version_1/vscode
endif
# A temp file to store sed replacement data
MTB_RECIPE__IDE_RECIPE_DATA_FILE_2=$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/vscode_launch2.temp
_MTB_CORE__VSCODE_WORKSPACE_NAME=$(CY_IDE_PRJNAME).code-workspace
CY_VSCODE_WORKSPACE_TEMPLATE_NAME?=wks.code-workspace
_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME=$(MTB_APPLICATION_NAME).code-workspace

_MTB_CORE__VSCODE_INCLUDES=$(foreach onedef,$(patsubst -I%,%,$(subst ",\\\\\\\",$(subst \,\\\\\\\\,$(_MTB_CORE__IDE_INCLUDES)))),\"$(onedef)\",)
_MTB_CORE__VSCODE_INCLUDES_LIST=$(patsubst %$(MTB__COMMA),%,$(subst $(MTB__SPACE),$(MTB__NEWLINE_MARKER),$(_MTB_CORE__VSCODE_INCLUDES)))

_MTB_CORE__VSCODE_DEFINES=$(foreach onedef,$(patsubst -D%,%,$(subst ",\\\\\\\",$(subst \,\\\\\\\\,$(_MTB_CORE__IDE_DEFINES)))),\"$(onedef)\",)
_MTB_CORE__VSCODE_DEFINES_LIST=$(patsubst %$(MTB__COMMA),%,$(subst $(MTB__SPACE),$(MTB__NEWLINE_MARKER),$(_MTB_CORE__VSCODE_DEFINES)))

# Toolchain-specific VFP and CPU settings for c_cpp_properties.json
ifeq ($(TOOLCHAIN),GCC_ARM)

ifeq ($(VFP_SELECT),hardfp)
_MTB_CORE__VSCODE_VFP=/hard
else
_MTB_CORE__VSCODE_VFP=/softfp
endif

ifeq ($(MTB_RECIPE__CORE),CM0)
_MTB_CORE__VSCODE_CPU=/arm-none-eabi/thumb/v6e-m
_MTB_CORE__VSCODE_VFP=
else ifeq ($(MTB_RECIPE__CORE),CM0P)
_MTB_CORE__VSCODE_CPU=/arm-none-eabi/thumb/v6e-m
_MTB_CORE__VSCODE_VFP=
else ifeq ($(MTB_RECIPE__CORE),CM4)
_MTB_CORE__VSCODE_CPU=/arm-none-eabi/thumb/v7e-m/fpv4-sp
else ifeq ($(MTB_RECIPE__CORE),CM33)
_MTB_CORE__VSCODE_CPU=/arm-none-eabi/thumb/v8-m.main/fpv5-sp
else
_MTB_CORE__VSCODE_CPU=
_MTB_CORE__VSCODE_VFP=
endif

else ifeq ($(TOOLCHAIN),IAR)

_MTB_CORE__VSCODE_VFP=

# Note: IAR requires intrinsic defines. Use the following to generate them to intrinsic.txt file
# CFLAGS= --predef_macros intrinsic.txt
# Then list only the difference here
ifneq ($(filter $(MTB_RECIPE__CORE),CM0 CM0P),)
_MTB_CORE__VSCODE_CPU=$(subst $(MTB__SPACE),$(MTB__NEWLINE_MARKER),$(foreach onedef,\
            __ARM_ARCH=6\
            __ARM_ARCH_ISA_THUMB=1\
            __ARM_FEATURE_COPROC=0\
            __CORE__=__ARM6M__\
            __JMP_BUF_NUM_ELEMENTS__=8\
            __STDC_NO_ATOMICS__=1\
            __SUBNORMAL_FLOATING_POINTS__=0\
            ,\"$(onedef)\",))
else ifeq ($(MTB_RECIPE__CORE),CM4)
_MTB_CORE__VSCODE_VFP=$(subst $(MTB__SPACE),$(MTB__NEWLINE_MARKER),$(foreach onedef,\
            __ARMVFPV1__=1\
            __ARMVFPV2__=2\
            __ARMVFPV3_D16__=1\
            __ARMVFPV3_FP16__=1\
            __ARMVFPV3__=3\
            __ARMVFPV4__=4\
            __ARMVFP_D16__=1\
            __ARMVFP_FP16__=1\
            __ARMVFP_SP__=1\
            __ARMVFP__=__ARMVFPV4__\
            ,\"$(onedef)\",))
_MTB_CORE__VSCODE_CPU=$(subst $(MTB__SPACE),$(MTB__NEWLINE_MARKER),$(foreach onedef,\
            __ARM6MEDIA__=6\
            __ARM6T2__=6\
            __ARM7EM__=13\
            __ARM7M__=7\
            __ARM7__=7\
            __ARM_ARCH=7\
            __ARM_ARCH_ISA_THUMB=2\
            __ARM_FEATURE_CLZ=1\
            __ARM_FEATURE_COPROC=15\
            __ARM_FEATURE_DSP=1\
            __ARM_FEATURE_FMA=1\
            __ARM_FEATURE_IDIV=1\
            __ARM_FEATURE_LDREX=7\
            __ARM_FEATURE_QBIT=1\
            __ARM_FEATURE_SAT=1\
            __ARM_FEATURE_SIMD32=1\
            __ARM_FEATURE_UNALIGNED=1\
            __ARM_FP=6\
            __ARM_MEDIA__=1\
            __ATOMIC_BOOL_LOCK_FREE=2\
            __ATOMIC_CHAR16_T_LOCK_FREE=2\
            __ATOMIC_CHAR32_T_LOCK_FREE=2\
            __ATOMIC_CHAR_LOCK_FREE=2\
            __ATOMIC_INT_LOCK_FREE=2\
            __ATOMIC_LLONG_LOCK_FREE=0\
            __ATOMIC_LONG_LOCK_FREE=2\
            __ATOMIC_POINTER_LOCK_FREE=2\
            __ATOMIC_SHORT_LOCK_FREE=2\
            __ATOMIC_WCHAR_T_LOCK_FREE=2\
            __CORE__=__ARM7EM__\
            __EDG_TYPE_TRAITS_ENABLED=1\
            __JMP_BUF_NUM_ELEMENTS__=16\
            __MEMORY_ORDER_ACQUIRE__=2\
            __MEMORY_ORDER_ACQ_REL__=4\
            __MEMORY_ORDER_CONSUME__=1\
            __MEMORY_ORDER_RELAXED__=0\
            __MEMORY_ORDER_RELEASE__=3\
            __MEMORY_ORDER_SEQ_CST__=5\
            __SUBNORMAL_FLOATING_POINTS__=1\
            ,\"$(onedef)\",))
else ifeq ($(MTB_RECIPE__CORE),CM7)
_MTB_CORE__VSCODE_VFP=$(subst $(CY_SPACE),$(CY_NEWLINE_MARKER),$(foreach onedef,\
            __ARMFPV5__=5\
            __ARMVFPV1__=1\
            __ARMVFPV2__=2\
            __ARMVFPV3_D16__=1\
            __ARMVFPV3_FP16__=1\
            __ARMVFPV3__=3\
            __ARMVFPV4__=4\
            __ARMVFP_D16__=1\
            __ARMVFP_FP16__=1\
            __ARMVFP_SP__=1\
            __ARMVFP__=__ARMFPV5__\
            ,\"$(onedef)\",))
_MTB_CORE__VSCODE_CPU=$(subst $(CY_SPACE),$(CY_NEWLINE_MARKER),$(foreach onedef,\
            __ARM6MEDIA__=6\
            __ARM6T2__=6\
            __ARM7EM__=13\
            __ARM7M__=7\
            __ARM7__=7\
            __ARM_ARCH=7\
            __ARM_ARCH_ISA_THUMB=2\
            __ARM_FEATURE_CLZ=1\
            __ARM_FEATURE_COPROC=15\
            __ARM_FEATURE_DSP=1\
            __ARM_FEATURE_FMA=1\
            __ARM_FEATURE_IDIV=1\
            __ARM_FEATURE_LDREX=7\
            __ARM_FEATURE_QBIT=1\
            __ARM_FEATURE_SAT=1\
            __ARM_FEATURE_SIMD32=1\
            __ARM_FEATURE_UNALIGNED=1\
            __ARM_MEDIA__=1\
            __ATOMIC_BOOL_LOCK_FREE=2\
            __ATOMIC_CHAR16_T_LOCK_FREE=2\
            __ATOMIC_CHAR32_T_LOCK_FREE=2\
            __ATOMIC_CHAR_LOCK_FREE=2\
            __ATOMIC_INT_LOCK_FREE=2\
            __ATOMIC_LLONG_LOCK_FREE=0\
            __ATOMIC_LONG_LOCK_FREE=2\
            __ATOMIC_POINTER_LOCK_FREE=2\
            __ATOMIC_SHORT_LOCK_FREE=2\
            __ATOMIC_WCHAR_T_LOCK_FREE=2\
            __CORE__=__ARM7EM__\
            __EDG_INT128_EXTENSIONS_ALLOWED=0\
            __ICCARM_OLD_DEFINED_VAARGS__=1\
            __JMP_BUF_NUM_ELEMENTS__=8\
            __MEMORY_ORDER_ACQUIRE__=2\
            __MEMORY_ORDER_ACQ_REL__=4\
            __MEMORY_ORDER_CONSUME__=1\
            __MEMORY_ORDER_RELAXED__=0\
            __MEMORY_ORDER_RELEASE__=3\
            __MEMORY_ORDER_SEQ_CST__=5\
            __SUBNORMAL_FLOATING_POINTS__=1\
            __VA_STACK_ALIGN__=8\
            __VTABLE_MEM__=\
            ,\"$(onedef)\",))
else ifeq ($(MTB_RECIPE__CORE),CM33)
_MTB_CORE__VSCODE_CPU=
else
_MTB_CORE__VSCODE_CPU=
endif

endif #ifeq ($(TOOLCHAIN),GCC_ARM)

# Path of project
_MTB_VSCODE_PROJECT_DIR_NAME=$(notdir $(realpath $(MTB_TOOLS__PRJ_DIR)))

_MTB_VSCODE_TARGET_BASE_DEPENDENCIES=$(_MTB_CORE__VSCODE_BACKUP_PATH) $(CY_VSCODE_OUT_TEMPLATE_PATH) $(MTB_RECIPE__IDE_RECIPE_DATA_FILE)

# make build -j argument. Default value is 2
_MTB_CORE___VSCODE_BUILD_NUM_PROCESSOR:=2
ifeq ($(OS),Windows_NT)
_MTB_CORE___VSCODE_BUILD_NUM_PROCESSOR:=$(strip $(word 1,$(NUMBER_OF_PROCESSORS) 2))
else
_MTB_CORE__VSCODE_UNAME:=$(shell uname -s)
ifeq ($(_MTB_CORE__VSCODE_UNAME),Linux)
_MTB_CORE___VSCODE_BUILD_NUM_PROCESSOR:=$(shell nproc)
else ifeq ($(_MTB_CORE__VSCODE_UNAME),Darwin)
_MTB_CORE___VSCODE_BUILD_NUM_PROCESSOR:=$(shell sysctl -n hw.logicalcpu)
endif
endif

################################################################################
# vscode multi-core defines
################################################################################

ifeq ($(MTB_TYPE),COMBINED)
# single core application
_MTB_VSCODE_GENERATE_PROJECT_FILES=true
endif #($(MTB_TYPE),COMBINED)
ifeq ($(MTB_TYPE),LEGACY)
# legacy project ModusToolbox 2.X
_MTB_VSCODE_GENERATE_PROJECT_FILES=true
endif #($(MTB_TYPE),LEGACY)
ifeq ($(MTB_TYPE),)
# legacy project ModusToolbox 2.X
_MTB_VSCODE_GENERATE_PROJECT_FILES=true
endif #($(MTB_TYPE),)

ifeq ($(MTB_TYPE),PROJECT)
ifneq ($(MTB_APPLICATION_SUBPROJECTS),)
_MTB_VSCODE_GENERATE_PROJECT_FILES=true
_MTB_VSCODE_IS_MULTI_CORE_APPLICATION=true
ifeq ($(word 1,$(MTB_APPLICATION_SUBPROJECTS)),$(_MTB_VSCODE_PROJECT_DIR_NAME))
# the project is the first core, generate the application level files and the project specific files.
_MTB_VSCODE_GENERATE_APPLICATION_FILES=true
endif #($(word 1,$(MTB_APPLICATION_SUBPROJECTS)),$(APPNAME))
endif #($(MTB_APPLICATION_SUBPROJECTS),)
endif #($(MTB_TYPE),PROJECT)

_MTB_VSCODE_APPLICATION_TARGET_BASE_DEPENDENCIES=$(_MTB_CORE__VSCODE_APPLICATION_BACKUP_PATH) $(CY_VSCODE_OUT_TEMPLATE_PATH) $(MTB_RECIPE__IDE_RECIPE_DATA_FILE)

#
# Generate the compilation database (cdb) file that is used by the .vscode/c_cpp_properties.json file
#
# Note: VSCode .cdb file needs to be known in multiple make files
ifneq ($(CY_BUILD_LOCATION),)
_MTB_CORE__VSCODE_CDB_FILE:=$(_MTB_CORE__CDB_FILE)
else
_MTB_CORE__VSCODE_CDB_FILE:=\$$\{workspaceFolder\}/$(notdir $(MTB_TOOLS__OUTPUT_BASE_DIR))/compile_commands.json
endif

################################################################################
# vscode targets
################################################################################

ifeq ($(MTB_CORE__APPLICATION_BOOTSTRAP),true)
# Need to force the other cores in multi-core to not skip first stage.
vscode_application_bootstrap:
	$(MTB__NOISE)$(MAKE) -C .. vscode CY_SECONDSTAGE=

vscode: vscode_application_bootstrap

_MTB_VSCODE_SKIP_CLEAN_UP=true
endif

ifeq ($(_MTB_VSCODE_GENERATE_PROJECT_FILES),true)
vscode: vsocde_project_gen CY_IDE_preprint _mtb_build_cdb_postprint
endif

ifeq ($(_MTB_VSCODE_GENERATE_APPLICATION_FILES),true)
vscode: vscode_application_gen CY_IDE_preprint _mtb_build_cdb_postprint
endif

# the replace string for mtb-shared in to be generated into the workspace file
_MTB_CORE__VSCODE_SEARCH=

ifeq ($(_MTB_VSCODE_IS_MULTI_CORE_APPLICATION),true)
vscode: vscode_application_sed CY_IDE_preprint _mtb_build_cdb_postprint
ifeq ($(_MTB_VSCODE_GENERATE_APPLICATION_FILES),true)
vscode: vscode_application_gen
vscode_application_workspace_sed:vscode_application_workspace_gen
endif
_MTB_CORE__VSCODE_SHARED=$(patsubst ../%,%,$(_MTB_CORE__IDE_SHARED))
else #($(_MTB_VSCODE_IS_MULTI_CORE_APPLICATION),true)
vscode: vscode_project_workspace
_MTB_CORE__VSCODE_SHARED=$(_MTB_CORE__IDE_SHARED)
endif #($(_MTB_VSCODE_IS_MULTI_CORE_APPLICATION),true)

ifneq ($(_MTB_CORE__VSCODE_SHARED),)
_MTB_CORE__VSCODE_SEARCH=\,\\n\\t\\t{\\n\\t\\t\\t\"path\": \"$(_MTB_CORE__VSCODE_SHARED)\"\\n\\t\\t}
endif

# temporary directory to store intermediate processing file
$(CY_VSCODE_OUT_TEMPLATE_PATH):
	$(MTB__NOISE)mkdir -p $(CY_VSCODE_OUT_TEMPLATE_PATH);
	
# generate a backup directory to move existing vscode export files if they already exists.
$(_MTB_CORE__VSCODE_BACKUP_PATH):
	$(MTB__NOISE)mkdir -p $(_MTB_CORE__VSCODE_BACKUP_PATH);

# generate a backup directory to move exsting application vscode export files if they already exists.
$(_MTB_CORE__VSCODE_APPLICATION_BACKUP_PATH):
	$(MTB__NOISE)mkdir -p $(_MTB_CORE__VSCODE_APPLICATION_BACKUP_PATH);

# generate bash sed replacement file. Used by the main vscode export
# mark this target also as phony to ensure that it gets regenerated if the file already exists.
$(MTB_RECIPE__IDE_RECIPE_DATA_FILE): $(MTB_TOOLS__OUTPUT_CONFIG_DIR) $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2)

# Split sed arguments into 2 files.
# macOS sed can only make so many substitutions in a single invocation.
$(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2):
	$(MTB__NOISE)echo "s|&&_MTB_CORE__VSCODE_CPU&&|$(_MTB_CORE__VSCODE_CPU)|" > $@;\
	echo "s|&&_MTB_CORE__VSCODE_VFP&&|$(_MTB_CORE__VSCODE_VFP)|" >> $@;\
	echo "s|&&_MTB_RECIPE__INCLUDE_LIST&&|$(_MTB_CORE__VSCODE_INCLUDES_LIST)|" >> $@;\
	echo "s|&&_MTB_RECIPE__DEFINE_LIST&&|$(_MTB_CORE__VSCODE_DEFINES_LIST)|"   >> $@;\
	echo "s|&&_MTB_RECIPE__CC&&|$(CC)|" >> $@;\
	echo "s|&&_MTB_CORE__VSCODE_CDB_FILE&&|$(_MTB_CORE__VSCODE_CDB_FILE)|" >> $@;\
	echo "s|&&_MTB_RECIPE__SEARCH_DIRS&&|$(_MTB_CORE__VSCODE_SEARCH)|" | sed s/'\\t'/'    '/g | sed s/'\\n'/'$(MTB__NEWLINE_MARKER)'/g >> $@;\
	echo "s|&&_MTB_CORE___VSCODE_BUILD_NUM_PROCESSOR&&|$(_MTB_CORE___VSCODE_BUILD_NUM_PROCESSOR)|" >> $@;\
	echo "s|&&_MTB_CORE__IDE_OUTPUT_SYNC&&|$(_MTB_CORE__IDE_OUTPUT_SYNC)|" >> $@;\
	echo;

################################################################################
# single-core targets
################################################################################

# When updating the settings.json file, we have to account for all of the different ways that the user may have
# added information to this file.  We will perform these steps to do that:
#    1. Create a new file (TMP1) with our settings and comments removed from the old settings.json file
#    2. Create a new file (TMP2) from TMP1 with all user comments removed
#    3. Delete all empty lines and trailing whitespace from the TMP2 file
#    4. Pull the last line (lastLine) from the TMP2 file
#    5. If the lastLine doesn't have a comma at the end, find that line in the TMP1 file and replace it with the same
#       line with a comma at the end
#    6. Create a new settings.json file with the information in the TMP1 file
#    7. Tack on our updated comments and settings to that new settings.json file
vscode_project_settings_json: $(_MTB_VSCODE_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)json="$(_MTB_CORE__VSCODE_TEMPLATE_PATH)/settings.json";\
	jsonFile="$${json##*/}";\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $$json | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile;\
	$(CY_VSCODE_JSON_PROCESSING)\
	if [ -f $(CY_VSCODE_OUT_PATH)/$$jsonFile ]; then\
		echo "Modifying existing settings.json file. Check against the backup copy in .vscode/backup";\
		mv -f $(CY_VSCODE_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile;\
		sed -e '/^{/d'\
			-e '/\/\/mtb\/\//d'\
			-e '/modustoolbox.toolsPath/d'\
			-e '/cortex-debug.armToolchainPath/d'\
			-e '/cortex-debug.openocdPath/d'\
			-e '/^}/d'\
			$(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile > $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile;\
		if [[ -z $$(grep '[^[:space:]]' $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile) ]]; then\
			cp -f $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
		else\
			sed 's/\/\/.*//g' $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile > $(CY_VSCODE_OUT_PATH)/__TMP2__$$jsonFile;\
			if [[ ! -z $$(grep '[^[:space:]]' $(CY_VSCODE_OUT_PATH)/__TMP2__$$jsonFile) ]]; then\
				sed -i.tmp '/^[[:space:]]*$$/d' $(CY_VSCODE_OUT_PATH)/__TMP2__$$jsonFile;\
				rm $(CY_VSCODE_OUT_PATH)/__TMP2__$$jsonFile.tmp;\
				lastLine=$$(tail -n 1 $(CY_VSCODE_OUT_PATH)/__TMP2__$$jsonFile);\
				lastLine=`echo "$$lastLine" | sed 's/[[:space:]]*$$//g'`;\
				echo "{" > $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
				if [[ $${lastLine: -1} != "," ]]; then\
					sed -i.tmp "s/$$lastLine/$$lastLine,/" $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile;\
					rm $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile.tmp;\
				fi;\
			fi;\
			cat $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile >> $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
			grep -v -e "^{" $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile >> $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
			rm $(CY_VSCODE_OUT_PATH)/__TMP2__$$jsonFile;\
		fi;\
		rm $(CY_VSCODE_OUT_PATH)/__TMP1__$$jsonFile;\
	else\
		cp $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
	fi;\
	echo "Generated $$jsonFile";

vscode_project_tasks_json: $(_MTB_VSCODE_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)json="$(_MTB_CORE__VSCODE_TEMPLATE_PATH)/tasks.json";\
	jsonFile="$${json##*/}";\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $$json | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile;\
	$(CY_VSCODE_JSON_PROCESSING)\
	if [ -f $(CY_VSCODE_OUT_PATH)/$$jsonFile ]; then\
		echo "The existing $$jsonFile file has been saved to .vscode/backup";\
		mv -f $(CY_VSCODE_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile;\
	fi;\
	cp $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
	echo "Generated $$jsonFile";

vscode_project_extensions_json: $(_MTB_VSCODE_TARGET_BASE_DEPENDENCIES) $(_MTB_CORE__QBUILD_MK_FILE)
	$(MTB__NOISE)json="$(_MTB_CORE__VSCODE_TEMPLATE_PATH)/extensions.json";\
	jsonFile="$${json##*/}";\
	if [ -f $(CY_VSCODE_OUT_PATH)/$$jsonFile ]; then\
		echo "The existing $$jsonFile file has been saved to .vscode/backup";\
		mv -f $(CY_VSCODE_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile;\
	fi;\
	cp $(_MTB_CORE__VSCODE_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
	echo "Generated $$jsonFile";

vscode_project_toolchain_c_cpp_json: $(_MTB_VSCODE_TARGET_BASE_DEPENDENCIES) $(_MTB_CORE__QBUILD_MK_FILE)
	$(MTB__NOISE)json="$(_MTB_CORE__VSCODE_TEMPLATE_PATH)/c_cpp_properties.json";\
	jsonFile="$${json##*/}";\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $$json | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile;\
	if [[ $$jsonFile == *"c_cpp_properties.json" ]]; then\
		jsonFile="c_cpp_properties.json";\
		mv $(CY_VSCODE_OUT_TEMPLATE_PATH)/c_cpp_properties.json $(CY_VSCODE_OUT_TEMPLATE_PATH)/c_cpp_properties.json;\
	fi;\
	$(CY_VSCODE_JSON_PROCESSING)\
	if [ -f $(CY_VSCODE_OUT_PATH)/$$jsonFile ]; then\
		echo "The existing $$jsonFile file has been saved to .vscode/backup";\
		mv -f $(CY_VSCODE_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile;\
	fi;\
	cp $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
	echo "Generated $$jsonFile";

vscode_project_recipe_json: $(_MTB_VSCODE_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)for json in $(CY_VSCODE_TEMPLATE_RECIPE_PATH)/$(MTB_RECIPE_VSCODE_TEMPLATE_SUBDIR)*; do\
		if [ -f $$json ]; then\
			jsonFile="$${json##*/}";\
			sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $$json | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
				sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile;\
			$(CY_VSCODE_JSON_PROCESSING)\
			if [ -f $(CY_VSCODE_OUT_PATH)/$$jsonFile ]; then\
				echo "The existing $$jsonFile file has been saved to .vscode/backup";\
				mv -f $(CY_VSCODE_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile;\
			fi;\
			cp $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_OUT_PATH)/$$jsonFile;\
			echo "Generated $$jsonFile";\
		fi;\
	done;

vscode_project_openocd_process: $(CY_VSCODE_OUT_TEMPLATE_PATH)
	$(MTB__NOISE)$(CY_VSCODE_OPENOCD_PROCESSING)
	$(MTB__NOISE)if [ -s $(CY_VSCODE_OUT_PATH)/openocd.tcl ]; then\
		mv $(CY_VSCODE_OUT_PATH)/openocd.tcl $(MTB_TOOLS__PRJ_DIR)/openocd.tcl;\
	fi;

vsocde_project_gen: vscode_project_settings_json vscode_project_tasks_json vscode_project_extensions_json vscode_project_toolchain_c_cpp_json vscode_project_recipe_json vscode_project_openocd_process
	@:

# generate the project's vscode workspace file
vscode_project_workspace: $(_MTB_VSCODE_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)if [ -f $(MTB_TOOLS__PRJ_DIR)/$(_MTB_CORE__VSCODE_WORKSPACE_NAME) ]; then\
		mv -f $(MTB_TOOLS__PRJ_DIR)/$(_MTB_CORE__VSCODE_WORKSPACE_NAME) $(_MTB_CORE__VSCODE_BACKUP_PATH)/$(_MTB_CORE__VSCODE_WORKSPACE_NAME);\
		echo "The existing $(_MTB_CORE__VSCODE_WORKSPACE_NAME) file has been saved to .vscode/backup";\
	fi;\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $(_MTB_CORE__VSCODE_TEMPLATE_PATH)/$(CY_VSCODE_WORKSPACE_TEMPLATE_NAME) | \
		sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n'/g > $(MTB_TOOLS__PRJ_DIR)/$(_MTB_CORE__VSCODE_WORKSPACE_NAME);\
	echo "Generated $(_MTB_CORE__VSCODE_WORKSPACE_NAME)";

################################################################################
# multi-core targets
################################################################################

vscode_application_workspace_gen: $(_MTB_VSCODE_APPLICATION_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)if [ -f $(MTB_TOOLS__PRJ_DIR)/../$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME) ]; then\
		mv -f $(MTB_TOOLS__PRJ_DIR)/../$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME) $(_MTB_CORE__VSCODE_BACKUP_PATH)/$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME);\
		echo "The existing $(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME) file has been saved to .vscode/backup";\
	fi;\
	echo "Generated $(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME)";\
	cp -f $(CY_VSCODE_TEMPLATE_RECIPE_PATH)/do_not_copy/$(CY_VSCODE_WORKSPACE_TEMPLATE_NAME) $(MTB_TOOLS__PRJ_DIR)/../$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME);

vscode_application_workspace_sed:
	$(MTB__NOISE)mv -f $(MTB_TOOLS__PRJ_DIR)/../$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME) $(CY_VSCODE_OUT_TEMPLATE_PATH)/$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME);\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $(CY_VSCODE_OUT_TEMPLATE_PATH)/$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME) | \
		sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n'/g > $(MTB_TOOLS__PRJ_DIR)/../$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME);\
	rm -f $(CY_VSCODE_OUT_TEMPLATE_PATH)/$(_MTB_CORE__VSCODE_APPLICATION_WORKSPACE_NAME);

vscode_application_tasks_json_gen: $(_MTB_VSCODE_APPLICATION_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)json="$(_MTB_CORE__VSCODE_TEMPLATE_PATH)/tasks.json";\
	jsonFile="$${json##*/}";\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $$json | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile;\
	$(CY_VSCODE_JSON_PROCESSING)\
	if [ -f $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile ]; then\
		echo "The existing $$jsonFile file has been saved to .vscode/backup";\
		mv -f $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_BACKUP_PATH)/$$jsonFile;\
	fi;\
	cp $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile;\
	echo "Generated $$jsonFile";

vscode_application_recipe_gen: $(_MTB_VSCODE_APPLICATION_TARGET_BASE_DEPENDENCIES)
	$(MTB__NOISE)for json in $(CY_VSCODE_TEMPLATE_RECIPE_PATH)/Application/*; do\
		if [ -f $$json ]; then\
			jsonFile="$${json##*/}";\
			sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $$json | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
				sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile;\
			$(CY_VSCODE_JSON_PROCESSING)\
			if [ -f $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile ]; then\
				echo "The existing $$jsonFile file has been saved to .vscode/backup";\
				mv -f $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile $(_MTB_CORE__VSCODE_APPLICATION_BACKUP_PATH)/$$jsonFile;\
			fi;\
			cp $(CY_VSCODE_OUT_TEMPLATE_PATH)/$$jsonFile $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile;\
			echo "Generated $$jsonFile";\
		fi;\
	done;

vscode_application_recipe_sed:
	$(MTB__NOISE)json="$(CY_VSCODE_APPLICATION_OUT_PATH)/launch.json";\
	jsonFile="$${json##*/}";\
	sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile | sed -f $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2) | \
		sed s/'$(MTB__NEWLINE_MARKER)'/$$'\\\n            '/g > $(CY_VSCODE_APPLICATION_OUT_PATH)/__$$jsonFile;\
	mv -f $(CY_VSCODE_APPLICATION_OUT_PATH)/__$$jsonFile $(CY_VSCODE_APPLICATION_OUT_PATH)/$$jsonFile;\
	echo "Generated $$jsonFile";

vscode_application_openocd_process:
	$(MTB__NOISE)$(CY_VSCODE_OPENOCD_PROCESSING_APPLICATION)
	$(MTB__NOISE)if [ -s $(CY_VSCODE_APPLICATION_OUT_PATH)/openocd.tcl ]; then\
		mv $(CY_VSCODE_APPLICATION_OUT_PATH)/openocd.tcl $(CY_VSCODE_APPLICATION_OUT_PATH)/../openocd.tcl;\
	fi;

vscode_application_gen: vscode_application_workspace_gen vscode_application_tasks_json_gen vscode_application_recipe_gen vsocde_project_gen vscode_application_openocd_process
	@:

vscode_application_sed: vscode_application_workspace_sed vscode_application_recipe_sed vsocde_project_gen
	@:

# Note: CDB generation happens in build.mk
# Note: Series of sed substitutions are intentional as macOs default BSD sed does not understand \t and \n
#
vscode:
ifeq (,$(filter vscode,$(MTB_RECIPE__IDE_SUPPORTED)))
	$(call mtb__error,Unable to proceed. Export is not supported for this device)
endif
ifneq ($(_MTB_VSCODE_SKIP_CLEAN_UP),true)
	$(info )
	$(info )
	$(info J-Link users, please see the comments at the top of the launch.json)
	$(info    file about setting the location of the gdb-server.)
	$(info )
	$(info Instructions:)
	$(info 1. Review the modustoolbox.toolsPath property in .vscode/settings.json)
	$(info 2. Open VSCode)
	$(info 3. Install "C/C++" and "Cortex-Debug" extensions)
	$(info 4. File->Open Workspace From File (Welcome page->Start->Open File))
	$(info 5. Select the .code-workspace file from app root directory and open)
	$(info 6. Builds: Terminal->Run Task)
	$(info 7. Debugging: "Bug icon" on the left-hand pane)
	$(info )
	$(MTB__NOISE)rm $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2);\
	rm -rf $(CY_VSCODE_OUT_TEMPLATE_PATH);
endif

.PHONY: vscode vscode_application_bootstrap $(MTB_RECIPE__IDE_RECIPE_DATA_FILE) $(MTB_RECIPE__IDE_RECIPE_DATA_FILE_2)

.PHONY: vsocde_project_gen vscode_project_settings_json vscode_project_tasks_json vscode_project_extensions_json vscode_project_toolchain_c_cpp_json vscode_project_recipe_json vscode_project_openocd_process vscode_project_workspace

.PHONY: vscode_application_gen vscode_application_sed vscode_application_workspace_gen vscode_application_workspace_sed vscode_application_recipe_gen vscode_application_recipe_sed vscode_application_openocd_process
