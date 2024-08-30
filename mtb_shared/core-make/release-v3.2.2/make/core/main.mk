################################################################################
# \file main.mk
#
# \brief
# Defines the public facing build targets common to all recipes and includes
# the core makefiles.
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
# Macros
################################################################################

#
# Prints for getting time
# $(1) : Type - firststage, secondstage
# $(2) : Identifier
# $(3) : BEGIN or END
#
ifneq ($(CY_INSTRUMENT_BUILD),)
# Note: Use perl as "date" in macOS is based on ancient BSD
CY_LOG_TIME=$(shell perl -MTime::HiRes -e 'printf("%-40s%.0f\n","$1 $2 $3:",Time::HiRes::time()*1000)')
endif


################################################################################
# User-facing make targets
################################################################################

all: build

getlibs:

prebuild:

build:

build_proj:

qbuild:

qbuild_proj:

program:

program_proj:

qprogram:

qprogram_proj:

debug:

qdebug:

clean:

clean_proj:

# Note: Define the help target in BSP/recipe for custom help
help:

modlibs:

config:

config_bt:

config_ezpd:

config_lin:

config_usbdev:

config_secure:

get_app_info:

eclipse:

ewarm8:

uvision5:

vscode:

#
# Targets that do not require a second build stage
#
all getlibs clean clean_proj help:
modlibs config config_bt config_usbdev config_secure config_ezpd config_lin:
bsp check get_app_info get_env_info printlibs:
app memcalc application_postbuild:

#
# Targets that require a second build stage
#
build build_proj app program program_proj debug eclipse vscode ewarm8 uvision5 ewarm uvision: secondstage
#
# Targets that require don't require second build stage, but does need auto-discovery
#
qbuild qbuild_proj qprogram qprogram_proj qdebug erase attach: qsecondstage

################################################################################
# Applicable for both first and second build stages
################################################################################

ifeq ($(MTB_TYPE),PROJECT)
ifeq ($(MTB_APPLICATION_SUBPROJECTS),)
# We are directly calling make target from the project that belongs to multi-core
# application - pass this target to the application level
MTB_CORE__APPLICATION_BOOTSTRAP=true
endif
endif

ifeq ($(MTB_CORE__APPLICATION_BOOTSTRAP),true)
clean_application_bootstrap:
	$(MTB__NOISE)$(MAKE) -C .. clean

clean: clean_application_bootstrap
else
clean: clean_proj
endif

clean_proj:
	rm -rf $(MTB_TOOLS__OUTPUT_CONFIG_DIR) $(MTB_TOOLS__OUTPUT_GENERATED_DIR)

# Backwards-compatibility variables
include $(MTB_TOOLS__CORE_DIR)/make/core/bwc.mk

CY_TIMESTAMP_MAIN_MK_BEGIN=$(call CY_LOG_TIME,bothstages,main.mk,BEGIN)

##########################
# Include make files
##########################

#
# Include utilities used by all make files
#
CY_TIMESTAMP_UTILS_MK_BEGIN=$(call CY_LOG_TIME,bothstages,core_utils.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/core_utils.mk
CY_TIMESTAMP_UTILS_MK_END=$(call CY_LOG_TIME,bothstages,core_utils.mk,END)

CY_TIMESTAMP_RECIPE_VERSION_MK_BEGIN=$(call CY_LOG_TIME,bothstages,recipe_version.mk,BEGIN)
-include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/recipe_version.mk
CY_TIMESTAMP_RECIPE_VERSION_MK_END=$(call CY_LOG_TIME,bothstages,recipe_version.mk,END)

CY_TIMESTAMP_FEATURES_MK_BEGIN=$(call CY_LOG_TIME,bothstages,features.mk,BEGIN)
-include $(MTB_TOOLS__RECIPE_DIR)/make/udd/features.mk
CY_TIMESTAMP_FEATURES_MK_END=$(call CY_LOG_TIME,bothstages,features.mk,END)

CY_TIMESTAMP_TOOLCHAIN_MK_BEGIN=$(call CY_LOG_TIME,bothstages,core_selection.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/core_selection.mk
CY_TIMESTAMP_TOOLCHAIN_MK_END=$(call CY_LOG_TIME,bothstages,core_selection.mk,END)

CY_TIMESTAMP_TOOLCHAIN_MK_BEGIN=$(call CY_LOG_TIME,bothstages,toolchain.mk,BEGIN)
# The GCC_ARM readelf is used by all toolchain build for memory calculation. So always include GCC_ARM toolchain.
-include $(MTB_TOOLS__RECIPE_DIR)/make/toolchains/GCC_ARM.mk
ifneq ($(TOOLCHAIN),GCC_ARM)
include $(MTB_TOOLS__RECIPE_DIR)/make/toolchains/$(TOOLCHAIN).mk
endif
CY_TIMESTAMP_TOOLCHAIN_MK_END=$(call CY_LOG_TIME,bothstages,toolchain.mk,END)

CY_TIMESTAMP_RECIPE_TC_TYPES_MK_BEGIN=$(call CY_LOG_TIME,bothstages,recipe_toolchain_file_types.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/recipe_toolchain_file_types.mk
CY_TIMESTAMP_RECIPE_TC_TYPES_MK_END=$(call CY_LOG_TIME,bothstages,recipe_toolchain_file_types.mk,END)

CY_TIMESTAMP_DEFINES_MK_BEGIN=$(call CY_LOG_TIME,bothstages,defines.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/defines.mk
CY_TIMESTAMP_DEFINES_MK_END=$(call CY_LOG_TIME,bothstages,defines.mk,END)

CY_TIMESTAMP_RECIPE_SETUP_MK_BEGIN=$(call CY_LOG_TIME,bothstages,recipe_setup.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/recipe_setup.mk
CY_TIMESTAMP_RECIPE_SETUP_MK_END=$(call CY_LOG_TIME,bothstages,recipe_setup.mk,END)

CY_TIMESTAMP_SEARCH_MK_BEGIN=$(call CY_LOG_TIME,bothstages,search.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/search.mk
CY_TIMESTAMP_SEARCH_MK_END=$(call CY_LOG_TIME,bothstages,search.mk,END)

CY_TIMESTAMP_LIBRARY_MK_BEGIN=$(call CY_LOG_TIME,bothstages,library.mk,BEGIN)
_MTB_CORE__LIB_MK=$(wildcard $(foreach dir,$(SEARCH_MTB_MK),$(dir)/library.mk))
-include $(_MTB_CORE__LIB_MK)
CY_TIMESTAMP_LIBRARY_MK_END=$(call CY_LOG_TIME,bothstages,library.mk,END)

#
# Configurator-related routines
#
CY_TIMESTAMP_CONFIG_MK_BEGIN=$(call CY_LOG_TIME,bothstages,config.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/config.mk
CY_TIMESTAMP_CONFIG_MK_END=$(call CY_LOG_TIME,bothstages,config.mk,END)

################################################################################
# Include make files continued only for first build stage
################################################################################

ifeq ($(CY_SECONDSTAGE),)

# Check that there's only 1 version of tools and inform the user if there is not.
ifneq ($(sort $(notdir $(wildcard $(CY_TOOLS_PATHS)))),$(notdir $(CY_TOOLS_DIR)))
CY_MESSAGE_multi_tools=INFO: Multiple tools versions were found in "$(sort $(CY_TOOLS_PATHS))".\
				This build is currently using "$(CY_TOOLS_DIR)".\
				Check that this is the correct version that should be used in this build.\
				To stop seeing this message, set the CY_TOOLS_PATHS environment variable to the location of\
				the tools directory. This can be done either as an environment variable or set in the application Makefile.
$(eval $(call CY_MACRO_INFO,CY_MESSAGE_multi_tools,$(CY_MESSAGE_multi_tools)))
endif

#
# Help documentation
#
CY_TIMESTAMP_HELP_MK_BEGIN=$(call CY_LOG_TIME,firststage,help.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/help.mk
CY_TIMESTAMP_HELP_MK_END=$(call CY_LOG_TIME,firststage,help.mk,END)

CY_TIMESTAMP_PREBUILD_MK_BEGIN=$(call CY_LOG_TIME,firststage,prebuild.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/prebuild.mk
CY_TIMESTAMP_PREBUILD_MK_END=$(call CY_LOG_TIME,firststage,prebuild.mk,END)
CY_TIMESTAMP_RECIPE_MK_BEGIN=$(call CY_LOG_TIME,firststage,recipe.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/recipe.mk
CY_TIMESTAMP_RECIPE_MK_END=$(call CY_LOG_TIME,firststage,recipe.mk,END)

#
# Device transtion related targets
#
CY_TIMESTAMP_TRANSITION_MK_BEGIN=$(call CY_LOG_TIME,firststage,transition.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/transition.mk
CY_TIMESTAMP_TRANSITION_MK_END=$(call CY_LOG_TIME,firststage,transition.mk,END)

##########################
# Environment check
##########################

CY_TIMESTAMP_PYTHON_BEGIN=$(call CY_LOG_TIME,firststage,PYTHON,BEGIN)

#
# Find Python path
# Note: This check has a dependency on target.mk and features.mk and
# is hence placed after these files are included.
#
ifeq ($(filter uvision5,$(MAKECMDGOALS)),uvision5)
CY_PYTHON_REQUIREMENT=true
endif
ifeq ($(filter ewarm8,$(MAKECMDGOALS)),ewarm8)
CY_PYTHON_REQUIREMENT=true
endif
ifeq ($(filter eclipse,$(MAKECMDGOALS)),eclipse)
# IDE does not require project generation. Hence no python
ifneq ($(CY_MAKE_IDE),eclipse)
CY_PYTHON_REQUIREMENT=true
endif
endif

ifeq ($(CY_PYTHON_REQUIREMENT),true)
ifeq ($(CY_PYTHON_PATH),)

ifeq ($(OS),Windows_NT)
#
# CygWin/MSYS
#

#
# On Windows, when using windows store python, cygwin or msys are not
# able to run the python executable downloaded from windows store. So,
# we run python from command prompt (in cygwin/msys) by prepending
# cmd /c.
# Do not remove the space at the end of the following variable assignment
#
CY_PYTHON_FROM_CMD=cmd /c 

#
# Other Windows environments
#
else
CY_PYTHON_FROM_CMD=
endif

# Look for python install in the cypress tools directory
ifeq ($(wildcard $(CY_TOOL_python_EXE_ABS)),)
CY_PYTHON_SEARCH_PATH=NotFoundError
else
CY_PYTHON_SEARCH_PATH=$(CY_TOOL_python_EXE_ABS)
endif

#
# Check for python 3 intallation in the user's PATH
#   py -3 Windows python installer from python.org
#   python3 - Standard python3
#   python - Mapped python3 to python
#
ifeq ($(CY_PYTHON_SEARCH_PATH),NotFoundError)
CY_PYTHON_SEARCH_PATH:=$(shell \
	if [[ $$(py -3 --version 2>&1) == "Python 3"* ]]; then\
		echo py -3;\
	elif [[ $$($(CY_PYTHON_FROM_CMD)python3 --version 2>&1) == "Python 3"* ]]; then\
		echo $(CY_PYTHON_FROM_CMD)python3;\
	elif [[ $$($(CY_PYTHON_FROM_CMD)python --version 2>&1) == "Python 3"* ]]; then\
		echo $(CY_PYTHON_FROM_CMD)python;\
	else\
		echo NotFoundError;\
	fi)
endif

ifeq ($(CY_PYTHON_SEARCH_PATH),NotFoundError)
$(info )
$(info Python 3 was not found in the user's PATH and it was not explicitly defined in the CY_PYTHON_PATH variable.\
This target requires a python 3 installation. You can obtain python 3 from "https://www.python.org" or you may\
obtain it using the following alternate methods.$(MTB_NEWLINE)\
$(MTB_NEWLINE)\
Windows: Windows Store$(MTB_NEWLINE)\
macOS: brew install python3 $(MTB_NEWLINE)\
Linux (Debian/Ubuntu): sudo apt-get install python3 $(MTB_NEWLINE)\
)
$(call mtb__error,)
endif

export CY_PYTHON_PATH=$(CY_PYTHON_SEARCH_PATH)

# User specified python path
else

ifeq ($(shell [[ $$($(CY_PYTHON_FROM_CMD)$(CY_PYTHON_PATH) --version 2>&1) == "Python 3"* ]] && { echo true; } || { echo false; }),false)
$(info The path "$(CY_PYTHON_PATH)" is either an invalid path or contains an incorrect version of python.$(MTB_NEWLINE)\
Please provide the path to the python 3 executable. For example, "usr/bin/python3".$(MTB_NEWLINE) )
$(call mtb__error,)
endif

endif # ifeq ($(CY_PYTHON_PATH),)
endif # ifeq ($(CY_PYTHON_REQUIREMENT),true)

# Note: leave the space after PYTHON. It's intentional for cosmetics
CY_TIMESTAMP_PYTHON_END=$(call CY_LOG_TIME,firststage,PYTHON ,END)

# get_app_info data file will be generated at the end of first stage.
CY_TIMESTAMP_GET_APP_INFO_MK_BEGIN=$(call CY_LOG_TIME,firststage,get_app_info.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/get_app_info.mk
CY_TIMESTAMP_GET_APP_INFO_END=$(call CY_LOG_TIME,firststage,get_app_info.mk,END)

##########################
# Second build stage target
##########################

# Export all the CY_INFO and CY_WARNING variables so that they can all be printed out in secondstage
export $(filter CY_INFO_%,$(.VARIABLES))
export $(filter CY_WARNING_%,$(.VARIABLES))

qsecondstage_build: $(_MTB_CORE__QBUILD_MK_FILE)
secondstage_build: $(_MTB_CORE__FORCEBUILD_MK_FILE)

# Note: always use -f as it's not passed down via MAKEFLAGS
qsecondstage_build secondstage_build:
	$(MTB__NOISE)echo "Commencing build operations..."
	$(MTB__NOISE)echo
	$(MTB__NOISE)$(MAKE) -f $(abspath $(firstword $(MAKEFILE_LIST))) $(MAKECMDGOALS) CY_SECONDSTAGE=true --no-print-directory

qsecondstage: qsecondstage_build
secondstage: secondstage_build_check

qsecondstage second_stage:
	$(info $(subst .cywarning ,$(MTB_NEWLINE),$(subst .cyinfo ,$(MTB_NEWLINE),$(call \
	mtb__file_read,$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/.cyinfo)$(call \
	mtb__file_read,$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/.cywarning))))

secondstage_build_check: secondstage_build
ifeq ($(wildcard $(MTB_TOOLS__RECIPE_DIR)),)
	$(info )
	$(call mtb__error,Cannot find the base library. Run "make getlibs" and/or check\
	that the library location is correct in the CY_BASELIB_PATH variable)
endif

################################################################################
# Include make files continued for second build stage
################################################################################

else # ifeq ($(CY_SECONDSTAGE),)

##########################
# User input check
##########################

ifneq ($(APPNAME),)
ifneq ($(LIBNAME),)
$(call mtb__error,An application cannot define both APPNAME and LIBNAME. Define one or the other)
endif
endif
ifneq ($(filter -I%,$(INCLUDES)),)
$(call mtb__error,INCLUDES must be directories without -I prepended)
endif
ifneq ($(filter -D%,$(DEFINES)),)
$(call mtb__error,DEFINES must be specified without -D prepended)
endif
ifneq ($(filter -I%,$(CFLAGS)),)
$(call mtb__error,Include paths must be specified in the INCLUDES variable instead\
of directly in CFLAGS. These must be directories without -I prepended)
endif
ifneq ($(filter -D%,$(CFLAGS)),)
$(call mtb__error,Defines must be specified in the DEFINES variable instead\
of directly in CFLAGS. These must be specified without -D prepended)
endif
ifneq ($(filter -I%,$(CXXFLAGS)),)
$(call mtb__error,Include paths must be specified in the INCLUDES variable instead\
of directly in CXXFLAGS. These must be directories without -I prepended)
endif
ifneq ($(filter -D%,$(CXXFLAGS)),)
$(call mtb__error,Defines must be specified in the DEFINES variable instead\
of directly in CXXFLAGS. These must be specified without -D prepended)
endif
ifneq ($(filter -I%,$(ASFLAGS)),)
$(call mtb__error,Include paths must be specified in the INCLUDES variable instead\
of directly in ASFLAGS. These must be directories without -I prepended)
endif
ifneq ($(filter -D%,$(ASFLAGS)),)
$(call mtb__error,Defines must be specified in the DEFINES variable instead\
of directly in ASFLAGS. These must be specified without -D prepended)
endif

##########################
# Search and build
##########################

#
# Build-related routines
#
CY_TIMESTAMP_CYQBUILD_MK_BEGIN=$(call CY_LOG_TIME,secondstage,cyqbuild.mk,BEGIN)
# Skip the auto-discovery and re-use the last build's source list
include $(_MTB_CORE__QBUILD_MK_FILE)
CY_TIMESTAMP_CYQBUILD_MK_END=$(call CY_LOG_TIME,secondstage,cyqbuild.mk,END)

CY_TIMESTAMP_SEARCH_FILTER_MK_BEGIN=$(call CY_LOG_TIME,secondstage,search_filter.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/search_filter.mk
CY_TIMESTAMP_SEARCH_FILTER_MK_END=$(call CY_LOG_TIME,secondstage,search_filter.mk,END)

CY_TIMESTAMP_RECIPE_MK_BEGIN=$(call CY_LOG_TIME,secondstage,recipe.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/recipe.mk
CY_TIMESTAMP_RECIPE_MK_END=$(call CY_LOG_TIME,secondstage,recipe.mk,END)

CY_TIMESTAMP_BUILD_MK_BEGIN=$(call CY_LOG_TIME,secondstage,build.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/build.mk
CY_TIMESTAMP_BUILD_MK_END=$(call CY_LOG_TIME,secondstage,build.mk,END)

#
# Setup JLink path for IDE export and make program
#
CY_TIMESTAMP_JLINK_MK_BEGIN=$(call CY_LOG_TIME,secondstage,jlink.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/jlink.mk
CY_TIMESTAMP_JLINK_MK_END=$(call CY_LOG_TIME,secondstage,jlink.mk,END)

#
# Optional recipe-specific program routine 
#
ifndef CY_BSP_PROGRAM
CY_TIMESTAMP_PROGRAM_MK_BEGIN=$(call CY_LOG_TIME,secondstage,program.mk,BEGIN)
-include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/program.mk
CY_TIMESTAMP_PROGRAM_MK_END=$(call CY_LOG_TIME,secondstage,program.mk,END)
endif

#
# IDE file generation
#
MTB_CORE__EXPORT_INTERFACE_VERSION:=3.0
ifeq ($(CY_TOOL_mtbideexport_EXPORT_INTERFACE),3.1)
ifeq ($(MTB_RECIPE__INTERFACE_VERSION),2)
MTB_CORE__EXPORT_INTERFACE_VERSION:=3.1
endif
endif

ifeq ($(MTB_CORE__EXPORT_INTERFACE_VERSION),3.0)
CY_TIMESTAMP_RECIPE_IDE_MK_BEGIN=$(call CY_LOG_TIME,secondstage,recipe_ide.mk,BEGIN)
-include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/interface_version_1/recipe_ide.mk
CY_TIMESTAMP_RECIPE_IDE_MK_END=$(call CY_LOG_TIME,secondstage,recipe_ide.mk,END)
CY_TIMESTAMP_IDE_MK_BEGIN=$(call CY_LOG_TIME,secondstage,ide.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/interface_version_1/ide.mk
CY_TIMESTAMP_IDE_MK_END=$(call CY_LOG_TIME,secondstage,ide.mk,END)
endif

ifeq ($(MTB_CORE__EXPORT_INTERFACE_VERSION),3.1)
uvision: uvision5
ewarm: ewarm8
CY_TIMESTAMP_RECIPE_IDE_MK_BEGIN=$(call CY_LOG_TIME,secondstage,recipe_ide.mk,BEGIN)
include $(MTB_TOOLS__RECIPE_DIR)/make/recipe/interface_version_2/recipe_ide.mk
CY_TIMESTAMP_RECIPE_IDE_MK_END=$(call CY_LOG_TIME,secondstage,recipe_ide.mk,END)
CY_TIMESTAMP_IDE_MK_BEGIN=$(call CY_LOG_TIME,secondstage,ide.mk,BEGIN)
include $(MTB_TOOLS__CORE_DIR)/make/core/interface_version_2/ide.mk
CY_TIMESTAMP_IDE_MK_END=$(call CY_LOG_TIME,secondstage,ide.mk,END)
endif

#
# Gather and print info messages so that they can be shown at the end of secondstage
#
CY_PRINT_INFO_VARIABLES=$(filter CY_INFO_%,$(.VARIABLES))
CY_PRINT_INFO_MESSAGES=$(foreach msg,$(CY_PRINT_INFO_VARIABLES),$($(msg)) .cyinfo )
ifneq ($(CY_PRINT_INFO_VARIABLES),)
CY_PRINT_INFO_HEADER=.cyinfo \
	============================================================================== .cyinfo \
	= INFO message(s) = .cyinfo \
	============================================================================== .cyinfo 
endif
$(call mtb__file_write,$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/.cyinfo,$(CY_PRINT_INFO_HEADER) $(CY_PRINT_INFO_MESSAGES))

#
# Gather and print warning messages so that they can be shown at the end of secondstage
#
CY_PRINT_WARNING_VARIABLES=$(filter CY_WARNING_%,$(.VARIABLES))
CY_PRINT_WARNING_MESSAGES=$(foreach msg,$(CY_PRINT_WARNING_VARIABLES),$($(msg)) .cywarning )
ifneq ($(CY_PRINT_WARNING_VARIABLES),)
CY_PRINT_WARNING_HEADER=.cywarning \
	============================================================================== .cywarning \
	= WARNING message(s) = .cywarning \
	============================================================================== .cywarning 
endif
$(call mtb__file_write,$(MTB_TOOLS__OUTPUT_CONFIG_DIR)/.cywarning,$(CY_PRINT_WARNING_HEADER) $(CY_PRINT_WARNING_MESSAGES))

# Empty on purpose
secondstage:
qsecondstage:

endif # ifeq ($(CY_SECONDSTAGE),)

CY_TIMESTAMP_MAIN_MK_END=$(call CY_LOG_TIME,bothstages,main.mk,END)

#
# Print the timestamps
#
ifneq ($(CY_INSTRUMENT_BUILD),)
CY_TIMESTAMP_LIST=UTILS_MK RECIPE_VERSION_MK EXTRA_INC FEATURES_MK DEFINES_MK TOOLCHAIN_MK CONFIG_MK TOOLS_MK HELP_MK\
					PREBUILD_MK RECIPE_MK TRANSITION_MK JLINK_MK PYTHON GET_APP_INFO \
					SEARCH_MK CYQBUILD_MK SEARCH_FILTER_MK RECIPE_MK BUILD_MK PROGRAM_MK RECIPE_IDE_MK IDE_MK

$(info )
$(info ==============================================================================)
$(info = Begin timestamps $(MTB_TAB)$(MTB_TAB)$(MTB_TAB)(milliseconds) = )
$(info ==============================================================================)
$(info $(CY_TIMESTAMP_MAIN_MK_BEGIN))
$(foreach timestamp,$(CY_TIMESTAMP_LIST),\
	$(if $(CY_TIMESTAMP_$(timestamp)_BEGIN),\
		$(info $(CY_TIMESTAMP_$(timestamp)_BEGIN))\
		$(info $(CY_TIMESTAMP_$(timestamp)_END))\
	)\
)
$(info $(CY_TIMESTAMP_MAIN_MK_END))
$(info ==============================================================================)
$(info = End timestamps = )
$(info ==============================================================================)
$(info )
endif

#
# Identify the phony targets
#
.PHONY: all getlibs clean clean_application_bootstrap clean_proj help
.PHONY: modlibs config config_bt config_usbdev config_secure config_ezpd config_lin
.PHONY: bsp check get_env_info printlibs
.PHONY: app memcalc help_default
.PHONY: secondstage_build_check second_stage qsecondstage secondstage_build qsecondstage_build

.PHONY: build build_proj qbuild qbuild_proj
.PHONY: program program_proj qprogram debug qdebug erase attach
