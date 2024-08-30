################################################################################
# \file help.mk
#
# \brief
# Default help documentation
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

#
# General targets
#
CY_HELP_all=Same as build.
CY_HELP_all_VERBOSE=This target is equivalent to the "build" target.
CY_HELP_getlibs=Clones the repositories and checks out the identified commit.
CY_HELP_getlibs_VERBOSE=When using .mtb files, the repos are cloned to the shared location $$(CY_GETLIBS_SHARED_PATH)/$$(CY_GETLIBS_SHARED_NAME).\
					By default, this directory is specified by the project Makefile.
CY_HELP_build=Builds the application.
CY_HELP_build_VERBOSE=The build process involves source auto-discovery, code generation, prebuild, and postbuilds.\
					For faster incremental builds, use the "qbuild" target to skip the auto-generation step.\
					For multi-core applications, running this target builds all projects in the application, and generates a combined hex file.
CY_HELP_build_proj=Build the current project.
CY_HELP_build_proj_VERBOSE=Build the current project in the application. In single-core applications, this target is the same as the "build" target.
CY_HELP_qbuild=Builds the application using the previous build's source list.
CY_HELP_qbuild_VERBOSE=When no other sources need to be auto-discovered, this target can be used to skip\
					the auto-discovery step for a faster incremental build.
CY_HELP_qbuild_proj=Builds the current project using the previous build's source list.\
					In single-core applications, this target is the same as the "qbuild" target.
CY_HELP_qbuild_proj_VERBOSE=When no other sources need to be auto-discovered, this target can be used to skip\
					the auto-discovery step for a faster incremental build.
CY_HELP_program=Builds the application and programs it to the target device. In multi-core applications, this will program the combined hex file.
CY_HELP_program_VERBOSE=The build process performs the same operations as the "build" target. Upon completion,\
					the artifact is programmed to the board.
CY_HELP_program_proj=Builds the current project and programs it to the target device." In single-core applications, this target is the same as the "program" target.
CY_HELP_program_proj_VERBOSE=The build process performs the same operations as the "build" target. Upon completion,\
					the artifact is programmed to the board.
CY_HELP_qprogram=Programs a built application to the target device without rebuilding.
CY_HELP_qprogram_VERBOSE=This target allows programming an existing artifact to the board without a build step.
CY_HELP_qprogram_proj=Programs the current built project to the target device without rebuilding.\
					In single-core applications, this target is the same as the "qprogram" target.
CY_HELP_qprogram_proj_VERBOSE=This target allows programming an existing artifact to the board without a build step.
CY_HELP_clean=Cleans the /build/<TARGET> directory.
CY_HELP_clean_VERBOSE=The directory and all its contents are deleted from disk.
CY_HELP_help=Prints the help documentation.
CY_HELP_help_VERBOSE=Use the CY_HELP=<Name of make target or variable> to see the verbose documentation for a\
					particular make target or variable.
CY_HELP_prebuild=Generates code for the application.
CY_HELP_prebuild_VERBOSE=Runs configurators and custom prebuild commands to generate source code.

#
# IDE targets
#
CY_HELP_eclipse=Generates Eclipse IDE launch configs and project files.
CY_HELP_eclipse_VERBOSE=This target generates .cproject and .project files if they do not exist in the application root directory.
CY_HELP_vscode=Generates VS Code IDE files.
CY_HELP_vscode_VERBOSE=This target generates VS Code files for debug/program launches, IntelliSense, and custom tasks.\
					These overwrite the existing files in the application directory except for settings.json.
CY_HELP_ewarm8=Generates IAR-EW v8 or later IDE .ipcf file.
CY_HELP_ewarm8_VERBOSE=This target generates an IAR Embedded Workbench compatible .ipcf file that can be imported\
					into IAR-EW. The .ipcf file is overwritten every time this target is run.\
					$(MTB__NEWLINE)Note: Project generation requires python3 to be installed and present in the PATH variable.
CY_HELP_uvision5=Generates Keil uVision v5 or later IDE .cpdsc, .gpdsc, and .cprj files.
CY_HELP_uvision5_VERBOSE=This target generates a CMSIS compatible .cpdsc and .gpdsc files that can be imported\
					into Keil uVision 5. Both files are overwritten every time this target is run.\
					Files in the default cmsis output directory will be automatically excluded when calling make uvision5.\
					$(MTB__NEWLINE)Note: Project generation requires python3 to be installed and present in the PATH variable.

#
# Utility targets
#
CY_HELP_progtool=Performs specified operations on the programmer/firmware-loader. Only available for devices that use KitProg3.
CY_HELP_progtool_VERBOSE=This target expects user-interaction on the shell while running it. When prompted, you must specify the\
					command(s) to run for the tool.
CY_HELP_check=Checks for the necessary tools.
CY_HELP_check_VERBOSE=Not all tools are necessary for every build recipe. This target allows you\
					to get an idea of which tools are missing if a build fails in an unexpected way.
CY_HELP_printlibs=Prints the status of the library repos.
CY_HELP_printlibs_VERBOSE=This target parses through the library repos and prints the SHA1 commit ID for each library.\
					It also shows whether the repo is clean (no changes) or dirty (modified or new files).

#
# Project defined targets
#
CY_HELP_project_prebuild=Project specific prebuild target.
CY_HELP_project_prebuild_VERBOSE=If defined this target will be executed during the prebuild step.
CY_HELP_project_postbuild=Project specific postbuild target.
CY_HELP_project_postbuild_VERBOSE=If defined this target will be executed between the linking and hex file generation step.

#
# Basic configuration
#
CY_HELP_TARGET=Specifies the target board/kit. (e.g. CY8CPROTO-062-4343W)
CY_HELP_TARGET_VERBOSE=Current target in this application is, [ $(CY_TARGET_MAKEFILE) ].\
						$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build TARGET=CY8CPROTO-062-4343W
CY_HELP_CORE=Specifies the name of the Arm core for which a project is building (e.g. CM4).
CY_HELP_CORE_VERBOSE=Use this variable to select compiler and linker options to build a project for a specified Arm core.\
				$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build CORE=CM4
CY_HELP_CORE_NAME=Specifies the name of the on-chip core for which a project is building (e.g. CM7_0).
CY_HELP_CORE_NAME_VERBOSE=Use this variable to select compiler and linker options to build a project for a specified on-chip core.\
				$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build CORE_NAME=CM7_0\
				$(MTB__NEWLINE)$(MTB__NEWLINE)Note: This variable is applicable for some multi-core devices only (e.g. XMC7xxx).
CY_HELP_APPNAME=Specifies the name of the app. (e.g. "AppV1" -> AppV1.elf)
CY_HELP_APPNAME_VERBOSE=This variable is used to set the name of the application artifact (programmable image).\
				$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build APPNAME="AppV1"\
				$(MTB__NEWLINE)$(MTB__NEWLINE)Note: This variable may also be used when generating launch configs when invoking the "eclipse" target.\
				$(MTB__NEWLINE)Example Usage: make eclipse APPNAME="AppV1"
CY_HELP_TOOLCHAIN=Specifies the toolchain for building the application. (e.g. GCC_ARM)
CY_HELP_TOOLCHAIN_VERBOSE=Supported toolchains for this target are: [ $(CY_SUPPORTED_TOOLCHAINS) ].\
							$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build TOOLCHAIN=IAR
CY_HELP_CONFIG=Specifies the configuration option for the build [Debug Release].
CY_HELP_CONFIG_VERBOSE=The CONFIG variable is not limited to Debug/Release. It can be\
						other values. However in those instances, the build system will not configure the optimization flags.\
						Debug=lowest optimization, Release=highest optimization. The optimization flags are toolchain-specific.\
						If you go with your custom config then you can manually set the optimization flag in the CFLAGS.\
						$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build CONFIG=Release
CY_HELP_VERBOSE=Specifies whether the build is silent [false] or verbose [true].
CY_HELP_VERBOSE_VERBOSE=Setting VERBOSE to true may help in debugging build errors/warnings. By default it is set to false.\
						$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build VERBOSE=true

#
# Advanced configuration
#
CY_HELP_SOURCES=Specifies C/C++ and assembly files outside of the application directory.
CY_HELP_SOURCES_VERBOSE=This can be used to include files external to the application directory.\
						The path can be both absolute or relative to the application directory.\
						$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): SOURCES+=path/to/file/Source1.c
CY_HELP_INCLUDES=Specifies include paths outside of the application directory.
CY_HELP_INCLUDES_VERBOSE=Note: These MUST NOT have -I prepended.\
						The path can be either absolute or relative to the application directory.\
						$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): INCLUDES+=path/to/headers
CY_HELP_DEFINES=Specifies additional defines passed to the compiler.
CY_HELP_DEFINES_VERBOSE=Note: These MUST NOT have -D prepended.\
						$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): DEFINES+=EXAMPLE_DEFINE=0x01
CY_HELP_MVE_SELECT=Selects M-Profile Vector Extension (MVE) operating mode [NO_MVE MVE-I MVE-F].
CY_HELP_MVE_SELECT_VERBOSE=If not defined, this value defaults to MVE-F.\
							$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): MVE_SELECT=MVE-I.\
							MVE_SELECT is ignored for IAR toolchain.
CY_HELP_VFP_SELECT=Selects hard/soft ABI or full software for floating-point operations [softfp hardfp softfloat].
CY_HELP_VFP_SELECT_VERBOSE=If not defined, this value defaults to softfp.\
							$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): VFP_SELECT=hardfp
CY_HELP_VFP_SELECT_PRECISION=Selects single-precision or double-precision operating mode for floating-point operations.
CY_HELP_VFP_SELECT_PRECISION_VERBOSE=If not defined, this value defaults to double-precision.\
							Enable single-precision mode by using the "singlefp" option.\
							$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): VFP_SELECT_PRECISION=singlefp
CY_HELP_CFLAGS=Prepends additional C compiler flags.
CY_HELP_CFLAGS_VERBOSE=Example Usage (within Makefile): CFLAGS+= -Werror -Wall -O2
CY_HELP_CXXFLAGS=Prepends additional C++ compiler flags.
CY_HELP_CXXFLAGS_VERBOSE=Example Usage (within Makefile): CXXFLAGS+= -finline-functions
CY_HELP_ASFLAGS=Prepends additional assembler flags.
CY_HELP_ASFLAGS_VERBOSE=Usage is similar to CFLAGS.
CY_HELP_LDFLAGS=Prepends additional linker flags.
CY_HELP_LDFLAGS_VERBOSE=Example Usage (within Makefile): LDFLAGS+= -nodefaultlibs
CY_HELP_LDLIBS=Includes application-specific prebuilt libraries.
CY_HELP_LDLIBS_VERBOSE=Note: If additional libraries need to be added using -l or -L, add to the\
						CY_RECIPE_EXTRA_LIBS make variable. Usage is similar to CFLAGS.\
						$(MTB__NEWLINE)$(MTB__NEWLINE) Example Usage (within Makefile): LDLIBS+=../MyBinaryFolder/binary.o
CY_HELP_LINKER_SCRIPT=Specifies a custom linker script location.
CY_HELP_LINKER_SCRIPT_VERBOSE=This linker script overrides the default. Note: Additional\
					linker scripts can be added for GCC via the LDFLAGS variable as a -L option.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): LINKER_SCRIPT=path/to/file/Custom_Linker1.ld
CY_HELP_PREBUILD=DEPRECATED. Replaced with "project_prebuild" make target.
CY_HELP_PREBUILD_VERBOSE=DEPRECATED. Replaced with "project_prebuild" make target.
CY_HELP_POSTBUILD=DEPRECATED. Replaced with "project_postbuild" make target.
CY_HELP_POSTBUILD_VERBOSE=DEPRECATED. Replaced with "project_postbuild" make target.
CY_HELP_COMPONENTS=Adds component-specific files to the build.
CY_HELP_COMPONENTS_VERBOSE=Create a directory named COMPONENT_<VALUE> and place your files.\
					Then include the <VALUE> to this make variable to have that feature library\
					be included in the build. For example, to include a directory named COMPONENT_ACCELEROMETER into auto-discovery,\
					then add the following make variable to the Makefile COMPONENT=ACCELEROMETER. If the make variable\
					does not include the <VALUE>, then that library will not be included in the build.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): COMPONENTS+=CUSTOM_CONFIGURATION
CY_HELP_DISABLE_COMPONENTS=Removes component-specific files from the build.
CY_HELP_DISABLE_COMPONENTS_VERBOSE=Include a <VALUE> to this make variable to have that feature library\
					be excluded in the build. For example, To exclude the contents of COMPONENT_BSP_DESIGN_MODUS\
					directory, set DISABLE_COMPONENTS=BSP_DESIGN_MODUS as shown.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): DISABLE_COMPONENTS=BSP_DESIGN_MODUS
CY_HELP_SEARCH=List of paths to include in auto-discovery. (e.g. ../mtb_shared/lib1)
CY_HELP_SEARCH_VERBOSE=The SEARCH variable can also be used by the application to include other directories to auto-discovery.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage (within Makefile): SEARCH+=directory_containing_source_files
CY_HELP_SKIP_CODE_GEN=Disables code generation for configurators when building.
CY_HELP_SKIP_CODE_GEN_VERBOSE=When set to a non-empty value, the build process will not run code generation for configurators.\
					$(MTB__NEWLINE)NOTE: By default code examples specify the GeneratedSource directory in .gitignore file.\
					If this variable is used, the GeneratedSouce directory should be removed from the .gitignore file.
CY_HELP_MERGE=List of projects in the application to include when generating a combined hex file.
CY_HELP_MERGE_VERBOSE=By Default, building a multi-core application will generate a combined hex file from its sub-projects.\
					This variable can be set from the application Makefile to override the set of projects to generate a combined hex file from.

#
# BSP variables
#
CY_HELP_DEVICE=Device ID for the primary MCU on the target board/kit. Set by bsp.mk.
CY_HELP_DEVICE_VERBOSE=The device identifier is mandatory for all board/kits.
CY_HELP_ADDITIONAL_DEVICES=IDs for additional devices on the target board/kit. Set by bsp.mk.
CY_HELP_ADDITIONAL_DEVICES_VERBOSE=These include devices such as radios on the board/kit. This variable is optional.
CY_HELP_BSP_PROGRAM_INTERFACE=Specifies the debugging and programming interface to use. The default value and valid values all depend on the BSP.
CY_HELP_BSP_PROGRAM_INTERFACE_VERBOSE=Possible values include KitProg3, JLink, and FTDI. Most BSPs will only support a subset of this list.

#
# Getlibs variables
#
# I thought CY_GETLIBS_NO_CACHE and CY_GETLIBS_CACHE_PATH were obsolete?
#
CY_HELP_CY_GETLIBS_NO_CACHE=Disables the cache when running getlibs when this variable is set to non-empty.
CY_HELP_CY_GETLIBS_NO_CACHE_VERBOSE=To improve the library creation time, the getlibs target uses a cache\
					located in the user's home directory ($$HOME for macOS/Linux and $$USERPROFILE for Windows). Disabling the\
					cache will result in slower application creation time but may be necessary for bringing in non-Infineon libraries.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make getlibs CY_GETLIBS_NO_CACHE=true
CY_HELP_CY_GETLIBS_OFFLINE=This feature is no longer supported starting in ModusToolbox 3.1 and has been replaced with Local Content Storage feature.
CY_HELP_CY_GETLIBS_OFFLINE_VERBOSE=This feature is no longer supported starting in ModusToolbox 3.1 and has been replaced with Local Content Storage feature.
CY_HELP_MTB_USE_LOCAL_CONTENT=If set to a non-empty value, enable local content storage.
CY_HELP_MTB_USE_LOCAL_CONTENT_VERBOSE=Enable local content storage to allow use of ModusToolbox without requiring internet access.\
					$(MTB__NEWLINE)See the LCS Manager CLI User Guide for more information.\
					$(MTB__NEWLINE)This feature is new as of ModusToolbox 3.1.
CY_HELP_CY_GETLIBS_PATH=Path to the intended location of libs info directory.
CY_HELP_CY_GETLIBS_PATH_VERBOSE=The directory contains local libraries and metadata files about shared libraries.
CY_HELP_CY_GETLIBS_DEPS_PATH=Path to where the library-manager stores .mtb files.
CY_HELP_CY_GETLIBS_DEPS_PATH_VERBOSE=Setting this path allows relocating the directory that the library-manager uses to store\
					the .mtb files in your application. The default location is in a directory named /deps.
CY_HELP_CY_GETLIBS_CACHE_PATH=Absolute path to the cache directory.
CY_HELP_CY_GETLIBS_CACHE_PATH_VERBOSE=The build system caches all cloned repos in a directory named /cache \
					(Default: <HOME>/.modustoolbox/cache). Setting this variable allows the cache to be relocated to\
					elsewhere on disk. Usage is similar to CY_GETLIBS_PATH. To disable the cache entirely, \
					set the CY_GETLIBS_NO_CACHE variable to [true].
CY_HELP_CY_GETLIBS_OFFLINE_PATH=This feature is no longer supported starting in ModusToolbox 3.1 and has been replaced with Local Content Storage feature.
CY_HELP_CY_GETLIBS_OFFLINE_PATH_VERBOSE=This feature is no longer supported starting in ModusToolbox 3.1 and has been replaced with Local Content Storage feature.
CY_HELP_CY_GETLIBS_SHARED_PATH=Relative path to the shared repo location.
CY_HELP_CY_GETLIBS_SHARED_PATH_VERBOSE=All .mtb files have the format, <URI><COMMIT><LOCATION>.\
					If the <LOCATION> field begins with $$$$ASSET_REPO$$$$, then the repo is deposited in the path\
					specified by the CY_GETLIBS_SHARED_PATH variable. The default this is set from the project Makefile.
CY_HELP_CY_GETLIBS_SHARED_NAME=Directory name of the shared repo location.
CY_HELP_CY_GETLIBS_SHARED_NAME_VERBOSE=All .mtb files have the format, <URI><COMMIT><LOCATION>.\
					If the <LOCATION> field begins with $$$$ASSET_REPO$$$$, then the repo is deposited in the directory\
					specified by the CY_GETLIBS_SHARED_NAME variable. By default this is set from the project Makefile.

#
# Path variables
#
CY_HELP_CY_APP_PATH=Relative path to the top-level of the project. (e.g. ./)
CY_HELP_CY_APP_PATH_VERBOSE=Setting this path to other than ./ allows the auto-discovery mechanism\
					to search from a root directory location that is higher than the application directory.\
					For example, CY_APP_PATH=../../ allows auto-discovery of files from a location that is\
					two directories above the location of the Makefile.
CY_HELP_CY_COMPILER_PATH=DEPRECATED. Use CY_COMPILER_GCC_ARM_DIR CY_COMPILER_ARM_DIR CY_COMPILER_IAR_DIR instead.
CY_HELP_CY_COMPILER_PATH_VERBOSE=DEPRECATED. Use CY_COMPILER_GCC_ARM_DIR CY_COMPILER_ARM_DIR CY_COMPILER_IAR_DIR instead.
CY_HELP_CY_COMPILER_GCC_ARM_DIR=Absolute path to the gcc-arm toolchain directory.
CY_HELP_CY_COMPILER_GCC_ARM_DIR_VERBOSE=Setting this path overrides the default GCC_ARM toolchain directory.\
					It is used when the compiler is located at a non-default directory.\
					Make uses this variable for the path to the assembler, compiler, linker, objcopy, and other toolchain binaries.\
					For example, CY_COMPILER_GCC_ARM_DIR=C:/Program Files (x86)GNU Arm Embedded Toolchain/10 2021.10\
					NOTE: when set in the Makefile, no quotes are required.
CY_HELP_CY_COMPILER_IAR_DIR=Absolute path to the IAR toolchain directory.
CY_HELP_CY_COMPILER_IAR_DIR_VERBOSE=Setting this path overrides the default IAR toolchain directory.\
					It is used when the compiler is located at a non-default directory.\
					Make uses this variable for the path to the assembler, compiler, and linker, and other toolchain binaries.\
					For example, CY_COMPILER_IAR_DIR=C:/Program Files/IAR Systems/Embedded Workbench 9.1/arm\
					NOTE: when set in the Makefile, no quotes are required.
CY_HELP_CY_COMPILER_ARM_DIR=Absolute path to the ARM toolchain directory.
CY_HELP_CY_COMPILER_ARM_DIR_VERBOSE=Setting this path overrides the default ARM toolchain directory.\
					It is used when the compiler is located at a non-default directory.\
					Make uses this variable for the path to the assembler, compiler, linker, objcopy, and other toolchain binaries.\
					For example, CY_COMPILER_ARM_DIR=C:/Program Files/ARMCompiler6.16\
					NOTE: when set in the Makefile, no quotes are required.
CY_HELP_CY_TOOLS_DIR=Absolute path to the tools root directory.
CY_HELP_CY_TOOLS_DIR_VERBOSE=Applications must specify the directory of the tools install, which contains the\
					root Makefile and the necessary tools and scripts to build an application. Application Makefiles\
					are configured to automatically search in the standard locations for various platforms.\
					If the tools are not located in the standard location, you may explicitly set this.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build CY_TOOLS_DIR="path/to/ModusToolbox/tools_x.y"
CY_HELP_CY_BUILD_LOCATION=Absolute path to the build output directory (Default: ./build).
CY_HELP_CY_BUILD_LOCATION_VERBOSE=The build output directory is structured as /<TARGET>/<CONFIG>/. Setting this variable\
					allows the build artifacts to be located in the directory pointed to by this variable.\
					Content of this directory is automatically excluded from auto-discovery.
CY_HELP_CY_PYTHON_PATH=Specifies the path to the Python executable.
CY_HELP_CY_PYTHON_PATH_VERBOSE=For make targets that depend on Python, the build system looks for a Python 3 in the user's PATH\
					and uses that to invoke python. If however CY_PYTHON_PATH is defined, it will use that python executable.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: CY_PYTHON_PATH="path/to/python/executable/python.exe"
CY_HELP_MTB_JLINK_DIR=Specifies the path to the SEGGER J-Link software install directory "JLink".
CY_HELP_MTB_JLINK_DIR_VERBOSE=Setting this path allows the make system to locate the JLink executable when calling make program.\
					If not specified, make will default to the JLink executable in the PATH variable.\
					When generating launch config for IDE, this will override the default JLink path.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: MTB_JLINK_DIR:=C:/Program Files/SEGGER/JLink

#
# Miscellaneous variables
#
CY_HELP_CY_IGNORE=Adds to the directory and file ignore list. (e.g. ./file1.c ./inc1)
CY_HELP_CY_IGNORE_VERBOSE=Directories and files listed in this variable are ignored in the auto-discovery.\
					This mechanism works in combination with any existing .cyignore files in the application.\
					$(MTB__NEWLINE)$(MTB__NEWLINE)Example Usage: make build CY_IGNORE="path/to/file/ignore_file"
CY_HELP_CY_SIMULATOR_GEN_AUTO=If set to non-empty, automatically generate a simulator archive (if supported by the target device).
CY_HELP_CY_SIMULATOR_GEN_AUTO_VERBOSE=When enabled, build make target will generate a debugging tgz archive for the\
					Infineon online simulator as part of the postbuild process.

# Pass these to CY_HELP to get the full verbose info

CY_HELP_TARGETS_ALL=all getlibs build build_proj qbuild qbuild_proj program program_proj qprogram qprogram_proj \
					debug qdebug attach clean prebuild help check project_prebuild project_postbuild \
					eclipse vscode ewarm8 uvision5 modlibs check printlibs
CY_HELP_BASIC_CFG_ALL=TARGET CORE CORE_NAME APPNAME TOOLCHAIN CONFIG VERBOSE
CY_HELP_ADVANCED_CFG_ALL=SOURCES INCLUDES DEFINES VFP_SELECT CFLAGS CXXFLAGS ASFLAGS LDFLAGS LDLIBS LINKER_SCRIPT \
					PREBUILD POSTBUILD COMPONENTS DISABLE_COMPONENTS SEARCH MERGE DEVICE ADDITIONAL_DEVICES
CY_HELP_GETLIBS_ALL=CY_GETLIBS_NO_CACHE CY_HELP_MTB_USE_LOCAL_CONTENT CY_GETLIBS_OFFLINE CY_GETLIBS_PATH CY_GETLIBS_DEPS_PATH CY_GETLIBS_CACHE_PATH \
					CY_GETLIBS_OFFLINE_PATH CY_GETLIBS_SHARED_PATH CY_GETLIBS_SHARED_NAME 
CY_HELP_PATHS_ALL=CY_APP_PATH CY_COMPILER_GCC_ARM_DIR CY_COMPILER_ARM_DIR CY_COMPILER_IAR_DIR \
					CY_COMPILER_PATH CY_TOOLS_DIR CY_BUILD_LOCATION CY_PYTHON_PATH
CY_HELP_MISC_ALL=CY_IGNORE
CY_HELP_PRINT_ALL=$(CY_HELP_TARGETS_ALL) $(CY_HELP_BASIC_CFG_ALL) $(CY_HELP_ADVANCED_CFG_ALL) \
					$(CY_HELP_GETLIBS_ALL) $(CY_HELP_PATHS_ALL) $(CY_HELP_MISC_ALL)

ifneq ($(CY_HELP),)
mtb_help_topic:
	@:
	$(foreach topic,$(CY_HELP),\
	$(if $(CY_HELP_$(topic)),\
	$(info $(MTB__NEWLINE)Topic-specific help for "$(topic)")\
	$(info $(MTB__SPACE)$(MTB__SPACE)Brief: $(CY_HELP_$(topic)))\
	$(info $(MTB__NEWLINE)$(CY_HELP_$(topic)_VERBOSE)),\
	$(info $(MTB__NEWLINE)Topic-specific help for "$(topic) not found")\
	))

help: mtb_help_topic
.PHONY: mtb_help_topic

else
mtb_help_header:
	@:
	$(info                                                                                    )
	$(info ==============================================================================     )
	$(info $(MTB__SPACE)Cypress Build System                                                    )
	$(info ==============================================================================     )
	$(info $(MTB__SPACE)Copyright 2018-2023 Cypress Semiconductor Corporation                   )
	$(info $(MTB__SPACE)SPDX-License-Identifier: Apache-2.0                                     )
	$(info                                                                                    )
	$(info $(MTB__SPACE)Licensed under the Apache License, Version 2.0 (the "License");         )
	$(info $(MTB__SPACE)you may not use this file except in compliance with the License.        )
	$(info $(MTB__SPACE)You may obtain a copy of the License at                                 )
	$(info                                                                                    )
	$(info $(MTB__SPACE)$(MTB__SPACE)    http://www.apache.org/licenses/LICENSE-2.0               )
	$(info                                                                                    )
	$(info $(MTB__SPACE)Unless required by applicable law or agreed to in writing, software     )
	$(info $(MTB__SPACE)distributed under the License is distributed on an "AS IS" BASIS,       )
	$(info $(MTB__SPACE)WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.)
	$(info $(MTB__SPACE)See the License for the specific language governing permissions and     )
	$(info $(MTB__SPACE)limitations under the License.                                          )
	$(info ==============================================================================     )
	$(info                                                                                    )
	$(info $(MTB__SPACE)This is the help documentation for the Cypress build system.            )
	$(info $(MTB__SPACE)It lists the supported make targets and make variables.                 )
	$(info                                                                                    )
	$(info $(MTB__SPACE)Usage:   make [target][variable]                                        )
	$(info $(MTB__SPACE)Example: make build TARGET=CY8CPROTO-062-4343W                          )
	$(info                                                                                    )
	$(info $(MTB__SPACE)The CY_HELP make variable can be used for information on a particular   )
	$(info $(MTB__SPACE)make target or variable                                                 )
	$(info                                                                                    )
	$(info $(MTB__SPACE)Usage:   make help CY_HELP=[target/variable]                            )
	$(info $(MTB__SPACE)Example: make help CY_HELP=APPNAME                                      )
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)General make targets                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)all                 $(CY_HELP_all))
	$(info $(MTB__SPACE)getlibs             $(CY_HELP_getlibs))
	$(info $(MTB__SPACE)build               $(CY_HELP_build))
	$(info $(MTB__SPACE)build_proj          $(CY_HELP_build_proj))
	$(info $(MTB__SPACE)qbuild              $(CY_HELP_qbuild))
	$(info $(MTB__SPACE)qbuild_proj         $(CY_HELP_qbuild_proj))
	$(info $(MTB__SPACE)program             $(CY_HELP_program))
	$(info $(MTB__SPACE)program_proj        $(CY_HELP_program_proj))
	$(info $(MTB__SPACE)qprogram            $(CY_HELP_qprogram))
	$(info $(MTB__SPACE)qprogram_proj       $(CY_HELP_qprogram_proj))
	$(info $(MTB__SPACE)clean               $(CY_HELP_clean))
	$(info $(MTB__SPACE)help                $(CY_HELP_help))
	$(info $(MTB__SPACE)prebuild            $(CY_HELP_prebuild))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)IDE make targets                                   )
	$(info =======================================                       )
	$(info $(MTB__SPACE)eclipse             $(CY_HELP_eclipse))
	$(info $(MTB__SPACE)vscode              $(CY_HELP_vscode))
	$(info $(MTB__SPACE)ewarm8              $(CY_HELP_ewarm8))
	$(info $(MTB__SPACE)uvision5            $(CY_HELP_uvision5))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)Utility make targets                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)progtool            $(CY_HELP_progtool))
	$(info $(MTB__SPACE)printlibs           $(CY_HELP_printlibs))
	$(info $(MTB__SPACE)check               $(CY_HELP_check))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)Basic configuration make variables                 )
	$(info =======================================                       )
	$(info $(MTB__SPACE)TARGET              $(CY_HELP_TARGET))
	$(info $(MTB__SPACE)CORE                $(CY_HELP_CORE))
	$(info $(MTB__SPACE)CORE_NAME           $(CY_HELP_CORE_NAME))
	$(info $(MTB__SPACE)APPNAME             $(CY_HELP_APPNAME))
	$(info $(MTB__SPACE)TOOLCHAIN           $(CY_HELP_TOOLCHAIN))
	$(info $(MTB__SPACE)CONFIG              $(CY_HELP_CONFIG))
	$(info $(MTB__SPACE)VERBOSE             $(CY_HELP_VERBOSE))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)Advanced configuration make variables              )
	$(info =======================================                       )
	$(info $(MTB__SPACE)SOURCES             $(CY_HELP_SOURCES))
	$(info $(MTB__SPACE)INCLUDES            $(CY_HELP_INCLUDES))
	$(info $(MTB__SPACE)DEFINES             $(CY_HELP_DEFINES))
	$(info $(MTB__SPACE)MVE_SELECT          $(CY_HELP_MVE_SELECT))
	$(info $(MTB__SPACE)VFP_SELECT          $(CY_HELP_VFP_SELECT))
	$(info $(MTB__SPACE)VFP_SELECT_PRECISION $(CY_HELP_VFP_SELECT_PRECISION))
	$(info $(MTB__SPACE)CFLAGS              $(CY_HELP_CFLAGS))
	$(info $(MTB__SPACE)CXXFLAGS            $(CY_HELP_CXXFLAGS))
	$(info $(MTB__SPACE)ASFLAGS             $(CY_HELP_ASFLAGS))
	$(info $(MTB__SPACE)LDFLAGS             $(CY_HELP_LDFLAGS))
	$(info $(MTB__SPACE)LDLIBS              $(CY_HELP_LDLIBS))
	$(info $(MTB__SPACE)LINKER_SCRIPT       $(CY_HELP_LINKER_SCRIPT))
	$(info $(MTB__SPACE)PREBUILD            $(CY_HELP_PREBUILD))
	$(info $(MTB__SPACE)POSTBUILD           $(CY_HELP_POSTBUILD))
	$(info $(MTB__SPACE)COMPONENTS          $(CY_HELP_COMPONENTS))
	$(info $(MTB__SPACE)DISABLE_COMPONENTS  $(CY_HELP_DISABLE_COMPONENTS))
	$(info $(MTB__SPACE)SEARCH              $(CY_HELP_SEARCH))
	$(info $(MTB__SPACE)SKIP_CODE_GEN       $(CY_HELP_SKIP_CODE_GEN))
	$(info $(MTB__SPACE)MERGE               $(CY_HELP_MERGE))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)BSP make variables                                 )
	$(info =======================================                       )
	$(info $(MTB__SPACE)DEVICE              $(CY_HELP_DEVICE))
	$(info $(MTB__SPACE)ADDITIONAL_DEVICES  $(CY_HELP_ADDITIONAL_DEVICES))
	$(info $(MTB__SPACE)BSP_PROGRAM_INTERFACE $(CY_HELP_BSP_PROGRAM_INTERFACE))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)Getlibs make variables                             )
	$(info =======================================                       )
ifeq ($(MTB_TOOLS__INTERFACE_VERSION),)
	$(info $(MTB__SPACE)CY_GETLIBS_NO_CACHE $(CY_HELP_CY_GETLIBS_NO_CACHE))
	$(info $(MTB__SPACE)CY_GETLIBS_CACHE_PATH  $(CY_HELP_CY_GETLIBS_CACHE_PATH))
	$(info $(MTB__SPACE)CY_GETLIBS_OFFLINE  $(CY_HELP_CY_GETLIBS_OFFLINE))
	$(info $(MTB__SPACE)CY_GETLIBS_OFFLINE_PATH  $(CY_HELP_CY_GETLIBS_OFFLINE_PATH))
endif
ifeq ($(MTB_TOOLS__INTERFACE_VERSION),1)
	$(info $(MTB__SPACE)MTB_USE_LOCAL_CONTENT $(CY_HELP_MTB_USE_LOCAL_CONTENT))
endif
	$(info $(MTB__SPACE)CY_GETLIBS_PATH     $(CY_HELP_CY_GETLIBS_PATH))
	$(info $(MTB__SPACE)CY_GETLIBS_DEPS_PATH  $(CY_HELP_CY_GETLIBS_DEPS_PATH))
	$(info $(MTB__SPACE)CY_GETLIBS_SHARED_PATH $(CY_HELP_CY_GETLIBS_SHARED_PATH))
	$(info $(MTB__SPACE)CY_GETLIBS_SHARED_NAME $(CY_HELP_CY_GETLIBS_SHARED_NAME))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)Path make variables                                )
	$(info =======================================                       )
	$(info $(MTB__SPACE)CY_APP_PATH         $(CY_HELP_CY_APP_PATH))
	$(info $(MTB__SPACE)CY_COMPILER_PATH    $(CY_HELP_CY_COMPILER_PATH))
	$(info $(MTB__SPACE)CY_COMPILER_GCC_ARM_DIR $(CY_HELP_CY_COMPILER_GCC_ARM_DIR))
	$(info $(MTB__SPACE)CY_COMPILER_IAR_DIR $(CY_HELP_CY_COMPILER_IAR_DIR))
	$(info $(MTB__SPACE)CY_COMPILER_ARM_DIR $(CY_HELP_CY_COMPILER_ARM_DIR))
	$(info $(MTB__SPACE)CY_TOOLS_DIR        $(CY_HELP_CY_TOOLS_DIR))
	$(info $(MTB__SPACE)CY_BUILD_LOCATION   $(CY_HELP_CY_BUILD_LOCATION))
	$(info $(MTB__SPACE)CY_PYTHON_PATH      $(CY_HELP_CY_PYTHON_PATH))
	$(info $(MTB__SPACE)MTB_JLINK_DIR       $(CY_HELP_MTB_JLINK_DIR))
	$(info                                                               )
	$(info =======================================                       )
	$(info $(MTB__SPACE)Miscellaneous make variables                       )
	$(info =======================================                       )
	$(info $(MTB__SPACE)CY_IGNORE                $(CY_HELP_CY_IGNORE))
	$(info $(MTB__SPACE)CY_SIMULATOR_GEN_AUTO    $(CY_HELP_CY_SIMULATOR_GEN_AUTO))
	$(info )

mtb_help_tools_start: mtb_help_header
	@:
	$(info =======================================                       )
	$(info $(MTB__SPACE)Tools targets                       )
	$(info =======================================                       )
mtb_help_tools_end: mtb_help_tools_start
help: mtb_help_tools_end
endif

.PHONY: help mtb_help_header mtb_help_tools_start mtb_help_tools_end
