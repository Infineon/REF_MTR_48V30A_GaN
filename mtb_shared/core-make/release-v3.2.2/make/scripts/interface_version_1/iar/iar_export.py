#!/usr/bin/env python3

import argparse
import os.path
import ntpath
import re
import sys
from pathlib import PurePath
from collections import namedtuple
from collections import defaultdict
from xml.dom import minidom
from xml.etree import ElementTree

PROJ_DIR_STR = "$PROJ_DIR$"
GRP_UNFILTERED = ("unf:/tered*",)
GRP_HEADER = "Header"
GRP_SRC = "Source"

ATTR_NAME = "name"
ATTR_VERSION = "version"

ELEM_NAME = "name"
ELEM_PATH = "path"
ELEM_IAR_PROJ_CONN = "iarProjectConnection"
ELEM_DEVICE = "device"
ELEM_INC_PATH = "includePath"
ELEM_DEFS = "defines"
ELEM_DEF = "define"
ELEM_ASM_INC_PATH = "asmIncludePath"
ELEM_ASM_DEFS = "asmDefines"
ELEM_LINK_FILE = "linkerFile"
ELEM_OVERRIDE = "override"
ELEM_GROUP = "group"
ELEM_FILES = "files"
ELEM_LINK_OPTS = "linkerExtraOptions"
ELEM_ARG = "arg"

REMOVE_QUOTE_REGEX = re.compile(r"^\"|\"$")

# build data file format
# ------------------------------------#
# AppName
# Core
# Linker script
# Defines (Comma separated list)
# Include Paths (Comma separated list)
# .c Source Files (Comma separated list)
# .S Asm Files (Comma separated list)
# .h Header files (Comma separated list)
# .a Lib files (Comma separated list)
# Search directories (Comma separated list)
# mtb_shared directory
# --------------------------------------

# recipe data file format
# ------------------------------------#
# device name
# --------------------------------------

def parseIarData(build_file, recipe_file):
    """
    Parses a .iardata file to extract the information necessary for project connection file creation

    Returns a tuple of (project_name device_name core linker_script defineList includePathList cSrcList asmSrcList headersList libsList)
    """
    ProjectData = namedtuple("ProjectData", "projectName deviceName core linkerScript defineList includePathList cSrcList asmSrcList headersList libsList searches sharedLibDir")

    with open(build_file, 'r') as fp:
        projectName = fp.readline().strip()
        core = fp.readline().strip()
        linkerScript = fp.readline()
        # filter() returns an iteration which can only be used once
        # hence converting it back to a list to able to iterate multiple times.
        defineList = list(filter(None, fp.readline().strip().split(',')))
        includePathList = list(filter(None, fp.readline().strip().split(',')))
        cSrcList = list(filter(None, fp.readline().strip().split(',')))
        asmSrcList = list(filter(None, fp.readline().strip().split(',')))
        headersList = list(filter(None, fp.readline().strip().split(',')))
        libsList = list(filter(None, fp.readline().strip().split(',')))
        searches = list(filter(None, fp.readline().strip().split(',')))
        sharedLibDir = fp.readline().strip()

    with open(recipe_file, 'r') as fp:
        device = fp.readline().strip()

    return ProjectData(projectName, device, core, linkerScript, defineList, includePathList, cSrcList, asmSrcList, headersList, libsList, searches, sharedLibDir)

def getDuplicateFiles(sourceList):
    seen = set()
    dups = []
    for file in sourceList:
        fileNameExt = os.path.basename(cleanUpPath(file))
        fileName = os.path.splitext(fileNameExt)[0]
        if fileName not in seen:
            seen.add(fileName)
        else:
            dups.append(fileNameExt)
    return dups

def cleanUpPath(path):
    """
    This function 
    1) Removes extra beginning/trailing quotes, spaces and newlines.
    2) Appends $PROJ_DIR$ to all paths. All paths are assumed to be relative to the Makefile.
    3) Normalizes the paths.

    Returns a clean up path.
    """
    # Remove extra quotes and spaces
    cleanPath = path.strip()
    cleanPath = cleanPath.strip("\"")
    # The normalize operation needs to happen before prepend project directory 
    # variable operation. After appending the placeholder variable to the output 
    # is not a standard path so the normalize operation does not work correctly.
    cleanPath = ntpath.normpath(cleanPath)
    # Append project dir
    cleanPath = ntpath.join(PROJ_DIR_STR, cleanPath)
    return cleanPath

def cleanUpDefine(define):
    """
    This function 
    1) Removes extra beginning/trailing quotes, spaces and newlines.
    2) Changes to Windows style slashes for paths in defines.

    Returns a clean up path.
    """
    # Remove extra quotes and trailing spacess
    cleanDefine = define.strip()
    cleanDefine = REMOVE_QUOTE_REGEX.sub("", cleanDefine)
    # Normalize paths in defines if any present.
    cleanDefine = ntpath.normpath(cleanDefine)
    return cleanDefine

def printPretty(root):
    ugly = ElementTree.tostring(root, encoding='utf8', method='xml')
    parsed = minidom.parseString(ugly) 
    return parsed.toprettyxml(indent="  ", encoding='utf8')

def processSearches(sharedLibsDir, searches):
    # return a tuple, the first is path of the search item, the second is tuple of sub groups to create
    cleanSharedLibsDir = cleanUpPath(sharedLibsDir)
    searchDict = {}

    for search in searches:
        cleanSearch = cleanUpPath(search)
        if cleanSharedLibsDir and cleanSearch.startswith(cleanSharedLibsDir):
            # for directory under sharedLibsDir, create recursive subgroup.
            searchDict[cleanSearch] = tuple(re.split('/|\\\\', cleanSearch[len(cleanSharedLibsDir) + 1 : ]))
        else:
            searchDict[cleanSearch] = (os.path.basename(re.split('/|\\\\', cleanSearch)[-1]),)
    return searchDict

def filterFiles(groupDict, searchDict, fileList):
    """
    Filters the files into groups. This function currently only support MTB project structure.
    It assumes a libs directory that contains all libaries used by the project and  creates a 
    group for each folder under libs directory. All application source is added at the top level
    project under IAR.
    """
    for fl in fileList:
        cleanFile = cleanUpPath(fl)
        dirsList = PurePath(fl.strip().strip("\"")).parts
        try:
            # Find the first libs directory.
            index = dirsList.index("libs")
            # Any child of libs directory is a group.
            grp = dirsList[index + 1]
            groupDict[(grp,)].append(cleanFile)
        except ValueError:
            # group items in make search into groups
            addedToSearchGroup = False
            for k, v in searchDict.items():
                if cleanFile.startswith(k):
                    groupDict[v].append(cleanFile)
                    addedToSearchGroup = True
            if not addedToSearchGroup:
                groupDict[GRP_UNFILTERED].append(cleanFile)
            
def createGroup(root, group, fileList):
    """
    Creates group for each key in the the groupDict.
    It also segregates files into Header and Source group based on file extension.
    """
    topGroupElem = ElementTree.SubElement(root, ELEM_GROUP, {ATTR_NAME: group[0]})
    if len(group) == 1:
        headerGroupElem = None
        sourceGroupElem = None
        pathElem = None
        for fl in fileList:
            if fl.endswith(".h"):
                if headerGroupElem == None:
                    headerGroupElem = ElementTree.SubElement(topGroupElem, ELEM_GROUP, {ATTR_NAME: GRP_HEADER})
                pathElem = ElementTree.SubElement(headerGroupElem, ELEM_PATH)
            else:
                if sourceGroupElem == None:
                    sourceGroupElem = ElementTree.SubElement(topGroupElem, ELEM_GROUP, {ATTR_NAME: GRP_SRC})
                pathElem = ElementTree.SubElement(sourceGroupElem, ELEM_PATH)
            pathElem.text = fl
    else:
        createGroup(topGroupElem, group[1:], fileList)

def generateIpcf(build_file, recipe_file, outFile):
    """
    Generates the IAR project connection files based on data from Make Build system.

    Returns error if error.
    """
def generateIpcf(build_file, recipe_file, outFile):
    projectData = parseIarData(build_file, recipe_file)
    if projectData.deviceName == None:
        raise Exception("Invalid file format for input file.\n")

    cSrcDups = getDuplicateFiles(projectData.cSrcList)
    asmSrcDups = getDuplicateFiles(projectData.asmSrcList)
    if len(cSrcDups) != 0 or len(asmSrcDups) != 0:
        print("WARNING: IAR Embedded Workbench does not support files with the same "
              "name even if they have different extensions. There were multiple source "
              "files detected with name(s):", end = ' ')
        print(*cSrcDups, sep = ";", end=' ')
        print(*asmSrcDups, sep = ";")

    root = ElementTree.Element(ELEM_IAR_PROJ_CONN, {ATTR_NAME: projectData.projectName, ATTR_VERSION: '1.9'})
    # Device element
    deviceElem = ElementTree.SubElement(root, ELEM_DEVICE)
    nameElem = ElementTree.SubElement(deviceElem, ELEM_NAME)
    nameElem.text = projectData.deviceName

    # The IAR compiler and assembler need to have the defines and include paths 
    # specified separately. The same list is passed to both below.

    # Include paths and Asm Include paths
    includeElem = ElementTree.SubElement(root, ELEM_INC_PATH)
    asmIncludeElem = ElementTree.SubElement(root, ELEM_ASM_INC_PATH)
    for inc in projectData.includePathList:
        cleanPath = cleanUpPath(inc)
        pathElem = ElementTree.SubElement(includeElem, ELEM_PATH)
        pathElem.text = cleanPath
        pathElem = ElementTree.SubElement(asmIncludeElem, ELEM_PATH)
        pathElem.text = cleanPath

    # Defines and Asm defines
    defElem = ElementTree.SubElement(root, ELEM_DEFS)
    asmDefElem = ElementTree.SubElement(root, ELEM_ASM_DEFS)
    for define in projectData.defineList:
        cleanDef = cleanUpDefine(define)
        pathElem = ElementTree.SubElement(defElem, ELEM_DEF)
        pathElem.text = cleanDef
        pathElem = ElementTree.SubElement(asmDefElem, ELEM_DEF)
        pathElem.text = cleanDef

    # Linker script file
    linkerFileElem = ElementTree.SubElement(root, ELEM_LINK_FILE)
    linkerOverrideElem = ElementTree.SubElement(linkerFileElem, ELEM_OVERRIDE)
    linkerOverrideElem.text = "true"
    linkerPathElem = ElementTree.SubElement(linkerFileElem, ELEM_PATH)
    linkerPathElem.text = cleanUpPath(projectData.linkerScript)

    # Lib Files
    linkerOptsElem = ElementTree.SubElement(root, ELEM_LINK_OPTS)
    for lib in projectData.libsList:
        argElem = ElementTree.SubElement(linkerOptsElem, ELEM_ARG)
        argElem.text = cleanUpPath(lib)

    searchesDict = processSearches(projectData.sharedLibDir, projectData.searches)
        
    groupDict = defaultdict(list)
    filterFiles(groupDict, searchesDict, projectData.headersList)
    filterFiles(groupDict, searchesDict, projectData.cSrcList)
    filterFiles(groupDict, searchesDict, projectData.asmSrcList)

    filesElem = ElementTree.SubElement(root, ELEM_FILES)
    for grp in sorted(groupDict.keys()):
        if grp != GRP_UNFILTERED:
            createGroup(filesElem, grp, groupDict[grp])

    for fl in groupDict[GRP_UNFILTERED]:
        pathElem = ElementTree.SubElement(filesElem, ELEM_PATH)
        pathElem.text = fl

    # The printPretty function takes care of encoding hence the file is
    # opened in binary mode.
    with open(outFile, 'wb') as opf:
        opf.write(printPretty(root))

def is_valid_file(parser, arg):
    if not os.path.isfile(arg):
        parser.error("The file %s does not exist!" % arg)
    return arg

def run_export():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-build_data", dest="build_file", required=True,
                    help="Project data file generated by `make TOOLCHAIN=IAR ewarm8`", metavar="FILE",
                    type=lambda x: is_valid_file(argParser, x))
    argParser.add_argument("-recipe_data", dest="recipe_data", required=True,
                    help="Project data file generated by `make TOOLCHAIN=IAR ewarm8`", metavar="FILE",
                    type=lambda x: is_valid_file(argParser, x))
    argParser.add_argument("-o", dest="outFile", required=True, 
                    help="IAR Project connection file. By default specified by make build system: (APPNAME).ipcf", metavar="FILE")

    args = argParser.parse_args()
    try:
        generateIpcf(args.build_file, args.recipe_data, args.outFile)
    except Exception as error:
        print("ERROR: %s" % error)
        sys.exit(1)
    else:
        print("Generated IAR Project connection file: %s" % args.outFile)

if __name__ == '__main__':
    try:
        run_export()
    except Exception as error:
        print("ERROR: %s" % error)
        sys.exit(1)
