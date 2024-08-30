#!/bin/bash
(set -o igncr) 2>/dev/null && set -o igncr
set -$-e${DEBUG+xv}
set -o errtrace

#######################################################################################################################
# This script creates a archive file for the Infineon online simulator
#
# usage:
#     simulator_gen.bash <OUTPUT_DIR> <APP_NAME> <APP_PATH> <GETLIBS_PATH> <INPUT_FILE>
#       The OUTPUT_DIR and APP_PATH are absolute paths.
#       The GETLIBS_PATH is relative.
#       The input file contains 1 line of space separated entry of source files followed by 1 line of space separated include dirs
#
#######################################################################################################################

# this function is called just before a script exits (for any reason). It's given the scripts exit code.
# E.g., if there is a failure due to "set -e" this function will still be called.
trap_exit() {
    # this is the return code the main part of the script want to return
    local result=$?

    # turn off the EXIT trap
    trap - EXIT

    # Print WARNING messages if they occur
    echo
    if [[ ${#WARNING_MESSAGES[@]} -ne 0 ]]; then
        echo ==============================================================================
        for line in "${WARNING_MESSAGES[@]}"; do
            echo "$line"
        done
        echo
    fi
    
    if [ "$result" != 0 ]; then
        echo ==============================================================================
        echo "--ABORTING--"
        echo "Script      : $0"
        echo "Bash path   : $BASH"
        echo "Bash version: $BASH_VERSION"
        echo "Exit code   : $result"
        echo "Call stack  : ${FUNCNAME[*]}"
    fi
    exit $result
}

trap "trap_exit" EXIT

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"

OUTPUT_DIR=$1
shift
APP_NAME=$1
shift
APP_LOC=$1
shift
GETLIBS_PATH=$1
shift
INPUT_FILE=$1

TAR_FILE=$APP_NAME.tar
TGZ_FILE=$OUTPUT_DIR/$TAR_FILE.tgz

lines=()
while IFS= read -r line; do
  lines+=("$line")
done < $INPUT_FILE

FILES=${lines[0]}
INCLUDES=${lines[1]}

APP_NAME=$(basename $APP_LOC)

SHARED_FILES=()
APP_FILES=()
ABSOLUTE_FILES=()

for value in $INCLUDES
do
    pushd $value >/dev/null
    for file in $(ls)
    do
        if [ -f "$file" ]; then
            base_name="${file%.*}"
            # this won't work for file without extension. That get included by base_name==file test.
            ext_name="${file##*.}"
            if [[ $base_name == $file || $ext_name == h || $ext_name == hpp || $ext_name == hxx || $ext_name == inc || $ext_name == .H || $ext_name == HPP || $ext_name == HXX || $ext_name == INC ]]; then
                FILES+=" $value/$file"
            fi
        fi
    done
    popd >/dev/null
done

# Figure out the ascestor directory
for value in $FILES
do
    if [[ $value == $GETLIBS_PATH/* ]]; then
        SHARED_FILES+=( "${value/$GETLIBS_PATH\//}" )
    elif [[ "$value" == "./*" || -f "./$value" || -f "$APP_LOC/$value" ]]; then
        APP_FILES+=( "$APP_NAME/${value#./}" )
    elif [[ $value == $APP_LOC/* ]]; then
        APP_FILES+=( "${value/$APP_LOC/$APP_NAME}" )
    else
        ABSOLUTE_FILES+=( "$value" )
    fi
done

rm -f $TAR_FILE

tar -cf $TAR_FILE -C $SCRIPT_DIR codetype.ini

if [[ ${APP_FILES[@]} != '' ]]; then
    tar -rf $TAR_FILE -C ".." ${APP_FILES[@]} >/dev/null
fi

if [[ ${SHARED_FILES[@]} != '' ]]; then
	tar -rf $TAR_FILE -C $GETLIBS_PATH ${SHARED_FILES[@]}
fi

for abspath in ${ABSOLUTE_FILES[@]}
do
	DIR_NAME=$(dirname $abspath)
	FILE_NAME=${abspath##*/}
	tar -rf $TAR_FILE -C $DIR_NAME $FILE_NAME
done

gzip < $TAR_FILE > $TGZ_FILE
rm -f $TAR_FILE

echo Simulator archive file $TGZ_FILE successfully generated
