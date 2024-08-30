#!/bin/bash 
(set -o igncr) 2>/dev/null && set -o igncr; # this comment is required
set -$-ue${DEBUG+x}

#######################################################################################################################
# This script generates the compilation data file.
#
# usage:
#	gen_compile_commands.bash <output_file> <working_directory> [input_file]+
#
# input file lines:
#   1: arguments
#   2: source files (space separated entries)
#   3: object files (space separated entries)
#
# Expects 3 inputs files, 1 for .s, 1 for .c, and 1 for .cpp
#
#######################################################################################################################

output_file=$1
shift 1

working_directory=$1
shift 1

echo '[' > $output_file

first_entry=true

for input_file in "$@"
do
    line_number=1
    while IFS= read -r line
    do
        if [ "$line_number" = "1" ]; then
            arguments=$line
        elif [ "$line_number" = "2" ]; then
            src_files=$line
        else
            obj_files=$line
        fi
        ((line_number++))
    done < $input_file

    IFS=' ' read -r -a src_file_array <<< "$src_files"
    IFS=' ' read -r -a obj_file_array <<< "$obj_files"

    array_length=${#src_file_array[@]}
    for (( i=0; i<array_length; i++ ));
    do
        src_file=${src_file_array[$i]}
        obj_file=${obj_file_array[$i]}

        if [ "$first_entry" != "true" ]; then
            echo "," >> $output_file
        else
            first_entry=false
        fi

        echo '    {' >> $output_file
        echo "        \"directory\": \"$working_directory\"," >> $output_file
        echo "        \"file\": \"$src_file\"," >> $output_file
        echo "        \"command\": \"$arguments $obj_file $src_file\"" >> $output_file
        echo -n '    }' >> $output_file
    done
done

echo >> $output_file
echo ']' >> $output_file
