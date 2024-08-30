#!/bin/bash
(set -o igncr) 2>/dev/null && set -o igncr; # this comment is required
set -$-ue${DEBUG+xv}

#######################################################################################################################
# This script processes the memory consumption of an application and prints it out to the console.
#
# usage:
#	memcalc.bash <READELFFILE> <AVAILABLEFLASH_KB> <STARTFLASH>
#
#######################################################################################################################

READELFFILE=$1              # file location of readelf output
AVAILABLEFLASH_KB=$2        # Max available internal flash in KB
STARTFLASH=$3               # Start of internal flash
AVAILABLEFLASH=$((AVAILABLEFLASH_KB << 10))

ENDFLASH=$((STARTFLASH + AVAILABLEFLASH))

# Gather the numbers
memcalc() {
    local internalFlash=0

    printf "   ---------------------------------------------------- \n"
    printf "  | %-20s |  %-10s   |  %-10s | \n" 'Section Name' 'Address' 'Size'
    printf "   ---------------------------------------------------- \n"

    while IFS=$' \t\n\r' read -r line; do
        local lineArray=($line)
        local numElem=${#lineArray[@]}

        # Only look at potentially valid lines
        if [[ $numElem -ge 6 ]]; then
            # Section headers
            if [[ ${lineArray[0]} == "["* ]]; then
                local sectionElement=NULL
                local addrElement=00000000
                local sizeElement=000000
                for (( idx = 0 ; idx <= $numElem-4 ; idx = $idx+1 ));
                do
                    if [[ ${lineArray[$idx]} == *"]" ]]; then
                        sectionElement=${lineArray[$idx+1]}
                    fi
                    # Look for regions with SHF_ALLOC = A
                    if [[ ${#lineArray[idx]} -eq 8 ]] && [[ ${#lineArray[idx+1]} -eq 6 ]] && [[ ${#lineArray[idx+2]} -ge 6 ]] && [[ ${#lineArray[idx+2]} -le 7 ]]\
                       && [[ ${lineArray[$idx+4]} == *"A"* ]] ; then
                        addrElement=${lineArray[$idx]}
                        sizeElement=${lineArray[$idx+2]}
                    fi
                done
                heapCheckArray+=($sectionElement)

                # Only consider non-zero size sections
                if [[ $((16#$sizeElement)) != "0" ]]; then
                    printf "  | %-20s |  0x%-10s |  %-10s | \n" $sectionElement $addrElement $((16#$sizeElement))
                fi
            # Program headers
            elif [[ ${lineArray[1]} == "0x"* ]] && [[ ${lineArray[2]} == "0x"* ]] && [[ ${lineArray[3]} == "0x"* ]] && [[ ${lineArray[4]} == "0x"* ]]\
                && [[ ${lineArray[3]} -ge "$STARTFLASH" ]] && [[ ${lineArray[3]} -lt "$ENDFLASH" ]]; then
                # Use the program headers for Flash tally
                internalFlash=$((internalFlash+${lineArray[4]}))
            fi
        fi
    done < "$READELFFILE"

    printf "   ---------------------------------------------------- \n\n"
    printf "  %-41s %-10s \n" 'Total Internal Flash (Available)' $AVAILABLEFLASH
    printf "  %-41s %-10s \n\n" 'Total Internal Flash (Utilized)' $internalFlash
}

memcalc
