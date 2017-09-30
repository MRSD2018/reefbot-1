#!/bin/bash

# Converts a list of bags to the new species id.
#
# Usage: BatchConvertSpeciesID.sh <file1> <file2> ... <fileN>

if [ "$#" == "0" ]; then
	echo "BatchConvertSpeciesID.sh <file1> <file2> ... <fileN>"
	exit 1
fi

while (( "$#" )); do

    INPUT_FILE=$1
    TMP_FILE=${INPUT_FILE}.new

    echo Processing ${INPUT_FILE}
    rosrun reefbot_msgs ConvertSpeciesID.py -i ${INPUT_FILE} -o ${TMP_FILE}
    rm ${INPUT_FILE}
    mv ${TMP_FILE} ${INPUT_FILE}

    shift

done