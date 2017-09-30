#!/bin/sh
# Configuration script that will extract some surf features on all the
# blob images

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/extractedFish/extractedFish051010_blob/*.blob\"
OUTPUT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/allHSVDescriptors/
PARAMS="--random_detector --random_frac 0.005 --color_descriptor --color_converter CV_BGR2HSV"
SUFFIX=hsv

mkdir -p ${OUTPUT_DIR}

# Script to do the actual extraction
bin/ExtractBatchFeatures.sh ${OUTPUT_DIR} ${INPUT_FILES} ${SUFFIX} ${PARAMS} > ${OUTPUT_DIR}/extraction.log