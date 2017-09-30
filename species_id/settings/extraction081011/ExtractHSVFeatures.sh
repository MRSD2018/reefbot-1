#!/bin/sh
# Configuration script that will extract some surf features on all the
# blob images

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blobs/*.blob\"
OUTPUT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/allHSVDescriptors/
PARAMS="--random_detector --random_frac 0.005 --color_descriptor --color_converter CV_BGR2HSV"
SUFFIX=hsv

mkdir -p ${OUTPUT_DIR}

# Script to do the actual extraction
bin/ExtractBatchFeatures.sh ${OUTPUT_DIR} ${INPUT_FILES} ${SUFFIX} ${PARAMS} > ${OUTPUT_DIR}/extraction.log