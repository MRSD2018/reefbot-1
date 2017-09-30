#!/bin/sh
# Configuration script that will extract some surf features on all the
# blob images

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/extractedFish/extractedFish031010/bigfish*.jpg\"
OUTPUT_DIR=/data/mdesnoye/fish/extractedFish/firstLabeling070610/allHSVDescriptors/
PARAMS="--random_detector --random_frac 0.005 --color_descriptor --color_converter CV_RGB2HSV"
SUFFIX=hsv

# Script to do the actual extraction
bin/ExtractBatchFeatures.sh ${OUTPUT_DIR} ${INPUT_FILES} ${SUFFIX} ${PARAMS}