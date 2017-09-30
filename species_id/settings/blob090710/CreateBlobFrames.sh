#!/bin/sh
# Configuration script that will create jpegs of all the blobs

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/extractedFish/extractedFish051010_blob/*.blob\"
OUTPUT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/frames/
PARAMS="--frame_width 500 --frame_height 500"

mkdir -p ${OUTPUT_DIR}

# Script to do the actual extraction
bin/CreateBatchBlobFrames.sh ${OUTPUT_DIR} ${INPUT_FILES} ${PARAMS} > ${OUTPUT_DIR}/frame_creation.log