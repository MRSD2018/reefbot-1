#!/bin/sh
# Configuration script that will extract some surf features on all the
# blob images

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/extractedFish/extractedFish031010/bigfish*.jpg\"
OUTPUT_DIR=/data/mdesnoye/fish/extractedFish/firstLabeling070610/allSurfDescriptors/
PARAMS="--surf_detector --surf_hessian_threshold=400 --surf_descriptor
--surf_octaves=3 --surf_octave_layers=4 --surf_extended=false"
SUFFIX=surf

# Script to do the actual extraction
bin/ExtractBatchFeatures.sh ${OUTPUT_DIR} ${INPUT_FILES} ${SUFFIX} ${PARAMS}