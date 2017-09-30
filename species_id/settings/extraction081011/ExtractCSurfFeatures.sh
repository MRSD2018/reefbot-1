#!/bin/sh
# Configuration script that will extract some surf features on all the
# blob images

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blobs/*.blob\"
OUTPUT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/allCSurfDescriptors/
PARAMS="--surf_detector --surf_hessian_threshold=400 --cinvariant_color_surf
--surf_octaves=3 --surf_octave_layers=4 --surf_extended=false"
SUFFIX=csurf

mkdir -p ${OUTPUT_DIR}

# Script to do the actual extraction
bin/ExtractBatchFeatures.sh ${OUTPUT_DIR} ${INPUT_FILES} ${SUFFIX} ${PARAMS} > ${OUTPUT_DIR}/extraction.log