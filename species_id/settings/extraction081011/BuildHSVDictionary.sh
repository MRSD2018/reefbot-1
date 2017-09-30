#!/bin/sh

INPUT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/allHSVDescriptors/
INPUT_FILES=\"*.hsv.*\"
OUTPUT=/data/mdesnoye/fish/experiments/extraction081011/20110102/hsv.dict
PARAMS="--max_iter=200 --init=kmeanspp --replicate=3 --distance_type=euclidean
--num_words=256 --fraction_use=0.05 --branching_factor=32 --hierarchical"

# Do the execution
bin/MakeDictionary.sh ${OUTPUT} ${INPUT_DIR} ${INPUT_FILES} ${PARAMS}