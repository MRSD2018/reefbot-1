#!/bin/sh

INPUT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/allSiftDescriptors/
INPUT_FILES=\"*.sift.*\"
OUTPUT=/data/mdesnoye/fish/experiments/extraction081011/20110102/sift.dict
PARAMS="--max_iter=200 --init=kmeanspp --replicate=3 --distance_type=euclidean
--num_words=25000 --fraction_use=0.35 --branching_factor=32 --hierarchical"

# Do the execution
bin/MakeDictionary.sh ${OUTPUT} ${INPUT_DIR} ${INPUT_FILES} ${PARAMS}
