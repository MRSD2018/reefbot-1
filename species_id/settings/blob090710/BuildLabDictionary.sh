#!/bin/sh

INPUT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/allLabDescriptors/
INPUT_FILES=\"*.lab.*\"
OUTPUT=/data/mdesnoye/fish/extractedFish/blob090710/lab.dict
PARAMS="--max_iter=200 --init=sample --replicate=3 --distance_type=euclidean
--num_words=256 --fraction_use=0.05"

# Do the execution
bin/MakeDictionary.sh ${OUTPUT} ${INPUT_DIR} ${INPUT_FILES} ${PARAMS}