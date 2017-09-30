#!/bin/sh

INPUT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/allCSurfDescriptors/
INPUT_FILES=\"*.csurf.*\"
OUTPUT=/data/mdesnoye/fish/extractedFish/blob090710/csurf.dict
PARAMS="--max_iter=200 --init=sample --replicate=3 --distance_type=euclidean
--num_words=25000 --fraction_use=0.3"

# Do the execution
bin/MakeDictionary.sh ${OUTPUT} ${INPUT_DIR} ${INPUT_FILES} ${PARAMS}