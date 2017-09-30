#!/bin/sh

INPUT_FILES=\"/data/mdesnoye/fish/extractedFish/firstLabeling070610/allSurfDescriptors/*.surf\"
OUTPUT=/data/mdesnoye/fish/extractedFish/firstLabeling070610/surf.dict
PARAMS="--max_iter=200 --init=sample --replicate=3 --distance_type=euclidean
--num_words=25000 --fraction_use=0.75"

# Do the execution
bin/MakeDictionary.sh ${OUTPUT} ${INPUT_FILES} ${PARAMS}