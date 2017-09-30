#!/bin/sh

INPUT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/allSurfDescriptors/
INPUT_FILES=\"*.surf.*\"
OUTPUT=/data/mdesnoye/fish/experiments/extraction081011/20110102/surf_linear.dict
PARAMS="--max_iter=200 --init=sample --replicate=3 --distance_type=euclidean
--num_words=25000 --fraction_use=0.28"

# Do the execution
bin/MakeDictionaryLinear.sh ${OUTPUT} ${INPUT_DIR} ${INPUT_FILES} ${PARAMS}
