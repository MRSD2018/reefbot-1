#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/set_test_saliency
INPUT_DIRS=(/data/mdesnoye/hima/set_test/set6 \
)

#export CPUPROFILE=/home/mdesnoye/tmp/vu_saliency.prof

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

python -m pdb src/EvaluateVisualUtility.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--annotations annotations_test.txt \
--vu_estimator SpectralSaliency \
--vu_estimator_wrapper RelativeEntropyVUWrapper \
__log:=${OUTPUT_DIR}/rosout.log #> ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log

#unset CPUPROFILE
