#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/hima/experiments/test_experiment
INPUT_DIRS=(/data/mdesnoye/hima/set_test/set6 \
)
MODEL_FILE=/home/mdesnoye/src/fish/ros/cascade_parts_detector/src/voc_release4/VOC2009/person_final.mat

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs.txt

mkdir -p ${OUTPUT_DIR}

rm ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

python -m pdb src/RunHimaExperiment.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--annotations annotations_test.txt \
--person_cascade \
--model_file ${MODEL_FILE} \
__log:=${OUTPUT_DIR}/rosout.log
