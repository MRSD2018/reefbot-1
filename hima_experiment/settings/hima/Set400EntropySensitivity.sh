#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/hima/experiments/set400entropysensitivity
INPUT_DIRS=(/data/mdesnoye/hima/set_400/set400 \
)
MODEL_FILE=/home/mdesnoye/src/fish/ros/cascade_parts_detector/src/voc_release4/VOC2009/person_final.mat

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

src/RunHimaExperiment.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--annotations annotations.txt \
--frame_estimator HighRelativeEntropy \
--person_cascade \
--model_file ${MODEL_FILE} \
--person_thresh 0 \
--morph_close_size 8 \
--min_entropy "[0.01, 0.05, 0.1, 0.2, 0.001, 0.5]" \
--frame_expansion "[2.0]" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
