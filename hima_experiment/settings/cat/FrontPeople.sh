#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/hima/experiments/cat/front_people_smoothed
INPUT_DIRS=(/data/mdesnoye/hima/cat/front_people \
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
--frame_subset_rate 1 \
--image_file_string "I%05i.png" \
--left_image_dir . \
--person_cascade \
--model_file ${MODEL_FILE} \
--person_thresh 0 \
--frame_estimator HighRelativeEntropy \
--morph_close_size 4 \
--gauss_sigma 15 \
--dist_decay 0.1 \
--min_entropy "[0.05]" \
--frame_expansion "[1.2]" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
