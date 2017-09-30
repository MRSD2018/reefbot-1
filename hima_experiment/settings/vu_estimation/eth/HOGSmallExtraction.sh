#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/hog_small
INPUT_DIRS=(/data/mdesnoye/pedestrian/eth/hima/Crossing \
/data/mdesnoye/pedestrian/eth/hima/Lowenplatz \
/data/mdesnoye/pedestrian/eth/hima/Linthescher \
)

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs1.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

src/EvaluateVisualUtility.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--output_name HOGSmallDetector \
--annotations annotations.txt \
--vu_estimator HOGDetector \
--no_hog_do_people \
--hog_model_file /data/mdesnoye/pedestrian/hog_training/32x64/model.hog \
--hog_do_cache \
--img_scaling 0.5 \
--vu_estimator_wrapper ScaledDetectorWrapper \
--left_image_dir left \
--image_file_string "img_%04i.png" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log


INPUT_DIRS=(/data/mdesnoye/pedestrian/eth/hima/set01 \
/data/mdesnoye/pedestrian/eth/hima/set02 \
/data/mdesnoye/pedestrian/eth/hima/set00 \
)

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs0.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

src/EvaluateVisualUtility.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--output_name HOGSmallDetector \
--annotations annotations.txt \
--vu_estimator HOGDetector \
--no_hog_do_people \
--hog_model_file /data/mdesnoye/pedestrian/hog_training/32x64/model.hog \
--img_scaling 0.5 \
--hog_do_cache \
--vu_estimator_wrapper ScaledDetectorWrapper \
--left_image_dir frames \
--image_file_string "I%05i.png" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
