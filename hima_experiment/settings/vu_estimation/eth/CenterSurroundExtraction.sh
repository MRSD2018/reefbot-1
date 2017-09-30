#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/surround
INPUT_DIRS=(/data/mdesnoye/pedestrian/eth/hima/set01 \
/data/mdesnoye/pedestrian/eth/hima/set02 \
/data/mdesnoye/pedestrian/eth/hima/set00 \
)
SCALES=( 0.4 0.6 0.8 1.0 1.2 1.5 2.0 )

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs0.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

for scale in ${SCALES[*]}; do
  OUTPUT_NAME=CenterSurround_${scale}_chisq
  src/EvaluateVisualUtility.py \
  --input_dirs ${INPUT_DIR_FILE} \
  --output_dir ${OUTPUT_DIR} \
  --output_name ${OUTPUT_NAME} \
  --annotations annotations.txt \
  --vu_estimator CenterSurroundHistogram \
  --hist_dist_type chisq \
  --estimator_scales $scale \
  --left_image_dir frames \
  --image_file_string "I%05i.png" \
  __log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
done


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

for scale in ${SCALES[*]}; do
  OUTPUT_NAME=CenterSurround_${scale}_chisq
  src/EvaluateVisualUtility.py \
  --input_dirs ${INPUT_DIR_FILE} \
  --output_dir ${OUTPUT_DIR} \
  --output_name ${OUTPUT_NAME} \
  --annotations annotations.txt \
  --vu_estimator CenterSurroundHistogram \
  --hist_dist_type chisq \
  --estimator_scales $scale \
  --left_image_dir left \
  --image_file_string "img_%04i.png" \
  __log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
done
