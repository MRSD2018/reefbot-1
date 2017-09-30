#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/haar_fullbody
INPUT_DIRS=(/data/mdesnoye/hima/set_6/set6 \
/data/mdesnoye/hima/set_9/set9 \
/data/mdesnoye/hima/set_10/set10 \
/data/mdesnoye/hima/set_19/set19 \
/data/mdesnoye/hima/set_22/set22 \
/data/mdesnoye/hima/set_400/set400 \
/data/mdesnoye/hima/set_600/set600 \
)

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs1.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir > ${INPUT_DIR_FILE}


  src/EvaluateVisualUtility.py \
      --input_dirs ${INPUT_DIR_FILE} \
      --output_dir ${OUTPUT_DIR} \
      --output_name HaarFullBody \
      --annotations annotations.txt \
      --vu_estimator CascadeDetector \
      --cascade_model_file /usr/local/share/opencv/haarcascades/haarcascade_fullbody.xml \
      --left_image_dir stereo_cameras/left \
      --image_file_string "img_%04i.bmp" \
      __log:=${OUTPUT_DIR}/rosout.log >> ${OUTPUT_DIR}/stdout.log 2>> ${OUTPUT_DIR}/stderr.log

done