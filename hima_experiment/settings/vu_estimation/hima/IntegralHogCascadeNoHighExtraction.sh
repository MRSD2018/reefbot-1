#!/bin/bash

MODEL_DIR=/data/mdesnoye/pedestrian/hima/integral_cascade_no_high_level/

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog_cascade_no_high_level/
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

  for modelFile in ${MODEL_DIR}/*.xml ; do
    model=`basename ${modelFile} | cut -d '_' -f 2,4 | sed 's/\.xml//g'`
    src/RunHimaCascade.py \
        --input_dirs ${INPUT_DIR_FILE} \
        --output_dir ${OUTPUT_DIR} \
        --output_name IntegralHOGCascadeHima_${model} \
        --annotations annotations.txt \
        --vu_estimator IntegralHOGCascade \
        --cascade_model_file ${modelFile} \
        --hog_do_people \
        --hog_do_cache \
        --left_image_dir stereo_cameras/left \
        --image_file_string "img_%04i.bmp" \
        __log:=${OUTPUT_DIR}/rosout_${model}.log >> ${OUTPUT_DIR}/stdout_${model}.log 2>> ${OUTPUT_DIR}/stderr_${model}.log
    done

done