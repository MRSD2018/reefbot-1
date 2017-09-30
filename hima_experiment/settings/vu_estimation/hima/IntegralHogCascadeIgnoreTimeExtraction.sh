#!/bin/bash

MODEL_DIR=/data/mdesnoye/pedestrian/hima/integral_cascade_ignore_time

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_cascade_ignore_time

DATA_DIR=/data/mdesnoye/hima/link_set/
MODELS=( set9 set10 set19 set22 set400 set600 )
FP_SCALES=( 0.5 1.0 2.0 5.0 0.2 10.0 )

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs1.txt

mkdir -p ${OUTPUT_DIR}

for model in ${MODELS[*]}; do
    for fpScale in ${FP_SCALES[*]}; do
        echo ${DATA_DIR}/${model} > ${INPUT_DIR_FILE}
    src/RunHimaCascade.py \
        --input_dirs ${INPUT_DIR_FILE} \
        --output_dir ${OUTPUT_DIR} \
        --output_name IntegralHOGCascadeHimaIgnoreTime_${model}_${fpScale} \
        --annotations annotations.txt \
        --vu_estimator IntegralHOGCascade \
        --cascade_model_file ${MODEL_DIR}/cascade_${model}_ignore_time_fp_${fpScale}.xml \
        --hog_do_people \
        --hog_do_cache \
        --left_image_dir stereo_cameras/left \
        --image_file_string "img_%04i.bmp" \
        __log:=${OUTPUT_DIR}/rosout_${model}_${fpScale}.log >> ${OUTPUT_DIR}/stdout_${model}_${fpScale}.log 2>> ${OUTPUT_DIR}/stderr_${model}_${fpScale}.log
    done
    rm -rf /home/mdesnoye/.ros/log
done
