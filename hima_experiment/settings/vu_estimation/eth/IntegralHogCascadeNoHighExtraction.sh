#!/bin/bash

MODEL_DIR=/data/mdesnoye/pedestrian/eth/integral_cascade_no_high_level

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/integral_hog_cascade_no_high_level
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

for modelFile in ${MODEL_DIR}/*.xml ; do
    model=`basename ${modelFile} | cut -d '_' -f 2,4 | sed 's/\.xml//g'`
    if [ $model == "Crossing_0.2" ]; then
      src/RunHimaCascade.py \
        --input_dirs ${INPUT_DIR_FILE} \
        --output_dir ${OUTPUT_DIR} \
        --output_name IntegralHOGCascadeETH_${model} \
        --annotations annotations.txt \
        --vu_estimator IntegralHOGCascade \
        --cascade_model_file ${modelFile} \
        --hog_do_people \
        --hog_do_cache \
        --left_image_dir left \
        --image_file_string "img_%04i.png" \
        __log:=${OUTPUT_DIR}/rosout_${model}.log > ${OUTPUT_DIR}/stdout_${model}.log 2> ${OUTPUT_DIR}/stderr_${model}.log
   fi
done
exit
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

for modelFile in ${MODEL_DIR}/*.xml ; do
    model=`basename ${modelFile} | cut -d '_' -f 2,4 | sed 's/\.xml//g'`
     src/RunHimaCascade.py \
        --input_dirs ${INPUT_DIR_FILE} \
        --output_dir ${OUTPUT_DIR} \
        --output_name IntegralHOGCascadeETH_${model} \
        --annotations annotations.txt \
        --vu_estimator IntegralHOGCascade \
        --cascade_model_file ${modelFile} \
        --hog_do_people \
        --hog_do_cache \
        --left_image_dir frames \
        --image_file_string "I%05i.png" \
        __log:=${OUTPUT_DIR}/rosout_${model}.log > ${OUTPUT_DIR}/stdout_${model}.log 2> ${OUTPUT_DIR}/stderr_${model}.log
done
