#!/bin/bash

SETS=(set06) #set07 set08 set09 set10)

for s in ${SETS[*]}
do

echo Processing set ${s}

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/caltech/${s}/motion
INPUT_DIR=/data/mdesnoye/pedestrian/caltech/hima/${s}

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs1.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIR}/V018*
do
  echo $dir >> ${INPUT_DIR_FILE}
done

src/EvaluateVisualUtility.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--output_name LABMotion \
--annotations annotations.txt \
--vu_estimator LABMotionVUEstimator \
--vu_estimator_wrapper RelativeEntropyVUWrapper \
--left_image_dir frames \
--image_file_string "I%05i.jpg" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log

done # for SETS
