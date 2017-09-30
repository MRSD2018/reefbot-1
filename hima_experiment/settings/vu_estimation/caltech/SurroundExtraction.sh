#!/bin/bash

SETS=(set06 set07 set08 set09 set10)

for s in ${SETS[*]}
do

echo Processing set ${s}

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/caltech/${s}/surround
INPUT_DIR=/data/mdesnoye/pedestrian/caltech/hima/${s}

SCALES=( 0.4 0.6 0.8 1.0 1.2 1.5 2.0 )

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs1.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIR}/V*
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
  --image_file_string "I%05i.jpg" \
  --frame_subset_rate 10 \
  __log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
done

done # for SETS