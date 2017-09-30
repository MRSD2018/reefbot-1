#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/surround
INPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/xmlBlobs/
MOVIE_DIR=/data/mdesnoye/fish/tank_videos/20110102/

SCALES=( 0.4 0.6 0.8 1.0 1.2 1.5 2.0 )

mkdir -p ${OUTPUT_DIR}

for scale in ${SCALES[*]}; do
  OUTPUT_NAME=CenterSurround_${scale}_chisq

  src/EvaluateVisualUtility.py \
      --blob_dir ${INPUT_DIR} \
      --movie_dir ${MOVIE_DIR} \
      --output_dir ${OUTPUT_DIR} \
      --fps 30 \
      --output_name ${OUTPUT_NAME} \
      --vu_estimator CenterSurroundHistogram \
      --hist_dist_type chisq \
      --estimator_scales $scale \
      __log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
done


