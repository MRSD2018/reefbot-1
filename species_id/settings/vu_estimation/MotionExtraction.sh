#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/motion
INPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/xmlBlobs/
MOVIE_DIR=/data/mdesnoye/fish/tank_videos/20110102/


mkdir -p ${OUTPUT_DIR}


src/EvaluateVisualUtility.py \
--blob_dir ${INPUT_DIR} \
--movie_dir ${MOVIE_DIR} \
--output_dir ${OUTPUT_DIR} \
--use_movie \
--fps 30 \
--movie_subset_rate 2 \
--opening_size 3 \
--output_name LABMotion \
--vu_estimator LABMotionVUEstimator \
--vu_estimator_wrapper RelativeEntropyVUWrapper \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log


