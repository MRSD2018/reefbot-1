#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/motion_test
INPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/testBlobs/
MOVIE_DIR=/data/mdesnoye/fish/tank_videos/20110102/


mkdir -p ${OUTPUT_DIR}


python -m pdb src/EvaluateVisualUtility.py \
--blob_dir ${INPUT_DIR} \
--movie_dir ${MOVIE_DIR} \
--output_dir ${OUTPUT_DIR} \
--use_movie \
--fps 30 \
--output_name LABMotion \
--vu_estimator LABMotionVUEstimator \
--vu_estimator_wrapper RelativeEntropyVUWrapper \
--do_testing \
__log:=${OUTPUT_DIR}/rosout.log #> ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log


