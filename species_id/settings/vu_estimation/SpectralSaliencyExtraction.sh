#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/spectral_saliency
INPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/xmlBlobs/
MOVIE_DIR=/data/mdesnoye/fish/tank_videos/20110102/


mkdir -p ${OUTPUT_DIR}


src/EvaluateVisualUtility.py \
--blob_dir ${INPUT_DIR} \
--movie_dir ${MOVIE_DIR} \
--output_dir ${OUTPUT_DIR} \
--fps 30 \
--output_name SpectralSaliency \
--vu_estimator SpectralSaliency \
--vu_estimator_wrapper RelativeEntropyVUWrapper \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log


