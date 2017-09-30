#!/bin/bash

BASE_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/
BLOB_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/xmlBlobs/
OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/vu_extraction/pr_analysis

mkdir -p ${OUTPUT_DIR}

PROCESS_DIRS=( laplace/*.bag motion/*.bag objectness/*.bag spectral_saliency/*.bag surround/*_1.0_chisq*.bag )

for curDir in ${PROCESS_DIRS[*]}; do
  VU_TYPE=`dirname ${curDir}`
  INPUT_DIR=${BASE_DIR}/"${curDir}"
  OUTPUT_NAME=${OUTPUT_DIR}/${VU_TYPE}.prstats

  src/RunVUEstimatorPRAnalysis.py \
      -i "${INPUT_DIR}" \
      --blob_dir ${BLOB_DIR} \
      -o ${OUTPUT_NAME} \
      --frame_subset_rate 2 \
      __log:=${OUTPUT_DIR}/${VU_TYPE}_rosout.log > ${OUTPUT_DIR}/${VU_TYPE}_stdout.log 2> ${OUTPUT_DIR}/${VU_TYPE}_stderr.log
done
