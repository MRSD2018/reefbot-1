#!/bin/sh 

INPUT_DIR=/data/mdesnoye/fish/tank_videos
SEARCH_STRING="201101*"
OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish

for dir in ${INPUT_DIR}/${SEARCH_STRING}
do
  DAY_PREFIX=`basename ${dir}`
  CUR_DIR=${OUTPUT_DIR}/${DAY_PREFIX}
  mkdir -p ${CUR_DIR}
  for video in ${dir}/*.mp4
  do
    VIDEO_PREFIX=`basename ${video} | sed 's/\..*//g'`
    bin/HighlightMotion --input ${video} \
        --output_dir ${CUR_DIR} \
        --do_blob_output \
        --pareto_thresh 0.04 \
        --min_obj_size 250 \
        --frame_prefix ${VIDEO_PREFIX}- \
        --blob_prefix ${VIDEO_PREFIX}- \
        --blob_rate 1.0 > ${CUR_DIR}/${VIDEO_PREFIX}.log
  done
done
