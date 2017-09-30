#!/bin/bash

DETECTOR_DIR=/data/mdesnoye/pedestrian/inria/subwindow_detectors
OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/subwindow_hog/timing
INPUT_IMAGES=( \
    /data/mdesnoye/pedestrian/eth/hima/set00/frames/I00147.png \
    /data/mdesnoye/pedestrian/eth/hima/set00/frames/I00247.png \
    /data/mdesnoye/pedestrian/eth/hima/set00/frames/I00347.png \
    /data/mdesnoye/pedestrian/eth/hima/set00/frames/I00447.png \
    /data/mdesnoye/pedestrian/eth/hima/set00/frames/I00547.png \
    /data/mdesnoye/pedestrian/eth/hima/set02/frames/I00049.png \
    /data/mdesnoye/pedestrian/eth/hima/set02/frames/I00149.png \
    /data/mdesnoye/pedestrian/eth/hima/set02/frames/I00249.png \
    /data/mdesnoye/pedestrian/eth/hima/set02/frames/I00349.png \
    /data/mdesnoye/pedestrian/eth/hima/Linthescher/left/img_0046.png \
    /data/mdesnoye/pedestrian/eth/hima/Linthescher/left/img_0146.png \
    /data/mdesnoye/pedestrian/eth/hima/Linthescher/left/img_0246.png \
    /data/mdesnoye/pedestrian/eth/hima/Linthescher/left/img_0316.png \
    /data/mdesnoye/pedestrian/eth/hima/Lowenplatz/left/img_0018.png \
    /data/mdesnoye/pedestrian/eth/hima/Lowenplatz/left/img_0118.png \
    /data/mdesnoye/pedestrian/eth/hima/Lowenplatz/left/img_0198.png \
)

BIN=`rospack find visual_utility`/bin/EstimateVUTiming

mkdir -p ${OUTPUT_DIR}

SIZES=`ls -1 ${DETECTOR_DIR}/*.xml | sed 's/.*\///g' | cut -d '-' -f 2,3 | sort | uniq`

for size in ${SIZES}
do
  echo ${BIN} \
      _vu_estimator:=IntegralHOGDetector \
      _use_cache:=true \
      _hog_model_file:=${DETECTOR_DIR}/inria_people_64x128-${size}-0-0.xml \
      _log:=${OUTPUT_DIR}/${size}_rosout.log \
      ${OUTPUT_DIR}/subwindow_${size}_timing.txt \
      ${INPUT_IMAGES[*]} \
      #> ${OUTPUT_DIR}/${size}_stdout.log >2 ${OUTPUT_DIR}/${size}_stderr.log
done
