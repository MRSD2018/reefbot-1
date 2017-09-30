#!/bin/bash

DETECTOR_DIR=/data/mdesnoye/pedestrian/inria/subwindow_detectors
OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog/timing
INPUT_IMAGES=( \
    /data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0010.bmp \
    /data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0110.bmp \
    /data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0310.bmp \
    /data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0510.bmp \
    /data/mdesnoye/hima/set_19/set19/stereo_cameras/left/img_0020.bmp \
    /data/mdesnoye/hima/set_19/set19/stereo_cameras/left/img_0120.bmp \
    /data/mdesnoye/hima/set_19/set19/stereo_cameras/left/img_0320.bmp \
    /data/mdesnoye/hima/set_19/set19/stereo_cameras/left/img_0520.bmp \
    /data/mdesnoye/hima/set_6/set6/stereo_cameras/left/img_0002.bmp \
    /data/mdesnoye/hima/set_6/set6/stereo_cameras/left/img_0102.bmp \
    /data/mdesnoye/hima/set_6/set6/stereo_cameras/left/img_0302.bmp \
    /data/mdesnoye/hima/set_6/set6/stereo_cameras/left/img_0502.bmp \
    /data/mdesnoye/hima/set_6/set6/stereo_cameras/left/img_0602.bmp \
)

BIN=`rospack find hog_detector`/bin/ProfileIntegralHogDetector

mkdir -p ${OUTPUT_DIR}

 ${BIN} \
    _log:=${OUTPUT_DIR}/profile_rosout.log \
    --output_dir ${OUTPUT_DIR} \
    ${INPUT_IMAGES[*]} \
    #> ${OUTPUT_DIR}/profile_stdout.log >2 ${OUTPUT_DIR}/profile_stderr.log
