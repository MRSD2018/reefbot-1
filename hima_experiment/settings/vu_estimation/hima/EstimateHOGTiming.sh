#!/bin/sh

rosrun visual_utility EstimateVUTiming \
_vu_estimator:=HOGDetector _use_cache:=false \
/data/mdesnoye/pedestrian/vu_estimation/hima/hog/hog_timing_full.txt \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0009.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0039.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0069.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0099.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0149.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0169.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0199.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0359.bmp \
/data/mdesnoye/hima/set_10/set10/stereo_cameras/left/img_0459.bmp \
/data/mdesnoye/hima/set_600/set600/stereo_cameras/left/img_0008.bmp \
/data/mdesnoye/hima/set_600/set600/stereo_cameras/left/img_0038.bmp \
/data/mdesnoye/hima/set_600/set600/stereo_cameras/left/img_0068.bmp \
/data/mdesnoye/hima/set_600/set600/stereo_cameras/left/img_0098.bmp \
/data/mdesnoye/hima/set_600/set600/stereo_cameras/left/img_0158.bmp \