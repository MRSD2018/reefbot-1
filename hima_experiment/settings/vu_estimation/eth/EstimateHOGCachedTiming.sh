#!/bin/sh

rosrun visual_utility EstimateVUTiming \
_vu_estimator:=HOGDetector _use_cache:=true \
_samples:=1000 \
/data/mdesnoye/pedestrian/vu_estimation/eth/hog_cached/hog_cached_timing3.txt \
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
    /data/mdesnoye/pedestrian/eth/hima/Lowenplatz/left/img_0198.png
