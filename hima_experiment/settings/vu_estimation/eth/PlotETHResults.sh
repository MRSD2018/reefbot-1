#!/bin/sh
OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/graphs/30step_newtiming/

mkdir -p ${OUTPUT_DIR}

src/VUAccuracy.py \
--frame_subset_rate 30 \
--skip_scaling \
--skip_resampling \
--hog_timing_file /data/mdesnoye/pedestrian/vu_estimation/eth/hog_cached/hog_cached_timing3.txt \
--valid_dataset "((Crossing)|(Linthescher)|(Lowenplatz)|(set0*))" \
--valid_vutype "((LABMotion)|(HaarFullBody)|(Laplacian5Entropy)|(Objectness$)|(SpectralSaliency)|(HOGSmallDetector)|(CenterSurround_1.0))" \
--output_data ${OUTPUT_DIR}/30step.stats \
--hog_name HOGCachedDetector \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
