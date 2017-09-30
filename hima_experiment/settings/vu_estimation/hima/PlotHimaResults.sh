#!/bin/sh
OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/graphs/30step/

mkdir -p ${OUTPUT_DIR}

src/VUAccuracy.py \
--frame_subset_rate 30 \
--skip_scaling \
--skip_resampling \
--hog_timing_file /data/mdesnoye/pedestrian/vu_estimation/hima/hog_cached/hog_cached_timing.txt \
--valid_dataset "(set[0-9]+)" \
--valid_vutype "((LABMotion)|(HaarFullBody)|(Laplacian5Entropy)|(Objectness)|(SpectralSaliency)|(HOGSmallDetector)|(CenterSurround_1.0))" \
--output_data ${OUTPUT_DIR}/30step.stats \
--hog_name HOGCachedDetector \
--root_results_dir /data/mdesnoye/pedestrian/vu_estimation/hima/ \
--cache_dir /data/mdesnoye/pedestrian/vu_estimation/hima/cache \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
