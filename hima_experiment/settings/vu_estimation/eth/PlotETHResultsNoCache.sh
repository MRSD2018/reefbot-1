#!/bin/sh
OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/graphs/no_cache/

mkdir -p ${OUTPUT_DIR}

src/VUAccuracy.py \
--frame_subset_rate 30 \
--skip_scaling \
--skip_resampling \
--hog_timing_file /data/mdesnoye/pedestrian/vu_estimation/eth/hog/hog_timing_full.txt \
--valid_dataset "((Crossing)|(Linthescher)|(Lowenplatz)|(set0*))" \
--valid_vutype "((LABMotion)|(HaarFullBody)|(Laplacian5Entropy)|(Objectness$)|(SpectralSaliency)|(HOGSmallDetector)|(CenterSurround_1.0))" \
--output_data ${OUTPUT_DIR}/NoCache.stats \
--hog_name HOGCachedDetector \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
