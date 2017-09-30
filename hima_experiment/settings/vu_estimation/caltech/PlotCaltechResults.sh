#!/bin/sh
OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/caltech/graphs/set06/30step/

mkdir -p ${OUTPUT_DIR}

src/VUAccuracy.py \
--frame_subset_rate 60 \
--skip_scaling \
--hog_timing_file /data/mdesnoye/pedestrian/vu_estimation/caltech/hog_cached_timing.txt \
--valid_dataset "(V[0-9]+)" \
--valid_vutype "((LABMotion)|(HaarFullBody)|(Laplacian5Entropy)|(Objectness)|(SpectralSaliency)|(CenterSurround_1.0_chisq))" \
--output_data ${OUTPUT_DIR}/30step.stats \
--hog_name HOGCachedDetector \
--root_results_dir /data/mdesnoye/pedestrian/vu_estimation/caltech/set06 \
--cache_dir /data/mdesnoye/pedestrian/vu_estimation/caltech/cache \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log