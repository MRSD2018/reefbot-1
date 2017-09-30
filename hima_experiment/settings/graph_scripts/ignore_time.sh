#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/graphs/ignore_time/

mkdir -p ${OUTPUT_DIR}

src/PlotVUAccuracy.py \
    -i /data/mdesnoye/pedestrian/vu_estimation/eth/graphs/30step_newtiming/30step.stats \
    --hog_timing_file /data/mdesnoye/pedestrian/vu_estimation/eth/hog_cached/hog_cached_timing3.txt \
    --output_prefix ${OUTPUT_DIR}/ignore_time_eth \
    --group_names "Laplacian,Objectness,Haar Cascade,Low-res HOG,Motion,Spectral Saliency,Center Surround" \
    --valid_entries "[3]" \
    --trained_stats "/data/mdesnoye/pedestrian/vu_estimation/eth/integral_hog_cascade/graphs_30k/*.stats" \
    --ignore_time_stats "/data/mdesnoye/pedestrian/vu_estimation/eth/integral_hog_cascade/graphs_ignore_time_rebalance/*.stats" \