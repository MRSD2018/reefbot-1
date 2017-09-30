#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/graphs/different_vuestimators/

mkdir -p ${OUTPUT_DIR}

src/PlotVUAccuracy.py \
    -i /data/mdesnoye/pedestrian/vu_estimation/eth/graphs/30step_newtiming/30step.stats \
    --hog_timing_file /data/mdesnoye/pedestrian/vu_estimation/eth/hog_cached/hog_cached_timing3.txt \
    --output_prefix ${OUTPUT_DIR}/different_vuestimators_eth \
    --group_names "Laplacian,Objectness,Haar Cascade,Low-res HOG,Motion,Spectral Saliency,Center Surround"