#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/integral_hog_cascade/graphs_30k
INPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/integral_hog_cascade/cross_30k
MODELS=( 0.05 0.1 0.2 0.5 0.7 1.0 1.5 2.0 5.0 10.0 )
DATASETS=( Crossing Linthescher Lowenplatz set00 set01 set02 )

mkdir -p ${OUTPUT_DIR}

for dataset in ${DATASETS[*]}; do
for model in ${MODELS[*]}; do
    src/CascadeAccuracy.py \
        --frame_subset_rate 30 \
        --output_data ${OUTPUT_DIR}/IntegralHOGCascadeETH_${dataset}_${model}.stats \
        --estimator_regex "vu_cascade_IntegralHOGCascadeETH_${dataset}_([0-9\.]+)_[0-9A-Za-z]+\.bag" \
        --valid_vutype ${model} \
        --valid_dataset "((Crossing)|(Linthescher)|(Lowenplatz)|(set0*))" \
        --root_results_dir ${INPUT_DIR} \
        --bag_regex "vu_cascade_.*\.bag" \
        __log:=${OUTPUT_DIR}/rosout_${model}.log > ${OUTPUT_DIR}/stdout_${model}.log 2> ${OUTPUT_DIR}/stderr_${model}.log

done
done


