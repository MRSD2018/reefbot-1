#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/integral_hog_cascade/graphs_ignore_time_rebalance
INPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/integral_cascade_ignore_time_rebalance
DATASETS=( Crossing Linthescher Lowenplatz set00 set01 set02 )
FP_SCALES=( 0.5 1.0 2.0 5.0 0.2 10.0 )

mkdir -p ${OUTPUT_DIR}
rm ${OUTPUT_DIR}/*

for fpScale in ${FP_SCALES[*]}; do
for dataset in ${DATASETS[*]}; do
    src/CascadeAccuracy.py \
        --frame_subset_rate 30 \
        --output_data ${OUTPUT_DIR}/IntegralHOGCascadeETHIgnoreTime_${dataset}_${fpScale}.stats \
        --estimator_regex "vu_cascade_IntegralHOGCascadeETHIgnoreTime_${dataset}_([0-9A-Za-z]+)_[0-9A-Za-z]+\.bag" \
        --valid_vutype ${fpScale} \
        --valid_dataset "((Crossing)|(Linthescher)|(Lowenplatz)|(set0*))" \
        --root_results_dir ${INPUT_DIR} \
        --bag_regex "vu_cascade_.*\.bag" \
        __log:=${OUTPUT_DIR}/rosout_${dataset}.log > ${OUTPUT_DIR}/stdout_${dataset}.log 2> ${OUTPUT_DIR}/stderr_${dataset}.log

done
done


