#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog_cascade/graphs
INPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog_cascade/
MODELS=( 0.05 0.1 0.2 0.5 0.7 1.0 1.5 2.0 5.0 10.0 )
DATASETS=( set6 set9 set10 set19 set22 set400 set600 )

rm -rf ${OUTPUT_DIR}
mkdir -p ${OUTPUT_DIR}

for dataset in ${DATASETS[*]}; do
for model in ${MODELS[*]}; do
    src/CascadeAccuracy.py \
        --frame_subset_rate 30 \
        --output_data ${OUTPUT_DIR}/IntegralHOGCascadeHima_${dataset}_${model}.stats \
        --estimator_regex "vu_cascade_IntegralHOGCascadeHima_${dataset}_([0-9\.]+)_[0-9A-Za-z]+\.bag" \
        --valid_vutype ${model} \
        --valid_dataset "(set[0-9]+)" \
        --root_results_dir ${INPUT_DIR} \
        --bag_regex "vu_cascade_.*\.bag" \
        __log:=${OUTPUT_DIR}/rosout_${model}.log > ${OUTPUT_DIR}/stdout_${model}.log 2> ${OUTPUT_DIR}/stderr_${model}.log

done
done


