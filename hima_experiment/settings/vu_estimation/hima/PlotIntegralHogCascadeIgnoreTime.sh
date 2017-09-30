#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog_cascade/graphs_ignore_time
INPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog_cascade_ignore_time/
FP_SCALES=( 0.5 1.0 2.0 5.0 0.2 10.0 )
DATASETS=( set6 set9 set10 set19 set22 set400 set600 )

rm -rf ${OUTPUT_DIR}
mkdir -p ${OUTPUT_DIR}

for dataset in ${DATASETS[*]}; do
for fpScale in ${FP_SCALES[*]}; do
    src/CascadeAccuracy.py \
        --frame_subset_rate 30 \
        --output_data ${OUTPUT_DIR}/IntegralHOGCascadeHimaIgnoreTime_${dataset}_${fpScale}.stats \
        --estimator_regex "vu_cascade_IntegralHOGCascadeHimaIgnoreTime_${dataset}_([0-9\.]+)_[0-9A-Za-z]+\.bag" \
        --valid_vutype ${fpScale} \
        --valid_dataset "(set[0-9]+)" \
        --root_results_dir ${INPUT_DIR} \
        --bag_regex "vu_cascade_.*\.bag" \
        __log:=${OUTPUT_DIR}/rosout_${model}.log > ${OUTPUT_DIR}/stdout_${model}.log 2> ${OUTPUT_DIR}/stderr_${model}.log

done
done


