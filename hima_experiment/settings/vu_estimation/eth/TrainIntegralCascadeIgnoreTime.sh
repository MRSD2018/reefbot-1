#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/integral_cascade_ignore_time_range_fprebalance
DETECTOR_LIST=/data/mdesnoye/pedestrian/inria/all_detectors.lst
TIMING_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/timing/
TRAIN_BIN=`rospack find hima_experiment`/bin/TrainCascadeUsingBag

BAG_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/hog_cached

DATASETS=( Crossing set00 set01 set02 Linthescher Lowenplatz )

rm -rf ${OUTPUT_DIR}
mkdir -p ${OUTPUT_DIR}

# false_pos_cost is nPositiveExamples / totalExamples
# 0.054
# new is (p(r) - p(gr)) / (1-p(gr))
# 0.00013062 

BASE_FP_COST=0.00013062
FP_SCALES=( 0.5 1.0 2.0 5.0 0.2 10.0 )

for dataset in ${DATASETS[*]}; do
    for fpScale in ${FP_SCALES[*]}; do
        FP_COST=$(echo "scale=10; ${BASE_FP_COST}*${fpScale}" | bc)
        INPUT_BAGS=`ls -1 ${BAG_DIR}/*.bag | sed "/.*${dataset}\.bag/d"`
        ${TRAIN_BIN} \
            --output ${OUTPUT_DIR}/cascade_${dataset}_ignore_time_fp_${fpScale}.xml \
            __log:=${OUTPUT_DIR}/rosout_${dataset}_ignore_time_fp_${fpScale}.log \
            --miss_cost 1.0 \
            --false_pos_cost ${FP_COST} \
            --nsamples 30000 \
            --time_cost_per_error 1e8 \
            --integral_hist_time ${TIMING_DIR}/integral_hist.txt \
            --fill_block_cache_time ${TIMING_DIR}/fill_cache.txt \
            --svm_eval_time ${TIMING_DIR}/svm_time.txt \
            --true_hog_time ${TIMING_DIR}/hog_cached_timing3.txt \
            ${DETECTOR_LIST} \
            ${INPUT_BAGS} \
            > ${OUTPUT_DIR}/stdout_${dataset}_ignore_time_fp_${fpScale}.log 2> ${OUTPUT_DIR}/stderr_${dataset}_ignore_time_fp_${fpScale}.log
    done
	
  done

#
