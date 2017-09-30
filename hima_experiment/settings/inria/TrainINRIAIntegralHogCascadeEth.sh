#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/inria/integral_cascade_eth
POS_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/hogPosFullfilename.lst
NEG_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/hogNegFramesFullfilenameETH.lst
DETECTOR_LIST=/data/mdesnoye/pedestrian/inria/all_detectors.lst
TIMING_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/timing/
TRAIN_BIN=`rospack find hog_detector`/bin/TrainCascadeUsingImages

rm -rf ${OUTPUT_DIR}
mkdir -p ${OUTPUT_DIR}

# false_pos_cost is nPositiveExamples / totalExamples
# 0.054
# new is (p(r) - p(gr)) / (1-p(gr))

#BASE_TIME_COST=0.000813
BASE_TIME_COST=0.000422
TIME_SCALES=( 0.05 0.1 0.2 0.5 0.7 1.0 1.5 2.0 5.0 10.0 )

for timeScale in ${TIME_SCALES[*]}; do 
  TIME_COST=$(echo "scale=10; ${BASE_TIME_COST}*${timeScale}" | bc)
  ${TRAIN_BIN} \
    --output ${OUTPUT_DIR}/cascade_time_${timeScale}.xml \
    __log:=${OUTPUT_DIR}/rosout_${timeScale}.log \
    --miss_cost 1.0 \
    --false_pos_cost 0.00013062 \
    --time_cost_per_error ${TIME_COST} \
    --integral_hist_time ${TIMING_DIR}/integral_hist.txt \
    --fill_block_cache_time ${TIMING_DIR}/fill_cache.txt \
    --svm_eval_time ${TIMING_DIR}/svm_time.txt \
    --true_hog_time ${TIMING_DIR}/hog_cached_timing2.txt \
    ${POS_LIST} \
    ${NEG_LIST} \
    ${DETECTOR_LIST} \
    > ${OUTPUT_DIR}/stdout_${timeScale}.log 2> ${OUTPUT_DIR}/stderr_${timeScale}.log
done

#
