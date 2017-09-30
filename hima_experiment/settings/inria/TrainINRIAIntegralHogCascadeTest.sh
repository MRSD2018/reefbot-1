#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/inria/integral_cascade_test
POS_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/hogPosFullfilenameTest.lst
NEG_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/hogNegFramesFullfilenameTest.lst
DETECTOR_LIST=/data/mdesnoye/pedestrian/inria/all_detectors_test.lst
TIMING_DIR=/data/mdesnoye/pedestrian/vu_estimation/eth/timing/
TRAIN_BIN=`rospack find hog_detector`/bin/TrainCascadeUsingImages

mkdir -p ${OUTPUT_DIR}


echo ${TRAIN_BIN} \
    --output ${OUTPUT_DIR}/test_cascade.xml \
    __log:=${OUTPUT_DIR}/rosout.log \
    --miss_cost 0.1 \
    --false_pos_cost 0.00013062 \
    --time_cost_per_error 0.000813 \
    --integral_hist_time ${TIMING_DIR}/integral_hist.txt \
    --fill_block_cache_time ${TIMING_DIR}/fill_cache.txt \
    --svm_eval_time ${TIMING_DIR}/svm_time.txt \
    --true_hog_time ${TIMING_DIR}/hog_cached_timing.txt \
    ${POS_LIST} \
    ${NEG_LIST} \
    ${DETECTOR_LIST} \
    #> ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log