#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/inria/integral_cascade_hima
POS_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/hogPosFullfilename.lst
NEG_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/hogNegFramesFullfilenameHima.lst
DETECTOR_LIST=/data/mdesnoye/pedestrian/inria/all_detectors.lst
TIMING_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/timing/
TRAIN_BIN=`rospack find hog_detector`/bin/TrainCascadeUsingImages

mkdir -p ${OUTPUT_DIR}

# false_pos_cost is nPositiveExamples / totalExamples

${TRAIN_BIN} \
    --output ${OUTPUT_DIR}/cascade_time0.3.xml \
    __log:=${OUTPUT_DIR}/rosout.log \
    --time_budget 0.3 \
    --miss_cost 1.0 \
    --false_pos_cost 0.011 \
    --integral_hist_time ${TIMING_DIR}/integral_hist.txt \
    --fill_block_cache_time ${TIMING_DIR}/fill_cache.txt \
    --svm_eval_time ${TIMING_DIR}/svm_time.txt \
    --true_hog_time ${TIMING_DIR}/hog_cached_timing.txt \
    ${POS_LIST} \
    ${NEG_LIST} \
    ${DETECTOR_LIST} \
    > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
