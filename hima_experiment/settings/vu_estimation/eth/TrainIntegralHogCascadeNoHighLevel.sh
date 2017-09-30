#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/eth/integral_cascade_no_high_level_equal_cost
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

TIME_SCALES=( 0.5 1.0 2.0 0.05 5.0 0.1 0.2 0.7 1.5 10.0 )
#TIME_SCALES=( 0.5 1.0 2.0 )

for dataset in ${DATASETS[*]}; do
  for timeScale in ${TIME_SCALES[*]}; do 
    INPUT_BAGS=`ls -1 ${BAG_DIR}/*.bag | sed "/.*${dataset}\.bag/d"`
    ${TRAIN_BIN} \
        --output ${OUTPUT_DIR}/cascade_${dataset}_time_${timeScale}.xml \
        __log:=${OUTPUT_DIR}/rosout_${dataset}_${timeScale}.log \
        --miss_cost 1.0 \
        --false_pos_cost 1.0 \
	      --nsamples 25000 \
        --time_cost_per_error ${timeScale} \
        --integral_hist_time ${TIMING_DIR}/integral_hist.txt \
        --fill_block_cache_time ${TIMING_DIR}/fill_cache.txt \
        --svm_eval_time ${TIMING_DIR}/svm_time.txt \
        --true_hog_time ${TIMING_DIR}/free_hog_timing.txt \
        ${DETECTOR_LIST} \
        ${INPUT_BAGS} \
        > ${OUTPUT_DIR}/stdout_${dataset}_${timeScale}.log 2> ${OUTPUT_DIR}/stderr_${dataset}_${timeScale}.log
	
  done
done

#
