#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/pedestrian/hima/integral_cascade
DETECTOR_LIST=/data/mdesnoye/pedestrian/inria/all_detectors.lst
TIMING_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/integral_hog/timing/
TRAIN_BIN=`rospack find hima_experiment`/bin/TrainCascadeUsingBag

BAG_DIR=/data/mdesnoye/pedestrian/vu_estimation/hima/hog_cached

DATASETS=( set6 set9 set10 set19 set22 set400 set600 )

rm -rf ${OUTPUT_DIR}
mkdir -p ${OUTPUT_DIR}

# false_pos_cost is nPositiveExamples / totalExamples
# 0.054
# new is (p(r) - p(gr)) / (1-p(gr))
# where h is the vu estimator, r is the hog and g is the ground truth

#BASE_TIME_COST=0.000813
BASE_TIME_COST=0.000422
TIME_SCALES=( 0.5 1.0 2.0 0.05 5.0 0.1 0.2 0.7 1.5 10.0 )
#TIME_SCALES=( 0.5 1.0 2.0 )

for dataset in ${DATASETS[*]}; do
  for timeScale in ${TIME_SCALES[*]}; do 
    TIME_COST=$(echo "scale=10; ${BASE_TIME_COST}*${timeScale}" | bc)
    INPUT_BAGS=`ls -1 ${BAG_DIR}/*.bag | sed "/.*${dataset}\.bag/d"`
    ${TRAIN_BIN} \
        --output ${OUTPUT_DIR}/cascade_${dataset}_time_${timeScale}.xml \
        __log:=${OUTPUT_DIR}/rosout_${dataset}_${timeScale}.log \
        --miss_cost 1.0 \
        --false_pos_cost 0.000070356 \
	      --nsamples 30000 \
        --time_cost_per_error ${timeScale} \
        --integral_hist_time ${TIMING_DIR}/integral_hist.txt \
        --fill_block_cache_time ${TIMING_DIR}/fill_cache.txt \
        --svm_eval_time ${TIMING_DIR}/svm_time.txt \
        --true_hog_time ${TIMING_DIR}/hog_cached_timing.txt \
        ${DETECTOR_LIST} \
        ${INPUT_BAGS} \
        > ${OUTPUT_DIR}/stdout_${dataset}_${timeScale}.log 2> ${OUTPUT_DIR}/stderr_${dataset}_${timeScale}.log
	
  done
done

#
