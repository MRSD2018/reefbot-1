#!/bin/bash

OUTPUT_DIR=/home/mdesnoye/data/hima/experiments/hima/saliency_entropy
INPUT_DIRS=(#/home/mdesnoye/data/hima/set_6/set6 \
/home/mdesnoye/data/hima/set_10/set10
#/home/mdesnoye/data/hima/set_19/set19
/home/mdesnoye/data/hima/set_22/set22
/home/mdesnoye/data/hima/set_400/set400
/home/mdesnoye/data/hima/set_600/set600
/home/mdesnoye/data/hima/set_9/set9
)
MODEL_FILE=/home/mdesnoye/src/fish/ros/cascade_parts_detector/src/voc_release4/VOC2009/person_final.mat

INPUT_DIR_FILE=${OUTPUT_DIR}/input_dirs.txt

mkdir -p ${OUTPUT_DIR}

rm -f ${INPUT_DIR_FILE}
for dir in ${INPUT_DIRS[*]}
do
  echo $dir >> ${INPUT_DIR_FILE}
done

src/RunHimaExperiment.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--annotations annotations.txt \
--frame_subset_rate 3 \
--frame_estimator HighRelativeEntropy \
--person_cascade \
--model_file ${MODEL_FILE} \
--person_thresh -3 \
--vu_estimator SpectralSaliency \
--morph_close_size 8 \
--gauss_sigma 30 \
--min_entropy "[0.05]" \
--frame_expansion "[2.0, 1.0, 1.5, 1.2, 1.7]" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log


src/RunHimaExperiment.py \
--input_dirs ${INPUT_DIR_FILE} \
--output_dir ${OUTPUT_DIR} \
--annotations annotations.txt \
--frame_subset_rate 3 \
--skip_vuexperiment \
--do_lowres_experiment \
--person_cascade \
--model_file ${MODEL_FILE} \
--person_thresh -3 \
--frac_framesize "[0.9, 0.4, 0.5, 0.6, 0.7, 0.8]" \
__log:=${OUTPUT_DIR}/rosout_lowres.log > ${OUTPUT_DIR}/stdout_lowres.log 2> ${OUTPUT_DIR}/stderr_lowres.log
