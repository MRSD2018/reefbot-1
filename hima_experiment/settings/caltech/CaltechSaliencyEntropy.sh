#!/bin/bash

OUTPUT_DIR=/data/mdesnoye/hima/experiments/caltech/saliency_entropy
INPUT_DIRS=(#/data/mdesnoye/pedestrian/caltech/hima/set06/V007
#/data/mdesnoye/pedestrian/caltech/hima/set06/V003
#/data/mdesnoye/pedestrian/caltech/hima/set06/V002
#/data/mdesnoye/pedestrian/caltech/hima/set06/V004
#/data/mdesnoye/pedestrian/caltech/hima/set06/V010
#/data/mdesnoye/pedestrian/caltech/hima/set06/V001
#/data/mdesnoye/pedestrian/caltech/hima/set06/V018
#/data/mdesnoye/pedestrian/caltech/hima/set06/V014
#/data/mdesnoye/pedestrian/caltech/hima/set06/V016
#/data/mdesnoye/pedestrian/caltech/hima/set06/V009
/data/mdesnoye/pedestrian/caltech/hima/set06/V011
/data/mdesnoye/pedestrian/caltech/hima/set06/V006
/data/mdesnoye/pedestrian/caltech/hima/set06/V008
/data/mdesnoye/pedestrian/caltech/hima/set06/V012
/data/mdesnoye/pedestrian/caltech/hima/set06/V017
/data/mdesnoye/pedestrian/caltech/hima/set06/V005
/data/mdesnoye/pedestrian/caltech/hima/set06/V015
/data/mdesnoye/pedestrian/caltech/hima/set06/V000
/data/mdesnoye/pedestrian/caltech/hima/set06/V013
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
--left_image_dir frames \
--image_file_string "I%05i.jpg" \
--frame_subset_rate 15 \
--frame_estimator HighRelativeEntropy \
--person_cascade \
--model_file ${MODEL_FILE} \
--person_thresh -3 \
--vu_estimator SpectralSaliency \
--morph_close_size 4 \
--gauss_sigma 30 \
--min_entropy "[0.05]" \
--frame_expansion "[1.2, 1.5, 1.0]" \
__log:=${OUTPUT_DIR}/rosout.log > ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
