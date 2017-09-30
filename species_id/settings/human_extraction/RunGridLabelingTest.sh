#!/bin/bash

EXPERIMENT_DIR=/data/mdesnoye/fish/experiments/human_extraction/grid_labeling_test/

mkdir -p ${EXPERIMENT_DIR}

python -m pdb src/ExtractGridClassification.py \
--blob_dir /data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/xmlBlobs \
--experiment_dir ${EXPERIMENT_DIR} \
--output_prefix test \
--image_list /data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/humanLabelsTest.txt \
--neg_blob_list /data/mdesnoye/fish/tank_videos/extracted_fish_newframeid/20110102/negListTest.txt \
--num_neg_blobs 5.0 \
--shape_dict_filename /data/mdesnoye/fish/experiments/extraction081011/20110102/osurf.dict \
--use_opponent_surf \
--do_testing \
__log:=${EXPERIMENT_DIR}/rosout.log #> ${EXPERIMENT_DIR}/stdout.log 2> ${EXPERIMENT_DIR}/stderr.log