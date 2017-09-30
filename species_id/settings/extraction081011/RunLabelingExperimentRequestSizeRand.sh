#!/bin/sh

EXPERIMENT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/mixing_experiment_bboxsize_rand

mkdir -p ${EXPERIMENT_DIR}


bin/RunSpeciesLabelingExperiment.py \
--blob_dir /data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blobs \
--experiment_dir ${EXPERIMENT_DIR} \
--image_list /home/mdesnoye/src/fish/ros/species_id/settings/extraction081011/imageLabels.txt \
--use_opponent_surf \
--shape_dict_filename /data/mdesnoye/fish/experiments/extraction081011/20110102/osurf.dict \
--color_dict_filename  /data/mdesnoye/fish/experiments/extraction081011/20110102/hsv.dict \
--shape_weights="[1.0]" \
--min_blob_size="[20]" \
--query_box_size="[0.5, 1.0, 2.0, 4.0, 8.0]" \
--video_regexp="(([0-9][0-9]-){3})" \
--video_ids="[None]" \
__log:=${EXPERIMENT_DIR}/rosout.log > ${EXPERIMENT_DIR}/stdout.txt
