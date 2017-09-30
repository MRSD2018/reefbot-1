#!/bin/sh

EXPERIMENT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/mixing_experiment_bboxsize

mkdir ${EXPERIMENT_DIR}


bin/RunSpeciesLabelingExperiment.py --blob_dir /data/mdesnoye/fish/extractedFish/extractedFish051010_blob/ --experiment_dir ${EXPERIMENT_DIR} --image_list /home/mdesnoye/src/fish/mturk/blob090710-imagelables.csv --use_opponent_surf --shape_dict_filename /data/mdesnoye/fish/extractedFish/blob090710/osurf.dict --shape_weights="[1.0]" --min_blob_size="[60]" --query_box_size="[0.5, 1.0, 2.0, 4.0, 8.0]" __log:=${EXPERIMENT_DIR}/rosout.log > ${EXPERIMENT_DIR}/stdout.txt