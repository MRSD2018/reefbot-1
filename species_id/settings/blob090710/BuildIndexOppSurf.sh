#!/bin/sh

EXPERIMENT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/index_build_osurf_050511

mkdir ${EXPERIMENT_DIR}


bin/RunSpeciesLabelingExperiment.py --blob_dir /data/mdesnoye/fish/extractedFish/extractedFish051010_blob/ --experiment_dir ${EXPERIMENT_DIR} --image_list /home/mdesnoye/src/fish/mturk/blob090710-imagelables.csv --use_opponent_surf --shape_dict_filename /data/mdesnoye/fish/extractedFish/blob090710/osurf.dict --shape_weights="[1.0]" --min_blob_size="[80]" __log:=${EXPERIMENT_DIR}/rosout.log --save_index #> ${EXPERIMENT_DIR}/stdout.txt
