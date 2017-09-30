#!/bin/sh

EXPERIMENT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/mixing_experiment_uniform_testset

mkdir ${EXPERIMENT_DIR}


bin/RunSpeciesLabelingExperiment.py --blob_dir /data/mdesnoye/fish/extractedFish/extractedFish051010_blob/ --experiment_dir ${EXPERIMENT_DIR} --image_list /home/mdesnoye/src/fish/mturk/blob090710-imagelables.csv --use_surf_descriptor --shape_dict_filename /data/mdesnoye/fish/extractedFish/blob090710/surf.dict --shape_weights="[x * 0.1 for x in range(11)]" --video_ids="[None]" __log:=${EXPERIMENT_DIR}/rosout.log --frac_test=0.2 > ${EXPERIMENT_DIR}/stdout.txt