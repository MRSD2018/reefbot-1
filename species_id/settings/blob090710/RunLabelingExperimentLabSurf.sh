#!/bin/sh

EXPERIMENT_DIR=/data/mdesnoye/fish/extractedFish/blob090710/mixing_experiment_surf_lab

mkdir ${EXPERIMENT_DIR}


bin/RunSpeciesLabelingExperiment.py --blob_dir /data/mdesnoye/fish/extractedFish/extractedFish051010_blob/ --experiment_dir ${EXPERIMENT_DIR} --image_list /home/mdesnoye/src/fish/mturk/blob090710-imagelables.csv --color_dict_filename=/data/mdesnoye/fish/extractedFish/blob090710/lab.dict --color_converter=CV_BGR2LAB --use_surf_descriptor --shape_dict_filename /data/mdesnoye/fish/extractedFish/blob090710/surf.dict --shape_weights="[x * 0.05 for x in range(11)]" --min_blob_size="range(80, 150, 60)" __log:=${EXPERIMENT_DIR}/rosout.log > ${EXPERIMENT_DIR}/stdout.txt