#!/bin/sh

EXPERIMENT_DIR=/data/mdesnoye/fish/experiments/extraction081011/20110102/test_experiment_geo

mkdir -p ${EXPERIMENT_DIR}


python -m pdb bin/RunSpeciesLabelingExperiment.py \
--blob_dir /data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blobs \
--experiment_dir ${EXPERIMENT_DIR} \
--image_list /home/mdesnoye/src/fish/ros/species_id/settings/extraction081011/imageLabelsTest.txt \
--use_surf_descriptor \
--shape_dict_filename /data/mdesnoye/fish/experiments/extraction081011/20110102/surf.dict \
--shape_weights="[1.0]" \
--video_regexp="(([0-9][0-9]-){3})" \
--video_ids="[None]" \
--do_geo_rerank \
__log:=${EXPERIMENT_DIR}/rosout.log \
--frac_test=-1 #> ${EXPERIMENT_DIR}/stdout.txt


if 0
then
--blob_prefix=/data/mdesnoye/fish/extractedFish/extractedFish051010_blob/ --color_dict_filename=/data/mdesnoye/fish/extractedFish/blob090710/hsv.dict --color_converter=CV_BGR2HSV --color_frac=0.1 --shape_dict_filename=/data/mdesnoye/fish/extractedFish/blob090710/surf.dict --surf_detector=True --surf_hessian_threshold=400 --surf_octaves=3 --surf_octave_layers=4 --surf_extended=False --surf_descriptor=True --sift_descriptor=False --shape_weight=1.0 --min_shape_val=0.01 --min_color_val=0.01 --min_score=0.01 --output=test.index --input=/home/mdesnoye/src/fish/ros/species_id/settings/blob090710/test_experiment/
fi

