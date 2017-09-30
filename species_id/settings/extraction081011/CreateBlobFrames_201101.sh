#!/bin/sh
# Configuration script that will create jpegs of all the blobs

# Configuration options
INPUT_FILES=\"/data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blobs/*.blob\"
WEB_OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blob_thumbs_web/
WEB_PARAMS="--frame_width 500 --frame_height 500"

mkdir -p ${WEB_OUTPUT_DIR}

# Script to do the actual extraction
bin/CreateBatchBlobFrames.sh ${WEB_OUTPUT_DIR} ${INPUT_FILES} ${WEB_PARAMS} > ${WEB_OUTPUT_DIR}/frame_creation.log

CLUSTER_OUTPUT_DIR=/data/mdesnoye/fish/tank_videos/extracted_fish/20110102/blob_thumb_cluster/
CLUSTER_PARAMS="--frame_width 2.0 --frame_height 2.0 --nouse_static_size --nodraw_rect"

mkdir -p ${CLUSTER_OUTPUT_DIR}

# Script to do the actual extraction
bin/CreateBatchBlobFrames.sh ${CLUSTER_OUTPUT_DIR} ${INPUT_FILES} ${CLUSTER_PARAMS} > ${CLUSTER_OUTPUT_DIR}/frame_creation.log