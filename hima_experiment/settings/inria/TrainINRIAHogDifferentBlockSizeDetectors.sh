#!/bin/bash

# Does the training for 64x128 models

OUTPUT_DIR=/data/mdesnoye/pedestrian/inria/blocksize_detectors
POS_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/posFullfilename.lst
NEG_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/negFullfilename.lst
OUTPUT_PREFIX=inria_people_64x128_blocksize

TRAIN_BIN=`rospack find hog_detector`/bin/integral_hog_trainer

mkdir -p ${OUTPUT_DIR}

for blockWidth in {16..64..8}; do
    for blockHeight in {16..128..8}; do
        OUTPUT_FILE=${OUTPUT_PREFIX}-${blockWidth}-${blockHeight}
        ${TRAIN_BIN} \
            --blockW ${blockWidth} \
            --blockH ${blockHeight} \
            --cellW `expr ${blockWidth} / 2` \
            --cellH `expr ${blockHeight} / 2` \
            --output ${OUTPUT_DIR}/${OUTPUT_FILE}.xml \
            --checkFrac 0.05 \
            --sampleNegs \
            __log:=${OUTPUT_DIR}/${OUTPUT_FILE}_rosout.log \
            ${POS_LIST} \
            ${NEG_LIST} \
            > ${OUTPUT_DIR}/${OUTPUT_FILE}_stdout.log 2> ${OUTPUT_DIR}/${OUTPUT_FILE}_stderr.log
    done
done