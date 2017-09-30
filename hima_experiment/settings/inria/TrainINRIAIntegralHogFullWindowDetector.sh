#!/bin/bash

# Does the training for 64x128 models

OUTPUT_DIR=/data/mdesnoye/pedestrian/inria/integral_detector_cache
POS_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/posFullfilename.lst
NEG_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/negFullfilename.lst
POS_TEST=/data/mdesnoye/pedestrian/inria/INRIAPerson/test_64x128_H96/posFullFilename.lst
NEG_TEST=/data/mdesnoye/pedestrian/inria/INRIAPerson/test_64x128_H96/negFullFilename.lst
OUTPUT_PREFIX=inria_people_64x128_fullwindow

TRAIN_BIN=`rospack find hog_detector`/bin/integral_hog_trainer

mkdir -p ${OUTPUT_DIR}

echo ${TRAIN_BIN} \
    --output ${OUTPUT_DIR}/${OUTPUT_PREFIX}.xml.gz \
    --sampleNegs \
    __log:=${OUTPUT_DIR}/rosout.log \
    --sampleNegs \
    --posTest ${POS_TEST} \
    --negTest ${NEG_TEST} \
    ${POS_LIST} \
    ${NEG_LIST} \
    #> ${OUTPUT_DIR}/stdout.log 2> ${OUTPUT_DIR}/stderr.log
