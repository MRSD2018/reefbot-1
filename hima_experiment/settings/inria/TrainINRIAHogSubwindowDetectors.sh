#!/bin/bash

# Does the training for 64x128 models

OUTPUT_DIR=/data/mdesnoye/pedestrian/inria/subwindow_detectors
POS_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/posFullfilename.lst
NEG_LIST=/data/mdesnoye/pedestrian/inria/INRIAPerson/train_64x128_H96/negFullfilename.lst
OUTPUT_PREFIX=inria_people_64x128

TRAIN_BIN=`rospack find hog_detector`/bin/integral_hog_trainer

mkdir -p ${OUTPUT_DIR}

for width in {16..64..16}; do
    maxWidth=`expr 64 - ${width}`
    for height in {16..128..16}; do
        maxHeight=`expr 128 - ${height}`
        for x_offset in `seq 0 16 ${maxWidth}`; do
            for y_offset in `seq 0 16 ${maxHeight}`; do
                OUTPUT_FILE=${OUTPUT_PREFIX}-${width}-${height}-${x_offset}-${y_offset}
                ${TRAIN_BIN} \
                    --subWinX ${x_offset} \
                    --subWinY ${y_offset} \
                    --subWinW ${width} \
                    --subWinH ${height} \
                    --output ${OUTPUT_DIR}/${OUTPUT_FILE}.xml \
                    --checkFrac 0.05 \
                    --sampleNegs \
                    __log:=${OUTPUT_DIR}/${OUTPUT_FILE}_rosout.log \
                    ${POS_LIST} \
                    ${NEG_LIST} \
                    > ${OUTPUT_DIR}/${OUTPUT_FILE}_stdout.log 2> ${OUTPUT_DIR}/${OUTPUT_FILE}_stderr.log
            done
        done
    done
done