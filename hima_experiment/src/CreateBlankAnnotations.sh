#!/bin/bash
#
# Creates an annotations.txt file where each line increments the frame number
# by one. There is one line per .png file in the directory
#
# usage: CreateBlankAnnotation.sh dir

curTime=734801.709940
curIdx=0

rm annotations.txt

for f in $1/*.png; do
    echo $curTime $curIdx -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 >> annotations.txt
    curTime=$(echo "scale=6; $curTime+(1/30)" | bc)
    let "curIdx += 1"
done