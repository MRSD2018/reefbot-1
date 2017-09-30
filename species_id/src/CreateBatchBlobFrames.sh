#!/bin/sh

# Will create jpegs for each blob in a batch Uses the CreateBlobFrames program
# internally.

# Usage: CreateBatchBlobFrames.sh <outputDir> <inputFileExpr> <CreateBlobFrames options> 

# ex. CreateBatchBlobFrames.sh results/thisRun "path/to/input/*.blob" "--frame_width 500 --frame_height 500"

if [ $# -lt 2 ]
then
    echo "Usage: CreateBatchBlobFrames.sh <outputDir> <inputFileExpr> <CreateBlobFrames options>"
    exit 2
fi

output_dir=`echo $1`
shift
input_expr=`echo $1 | sed 's/\"//g'`
shift

mkdir -p ${output_dir}

for input_file in `ls ${input_expr}`
do
  bin/CreateBlobFrames "$@" --output_dir ${output_dir} --input ${input_file}
done