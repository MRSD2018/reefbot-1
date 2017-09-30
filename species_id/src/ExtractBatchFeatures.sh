#!/bin/sh

# Script that will extract features from a batch of files and
# dump them to another directory. Uses the ExtractFeatures program
# internally.

# Usage: ExtractBatchFeatures.sh <outputDir> <inputFileExpr> <suffix> <ExtractFeatures options> 

# ex. ExtractBatchFeatures.sh results/thisRun "path/to/input/*.blob" sift "--fast_detector --sift_descriptor"

if [ $# -lt 3 ]
then
    echo "Usage: ExtractBatchFeatures.sh <outputDir> <inputFileExpr> <suffix> <ExtractFeatures options>"
    exit 2
fi

output_dir=`echo $1`
shift
input_expr=`echo $1 | sed 's/\"//g'`
shift
suffix=`echo $1`
shift

mkdir -p ${output_dir}

for input_file in `ls ${input_expr}`
do
  base_file=`basename ${input_file} | sed 's/\.[a-zA-Z]*$//g'`
  output_file=${output_dir}/${base_file}.${suffix}
  bin/ExtractFeatures "$@" --output ${output_file} --input ${input_file}
done