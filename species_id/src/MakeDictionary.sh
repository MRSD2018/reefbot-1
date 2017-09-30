#!/bin/sh

# Creates a dictionary file using desc2dict.py. Run desc2dict.py -h
# for the options

# Usage: bin/MakeDictionary.sh <output_file> <input_directory> <filename_filter> <options>

BIN=../../external/imagesearch/desc2dict.py

if [ $# -lt 3 ]
then
    echo "Usage: MakeDictionary.sh <output_file> <input_directory> <filename_filter> <options>"
    exit 2
fi

output_file=`echo $1`
shift
input_dir=`echo $1 | sed 's/\"//g'`
shift
filename_filter=`echo $1 | sed 's/\"//g'`
shift

mkdir -p `dirname ${output_file}`

# Create a text file with all the input files named
TMP_TXT=$(tempfile) || exit
find ${input_dir} -iname $filename_filter > ${TMP_TXT}

# Now run the utility to create the dictionary
python ${BIN} -i ${TMP_TXT} -o ${output_file} "$@"

rm ${TMP_TXT}