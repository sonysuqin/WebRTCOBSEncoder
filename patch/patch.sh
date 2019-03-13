#!/bin/bash

patchDir=`pwd`
obsDir=../obs
basename=`basename $0`

files=`ls`
cd $obsDir
for file in $files; do
    if [ $file == $basename ]; then
        continue
    fi

    filePath=${patchDir}"/"${file}
    echo Patching $filePath
    git apply $filePath
done
