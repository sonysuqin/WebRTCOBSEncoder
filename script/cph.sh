#!/bin/bash

if [ $# -ne 2 ]; then
    echo Invalid param, input src and dst directory.
    exit
fi

src=$1
dst=$2

if [ ! -d $src ]; then
    echo $src not exist.
    exit
fi

if [ ! -d $dst ]; then
    mkdir -p $dst
fi

copyType() {
    type=$1
    files=`find $src -name $type`
    for file in $files; do
        srcDir=${file%/*}
        [ -d $dst/$srcDir ] || mkdir -p $dst/$srcDir
        cp -pv $file $dst/$srcDir
    done
}

copyType "*.h"
copyType "*.hpp"

echo Done.
