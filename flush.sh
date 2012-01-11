#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1

if [ -e .config ]; then
    cp .config config.orig
fi
make clean
make CROSS_COMPILE=$CCOMPILER clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER mrproper
if [ -e config.orig ]; then
    cp config.orig .config
    rm config.orig
fi