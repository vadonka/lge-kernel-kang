#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache

make clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER clean
