#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1
make clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER clean
