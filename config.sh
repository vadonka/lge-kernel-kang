#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1

make ARCH=arm CROSS_COMPILE=$CCOMPILER menuconfig
