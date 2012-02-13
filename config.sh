#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache

make ARCH=arm CROSS_COMPILE=$CCOMPILER menuconfig 2> /tmp/warn.log
