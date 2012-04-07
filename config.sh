#!/bin/sh

export cc=arm-linux-gnueabi-
#export USE_CCACHE=1
#export CCACHE_DIR=~/android/ccache

make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
