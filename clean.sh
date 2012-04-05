#!/bin/sh

#export cc=arm-linux-gnueabi-
export cc=/home/android/android/android-toolchain-eabi/bin/arm-eabi-
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
