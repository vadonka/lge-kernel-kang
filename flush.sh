#!/bin/sh

#export cc=arm-linux-gnueabi-
export cc=/home/android/android/android-toolchain-eabi/bin/arm-eabi-
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache

if [ -e .config ]; then
    cp .config config.orig
fi
make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
if [ -e config.orig ]; then
    cp config.orig .config
    rm config.orig
fi
