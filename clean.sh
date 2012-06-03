#!/bin/sh

export cc=/home/android/android/android-toolchain-eabi_4.y/4.7.1/bin/arm-eabi-

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
