#!/bin/sh

export cc=/home/android/android/android-toolchain-eabi-4.7.0/bin/arm-eabi-

make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
