#!/bin/sh

export cc=/home/android/android/linaro-toolchain/4.7.2/bin/arm-eabi-

make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
