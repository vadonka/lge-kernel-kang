#!/bin/bash

source compiler.def
export cc=/home/android/android/android-toolchain-eabi_4.y/$gccversion/bin/arm-eabi-

make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
