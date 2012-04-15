#!/bin/bash

source compiler.def
export cc=/home/android/android/android-toolchain-eabi_4.y/$gccversion/bin/arm-eabi-

make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
