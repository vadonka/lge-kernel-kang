#!/bin/bash

source variables

cc=/home/android/android/linaro-toolchain/$gccversion/bin/$gccstring-

make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
