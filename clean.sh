#!/bin/bash

source variables

cc=/home/android/android/linaro-toolchain/$gccversion/bin/$gccstring-

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
