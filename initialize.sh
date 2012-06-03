#!/bin/bash

source variables

cc=/home/android/android/linaro-toolchain/$gccversion/bin/$gccstring-

if [ -e .config ]; then
	rm -f .config
fi

if [ ! -z $1 ]; then
	export defconfig="$1"
fi

make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
make ARCH=arm CROSS_COMPILE=$cc $defconfig
