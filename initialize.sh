#!/bin/bash

source variables
export cc=/home/android/android/android-toolchain-eabi_4.y/$gccversion/bin/arm-eabi-

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
