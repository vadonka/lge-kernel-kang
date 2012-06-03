#!/bin/sh

export cc=/home/android/android/android-toolchain-eabi_4.y/4.7.1/bin/arm-eabi-

if [ -e .config ]; then
	rm -f .config
fi

if [ -z $1 ]; then
	export DEFCONFIG="vadonka_loc_lite_defconfig"
else
	export DEFCONFIG="$1"
fi

make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
make ARCH=arm CROSS_COMPILE=$cc $DEFCONFIG
