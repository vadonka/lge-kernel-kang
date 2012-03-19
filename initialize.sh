#!/bin/sh

export cc=arm-linux-gnueabi-
#export cc=/home/android/android/android-toolchain-eabi-4.5/bin/arm-eabi-
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache

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
