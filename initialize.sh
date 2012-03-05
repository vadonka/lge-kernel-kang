#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache

if [ -e .config ]; then
	rm -f .config
fi

if [ -z $1 ]; then
	export DEFCONFIG="vadonka_loc_ds_defconfig"
else
	export DEFCONFIG="$1"
fi

make clean
make CROSS_COMPILE=$CCOMPILER clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER mrproper
make ARCH=arm CROSS_COMPILE=$CCOMPILER $DEFCONFIG
