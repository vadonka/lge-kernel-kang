#!/bin/sh

source compiler.def

if [ -e .config ]; then
	rm -f .config
fi

if [ -z $1 ]; then
	export DEFCONFIG="vadonka_defconfig"
else
	export DEFCONFIG="$1"
fi

make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
make ARCH=arm CROSS_COMPILE=$cc $DEFCONFIG
