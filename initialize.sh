#!/bin/sh

export CCOMPILER=arm-linux-gnueabi-
export USE_CCACHE=1

make clean
make CROSS_COMPILE=$CCOMPILER clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER clean
make ARCH=arm CROSS_COMPILE=$CCOMPILER mrproper
make ARCH=arm CROSS_COMPILE=$CCOMPILER vadonka_defconfig
