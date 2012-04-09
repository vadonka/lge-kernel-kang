#!/bin/sh

export cc=arm-linux-gnueabi-

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
