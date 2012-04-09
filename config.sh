#!/bin/sh

export cc=arm-linux-gnueabi-

make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
