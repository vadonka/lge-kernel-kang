#!/bin/sh

source compiler.def

make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
