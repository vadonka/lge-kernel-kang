#!/bin/sh

source compiler.def

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
