#!/bin/bash

source compiler.def

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
