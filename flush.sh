#!/bin/bash

source compiler.def

if [ -e .config ]; then
    cp .config config.orig
fi
make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
if [ -e config.orig ]; then
    cp config.orig .config
    rm config.orig
fi
