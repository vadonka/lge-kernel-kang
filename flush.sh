#!/bin/bash

source variables

make clean
make CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc clean
make ARCH=arm CROSS_COMPILE=$cc mrproper
