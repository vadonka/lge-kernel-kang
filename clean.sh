#!/bin/bash

source variables

make clean
make ARCH=arm CROSS_COMPILE=$cc clean
