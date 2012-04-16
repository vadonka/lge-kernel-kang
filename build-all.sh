#!/bin/bash

./initialize.sh vadonka_defconfig

#no overclock with stock battery driver
./compile.sh --gcc 4.7.1

#no overclock with DS battery driver
./compile.sh --ds --gcc 4.7.1

#overclock with stock battery driver
./compile.sh --oc --gcc 4.7.1

#overclock with DS battery driver
./compile.sh --oc --ds --gcc 4.7.1
