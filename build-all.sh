#!/bin/bash

./initialize.sh vadonka_defconfig

#no overclock with stock battery driver
./compile.sh

#no overclock with DS battery driver
./compile.sh --ds

#overclock with stock battery driver
./compile.sh --oc

#overclock with DS battery driver
./compile.sh --oc --ds
