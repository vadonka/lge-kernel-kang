#!/bin/bash

#no overclock version
./initialize.sh vadonka_defconfig
./compile.sh

#no overclock version with DS battery driver
./initialize.sh vadonka_ds_defconfig
./compile.sh

#overclock version
./initialize.sh vadonka_oc_defconfig
./compile.sh

#overclock version with DS battery driver
./initialize.sh vadonka_oc_ds_defconfig
./compile.sh
