#!/bin/sh

#low overclock version
./initialize.sh vadonka_loc_defconfig
./compile.sh

#low overclock version with DS battery driver
./initialize.sh vadonka_loc_ds_defconfig
./compile.sh

#high overclock version
./initialize.sh vadonka_hoc_defconfig
./compile.sh

#high overclock version with DS battery driver
./initialize.sh vadonka_hoc_ds_defconfig
./compile.sh
