#!/bin/sh

#standard version
./initialize.sh vadonka_defconfig
#noramhack
./compile.sh 0
#32m
./compile.sh 32
#48m
./compile.sh 48
#64m
./compile.sh 64
#80m
./compile.sh 80
#96m
./compile.sh 96

#low overclock version
./initialize.sh vadonka_loc_defconfig
#noramhack
./compile.sh 0
#32m
./compile.sh 32
#48m
./compile.sh 48
#64m
./compile.sh 64
#80m
./compile.sh 80
#96m
./compile.sh 96

#high overclock version
./initialize.sh vadonka_hoc_defconfig
#noramhack
./compile.sh 0
#32m
./compile.sh 32
#48m
./compile.sh 48
#64m
./compile.sh 64
#80m
./compile.sh 80
#96m
./compile.sh 96
