#!/bin/sh

#standard version
./initialize.sh
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
#48mS
./compile.sh 48 shared 16
#64m
./compile.sh 64 shared 32
#80m
./compile.sh 80 shared 32
#96m
./compile.sh 96 shared 32
