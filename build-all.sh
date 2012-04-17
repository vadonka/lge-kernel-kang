#!/bin/bash

#no overclock with stock battery driver
./compile.sh --gcc 4.7.1 --ccache

#no overclock with DS battery driver
./compile.sh --ds --gcc 4.7.1 --ccache

#overclock with stock battery driver
./compile.sh --oc --gcc 4.7.1 --ccache

#overclock with DS battery driver
./compile.sh --oc --ds --gcc 4.7.1 --ccache
