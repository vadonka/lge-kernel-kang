#!/bin/bash

source variables

if [ ! -f .config ]; then
	make ARCH=arm CROSS_COMPILE=$cc mrproper
	make ARCH=arm CROSS_COMPILE=$cc $defconfig
fi

while [ -n "$*" ]; do
flag=$1
	case "$flag" in
	"--update-defconfig")
	if [ -f .config ]; then
		cp -f .config arch/arm/configs/$defconfig
		echo "$defconfig is updated succesfully"
		exit 0
	else
		echo ".config file not exist"
		exit 0
	fi
	;;
	esac
	shift
done

export cc=/home/android/android/android-toolchain-eabi_4.y/$gccversion/bin/arm-eabi-
make ARCH=arm CROSS_COMPILE=$cc menuconfig 2> warn.log
