#!/bin/bash
# ANYKERNEL compiler script by vadonka v1.2.6
# Date: 2012.06.18
#
# You need to define this below:
######################################################
# KERNEL home directory
export kh=`pwd`
# Compiled CWM zip files home directory
export ch=/home/android/android/compiled
# LOG file location
export WARNLOG=`pwd`/warn.log
# Kernel installer source
export kinstsrc=/home/android/android/kernel-installer/source
# Maximum thread number
export mthd=`grep 'processor' /proc/cpuinfo | wc -l`
# Variable file
source variables
######################################################

# Check executables
if [ ! -f /usr/bin/zip ]; then
	echo "ERROR: zip not found! Please install"
	exit 1
fi

# Check .config
if [ ! -f .config ]; then
    make ARCH=arm CROSS_COMPILE=$cc mrproper
    make ARCH=arm CROSS_COMPILE=$cc $defconfig
fi

## Check compiler options
# set DS battery driver flag to zero by default
dsbatt="0"
# set the OC flag to zero by default
overclock="0"
## Procedure begin
while [ -n "$*" ]; do
flag=$1
value=$2
	case "$flag" in
	"--ds")
	config_ds_orig=`grep "CONFIG_USE_DS_BATTERY_DRIVER" $kh/.config`
	config_ds_enable=`echo "CONFIG_USE_DS_BATTERY_DRIVER=y"`
	sed -i "s/$config_ds_orig/$config_ds_enable/g" $kh/.config
	dsbatt="1"
	;;
	"--oc")
	config_oc_orig=`grep "CONFIG_OVERCLOCK" $kh/.config`
	config_oc_enable=`echo "CONFIG_OVERCLOCK=y"`
	sed -i "s/$config_oc_orig/$config_oc_enable/g" $kh/.config
	overclock="1"
	;;
	esac
	shift
done

export starttime=`date +%s`

# Read current kernel version
export cver=`grep "^CONFIG_LOCALVERSION" $kh/.config`

if [ "$overclock" == "1" ]; then
	export ocver="OC"
else
	export ocver="STOCK"
fi

if [ "$dsbatt" == "0" ]; then
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'"'`
		sed -i "s/$cver/$nver/g" $kh/.config
else
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_DS"'`
		sed -i "s/$cver/$nver/g" $kh/.config
fi

make clean -j $mthd > /dev/null 2>&1
clear
echo "Kernel home: $kh"
echo "Cross Compiler: $cc"
export kver=`echo $nver | awk 'BEGIN { FS = "=" } ; { print $2 }' | sed 's/"//g'`
echo "Kernel version string: $kver"
export cdir=`date +%y%m%d%H%M`$kver-3.0.y
echo "Kernel CWM file name: $cdir.zip"
sleep 2
make ARCH=arm CROSS_COMPILE=$cc clean -j $mthd > /dev/null 2>&1
make ARCH=arm CROSS_COMPILE=$cc -j $mthd 2> $WARNLOG
export endtime=`date +%s`

if [ -f $kh/arch/arm/boot/zImage ]; then
	mkdir -p $ch/$cdir
	cp -r $kinstsrc/* -d $ch/$cdir

	mkdir -p $ch/$cdir/system/lib/modules
	for m in `find $kh -name '*.ko'`; do
		cp $m $ch/$cdir/system/lib/modules/
	done

	cp $kh/arch/arm/boot/zImage $ch/$cdir/tmp
	cd $ch/$cdir && zip -rq9 $ch/$cdir.zip .
	cp $kh/arch/arm/boot/zImage $ch/$cdir/tmp
fi

echo "Building time: $(($endtime-$starttime)) seconds"

# Disable the extra options by the default
config_ds_new=`grep "CONFIG_USE_DS_BATTERY_DRIVER" $kh/.config`
sed -i "s/$config_ds_new/# CONFIG_USE_DS_BATTERY_DRIVER is not set/g" $kh/.config
config_oc_new=`grep "CONFIG_OVERCLOCK" $kh/.config`
sed -i "s/$config_oc_new/# CONFIG_OVERCLOCK is not set/g" $kh/.config
