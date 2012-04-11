#!/bin/bash
# ANYKERNEL compiler script by vadonka v1.2.2
# Date: 2012.04.11
#
# You need to define this below:
######################################################
# KERNEL home directory
export kh=`pwd`
# Compiled CWM zip files home directory
export ch=/home/android/android/compiled
# CM7 original lge kernel boot.img location
export cm7b=/home/android/android/cm7orig_kernel
# LOG file location
export WARNLOG=`pwd`/warn.log
# Kernel installer source
export kinstsrc=/home/android/android/kernel-installer/source
# Maximum thread number
export mthd=`grep 'processor' /proc/cpuinfo | wc -l`
# Compiler
source compiler.def
######################################################

# Check executables
if [ -f /usr/bin/zip ]; then
	if [ -f /usr/bin/unzip ]; then
		if [ -f /usr/bin/abootimg ]; then
			echo "Required executables are found. OK!"
		else
			echo "ERROR: abootimg not found! Please install"
			exit 1
		fi
	else
		echo "ERROR: unzip not found! Please install"
		exit 1
	fi
else
	echo "ERROR: zip not found! Please install"
	exit 1
fi

## Check compiler options
# declare makeflags variable
makeflags=""
# set DS battery driver flag to zero by default
dsbatt="0"
# set the OC flag to zero by default
overclock="0"
## Procedure begin
while [ -n "$*" ]; do
flag=$1
	case "$flag" in
	"--ds")
	makeflags=$makeflags" CONFIG_USE_DS_BATTERY_DRIVER=y"
	dsbatt="1"
	;;
	"--oc")
	makeflags=$makeflags" CONFIG_FAKE_SHMOO=y"
	overclock="1"
	;;
	esac
	shift
done

# Read current kernel version
export cver=`grep "^CONFIG_LOCALVERSION" $kh/.config`
export otf=`grep -c "^CONFIG_SPICA_OTF" $kh/.config`

if [ "$overclock" == "1" ]; then
	export ocver="OC"
else
	export ocver="STOCK"
fi

if [ "$dsbatt" == "0" ]; then
	if [ "$otf" == "0" ]; then
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'"'`
		sed -i "s/$cver/$nver/g" $kh/.config
	else
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_OTF"'`
		sed -i "s/$cver/$nver/g" $kh/.config
	fi
else
	if [ "$otf" == "0" ]; then
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_DS"'`
		sed -i "s/$cver/$nver/g" $kh/.config
	else
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_DS_OTF"'`
		sed -i "s/$cver/$nver/g" $kh/.config
	fi
fi

export starttime=`date +%s`
make clean -j $mthd > /dev/null 2>&1
clear
echo "Kernel home: $kh"
echo "Cross Compiler: $cc"
echo "Extra Makeflags:$makeflags"
echo "Build command: "`echo "make ARCH=arm CROSS_COMPILE=$cc -j $mthd $makeflags 2> $WARNLOG"`
export kver=`echo $nver | awk 'BEGIN { FS = "=" } ; { print $2 }' | sed 's/"//g'`
echo "Kernel version string: $kver"
export cdir=`date +%y%m%d%H%M`$kver-3.0.y
echo "Kernel CWM file name: $cdir.zip"
exit 0
sleep 2
make ARCH=arm CROSS_COMPILE=$cc clean -j $mthd > /dev/null 2>&1
make ARCH=arm CROSS_COMPILE=$cc -j $mthd $makeflags 2> $WARNLOG
export endtime=`date +%s`

if [ -e $kh/arch/arm/boot/zImage ]; then

mkdir -p $ch/$cdir
cp -r $kinstsrc/* -d $ch/$cdir

for m in `find $kh -name '*.ko'`; do
    cp $m $ch/$cdir/system/lib/modules/
done

cp $kh/arch/arm/boot/zImage $ch/$cdir/tmp
cd $ch/$cdir && zip -rq9 $ch/$cdir.zip .
cp $kh/arch/arm/boot/zImage $ch/$cdir/tmp

fi

if [ "$(($endtime-$starttime))" -lt "180" ]; then
    echo "Building time: $(($endtime-$starttime)) seconds"
else
    echo "Building time: $(($endtime/60-$starttime/60)) minutes"
fi
