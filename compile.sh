#!/bin/bash
# ANYKERNEL compiler script by vadonka v1.1.7
# Date: 2012.03.09
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
# Maximum thread number, multiplier
export mthd=`grep 'processor' /proc/cpuinfo | wc -l`
export mthm=1
# Compiler
export cc=arm-linux-gnueabi-
export cc=/home/android/android/android-toolchain-eabi_4.y/4.7.1/bin/arm-eabi-
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

# Check variables
#if [ -z $1 ]; then
#	export rh="0"
#	echo "Ramhack: no ramhack defined"
#elif [[ $1 = [0-9]* ]]; then
#	let rh=$1
#	echo "Ramhack: ramhack is defined, size is: $1MB"
#else
#	echo "Invalid ramhack size, ramhack is not used"
#	export rh="0"
#fi

#let csize=$((128-$rh))
#echo "Using traditional ramhack mode"

# Carveout size tweak
#export cout=`grep "^CONFIG_GPU_MEM_CARVEOUT" $kh/.config`
#export cnew=`echo 'CONFIG_GPU_MEM_CARVEOUT_SZ='$(($csize))`
#sed -i "s/$cout/$cnew/g" $kh/.config

# Read current kernel version
export cver=`grep "^CONFIG_LOCALVERSION" $kh/.config`
export nooc=`grep -c "# CONFIG_FAKE_SHMOO" $kh/.config`
export loc=`grep -c "^CONFIG_STOCK_VOLTAGE" $kh/.config`
export dsbatt=`grep -c "^CONFIG_USE_DS_BATTERY" $kh/.config`
export litebatt=`grep -c "^CONFIG_USE_LITE_BATTERY" $kh/.config`
export otf=`grep -c "^CONFIG_SPICA_OTF" $kh/.config`

if [ "$nooc" == "0" ]; then
	if [ "$loc" == "1" ]; then
		export ocver="LOC"
	else
		export ocver="HOC"
	fi
else
	export ocver="STOCK"
fi

if [ "$dsbatt" == "0" ]; then
	if [ "$litebatt" == "0" ]; then
		if [ "$otf" == "0" ]; then
			export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'"'`
			sed -i "s/$cver/$nver/g" $kh/.config
		else
			export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_OTF"'`
			sed -i "s/$cver/$nver/g" $kh/.config
		fi
	elif [ "$otf" == "0" ]; then
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_LITE"'`
		sed -i "s/$cver/$nver/g" $kh/.config
	else
		export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_LITE_OTF"'`
		sed -i "s/$cver/$nver/g" $kh/.config
	fi
elif [ "$otf" == "0" ]; then
	export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_DS"'`
	sed -i "s/$cver/$nver/g" $kh/.config
else
	export nver=`echo 'CONFIG_LOCALVERSION="-ETaNa_'$ocver'_DS_OTF"'`
	sed -i "s/$cver/$nver/g" $kh/.config
fi

export starttime=`date +%s`
export USE_CCACHE=1
export CCACHE_DIR=~/android/ccache
make clean -j $(($mthd*$mthm))
make ARCH=arm CROSS_COMPILE=$cc clean -j $mthd
make ARCH=arm CROSS_COMPILE=$cc -j $mthd 2> $WARNLOG
export endtime=`date +%s`

if [ -e $kh/arch/arm/boot/zImage ]; then
export kver=`echo $nver | awk 'BEGIN { FS = "=" } ; { print $2 }' | sed 's/"//g'`

export cdir=`date +%y%m%d%H%M`$kver
mkdir -p $ch/$cdir
cp -r $kinstsrc/* -d $ch/$cdir

for m in `find $kh -name '*.ko'`; do
    cp $m $ch/$cdir/system/lib/modules
done

cp $kh/arch/arm/boot/zImage $ch/$cdir/tmp
#cp $cm7b/boot.img $ch/$cdir/tmp
#abootimg -u $ch/$cdir/tmp/boot.img -k $kh/arch/arm/boot/zImage
#abootimg -u $ch/$cdir/tmp/boot.img -c "cmdline = mem=`echo $((383+$rh))'M@0M'` nvmem=`echo $((128-$rh))'M@'$((384+$rh))'M'` loglevel=0 muic_state=1 \
#lpj=9994240 CRC=3010002a8e458d7 vmalloc=256M brdrev=1.0 video=tegrafb console=ttyS0,115200n8 \
#usbcore.old_scheme_first=1 tegraboot=sdmmc tegrapart=recovery:35e00:2800:800,linux:34700:1000:800,\
#mbr:400:200:800,system:600:2bc00:800,cache:2c200:8000:800,misc:34200:400:800,\
#userdata:38700:c0000:800 androidboot.hardware=p990"
#abootimg -i $ch/$cdir/tmp/boot.img > $ch/$cdir/tmp/bootimg.info
cd $ch/$cdir && zip -rq9 $ch/$cdir.zip .
cp $kh/arch/arm/boot/zImage $ch/$cdir/tmp

fi

if [ "$(($endtime-$starttime))" -lt "180" ]; then
    echo "Building time: $(($endtime-$starttime)) seconds"
else
    echo "Building time: $(($endtime/60-$starttime/60)) minutes"
fi
