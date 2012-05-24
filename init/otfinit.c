/*
 *  linux/init/otfinit.c
 *  (c) vadonka 2012
 */

#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/smp.h>

/* AVP Freq */
unsigned int avpfreq;
unsigned int MIN_AVPFREQ = 200000;
unsigned int MAX_AVPFREQ = 280000;

EXPORT_SYMBOL(avpfreq);
EXPORT_SYMBOL(MIN_AVPFREQ);
EXPORT_SYMBOL(MAX_AVPFREQ);

static int __init set_avpfreq(char *str)
{
	unsigned int i;

	if ( simple_strtoul(str,NULL,0) < MIN_AVPFREQ || simple_strtoul(str,NULL,0) > MAX_AVPFREQ )
	{
		avpfreq = 240000; // Default value
	}
	else
	{
		avpfreq = simple_strtoul(str,NULL,0);
	}
	return 1;
}
__setup("avpfreq=", set_avpfreq);
/* AVP Freq End */

/* GPU Freq */
unsigned int gpufreq;
unsigned int MIN_GPUFREQ = 300000;
unsigned int MAX_GPUFREQ = 366000;

EXPORT_SYMBOL(gpufreq);
EXPORT_SYMBOL(MIN_GPUFREQ);
EXPORT_SYMBOL(MAX_GPUFREQ);

static int __init set_gpufreq(char *str)
{
	unsigned int i;

	if ( simple_strtoul(str,NULL,0) < MIN_GPUFREQ || simple_strtoul(str,NULL,0) > MAX_GPUFREQ )
	{
		gpufreq = 333000; // Default value
	}
	else
	{
		gpufreq = simple_strtoul(str,NULL,0);
	}
	return 1;
}
__setup("gpufreq=", set_gpufreq);
/* GPU Freq End */

/* VDE Freq */
unsigned int vdefreq;
unsigned int MIN_VDEFREQ = 600000;
unsigned int MAX_VDEFREQ = 700000;

EXPORT_SYMBOL(vdefreq);
EXPORT_SYMBOL(MIN_VDEFREQ);
EXPORT_SYMBOL(MAX_VDEFREQ);

static int __init set_vdefreq(char *str)
{
	unsigned int i;

	if ( simple_strtoul(str,NULL,0) < MIN_VDEFREQ || simple_strtoul(str,NULL,0) > MAX_VDEFREQ )
	{
		vdefreq = 650000; // Default value
	}
	else
	{
		vdefreq = simple_strtoul(str,NULL,0);
	}
	return 1;
}
__setup("vdefreq=", set_vdefreq);
/* VDE Freq End */
