#ifndef _OTF_H
#define _OTF_H
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/uaccess.h>

#undef CFS_BOOST
#undef CONFIG_CFS_BOOST
#define CFS_BOOST_NICE -15

extern unsigned int NVRM_CPU1_ON_MIN_KHZ;
extern unsigned int NVRM_CPU1_OFF_MAX_KHZ;
extern unsigned int NVRM_CPU1_ON_PENDING_MS;
extern unsigned int NVRM_CPU1_OFF_PENDING_MS;
extern unsigned int avpfreq;
extern unsigned int gpufreq;
extern unsigned int vdefreq;

#endif
