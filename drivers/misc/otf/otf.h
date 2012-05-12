#ifndef _SPICA_H
#define _SPICA_H
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#undef CFS_BOOST
#undef CONFIG_CFS_BOOST
#define CFS_BOOST_NICE -15

extern void suspend_prepare_freq(void);
extern void resume_prepare_freq(void);

extern unsigned int NVRM_CPU1_ON_MIN_KHZ;
extern unsigned int NVRM_CPU1_OFF_MAX_KHZ;
extern unsigned int AVPFREQ;
extern unsigned int AVPLOW;
extern unsigned int AVPHIGH;
extern unsigned int VDEFREQ;
extern unsigned int VDELOW;
extern unsigned int VDEHIGH;
extern unsigned int GPUFREQ;
extern unsigned int GPULOW;
extern unsigned int GPUHIGH;
extern unsigned int PWONOFF;
extern unsigned int NITROONOFF;
extern unsigned int SCREENOFFFREQ;
extern unsigned int SMFREQLOW;
extern unsigned int SMFREQHIGH;
extern unsigned int NVRM_AP20_DDR2_MIN_KHZ;
extern unsigned int NVRM_AP20_LPDDR2_MIN_KHZ;

struct proc_dir_entry* spica_add(const char* name);
void spica_remove(const char* name);

#endif
