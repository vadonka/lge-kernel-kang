#ifndef _SPICA_H
#define _SPICA_H
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#undef CFS_BOOST
#define CFS_BOOST_NICE -15

//extern unsigned int USE_FAKE_SHMOO;

extern void suspend_prepare_freq(void);
extern void resume_prepare_freq(void);
extern unsigned int NVRM_CPU1_ON_MIN_KHZ;
extern unsigned int NVRM_CPU1_OFF_MAX_KHZ;
extern unsigned int VDEFREQ;
extern unsigned int GPUFREQ;
extern unsigned int SDRAMFREQ;
extern unsigned int PWONOFF;
extern unsigned int NITROONOFF;
extern unsigned int NVRM_AP20_LOW_CORE_MV;
extern unsigned int NVRM_AP20_LOW_CPU_MV;
extern unsigned int NVRM_CPU1_OFF_PENDING_MS;
//extern unsigned int VM_MAX_READAHEAD;
//extern unsigned int swappiness;
//extern unsigned int vm_swappiness;
//extern unsigned int CARVEOUT_SIZE;
//extern unsigned int STAR_RAM_CONSOLE_BASE;
//extern unsigned int STAR_RAM_CONSOLE_SIZE;
//extern unsigned int USE_FG;
//extern unsigned int RAM_CONSOLE_RESERVED_SIZE;
extern unsigned int MAXSCREENOFFCPUFREQ;
//extern unsigned int USE_VALUE;
//extern unsigned int RAMHACK;

//extern unsigned int STAR_RAM_CONSOLE_BASE;
//extern unsigned int STAR_RAM_CONSOLE_BASE;
extern const unsigned int NVRM_AP20_SUSPEND_CORE_MV;
extern unsigned int NVRM_AP20_DDR2_MIN_KHZ;
extern unsigned int NVRM_AP20_LPDDR2_MIN_KHZ;
//static unsigned STAR_RAM_CONSOLE_SIZE;
//static unsigned STAR_RAM_CONSOLE_BASE;
//extern STAR_RAM_CONSOLE_BASE;
//extern unsigned int STAR_RAM_CONSOLE_SIZE;
//
struct proc_dir_entry* spica_add(const char* name);
void spica_remove(const char* name);

#endif
