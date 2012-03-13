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

//extern unsigned int USE_FAKE_SHMOO;

extern void suspend_prepare_freq(void);
extern void resume_prepare_freq(void);

#ifdef CONFIG_OTF_CPU1
extern unsigned int NVRM_CPU1_ON_MIN_KHZ;
extern unsigned int NVRM_CPU1_OFF_MAX_KHZ;
extern unsigned int NVRM_CPU1_OFF_PENDING_MS;
#endif

#ifdef CONFIG_OTF_AVP
extern unsigned int AVPFREQ;
#endif

#ifdef CONFIG_OTF_VDE
extern unsigned int VDEFREQ;
#endif

#ifdef CONFIG_OTF_GPU
extern unsigned int GPUFREQ;
#endif

extern unsigned int PWONOFF;
extern unsigned int NITROONOFF;

#ifdef CONFIG_OTF_AP20LC
extern unsigned int NVRM_AP20_LOW_CORE_MV;
extern unsigned int NVRM_AP20_LOW_CPU_MV;
#endif

//extern unsigned int VM_MAX_READAHEAD;
//extern unsigned int swappiness;
//extern unsigned int vm_swappiness;
//extern unsigned int CARVEOUT_SIZE;
//extern unsigned int STAR_RAM_CONSOLE_BASE;
//extern unsigned int STAR_RAM_CONSOLE_SIZE;
//extern unsigned int USE_FG;
//extern unsigned int RAM_CONSOLE_RESERVED_SIZE;

#ifdef CONFIG_OTF_MAXSCOFF
extern unsigned int SCREENOFFFREQ;
#endif

//extern unsigned int USE_VALUE;
//extern unsigned int RAMHACK;

#ifdef CONFIG_OTF_GPURAM
extern unsigned int GPURAMSIZE;
#endif

//extern unsigned int STAR_RAM_CONSOLE_BASE;
//extern unsigned int STAR_RAM_CONSOLE_BASE;
#ifdef CONFIG_OTF_SCMV
extern const unsigned int NVRM_AP20_SUSPEND_CORE_MV;
#endif

#ifdef CONFIG_OTF_DDR2MIN
extern unsigned int NVRM_AP20_DDR2_MIN_KHZ;
#endif

#ifdef CONFIG_OTF_LPDDR2
extern unsigned int NVRM_AP20_LPDDR2_MIN_KHZ;
#endif

//static unsigned STAR_RAM_CONSOLE_SIZE;
//static unsigned STAR_RAM_CONSOLE_BASE;
//extern STAR_RAM_CONSOLE_BASE;
//extern unsigned int STAR_RAM_CONSOLE_SIZE;
//
struct proc_dir_entry* spica_add(const char* name);
void spica_remove(const char* name);

#endif
