#include <linux/spica.h>
#define PROC_DIR "spica"

#ifdef CONFIG_OTF_CPU1
/*******************************************/
/* CPU1 On Min Khz */
/* Default value */
unsigned int NVRM_CPU1_ON_MIN_KHZ = 810000;
/* Min & Max Limit */
unsigned int ONMINLOW = 216000;
unsigned int ONMINHIGH = 1015000;
/*******************************************/
/* CPU1 Off Max Khz */
/* Default value */
unsigned int NVRM_CPU1_OFF_MAX_KHZ = 860000;
/* Min & Max Limit */
unsigned int OFFMAXLOW = 216000;
unsigned int OFFMAXHIGH = 1015000;
/*******************************************/
unsigned int NVRM_CPU1_OFF_PENDING_MS = 600;
#endif

#ifdef CONFIG_OTF_AVP
/*******************************************/
/* Default AVP Freq */
unsigned int AVPFREQ = 240000;
/* Min & Max Limit */
unsigned int AVPLOW = 200000;
unsigned int AVPHIGH = 250000;
/*******************************************/
#endif

#ifdef CONFIG_OTF_GPU
/*******************************************/
/* Default GPU Freq */
unsigned int GPUFREQ = 380000;
/* Min & Max Limit */
unsigned int GPULOW = 280000;
unsigned int GPUHIGH = 400000;
/*******************************************/
#endif

#ifdef CONFIG_OTF_VDE
/*******************************************/
/* Default VDE Freq */
unsigned int VDEFREQ = 680000;
/* Min & Max Limit */
unsigned int VDELOW = 500000;
unsigned int VDEHIGH = 700000;
/*******************************************/
#endif

#ifdef CONFIG_OTF_PSNIT
unsigned int PWONOFF = 0;
unsigned int NITROONOFF = 0;
#endif
#ifdef CONFIG_OTF_AP20LC
unsigned int NVRM_AP20_LOW_CORE_MV = 925;
unsigned int NVRM_AP20_LOW_CPU_MV = 770;
#endif

#ifdef CONFIG_OTF_MAXSCOFF
/*******************************************/
/* Default Screen Off Max CPU Freq */
unsigned int SCREENOFFFREQ = 324000;
/* Min & Max Limit */
unsigned int SMFREQLOW = 216000;
unsigned int SMFREQHIGH = 816000;
/*******************************************/
#endif

#ifdef CONFIG_OTF_GPURAM
unsigned int GPURAMSIZE = CONFIG_GPU_MEM_CARVEOUT_SZ;
#endif

#ifdef CONFIG_OTF_SCMV
/*******************************************/
/* Default Suspend Core mV */
const unsigned int NVRM_AP20_SUSPEND_CORE_MV = 1000;
/* Min & Max Limit */
unsigned int SCMVLOW = 600;
unsigned int SCMVHIGH = 1100;
/*******************************************/
#endif

#ifdef CONFIG_OTF_DDR2MIN
/*******************************************/
/* Default DDR2 Min Khz */
unsigned int NVRM_AP20_DDR2_MIN_KHZ = 50000;
/* Min & Max Limit */
unsigned int DDR2MINLOW = 10000;
unsigned int DDR2MINHIGH = 55000;
/*******************************************/
#endif

#ifdef CONFIG_OTF_LPDDR2
/*******************************************/
/* Default LPDDR2 Min Khz */
unsigned int NVRM_AP20_LPDDR2_MIN_KHZ = 18000;
/* Min & Max Limit */
unsigned int LPDDR2MINLOW = 7000;
unsigned int LPDDR2MINHIGH = 20000;
/*******************************************/
#endif

static struct proc_dir_entry *spica_dir = NULL;

void spica_init() {
spica_dir = proc_mkdir(PROC_DIR, NULL);
}

struct proc_dir_entry* spica_add(const char* name) {
if ( spica_dir == NULL ) spica_init();
return create_proc_entry(name, 0777, spica_dir);
}

void spica_remove(const char* name) {
remove_proc_entry(name, spica_dir);
}
