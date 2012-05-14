#include "otf.h"

#define PROC_DIR "spica"

/*******************************************/
/* Default AVP Freq */
unsigned int AVPFREQ = 240000;
/* Min & Max Limit */
unsigned int AVPLOW = 200000;
unsigned int AVPHIGH = 280000;
/*******************************************/
/* Default GPU Freq */
unsigned int GPUFREQ = 350000;
/* Min & Max Limit */
unsigned int GPULOW = 300000;
unsigned int GPUHIGH = 350000;
/*******************************************/
/* Default VDE Freq */
unsigned int VDEFREQ = 650000;
/* Min & Max Limit */
unsigned int VDELOW = 600000;
unsigned int VDEHIGH = 700000;
/*******************************************/
unsigned int PWONOFF = 0;
unsigned int NITROONOFF = 0;
/*******************************************/
/* Default Screen Off Max CPU Freq */
unsigned int SCREENOFFFREQ = 324000;
/* Min & Max Limit */
unsigned int SMFREQLOW = 216000;
unsigned int SMFREQHIGH = 816000;
/*******************************************/

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
