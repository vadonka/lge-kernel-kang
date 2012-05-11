#include <linux/spica.h>

static struct proc_dir_entry *spica_dir;

/* CPU1 ON MIN */
#define CPUON_PROCFS_NAME "mincpu1on"
#define CPUON_PROCFS_SIZE 8
static unsigned long procfs_buffer_size_cpuon = 0;
static struct proc_dir_entry *CPUON_Proc_File;
static char procfs_buffer_cpuon[CPUON_PROCFS_SIZE];
extern unsigned int ONMINLOW;
extern unsigned int ONMINHIGH;

int cpuon_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "procfile_read (/proc/spica/%s) called\n", CPUON_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_cpuon, procfs_buffer_size_cpuon);
		ret = procfs_buffer_size_cpuon;
	}

	return ret;
}

int cpuon_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_cpuon;
	temp_cpuon = 0;

	if ( sscanf(buffer,"%d",&temp_cpuon) < 1 )
		return procfs_buffer_size_cpuon;

	if ( temp_cpuon < ONMINLOW || temp_cpuon > ONMINHIGH )
		return procfs_buffer_size_cpuon;

	procfs_buffer_size_cpuon = count;

	if (procfs_buffer_size_cpuon > CPUON_PROCFS_SIZE) {
		procfs_buffer_size_cpuon = CPUON_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_cpuon, buffer, procfs_buffer_size_cpuon)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_cpuon,"%u",&NVRM_CPU1_ON_MIN_KHZ);
	return procfs_buffer_size_cpuon;
}

static int __init init_cpuon_procsfs(void) {
	CPUON_Proc_File = spica_add(CPUON_PROCFS_NAME);

	if (CPUON_Proc_File == NULL) {
		spica_remove(CPUON_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", CPUON_PROCFS_NAME);
		return -ENOMEM;
	} else {
		CPUON_Proc_File->read_proc	= cpuon_procfile_read;
		CPUON_Proc_File->write_proc	= cpuon_procfile_write;
		CPUON_Proc_File->mode		= S_IFREG | S_IRUGO;
		CPUON_Proc_File->uid		= 0;
		CPUON_Proc_File->gid		= 0;
		CPUON_Proc_File->size		= 37;
		sprintf(procfs_buffer_cpuon,"%d",NVRM_CPU1_ON_MIN_KHZ);
		procfs_buffer_size_cpuon	= strlen(procfs_buffer_cpuon);
		printk(KERN_INFO "/proc/spica/%s created\n", CPUON_PROCFS_NAME);
	}

	return 0;
}

module_init(init_cpuon_procsfs);

static void __exit cleanup_cpuon_procsfs(void) {
	spica_remove(CPUON_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", CPUON_PROCFS_NAME);
}

module_exit(cleanup_cpuon_procsfs);

/* CPU1 OFF MAX */
#define CPUOFF_PROCFS_NAME "maxcpu1off"
#define CPUOFF_PROCFS_SIZE 8
extern unsigned int OFFMAXLOW;
extern unsigned int OFFMAXHIGH;
static struct proc_dir_entry *CPUOFF_Proc_File;
static char procfs_buffer_cpuoff[CPUOFF_PROCFS_SIZE];
static unsigned long procfs_buffer_size_cpuoff = 0;

int cpuoff_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "cpuoff_procfile_read (/proc/spica/%s) called\n", CPUOFF_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_cpuoff, procfs_buffer_size_cpuoff);
		ret = procfs_buffer_size_cpuoff;
	}

	return ret;
}

int cpuoff_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_cpuoff;
	temp_cpuoff = 0;

	if ( sscanf(buffer,"%d",&temp_cpuoff) < 1 )
		return procfs_buffer_size_cpuoff;

	if ( temp_cpuoff < OFFMAXLOW || temp_cpuoff > OFFMAXHIGH )
		return procfs_buffer_size_cpuoff;

	procfs_buffer_size_cpuoff = count;

	if (procfs_buffer_size_cpuoff > CPUOFF_PROCFS_SIZE) {
		procfs_buffer_size_cpuoff = CPUOFF_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_cpuoff, buffer, procfs_buffer_size_cpuoff)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_cpuoff,"%u",&NVRM_CPU1_OFF_MAX_KHZ);
	return procfs_buffer_size_cpuoff;
}

static int __init cpuoff_procsfs(void) {
	CPUOFF_Proc_File = spica_add(CPUOFF_PROCFS_NAME);

	if (CPUOFF_Proc_File == NULL) {
		spica_remove(CPUOFF_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", CPUOFF_PROCFS_NAME);
		return -ENOMEM;
	} else {
		CPUOFF_Proc_File->read_proc	= cpuoff_procfile_read;
		CPUOFF_Proc_File->write_proc	= cpuoff_procfile_write;
		CPUOFF_Proc_File->mode		= S_IFREG | S_IRUGO;
		CPUOFF_Proc_File->uid		= 0;
		CPUOFF_Proc_File->gid		= 0;
		CPUOFF_Proc_File->size		= 37;
		sprintf(procfs_buffer_cpuoff,"%d",NVRM_CPU1_OFF_MAX_KHZ);
		procfs_buffer_size_cpuoff	= strlen(procfs_buffer_cpuoff);
		printk(KERN_INFO "/proc/spica/%s created\n", CPUOFF_PROCFS_NAME);
}

	return 0;
}

module_init(cpuoff_procsfs);

static void __exit cpu_cleanup_cpuoff_procsfs(void) {
	spica_remove(CPUOFF_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", CPUOFF_PROCFS_NAME);
}

module_exit(cpu_cleanup_cpuoff_procsfs);
