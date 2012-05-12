#include "otf.h"

#define MAXSM_PROCFS_NAME "screenoff_maxcpufreq"
#define MAXSM_PROCFS_SIZE 7
static struct proc_dir_entry *MAXSM_Proc_File;
static char procfs_buffer_sm[MAXSM_PROCFS_SIZE];
static unsigned long procfs_buffer_size_sm = 0;

int maxsm_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "procfile_read (/proc/spica/%s) called\n", MAXSM_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_sm, procfs_buffer_size_sm);
		ret = procfs_buffer_size_sm;
	}

	return ret;
}

int maxsm_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_sm;
	temp_sm = 0;

	if ( sscanf(buffer,"%d",&temp_sm) < 1 )
		return procfs_buffer_size_sm;

	if ( temp_sm < SMFREQLOW || temp_sm > SMFREQHIGH )
		return procfs_buffer_size_sm;

	procfs_buffer_size_sm = count;

	if (procfs_buffer_size_sm > MAXSM_PROCFS_SIZE) {
		procfs_buffer_size_sm = MAXSM_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_sm, buffer, procfs_buffer_size_sm)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_sm,"%u",&SCREENOFFFREQ);
	return procfs_buffer_size_sm;
}

static int __init init_maxsm_procsfs(void) {
	MAXSM_Proc_File = spica_add(MAXSM_PROCFS_NAME);

	if (MAXSM_Proc_File == NULL) {
		spica_remove(MAXSM_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", MAXSM_PROCFS_NAME);
		return -ENOMEM;
	} else {
		MAXSM_Proc_File->read_proc	= maxsm_procfile_read;
		MAXSM_Proc_File->write_proc	= maxsm_procfile_write;
		MAXSM_Proc_File->mode		= S_IFREG | S_IRUGO;
		MAXSM_Proc_File->uid		= 0;
		MAXSM_Proc_File->gid		= 0;
		MAXSM_Proc_File->size		= 37;
		sprintf(procfs_buffer_sm,"%d",SCREENOFFFREQ);
		procfs_buffer_size_sm		= strlen(procfs_buffer_sm);
		printk(KERN_INFO "/proc/spica/%s created\n", MAXSM_PROCFS_NAME);
	}

	return 0;
}

module_init(init_maxsm_procsfs);

static void __exit cleanup_maxsm_procsfs(void) {
	spica_remove(MAXSM_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", MAXSM_PROCFS_NAME);
}

module_exit(cleanup_maxsm_procsfs);
