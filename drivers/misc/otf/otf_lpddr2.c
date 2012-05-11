#include <linux/spica.h>

static struct proc_dir_entry *spica_dir;

/* LPDDR2 Min Khz */
#define LPDDR2_PROCFS_NAME "lpddr2_min_khz"
#define LPDDR2_PROCFS_SIZE 6
static unsigned long procfs_buffer_size_lpddr2 = 0;
static struct proc_dir_entry *LPDDR2_Proc_File;
static char procfs_buffer_lpddr2[LPDDR2_PROCFS_SIZE];
extern unsigned int LPDDR2MINLOW;
extern unsigned int LPDDR2MINHIGH;

int lpddr2_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "procfile_read (/proc/spica/%s) called\n", LPDDR2_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_lpddr2, procfs_buffer_size_lpddr2);
		ret = procfs_buffer_size_lpddr2;
	}

	return ret;
}

int lpddr2_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_lpddr2;
	temp_lpddr2 = 0;

	if ( sscanf(buffer,"%d",&temp_lpddr2) < 1 )
		return procfs_buffer_size_lpddr2;

	if ( temp_lpddr2 < LPDDR2MINLOW || temp_lpddr2 > LPDDR2MINHIGH )
		return procfs_buffer_size_lpddr2;

	procfs_buffer_size_lpddr2 = count;

	if (procfs_buffer_size_lpddr2 > LPDDR2_PROCFS_SIZE) {
		procfs_buffer_size_lpddr2 = LPDDR2_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_lpddr2, buffer, procfs_buffer_size_lpddr2)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_lpddr2,"%u",&NVRM_AP20_LPDDR2_MIN_KHZ);
	return procfs_buffer_size_lpddr2;
}

static int __init init_lpddr2_procsfs(void) {
	LPDDR2_Proc_File = spica_add(LPDDR2_PROCFS_NAME);

	if (LPDDR2_Proc_File == NULL) {
		spica_remove(LPDDR2_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", LPDDR2_PROCFS_NAME);
		return -ENOMEM;
	} else {
		LPDDR2_Proc_File->read_proc	= lpddr2_procfile_read;
		LPDDR2_Proc_File->write_proc	= lpddr2_procfile_write;
		LPDDR2_Proc_File->mode		= S_IFREG | S_IRUGO;
		LPDDR2_Proc_File->uid		= 0;
		LPDDR2_Proc_File->gid		= 0;
		LPDDR2_Proc_File->size		= 37;
		sprintf(procfs_buffer_lpddr2,"%d",NVRM_AP20_LPDDR2_MIN_KHZ);
		procfs_buffer_size_lpddr2	= strlen(procfs_buffer_lpddr2);
	}

	return 0;
}

module_init(init_lpddr2_procsfs);

static void __exit cleanup_lpddr2_procsfs(void) {
	spica_remove(LPDDR2_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", LPDDR2_PROCFS_NAME);
}

module_exit(cleanup_lpddr2_procsfs);
