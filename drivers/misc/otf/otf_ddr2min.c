#include "otf.h"

static struct proc_dir_entry *spica_dir;

/* DDR2 Min Khz */
#define DDR2_PROCFS_NAME "ddr2_min_khz"
#define DDR2_PROCFS_SIZE 6
static unsigned long procfs_buffer_size_ddr2 = 0;
static struct proc_dir_entry *DDR2_Proc_File;
static char procfs_buffer_ddr2[DDR2_PROCFS_SIZE];
extern unsigned int DDR2MINLOW;
extern unsigned int DDR2MINHIGH;

int ddr2_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "procfile_read (/proc/spica/%s) called\n", DDR2_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_ddr2, procfs_buffer_size_ddr2);
		ret = procfs_buffer_size_ddr2;
	}

	return ret;
}

int ddr_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_ddr2;
	temp_ddr2 = 0;

	if ( sscanf(buffer,"%d",&temp_ddr2) < 1 )
		return procfs_buffer_size_ddr2;

	if ( temp_ddr2 < DDR2MINLOW || temp_ddr2 > DDR2MINHIGH )
		return procfs_buffer_size_ddr2;

	procfs_buffer_size_ddr2 = count;

	if (procfs_buffer_size_ddr2 > DDR2_PROCFS_SIZE) {
		procfs_buffer_size_ddr2 = DDR2_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_ddr2, buffer, procfs_buffer_size_ddr2)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_ddr2,"%u",&NVRM_AP20_DDR2_MIN_KHZ);
	return procfs_buffer_size_ddr2;
}

static int __init init_ddr2_procsfs(void) {
	DDR2_Proc_File = spica_add(DDR2_PROCFS_NAME);

	if (DDR2_Proc_File == NULL) {
		spica_remove(DDR2_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", DDR2_PROCFS_NAME);
		return -ENOMEM;
	} else {
		DDR2_Proc_File->read_proc	= ddr2_procfile_read;
		DDR2_Proc_File->write_proc	= ddr_procfile_write;
		DDR2_Proc_File->mode		= S_IFREG | S_IRUGO;
		DDR2_Proc_File->uid		= 0;
		DDR2_Proc_File->gid		= 0;
		DDR2_Proc_File->size		= 37;
		sprintf(procfs_buffer_ddr2,"%d",NVRM_AP20_DDR2_MIN_KHZ);
		procfs_buffer_size_ddr2		= strlen(procfs_buffer_ddr2);
		printk(KERN_INFO "/proc/spica/%s created\n", DDR2_PROCFS_NAME);
	}

	return 0;
}

module_init(init_ddr2_procsfs);

static void __exit cleanup_ddr2_procsfs(void) {
	spica_remove(DDR2_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", DDR2_PROCFS_NAME);
}

module_exit(cleanup_ddr2_procsfs);
