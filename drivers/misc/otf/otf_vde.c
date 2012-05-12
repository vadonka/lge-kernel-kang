#include "otf.h"

static struct proc_dir_entry *spica_dir;

/* VDE Freq */
#define VDE_PROCFS_NAME "vdefreq"
#define VDE_PROCFS_SIZE 8
static struct proc_dir_entry *VDE_Proc_File;
static char procfs_buffer_vde[VDE_PROCFS_SIZE];
static unsigned long procfs_buffer_size_vde = 0;

int vde_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "vde_procfile_read (/proc/spica/%s) called\n", VDE_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_vde, procfs_buffer_size_vde);
		ret = procfs_buffer_size_vde;
	}

	return ret;
}

int vde_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_vde;
	temp_vde = 0;

	if ( sscanf(buffer,"%d",&temp_vde) < 1 )
		return procfs_buffer_size_vde;

	if ( temp_vde < VDELOW || temp_vde > VDEHIGH )
		return procfs_buffer_size_vde;

	procfs_buffer_size_vde = count;

	if (procfs_buffer_size_vde > VDE_PROCFS_SIZE) {
		procfs_buffer_size_vde = VDE_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_vde, buffer, procfs_buffer_size_vde)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_vde,"%u",&VDEFREQ);
	return procfs_buffer_size_vde;
}

static int __init init_vde_procsfs(void) {
	VDE_Proc_File = spica_add(VDE_PROCFS_NAME);

	if (VDE_Proc_File == NULL) {
		spica_remove(VDE_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", VDE_PROCFS_NAME);
		return -ENOMEM;
	} else {
		VDE_Proc_File->read_proc	= vde_procfile_read;
		VDE_Proc_File->write_proc	= vde_procfile_write;
		VDE_Proc_File->mode		= S_IFREG | S_IRUGO;
		VDE_Proc_File->uid		= 0;
		VDE_Proc_File->gid		= 0;
		VDE_Proc_File->size		= 37;
		sprintf(procfs_buffer_vde,"%d",VDEFREQ);
		procfs_buffer_size_vde		= strlen(procfs_buffer_vde);
		printk(KERN_INFO "/proc/spica/%s created\n", VDE_PROCFS_NAME);
	}

	return 0;
}

module_init(init_vde_procsfs);

static void __exit cleanup_vde_procsfs(void) {
	spica_remove(VDE_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", VDE_PROCFS_NAME);
}

module_exit(cleanup_vde_procsfs);
