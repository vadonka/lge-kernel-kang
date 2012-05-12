#include "otf.h"

static struct proc_dir_entry *spica_dir;

/* GPU Freq */
#define GPU_PROCFS_NAME "gpufreq"
#define GPU_PROCFS_SIZE 8
static struct proc_dir_entry *GPU_Proc_File;
static char procfs_buffer_gpu[GPU_PROCFS_SIZE];
static unsigned long procfs_buffer_size_gpu = 0;

int gpu_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "gpu_procfile_read (/proc/spica/%s) called\n", GPU_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_gpu, procfs_buffer_size_gpu);
		ret = procfs_buffer_size_gpu;
	}

	return ret;
}

int gpu_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_gpu;
	temp_gpu = 0;

	if ( sscanf(buffer,"%d",&temp_gpu) < 1 )
		return procfs_buffer_size_gpu;

	if ( temp_gpu < GPULOW || temp_gpu > GPUHIGH )
		return procfs_buffer_size_gpu;

	procfs_buffer_size_gpu = count;

	if (procfs_buffer_size_gpu > GPU_PROCFS_SIZE) {
		procfs_buffer_size_gpu = GPU_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_gpu, buffer, procfs_buffer_size_gpu)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_gpu,"%u",&GPUFREQ);
	return procfs_buffer_size_gpu;
}

static int __init init_gpu_procsfs(void) {
	GPU_Proc_File = spica_add(GPU_PROCFS_NAME);

	if (GPU_Proc_File == NULL) {
		spica_remove(GPU_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", GPU_PROCFS_NAME);
		return -ENOMEM;
	} else {
		GPU_Proc_File->read_proc	= gpu_procfile_read;
		GPU_Proc_File->write_proc	= gpu_procfile_write;
		GPU_Proc_File->mode		= S_IFREG | S_IRUGO;
		GPU_Proc_File->uid		= 0;
		GPU_Proc_File->gid		= 0;
		GPU_Proc_File->size		= 37;
		sprintf(procfs_buffer_gpu,"%d",GPUFREQ);
		procfs_buffer_size_gpu		= strlen(procfs_buffer_gpu);
		printk(KERN_INFO "/proc/spica/%s created\n", GPU_PROCFS_NAME);
	}

	return 0;
}

module_init(init_gpu_procsfs);

static void __exit cleanup_gpu_procsfs(void) {
	spica_remove(GPU_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", GPU_PROCFS_NAME);
}

module_exit(cleanup_gpu_procsfs);
