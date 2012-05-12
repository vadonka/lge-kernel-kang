#include "otf.h"

/* PowerSave and Nitro */
#define PW_PROCFS_NAME "powersave"
#define PW_PROCFS_SIZE 2
#define NITRO_PROCFS_NAME "nitros"
#define NITRO_PROCFS_SIZE 2

static struct proc_dir_entry *PW_Proc_File;
static char procfs_buffer_pw[PW_PROCFS_SIZE];
static unsigned long procfs_buffer_size_pw = 0;
static struct proc_dir_entry *NITRO_Proc_File;
static char procfs_buffer_nitro[NITRO_PROCFS_SIZE];
static unsigned long procfs_buffer_size_nitro = 0;

int pw_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "pw_procfile_read (/proc/spica/%s) called\n", PW_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_pw, procfs_buffer_size_pw);
		ret = procfs_buffer_size_pw;
	}

	return ret;
}

int nitro_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "nitro_procfile_read (/proc/spica/%s) called\n", NITRO_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_nitro, procfs_buffer_size_nitro);
		ret = procfs_buffer_size_nitro;
	}

	return ret;
}

int pw_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_pw;
	temp_pw = 0;

	if ( sscanf(buffer,"%d",&temp_pw) < 0 )
		return procfs_buffer_size_pw;

	if ( temp_pw < 0 || temp_pw > 6 )
		return procfs_buffer_size_pw;

	procfs_buffer_size_pw = count;

	if (procfs_buffer_size_pw > PW_PROCFS_SIZE) {
		procfs_buffer_size_pw = PW_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_pw, buffer, procfs_buffer_size_pw)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_pw,"%u",&PWONOFF);
	return procfs_buffer_size_pw;
}

int nitro_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_nitro;
	temp_nitro = 0;

	if ( sscanf(buffer,"%d",&temp_nitro) < 0 )
		return procfs_buffer_size_nitro;

	if ( temp_nitro < 0 || temp_nitro > 1 )
		return procfs_buffer_size_nitro;

	procfs_buffer_size_nitro = count;

	if (procfs_buffer_size_nitro > NITRO_PROCFS_SIZE) {
		procfs_buffer_size_nitro = NITRO_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_nitro, buffer, procfs_buffer_size_nitro)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_nitro,"%u",&NITROONOFF);
	return procfs_buffer_size_nitro;
}

static int __init init_pw_procsfs(void) {
	PW_Proc_File = spica_add(PW_PROCFS_NAME);

	if (PW_Proc_File == NULL) {
		spica_remove(PW_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", PW_PROCFS_NAME);
		return -ENOMEM;
	} else {
		PW_Proc_File->read_proc		= pw_procfile_read;
		PW_Proc_File->write_proc	= pw_procfile_write;
		PW_Proc_File->mode		= S_IFREG | S_IRUGO;
		PW_Proc_File->uid		= 0;
		PW_Proc_File->gid		= 0;
		PW_Proc_File->size		= 37;
		sprintf(procfs_buffer_pw,"%d",PWONOFF);
		procfs_buffer_size_pw		= strlen(procfs_buffer_pw);
		printk(KERN_INFO "/proc/spica/%s created\n", PW_PROCFS_NAME);
	}

	return 0;
}

module_init(init_pw_procsfs);

static int __init init_nitro_procsfs(void) {
	NITRO_Proc_File = spica_add(NITRO_PROCFS_NAME);

	if (NITRO_Proc_File == NULL) {
		spica_remove(NITRO_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", NITRO_PROCFS_NAME);
		return -ENOMEM;
	} else {
		NITRO_Proc_File->read_proc	= nitro_procfile_read;
		NITRO_Proc_File->write_proc	= nitro_procfile_write;
		NITRO_Proc_File->mode		= S_IFREG | S_IRUGO;
		NITRO_Proc_File->uid		= 0;
		NITRO_Proc_File->gid		= 0;
		NITRO_Proc_File->size		= 37;
		sprintf(procfs_buffer_nitro,"%d",NITROONOFF);
		procfs_buffer_size_nitro	= strlen(procfs_buffer_nitro);
		printk(KERN_INFO "/proc/spica/%s created\n", NITRO_PROCFS_NAME);
}

	return 0;
}

module_init(init_nitro_procsfs);

static void __exit cleanup_pw_procsfs(void) {
	spica_remove(PW_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", PW_PROCFS_NAME);
}

module_exit(cleanup_pw_procsfs);

static void __exit cleanup_nitro_procsfs(void) {
	spica_remove(NITRO_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", NITRO_PROCFS_NAME);
}

module_exit(cleanup_nitro_procsfs);
