#include <linux/spica.h>

static struct proc_dir_entry *spica_dir;

/* AVP Freq */
#define AVP_PROCFS_NAME "avpfreq"
#define AVP_PROCFS_SIZE 8
static struct proc_dir_entry *AVP_Proc_File;
static char procfs_buffer_avp[AVP_PROCFS_SIZE];
static unsigned long procfs_buffer_size_avp = 0;

int avp_procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data) {
	int ret;
	printk(KERN_INFO "avp_procfile_read (/proc/spica/%s) called\n", AVP_PROCFS_NAME);

	if (offset > 0) {
		ret = 0;
	} else {
		memcpy(buffer, procfs_buffer_avp, procfs_buffer_size_avp);
		ret = procfs_buffer_size_avp;
	}

	return ret;
}

int avp_procfile_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	int temp_avp;
	temp_avp = 0;

	if ( sscanf(buffer,"%d",&temp_avp) < 1 )
		return procfs_buffer_size_avp;

	if ( temp_avp < AVPLOW || temp_avp > AVPHIGH )
		return procfs_buffer_size_avp;

	procfs_buffer_size_avp = count;

	if (procfs_buffer_size_avp > AVP_PROCFS_SIZE) {
		procfs_buffer_size_avp = AVP_PROCFS_SIZE;
	}

	if (copy_from_user(procfs_buffer_avp, buffer, procfs_buffer_size_avp)) {
		printk(KERN_INFO "buffer_size error\n");
		return -EFAULT;
	}

	sscanf(procfs_buffer_avp,"%u",&AVPFREQ);
	return procfs_buffer_size_avp;
}

static int __init init_avp_procsfs(void) {
	AVP_Proc_File = spica_add(AVP_PROCFS_NAME);

	if (AVP_Proc_File == NULL) {
		spica_remove(AVP_PROCFS_NAME);
		printk(KERN_ALERT "Error: Could not initialize /proc/spica/%s\n", AVP_PROCFS_NAME);
		return -ENOMEM;
	} else {
		AVP_Proc_File->read_proc	= avp_procfile_read;
		AVP_Proc_File->write_proc	= avp_procfile_write;
		AVP_Proc_File->mode		= S_IFREG | S_IRUGO;
		AVP_Proc_File->uid		= 0;
		AVP_Proc_File->gid		= 0;
		AVP_Proc_File->size		= 37;
		sprintf(procfs_buffer_avp,"%d",AVPFREQ);
		procfs_buffer_size_avp		= strlen(procfs_buffer_avp);
		printk(KERN_INFO "/proc/spica/%s created\n", AVP_PROCFS_NAME);
	}

	return 0;
}

module_init(init_avp_procsfs);

static void __exit cleanup_avp_procsfs(void) {
	spica_remove(AVP_PROCFS_NAME);
	printk(KERN_INFO "/proc/spica/%s removed\n", AVP_PROCFS_NAME);
}

module_exit(cleanup_avp_procsfs);
