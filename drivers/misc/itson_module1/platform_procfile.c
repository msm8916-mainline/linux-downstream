/*
 * platform_timestamp.c
 *
 *  Created on: Jan 19, 2015
 *      Author: Ian Smith
 */
 
#include "platform.h"
#include "platform_timestamp.h"


#ifdef __KERNEL__

#include <linux/version.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>


int
ItsOnCreateKernelFolder(char *name)
{
	struct proc_dir_entry *proc_file;

	proc_file = proc_mkdir(name, NULL);
	if (proc_file) {
		return 0;
	} else {
		return -1;
	}
}
EXPORT_SYMBOL(ItsOnCreateKernelFolder);


int
ItsOnCreateKernelFile(char *name, void *readFunc, void *writeFunc, unsigned short perm)
{
	struct proc_dir_entry *proc_file;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	struct file_operations* file_ops = kzalloc(sizeof(struct file_operations), GFP_ATOMIC);
	if (file_ops == NULL) {
		ERR("Could not allocate file_ops");
		return -1;
	}
	file_ops->owner = THIS_MODULE;
	file_ops->write = writeFunc;
	file_ops->read = readFunc;

	proc_file = proc_create(name, perm, NULL, (const struct file_operations*)file_ops);
	if (proc_file) {
		return 0;
	} else {
		return -1;
	}
#else
	proc_file = create_proc_entry(name, perm, NULL);
	if (proc_file)
	{
		proc_file->read_proc = readFunc;
		proc_file->write_proc = writeFunc;
		return 0;
	}
	else
		return -1;
#endif
}
EXPORT_SYMBOL(ItsOnCreateKernelFile);


// The following are for "simple" kernel files. These are files which
// offer a dump of data to users (e.g. status reports) but don't require
// block-based reads, seeks, ioctls, or other complicated stuff.
static int
ItsOnKernelFileSimpleOpen(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
	return single_open(file, PDE_DATA(inode), NULL);
#else
	return single_open(file, PDE(inode)->data, NULL);
#endif
}


int
ItsOnCreateKernelFileSimple(char *name, int (*showFunc)(void *, void *),
    unsigned int (*writeFunc)(struct file *, const char *, size_t, void *),
    unsigned short perm)
{
	struct proc_dir_entry *proc_file;


	struct file_operations* file_ops = kzalloc(sizeof(struct file_operations), GFP_ATOMIC);
	if (file_ops == NULL) {
		ERR("Could not allocate file_ops");
		return -1;
	}
	file_ops->owner = THIS_MODULE;
	file_ops->open = ItsOnKernelFileSimpleOpen;
	file_ops->read = seq_read;
	file_ops->llseek = seq_lseek;
	file_ops->release = single_release;
	file_ops->write = (ssize_t (*)(struct file *, const char __user *, size_t, loff_t *))writeFunc;

	proc_file = proc_create_data(name, perm, NULL, (const struct file_operations*)file_ops, showFunc);
	if (proc_file) {
		return 0;
	} else
		return -1;
}
EXPORT_SYMBOL(ItsOnCreateKernelFileSimple);


// Size of the printf buffer.  The whole stack frame can't be larger than
// 1024, so this is limited.
#define	PRINTF_BUF		1000

int
itson_proc_output(void *ksf_handle, const char *format, ...)
{
	// We don't have seq_vprintf, apparently, so we need to use a buffer...
    char buf[PRINTF_BUF];
	va_list ap;
	
    va_start(ap, format);
    // This may truncate; if so, too bad.
    vsnprintf(buf, PRINTF_BUF, format, ap);
    va_end(ap);
    
	return seq_printf((struct seq_file *) ksf_handle, "%s", buf);
}
EXPORT_SYMBOL(itson_proc_output);


void
ItsOnDeleteKernelFile(char *name)
{
	remove_proc_entry(name, NULL);
}
EXPORT_SYMBOL(ItsOnDeleteKernelFile);


#endif	// __KERNEL__



#ifdef __LINUX_UNITTESTS__
/*******************Linux User space version ****************/


int
itson_proc_output(void *ksf_handle, const char *format, ...)
{
	va_list ap;
	int stat;
	
    va_start(ap, format);
    stat = vprintf(format, ap);
    va_end(ap);

	return stat;
}


#endif	// __LINUX_UNITTESTS__

