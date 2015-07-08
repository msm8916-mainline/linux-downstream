/*
 *  linux/arch/arm/mach-mmp/board-info.c
 *
 *  Support for the Marvell PXA988-based Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hardware_info.h>


HARDWARE_INFORMATION_OUTPUT_1 HARDWARE_INFORMATION_OUTPUT[MAX_HW_INFO_DEVICE];
static struct proc_dir_entry *hw_info_dir;

void hw_info_set(enum hw_info dev_id, char *type, char *id, char *ven,
    char *fw, char *cmt)
{
    if(dev_id >= MAX_HW_INFO_DEVICE)
    {
        pr_err("Error dev_id:%d, set hw info fail!!!\n", dev_id);
        return;
    }
    sprintf(HARDWARE_INFORMATION_OUTPUT[dev_id].IC_type, "%s", type);
    sprintf(HARDWARE_INFORMATION_OUTPUT[dev_id].IC_id, "%s", id);
    sprintf(HARDWARE_INFORMATION_OUTPUT[dev_id].vendor,"%s", ven);
    sprintf(HARDWARE_INFORMATION_OUTPUT[dev_id].firmware, "%s",fw);
    sprintf(HARDWARE_INFORMATION_OUTPUT[dev_id].comments,"%s", cmt);
}

static ssize_t hw_info_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char* p_str = page;
    int i = 0;

    if (*ppos)
    {
        return 0;
    }

    for(i = 0; i < MAX_HW_INFO_DEVICE; i++)
    {
        p_str += sprintf(p_str,"%s:%s:%s:%s:%s\n"
                        ,HARDWARE_INFORMATION_OUTPUT[i].IC_type
                        ,HARDWARE_INFORMATION_OUTPUT[i].IC_id
                        ,HARDWARE_INFORMATION_OUTPUT[i].vendor
                        ,HARDWARE_INFORMATION_OUTPUT[i].firmware
                        ,HARDWARE_INFORMATION_OUTPUT[i].comments);
    }
	*ppos += p_str - page;
    return p_str-page;
}

static void hardware_info_init(void)
{
    memset(&HARDWARE_INFORMATION_OUTPUT, 0, sizeof(HARDWARE_INFORMATION_OUTPUT_1)*MAX_HW_INFO_DEVICE);
}

static const struct file_operations hw_info_fops = {
	.read		= hw_info_read_proc,
};

static int create_hw_info_proc_file(void)
{
	struct proc_dir_entry *ent;

	hw_info_dir = proc_mkdir("hw_info", NULL);
	if (hw_info_dir == NULL) {
		pr_info("Unable to create /proc/hw_info directory");
		return -ENOMEM;
	}

    ent = proc_create("info", 0, hw_info_dir, &hw_info_fops);
	if (ent == NULL) {
		pr_info("Unable to create /proc/hw_info/info entry");
		goto fail;
	}
	hardware_info_init();
	return 0;

fail:
	remove_proc_entry("info", hw_info_dir);
	return ENOMEM;
}

static int __init board_info_init(void)
{
	/* create proc for hardware info */
	create_hw_info_proc_file();

	return 0;
}
module_init(board_info_init);

static void __exit board_info_exit(void)
{
	return;
}
module_exit(board_info_exit);

EXPORT_SYMBOL(HARDWARE_INFORMATION_OUTPUT);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell 88PM80x board info driver");
