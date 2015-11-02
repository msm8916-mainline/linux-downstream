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
#include <linux/camera_info.h>


CAMERA_INFORMATION_OUTPUT_1 CAMERA_INFORMATION_OUTPUT[MAX_CAMERA_INFO_DEVICE];
static struct proc_dir_entry *camera_info_dir;

void camera_info_set(enum camera_info dev_id, char *type, char *id, char *ven,
     char *cmt)
{
    if(dev_id >= MAX_CAMERA_INFO_DEVICE)
    {
        pr_err("Error dev_id:%d, set camera info fail!!!\n", dev_id);
        return;
    }
    sprintf(CAMERA_INFORMATION_OUTPUT[dev_id].IC_type, "%s", type);
    sprintf(CAMERA_INFORMATION_OUTPUT[dev_id].IC_id, "%s", id);
    sprintf(CAMERA_INFORMATION_OUTPUT[dev_id].vendor,"%s", ven);
    sprintf(CAMERA_INFORMATION_OUTPUT[dev_id].comments,"%s", cmt);
}

static ssize_t camera_info_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char* p_str = page;
    int i = 0;

    if (*ppos)
    {
        return 0;
    }

    for(i = 0; i < MAX_CAMERA_INFO_DEVICE; i++)
    {
        p_str += sprintf(p_str,"%s: %s: %s: %s\n"
                        ,CAMERA_INFORMATION_OUTPUT[i].IC_type
                        ,CAMERA_INFORMATION_OUTPUT[i].IC_id
                        ,CAMERA_INFORMATION_OUTPUT[i].vendor
                        ,CAMERA_INFORMATION_OUTPUT[i].comments);
    }
	*ppos += p_str - page;
    return p_str-page;
}

static void camera_info_init(void)
{
    memset(&CAMERA_INFORMATION_OUTPUT, 0, sizeof(CAMERA_INFORMATION_OUTPUT_1)*MAX_CAMERA_INFO_DEVICE);
}

static const struct file_operations camera_info_fops = {
	.read		= camera_info_read_proc,
};

static int create_camera_info_proc_file(void)
{
	struct proc_dir_entry *ent;

	camera_info_dir = proc_mkdir("camera_info", NULL);
	if (camera_info_dir == NULL) {
		pr_info("Unable to create /proc/camera_info directory");
		return -ENOMEM;
	}

    ent = proc_create("info", 0, camera_info_dir, &camera_info_fops);
	if (ent == NULL) {
		pr_info("Unable to create /proc/camera_info/info entry");
		goto fail;
	}
	camera_info_init();
	return 0;

fail:
	remove_proc_entry("info", camera_info_dir);
	return ENOMEM;
}

static int __init board_info_init(void)
{
	/* create proc for camera info */
	create_camera_info_proc_file();

	return 0;
}
module_init(board_info_init);

static void __exit board_info_exit(void)
{
	return;
}
module_exit(board_info_exit);

EXPORT_SYMBOL(CAMERA_INFORMATION_OUTPUT);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell 88PM80x board info driver");
