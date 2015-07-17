/* drivers/input/tct_common.c
*
* Provide some debug interface for input devices.
*
* Version:1.0
*		 V1.0:2014/05/27,create file.	BY TCTSZ-WH.
* Version:1.1
*		 V1.1:2014/05/29,add entry tp_firmware_ver .	BY TCTSZ-weihong.chen.
*/

#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>

//wxc #define HAVE_HALL 1
#define HAVE_HALL 0
#if HAVE_HALL
#include "misc/hall.h"
#endif
struct proc_dir_entry *proc_dir;
static struct proc_dir_entry *tp_dir,*tp_device_name,*tp_config_ver,*tp_firmware_ver,*tp_wakeup_gesture,*tp_debug_on,*tp_palm_lock;
#if HAVE_HALL
static struct proc_dir_entry *hall_dir,*hall_status;
#endif
char* g_tp_device_name;
int g_tp_cfg_ver =0;
int g_tp_firmware_ver =0;
//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen, 2014/06/03, add a attribute to control disable/enable 
//wake_gesture(dobule click to  light on screen and palm to  light off  screen )
u8 g_wakeup_gesture = 0;	/*MOD by TCTSZ-WH, Disable by default. 1-on, 0-off.*/
//[FEATURE]-Add-END by TCTSZ. weihong.chen,
u8 g_tp_debug_on = 0;

#if HAVE_HALL
extern struct hall_data *hall;
#endif
u8 g_palm_lock_switch = 0;	/*ADD by TCTSZ-WH, Switch on/off palm lock, 1-on, 0-off.*/

static int tp_device_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", g_tp_device_name);
	return 0;
}

static int tp_device_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, tp_device_proc_show, NULL);
}

static int tp_cfg_ver_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_tp_cfg_ver);
	return 0;
}

static int tp_cfg_ver_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, tp_cfg_ver_proc_show, NULL);
}
//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen,PR-674715 2014/05/29, add show firmware version

static int tp_firmware_ver_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_tp_firmware_ver);
	return 0;
}

static int tp_firmware_ver_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, tp_firmware_ver_proc_show, NULL);
}
//[FEATURE]-Add-END by TCTSZ. weihong.chen,

static const struct file_operations tp_device_name_fops = {
	.open 		= tp_device_open_proc,
	.read 		= seq_read,
	.owner 		= THIS_MODULE,
};

static const struct file_operations tp_cfg_ver_fops = {
	.open 		= tp_cfg_ver_open_proc,
	.read 		= seq_read,
	.owner 		= THIS_MODULE,
};
//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen, 2014/06/03, add a attribute to control disable/enable 
//wake_gesture(dobule click to  light on screen and palm to  light off  screen )
static int tp_wakeup_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", g_wakeup_gesture);
	return 0;
}

static int tp_wakeup_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_wakeup_proc_show, NULL);
}

static ssize_t tp_wakeup_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int err;
	err = kstrtou8_from_user(buffer, count, 0, &g_wakeup_gesture);
	if (err)
		return err;
	else
		return count;
}
//[FEATURE]-Add-END by TCTSZ. weihong.chen

/*ADD Begin by TCTSZ.WH,2014-6-16,Add tp debug switch.*/
static int tp_debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", g_tp_debug_on);
	return 0;
}

static int tp_debug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_debug_proc_show, NULL);
}

static ssize_t tp_debug_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int err;
	err = kstrtou8_from_user(buffer, count, 0, &g_tp_debug_on);
	if (err)
		return err;
	else
		return count;
}
/*ADD Begin by TCTSZ.WH,2014-6-16,Add tp debug switch.*/

#if HAVE_HALL
static int hall_status_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", !gpio_get_value(hall->irq_gpio));
	return 0;
}

static int hall_status_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, hall_status_proc_show, NULL);
}
#endif
static int tp_palm_lock_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n", g_palm_lock_switch);
	return 0;
}

static int tp_palm_lock_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_palm_lock_proc_show, NULL);
}

static ssize_t tp_palm_lock_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int err;
	err = kstrtou8_from_user(buffer, count, 0, &g_palm_lock_switch);
	if (err)
		return err;
	else
		return count;
}

//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen,PR-674715 2014/05/29, add show firmware version

static const struct file_operations tp_firmware_ver_fops = {
	.open 		= tp_firmware_ver_open_proc,
	.read 		= seq_read,
	.owner 		= THIS_MODULE,
};
//[FEATURE]-Add-END by TCTSZ. weihong.chen,

//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen, 2014/06/03, add a attribute to control disable/enable 
//wake_gesture(dobule click to  light on screen and palm to  light off  screen )

static const struct file_operations tp_wakeup_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= tp_wakeup_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= tp_wakeup_proc_write,
};
//[FEATURE]-Add-END by TCTSZ. weihong.chen,

static const struct file_operations tp_debug_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= tp_debug_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= tp_debug_proc_write,
};

#if HAVE_HALL
static const struct file_operations hall_status_fops = {
	.open 		= hall_status_open_proc,
	.read 		= seq_read,
	.owner 		= THIS_MODULE,
};
#endif
static const struct file_operations tp_palm_lock_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= tp_palm_lock_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= tp_palm_lock_proc_write,
};

static int __init tct_debug_init(void)
{
	proc_dir = proc_mkdir("tct_debug",NULL);
	tp_dir = proc_mkdir("tp",proc_dir);
	tp_device_name = proc_create("tp_device",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			tp_dir,
			&tp_device_name_fops);
	if (tp_device_name == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}

	tp_config_ver = proc_create("tp_config_ver",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			tp_dir,
			&tp_cfg_ver_fops);
	if (tp_config_ver == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}
	//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen,PR-674715 2014/05/29, add show firmware version
	tp_firmware_ver = proc_create("tp_firmware_ver",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			tp_dir,
			&tp_firmware_ver_fops);
	if (tp_firmware_ver == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}
	//[FEATURE]-Add-END by TCTSZ. weihong.chen,

	//[FEATURE]-Add-BEGIN by TCTSZ. weihong.chen, 2014/06/03, add a attribute to control disable/enable
	//wake_gesture(dobule click to  light on screen and palm to  light off  screen )
	tp_wakeup_gesture = proc_create("wakeup_gesture",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			tp_dir,
			&tp_wakeup_proc_fops);
	if (tp_wakeup_gesture == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}
	//[FEATURE]-Add-END by TCTSZ. weihong.chen,

	tp_debug_on = proc_create("tp_debug_on",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			tp_dir,
			&tp_debug_proc_fops);
	if (tp_debug_on == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}

	tp_palm_lock = proc_create("tp_palm_lock",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			tp_dir,
			&tp_palm_lock_proc_fops);
	if (tp_palm_lock == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}

#if HAVE_HALL
	hall_dir = proc_mkdir("hall",proc_dir);

	hall_status = proc_create("hall_status",
			S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH | S_IWOTH,
			hall_dir,
			&hall_status_fops);
	if (hall_status == NULL) {
		pr_err("Couldn't create proc entry!");
		return -1;
	}
#endif
	return 0;
}


static void __exit tct_debug_exit(void)
{
	proc_remove(tp_device_name);
	proc_remove(tp_config_ver);
	proc_remove(tp_wakeup_gesture);
	proc_remove(tp_debug_on);

#if HAVE_HALL
	proc_remove(hall_status);
#endif
	proc_remove(tp_palm_lock);
}

module_init(tct_debug_init);
module_exit(tct_debug_exit);
MODULE_LICENSE("GPL");

