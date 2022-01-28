#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include "tfa98xx-debug-common.h"

#define AM_DEV_NAME   "tfa98xx"

struct tfa98xx_msg {
	char msgs[256];
    char reserved[252];
    int msg_result;
};

#define TFA_CTL_IOC_MAGIC  'T'
#define TFA_IOCTL_SPK_REST  _IOW(TFA_CTL_IOC_MAGIC, 0x01, int)
#define TFA_IOCTL_SPK_INTS   _IOR(TFA_CTL_IOC_MAGIC, 0x02, struct tfa98xx_msg)
#define TFA_IOCTL_SPK_INTT  _IOW(TFA_CTL_IOC_MAGIC, 0x03, int)
#define TFA_IOCTL_SPK_RFDES 	_IOR(TFA_CTL_IOC_MAGIC, 0x04, struct tfa98xx_msg)
#define TFA_IOCTL_SPK_CHCK _IOR(TFA_CTL_IOC_MAGIC, 0x05, int)

extern int tfa98xx_reset_mtp_dbg(void);
extern int tfa98xx_check_mtp_dbg(void);
extern int tfa98xx_init_dbg(char *buffer, int size);
extern int tfa98xx_read_freq_dbg(char *buffer, int size);


struct i2c_client * tfa98xx_debug_client;

static ssize_t tfa98xx_debug_read (struct file *file,
	char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;
	struct i2c_client *client = tfa98xx_debug_client;
	
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	else printk("[TFA98xx]%s: transfer error %d\n", __func__, ret);		
	kfree(tmp);
	return ret;
}
 
static ssize_t tfa98xx_debug_write (struct file *file,
	const char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;
	struct i2c_client *client = tfa98xx_debug_client;
	
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) 
		return PTR_ERR(tmp);
	ret = i2c_master_send(client, tmp, count);
	if(ret < 0)
	  printk("[TFA98xx]%s: transfer error %d\n", __func__, ret);	
	kfree(tmp);
	return ret;
}
static long  tfa98xx_debug_ioctl (struct file *file,
	unsigned int cmd, unsigned long arg)
{
    int  ret = 0, check = 0;
	char *tbuf;
	struct tfa98xx_msg msg;
	struct tfa98xx_msg __user *_msg = (struct tfa98xx_msg __user *)arg;

	memset(&msg, 0, sizeof(struct tfa98xx_msg));
    switch(cmd) {
	/* Reset MTP */
	case TFA_IOCTL_SPK_REST:
		printk("tfa98xx_ioctl SPK_REST\n");
		tfa98xx_reset_mtp_dbg();
		break;
	/* calibrate */
	case TFA_IOCTL_SPK_INTS:
		printk("tfa98xx_ioctl SPK_INTS\n");
		tbuf = (char *)kzalloc(sizeof(struct tfa98xx_msg), GFP_KERNEL);
		check = tfa98xx_init_dbg(tbuf, sizeof(struct tfa98xx_msg));
		ret = copy_to_user((char __user *)_msg, tbuf, sizeof(struct tfa98xx_msg));
		_msg->msg_result = check;
		break;
	case TFA_IOCTL_SPK_INTT:

		printk("tfa98xx_ioctl SPK_INT\n");
		break;
		
	case TFA_IOCTL_SPK_RFDES:
		usleep(10*1000);
        printk("tfa98xx_ioctl SPK_ReadFDes\n");
		tbuf = (char *)kzalloc(sizeof(struct tfa98xx_msg), GFP_KERNEL);
        ret = tfa98xx_read_freq_dbg(tbuf, sizeof(struct tfa98xx_msg));
		ret = copy_to_user((char __user *)_msg, tbuf, sizeof(struct tfa98xx_msg));
		break;
	/* checkmtp */
	case TFA_IOCTL_SPK_CHCK:
		printk("tfa98xx_ioctl SPK Check MtpEx\n");
		check = tfa98xx_check_mtp_dbg();
		pr_info("%s check %d.\n", __func__, check);
		ret = copy_to_user((__user int*)arg, &check, sizeof(int));
		break;
	default:
		printk("tfa98xx Fail IOCTL command no such ioctl cmd = %x\n", cmd);
		ret = -1;
		break;
    }

    return ret;
}

static int tfa98xx_debug_open(
	struct inode *inode, struct file *file)
{
	printk("[TFA98xx]%s\n", __func__);
	return 0;
}

int tfa98xx_debug_release(
	struct inode *inode, struct file *file)
{
	printk("[TFA98xx]%s\n", __func__);
	return 0;
}

static const struct file_operations tfa98xx_debug_fileops = {
	.owner = THIS_MODULE,
	.open  = tfa98xx_debug_open,
	.read  = tfa98xx_debug_read,
	.write = tfa98xx_debug_write,
	.unlocked_ioctl = tfa98xx_debug_ioctl,
	.release = tfa98xx_debug_release,
};

static struct miscdevice tfa98xx_debug_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AM_DEV_NAME,
	.fops = &tfa98xx_debug_fileops,
};

int tfa98xx_debug_probe(struct i2c_client *client)
{
	int err = 0;

	printk("%s\n",__func__);

	tfa98xx_debug_client = client;

	err = misc_register(&tfa98xx_debug_device);
	if (err) {
		printk("%s: tfa98xx_device register failed\n", __func__);
		return err;
	}

	return 0;
}

MODULE_DESCRIPTION("NXP TFA98xx debug driver");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_LICENSE("GPL");
