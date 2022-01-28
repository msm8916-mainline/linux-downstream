/*
 * mach5 driver
 *
 * Copyright (c) 2014 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.	In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *	claim that you wrote the original software. If you use this software
 *	in a product, an acknowledgment in the product documentation would be
 *	appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *	misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#include "vivo-codec-common.h"
#include "spi-mach5-common.h"

#ifdef CONFIG_VIVO_REGDUMP
	#include <linux/vivo-regdump.h>
	static int mach5_clock_on_flag = 0;
	static int mach5_power_on_flag = 0;
	static int mach5_mic_swch_flag = 0;
	static int mach5_hp_swch_flag = 0;
#endif


static struct cdev mach5_cdev;
static struct class *mach5_class;
static int dev_major;
static int dev_mainor;
static dev_t devt;
struct spi_device *mach5_spi;

struct vivo_codec_function *function; //zsy M

//extern int vivo_codec_set_hp_path(int path);
//extern int vivo_codec_set_mic_path(int path);

static int spi_rw(u8 *tx, u8 *rx, int len)
{
	struct spi_message spi_msg;
	struct spi_transfer spi_xfer;

	/* Initialize SPI ,message */
	spi_message_init(&spi_msg);

	/* Initialize SPI transfer */
	memset(&spi_xfer, 0, sizeof spi_xfer);
	spi_xfer.len = len;
	spi_xfer.tx_buf	= tx;
	spi_xfer.rx_buf	= rx;

	/* Add SPI transfer to SPI message */
	spi_message_add_tail(&spi_xfer, &spi_msg);

	/* Perform synchronous SPI transfer */
	if (spi_sync(mach5_spi, &spi_msg)) {
		dev_err(&mach5_spi->dev, "spi_sync failure\n");
		return -EIO;
	}

	return 0;
}

#define FIX_BUFF_SIZE	(128)

static int ioctl_read_reg(unsigned long arg)
{
	u8 *rx = NULL;
	u8 buf[FIX_BUFF_SIZE];
	u8 *read_buf = buf;
	int err = 0;
	struct read_reg reg;

	if (copy_from_user(&reg, (void *)arg, sizeof(reg)) != 0)
		return -EFAULT;

	if ((reg.size+2) > sizeof(buf)) {
		rx = kmalloc(reg.size+2, GFP_KERNEL);
		if (rx == NULL)
			return -EFAULT;
		read_buf = rx;
	}

	read_buf[0] = (u8)(reg.address<<1) | 0x80;
	if (reg.size > 1)
		read_buf[0] |= 0x01;	/* burst */

	spi_rw(read_buf, read_buf + 2, reg.size + 1);

	if (copy_to_user((void *)reg.data, (void *)&read_buf[3], reg.size)
									!= 0)
		err = -EFAULT;

	if (rx != NULL)
		kfree(rx);

	return err;
}

static int ioctl_write_reg(unsigned long arg)
{
	int err = 0;
	u8 *tx = NULL;
	u8 buf[FIX_BUFF_SIZE];
	u8 *write_buf = buf;
	struct write_reg reg;

	if (copy_from_user(&reg, (void *)arg, sizeof(reg)) != 0)
		return -EFAULT;

	if (reg.size > sizeof(buf)) {
		tx = kmalloc(reg.size, GFP_KERNEL);
		if (tx == NULL)
			return -EFAULT;
		write_buf = tx;
	}

	if (copy_from_user(write_buf, (void *)reg.data, reg.size) != 0)
		err = -EFAULT;

	spi_rw((u8 *)write_buf, NULL, reg.size);

	if (tx != NULL)
		kfree(tx);

	return 0;
}

static int mach5_clock_on(void)
{
	printk("ktv mach5_clock_on\n");
	if (function->mclk_enable)
	{
		function->mclk_enable(VIVO_CODEC_KTV, 19200000);
	#ifdef CONFIG_VIVO_REGDUMP
		mach5_clock_on_flag= 1;
	#endif
	}
	else
		printk("mach5_clock_on failed\n");
/*
	if (function->power_up)
		function->power_up(VIVO_CODEC_KTV);*/
	return 0;
}

static int mach5_clock_off(void)
{
	printk("ktv mach5_clock_off\n");
	if (function->mclk_disable)
	{
		function->mclk_disable(VIVO_CODEC_KTV);
	#ifdef CONFIG_VIVO_REGDUMP
		mach5_clock_on_flag= 0;
	#endif
	}
	else
		printk("mach5_clock_off failed\n");
	return 0;
}
static long mach5_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	switch (cmd) {
	case MACH5_IOCTL_READ_REG:
		return ioctl_read_reg(arg);
	case MACH5_IOCTL_WRITE_REG:
		return ioctl_write_reg(arg);
	case MACH5_IOCTL_CLOCK_ON:
		return mach5_clock_on();
	case MACH5_IOCTL_CLOCK_OFF:
		return mach5_clock_off();
	case MACH5_IOCTL_LDOD_ON:
		if (function->power_up)
			{
				function->power_up(VIVO_CODEC_KTV);
			#ifdef CONFIG_VIVO_REGDUMP
				mach5_power_on_flag = 1;
			#endif
			}
		else
			printk("ktv ldo enable failed\n");
		break;
	case MACH5_IOCTL_LDOD_OFF:
		if (function->power_down)
			{
				function->power_down(VIVO_CODEC_KTV);
				#ifdef CONFIG_VIVO_REGDUMP
					mach5_power_on_flag = 0;
				#endif
			}
		else
			printk("ktv ldo disable failed\n");
		break;
	case MACH5_IOCTL_MIC_SWCH_ON:
		{
			#ifdef CONFIG_VIVO_REGDUMP
				mach5_mic_swch_flag= 1;
			#endif
			printk("ktv_ctl_ioctl MACH5_IOCTL_MIC_SWCH_ON\n");
			//vivo_codec_set_mic_path(MIC_PATH_KTV);
			break;
		}
	case MACH5_IOCTL_MIC_SWCH_OFF:
		{
			#ifdef CONFIG_VIVO_REGDUMP
				mach5_mic_swch_flag= 0;
			#endif
			printk("ktv_ctl_ioctl MACH5_IOCTL_MIC_SWCH_OFF\n");
			//vivo_codec_set_mic_path(MIC_PATH_NORMAL);
			break;
		}
	case MACH5_IOCTL_HP_SWCH_ON:
		{
			#ifdef CONFIG_VIVO_REGDUMP
				mach5_hp_swch_flag= 1;
			#endif
			printk("ktv_ctl_ioctl MACH5_IOCTL_HP_SWCH_ON\n");
			//vivo_codec_set_hp_path(HP_PATH_KTV);
			break;
		}
	case MACH5_IOCTL_HP_SWCH_OFF:
		{
			#ifdef CONFIG_VIVO_REGDUMP
				mach5_mic_swch_flag= 0;
			#endif
			printk("ktv_ctl_ioctl MACH5_IOCTL_HP_SWCH_OFF\n");
			//vivo_codec_set_hp_path(HP_PATH_OFF);
			break;
		}
	default:
		printk("%s:not such cmd :%d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static int ioctl_read_reg_compat(struct read_reg32 __user * arg32)
{
	u8 *rx = NULL;
	u8 buf[FIX_BUFF_SIZE];
	u8 *read_buf = buf;
	int err = 0;
	compat_caddr_t _buf;
	u32 addr, size;

	if (get_user(_buf, &arg32->data) ||
	    get_user(addr, &arg32->address) ||
	    get_user(size, &arg32->size))
		return -EFAULT;

	//printk("size %d\n", size);

	if ((size+2) > sizeof(buf)) {
		rx = kmalloc(size+2, GFP_KERNEL);
		if (rx == NULL)
			return -EFAULT;
		read_buf = rx;
	}

	read_buf[0] = (u8)(addr<<1) | 0x80;
	if (size > 1)
		read_buf[0] |= 0x01;	/* burst */

	spi_rw(read_buf, read_buf + 2, size + 1);

	err = copy_to_user((u8 __user *)(compat_ptr(_buf)), &read_buf[3], size);

	if (err!= 0) {
		printk("mach5 ioctl_read_reg copy to user error:%d\n",err);
		err = -EFAULT;
	}

	if (rx != NULL)
		kfree(rx);

	return err;
}

static int ioctl_write_reg_compat(struct write_reg32 __user * arg32)
{
	int err = 0;
	u8 *tx = NULL;
	u8 buf[FIX_BUFF_SIZE];
	u8 *write_buf = buf;
	compat_caddr_t _buf;
	u32 size;

	if (get_user(_buf, &arg32->data) ||
	    get_user(size, &arg32->size))
		return -EFAULT;

	printk("size %d\n", size);

	if (size > sizeof(buf)) {
		tx = kmalloc(size, GFP_KERNEL);
		if (tx == NULL)
			return -EFAULT;
		write_buf = tx;
	}

	if (copy_from_user(write_buf, (u8 __user *)(compat_ptr(_buf)), size) != 0)
		err = -EFAULT;

	spi_rw((u8 *)write_buf, NULL, size);

	if (tx != NULL)
		kfree(tx);

	return 0;
}

static long mach5_ioctl_compat(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = compat_ptr(arg);

	//printk("%s:cmd :%d\n", __func__, cmd);

	if((cmd & 0xFFFF) == (MACH5_IOCTL_READ_REG & 0xFFFF))
		cmd = MACH5_IOCTL_READ_REG;
	if((cmd & 0xFFFF) == (MACH5_IOCTL_WRITE_REG & 0xFFFF))
		cmd = MACH5_IOCTL_WRITE_REG;	

	switch (cmd) {
	case MACH5_IOCTL_READ_REG:
		return ioctl_read_reg_compat(argp);
	case MACH5_IOCTL_WRITE_REG:
		return ioctl_write_reg_compat(argp);
	case MACH5_IOCTL_CLOCK_ON:
		return mach5_clock_on();
	case MACH5_IOCTL_CLOCK_OFF:
		return mach5_clock_off();
		case MACH5_IOCTL_LDOD_ON:
			if (function->power_up)
				{
					function->power_up(VIVO_CODEC_KTV);
				#ifdef CONFIG_VIVO_REGDUMP
					mach5_power_on_flag = 1;
				#endif
				}
			else
				printk("ktv ldo enable failed\n");
			break;
		case MACH5_IOCTL_LDOD_OFF:
			if (function->power_down)
				{
					function->power_down(VIVO_CODEC_KTV);
				#ifdef CONFIG_VIVO_REGDUMP
						mach5_power_on_flag = 0;
				#endif
				}
			else
				printk("ktv ldo disable failed\n");
			break;
		case MACH5_IOCTL_MIC_SWCH_ON:
			{
				#ifdef CONFIG_VIVO_REGDUMP
					mach5_mic_swch_flag= 1;
				#endif
				printk("ktv_ctl_ioctl MACH5_IOCTL_MIC_SWCH_ON\n");
				//vivo_codec_set_mic_path(MIC_PATH_KTV);
				break;
			}
		case MACH5_IOCTL_MIC_SWCH_OFF:
			{
				#ifdef CONFIG_VIVO_REGDUMP
					mach5_mic_swch_flag= 0;
				#endif
				printk("ktv_ctl_ioctl MACH5_IOCTL_MIC_SWCH_OFF\n");
				//vivo_codec_set_mic_path(MIC_PATH_NORMAL);
				break;
			}
		case MACH5_IOCTL_HP_SWCH_ON:
			{
				#ifdef CONFIG_VIVO_REGDUMP
					mach5_hp_swch_flag= 1;
				#endif
				printk("ktv_ctl_ioctl MACH5_IOCTL_HP_SWCH_ON\n");
				//vivo_codec_set_hp_path(HP_PATH_KTV);
				break;
			}
		case MACH5_IOCTL_HP_SWCH_OFF:
			{
				#ifdef CONFIG_VIVO_REGDUMP
					mach5_mic_swch_flag= 0;
				#endif
				printk("ktv_ctl_ioctl MACH5_IOCTL_HP_SWCH_OFF\n");
				//vivo_codec_set_hp_path(HP_PATH_OFF);
				break;
			}
	default:
		printk("%s:not such cmd :%d\n", __func__, cmd);
		return 0;
	}

	return 0;
}

#define mach5_ioctl_compat_fun	mach5_ioctl_compat
#else
#define mach5_ioctl_compat_fun	NULL
#endif
const struct file_operations mach5_fops = {
	.unlocked_ioctl = mach5_ioctl,
	.compat_ioctl = mach5_ioctl_compat_fun,
};

#ifdef CONFIG_VIVO_REGDUMP
void mach5_regdump_callback(void)
{
	pr_info("%s() enter\n", __func__);
	pr_info("%s() clock %s\n", __func__ ,mach5_clock_on_flag?"on":"off");
	pr_info("%s() power %s\n", __func__ ,mach5_power_on_flag?"on":"off");
	pr_info("%s() mic %s\n", __func__ ,mach5_mic_swch_flag?"ktv":"normal");
	pr_info("%s() headset %s\n", __func__ ,mach5_hp_swch_flag?"ktv":"normal");
	pr_info("%s() leave\n", __func__);
	return;
}
static struct vivo_regdump_handler mach5_regdump_handler = {
	.name = "mach5 ktv",
	.callback = mach5_regdump_callback,
};
#endif

static int mach5_spi_probe(struct spi_device *spi)
{
	//struct vivo_codec_function *function; //zsy M
	int ret;
	dev_t dev;
	struct device *mach5_dev;
	unsigned char reg, read[4], write[4];

	dev_err(&spi->dev, "mach5_spi_probe()\n");

	function = get_vivo_codec_function();
	if (!function) {
		dev_err(&spi->dev, "%s vivo_codec_function is NULL\n",
			__func__);
		return -EINVAL;
	}

	if (function->mclk_enable)
		function->mclk_enable(VIVO_CODEC_KTV, 19200000);

	if (function->power_up)
		function->power_up(VIVO_CODEC_KTV);

	ret = alloc_chrdev_region(&dev, 0, 1, "mach5");
	if (ret)
		goto err_alloc_chrdev;

	dev_major = MAJOR(dev);
	dev_mainor = 0;
	devt = MKDEV(dev_major, dev_mainor);

	cdev_init(&mach5_cdev, &mach5_fops);
	mach5_cdev.owner = THIS_MODULE;
	mach5_cdev.ops = &mach5_fops;

	ret = cdev_add(&mach5_cdev, devt, 1);
	if (ret)
		goto err_cdev_add;

	mach5_class = class_create(THIS_MODULE, "mach5");
	if (IS_ERR(mach5_class))
		goto err_class_create;

	mach5_dev = device_create(
		mach5_class,
		NULL,
		devt,
		NULL,
		"mach5_spi");

	if (IS_ERR(mach5_dev))
		goto err_device_create;

	mach5_spi = spi;

	/* write 0 to register 2 to disable hw reset */
	reg = 2;
	write[0] = (0 << 7) | (reg << 1) | (0 << 0);
	write[1] = 0;
	spi_rw(write, read, 2);

	/* write 0 to register 3 to disable sw reset */
	reg = 3;
	write[0] = (0 << 7) | (reg << 1) | (0 << 0);
	write[1] = 0;
	spi_rw(write, read, 2);

	/* read ID */
	/* wirte register 0 to tell mach 5 that we need to read register 0(ID) of A_REG */
	reg = 0;
	write[0] = (0 << 7) | (reg << 1) | (0 << 0);
	write[1] = 1 << 7 | 0 << 0;
	reg = 1;
	/* read register 1 to get the value of register 0 in A_REG */
	write[2] = (1 << 7) | (reg << 1) | (0 << 0);
	spi_rw(write, read, 4);
	dev_err(&spi->dev, "mach5_spi_probe() : read ID 0x%x\n", read[3]);
	
	if (function->mclk_disable)
		function->mclk_disable(VIVO_CODEC_KTV);

	if (function->power_down)
		function->power_down(VIVO_CODEC_KTV);
		
	if (read[3] != 0x81) {
		dev_err(&spi->dev, "mach5_spi_probe() : read ID error!\n");
		goto err_id;
	}

#ifdef CONFIG_VIVO_REGDUMP
	vivo_audio_regdump_register_callback(&mach5_regdump_handler);
#endif
	
//	goto err_id;
	return 0;
//err_id:
	//mach5_spi = NULL;
err_device_create:
	class_destroy(mach5_class);
err_class_create:
	cdev_del(&mach5_cdev);
err_cdev_add:
	unregister_chrdev_region(devt, 1);
err_alloc_chrdev:
err_id:
	return -1;
}

static int mach5_spi_remove(struct spi_device *spi)
{
	device_destroy(mach5_class, devt);

	class_destroy(mach5_class);

	cdev_del(&mach5_cdev);

	unregister_chrdev_region(devt, 1);

#ifdef CONFIG_VIVO_REGDUMP
	vivo_audio_regdump_deregister_callback(mach5_regdump_handler.name);
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id device_mach5_of_match[] = {
    {.compatible = "yamaha,mach5",},
    {},
};
#else
#define device_mach5_of_match 0
#endif

static const struct spi_device_id mach5_id[] = {
	{ "mach5" },
	{}
};
MODULE_DEVICE_TABLE(spi, mach5_id);

static struct spi_driver mach5_spi_driver = {
	.driver = {
		.name = "mach5",
		.of_match_table = device_mach5_of_match,
		.owner = THIS_MODULE,
	},
	.probe = mach5_spi_probe,
	.remove = mach5_spi_remove,
	.id_table = mach5_id,
};
module_spi_driver(mach5_spi_driver);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_DESCRIPTION("Yamaha MACH5 spi driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.8.0");
