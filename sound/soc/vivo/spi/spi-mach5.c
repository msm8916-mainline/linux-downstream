/*
 * mach5 driver
 *
 * Copyright (c) 2014-2015 Yamaha Corporation
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
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include "spi-mach5.h"
#include "mach5_ex/yad1driver.h"
#include "../vivo-codec-common.h"
#include "mach5_ex/mach5_micin.h"
#include <sound/asound.h>
#include <sound/control.h>
#include <sound/soc.h>

#define BBK_VIVO_AUDIO_DEBUG 1

#ifdef CONFIG_VIVO_REGDUMP
	#include <linux/vivo-regdump.h>
	static int mach5_clock_on_flag = 0;
	static int mach5_power_on_flag = 0;
	static int mach5_mic_swch_flag = 0;
	static int mach5_hp_swch_flag = 0;
#endif
#define DEVICE_19_2MHZ

static struct cdev mach5_cdev;
static struct class *mach5_class;
static int dev_major;
static int dev_mainor;
static dev_t devt;
struct spi_device *mach5_spi;
struct mutex mach5_lock;
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
#ifdef CONFIG_SPI_MACH5_READ_ONE_BYTE_BURST
	u8 tmp[16];
	unsigned int i;
#endif

	if (copy_from_user(&reg, (void *)arg, sizeof(reg)) != 0)
		return -EFAULT;

	if ((reg.size+3) > sizeof(buf)) {
		rx = kmalloc(reg.size+3, GFP_KERNEL);
		if (rx == NULL)
			return -EFAULT;
		read_buf = rx;
	}
	mutex_lock(&mach5_lock);
#ifdef CONFIG_SPI_MACH5_READ_ONE_BYTE_BURST
	tmp[0] = (u8)(reg.address<<1) | 0x80;

	for (i = 0; i < reg.size; i++) {
		spi_rw(tmp, tmp+2, 1+1);
		read_buf[3 + i] = tmp[3];
	}
#else
	read_buf[0] = (u8)(reg.address<<1) | 0x80;
	if (reg.size > 1)
		read_buf[0] |= 0x01;	/* burst */

	spi_rw(read_buf, read_buf+2, reg.size+1);
#endif

	if (copy_to_user((void *)(uintptr_t)reg.data, (void *)&read_buf[3],
								reg.size) != 0)
		err = -EFAULT;

	if (rx != NULL)
		kfree(rx);
	mutex_unlock(&mach5_lock);
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
	mutex_lock(&mach5_lock);
	if (copy_from_user(write_buf, (void *)(uintptr_t)reg.data,
							reg.size) != 0)
		err = -EFAULT;

	spi_rw((u8 *)write_buf, NULL, reg.size);

	if (tx != NULL)
		kfree(tx);
	mutex_unlock(&mach5_lock);
	return err;
}

int mach5_spi_write(unsigned char com, const unsigned char *data,
							unsigned long size)
{
	u8 *tx = NULL;
	u8 buf[FIX_BUFF_SIZE];
	u8 *write_buf = buf;

	if (size+1 > sizeof(buf)) {
		tx = kmalloc(size+1, GFP_KERNEL);
		if (tx == NULL)
			return -EFAULT;
		write_buf = tx;
	}
	mutex_lock(&mach5_lock);
	write_buf[0] = com;
	memcpy(&write_buf[1], (void *)data, size);

	spi_rw((u8 *)write_buf, NULL, size+1);

	if (tx != NULL)
		kfree(tx);
	mutex_unlock(&mach5_lock);
	return 0;
}

int mach5_spi_read(unsigned char com, const unsigned char *data,
							unsigned long size)
{
	u8 *rx = NULL;
	u8 buf[FIX_BUFF_SIZE];
	u8 *read_buf = buf;
	int err = 0;
#ifdef CONFIG_SPI_MACH5_READ_ONE_BYTE_BURST
	u8 tmp[16];
	unsigned int i;
#endif

	if ((size+3) > sizeof(buf)) {
		rx = kmalloc(size+3, GFP_KERNEL);
		if (rx == NULL)
			return -EFAULT;
		read_buf = rx;
	}
	mutex_lock(&mach5_lock);
#ifdef CONFIG_SPI_MACH5_READ_ONE_BYTE_BURST
	tmp[0] = com;
	tmp[0] &= ~0x01;

	for (i = 0; i < size; i++) {
		spi_rw(tmp, tmp+2, 1+1);
		read_buf[3 + i] = tmp[3];
	}
#else
	read_buf[0] = com;

	spi_rw(read_buf, read_buf+2, size+1);
#endif

	memcpy((void *)data, (void *)&read_buf[3], size);

	if (rx != NULL)
		kfree(rx);
	mutex_unlock(&mach5_lock);
	return err;
}

int start(int input, int output, int mic_vol)
{
	int ret;
	unsigned char com;
	unsigned char adr;
	unsigned char adr2;
	unsigned char data;

	/* ANA_ID check */
	com = (6 << 1);
	data = 0;
	mach5_spi_write(com, &data, 1);
	com = (7 << 1);
	mach5_spi_read(com, &data, 1);
	if (data == 0x98) {
		if ((input != MACH5_INPUT_MIC1) &&
			(input != MACH5_INPUT_MIC2) &&
			(input != MACH5_INPUT_MIC5) &&
			(input != MACH5_INPUT_MUTE)) {
			return -1;
		}

		if ((output != MACH5_OUTPUT_HP) &&
			(output != MACH5_OUTPUT_EXTOUT)) {
			return -1;
		}
	} else {
		if ((input != MACH5_INPUT_MIC1) &&
			(input != MACH5_INPUT_MIC2) &&
			(input != MACH5_INPUT_MIC3) &&
			(input != MACH5_INPUT_MIC4) &&
			(input != MACH5_INPUT_MIC5) &&
			(input != MACH5_INPUT_MUTE)) {
			return -1;
		}

		if ((output != MACH5_OUTPUT_SP) &&
			(output != MACH5_OUTPUT_HP) &&
			(output != MACH5_OUTPUT_EXTOUT)) {
			return -1;
		}
	}
	printk("ktv ftm mode: input(%d) output(%d)\n",input,output);
	/* mic_vol (33 == 0dB) */
	/* 0 Mute     16 -17.0  32 -1.0   48 15.0 */
	/* 1 Mute     17 -16.0  33 0.0    49 16.0 */
	/* 2 Mute     18 -15.0  34 1.0    50 17.0 */
	/* 3 -30.0    19 -14.0  35 2.0    51 18.0 */
	/* 4 -29.0    20 -13.0  36 3.0    52 19.0 */
	/* 5 -28.0    21 -12.0  37 4.0    53 20.0 */
	/* 6 -27.0    22 -11.0  38 5.0    54 21.0 */
	/* 7 -26.0    23 -10.0  39 6.0    55 21.5 */
	/* 8 -25.0    24 -9.0   40 7.0    56 22.0 */
	/* 9 -24.0    25 -8.0   41 8.0    57 22.5 */
	/* 10 -23.0   26 -7.0   42 9.0    58 23.0 */
	/* 11 -22.0   27 -6.0   43 10.0   59 23.5 */
	/* 12 -21.0   28 -5.0   44 11.0   60 24.0 */
	/* 13 -20.0   29 -4.0   45 12.0   61 26.0 */
	/* 14 -19.0   30 -3.0   46 13.0   62 28.0 */
	/* 15 -18.0   31 -2.0   47 14.0   63 30.0 */
	if ((mic_vol < 0) || (63 < mic_vol))
		return -1;
		
	if (function->mclk_enable)
		function-> mclk_enable(VIVO_CODEC_KTV, 19200000);

	if (function->power_up)
		function->power_up(VIVO_CODEC_KTV);
#ifdef DEVICE_19_2MHZ
	ret = YAD1DD_BatchProcess(au08mach5_micin_00_19_2MHz,
					sizeof(au08mach5_micin_00_19_2MHz));
	if (ret != YAD1DD_SUCCESS) {
		printk(KERN_ERR "au08mach5_micin_00_19_2MHz set error\n");
		return -1;
	}
#else
	ret = YAD1DD_BatchProcess(au08mach5_micin_00,
						sizeof(au08mach5_micin_00));
	if (ret != YAD1DD_SUCCESS) {
		printk(KERN_ERR "au08mach5_micin_00 set error\n");
		return -1;
	}
#endif

	ret = YAD1DD_BatchProcess(au08mach5_micin_16,
						sizeof(au08mach5_micin_16));
	if (ret != YAD1DD_SUCCESS) {
		printk(KERN_ERR "au08mach5_micin_16 set error\n");
		return -1;
	}

	ret = YAD1DD_BatchProcess(au08mach5_micin_17,
						sizeof(au08mach5_micin_17));
	if (ret != YAD1DD_SUCCESS) {
		printk(KERN_ERR "au08mach5_micin_17 set error\n");
		return -1;
	}

	/* MUTE */
	if (input == MACH5_INPUT_MUTE)
		return 0;

	if (input == MACH5_INPUT_MIC5) {
		/* MA_REG #14 = #15 = mic_vol + 51 */
		com = (12 << 1);
		data = 14;
		mach5_spi_write(com, &data, 1);
		com = (13 << 1);
		data = (unsigned char)mic_vol + 51;
		mach5_spi_write(com, &data, 1);

		com = (12 << 1);
		data = 15;
		mach5_spi_write(com, &data, 1);
		com = (13 << 1);
		data = (unsigned char)mic_vol + 51;
		mach5_spi_write(com, &data, 1);
	} else {
		/* MA_REG #14 = #15 = 0x60 */
		com = (12 << 1);
		data = 14;
		mach5_spi_write(com, &data, 1);
		com = (13 << 1);
		data = 0x60;
		mach5_spi_write(com, &data, 1);

		com = (12 << 1);
		data = 15;
		mach5_spi_write(com, &data, 1);
		com = (13 << 1);
		data = 0x60;
		mach5_spi_write(com, &data, 1);
	}

	adr = 0;
	adr2 = 0;
	if (input != MACH5_INPUT_MIC5) {
		/* IN: ANA_REG #27 or #28 or #29 or #30 = 0x38 */
		switch (input) {
		case MACH5_INPUT_MIC1:
			adr = 27;
			break;
		case MACH5_INPUT_MIC2:
			adr = 28;
			break;
		case MACH5_INPUT_MIC3:
			adr = 29;
			break;
		case MACH5_INPUT_MIC4:
			adr = 30;
			break;
		}
		com = (6 << 1);
		data = adr;
		mach5_spi_write(com, &data, 1);
		com = (7 << 1);
		data = (unsigned char)mic_vol;
		mach5_spi_write(com, &data, 1);
	}

	/* OUT: MA_REG #22/#23 or #28/#29 or #30/#31 = 0x60 */
	switch (output) {
	case MACH5_OUTPUT_SP:
		adr = 30;
		adr2 = 31;
		break;
	case MACH5_OUTPUT_HP:
		adr = 28;
		adr2 = 29;
		break;
	case MACH5_OUTPUT_EXTOUT:
		adr = 22;
		adr2 = 23;
		break;
	}
	com = (12 << 1);
	data = adr;
	mach5_spi_write(com, &data, 1);
	com = (13 << 1);
	data = 0x60;
	mach5_spi_write(com, &data, 1);

	com = (12 << 1);
	data = adr2;
	mach5_spi_write(com, &data, 1);
	com = (13 << 1);
	data = 0x60;
	mach5_spi_write(com, &data, 1);

	if (input == MACH5_INPUT_MIC5) {
		com = (32 << 1);
		data = 42;
		mach5_spi_write(com, &data, 1);
		com = (33 << 1);
		data = 0x33;
		mach5_spi_write(com, &data, 1);

		com = (32 << 1);
		data = 72;
		mach5_spi_write(com, &data, 1);
		com = (33 << 1);
		data = 0x08;
		mach5_spi_write(com, &data, 1);
	}

	return 0;
}

int stop(void)
{
	int ret;
	unsigned char com;
	unsigned char data;

	/* mute */
	ret = YAD1DD_BatchProcess(au08mach5_micin_18,
						sizeof(au08mach5_micin_18));
	if (ret != YAD1DD_SUCCESS)
		return -1;

	/* I/F #2 0x01 */
	com = (2 << 1);
	data = 0x01;
	mach5_spi_write(com, &data, 1);
	if (function->mclk_disable)
		function->mclk_disable(VIVO_CODEC_KTV);

	if (function->power_down)
		function->power_down(VIVO_CODEC_KTV);
	return 0;
}
int ktv_ftm_enable(int mode)
{
	int ret;
	switch (mode) {
	case 1:
		ret = start(MACH5_INPUT_MIC2,MACH5_OUTPUT_HP, 53);
		break;
	case 2:
		ret = start(MACH5_INPUT_MIC4,MACH5_OUTPUT_EXTOUT, 53);
		break;
	case 3:
		ret = start(MACH5_INPUT_MIC5,MACH5_OUTPUT_HP, 53);
		break;
	case 0:
	default :
		ret = stop();
	}
	return ret;
}

static int ioctl_start(unsigned long arg)
{
	struct start_info info;

	if (copy_from_user(&info, (void *)arg, sizeof(info)) != 0)
		return -EFAULT;

	return start((int)info.input, (int)info.output, (int)info.mic_vol);
}

static int ioctl_stop(void)
{
	return stop();
}

static int ioctl_read_reg_2(unsigned long arg)
{
	u8 read_buf[FIX_BUFF_SIZE];
	u8 write_buf[2];
	u8 tmp[16];
	int err = 0;
	struct read_reg_2 reg_2;
#ifdef CONFIG_SPI_MACH5_READ_ONE_BYTE_BURST
	unsigned int i;
#endif

	if (copy_from_user(&reg_2, (void *)arg, sizeof(reg_2)) != 0)
		return -EFAULT;

	if (reg_2.size < 20)
		return -EFAULT;

	write_buf[0] = (u8)(48 << 1);
	write_buf[1] = (u8)(0x80 | reg_2.address);
	do {
		spi_rw((u8 *)write_buf, NULL, 2);

		read_buf[0] = (u8)(49 << 1) | 0x80;
		tmp[0] = read_buf[0];

#ifdef CONFIG_SPI_MACH5_READ_ONE_BYTE_BURST
		for (i = 0; i < reg_2.size; i++) {
			spi_rw(tmp, tmp+2, 1+1);
			read_buf[3 + i] = tmp[3];
		}
#else
		read_buf[0] |= 0x01;
		spi_rw(read_buf, read_buf+2, reg_2.size+1);
#endif

		spi_rw((u8 *)write_buf, NULL, 2);
		spi_rw(tmp, tmp+2, 1+1);

		if (read_buf[3] == tmp[3])
			break;

		if (reg_2.retry == 0)
			return -EFAULT;

		reg_2.retry--;
	} while (1);


	if (copy_to_user((void *)(uintptr_t)reg_2.data, (void *)&read_buf[3],
							reg_2.size) != 0)
		err = -EFAULT;

	return err;
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
	case MACH5_IOCTL_START:
		return ioctl_start(arg);
	case MACH5_IOCTL_STOP:
		return ioctl_stop();
	case MACH5_IOCTL_READ_REG_2:
		return ioctl_read_reg_2(arg);
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


static long mach5_compat_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	return mach5_ioctl(filp, cmd, arg);
}

const struct file_operations mach5_fops = {
	.unlocked_ioctl = mach5_ioctl,
	.compat_ioctl = mach5_compat_ioctl,
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

#ifdef BBK_VIVO_AUDIO_DEBUG
static struct dentry *debugfs_root;
static struct dentry *debugfs_i2c;
static struct dentry *mach5_debugfs_reg;

static int mach5_debugfs_open(struct inode *inode, struct file *pfile)
{
	return 0;
}
static ssize_t mach5_i2c_read(struct file *pfile, char __user *buf,
								size_t count, loff_t *pos)
{
	unsigned char reg, read[4], write[4];
	int size = 512;
	char kbuf[size];
	int n =0;
	if (!function)
	{
		printk(KERN_ERR "%s get_vivo_codec_function NULL\n",__func__);
		read[3] = 0;
		goto exit;
	}
	if (function->mclk_enable)
		function->mclk_enable(VIVO_CODEC_KTV, 19200000);

	if (function->power_up)
		function->power_up(VIVO_CODEC_KTV);

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
	/* write register 0 to tell mach 5 that we need to read register 0(ID) of A_REG */
	reg = 0;
	write[0] = (0 << 7) | (reg << 1) | (0 << 0);
	write[1] = 1 << 7 | 0 << 0;
	reg = 1;
	/* read register 1 to get the value of register 0 in A_REG */
	write[2] = (1 << 7) | (reg << 1) | (0 << 0);
	spi_rw(write, read, 4);

	if (function->mclk_disable)
		function->mclk_disable(VIVO_CODEC_KTV);

	if (function->power_down)
		function->power_down(VIVO_CODEC_KTV);

	if (read[3] != 0x81) {
		printk(KERN_ERR "mach5: read ID error 0x%x. \n",read[3]);
		read[3] = 0;
	}else
		printk(KERN_ERR "mach5: read ID 0x%x\n", read[3]);

exit:
	n += scnprintf(kbuf+n,size-n,"mach5 i2c read %s \n",
				 read[3]?"OK":"ERROR");
	kbuf[n]='\0';

	return simple_read_from_buffer(buf,count,pos,kbuf,n);
}

static int mach5_debug_release (struct inode *inode, struct file *filep)
{
	return 0;
}

static struct file_operations mach5_i2c_fops = {
	.open    = mach5_debugfs_open,
	.read    = mach5_i2c_read,
	.release = mach5_debug_release,
};
static int mach5_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t mach5_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;
	unsigned char com[1];
	unsigned char write[3];
	unsigned char read[4];
	
	if (!function)
	{
		printk(KERN_ERR "%s get_vivo_codec_function NULL\n",__func__);
		goto exit;
	}
	if (function->mclk_enable)
		function->mclk_enable(VIVO_CODEC_KTV, 19200000);

	if (function->power_up)
		function->power_up(VIVO_CODEC_KTV);

	/* write data to 0x3A */
	write[0] = 6 << 1;
	write[1] = 0x02;
	spi_rw(write, NULL, 2);

	/* read ID */
	/* write register 0 to tell mach 5 that we need to read register 2(ID) of ANA_REG */
	com[0] = (7<<1) | 0x80;
	spi_rw(com, read, 2);

	if (function->mclk_disable)
		function->mclk_disable(VIVO_CODEC_KTV);

	if (function->power_down)
		function->power_down(VIVO_CODEC_KTV);
	
	printk("%s:The digital mic pin state is :%x [float:0,Power:1]\n",__func__,read[1]);
	read[1] = (read[1] & 0x08) >> 3; 
	n += scnprintf(buffer+n,size-n,"The mach5 digital mic pin state is :%s [float:1,Power:0]\n",read[1]?"float":"Power On");
	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
exit:
	n += scnprintf(buffer+n,size-n,"The get_vivo_codec_function is NULL \n");
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, pos, buffer, n);
}
static struct file_operations mach5_debugfs_fops = {
	.open = mach5_debug_open,
	.read = mach5_debug_read,
	//.write = mach5_debug_write,
};
static void mach5_debugfs_init(void)
{
	debugfs_root = debugfs_create_dir("mach5",NULL);
	if(NULL == debugfs_root){
		printk(KERN_ERR "%s create mach5 debugfs root failed! \n",__func__);
	}else if(IS_ERR(debugfs_root)){
		printk(KERN_ERR "%s kernle not support debugfs\n",__func__);
		debugfs_root = NULL;
	}

	debugfs_i2c = debugfs_create_file("i2c",0444,debugfs_root,NULL,&mach5_i2c_fops);
	if(NULL == debugfs_i2c){
		printk(KERN_ERR "%s create mach5 debugfs file failed \n",__func__);
	}
	
	mach5_debugfs_reg = debugfs_create_file("dmic_pin",0644,debugfs_root,NULL,&mach5_debugfs_fops);
	if(!mach5_debugfs_reg)
	{
		printk("mach5 debugfs create fail \n");
	}
	return ;
}

static void mach5_debugfs_deinit(void)
{
	debugfs_remove(debugfs_i2c);
	debugfs_remove(debugfs_root);
	return ;
}
#endif

static int mach5_spi_probe(struct spi_device *spi)
{
	int ret;
	dev_t dev;
	struct device *mach5_dev;
	unsigned char reg, read[4], write[4];

	dev_err(&spi->dev, "mach5_spi_probe()\n");

#ifdef BBK_VIVO_AUDIO_DEBUG
		mach5_debugfs_init();
#endif

	function = get_vivo_codec_function();
	if (!function) {
		dev_err(&spi->dev, "%s vivo_codec_function is NULL\n",
			__func__);
		return -EINVAL;
	}

	if (function->mclk_enable)
		function-> mclk_enable(VIVO_CODEC_KTV, 19200000);

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
	mutex_init(&mach5_lock);
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
//		goto err_id;
	}
	function->ktv_ftm_enable = ktv_ftm_enable;
#ifdef CONFIG_VIVO_REGDUMP
	vivo_audio_regdump_register_callback(&mach5_regdump_handler);
#endif
	
//	goto err_id;
	return 0;

err_device_create:
	class_destroy(mach5_class);
err_class_create:
	cdev_del(&mach5_cdev);
err_cdev_add:
	unregister_chrdev_region(devt, 1);
err_alloc_chrdev:
//err_id:
	return -1;
}

static int mach5_spi_remove(struct spi_device *spi)
{
#ifdef BBK_VIVO_AUDIO_DEBUG
	mach5_debugfs_deinit();
#endif

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
MODULE_VERSION("0.9.0");
