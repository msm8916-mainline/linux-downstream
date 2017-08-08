/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#define LGTP_MODULE "[S3320]"

#include <linux/uaccess.h>
#include <linux/cdev.h>

#include <linux/input/unified_driver_2/lgtp_common.h>
#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_model_config.h>
#include <linux/input/unified_driver_2/lgtp_device_s3320.h>

#define CHAR_DEVICE_NAME 		"rmi"
#define DEVICE_CLASS_NAME 		"rmidev"
#define SYSFS_FOLDER_NAME 		"rmidev"
#define DEV_NUMBER 				1
#define REG_ADDR_LIMIT 			0xFFFF

#define MAX_NUMBER_OF_BUTTONS 	4
#define MAX_INTR_REGISTERS 		4

#define TX_RX_CHANEL_DATA 		256


#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141016 : */
#define MASK_16BIT 				0xFFFF
#define MASK_8BIT 				0xFF
#define MASK_7BIT 				0x7F
#define MASK_6BIT				0x3F
#define MASK_5BIT 				0x1F
#define MASK_4BIT 				0x0F
#define MASK_3BIT 				0x07
#define MASK_2BIT 				0x03
#define MASK_1BIT 				0x01

int synaptics_rmi4_reg_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

	/* page read */
	retval = Touch_I2C_Read(ts->client, PAGE_SELECT_REG, &page_old, sizeof(page_old));

	if (retval < 0) {
		TOUCH_ERR("Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

	/* page compare & change */
	if (page_old == page_new) {
		page_changed = false;
	} else {
		retval = Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_ERR("Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

	/* read register */
	retval = Touch_I2C_Read(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), data, length);

	if (retval < 0) {
		TOUCH_ERR("Failed to read the register(addr=0x%04x)\n", addr);
		return retval;
	}

	/* page restore */
	if (page_changed) {
		retval = Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_ERR("Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

int synaptics_rmi4_reg_write(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

	/* page read */
	retval = Touch_I2C_Read(ts->client, PAGE_SELECT_REG, &page_old, sizeof(page_old));

	if (retval < 0) {
		TOUCH_ERR("Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

	/* page compare & change */
	if (page_old == page_new) {
		page_changed = false;
	} else {
		retval = Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_ERR("Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

	/* write register */
	retval = Touch_I2C_Write(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), data, length);

	if (retval < 0) {
		TOUCH_ERR("Failed to write to the register(addr=0x%04x)\n", addr);
		return retval;
	}

	/* page restore */
	if (page_changed) {
		retval = Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_ERR("Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

int synaptics_rmi4_byte_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

/* page read */
	retval = Touch_I2C_Read(ts->client, PAGE_SELECT_REG, &page_old, sizeof(page_old));

	if (retval < 0) {
		TOUCH_ERR("Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

	/* page compare & change */
	if (page_old == page_new) {
		page_changed = false;
	} else {
		retval = Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_ERR("Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

	/* read register */
	retval = Touch_I2C_Read(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), data, length);

	if (retval < 0) {
		TOUCH_ERR("Failed to long read the register(addr=0x%04x)\n", addr);
		return retval;
	}

	/* page restore */
	if (page_changed) {
		retval = Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_ERR("Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}
#endif

static ssize_t synaptics_rmi4_show_error(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t rmidev_sysfs_data_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t rmidev_sysfs_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_release_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_attn_state_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidev_sysfs_pid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidev_sysfs_pid_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_term_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t rmidev_sysfs_intr_mask_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t rmidev_sysfs_intr_mask_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

struct rmidev_handle {
	dev_t dev_no;
	pid_t pid;
	unsigned char intr_mask;
	struct device dev;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_ts_data *ts_data;
	struct kobject *sysfs_dir;
	struct siginfo interrupt_signal;
	struct siginfo terminate_signal;
	struct task_struct *task;
	void *data;
	bool irq_enabled;
};

struct rmidev_data {
	int ref_count;
	struct cdev main_dev;
	struct class *device_class;
	struct mutex file_mutex;
	struct rmidev_handle *rmi_dev;
};

static struct bin_attribute attr_data = {
	.attr = {
		.name = "data",
		.mode = (S_IRUGO | S_IWUGO),
	},
	.size = 0,
	.read = rmidev_sysfs_data_show,
	.write = rmidev_sysfs_data_store,
};

static struct device_attribute attrs[] = {
	__ATTR(open, S_IWUGO,
			synaptics_rmi4_show_error,
			rmidev_sysfs_open_store),
	__ATTR(release, S_IWUGO,
			synaptics_rmi4_show_error,
			rmidev_sysfs_release_store),
	__ATTR(attn_state, S_IRUGO,
			rmidev_sysfs_attn_state_show,
			synaptics_rmi4_store_error),
	__ATTR(pid, S_IRUGO | S_IWUGO,
			rmidev_sysfs_pid_show,
			rmidev_sysfs_pid_store),
	__ATTR(term, S_IWUGO,
			synaptics_rmi4_show_error,
			rmidev_sysfs_term_store),
	__ATTR(intr_mask, S_IRUGO | S_IWUGO,
			rmidev_sysfs_intr_mask_show,
			rmidev_sysfs_intr_mask_store),
};

static int rmidev_major_num;

static struct class *rmidev_device_class;

static struct rmidev_handle *rmidev;

DECLARE_COMPLETION(rmidev_remove_complete);

static ssize_t synaptics_rmi4_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	dev_warn(dev, "%s Attempted to read from write-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static ssize_t synaptics_rmi4_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	dev_warn(dev, "%s Attempted to write to read-only attribute %s\n",
			__func__, attr->attr.name);
	return -EPERM;
}

static ssize_t rmidev_sysfs_data_show(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int retval;
	unsigned int length = (unsigned int)count;
	unsigned short address = (unsigned short)pos;

	TOUCH_FUNC();

	if (length > (REG_ADDR_LIMIT - address)) {
		TOUCH_ERR("[%s] Out of register map limit\n", __FUNCTION__);
		return -EINVAL;
	}

	if (length) {
		retval = synaptics_rmi4_reg_read(rmidev->ts_data,
				address,
				(unsigned char *)buf,
				length);
		if (retval < 0) {
			TOUCH_ERR("[%s]  Failed to read data\n", __FUNCTION__);
			return retval;
		}
	} else {
		return -EINVAL;
	}

	return length;
}

static ssize_t rmidev_sysfs_data_store(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int retval;
	unsigned int length = (unsigned int)count;
	unsigned short address = (unsigned short)pos;

	TOUCH_FUNC();

	if (length > (REG_ADDR_LIMIT - address)) {
		TOUCH_ERR("[%s] Out of register map limit\n", __FUNCTION__);
		return -EINVAL;
	}

	if (length) {
		retval = synaptics_rmi4_reg_write(rmidev->ts_data,
				address,
				(unsigned char *)buf,
				length);
		if (retval < 0) {
			TOUCH_ERR("[%s] Failed to write data\n", __FUNCTION__);
			return retval;
		}
	} else {
		return -EINVAL;
	}

	return length;
}

static ssize_t rmidev_sysfs_open_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	TOUCH_FUNC();

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	return count;
}

static ssize_t rmidev_sysfs_release_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	TOUCH_FUNC();

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	return count;
}

static ssize_t rmidev_sysfs_attn_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int attn_state = 0;

	TOUCH_FUNC();

	return snprintf(buf, PAGE_SIZE, "%u\n", attn_state);
}

static ssize_t rmidev_sysfs_pid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	TOUCH_FUNC();

	return snprintf(buf, PAGE_SIZE, "%u\n", rmidev->pid);
}

static ssize_t rmidev_sysfs_pid_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	TOUCH_FUNC();

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmidev->pid = input;

	if (rmidev->pid) {
		rmidev->task = pid_task(find_vpid(rmidev->pid), PIDTYPE_PID);
		if (!rmidev->task) {
			TOUCH_ERR("[%s] Failed to locate PID of data logging tool\n", __FUNCTION__);
			return -EINVAL;
		}
	}

	return count;
}

static ssize_t rmidev_sysfs_term_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	TOUCH_FUNC();

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	if (rmidev->pid)
		send_sig_info(SIGTERM, &rmidev->terminate_signal, rmidev->task);

	return count;
}

static ssize_t rmidev_sysfs_intr_mask_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	TOUCH_FUNC();

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", rmidev->intr_mask);
}

static ssize_t rmidev_sysfs_intr_mask_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;

	TOUCH_FUNC();

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmidev->intr_mask = (unsigned char)input;

	return count;
}

/*
 * rmidev_llseek - set register address to access for RMI device
 *
 * @filp: pointer to file structure
 * @off:
 *	if whence == SEEK_SET,
 *		off: 16-bit RMI register address
 *	if whence == SEEK_CUR,
 *		off: offset from current position
 *	if whence == SEEK_END,
 *		off: offset from end position (0xFFFF)
 * @whence: SEEK_SET, SEEK_CUR, or SEEK_END
 */
static loff_t rmidev_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	struct rmidev_data *dev_data = filp->private_data;

	TOUCH_FUNC();

	if (IS_ERR(dev_data)) {
		TOUCH_ERR("[%s] Pointer of char device data is invalid\n",
					__FUNCTION__);
		return -EBADF;
	}

	mutex_lock(&(dev_data->file_mutex));

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;
	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;
	case SEEK_END:
		newpos = REG_ADDR_LIMIT + off;
		break;
	default:
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > REG_ADDR_LIMIT) {
		TOUCH_ERR("[%s] New position 0x%04x is invalid\n",
					__FUNCTION__, whence);
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:
	mutex_unlock(&(dev_data->file_mutex));

	return newpos;
}

/*
 * rmidev_read: read register data from RMI device
 *
 * @filp: pointer to file structure
 * @buf: pointer to user space buffer
 * @count: number of bytes to read
 * @f_pos: starting RMI register address
 */
static ssize_t rmidev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval;
	unsigned char *tmpbuf;
	struct rmidev_data *dev_data = filp->private_data;

//	TOUCH_FUNC();

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	mutex_lock(&(dev_data->file_mutex));

	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0) {
		retval = 0;
		goto unlock;
	}

	if (*f_pos > REG_ADDR_LIMIT) {
		retval = -EFAULT;
		goto unlock;
	}

	tmpbuf = kzalloc(count + 1, GFP_KERNEL);
	if (!tmpbuf) {
		retval = -ENOMEM;
		goto unlock;
	}

	if (count >= TX_RX_CHANEL_DATA) {
		retval = synaptics_rmi4_byte_read(rmidev->ts_data,
				*f_pos,
				tmpbuf,
				count);
	}else {
		retval = synaptics_rmi4_reg_read(rmidev->ts_data,
				*f_pos,
				tmpbuf,
				count);
	}

	if (retval < 0)
		goto clean_up;

	if (copy_to_user(buf, tmpbuf, count))
		retval = -EFAULT;
	else
		*f_pos += retval;

clean_up:
	kfree(tmpbuf);
unlock:
	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

/*
 * rmidev_write: write register data to RMI device
 *
 * @filp: pointer to file structure
 * @buf: pointer to user space buffer
 * @count: number of bytes to write
 * @f_pos: starting RMI register address
 */
static ssize_t rmidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t retval;
	unsigned char *tmpbuf;
	struct rmidev_data *dev_data = filp->private_data;

	TOUCH_FUNC();

	if (IS_ERR(dev_data)) {
		pr_err("%s: Pointer of char device data is invalid", __func__);
		return -EBADF;
	}

	mutex_lock(&(dev_data->file_mutex));

	if (*f_pos > REG_ADDR_LIMIT) {
		retval = -EFAULT;
		goto unlock;
	}

	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0) {
		retval = 0;
		goto unlock;
	}

	tmpbuf = kzalloc(count + 1, GFP_KERNEL);
	if (!tmpbuf) {
		retval = -ENOMEM;
		goto unlock;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		retval = -EFAULT;
		goto clean_up;
	}

	retval = synaptics_rmi4_reg_write(rmidev->ts_data,
			*f_pos,
			tmpbuf,
			count);
	if (retval >= 0)
		*f_pos += retval;

clean_up:
	kfree(tmpbuf);
unlock:
	mutex_unlock(&(dev_data->file_mutex));
	return retval;
}

static int rmidev_open(struct inode *inp, struct file *filp)
{
	struct rmidev_data *dev_data =
			container_of(inp->i_cdev, struct rmidev_data, main_dev);
	int retval = 0;

	TOUCH_FUNC();

	if (!dev_data)
		return -EACCES;

	filp->private_data = dev_data;

	mutex_lock(&(dev_data->file_mutex));

	TouchDisableIrq();

	if (dev_data->ref_count < 1)
		dev_data->ref_count++;
	else
		retval = -EACCES;

	mutex_unlock(&(dev_data->file_mutex));

	return retval;
}

static int rmidev_release(struct inode *inp, struct file *filp)
{
	struct rmidev_data *dev_data =
			container_of(inp->i_cdev, struct rmidev_data, main_dev);

	TOUCH_FUNC();

	if (!dev_data)
		return -EACCES;

	mutex_lock(&(dev_data->file_mutex));

	TouchEnableIrq();

	dev_data->ref_count--;
	if (dev_data->ref_count < 0)
		dev_data->ref_count = 0;

	mutex_unlock(&(dev_data->file_mutex));

	return 0;
}

static const struct file_operations rmidev_fops = {
	.owner = THIS_MODULE,
	.llseek = rmidev_llseek,
	.read = rmidev_read,
	.write = rmidev_write,
	.open = rmidev_open,
	.release = rmidev_release,
};

static void rmidev_device_cleanup(struct rmidev_data *dev_data)
{
	dev_t devno;

	TOUCH_FUNC();

	if (dev_data) {
		devno = dev_data->main_dev.dev;

		if (dev_data->device_class)
			device_destroy(dev_data->device_class, devno);

		cdev_del(&dev_data->main_dev);

		unregister_chrdev_region(devno, 1);

		TOUCH_DBG("[%s] rmidev device removed\n", __FUNCTION__);
	}

	return;
}

static char *rmi_char_devnode(struct device *dev, umode_t *mode)
{
	TOUCH_FUNC();

	if (!mode)
		return NULL;

	*mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

	return kasprintf(GFP_KERNEL, "rmi/%s", dev_name(dev));
}

static int rmidev_create_device_class(void)
{
	rmidev_device_class = class_create(THIS_MODULE, DEVICE_CLASS_NAME);

	TOUCH_FUNC();

	if (IS_ERR(rmidev_device_class)) {
		pr_err("%s: Failed to create /dev/%s\n",
				__func__, CHAR_DEVICE_NAME);
		return -ENODEV;
	}

	rmidev_device_class->devnode = rmi_char_devnode;

	return 0;
}

static void rmidev_attn(struct synaptics_ts_data *ts,
		unsigned char intr_mask)
{
	TOUCH_FUNC();

	if (!rmidev)
		return;

	if (rmidev->pid && (rmidev->intr_mask & intr_mask))
		send_sig_info(SIGIO, &rmidev->interrupt_signal, rmidev->task);

	return;
}

static int rmidev_init_device(struct synaptics_ts_data *ts)
{
	int retval;
	TouchDriverData *lge_ts;
	dev_t dev_no;
	unsigned char attr_count;
	struct rmidev_data *dev_data;
	struct device *device_ptr;

	TOUCH_FUNC();

	lge_ts = i2c_get_clientdata(ts->client);
	rmidev = kzalloc(sizeof(*rmidev), GFP_KERNEL);
	if (!rmidev) {
		TOUCH_ERR("[%s] Failed to alloc mem for rmidev\n", __FUNCTION__);
		retval = -ENOMEM;
		goto err_rmidev;
	}

	rmidev->ts_data = ts;

	memset(&rmidev->interrupt_signal, 0, sizeof(rmidev->interrupt_signal));
	rmidev->interrupt_signal.si_signo = SIGIO;
	rmidev->interrupt_signal.si_code = SI_USER;

	memset(&rmidev->terminate_signal, 0, sizeof(rmidev->terminate_signal));
	rmidev->terminate_signal.si_signo = SIGTERM;
	rmidev->terminate_signal.si_code = SI_USER;

	retval = rmidev_create_device_class();
	if (retval < 0) {
		TOUCH_ERR("[%s] ailed to create device class\n", __FUNCTION__);
		goto err_device_class;
	}

	if (rmidev_major_num) {
		dev_no = MKDEV(rmidev_major_num, DEV_NUMBER);
		retval = register_chrdev_region(dev_no, 1, CHAR_DEVICE_NAME);
	} else {
		retval = alloc_chrdev_region(&dev_no, 0, 1, CHAR_DEVICE_NAME);
		if (retval < 0) {
			TOUCH_ERR("[%s] Failed to allocate char device region\n", __FUNCTION__);
			goto err_device_region;
		}

		rmidev_major_num = MAJOR(dev_no);
		TOUCH_DBG("[%s] Major number of rmidev = %d\n",
					__FUNCTION__, rmidev_major_num);
	}

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		TOUCH_ERR("[%s] Failed to alloc mem for dev_data\n", __FUNCTION__);
		retval = -ENOMEM;
		goto err_dev_data;
	}

	mutex_init(&dev_data->file_mutex);
	dev_data->rmi_dev = rmidev;
	rmidev->data = dev_data;

	cdev_init(&dev_data->main_dev, &rmidev_fops);

	retval = cdev_add(&dev_data->main_dev, dev_no, 1);
	if (retval < 0) {
		TOUCH_ERR("[%s] Failed to add rmi char device\n", __FUNCTION__);
		goto err_char_device;
	}

	dev_set_name(&rmidev->dev, "rmidev%d", MINOR(dev_no));
	dev_data->device_class = rmidev_device_class;

	device_ptr = device_create(dev_data->device_class, NULL, dev_no,
			NULL, CHAR_DEVICE_NAME"%d", MINOR(dev_no));
	if (IS_ERR(device_ptr)) {
		TOUCH_ERR("[%s] Failed to create rmi char device\n", __FUNCTION__);
		retval = -ENODEV;
		goto err_char_device;
	}

#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141016 : */
#else
	retval = gpio_export(ts->pdata->int_pin, false);
	if (retval < 0) {
		TOUCH_ERR("[%s] Failed to export attention gpio\n", __FUNCTION__);
	} else {
		retval = gpio_export_link(&(lge_ts->input_dev->dev), "attn", ts->pdata->int_pin);

		if (retval < 0) {
			TOUCH_ERR("[%s] Failed to create gpio symlink\n", __FUNCTION__);
		} else {
			TOUCH_DBG("[%s] Exported attention gpio %d\n",
						__FUNCTION__, ts->pdata->int_pin);
		}
	}
#endif

	rmidev->sysfs_dir = kobject_create_and_add(SYSFS_FOLDER_NAME, &lge_ts->input_dev->dev.kobj);

	if (!rmidev->sysfs_dir) {
		TOUCH_ERR("[%s] Failed to create sysfs directory\n", __FUNCTION__);
		retval = -ENODEV;
		goto err_sysfs_dir;
	}

	retval = sysfs_create_bin_file(rmidev->sysfs_dir,
			&attr_data);
	if (retval < 0) {
		TOUCH_ERR("[%s] Failed to create sysfs bin file\n", __FUNCTION__);
		goto err_sysfs_bin;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(rmidev->sysfs_dir,
				&attrs[attr_count].attr);
		if (retval < 0) {
			TOUCH_ERR("[%s] Failed to create sysfs attributes\n", __FUNCTION__);
			retval = -ENODEV;
			goto err_sysfs_attrs;
		}
	}

	return 0;

err_sysfs_attrs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(rmidev->sysfs_dir, &attrs[attr_count].attr);

	sysfs_remove_bin_file(rmidev->sysfs_dir, &attr_data);

err_sysfs_bin:
	kobject_put(rmidev->sysfs_dir);

err_sysfs_dir:
err_char_device:
	rmidev_device_cleanup(dev_data);
	kfree(dev_data);

err_dev_data:
	unregister_chrdev_region(dev_no, 1);

err_device_region:
	class_destroy(rmidev_device_class);

err_device_class:
	kfree(rmidev);
	rmidev = NULL;

err_rmidev:
	return retval;
}

static void rmidev_remove_device(struct synaptics_ts_data *ts)
{
	unsigned char attr_count;
	struct rmidev_data *dev_data;

	TOUCH_FUNC();

	if (!rmidev)
		goto exit;

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(rmidev->sysfs_dir, &attrs[attr_count].attr);

	sysfs_remove_bin_file(rmidev->sysfs_dir, &attr_data);

	kobject_put(rmidev->sysfs_dir);

	dev_data = rmidev->data;
	if (dev_data) {
		rmidev_device_cleanup(dev_data);
		kfree(dev_data);
	}

	unregister_chrdev_region(rmidev->dev_no, 1);

	class_destroy(rmidev_device_class);

	kfree(rmidev);
	rmidev = NULL;

exit:
	complete(&rmidev_remove_complete);

	return;
}

static struct synaptics_ts_exp_fn rmidev_module = {
	.init = rmidev_init_device,
	.remove = rmidev_remove_device,
	.reset = NULL,
	.reinit = NULL,
	.early_suspend = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = rmidev_attn,
};

static int __init rmidev_module_init(void)
{
	TOUCH_FUNC();

	synaptics_ts_rmidev_function(&rmidev_module, true);

	return 0;
}

static void __exit rmidev_module_exit(void)
{
	TOUCH_FUNC();

	synaptics_ts_rmidev_function(&rmidev_module, false);

	wait_for_completion(&rmidev_remove_complete);

	return;
}

module_init(rmidev_module_init);
module_exit(rmidev_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX RMI Dev Module");
MODULE_LICENSE("GPL v2");
