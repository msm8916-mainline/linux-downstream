/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This driver adds support for generic RMI4 devices from Synpatics. It
 * implements the mandatory f01 RMI register and depends on the presence of
 * other required RMI functions.
 *
 * The RMI4 specification can be found here (URL split after files/ for
 * style reasons):
 * http://www.synaptics.com/sites/default/files/
 *           511-000136-01-Rev-E-RMI4%20Intrfacing%20Guide.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/rmi.h>
#include "rmi_driver.h"
#include "rmi_f01.h"
#include "rmi_f34.h"
#include <linux/wakelock.h> 

#include <linux/vivo_touchscreen_config.h>
#include <linux/vivo_touchscreen_common.h>

#ifdef CONFIG_RMI4_DEBUG
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#endif
#include <linux/bbk_drivers_info.h>


#define REGISTER_DEBUG 0

#define HAS_NONSTANDARD_PDT_MASK 0x40
#define RMI4_MAX_PAGE 50 //0xff
#define RMI4_PAGE_SIZE 0x100

#define RMI_DEVICE_RESET_CMD	0x01
#define DEFAULT_RESET_DELAY_MS	100

extern int is_i2c_probe_done;

/*
#if defined(PD1401V) || defined(PD1401F)|| defined(PD1401F_EX)
#define BC_MODE_LOCATION                0x0426
#define ER_MODE_LOCATION                0x010d
#define FAST_RELAXTION_LOCATION                0x011d
#endif
*/
struct rmi_driver_data *global_ddata = NULL;
//wangyong add for charger noise
static int charger_flag=0;
//wangyong add end
struct rmi_device_platform_data *global_pdata = NULL;
//liukangfei add for charger noise
extern int usb_charger_flag;

//#if defined(BBK_GLOVES_MODE)
#define RMI_GLOVES_MODE_OFFSET 47
//#endif

#if defined (BBK_SAVEPOWER)
extern int BBK_powersave_flag;
#endif
extern int rmi_dev_get_fn54_data(struct rmi_function_container *fc, char **dest_data,
					unsigned char function_number);

//#if defined(BBK_DCLICK_WAKE)
extern int rmi_f11_change_report_mode_to_dclick(struct rmi_function_container *fc, bool yes);
//#endif
extern int rmi_f01_change_to_sleep_mode(struct rmi_function_container *fc, bool yes);
//#if defined(BBK_DCLICK_WAKE)
extern void rmi_f11_get_dclick_flag_info(struct rmi_function_container *fc, 
									int *whether_in_dclick_mode, int *has_tp_suspend);
extern void rmi_f11_set_dclick_flag_info(struct rmi_function_container *fc, 
									int *whether_in_dclick_mode, int *has_tp_suspend);
//#endif
extern int rmi_f11_get_sensor_electrodes(struct rmi_function_container *fc,
									int *rx_num, int *tx_num);
extern void rmi_f11_finger_realse(struct rmi_driver_data *ddata);


extern int hardware_id_check(void);


//unsigned int is_atboot;  //add  temporarily

extern unsigned int is_atboot;

struct rmi_function_container *rmi_get_function_container(
					struct rmi_driver_data *ddata, int function_number);
static int set_gloves_mode_para(struct rmi_driver_data *ddata, int on);
#ifndef CONFIG_HAS_EARLYSUSPEND
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#endif
static void wait_resetirq_worker(struct work_struct *work);

//#if defined (BBK_DCLICK_WAKE)
void ts_scan_switch(bool on);
//#endif
#if defined (WATCH_DOG_TIMER)
static void stop_watch_dog_timer(struct rmi_driver_data *data );
static void start_watch_dog_timer(struct rmi_driver_data *data );
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void rmi_driver_early_suspend(struct early_suspend *h);
static void rmi_driver_late_resume(struct early_suspend *h);
#endif

/* sysfs files for attributes for driver values. */
static ssize_t rmi_driver_bsr_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_driver_bsr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_driver_enabled_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_driver_enabled_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

#if REGISTER_DEBUG
static ssize_t rmi_driver_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);
#endif

#ifdef CONFIG_RMI4_DEBUG

struct driver_debugfs_data {
	bool done;
	struct rmi_device *rmi_dev;
};

static int debug_open(struct inode *inodep, struct file *filp)
{
	struct driver_debugfs_data *data;

	data = kzalloc(sizeof(struct driver_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->rmi_dev = inodep->i_private;
	filp->private_data = data;
	return 0;
}

static int debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

#ifdef CONFIG_RMI4_SPI
#define DELAY_NAME "delay"

static ssize_t delay_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	struct driver_debugfs_data *data = filp->private_data;
	struct rmi_device_platform_data *pdata =
			data->rmi_dev->phys->dev->platform_data;
	int retval;
	char local_buf[size];

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%d %d %d %d %d\n",
		pdata->spi_data.read_delay_us, pdata->spi_data.write_delay_us,
		pdata->spi_data.block_delay_us,
		pdata->spi_data.pre_delay_us, pdata->spi_data.post_delay_us);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t delay_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset) {
	struct driver_debugfs_data *data = filp->private_data;
	struct rmi_device_platform_data *pdata =
			data->rmi_dev->phys->dev->platform_data;
	int retval;
	char local_buf[size];
	unsigned int new_read_delay;
	unsigned int new_write_delay;
	unsigned int new_block_delay;
	unsigned int new_pre_delay;
	unsigned int new_post_delay;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;

	retval = sscanf(local_buf, "%u %u %u %u %u", &new_read_delay,
			&new_write_delay, &new_block_delay,
			&new_pre_delay, &new_post_delay);
	if (retval != 5) {
		VIVO_TS_LOG_ERR("[%s]:Incorrect number of values provided for delay.", __func__);
		return -EINVAL;
	}
	if (new_read_delay < 0) {
		VIVO_TS_LOG_ERR("[%s]:Byte delay must be positive microseconds.\n", __func__);
		return -EINVAL;
	}
	if (new_write_delay < 0) {
		VIVO_TS_LOG_ERR("[%s]:Write delay must be positive microseconds.\n", __func__);
		return -EINVAL;
	}
	if (new_block_delay < 0) {
		VIVO_TS_LOG_ERR("[%s]:Block delay must be positive microseconds.\n", __func__);
		return -EINVAL;
	}
	if (new_pre_delay < 0) {
		VIVO_TS_LOG_ERR("[%s]:Pre-transfer delay must be positive microseconds.\n", __func__);
		return -EINVAL;
	}
	if (new_post_delay < 0) {
		VIVO_TS_LOG_ERR("[%s]:Post-transfer delay must be positive microseconds.\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_DBG("[%s]:Setting delays to %u %u %u %u %u.\n", __func__, new_read_delay,
		 new_write_delay, new_block_delay, new_pre_delay,
		 new_post_delay);
	pdata->spi_data.read_delay_us = new_read_delay;
	pdata->spi_data.write_delay_us = new_write_delay;
	pdata->spi_data.block_delay_us = new_block_delay;
	pdata->spi_data.pre_delay_us = new_pre_delay;
	pdata->spi_data.post_delay_us = new_post_delay;

	return size;
}

static const struct file_operations delay_fops = {
	.owner = THIS_MODULE,
	.open = debug_open,
	.release = debug_release,
	.read = delay_read,
	.write = delay_write,
};
#endif /* CONFIG_RMI4_SPI */

#define PHYS_NAME "phys"

static ssize_t phys_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	struct driver_debugfs_data *data = filp->private_data;
	struct rmi_phys_info *info = &data->rmi_dev->phys->info;
	int retval;
	char local_buf[size];

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, PAGE_SIZE,
		"%-5s %ld %ld %ld %ld %ld %ld %ld\n",
		 info->proto ? info->proto : "unk",
		 info->tx_count, info->tx_bytes, info->tx_errs,
		 info->rx_count, info->rx_bytes, info->rx_errs,
		 info->attn_count);
	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static const struct file_operations phys_fops = {
	.owner = THIS_MODULE,
	.open = debug_open,
	.release = debug_release,
	.read = phys_read,
};

static int setup_debugfs(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
#ifdef CONFIG_RMI4_SPI
	struct rmi_phys_info *info = &rmi_dev->phys->info;
#endif
	int retval = 0;

	if (!rmi_dev->debugfs_root)
		return -ENODEV;

#ifdef CONFIG_RMI4_SPI
	if (!strncmp("spi", info->proto, 3)) {
		data->debugfs_delay = debugfs_create_file(DELAY_NAME, RMI_RW_ATTR,
					rmi_dev->debugfs_root, rmi_dev, &delay_fops);
		if (!data->debugfs_delay || IS_ERR(data->debugfs_delay)) {
			VIVO_TS_LOG_ERR("[%s]:Failed to create debugfs delay.\n", __func__);
			data->debugfs_delay = NULL;
		}
	}
#endif

	data->debugfs_phys = debugfs_create_file(PHYS_NAME, RMI_RO_ATTR,
				rmi_dev->debugfs_root, rmi_dev, &phys_fops);
	if (!data->debugfs_phys || IS_ERR(data->debugfs_phys)) {
		VIVO_TS_LOG_ERR("[%s]:Failed to create debugfs phys.\n", __func__);
		data->debugfs_phys = NULL;
	}

	return retval;
}

static void teardown_debugfs(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

#ifdef CONFIG_RMI4_SPI
	if (!data->debugfs_delay)
		debugfs_remove(data->debugfs_delay);
#endif
	if (!data->debugfs_phys)
		debugfs_remove(data->debugfs_phys);
}
#endif

static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev);

static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev);

static int rmi_driver_irq_restore(struct rmi_device *rmi_dev);

static struct device_attribute attrs[] = {
	__ATTR(enabled, RMI_RW_ATTR,
	       rmi_driver_enabled_show, rmi_driver_enabled_store),
#if REGISTER_DEBUG
	__ATTR(reg, RMI_WO_ATTR,
	       rmi_show_error, rmi_driver_reg_store),
#endif
};

static struct device_attribute bsr_attribute = __ATTR(bsr, RMI_RW_ATTR,
	       rmi_driver_bsr_show, rmi_driver_bsr_store);

/* Useful helper functions for u8* */

void u8_set_bit(u8 *target, int pos)
{
	target[pos/8] |= 1<<pos%8;
}

void u8_clear_bit(u8 *target, int pos)
{
	target[pos/8] &= ~(1<<pos%8);
}

bool u8_is_set(u8 *target, int pos)
{
	return target[pos/8] & 1<<pos%8;
}

bool u8_is_any_set(u8 *target, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		if (target[i])
			return true;
	}
	return false;
}

void u8_or(u8 *dest, u8 *target1, u8 *target2, int size)
{
	int i;
	for (i = 0; i < size; i++)
		dest[i] = target1[i] | target2[i];
}

void u8_and(u8 *dest, u8 *target1, u8 *target2, int size)
{
	int i;
	for (i = 0; i < size; i++)
		dest[i] = target1[i] & target2[i];
}

/* Utility routine to set bits in a register. */
int rmi_set_bits(struct rmi_device *rmi_dev, u16 address,
		 unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read_block(rmi_dev, address, &reg_contents, 1);
	if (retval)
		return retval;
	reg_contents = reg_contents | bits;
	retval = rmi_write_block(rmi_dev, address, &reg_contents, 1);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EIO;
	return retval;
}
EXPORT_SYMBOL(rmi_set_bits);

/* Utility routine to clear bits in a register. */
int rmi_clear_bits(struct rmi_device *rmi_dev, u16 address,
		   unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read_block(rmi_dev, address, &reg_contents, 1);
	if (retval)
		return retval;
	reg_contents = reg_contents & ~bits;
	retval = rmi_write_block(rmi_dev, address, &reg_contents, 1);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EIO;
	return retval;
}
EXPORT_SYMBOL(rmi_clear_bits);

static void rmi_free_function_list(struct rmi_device *rmi_dev)
{
	struct rmi_function_container *entry, *n;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	if (!data) {
		VIVO_TS_LOG_ERR("[%s]:WTF: No driver data\n", __func__);
		return;
	}

	if (data->f01_container) {
		if (data->f01_container->fh && data->f01_container->fh->remove) {
			data->f01_container->fh->remove(data->f01_container);
		}
		device_unregister(&data->f01_container->dev);
		VIVO_TS_LOG_ERR("[%s]:start\n", __func__);
		kfree(data->f01_container->irq_mask);
		VIVO_TS_LOG_ERR("[%s]:end\n", __func__);
		kfree(data->f01_container);
		data->f01_container = NULL;
	}

	if (list_empty(&data->rmi_functions.list))
		return;

	list_for_each_entry_safe(entry, n, &data->rmi_functions.list, list) {
		if (entry->fh) {
			if (entry->fh->remove)
				entry->fh->remove(entry);
			device_unregister(&entry->dev);
		}
		kfree(entry->irq_mask);
		list_del(&entry->list);
		kfree(entry);
	}
}

static void no_op(struct device *dev)
{
	VIVO_TS_LOG_DBG("[%s]:REMOVING KOBJ!", __func__);
	kobject_put(&dev->kobj);
}

static int init_one_function(struct rmi_device *rmi_dev,
			     struct rmi_function_container *fc)
{
	int retval;

	if (!fc->fh) {
		struct rmi_function_handler *fh =
			rmi_get_function_handler(fc->fd.function_number);
		if (!fh) {
			VIVO_TS_LOG_ERR("[%s]:No handler for F%02X.\n", __func__,
				fc->fd.function_number);
			return 0;
		}
		fc->fh = fh;
	}

	if (!fc->fh->init)
		return 0;
	/* This memset might not be what we want to do... */
	memset(&(fc->dev), 0, sizeof(struct device));
	dev_set_name(&(fc->dev), "fn%02x", fc->fd.function_number);
	fc->dev.release = no_op;

	fc->dev.parent = &rmi_dev->dev;
	VIVO_TS_LOG_DBG("[%s]:Register F%02X.\n", __func__, fc->fd.function_number);
	retval = device_register(&fc->dev);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed device_register for F%02X.\n", __func__,
			fc->fd.function_number);
		return retval;
	}

	retval = fc->fh->init(fc);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to initialize function F%02x\n", __func__,
			fc->fd.function_number);
		goto error_exit;
	}

	return 0;

error_exit:
	device_unregister(&fc->dev);
	return retval;
}

static void rmi_driver_fh_add(struct rmi_device *rmi_dev,
			      struct rmi_function_handler *fh)
{
	struct rmi_function_container *entry;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	if (!data)
		return;
	if (fh->func == 0x01) {
		if (data->f01_container)
			data->f01_container->fh = fh;
	} else if (!list_empty(&data->rmi_functions.list)) {
		mutex_lock(&data->pdt_mutex);
		list_for_each_entry(entry, &data->rmi_functions.list, list)
			if (entry->fd.function_number == fh->func) {
				entry->fh = fh;
				if (init_one_function(rmi_dev, entry) < 0)
					entry->fh = NULL;
			}
		mutex_unlock(&data->pdt_mutex);
	}

}

static void rmi_driver_fh_remove(struct rmi_device *rmi_dev,
				 struct rmi_function_handler *fh)
{
	struct rmi_function_container *entry, *temp;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	if (fh->func == 0x01) {
		/* don't call remove here since
		 * rmi_f01_initialize just get call one time */
		if (data->f01_container)
			data->f01_container->fh = NULL;
		return;
	}

	list_for_each_entry_safe(entry, temp, &data->rmi_functions.list,
									list) {
		if (entry->fh && entry->fd.function_number == fh->func) {
			if (fh->remove)
				fh->remove(entry);

			entry->fh = NULL;
			device_unregister(&entry->dev);
		}
	}
}

static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
//	struct device *dev = &rmi_dev->dev;
	struct rmi_function_container *entry;
	int retval;

	/* Device control (F01) is handled before anything else. */

	if (data->f01_container && data->f01_container->fh &&
			data->f01_container->fh->reset) {
		retval = data->f01_container->fh->reset(data->f01_container);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]:F%02x reset handler failed: %d.\n", __func__,
				data->f01_container->fh->func, retval);
			return retval;
		}
	}

	if (list_empty(&data->rmi_functions.list))
		return 0;

	list_for_each_entry(entry, &data->rmi_functions.list, list) {
		if (entry->fh && entry->fh->reset) {
			retval = entry->fh->reset(entry);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]:F%02x reset handler failed: %d\n", __func__,
					entry->fh->func, retval);
				return retval;
			}
		}
	}
	
	return 0;
}

static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
//	struct device *dev = &rmi_dev->dev;
	struct rmi_function_container *entry;
	int retval;

	/* Device control (F01) is handled before anything else. */

	if (data->f01_container && data->f01_container->fh &&
			data->f01_container->fh->config) {
		retval = data->f01_container->fh->config(data->f01_container);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]:F%02x config handler failed: %d.\n", __func__,
					data->f01_container->fh->func, retval);
			return retval;
		}
	}

	if (list_empty(&data->rmi_functions.list))
		return 0;

	list_for_each_entry(entry, &data->rmi_functions.list, list) {
		if (entry->fh && entry->fh->config) {
			retval = entry->fh->config(entry);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]:F%02x config handler failed: %d.\n", __func__,
					entry->fh->func, retval);
				return retval;
			}
		}
	}

	return 0;
}

static void construct_mask(u8 *mask, int num, int pos)
{
	int i;

	for (i = 0; i < num; i++)
		u8_set_bit(mask, pos+i);
}

static int process_interrupt_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
//	struct device *dev = &rmi_dev->dev;
	struct rmi_function_container *entry;
	u8 irq_status[data->num_of_irq_regs];
	u8 irq_bits[data->num_of_irq_regs];
	int error;

	error = rmi_read_block(rmi_dev,
				data->f01_container->fd.data_base_addr + 1,
				irq_status, data->num_of_irq_regs);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read irqs, code=%d\n", __func__, error);
		return error;
	}
	/* Device control (F01) is handled before anything else. */
	if (data->f01_container->irq_mask && data->f01_container->fh->attention) {
		u8_and(irq_bits, irq_status, data->f01_container->irq_mask,
				data->num_of_irq_regs);
		if (u8_is_any_set(irq_bits, data->num_of_irq_regs))
			data->f01_container->fh->attention(
					data->f01_container, irq_bits);
	}

	u8_and(irq_status, irq_status, data->current_irq_mask,
	       data->num_of_irq_regs);
	/* At this point, irq_status has all bits that are set in the
	 * interrupt status register and are enabled.
	 */

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->irq_mask && entry->fh && entry->fh->attention) {
			u8_and(irq_bits, irq_status, entry->irq_mask,
			       data->num_of_irq_regs);
			if (u8_is_any_set(irq_bits, data->num_of_irq_regs)) {
				error = entry->fh->attention(entry, irq_bits);
				if (error < 0)
					VIVO_TS_LOG_ERR("[%s]:f%.2x"
						" attention handler failed:"
						" %d\n", __func__,
						entry->fh->func, error);
			}
		}

	return 0;
}

static void rmi_driver_clear_irq_status(struct rmi_device *rmi_dev,
						struct rmi_driver_data *data)
{
	u8 irq_status[data->num_of_irq_regs];
	int error = 0;
	
	error = rmi_read_block(rmi_dev,
				data->f01_container->fd.data_base_addr + 1,
				irq_status, data->num_of_irq_regs);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read irqs, code=%d\n", __func__, error);
	}
}

extern int qup_i2c_suspended;
/*qgf add for iic suspended. So wait the IIC BUs ready.*/
static void rmi4_driver_irq_err_work_func(struct work_struct *work)
{
	struct rmi_driver_data *data = container_of(work, struct rmi_driver_data,irq_err_work);
	int count = 0;
	/* Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->rmi_dev ||!data->f01_container || !data->f01_container->fh) {
		VIVO_TS_LOG_INF("[%s]:Not ready to handle interrupts yet!\n",__func__);
		return ;
	}

	if (data->early_suspended) {
		/* wait i2c adapter resume */
		while(qup_i2c_suspended > 0 && count < 80) {
			msleep(5);
			count++;
		}

		if (count == 80) {
			VIVO_TS_LOG_ERR("[%s]:The i2c bus stilll suspend after 20 times try\n", __func__);
		}
	}  

	process_interrupt_requests(data->rmi_dev);
	return ;
}

static int rmi_driver_irq_handler(struct rmi_device *rmi_dev)	/* wyl delete the irq request */
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	int count = 0;
	/* Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container || !data->f01_container->fh) {
		VIVO_TS_LOG_ERR("[%s]:Not ready to handle interrupts yet!\n", __func__);
		return 0;
	}

	if (data->early_suspended) {
		/* wait i2c adapter resume */
		while(qup_i2c_suspended > 0 && count < 20) {
			msleep(5);
			count++;
		}

		if (count == 20) {
			VIVO_TS_LOG_ERR("[%s]:The i2c bus stilll suspend after 20 times try\n", __func__);
			queue_work(data->irq_err_workqueue, &data->irq_err_work);
			return IRQ_HANDLED;
		}
	}  

	return process_interrupt_requests(rmi_dev);
}

/* wangyuanliang add temply for disable relax*/
#if 0
static void rmi_f11_close_fast_relax(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *ddata = rmi_get_driverdata(rmi_dev);
	int ret;
	u8 temp;
	struct rmi_function_container * f54;
	u16 f54_control_base_addr, f54_command_base_addr;
	u8 fast_relax;
	
	f54 = rmi_get_function_container(ddata, 0x54);
	if (!f54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return;
	}
	f54_control_base_addr  = f54->fd.control_base_addr;
	f54_command_base_addr = f54->fd.command_base_addr;
	VIVO_TS_LOG_ERR("[%s]:f54_control_base_addr = %x, f54_command_base_addr = %x~~~~~~~~~\n",__func__,f54_control_base_addr,f54_command_base_addr);
	ret = rmi_read(rmi_dev, f54_control_base_addr +16, &fast_relax);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_read is error =======\n",__func__);
		return;
	}
	VIVO_TS_LOG_INF("[%s]:sensor->fast_relax = %d~~~~~~~\n",__func__,fast_relax);


	//set Fast Relaxation Rate = 0 ---2D
	ret = rmi_write(rmi_dev, f54_control_base_addr +16, 0);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_write is error =======\n",__func__);
		return;
	}
  //set fast relaxation rate = 0  --- 0D
	ret = rmi_write(rmi_dev, f54_control_base_addr +85, 0);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_write is error =======\n",__func__);
		return;
	}
	//disable Energy Ratio Relaxation
	ret = rmi_read(rmi_dev, f54_control_base_addr, &temp);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_read is error =======\n",__func__);
		return;
	}
	temp = temp & 0xdf;
	VIVO_TS_LOG_INF("[%s]:disable Energy~~~~temp |  0xdf = %x~~~~~~~~\n",__func__,temp);
	ret = rmi_write(rmi_dev, f54_control_base_addr, temp);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_write is error=======\n",__func__);
		return;
	}
	
	//Force update
	ret = rmi_read(rmi_dev,f54_command_base_addr, &temp);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_read is error =======\n",__func__);
		return;
	}
	temp = temp | 0x04;
	VIVO_TS_LOG_ERR("[%s]:Force update~~temp |  0x04 = %x~~~~~~~~\n",__func__,temp);
	ret = rmi_write(rmi_dev, f54_command_base_addr, temp);
	if (ret < 0)
	{
		VIVO_TS_LOG_ERR("[%s]:[SYNA]=======rmi_write is error=======\n",__func__);
		return;
	}
			

}
#endif //wangyong del

//wangyong add for reset

void reset_dlick_setting(struct rmi_driver_data *ddata, bool on)
{
	struct rmi_function_container *f01, *f11;

	if (!ddata) {
		VIVO_TS_LOG_ERR("[%s]: rmi driver data is NULL\n", __func__);
		return;
	}

	f01 = ddata->f01_container;
	if (!f01) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
		return;
	}

	f11 = rmi_get_function_container(ddata, 0x11);
	if (!f11) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x11 function container\n", __func__);
		return; 
	}
	
	mutex_lock(&ddata->suspend_mutex);
	if (on) {
		
		if (rmi_f01_change_to_sleep_mode(f01, false)) {
			VIVO_TS_LOG_ERR("[%s]:Failed to change sleep mode-false!\n", __func__);
		}

		msleep(5);
//#if defined(BBK_DCLICK_WAKE)
        if (vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		    if (rmi_f11_change_report_mode_to_dclick(f11, true)) {
			    VIVO_TS_LOG_ERR("[%s]:Failed to change report mode-true!\n", __func__);
		    }
		}
//#endif
		//atomic_set(&ddata->ts_state, TOUCHSCREEN_GESTURE);
		rmi_driver_clear_irq_status(ddata->rmi_dev, ddata);
	}else{
	//#if defined(BBK_DCLICK_WAKE)
		if (vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			if (rmi_f11_change_report_mode_to_dclick(f11, false)) {
				VIVO_TS_LOG_ERR("[%s]:Failed to change report mode-false!\n", __func__);
			}
		}
	//#endif

		msleep(5);

		if (rmi_f01_change_to_sleep_mode(f01, true)) {
			VIVO_TS_LOG_ERR("[%s]:Failed to change sleep mode-true!\n", __func__);
		}
		//atomic_set(&ddata->ts_state, TOUCHSCREEN_SLEEP);
	}
	mutex_unlock(&ddata->suspend_mutex);
	
}


//wangyong add for USB connect Noise  1:connect 0:disconnect
int usb_charge_connected = -1;
static int rmi_driver_reset_handler(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	int error, content;

	/* Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container || !data->f01_container->fh) {
		VIVO_TS_LOG_INF("[%s]:Not ready to handle reset yet!\n", __func__);
		return 0;
	}

	error = rmi_driver_process_reset_requests(rmi_dev);
	if (error < 0)
		return error;


	error = rmi_driver_process_config_requests(rmi_dev);
	if (error < 0)
		return error;


	//wangyong add for reset which from IC or host
	VIVO_TS_LOG_INF("[%s]: reset is call!==========\n",__func__);
//#if defined(BBK_GLOVES_MODE)
    if (vivo_touchscreen_is_support(FS_GLOVES_MODE))
	    VIVO_TS_LOG_INF("[%s]: early_suspended(%d) charger_connect_flag(%d) ts_gloves_mode(%d)\n", __func__, data->early_suspended, data->charger_connect_flag, data->ts_gloves_mode);
//#else
    else
	    VIVO_TS_LOG_INF("[%s]: early_suspended(%d) charger_connect_flag(%d)\n", __func__, data->early_suspended, data->charger_connect_flag);
//#endif

	data->wakeup_event = WAKEUP_BY_RESETIRQ;
	wake_up_interruptible(&data->resetirq_wait);
	
//#if defined(BBK_DCLICK_WAKE)
if (vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
	if(data->early_suspended)
		{
			if (atomic_read(&data->ts_state) == TOUCHSCREEN_SLEEP) 
				reset_dlick_setting(data, 0);
			else if(atomic_read(&data->ts_state) == TOUCHSCREEN_GESTURE) {
	             /*		
	        #if defined(PD1401V) || defined(PD1401F) || defined(PD1401F_EX)
				if(rmi_get_module()){
				printk("[SYNA]======%s: goto gesture mode \n",__func__); 
				//add for change mode
				error = rmi_write(rmi_dev, ER_MODE_LOCATION,0x20 );

				if (error < 0)
					printk("[SYNA]==%s: ER_MODE_LOCATION failed, .\n",__func__);


				error = rmi_write(rmi_dev, BC_MODE_LOCATION, 0x02 );
				if (error < 0)
					printk("[SYNA]==%s: BC_MODE_LOCATION failed, .\n",__func__);


				error = rmi_write(rmi_dev, FAST_RELAXTION_LOCATION,250 );
				if (error < 0)
					printk("[SYNA]==%s: FAST_RELAXTION_LOCATION failed, .\n",__func__);
				}
	        #endif
	        */
				reset_dlick_setting(data, 1);
			}
		
		}
	}
	else
//#endif
	{
		if(data->charger_connect_flag)
		{
			//do nothing
		}
//#if defined(BBK_GLOVES_MODE)
		else
		{
		   if (vivo_touchscreen_is_support(FS_GLOVES_MODE)) {
			    if(data->ts_gloves_mode) {
				     set_gloves_mode_para(data,1);				
			    }
			}			
		}
//#endif
/*
#if defined(PD1401V) || defined(PD1401F)|| defined(PD1401F_EX)
		//add for change mode
	//	error = rmi_write(rmi_dev, BC_MODE_LOCATION, 0x02);
	//	if (error < 0)
	//		printk("[SYNA]==%s: BC_MODE_LOCATION failed, .\n",__func__);
		if(rmi_get_module()){
		error = rmi_write(rmi_dev, ER_MODE_LOCATION,0x00 );
		if (error < 0)
			printk("[SYNA]==%s: ER_MODE_LOCATION failed, .\n",__func__);


		error = rmi_write(rmi_dev, FAST_RELAXTION_LOCATION,100 );
		if (error < 0)
			printk("[SYNA]==%s: FAST_RELAXTION_LOCATION failed, .\n",__func__);
		}
#endif
*/

	}
	rmi_f11_finger_realse(data);
	
	if (data->irq_stored) {
		error = rmi_driver_irq_restore(rmi_dev);
		if (error < 0)
			return error;
	}


	if(usb_charge_connected==1) {
		content = 0x20;	
	}else if(usb_charge_connected == 0) {
		content = 0;	
	}


	error = rmi_write(data->rmi_dev, 0x0051, content);			
	if (error < 0)
		VIVO_TS_LOG_ERR("[%s]:==charger_connect write charger bit  failed, code: %d.\n", __func__, error);
	VIVO_TS_LOG_INF("[%s]:==charger_connect write charger bit success\n", __func__);

	VIVO_TS_LOG_INF("[%s]: reset is End!==========\n", __func__);
	return 0;
}



/*
 * Construct a function's IRQ mask. This should
 * be called once and stored.
 */
static u8 *rmi_driver_irq_get_mask(struct rmi_device *rmi_dev,
				   struct rmi_function_container *fc) {
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	u8 *irq_mask = kzalloc(sizeof(u8) * data->num_of_irq_regs, GFP_KERNEL);
	if (irq_mask)
		construct_mask(irq_mask, fc->num_of_irqs, fc->irq_pos);

	return irq_mask;
}

/*
 * This pair of functions allows functions like function 54 to request to have
 * other interupts disabled until the restore function is called. Only one store
 * happens at a time.
 */
static int rmi_driver_irq_save(struct rmi_device *rmi_dev, u8 * new_ints)
{
	int retval = 0;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
//	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);
	if (!data->irq_stored) {
		/* Save current enabled interrupts */
		retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: Failed to read enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		/*
		 * Disable every interrupt except for function 54
		 * TODO:Will also want to not disable function 1-like functions.
		 * No need to take care of this now, since there's no good way
		 * to identify them.
		 */
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				new_ints, data->num_of_irq_regs);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: Failed to change enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		memcpy(data->current_irq_mask, new_ints,
					data->num_of_irq_regs * sizeof(u8));
		data->irq_stored = true;
	} else {
		retval = -ENOSPC; /* No space to store IRQs.*/
		VIVO_TS_LOG_ERR("[%s]: Attempted to save values when"
						" already stored!", __func__);
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_irq_restore(struct rmi_device *rmi_dev)
{
	int retval = 0;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
//	struct device *dev = &rmi_dev->dev;
	mutex_lock(&data->irq_mutex);

	if (data->irq_stored) {
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: Failed to write enabled interupts!\n",
								__func__);
			goto error_unlock;
		}
		memcpy(data->current_irq_mask, data->irq_mask_store,
					data->num_of_irq_regs * sizeof(u8));
		data->irq_stored = false;
	} else {
		retval = -EINVAL;
		VIVO_TS_LOG_INF("[%s]: Attempted to restore values when not stored!\n",
			__func__);
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_fn_generic(struct rmi_device *rmi_dev,
				     struct pdt_entry *pdt_ptr,
				     int *current_irq_count,
				     u16 page_start)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *fc = NULL;
	int retval = 0;
//	struct device *dev = &rmi_dev->dev;
	struct rmi_device_platform_data *pdata;

	pdata = to_rmi_platform_data(rmi_dev);

	VIVO_TS_LOG_ERR("[%s]:Initializing F%02X for %s.\n", __func__, pdt_ptr->function_number,
		pdata->sensor_name);

	fc = kzalloc(sizeof(struct rmi_function_container),
			GFP_KERNEL);
	if (!fc) {
		VIVO_TS_LOG_ERR("[%s]:Failed to allocate container for F%02X.\n", __func__,
			pdt_ptr->function_number);
		retval = -ENOMEM;
		goto error_free_data;
	}

	copy_pdt_entry_to_fd(pdt_ptr, &fc->fd, page_start);

	fc->rmi_dev = rmi_dev;
	fc->num_of_irqs = pdt_ptr->interrupt_source_count;
	fc->irq_pos = *current_irq_count;
	*current_irq_count += fc->num_of_irqs;

	retval = init_one_function(rmi_dev, fc);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to initialize F%.2x\n", __func__,
			pdt_ptr->function_number);
		goto error_free_data;
	}

	INIT_LIST_HEAD(&fc->list);
	list_add_tail(&fc->list, &data->rmi_functions.list);
	return 0;

error_free_data:
	kfree(fc);
	return retval;
}

/*
 * F01 was once handled very differently from all other functions.  It is
 * now only slightly special, and as the driver is refined we expect this
 * function to go away.
 */
static int rmi_driver_fn_01_specific(struct rmi_device *rmi_dev,
				     struct pdt_entry *pdt_ptr,
				     int *current_irq_count,
				     u16 page_start)
{
	struct rmi_driver_data *data = NULL;
	struct rmi_function_container *fc = NULL;
	union f01_device_status device_status;
	int retval = 0;
//	struct device *dev = &rmi_dev->dev;
	struct rmi_function_handler *fh =
		rmi_get_function_handler(0x01);
	struct rmi_device_platform_data *pdata;

	pdata = to_rmi_platform_data(rmi_dev);
	data = rmi_get_driverdata(rmi_dev);

	retval = rmi_read(rmi_dev, pdt_ptr->data_base_addr,
			  device_status.regs);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read device status.\n", __func__);
		return retval;
	}
	VIVO_TS_LOG_INF("[%s]:device_status.reg is %x\n", __func__, device_status.regs[0]);

	VIVO_TS_LOG_INF("[%s]:Initializing F01 for %s.\n", __func__, pdata->sensor_name);
	if (!fh)
		VIVO_TS_LOG_DBG("[%s]: No function handler for F01?!", __func__);

	fc = kzalloc(sizeof(struct rmi_function_container), GFP_KERNEL);
	if (!fc) {
		retval = -ENOMEM;
		return retval;
	}

	copy_pdt_entry_to_fd(pdt_ptr, &fc->fd, page_start);
	fc->num_of_irqs = pdt_ptr->interrupt_source_count;
	fc->irq_pos = *current_irq_count;
	*current_irq_count += fc->num_of_irqs;

	fc->rmi_dev        = rmi_dev;
	fc->dev.parent     = &fc->rmi_dev->dev;
	fc->fh = fh;

	dev_set_name(&(fc->dev), "fn%02x", fc->fd.function_number);
	fc->dev.release = no_op;

	VIVO_TS_LOG_ERR("[%s]:Register F01.\n", __func__);
	retval = device_register(&fc->dev);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]: Failed device_register for F01.\n", __func__);
		goto error_free_data;
	}

	data->f01_container = fc;
	data->f01_bootloader_mode = device_status.flash_prog;
	if (device_status.flash_prog)
		VIVO_TS_LOG_ERR("[%s]: RMI4 device is in bootloader mode!\n", __func__);

	INIT_LIST_HEAD(&fc->list);

	return retval;

error_free_data:
	kfree(fc);
	return retval;
}

/*
 * Scan the PDT for F01 so we can force a reset before anything else
 * is done.  This forces the sensor into a known state, and also
 * forces application of any pending updates from reflashing the
 * firmware or configuration.  We have to do this before actually
 * building the PDT because the reflash might cause various registers
 * to move around.
 */
static int do_initial_reset(struct rmi_device *rmi_dev)
{
	struct pdt_entry pdt_entry;
	int page;
//	struct device *dev = &rmi_dev->dev;
	bool done = false;
	bool has_f01 = false;
#ifdef	CONFIG_RMI4_FWLIB
	bool has_f34 = false;
	struct pdt_entry f34_pdt, f01_pdt;
#endif
	int i;
	int retval;
	struct rmi_device_platform_data *pdata;
	struct rmi_driver_data *ddata = NULL;
	int reset_times = 0;

	VIVO_TS_LOG_DBG("[%s]:Initial reset.\n", __func__);
	pdata = to_rmi_platform_data(rmi_dev);
	ddata = rmi_get_driverdata(rmi_dev);
	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				VIVO_TS_LOG_ERR("[%s]:Read PDT entry at 0x%04x"
					"failed, code = %d.\n", __func__, i, retval);
				return retval;
			}

			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;
			done = false;

			if (pdt_entry.function_number == 0x01 && reset_times == 0) {
				u16 cmd_addr = page_start +
					pdt_entry.command_base_addr;
				u8 cmd_buf = RMI_DEVICE_RESET_CMD;

				reset_times++;

				retval = rmi_write_block(rmi_dev, cmd_addr,
						&cmd_buf, 1);
				if (retval < 0) {
					VIVO_TS_LOG_ERR("[%s]:Initial reset failed. "
						"Code = %d.\n", __func__, retval);
					return retval;
				}
				mdelay(pdata->reset_delay_ms);
#ifndef CONFIG_RMI4_FWLIB
				done = true;
#else
				memcpy(&f01_pdt, &pdt_entry, sizeof(pdt_entry));
#endif
				has_f01 = true;
				break;
			}
#ifdef	CONFIG_RMI4_FWLIB
			else if (pdt_entry.function_number == 0x34) {
				memcpy(&f34_pdt, &pdt_entry, sizeof(pdt_entry));
				has_f34 = true;
			}
#endif
		}
	}

	if (!has_f01) {
		VIVO_TS_LOG_ERR("[%s]: Failed to find F01 for initial reset.\n", __func__);
		return -ENODEV;
	}

#ifdef CONFIG_RMI4_FWLIB
	if (has_f34)
		(void)rmi4_fw_update(rmi_dev, &f01_pdt, &f34_pdt, NULL);
	else
		VIVO_TS_LOG_ERR("[%s]:WARNING: No F34, firmware update will not be done.\n", __func__);
#endif

	return 0;
}

static int rmi_scan_pdt(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data;
	struct pdt_entry pdt_entry;
	int page;
//	struct device *dev = &rmi_dev->dev;
	int irq_count = 0;
	bool done = false;
	int i;
	int retval;

	VIVO_TS_LOG_ERR("[%s]:Scanning PDT...\n", __func__);

	data = rmi_get_driverdata(rmi_dev);
	mutex_lock(&data->pdt_mutex);

	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				VIVO_TS_LOG_ERR("[%s]:Read PDT entry at 0x%04x "
					"failed.\n", __func__, i);
				goto error_exit;
			}

			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;

			VIVO_TS_LOG_ERR("[%s]: Found F%.2X on page 0x%02X\n",
				__func__, pdt_entry.function_number, page);
			done = false;

			if (pdt_entry.function_number == 0x01)
				retval = rmi_driver_fn_01_specific(rmi_dev,
						&pdt_entry, &irq_count,
						page_start);
			else
				retval = rmi_driver_fn_generic(rmi_dev,
						&pdt_entry, &irq_count,
						page_start);

			if (retval)
				goto error_exit;
		}
		done = done || data->f01_bootloader_mode;
	}
	data->irq_count = irq_count;
	data->num_of_irq_regs = (irq_count + 7) / 8;
	VIVO_TS_LOG_DBG("[%s]: Done with PDT scan.\n", __func__);
	retval = 0;

error_exit:
	mutex_unlock(&data->pdt_mutex);
	return retval;
}

/* wyl add for bbk sys interface */
struct rmi_function_container *rmi_get_function_container(
					struct rmi_driver_data *ddata, int function_number)
{
	struct rmi_function_container *entry;

	if (!list_empty(&ddata->rmi_functions.list)) {
		mutex_lock(&ddata->pdt_mutex);
		list_for_each_entry(entry, &ddata->rmi_functions.list, list)
			if (entry->fd.function_number == function_number) {
				mutex_unlock(&ddata->pdt_mutex);
				return entry;
			}
		mutex_unlock(&ddata->pdt_mutex);
	}

	return NULL;
}


//wangyong add for gloves mode
//#if defined(BBK_GLOVES_MODE)

void get_gloves_mode_para(struct rmi_driver_data *ddata)
{

	struct rmi_function_container *fc11,*fc54;
	u8 temp;
	int retval = 0;
	
	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return ;
	}

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return ;
	}

	retval = rmi_read_block(ddata->rmi_dev, fc11->fd.control_base_addr+46,
					&temp, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves Glove Sensitivity and Response\n", __func__);
		return ;
	}
	VIVO_TS_LOG_INF("[%s]:Glove Sensitivity = %x ,  Glove Response = %x \n", __func__, temp&0xF,temp&0xF0);

	retval = rmi_read_block(ddata->rmi_dev, fc11->fd.control_base_addr+RMI_GLOVES_MODE_OFFSET,&temp, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves Encoding \n", __func__);
		return ;
	}
	VIVO_TS_LOG_INF("[%s]: Glove Encoding = %x \n", __func__, temp&0x3);
	
	retval = rmi_read_block(ddata->rmi_dev, fc54->fd.control_base_addr+2,
					&temp, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves SaturationCap LSB\n", __func__);
		return ;
	}
	VIVO_TS_LOG_INF("[%s]: Glove SaturationCap LSB = %x \n", __func__, temp);

	retval = rmi_read_block(ddata->rmi_dev, fc54->fd.control_base_addr+3,
					&temp, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves SaturationCap MSB\n", __func__);
		return ;
	}
	VIVO_TS_LOG_INF("[%s]:Glove SaturationCap MSB = %x \n", __func__,temp);

	retval = rmi_read_block(ddata->rmi_dev, fc54->fd.control_base_addr+4,
					&temp, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves mode\n", __func__);
		return ;
	}
	VIVO_TS_LOG_ERR("[%s]:Glove Pixel Touch Thres = %x \n", __func__, temp);
	
}

extern int get_colorprop(void);

static int set_gloves_mode_para(struct rmi_driver_data *ddata, int on)
{

	struct rmi_function_container *fc01,*fc11,*fc54;
	u8 gloves_mode, temp;
	u8 module_flag[8];
	//add by qgf,for set more scan freq.
#if 0
	u8 freq_reg;
#endif
	int retval = 0;

	if (!ddata) {
		VIVO_TS_LOG_ERR("[%s]: rmi driver data is NULL\n", __func__);
		return -EIO;
	}

	fc01 = ddata->f01_container;
	
	
	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EIO;
	}

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EIO;;
	}
	
	retval = rmi_read_block(ddata->rmi_dev, fc11->fd.control_base_addr+RMI_GLOVES_MODE_OFFSET,
					&gloves_mode, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves mode\n", __func__);
		return retval;
	}
	else 
		retval = 0;
//add by qgf,for set more scan freq.
#if 0
		retval = rmi_read_block(ddata->rmi_dev, 0x0121, &freq_reg, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves mode\n", __func__);
		return retval;
	}
	else 
	{
		retval = 0;
	}
#endif
	if (on) {
//add by qgf
#if 0
	freq_reg = freq_reg | 0x08;

#endif	
	retval = rmi_read_block(ddata->rmi_dev, fc01->fd.query_base_addr+11, module_flag, 8);
	module_flag[7] = '\0';
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read gloves mode\n", __func__);
		return retval;
	}
	else 
		retval = 0;
	
	
	//	if(strncmp("JTH-GFF", module_flag, 7) == 0 && get_colorprop() ==1)	
		{

			gloves_mode = 0;
			VIVO_TS_LOG_ERR("[%s]: gloves_mode = %x======\n",__func__,gloves_mode);

			retval = rmi_write_block(ddata->rmi_dev, 0x0419, &gloves_mode, 1);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]: failed to write gloves mode\n", __func__);
				return retval;
			}
			else
				retval = 0;

			//Force update
			retval = rmi_read_block(ddata->rmi_dev,fc54->fd.command_base_addr, &temp, 1);
			if (retval < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error=======\n",__func__);
				return retval;
			}else
				retval = 0;
			
			temp = temp | 0x04;
			VIVO_TS_LOG_INF("[%s]:Force update~~temp |  0x04 = %x~~~~~~~~\n",__func__,temp);

			retval = rmi_write_block(ddata->rmi_dev, fc54->fd.command_base_addr, &temp,1);
			if (retval < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error =======\n",__func__);
				return retval;
			}else
				retval = 0;
		}
	#if 0
		else 
		{
			//set Glove Encoding bit(1:0) : 0x01
			gloves_mode = (gloves_mode & 0xfc) | 0x01;
			VIVO_TS_LOG_INF("[%s]: gloves_mode = %x======\n",__func__,gloves_mode);

			retval = rmi_write_block(ddata->rmi_dev, fc11->fd.control_base_addr+RMI_GLOVES_MODE_OFFSET,
							&gloves_mode, 1);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]: failed to write gloves mode\n", __func__);
				return retval;
			}
			else
				retval = 0;

			//set SaturationCap LSB:210
			temp = 220;
			retval = rmi_write_block(ddata->rmi_dev, fc54->fd.control_base_addr+2,
							&temp, 1);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]: failed to write SaturationCap LSB\n", __func__);
				return retval;
			}
			else
				retval = 0;

			//set SaturationCap MSB:0
			temp = 0;
			retval = rmi_write_block(ddata->rmi_dev, fc54->fd.control_base_addr+3,
							&temp, 1);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]: failed to write SaturationCap MSB\n", __func__);
				return retval;
			}
			else
				retval = 0;

			//set Pixel Touch Thres:0.7  (1 values 128   so  0.8*128 = 102)
			temp = 0x66;
			retval = rmi_write_block(ddata->rmi_dev, fc54->fd.control_base_addr+4,
							&temp, 1);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]: failed to write Pixel Touch Thres\n", __func__);
				return retval;
			}
			else
				retval = 0;
			
			//Force update
			retval = rmi_read_block(ddata->rmi_dev,fc54->fd.command_base_addr, &temp, 1);
			if (retval < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error =======\n",__func__);
				return retval;
			}else
				retval = 0;
			
			temp = temp | 0x04;
			VIVO_TS_LOG_INF("[%s]:Force update~~temp |  0x04 = %x~~~~~~~~\n",__func__,temp);

			retval = rmi_write_block(ddata->rmi_dev, fc54->fd.command_base_addr, &temp,1);
			if (retval < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error =======\n",__func__);
				return retval;
			}else
				retval = 0;
		}

		#endif
	}
	else
	{
//add by qgf
#if 0
	freq_reg = freq_reg & 0xf7;
#endif	
		//Default gloves mode is close
		retval = rmi_write(ddata->rmi_dev, ddata->f01_container->fd.command_base_addr, RMI_DEVICE_RESET_CMD);
		if (retval < 0)
			VIVO_TS_LOG_ERR("[%s]:- post-flash reset failed, code: %d.\n",__func__,retval);
		//msleep(DEFAULT_RESET_DELAY_MS);
		//rmi_f11_finger_realse(global_ddata);
	}

//add by qgf
#if 0
		retval = rmi_write_block(ddata->rmi_dev, 0x0121, &freq_reg, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write gloves mode\n", __func__);
			return retval;
		}
#endif

	return retval;

}

//#endif


static int rmi_get_color(struct rmi_driver_data *ddata)
{
	int retval;
	struct f34_userdata version_info;
	struct rmi_function_container *entry;
	u16 f34_control_base_addr = 0;

	
	entry = rmi_get_function_container(ddata, 0x34);
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x34 function container\n", __func__);
		return -1;
	}
	f34_control_base_addr = entry->fd.control_base_addr;

	retval = rmi_read_block(ddata->rmi_dev, f34_control_base_addr,
			(u8 *)&version_info, 4);
	
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:read version info failed\n", __func__);
		return -1;
	}

	return version_info.color;
}


//wangyong add start
void rmi_black_firmware_update(int color)
{
	struct pdt_entry f34_pdt, f01_pdt;
	struct rmi_function_container *entry;
	int retval;
	int temp_color;
	struct rmi_driver_data *ddata;

	if (!global_ddata) {
		VIVO_TS_LOG_ERR("[%s]:global_ddata is NULL!!!\n", __func__);
		return ;
	}
	
    ddata = global_ddata;


	if(color !=1 && color !=2)
		return;



	temp_color = rmi_get_color(global_ddata);
	
	
	if(color == temp_color || color==(temp_color+2))
		return;
	

	entry = ddata->f01_container;
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
		return;
	}

	f01_pdt.query_base_addr = entry->fd.query_base_addr;
	f01_pdt.command_base_addr = entry->fd.command_base_addr;
	f01_pdt.control_base_addr = entry->fd.control_base_addr;
	f01_pdt.data_base_addr = entry->fd.data_base_addr;

	entry = rmi_get_function_container(ddata, 0x34);
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
		return;
	}
	f34_pdt.query_base_addr = entry->fd.query_base_addr;
	f34_pdt.command_base_addr = entry->fd.command_base_addr;
	f34_pdt.control_base_addr = entry->fd.control_base_addr;
	f34_pdt.data_base_addr = entry->fd.data_base_addr;

	retval = rmi4_fw_update(ddata->rmi_dev, &f01_pdt, &f34_pdt, NULL);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Update Firmware Failed!!!\n", __func__);
		return ;
	}

	VIVO_TS_LOG_INF("[%s]:Update firmware success!\n", __func__);
}

//wangyong add end



static ssize_t rmi_firmware_update_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	struct pdt_entry f34_pdt, f01_pdt;
	struct rmi_function_container *entry;
	int retval;
	char name[32];

	if (!ddata) {
		return -EINVAL;
	}
	if (!buf) {
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter!!!\n", __func__);
		return -EINVAL;
	}

	if (sscanf(buf, "%s", name) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter\n", __func__);
		return -EINVAL;
	}
	VIVO_TS_LOG_ERR("[%s]:name is %s\n", __func__, buf);
	VIVO_TS_LOG_ERR("[%s]:name is %s\n", __func__, name);

	entry = ddata->f01_container;
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
		return -EIO;
	}

	f01_pdt.query_base_addr = entry->fd.query_base_addr;
	f01_pdt.command_base_addr = entry->fd.command_base_addr;
	f01_pdt.control_base_addr = entry->fd.control_base_addr;
	f01_pdt.data_base_addr = entry->fd.data_base_addr;

	entry = rmi_get_function_container(ddata, 0x34);
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
		return -EIO;
	}
	f34_pdt.query_base_addr = entry->fd.query_base_addr;
	f34_pdt.command_base_addr = entry->fd.command_base_addr;
	f34_pdt.control_base_addr = entry->fd.control_base_addr;
	f34_pdt.data_base_addr = entry->fd.data_base_addr;

	VIVO_TS_LOG_INF("[%s]:f01_pdt.query_base_addr (%d)\n", __func__, f01_pdt.query_base_addr);
	VIVO_TS_LOG_INF("[%s]:f01_pdt.command_base_addr (%d)\n", __func__, f01_pdt.command_base_addr);
	VIVO_TS_LOG_INF("[%s]:f01_pdt.control_base_addr (%d)\n", __func__, f01_pdt.control_base_addr);
	VIVO_TS_LOG_INF("[%s]:f01_pdt.data_base_addr (%d)\n", __func__, f01_pdt.data_base_addr);
	VIVO_TS_LOG_INF("[%s]:f34_pdt.query_base_addr(%d)\n", __func__, f34_pdt.query_base_addr);
	VIVO_TS_LOG_INF("[%s]:f34_pdt.command_base_addr (%d)\n", __func__, f34_pdt.command_base_addr);
	VIVO_TS_LOG_INF("[%s]:f34_pdt.control_base_addr (%d)\n", __func__, f34_pdt.control_base_addr);
	VIVO_TS_LOG_INF("[%s]:f34_pdt.data_base_addr (%d)\n", __func__, f34_pdt.data_base_addr);
#if defined (WATCH_DOG_TIMER)
	stop_watch_dog_timer(ddata);
#endif
	retval = rmi4_fw_update(ddata->rmi_dev, &f01_pdt, &f34_pdt, name);
#if defined (WATCH_DOG_TIMER)
	start_watch_dog_timer(ddata);
#endif
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Update Firmware Failed!!!\n", __func__);
		return -1;
	}

	VIVO_TS_LOG_INF("[%s]:Update firmware success!\n", __func__);

	return count;
}

static ssize_t rmi_module_id_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
#if 0
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	u8 module_id[9];
	int ret;
	module_id[8] = '\0'; 
	
	
	if (!ddata) {
		VIVO_TS_LOG_ERR("[%s]: no valide driver data\n", __func__);
		return -EIO;
	}
	if(ddata->f01_container)
	{
		ret = rmi_read_block(ddata->rmi_dev, ddata->f01_container->fd.query_base_addr+11, module_id,8);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:~~~~~~rmi_read is error!!\n ",__func__);
		}
	}
	return sprintf(buf, "%s\n", module_id);
#endif
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	struct rmi_function_container *entry;
	u16 f34_control_base_addr = 0;
	u8 module_id;
	entry = rmi_get_function_container(ddata, 0x34);
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x34 function container\n", __func__);
		return sprintf(buf, "%s\n", "No module id address");
	}
	f34_control_base_addr = entry->fd.control_base_addr;
	module_id = rmi_get_module_id(ddata->rmi_dev, f34_control_base_addr);
	if (module_id < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to get module_id\n", __func__);
		return sprintf(buf, "%s\n", "Get module_id failed");
	}else{
		return sprintf(buf, "%02x\n", module_id);
	}
}

static ssize_t rmi_hardware_id_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int hardware_id = -1;
		
	hardware_id= hardware_id_check();
	if(hardware_id < 0)
		return sprintf(buf, "get hardware_id fail!\n");
	else
		return sprintf(buf, "%02X\n", hardware_id);
}

static ssize_t rmi_version_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	struct rmi_function_container *entry;
	u16 f34_control_base_addr = 0;
	u8 version;
	
	entry = rmi_get_function_container(ddata, 0x34);
	if (!entry) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x34 function container\n", __func__);
		return sprintf(buf, "%s\n", "No module id address");
	}
	f34_control_base_addr = entry->fd.control_base_addr;

	version = rmi_get_version(ddata->rmi_dev, f34_control_base_addr);

	if (version < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to get version\n", __func__);
		return sprintf(buf, "%s\n", "Get version failed");
	}else{
		return sprintf(buf, "0x%02x\n", version);
	}
}

static ssize_t rmi_color_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	int color = rmi_get_color(ddata);


	if (color < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to get color_id\n", __func__);
		return sprintf(buf, "%s\n", "Get color failed");
	}else{
		return sprintf(buf, "0x%02x\n", color);
	}
}



static ssize_t rmi_baseline_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	struct rmi_function_container *fc54, *fc11;
	int error;
	int i, j, count = 0;
	int rx, tx;
	char *data;

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EINVAL;
	}

	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EINVAL;
	}

	error = rmi_f11_get_sensor_electrodes(fc11, &rx, &tx);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get sensor elsectrodes failed\n", __func__);
		return -EIO;
	}

	error = rmi_dev_get_fn54_data(fc54, &data, 9);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get baseline data failed\n", __func__);
		return -1;
	}

	count += sprintf(&buf[count], "tp channel: %u * %u\n",
					tx, rx);

	if (ddata->is_button_independent) 
		rx++; /* 0D sensor use one rx*/

	for (i = 0; i < tx; i++) {
		for (j = 0; j < rx * 2; j += 2) {
			count += sprintf(&buf[count], "%u ", 
					(u16)(data[i * rx * 2 + j + 1]) << 8 | data[i * rx * 2 + j]);
		}
		count += sprintf(&buf[count], "\n");
	}

	return count;
}

static ssize_t rmi_rawdata_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	struct rmi_function_container *fc54, *fc11;
	int error;
	int i, j, count = 0;
	int rx, tx;
	char *data;

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EINVAL;
	}

	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EINVAL;
	}

	error = rmi_f11_get_sensor_electrodes(fc11, &rx, &tx);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get sensor elsectrodes failed\n", __func__);
		return -EIO;
	}

	error = rmi_dev_get_fn54_data(fc54, &data, 3);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get baseline data failed\n", __func__);
		return -1;
	}
	count += sprintf(&buf[count], "tp channel: %u * %u\n",
					tx, rx);

	if (ddata->is_button_independent) 
		rx++; /* 0D sensor use one rx*/

	for (i = 0; i < tx; i++) {
		for (j = 0; j < rx * 2; j += 2) {
			count += sprintf(&buf[count], "%u ", 
							(u16)(data[i * rx * 2 + j + 1]) << 8 | (u16)data[i * rx * 2 + j]);
		}
		count += sprintf(&buf[count], "\n");
	}

	return count;
}

static ssize_t rmi_delta_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	struct rmi_function_container *fc54, *fc11;
	int error;
	int i, j, count = 0;
	int rx, tx;
	char *data;

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EINVAL;
	}
	
	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EINVAL;
	}

	error = rmi_f11_get_sensor_electrodes(fc11, &rx, &tx);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get sensor elsectrodes failed\n", __func__);
		return -EIO;
	}

	error = rmi_dev_get_fn54_data(fc54, &data, 2);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get baseline data failed\n", __func__);
		return -1;
	}
	count += sprintf(&buf[count], "tp channel: %u * %u\n",
					tx, rx);

	if (ddata->is_button_independent) 
		rx++; /* 0D sensor use one rx*/

	for (i = 0; i < tx; i++) {
		for (j = 0; j < rx * 2; j += 2) {
			count += sprintf(&buf[count], "%4d ", 
				(short)((u16)(data[i * rx * 2 + j + 1]) << 8 | (u16)data[i * rx * 2 + j]));
		}
		count += sprintf(&buf[count], "\n");
	}

	return count;
}

static ssize_t rmi_reg_data_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int address;
	u8 val;

	sscanf(buf, "%d", &address);
	VIVO_TS_LOG_INF("[%s]:address is %d\n", __func__, address);

	rmi_read_block(ddata->rmi_dev, address, &val, 1);

	VIVO_TS_LOG_ERR("[%s]:val is %u\n", __func__, val);

	return count;

}

static ssize_t rmi_log_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_ERR("[%s]:parameter is %d\n", __func__, val);
	if (val == 0) {
		ddata->log_switch = 0;
	}else if (val == 1) {
		ddata->log_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter passed(%d)\n", __func__, val);
		return -EINVAL;
	}

	return count;
}


static ssize_t rmi_log_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "log_switch = %d\n", ddata->log_switch);
}

static ssize_t rmi_usb_reset_enable_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("[%s]:parameter is %d\n", __func__, val);
	if (val == 0) {
		ddata->usb_reset_enable_switch = 0;
	}else if (val == 1) {
		ddata->usb_reset_enable_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter passed(%d)\n", __func__, val);
		return -EINVAL;
	}

	return count;
}


static ssize_t rmi_usb_reset_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "usb_reset_enable_switch = %d\n", ddata->usb_reset_enable_switch);
}

/* 
static ssize_t rmi_ts_reset_gpio_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int val = gpio_get_value(7);
	return sprintf(buf, "Reset GPIO value is %d\n", val);

}

static ssize_t rmi_ts_reset_gpio_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int val;
	
	if (sscanf(buf, "%d", &val) != 1) {
		dev_err(&ddata->rmi_dev->dev, "Invalide number of parameters passed\n");
		return -EINVAL;
	}

	dev_err(&ddata->rmi_dev->dev, "parameter is %d\n", val);
	if (val == 1) {
		gpio_request(7, "rmi4_rst");
		gpio_direction_output(7, 1);
		msleep(10);
	}
	else if (val == 0)
	{
		gpio_request(7, "rmi4_rst");
		gpio_direction_output(7, 0);
		msleep(200);
	}else{
		dev_err(&ddata->rmi_dev->dev, "Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}
	
	return count;
}*/

static ssize_t rmi_is_calling_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int val;
	
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("[%s]:parameter is %d\n", __func__, val);
	if (val == 0) {
		ddata->is_calling = 0;
	}else if (val == 1) {
		ddata->is_calling = 1;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter passed(%d)\n", __func__, val);
		return -EINVAL;
	}

	return count;
}

static ssize_t rmi_is_calling_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data,kobject_debug); 
													
	return sprintf(buf, "is_calling = %d\n", ddata->is_calling);
}

static ssize_t rmi_is_GSM_opening_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int val;
	
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("[%s]:parameter is %d\n", __func__, val);
	if (val == 0) {
		ddata->is_GSM_opening = 0;


	}else if (val == 1) {
		ddata->is_GSM_opening = 1;

	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter passed(%d)\n", __func__, val);
		return -EINVAL;
	}

	return count;
}

static ssize_t rmi_is_GSM_opening_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data,kobject_debug); 
													
	return sprintf(buf, "is_GSM_opening = %d\n", ddata->is_GSM_opening);
}

//wangyong add for gloves mode
//#if defined(BBK_GLOVES_MODE)

static ssize_t rmi_gloves_para_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	

	struct rmi_function_container *fc54;
	int temp1,temp2,temp3;
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data,kobject_debug);
	int retval = 0;
	
	sscanf(buf, "%d,%d,%d", &temp1,&temp2,&temp3);

			


	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EIO;;
	}						
	//set SaturationCap LSB:210
		retval = rmi_write_block(ddata->rmi_dev, fc54->fd.control_base_addr+2,
						(u8 *)&temp1, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write SaturationCap LSB\n", __func__);
			return retval;
		}
		else
			retval = 0;
	
		//set SaturationCap MSB:0

		retval = rmi_write_block(ddata->rmi_dev, fc54->fd.control_base_addr+3,
						(u8 *)&temp2, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write SaturationCap MSB\n", __func__);
			return retval;
		}
		else
			retval = 0;
	
		//set Pixel Touch Thres:0.7  (1 values 128	 so  0.7*128 = 0.7)

		retval = rmi_write_block(ddata->rmi_dev, fc54->fd.control_base_addr+4,
						(u8 *)&temp3, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write Pixel Touch Thres\n", __func__);
			return retval;
		}
		else
			retval = 0;
		
		//Force update
		retval = rmi_read_block(ddata->rmi_dev,fc54->fd.command_base_addr, (u8 *)&temp1, 1);
		if (retval < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error =======\n",__func__);
			return retval;
		}else
			retval = 0;
		
		temp1 = temp1 | 0x04;
		VIVO_TS_LOG_ERR("[%s]:Force update~~temp |	0x04 = %x~~~~~~~~\n",__func__,temp1);
	
		retval = rmi_write_block(ddata->rmi_dev, fc54->fd.command_base_addr, (u8 *)&temp1,1);
		if (retval < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error =======\n",__func__);
			return retval;
		}else
			retval = 0;


	return count;
}


static ssize_t rmi_gloves_mode_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data,kobject_debug);
	get_gloves_mode_para(ddata);
	return 0;
}


static ssize_t rmi_gloves_mode_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data,kobject_debug);

	int gloves_mode;

	if (sscanf(buf, "%d", &gloves_mode) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
										
	if (gloves_mode == 0) {
		VIVO_TS_LOG_ERR("[%s]:[SYNA] gloves_mode is close! \n", __func__);
	    set_gloves_mode_para(ddata,0);
		ddata->ts_gloves_mode = 0;

	}else if(gloves_mode == 1){
		if(!ddata->charger_connect_flag)//主要防止首次插USB 开机上层写
		{
			VIVO_TS_LOG_ERR("[%s]:gloves_mode is open! \n", __func__);
			set_gloves_mode_para(ddata,1);
		}
		ddata->ts_gloves_mode = 1;

	}else
	{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter(%d)\n", __func__, gloves_mode);
		return -EINVAL;
	}

	return count;
}



//#endif

static ssize_t rmi_charger_flag_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)

{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data,kobject_debug);
	
	return sprintf(buf, "%d\n", ddata->charger_connect_flag);
}


//#if defined(BBK_DCLICK_WAKE)
static ssize_t rmi_ipod_flag_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("[%s]:parameter is %d\n", __func__, val);
	if (val == 0) {
		ddata->ts_ipod_flag = 0;
	}else if (val == 1) {
		ddata->ts_ipod_flag = 1;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter passed(%d)\n", __func__, val);
		return -EINVAL;
	}

	return count;
}

static ssize_t rmi_ipod_flag_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "%d\n", ddata->ts_ipod_flag);
}

static ssize_t rmi_dclick_flag_info_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	struct rmi_function_container *fc11;
	int whether_in_dclick_mode;
	int has_tp_suspend;

	if (sscanf(buf, "%d %d", &whether_in_dclick_mode, &has_tp_suspend) != 2) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
	
	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EIO;
	}
//#if defined(BBK_DCLICK_WAKE)
	rmi_f11_set_dclick_flag_info(fc11, &whether_in_dclick_mode, &has_tp_suspend);
//#endif

	return count;
}
static ssize_t rmi_dclick_flag_info_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	struct rmi_function_container *fc11;
	int whether_in_dclick_mode;
	int has_tp_suspend;

	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EIO;
	}
//#if defined(BBK_DCLICK_WAKE)
	rmi_f11_get_dclick_flag_info(fc11, &whether_in_dclick_mode, &has_tp_suspend);
//#endif

	return sprintf(buf, "whether_in_dclick_mode = %d, has_tp_suspend = %d\n",
						whether_in_dclick_mode, has_tp_suspend);
}

static ssize_t rmi_dclick_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "%d\n", ddata->ts_dclick_switch);
}

static ssize_t rmi_dclick_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int dclick_switch;

	if (sscanf(buf, "%d", &dclick_switch) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
	if (dclick_switch == 0 || dclick_switch == 1) {
		ddata->ts_dclick_switch = dclick_switch;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter(%d)\n", __func__, dclick_switch);
		return -EINVAL;
	}

	return count;
}


static ssize_t rmi_swipe_down_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "%d\n", ddata->swipe_down_switch);
}


static ssize_t rmi_swipe_down_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int swipe_down_switch;

	if (sscanf(buf, "%d", &swipe_down_switch) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
	if (swipe_down_switch == 0 || swipe_down_switch == 1) {
		ddata->swipe_down_switch = swipe_down_switch;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter(%d)\n", __func__, swipe_down_switch);
		return -EINVAL;
	}

	return count;
}


static ssize_t rmi_dclick_simulate_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "%d\n", ddata->ts_dclick_simulate_switch);
}

static ssize_t rmi_dclick_simulate_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int dclick_simulate_switch;

	if (sscanf(buf, "%d", &dclick_simulate_switch) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
	if (dclick_simulate_switch == 0 || dclick_simulate_switch == 1) {
		ddata->ts_dclick_simulate_switch = dclick_simulate_switch;
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameter(%d)\n", __func__, dclick_simulate_switch);
		return -EINVAL;
	}

	return count;
}

static ssize_t rmi_dclick_proximity_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int dclick_switch;
	if (sscanf(buf, "%d", &dclick_switch) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
	VIVO_TS_LOG_INF("[%s]: dclick_switch(%d)\n", __func__, dclick_switch);

	mutex_lock(&ddata->suspend_mutex);
	ddata->proximity_state = dclick_switch;
	if (atomic_read(&ddata->ts_state) == TOUCHSCREEN_NORMAL) {
		ddata->need_change_to_dclick = dclick_switch;
		VIVO_TS_LOG_INF("[%s]:Not first switch set in normal mode\n", __func__);
		mutex_unlock(&ddata->suspend_mutex);
		return count;
	}

	/* normal change state */
	ts_scan_switch(dclick_switch);

	mutex_unlock(&ddata->suspend_mutex);
	return count;
}

static ssize_t rmi_dclick_lcd_state_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "lcd_state is %s\n", ddata->has_lcd_shutoff == 1?"off":"on");
	
}

static ssize_t rmi_dclick_lcd_state_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	int lcd_state;
	if (sscanf(buf, "%d", &lcd_state) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}
	VIVO_TS_LOG_INF("[%s]: lcd_state(%d)\n", __func__, lcd_state);

	if (lcd_state == 1 || lcd_state == 0) {
		ddata->has_lcd_shutoff = (lcd_state == 0);
	}else{
		VIVO_TS_LOG_ERR("[%s]:Invalide parameters passed\n", __func__);
		return -EINVAL;
	}

	return count;
}
//#endif

#define RAWDATA_LIMIT	1000
/* 
   touchkey_tx used for detecting 0D key design
   the content for the array is the 0D key tx line which is started by 0
   the size of array is the number of key
*/
static int touchkey_tx[1] = {0};
static ssize_t rmi_touchscreen_module_state_show(struct kobject *kobj,
								struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	struct rmi_function_container *fc54, *fc11;
	int error;
	int i, j, count = 0;
	int rx, tx;
	char *data;
	u16 *rawdata;
	unsigned long tempvalue;
	bool has_print_rx_failed = false;
	bool has_print_tx_failed = false;
	int tmp;
	int touchkey_index = 0;

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EINVAL;
	}

	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EINVAL;
	}

	error = rmi_f11_get_sensor_electrodes(fc11, &rx, &tx);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get sensor elsectrodes failed\n", __func__);
		return -EIO;
	}

	rawdata = kzalloc(rx * tx * sizeof(u16), GFP_KERNEL);
	if (!rawdata) {
		VIVO_TS_LOG_ERR("[%s]:Can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	error = rmi_dev_get_fn54_data(fc54, &data, 3);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get baseline data failed\n", __func__);
		return -1;
	}
	count += sprintf(&buf[count], "tp channel: %u * %u\n",
					tx, rx);

	if (ddata->is_button_independent) 
		rx++; /* 0D sensor use one rx*/

	for (i = 0; i < tx; i++) {
		for (j = 0; j < rx * 2; j += 2) {
			rawdata[i * rx + j / 2] = (u16)(data[i * rx * 2 + j + 1]) << 8 | (u16)data[i * rx * 2 + j];
		}
	}

	for (i = 0; i < tx; i++) {
		tempvalue = 0;
		tmp = rx;
		if (ddata->is_button_independent) {
			if (i == touchkey_tx[touchkey_index]) {
				tmp = rx;
				touchkey_index++;
			} else {
				tmp = rx - 1;
			}
		}
		for (j = 0; j < tmp; j++) {
			//printk("%d ", rawdata[i * rx + j]);
			tempvalue += rawdata[i * rx + j];
		}
		//printk("\n");

		/* judge the average value of the line */
		if (tempvalue / tmp < RAWDATA_LIMIT) {
			if (!has_print_tx_failed) {
				//printk("average is %d\n", tempvalue / tmp);
				count += sprintf(&buf[count], "TP module failed at tx: ");
				has_print_tx_failed = true;
			}
			count += sprintf(&buf[count], "%d ", i + 1);
		}
	}
	if (has_print_tx_failed) 
		count += sprintf(&buf[count], "\n");

	if (ddata->is_button_independent) {
		tmp = rx - 1;
	} else {
		tmp = rx;
	}

	for (i = 0; i < tmp; i++) {
		tempvalue = 0;
		for (j = 0; j < tx; j++) {
			//printk("%d ", rawdata[i + j * rx]);
			tempvalue += rawdata[i + j * rx];
		}
		//printk("\n");
		if (tempvalue / tx < RAWDATA_LIMIT) {
			if (!has_print_rx_failed) {
				//printk("average is %d\n", tempvalue / tx);
				count += sprintf(&buf[count], "TP module failed at rx: ");
				has_print_rx_failed = true;
			}
			count += sprintf(&buf[count], "%d ", i + 1);
		}
	}

	if (ddata->is_button_independent) {  
		tempvalue = 0;
		for (i = 0; i < touchkey_index; i++) {
			tempvalue += rawdata[tmp + rx * touchkey_tx[i]];
		}
		if (tempvalue / touchkey_index < RAWDATA_LIMIT) {
			if (!has_print_rx_failed) {
				count += sprintf(&buf[count], "TP module failed at rx: %d", rx);
				has_print_rx_failed = true;
			} else {
				count += sprintf(&buf[count], "%d ", rx);
			}
		}
	}
	if (has_print_rx_failed) 
		count += sprintf(&buf[count], "\n");


	if (!has_print_tx_failed && !has_print_rx_failed) {
		count += sprintf(&buf[count], "TP check pass\n");
	}

#if 0
	count += sprintf(&buf[count], "The Rawdata:\n");
	for (i = 0; i < tx; i++) {
		for (j = 0; j < rx; j++) {
			count += sprintf(&buf[count], "%u ", rawdata[i * rx + j]);
		}
		count += sprintf(&buf[count], "\n");
	}
#endif

	kfree(rawdata);
	return count;
}


static ssize_t rmi_null_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
		return -EINVAL;
}
static ssize_t rmi_null_store(struct kobject *kobj,
							struct kobj_attribute *attr,  const char *buf, size_t count)
{
		return -EINVAL;
}


static ssize_t rmi_interrupt_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
//	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
//													kobject_debug);
	return sprintf(buf, "not support\n");
}

static ssize_t rmi_interrupt_enable_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
//	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
//													kobject_debug);
#if 0
	enable_irq(gpio_to_irq(6));
#endif
	return count;
}


static ssize_t rmi_gesture_point_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);

	return sprintf(buf,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \n",
	ddata->buf_gesture[0],ddata->buf_gesture[1],ddata->buf_gesture[2],ddata->buf_gesture[3],ddata->buf_gesture[4],ddata->buf_gesture[5],ddata->buf_gesture[6],ddata->buf_gesture[7],ddata->buf_gesture[8],ddata->buf_gesture[9],ddata->buf_gesture[10],
	ddata->buf_gesture[11],ddata->buf_gesture[12],ddata->buf_gesture[13],ddata->buf_gesture[14],ddata->buf_gesture[15],ddata->buf_gesture[16],ddata->buf_gesture[17],ddata->buf_gesture[18],ddata->buf_gesture[19]);

}

/**wangyong add for all gesture expect down swipe and dclick ***
* 0x01: Lift or Right swipe  0x02: up swipe 	0x04: O 
* 0x08: W			 0x10: M			   0x20: e		0x40: C 
*/

static ssize_t rmi_gesture_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{	
	int val;
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
														kobject_debug);
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]:Invalide number of parameters\n", __func__);
		return -EINVAL;
	}

	if (val < 0x80) {
		ddata->gesture_switch = val;
	}else{
		VIVO_TS_LOG_ERR("[%s]: Invalide parameter %u \n", __func__,val);
		return -EINVAL;
	}
	return count;
}

static ssize_t rmi_gesture_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, 
													kobject_debug);
	return sprintf(buf, "0x01:LR 0x02:up 0x04:O 0x08:W 0x10:M 0x20:e 0x40:C  gesture_switch = 0x%x\n",ddata->gesture_switch);

}

static ssize_t rmi_gesture_switch_export_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{	
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	unsigned int val = 0;
	
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]: Invalide number of parameters\n", __func__);
		return -EINVAL;
	}

	if (val < 0x08) {
		ddata->gesture_switch_export = val;
		VIVO_TS_LOG_ERR("[%s]: write gesture_switch_export with number:%d\n", __func__, val);
	}else{
		VIVO_TS_LOG_ERR("[%s]: Invalide parameter %u \n", __func__, val);
		return -EINVAL;
	}
	return count;
}
static ssize_t rmi_gesture_switch_export_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	return sprintf(buf, "0x02:@ 0x04:f --- gesture_switch_export = 0x%x\n", ddata->gesture_switch_export);
}

//Provide sensor rx tx to Engineering model
static ssize_t rmi_sensor_rx_tx_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct rmi_driver_data *ddata = container_of(kobj, struct rmi_driver_data, kobject_debug);
	struct rmi_function_container *fc11;
	int error;
	int rx, tx ,temp;

	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return -EINVAL;
	}

	error = rmi_f11_get_sensor_electrodes(fc11, &rx, &tx);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get sensor elsectrodes failed\n", __func__);
		return -EIO;
	}
	
	temp = rx<<8 | tx;
	VIVO_TS_LOG_INF("[%s]: rx=%d tx=%d temp = %d\n",__func__,rx,tx,temp);
	return sprintf(buf, "%d",temp);

}


static struct rmi_driver_sysfs_entry rmi_firmware_update =
__ATTR(firmware_update, S_IRUGO|S_IWUSR, 
		rmi_null_show, rmi_firmware_update_store);
static struct rmi_driver_sysfs_entry rmi_firmware_version =
	__ATTR(firmware_version, RMI_RW_ATTR, 
			rmi_version_show, rmi_null_store);
static struct rmi_driver_sysfs_entry rmi_module_color =
	__ATTR(module_color, RMI_RW_ATTR, 
			rmi_color_show, rmi_null_store);

static struct rmi_driver_sysfs_entry rmi_firmware_module_id =
	__ATTR(firmware_module_id, RMI_RW_ATTR,	
			rmi_module_id_show, rmi_null_store);
static struct rmi_driver_sysfs_entry rmi_hardware_id =
	__ATTR(hardware_id, RMI_RW_ATTR,	
			rmi_hardware_id_show, rmi_null_store);

static struct rmi_driver_sysfs_entry rmi_sensor_rx_tx =
	__ATTR(sensor_rx_tx, RMI_RW_ATTR,	
			rmi_sensor_rx_tx_show, rmi_null_store);
static struct rmi_driver_sysfs_entry rmi_sensor_baseline =
	__ATTR(sensor_baseline, RMI_RW_ATTR, 
			rmi_baseline_show, rmi_null_store);
static struct rmi_driver_sysfs_entry rmi_sensor_rawdata =
	__ATTR(sensor_rawdata, RMI_RW_ATTR,	
			rmi_rawdata_show, rmi_null_store);
static struct rmi_driver_sysfs_entry rmi_sensor_delta =
	__ATTR(sensor_delta, RMI_RW_ATTR, 
			rmi_delta_show, rmi_null_store);
static struct rmi_driver_sysfs_entry rmi_read_reg_data =
	__ATTR(read_reg_data, RMI_RW_ATTR, 
			rmi_null_show, rmi_reg_data_store);
static struct rmi_driver_sysfs_entry rmi_ts_log_switch =
	__ATTR(log_switch, RMI_RW_ATTR,
						rmi_log_switch_show, rmi_log_switch_store);
static struct rmi_driver_sysfs_entry rmi_usb_reset_enable_switch =
	__ATTR(usb_reset_enable, RMI_RW_ATTR,
						rmi_usb_reset_enable_show, rmi_usb_reset_enable_store);
												
static struct rmi_driver_sysfs_entry rmi_ts_is_calling =
	__ATTR(ts_is_calling, RMI_RW_ATTR,
						rmi_is_calling_show, rmi_is_calling_store);						

static struct rmi_driver_sysfs_entry rmi_ts_is_GSM_opening =
	__ATTR(ts_is_GSM_opening, RMI_RW_ATTR,
						rmi_is_GSM_opening_show, rmi_is_GSM_opening_store);	
/*
 static struct rmi_driver_sysfs_entry rmi_ts_reset_gpio =
	__ATTR(ts_reset_gpio, 0777,rmi_ts_reset_gpio_show, rmi_ts_reset_gpio_store);
*/

//#if defined(BBK_GLOVES_MODE)
static struct rmi_driver_sysfs_entry rmi_gloves_mode_switch =
	__ATTR(gloves_mode_switch, 0777,rmi_gloves_mode_show, rmi_gloves_mode_store);

static struct rmi_driver_sysfs_entry rmi_gloves_mode_para =
	__ATTR(gloves_mode_para, 0777,rmi_null_show, rmi_gloves_para_store);
//#endif


static struct rmi_driver_sysfs_entry rmi_gesture_point =
	__ATTR(gesture_point, 0777, rmi_gesture_point_show, rmi_null_store);	

static struct rmi_driver_sysfs_entry rmi_gesture_switch =
	__ATTR(gesture_switch, 0777, 
			rmi_gesture_switch_show, rmi_gesture_switch_store);		
static struct rmi_driver_sysfs_entry rmi_gesture_switch_export =
	__ATTR(gesture_switch_export, 0644, rmi_gesture_switch_export_show, rmi_gesture_switch_export_store);	


//#if defined(BBK_DCLICK_WAKE)
static struct rmi_driver_sysfs_entry rmi_ts_ipod_flag =
	__ATTR(ts_ipod_flag, RMI_RW_ATTR, 
			rmi_ipod_flag_show, rmi_ipod_flag_store);
static struct rmi_driver_sysfs_entry rmi_dclick_flag_info =
	__ATTR(dclick_flag_info, RMI_RW_ATTR, 
			rmi_dclick_flag_info_show, rmi_dclick_flag_info_store);
static struct rmi_driver_sysfs_entry rmi_dclick_switch =
	__ATTR(dclick_switch, 0777,
						rmi_dclick_switch_show, rmi_dclick_switch_store);
static struct rmi_driver_sysfs_entry rmi_dclick_simulate_switch =
	__ATTR(dclick_simulate_switch, 0777,
						rmi_dclick_simulate_switch_show, rmi_dclick_simulate_switch_store);
static struct rmi_driver_sysfs_entry rmi_dclick_proximity_switch =
	__ATTR(dclick_proximity_switch, 0777,
						rmi_null_show, rmi_dclick_proximity_switch_store);
static struct rmi_driver_sysfs_entry rmi_dclick_lcd_state =
	__ATTR(dclick_lcd_state, 0777,
						rmi_dclick_lcd_state_show, rmi_dclick_lcd_state_store);
static struct rmi_driver_sysfs_entry rmi_swipe_down_switch =
	__ATTR(swipe_switch, 0777,
						rmi_swipe_down_switch_show, rmi_swipe_down_switch_store);
//#endif

static struct rmi_driver_sysfs_entry rmi_charger_connect_flag =
	__ATTR(charger_connect_flag, RMI_RO_ATTR, 
			rmi_charger_flag_show, rmi_null_store);

static struct rmi_driver_sysfs_entry rmi_touchscreen_module_check =
	__ATTR(touchscreen_module_check, RMI_RO_ATTR, 
			rmi_touchscreen_module_state_show, NULL);

static struct rmi_driver_sysfs_entry rmi_interrupt_enable =
	__ATTR(interrupt_enable, 0777,
						rmi_interrupt_enable_show, rmi_interrupt_enable_store);

static struct attribute *our_own_sys_attrs[] = {
	&rmi_firmware_update.attr,
	&rmi_firmware_version.attr,
	&rmi_module_color.attr,
	&rmi_sensor_rx_tx.attr,
	&rmi_firmware_module_id.attr,
	&rmi_hardware_id.attr,
	&rmi_sensor_baseline.attr,
	&rmi_sensor_rawdata.attr,
	&rmi_sensor_delta.attr,
	&rmi_read_reg_data.attr,
	&rmi_ts_log_switch.attr,
	&rmi_usb_reset_enable_switch.attr,
	&rmi_ts_is_calling.attr,
	&rmi_ts_is_GSM_opening.attr,
//	&rmi_ts_reset_gpio.attr,
	&rmi_interrupt_enable.attr,
	&rmi_touchscreen_module_check.attr,
	NULL
};

//#if defined(BBK_GLOVES_MODE)
static const struct attribute *rmi_gloves_sys_attrs[] = {
	&rmi_gloves_mode_switch.attr,	
	&rmi_gloves_mode_para.attr,
	NULL
};
//#endif

//#if defined(BBK_DCLICK_WAKE)
static const struct attribute *rmi_dclick_wake_sys_attrs[] = {
	&rmi_ts_ipod_flag.attr,
	&rmi_dclick_flag_info.attr,
	&rmi_dclick_switch.attr,
	&rmi_dclick_simulate_switch.attr,
	&rmi_dclick_proximity_switch.attr,
	&rmi_dclick_lcd_state.attr,
	&rmi_swipe_down_switch.attr,
	&rmi_gesture_point.attr,
    	&rmi_gesture_switch.attr,
    	&rmi_gesture_switch_export.attr,
	NULL
};
//#endif

static const struct attribute *rmi_charger_sys_attrs[] = {
	&rmi_charger_connect_flag.attr,
	NULL
};

static ssize_t rmi_debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t rmi_debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void rmi_debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops rmi_debug_object_sysfs_ops = {
	.show = rmi_debug_object_show,
	.store = rmi_debug_object_store,
};
static struct kobj_type rmi_debug_object_type = {
	.sysfs_ops	= &rmi_debug_object_sysfs_ops,
	.release	= rmi_debug_object_release,
	.default_attrs = our_own_sys_attrs,
};

static int rmi_creat_our_own_sys_file(struct rmi_driver_data *data) 
{ 
    int ret; 

	ret = kobject_init_and_add(&data->kobject_debug, &rmi_debug_object_type,
					NULL, "touchscreen");
    if (ret) 
    { 
        VIVO_TS_LOG_ERR("[%s]: Create kobjetct error!\n", __func__); 
        return -1; 
    } 
	
	if(vivo_touchscreen_is_support(FS_GLOVES_MODE)) {
		ret = sysfs_create_files(&data->kobject_debug,rmi_gloves_sys_attrs);
		if (ret) { 
			VIVO_TS_LOG_ERR("[%s]: Create rmi_gloves_sys_attrs error!\n", __func__); 
			return -1; 
        }
    }		
	
	if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		ret = sysfs_create_files(&data->kobject_debug,rmi_dclick_wake_sys_attrs);
		if (ret) { 
			VIVO_TS_LOG_ERR("[%s]: Create rmi_dclick_wake_sys_attrs error!\n", __func__); 
			return -1; 
        }
    }	

     if(vivo_touchscreen_is_support(FS_CHR_CONN_JUDGE)) {
		ret = sysfs_create_files(&data->kobject_debug,rmi_charger_sys_attrs);
		if (ret) { 
			VIVO_TS_LOG_ERR("[%s]: Create rmi_dclick_wake_sys_attrs error!\n", __func__); 
			return -1; 
        }
    }	
	
    return 0; 
} 

static void rmi_log_switch_set(bool on)
{
	global_ddata->log_switch = on;
	global_ddata->usb_reset_enable_switch = on;/*Set usb_reset_enable_switch Qiuguifu add.*/
}


struct bbk_drivers_callback_handler rmi_log_switch_handler = {
	.name = "rmi_driver",
	.callback = rmi_log_switch_set,
};


//#if defined(BBK_GLOVES_MODE)
static enum hrtimer_restart gloves_mode_func(struct hrtimer *timer)
{
	struct rmi_driver_data *data = container_of(timer,
					struct rmi_driver_data, reset_timer);
	schedule_work(&(data->reset_work));
	return HRTIMER_NORESTART;
}
static void gloves_mode_setting_worker(struct work_struct *work)
{
	struct rmi_driver_data *data = container_of(work,struct rmi_driver_data,reset_work);

	set_gloves_mode_para(data,data->ts_gloves_mode);	
}

//#endif

#if defined(WATCH_DOG_TIMER)
static enum hrtimer_restart watch_dog_func(struct hrtimer *timer)
{
	struct rmi_driver_data *data = container_of(timer,
					struct rmi_driver_data, watch_dog_timer);
	if(data == NULL ||  global_pdata == NULL)
		return HRTIMER_NORESTART;

	if(1 == data->log_switch)
	VIVO_TS_LOG_INF("[%s]: .\n",__func__);

	if( gpio_get_value(global_pdata->attn_gpio)  == 1 )
	{
		if(1 == data->log_switch)
		VIVO_TS_LOG_INF("[%s]: Atten_gpio = %d.\n",__func__,gpio_get_value(global_pdata->attn_gpio));
		start_watch_dog_timer(data );
	}
	else
	{
		VIVO_TS_LOG_INF("[%s]:  Atten_gpio = %d..\n",__func__,gpio_get_value(global_pdata->attn_gpio));
		schedule_work(&(data->watch_dog_work));
	}
		
	return HRTIMER_NORESTART;
}
 
static void watch_dog_worker(struct work_struct *work)
{
	struct rmi_driver_data *data = container_of(work,struct rmi_driver_data,watch_dog_work);
	
	u8 irq_status[data->num_of_irq_regs];
	int error = 0;
	int i = 0;;
	VIVO_TS_LOG_INF("[%s]: Atten is 0...Time = %d\n",__func__,i);

	for(i = 0;i <= 100;i++)
	{
		msleep(5);
		if(gpio_get_value(global_pdata->attn_gpio)  == 0 )
		{
			if(i == 100)
			{
				VIVO_TS_LOG_INF("[%s]: Atten is 0...Time = %d\n",__func__,i);
				if(data->rmi_dev && data->f01_container)
				{
					error = rmi_read_block(data->rmi_dev,
					data->f01_container->fd.data_base_addr + 1,
					irq_status, data->num_of_irq_regs);
					if (error < 0) {
						VIVO_TS_LOG_ERR("[%s]:Failed to read irqs, code=%d\n", __func__, error);
					}
				}else
				{
					VIVO_TS_LOG_ERR("[%s]: data is not ready for iic read.\n", __func__);
				}
			}
			continue;
		}
		else
		{
			break;
		}
	}
	start_watch_dog_timer(data );
}

#endif
static void charger_connect_worker(struct work_struct *work)
{
	struct rmi_driver_data *data = container_of(work,struct rmi_driver_data,charger_work);
	int retval;

	if(is_i2c_probe_done == 1) {

		if(data && data->f01_container)
		{		
			VIVO_TS_LOG_INF("[%s]: detect charger and rmi_driver_probe is done.reset...\n", __func__);	
			#if 1
			retval = rmi_write(data->rmi_dev, data->f01_container->fd.command_base_addr, RMI_DEVICE_RESET_CMD);
			if (retval < 0)
				VIVO_TS_LOG_ERR("[%s]:==charger_connect reset failed, code: %d.\n", __func__, retval);
			#else
			
			mutex_lock(&global_ddata->syna_i2c_reset_mutex);

			gpio_direction_output(data->rst_gpio, 0);
			msleep(10);
			gpio_direction_output(data->rst_gpio, 1);
			mdelay(60);
			
			mutex_unlock(&global_ddata->syna_i2c_reset_mutex);
			#endif
		}
	} else {
		VIVO_TS_LOG_INF("[%s]: detect charger but rmi_driver_probe not done\n", __func__);	
	}
	
}

#if 0   //wangyong
static void charger_connect_worker(struct work_struct *work)
{
	struct rmi_driver_data *data = container_of(work,struct rmi_driver_data,charger_work);
	int retval;

	if(data->early_suspended)
	{
		//USB connect or disconnect, TP will resume and reset, so here do nothing.
		VIVO_TS_LOG_ERR("[%s]:system is suspend, USB connect or disconnect do nothing!\n", __func__);
	}
	else
	{
		if(0 == data->charger_connect_flag)
		{

			VIVO_TS_LOG_INF("[%s]:=====USB  is disconnect  and TP Resetting.... f01.command_base_addr = %x===\n", __func__, global_ddata->f01_container->fd.command_base_addr);
			retval = rmi_write(data->rmi_dev, data->f01_container->fd.command_base_addr, RMI_DEVICE_RESET_CMD);
			if (retval < 0)
				VIVO_TS_LOG_ERR("[%s]:==WARNING - post-flash reset failed, code: %d.\n", __func__, retval);
			msleep(DEFAULT_RESET_DELAY_MS);
			rmi_f11_finger_realse(data);
			if(data->ts_gloves_mode)
			{
				set_gloves_mode_para(data,data->ts_gloves_mode);
			}

		}
		else
		{

			VIVO_TS_LOG_INF("[%s]:=====USB  is disconnect  and TP Resetting.... f01.command_base_addr = %x===\n", __func__, global_ddata->f01_container->fd.command_base_addr);
			retval = rmi_write(data->rmi_dev, data->f01_container->fd.command_base_addr, RMI_DEVICE_RESET_CMD);
			if (retval < 0)
			VIVO_TS_LOG_INF("[%s]:==WARNING - post-flash reset failed, code: %d.\n", __func__,retval);
			msleep(DEFAULT_RESET_DELAY_MS);
			rmi_f11_finger_realse(data);
			if(data->ts_gloves_mode)
			{
				set_gloves_mode_para(data,data->ts_gloves_mode);
			}

		}
	}
		
}

#endif 

static void charger_connect_judge(char on_or_off)
{
	//wangyong add

	if (!global_ddata) {
		VIVO_TS_LOG_ERR("[%s]: no valide driver data\n", __func__);
		charger_flag = on_or_off;
		return;
	}
	
	if (usb_charge_connected == on_or_off) {
		return;
	}
	usb_charge_connected = on_or_off;

	VIVO_TS_LOG_INF("[%s]: USB connect = %d =======\n",__func__,on_or_off);
	global_ddata->charger_connect_flag = on_or_off;
	if(global_ddata->usb_reset_enable_switch == 1)
	{
		VIVO_TS_LOG_INF("[%s]:usb_reset_enable_switch = 1.So no reset\n", __func__);
		return ;
	}
	if(!global_ddata->early_suspended)
	{
		schedule_work(&(global_ddata->charger_work));
	}	
}



#if 0
static void do_update_firmware_again(void)
{
	struct pdt_entry f34_pdt, f01_pdt;
	struct rmi_function_container *entry;
	struct rmi_driver_data *data = global_ddata;

	if (get_is_update_need_redo()) {
		VIVO_TS_LOG_ERR("[%s]:Redo the update process\n", __func__);
		
		entry = data->f01_container;
		if (!entry) {
			VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
			return;
		}
		f01_pdt.query_base_addr = entry->fd.query_base_addr;
		f01_pdt.command_base_addr = entry->fd.command_base_addr;
		f01_pdt.control_base_addr = entry->fd.control_base_addr;
		f01_pdt.data_base_addr = entry->fd.data_base_addr;

		entry = rmi_get_function_container(data, 0x34);
		if (!entry) {
			VIVO_TS_LOG_ERR("[%s]:Can't get 0x34 function container\n", __func__);
			return;
		}
		f34_pdt.query_base_addr = entry->fd.query_base_addr;
		f34_pdt.command_base_addr = entry->fd.command_base_addr;
		f34_pdt.control_base_addr = entry->fd.control_base_addr;
		f34_pdt.data_base_addr = entry->fd.data_base_addr;
		(void)rmi4_fw_update(data->rmi_dev, &f01_pdt, &f34_pdt, NULL);
	}
}
/* 
static void redo_update_firmware_work_func(struct work_struct *work) {
	do_update_firmware_again();
} */
void redo_update_firmware(void) 
{
	do_update_firmware_again();
	//schedule_work(&redo_fw_update_work);
}
#endif

//add for charge judge and log_switch interface. gtp_common_data will be register in probe
static int rmi_get_log_switch(void)
{
	return global_ddata->log_switch;
}
static vivo_touchscreen_common_data rmi_common_data = {
	.driver_name = DRIVER_NAME,
	.charge_connect_judge = charger_connect_judge,
    .get_ts_log_switch = rmi_get_log_switch,
};

static int rmi_driver_probe(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = NULL;
	struct rmi_function_container *fc;
	struct rmi_device_platform_data *pdata;
	int i,retval = 0;
	struct device *dev = &rmi_dev->dev;
	int attr_count = 0;
	
	VIVO_TS_LOG_DBG("[%s]: Starting probe.\n", __func__);
	
	pdata = to_rmi_platform_data(rmi_dev);

	data = kzalloc(sizeof(struct rmi_driver_data), GFP_KERNEL);
	if (!data) {
		VIVO_TS_LOG_ERR("[%s]: Failed to allocate driver data.\n", __func__);
		return -ENOMEM;
	}
	wake_lock_init(&data->wakelock, WAKE_LOCK_SUSPEND, "smartwake");

//#if defined(BBK_DCLICK_WAKE)
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		data->ts_dclick_switch = 1;
		data->ts_dclick_simulate_switch = 0;
		atomic_set(&data->ts_state, TOUCHSCREEN_NORMAL);
	}
//#endif
	//wangyong add gesture switch
	data->gesture_switch = 0;
	
	data->swipe_down_switch = 0;
	//end
	data->is_GSM_opening = 0;
//#if defined(BBK_GLOVES_MODE)
    if(vivo_touchscreen_is_support(FS_GLOVES_MODE)) {
		data->ts_gloves_mode = 0;
		hrtimer_init(&data->reset_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		data->reset_timer.function = gloves_mode_func;
		INIT_WORK(&data->reset_work, gloves_mode_setting_worker);
	}
//#endif

	//wangyong add for usb noise
	if(vivo_touchscreen_is_support(FS_CHR_CONN_JUDGE)) {
		INIT_WORK(&data->charger_work, charger_connect_worker);
	}
	if (-1 == register_touchscreen_common_interface(&rmi_common_data)) {
		goto err_create_synaptics_wq;
	}
	
	data->irq_err_workqueue = create_singlethread_workqueue("synaptics_wq");
	if (!data->irq_err_workqueue) {
		VIVO_TS_LOG_ERR("[%s]: can't create irq err worqueue\n",
				__func__);
		//return -ENOMEM;
		goto err_create_synaptics_wq;
	}
	INIT_WORK(&data->irq_err_work, rmi4_driver_irq_err_work_func);
	
#if defined(WATCH_DOG_TIMER)
	hrtimer_init(&data->watch_dog_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->watch_dog_timer.function = watch_dog_func;
	INIT_WORK(&data->watch_dog_work, watch_dog_worker);
#endif
	INIT_WORK(&data->wait_resetirq_work, wait_resetirq_worker);
	init_waitqueue_head(&data->resetirq_wait);
	data->wakeup_event = WAKEUP_NONE;
	data->rst_gpio = pdata->rst_gpio;
	global_pdata = pdata;
	/* for don't report event when lcd off */
	global_ddata = data;
	data->rmi_dev = rmi_dev;
	data->has_lcd_shutoff = false;
	data->is_button_independent = true;
	
	INIT_LIST_HEAD(&data->rmi_functions.list);
	rmi_set_driverdata(rmi_dev, data);
	mutex_init(&data->pdt_mutex);
	data->AA_area_point_pressed = 0;

	data->largetouch_flag = 0;
	data->mute_switch =1;
	data->charger_connect_flag = 0;
	data->usb_reset_enable_switch = 0;
	data->probe_over_flag = 0;
	
	mutex_init(&data->syna_i2c_reset_mutex);
	wake_lock_init(&data->suspend_wakelock, WAKE_LOCK_SUSPEND, "smartwake");
	
	//wangyong add for gesture
	for(i=0; i<20; i++)
	{
		data->buf_gesture[i]= 65535;
	}
	
	if (!pdata->reset_delay_ms)
		pdata->reset_delay_ms = DEFAULT_RESET_DELAY_MS;
	retval = do_initial_reset(rmi_dev);
	if (retval)
		VIVO_TS_LOG_ERR("[%s]:RMI initial reset failed! Soldiering on.\n", __func__);


	retval = rmi_scan_pdt(rmi_dev);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:PDT scan for %s failed with code %d.\n", __func__,
			pdata->sensor_name, retval);
		goto err_free_data_scan_pdt;
	}

	if (!data->f01_container) {
		VIVO_TS_LOG_ERR("[%s]:missing F01 container!\n", __func__);
		retval = -EINVAL;
		goto err_free_data_scan_pdt;
	}

	data->f01_container->irq_mask = kzalloc(
			sizeof(u8) * data->num_of_irq_regs, GFP_KERNEL);
	if (!data->f01_container->irq_mask) {
		VIVO_TS_LOG_ERR("[%s]:Failed to allocate F01 IRQ mask.\n", __func__);
		retval = -ENOMEM;
		goto err_free_data_scan_pdt;
	}
	construct_mask(data->f01_container->irq_mask,
		       data->f01_container->num_of_irqs,
		       data->f01_container->irq_pos);
	list_for_each_entry(fc, &data->rmi_functions.list, list)
		fc->irq_mask = rmi_driver_irq_get_mask(rmi_dev, fc);

	retval = rmi_driver_f01_init(rmi_dev);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to initialize F01.\n", __func__);
		goto err_free_data_scan_pdt;
	}

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION,
			 data->pdt_props.regs);
	if (retval < 0) {
		/* we'll print out a warning and continue since
		 * failure to get the PDT properties is not a cause to fail
		 */
		VIVO_TS_LOG_ERR("[%s]:Could not read PDT properties from 0x%04x. "
			 "Assuming 0x00.\n", __func__, PDT_PROPERTIES_LOCATION);
	}

	VIVO_TS_LOG_DBG("[%s]: Creating sysfs files.", __func__);
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = device_create_file(dev, &attrs[attr_count]);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: Failed to create sysfs file %s.\n",
				__func__, attrs[attr_count].attr.name);
				goto err_free_data_create_file_attrs;
		}
	}

	retval = rmi_creat_our_own_sys_file(data);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: Failed to create our own sysfs file\n",
				__func__);
		goto err_free_data_create_file_attrs;
	}
	
	if (data->pdt_props.has_bsr) {
		retval = device_create_file(dev, &bsr_attribute);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: Failed to create sysfs file bsr.\n",
				__func__);
			goto err_free_data_cteate_file_bsr_attribute;
		}
	}

	mutex_init(&data->irq_mutex);
	data->current_irq_mask = kzalloc(sizeof(u8) * data->num_of_irq_regs,
					 GFP_KERNEL);
	if (!data->current_irq_mask) {
		VIVO_TS_LOG_ERR("[%s]:Failed to allocate current_irq_mask.\n", __func__);
		retval = -ENOMEM;
		goto err_free_data_irq_mask;
	}
	retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->current_irq_mask, data->num_of_irq_regs);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: Failed to read current IRQ mask.\n",
			__func__);
		goto err_free_data_irq_mask;
	}
	data->irq_mask_store = kzalloc(sizeof(u8) * data->num_of_irq_regs,
				       GFP_KERNEL);
	if (!data->irq_mask_store) {
		VIVO_TS_LOG_ERR("[%s]:Failed to allocate mask store.\n", __func__);
		retval = -ENOMEM;
		goto err_free_data;
	}

	data->AA_area_point_release_time = jiffies;
	
	data->lcd_dimension_x = pdata->lcd_dimension_x;
	data->lcd_dimension_y = pdata->lcd_dimension_y;
	data->ts_dimension_x = pdata->ts_dimension_x;
	data->ts_dimension_y = pdata->ts_dimension_y;

#ifdef	CONFIG_PM
	data->pm_data = pdata->pm_data;
	data->pre_suspend = pdata->pre_suspend;
	data->post_suspend = pdata->post_suspend;
	data->pre_resume = pdata->pre_resume;
	data->post_resume = pdata->post_resume;

	mutex_init(&data->suspend_mutex);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	rmi_dev->early_suspend_handler.level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi_dev->early_suspend_handler.suspend = rmi_driver_early_suspend;
	rmi_dev->early_suspend_handler.resume = rmi_driver_late_resume;
	register_early_suspend(&rmi_dev->early_suspend_handler);
#elif defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&data->fb_notif);

	if (retval)
		VIVO_TS_LOG_ERR("[%s]:Unable to register fb_notifier: %d\n", __func__,
			retval);
#endif /* CONFIG_HAS_EARLYSUSPEND */

#endif /* CONFIG_PM */
	data->enabled = true;

#ifdef CONFIG_RMI4_DEBUG
	retval = setup_debugfs(rmi_dev);
	if (retval < 0)
		VIVO_TS_LOG_ERR("[%s]:Failed to setup debugfs. Code: %d.\n", __func__,
			 retval);
#endif

  retval = bbk_drivers_info_switch_register_callback(&rmi_log_switch_handler);

	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to register callback handler\n", __func__);
	}

	//wangyong del rmi_f11_close_fast_relax(rmi_dev);		/* wangyuanliang add temply for disable relax*/
	data->probe_over_flag = 1;
	
	//wangyong add for charger connect noise handle
	 if(vivo_touchscreen_is_support(FS_CHR_CONN_JUDGE)) {
		if(usb_charger_flag) {
			charger_connect_judge(1);
			VIVO_TS_LOG_INF("[%s]:Charger Bit was written before probe\n", __func__);
		}
	}

	//wangyong add end
#if  0
	disable_irq_nosync(gpio_to_irq(6));
#endif
	#if defined(WATCH_DOG_TIMER)
		start_watch_dog_timer(data );
	#endif

	VIVO_TS_LOG_INF("[%s]:SYNA rmi_driver_probe finished!\n", __func__);

	return 0;

 err_free_data:
 	VIVO_TS_LOG_ERR("[%s]:rmi   err_free_data\n",__func__);
	kfree(data->irq_mask_store);	
err_free_data_irq_mask:
	VIVO_TS_LOG_ERR("[%s]:rmi   err_free_data_irq_maskn",__func__);
	kfree(data->current_irq_mask);
err_free_data_cteate_file_bsr_attribute:
	VIVO_TS_LOG_ERR("[%s]:rmi   err_free_data_cteate_file_bsr_attribute\n",__func__);
	if (data->pdt_props.has_bsr)
		device_remove_file(dev, &bsr_attribute);	
err_free_data_create_file_attrs:
	for (attr_count--; attr_count >= 0; attr_count--)
		device_remove_file(dev, &attrs[attr_count]);	
//err_free_data_f01_init:
	VIVO_TS_LOG_ERR("[%s]:rmi err_free_data_create_file_attrs\n",__func__);
	//if (data->f01_container){
	//	kfree(data->f01_container->irq_mask);
	//	dev_err(dev, "rmi  reboot test failed 01 : %s\n",__func__);
	//	}
err_free_data_scan_pdt:
	rmi_free_function_list(rmi_dev);

	VIVO_TS_LOG_ERR("[%s]:rmi  err_free_data_scan_pdt\n",__func__);
	destroy_workqueue(data->irq_err_workqueue);
err_create_synaptics_wq:
	VIVO_TS_LOG_ERR("[%s]:rmi  err_create_synaptics_wq\n",__func__);
	wake_lock_destroy(&data->wakelock);
	kfree(data);
	global_ddata = NULL;
	rmi_set_driverdata(rmi_dev, NULL);
	return retval;
}
static void disable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	rmi_dev->phys->disable_device(rmi_dev->phys);

	data->enabled = false;
}

static int enable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	int retval = 0;

	retval = rmi_dev->phys->enable_device(rmi_dev->phys);
	if (retval)
		return retval;

	data->enabled = true;

	return 0;
}

//#if defined(BBK_DCLICK_WAKE)
void ts_scan_switch(bool on)
{
	struct rmi_driver_data *ddata = global_ddata;
	struct rmi_function_container *f01, *f11;
	int retval;
	#if 0
	int old_suspend_state = global_ddata->early_suspended;
	#endif
	VIVO_TS_LOG_INF("[%s]: parameter is %d~~~\n", __func__, on);

	if (!ddata) {
		VIVO_TS_LOG_ERR("[%s]: rmi driver data is NULL\n", __func__);
		return;
	}

	f01 = ddata->f01_container;
	if (!f01) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x01 function container\n", __func__);
		return;
	}

	f11 = rmi_get_function_container(ddata, 0x11);
	if (!f11) {
		VIVO_TS_LOG_ERR("[%s]:Can't get 0x11 function container\n", __func__);
		return; 
	}

	if (on) {
		atomic_set(&ddata->ts_state, TOUCHSCREEN_GESTURE);
		#if 1
		retval = rmi_write(ddata->rmi_dev, f01->fd.command_base_addr, RMI_DEVICE_RESET_CMD);
		if (retval < 0)
			VIVO_TS_LOG_ERR("[%s]: reset failed, code: %d.\n",__func__,retval);
		#else
		VIVO_TS_LOG_INF("[%s]: reset TP==\n",__func__); 
		
		mutex_lock(&global_ddata->syna_i2c_reset_mutex);
		gpio_direction_output(ddata->rst_gpio, 0);
		msleep(10);
		gpio_direction_output(ddata->rst_gpio, 1);
		mdelay(60);	
		mutex_unlock(&global_ddata->syna_i2c_reset_mutex);
		#endif
		#if 0
		msleep(DEFAULT_RESET_DELAY_MS);

		if (old_suspend_state != global_ddata->early_suspended)
			return;
		if (rmi_f01_change_to_sleep_mode(f01, false)) {
			VIVO_TS_LOG_ERR("[%s]:Failed to change sleep mode-false!\n", __func__);
		}

		msleep(10);

		if (rmi_f11_change_report_mode_to_dclick(f11, true)) {
			VIVO_TS_LOG_ERR("[%s]:Failed to change report mode-true!\n", __func__);
		}
		#endif
		
	   //rmi_driver_clear_irq_status(ddata->rmi_dev, ddata);
	}else{
		//#if defined(BBK_DCLICK_WAKE)
		if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			if (rmi_f11_change_report_mode_to_dclick(f11, false)) {
				VIVO_TS_LOG_ERR("[%s]:Failed to change report mode-false!\n", __func__);
			}
		}
		//#endif

		msleep(10);

		if (rmi_f01_change_to_sleep_mode(f01, true)) {
			VIVO_TS_LOG_ERR("[%s]:Failed to change sleep mode-true!\n", __func__);
		}
		atomic_set(&ddata->ts_state, TOUCHSCREEN_SLEEP);
	}
	
}
//#endif

#ifdef CONFIG_PM
static int standard_suspend(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int retval = 0;

	data = rmi_get_driverdata(rmi_dev);

	mutex_lock(&data->suspend_mutex);
	if (data->suspended)
		goto exit;

#if !defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND)
	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval)
			goto exit;
	}
#endif  /* !CONFIG_HAS_EARLYSUSPEND */

//#if !defined(BBK_DCLICK_WAKE)
    if(!vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		disable_sensor(rmi_dev);
	}
//#endif

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->suspend) {
			retval = entry->fh->suspend(entry);
			if (retval < 0)
				goto exit;
		}

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->suspend) {
		retval = data->f01_container->fh->suspend(data->f01_container);
		if (retval < 0)
			goto exit;
	}

	data->suspended = true;
	data->largetouch_flag = 0;
	data->mute_switch = 0;
	

	if (data->post_suspend)
		retval = data->post_suspend(data->pm_data);

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

static int standard_resume(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int retval = 0;
	data = rmi_get_driverdata(rmi_dev);

	//wake_unlock(&ps_suspend_lock);

	mutex_lock(&data->suspend_mutex);
	if (!data->suspended)
		goto exit;

	if (data->pre_resume) {
		retval = data->pre_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->resume) {
		retval = data->f01_container->fh->resume(data->f01_container);
		if (retval < 0)
			goto exit;
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->resume) {
			retval = entry->fh->resume(entry);
			if (retval < 0)
				goto exit;
		}

//#if !defined(BBK_DCLICK_WAKE)
    if(!vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		retval = enable_sensor(rmi_dev);
		if (retval)
			goto exit;
	}
//#endif

#if !defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND)
	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval)
			goto exit;
	}
#endif

	data->suspended = false;
	data->largetouch_flag = 0;
	data->mute_switch = 1;

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

#if defined(WATCH_DOG_TIMER)
static void stop_watch_dog_timer(struct rmi_driver_data *data )
{
	if(data == NULL)
		return ;
	VIVO_TS_LOG_DBG("[%s]: hrtimer and work cancel is call! \n",__func__);
	hrtimer_cancel(&data->watch_dog_timer);	
	cancel_work_sync(&data->watch_dog_work);
	hrtimer_cancel(&data->watch_dog_timer);	
}
static void start_watch_dog_timer(struct rmi_driver_data *data )
{
	if(data == NULL)
		return ;
	VIVO_TS_LOG_DBG("[%s]: hrtimer_start is call! \n",__func__);
	hrtimer_start(&data->watch_dog_timer, ktime_set(0, 2000*1000*1000),HRTIMER_MODE_REL);
}
#endif
static void wait_resetirq_worker(struct work_struct *work)
{
	struct rmi_driver_data *data = container_of(work,struct rmi_driver_data,wait_resetirq_work);
	int timeout = WAIT_RESETIRQ_TIMELIMIT;

	timeout = wait_event_interruptible_timeout(data->resetirq_wait,
						data->wakeup_event != WAKEUP_NONE, 
						WAIT_RESETIRQ_TIMELIMIT);

	if (timeout == 0) {
		VIVO_TS_LOG_INF("[%s]: reset cmd no effect\n", __func__);

		/* no sync consider for it will only effect suspend one time */
		/* for only use in 1225 use the constant gpio num */
		gpio_direction_output(data->rst_gpio, 0);
		msleep(10);
		gpio_direction_output(data->rst_gpio, 1);
		/* no sleep needed for ic will not communicate with cpu */
	} else {
		VIVO_TS_LOG_INF("[%s]: reset cmd worked wakeup by %d\n", __func__, data->wakeup_event);
		/* nothing to do */
		//if (atomic_read(&data->wakeup_event) == WAKEUP_BY_SUSPEND) {
		//}
	}
}
#if defined(CONFIG_FB) || (defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND))
static int rmi_suspend(struct rmi_driver_data *ddata)
{
//	struct rmi_device *rmi_dev = ddata->rmi_dev;
	struct rmi_driver_data *data = ddata;
	struct rmi_function_container *entry;
	int retval = 0;

	VIVO_TS_LOG_INF("[%s]: is begain to called! \n",__func__);  //wangyong add for test
	if (data->wakeup_event == WAKEUP_NONE) {
		/* for may suspend when reset irq not happened make the gpio reset not happend */
		VIVO_TS_LOG_INF("[%s]: reset irq is waiting wake up it\n", __func__);
		data->wakeup_event = WAKEUP_BY_SUSPEND;
		wake_up_interruptible(&data->resetirq_wait);
	}

	//#if defined(BBK_GLOVES_MODE)
	if(vivo_touchscreen_is_support(FS_GLOVES_MODE)) {
		if(1 == hrtimer_active(&data->reset_timer))
		{		
			VIVO_TS_LOG_ERR("[%s]: hrtimer_cancel is call! \n",__func__);
			hrtimer_cancel(&data->reset_timer);	
		}
	}
	//#endif
	#if defined WATCH_DOG_TIMER		
		stop_watch_dog_timer(data);
	#endif
	mutex_lock(&data->suspend_mutex);
	if (data->early_suspended)
		goto exit;

	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval) {
			VIVO_TS_LOG_ERR("[%s]:Presuspend failed with %d.\n", __func__,
				retval);
			goto exit;
		}
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->early_suspend) {
			retval = entry->fh->early_suspend(entry);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]:F%02x early suspend failed with %d.\n", __func__,
					entry->fd.function_number, retval);
				goto exit;
			}
		}

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->early_suspend) {
		retval = data->f01_container->fh->early_suspend(
				data->f01_container);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]:[SYNA]F01 early suspend failed with %d.\n", __func__,
				retval);
			goto exit;
		}
	}

//#if defined(BBK_DCLICK_WAKE)
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		if (data->need_change_to_dclick == 1) {
			data->need_change_to_dclick = 0;
			ts_scan_switch(1);
		}else{
			atomic_set(&data->ts_state, TOUCHSCREEN_SLEEP);
		}

		if(data->fast_lcd_shutoff){
			VIVO_TS_LOG_INF("[%s]:enter fast_lcd_shutoff = %d",__func__,data->fast_lcd_shutoff);
			if(data->proximity_state)
			{
				ts_scan_switch(1);
			}
			else 
			{
				ts_scan_switch(0);
			}
			data->fast_lcd_shutoff = 0;
		}
	}
//#endif

	data->early_suspended = true;
	VIVO_TS_LOG_INF("[%s]: is end !\n",__func__);  //wangyong add for test
exit:
	mutex_unlock(&data->suspend_mutex);

	return retval;

}

static int rmi_resume(struct rmi_driver_data *ddata)
{
	struct rmi_device *rmi_dev = ddata->rmi_dev;
	struct rmi_driver_data *data = ddata;
	struct rmi_function_container *entry;
	int retval = 0;

	//wake_unlock(&ps_suspend_lock);

	VIVO_TS_LOG_INF("[%s]: is begain to called! \n",__func__);  //wangyong add for test
	
	mutex_lock(&data->suspend_mutex);
	/* when in late resume the proximity should not change ts state */
//#if defined(BBK_DCLICK_WAKE)
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		atomic_set(&data->ts_state, TOUCHSCREEN_NORMAL);
		data->need_change_to_dclick = 0;
	}
//#endif
	if(data->has_lcd_shutoff )
		data->fast_lcd_shutoff = 1;
	if (!data->early_suspended)
		goto exit;

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->late_resume) {
		retval = data->f01_container->fh->late_resume(
				data->f01_container);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]:F01 late resume failed with %d.\n", __func__,
				retval);
			goto exit;
		}
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->late_resume) {
			retval = entry->fh->late_resume(entry);
			if (retval < 0) {
				VIVO_TS_LOG_ERR("[%s]:F%02X late resume failed with %d.\n", __func__,
					entry->fd.function_number, retval);
				goto exit;
			}
		}

	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval) {
			VIVO_TS_LOG_ERR("[%s]:Post resume failed with %d.\n", __func__,
				retval);
			goto exit;
		}
	}

	data->early_suspended = false;
	
	if (data->f01_container)
	{
		#if 1
		retval = rmi_write(rmi_dev, data->f01_container->fd.command_base_addr, RMI_DEVICE_RESET_CMD);
		if (retval < 0)
			VIVO_TS_LOG_ERR("[%s]: reset failed, code: %d.\n",__func__,retval);
		VIVO_TS_LOG_INF("[%s]: reset TP==\n",__func__);
		#else
		mutex_lock(&global_ddata->syna_i2c_reset_mutex);
		gpio_direction_output(data->rst_gpio, 0);
		msleep(10);
		gpio_direction_output(data->rst_gpio, 1);
		mdelay(60);	
		mutex_unlock(&global_ddata->syna_i2c_reset_mutex);
		#endif
		
		//msleep(DEFAULT_RESET_DELAY_MS);
		//#if defined(BBK_GLOVES_MODE)
		if(vivo_touchscreen_is_support(FS_GLOVES_MODE)) {
			if(0)
				hrtimer_start(&data->reset_timer, ktime_set(0, DEFAULT_RESET_DELAY_MS*1000*1000),HRTIMER_MODE_REL);
		}
		//#endif
		
		#if defined(WATCH_DOG_TIMER)
			start_watch_dog_timer(data);
		#endif

		rmi_f11_finger_realse(data);
	}
	else
	{
		rmi_driver_clear_irq_status(rmi_dev, data);
	}

	data->wakeup_event = WAKEUP_NONE;
	schedule_work(&data->wait_resetirq_work);

	VIVO_TS_LOG_INF("[%s]: is end to called! \n",__func__);  //wangyong add for test

	//wangyong del rmi_f11_close_fast_relax(rmi_dev);	/* wangyuanliang add temply for disable relax*/
exit:
	mutex_unlock(&data->suspend_mutex);

	return retval;
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND)

static void standard_early_suspend(struct early_suspend *h)
{
    struct rmi_device *rmi_dev =
		        container_of(h, struct rmi_device, early_suspend_handler);
	struct rmi_driver_data *data;

	data = rmi_get_driverdata(rmi_dev);

	(void)rmi_suspend(data);
}

static void standard_late_resume(struct early_suspend *h)
{
	struct rmi_device *rmi_dev =
		        container_of(h, struct rmi_device, early_suspend_handler);
	struct rmi_driver_data *data;

	data = rmi_get_driverdata(rmi_dev);

	(void)rmi_resume(data);
}

//#endif /* defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND) */
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct rmi_driver_data *ddata =
		container_of(self, struct rmi_driver_data, fb_notif);
	
	VIVO_TS_LOG_INF("[%s]:~~~~~fb_notifier_callback is called!\n", __func__);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ddata) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			rmi_resume(ddata);
		else if (*blank == FB_BLANK_POWERDOWN)
			rmi_suspend(ddata);
	}

	return 0;
}
#endif

#ifdef CONFIG_RMI4_SPECIAL_EARLYSUSPEND
static int rmi_driver_suspend(struct device *dev) {
	return 0;
}

static int rmi_driver_resume(struct device *dev) {
	return 0;
}

static void rmi_driver_early_suspend(struct early_suspend *h) {
	struct rmi_device *rmi_dev =
	    container_of(h, struct rmi_device, early_suspend_handler);
	standard_suspend(rmi_dev);
}

static void rmi_driver_late_resume(struct early_suspend *h) {
	struct rmi_device *rmi_dev =
	    container_of(h, struct rmi_device, early_suspend_handler);
	standard_resume(rmi_dev);
}
#else
static int rmi_driver_suspend(struct device *dev) {
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	return standard_suspend(rmi_dev);
}

static int rmi_driver_resume(struct device *dev) {
	struct rmi_device *rmi_dev = to_rmi_device(dev);
	return standard_resume(rmi_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rmi_driver_early_suspend(struct early_suspend *h) {
	return standard_early_suspend(h);
}

static void rmi_driver_late_resume(struct early_suspend *h) {
	return standard_late_resume(h);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_RMI4_SPECIAL_EARLYSUSPEND */

#endif /* CONFIG_PM */

static int rmi_driver_remove(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *entry;
	int i;
#if defined(WATCH_DOG_TIMER)
	stop_watch_dog_timer(data);
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&rmi_dev->early_suspend_handler);
#elif defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		VIVO_TS_LOG_ERR("[%s]:Error occurred while unregistering fb_notifier.\n", __func__);
#endif
#ifdef	CONFIG_RMI4_DEBUG
	teardown_debugfs(rmi_dev);
#endif

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->remove)
			entry->fh->remove(entry);

	rmi_free_function_list(rmi_dev);
	for (i = 0; i < ARRAY_SIZE(attrs); i++)
		device_remove_file(&rmi_dev->dev, &attrs[i]);
	if (data->pdt_props.has_bsr)
		device_remove_file(&rmi_dev->dev, &bsr_attribute);
	kfree(data->f01_container->irq_mask);
	kfree(data->irq_mask_store);
	kfree(data->current_irq_mask);
	kfree(data);

	return 0;
}

#ifdef UNIVERSAL_DEV_PM_OPS
static UNIVERSAL_DEV_PM_OPS(rmi_driver_pm, rmi_driver_suspend,
			    rmi_driver_resume, NULL);
#endif

static struct rmi_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "rmi_generic",
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef UNIVERSAL_DEV_PM_OPS
		.pm = &rmi_driver_pm,
#endif
#endif
	},
	.probe = rmi_driver_probe,
	.irq_handler = rmi_driver_irq_handler,
	.reset_handler = rmi_driver_reset_handler,
	.fh_add = rmi_driver_fh_add,
	.fh_remove = rmi_driver_fh_remove,
	.get_func_irq_mask = rmi_driver_irq_get_mask,
	.store_irq_mask = rmi_driver_irq_save,
	.restore_irq_mask = rmi_driver_irq_restore,
	.remove = rmi_driver_remove
};

/* sysfs show and store fns for driver attributes */

static ssize_t rmi_driver_bsr_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->bsr);
}

static ssize_t rmi_driver_bsr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int retval;
	unsigned long val;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	/* need to convert the string data to an actual value */
	retval = strict_strtoul(buf, 10, &val);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Invalid value '%s' written to BSR.\n", __func__, buf);
		return -EINVAL;
	}

	retval = rmi_write(rmi_dev, BSR_LOCATION, (unsigned char)val);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]: failed to write bsr %u to 0x%x\n",
			__func__, (unsigned int)val, BSR_LOCATION);
		return retval;
	}

	data->bsr = val;

	return count;
}

static ssize_t rmi_driver_enabled_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->enabled);
}

static ssize_t rmi_driver_enabled_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval;
	int new_value;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	if (sysfs_streq(buf, "0"))
		new_value = false;
	else if (sysfs_streq(buf, "1"))
		new_value = true;
	else
		return -EINVAL;

	if (new_value) {
		retval = enable_sensor(rmi_dev);
		if (retval) {
			VIVO_TS_LOG_ERR("[%s]:Failed to enable sensor, code=%d.\n", __func__,
				retval);
			return -EIO;
		}
	} else {
		disable_sensor(rmi_dev);
	}

	return count;
}

#if REGISTER_DEBUG
static ssize_t rmi_driver_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval;
	u32 address;	/* use large addr here so we can catch overflow */
	unsigned int bytes;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	u8 readbuf[128];
	unsigned char outbuf[512];
	unsigned char *bufptr = outbuf;
	int i;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	retval = sscanf(buf, "%x %u", &address, &bytes);
	if (retval != 2) {
		VIVO_TS_LOG_ERR("[%s]:Invalid input (code %d) for reg store: %s", __func__,
			retval, buf);
		return -EINVAL;
	}
	if (address < 0 || address > 0xFFFF) {
		VIVO_TS_LOG_ERR("[%s]:Invalid address for reg store '%#06x'.\n", __func__,
			address);
		return -EINVAL;
	}
	if (bytes < 0 || bytes >= sizeof(readbuf) || address+bytes > 65535) {
		VIVO_TS_LOG_ERR("[%s]:Invalid byte count for reg store '%d'.\n", __func__,
			bytes);
		return -EINVAL;
	}

	retval = rmi_read_block(rmi_dev, address, readbuf, bytes);
	if (retval != bytes) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read %d registers at %#06x, code %d.\n", __func__,
			bytes, address, retval);
		return retval;
	}

	VIVO_TS_LOG_INF("[%s]:Reading %d bytes from %#06x.\n", __func__, bytes, address);
	for (i = 0; i < bytes; i++) {
		retval = snprintf(bufptr, 4, "%02X ", readbuf[i]);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to format string. Code: %d", __func__,
				retval);
			return retval;
		}
		bufptr += retval;
	}
	VIVO_TS_LOG_INF("[%s]:\n", outbuf);

	return count;
}
#endif

static int __init rmi_driver_init(void)
{
	if(is_atboot == 1)
		return 0;

	return rmi_register_driver(&sensor_driver);
}

static void __exit rmi_driver_exit(void)
{
	rmi_unregister_driver(&sensor_driver);
}
//qgf add 
#if defined(SWITCH_FAST_ENERGY_RALAX)
extern void rmi_f11_set_realse_3count_zero(void);
#endif
//end




module_init(rmi_driver_init);
module_exit(rmi_driver_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI generic driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
