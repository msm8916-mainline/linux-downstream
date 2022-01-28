/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
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
#ifndef _RMI_DRIVER_H
#define _RMI_DRIVER_H

#define RMI_DRIVER_VERSION "1.3"

#define DRIVER_NAME "rmi"

#define RMI_PRODUCT_ID_LENGTH    10
#define RMI_PRODUCT_INFO_LENGTH   2
#define RMI_DATE_CODE_LENGTH      3
#define I2C_RETRY_TIMES_LIMIT		3

#include <linux/ctype.h>
#include <linux/jiffies.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
/* Sysfs related macros */

/* You must define FUNCTION_DATA and FNUM to use these functions. */
#define RMI4_SYSFS_DEBUG defined(CONFIG_RMI4_DEBUG) || defined(CONFIG_ANDROID))

#if defined(FNUM) && defined(FUNCTION_DATA)

#define tricat(x,y,z) tricat_(x,y,z)

#define tricat_(x,y,z) x##y##z

/* wyl add the "static" before each DEVICE_ATTR */
#define show_union_struct_prototype(propname)\
static ssize_t tricat(rmi_fn_,FNUM,_##propname##_show)(\
					struct device *dev,\
					struct device_attribute *attr,\
					char *buf);\
\
static DEVICE_ATTR(propname, RMI_RO_ATTR,\
		tricat(rmi_fn_,FNUM,_##propname##_show),\
		rmi_store_error);

#define store_union_struct_prototype(propname)\
static ssize_t tricat(rmi_fn_,FNUM,_##propname##_store)(\
					struct device *dev,\
					struct device_attribute *attr,\
					const char *buf, size_t count);\
\
static DEVICE_ATTR(propname, RMI_WO_ATTR,\
		rmi_show_error,\
		tricat(rmi_fn_, FNUM, _##propname##_store));


#define show_store_union_struct_prototype(propname)\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_show)(\
					struct device *dev,\
					struct device_attribute *attr,\
					char *buf);\
\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_store)(\
					struct device *dev,\
					struct device_attribute *attr,\
					const char *buf, size_t count);\
\
static DEVICE_ATTR(propname, RMI_RW_ATTR,\
		tricat(rmi_fn_, FNUM, _##propname##_show),\
		tricat(rmi_fn_, FNUM, _##propname##_store));

#define simple_show_union_struct(regtype, propname, fmt)\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_show)(struct device *dev,\
				struct device_attribute *attr, char *buf) {\
	struct rmi_function_container *fc;\
	struct FUNCTION_DATA *data;\
\
	fc = to_rmi_function_container(dev);\
	data = fc->data;\
\
	return snprintf(buf, PAGE_SIZE, fmt,\
			data->regtype.propname);\
}

#define show_union_struct(regtype, reg_group, propname, fmt)\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_show)(\
					struct device *dev,\
					struct device_attribute *attr,\
					char *buf) {\
	struct rmi_function_container *fc;\
	struct FUNCTION_DATA *data;\
	int result;\
\
	fc = to_rmi_function_container(dev);\
	data = fc->data;\
\
	mutex_lock(&data->regtype##_mutex);\
	/* Read current regtype values */\
	result = rmi_read_block(fc->rmi_dev, data->regtype.reg_group->address,\
				(u8 *)data->regtype.reg_group,\
				sizeof(data->regtype.reg_group->regs));\
	mutex_unlock(&data->regtype##_mutex);\
	if (result < 0) {\
		dev_dbg(dev, "%s : Could not read regtype at 0x%x\\n",\
					__func__, data->regtype.reg_group->address);\
		return result;\
	}\
	return snprintf(buf, PAGE_SIZE, fmt,\
			data->regtype.reg_group->propname);\
}\

#define show_store_union_struct(regtype, reg_group, propname, fmt)\
show_union_struct(regtype, reg_group, propname, fmt)\
\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_store)(\
					struct device *dev,\
					struct device_attribute *attr,\
					const char *buf, size_t count) {\
	int result;\
	unsigned long val;\
	unsigned long old_val;\
	struct rmi_function_container *fc;\
	struct FUNCTION_DATA *data;\
\
	fc = to_rmi_function_container(dev);\
	data = fc->data;\
\
	/* need to convert the string data to an actual value */\
	result = strict_strtoul(buf, 10, &val);\
\
	/* if an error occured, return it */\
	if (result)\
		return result;\
	/* Check value maybe */\
\
	/* Read current regtype values */\
	mutex_lock(&data->regtype##_mutex);\
	result =\
	    rmi_read_block(fc->rmi_dev, data->regtype.reg_group->address,\
				(u8 *)data->regtype.reg_group,\
				sizeof(data->regtype.reg_group->regs));\
\
	if (result < 0) {\
		mutex_unlock(&data->regtype##_mutex);\
		dev_dbg(dev, "%s : Could not read regtype at 0x%x\\n",\
					 __func__,\
					data->regtype.reg_group->address);\
		return result;\
	}\
	/* if the current regtype registers are already set as we want them,\
	 * do nothing to them */\
	if (data->regtype.reg_group->propname == val) {\
		mutex_unlock(&data->regtype##_mutex);\
		return count;\
	}\
	/* Write the regtype back to the regtype register */\
	old_val = data->regtype.reg_group->propname;\
	data->regtype.reg_group->propname = val;\
	result =\
	    rmi_write_block(fc->rmi_dev, data->regtype.reg_group->address,\
				(u8 *)data->regtype.reg_group,\
				sizeof(data->regtype.reg_group->regs));\
	if (result < 0) {\
		dev_dbg(dev, "%s : Could not write regtype to 0x%x\\n",\
					__func__,\
					data->regtype.reg_group->address);\
		/* revert change to local value if value not written */\
		data->regtype.reg_group->propname = old_val;\
		mutex_unlock(&data->regtype##_mutex);\
		return result;\
	}\
	mutex_unlock(&data->regtype##_mutex);\
	return count;\
}


#define show_repeated_union_struct(regtype, reg_group, propname, fmt)\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_show)(struct device *dev,\
					struct device_attribute *attr,\
					char *buf) {\
	struct rmi_function_container *fc;\
	struct FUNCTION_DATA *data;\
	int reg_length;\
	int result, size = 0;\
	char *temp;\
	int i;\
\
	fc = to_rmi_function_container(dev);\
	data = fc->data;\
	mutex_lock(&data->regtype##_mutex);\
\
	/* Read current regtype values */\
	reg_length = data->regtype.reg_group->length;\
	result = rmi_read_block(fc->rmi_dev, data->regtype.reg_group->address,\
			(u8*) data->regtype.reg_group->regs,\
			reg_length * sizeof(u8));\
	mutex_unlock(&data->regtype##_mutex);\
	if (result < 0) {\
		dev_dbg(dev, "%s : Could not read regtype at 0x%x\n"\
					"Data may be outdated.", __func__,\
					data->regtype.reg_group->address);\
	}\
	temp = buf;\
	for (i = 0; i < reg_length; i++) {\
		result = snprintf(temp, PAGE_SIZE - size, fmt " ",\
					data->regtype.reg_group->regs[i].propname);\
		if (result < 0) {\
			dev_err(dev, "%s : Could not write output.", __func__);\
			return result;\
		}\
		size += result;\
		temp += result;\
	}\
	result = snprintf(temp, PAGE_SIZE - size, "\n");\
	if (result < 0) {\
			dev_err(dev, "%s : Could not write output.", __func__);\
			return result;\
	}\
	return size + result;\
}

#define show_store_repeated_union_struct(regtype, reg_group, propname, fmt)\
show_repeated_union_struct(regtype, reg_group, propname, fmt)\
\
static ssize_t tricat(rmi_fn_, FNUM, _##propname##_store)(struct device *dev,\
				   struct device_attribute *attr,\
				   const char *buf, size_t count) {\
	struct rmi_function_container *fc;\
	struct FUNCTION_DATA *data;\
	int reg_length;\
	int result;\
	const char *temp;\
	int i;\
	unsigned int newval;\
\
	fc = to_rmi_function_container(dev);\
	data = fc->data;\
	mutex_lock(&data->regtype##_mutex);\
\
	/* Read current regtype values */\
\
	reg_length = data->regtype.reg_group->length;\
	result = rmi_read_block(fc->rmi_dev, data->regtype.reg_group->address,\
			(u8*) data->regtype.reg_group->regs,\
			reg_length * sizeof(u8));\
\
	if (result < 0) {\
		dev_dbg(dev, "%s : Could not read regtype at 0x%x\n"\
					"Data may be outdated.", __func__,\
					data->regtype.reg_group->address);\
	}\
	/* parse input */\
	\
	temp = buf;\
	for (i = 0; i < reg_length; i++) {\
		if(sscanf(temp, fmt, &newval) == 1) {\
			data->regtype.reg_group->regs[i].propname = newval;\
		} else {\
			/* If we don't read a value for each position, abort, restore
			 * previous values locally by rereading */\
			result = rmi_read_block(fc->rmi_dev, data->regtype.reg_group->address,\
											(u8*) data->regtype.reg_group->regs,\
											reg_length * sizeof(u8));\
\
			if (result < 0) {\
				dev_dbg(dev, "%s : Could not read regtype at 0x%x\n"\
							"Local data may be innacurrate.", __func__,\
							data->regtype.reg_group->address);\
			}\
			return -EINVAL;\
		}\
		/* move to next number */\
		while (*temp != 0) {\
			temp++;\
			if (isspace(*(temp - 1)) && !isspace(*temp))\
				break;\
		}\
	}\
	result = rmi_write_block(fc->rmi_dev, data->regtype.reg_group->address,\
			(u8*) data->regtype.reg_group->regs,\
			reg_length * sizeof(u8));\
	mutex_unlock(&data->regtype##_mutex);\
	if (result < 0) {\
		dev_dbg(dev, "%s : Could not write new values"\
			" to 0x%x\n", __func__, data->regtype.reg_group->address);\
		return result;\
	}\
	return count;\
}

/* Create templates for given types */
#define simple_show_union_struct_unsigned(regtype, propname)\
simple_show_union_struct(regtype, propname, "%u\n")

#define show_union_struct_unsigned(regtype, reg_group, propname)\
show_union_struct(regtype, reg_group, propname, "%u\n")

#define show_store_union_struct_unsigned(regtype, reg_group, propname)\
show_store_union_struct(regtype, reg_group, propname, "%u\n")

#define show_repeated_union_struct_unsigned(regtype, reg_group, propname)\
show_repeated_union_struct(regtype, reg_group, propname, "%u")

#define show_store_repeated_union_struct_unsigned(regtype, reg_group, propname)\
show_store_repeated_union_struct(regtype, reg_group, propname, "%u")

/* Remove access to raw format string versions */
/*#undef simple_show_union_struct
#undef show_union_struct_unsigned
#undef show_store_union_struct
#undef show_repeated_union_struct
#undef show_store_repeated_union_struct*/

#endif

#define GROUP(_attrs) { \
	.attrs = _attrs,  \
}

#define attrify(nm) &dev_attr_##nm.attr

#define PDT_PROPERTIES_LOCATION 0x00EF
#define BSR_LOCATION 0x00FE

union pdt_properties {
	struct {
		u8 reserved_1:6;
		u8 has_bsr:1;
		u8 reserved_2:1;
	};
	u8 regs[1];
};

#define PDT_START_SCAN_LOCATION 0x00e9
#define PDT_END_SCAN_LOCATION	0x0005
#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)

struct pdt_entry {
	u8 query_base_addr:8;
	u8 command_base_addr:8;
	u8 control_base_addr:8;
	u8 data_base_addr:8;
	u8 interrupt_source_count:3;
	u8 bits3and4:2;
	u8 function_version:2;
	u8 bit7:1;
	u8 function_number:8;
};

#define	TOUCHSCREEN_NORMAL  0
#define TOUCHSCREEN_SLEEP   1
#define TOUCHSCREEN_GESTURE 2

#define WAKEUP_NONE			0
#define WAKEUP_BY_RESETIRQ	1
#define WAKEUP_BY_SUSPEND	2
#define WAIT_RESETIRQ_TIMELIMIT 100

struct rmi_driver_data {
	struct rmi_function_container rmi_functions;

	struct rmi_function_container *f01_container;
	bool f01_bootloader_mode;

	int num_of_irq_regs;
	int irq_count;
	u8 *current_irq_mask;
	u8 *irq_mask_store;
	bool irq_stored;
	struct mutex irq_mutex;
	struct mutex pdt_mutex;

	union pdt_properties pdt_props;
	unsigned char bsr;
	bool enabled;
	bool has_lcd_shutoff;
	bool fast_lcd_shutoff;
	bool proximity_state;
#ifdef CONFIG_PM
	bool suspended;
#if defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND)
	bool early_suspended;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
	bool early_suspended;
#endif
	struct mutex suspend_mutex;

	struct kobject kobject_debug;
	struct rmi_device *rmi_dev;

	void *pm_data;
	int (*pre_suspend) (const void *pm_data);
	int (*post_suspend) (const void *pm_data);
	int (*pre_resume) (const void *pm_data);
	int (*post_resume) (const void *pm_data);
#endif

#ifdef CONFIG_RMI4_DEBUG
#ifdef CONFIG_RMI4_SPI
	struct dentry *debugfs_delay;
#endif
	struct dentry *debugfs_phys;
	struct dentry *debugfs_reg_ctl;
	struct dentry *debugfs_reg;
	u16 reg_debug_addr;
	u8 reg_debug_size;
#endif

//#if defined(BBK_GLOVES_MODE)
	int ts_gloves_mode;
	struct hrtimer reset_timer;
	struct work_struct reset_work;
//#endif
#define WATCH_DOG_TIMER 
#if defined(WATCH_DOG_TIMER)
	struct hrtimer watch_dog_timer;
	struct work_struct watch_dog_work;
#endif
	struct work_struct wait_resetirq_work;
	wait_queue_head_t   resetirq_wait;
	int wakeup_event;
	int rst_gpio;
	struct work_struct irq_err_work;
	struct workqueue_struct *irq_err_workqueue;
	int charger_connect_flag;
    struct work_struct charger_work;


//#if defined(BBK_DCLICK_WAKE)
	int ts_ipod_flag;
	int ts_dclick_switch;
	int ts_dclick_simulate_switch;
	bool need_change_to_dclick;
	atomic_t ts_state;
//#endif
	struct wake_lock wakelock;
	u8 gesture_switch;
	u8 gesture_switch_export;
	u8 swipe_down_switch;

	bool log_switch;
	bool usb_reset_enable_switch;
	bool is_calling;
	bool is_GSM_opening;
	bool largetouch_flag;
	bool mute_switch;
	bool probe_over_flag;
	void *data;
	int AA_area_point_pressed;
	unsigned long AA_area_point_release_time;
	bool is_button_independent;
	u16 buf_gesture[20];
	
	struct wake_lock suspend_wakelock;
	
	int lcd_dimension_x;
	int lcd_dimension_y;
	
	int ts_dimension_x;
	int ts_dimension_y;
	
	struct mutex syna_i2c_reset_mutex;
};




int rmi_driver_f01_init(struct rmi_device *rmi_dev);

static inline void copy_pdt_entry_to_fd(struct pdt_entry *pdt,
				 struct rmi_function_descriptor *fd,
				 u16 page_start)
{
	fd->query_base_addr = pdt->query_base_addr + page_start;
	fd->command_base_addr = pdt->command_base_addr + page_start;
	fd->control_base_addr = pdt->control_base_addr + page_start;
	fd->data_base_addr = pdt->data_base_addr + page_start;
	fd->function_number = pdt->function_number;
	fd->interrupt_source_count = pdt->interrupt_source_count;
	fd->function_version = pdt->function_version;
}

#ifdef	CONFIG_RMI4_FWLIB
extern int rmi4_fw_update(struct rmi_device *rmi_dev,
		struct pdt_entry *f01_pdt, struct pdt_entry *f34_pdt, char name[]);
#endif
extern int rmi_get_version(struct rmi_device *dev, u16 f34_pdt_control_base_addr);
extern int rmi_get_module_id(struct rmi_device *dev, u16 f34_pdt_control_base_addr);

#if 0
extern bool rmi_get_lens_module(void);
#endif
/*
#if defined(PD1401F)||defined(PD1401F_EX)||defined(PD1401V) 
bool rmi_get_module(void);
#endif
*/
struct rmi_driver_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
};

#endif

