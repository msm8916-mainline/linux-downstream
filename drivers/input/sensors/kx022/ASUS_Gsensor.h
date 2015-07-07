/* 
 * Copyright (C) 2015 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_ASUS_GSENSOR_H
#define __LINUX_ASUS_GSENSOR_H
#include <linux/device.h>
#include <linux/types.h>

/*************************************************
 *           Use to Setting device state          *
 *************************************************/
#define KX022_DEVICE_DISABLE	0
#define KX022_ACC_ENABLE	1
#define KX022_ORI_ENABLE 	2
#define KX022_BOTH_ENABLE 	3

extern ssize_t gsensor_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_status_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_dump_reg(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_read_raw(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_r_en_mt(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_w_en_mt(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

extern ssize_t gsensor_show_message(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_set_message(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t gsensor_get_poll(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_set_poll(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t gsensor_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t gsensor_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t get_gsensor_data(struct device *dev, struct device_attribute *devattr, char *buf);
extern ssize_t get_gsensor_state(struct device *dev, struct device_attribute *devattr, char *buf);
extern ssize_t read_gsensor_resolution(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t write_gsensor_resolution(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t read_gsensor_wufe(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t write_gsensor_wufe(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t read_gsensor_reg2_rate(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t write_gsensor_reg2_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t reset_gsensor(struct device *dev, struct device_attribute *attr, char *buf);
//added by Eason
extern ssize_t init_gsensor_double_tap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_flip(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t init_gsensor_flick(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#endif
