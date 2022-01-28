/* include/linux/hscdtd.h
 *
 * GeoMagneticField device driver
 *
 * Copyright (C) 2012-2013 ALPS ELECTRIC CO., LTD. All Rights Reserved.
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

#ifndef HSCDTD_H
#define HSCDTD_H

#include <linux/i2c.h>

struct alps_platform_data {
	char layout;
	int	auto_report;
	int gpio_DRDY;
	int gpio_rstn;
	int gpio_int;
	unsigned int int_flags;
	bool use_int;
};

struct hscdtd_platform_data {
	char layout;
	int	auto_report;
	int gpio_DRDY;
	int gpio_rstn;
	int gpio_int;
	unsigned int int_flags;
	bool use_int;
	int axis[9];
};

struct accsns_platform_data {
	int axis[9];
};

struct accsns_function {
	int (*get_data)(struct i2c_client *acc_client, int *xyz);
	void (*measure_start)(struct i2c_client *acc_client);
	void (*measure_stop)(struct i2c_client *acc_client);
	int (*set_delay)(struct i2c_client *acc_client, int delay_ms);
};

struct hscdtd_acc_dev {
	char *name;
	struct i2c_client *acc_client;
	struct accsns_function *fn;
};

extern int accsns_register(struct hscdtd_acc_dev *acc_dev, int *axis);
extern void accsns_unregister(struct hscdtd_acc_dev *acc_dev);

#endif

