/* include/linux/sensor/pas230.h
 * Copyright (C) 2014 Partron Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */


#ifndef __LINUX_PAS230_H
#define __LINUX_PAS230

#include <linux/types.h>
#include <linux/i2c.h>

#ifdef __KERNEL__
struct pas230_platform_data {
	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
#ifndef CONFIG_LGE
    int (*proximity_power)(struct i2c_client*, bool); /* ldo power for the proximity */
#endif

	unsigned int irq_gpio;
	bool i2c_pull_up;
	bool digital_pwr_regulator;

	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;

	u32 vdd_ana_supply_min;
	u32 vdd_ana_supply_max;
	u32 vdd_ana_load_ua;

	u32 vddio_dig_supply_min;
	u32 vddio_dig_supply_max;
	u32 vddio_dig_load_ua;

	u32 vddio_i2c_supply_min;
	u32 vddio_i2c_supply_max;
	u32 vddio_i2c_load_ua;

	u32 near_offset;
	u32 far_offset;
    u32 cal_offset;
    u32 cal_hysteresis;
	u32 crosstalk_max;

	unsigned int ppcount;
	unsigned int ps_led_current;
#ifdef PAS230_ALS_SENSOR_INT
	u32 als_up_thres;
	u32 als_low_thres;
	u32 als_lux_coeff;
    u32 als_gain;
#endif
};
#endif /* __KERNEL__ */

#endif
