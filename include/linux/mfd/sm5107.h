/*
 * SM5107 MFD Driver
 *
 * Copyright 2015 LG Electronics Inc,
 *
  * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_SM5107_H__
#define __MFD_SM5107_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define LCD_OFF			0
#define LCD_ON			1
#define LCD_OFF_KNOCKON	2
#define LCD_ON_KNOCKON	3
#define LCD_OFF_TOGGLE	4
#define LCD_ON_TOGGLE	5

struct sm5107_platform_data {
	const char *name;
};

struct sm5107 {
	struct device *dev;
	struct regmap *regmap;
	struct sm5107_platform_data *pdata;
};

int sm5107_read_byte(struct sm5107 *sm5107, u8 reg, u8 *read);
int sm5107_write_byte(struct sm5107 *sm5107, u8 reg, u8 data);
int sm5107_ctrl(int mode);
#endif
