/*
 * DW8768 MFD Driver
 *
 * Copyright 2015 LG Electronics Inc,
 *
  * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_DW8768_H__
#define __MFD_DW8768_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define DW8768_ENABLE_REG 0x05

#define DW8768_OFF_SEQ1 0x06
#define DW8768_OFF_SEQ2 0x04

#define DW8768_MAX_REGISTERS 0x04

struct dw8768_platform_data {
	const char *name;
};

struct dw8768 {
	struct device *dev;
	struct regmap *regmap;
	struct dw8768_platform_data *pdata;
};

extern int dw8768_read_byte(struct dw8768 *dw8768, u8 reg, u8 *read);
extern int dw8768_write_byte(struct dw8768 *dw8768, u8 reg, u8 data);
extern int dw8768_off_seq(void);
#endif
