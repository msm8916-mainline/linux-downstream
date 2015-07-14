/*
* Simple driver for Texas Instruments LM3630 LED Flash driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#ifndef __LINUX_LM3632_H
#define __LINUX_LM3632_H

#define LM3632_NAME "lm3632_bl"

struct backlight_platform_data {
	void (*platform_init)(void);
	int bl_gpio;
	int dsv_p_gpio;
	int dsv_n_gpio;
	u32 init_on_kernel;
	int i2c_sda_gpio;
	int i2c_scl_gpio;
	unsigned int mode;
	int max_current;
	int init_on_boot;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int factory_brightness;
	int blmap_size;
	char *blmap;
};

struct lm3632_device {
	struct i2c_client *client;
	struct backlight_device *bl_dev;
	int bl_gpio;
	int dsv_p_gpio;
	int dsv_n_gpio;
	int i2c_sda_gpio;
	int i2c_scl_gpio;
	int max_current;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int factory_brightness;
	struct mutex bl_mutex;
	int blmap_size;
	char *blmap;
};

#endif /* __LINUX_LM3632_H */
