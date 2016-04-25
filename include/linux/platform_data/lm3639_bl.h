/*
* Simple driver for Texas Instruments LM3630 LED Flash driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#ifndef __LINUX_LM3639_H
#define __LINUX_LM3639_H

#define LM3639_NAME "lm3639_bl"

enum lm3639_pwm {
	LM3639_PWM_DISABLE = 0x00,
	LM3639_PWM_EN_ACTLOW = 0x48,
	LM3639_PWM_EN_ACTHIGH = 0x40,
};

enum lm3639_strobe {
	LM3639_STROBE_DISABLE = 0x00,
	LM3639_STROBE_EN_ACTLOW = 0x10,
	LM3639_STROBE_EN_ACTHIGH = 0x30,
};

enum lm3639_txpin {
	LM3639_TXPIN_DISABLE = 0x00,
	LM3639_TXPIN_EN_ACTLOW = 0x04,
	LM3639_TXPIN_EN_ACTHIGH = 0x0C,
};

enum lm3639_fleds {
	LM3639_FLED_DIASBLE_ALL = 0x00,
	LM3639_FLED_EN_1 = 0x40,
	LM3639_FLED_EN_2 = 0x20,
	LM3639_FLED_EN_ALL = 0x60,
};

enum lm3639_bleds {
	LM3639_BLED_DIASBLE_ALL = 0x00,
	LM3639_BLED_EN_1 = 0x10,
	LM3639_BLED_EN_2 = 0x08,
	LM3639_BLED_EN_ALL = 0x18,
};
enum lm3639_bled_mode {
	LM3639_BLED_MODE_EXPONETIAL = 0x00,
	LM3639_BLED_MODE_LINEAR = 0x10,
};

struct backlight_platform_data {
	void (*platform_init)(void);
	int bl_gpio;
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

struct lm3639_device {
	struct i2c_client *client;
	struct backlight_device *bl_dev;
	int bl_gpio;
	int max_current;
	int min_brightness;
	int max_brightness;
	int default_brightness;
	int factory_brightness;
	struct mutex bl_mutex;
	int blmap_size;
	char *blmap;
};
#endif /* __LINUX_LM3639_H */
