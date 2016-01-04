/* drivers/input/touchscreen/NVTtouch_205.h
 *
 * Copyright (C) 2010 - 2014 Novatek, Inc.
 *
 * Revision : V2 (2014/10/22)
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>

//---I2C driver info.---
#define NVT_I2C_NAME "NVTouch_205"	//"NVT-ts"
#define NVT_I2C_BUS_NUM 4
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x70

#define TOUCH_KEY_NUM 4
#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM]={
	KEY_MENU,
	KEY_HOMEPAGE,
	KEY_BACK,
	KEY_SEARCH
};
#endif

//---Input device info.---
#define NVT_TS_NAME "currency_tp"


//---Touch info.---
#define TOUCH_MAX_WIDTH  720
#define TOUCH_MAX_HEIGHT 1280
#define TOUCH_MAX_FINGER_NUM 5

#define BUFFER_LENGTH 32768


//---Customerized func.---
#define NVT_TOUCH_CTRL_DRIVER 1
#define BOOT_UPDATE_FIRMWARE 1

//#define CONFIG_TOUCHSCREEN_WITH_PROXIMITY

#define WAKEUP_GESTURE  0
#if WAKEUP_GESTURE
const uint16_t gesture_key_array[]={
	KEY_POWER,	//GESTURE_WORD_C
	KEY_POWER,	//GESTURE_WORD_W
	KEY_POWER,	//GESTURE_WORD_V
	KEY_POWER,	//GESTURE_DOUBLE_CLICK
	KEY_POWER,	//GESTURE_WORD_Z
	KEY_POWER,	//GESTURE_WORD_M
	KEY_POWER,	//GESTURE_WORD_O
	KEY_POWER,	//GESTURE_WORD_e
	KEY_POWER,	//GESTURE_WORD_S
	KEY_POWER,	//GESTURE_SLIDE_UP
	KEY_POWER,	//GESTURE_SLIDE_DOWN
	KEY_POWER,	//GESTURE_SLIDE_LEFT
	KEY_POWER,	//GESTURE_SLIDE_RIGHT	
};
#endif

struct nvt_platform_data {
    u32 irq_gpio;
	u32 reset_gpio;
	const char *name;
	u32 irq_gpio_flags;
	u32 reset_gpio_flags;
    const char *vdd_name;
};

struct nvt_ts_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	//struct nvt_platform_data *pdata;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	char phys[32];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
};

#if NVT_TOUCH_CTRL_DRIVER
struct nvt_flash_data{
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};
#endif	

#endif /* _LINUX_NVT_TOUCH_H */
