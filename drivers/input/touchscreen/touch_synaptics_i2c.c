/* Touch_synaptics.c
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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

#include <linux/i2c.h>
#include <linux/input/touch_synaptics.h>

#include "lge_touch_platform.h"

#define BUFFER_SIZE_TO_DIVIDE 	255

#define MASK_16BIT 				0xFFFF
#define MASK_8BIT 				0xFF
#define MASK_7BIT 				0x7F
#define MASK_6BIT				0x3F
#define MASK_5BIT 				0x1F
#define MASK_4BIT 				0x0F
#define MASK_3BIT 				0x07
#define MASK_2BIT 				0x03
#define MASK_1BIT 				0x01

#define DEFAULT_PAGE			0x00
#define PAGE_SELECT_REG			0xFF

 //i2c control
int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

#ifdef CONFIG_MTK_TOUCHPANEL
	if (ts_i2c_msg_transfer(client, msgs, 2) < 0) {
#else
	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
#endif
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	}

	return 0;
}

int touch_i2c_read_byte(struct i2c_client *client, u8 reg, int len, u8 *buf)
{
	struct i2c_msg* msgs = NULL;
	int msg_count = ((len - 1) / BUFFER_SIZE_TO_DIVIDE) + 2;
	int msg_rest_count = len % BUFFER_SIZE_TO_DIVIDE;
	int i;
	int data_len = 0;

	msgs = (struct i2c_msg*)kcalloc(msg_count, sizeof(struct i2c_msg), GFP_KERNEL);
	if(msgs == NULL)
		return -EIO;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	if (!msg_rest_count)
		msg_rest_count = BUFFER_SIZE_TO_DIVIDE;

	for (i = 0 ; i < (msg_count - 1) ; i++) {
		if (i == (msg_count - 2))
			data_len = msg_rest_count;
		else
			data_len = BUFFER_SIZE_TO_DIVIDE;

		msgs[i + 1].addr = client->addr;
		msgs[i + 1].flags = I2C_M_RD;
		msgs[i + 1].len = data_len;
		msgs[i + 1].buf = buf + BUFFER_SIZE_TO_DIVIDE * i;
	}

#ifdef CONFIG_MTK_TOUCHPANEL
	if (ts_i2c_msg_transfer(client, msgs, msg_count) < 0) {
#else
	if (i2c_transfer(client->adapter, msgs, msg_count) == 2) {
#endif
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	}

	kfree(msgs);
	return 0;
}

int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 * buf)
{
	unsigned char send_buf[len + 1];

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = len+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	memcpy(&send_buf[1], buf, len);

#ifdef CONFIG_MTK_TOUCHPANEL
	if (ts_i2c_msg_transfer(client, msgs, 1) < 0) {
#else
	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
#endif
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	}

	return 0;
}

int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	unsigned char send_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = 2,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;

#ifdef CONFIG_MTK_TOUCHPANEL
	memcpy(&send_buf[1], &data, 1);
	if (ts_i2c_msg_transfer(client, msgs, 1) < 0) {
#else
	send_buf[1] = (unsigned char)data;
	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
#endif
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	}

	return 0;
}

int synaptics_rmi4_reg_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

	/* page read */
	retval = touch_i2c_read(ts->client, PAGE_SELECT_REG, sizeof(page_old), &page_old);

	if (retval < 0) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

	/* page compare & change */
	if (page_old == page_new) {
		page_changed = false;
	} else {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

	/* read register */
	retval = touch_i2c_read(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), length, data);

	if (retval < 0) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to read the register(addr=0x%04x)\n", addr);
		return retval;
	}

	/* page restore */
	if (page_changed) {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

int synaptics_rmi4_reg_write(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

	/* page read */
	retval = touch_i2c_read(ts->client, PAGE_SELECT_REG, sizeof(page_old), &page_old);

	if (retval < 0) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

	/* page compare & change */
	if (page_old == page_new) {
		page_changed = false;
	} else {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

	/* write register */
	retval = touch_i2c_write(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), length, data);

	if (retval < 0) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to write to the register(addr=0x%04x)\n", addr);
		return retval;
	}

	/* page restore */
	if (page_changed) {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

int synaptics_rmi4_byte_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char page_old = 0;
	unsigned char page_new = 0;
	bool page_changed;

/* page read */
	retval = touch_i2c_read(ts->client, PAGE_SELECT_REG, sizeof(page_old), &page_old);

	if (retval < 0) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to read from Page_Select register\n");
		return retval;
	}

	page_new = (addr >> 8);

	/* page compare & change */
	if (page_old == page_new) {
		page_changed = false;
	} else {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_new);

		if (retval < 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to change Page_Select register\n");
			return retval;
		}

		page_changed = true;
	}

	/* read register */
	retval = touch_i2c_read_byte(ts->client, (unsigned char)(addr & ~(MASK_8BIT << 8)), length, data);

	if (retval < 0) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to long read the register(addr=0x%04x)\n", addr);
		return retval;
	}

	/* page restore */
	if (page_changed) {
		retval = touch_i2c_write_byte(ts->client, PAGE_SELECT_REG, page_old);

		if (retval < 0) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Failed to restore Page_Select register\n");
			return retval;
		}
	}

	return 0;
}

int synaptics_ts_page_data_read(struct i2c_client *client,
		u8 page, u8 reg, int size, u8 *data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_read(client, reg, size, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
	error);

	return 0;
error:
	return -1;
}

int synaptics_ts_page_data_write(struct i2c_client *client,
		u8 page, u8 reg, int size, u8 *data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_write(client, reg, size, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
	error);

	return 0;
error:
	return -1;
}

int synaptics_ts_page_data_write_byte(struct i2c_client *client,
		u8 page, u8 reg, u8 data)
{
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, page), error);
	DO_SAFE(touch_i2c_write_byte(client, reg, data), error);
	DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE),
	error);

	return 0;
error:
	return -1;
}
