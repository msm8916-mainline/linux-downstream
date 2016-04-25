/* Lge_touch_platform.h
 *
 * Copyright (C) 2011 LGE.
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
#ifndef LGE_TOUCH_SYNAPTICS_I2C_H
#define LGE_TOUCH_SYNAPTICS_I2C_H

int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf);

int touch_i2c_read_byte(struct i2c_client *client, u8 reg, int len, u8 *buf);

int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 *buf);

int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data);

int synaptics_rmi4_reg_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length);

int synaptics_rmi4_reg_write(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length);

int synaptics_rmi4_byte_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length);

int synaptics_ts_page_data_read(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data);

int synaptics_ts_page_data_write(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data);

int synaptics_ts_page_data_write_byte(struct i2c_client *client,
	 u8 page, u8 reg, u8 data);

#endif //LGE_TOUCH_SYNAPTICS_I2C_H
