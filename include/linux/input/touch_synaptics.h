/* include/linux/lge_touch_core.h
 *
 * Copyright (C) 2012 LGE.
 *
 * Writer: yehan.ahn@lge.com, 	hyesung.shin@lge.com
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

#ifndef LGE_TOUCH_SYNAPTICS_H

#include <linux/input/lge_touch_core.h>

#define LGE_TOUCH_SYNAPTICS_H

#define NUM_OF_EACH_FINGER_DATA_REG		8
#define MAX_NUM_OF_FINGERS			10

#define DESCRIPTION_TABLE_START			0xe9

#define PAGE_SELECT_REG				0xFF
#define PAGE_MAX_NUM				5

#define FW_VER_INFO_NUM				4

struct function_descriptor {
	u8 	query_base;
	u8 	command_base;
	u8 	control_base;
	u8 	data_base;
	u8 	int_source_count;
	u8 	id;
};

struct ts_ic_function {
	struct function_descriptor dsc;
	u8 	function_page;
};

struct finger_data {
	u8	finger_reg[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
};

struct button_data {
	u16	key_code;
};

struct cur_touch_data {
	u8	device_status_reg;
	u8	interrupt_status_reg;
	u8	button_data_reg;
	struct finger_data	finger;
	struct button_data	button;
};

struct synaptics_ts_fw_info {
	u8		fw_version[FW_VER_INFO_NUM+1];
	u8		fw_product_id[11];
	u8		fw_image_version[5];
	u8		fw_image_product_id[11];
	unsigned char	*fw_start;
	unsigned long	fw_size;
	u8		need_rewrite_firmware;
};

struct lpwg_control {
	u8		lpwg_mode;
	u8		screen;
	u8		sensor;
	u8		qcover;
	u8		double_tap_enable;
	u8 		password_enable;
	u8		signature_enable;
	u8		lpwg_is_enabled;
	atomic_t	is_suspend;
};

struct lpwg_password_data {
	u8		tap_count;
	u8		data_num;
	u8		double_tap_check;
	struct point 	data[MAX_POINT_SIZE_FOR_LPWG];
};

struct synaptics_ts_data {
	u8	is_probed;
	u8	is_init;
	struct lpwg_control	lpwg_ctrl;
	struct lpwg_password_data	pw_data;
	struct regulator	*regulator_vdd;
	struct regulator	*regulator_vio;
	struct i2c_client	*client;
	struct ts_ic_function	common_fc;
	struct ts_ic_function	finger_fc;
	struct ts_ic_function	button_fc;
	struct ts_ic_function	analog_fc;
	struct ts_ic_function	sensor_fc;
	struct ts_ic_function	flash_fc;
	struct cur_touch_data	ts_data;
	struct synaptics_ts_fw_info	fw_info;
	struct delayed_work	work_timer;
	struct wake_lock	timer_wake_lock;
	const struct touch_platform_data	*pdata;
	const struct state_info	*state;
};

struct synaptics_ts_exp_fn {
	int (*init)(struct synaptics_ts_data *ts);
	void (*remove)(struct synaptics_ts_data *ts);
	void (*reset)(struct synaptics_ts_data *ts);
	void (*reinit)(struct synaptics_ts_data *ts);
	void (*early_suspend)(struct synaptics_ts_data *ts);
	void (*suspend)(struct synaptics_ts_data *ts);
	void (*resume)(struct synaptics_ts_data *ts);
	void (*late_resume)(struct synaptics_ts_data *ts);
	void (*attn)(struct synaptics_ts_data *ts, unsigned char intr_mask);
};

struct synaptics_ts_exp_fhandler {
	struct synaptics_ts_exp_fn *exp_fn;
	bool inserted;
	bool initialized;
};

struct synaptics_ts_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query_6;

			struct {
				unsigned char ctrl_00_is_present:1;
				unsigned char ctrl_01_is_present:1;
				unsigned char ctrl_02_is_present:1;
				unsigned char ctrl_03_is_present:1;
				unsigned char ctrl_04_is_present:1;
				unsigned char ctrl_05_is_present:1;
				unsigned char ctrl_06_is_present:1;
				unsigned char ctrl_07_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_08_is_present:1;
				unsigned char ctrl_09_is_present:1;
				unsigned char ctrl_10_is_present:1;
				unsigned char ctrl_11_is_present:1;
				unsigned char ctrl_12_is_present:1;
				unsigned char ctrl_13_is_present:1;
				unsigned char ctrl_14_is_present:1;
				unsigned char ctrl_15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_16_is_present:1;
				unsigned char ctrl_17_is_present:1;
				unsigned char ctrl_18_is_present:1;
				unsigned char ctrl_19_is_present:1;
				unsigned char ctrl_20_is_present:1;
				unsigned char ctrl_21_is_present:1;
				unsigned char ctrl_22_is_present:1;
				unsigned char ctrl_23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_24_is_present:1;
				unsigned char ctrl_25_is_present:1;
				unsigned char ctrl_26_is_present:1;
				unsigned char ctrl_27_is_present:1;
				unsigned char ctrl_28_is_present:1;
				unsigned char ctrl_29_is_present:1;
				unsigned char ctrl_30_is_present:1;
				unsigned char ctrl_31_is_present:1;
			} __packed;
		};

		unsigned char data[5];
	};
};

struct synaptics_ts_f12_query_8
{
	union {
		struct {
			unsigned char size_of_query_9;

			struct {
				unsigned char data_00_is_present:1;
				unsigned char data_01_is_present:1;
				unsigned char data_02_is_present:1;
				unsigned char data_03_is_present:1;
				unsigned char data_04_is_present:1;
				unsigned char data_05_is_present:1;
				unsigned char data_06_is_present:1;
				unsigned char data_07_is_present:1;
			} __packed;
			struct {
				unsigned char data_08_is_present:1;
				unsigned char data_09_is_present:1;
				unsigned char data_10_is_present:1;
				unsigned char data_11_is_present:1;
				unsigned char data_12_is_present:1;
				unsigned char data_13_is_present:1;
				unsigned char data_14_is_present:1;
				unsigned char data_15_is_present:1;
			} __packed;
		};

		unsigned char data[3];
	};
};

/* extern function */
extern int FirmwareUpgrade(struct synaptics_ts_data *ts, const char* fw_path);

int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert);

#endif
