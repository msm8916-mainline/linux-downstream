/* linux/driver/input/drivers/sensor/pas230.c
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "pas230.h"

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include "lge_log.h"
#define SENSOR_TAG	"[LGE_Proximity]"

enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct sensor_dt_to_pdata_map {
	const char					*dt_name;
	void						*ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type	type;
	int							default_val;
};
#endif

#define	VENDOR		"PARTRON"
#define	CHIP_ID		"PAS230"
#define LGE_PROXIMITY_NAME	"lge_proximity"
#ifdef PAS230_ALS_SENSOR_ENABLE
#define LGE_LIGHT_NAME		"lge_light"
#endif
#define PAS230_DRV_NAME		"pas230"
#define PROX_CAL_DATA_PATH	"/sns/prox_calibration.dat"
#define PROX_READ_NUM		1	/*40*/

enum {
	MAIN_CTRL=0x00,
	PS_LED,
	PS_PULSES,
	PS_MEAS_RATE,
	ALS_CS_MEAS_RATE,
	ALS_CS_GAIN,
	PART_ID,
	MAIN_STATUS,
	PS_DATA=0x08,
	CLEAR_DATA=0x0a,
	GREEN_DATA=0x0d,
	BLUE_DATA=0x10,
	RED_DATA=0x13,
	COMP_DATA=0x16,
	INT_CFG=0x19,
	INT_PST,
	PS_THRES_UP=0x1b,
	PS_THRES_LOW=0x1d,
	PS_CAN=0x1f,
	ALS_THRES_UP=0x21,
	ALS_THRES_LOW=0x24,
	ALS_THRES_VAR=0x27
};

static u8 reg_defaults[40] = {
 	0x03, /* 0x00_0 : MAIN_CTRL */
	0x36, /* 0x01_1 : PS_LED */
	0x08, /* 0x02_2 : PS_PULSES */
	0x55, /* 0x03_3 : PS_MEAS_RATE */
	0x22, /* 0x04_4 : ALS_CS_MEAS_RATE */
	0x01, /* 0x05_5 : ALS_CS_GAIN */
	0xb0, /* 0x06_6 : PART_ID */
	0x00, /* 0x07_7 : MAIN_STATUS */
	0x00, 0x00, /* 0x08_8 : PS_DATA */
	0x00, 0x00, 0x00, /* 0x0a_10 : CLEAR_DATA */
	0x00, 0x00, 0x00, /* 0x0d_13 : GREEN_DATA */
	0x00, 0x00, 0x00, /* 0x10_16 : BLUE_DATA */
	0x00, 0x00, 0x00, /* 0x13_19 : RED_DATA */
	0x00, 0x00, 0x00, /* 0x16_22 : COMP_DATA */
#ifdef PAS230_ALS_SENSOR_INT
	0x15, /* 0x19_25 : INT_CFG (0x15 = ALS & PS INT ENABLE) */
#else
	0x11, /* 0x19_25 : INT_CFG (0x11 = PS INT ONLY)*/
#endif
	0x11, /* 0x1a_26 : INT_PST */
	0x14, 0x00, /* 0x1b_27 : PS_THRES_UP, 2047_80 */
	0x0a, 0x00, /* 0x1d_29 : PS_THRES_LOW, 0_65 */
	0x00, 0x00, /* 0x1f_31 : PS_CAN, 2047_0 */
	0xff, 0xff, 0x0f, /* 0x21_33 : ALS_THRES_UP */
	0x00, 0x00, 0x00, /* 0x24_36 : ALS_THRES_LOW */
	0x00, /* 0x27_39 : ALS_THRES_VAR */
};

#define PS_ON			(reg_defaults[MAIN_CTRL]&0x01)
#define PS_OFF			(reg_defaults[MAIN_CTRL]&(0x01^0xff))
#define ALS_CS_ON		(reg_defaults[MAIN_CTRL]&0x02)
#define ALS_CS_OFF		(reg_defaults[MAIN_CTRL]&(0x02^0xff))
#define ALL_ON			(reg_defaults[MAIN_CTRL]&0x03)
#define ALL_OFF			(reg_defaults[MAIN_CTRL]&(0x03^0xff))

#ifdef PAS230_ALS_SENSOR_INT
#define PRX_INT_MASK	(0x02)
#define PRX_LGC_MASK	(0x04)
#define ALS_INT_MASK	(0x10)
#define ALS_THRESHOLD_MAX	524287
#define ALS_THRESHOLD_MIN	0
#define ALS_LUX_THRESHOLD	4	//Matching the Pocket Detection thresold
static atomic_t irq_status = ATOMIC_INIT(-1);
#endif
static atomic_t part_id = ATOMIC_INIT(0); //0= "b4" sample, 1= "b1"
static bool prx_state = false;
enum {
#ifdef PAS230_ALS_SENSOR_ENABLE
	LIGHT_ENABLED = BIT(0),
#endif
	PROXIMITY_ENABLED = BIT(1),
};

/* driver data */
struct pas230_data {
	struct input_dev *proximity_input_dev;
	struct i2c_client *i2c_client;
	struct work_struct work_prox;
	struct hrtimer prox_timer;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct workqueue_struct *prox_wq;
	struct class *proximity_class;
	struct device *proximity_dev;
	struct pas230_platform_data *pdata;
	int irq;
	int avg[3];
	int crosstalk;
	int near_threshold;
	int far_threshold;
#ifdef PAS230_ALS_SENSOR_INT
	int als_low_threshold;
	int als_up_threshold;
#endif

	ktime_t prox_poll_delay;
	u8 power_state;

	int ps_cal_result;
	int ps_value;

#ifdef PAS230_ALS_SENSOR_ENABLE
	struct input_dev *light_input_dev;
	struct work_struct work_light;
	struct hrtimer light_timer;
	struct workqueue_struct *light_wq;
	struct device *lightsensor_dev;
	struct class *lightsensor_class;
	ktime_t light_poll_delay;
#endif
};

#ifdef CONFIG_OF
static int sensor_platform_hw_init(struct i2c_client *client)
{
	struct pas230_data *data = i2c_get_clientdata(client);
	int error;
	SENSOR_FUN();

	if (gpio_is_valid(data->pdata->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(data->pdata->irq_gpio, "pas230_irq_gpio");
		if (error) {
			SENSOR_ERR("unable to request gpio [%d]", data->pdata->irq_gpio);
		}
		error = gpio_direction_input(data->pdata->irq_gpio);
		if (error) {
			SENSOR_ERR("unable to set direction for gpio [%d]", data->pdata->irq_gpio);
		}
		data->irq = client->irq = gpio_to_irq(data->pdata->irq_gpio);
	} else {
		SENSOR_ERR("irq gpio not provided");
	}
	return 0;
}

static void sensor_platform_hw_exit(struct i2c_client *client)
{
	struct pas230_data *data = i2c_get_clientdata(client);;
	SENSOR_FUN();

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

}

static int sensor_parse_dt(struct device *dev,
				struct pas230_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret, err = 0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		{"Partron,irq-gpio",				&pdata->irq_gpio,				DT_REQUIRED,	DT_GPIO,	0},
		{"Partron,near_offset",				&pdata->near_offset,			DT_SUGGESTED,	DT_U32,		0},
		{"Partron,far_offset",				&pdata->far_offset,				DT_SUGGESTED,	DT_U32,		0},
		{"Partron,crosstalk_max",			&pdata->crosstalk_max,			DT_SUGGESTED,	DT_U32,		0},
		{"Partron,ppcount",					&pdata->ppcount,				DT_SUGGESTED,	DT_U32,		8},
		{"Partron,ps_led_current",			&pdata->ps_led_current,			DT_SUGGESTED,	DT_U32,		6},
		{"Partron,ps_crosstalk",				&pdata->ps_crosstalk,			DT_SUGGESTED,	DT_U32,		50},
		{"Partron,ps_crosstalk_offset",		&pdata->ps_crosstalk_offset,	DT_SUGGESTED,	DT_U32,		0},
#ifdef PAS230_ALS_SENSOR_INT
		{"Partron,als_up_thres",				&pdata->als_up_thres,			DT_SUGGESTED,	DT_U32,		0},
		{"Partron,als_low_thres",			&pdata->als_low_thres,		DT_SUGGESTED,	DT_U32,		0},
		{"Partron,als_lux_coeff",				&pdata->als_lux_coeff,			DT_SUGGESTED,	DT_U32,		0},
		{"Partron,als_gain",					&pdata->als_gain,				DT_SUGGESTED,	DT_U32,		1},
#endif
		{NULL,								NULL,							0,				0,			0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(np, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *) itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(np, itr->dt_name, (u32 *) itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) = of_property_read_bool(np, itr->dt_name);
			ret = 0;
			break;
		default:
			SENSOR_LOG("%d is an unknown DT entry type", itr->type);
			ret = -EBADE;
		}

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				SENSOR_LOG("Missing '%s' DT entry", itr->dt_name);
				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;

	reg_defaults[PS_THRES_UP] = (u8)pdata->near_offset;
	reg_defaults[PS_THRES_UP+1] = (u8)((pdata->near_offset>>8) & 0x07);
	reg_defaults[PS_THRES_LOW] = (u8)pdata->far_offset;
	reg_defaults[PS_THRES_LOW+1] = (u8)((pdata->far_offset>>8) & 0x07);

	reg_defaults[PS_LED] = (reg_defaults[PS_LED] & 0xf8) | (u8)(pdata->ps_led_current);
#ifdef PAS230_ALS_SENSOR_INT
	reg_defaults[ALS_THRES_UP] = (u8)pdata->als_up_thres;
	reg_defaults[ALS_THRES_UP+1] = (u8)(pdata->als_up_thres>>8);
	reg_defaults[ALS_THRES_UP+2] = (u8)((pdata->als_up_thres>>16) & 0x07);
	reg_defaults[ALS_THRES_LOW] = (u8)pdata->als_low_thres;
	reg_defaults[ALS_THRES_LOW+1] = (u8)(pdata->als_low_thres>>8);
	reg_defaults[ALS_THRES_LOW+2] = (u8)((pdata->als_low_thres>>16) & 0x07);
#endif

	return err;

}
#endif

static int pas230_i2c_read(struct pas230_data *pas230, u8 cmd, u8 *val)
{
	int err = 0;
	int retry = 3;
	struct i2c_client *client = pas230->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		err = i2c_smbus_read_i2c_block_data(client, cmd, 1, val);
		if (err >= 0)
			return err;
	}

	return err;
}

static int pas230_i2c_readn(struct pas230_data *pas230, u8 cmd, u8 *val, u8 cnt)
{
	int err = 0;
	int retry = 3;
	struct i2c_client *client = pas230->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		err = i2c_smbus_read_i2c_block_data(client, cmd, cnt, val);
		if (err >= 0)
			return err;
	}

	return err;
}

static int pas230_i2c_write(struct pas230_data *pas230, u8 cmd, u8 val)
{
	u8 data[2]={0, };
	int err = 0;
	int retry = 3;
	struct i2c_msg msg[1];
	struct i2c_client *client = pas230->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	data[0]=cmd;
	data[1]=val;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	while (retry--) {
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0)
			return err;
	}

	return err;
}

static int pas230_set_ppcount(struct i2c_client *client, int ppcount)
{
	int ret = 0;
	struct pas230_data *pas230 = i2c_get_clientdata(client);

	SENSOR_LOG("set ps_pulses_count = %d", ppcount);
	reg_defaults[PS_PULSES] = ((u8)ppcount);

	mutex_lock(&pas230->power_lock);
	ret = pas230_i2c_write(pas230, PS_PULSES, reg_defaults[PS_PULSES]);
	mutex_unlock(&pas230->power_lock);

	return ret;
}

static int pas230_set_ps_led_current(struct i2c_client *client, int ps_led_current)
{
	int ret = 0;
	struct pas230_data *pas230 = i2c_get_clientdata(client);

	SENSOR_LOG("set ps_led_current = %d", ps_led_current);
	reg_defaults[PS_LED] = (reg_defaults[PS_LED] & 0xf8) | ((u8)ps_led_current);

	mutex_lock(&pas230->power_lock);
	ret = pas230_i2c_write(pas230, PS_LED, reg_defaults[PS_LED]);
	mutex_unlock(&pas230->power_lock);

	return ret;
}

static int pas230_write_crosstalk_data_to_fs(unsigned int crosstalk)
{
	int fd;
	int ret = 0;
	char crosstalk_buf[50];
	mm_segment_t old_fs = get_fs();

	SENSOR_FUN();

	memset(crosstalk_buf, 0, sizeof(crosstalk_buf));
	SENSOR_LOG("crosstalk_buf : %d", crosstalk);
	sprintf(crosstalk_buf, "%d", crosstalk);

	set_fs(KERNEL_DS);
	fd = sys_open(PROX_CAL_DATA_PATH, O_WRONLY | O_CREAT, 0664);

	if (fd >= 0) {
		sys_write(fd, crosstalk_buf, sizeof(crosstalk_buf));
		sys_fsync(fd); /*ensure calibration data write to file system*/
		sys_close(fd);
		sys_chmod(PROX_CAL_DATA_PATH, 0664);
		set_fs(old_fs);
	} else {
		ret++;
		sys_close(fd);
		set_fs(old_fs);
		return ret;
	}

	return ret;
}

static void pas230_set_proximity_thresh(struct pas230_data *pas230, u16 thresh_far, u16 thresh_near)
{
	reg_defaults[PS_THRES_UP] = thresh_near&0xff;
	reg_defaults[PS_THRES_UP+1] = (thresh_near>>8)&0xff;
	reg_defaults[PS_THRES_LOW] = thresh_far&0xff;
	reg_defaults[PS_THRES_LOW+1] = (thresh_far>>8)&0xff;

	pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
	pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
	pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
	pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
}

#ifdef PAS230_ALS_SENSOR_INT
static int pas230_write_als_thres_up(int val, struct pas230_data *pas230)
{
	u8 als_thres_up[3] = {0, };

	SENSOR_LOG("set als_up_thres = %d", val);
	pas230->als_up_threshold = val;

	als_thres_up[0] = (u8)val;
	als_thres_up[1] = (u8)(val>>8);
	als_thres_up[2] = (u8)((val>>16) & 0x07);

	//mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_THRES_UP, als_thres_up[0]);
	pas230_i2c_write(pas230, ALS_THRES_UP+1, als_thres_up[1]);
	pas230_i2c_write(pas230, ALS_THRES_UP+2, als_thres_up[2]);
	//mutex_unlock(&pas230->power_lock);

	return 0;

}

static int pas230_write_als_thres_low(int val, struct pas230_data *pas230)
{
	u8 als_thres_low[3] = {0, };

	SENSOR_LOG("set als_low_thres = %d", val);
	pas230->als_low_threshold = val;

	als_thres_low[0] = (u8)val;
	als_thres_low[1] = (u8)(val>>8);
	als_thres_low[2] = (u8)((val>>16) & 0x07);

	//mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_THRES_LOW, als_thres_low[0]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+1, als_thres_low[1]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+2, als_thres_low[2]);
	//(&pas230->power_lock);

	return 0;

}
#endif

static int pas230_read_crosstalk_data_from_fs(struct pas230_data *pas230)
{
	int fd;
	int ret = 0;
	int len = 0;
	unsigned int crosstalk = 0;
	char read_buf[50];
	mm_segment_t old_fs = get_fs();
	memset(read_buf, 0, sizeof(read_buf));
	set_fs(KERNEL_DS);

	fd = sys_open(PROX_CAL_DATA_PATH, O_RDONLY, 0);
	if (fd >= 0) {
		SENSOR_LOG("Success read Prox Cross-talk from FS");
		len = sys_read(fd, read_buf, sizeof(read_buf));
		SENSOR_LOG("Proximity Calibration File size is = %d", len);
		if (len <= 0) {
			ret = -1;
			sys_close(fd);
			set_fs(old_fs);
			if (pas230->pdata->ps_crosstalk > 0) {
				crosstalk = pas230->pdata->ps_crosstalk;
			} else {
				crosstalk = 50;	//default cal
			}
			if (pas230->pdata->ps_crosstalk_offset > 0)
				crosstalk += pas230->pdata->ps_crosstalk_offset;
			pas230->near_threshold = pas230->pdata->near_offset + crosstalk;
			pas230->far_threshold = pas230->near_threshold - pas230->pdata->far_offset;
			return crosstalk;
		}
		sys_close(fd);
		set_fs(old_fs);
		crosstalk = (simple_strtol(read_buf, NULL, 10));
		if (pas230->pdata->ps_crosstalk_offset > 0)
			crosstalk += pas230->pdata->ps_crosstalk_offset;
	} else {
		SENSOR_LOG("Fail read Prox Cross-talk FS (err:%d)", fd);
		ret = -1;
		sys_close(fd);
		set_fs(old_fs);
		if (pas230->pdata->ps_crosstalk > 0) {
			crosstalk = pas230->pdata->ps_crosstalk;
		} else {
			crosstalk = 50;	//default cal
		}
		if (pas230->pdata->ps_crosstalk_offset > 0)
			crosstalk += pas230->pdata->ps_crosstalk_offset;
		SENSOR_LOG("Proximity Cross-talk : %d", crosstalk);
	}

	// save crosstalk value to default register setting (PS_CAN)
	//crosstalk = (simple_strtol(read_buf, NULL, 10));
	//reg_defaults[PS_CAN] = (u8)crosstalk;
	//reg_defaults[PS_CAN+1] = (u8)((crosstalk>>8) & 0x07);
	pas230->near_threshold = pas230->pdata->near_offset + crosstalk;
	pas230->far_threshold = pas230->near_threshold - pas230->pdata->far_offset;

	reg_defaults[PS_THRES_UP] = (u8)(pas230->near_threshold);
	reg_defaults[PS_THRES_UP+1] = (u8)((pas230->near_threshold>>8) & 0x07);
	reg_defaults[PS_THRES_LOW] = (u8)pas230->far_threshold;
	reg_defaults[PS_THRES_LOW+1] = (u8)((pas230->far_threshold>>8) & 0x07);

	return crosstalk;
}

static bool pas230_proximity_rsp(struct pas230_data *pas230, u32 val)
{
	uint16_t near_val = ((reg_defaults[PS_THRES_UP+1]<<8) |reg_defaults[PS_THRES_UP]);
	uint16_t far_val = ((reg_defaults[PS_THRES_LOW+1]<<8) |reg_defaults[PS_THRES_LOW]);

	SENSOR_LOG("near_val:%d, far_val:%d", near_val, far_val);

	if((prx_state == false) && (val >= near_val))
	{
		prx_state = true; // near
		pas230_set_proximity_thresh(pas230, pas230->far_threshold, 1023);
	}
	else if((prx_state == false) && (val < near_val))
	{
		prx_state = false; // open == far
		pas230_set_proximity_thresh(pas230, 0, pas230->near_threshold);
	}
	else if((prx_state == true) && (val >= far_val))
	{
		prx_state = true; // hysteresis near
		pas230_set_proximity_thresh(pas230, pas230->far_threshold, 1023);
	}
	else if((prx_state == true) && (val < far_val))
	{
		prx_state = false; // near state -> far
		pas230_set_proximity_thresh(pas230, 0, pas230->near_threshold);
	}

	return prx_state;
}

void pas230_swap(int *x, int *y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}

static int pas230_run_calibration(struct pas230_data *pas230)
{
	u8 ps_value[2] = {0, };
	u8 ps_meas_rate = 0;
	u8 cal_meas_rate = 1;	/* 1 = 6.25ms */
	u8 tmp;
	unsigned int sum_of_pdata, temp_pdata[20];
	unsigned int ret = 0, i = 0, j = 0, ArySize = 20, cal_check_flag = 0;
	unsigned int old_enable = 0;

	SENSOR_FUN();

RE_CALIBRATION:
	sum_of_pdata = 0;
	old_enable = (pas230->power_state & PROXIMITY_ENABLED)? 1:0;

	if (old_enable == 1) {
		/* PS off */
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp & PS_OFF);

		/* change the register */
		ps_meas_rate = (reg_defaults[PS_MEAS_RATE] & 0xf8) | ((u8)cal_meas_rate);
		pas230_i2c_write(pas230, PS_MEAS_RATE, ps_meas_rate);

		/* PS on */
		//pas230->pdata->proximity_power(pas230->i2c_client, 1);
		pas230->power_state |= PROXIMITY_ENABLED;
		pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
		pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
		pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
		pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
		pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
		pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);

		for (i = 0; i < 20; i++)	{
			mdelay(5);
			pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
			temp_pdata[i] = ((ps_value[1]&0x07)<<8) | ps_value[0];
		}

		for (i = 0; i < ArySize - 1; i++)
			for (j = i + 1; j < ArySize; j++)
				if (temp_pdata[i] > temp_pdata[j])
					pas230_swap(temp_pdata + i, temp_pdata + j);

		for (i = 5; i < 15; i++) {
			sum_of_pdata = sum_of_pdata + temp_pdata[i];
			SENSOR_LOG("pdata%d = %d", i, temp_pdata[i]);
			}

		pas230->crosstalk = sum_of_pdata / 10;
		SENSOR_LOG("sum_of_pdata/10 = %d", pas230->crosstalk);
		if (pas230->crosstalk > pas230->pdata->crosstalk_max) {
			if (cal_check_flag == 0) {
				cal_check_flag = 1;
				goto RE_CALIBRATION;
			} else {
				/* recover register */
				pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);

				return -1;
			}
		}

		ret = pas230_write_crosstalk_data_to_fs(pas230->crosstalk);

		/* recover register */
		pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);

		return pas230->crosstalk;
	} else {
		mutex_lock(&pas230->power_lock);
		/* PS on */
		pas230->power_state |= PROXIMITY_ENABLED;

		pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
		pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
		pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
		pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
		pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);			//setting to happen the interrupt fastest.
		pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
		pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);
		pas230_i2c_read(pas230, MAIN_STATUS, &tmp);

#ifdef PAS230_ALS_SENSOR_INT
		if(atomic_read(&irq_status) == -1) {
			enable_irq(pas230->irq);
			enable_irq_wake(pas230->irq);
			atomic_set(&irq_status, atomic_read(&irq_status)+1);
		} else {
			atomic_set(&irq_status, atomic_read(&irq_status)+1);
		}
#else
		enable_irq(pas230->irq);
		enable_irq_wake(pas230->irq);
#endif
		mutex_unlock(&pas230->power_lock);
		/* PS off */
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp & PS_OFF);

		/* change the register */
		ps_meas_rate = (reg_defaults[PS_MEAS_RATE] & 0xf8) | ((u8)cal_meas_rate);
		pas230_i2c_write(pas230, PS_MEAS_RATE, ps_meas_rate);

		/* PS on */
		//pas230->pdata->proximity_power(pas230->i2c_client, 1);
		pas230->power_state |= PROXIMITY_ENABLED;
		pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
		pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
		pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
		pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
		pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
		pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);

		for (i = 0; i < 20; i++)	{
			mdelay(5);
			pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
			temp_pdata[i] = ((ps_value[1]&0x07)<<8) | ps_value[0];
		}

		for (i = 0; i < ArySize - 1; i++)
			for (j = i + 1; j < ArySize; j++)
				if (temp_pdata[i] > temp_pdata[j])
					pas230_swap(temp_pdata + i, temp_pdata + j);

		for (i = 5; i < 15; i++) {
			sum_of_pdata = sum_of_pdata + temp_pdata[i];
			SENSOR_LOG("pdata%d = %d", i, temp_pdata[i]);
			}

		pas230->crosstalk = sum_of_pdata / 10;
		SENSOR_LOG("sum_of_pdata/10 = %d", pas230->crosstalk);
		if (pas230->crosstalk > pas230->pdata->crosstalk_max) {
			if (cal_check_flag == 0) {
				cal_check_flag = 1;
				goto RE_CALIBRATION;
			} else {
				/* recover register */
				pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);

				return -1;
			}
		}

		ret = pas230_write_crosstalk_data_to_fs(pas230->crosstalk);

		/* recover register */
		pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);
		/* PS off */
		mutex_lock(&pas230->power_lock);

		pas230->power_state &= ~PROXIMITY_ENABLED;
#ifdef PAS230_ALS_SENSOR_INT
		if(atomic_read(&irq_status) == 1) {
			atomic_set(&irq_status, atomic_read(&irq_status)-1);
		} else {
			disable_irq_wake(pas230->irq);
			disable_irq(pas230->irq);
			atomic_set(&irq_status, atomic_read(&irq_status)-1);
		}
#else
		disable_irq_wake(pas230->irq);
		disable_irq(pas230->irq);
#endif
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp & PS_OFF);
		mutex_unlock(&pas230->power_lock);

		return pas230->crosstalk;
	}
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static void pas230_light_enable(struct pas230_data *pas230)
{
	u8 tmp;
	int64_t temp_time = 0;

	input_report_abs(pas230->light_input_dev, ABS_MISC, 30001);
	input_sync(pas230->light_input_dev);

	if (ktime_to_ns(pas230->light_poll_delay) == 1000000000) {
		temp_time = ktime_to_ns(pas230->light_poll_delay) - 975000000;
	} else {
		temp_time = ktime_to_ns(pas230->light_poll_delay) + 100000000;
	}
	pas230_i2c_read(pas230, PART_ID, &tmp);
//	pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
#ifdef PAS230_ALS_SENSOR_INT
	pas230_write_als_thres_up(ALS_THRESHOLD_MAX, pas230);
	pas230_write_als_thres_low(pas230->pdata->als_low_thres, pas230);
#endif
	pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
	pas230_i2c_write(pas230, MAIN_CTRL, tmp | ALS_CS_ON);

	SENSOR_DBG("temp_time=%lld", temp_time);

	hrtimer_start(&pas230->light_timer, ns_to_ktime(temp_time),
						HRTIMER_MODE_REL);
}

static void pas230_light_disable(struct pas230_data *pas230)
{
	u8 tmp;

	SENSOR_FUN();

	pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
	pas230_i2c_write(pas230, MAIN_CTRL, tmp & ALS_CS_OFF);

	SENSOR_LOG("MAIN_CTRL=0x%02x", tmp & ALS_CS_OFF);

	hrtimer_cancel(&pas230->light_timer);
	cancel_work_sync(&pas230->work_light);
}

static int lightsensor_get_alsvalue(struct pas230_data *pas230)
{
	int value = 0;
	u8 als_value[3] = {0, };

	/* get ALS */
	pas230_i2c_readn(pas230, GREEN_DATA, als_value, 3);
	value = ((als_value[2]<<16) | (als_value[1]<<8) | als_value[0]);

	if ( (pas230->pdata->als_lux_coeff) >= 1) {
		value = (value * (pas230->pdata->als_lux_coeff)) / 10;
		//SENSOR_LOG("als value=%d", value);
		return value;
	} else {
		value -= 4;
		if (value <0) {
			value = 0;
		} else {
			value /= 3;
		}
		//SENSOR_LOG("als lux_value=%d", value);
		return value;
	}

	/* get ALS sensitivity(18bit, Gain 3) = 2.4/(2^(18-16)*3) = 0.2 */
	/* value /= 5; */
	/* glass conpensation = x5 */
	/* value *= 5; */

	/* SENSOR_LOG("als value=%d", value); */

	/* return value; */
}
#endif

static void proxsensor_get_avgvalue(struct pas230_data *pas230)
{
	int min = 0, max = 0, avg = 0;
	int i;
	int value = 0;
	u8 ps_value[2] = {0, };

	for (i = 0; i < PROX_READ_NUM; i++) {
		msleep(101);
		pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
		value = ((ps_value[1]&0x07)<<8) | ps_value[0];
		avg += value;

		if (!i)
			min = value;
		else if (value < min)
			min = value;

		if (value > max)
			max = value;
	}
	avg /= PROX_READ_NUM;

	pas230->avg[0] = min;
	pas230->avg[1] = avg;
	pas230->avg[2] = max;

	SENSOR_DBG("min(%3d),avg(%3d),max(%3d)", min, avg, max);

}

static ssize_t pas230_proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	SENSOR_DBG("value=%d\n",  pas230->ps_value);

	return sprintf(buf, "%d\n", ((pas230->ps_value)==true) ? 1 : 0);

}

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_alsdata_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int als = 0;

	if (!(pas230->power_state & LIGHT_ENABLED))
		pas230_light_enable(pas230);

	als = lightsensor_get_alsvalue(pas230);

	SENSOR_DBG("als=%d",  als);

	if (!(pas230->power_state & LIGHT_ENABLED))
		pas230_light_disable(pas230);

	return sprintf(buf, "%d\n", als);
}
#endif

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	SENSOR_LOG("poll_delay=%lld", ktime_to_ns(pas230->light_poll_delay));

	return sprintf(buf, "%lld\n", ktime_to_ns(pas230->light_poll_delay));
}

static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	SENSOR_FUN();

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	SENSOR_LOG("new_delay=%lld",  new_delay);

	mutex_lock(&pas230->power_lock);
	if (new_delay != ktime_to_ns(pas230->light_poll_delay)) {
		pas230->light_poll_delay = ns_to_ktime(new_delay);
		if (pas230->power_state & LIGHT_ENABLED) {
			pas230_light_disable(pas230);
			pas230_light_enable(pas230);
		}
	}
	mutex_unlock(&pas230->power_lock);

	return size;
}
#endif

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_light_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
				(pas230->power_state & LIGHT_ENABLED) ? 1 : 0);
}
#endif

static ssize_t pas230_proximity_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
				(pas230->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_light_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d", *buf);
		return -EINVAL;
	}

	SENSOR_LOG("new_value=%d", new_value);

	mutex_lock(&pas230->power_lock);
	if (new_value && !(pas230->power_state & LIGHT_ENABLED)) {
		pas230->power_state |= LIGHT_ENABLED;
#ifdef PAS230_ALS_SENSOR_INT
		if (atomic_read(&irq_status) == -1) {
			enable_irq(pas230->irq);
			enable_irq_wake(pas230->irq);
			atomic_set(&irq_status, atomic_read(&irq_status)+1);
		} else {
			atomic_set(&irq_status, atomic_read(&irq_status)+1);
		}
#endif
		pas230_light_enable(pas230);
	} else if (!new_value && (pas230->power_state & LIGHT_ENABLED)) {
		pas230_light_disable(pas230);
#ifdef PAS230_ALS_SENSOR_INT
		if(atomic_read(&irq_status) == 1) {
			atomic_set(&irq_status, atomic_read(&irq_status)-1);
		} else {
			disable_irq_wake(pas230->irq);
			disable_irq(pas230->irq);
			atomic_set(&irq_status, atomic_read(&irq_status)-1);
		}
#endif
		pas230->power_state &= ~LIGHT_ENABLED;
	}
	mutex_unlock(&pas230->power_lock);
	return size;
}
#endif

static ssize_t pas230_proximity_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	bool new_value;
	u8 tmp;
	int value = 0;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d", *buf);
		return -EINVAL;
	}

	SENSOR_LOG("proximity_status=%d", new_value);

	mutex_lock(&pas230->power_lock);
	if (new_value && !(pas230->power_state & PROXIMITY_ENABLED)) {
		/* read crosstalk value from fs */
		pas230->crosstalk = pas230_read_crosstalk_data_from_fs(pas230);
		//pas230->pdata->proximity_power(pas230->i2c_client, 1);
		pas230->power_state |= PROXIMITY_ENABLED;
		pas230_i2c_read(pas230, PART_ID, &tmp);
//		pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
		pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
		pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
		pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
		pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
		pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);			//setting to happen the interrupt fastest.
		pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
		pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp | PS_ON);
		pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
#ifdef PAS230_ALS_SENSOR_INT
		if(atomic_read(&irq_status) == -1) {
			enable_irq(pas230->irq);
			enable_irq_wake(pas230->irq);
			atomic_set(&irq_status, atomic_read(&irq_status)+1);
		} else {
			atomic_set(&irq_status, atomic_read(&irq_status)+1);
		}
#else
		enable_irq(pas230->irq);
		enable_irq_wake(pas230->irq);
#endif
		value = !((tmp&0x04)>>2);
		/* 0 is close, 1 is far */
		input_report_abs(pas230->proximity_input_dev, ABS_DISTANCE, value);
		input_sync(pas230->proximity_input_dev);
	} else if (!new_value && (pas230->power_state & PROXIMITY_ENABLED)) {
		pas230->power_state &= ~PROXIMITY_ENABLED;
#ifdef PAS230_ALS_SENSOR_INT
		if(atomic_read(&irq_status) == 1) {
			atomic_set(&irq_status, atomic_read(&irq_status)-1);
		} else {
			disable_irq_wake(pas230->irq);
			disable_irq(pas230->irq);
			atomic_set(&irq_status, atomic_read(&irq_status)-1);
		}
#else
		disable_irq_wake(pas230->irq);
		disable_irq(pas230->irq);
#endif
		pas230_i2c_read(pas230, MAIN_CTRL, &tmp);
		pas230_i2c_write(pas230, MAIN_CTRL, tmp & PS_OFF);
		//pas230->pdata->proximity_power(pas230->i2c_client, 0);
	}
	mutex_unlock(&pas230->power_lock);
	return size;
}

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	proxsensor_get_avgvalue(pas230);
	return sprintf(buf, "%d\n", pas230->avg[1]);
}

#ifdef DEBUG
static ssize_t proximity_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 dat[20]={0, };
	u8 data[20]={0, };

	pas230_i2c_readn(pas230, MAIN_CTRL, dat, 20);
	pas230_i2c_readn(pas230, INT_PST, data, 20);

	return sprintf(buf, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n"
					"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n"
					"%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
					dat[0],dat[1],dat[2],dat[3],dat[4],dat[5],dat[6],dat[7],dat[8],dat[9],
					dat[10],dat[11],dat[12],dat[13],dat[14],dat[15],dat[16],dat[17],dat[18],dat[19],
					data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],
					data[10],data[11],data[12],data[13],data[14],data[15],data[16],data[17],data[18],data[19]);
}


static ssize_t pas230_proximity_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned int reg, val;

	sscanf(buf,"%x%x\n", &reg, &val);

	reg_defaults[(u8)reg] = (u8)val;

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, reg, reg_defaults[reg]);
	mutex_unlock(&pas230->power_lock);

	SENSOR_ERR("reg=0x%x, write val=0x%x", reg, val);

	return size;
}
#endif

// sysfs for far_threshold
static ssize_t pas230_ps_thres_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int err=0, value=0;
	u8 ps_thres_low[2] = {0, };

	err = pas230_i2c_readn(pas230, PS_THRES_LOW, ps_thres_low, 2);
	value = ((ps_thres_low[1]&0x07)<<8) | ps_thres_low[0];

	if (err < 0) {
		SENSOR_ERR("read ps_thres_low failed");
		err = -EIO;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t pas230_ps_thres_low_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 ps_thres_low[2] = {0, };
	unsigned int val;
	sscanf(buf,"%d\n", &val);

	SENSOR_LOG("set near_offset = %d", val);
	pas230->near_threshold = val;

	ps_thres_low[0] = (u8)val;
	ps_thres_low[1] = (u8)((val>>8) & 0x07);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_THRES_LOW, ps_thres_low[0]);
	pas230_i2c_write(pas230, PS_THRES_LOW+1, ps_thres_low[1]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

// sysfs for near_threshold
static ssize_t pas230_ps_thres_up_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int err=0, value=0;
	u8 ps_thres_up[2] = {0, };

	err = pas230_i2c_readn(pas230, PS_THRES_UP, ps_thres_up, 2);
	value = ((ps_thres_up[1]&0x07)<<8) | ps_thres_up[0];

	if (err < 0) {
		SENSOR_ERR("read ps_thres_high failed");
		err = -EIO;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t pas230_ps_thres_up_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 ps_thres_up[2] = {0, };
	unsigned int val;
	sscanf(buf,"%d\n", &val);

	SENSOR_LOG("set far_offset = %d", val);
	pas230->far_threshold = val;

	ps_thres_up[0] = (u8)val;
	ps_thres_up[1] = (u8)((val>>8) & 0x07);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_THRES_UP, ps_thres_up[0]);
	pas230_i2c_write(pas230, PS_THRES_UP+1, ps_thres_up[1]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

#ifdef PAS230_ALS_SENSOR_INT
static ssize_t pas230_als_thres_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int err=0, value=0;
	u8 als_thres_low[3] = {0, };

	err = pas230_i2c_readn(pas230, ALS_THRES_LOW, als_thres_low, 3);
	value = ((als_thres_low[2]&0x07)<<16) | (als_thres_low[1]<<8) | als_thres_low[0];

	if (err < 0) {
		SENSOR_ERR("read als_thres_low failed");
		err = -EIO;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t pas230_als_thres_low_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 als_thres_low[3] = {0, };
	unsigned int val;
	sscanf(buf,"%d\n", &val);

	SENSOR_LOG("set als_low_thres = %d", val);
	pas230->als_low_threshold = val;

	als_thres_low[0] = (u8)val;
	als_thres_low[1] = (u8)(val>>8);
	als_thres_low[2] = (u8)((val>>16) & 0x07);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_THRES_LOW, als_thres_low[0]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+1, als_thres_low[1]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+2, als_thres_low[2]);
	mutex_unlock(&pas230->power_lock);

	return size;
}


static ssize_t pas230_als_thres_up_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int err=0, value=0;
	u8 als_thres_up[3] = {0, };

	err = pas230_i2c_readn(pas230, ALS_THRES_UP, als_thres_up, 2);
	value = ((als_thres_up[2]&0x07)<<16) | (als_thres_up[1]<<8) |als_thres_up[0];

	if (err < 0) {
		SENSOR_ERR("read als_thres_high failed");
		err = -EIO;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t pas230_als_thres_up_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 als_thres_up[3] = {0, };
	unsigned int val;
	sscanf(buf,"%d\n", &val);

	SENSOR_LOG("set als_up_thres = %d", val);
	pas230->als_up_threshold = val;

	als_thres_up[0] = (u8)val;
	als_thres_up[1] = (u8)(val>>8);
	als_thres_up[2] = (u8)((val>>16) & 0x07);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_THRES_UP, als_thres_up[0]);
	pas230_i2c_write(pas230, ALS_THRES_UP+1, als_thres_up[1]);
	pas230_i2c_write(pas230, ALS_THRES_UP+2, als_thres_up[2]);
	mutex_unlock(&pas230->power_lock);

	return size;
}
#endif

// sysfs for crosstalk_max
static ssize_t pas230_crosstalk_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pas230->pdata->crosstalk_max);
}

static ssize_t pas230_crosstalk_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	pas230->pdata->crosstalk_max = val;
	return size;
}

// sysfs for prox_cal_data
static ssize_t pas230_prox_cal_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int ret=0;
	if((ret = pas230_read_crosstalk_data_from_fs(pas230)) < 0)
		return sprintf(buf, "fail to read crosstalk data\n");
	return sprintf(buf, "%d\n", ret);
}

static ssize_t pas230_prox_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if ((ret = pas230_write_crosstalk_data_to_fs(val)) != 0)
		return SENSOR_LOG("fail to write crosstalk data to fs (err:%d)", ret);

	SENSOR_LOG("crosstalk data %d is written on fs", (int)val);
	return size;
}

// sysfs for run_calibration
static ssize_t pas230_run_calibration_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	int ret=0;

	if((ret = pas230_run_calibration(pas230)) < 0) {
		pas230->ps_cal_result = 0;
		SENSOR_LOG("fail proximity calibration (err:%d)", ret);
	}
	else {
		pas230->ps_cal_result = 1;
		SENSOR_LOG("success proximity calibration. crosstalk=%d", ret);
	}
	return size;
}

static ssize_t pas230_run_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pas230->ps_cal_result);
}

static ssize_t pas230_ps_led_freq_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set ps_led_freq = %ld", val);
	reg_defaults[PS_LED] = (reg_defaults[PS_LED] & 0x8f) | ((u8)val << 4 );

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_LED, reg_defaults[PS_LED]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_ps_led_freq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, PS_LED, &tmp);
	SENSOR_LOG("ps_led_reg = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read PS_LED_FREQ data\n");

	return sprintf(buf, "0x%02x\n", (tmp>>4) & 0x07);
}

static ssize_t pas230_ps_led_current_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err = 0;
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned int val = simple_strtoul(buf, NULL, 10);

	err = pas230_set_ps_led_current(pas230->i2c_client, val);
	if (err < 0) {
		SENSOR_ERR("set_ppcount failed to set ps_led_current %d", val);
		return err;
	}

	return size;
}

static ssize_t pas230_ps_led_current_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, PS_LED, &tmp);
	SENSOR_LOG("ps_led_reg = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read PS_LED_FREQ data\n");

	return sprintf(buf, "0x%02x\n", tmp & 0x07);
}

static ssize_t pas230_ps_pulses_count_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err = 0;
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned int val = simple_strtoul(buf, NULL, 10);

	err = pas230_set_ppcount(pas230->i2c_client, val);
	if (err < 0) {
		SENSOR_ERR("set_ppcount failed to set ppcount %d", val);
		return err;
	}

	return size;
}

static ssize_t pas230_ps_pulses_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, PS_PULSES, &tmp);
	SENSOR_LOG("ps_pulses_count = %d", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read PS_PULSES_COUNT data\n");

	return sprintf(buf, "%d\n", tmp);
}

static ssize_t pas230_ps_res_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set ps_resolution = %ld", val);
	reg_defaults[PS_MEAS_RATE] = (reg_defaults[PS_MEAS_RATE] & 0xe7) | ((u8)val << 3);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_ps_res_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, PS_MEAS_RATE, &tmp);
	SENSOR_LOG("ps_meas_rate = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read ps_resolution data\n");

	return sprintf(buf, "0x%02x\n", (tmp>>3) & 0x3 );
}

static ssize_t pas230_ps_meas_rate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set ps_meas_rate = %ld", val);
	reg_defaults[PS_MEAS_RATE] = (reg_defaults[PS_MEAS_RATE] & 0xf8) | ((u8)val);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_ps_meas_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, PS_MEAS_RATE, &tmp);
	SENSOR_LOG("ps_meas_rate = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read ps_resolution data\n");

	return sprintf(buf, "0x%02x\n", (tmp) & 0x7 );
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_als_res_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, ALS_CS_MEAS_RATE, &tmp);
	SENSOR_LOG("als_meas_rate = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read als_resolution data\n");

	return sprintf(buf, "0x%02x\n", (tmp>>4) & 0x3 );
}

static ssize_t pas230_als_res_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set als_resolution = %ld", val);
	reg_defaults[ALS_CS_MEAS_RATE] = (reg_defaults[ALS_CS_MEAS_RATE] & 0x8f) | ((u8)val << 4);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_CS_MEAS_RATE, reg_defaults[ALS_CS_MEAS_RATE]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_als_meas_rate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set als_meas_rate = %ld", val);
	reg_defaults[ALS_CS_MEAS_RATE] = (reg_defaults[ALS_CS_MEAS_RATE] & 0xf8) | ((u8)val);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_CS_MEAS_RATE, reg_defaults[ALS_CS_MEAS_RATE]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_als_meas_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, ALS_CS_MEAS_RATE, &tmp);
	SENSOR_LOG("als_meas_rate = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read als_resolution data\n");

	return sprintf(buf, "0x%02x\n", (tmp) & 0x7 );
}

static ssize_t pas230_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set als_gain = %ld", val);
	reg_defaults[ALS_CS_GAIN] = (reg_defaults[ALS_CS_GAIN] & 0xf8) | ((u8)val);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, ALS_CS_GAIN, reg_defaults[ALS_CS_GAIN]);
	mutex_unlock(&pas230->power_lock);

	return size;
}

static ssize_t pas230_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	u8 tmp = 0;
	int err = 0;

	err = pas230_i2c_read(pas230, ALS_CS_GAIN, &tmp);
	SENSOR_LOG("als_gain = 0x%02x", tmp);
	if(err < 0)
		return sprintf(buf, "fail to read als_gain data\n");

	return sprintf(buf, "0x%02x\n", (tmp) & 0x7 );
}

#endif

#ifdef DEBUG
static DEVICE_ATTR(proximity_reg, 0644, proximity_reg_show, pas230_proximity_reg_store);
#endif
#ifdef PAS230_ALS_SENSOR_ENABLE
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, poll_delay_show, poll_delay_store);
#endif
static DEVICE_ATTR(pdata, S_IRUGO | S_IWUSR | S_IWGRP, proximity_avg_show, NULL);
static DEVICE_ATTR(value, S_IRUGO | S_IWUSR | S_IWGRP, pas230_proximity_state_show, NULL);
#ifdef PAS230_ALS_SENSOR_ENABLE
static DEVICE_ATTR(alsdata, S_IRUGO | S_IWUSR | S_IWGRP, pas230_alsdata_show, NULL);
static DEVICE_ATTR(als_res, S_IRUGO | S_IWUSR | S_IWGRP, pas230_als_res_show, pas230_als_res_store);
static DEVICE_ATTR(als_meas_rate, S_IRUGO | S_IWUSR | S_IWGRP, pas230_als_meas_rate_show, pas230_als_meas_rate_store);
static DEVICE_ATTR(als_gain, S_IRUGO | S_IWUSR | S_IWGRP, pas230_als_gain_show, pas230_als_gain_store);

#endif
#ifdef PAS230_ALS_SENSOR_INT
static DEVICE_ATTR(als_up_thres, S_IRUGO | S_IWUSR | S_IWGRP, pas230_als_thres_up_show, pas230_als_thres_up_store);
static DEVICE_ATTR(als_low_thres, S_IRUGO | S_IWUSR | S_IWGRP, pas230_als_thres_low_show, pas230_als_thres_low_store);
#endif
static DEVICE_ATTR(far_offset, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_thres_low_show, pas230_ps_thres_low_store);
static DEVICE_ATTR(near_offset, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_thres_up_show, pas230_ps_thres_up_store);
static DEVICE_ATTR(prox_cal_data, S_IRUGO | S_IWUSR | S_IWGRP, pas230_prox_cal_data_show, pas230_prox_cal_data_store);
static DEVICE_ATTR(crosstalk_max, S_IRUGO | S_IWUSR | S_IWGRP, pas230_crosstalk_max_show, pas230_crosstalk_max_store);
static DEVICE_ATTR(run_calibration, S_IRUGO | S_IWUSR | S_IWGRP, pas230_run_calibration_show, pas230_run_calibration_store);
static DEVICE_ATTR(ps_led_freq, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_led_freq_show, pas230_ps_led_freq_store);
static DEVICE_ATTR(ps_led_current, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_led_current_show, pas230_ps_led_current_store);
static DEVICE_ATTR(ppcount, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_pulses_count_show, pas230_ps_pulses_count_store);
static DEVICE_ATTR(ps_res, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_res_show, pas230_ps_res_store);
static DEVICE_ATTR(ps_meas_rate, S_IRUGO | S_IWUSR | S_IWGRP, pas230_ps_meas_rate_show, pas230_ps_meas_rate_store);

#ifdef PAS230_ALS_SENSOR_ENABLE
static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		pas230_light_enable_show, pas230_light_enable_store);
#endif

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		pas230_proximity_enable_show, pas230_proximity_enable_store);

#ifdef PAS230_ALS_SENSOR_ENABLE
static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_alsdata.attr,
	&dev_attr_proximity_reg.attr,
	&dev_attr_als_res.attr,
	&dev_attr_als_meas_rate.attr,
	&dev_attr_als_gain.attr,
#ifdef PAS230_ALS_SENSOR_INT
	&dev_attr_als_up_thres.attr,
	&dev_attr_als_low_thres.attr,
#endif
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};
#endif

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_reg.attr,
	&dev_attr_proximity_enable.attr,
	&dev_attr_pdata.attr,
	&dev_attr_value.attr,
	&dev_attr_near_offset.attr,
	&dev_attr_far_offset.attr,
	&dev_attr_prox_cal_data.attr,
	&dev_attr_crosstalk_max.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_ps_led_freq.attr,
	&dev_attr_ps_led_current.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_ps_res.attr,
	&dev_attr_ps_meas_rate.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

#ifdef PAS230_ALS_SENSOR_ENABLE
static void pas230_work_func_light(struct work_struct *work)
{
	int als;
	struct pas230_data *pas230 = container_of(work, struct pas230_data,
				work_light);

	als = lightsensor_get_alsvalue(pas230);

	input_report_abs(pas230->light_input_dev, ABS_MISC, als);
	input_sync(pas230->light_input_dev);
}
#endif

#ifdef PAS230_POLLING_MODE_ENABLE
static void pas230_work_func_prox(struct work_struct *work)
{
	struct pas230_data *pas230 = container_of(work, struct pas230_data, work_prox);

	proxsensor_get_avgvalue(pas230);
}
#endif

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
#ifdef PAS230_ALS_SENSOR_ENABLE
static enum hrtimer_restart pas230_light_timer_func(struct hrtimer *timer)
{
	struct pas230_data *pas230
			= container_of(timer, struct pas230_data, light_timer);
	queue_work(pas230->light_wq, &pas230->work_light);
	hrtimer_forward_now(&pas230->light_timer, pas230->light_poll_delay);
	return HRTIMER_RESTART;
}
#endif

#ifdef PAS230_POLLING_MODE_ENABLE
static enum hrtimer_restart pas230_prox_timer_func(struct hrtimer *timer)
{
	struct pas230_data *pas230
			= container_of(timer, struct pas230_data, prox_timer);
	queue_work(pas230->prox_wq, &pas230->work_prox);
	hrtimer_forward_now(&pas230->prox_timer, pas230->prox_poll_delay);
	return HRTIMER_RESTART;
}
#endif

#ifdef PAS230_ALS_SENSOR_INT
/* interrupt happened due to transition/change of ALS & proximity state */
irqreturn_t pas230_irq_thread_fn(int irq, void *data)
{
	struct pas230_data *pas230 = data;
	u8 tmp = 0;
	int ps_data = 0;
	int als_data = 0;
	int value = 0;
	u8 ps_value[2] = {0, };

	SENSOR_LOG("pas230_interrupt");

	if (wake_lock_active(&pas230->prx_wake_lock))
		wake_unlock(&pas230->prx_wake_lock);
	wake_lock_timeout(&pas230->prx_wake_lock, 2 * HZ);

	if (gpio_get_value(pas230->pdata->irq_gpio) < 0) {
		SENSOR_ERR("gpio_get_value error");
		return IRQ_HANDLED;
	}

	/*reset IRQ*/
	pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
	SENSOR_LOG("status = 0x%x", tmp);

	if(tmp & PRX_INT_MASK)
	{
		/* for debugging : going to be removed */
		pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
		ps_data = ((ps_value[1]&0x07)<<8) | ps_value[0];

		value = !((tmp & PRX_LGC_MASK)>>2);

		if (value == 0){	// FAR -> NEAR
			SENSOR_LOG("proximity status is changed from FAR -> NEAR, ps_data=%d", ps_data);
			pas230->ps_value = value;
		}
		else if (value == 1) {	// NEAR -> FAR
			SENSOR_LOG("proximity status is changed from NEAR -> FAR, ps_data=%d", ps_data);
			pas230->ps_value = value;
		}
		if (atomic_read(&part_id) == 0) {
		pas230_proximity_rsp(pas230, ps_data);
		}

		/* 0 is close, 1 is far */
		input_report_abs(pas230->proximity_input_dev, ABS_DISTANCE, value);
		input_sync(pas230->proximity_input_dev);
	}

	if(tmp & ALS_INT_MASK)
	{
		als_data = lightsensor_get_alsvalue(pas230);

		input_report_abs(pas230->light_input_dev, ABS_MISC, als_data);
		input_sync(pas230->light_input_dev);

		if (als_data  >= ALS_LUX_THRESHOLD)
		{
			SENSOR_LOG("als_data(%d)>=THRESOLD(%d), als state is changed dark to bright"
				, als_data, ALS_LUX_THRESHOLD);

			pas230_write_als_thres_up(ALS_THRESHOLD_MAX, pas230);
			pas230_write_als_thres_low(pas230->pdata->als_low_thres, pas230);
		}
		else if (als_data  < ALS_LUX_THRESHOLD)
		{
			SENSOR_LOG("als_data(%d)<THRESOLD(%d), als state is changed bright to dark"
				, als_data, ALS_LUX_THRESHOLD);

			pas230_write_als_thres_up(pas230->pdata->als_up_thres, pas230);
			pas230_write_als_thres_low(ALS_THRESHOLD_MIN, pas230);
		}
		else
		{
			SENSOR_LOG("als state is not changed. als_data(%d)", als_data);
		}
	}
	return IRQ_HANDLED;
}
#else
/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t pas230_irq_thread_fn(int irq, void *data)
{
	struct pas230_data *pas230 = data;
	u8 tmp = 0;
	int ps_data = 0;
	int value = 0;
	u8 ps_value[2] = {0, };

	SENSOR_LOG("proximity_interrupt");

	if (wake_lock_active(&pas230->prx_wake_lock))
		wake_unlock(&pas230->prx_wake_lock);
	wake_lock_timeout(&pas230->prx_wake_lock, 2 * HZ);

	if (gpio_get_value(pas230->pdata->irq_gpio) < 0) {
		SENSOR_ERR("gpio_get_value error");
		return IRQ_HANDLED;
	}

	/*reset IRQ*/
	pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
	SENSOR_LOG("status = 0x%x", tmp);

	/* for debugging : going to be removed */
#if DEBUG
	pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
	ps_data = ((ps_value[1]&0x07)<<8) | ps_value[0];
#endif

	value = !((tmp&0x04)>>2);

	if (value == 0){	// FAR -> NEAR
		SENSOR_LOG("proximity status is changed from FAR -> NEAR, ps_data=%d", ps_data);
		pas230->ps_value = value;
	}
	else if (value == 1) {	// NEAR -> FAR
		SENSOR_LOG("proximity status is changed from NEAR -> FAR, ps_data=%d", ps_data);
		pas230->ps_value = value;
	}
	if (atomic_read(&part_id) == 0) {
		pas230_proximity_rsp(pas230, ps_data);
	}

	/* 0 is close, 1 is far */
	input_report_abs(pas230->proximity_input_dev, ABS_DISTANCE, value);
	input_sync(pas230->proximity_input_dev);

	return IRQ_HANDLED;
}
#endif

static int pas230_setup_irq(struct pas230_data *pas230)
{
	int rc = -EIO;
	int irq;

	if (gpio_is_valid(pas230->pdata->irq_gpio)) {
		irq = gpio_to_irq(pas230->pdata->irq_gpio);
		rc = request_threaded_irq(irq, NULL, pas230_irq_thread_fn,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				PAS230_DRV_NAME, (void*)pas230);
		if (rc < 0) {
			SENSOR_ERR("request_irq(%d) failed for gpio %d (%d)", irq, irq, rc);
			return rc;
		}

		/* start with interrupts disabled */
		disable_irq(irq);
		pas230->irq = irq;
	}
	else {
		SENSOR_ERR("irq gpio not provided");
	}
	return rc;
}

static int pas230_setup_reg(struct pas230_data *pas230)
{
	int err = 0;
	u8 tmp;
	int value = 0;
	u8 ps_value[2] = {0, };

	SENSOR_FUN();

	/* initializing the proximity and light sensor registers */
	mutex_lock(&pas230->power_lock);
	//pas230->pdata->proximity_power(pas230->i2c_client, 1);
	pas230_i2c_read(pas230, PART_ID, &tmp);
	if ( tmp == 0xb4) {
		atomic_set(&part_id, 0);
		SENSOR_LOG("PAS230 PART_ID = 0xb4");
	} else if ( tmp == 0xb1) {
		atomic_set(&part_id, 1);
		SENSOR_LOG("PAS230 PART_ID = 0xb1");
	}
	pas230_i2c_read(pas230, MAIN_STATUS, &tmp);
	pas230_i2c_write(pas230, PS_LED, reg_defaults[PS_LED]);
	pas230_i2c_write(pas230, PS_PULSES, pas230->pdata->ppcount);
	pas230_i2c_write(pas230, PS_MEAS_RATE, reg_defaults[PS_MEAS_RATE]);
#ifdef PAS230_ALS_SENSOR_ENABLE
	pas230_i2c_write(pas230, ALS_CS_MEAS_RATE, reg_defaults[ALS_CS_MEAS_RATE]);
	if(pas230->pdata->als_gain >= 0) {
		pas230_i2c_write(pas230, ALS_CS_GAIN, pas230->pdata->als_gain);
		SENSOR_LOG("als_gain = %d", pas230->pdata->als_gain);
	} else {
		pas230_i2c_write(pas230, ALS_CS_GAIN, reg_defaults[ALS_CS_GAIN]);
	}
#endif
	pas230_i2c_write(pas230, INT_CFG, reg_defaults[INT_CFG]);
	pas230_i2c_write(pas230, INT_PST, reg_defaults[INT_PST]);
	pas230_i2c_write(pas230, PS_THRES_UP, reg_defaults[PS_THRES_UP]);
	pas230_i2c_write(pas230, PS_THRES_UP+1, reg_defaults[PS_THRES_UP+1]);
	pas230_i2c_write(pas230, PS_THRES_LOW, reg_defaults[PS_THRES_LOW]);
	pas230_i2c_write(pas230, PS_THRES_LOW+1, reg_defaults[PS_THRES_LOW+1]);
	pas230_i2c_write(pas230, PS_CAN, reg_defaults[PS_CAN]);
	pas230_i2c_write(pas230, PS_CAN+1, reg_defaults[PS_CAN+1]);
#ifdef PAS230_ALS_SENSOR_ENABLE
	pas230_i2c_write(pas230, ALS_THRES_UP, reg_defaults[ALS_THRES_UP]);
	pas230_i2c_write(pas230, ALS_THRES_UP+1, reg_defaults[ALS_THRES_UP+1]);
	pas230_i2c_write(pas230, ALS_THRES_UP+2, reg_defaults[ALS_THRES_UP+2]);
	pas230_i2c_write(pas230, ALS_THRES_LOW, reg_defaults[ALS_THRES_LOW]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+1, reg_defaults[ALS_THRES_LOW+1]);
	pas230_i2c_write(pas230, ALS_THRES_LOW+2, reg_defaults[ALS_THRES_LOW+2]);
	pas230_i2c_write(pas230, ALS_THRES_VAR, reg_defaults[ALS_THRES_VAR]);
#endif
	pas230_i2c_write(pas230, MAIN_CTRL, ALL_ON);
	mutex_unlock(&pas230->power_lock);

	/* printing the inital proximity value with no contact */
	msleep(101);
	err = pas230_i2c_readn(pas230, PS_DATA, ps_value, 2);
	value = ((ps_value[1]&0x07)<<8) | ps_value[0];
	if (err < 0) {
		SENSOR_ERR("read ps_data failed");
		err = -EIO;
	}

	SENSOR_DBG("proximity value=%d", value);

	mutex_lock(&pas230->power_lock);
	pas230_i2c_write(pas230, MAIN_CTRL, ALL_OFF);
	//pas230->pdata->proximity_power(pas230->i2c_client, 0);
	mutex_unlock(&pas230->power_lock);

	return err;
}

static int pas230_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct input_dev *input_dev;
	struct pas230_data *pas230;
#ifdef CONFIG_OF
	struct pas230_platform_data *platform_data;
#endif

	SENSOR_LOG("probe start");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_ERR("I2c functionality check failed!");
		return ret;
	}

	pas230 = devm_kzalloc(&client->dev, sizeof(struct pas230_data), GFP_KERNEL);
	if (!pas230) {
		SENSOR_ERR("failed to alloc memory for module data");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev, sizeof(struct pas230_platform_data), GFP_KERNEL);
		if (!platform_data) {
			SENSOR_ERR("Failed to allocate memory");
			return -ENOMEM;
		}
		pas230->pdata = platform_data;
		client->dev.platform_data = platform_data;
		if ((ret = sensor_parse_dt(&client->dev, pas230->pdata))) {
			SENSOR_ERR("Sensor_parse_dt error");
			return ret;
		}
	}
#else
	pas230->pdata = client->dev.platform_data;
#endif
	pas230->i2c_client = client;
	i2c_set_clientdata(client, pas230);

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init) {
		ret = platform_data->init(client);
	}
/*
	if (platform_data->proximity_power) {
		ret = platform_data->proximity_power(client, true);
	}
*/
#endif

	/* wake lock init */
	wake_lock_init(&pas230->prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");
	mutex_init(&pas230->power_lock);

	/* setup initial registers */
	ret = pas230_setup_reg(pas230);
	if (ret < 0) {
		SENSOR_ERR("could not setup regs");
		goto err_setup_reg;
	}

	ret = pas230_setup_irq(pas230);
	if (ret) {
		SENSOR_ERR("could not setup irq");
		goto err_setup_irq;
	}

	/* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		SENSOR_ERR("could not allocate input device");
		goto err_input_allocate_device_proximity;
	}
	pas230->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, pas230);
	input_dev->name = "proximity";
	input_dev->uniq = PAS230_DRV_NAME;
	input_dev->dev.init_name = LGE_PROXIMITY_NAME;
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		SENSOR_ERR("could not register input device");
		input_free_device(input_dev);
		goto err_input_register_device_proximity;
	}
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &proximity_attribute_group);
	if (ret) {
		SENSOR_ERR("could not create sysfs group");
		goto err_sysfs_create_group_proximity;
	}

#ifdef PAS230_ALS_SENSOR_ENABLE
	/* light_timer settings. we poll for light values using a timer. */
	hrtimer_init(&pas230->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pas230->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	pas230->light_timer.function = pas230_light_timer_func;
#endif

#ifdef PAS230_POLLING_MODE_ENABLE
	/* prox_timer settings. we poll for light values using a timer. */
	hrtimer_init(&pas230->prox_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pas230->prox_poll_delay = ns_to_ktime(2000 * NSEC_PER_MSEC);
	pas230->prox_timer.function = pas230_prox_timer_func;
#endif

#ifdef PAS230_ALS_SENSOR_ENABLE
	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	pas230->light_wq = create_singlethread_workqueue("pas230_light_wq");
	if (!pas230->light_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create light workqueue");
		goto err_create_light_workqueue;
	}
#endif

#ifdef PAS230_POLLING_MODE_ENABLE
	pas230->prox_wq = create_singlethread_workqueue("pas230_prox_wq");
	if (!pas230->prox_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create prox workqueue");
		goto err_create_prox_workqueue;
	}
#endif

	/* this is the thread function we run on the work queue */
#ifdef PAS230_ALS_SENSOR_ENABLE
	INIT_WORK(&pas230->work_light, pas230_work_func_light);
#endif
#ifdef PAS230_POLLING_MODE_ENABLE
	INIT_WORK(&pas230->work_prox, pas230_work_func_prox);
#endif

#ifdef PAS230_ALS_SENSOR_ENABLE
	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		SENSOR_ERR("could not allocate input device");
		ret = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, pas230);
	input_dev->name = "light";
	input_dev->uniq = PAS230_DRV_NAME;
	input_dev->dev.init_name = LGE_LIGHT_NAME;

	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);


	ret = input_register_device(input_dev);
	if (ret < 0) {
		SENSOR_ERR("could not register input device");
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	pas230->light_input_dev = input_dev;
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &light_attribute_group);
	if (ret) {
		SENSOR_ERR("could not create sysfs group");
		goto err_sysfs_create_group_light;
	}
#endif

	/* set initial proximity value as 1. ahn_KT */
	input_report_abs(pas230->proximity_input_dev, ABS_DISTANCE, 1);
	input_sync(pas230->proximity_input_dev);

	SENSOR_ERR("proximity sensor probed successfully");

	goto done;

#ifdef PAS230_ALS_SENSOR_ENABLE
err_sysfs_create_group_light:
	sysfs_remove_group(&input_dev->dev.kobj, &light_attribute_group);
err_input_register_device_light:
	input_unregister_device(pas230->light_input_dev);
err_input_allocate_device_light:
#endif

#ifdef PAS230_POLLING_MODE_ENABLE
err_create_prox_workqueue:
	destroy_workqueue(pas230->prox_wq);
#endif

#ifdef PAS230_ALS_SENSOR_ENABLE
err_create_light_workqueue:
	destroy_workqueue(pas230->light_wq);
#endif
err_sysfs_create_group_proximity:
	sysfs_remove_group(&pas230->proximity_input_dev->dev.kobj,
			&proximity_attribute_group);
err_input_register_device_proximity:
	input_unregister_device(pas230->proximity_input_dev);
err_input_allocate_device_proximity:

err_setup_irq:
	free_irq(pas230->irq, NULL);
err_setup_reg:
	wake_lock_destroy(&pas230->prx_wake_lock);
	mutex_destroy(&pas230->power_lock);
done:
	return ret;
}

static int pas230_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   pas230->power_state because we use that state in resume.
	*/
	struct pas230_data *pas230 = dev_get_drvdata(dev);
	SENSOR_FUN();

	hrtimer_cancel(&pas230->light_timer);
	cancel_work_sync(&pas230->work_light);
	return 0;
}

static int pas230_resume(struct device *dev)
{
	/* Turn power back on if we were before suspend. */
	SENSOR_FUN();
	return 0;
}

static int pas230_i2c_remove(struct i2c_client *client)
{
	struct pas230_data *pas230 = i2c_get_clientdata(client);
	SENSOR_FUN();

	/* free irq */
	free_irq(pas230->irq, NULL);

	/* device off */
	if (pas230->power_state) {
#ifdef PAS230_ALS_SENSOR_ENABLE
		if (pas230->power_state & LIGHT_ENABLED)
			pas230_light_disable(pas230);
#endif
		if (pas230->power_state & PROXIMITY_ENABLED) {
			pas230_i2c_write(pas230, MAIN_CTRL, PS_OFF);
			//pas230->pdata->proximity_power(pas230->i2c_client, 0);
		}
	}

	/* destroy workqueue */
#ifdef PAS230_ALS_SENSOR_ENABLE
	destroy_workqueue(pas230->light_wq);
#endif
#ifdef PAS230_POLLING_MODE_ENABLE
	destroy_workqueue(pas230->prox_wq);
#endif

	/* input device destroy */
#ifdef PAS230_ALS_SENSOR_ENABLE
	sysfs_remove_group(&pas230->light_input_dev->dev.kobj, &light_attribute_group);
	input_unregister_device(pas230->light_input_dev);
#endif
	sysfs_remove_group(&pas230->proximity_input_dev->dev.kobj, &proximity_attribute_group);
	input_unregister_device(pas230->proximity_input_dev);

	/* lock destroy */
	mutex_destroy(&pas230->power_lock);
	wake_lock_destroy(&pas230->prx_wake_lock);

	return 0;
}

static const struct i2c_device_id pas230_device_id[] = {
	{"pas230", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pas230_device_id);

static const struct dev_pm_ops pas230_pm_ops = {
	.suspend = pas230_suspend,
	.resume = pas230_resume
};

#ifdef CONFIG_OF
static struct of_device_id pas230_match_table[] = {
	{ .compatible = "partron,pas230",},
	{ },
};
#else
#define pas230_match_table NULL
#endif

static struct i2c_driver pas230_i2c_driver = {
	.driver = {
		.name = "pas230",
		.owner = THIS_MODULE,
		.of_match_table = pas230_match_table,
		.pm = &pas230_pm_ops
	},
	.probe		= pas230_i2c_probe,
	.remove		= pas230_i2c_remove,
	.id_table	= pas230_device_id,
};


static int __init pas230_init(void)
{
	SENSOR_FUN();
	return i2c_add_driver(&pas230_i2c_driver);
}

static void __exit pas230_exit(void)
{
	SENSOR_FUN();
	i2c_del_driver(&pas230_i2c_driver);
}

module_init(pas230_init);
module_exit(pas230_exit);

MODULE_AUTHOR("partron@partron.co.kr");
MODULE_DESCRIPTION("Optical Sensor driver for pas230");
MODULE_LICENSE("GPL");
