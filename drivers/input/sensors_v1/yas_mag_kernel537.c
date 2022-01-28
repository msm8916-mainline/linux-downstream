/*
 * Copyright (c) 2014-2015 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "yas537.h"


#define YAS_I2C_NAME		"yas537"
#define YAS_INPUT_DEVICE_NAME   "compass"


#define YAS537_REG_DIDR			(0x80)
#define YAS537_REG_CMDR			(0x81)
#define YAS537_REG_CONFR		(0x82)
#define YAS537_REG_INTRVLR		(0x83)
#define YAS537_REG_OXR			(0x84)
#define YAS537_REG_OY1R			(0x85)
#define YAS537_REG_OY2R			(0x86)
#define YAS537_REG_AVRR			(0x87)
#define YAS537_REG_HCKR			(0x88)
#define YAS537_REG_LCKR			(0x89)
#define YAS537_REG_SRSTR		(0x90)
#define YAS537_REG_ADCCALR		(0x91)
#define YAS537_REG_MTCR			(0x93)
#define YAS537_REG_OCR			(0x9e)
#define YAS537_REG_TRMR			(0x9f)
#define YAS537_REG_RCMR			(0xa0)
#define YAS537_REG_DATAR		(0xb0)
#define YAS537_REG_CALR			(0xc0)

#define YAS537_DATA_UNDERFLOW		(0)
#define YAS537_DATA_OVERFLOW		(16383)
#define YAS537_DEVICE_ID		(0x07)	/* YAS537 (MS-3T) */

#define YAS_X_OVERFLOW			(0x01)
#define YAS_X_UNDERFLOW			(0x02)
#define YAS_Y1_OVERFLOW			(0x04)
#define YAS_Y1_UNDERFLOW		(0x08)
#define YAS_Y2_OVERFLOW			(0x10)
#define YAS_Y2_UNDERFLOW		(0x20)
#define YAS_OVERFLOW	(YAS_X_OVERFLOW|YAS_Y1_OVERFLOW|YAS_Y2_OVERFLOW)
#define YAS_UNDERFLOW	(YAS_X_UNDERFLOW|YAS_Y1_UNDERFLOW|YAS_Y2_UNDERFLOW)

#define YAS537_MAG_STATE_NORMAL		(0)
#define YAS537_MAG_STATE_INIT_COIL	(1)
#define YAS537_MAG_INITCOIL_TIMEOUT	(1000)	/* msec */
#define YAS537_MAG_POWER_ON_RESET_TIME	(4000)	/* usec */
#define YAS537_MAG_NOTRANS_POSITION	(2)

#define YAS537_MAG_AVERAGE_8		(0)
#define YAS537_MAG_AVERAGE_16		(1)
#define YAS537_MAG_AVERAGE_32		(2)
#define YAS537_MAG_AVERAGE_64		(3)
#define YAS537_MAG_AVERAGE_128		(4)
#define YAS537_MAG_AVERAGE_256		(5)

#define YAS537_MAG_RCOIL_TIME		(65)

#define set_vector(to, from) \
	{int _l; for (_l = 0; _l < 3; _l++) (to)[_l] = (from)[_l]; }

static struct i2c_client *this_client;

struct yas_state {
	struct mutex lock;
	struct yas_mag_driver mag;
	struct input_dev *input_dev;
	struct sensors_classdev cdev;
	struct delayed_work work;
	int32_t poll_delay;
	atomic_t enable;
	int32_t compass_data[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif
	struct device *dev;
	struct class *class;
	/*laizhilong add start*/
	uint8_t data_valid;
	struct	regulator		*vdd;
	struct	regulator		*vio;
	/*laizhilong add end*/
	/*shuaofeng add start*/
	int i2c_add;
	/*shuaofeng add start*/
	
};

/* POWER SUPPLY VOLTAGE RANGE */
#define YAS_VDD_MIN_UV	2850000
#define YAS_VDD_MAX_UV	2850000
#define YAS_VIO_MIN_UV	1800000
#define YAS_VIO_MAX_UV	1800000

#define TAG  "Msensor "       //"Gsensor" "PAsensor" "GYsensor"
#define TAGI "Msensor.I "     //KERN_INFO 
#define TAGE "Msensor.E "     //KERN_ERR 

static struct sensors_classdev sensors_cdev = {
	.name = "yas537-mag",
	.vendor = "Yamaha",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "2000",
	.resolution = "1",
	.sensor_power = "0.28",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10000,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


/***************yas537_mag_drv   begin!***************/
struct yas_cal {
	int16_t a[9];
	uint8_t k, ver;
};

struct yas_cdriver {
	int initialized;
	struct yas_driver_callback cbk;
	int measure_state;
	int invalid_data;
	uint32_t invalid_data_time;
	int position;
	int32_t delay;
	int enable;
	uint8_t dev_id;
	const int8_t *transform;
	int average;
	int8_t hard_offset[3];
	uint32_t current_time;
	uint16_t last_raw[4];
	uint16_t last_after_rcoil[3];
	struct yas_cal cal;
	int16_t overflow[3], underflow[3];
	struct yas_matrix static_matrix;
	int noise_rcoil_flag;
	int flowflag;
};

static const struct yas_matrix no_conversion
	= { {10000, 0, 0, 0, 10000, 0, 0, 0, 10000} };
static const int measure_time_worst[] = {800, 1100, 1500, 3000, 6000, 12000};

static const int8_t YAS537_TRANSFORMATION[][9] = {
	{-1,  0,  0,  0, -1,  0,  0,  0,  1 },
	{ 0, -1,  0,  1,  0,  0,  0,  0,  1 },
	{ 1,  0,  0,  0,  1,  0,  0,  0,  1 },
	{ 0,  1,  0, -1,  0,  0,  0,  0,  1 },
	{ 1,  0,  0,  0, -1,  0,  0,  0, -1 },
	{ 0,  1,  0,  1,  0,  0,  0,  0, -1 },
	{-1,  0,  0,  0,  1,  0,  0,  0, -1 },
	{ 0, -1,  0, -1,  0,  0,  0,  0, -1 },
};
static struct yas_cdriver driver;

static int yas_set_enable_wrap(int enable, int rcoil);
static int yas_set_enable(int enable);
static int single_read(int ldtc, int fors, int *busy, uint16_t *t,
		uint16_t *xy1y2, int *ouflow, int corr);

static int yas_open(void)
{
	if (driver.cbk.device_open(YAS_TYPE_MAG) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	driver.cbk.usleep(YAS537_MAG_POWER_ON_RESET_TIME);
	return YAS_NO_ERROR;
}
#define yas_read(a, b, c) \
	(driver.cbk.device_read(YAS_TYPE_MAG, (a), (b), (c)))
static int yas_single_write(uint8_t addr, uint8_t data)
{
	return driver.cbk.device_write(YAS_TYPE_MAG, addr, &data, 1);
}

static void apply_matrix(struct yas_vector *xyz, struct yas_matrix *m)
{
	int32_t tmp[3];
	int i;
	if (m == NULL)
		return;
	for (i = 0; i < 3; i++)
		tmp[i] = ((m->m[i*3]/10) * (xyz->v[0]/10)
				+ (m->m[i*3+1]/10) * (xyz->v[1]/10)
				+ (m->m[i*3+2]/10) * (xyz->v[2]/10)) / 100;
	for (i = 0; i < 3; i++)
		xyz->v[i] = tmp[i];
}

static uint32_t curtime(void)
{
	if (driver.cbk.current_time)
		return driver.cbk.current_time();
	else
		return driver.current_time;
}

static int invalid_magnetic_field(uint16_t *cur, uint16_t *last)
{
	int16_t invalid_thresh[] = {1000, 1000, 1000};
	int i;
	for (i = 0; i < 3; i++)
		if (invalid_thresh[i] < ABS(cur[i] - last[i]))
			return 1;
	return 0;
}

static int start_yas537(int ldtc, int fors, int cont)
{
	uint8_t data = 0x01;
	data = (uint8_t)(data | (ldtc<<1));
	data = (uint8_t)(data | (fors<<2));
	data = (uint8_t)(data | (cont<<5));
	if (yas_single_write(YAS537_REG_CMDR, data) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	return YAS_NO_ERROR;
}

static int cont_start_yas537(void)
{
	if (start_yas537(0, 0, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	/* wait for the first measurement */
	driver.cbk.usleep(measure_time_worst[driver.average]);
	return YAS_NO_ERROR;
}

static void sensitivity_correction(uint16_t *xy1y2, uint16_t t)
{
	int32_t h[3];
	int i;
	if (t == 8176)
		return;
	for (i = 0; i < 3; i++) {
		h[i] = (int32_t) xy1y2[i] - 8192;
		h[i] = 100000 * h[i] / (100000 - 11 * (t - 8176));
		xy1y2[i] = CLIP(h[i], -8192, 8191) + 8192;
	}
}

static int read_yas537(int *busy, uint16_t *t, uint16_t *xy1y2,
		int *ouflow, int corr)
{
	struct yas_cal *c = &driver.cal;
	int32_t s[3], h[3];
	uint8_t data[8];
	int i;
	if (yas_read(YAS537_REG_DATAR, data, 8) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	*busy = data[2]>>7;
	*t = (uint16_t)((data[0]<<8) | data[1]);
	xy1y2[0] = (uint16_t)(((data[2]&0x3f)<<8) | data[3]);
	xy1y2[1] = (uint16_t)((data[4]<<8) | data[5]);
	xy1y2[2] = (uint16_t)((data[6]<<8) | data[7]);
	for (i = 0; i < 3; i++)
		s[i] = xy1y2[i] - 8192;
	*ouflow = 0;
	for (i = 0; i < 3; i++) {
		h[i] = (c->k * (c->a[i*3]*s[0] + c->a[i*3+1]*s[1]
					+ c->a[i*3+2]*s[2])) / 8192;
		h[i] = CLIP(h[i], -8192, 8191) + 8192;
		if (corr)
			xy1y2[i] = h[i];
		driver.last_raw[i] = h[i];
		if (driver.overflow[i] <= h[i])
			*ouflow |= (1<<(i*2));
		if (h[i] <= driver.underflow[i])
			*ouflow |= (1<<(i*2+1));
	}
	driver.last_raw[i] = *t;
	return YAS_NO_ERROR;
}

static int update_intrvlr(int32_t delay)
{
	uint8_t data;
	/* delay 4.1 x SMPLTIM [7:0] msec */
	if (((int32_t)4100 * 255 + measure_time_worst[driver.average]) / 1000
			< delay)
		delay = (int32_t)4100 * 255
			+ measure_time_worst[driver.average];
	else
		delay *= 1000;
	delay = (delay - measure_time_worst[driver.average]) / 4100;
	if (delay <= 1)
		data = 2;
	else
		data = (uint8_t) delay;
	if (yas_single_write(YAS537_REG_INTRVLR, data) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	return YAS_NO_ERROR;
}

static int rcoil(int mode, int num)
{
	struct yas_cal *c = &driver.cal;
	uint8_t mode0 = c->ver == 1 ? 0x02 : 0x00;
	uint8_t mode1 = c->ver == 1 ? 0x00 : 0x02;
	int i;
	if (yas_single_write(YAS537_REG_RCMR, mode == 0 ? mode0 : mode1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	for (i = 0; i < num; i++) {
		if (yas_single_write(YAS537_REG_CONFR, 0x08) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		driver.cbk.usleep(YAS537_MAG_RCOIL_TIME);
	}
	return YAS_NO_ERROR;
}

static int rcoil_and_record_data(int flowflag, uint16_t *xy1y2, int *ouflow)
{
	uint16_t t;
	int rt, busy;

	if (flowflag) {
		if (rcoil(0, 1) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (rcoil(1, 5) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	} else {
		if (rcoil(1, 1) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	rt = single_read(0, 0, &busy, &t, xy1y2, ouflow, 1);
	if (rt < 0)
		return rt;
	if (busy)
		return YAS_ERROR_BUSY;
	sensitivity_correction(xy1y2, t);
	set_vector(driver.last_after_rcoil, xy1y2);
	return YAS_NO_ERROR;
}

static int enable_rcoil(void)
{
	uint16_t xy1y2[3], xy1y2_after[3], t;
	int rt, busy, overflow;

	rt = single_read(0, 0, &busy, &t, xy1y2, &overflow, 1);
	if (rt < 0)
		return rt;
	if (busy)
		return YAS_ERROR_BUSY;
	sensitivity_correction(xy1y2, t);
	if (!overflow) {
		rt = rcoil_and_record_data(0, xy1y2_after, &overflow);
		if (rt < 0)
			return rt;
		if (!overflow && !invalid_magnetic_field(xy1y2, xy1y2_after))
			return 1;
	}
	rt = rcoil_and_record_data(1, xy1y2, &overflow);
	if (rt < 0)
		return rt;
	if (overflow)
		return 0;
	return 1;
}

static int reset_yas537(void)
{
	static const uint8_t avrr[] = {0x50, 0x60, 0x70, 0x71, 0x72, 0x73};
	uint8_t data[17];
	int i;

	if (yas_single_write(YAS537_REG_SRSTR, 0x02) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_read(YAS537_REG_CALR, data, 17) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	for (i = 0; i < 3; i++) {
		if (yas_single_write(YAS537_REG_MTCR+i,
					data[i]) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (yas_single_write(YAS537_REG_OXR+i,
					data[i+12]) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (yas_single_write(YAS537_REG_MTCR+i,
				(data[i] & 0xe0) | 0x10) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_HCKR, (data[15]>>3)&0x1e) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_LCKR, (data[15]<<1)&0x1e) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_OCR, data[16]&0x3f) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_ADCCALR, 0x03) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_ADCCALR+1, 0xf8) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_TRMR, 0xff) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (update_intrvlr(driver.delay) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_single_write(YAS537_REG_AVRR, avrr[driver.average]) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	return YAS_NO_ERROR;
}

static int single_read(int ldtc, int fors, int *busy, uint16_t *t,
		uint16_t *xy1y2, int *ouflow, int corr)
{
	if (start_yas537(ldtc, fors, 0) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	driver.cbk.usleep(measure_time_worst[driver.average]);
	if (read_yas537(busy, t, xy1y2, ouflow, corr) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	return YAS_NO_ERROR;
}

static void xy1y2_to_xyz(uint16_t *xy1y2, int32_t *xyz)
{
	xyz[0] = ((int32_t)xy1y2[0] - 8192) * 300;
	xyz[1] = ((int32_t)xy1y2[1] - xy1y2[2]) * 1732 / 10;
	xyz[2] = ((int32_t)-xy1y2[1] - xy1y2[2] + 16384) * 300;
}

static int sensitivity_measuremnet(int32_t *sx, int32_t *sy)
{
	uint16_t p[3], m[3], t;
	struct yas_cal *c = &driver.cal;
	int busy, flowon = 0, flowoff = 0, rt;
	rt = single_read(1, 0, &busy, &t, p, &flowon, 0);
	if (rt < 0)
		return rt;
	if (busy)
		return YAS_ERROR_BUSY;
	rt = single_read(1, 1, &busy, &t, m, &flowoff, 0);
	if (rt < 0)
		return rt;
	if (busy)
		return YAS_ERROR_BUSY;
	*sx = (int32_t)c->k * 128 * (p[0] - m[0]) / 8192 * 300 / YAS_MAG_VCORE;
	*sy = (int32_t)c->k * (c->a[4] * (p[1] - m[1])
			- c->a[8] * (p[2] - m[2])) / 8192 * 1732
		/ YAS_MAG_VCORE / 10;
	return flowon | flowoff;
}

static int yas_get_position(void)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	return driver.position;
}

static int yas_set_position(int position)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (position < 0 || 7 < position)
		return YAS_ERROR_ARG;
	if (position == YAS537_MAG_NOTRANS_POSITION)
		driver.transform = NULL;
	else
		driver.transform = YAS537_TRANSFORMATION[position];
	driver.position = position;
	return YAS_NO_ERROR;
}

static int yas_measure(struct yas_data *data, int num, int *ouflow)
{
	int32_t xyz_tmp[3];
	int i, busy, rt;
	uint16_t xy1y2[3], t;
	uint32_t tm;
	*ouflow = 0;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (data == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (driver.cbk.current_time == NULL)
		driver.current_time += (uint32_t)driver.delay;
	if (num == 0)
		return 0;
	if (!driver.enable)
		return 0;
	if (driver.measure_state == YAS537_MAG_STATE_INIT_COIL) {
		tm = curtime();
		if (YAS537_MAG_INITCOIL_TIMEOUT
				<= tm - driver.invalid_data_time) {
			driver.invalid_data_time = tm;
			rt = reset_yas537();
			if (rt < 0)
				return rt;
			rt = rcoil_and_record_data(driver.flowflag, xy1y2,
					ouflow);
			if (rt < 0)
				return rt;
			if (!*ouflow) {
				driver.flowflag = 0;
				driver.invalid_data = 0;
				driver.measure_state = YAS537_MAG_STATE_NORMAL;
			}
			if (cont_start_yas537() < 0)
				return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	}
	if (read_yas537(&busy, &t, xy1y2, ouflow, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	sensitivity_correction(xy1y2, t);
	xy1y2_to_xyz(xy1y2, data->xyz.v);
	if (driver.transform != NULL) {
		for (i = 0; i < 3; i++) {
			xyz_tmp[i] = driver.transform[i*3] * data->xyz.v[0]
				+ driver.transform[i*3+1] * data->xyz.v[1]
				+ driver.transform[i*3+2] * data->xyz.v[2];
		}
		set_vector(data->xyz.v, xyz_tmp);
	}
	apply_matrix(&data->xyz, &driver.static_matrix);
	for (i = 0; i < 3; i++) {
		data->xyz.v[i] -= data->xyz.v[i] % 10;
		if (*ouflow & (1<<(i*2)))
			data->xyz.v[i] += 1; /* set overflow */
		if (*ouflow & (1<<(i*2+1)))
			data->xyz.v[i] += 2; /* set underflow */
	}
	tm = curtime();
	data->type = YAS_TYPE_MAG;
	if (driver.cbk.current_time)
		data->timestamp = tm;
	else
		data->timestamp = 0;
	data->accuracy = 0;
	if (busy)
		return YAS_ERROR_BUSY;
	if (*ouflow || invalid_magnetic_field(xy1y2, driver.last_after_rcoil)) {
		if (!driver.invalid_data) {
			driver.invalid_data_time = tm;
			driver.invalid_data = 1;
		}
		if (*ouflow)
			driver.flowflag = 1;
		driver.measure_state = YAS537_MAG_STATE_INIT_COIL;
		for (i = 0; i < 3; i++) {
			if (!*ouflow)
				data->xyz.v[i] += 3;
		}
	}
	return 1;
}

static int yas_measure_wrap(struct yas_data *data, int num)
{
	int ouflow;
	return yas_measure(data, num, &ouflow);
}

static int yas_get_delay(void)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	return driver.delay;
}

static int yas_set_delay(int delay)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (delay < 0)
		return YAS_ERROR_ARG;
	driver.delay = delay;
	if (!driver.enable)
		return YAS_NO_ERROR;
	yas_set_enable(0);
	yas_set_enable_wrap(1, 0);
	return YAS_NO_ERROR;
}

static int yas_get_enable(void)
{
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	return driver.enable;
}

static int yas_set_enable_wrap(int enable, int rcoil)
{
	int rt;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	enable = !!enable;
	if (driver.enable == enable)
		return YAS_NO_ERROR;
	if (enable) {
		if (yas_open() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		rt = reset_yas537();
		if (rt < 0) {
			driver.cbk.device_close(YAS_TYPE_MAG);
			return rt;
		}
		if (rcoil) {
			rt = enable_rcoil();
			if (rt < 0) {
				driver.cbk.device_close(YAS_TYPE_MAG);
				return rt;
			}
			if (rt) {
				driver.flowflag = 0;
				driver.invalid_data = 0;
				driver.measure_state = YAS537_MAG_STATE_NORMAL;
			} else {
				driver.invalid_data_time = curtime();
				driver.flowflag = 1;
				driver.invalid_data = 1;
				driver.measure_state
					= YAS537_MAG_STATE_INIT_COIL;
			}
		}
		if (cont_start_yas537() < 0) {
			driver.cbk.device_close(YAS_TYPE_MAG);
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
	} else {
		yas_single_write(YAS537_REG_SRSTR, 0x02);
		driver.cbk.device_close(YAS_TYPE_MAG);
	}
	driver.enable = enable;
	return YAS_NO_ERROR;
}

static int yas_set_enable(int enable)
{
	return yas_set_enable_wrap(enable, 1);
}

static int yas_ext(int32_t cmd, void *p)
{
	struct yas537_self_test_result *r;
	struct yas_data data;
	int32_t *xyz;
	int16_t *ouflow, *m;
	int8_t average, *hard_offset;
	int rt, i, enable, overflow, busy, position;
	uint16_t xy1y2[3], t;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (p == NULL)
		return YAS_ERROR_ARG;
	switch (cmd) {
	case YAS537_SELF_TEST:
		r = (struct yas537_self_test_result *) p;
		r->id = driver.dev_id;
		enable = driver.enable;
		if (!enable) {
			if (yas_open() < 0)
				return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		rt = reset_yas537();
		if (rt < 0)
			goto self_test_exit;
		rt = enable_rcoil();
		if (rt < 0)
			goto self_test_exit;
		rt = single_read(0, 0, &busy, &t, xy1y2, &overflow, 1);
		if (rt < 0)
			goto self_test_exit;
		if (busy) {
			rt = YAS_ERROR_BUSY;
			goto self_test_exit;
		}
		sensitivity_correction(xy1y2, t);
		xy1y2_to_xyz(xy1y2, r->xyz);
		for (i = 0; i < 3; i++)
			r->xyz[i] = r->xyz[i] / 1000;
		if (overflow & YAS_OVERFLOW) {
			rt = YAS_ERROR_OVERFLOW;
			goto self_test_exit;
		}
		if (overflow & YAS_UNDERFLOW) {
			rt = YAS_ERROR_UNDERFLOW;
			goto self_test_exit;
		}
		if (r->xyz[0] == 0 && r->xyz[1] == 0 && r->xyz[2] == 0) {
			rt = YAS_ERROR_DIRCALC;
			goto self_test_exit;
		}
		r->dir = 99;
		rt = sensitivity_measuremnet(&r->sx, &r->sy);
		if (rt < 0)
			goto self_test_exit;
		if (rt & YAS_OVERFLOW) {
			rt = YAS_ERROR_OVERFLOW;
			goto self_test_exit;
		}
		if (rt & YAS_UNDERFLOW) {
			rt = YAS_ERROR_UNDERFLOW;
			goto self_test_exit;
		}
		rt = YAS_NO_ERROR;
self_test_exit:
		if (enable)
			cont_start_yas537();
		else
			driver.cbk.device_close(YAS_TYPE_MAG);
		return rt;
	case YAS537_SELF_TEST_NOISE:
		xyz = (int32_t *) p;
		enable = driver.enable;
		if (!enable) {
			if (driver.noise_rcoil_flag)
				rt = yas_set_enable_wrap(1, 1);
			else
				rt = yas_set_enable_wrap(1, 0);
			if (rt < 0)
				return rt;
			driver.noise_rcoil_flag = 0;
		}
		position = yas_get_position();
		yas_set_position(YAS537_MAG_NOTRANS_POSITION);
		rt = yas_measure(&data, 1, &overflow);
		yas_set_position(position);
		if (rt < 0) {
			if (!enable)
				yas_set_enable(0);
			return rt;
		}
		for (i = 0; i < 3; i++)
			xyz[i] = data.xyz.v[i] / 300;
		if (!enable)
			yas_set_enable(0);
		return YAS_NO_ERROR;
	case YAS537_GET_LAST_RAWDATA:
		for (i = 0; i < 4; i++)
			((uint16_t *) p)[i] = driver.last_raw[i];
		return YAS_NO_ERROR;
	case YAS537_GET_AVERAGE_SAMPLE:
		*(int8_t *) p = driver.average;
		return YAS_NO_ERROR;
	case YAS537_SET_AVERAGE_SAMPLE:
		average = *(int8_t *) p;
		if (average < YAS537_MAG_AVERAGE_8
				|| YAS537_MAG_AVERAGE_256 < average)
			return YAS_ERROR_ARG;
		driver.average = average;
		if (!driver.enable)
			return YAS_NO_ERROR;
		yas_set_enable(0);
		yas_set_enable_wrap(1, 0);
		return YAS_NO_ERROR;
	case YAS537_GET_HW_OFFSET:
		hard_offset = (int8_t *) p;
		for (i = 0; i < 3; i++)
			hard_offset[i] = driver.hard_offset[i];
		return YAS_NO_ERROR;
	case YAS537_GET_STATIC_MATRIX:
		m = (int16_t *) p;
		for (i = 0; i < 9; i++)
			m[i] = driver.static_matrix.m[i];
		return YAS_NO_ERROR;
	case YAS537_SET_STATIC_MATRIX:
		m = (int16_t *) p;
		for (i = 0; i < 9; i++)
			driver.static_matrix.m[i] = m[i];
		return YAS_NO_ERROR;
	case YAS537_GET_OUFLOW_THRESH:
		ouflow = (int16_t *) p;
		for (i = 0; i < 3; i++) {
			ouflow[i] = driver.overflow[i];
			ouflow[i+3] = driver.underflow[i];
		}
		return YAS_NO_ERROR;
	default:
		break;
	}
	return YAS_ERROR_ARG;
}

static int yas_init(void)
{
	int16_t cxy1y2[3];
	struct yas_cal *c = &driver.cal;
	int32_t of[3], efxy1y2[3];
	uint8_t data[17];
	int i, cal_valid = 0;
	if (driver.initialized)
		return YAS_ERROR_INITIALIZE;
	if (yas_open() < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_read(YAS537_REG_DIDR, data, 1) < 0) {
		driver.cbk.device_close(YAS_TYPE_MAG);
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	driver.dev_id = data[0];
	if (driver.dev_id != YAS537_DEVICE_ID) {
		driver.cbk.device_close(YAS_TYPE_MAG);
		return YAS_ERROR_CHIP_ID;
	}
	if (yas_read(YAS537_REG_CALR, data, 17) < 0) {
		driver.cbk.device_close(YAS_TYPE_MAG);
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	c->ver = data[16] >> 6;
	for (i = 0; i < 17; i++) {
		if (i < 16 && data[i] != 0)
			cal_valid = 1;
		if (i == 16 && (data[i] & 0x3f) != 0)
			cal_valid = 1;
	}
	if (!cal_valid) {
		driver.cbk.device_close(YAS_TYPE_MAG);
		return YAS_ERROR_CALREG;
	}
	if (c->ver != 1 && c->ver != 3) {
		driver.cbk.device_close(YAS_TYPE_MAG);
		return YAS_ERROR_CALREG;
	}
	driver.cbk.device_close(YAS_TYPE_MAG);

	cxy1y2[0] = ((data[0]<<1) | (data[1]>>7)) - 256;
	cxy1y2[1] = (((data[1]<<2)&0x1fc) | (data[2]>>6)) - 256;
	cxy1y2[2] = (((data[2]<<3)&0x1f8) | (data[3]>>5)) - 256;
	c->a[0] = 128;
	c->a[1] = (((data[3]<<2)&0x7c) | (data[4]>>6)) - 64;
	c->a[2] = (((data[4]<<1)&0x7e) | (data[5]>>7)) - 64;
	c->a[3] = (((data[5]<<1)&0xfe) | (data[6]>>7)) - 128;
	c->a[4] = (((data[6]<<2)&0x1fc) | (data[7]>>6)) - 112;
	c->a[5] = (((data[7]<<1)&0x7e) | (data[8]>>7)) - 64;
	c->a[6] = (((data[8]<<1)&0xfe) | (data[9]>>7)) - 128;
	c->a[7] = (data[9]&0x7f) - 64;
	c->a[8] = (((data[10]<<1)&0x1fe) | (data[11]>>7)) - 112;
	c->k = data[11]&0x7f;
	for (i = 0; i < 3; i++)
		efxy1y2[i] = 8000 - (int32_t)ABS(cxy1y2[i]) * 225 / 16;
	of[0] = c->k * (c->a[0] * efxy1y2[0] - ABS(c->a[1]) * efxy1y2[1]
			- ABS(c->a[2]) * efxy1y2[2]) / 8192;
	of[1] = c->k * (-ABS(c->a[3]) * efxy1y2[0] + c->a[4] * efxy1y2[1]
			- ABS(c->a[5]) * efxy1y2[2]) / 8192;
	of[2] = c->k * (-ABS(c->a[6]) * efxy1y2[0] - ABS(c->a[7]) * efxy1y2[1]
			+ c->a[8] * efxy1y2[2]) / 8192;
	for (i = 0; i < 3; i++) {
		driver.hard_offset[i] = data[i+12];
		if (YAS537_DATA_OVERFLOW < 8192 + of[i])
			driver.overflow[i] = YAS537_DATA_OVERFLOW;
		else
			driver.overflow[i] = (int16_t) (8192 + of[i]);
		if (8192 - of[i] < YAS537_DATA_UNDERFLOW)
			driver.underflow[i] = YAS537_DATA_UNDERFLOW;
		else
			driver.underflow[i] = (int16_t) (8192 - of[i]);
	}

	driver.measure_state = YAS537_MAG_STATE_NORMAL;
	if (driver.cbk.current_time)
		driver.current_time =  driver.cbk.current_time();
	else
		driver.current_time = 0;
	driver.invalid_data = 0;
	driver.invalid_data_time = driver.current_time;
	driver.position = YAS537_MAG_NOTRANS_POSITION;
	driver.delay = YAS_DEFAULT_SENSOR_DELAY;
	driver.enable = 0;
	driver.transform = NULL;
	driver.average = YAS537_MAG_AVERAGE_32;
	driver.noise_rcoil_flag = 1;
	driver.flowflag = 0;
	for (i = 0; i < 3; i++)
		driver.last_after_rcoil[i] = 0;
	for (i = 0; i < 4; i++)
		driver.last_raw[i] = 0;
	driver.static_matrix = no_conversion;
	driver.initialized = 1;
	return YAS_NO_ERROR;
}

static int yas_term(void)
{
	int rt;
	if (!driver.initialized)
		return YAS_ERROR_INITIALIZE;
	rt = yas_set_enable(0);
	driver.initialized = 0;
	return rt;
}

int yas_mag_driver_init_537(struct yas_mag_driver *f)
{
	if (f == NULL || f->callback.device_open == NULL
			|| f->callback.device_close == NULL
			|| f->callback.device_read == NULL
			|| f->callback.device_write == NULL
			|| f->callback.usleep == NULL
	   )
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->measure = yas_measure_wrap;
	f->ext = yas_ext;
	driver.cbk = f->callback;
	yas_term();
	return YAS_NO_ERROR;
}
/***************yas537_mag_drv   end!***************/


static int yas_device_open(int32_t type)
{
	return 0;
}

static int yas_device_close(int32_t type)
{
	return 0;
}

static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf,
		int len)
{
	uint8_t tmp[2];
	int err;
	if (sizeof(tmp) - 1 < len)
		return -EPERM;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	err = i2c_master_send(this_client, tmp, len + 1);
	if (unlikely(err < 0)) {
		printk(KERN_ERR TAGE "I2C send error: %d\n", err);
		return err;
	}
	return 0;
}

static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len)
{
	struct i2c_msg msg[2];
	int err;
	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	err = i2c_transfer(this_client->adapter, msg, 2);
	if (err != 2) {
		printk(KERN_ERR TAGE
				"i2c_transfer() read error: "
				"slave_addr=%02x, reg_addr=%02x, err=%d\n",
				this_client->addr, addr, err);
		return err;
	}
	return 0;
}

static void yas_usleep(int us)
{
	usleep_range(us, us + 1000);
}

static uint32_t yas_current_time(void)
{
	return jiffies_to_msecs(jiffies);
}

static int yas_enable(struct yas_state *st)
{   
	if (!atomic_cmpxchg(&st->enable, 0, 1)) {
		mutex_lock(&st->lock);
		st->mag.set_enable(1);
		mutex_unlock(&st->lock);
		schedule_delayed_work(&st->work, 0);
	}
	return 0;
}

static int yas_disable(struct yas_state *st)
{
	if (atomic_cmpxchg(&st->enable, 1, 0)) {
		cancel_delayed_work_sync(&st->work);
		mutex_lock(&st->lock);
		st->mag.set_enable(0);
		mutex_unlock(&st->lock);
	}
	return 0;
}

/* Sysfs interface */

static ssize_t yas_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.get_position();
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%d\n", ret);
}

static ssize_t yas_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, position;
	ret = sscanf(buf, "%d\n", &position);
	if (ret != 1)
		return -EFAULT;
	mutex_lock(&st->lock);
	ret = st->mag.set_position(position);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}
static int yas_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{   
    
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (enable)
		yas_enable(st);
	else
		yas_disable(st);
	return 0;
}

static int yas_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_ms)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	if (delay_ms <= 0)
		delay_ms = 10;
	mutex_lock(&st->lock);
	if (st->mag.set_delay(delay_ms) == YAS_NO_ERROR)
		{
		st->poll_delay = delay_ms;
		}
	mutex_unlock(&st->lock);

	return 0;

}


static ssize_t yas_hard_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t hard_offset[3];
	int ret;
	mutex_lock(&st->lock);

	ret = st->mag.ext(YAS537_GET_HW_OFFSET, hard_offset);

	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%d %d %d\n",
			hard_offset[0], hard_offset[1], hard_offset[2]);
}




static ssize_t yas_static_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t m[9];
	int ret;
	mutex_lock(&st->lock);

	ret = st->mag.ext(YAS537_GET_STATIC_MATRIX, m);

	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n", m[0], m[1], m[2],
			m[3], m[4], m[5], m[6], m[7], m[8]);
}

static ssize_t yas_static_matrix_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t m[9];
	int ret;
	ret = sscanf(buf, "%hd %hd %hd %hd %hd %hd %hd %hd %hd\n", &m[0],
			&m[1], &m[2], &m[3], &m[4], &m[5], &m[6], &m[7],
			&m[8]);
	if (ret != 9)
		return -EFAULT;
	mutex_lock(&st->lock);

	ret = st->mag.ext(YAS537_SET_STATIC_MATRIX, m);

	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_self_test_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz_raw[3];
	int ret;
	mutex_lock(&st->lock);

	ret = st->mag.ext(YAS537_SELF_TEST_NOISE, xyz_raw);

	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", xyz_raw[0], xyz_raw[1], xyz_raw[2]);
}



static ssize_t yas_mag_average_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t mag_average_sample;
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return snprintf(buf, PAGE_SIZE, "%d\n", mag_average_sample);
}

static ssize_t yas_mag_average_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp;
	int8_t mag_average_sample;
	int ret;
	ret = sscanf(buf, "%d\n", &tmp);
	if (ret != 1)
		return -EFAULT;
	mag_average_sample = (int8_t)tmp;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}


static ssize_t yas_ouflow_thresh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int16_t thresh[6];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_OUFLOW_THRESH, thresh);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d %d %d %d\n", thresh[0], thresh[1],
			thresh[2], thresh[3], thresh[4], thresh[5]);
}

static ssize_t yas_data_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t last[3], i;
	mutex_lock(&st->lock);
	for (i = 0; i < 3; i++)
		last[i] = st->compass_data[i];
	mutex_unlock(&st->lock);
	return snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%d %d %d\n",
					last[0], last[1], last[2]);
}


static DEVICE_ATTR(data, S_IRUGO, yas_data_show, NULL);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR, yas_position_show,
		yas_position_store);		
static DEVICE_ATTR(hard_offset, S_IRUGO, yas_hard_offset_show, NULL);

static DEVICE_ATTR(static_matrix, S_IRUGO|S_IWUSR,
		yas_static_matrix_show, yas_static_matrix_store);
static DEVICE_ATTR(self_test_noise, S_IRUGO, yas_self_test_noise_show, NULL);
static DEVICE_ATTR(mag_average_sample, S_IRUGO|S_IWUSR,
		yas_mag_average_sample_show, yas_mag_average_sample_store);
static DEVICE_ATTR(ouflow_thresh, S_IRUGO, yas_ouflow_thresh_show, NULL);

static struct attribute *yas_attributes[] = {

	&dev_attr_data.attr,
	&dev_attr_position.attr,
	&dev_attr_hard_offset.attr,

	&dev_attr_static_matrix.attr,
	&dev_attr_self_test_noise.attr,
	&dev_attr_mag_average_sample.attr,
	&dev_attr_ouflow_thresh.attr,

	NULL
};

static struct attribute_group yas_attribute_group = {
	.attrs = yas_attributes
};

static void yas_work_func(struct work_struct *work)
{
	struct yas_state *st
		= container_of((struct delayed_work *)work,
			struct yas_state, work);
	struct yas_data mag[1];
	int32_t poll_delay;
	uint32_t time_before, time_after;
	int ret, i;

	time_before = yas_current_time();
	mutex_lock(&st->lock);
	ret = st->mag.measure(mag, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++)
			st->compass_data[i] = mag[0].xyz.v[i];
	    /*laizhilong add start*/
		st->data_valid=0x7f;
		/*laizhilong add end*/
	}
	poll_delay = st->poll_delay;
	mutex_unlock(&st->lock);
	if (ret == 1) {
		/* report magnetic data in [nT] */
		input_report_abs(st->input_dev, ABS_X, mag[0].xyz.v[0]);
		input_report_abs(st->input_dev, ABS_Y, mag[0].xyz.v[1]);
		input_report_abs(st->input_dev, ABS_Z, mag[0].xyz.v[2]);
		input_sync(st->input_dev);
	}
	time_after = yas_current_time();
	poll_delay = poll_delay - (time_after - time_before);
	if (poll_delay <= 0)
		poll_delay = 1;
	schedule_delayed_work(&st->work, msecs_to_jiffies(poll_delay));
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas_early_suspend(struct early_suspend *h)
{
	struct yas_state *st = container_of(h, struct yas_state, sus);
	if (atomic_read(&st->enable)) {
		cancel_delayed_work_sync(&st->work);
		st->mag.set_enable(0);
	}
}


static void yas_late_resume(struct early_suspend *h)
{
	struct yas_state *st = container_of(h, struct yas_state, sus);
	if (atomic_read(&st->enable)) {
		st->mag.set_enable(1);
		schedule_delayed_work(&st->work, 0);
	}
}
#endif

/*laizhilong add start*/
static int yas_mag_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int yas_mag_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver yas_mag_pd_driver = {
	.probe  = yas_mag_pd_probe,
	.remove = yas_mag_pd_remove,    
	.driver = {
		.name  = "msensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device yas_mag_pd_device_537 = {
    .name = "msensor",
    .id   = -1,
};

static int criteria_check(void *p)
{
	struct yas537_self_test_result *r
		= (struct yas537_self_test_result *) p;
	return r->id == 0x07 &&
		0 <= r->dir && r->dir <= 359 &&
		24 <= r->sx && 31 <= r->sy &&
		-1700 <= r->xyz[0] && r->xyz[0] <= 1700 &&
		-1700 <= r->xyz[1] && r->xyz[1] <= 1700 &&
		-1700 <= r->xyz[2] && r->xyz[2] <= 1700;
	return 0;
}

static ssize_t yas_mag_shipment_test(struct device_driver *ddri, char *buf)
{
	struct yas_state *st = NULL;
	int ret;
	struct yas537_self_test_result r;

	if (this_client)
		st = i2c_get_clientdata(this_client);
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SELF_TEST, &r);

	mutex_unlock(&st->lock);
	if (ret) {
		ret = -1;
		printk(KERN_ERR "%s ret %d\n", __func__, ret);
	} else {
		if (criteria_check(&r))
			ret = 0x0f;
		else
			ret = -1;
	}

	YAS_REM_PRINTK("%s x %d y %d z %d\n", __func__, r.xyz[0], r.xyz[1],
			r.xyz[2]);
	return sprintf(buf, "%d\n", ret);
}
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	/*char sensordata[SENSOR_DATA_SIZE];*/
	char strbuf[0x20];
	struct yas_state *st=NULL;
	int ret,i;
	struct yas_data mag[1];
	if(this_client)
	    st=i2c_get_clientdata(this_client);

    if(st)
    {
		if (!atomic_read(&st->enable))
		{
			st->mag.set_enable(1);
        }

		if(!st->data_valid)
		{
			mutex_lock(&st->lock);
			ret = st->mag.measure(mag, 1);
			if (ret == 1) {
				for (i = 0; i < 3; i++)
					st->compass_data[i] = mag[0].xyz.v[i];
			}
			mutex_unlock(&st->lock);
		}
	}
	
	mutex_lock(&st->lock);
	sprintf(strbuf, "%d %d %d\n", st->compass_data[0],st->compass_data[1],st->compass_data[2]);
	st->data_valid=0x00;
	mutex_unlock(&st->lock);

	return sprintf(buf, "%s\n", strbuf);
}

static DRIVER_ATTR(selftest,S_IRUGO | S_IWUSR, yas_mag_shipment_test, NULL);
static DRIVER_ATTR(data,  S_IRUGO, show_sensordata_value, NULL);

static struct driver_attribute *yas_mag_attr_list[] = {
	&driver_attr_selftest,
	&driver_attr_data,
};

static int yas_mag_create_attr(struct device_driver *driver) 
{
	int i;
	for (i = 0; i < ARRAY_SIZE(yas_mag_attr_list); i++)
		if (driver_create_file(driver, yas_mag_attr_list[i]))
			goto error;
	return 0;

error:
	for (i--; i >= 0; i--)
		driver_remove_file(driver, yas_mag_attr_list[i]);
	printk(KERN_ERR "%s:Unable to create interface\n", __func__);
	return -1;
}

#ifdef CONFIG_OF
/*shuaofeng add start*/
static int yas_compass_parse_dt_1(struct device *dev, struct yas_state *st)
{
    struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	rc = of_property_read_u32(np, "iic-address", &temp_val);
	if (rc) {
		printk(KERN_ERR TAGE "Unable to read iic-address\n");
		st->i2c_add=-2;
	} else {
		st->i2c_add=temp_val;
	}

	return 0;
}

static int yas_compass_parse_dt_2(struct device *dev, struct yas_state *st)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "ak,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read yas,layout\n");
		return rc;
	} else {
		rc  = st->mag.set_position(temp_val);
	}
    
	return 0;
}
/*shuaofeng add end*/

#else
static int yas_compass_parse_dt_1(struct device *dev, struct yas_state *st)
{
	return -EINVAL;
}

static int yas_compass_parse_dt_2(struct device *dev, struct yas_state *st)
{
	return -EINVAL;
}

#endif /* !CONFIG_OF */

static int yas_compass_power_init(struct yas_state *data, bool on)
{
	int rc;
	struct i2c_client * i2c=this_client;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				YAS_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				YAS_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			printk(KERN_ERR TAGE
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				YAS_VDD_MIN_UV, YAS_VDD_MAX_UV);
			if (rc) {
				printk(KERN_ERR TAGE
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		rc = regulator_enable(data->vdd);
		if(rc)
		{
			rc = PTR_ERR(data->vdd);
			printk(KERN_ERR TAGE
				"Regulator get failed vdd rc=%d\n", rc);
			goto reg_vdd_set;

		}

		data->vio = regulator_get(&i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			printk(KERN_ERR TAGE
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				YAS_VIO_MIN_UV, YAS_VIO_MAX_UV);
			if (rc) {
				printk(KERN_ERR TAGE
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}

		rc = regulator_enable(data->vio);
		if(rc)
		{
			rc = PTR_ERR(data->vio);
			printk(KERN_ERR TAGE
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vio_set;
		
		}

	}

	return 0;

reg_vio_set:
	if (regulator_count_voltages(data->vio) > 0)
		regulator_set_voltage(data->vio, 0, YAS_VIO_MAX_UV);
reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, YAS_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

/*laizhilong add end*/
static int yas_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{ 
    
	struct yas_state *st = NULL;
	struct input_dev *input_dev = NULL;
	int ret, i;
	int flag_inputdev_unregistered = 0;
    printk(KERN_INFO TAGI "start probing 1.\n");
	this_client = i2c;
	input_dev = input_allocate_device();
	printk(KERN_INFO TAGI "start probing 2.\n");
	if (input_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st = kzalloc(sizeof(struct yas_state), GFP_KERNEL);
	printk(KERN_INFO TAGI "start probing 3.\n");
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_free_input_device;
	}
	i2c_set_clientdata(i2c, st);

	input_dev->name = YAS_INPUT_DEVICE_NAME;
	input_dev->dev.parent = &i2c->dev;
	input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);
    printk(KERN_INFO TAGI "start probing 4.\n");
	input_set_drvdata(input_dev, st);
	printk(KERN_INFO TAGI "start probing 5.\n");
	atomic_set(&st->enable, 0);
	st->input_dev = input_dev;
	st->poll_delay = YAS_DEFAULT_SENSOR_DELAY;
	st->mag.callback.device_open = yas_device_open;
	st->mag.callback.device_close = yas_device_close;
	st->mag.callback.device_write = yas_device_write;
	st->mag.callback.device_read = yas_device_read;
	st->mag.callback.usleep = yas_usleep;
	st->mag.callback.current_time = yas_current_time;
	INIT_DELAYED_WORK(&st->work, yas_work_func);
	printk(KERN_INFO TAGI "start probing 6.\n");
	mutex_init(&st->lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->sus.suspend = yas_early_suspend;
	st->sus.resume = yas_late_resume;
	register_early_suspend(&st->sus);
#endif
	for (i = 0; i < 3; i++)
		st->compass_data[i] = 0;

	ret = input_register_device(input_dev);
	printk(KERN_INFO TAGI "start probing 7.\n");
	if (ret)
		goto error_free;

	st->cdev = sensors_cdev;
	st->cdev.sensors_enable = yas_enable_set;
	st->cdev.sensors_poll_delay = yas_poll_delay_set;

	ret = sensors_classdev_register(&i2c->dev, &st->cdev);
	printk(KERN_INFO TAGI "start probing 8.\n");
	if (ret) {
		printk(KERN_ERR TAGE "class device create failed: %d\n", ret);
		goto err_unregister_input_device;
	}

	st->dev = st->cdev.dev;

	ret = sysfs_create_group(&st->dev->kobj, &yas_attribute_group);
	printk(KERN_INFO TAGI "start probing 9.\n");
	if (ret)
		goto error_classdev_unregister;
	ret = yas_mag_driver_init_537(&st->mag);
	printk(KERN_INFO TAGI "start probing 10.\n");
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}

	ret = yas_compass_parse_dt_1(&i2c->dev, st);       //shuaofeng add for i2c address transform
	printk(KERN_INFO TAGI "start probing 11.\n");
	
	if(0<=st->i2c_add&& 0xff>=st->i2c_add)
	{
			printk(KERN_INFO TAGI "%s change the iic address from %x to %x\n",
			__func__, i2c->addr, st->i2c_add);
			i2c->addr=st->i2c_add;
	}		
	
	ret = yas_compass_power_init(st, true);
	printk(KERN_INFO TAGI "start probing 12.\n");
	if (ret/*err < 0*/)
		goto error_remove_sysfs;

	ret = st->mag.init();
	printk(KERN_INFO TAGI "start probing 13.\n");
	if (ret < 0) {
		ret = -EFAULT;
		goto err_compass_pwr_init;
	}

    ret = yas_compass_parse_dt_2(&i2c->dev, st);       //shuaofeng add for layout setting
	printk(KERN_INFO TAGI "start probing 14.\n");

	/*laizhilong add start*/
    if((ret = platform_driver_register(&yas_mag_pd_driver)))
	{
		printk(KERN_ERR "%s failed to register yas_mag_driver err %d\n", __func__, ret);
		goto err_compass_pwr_init/*err_free_irq2*/;
	}
    printk(KERN_INFO TAGI "start probing 15.\n");
	
    if((ret = platform_device_register(&yas_mag_pd_device_537)))
    {
		printk(KERN_ERR "%s failed to register yas_mag_device err %d\n", __func__, ret);
		goto err_free_driver;
    }
    printk(KERN_INFO TAGI "start probing 16.\n");
	
	if((ret = yas_mag_create_attr(&yas_mag_pd_driver.driver)))
	{
		printk("%s lis3dh create attribute err = %d\n", __func__, ret);
		goto err_free_device;
	}
	printk(KERN_INFO TAGI "start probing 17.\n");
	/*laizhilong add end*/
	return 0;

/*laizhilong add start*/
err_free_device:
	platform_device_unregister(&yas_mag_pd_device_537);
err_free_driver:
	platform_driver_unregister(&yas_mag_pd_driver);
err_compass_pwr_init:
	yas_compass_power_init(st, false);
/*laizhilong add end*/
error_remove_sysfs:
	sysfs_remove_group(&st->dev->kobj, &yas_attribute_group);
error_classdev_unregister:
	 sensors_classdev_unregister(&st->cdev);
err_unregister_input_device:
	 input_unregister_device(input_dev);
	 flag_inputdev_unregistered = 1;
error_free:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st->sus);
#endif
	kfree(st);
error_free_input_device:
	if(flag_inputdev_unregistered != 1)
	    input_free_device(input_dev);
	printk("%s inputdev flag_inputdev_unregistered= %d\n", __func__, flag_inputdev_unregistered);
error_ret:
	i2c_set_clientdata(i2c, NULL);
	this_client = NULL;
	return ret;
}

static int yas_remove(struct i2c_client *i2c)
{
	struct yas_state *st = i2c_get_clientdata(i2c);
	if (st != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->sus);
#endif
		yas_disable(st);
		st->mag.term();
		sysfs_remove_group(&st->dev->kobj,
				&yas_attribute_group);
		input_unregister_device(st->input_dev);
		input_free_device(st->input_dev);
		device_unregister(st->dev);
		class_destroy(st->class);
		kfree(st);
		this_client = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int yas_suspend(struct device *dev)
{
	struct i2c_client *i2c = dev_get_drvdata(dev);
	struct yas_state *st = i2c_get_clientdata(this_client);

	YAS_REM_PRINTK("%s i2c_client %p this_client %p\n", __func__, i2c, this_client);
	if(!st)
	{
		printk(KERN_ERR "%s null pointer\n", __func__);
		return 0;
	}

	if (atomic_read(&st->enable)) {
		cancel_delayed_work_sync(&st->work);
		st->mag.set_enable(0);
	}
	return 0;
}

static int yas_resume(struct device *dev)
{
	struct i2c_client *i2c = dev_get_drvdata(dev);
	struct yas_state *st = i2c_get_clientdata(this_client);
	YAS_REM_PRINTK("%s i2c_client %p this_client %p\n", __func__, i2c, this_client);
	if(!st)
	{
		printk(KERN_ERR "%s null pointer\n", __func__);
		return 0;
	}

	if (atomic_read(&st->enable)) {
		st->mag.set_enable(1);
		schedule_delayed_work(&st->work, 0);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(yas_pm_ops, yas_suspend, yas_resume);
#define YAS_PM_OPS (&yas_pm_ops)
#else
#define YAS_PM_OPS NULL
#endif

static const struct i2c_device_id yas_id[] = {
	{YAS_I2C_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);


static struct of_device_id yas_match_table[] = {
	{ .compatible = "yas537_mag", },
	{ },
};

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_I2C_NAME,
		.owner	= THIS_MODULE,
		.pm	= YAS_PM_OPS,
		.of_match_table = yas_match_table,
	},
	.probe		= yas_probe,
	.remove		= yas_remove,
	.id_table	= yas_id,
};
static int __init yas_driver_init(void)
{
	return i2c_add_driver(&yas_driver);
}

static void __exit yas_driver_exit(void)
{
	i2c_del_driver(&yas_driver);
}

module_init(yas_driver_init);
module_exit(yas_driver_exit);

MODULE_DESCRIPTION("Yamaha Magnetometer I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.6.9.1210");
