/*
 * Copyright (c) 2013-2014 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/sensors.h>
#include <linux/delay.h>

#define USB_REM_PRINTK(fmt,args...) //


#define YAS_NO_ERROR			(0) /*!< Succeed */
#define YAS_ERROR_ARG			(-1) /*!< Invalid argument */
#define YAS_ERROR_INITIALIZE		(-2) /*!< Invalid initialization status
					      */
#define YAS_ERROR_BUSY			(-3) /*!< Sensor is busy */
#define YAS_ERROR_DEVICE_COMMUNICATION	(-4) /*!< Device communication error */
#define YAS_ERROR_CHIP_ID		(-5) /*!< Invalid chip id */
#define YAS_ERROR_CALREG		(-6) /*!< Invalid CAL register */
#define YAS_ERROR_OVERFLOW		(-7) /*!< Overflow occured */
#define YAS_ERROR_UNDERFLOW		(-8) /*!< Underflow occured */
#define YAS_ERROR_DIRCALC		(-9) /*!< Direction calcuration error */
#define YAS_ERROR_ERROR			(-128) /*!< other error */

#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80


#define YAS532_REG_DEVID		(0x80)
#define YAS532_REG_RCOILR		(0x81)
#define YAS532_REG_CMDR			(0x82)
#define YAS532_REG_CONFR		(0x83)
#define YAS532_REG_DLYR			(0x84)
#define YAS532_REG_OXR			(0x85)
#define YAS532_REG_OY1R			(0x86)
#define YAS532_REG_OY2R			(0x87)
#define YAS532_REG_TEST1R		(0x88)
#define YAS532_REG_TEST2R		(0x89)
#define YAS532_REG_CALR			(0x90)
#define YAS532_REG_DATAR		(0xB0)

#define YAS532_VERSION_AC_COEF_X	(850)
#define YAS532_VERSION_AC_COEF_Y1	(750)
#define YAS532_VERSION_AC_COEF_Y2	(750)
#define YAS532_DATA_CENTER		(4096)
#define YAS532_DATA_UNDERFLOW		(0)
#define YAS532_DATA_OVERFLOW		(8190)
#define YAS532_DEVICE_ID		(0x02)	/* YAS532 (MS-3R/3F) */
#define YAS532_TEMP20DEGREE_TYPICAL	(390)

#define YAS_X_OVERFLOW			(0x01)
#define YAS_X_UNDERFLOW			(0x02)
#define YAS_Y1_OVERFLOW			(0x04)
#define YAS_Y1_UNDERFLOW		(0x08)
#define YAS_Y2_OVERFLOW			(0x10)
#define YAS_Y2_UNDERFLOW		(0x20)
#define YAS_OVERFLOW	(YAS_X_OVERFLOW|YAS_Y1_OVERFLOW|YAS_Y2_OVERFLOW)
#define YAS_UNDERFLOW	(YAS_X_UNDERFLOW|YAS_Y1_UNDERFLOW|YAS_Y2_UNDERFLOW)

#define YAS532_MAG_STATE_NORMAL		(0)
#define YAS532_MAG_STATE_INIT_COIL	(1)
#define YAS532_MAG_STATE_MEASURE_OFFSET	(2)
#define YAS532_MAG_INITCOIL_TIMEOUT	(1000)	/* msec */
#define YAS532_MAG_TEMPERATURE_LOG	(10)
#define YAS532_MAG_NOTRANS_POSITION	(3)

#define YAS_TYPE_ACC_NONE		(0x00000001) /*!< No Acceleration */
#define YAS_TYPE_MAG_NONE		(0x00000002) /*!< No Magnetometer */
#define YAS_TYPE_GYRO_NONE		(0x00000004) /*!< No Gyroscope */
#define YAS_TYPE_A_ACC			(0x00000008) /*!< 3-axis Acceleration */
#define YAS_TYPE_M_MAG			(0x00000010) /*!< 3-axis Magnetometer */
#define YAS_TYPE_G_GYRO			(0x00000020) /*!< 3-axis Gyroscope */
#define YAS_TYPE_AM_ACC			(0x00100000) /*!< 6-axis (Acc+Mag)
						       Acceleration */
#define YAS_TYPE_AM_MAG			(0x00200000) /*!< 6-axis (Acc+Mag)
						       Magnetometer */
#define YAS_TYPE_AG_ACC			(0x01000000) /*!< 6-axis (Acc+Gyro)
						       Acceleration */
#define YAS_TYPE_AG_GYRO		(0x02000000) /*!< 6-axis (Acc+Gyro)
						       Gyroscope */
#define YAS_TYPE_AMG_ACC		(0x10000000) /*!< 9-axis (Acc+Gyro+Mag)
						       Acceleration */
#define YAS_TYPE_AMG_MAG		(0x20000000) /*!< 9-axis (Acc+Gyro+Mag)
						       Magnetometer */
#define YAS_TYPE_AMG_GYRO		(0x40000000) /*!< 9-axis (Acc+Gyro+Mag)*/


#define YAS_TYPE_MAG YAS_TYPE_M_MAG

/* ----------------------------------------------------------------------------
 *                      Extension Command Definition
 *--------------------------------------------------------------------------- */
/*! YAS532 extension command: self test */
#define YAS532_SELF_TEST		(0x00000001)
/*! YAS532 extension command: self test noise */
#define YAS532_SELF_TEST_NOISE		(0x00000002)
/*! YAS532 extension command: obtains the hardware offset */
#define YAS532_GET_HW_OFFSET		(0x00000003)
/*! YAS532 extension command: sets the hardware offset */
#define YAS532_SET_HW_OFFSET		(0x00000004)
/*! YAS532 extension command: obtains last raw data (x, y1, y2, t) */
#define YAS532_GET_LAST_RAWDATA		(0x00000006)

/*! YAS535 extension command: obtains last raw data (xy1y2[3] t xyz[3]) */
#define YAS535_GET_LAST_RAWDATA		(0x00000001)
/*! YAS535 extension command: self test for magnetometer */
#define YAS535_MAG_SELF_TEST		(0x00000002)
/*! YAS535 extension command: self test noise for magnetometer */
#define YAS535_MAG_SELF_TEST_NOISE	(0x00000003)
/*! YAS535 extension command: obtains the hardware offset */
#define YAS535_MAG_GET_HW_OFFSET	(0x00000004)
/*! YAS535 extension command: sets the hardware offset */
#define YAS535_MAG_SET_HW_OFFSET	(0x00000005)
/*! YAS535 extension command: obtains the average samples for magnetometer */
#define YAS535_MAG_GET_AVERAGE_SAMPLE	(0x00000006)
/*! YAS535 extension command: sets the average samples for magnetometer */
#define YAS535_MAG_SET_AVERAGE_SAMPLE	(0x00000007)
/*! YAS535 extension command: self test for accelerometer */
#define YAS535_ACC_SELF_TEST		(0x00010000)
/*! YAS535 extension command: obtains the average samples for accelerometer */
#define YAS535_ACC_GET_AVERAGE_SAMPLE	(0x00020000)
/*! YAS535 extension command: sets the average samples for accelerometer */
#define YAS535_ACC_SET_AVERAGE_SAMPLE	(0x00040000)

/*! YAS536 extension command: self test */
#define YAS536_SELF_TEST		(0x00000001)
/*! YAS536 extension command: obtains the hardware offset */
#define YAS536_GET_HW_OFFSET		(0x00000002)
/*! YAS536 extension command: obtains the average filter length */
#define YAS536_GET_AVERAGE_LEN		(0x00000004)
/*! YAS536 extension command: sets the average filter length */
#define YAS536_SET_AVERAGE_LEN		(0x00000005)
/*! YAS536 extension command: obtains last raw data (x, y1, y2, t) */
#define YAS536_GET_LAST_RAWDATA		(0x00000006)

/*! YAS537 extension command: self test */
#define YAS537_SELF_TEST		(0x00000001)
/*! YAS537 extension command: self test noise */
#define YAS537_SELF_TEST_NOISE		(0x00000002)
/*! YAS537 extension command: obtains last raw data (x, y1, y2, t) */
#define YAS537_GET_LAST_RAWDATA		(0x00000003)
/*! YAS537 extension command: obtains the average samples */
#define YAS537_GET_AVERAGE_SAMPLE	(0x00000004)
/*! YAS537 extension command: sets the average samples */
#define YAS537_SET_AVERAGE_SAMPLE	(0x00000005)
/*! YAS537 extension command: obtains the hardware offset */
#define YAS537_GET_HW_OFFSET		(0x00000006)

#define YAS_DEFAULT_SENSOR_DELAY		(50)


#define YAS532_DRIVER_NO_SLEEP (0)
#if YAS532_DRIVER_NO_SLEEP
#define YAS_MAG_MAX_BUSY_LOOP		(1000)
#endif

#define NUMBERBIT16		32768
#define YAS533_MAG_DEV_NAME "compass"


/*! Mangetic vdd in mV */
#define YAS_MAG_VCORE				(2600)

#define ID_ACCELEROMETER		0
#define ID_GEOMAGNETIC		1
#define ID_ORIENTATION		2
#define ID_LIGHT			3
#define ID_PROXIMITY		4
#define ID_GYROSCOPE		5
#define ID_PRESSURE			6
#define ID_TEMPERATURE                  (ID_PRESSURE + 1)
#define ID_GRAVITY                      (ID_PRESSURE + 2)
#define ID_LINEAR_ACCELERATION          (ID_PRESSURE + 3)
#define ID_ROTATION_VECTOR              (ID_PRESSURE + 4)
#define ID_RELATIVE_HUMIDITY            (ID_PRESSURE + 5)
#define ID_AMBIENT_TEMPERATURE          (ID_PRESSURE + 6)
#define ID_MAGNETIC_FIELD_UNCALIBRATED  (ID_PRESSURE + 7)
#define ID_GAME_ROTATION_VECTOR         (ID_PRESSURE + 8)
#define ID_GYROSCOPE_UNCALIBRATED       (ID_PRESSURE + 9)
#define ID_SIGNIFICANT_MOTION           (ID_PRESSURE + 10)
#define ID_STEP_DETECTOR                (ID_PRESSURE + 11)
#define ID_STEP_COUNTER                 (ID_PRESSURE + 12)
#define ID_GEOMAGNETIC_ROTATION_VECTOR  (ID_PRESSURE + 13)


#define set_vector(to, from) \
	{int _l; for (_l = 0; _l < 3; _l++) (to)[_l] = (from)[_l]; }
#define is_valid_offset(a) \
	(((a)[0] <= 31) && ((a)[1] <= 31) && ((a)[2] <= 31) \
		&& (-31 <= (a)[0]) && (-31 <= (a)[1]) && (-31 <= (a)[2]))

struct yas_vector {
	int32_t v[3]; /*!< vector data */
};

struct yas_data {
	int32_t type; /*!< Sensor type */
	struct yas_vector xyz; /*!< X, Y, Z measurement data of the sensor */
	uint32_t timestamp; /*!< Measurement time */
	uint8_t accuracy; /*!< Measurement data accuracy */
};

struct yas532_self_test_result {
	int32_t id;
	int8_t xy1y2[3];
	int32_t dir;
	int32_t sx, sy;
	int32_t xyz[3];
};


struct yas_cal_data {
	int8_t rxy1y2[3];
	uint8_t fxy1y2[3];
	int32_t cx, cy1, cy2;
	int32_t a2, a3, a4, a5, a6, a7, a8, a9, k;
};
#if 1 < YAS532_MAG_TEMPERATURE_LOG
struct yas_temperature_filter {
	uint16_t log[YAS532_MAG_TEMPERATURE_LOG];
	int num;
	int idx;
};
#endif
struct yas533_mag_sensor{
	struct i2c_client *client;
	struct sensors_classdev cdev_mag;
	struct sensors_classdev cdev_orien;
	struct mutex lock;
	struct input_dev *input_dev;
	struct workqueue_struct * mag_workqueue;
	struct delayed_work mag_poll_work;

	int initialized;
	struct yas_cal_data cal;
	/*struct yas_driver_callback cbk;*/
	int measure_state;
	int8_t hard_offset[3];
	int32_t coef[3];
	int overflow;
	uint32_t overflow_time;
	int position;
	int delay;
	int enable;
	uint8_t dev_id;
	const int8_t *transform;
#if 1 < YAS532_MAG_TEMPERATURE_LOG
	struct yas_temperature_filter t;
#endif
	uint32_t current_time;
	uint16_t last_raw[4];
#if YAS532_DRIVER_NO_SLEEP
	int start_flag;
	int wait_flag;
#endif
};

static const int yas532_version_ac_coef[] = {YAS532_VERSION_AC_COEF_X,
	YAS532_VERSION_AC_COEF_Y1, YAS532_VERSION_AC_COEF_Y2};
static const int8_t INVALID_OFFSET[] = {0x7f, 0x7f, 0x7f};
static const int8_t YAS532_TRANSFORMATION[][9] = {
	{ 0,  1,  0, -1,  0,  0,  0,  0,  1 },
	{-1,  0,  0,  0, -1,  0,  0,  0,  1 },
	{ 0, -1,  0,  1,  0,  0,  0,  0,  1 },
	{ 1,  0,  0,  0,  1,  0,  0,  0,  1 },
	{ 0, -1,  0, -1,  0,  0,  0,  0, -1 },
	{ 1,  0,  0,  0, -1,  0,  0,  0, -1 },
	{ 0,  1,  0,  1,  0,  0,  0,  0, -1 },
	{-1,  0,  0,  0,  1,  0,  0,  0, -1 },
};

#define SENSOR_HAL_VERSION              (5021020)


static struct sensors_classdev yas533_mag_cdev = {
	.name = "yas533 magnetic",
	.vendor = "Yamaha Corporation",
	.version = SENSOR_HAL_VERSION,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1200.0",
	.resolution = "0.3",
	.sensor_power = "0.16",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev yas533_orien_cdev = {
	.name = "yamaha orientation",
	.vendor = "Yamaha",
	.version = SENSOR_HAL_VERSION,
	.handle = ID_ORIENTATION,
	.type = SENSOR_TYPE_ORIENTATION,
	.max_range = "0.0",
	.resolution = "0.0",
	.sensor_power = "0.0",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int yas533_mag_i2c_read(struct yas533_mag_sensor *mag,
				u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = mag->client->addr,
			.flags = mag->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = mag->client->addr,
			.flags = (mag->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(mag->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&mag->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int yas533_mag_i2c_write(struct yas533_mag_sensor *mag, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = mag->client->addr,
			.flags = mag->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(mag->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&mag->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int yas_read_bytes(struct yas533_mag_sensor *sensor_data,uint8_t addr, uint8_t *data, uint8_t number)
{
    int err=0;
	
	data[0] = addr;
	err = yas533_mag_i2c_read(sensor_data, data, number);
	if (err<0)
		printk(KERN_ERR "%s iic read error", __func__);
	
	return err;

}


static int yas_write_byte(struct yas533_mag_sensor *sensor_data,uint8_t addr, uint8_t data)
{
    uint8_t buf[2];
	int err=0;
	
	buf[0] = addr;
	buf[1] = data;
	err = yas533_mag_i2c_write(sensor_data, buf, 1);
	if (err<0)
		printk(KERN_ERR "%s iic write error", __func__);

	return err;
}

static uint32_t curtime(struct yas533_mag_sensor *sensor_data)
{

	return sensor_data->current_time;
}

static void xy1y2_to_linear(struct yas533_mag_sensor *sensor_data,uint16_t *xy1y2, int32_t *xy1y2_linear)
{
	static const uint16_t cval[] = {3721, 3971, 4221, 4471};
	int i;
	for (i = 0; i < 3; i++)
		xy1y2_linear[i] = xy1y2[i] - cval[sensor_data->cal.fxy1y2[i]]
			+ (sensor_data->hard_offset[i] - sensor_data->cal.rxy1y2[i])
			* sensor_data->coef[i];
}

static int get_cal_data_yas532(struct yas533_mag_sensor *sensor_data,struct yas_cal_data *c)
{
	uint8_t data[14]; int i;
	USB_REM_PRINTK("%s start\n",__func__);
	if (yas_read_bytes(sensor_data, YAS532_REG_CALR, data, 14) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_read_bytes(sensor_data, YAS532_REG_CALR, data, 14) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	c->fxy1y2[0] = (uint8_t)(((data[10]&0x01)<<1) | ((data[11]>>7)&0x01));
	c->rxy1y2[0] = ((int8_t)(((data[10]>>1) & 0x3f)<<2))>>2;
	c->fxy1y2[1] = (uint8_t)(((data[11]&0x01)<<1) | ((data[12]>>7)&0x01));
	c->rxy1y2[1] = ((int8_t)(((data[11]>>1) & 0x3f)<<2))>>2;
	c->fxy1y2[2] = (uint8_t)(((data[12]&0x01)<<1) | ((data[13]>>7)&0x01));
	c->rxy1y2[2] = ((int8_t)(((data[12]>>1) & 0x3f)<<2))>>2;
	c->cx = data[0] * 10 - 1280;
	c->cy1 = data[1] * 10 - 1280;
	c->cy2 = data[2] * 10 - 1280;
	c->a2 = ((data[3]>>2)&0x03f) - 32;
	c->a3 = (uint8_t)(((data[3]<<2) & 0x0c) | ((data[4]>>6) & 0x03)) - 8;
	c->a4 = (uint8_t)(data[4] & 0x3f) - 32;
	c->a5 = ((data[5]>>2) & 0x3f) + 38;
	c->a6 = (uint8_t)(((data[5]<<4) & 0x30) | ((data[6]>>4) & 0x0f)) - 32;
	c->a7 = (uint8_t)(((data[6]<<3) & 0x78) | ((data[7]>>5) & 0x07)) - 64;
	c->a8 = (uint8_t)(((data[7]<<1) & 0x3e) | ((data[8]>>7) & 0x01)) - 32;
	c->a9 = (uint8_t)(((data[8]<<1) & 0xfe) | ((data[9]>>7) & 0x01));
	c->k = (uint8_t)((data[9]>>2) & 0x1f);
	for (i = 0; i < 13; i++)
		if (data[i] != 0)
			return YAS_NO_ERROR;
	if (data[13] & 0x80)
		return YAS_NO_ERROR;
	return YAS_ERROR_CALREG;
}

#if YAS532_DRIVER_NO_SLEEP
static int busy_wait(struct yas533_mag_sensor *sensor_data)
{
	int i;
	uint8_t busy;
	for (i = 0; i < YAS_MAG_MAX_BUSY_LOOP; i++) {
		if (yas_read_bytes(sensor_data, YAS532_REG_DATAR, &busy, 1) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (!(busy & 0x80))
			return YAS_NO_ERROR;
	}
	return YAS_ERROR_BUSY;
}

static int wait_if_busy(void)
{
	int rt;
	if (sensor_data->start_flag && sensor_data->wait_flag) {
		rt = busy_wait();
		if (rt < 0)
			return rt;
		sensor_data->wait_flag = 0;
	}
	return YAS_NO_ERROR;
}
#endif

static int measure_start_yas532(struct yas533_mag_sensor *sensor_data,int ldtc, int fors, int wait)
{
	uint8_t data = 0x01;
	data = (uint8_t)(data | (((!!ldtc)<<1) & 0x02));
	data = (uint8_t)(data | (((!!fors)<<2) & 0x04));
	USB_REM_PRINTK("%s start\n",__func__);
	if (yas_write_byte(sensor_data,YAS532_REG_CMDR, data) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
#if YAS532_DRIVER_NO_SLEEP
	if (wait) {
		int rt;
		rt = busy_wait();
		if (rt < 0)
			return rt;
		sensor_data->wait_flag = 0;
	} else
		sensor_data->wait_flag = 1;
	sensor_data->start_flag = 1;
#else
	(void) wait;
	usleep(1500);/*sensor_data->cbk.usleep(1500);*/
#endif
	return YAS_NO_ERROR;
}

static int measure_normal_yas532(struct yas533_mag_sensor *sensor_data,int ldtc, int fors, int *busy, uint16_t *t,
		uint16_t *xy1y2, int *ouflow)
{
	uint8_t data[8];
	int i, rt;
	USB_REM_PRINTK("%s start\n",__func__);
#if YAS532_DRIVER_NO_SLEEP
	if (!sensor_data->start_flag) {
#endif
		rt = measure_start_yas532(sensor_data, ldtc, fors, 1);
		if (rt < 0)
			return rt;
#if YAS532_DRIVER_NO_SLEEP
	}
#endif
	if (yas_read_bytes(sensor_data, YAS532_REG_DATAR, data, 8) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
#if YAS532_DRIVER_NO_SLEEP
	sensor_data->start_flag = 0;
#endif
	*busy = (data[0]>>7) & 0x01;
	*t = (uint16_t)((((int32_t)data[0]<<3) & 0x3f8)|((data[1]>>5) & 0x07));
	xy1y2[0] = (uint16_t)((((int32_t)data[2]<<6) & 0x1fc0)
			| ((data[3]>>2) & 0x3f));
	xy1y2[1] = (uint16_t)((((int32_t)data[4]<<6) & 0x1fc0)
			| ((data[5]>>2) & 0x3f));
	xy1y2[2] = (uint16_t)((((int32_t)data[6]<<6) & 0x1fc0)
			| ((data[7]>>2) & 0x3f));
	*ouflow = 0;
	for (i = 0; i < 3; i++) {
		if (xy1y2[i] == YAS532_DATA_OVERFLOW)
			*ouflow |= (1<<(i*2));
		if (xy1y2[i] == YAS532_DATA_UNDERFLOW)
			*ouflow |= (1<<(i*2+1));
	}
	return YAS_NO_ERROR;
}

static int yas_cdrv_set_offset(struct yas533_mag_sensor *sensor_data,const int8_t *offset)
{USB_REM_PRINTK("%s start\n",__func__);
	if (yas_write_byte(sensor_data, YAS532_REG_OXR, (uint8_t)offset[0]) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_write_byte(sensor_data, YAS532_REG_OY1R, (uint8_t)offset[1]) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_write_byte(sensor_data, YAS532_REG_OY2R, (uint8_t)offset[2]) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	set_vector(sensor_data->hard_offset, offset);
	return YAS_NO_ERROR;
}

static int yas_cdrv_measure_and_set_offset(struct yas533_mag_sensor *sensor_data)
{
	static const int correct[5] = {16, 8, 4, 2, 1};
	int8_t hard_offset[3] = {0, 0, 0};
	uint16_t t, xy1y2[3];
	int32_t flag[3];
	int i, j, busy, ouflow, rt;
#if YAS532_DRIVER_NO_SLEEP
	sensor_data->start_flag = 0;
#endif
	USB_REM_PRINTK("%s start\n",__func__);

	for (i = 0; i < 5; i++) {
		rt = yas_cdrv_set_offset(sensor_data, hard_offset);
		if (rt < 0)
			return rt;
		rt = measure_normal_yas532(sensor_data, 0, 0, &busy, &t, xy1y2, &ouflow);
		if (rt < 0)
			return rt;
		if (busy)
			return YAS_ERROR_BUSY;
		for (j = 0; j < 3; j++) {
			if (YAS532_DATA_CENTER == xy1y2[j])
				flag[j] = 0;
			if (YAS532_DATA_CENTER < xy1y2[j])
				flag[j] = 1;
			if (xy1y2[j] < YAS532_DATA_CENTER)
				flag[j] = -1;
		}
		for (j = 0; j < 3; j++)
			if (flag[j])
				hard_offset[j] = (int8_t)(hard_offset[j]
						+ flag[j] * correct[i]);
	}
	return yas_cdrv_set_offset(sensor_data, hard_offset);
}

static int yas_cdrv_sensitivity_measuremnet(struct yas533_mag_sensor *sensor_data, int32_t *sx, int32_t *sy)
{
	struct yas_cal_data *c = &sensor_data->cal;
	uint16_t xy1y2_on[3], xy1y2_off[3], t;
	int busy, flowon = 0, flowoff = 0;
	USB_REM_PRINTK("%s start\n",__func__);
	if (measure_normal_yas532(sensor_data, 1, 0, &busy, &t, xy1y2_on, &flowon) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (busy)
		return YAS_ERROR_BUSY;
	if (measure_normal_yas532(sensor_data, 1, 1, &busy, &t, xy1y2_off, &flowoff) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (busy)
		return YAS_ERROR_BUSY;
	*sx = c->k * (xy1y2_on[0] - xy1y2_off[0]) * 10 / YAS_MAG_VCORE;
	*sy = c->k * c->a5 * ((xy1y2_on[1] - xy1y2_off[1])
			- (xy1y2_on[2] - xy1y2_off[2])) / 10 / YAS_MAG_VCORE;
	return flowon | flowoff;
}

static int yas_get_position(struct yas533_mag_sensor *sensor_data)
{
    USB_REM_PRINTK("%s start\n",__func__);
	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	return sensor_data->position;
}

static int yas_set_position(struct yas533_mag_sensor *sensor_data,int position)
{USB_REM_PRINTK("%s start\n",__func__);
	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	if (position < 0 || 7 < position)
		return YAS_ERROR_ARG;
	if (position == YAS532_MAG_NOTRANS_POSITION)
		sensor_data->transform = NULL;
	else
		sensor_data->transform = YAS532_TRANSFORMATION[position];
	sensor_data->position = position;
	return YAS_NO_ERROR;
}

static int yas_set_offset(struct yas533_mag_sensor *sensor_data, const int8_t *hard_offset)
{USB_REM_PRINTK("%s start\n",__func__);
	if (!sensor_data->enable) {
		set_vector(sensor_data->hard_offset, hard_offset);
		return YAS_NO_ERROR;
	}
	if (is_valid_offset(hard_offset)) {
#if YAS532_DRIVER_NO_SLEEP
		int rt;
		rt = wait_if_busy();
		if (rt < 0)
			return rt;
#endif
		if (yas_cdrv_set_offset(sensor_data, hard_offset) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		sensor_data->measure_state = YAS532_MAG_STATE_NORMAL;
	} else {
		set_vector(sensor_data->hard_offset, INVALID_OFFSET);
		sensor_data->measure_state = YAS532_MAG_STATE_MEASURE_OFFSET;
	}
	return YAS_NO_ERROR;
}

static int yas_measure(struct yas533_mag_sensor *sensor_data,struct yas_data *data, int num, int temp_correction,
		int *ouflow)
{
	struct yas_cal_data *c = &sensor_data->cal;
	int32_t xy1y2_linear[3];
	int32_t xyz_tmp[3], tmp;
	int32_t sx, sy1, sy2, sy, sz;
	int i, busy;
	uint16_t t, xy1y2[3];
	uint32_t tm;
	int rt;
#if 1 < YAS532_MAG_TEMPERATURE_LOG
	int32_t sum = 0;
#endif
	*ouflow = 0;
	USB_REM_PRINTK("%s start\n",__func__);

	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	if (data == NULL || num < 0)
		return YAS_ERROR_ARG;

	sensor_data->current_time += (uint32_t)sensor_data->delay;
	if (num == 0)
		return 0;
	if (!sensor_data->enable)
		return 0;
	switch (sensor_data->measure_state) {
	case YAS532_MAG_STATE_INIT_COIL:
		tm = curtime(sensor_data);
		if (tm - sensor_data->overflow_time < YAS532_MAG_INITCOIL_TIMEOUT)
			break;
		sensor_data->overflow_time = tm;
		if (yas_write_byte(sensor_data, YAS532_REG_RCOILR, 0x00) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (!sensor_data->overflow && is_valid_offset(sensor_data->hard_offset)) {
			sensor_data->measure_state = YAS532_MAG_STATE_NORMAL;
			break;
		}
		/* FALLTHRU */
	case YAS532_MAG_STATE_MEASURE_OFFSET:
		rt = yas_cdrv_measure_and_set_offset(sensor_data);
		if (rt < 0)
			return rt;
		sensor_data->measure_state = YAS532_MAG_STATE_NORMAL;
		break;
	}

	if (measure_normal_yas532(sensor_data, 0, 0, &busy, &t, xy1y2, ouflow) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	xy1y2_to_linear(sensor_data, xy1y2, xy1y2_linear);
#if 1 < YAS532_MAG_TEMPERATURE_LOG
	sensor_data->t.log[sensor_data->t.idx++] = t;
	if (YAS532_MAG_TEMPERATURE_LOG <= sensor_data->t.idx)
		sensor_data->t.idx = 0;
	sensor_data->t.num++;
	if (YAS532_MAG_TEMPERATURE_LOG <= sensor_data->t.num)
		sensor_data->t.num = YAS532_MAG_TEMPERATURE_LOG;
	for (i = 0; i < sensor_data->t.num; i++)
		sum += sensor_data->t.log[i];
	tmp = sum * 10 / sensor_data->t.num - YAS532_TEMP20DEGREE_TYPICAL * 10;
#else
	tmp = (t - YAS532_TEMP20DEGREE_TYPICAL) * 10;
#endif
	sx  = xy1y2_linear[0];
	sy1 = xy1y2_linear[1];
	sy2 = xy1y2_linear[2];
	if (temp_correction) {
		sx  -= (c->cx  * tmp) / 1000;
		sy1 -= (c->cy1 * tmp) / 1000;
		sy2 -= (c->cy2 * tmp) / 1000;
	}
	sy = sy1 - sy2;
	sz = -sy1 - sy2;
	data->xyz.v[0] = c->k * ((100   * sx + c->a2 * sy + c->a3 * sz) / 10);
	data->xyz.v[1] = c->k * ((c->a4 * sx + c->a5 * sy + c->a6 * sz) / 10);
	data->xyz.v[2] = c->k * ((c->a7 * sx + c->a8 * sy + c->a9 * sz) / 10);
	if (sensor_data->transform != NULL) {
		for (i = 0; i < 3; i++) {
			xyz_tmp[i] = sensor_data->transform[i*3] * data->xyz.v[0]
				+ sensor_data->transform[i*3+1] * data->xyz.v[1]
				+ sensor_data->transform[i*3+2] * data->xyz.v[2];
		}
		set_vector(data->xyz.v, xyz_tmp);
	}
	for (i = 0; i < 3; i++) {
		data->xyz.v[i] -= data->xyz.v[i] % 10;
		if (*ouflow & (1<<(i*2)))
			data->xyz.v[i] += 1; /* set overflow */
		if (*ouflow & (1<<(i*2+1)))
			data->xyz.v[i] += 2; /* set underflow */
	}
	tm = curtime(sensor_data);
	data->type = YAS_TYPE_MAG;
	data->accuracy = 0;
	if (busy)
		return YAS_ERROR_BUSY;
	if (0 < *ouflow) {
		if (!sensor_data->overflow)
			sensor_data->overflow_time = tm;
		sensor_data->overflow = 1;
		sensor_data->measure_state = YAS532_MAG_STATE_INIT_COIL;
	} else
		sensor_data->overflow = 0;
	for (i = 0; i < 3; i++)
		sensor_data->last_raw[i] = xy1y2[i];
	sensor_data->last_raw[i] = t;
#if YAS532_DRIVER_NO_SLEEP
	rt = measure_start_yas532(0, 0, 0);
	if (rt < 0)
		return rt;
#endif
	return 1;
}

static int yas_measure_wrap(struct yas533_mag_sensor *sensor_data, 
	                              struct yas_data *data, int num)
{
	int ouflow;
	USB_REM_PRINTK("%s start\n",__func__);
	return yas_measure(sensor_data,data, num, 1, &ouflow);
}

/*static int yas_get_enable(struct yas533_mag_sensor *sensor_data)
{
	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	return sensor_data->enable;
}*/

static int yas533_mag_set_enable(struct yas533_mag_sensor *sensor_data,int enable)
{
	int rt = YAS_NO_ERROR;
	USB_REM_PRINTK("%s start\n",__func__);
	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	enable = !!enable;
	if (sensor_data->enable == enable)
		return YAS_NO_ERROR;
	if (enable) {
		if (yas_write_byte(sensor_data, YAS532_REG_TEST1R, 0x00) < 0) {
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		if (yas_write_byte(sensor_data, YAS532_REG_TEST2R, 0x00) < 0) {
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		if (yas_write_byte(sensor_data, YAS532_REG_RCOILR, 0x00) < 0) {
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		if (is_valid_offset(sensor_data->hard_offset)) {
			if (yas_cdrv_set_offset(sensor_data, sensor_data->hard_offset) < 0) {
				return YAS_ERROR_DEVICE_COMMUNICATION;
			}
			sensor_data->measure_state = YAS532_MAG_STATE_NORMAL;
		} else {
			set_vector(sensor_data->hard_offset, INVALID_OFFSET);
			sensor_data->measure_state = YAS532_MAG_STATE_MEASURE_OFFSET;
		}
	}
	else 
	{
#if YAS532_DRIVER_NO_SLEEP
		rt = wait_if_busy();
#endif
	}
	sensor_data->enable = enable;
	return rt;
}

static int yas_ext(struct yas533_mag_sensor *sensor_data, int32_t cmd, void *p)
{
	struct yas532_self_test_result *r;
	struct yas_data data;
	int32_t xy1y2_linear[3], *raw_xyz;
	int rt, i, enable, ouflow, position;
	USB_REM_PRINTK("%s start\n",__func__);
	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	if (p == NULL)
		return YAS_ERROR_ARG;
	switch (cmd) {
	case YAS532_SELF_TEST:
		r = (struct yas532_self_test_result *) p;
		r->id = sensor_data->dev_id;
		enable = sensor_data->enable;
		if (!enable) {
			rt = yas533_mag_set_enable(sensor_data, 1);
			if (rt < 0)
				return rt;
		}
#if YAS532_DRIVER_NO_SLEEP
		rt = wait_if_busy();
		if (rt < 0)
			return rt;
#endif
		if (yas_write_byte(sensor_data, YAS532_REG_RCOILR, 0x00) < 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return YAS_ERROR_DEVICE_COMMUNICATION;
		}
		yas_set_offset(sensor_data, INVALID_OFFSET);
		position = yas_get_position(sensor_data);
		yas_set_position(sensor_data, YAS532_MAG_NOTRANS_POSITION);
		rt = yas_measure(sensor_data, &data, 1, 0, &ouflow);
		yas_set_position(sensor_data,position);
		set_vector(r->xy1y2, sensor_data->hard_offset);
		if (rt < 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return rt;
		}
		if (ouflow & YAS_OVERFLOW) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return YAS_ERROR_OVERFLOW;
		}
		if (ouflow & YAS_UNDERFLOW) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return YAS_ERROR_UNDERFLOW;
		}
		if (data.xyz.v[0] == 0 && data.xyz.v[1] == 0
				&& data.xyz.v[2] == 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return YAS_ERROR_DIRCALC;
		}
		r->dir = 99;
		for (i = 0; i < 3; i++)
			r->xyz[i] = data.xyz.v[i] / 1000;
#if YAS532_DRIVER_NO_SLEEP
		rt = wait_if_busy();
		if (rt < 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return rt;
		}
		sensor_data->start_flag = 0;
#endif
		rt = yas_cdrv_sensitivity_measuremnet(sensor_data, &r->sx, &r->sy);
		if (rt < 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return rt;
		}
		if (rt & YAS_OVERFLOW) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return YAS_ERROR_OVERFLOW;
		}
		if (rt & YAS_UNDERFLOW) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return YAS_ERROR_UNDERFLOW;
		}
		if (!enable)
			yas533_mag_set_enable(sensor_data, 0);
		return YAS_NO_ERROR;
	case YAS532_SELF_TEST_NOISE:
		raw_xyz = (int32_t *) p;
		enable = sensor_data->enable;
		if (!enable) {
			rt = yas533_mag_set_enable(sensor_data, 1);
			if (rt < 0)
				return rt;
		}
#if YAS532_DRIVER_NO_SLEEP
		rt = wait_if_busy();
		if (rt < 0)
			return rt;
#endif
		rt = yas_measure(sensor_data, &data, 1, 0, &ouflow);
		if (rt < 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return rt;
		}
#if YAS532_DRIVER_NO_SLEEP
		rt = wait_if_busy();
		if (rt < 0) {
			if (!enable)
				yas533_mag_set_enable(sensor_data, 0);
			return rt;
		}
#endif
		xy1y2_to_linear(sensor_data, sensor_data->last_raw, xy1y2_linear);
		raw_xyz[0] = xy1y2_linear[0];
		raw_xyz[1] = xy1y2_linear[1] - xy1y2_linear[2];
		raw_xyz[2] = -xy1y2_linear[1] - xy1y2_linear[2];
		if (!enable)
			yas533_mag_set_enable(sensor_data, 0);
		return YAS_NO_ERROR;
	case YAS532_GET_HW_OFFSET:
		set_vector((int8_t *) p, sensor_data->hard_offset);
		return YAS_NO_ERROR;
	case YAS532_SET_HW_OFFSET:
		return yas_set_offset(sensor_data, (int8_t *) p);
	case YAS532_GET_LAST_RAWDATA:
		for (i = 0; i < 4; i++)
			((uint16_t *) p)[i] = sensor_data->last_raw[i];
		return YAS_NO_ERROR;
	default:
		break;
	}
	return YAS_ERROR_ARG;
}

static int yas533_mag_chip_init(struct yas533_mag_sensor *sensor_data)
{
	int i, rt;
	uint8_t data;
	USB_REM_PRINTK("%s start\n",__func__);
	if (sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;

	if (yas_read_bytes(sensor_data, YAS532_REG_DEVID, &data, 1) < 0)
	{
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	sensor_data->dev_id = data;
	if (sensor_data->dev_id != YAS532_DEVICE_ID)
	{
		return YAS_ERROR_CHIP_ID;
	}
	rt = get_cal_data_yas532(sensor_data, &sensor_data->cal);
	if (rt < 0)
	{
		return rt;
	}

	sensor_data->measure_state = YAS532_MAG_STATE_INIT_COIL;
	set_vector(sensor_data->hard_offset, INVALID_OFFSET);
	sensor_data->overflow = 0;
	sensor_data->overflow_time = sensor_data->current_time;
	sensor_data->position = YAS532_MAG_NOTRANS_POSITION;
	sensor_data->delay = YAS_DEFAULT_SENSOR_DELAY;
	sensor_data->enable = 0;
	sensor_data->transform = NULL;
#if YAS532_DRIVER_NO_SLEEP
	sensor_data->start_flag = 0;
	sensor_data->wait_flag = 0;
#endif
#if 1 < YAS532_MAG_TEMPERATURE_LOG
	sensor_data->t.num = sensor_data->t.idx = 0;
#endif
	sensor_data->current_time = curtime(sensor_data);
	for (i = 0; i < 3; i++) {
		sensor_data->coef[i] = yas532_version_ac_coef[i];
		sensor_data->last_raw[i] = 0;
	}
	sensor_data->last_raw[3] = 0;
	sensor_data->initialized = 1;
	return YAS_NO_ERROR;
}

/*static int yas_term(struct yas533_mag_sensor *sensor_data)
{
	int rt;
	if (!sensor_data->initialized)
		return YAS_ERROR_INITIALIZE;
	rt = yas533_mag_set_enable(sensor_data, 0);
	sensor_data->initialized = 0;
	return rt;
}*/

static void yas533_mag_poll_work(struct work_struct *work)
{

	struct yas_data mag;
	struct yas533_mag_sensor *sensor_data =container_of((struct delayed_work *)work,
				struct yas533_mag_sensor, mag_poll_work);
	int ret;
	USB_REM_PRINTK("%s interval %d\n",__func__, sensor_data->delay);

	mutex_lock(&sensor_data->lock);
	ret = yas_measure_wrap(sensor_data,&mag, 1);
	if (ret == 1)
	{
		input_report_abs(sensor_data->input_dev, ABS_X, mag.xyz.v[0]);
		input_report_abs(sensor_data->input_dev, ABS_Y, mag.xyz.v[1]);
		input_report_abs(sensor_data->input_dev, ABS_Z, mag.xyz.v[2]);
		input_sync(sensor_data->input_dev);

	}
	mutex_unlock(&sensor_data->lock);

	queue_delayed_work(sensor_data->mag_workqueue, 
		               &sensor_data->mag_poll_work, 
	                   msecs_to_jiffies(sensor_data->delay));

}

static int yas533_mag_input_init(struct yas533_mag_sensor *stat)
{
	int err=0;
    
	if(!stat)
	{
       return -EINVAL;
	}
    USB_REM_PRINTK("%s\n",__func__);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev)
	{
		err = -ENOMEM;
		return err;
	}

	stat->input_dev->name = YAS533_MAG_DEV_NAME;

	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

	input_set_abs_params(stat->input_dev, ABS_X, -NUMBERBIT16-1, NUMBERBIT16, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Y, -NUMBERBIT16-1, NUMBERBIT16, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Z, -NUMBERBIT16-1, NUMBERBIT16, 0, 0);

	err = input_register_device(stat->input_dev);
	if (err)
	{
		input_free_device(stat->input_dev);
		return err;
	}

	return 0;
}

static ssize_t yas533_mag_polling_rate_show(struct device *dev,
				                          struct device_attribute *attr,
				                          char *buf)
{
	int val;
	struct yas533_mag_sensor *sensor_data = dev_get_drvdata(dev);
	USB_REM_PRINTK("%s start\n",__func__);
	mutex_lock(&sensor_data->lock);
	val = sensor_data->delay;
	mutex_unlock(&sensor_data->lock);
	return snprintf(buf, 8, "%d\n", val);

}

static ssize_t yas533_mag_polling_rate_store(struct device *dev,
				                          struct device_attribute *attr,
				                          const char *buf, 
				                          size_t size)
{
    struct yas533_mag_sensor *sensor_data = dev_get_drvdata(dev);
    unsigned long interval_ms;
    
    if (kstrtoul(buf, 10, &interval_ms))
    	return -EINVAL;
    if (!interval_ms)
    	return -EINVAL;
	
    mutex_lock(&sensor_data->lock);
    sensor_data->delay= interval_ms;
    mutex_unlock(&sensor_data->lock);
	USB_REM_PRINTK("%s interval %d %lu\n",__func__, sensor_data->delay, interval_ms);
    return size;

}

static ssize_t yas533_mag_enable_show(struct device *dev,
				                          struct device_attribute *attr,
				                          char *buf)
{
	struct yas533_mag_sensor *sensor_data = dev_get_drvdata(dev);
	int val;
	USB_REM_PRINTK("%s start\n",__func__);
	mutex_lock(&sensor_data->lock);
	val = sensor_data->enable;
	mutex_unlock(&sensor_data->lock);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t yas533_mag_enable_store(struct device *dev,
				                          struct device_attribute *attr,
				                          const char *buf, 
				                          size_t size)
{
	struct yas533_mag_sensor *sensor_data = dev_get_drvdata(dev);
	unsigned long val;
	int err=0;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
    
    val=!!val;
	USB_REM_PRINTK("%s enable %lu\n",__func__, val);
	mutex_lock(&sensor_data->lock);
    err=yas533_mag_set_enable(sensor_data, val);
	mutex_unlock(&sensor_data->lock);
	if(err)
	{
		printk(KERN_ERR "%s enable %ld failed: %d\n", __func__, val, err);
		return -ENOTSUPP;
	}
	else
	{
        if(val)
			queue_delayed_work(sensor_data->mag_workqueue, &sensor_data->mag_poll_work, 0);			
	}
	if(!val)
	    cancel_delayed_work_sync(&sensor_data->mag_poll_work);
	return 0;
}

static ssize_t yas533_mag_hard_offset_show(struct device *dev,
				                          struct device_attribute *attr,
				                          char *buf)
{
	struct yas533_mag_sensor *sensor_data = dev_get_drvdata(dev);
	int8_t hard_offset[3];
	int ret;
    USB_REM_PRINTK("%s start\n",__func__);
	mutex_lock(&sensor_data->lock);
	ret = yas_ext(sensor_data, YAS532_GET_HW_OFFSET, hard_offset);
	mutex_unlock(&sensor_data->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", hard_offset[0], hard_offset[1], hard_offset[2]);
}

static ssize_t yas533_mag_hard_offset_store(struct device *dev,
				                          struct device_attribute *attr,
				                          const char *buf, 
				                          size_t size)
{
	struct yas533_mag_sensor *sensor_data = dev_get_drvdata(dev);
	int32_t tmp[3];
	int8_t hard_offset[3];
	int ret, i;
    
	sscanf(buf, "%d %d %d\n", &tmp[0], &tmp[1], &tmp[2]);
	USB_REM_PRINTK("%s hard offset %d %d %d\n",__func__, tmp[0], tmp[1], tmp[2]);
	for (i = 0; i < 3; i++)
		hard_offset[i] = (int8_t)tmp[i];
	mutex_lock(&sensor_data->lock);
	ret = yas_ext(sensor_data, YAS532_SET_HW_OFFSET, hard_offset);
	mutex_unlock(&sensor_data->lock);
	if (ret < 0)
		return -EFAULT;
	return size;

}


static struct device_attribute yas533_mag_attributes[] = {
	__ATTR(poll_delay, 0666, yas533_mag_polling_rate_show, yas533_mag_polling_rate_store),
	__ATTR(enable, 0666, yas533_mag_enable_show, yas533_mag_enable_store),
	__ATTR(hard_offset, 0666, yas533_mag_hard_offset_show, yas533_mag_hard_offset_store),

};


static int yas533_create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err=0;
	USB_REM_PRINTK("%s\n",__func__);
	for (i=0; i<ARRAY_SIZE(yas533_mag_attributes); i++)
	{
		err=device_create_file(dev, yas533_mag_attributes + i);
		if (err)
		{
			for (--i; i >= 0; --i)
				device_remove_file(dev, yas533_mag_attributes+i);
			return err;
		}
	}
	return 0;
}

static int yas533_mag_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct yas533_mag_sensor *sensor_data = container_of(sensors_cdev,
		struct yas533_mag_sensor, cdev_mag);
	int err=0;

	USB_REM_PRINTK("%s enable %d\n",__func__, enable);
	mutex_lock(&sensor_data->lock);
    err=yas533_mag_set_enable(sensor_data, enable);
	mutex_unlock(&sensor_data->lock);
	if(err)
	{
		printk(KERN_ERR "%s enable %d failed: %d\n", __func__, enable, err);
		return -ENOTSUPP;
	}
	else
	{
        if(enable)
			queue_delayed_work(sensor_data->mag_workqueue, &sensor_data->mag_poll_work, 0);			
	}
	if(!enable)
	    cancel_delayed_work_sync(&sensor_data->mag_poll_work);
	return 0;

}

static int yas533_mag_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	int err=0;
	struct yas533_mag_sensor *sensor_data = container_of(sensors_cdev,
		struct yas533_mag_sensor, cdev_mag);
	
    mutex_lock(&sensor_data->lock);
    sensor_data->delay= delay_msec;
    mutex_unlock(&sensor_data->lock);
	USB_REM_PRINTK("%s interval %d %u\n",__func__, sensor_data->delay, delay_msec);

	return err;
}


static void yas533_mag_input_cleanup(struct yas533_mag_sensor *stat)
{
    USB_REM_PRINTK("%s\n",__func__);
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int yas533_mag_remove_sysfs_interfaces(struct device *dev)
{
	int i;
	USB_REM_PRINTK("%s\n",__func__);
	for (i = 0; i < ARRAY_SIZE(yas533_mag_attributes); i++)
		device_remove_file(dev, yas533_mag_attributes + i);
	return 0;
}

static int yas533_mag_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct yas533_mag_sensor *stat;
	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;
	int err=0;

	if(!client /*|| !devid*/)
	{
        printk(KERN_ERR "%s null pointer %p %p\n", __func__, client, devid);
		err=-EINVAL;
		goto error_exit;
	}
    USB_REM_PRINTK("%s start\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		USB_REM_PRINTK("%s adapter not support i2c\n", __func__);
		if (!i2c_check_functionality(client->adapter, smbus_func))
		{
			err = -ENODEV;
			printk(KERN_ERR  "%s adapter not support SMBUS\n", __func__);
			goto error_exit;
		}
	}

	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (NULL==stat)
	{
		printk(KERN_ERR "%s allocate memory for mag sensor failed\n", __func__);
		err = -ENOMEM;
		goto error_exit;
	}

	stat->mag_workqueue = create_singlethread_workqueue("yas533_mag_workqueue");
	if(!stat->mag_workqueue)
    {
		err = -ENOMEM;
		printk(KERN_ERR "%s create work queue failed\n",__func__);
		goto workqueue_create_failed;
	}

	mutex_init(&stat->lock);
	stat->client = client;

    /*add init data code*/
	i2c_set_clientdata(client, stat);

	err = yas533_mag_input_init(stat);
	if(err)
	{
		printk(KERN_ERR "%s create input device failed: %d\n",__func__, err);
		err = -ENOMEM;
		goto input_create_failed;
	}


	err = yas533_create_sysfs_interfaces(&client->dev);
	if(err)
	{
		printk(KERN_ERR "%s device file creat failed: %d\n", __func__, err);
		goto device_file_create_failed;
	}

	stat->cdev_mag = yas533_mag_cdev;
	stat->cdev_mag.sensors_enable = yas533_mag_enable_set;
	stat->cdev_mag.sensors_poll_delay = yas533_mag_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &stat->cdev_mag);
	if(err)
	{
		printk(KERN_ERR  "%s sensor class mag create failed: %d\n", __func__, err);
		goto sensor_class_mag_create_failed;
	}

	stat->cdev_orien = yas533_orien_cdev;
	stat->cdev_orien.sensors_enable = NULL;
	stat->cdev_orien.sensors_poll_delay = NULL;
	err = sensors_classdev_register(&client->dev, &stat->cdev_orien);
	if(err)
	{
		printk(KERN_ERR  "%s sensor class orientation create failed: %d\n", __func__, err);
		goto sensor_class_orien_create_failed;
	}

	INIT_DELAYED_WORK(&stat->mag_poll_work,yas533_mag_poll_work);/**/

	err=yas533_mag_chip_init(stat);
	if(err)
	{
	    printk(KERN_ERR  "%s chip init failed: %d\n", __func__, err);
		goto sensor_class_init_failed;
	}

	
	printk(KERN_ERR "%s haha probe successfully\n",__func__);
	return 0;

sensor_class_init_failed:
	sensors_classdev_unregister(&stat->cdev_orien);
sensor_class_orien_create_failed:
    sensors_classdev_unregister(&stat->cdev_mag);
sensor_class_mag_create_failed:
	yas533_mag_remove_sysfs_interfaces(&client->dev);
device_file_create_failed:
    yas533_mag_input_cleanup(stat);
input_create_failed:
	destroy_workqueue(stat->mag_workqueue);
workqueue_create_failed:
	kfree(stat);
error_exit:
	return err;
}


static int yas533_mag_remove(struct i2c_client *client)
{
	struct yas533_mag_sensor *stat = i2c_get_clientdata(client);

    USB_REM_PRINTK("%s\n",__func__);
	cancel_delayed_work_sync(&stat->mag_poll_work);
	if(stat->mag_workqueue) {
		flush_workqueue(stat->mag_workqueue);
		destroy_workqueue(stat->mag_workqueue);
	}

	/*add disable sensor code*//**/
	sensors_classdev_unregister(&stat->cdev_mag);
	sensors_classdev_unregister(&stat->cdev_orien);
    yas533_mag_remove_sysfs_interfaces(&client->dev);
	yas533_mag_input_cleanup(stat);
	kfree(stat);
	return 0;
}

static int yas533_mag_suspend(struct device *dev)
{
	return 0;
}

static int yas533_mag_resume(struct device *dev)
{
	return 0;
}


static const struct i2c_device_id yas533_mag_id[] = {
	{ YAS533_MAG_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, yas533_mag_id);

static const struct dev_pm_ops yas533_mag_pm = {
	.suspend = yas533_mag_suspend,
	.resume = yas533_mag_resume,
};

static struct of_device_id yas533_mag_match_table[] = {
	{ .compatible = "yas533_mag", },
	{ },
};

static struct i2c_driver yas533_mag_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = YAS533_MAG_DEV_NAME,
			.pm = &yas533_mag_pm,
			.of_match_table = yas533_mag_match_table,
	},
	.probe = yas533_mag_probe,
	.remove = yas533_mag_remove,
	.id_table = yas533_mag_id,

};

static int __init yas533_mag_init(void)
{
    USB_REM_PRINTK("%s\n",__func__);
	return i2c_add_driver(&yas533_mag_driver);
}

static void __exit yas533_mag_exit(void)
{
    USB_REM_PRINTK("%s\n",__func__);
	i2c_del_driver(&yas533_mag_driver);
	return;
}

module_init(yas533_mag_init);
module_exit(yas533_mag_exit);

MODULE_DESCRIPTION("yas533 magoscope driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
