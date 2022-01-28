/*
 * MPU6500 6-axis gyroscope + accelerometer driver
 *
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>
#include "mpu6500.h"

#define DEBUG_NODE

#define IS_ODD_NUMBER(x)	(x & 1UL)

#define MPU6500_ACCEL_MIN_VALUE	-32768
#define MPU6500_ACCEL_MAX_VALUE	32767
#define MPU6500_GYRO_MIN_VALUE	-32768
#define MPU6500_GYRO_MAX_VALUE	32767

#define GRAVITY_EARTH                   981
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5

#define MPU6500_MAX_EVENT_CNT	170
/* Limit mininum delay to 10ms as we do not need higher rate so far */
#define MPU6500_ACCEL_MIN_POLL_INTERVAL_MS	10
#define MPU6500_ACCEL_MAX_POLL_INTERVAL_MS	5000
#define MPU6500_ACCEL_DEFAULT_POLL_INTERVAL_MS	200
#define MPU6500_ACCEL_INT_MAX_DELAY			19

#define MPU6500_GYRO_MIN_POLL_INTERVAL_MS	10
#define MPU6500_GYRO_MAX_POLL_INTERVAL_MS	5000
#define MPU6500_GYRO_DEFAULT_POLL_INTERVAL_MS	200
#define MPU6500_GYRO_INT_MAX_DELAY		18

#define MPU6500_RAW_ACCEL_DATA_LEN	6
#define MPU6500_RAW_GYRO_DATA_LEN	6
#define MPU6500_AXES_NUM        3

#define MPU6500_RESET_SLEEP_US	10

#define MPU6500_NAME            "MPU6500"
#define MPU6500_DEV_NAME_ACCEL	"MPU6500-accel"
#define MPU6500_DEV_NAME_GYRO	"MPU6500-gyro"

#define MPU6500_PINCTRL_DEFAULT	"mpu_default"
#define MPU6500_PINCTRL_SUSPEND	"mpu_sleep"

/*----------------------------------------------------------------------------*/
#define TAG  "Gsensor "       //"Msensor" "PAsensor" "GYsensor"
#define TAGI "Gsensor.I "     //KERN_INFO 
#define TAGE "Gsensor.E "     //KERN_ERR 
extern void print_vivo_init(const char* fmt, ...);
extern void print_vivo_main(const char* fmt, ...);

#define CONT_INT_TIME     50    //50ms 
#define POLL_DELAY_TIMES  10     //2X

#define CONT_INT_GATE   20 
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0
/*----------------------------------------------------------------------------*/

enum mpu6500_place {
	MPU6500_PLACE_PU = 0,
	MPU6500_PLACE_PR = 1,
	MPU6500_PLACE_LD = 2,
	MPU6500_PLACE_LL = 3,
	MPU6500_PLACE_PU_BACK = 4,
	MPU6500_PLACE_PR_BACK = 5,
	MPU6500_PLACE_LD_BACK = 6,
	MPU6500_PLACE_LL_BACK = 7,
	MPU6500_PLACE_UNKNOWN = 8,
	MPU6500_AXIS_REMAP_TAB_SZ = 8
};

struct mpu6500_place_name {
	char name[32];
	enum mpu6500_place place;
};

struct calibrate_data {
	int x;
	int y;
	int z;
	int self_test;
};

struct axis_data {
	int x;
	int y;
	int z;
};

/**
 *  struct mpu6500_sensor - Cached chip configuration data
 *  @client:		I2C client
 *  @dev:		device structure
 *  @accel_dev:		accelerometer input device structure
 *  @gyro_dev:		gyroscope input device structure
 *  @accel_cdev:		sensor class device structure for accelerometer
 *  @gyro_cdev:		sensor class device structure for gyroscope
 *  @pdata:	device platform dependent data
 *  @op_lock:	device operation mutex
 *  @chip_type:	sensor hardware model
 *  @accel_poll_work:	accelerometer delay work structur
 *  @gyro_poll_work:	gyroscope delay work structure
 *  @fifo_flush_work:	work structure to flush sensor fifo
 *  @reg:		notable slave registers
 *  @cfg:		cached chip configuration data
 *  @axis:	axis data reading
 *  @gyro_poll_ms:	gyroscope polling delay
 *  @accel_poll_ms:	accelerometer polling delay
 *  @accel_latency_ms:	max latency for accelerometer batching
 *  @gyro_latency_ms:	max latency for gyroscope batching
 *  @accel_en:	accelerometer enabling flag
 *  @gyro_en:	gyroscope enabling flag
 *  @use_poll:		use polling mode instead of  interrupt mode
 *  @motion_det_en:	motion detection wakeup is enabled
 *  @batch_accel:	accelerometer is working on batch mode
 *  @batch_gyro:	gyroscope is working on batch mode
 *  @vlogic:	regulator data for Vlogic
 *  @vdd:	regulator data for Vdd
 *  @vi2c:	I2C bus pullup
 *  @enable_gpio:	enable GPIO
 *  @power_enabled:	flag of device power state
 *  @pinctrl:	pinctrl struct for interrupt pin
 *  @pin_default:	pinctrl default state
 *  @pin_sleep:	pinctrl sleep state
 *  @flush_count:	number of flush
 *  @fifo_start_ns:		timestamp of first fifo data
 */
struct mpu6500_sensor {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *accel_dev;
	struct input_dev *gyro_dev;
	struct sensors_classdev accel_cdev;
	struct sensors_classdev gyro_cdev;
	struct mpu6500_platform_data *pdata;
	struct mutex op_lock;
	enum inv_devices chip_type;
	struct workqueue_struct *data_wq;
	struct work_struct accel_poll_work;
	struct hrtimer timer_acc;
	ktime_t ktime_acc;
	struct work_struct gyro_poll_work;
	struct hrtimer timer_gyro;
	ktime_t ktime_gyro;
	struct mpu_reg_map reg;
	struct mpu_chip_config cfg;
	
	struct axis_data accel_lastraw;	   /* last measured data */
	struct axis_data accel_lastdata;       /* last measured data */
	struct calibrate_data accel_calibration_data;
	int	accel_calibrate_process;
	
	struct axis_data gyro_lastdata;       /* last measured data */
	struct calibrate_data gyro_calibration_data;
	int	gyro_calibrate_process;
	
	u32 gyro_poll_ms;
	u32 accel_poll_ms;
	atomic_t accel_en;
	atomic_t gyro_en;
	bool use_poll;
	
	atomic_t sensor_int;

	 int irq_gsensor;
	struct work_struct irq_work;
	bool sensor_power;

	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct inv_selftest_device mpu_selftest_device;
};

static struct i2c_client *mpu6500_i2c_client;

/* Accelerometer information read by HAL */
static struct sensors_classdev mpu6500_acc_cdev = {
	.name = "MPU6500-accel",
	.vendor = "Invensense",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",	/* m/s^2 */
	.resolution = "0.000598144",	/* m/s^2 */
	.sensor_power = "0.5",	/* 0.5 mA */
	.min_delay = MPU6500_ACCEL_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = MPU6500_ACCEL_MAX_POLL_INTERVAL_MS,
	.delay_msec = MPU6500_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
};

/* gyroscope information read by HAL */
static struct sensors_classdev mpu6500_gyro_cdev = {
	.name = "MPU6500-gyro",
	.vendor = "Invensense",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "34.906586",	/* rad/s */
	.resolution = "0.0010681152",	/* rad/s */
	.sensor_power = "3.6",	/* 3.6 mA */
	.min_delay = MPU6500_GYRO_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = MPU6500_GYRO_MAX_POLL_INTERVAL_MS,
	.delay_msec = MPU6500_ACCEL_DEFAULT_POLL_INTERVAL_MS,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.max_latency = 0,
	.flags = 0, /* SENSOR_FLAG_CONTINUOUS_MODE */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_enable_wakeup = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
};

struct sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

static const struct sensor_axis_remap
mpu6500_axis_remap_tab[MPU6500_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static const struct mpu6500_place_name
mpu6500_place_name2num[MPU6500_AXIS_REMAP_TAB_SZ] = {
	{"Portrait Up", MPU6500_PLACE_PU},
	{"Landscape Right", MPU6500_PLACE_PR},
	{"Portrait Down", MPU6500_PLACE_LD},
	{"Landscape Left", MPU6500_PLACE_LL},
	{"Portrait Up Back Side", MPU6500_PLACE_PU_BACK},
	{"Landscape Right Back Side", MPU6500_PLACE_PR_BACK},
	{"Portrait Down Back Side", MPU6500_PLACE_LD_BACK},
	{"Landscape Left Back Side", MPU6500_PLACE_LL_BACK},
};

/* Map gyro measurement range setting to number of bit to shift */
static const u8 mpu_gyro_fs_shift[NUM_FSR] = {
	GYRO_SCALE_SHIFT_FS0, /* MPU_FSR_250DPS */
	GYRO_SCALE_SHIFT_FS1, /* MPU_FSR_500DPS */
	GYRO_SCALE_SHIFT_FS2, /* MPU_FSR_1000DPS */
	GYRO_SCALE_SHIFT_FS3, /* MPU_FSR_2000DPS */
};

/* Map accel measurement range setting to number of bit to shift */
static const u8 mpu_accel_fs_shift[NUM_ACCL_FSR] = {
	ACCEL_SCALE_SHIFT_02G, /* ACCEL_FS_02G */
	ACCEL_SCALE_SHIFT_04G, /* ACCEL_FS_04G */
	ACCEL_SCALE_SHIFT_08G, /* ACCEL_FS_08G */
	ACCEL_SCALE_SHIFT_16G, /* ACCEL_FS_16G */
};

void gpio_switch_setstate(int state);
/* Function declarations */
static int mpu6500_set_power_mode(struct mpu6500_sensor *sensor,
					bool power_on);

static int mpu6500_config_sample_rate(struct mpu6500_sensor *sensor);

/**
 * mpu6500_read_reg() - read multiple register data
 * @start_addr: register address read from
 * @buffer: provide register addr and get register
 * @length: length of register
 *
 * Reads the register values in one transaction or returns a negative
 * error code on failure.
 */
static int mpu6500_read_reg(struct mpu6500_sensor *sensor,u8 start_addr,
			       u8 *buffer, int length)
{
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	int err;
	int tries = 0;
	
	struct i2c_msg msg[] = {
		{
			.addr = sensor->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &start_addr,
		},
		{
			.addr = sensor->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};
	
	do {
		err = i2c_transfer(sensor->client->adapter, msg, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		printk(KERN_ERR TAGE "%s i2c_transfer error: (%d %p %d) %d\n", __func__, start_addr, buffer, length, err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

/**
 * mpu6500_read_accel_data() - get accelerometer data from device
 * @sensor: sensor device instance
 * @data: axis data to update
 *
 * Return the converted X Y and Z data from the sensor device
 */
static int mpu6500_read_accel_data(struct mpu6500_sensor *sensor,
			     struct axis_data *data)
{
	int err = -1;
	u16 buffer[3];
	s16 tmp[3];
	const struct sensor_axis_remap *remap;
	static int same_num_x = 0;
	static int same_num_y = 0;
	static int same_num_z = 0;
	
	err = mpu6500_read_reg(sensor, sensor->reg.raw_accel,
		(u8 *)buffer, MPU6500_RAW_ACCEL_DATA_LEN);
	if (err < 0)
		return err;
	tmp[0] = be16_to_cpu(buffer[0]);
	tmp[1] = be16_to_cpu(buffer[1]);
	tmp[2] = be16_to_cpu(buffer[2]);
//	printk(KERN_INFO TAGI "%s tmp[0] = %d,tmp[1] = %d,tmp[2] = %d\n", __func__,tmp[0],tmp[1],tmp[2]);
	tmp[0] = tmp[0]>>4;
	tmp[1] = tmp[1]>>4;
	tmp[2] = tmp[2]>>4;
//	printk(KERN_INFO TAGI "%s tmp[0] = %d,tmp[1] = %d,tmp[2] = %d\n", __func__,tmp[0],tmp[1],tmp[2]);
	remap = &mpu6500_axis_remap_tab[sensor->pdata->place];
//	printk(KERN_INFO TAGI "%s place = %d!!!\n", __func__,sensor->pdata->place);
	data->x = tmp[remap->src_x] * remap->sign_x;
	data->y = tmp[remap->src_y] * remap->sign_y;
	data->z = tmp[remap->src_z] * remap->sign_z;
//	printk(KERN_INFO TAGI "%s data->x = %d,data->y = %d,data->z = %d\n", __func__,data->x,data->y,data->z);
    if(sensor->accel_lastraw.x == data->x)
    {
        if(same_num_x < SAME_DATA_GATE)
            same_num_x++;
        else
            printk(KERN_ERR TAGE "same data x error, x = %d\n",data->x);
    }
    else
    {
        same_num_x = 0;
    }
	if(sensor->accel_lastraw.y == data->y)
    {
        if(same_num_y < SAME_DATA_GATE)
            same_num_y++;
        else
            printk(KERN_ERR TAGE "same data y error, y = %d\n",data->y);
    }
    else
    {
        same_num_y = 0;
    }
	if(sensor->accel_lastraw.z == data->z)
    {
        if(same_num_z < SAME_DATA_GATE)
            same_num_z++;
        else
            printk(KERN_ERR TAGE "same data z error, z = %d\n",data->z);
    }
    else
    {
        same_num_z = 0;
    }
	sensor->accel_lastraw.x = data->x;
    sensor->accel_lastraw.y = data->y;
    sensor->accel_lastraw.z = data->z;
    
    sensor->accel_lastdata.x = data->x * 1000/1024 * GRAVITY_EARTH/100;
    sensor->accel_lastdata.y = data->y * 1000/1024 * GRAVITY_EARTH/100;
    sensor->accel_lastdata.z = data->z * 1000/1024 * GRAVITY_EARTH/100;
//	printk(KERN_INFO TAGI "%s sensor->accel_lastdata.x = %d,sensor->accel_lastdata.y = %d,sensor->accel_lastdata.z = %d\n",
//	 __func__,sensor->accel_lastdata.x,sensor->accel_lastdata.y,sensor->accel_lastdata.z);
	return 0;
}

/**
 * mpu6500_read_gyro_data() - get gyro data from device
 * @sensor: sensor device instance
 * @data: axis data to update
 *
 * Return the converted RX RY and RZ data from the sensor device
 */
static void mpu6500_read_gyro_data(struct mpu6500_sensor *sensor,
			     struct axis_data *data)
{
	u16 buffer[3];
	s16 tmp[3];
	const struct sensor_axis_remap *remap;
	
	mpu6500_read_reg(sensor, sensor->reg.raw_gyro,
		(u8 *)buffer, MPU6500_RAW_GYRO_DATA_LEN);
	tmp[0] = be16_to_cpu(buffer[0]);
	tmp[1] = be16_to_cpu(buffer[1]);
	tmp[2] = be16_to_cpu(buffer[2]);
//	printk(KERN_INFO TAGI "%s tmp[0] = %d,tmp[1] = %d,tmp[2] = %d\n", __func__,tmp[0],tmp[1],tmp[2]);
	
	remap = &mpu6500_axis_remap_tab[sensor->pdata->place];
	data->x = tmp[remap->src_x] * remap->sign_x;
	data->y = tmp[remap->src_y] * remap->sign_y;
	data->z = tmp[remap->src_z] * remap->sign_z;
//	printk(KERN_INFO TAGI "%s data->x = %d,data->y = %d,data->z = %d\n", __func__,data->x,data->y,data->z);
	
	sensor->gyro_lastdata.x = data->x;
	sensor->gyro_lastdata.y = data->y;
	sensor->gyro_lastdata.z = data->z;
	
}

/**
 * mpu6500_accel_work_fn() - polling accelerometer data
 * @work: the work struct
 *
 * Called by the work queue; read sensor data and generate an input
 * event
 */
static void mpu6500_accel_work_fn(struct work_struct *work)
{
	struct mpu6500_sensor *sensor;
	struct axis_data data;
//	ktime_t timestamp;
	sensor = container_of(work,struct mpu6500_sensor, accel_poll_work);
				
	if (sensor->sensor_power == false)
    {
        mpu6500_set_power_mode(sensor, true);
    }
	mpu6500_read_accel_data(sensor, &data);
//	timestamp = ktime_get_boottime();
	input_report_abs(sensor->accel_dev, ABS_X, data.x-sensor->accel_calibration_data.x);
	input_report_abs(sensor->accel_dev, ABS_Y, data.y-sensor->accel_calibration_data.y);
	input_report_abs(sensor->accel_dev, ABS_Z, data.z-sensor->accel_calibration_data.z);
	input_event(sensor->accel_dev,EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(sensor->ktime_acc).tv_sec);
	input_event(sensor->accel_dev,EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(sensor->ktime_acc).tv_nsec);
//	printk(KERN_INFO TAGI "%s read time_s=%ld, time_ns=%ld \n",
//		__func__, ktime_to_timespec(sensor->ktime_acc).tv_sec,ktime_to_timespec(sensor->ktime_acc).tv_nsec);
	input_sync(sensor->accel_dev);
	
//	if (atomic_read(&sensor->accel_en))
//		queue_delayed_work(sensor->data_wq,&sensor->accel_poll_work,
//			msecs_to_jiffies(sensor->accel_poll_ms));
}

/**
 * mpu6500_acc_timer_handle() - timer_acc handle to schedule workerthread
 * @hrtimer: the hrtimer struct
 *
 * handle gets called based on the poll time to schedule worker thread
 */
static enum hrtimer_restart mpu6500_acc_timer_handle(struct hrtimer *hrtimer)
{
	ktime_t ktime;
	struct mpu6500_sensor *sensor;
	sensor = container_of(hrtimer, struct mpu6500_sensor, timer_acc);

	ktime = sensor->ktime_acc =ktime_get_boottime();
	if (atomic_read(&sensor->accel_en))
		queue_work(sensor->data_wq, &sensor->accel_poll_work);

	ktime = ktime_set(0,sensor->accel_poll_ms * NSEC_PER_MSEC);
	hrtimer_start(&sensor->timer_acc, ktime, HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}
 
/**
 * mpu6500_gyro_work_fn() - polling gyro data
 * @work: the work struct
 *
 * Called by the work queue; read sensor data and generate an input
 * event
 */
static void mpu6500_gyro_work_fn(struct work_struct *work)
{
	struct mpu6500_sensor *sensor;
	struct axis_data data;
//	ktime_t timestamp;

	sensor = container_of(work,struct mpu6500_sensor, gyro_poll_work);

	if (sensor->sensor_power == false)
    {
        mpu6500_set_power_mode(sensor, true);
    }
	mpu6500_read_gyro_data(sensor, &data);
//	timestamp = ktime_get_boottime();
 //   printk(KERN_INFO TAGI "%s mpu6500_gyro_work_fn sensor->gyro_poll_ms=%d\n", __func__, sensor->gyro_poll_ms);
//	shift = mpu_gyro_fs_shift[sensor->cfg.fsr];
	input_report_abs(sensor->gyro_dev, ABS_X, data.x-sensor->gyro_calibration_data.x);
	input_report_abs(sensor->gyro_dev, ABS_Y, data.y-sensor->gyro_calibration_data.y);
	input_report_abs(sensor->gyro_dev, ABS_Z, data.z-sensor->gyro_calibration_data.z);
	input_event(sensor->gyro_dev,EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(sensor->ktime_gyro).tv_sec);
	input_event(sensor->gyro_dev,EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(sensor->ktime_gyro).tv_nsec);
//	printk(KERN_INFO TAGI "%s read time_s=%ld, time_ns=%ld \n",
//		__func__, ktime_to_timespec(timestamp).tv_sec,ktime_to_timespec(timestamp).tv_nsec);
	input_sync(sensor->gyro_dev);

//	if (atomic_read(&sensor->gyro_en))
//		queue_delayed_work(sensor->data_wq,&sensor->gyro_poll_work,
//			msecs_to_jiffies(sensor->gyro_poll_ms));
}
 
/**
 * mpu6500_gyro_timer_handle() - timer_gyro handle to schedule workerthread
 * @hrtimer: the hrtimer struct
 *
 * handle gets called based on the poll time to schedule worker thread
 */
static enum hrtimer_restart mpu6500_gyro_timer_handle(struct hrtimer *hrtimer)
{
	ktime_t ktime;
	struct mpu6500_sensor *sensor;
	sensor = container_of(hrtimer, struct mpu6500_sensor, timer_gyro);

	ktime = sensor->ktime_gyro =ktime_get_boottime();
	if (atomic_read(&sensor->gyro_en))
		queue_work(sensor->data_wq, &sensor->gyro_poll_work);

	ktime = ktime_set(0,sensor->gyro_poll_ms * NSEC_PER_MSEC);
	hrtimer_start(&sensor->timer_gyro, ktime, HRTIMER_MODE_REL);

    return HRTIMER_NORESTART;
}

/**
 *  mpu6500_set_lpa_freq() - set low power wakeup frequency.
 */
static int mpu6500_set_lpa_freq(struct mpu6500_sensor *sensor, int lpa_freq)
{
	int ret;
	u8 data;

	/* only for MPU6500 with fixed rate, need expend */
	if (INV_MPU6050 == sensor->chip_type) {
		ret = i2c_smbus_read_byte_data(sensor->client,
				sensor->reg.pwr_mgmt_2);
		if (ret < 0)
			return ret;

		data = (u8)ret;
		data &= ~BIT_LPA_FREQ_MASK;
		data |= MPU6050_LPA_5HZ;
		ret = i2c_smbus_write_byte_data(sensor->client,
				sensor->reg.pwr_mgmt_2, data);
		if (ret < 0)
			return ret;
	}
	sensor->cfg.lpa_freq = lpa_freq;

	return 0;
}

static int mpu6500_switch_engine(struct mpu6500_sensor *sensor,
				bool en, u32 mask)
{
	struct mpu_reg_map *reg;
	u8 data; 
	u8 mgmt_1;
	int ret;

	reg = &sensor->reg;
	/*
	 * switch clock needs to be careful. Only when gyro is on, can
	 * clock source be switched to gyro. Otherwise, it must be set to
	 * internal clock
	 */
//	printk(KERN_INFO TAGI "%s : en=%d ,mask=%d \n", __func__,en,mask);  //wtl add for gyro
	mgmt_1 = MPU_CLK_INTERNAL;
	if (BIT_PWR_GYRO_STBY_MASK == mask) {
		ret = i2c_smbus_read_byte_data(sensor->client,
			reg->pwr_mgmt_1);
		if (ret < 0)
			goto error;
		mgmt_1 = (u8)ret;
		mgmt_1 &= ~BIT_CLK_MASK;
	}

	if ((BIT_PWR_GYRO_STBY_MASK == mask) && (!en)) {
		/*
		 * turning off gyro requires switch to internal clock first.
		 * Then turn off gyro engine
		 */
		mgmt_1 |= MPU_CLK_INTERNAL;
		ret = i2c_smbus_write_byte_data(sensor->client,
			reg->pwr_mgmt_1, mgmt_1);
		if (ret < 0)
			goto error;
	}

	ret = i2c_smbus_read_byte_data(sensor->client,
			reg->pwr_mgmt_2);
	if (ret < 0)
		goto error;
	data = (u8)ret;
	if (en)
		data &= (~mask);
	else
	{
		if (BIT_PWR_GYRO_STBY_MASK == mask)
			data |= mask;
	}
	ret = i2c_smbus_write_byte_data(sensor->client,
			reg->pwr_mgmt_2, data);
	if (ret < 0)
		goto error;

	if ((BIT_PWR_GYRO_STBY_MASK == mask) && en) {
		/* wait gyro stable */
		msleep(SENSOR_UP_TIME_MS);
		/* after gyro is on & stable, switch internal clock to PLL */
		mgmt_1 |= MPU_CLK_PLL_X;
		ret = i2c_smbus_write_byte_data(sensor->client,
				reg->pwr_mgmt_1, mgmt_1);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	printk(KERN_ERR TAGE "%s Fail to switch MPU engine\n",__func__);
	return ret;
}

static int mpu6500_init_engine(struct mpu6500_sensor *sensor)
{
	int ret;

	ret = mpu6500_switch_engine(sensor, false, BIT_PWR_GYRO_STBY_MASK);
	if (ret)
		return ret;

	ret = mpu6500_switch_engine(sensor, false, BIT_PWR_ACCEL_STBY_MASK);
	if (ret)
		return ret;

	return 0;
}

/**
 * mpu6500_set_power_mode() - set the power mode
 * @sensor: sensor data structure
 * @power_on: value to switch on/off of power, 1: normal power,
 *    0: low power
 *
 * Put device to normal-power mode or low-power mode.
 */
static int mpu6500_set_power_mode(struct mpu6500_sensor *sensor,
					bool power_on)
{
	struct i2c_client *client = sensor->client;
	s32 ret;
	u8 val;
	
	printk(KERN_INFO TAGI "%s power_on = %d\n",__func__,power_on); 
//	if (power_on == sensor->sensor_power)
//    {
//        printk(KERN_INFO TAGI "Sensor power status is newest!\n");
//			return 0;
//    }
	ret = i2c_smbus_read_byte_data(client, sensor->reg.pwr_mgmt_1);
	if (ret < 0) {
		printk(KERN_ERR TAGE "Fail to read power mode, ret=%d\n", ret);
		return ret;
	}

	if (power_on)
		val = (u8)ret & ~BIT_SLEEP;
	else
		val = (u8)ret | BIT_SLEEP;
	ret = i2c_smbus_write_byte_data(client, sensor->reg.pwr_mgmt_1, val);
	if (ret < 0) {
		printk(KERN_ERR TAGE "Fail to write power mode, ret=%d\n", ret);
		return ret;
	}
	sensor->sensor_power = power_on;
	return 0;
}

/* weitianlei add for ts use acc data 
   don't consider the synchronization 
   for if write then the gsensor has suspend
   won't has other's control at the same time
   if read has make some synchronization condition happened 
   the effect is not serios for just sleep once all recover */

#define Y_MAX_THRESHOLD	(-500)
#define Z_MAX_THRESHOLD	500
#define Z_MIN_THRESHOLD	(-700)
extern bool (*acc_for_ts_judge_dir)(void);
static struct wake_lock ts_judge_phone_direction_wakelock;
bool mpu6500_for_ts_judge_dir(void)
{
	int result = 0;
	struct i2c_client *client = mpu6500_i2c_client;	
	struct axis_data data;
	struct mpu6500_sensor *sensor;

    if(!client)
		return false;
	sensor = i2c_get_clientdata(client);
	wake_lock_timeout(&ts_judge_phone_direction_wakelock, HZ/2); 
	mpu6500_set_power_mode(sensor,true);
	msleep(20);
	result = mpu6500_read_accel_data(sensor, &data);
	msleep(10);
	result = mpu6500_read_accel_data(sensor, &data);
	if (result < 0)
	{
		printk(KERN_INFO TAGI "<<-GTP->>%s get_acceleration_data failed\n", __func__);
		return false;
	}	

	printk(KERN_INFO TAGI "<<-GTP-INFO->>%s data is X(%d) Y(%d) Z(%d)\n", __func__, data.x,data.y, data.z);

	if (data.y < Y_MAX_THRESHOLD && (data.z > Z_MIN_THRESHOLD && data.z < Z_MAX_THRESHOLD)) 
	{
		printk(KERN_INFO TAGI "<<-GTP-INFO->>%s The phone is handstand\n", __func__);
		return true;
	} else {
		printk(KERN_INFO TAGI "<<-GTP-INFO->>%s The phone is NOT handstand\n", __func__);
		return false;
	}

}

/* weitianlei add end */

static int mpu6500_gyro_enable(struct mpu6500_sensor *sensor, bool on)
{
	int ret;
	u8 data;
	
	printk(KERN_INFO TAGI "%s ,enable=%d\n",__func__,on);
	if (sensor->cfg.is_asleep) {
		printk(KERN_ERR TAGE "Fail to set gyro state, device is asleep.\n");
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(sensor->client,
				sensor->reg.pwr_mgmt_1);
	if (ret < 0) {
		printk(KERN_ERR TAGE "Fail to get sensor power state, ret=%d\n", ret);
		return ret;
	}

	data = (u8)ret;
	if (on) {
		ret = mpu6500_switch_engine(sensor, true, BIT_PWR_GYRO_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.gyro_enable = 1;

		data &= ~BIT_SLEEP;
		ret = i2c_smbus_write_byte_data(sensor->client,
				sensor->reg.pwr_mgmt_1, data);
		if (ret < 0) {
			printk(KERN_ERR TAGE "Fail to set sensor power state, ret=%d\n",ret);
			return ret;
		}
		sensor->sensor_power = true;
		sensor->cfg.enable = 1;
	} else {

		ret = mpu6500_switch_engine(sensor, false, BIT_PWR_GYRO_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.gyro_enable = 0;
		if (!sensor->cfg.accel_enable) {
			data |=  BIT_SLEEP;
			ret = i2c_smbus_write_byte_data(sensor->client,
					sensor->reg.pwr_mgmt_1, data);
			if (ret < 0) {
				printk(KERN_ERR TAGE "Fail to set sensor power state, ret=%d\n",ret);
				return ret;
			}
			sensor->sensor_power = false;
			sensor->cfg.enable = 0;
		}
	}
	return 0;
}

/**
 * mpu6500_restore_context() - update the sensor register context
 */

static int mpu6500_restore_context(struct mpu6500_sensor *sensor)
{
	struct mpu_reg_map *reg;
	struct i2c_client *client;
	int ret;
	u8 data, pwr_ctrl;

	client = sensor->client;
	reg = &sensor->reg;

	/* Save power state and wakeup device from sleep */
	ret = i2c_smbus_read_byte_data(client, reg->pwr_mgmt_1);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s read power ctrl failed.\n",__func__);
		goto exit;
	}
	pwr_ctrl = (u8)ret;

	ret = i2c_smbus_write_byte_data(client, reg->pwr_mgmt_1,
		BIT_WAKEUP_AFTER_RESET);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s wakeup sensor failed.\n",__func__);
		goto exit;
	}
	ret = i2c_smbus_write_byte_data(client, reg->gyro_config,
			sensor->cfg.fsr << GYRO_CONFIG_FSR_SHIFT);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s update fsr failed.\n",__func__);
		goto exit;
	}

	ret = i2c_smbus_write_byte_data(client, reg->lpf, sensor->cfg.lpf);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s update lpf failed.\n",__func__);
		goto exit;
	}

	ret = i2c_smbus_write_byte_data(client, reg->accel_config,
			(sensor->cfg.accel_fs << ACCL_CONFIG_FSR_SHIFT));
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s update accel_fs failed.\n",__func__);
		goto exit;
	}

	ret = i2c_smbus_write_byte_data(client, reg->sample_rate_div,0x07);
		//	sensor->cfg.rate_div);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s set sample_rate_div failed.\n",__func__);
		goto exit;
	}

	ret = i2c_smbus_read_byte_data(client, reg->fifo_en);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s read fifo_en failed.\n",__func__);
		goto exit;
	}

	data = (u8)ret;

	if (sensor->cfg.accel_fifo_enable)
		data |= BIT_ACCEL_FIFO;

	if (sensor->cfg.gyro_fifo_enable)
		data |= BIT_GYRO_FIFO;

	if (sensor->cfg.accel_fifo_enable || sensor->cfg.gyro_fifo_enable) {
		ret = i2c_smbus_write_byte_data(client, reg->fifo_en, data);
		if (ret < 0) {
			printk(KERN_ERR TAGE "%s write fifo_en failed.\n",__func__);
			goto exit;
		}
	}

	if (sensor->cfg.cfg_fifo_en) {
		/* Assume DMP and external I2C is not in use*/
		ret = i2c_smbus_write_byte_data(client, reg->user_ctrl,
				BIT_FIFO_EN);
		if (ret < 0) {
			printk(KERN_ERR TAGE "%s enable FIFO R/W failed.\n",__func__);
			goto exit;
		}
	}

	/* Accel and Gyro should set to standby by default */
	ret = i2c_smbus_write_byte_data(client, reg->pwr_mgmt_2,BIT_PWR_GYRO_STBY_MASK);
	//		BITS_PWR_ALL_AXIS_STBY);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s set pwr_mgmt_2 failed.\n",__func__);
		goto exit;
	}

	ret = mpu6500_set_lpa_freq(sensor, sensor->cfg.lpa_freq);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s set lpa_freq failed.\n",__func__);
		goto exit;
	}

	/*ret = i2c_smbus_write_byte_data(client, reg->int_pin_cfg,
			sensor->cfg.int_pin_cfg);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s set int_pin_cfg failed.\n",__func__);
		goto exit;
	}*/

	ret = i2c_smbus_write_byte_data(client, reg->pwr_mgmt_1,
		pwr_ctrl);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s write saved power state failed.\n",__func__);
		goto exit;
	}

	printk(KERN_INFO TAGI "%s restore context finished\n",__func__);

exit:
	return ret;
}

/**
 * mpu6500_reset_chip() - reset chip to default state
 */
static void mpu6500_reset_chip(struct mpu6500_sensor *sensor)
{
	struct i2c_client *client;
	int ret, i;

	client = sensor->client;

	ret = i2c_smbus_write_byte_data(client, sensor->reg.pwr_mgmt_1,
			BIT_RESET_ALL);
	if (ret < 0) {
		printk(KERN_ERR TAGE "Reset chip fail!\n");
		goto exit;
	}
	for (i = 0; i < MPU6500_RESET_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(sensor->client,
					sensor->reg.pwr_mgmt_1);
		if (ret < 0) {
			printk(KERN_ERR TAGE "Fail to get reset state ret=%d\n", ret);
			goto exit;
		}

		if ((ret & BIT_H_RESET) == 0) {
			printk(KERN_INFO TAGI "Chip reset success! i=%d\n", i);
			break;
		}

		usleep(MPU6500_RESET_SLEEP_US);
	}

exit:
	return;
}

static int mpu6500_gyro_set_enable(struct mpu6500_sensor *sensor, bool enable)
{
	int ret = 0;
	ktime_t ktime;
//	printk(KERN_INFO TAGI "%s: enable=%d\n", __func__, enable);
	mutex_lock(&sensor->op_lock);
	if (enable) {
	/*	if (!sensor->power_enabled) {
			ret = mpu6500_restore_context(sensor);
			if (ret < 0) {
				dev_err(&sensor->client->dev,
						"Failed to restore context\n");
				goto exit;
			}
		}*/

		ret = mpu6500_gyro_enable(sensor, true);
		if (ret) {
			printk(KERN_ERR TAGE "Fail to enable gyro engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}

		ret = mpu6500_config_sample_rate(sensor);
		if (ret < 0)
			 printk(KERN_ERR TAGE "%s Unable to update sampling rate! ret=%d\n", __func__,ret);

//		queue_delayed_work(sensor->data_wq,&sensor->gyro_poll_work,
//				msecs_to_jiffies(sensor->gyro_poll_ms));
		ktime = ktime_set(0,sensor->gyro_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->timer_gyro, ktime, HRTIMER_MODE_REL);
		atomic_set(&sensor->gyro_en, 1);
	} else {
		atomic_set(&sensor->gyro_en, 0);
		hrtimer_cancel(&sensor->timer_gyro);
//		cancel_delayed_work_sync(&sensor->gyro_poll_work);
		ret = mpu6500_gyro_enable(sensor, false);
		if (ret) {
			printk(KERN_ERR TAGE "%s Fail to disable gyro engine ret=%d\n", __func__, ret);
			ret = -EBUSY;
			goto exit;
		}

	}

exit:
	mutex_unlock(&sensor->op_lock);
	return ret;
}

/* Update sensor sample rate divider upon accel and gyro polling rate. */
static int mpu6500_config_sample_rate(struct mpu6500_sensor *sensor)
{
	int ret;
	u32 delay_ms;
	u8 div, saved_pwr;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	if (sensor->accel_poll_ms <= sensor->gyro_poll_ms)
		delay_ms = sensor->accel_poll_ms;
	else
		delay_ms = sensor->gyro_poll_ms;

	/* Sample_rate = internal_ODR/(1+SMPLRT_DIV) */
	if ((sensor->cfg.lpf != MPU_DLPF_256HZ_NOLPF2) &&
		(sensor->cfg.lpf != MPU_DLPF_RESERVED)) {
		if (delay_ms > DELAY_MS_MAX_DLPF)
			delay_ms = DELAY_MS_MAX_DLPF;
		if (delay_ms < DELAY_MS_MIN_DLPF)
			delay_ms = DELAY_MS_MIN_DLPF;

		div = (u8)(((ODR_DLPF_ENA * delay_ms) / MSEC_PER_SEC) - 1);
	} else {
		if (delay_ms > DELAY_MS_MAX_NODLPF)
			delay_ms = DELAY_MS_MAX_NODLPF;
		if (delay_ms < DELAY_MS_MIN_NODLPF)
			delay_ms = DELAY_MS_MIN_NODLPF;
		div = (u8)(((ODR_DLPF_DIS * delay_ms) / MSEC_PER_SEC) - 1);
	}

//	if (sensor->cfg.rate_div == div)
//		return 0;

	ret = i2c_smbus_read_byte_data(sensor->client, sensor->reg.pwr_mgmt_1);
	if (ret < 0)
		goto err_exit;

	saved_pwr = (u8)ret;

	ret = i2c_smbus_write_byte_data(sensor->client, sensor->reg.pwr_mgmt_1,
		(saved_pwr & ~BIT_SLEEP));
	if (ret < 0)
		goto err_exit;

	ret = i2c_smbus_write_byte_data(sensor->client,
	sensor->reg.sample_rate_div, 0x07);
	//	sensor->reg.sample_rate_div, div);
	if (ret < 0)
		goto err_exit;

	ret = i2c_smbus_write_byte_data(sensor->client, sensor->reg.pwr_mgmt_1,
		saved_pwr);
	if (ret < 0)
		goto err_exit;

	sensor->cfg.rate_div = div;

	return 0;
err_exit:
	printk(KERN_ERR TAGE "%s update sample div failed, div=%d, ret=%d\n",__func__,div, ret);
	return ret;
}

static int mpu6500_gyro_set_poll_delay(struct mpu6500_sensor *sensor,
					unsigned long delay)
{
	int ret;
	ktime_t ktime;
	
	printk(KERN_INFO TAGI "%s delay=%ld\n", __func__, delay);
	
	if (delay < MPU6500_GYRO_MIN_POLL_INTERVAL_MS)
		delay = MPU6500_GYRO_MIN_POLL_INTERVAL_MS;
	if (delay > MPU6500_GYRO_MAX_POLL_INTERVAL_MS)
		delay = MPU6500_GYRO_MAX_POLL_INTERVAL_MS;

	mutex_lock(&sensor->op_lock);
	if (sensor->gyro_poll_ms == delay)
		goto exit;

	sensor->gyro_poll_ms = delay;
//	printk(KERN_INFO TAGI "%s sensor->gyro_poll_ms=%d\n", __func__, sensor->gyro_poll_ms);
	if (!atomic_read(&sensor->gyro_en))
		goto exit;

	if (sensor->use_poll) {
//		cancel_delayed_work_sync(&sensor->gyro_poll_work);
//		queue_delayed_work(sensor->data_wq,&sensor->gyro_poll_work,
//				msecs_to_jiffies(sensor->gyro_poll_ms));
		hrtimer_cancel(&sensor->timer_gyro);
		ktime = ktime_set(0,sensor->gyro_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->timer_gyro, ktime, HRTIMER_MODE_REL);
	} else {
		ret = mpu6500_config_sample_rate(sensor);
		if (ret < 0)
			printk(KERN_ERR TAGE "Unable to set polling delay for gyro!\n");
	}

exit:
	mutex_unlock(&sensor->op_lock);
	return 0;
}

static int mpu6500_gyro_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct mpu6500_sensor *sensor = container_of(sensors_cdev,
			struct mpu6500_sensor, gyro_cdev);

	return mpu6500_gyro_set_enable(sensor, enable);
}

static int mpu6500_gyro_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int delay_ms)
{
	struct mpu6500_sensor *sensor = container_of(sensors_cdev,
			struct mpu6500_sensor, gyro_cdev);

	return mpu6500_gyro_set_poll_delay(sensor, delay_ms);
}

/**
 * mpu6500_gyro_attr_get_polling_delay() - get the sampling rate
 */
static ssize_t mpu6500_gyro_attr_get_polling_delay(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int val;
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);

	val = sensor ? sensor->gyro_poll_ms : 0;
	return snprintf(buf, 8, "%d\n", val);
}

/**
 * mpu6500_gyro_attr_set_polling_delay() - set the sampling rate
 */
static ssize_t mpu6500_gyro_attr_set_polling_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int ret;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	ret = mpu6500_gyro_set_poll_delay(sensor, interval_ms);

	return ret ? -EBUSY : size;
}

static ssize_t mpu6500_gyro_attr_get_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);

	return snprintf(buf, 4, "%d\n", sensor->cfg.gyro_enable);
}

/**
 * mpu6500_gyro_attr_set_enable() -
 *    Set/get enable function is just needed by sensor HAL.
 */
static ssize_t mpu6500_gyro_attr_set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ret = mpu6500_gyro_set_enable(sensor, true);
	else
		ret = mpu6500_gyro_set_enable(sensor, false);

	return ret ? -EBUSY : count;
}

static struct device_attribute gyro_attr[] = {
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		mpu6500_gyro_attr_get_polling_delay,
		mpu6500_gyro_attr_set_polling_delay),
	__ATTR(enable, S_IRUGO | S_IWUSR,
		mpu6500_gyro_attr_get_enable,
		mpu6500_gyro_attr_set_enable),
};

static int create_gyro_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(gyro_attr); i++) {
		err = device_create_file(dev, gyro_attr + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, gyro_attr + i);
	printk(KERN_ERR TAGE "%s Unable to create interface\n", __func__);
	return err;
}

static int remove_gyro_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(gyro_attr); i++)
		device_remove_file(dev, gyro_attr + i);
	return 0;
}

static int mpu6500_accel_enable(struct mpu6500_sensor *sensor, bool on)
{
	int ret;
	u8 data;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(sensor->client,
				sensor->reg.pwr_mgmt_1);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s Fail to get sensor power state, ret=%d\n", __func__, ret);
		return ret;
	}

	data = (u8)ret;
	if (on) {
		ret = mpu6500_switch_engine(sensor, true, BIT_PWR_ACCEL_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.accel_enable = 1;

		data &= ~BIT_SLEEP;
		ret = i2c_smbus_write_byte_data(sensor->client,
				sensor->reg.pwr_mgmt_1, data);
		if (ret < 0) {
			printk(KERN_ERR TAGE "%s Fail to set sensor power state, ret=%d\n", __func__,ret);
			return ret;
		}

		sensor->cfg.enable = 1;
		sensor->sensor_power = true;
	} else {
		ret = mpu6500_switch_engine(sensor, false, BIT_PWR_ACCEL_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.accel_enable = 0;

		if (!sensor->cfg.gyro_enable) {
			data |=  BIT_SLEEP;
			ret = i2c_smbus_write_byte_data(sensor->client,
					sensor->reg.pwr_mgmt_1, data);
			if (ret < 0) {
				printk(KERN_ERR TAGE "%s Fail to set sensor power state for accel, ret=%d\n", __func__,ret);
				return ret;
			}
			sensor->cfg.enable = 0;
			sensor->sensor_power = false;
		}
	}
	return 0;
}

static int mpu6500_accel_set_enable(struct mpu6500_sensor *sensor, bool enable)
{
	int ret = 0;
	ktime_t ktime;
	
	printk(KERN_INFO TAGI "%s: enable=%d\n", __func__, enable);
	mutex_lock(&sensor->op_lock);
	if (enable) {
	/*	if (!sensor->power_enabled) {

			ret = mpu6500_restore_context(sensor);
			if (ret < 0) {
				dev_err(&sensor->client->dev,
					"Failed to restore context");
				goto exit;
			}
		}*/

		ret = mpu6500_accel_enable(sensor, true);
		if (ret) {
			printk(KERN_ERR TAGE "Fail to enable accel engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}

		ret = mpu6500_config_sample_rate(sensor);
		if (ret < 0)
			 printk(KERN_ERR TAGE "Unable to update sampling rate! ret=%d\n",ret);

//		queue_delayed_work(sensor->data_wq,&sensor->accel_poll_work,
//				msecs_to_jiffies(sensor->accel_poll_ms));
		ktime = ktime_set(0,sensor->accel_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->timer_acc, ktime, HRTIMER_MODE_REL);
		atomic_set(&sensor->accel_en, 1);
	} else {
		atomic_set(&sensor->accel_en, 0);
		hrtimer_cancel(&sensor->timer_acc);
//		cancel_delayed_work_sync(&sensor->accel_poll_work);

		ret = mpu6500_accel_enable(sensor, false);
		if (ret) {
			printk(KERN_ERR TAGE "Fail to disable accel engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}

	}

exit:
	mutex_unlock(&sensor->op_lock);
	return ret;
}

static int mpu6500_accel_set_poll_delay(struct mpu6500_sensor *sensor,
					unsigned long delay)
{
	int ret;
	ktime_t ktime;
	
	printk(KERN_INFO TAGI "%s: delay_ms=%ld\n",__func__, delay);
	if (delay < MPU6500_ACCEL_MIN_POLL_INTERVAL_MS)
		delay = MPU6500_ACCEL_MIN_POLL_INTERVAL_MS;
	if (delay > MPU6500_ACCEL_MAX_POLL_INTERVAL_MS)
		delay = MPU6500_ACCEL_MAX_POLL_INTERVAL_MS;

	mutex_lock(&sensor->op_lock);
	if (sensor->accel_poll_ms == delay)
		goto exit;

	sensor->accel_poll_ms = delay;

	if (!atomic_read(&sensor->accel_en))
		goto exit;

	if (sensor->use_poll) {
//		cancel_delayed_work_sync(&sensor->accel_poll_work);
//		queue_delayed_work(sensor->data_wq,&sensor->accel_poll_work,
//				msecs_to_jiffies(sensor->accel_poll_ms));
		hrtimer_cancel(&sensor->timer_acc);
		ktime = ktime_set(0,sensor->accel_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->timer_acc, ktime, HRTIMER_MODE_REL);
	} else {
		ret = mpu6500_config_sample_rate(sensor);
		if (ret < 0)
			printk(KERN_ERR TAGE "Unable to set polling delay for accel!\n");
	}

exit:
	mutex_unlock(&sensor->op_lock);
	return 0;
}

static int mpu6500_accel_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct mpu6500_sensor *sensor = container_of(sensors_cdev,
			struct mpu6500_sensor, accel_cdev);

	return mpu6500_accel_set_enable(sensor, enable);
}

static int mpu6500_accel_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int delay_ms)
{
	struct mpu6500_sensor *sensor = container_of(sensors_cdev,
			struct mpu6500_sensor, accel_cdev);

	return mpu6500_accel_set_poll_delay(sensor, delay_ms);
}

/**
 * mpu6500_accel_attr_get_polling_delay() - get the sampling rate
 */
static ssize_t mpu6500_accel_attr_get_polling_delay(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int val;
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);

	val = sensor ? sensor->accel_poll_ms : 0;
	return snprintf(buf, 8, "%d\n", val);
}

/**
 * mpu6500_accel_attr_set_polling_delay() - set the sampling rate
 */
static ssize_t mpu6500_accel_attr_set_polling_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int ret;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	ret = mpu6500_accel_set_poll_delay(sensor, interval_ms);

	return ret ? -EBUSY : size;
}

static ssize_t mpu6500_accel_attr_get_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);

	return snprintf(buf, 4, "%d\n", sensor->cfg.accel_enable);
}

/**
 * mpu6500_accel_attr_set_enable() -
 *    Set/get enable function is just needed by sensor HAL.
 */

static ssize_t mpu6500_accel_attr_set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ret = mpu6500_accel_set_enable(sensor, true);
	else
		ret = mpu6500_accel_set_enable(sensor, false);

	return ret ? -EBUSY : count;
}

#ifdef DEBUG_NODE
u8 mpu6500_address;
u8 mpu6500_data;

static ssize_t mpu6500_accel_attr_get_reg_addr(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, 8, "%d\n", mpu6500_address);
}

static ssize_t mpu6500_accel_attr_set_reg_addr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long addr;

	if (kstrtoul(buf, 10, &addr))
		return -EINVAL;
	if ((addr < 0) || (addr > 255))
		return -EINVAL;

	mpu6500_address = addr;
	printk(KERN_INFO TAGI "mpu6500_address =%d\n", mpu6500_address);

	return size;
}

static ssize_t mpu6500_accel_attr_get_data(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);
	int ret;

	ret = i2c_smbus_read_byte_data(sensor->client, mpu6500_address);
	printk(KERN_INFO TAGI "read addr(0x%x)=0x%x\n", mpu6500_address, ret);
	if (ret >= 0 && ret <= 255)
		mpu6500_data = ret;
	return snprintf(buf, 8, "0x%x\n", ret);
}

static ssize_t mpu6500_accel_attr_set_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long reg_data;

	if (kstrtoul(buf, 10, &reg_data))
		return -EINVAL;
	if ((reg_data < 0) || (reg_data > 255))
		return -EINVAL;

	mpu6500_data = reg_data;
	printk(KERN_INFO TAGI "set mpu6500_data =0x%x\n", mpu6500_data);

	return size;
}
static ssize_t mpu6500_accel_attr_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct mpu6500_sensor *sensor = dev_get_drvdata(dev);
	int ret;

	ret = i2c_smbus_write_byte_data(sensor->client,
		mpu6500_address, mpu6500_data);
	printk(KERN_INFO TAGI "write addr(0x%x)<-0x%x ret=%d\n",
		mpu6500_address, mpu6500_data, ret);

	return size;
}

#endif

static struct device_attribute accel_attr[] = {
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		mpu6500_accel_attr_get_polling_delay,
		mpu6500_accel_attr_set_polling_delay),
	__ATTR(enable, S_IRUGO | S_IWUSR,
		mpu6500_accel_attr_get_enable,
		mpu6500_accel_attr_set_enable),
#ifdef DEBUG_NODE
	__ATTR(addr, S_IRUSR | S_IWUSR,
		mpu6500_accel_attr_get_reg_addr,
		mpu6500_accel_attr_set_reg_addr),
	__ATTR(reg, S_IRUSR | S_IWUSR,
		mpu6500_accel_attr_get_data,
		mpu6500_accel_attr_set_data),
	__ATTR(write, S_IWUSR,
		NULL,
		mpu6500_accel_attr_reg_write),
#endif
};

static int create_accel_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(accel_attr); i++) {
		err = device_create_file(dev, accel_attr + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, accel_attr + i);
	printk(KERN_ERR TAGE "%s Unable to create interface\n",__func__);
	return err;
}

static int remove_accel_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(accel_attr); i++)
		device_remove_file(dev, accel_attr + i);
	return 0;
}

static ssize_t mpu6500_accel_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	struct axis_data data,accel;
	
	mpu6500_switch_engine(sensor, true, BIT_PWR_ACCEL_STBY_MASK);
	if (sensor->sensor_power == false)
    {
        mpu6500_set_power_mode(sensor, true);
		mpu6500_config_sample_rate(sensor);
		msleep(20);
		mpu6500_read_accel_data(sensor, &data);
		msleep(10);
    }
	mpu6500_read_accel_data(sensor, &data);
	accel = sensor->accel_lastraw;
	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

static ssize_t mpu6500_accel_final_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	struct axis_data data,accel;

	printk(KERN_INFO TAGI "%s wtl sensor_power = %d!!!\n",__func__,sensor->sensor_power); 
	mpu6500_switch_engine(sensor, true, BIT_PWR_ACCEL_STBY_MASK);
    if (sensor->sensor_power == false)
    {
        mpu6500_set_power_mode(sensor, true);
		mpu6500_config_sample_rate(sensor);
		msleep(20);
		mpu6500_read_accel_data(sensor, &data);
		msleep(10);
    }
	mpu6500_read_accel_data(sensor, &data);
	accel = sensor->accel_lastdata;
	printk(KERN_INFO TAGI "%s wtl x = %d,y = %d,z = %d!!!\n",__func__, accel.x, accel.y, accel.z);
	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

static ssize_t accel_sensor_calibratecmd_xy(struct mpu6500_sensor *sensor)
{	
	s16 i,sumofx,sumofy,sumofz;
	struct axis_data data;
    unsigned long delay = 20;
	sumofx = 0;
	sumofy = 0;
	sumofz = 0;
	printk(KERN_INFO TAGI "%s begin!\n",__func__);
	
	for (i=0;i<50;i++)
	{
        mpu6500_read_accel_data(sensor, &data);
		sumofx = sumofx + data.x;		
		sumofy = sumofy + data.y;		
		sumofz = sumofz + data.z-1024;	
		mdelay(delay + 1);
	}
	sensor->accel_calibration_data.x = sumofx / 50;
	sensor->accel_calibration_data.y = sumofy / 50;
	sensor->accel_calibration_data.z = sumofz / 50;
	printk(KERN_INFO TAGI "accel_calibration_dataX:%d  accel_calibration_dataY:%d  accel_calibration_dataZ:%d\n",
	sensor->accel_calibration_data.x, sensor->accel_calibration_data.y, sensor->accel_calibration_data.z);
	
	return 0;
}

static ssize_t accel_sensor_calibratecmd_z(struct mpu6500_sensor *sensor)
{
	int i,sumofz;
	struct axis_data data;
    unsigned long delay = 50;
	sumofz = 0;
	printk(KERN_INFO TAGI "%s begin!\n",__func__);
	for (i=0;i<50;i++)
	{
        mpu6500_read_accel_data(sensor, &data);
		sumofz = sumofz + data.z;
		printk(KERN_INFO TAGI "data.z = %d\n",data.z);
		mdelay(delay + 1);
	}
	sensor->accel_calibration_data.z = sumofz / 50;
	printk(KERN_INFO TAGI "accel_sensor_calibratecmd_z = %d\n",sensor->accel_calibration_data.z);
	return 0;
}

int inv_i2c_single_write(struct i2c_client *client, u8 reg, u8 data)
{
    u8 databuf[2] = {0};
    int res = 0;

    databuf[1] = data;
    databuf[0] = reg;

	res = i2c_smbus_write_byte_data(client, reg, data);

    if (res < 0)
    {
        return MPU6500_ERR_I2C;
    }
    else
    {
        return MPU6500_SUCCESS;
    }

}

/**
* inv_check_6500_accel_self_test() - check 6500 accel self test. this function
*                                   returns zero as success. A non-zero return
*                                   value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg:  average value of self test
*/
static int inv_check_6500_accel_self_test(struct inv_selftest_device *st,
						int *reg_avg, int *st_avg) {
	int ret_val, result;
	int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
	u8 regs[3];
	int otp_value_zero = 0;
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
#define ACCEL_ST_AL_MIN ((DEF_ACCEL_ST_AL_MIN * DEF_ST_SCALE \
				 / DEF_ST_6500_ACCEL_FS_MG) * DEF_ST_PRECISION)
#define ACCEL_ST_AL_MAX ((DEF_ACCEL_ST_AL_MAX * DEF_ST_SCALE \
				 / DEF_ST_6500_ACCEL_FS_MG) * DEF_ST_PRECISION)

	ret_val = 0;
	
	result = mpu6500_read_reg(sensor, REG_6500_XA_ST_DATA, regs, 3);
	if (result)
		return result;
	 printk(KERN_INFO TAGI "%s self_test accel shift_code - %02x %02x %02x\n",
		 st->name, regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	 printk(KERN_INFO TAGI "%s self_test accel st_shift_prod - %+d %+d %+d\n",
		 st->name, st_shift_prod[0], st_shift_prod[1],
		 st_shift_prod[2]);

	if (!otp_value_zero) {
		/* Self Test Pass/Fail Criteria A */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = st_avg[i] - reg_avg[i];
			st_shift_ratio[i] = abs(st_shift_cust[i] /
					st_shift_prod[i] - DEF_ST_PRECISION);
			if (st_shift_ratio[i] > DEF_6500_ACCEL_ST_SHIFT_DELTA)
			{
				ret_val = 1;
				 printk(KERN_ERR TAGE "%s, Fail, A\n", __func__);
			}
		}
	} else {
		/* Self Test Pass/Fail Criteria B */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = abs(st_avg[i] - reg_avg[i]);
			if (st_shift_cust[i] < ACCEL_ST_AL_MIN ||
					st_shift_cust[i] > ACCEL_ST_AL_MAX)
			{
				ret_val = 1;
				 printk(KERN_ERR TAGE "%s, Fail, B\n", __func__);
			}
		}
	}
	 printk(KERN_INFO TAGI "%s self_test accel st_shift_cust - %+d %+d %+d\n",
		 st->name, st_shift_cust[0], st_shift_cust[1],
		 st_shift_cust[2]);

	return ret_val;
}

/*
 *  inv_do_test() - do the actual test of self testing
 */
static int inv_do_test(struct inv_selftest_device *st, int self_test_flag,
		int *gyro_result, int *accel_result)
{
	int result, i, j, packet_size;
	u8 data[BYTES_PER_SENSOR * 2], d;
	int fifo_count, packet_count, ind, s;
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	packet_size = BYTES_PER_SENSOR * 2;

	result = inv_i2c_single_write(client, REG_INT_ENABLE, 0);
	if (result)
		return result;
	/* disable the sensor output to FIFO */
	result = inv_i2c_single_write(client, REG_FIFO_EN, 0);
	if (result)
		return result;
	/* disable fifo reading */
	result = inv_i2c_single_write(client, REG_USER_CTRL, 0);
	if (result)
		return result;
	/* clear FIFO */
	result = inv_i2c_single_write(client, REG_USER_CTRL, BIT_FIFO_RST);
	if (result)
		return result;
	/* setup parameters */
	result = inv_i2c_single_write(client, REG_CONFIG, MPU_DLPF_98HZ);
	if (result)
		return result;

	if (INV_MPU6500 == st->chip_type) {
		/* config accel LPF register for MPU6500 */
		result = inv_i2c_single_write(client, REG_6500_ACCEL_CONFIG2,
						DEF_ST_MPU6500_ACCEL_LPF |
						BIT_FIFO_SIZE_1K);
		if (result)
			return result;
	}

	result = inv_i2c_single_write(client, REG_SAMPLE_RATE_DIV,
			DEF_SELFTEST_SAMPLE_RATE);
	if (result)
		return result;
	/* wait for the sampling rate change to stabilize */
	mdelay(INV_MPU_SAMPLE_RATE_CHANGE_STABLE);
	result = inv_i2c_single_write(client, REG_GYRO_CONFIG,
		self_test_flag | DEF_SELFTEST_GYRO_FS);
	if (result)
		return result;
	if (INV_MPU6500 == st->chip_type)
		d = DEF_SELFTEST_6500_ACCEL_FS;
	else
		d = DEF_SELFTEST_ACCEL_FS;
	d |= self_test_flag;
	result = inv_i2c_single_write(client, REG_ACCEL_CONFIG, d);
	if (result)
		return result;
	/* wait for the output to get stable */
	if (self_test_flag) {
		if (INV_MPU6500 == st->chip_type)
			msleep(DEF_ST_6500_STABLE_TIME);
		else
			msleep(DEF_ST_STABLE_TIME);
	}

	/* enable FIFO reading */
	result = inv_i2c_single_write(client, REG_USER_CTRL, BIT_FIFO_EN);
	if (result)
		return result;
	/* enable sensor output to FIFO */
	d = BITS_GYRO_OUT | BIT_ACCEL_OUT;
	for (i = 0; i < THREE_AXIS; i++) {
		gyro_result[i] = 0;
		accel_result[i] = 0;
	}
	s = 0;
	while (s < st->samples) {
		result = inv_i2c_single_write(client, REG_FIFO_EN, d);
		if (result)
			return result;
		mdelay(DEF_GYRO_WAIT_TIME);
		result = inv_i2c_single_write(client, REG_FIFO_EN, 0);
		if (result)
			return result;
		result = mpu6500_read_reg(sensor,REG_FIFO_COUNT_H,
			       data, FIFO_COUNT_BYTE);
		if (result)
			return result;
		fifo_count = be16_to_cpup((__be16 *)(&data[0]));
		 printk(KERN_INFO TAGI "%s self_test fifo_count - %d\n",
			 st->name, fifo_count);
		packet_count = fifo_count / packet_size;
		i = 0;
		while ((i < packet_count) && (s < st->samples)) {
			short vals[3];
//			u8 pwr_mgmt_2; //wtl add for gyro
			result = mpu6500_read_reg(sensor,REG_FIFO_R_W,
			       data, packet_size/2);
			if (result)
				return result;
			ind = 0;
			for (j = 0; j < THREE_AXIS; j++) {
				vals[j] = (short)be16_to_cpup(
				    (__be16 *)(&data[ind + 2 * j]));
				accel_result[j] += vals[j];
			}
			ind += BYTES_PER_SENSOR;
//			 printk(KERN_INFO TAGI "%s self_test accel data - %d %+d %+d %+d \n",
//			    st->name, s, vals[0], vals[1], vals[2]);
		//wtl add for gyro start
//			result = mpu6500_read_reg(sensor,REG_PWR_MGMT_2,
//			       &pwr_mgmt_2, 1);
//		    printk(KERN_INFO TAGI "%s self_test reg_pwr_mgmt_2 - %d %+d \n",
//			    st->name, s, pwr_mgmt_2);	
		//wtl add for gyro end		
			result = mpu6500_read_reg(sensor,REG_FIFO_R_W,
			       data+packet_size/2, packet_size/2);
			if (result)
				return result;
			for (j = 0; j < THREE_AXIS; j++) {
				vals[j] = (short)be16_to_cpup(
					(__be16 *)(&data[ind + 2 * j]));
				gyro_result[j] += vals[j];
			}
//			 printk(KERN_INFO TAGI "%s self_test gyro data - %d %+d %+d %+d \n",
//				 st->name, s, vals[0], vals[1], vals[2]);

			s++;
			i++;
		}
	}

	for (j = 0; j < THREE_AXIS; j++) {
		accel_result[j] = accel_result[j] / s;
		accel_result[j] *= DEF_ST_PRECISION;
	}
	for (j = 0; j < THREE_AXIS; j++) {
		gyro_result[j] = gyro_result[j] / s;
		gyro_result[j] *= DEF_ST_PRECISION;
	}

	return 0;
}

/*
 *  inv_store_setting() store the old settings before selftest
 */
static void inv_store_setting(struct inv_selftest_device *st)
{
	int result;
	u8 data;
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	
	result = mpu6500_read_reg(sensor,REG_SAMPLE_RATE_DIV,&data,1);
	if (result)
		 printk(KERN_ERR TAGE "%s read REG_SAMPLE_RATE_DIV fail\n",st->name);
	else st->sample_rate_div = data;

	result = mpu6500_read_reg(sensor,REG_CONFIG,&data,1);
	if (result)
		 printk(KERN_ERR TAGE "%s read REG_CONFIG fail\n",st->name);
	else st->config = data;

	result = mpu6500_read_reg(sensor,REG_GYRO_CONFIG,&data,1);
	if (result)
		 printk(KERN_ERR TAGE "%s read REG_GYRO_CONFIG fail\n",st->name);
	else st->gyro_config = data;

	result = mpu6500_read_reg(sensor,REG_ACCEL_CONFIG,&data,1);
	if (result)
		 printk(KERN_ERR TAGE "%s read REG_ACCEL_CONFIG fail\n",st->name);
	else st->accel_config = data;
}

/*
 *  inv_recover_setting() recover the old settings after everything is done
 */
static void inv_recover_setting(struct inv_selftest_device *st)
{
	struct i2c_client *client = mpu6500_i2c_client;

	inv_i2c_single_write(client, REG_SAMPLE_RATE_DIV, st->sample_rate_div);
	inv_i2c_single_write(client, REG_CONFIG, st->config);
	inv_i2c_single_write(client, REG_GYRO_CONFIG, st->gyro_config);
	inv_i2c_single_write(client, REG_ACCEL_CONFIG, st->accel_config);
}

int mpu6500g_hw_self_test(struct inv_selftest_device *st)
{
	int result;
	int gyro_bias_st[THREE_AXIS], gyro_bias_regular[THREE_AXIS];
	int accel_bias_st[THREE_AXIS], accel_bias_regular[THREE_AXIS];
	int test_times, i;
	char compass_result, accel_result, gyro_result;

	inv_store_setting(st);
//	result = inv_power_up_self_test(st);
//	if (result)
//		return result;
	compass_result = 0;
	accel_result = 0;
	gyro_result = 0;
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(st, 0, gyro_bias_regular,
			accel_bias_regular);
		if (result == -EAGAIN)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		goto test_fail;
	 printk(KERN_INFO TAGI "%s self_test accel bias_regular - %+d %+d %+d\n",
		 st->name, accel_bias_regular[0],
		 accel_bias_regular[1], accel_bias_regular[2]);
	 printk(KERN_INFO TAGI "%s self_test gyro bias_regular - %+d %+d %+d\n",
		 st->name, gyro_bias_regular[0], gyro_bias_regular[1],
		 gyro_bias_regular[2]);

	for (i = 0; i < 3; i++) {
		st->gyro_bias[i] = gyro_bias_regular[i];
		st->accel_bias[i] = accel_bias_regular[i];
	}

	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(st, BITS_SELF_TEST_EN, gyro_bias_st,
					accel_bias_st);
		if (result == -EAGAIN)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;
	 printk(KERN_INFO TAGI "%s self_test accel bias_st - %+d %+d %+d\n",
		 st->name, accel_bias_st[0], accel_bias_st[1],
		 accel_bias_st[2]);
	 printk(KERN_INFO TAGI "%s self_test gyro bias_st - %+d %+d %+d\n",
		 st->name, gyro_bias_st[0], gyro_bias_st[1],
		 gyro_bias_st[2]);

	for (i = 0; i < 3; i++) {
		st->gyro_bias_st[i] = gyro_bias_st[i];
		st->accel_bias_st[i] = accel_bias_st[i];
	}
    //TBD compass selftest
//	if (st->chip_config.has_compass)
//		compass_result = !st->slave_compass->self_test(st);

	if (INV_MPU6500 == st->chip_type) {
		accel_result = !inv_check_6500_accel_self_test(st,
			accel_bias_regular, accel_bias_st);
	//	gyro_result = !inv_check_6500_gyro_self_test(st,
	//		gyro_bias_regular, gyro_bias_st);
	}

test_fail:
	inv_recover_setting(st);

	result = accel_result;
	printk(KERN_INFO TAGI "%s, result=%d\n", __func__, result);
	//1:PASS 0:FAIL
	return result;
}

static int MPU6500g_SelfTest(struct i2c_client *client)
{
    int testRes = 0;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	
    sensor->mpu_selftest_device.name = "MPU6XXX";
    sensor->mpu_selftest_device.chip_type = INV_MPU6500;//MPU6515 don't need to modify this chip_type
    sensor->mpu_selftest_device.samples = INIT_ST_SAMPLES;	
	
	testRes = mpu6500g_hw_self_test(&(sensor->mpu_selftest_device));
	
	return testRes;
}

static ssize_t accel_sensor_calibratecmd_selftest(struct mpu6500_sensor *sensor)
{
	struct i2c_client *client = mpu6500_i2c_client;
	
	sensor->accel_calibration_data.self_test = 2;
    sensor->accel_calibrate_process = 2;
	{
		sensor->accel_calibration_data.self_test = MPU6500g_SelfTest(client);
	}
	return 0;
}

static ssize_t accel_sensor_calibration_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
    struct calibrate_data value = {5,5,6,6}; 
    int i,j=0;
    char bufnew[30];
    printk(KERN_INFO TAGI "%s begin!\n",__func__);
    for(i=0;i<=30;i++)
    {	
        bufnew[i] = *(buf+i);
	    if(*(buf+i)=='\0')
	    break;
    }
    for(i=0;i<=30;i++)
    {	
        if(*(bufnew+i)==' ')
	    {	
	        *(bufnew+i)='\0';
	        value.x = simple_strtol(bufnew, NULL, 10);
	        *(bufnew+i)=' ';
	        j=i;
	        break;
	    }
    }
    i++;
    for(;i<=30;i++)
    {	
        if(*(bufnew+i)==' ')
	    {	
	        *(bufnew+i)='\0';
	        value.y = simple_strtol(bufnew+j+1, NULL, 10);
	        *(bufnew+i)=' ';
	        j=i;
	        break;
	    }
    }
    i++;
    for(;i<=30;i++)
    {	
        if(*(bufnew+i)==' ')
	    {	
	        *(bufnew+i)='\0';
		    value.z = simple_strtol(bufnew+j+1, NULL, 10);
		    *(bufnew+i)=' ';
		    j=i;	
		    break;
	    }
    }
    i++;
    for(;i<=30;i++)
    {	
        if(*(bufnew+i)==' ')
	    {	
	        *(bufnew+i)='\0';
	        value.self_test = simple_strtol(bufnew+j+1, NULL, 10);
	        *(bufnew+i)=' ';
	        j=i;	
	        break;
	    }
    }
    sensor->accel_calibration_data = value;
    return count;
}

static ssize_t accel_sensor_calibration_show(struct device_driver *dev_driver, char *buf)
{
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
    struct calibrate_data value;
	printk(KERN_INFO TAGI "%s begin!\n",__func__);
    value = sensor->accel_calibration_data;
    return sprintf(buf, "%d %d %d %d  ov\n", value.x,value.y,value.z,value.self_test);
}

static ssize_t accel_sensor_calibratecmd_store(struct device_driver *dev_driver, const char *buf, size_t count)
{  
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	int res;
	unsigned long cmd = simple_strtoul(buf, NULL, 10);
	printk(KERN_INFO TAGI "%s: cmd= %d\n",__func__,(int)cmd);
	res = mpu6500_set_power_mode(sensor,true);
	if(res < 0 ) //
	{
		printk(KERN_ERR TAGE "MPU6500 power on faild!\n");
	}
	if (cmd == 1)
	{	sensor->accel_calibrate_process = 1;
		accel_sensor_calibratecmd_xy(sensor);
		sensor->accel_calibrate_process = 0;
	}
	else if (cmd == 2) 
	{	sensor->accel_calibrate_process = 1;
		accel_sensor_calibratecmd_z(sensor);
		sensor->accel_calibrate_process = 0;
	}
	else if (cmd == 3) 
	{	sensor->accel_calibrate_process = 1;
		accel_sensor_calibratecmd_selftest(sensor);
		sensor->accel_calibrate_process = 0;
	}
	else
	{
	}
	res = mpu6500_set_power_mode(sensor, false);
	if(res < 0 ) //
	{
		printk(KERN_ERR TAGE "MPU6500 power down faild!\n");
	}
	return count;
}

static ssize_t accel_sensor_calibratecmd_show(struct device_driver *dev_driver, char *buf)
{
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
    int value;
  	printk(KERN_INFO TAGI "%s begin!\n",__func__);
    value = sensor->accel_calibrate_process;
    return sprintf(buf, "%d\n", value);
}

static void accel_sensor_eint_enable(int flag)
{
  //  u8 i = 0;
    u8 data;
//	int ret = -100;
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	
	if(flag)
    {
		gpio_switch_setstate(0);
		//make sure gsensor is running
		mpu6500_read_reg(sensor, 0x6b, &data , 1);
		if(data & 0x60)
		{
		    data = data & 0x9f;
			inv_i2c_single_write(client, 0x6b, data);
			mdelay(5);
		}
		
		mpu6500_read_reg(sensor, 0x6c, &data , 1);
		if(data & 0x38)
		{
		    data = data & 0xc7;
			inv_i2c_single_write(client, 0x6c, data);
			mdelay(5);
		}
		
//		mpu6500_read_reg(sensor, 0x1a, &data , 1);
//		data = data & 0xf8;
//		inv_i2c_single_write(client, 0x1a, data);
//		mdelay(5);
        //enable the WOM interrupt
        inv_i2c_single_write(client, 0x38, 0x40);
		mdelay(5);
		inv_i2c_single_write(client, 0x37, 0x20);
		mdelay(5);
        //enables the Wake-on-Motion detection logic
        //MPU-6500 mode - compare the current sample with the previous sample
        inv_i2c_single_write(client, 0x69, 0xc0);
		mdelay(5);
    
        //wake on motion threshold = 2*4mg
        inv_i2c_single_write(client, 0x1f, 0x19);
		mdelay(5);
		printk(KERN_INFO TAGI "%s set success!\n",__func__);
	}
	else
    {
		mpu6500_read_reg(sensor, 0x3a, &data , 1);
		inv_i2c_single_write(client, 0x38, 0x00);
		gpio_switch_setstate(0);
		printk(KERN_INFO TAGI "%s clear int trigger!\n",__func__);
    }
}

static ssize_t
accel_sensor_eint_store(struct device_driver *dev_driver, const char *buffer, size_t count)
{
	unsigned int eint_state = simple_strtoul(buffer, NULL, 10);
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	printk(KERN_INFO TAGI "%s: eint_state = %d!\n",__func__,eint_state);
    accel_sensor_eint_enable(eint_state);	
	atomic_set(&sensor->sensor_int,eint_state);  
	return count;
}

static ssize_t
accel_sensor_eint_show(struct device_driver *dev_driver, char *buf)
{
    int value;
    struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
    value = atomic_read(&sensor->sensor_int);  
    return sprintf(buf, "%d\n", value);
}

static void dumpReg(struct mpu6500_sensor *sensor)
{
    int i=0;
    u8 addr = 0x00;
    u8 regdata=0;
    for(i=0x00; i<0x8f; i++)
    {
        //dump all
        mpu6500_read_reg(sensor,addr,&regdata,1);
	    printk(KERN_INFO TAGI "mpu6500 Reg addr=%x regdata=%x\n",addr,regdata);
	    addr++;
		mdelay(5);
    }
}

static ssize_t sensor_readreg_show(struct device_driver *dev_driver, char *buf)
{    
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	dumpReg(sensor);   
	return 1;
}
static ssize_t sensor_writereg_store(struct device_driver *dev_driver, 
	const char *buf, size_t count)
{	
	int addr;
	int value;	
	int tem;	
	struct i2c_client *client = mpu6500_i2c_client;	
	tem = simple_strtoul(buf, NULL, 16);	
	addr = (tem&0xff00)>>8;	
	value = tem&0x00ff;
	if(i2c_smbus_write_byte_data(client,addr,value) < 0)	
	{      
		printk(KERN_INFO TAGI "mpu6500 reg==>[%x]/[%x]\n",addr,value);
	}	
	return count;
}

static DRIVER_ATTR(raw_data, 0666, mpu6500_accel_raw_data_show, NULL);
static DRIVER_ATTR(data, 0666, mpu6500_accel_final_data_show, NULL);
static DRIVER_ATTR(calibration, 0666, accel_sensor_calibration_show, accel_sensor_calibration_store);
static DRIVER_ATTR(calibratecmd, 0666, accel_sensor_calibratecmd_show, accel_sensor_calibratecmd_store);
static DRIVER_ATTR(set_eint, 0666, accel_sensor_eint_show, accel_sensor_eint_store);
static DRIVER_ATTR(get_reg, 0666, sensor_readreg_show, sensor_writereg_store);

static struct driver_attribute *mpu6500_accel_attr_list[] = {
	&driver_attr_raw_data,
	&driver_attr_data,
	&driver_attr_calibration,
    &driver_attr_calibratecmd,
    &driver_attr_set_eint,
	&driver_attr_get_reg,
};

static int mpu6500_accel_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mpu6500_accel_attr_list)/sizeof(mpu6500_accel_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, mpu6500_accel_attr_list[idx])))
		{            
			 printk(KERN_ERR TAGE "driver_create_file (%s) = %d\n", mpu6500_accel_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static ssize_t mpu6500_gyro_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	struct axis_data data;
	int ret;
	
	mpu6500_switch_engine(sensor, true, BIT_PWR_GYRO_STBY_MASK);
//	printk(KERN_INFO TAGI "%s wtl power_on = %d\n",__func__,sensor->sensor_power);
	ret = i2c_smbus_read_byte_data(client, sensor->reg.pwr_mgmt_1);
    if (ret&BIT_SLEEP)
    {
        mpu6500_set_power_mode(sensor, true);
		mpu6500_config_sample_rate(sensor);
		msleep(30);
    }
	mpu6500_read_gyro_data(sensor, &data);
	printk(KERN_INFO TAGI "%s gyro x = %d,y = %d,z = %d!!!\n",__func__, data.x, data.y, data.z);
	
	return sprintf(buf, "%d %d %d\n", data.x, data.y, data.z);
}

static ssize_t mpu6500_gyro_xyz_dps_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	struct axis_data data;
	int ret;
	
	mpu6500_switch_engine(sensor, true, BIT_PWR_GYRO_STBY_MASK);
//	printk(KERN_INFO TAGI "%s wtl power_on = %d\n",__func__,sensor->sensor_power); 
 	ret = i2c_smbus_read_byte_data(client, sensor->reg.pwr_mgmt_1);
    if (ret&BIT_SLEEP)
    {
        mpu6500_set_power_mode(sensor, true);
		mpu6500_config_sample_rate(sensor);
		msleep(30);
    }
	mpu6500_read_gyro_data(sensor, &data);
	data.x = data.x*10/164;
	data.y = data.y*10/164;
	data.z = data.z*10/164;
	printk(KERN_INFO TAGI "%s gyro x = %d,y = %d,z = %d!!!\n",__func__, data.x, data.y, data.z);
	
	return sprintf(buf, "%d %d %d\n", data.x, data.y, data.z);
}

static ssize_t mpu6500_gyro_calibratexyz(struct mpu6500_sensor *sensor)
{	
	struct i2c_client *client = mpu6500_i2c_client;
	int i,sumofx,sumofy,sumofz;	
	struct axis_data data;	
	sumofx = 0;	sumofy = 0;	sumofz = 0;
	sensor = i2c_get_clientdata(client);
	
	for (i=0;i<20;i++)	
	{		
		mpu6500_read_gyro_data(sensor, &data);			
		printk(KERN_INFO TAGI "[mpu6500_gy] %d :%d, %d, %d\n",i,data.x,data.y,data.z);		
		sumofx = sumofx + data.x;		
		sumofy = sumofy + data.y;		
		sumofz = sumofz + data.z;		
		mdelay(20);	
	}	
	sensor->gyro_calibration_data.x = sumofx / 20;	
	sensor->gyro_calibration_data.y = sumofy / 20;	
	sensor->gyro_calibration_data.z = sumofz / 20;	
	printk(KERN_INFO TAGI "[mpu6500_gy] calibratecmd_x =%d	\n",sensor->gyro_calibration_data.x);	
	printk(KERN_INFO TAGI "[mpu6500_gy] calibratecmd_y =%d	\n",sensor->gyro_calibration_data.y);	
	printk(KERN_INFO TAGI "[mpu6500_gy] calibratecmd_z =%d	\n",sensor->gyro_calibration_data.z);		
	return 0;
}

	/**
* inv_check_6500_gyro_self_test() - check 6500 gyro self test. this function
*                                   returns zero as success. A non-zero return
*                                   value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg:  average value of self test
*/
static int inv_check_6500_gyro_self_test(struct inv_selftest_device *st,
						int *reg_avg, int *st_avg) {
	u8 regs[3];
	int ret_val, result;
	int otp_value_zero = 0;
	int st_shift_prod[3], st_shift_cust[3], i;
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	ret_val = 0;
	
	result = mpu6500_read_reg(sensor, REG_6500_XG_ST_DATA, regs, 3);
	if (result)
		return result;
	 printk(KERN_INFO TAGI "%s self_test gyro shift_code - %02x %02x %02x\n",
		 st->name, regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	 printk(KERN_INFO TAGI "%s self_test gyro st_shift_prod - %+d %+d %+d\n",
		 st->name, st_shift_prod[0], st_shift_prod[1],
		 st_shift_prod[2]);

	for (i = 0; i < 3; i++) {
		st_shift_cust[i] = st_avg[i] - reg_avg[i];
		if (!otp_value_zero) {
			/* Self Test Pass/Fail Criteria A */
			if (st_shift_cust[i] < DEF_6500_GYRO_CT_SHIFT_DELTA
						* st_shift_prod[i])
			{
					ret_val = 1;
					 printk(KERN_ERR TAGE "%s, Fail, A\n", __func__);
			}
		} else {
			/* Self Test Pass/Fail Criteria B */
			if (st_shift_cust[i] < DEF_GYRO_ST_AL *
						DEF_SELFTEST_GYRO_SENS *
						DEF_ST_PRECISION)
			{
				ret_val = 1;
				 printk(KERN_ERR TAGE "%s, Fail, B\n", __func__);
			}
		}
	}
	 printk(KERN_INFO TAGI "%s self_test gyro st_shift_cust - %+d %+d %+d\n",
		 st->name, st_shift_cust[0], st_shift_cust[1],
		 st_shift_cust[2]);

	if (ret_val == 0) {
		/* Self Test Pass/Fail Criteria C */
		for (i = 0; i < 3; i++)
			if (abs(reg_avg[i]) > DEF_GYRO_OFFSET_MAX *
						DEF_SELFTEST_GYRO_SENS *
						DEF_ST_PRECISION)
			{
				ret_val = 1;
				 printk(KERN_ERR TAGE "%s, Fail, C\n", __func__);
			}
	}

	return ret_val;
}

int mpu6500gy_hw_self_test(struct inv_selftest_device *st)
{
	int result;
	int gyro_bias_st[THREE_AXIS], gyro_bias_regular[THREE_AXIS];
	int accel_bias_st[THREE_AXIS], accel_bias_regular[THREE_AXIS];
	int test_times, i;
	char compass_result, accel_result, gyro_result;

	inv_store_setting(st);
//	result = inv_power_up_self_test(st);
//	if (result)
//		return result;
	compass_result = 0;
	accel_result = 0;
	gyro_result = 0;
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(st, 0, gyro_bias_regular,
			accel_bias_regular);
		if (result == -EAGAIN)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		goto test_fail;
	 printk(KERN_INFO TAGI "%s self_test accel bias_regular - %+d %+d %+d\n",
		 st->name, accel_bias_regular[0],
		 accel_bias_regular[1], accel_bias_regular[2]);
	 printk(KERN_INFO TAGI "%s self_test gyro bias_regular - %+d %+d %+d\n",
		 st->name, gyro_bias_regular[0], gyro_bias_regular[1],
		 gyro_bias_regular[2]);

	for (i = 0; i < 3; i++) {
		st->gyro_bias[i] = gyro_bias_regular[i];
		st->accel_bias[i] = accel_bias_regular[i];
	}

	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(st, BITS_SELF_TEST_EN, gyro_bias_st,
					accel_bias_st);
		if (result == -EAGAIN)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;
	 printk(KERN_INFO TAGI "%s self_test accel bias_st - %+d %+d %+d\n",
		 st->name, accel_bias_st[0], accel_bias_st[1],
		 accel_bias_st[2]);
	 printk(KERN_INFO TAGI "%s self_test gyro bias_st - %+d %+d %+d\n",
		 st->name, gyro_bias_st[0], gyro_bias_st[1],
		 gyro_bias_st[2]);

	for (i = 0; i < 3; i++) {
		st->gyro_bias_st[i] = gyro_bias_st[i];
		st->accel_bias_st[i] = accel_bias_st[i];
	}
    //TBD compass selftest
//	if (st->chip_config.has_compass)
//		compass_result = !st->slave_compass->self_test(st);

	if (INV_MPU6500 == st->chip_type) {
	//	accel_result = !inv_check_6500_accel_self_test(st,
	//		accel_bias_regular, accel_bias_st);
		gyro_result = !inv_check_6500_gyro_self_test(st,
			gyro_bias_regular, gyro_bias_st);
	}

test_fail:
	inv_recover_setting(st);

	result = gyro_result;
	printk(KERN_INFO TAGI "%s, result=%d\n", __func__, result);
	//1:PASS 0:FAIL
	return result;
}

static int MPU6500gy_SelfTest(struct i2c_client *client)
{
    int testRes = 0;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	
    sensor->mpu_selftest_device.name = "MPU6XXX";
    sensor->mpu_selftest_device.chip_type = INV_MPU6500;//MPU6515 don't need to modify this chip_type
    sensor->mpu_selftest_device.samples = INIT_ST_SAMPLES;	
	
	testRes = mpu6500gy_hw_self_test(&(sensor->mpu_selftest_device));
	
	return testRes;
}

static ssize_t mpu6500_calibratecmd_selftest(struct mpu6500_sensor *sensor)
{
	struct i2c_client *client = mpu6500_i2c_client;

	sensor->gyro_calibration_data.self_test = 2;
    sensor->gyro_calibrate_process = 2;
	{
		sensor->gyro_calibration_data.self_test = MPU6500gy_SelfTest(client);
	}
	return 0;
}

static ssize_t gyro_sensor_calibratecmd_store(struct device_driver *dev_driver, const char *buf, size_t count)
{	
  	int res;
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	unsigned long cmd = simple_strtoul(buf, NULL, 10);
	printk(KERN_INFO TAGI "[mpu6500_gy] sensor_calibratecmd_store cmd= %d\n",(int)cmd);	
	mpu6500_switch_engine(sensor, true, BIT_PWR_GYRO_STBY_MASK);
	res = mpu6500_set_power_mode(sensor,true);
	mdelay(400);
	if(res != MPU6500_SUCCESS ) //
	{
		printk(KERN_ERR TAGE "mpu6500 power on faild!\n");
	}
	if (cmd == 1)	
	{	
		sensor->gyro_calibrate_process = 1;		
		mpu6500_gyro_calibratexyz(sensor);		
		sensor->gyro_calibrate_process = 0;	
	}	
	else if (cmd == 2) 	
	{	
	 	printk(KERN_INFO TAGI "[mpu6500_gy] mpu6500_calibratecmd_store cmd = 2 performed!\r\n");
		sensor->gyro_calibrate_process = 1;		
		mpu6500_calibratecmd_selftest(sensor);	
		sensor->gyro_calibrate_process = 0;	
	}	
	else	{
	}	
	res = mpu6500_set_power_mode(sensor, false);
//	mpu6500_switch_engine(sensor, false, BIT_PWR_GYRO_STBY_MASK);
	if(res != MPU6500_SUCCESS ) //
	{
		printk(KERN_ERR TAGE "mpu6500 power on faild!\n");
	}
	return count;
}

static ssize_t gyro_sensor_calibratecmd_show(struct device_driver *dev_driver, char *buf)
{    
	int value;
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	printk(KERN_INFO TAGI "%s begin!\n",__func__); 
	value = sensor->gyro_calibrate_process;       
	return sprintf(buf, "%d\n", value);
}

static ssize_t gyro_sensor_calibration_store(struct device_driver *dev_driver, const char *buf, size_t count)
{    
	struct calibrate_data value = {5,5,6,6}; 	
	int i,j=0;	
	char bufnew[30];	
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	for(i=0;i<=30;i++)	
	{			
		bufnew[i] = *(buf+i);		
		printk(KERN_INFO TAGI "%c",*(buf+i));		
		if(*(buf+i)=='\0')		
			break;	
	}	
	for(i=0;i<=30;i++)		
	{			
		if(*(bufnew+i)==' ')		
		{				
			*(bufnew+i)='\0';			
			value.x = simple_strtol(bufnew, NULL, 10);		
			*(bufnew+i)=' ';			
			j=i;			
			break;		
		}	
	}		
	i++;	
	for(;i<=30;i++)	
	{			
		if(*(bufnew+i)==' ')		
		{				
			*(bufnew+i)='\0';			
			value.y = simple_strtol(bufnew+j+1, NULL, 10);			
			*(bufnew+i)=' ';			
			j=i;			
			break;		
		}	
	}	
	i++;	
	for(;i<=30;i++)	
	{			
		if(*(bufnew+i)==' ')		
		{				
			*(bufnew+i)='\0';			
			value.z = simple_strtol(bufnew+j+1, NULL, 10);			
			*(bufnew+i)=' ';			
			j=i;				
			break;		
		}	
	}	
	i++;	
	for(;i<=30;i++)	
	{			
		if(*(bufnew+i)==' ')		
		{				
			*(bufnew+i)='\0';			
			value.self_test = simple_strtol(bufnew+j+1, NULL, 10);			
			*(bufnew+i)=' ';			
			j=i;				
			break;		
		}	
	}	   
	sensor->gyro_calibration_data = value;  
	return count;
}

static ssize_t gyro_sensor_calibration_show(struct device_driver *dev_driver, char *buf)
{    
	struct calibrate_data value;	
	struct i2c_client *client = mpu6500_i2c_client;
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);	
	value = sensor->gyro_calibration_data;       
	printk(KERN_INFO TAGI "%s: %d,%d,%d\n",__func__,value.x,value.y,value.z);  
	return sprintf(buf, "%d %d %d %d  ov\n", value.x ,value.y  ,value.z ,value.self_test);
}

static struct driver_attribute mpu6500_gyro_attr_list[] = {
  __ATTR(rawdata, 0666, mpu6500_gyro_raw_data_show, NULL),
  __ATTR(get_dps, 0666, mpu6500_gyro_xyz_dps_show, NULL),
  __ATTR(calibration, 0666, gyro_sensor_calibration_show, gyro_sensor_calibration_store),
  __ATTR(calibratecmd, 0666, gyro_sensor_calibratecmd_show, gyro_sensor_calibratecmd_store),
};

static int mpu6500_gyro_create_attr(struct device_driver *driver) 
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mpu6500_gyro_attr_list); i++)
		if (driver_create_file(driver, mpu6500_gyro_attr_list+i))
			goto error;
	return 0;

error:
	for (i--; i >= 0; i--)
		driver_remove_file(driver, mpu6500_gyro_attr_list+i);
	printk(KERN_ERR TAGE "%s Unable to create interface\n",__func__);
	return -1;
}

static void setup_mpu6500_reg(struct mpu_reg_map *reg)
{
	reg->sample_rate_div	= REG_SAMPLE_RATE_DIV;
	reg->lpf		= REG_CONFIG;
	reg->fifo_en		= REG_FIFO_EN;
	reg->gyro_config	= REG_GYRO_CONFIG;
	reg->accel_config	= REG_ACCEL_CONFIG;
	reg->mot_thr		= REG_ACCEL_MOT_THR;
	reg->mot_dur		= REG_ACCEL_MOT_DUR;
	reg->fifo_count_h	= REG_FIFO_COUNT_H;
	reg->fifo_r_w		= REG_FIFO_R_W;
	reg->raw_gyro		= REG_RAW_GYRO;
	reg->raw_accel		= REG_RAW_ACCEL;
	reg->temperature	= REG_TEMPERATURE;
	reg->int_pin_cfg	= REG_INT_PIN_CFG;
	reg->int_enable		= REG_INT_ENABLE;
	reg->int_status		= REG_INT_STATUS;
	reg->user_ctrl		= REG_USER_CTRL;
	reg->pwr_mgmt_1		= REG_PWR_MGMT_1;
	reg->pwr_mgmt_2		= REG_PWR_MGMT_2;
};

/**
 * mpu_check_chip_type() - check and setup chip type.
 */
static int mpu_check_chip_type(struct mpu6500_sensor *sensor,
		const struct i2c_device_id *id)
{
	struct i2c_client *client = sensor->client;
	struct mpu_reg_map *reg;
	s32 ret;

	if (!strcmp(id->name, "mpu6050"))
		sensor->chip_type = INV_MPU6050;
	else if (!strcmp(id->name, "mpu6500"))
		sensor->chip_type = INV_MPU6500;
	else if (!strcmp(id->name, "mpu6xxx"))
		sensor->chip_type = INV_MPU6050;
	else
		return -EPERM;

	reg = &sensor->reg;
	setup_mpu6500_reg(reg);

	/* turn off and turn on power to ensure gyro engine is on */

	ret = mpu6500_set_power_mode(sensor, false);
	if (ret)
		return ret;
	ret = mpu6500_set_power_mode(sensor, true);
	if (ret)
		return ret;

	if (!strcmp(id->name, "mpu6xxx")) {
		ret = i2c_smbus_read_byte_data(client,
				REG_WHOAMI);
		if (ret < 0)
			return ret;

		if (ret == MPU6500_ID) {
			sensor->chip_type = INV_MPU6500;
		} else if (ret == MPU6050_ID) {
			sensor->chip_type = INV_MPU6050;
		} else {
			printk(KERN_ERR TAGE "%s Invalid chip ID %d\n",__func__ , ret);
			return -ENODEV;
		}
	}
	return 0;
}

/**
 *  mpu6500_init_config() - Initialize hardware, disable FIFO.
 *  @indio_dev:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  AFS: 2G
 */
static int mpu6500_init_config(struct mpu6500_sensor *sensor)
{
	struct mpu_reg_map *reg;
	struct i2c_client *client;
	s32 ret;
	u8 data;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	reg = &sensor->reg;
	client = sensor->client;

	mpu6500_reset_chip(sensor);

	memset(&sensor->cfg, 0, sizeof(struct mpu_chip_config));

	/* Wake up from sleep */
	ret = i2c_smbus_write_byte_data(client, reg->pwr_mgmt_1,
		BIT_WAKEUP_AFTER_RESET);
	if (ret < 0)
		return ret;

	/* Gyro full scale range configure */
	ret = i2c_smbus_write_byte_data(client, reg->gyro_config,
		MPU_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT);
	if (ret < 0)
		return ret;
	sensor->cfg.fsr = MPU_FSR_2000DPS;

	ret = i2c_smbus_write_byte_data(client, reg->lpf, MPU_DLPF_42HZ);
	if (ret < 0)
		return ret;
	sensor->cfg.lpf = MPU_DLPF_42HZ;

	//data = (u8)(ODR_DLPF_ENA / INIT_FIFO_RATE - 1);
	//ret = i2c_smbus_write_byte_data(client, reg->sample_rate_div, data);
	// Set 125HZ sample rate
	ret = i2c_smbus_write_byte_data(client, reg->sample_rate_div, 0x07);
	if (ret < 0)
		return ret;
//	sensor->cfg.rate_div = data;

	ret = i2c_smbus_write_byte_data(client, reg->accel_config,
		(ACCEL_FS_02G << ACCL_CONFIG_FSR_SHIFT));
	if (ret < 0)
		return ret;
	sensor->cfg.accel_fs = ACCEL_FS_02G;

/*	if ((sensor->pdata->int_flags & IRQF_TRIGGER_FALLING) ||
		(sensor->pdata->int_flags & IRQF_TRIGGER_LOW))
		data = BIT_INT_CFG_DEFAULT | BIT_INT_ACTIVE_LOW;
	else
		data = BIT_INT_CFG_DEFAULT;*/
	data = 0 ;
	ret = i2c_smbus_write_byte_data(client, reg->int_pin_cfg, data);
	if (ret < 0)
		return ret;
	sensor->cfg.int_pin_cfg = data;

	/* Put sensor into sleep mode */
	ret = i2c_smbus_read_byte_data(client,
		sensor->reg.pwr_mgmt_1);
	if (ret < 0)
		return ret;

	data = (u8)ret;
	data |=  BIT_SLEEP;
	ret = i2c_smbus_write_byte_data(client,
		sensor->reg.pwr_mgmt_1, data);
	if (ret < 0)
		return ret;

	sensor->cfg.gyro_enable = 0;
	sensor->cfg.gyro_fifo_enable = 0;
	sensor->cfg.accel_enable = 0;
	sensor->cfg.accel_fifo_enable = 0;

	return 0;
}

static irqreturn_t mpu6500_gsensor_irq_handler(int irq, void *dev_id)
{
	struct mpu6500_sensor *sensor =
	    (struct mpu6500_sensor *)dev_id;

    printk(KERN_ERR TAGI "%s\n", __func__);
    disable_irq_nosync(sensor->irq_gsensor);
    schedule_work(&sensor->irq_work);
	return IRQ_HANDLED;
}

static void mpu6500_gsensor_irq_work(struct work_struct *work)
{
    struct mpu6500_sensor *sensor=container_of(work, struct mpu6500_sensor, irq_work);
	int state;
	
    if(NULL==sensor)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        goto exit_err;
    }
    state = gpio_get_value(sensor->pdata->gpio_int);
    gpio_switch_setstate(state);            
    printk(KERN_ERR TAGI "%s gpio state %d\n", __func__, state);
    enable_irq(sensor->irq_gsensor);

exit_err:
    ;
}

static int mpu6500_accel_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int mpu6500_accel_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mpu6500_accel_pd_driver = {
	.probe  = mpu6500_accel_pd_probe,
	.remove = mpu6500_accel_pd_remove,    
	.driver = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device mpu6500_accel_pd_device = {
    .name = "gsensor",
    .id   = -1,
};

static int mpu6500_gyro_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int mpu6500_gyro_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mpu6500_gyro_pd_driver = {
	.probe  = mpu6500_gyro_pd_probe,
	.remove = mpu6500_gyro_pd_remove,    
	.driver = {
		.name  = "gyroscope",
		.owner = THIS_MODULE,
	}
};

struct platform_device mpu6500_gyro_pd_device = {
    .name = "gyroscope",
    .id   = -1,
};

#ifdef CONFIG_OF
static int mpu6500_dt_get_place(struct device *dev,
			struct mpu6500_platform_data *pdata)
{
	const char *place_name;
	int rc;
	int i;

	rc = of_property_read_string(dev->of_node, "invn,place", &place_name);
	if (rc) {
		printk(KERN_ERR TAGE "Cannot get place configuration!\n");
		return -EINVAL;
	}

	for (i = 0; i < MPU6500_AXIS_REMAP_TAB_SZ; i++) {
		if (!strcmp(place_name, mpu6500_place_name2num[i].name)) {
			pdata->place = mpu6500_place_name2num[i].place;
			break;
		}
	}
	if (i >= MPU6500_AXIS_REMAP_TAB_SZ) {
		 printk(KERN_INFO TAGI "Invalid place parameter, use default value 0\n");
		pdata->place = 0;
	}
//	 printk(KERN_INFO"pdata->place = %d\n",pdata->place);
	return 0;
}

static int mpu6500_parse_dt(struct device *dev,
			struct mpu6500_platform_data *pdata)
{
	int rc;

	rc = mpu6500_dt_get_place(dev, pdata);
	if (rc)
		return rc;
	if(of_property_read_bool(dev->of_node,"sensor,gpio-irq"))
    {
    	rc = of_get_named_gpio_flags(dev->of_node,
    				"sensor,gpio-irq", 0, &pdata->int_flags);
    	if (rc < 0) {
    		printk(KERN_ERR TAGE "%s read interrupt int pin error %d\n", __func__, rc);
            pdata->gpio_int= -2;
    	}
        else
        {
            pdata->gpio_int= rc;
        }
	}
	else
	{
        pdata->gpio_int= -2;
        printk(KERN_ERR TAGE "%s no sensor,step-irq\n", __func__);
	}
	return 0;
}
#else
static int mpu6500_parse_dt(struct device *dev,
			struct mpu6500_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static struct sensor_regulator mpu6500_vreg[] = {
	{NULL, "vdd", 2850000, 2850000},
	{NULL, "vddio", 1800000, 1800000},
};

static int mpu6500_config_regulator(struct mpu6500_sensor *sensor, bool on)
{
	int rc = 0, i;
	struct sensor_regulator *mpu6500_vreg_config=mpu6500_vreg;
	int num_reg = sizeof(mpu6500_vreg_config) / sizeof(struct sensor_regulator);

	if (on) 
    {
		for (i = 0; i < num_reg; i++) 
        {
			mpu6500_vreg_config[i].vreg = regulator_get(&sensor->client->dev, mpu6500_vreg_config[i].name);
            
			if (IS_ERR(mpu6500_vreg_config[i].vreg)) 
            {
				rc = PTR_ERR(mpu6500_vreg_config[i].vreg);
				printk(KERN_ERR TAGE "regulator get failed rc=%d\n", rc);
				mpu6500_vreg_config[i].vreg = NULL;
			}

			if (regulator_count_voltages(mpu6500_vreg_config[i].vreg) > 0) 
            {
				rc = regulator_set_voltage(
					mpu6500_vreg_config[i].vreg,
					mpu6500_vreg_config[i].min_uV,
					mpu6500_vreg_config[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE "set voltage failed rc=%d\n", rc);
					regulator_put(mpu6500_vreg_config[i].vreg);
					mpu6500_vreg_config[i].vreg = NULL;
				}
			}

			rc = regulator_enable(mpu6500_vreg_config[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE "regulator_enable failed rc =%d\n", rc);
				if (regulator_count_voltages(mpu6500_vreg_config[i].vreg) > 0) 
                {
					regulator_set_voltage(
						mpu6500_vreg_config[i].vreg, 0,
						mpu6500_vreg_config[i].max_uV);
				}
				regulator_put(mpu6500_vreg_config[i].vreg);
				mpu6500_vreg_config[i].vreg = NULL;
			}
		}
	} 
    else 
    {
		i = num_reg;
	}
    
	return rc;
}

/**
 * mpu6500_probe() - device detection callback
 * @client: i2c client of found device
 * @id: id match information
 *
 * The I2C layer calls us when it believes a sensor is present at this
 * address. Probe to see if this is correct and to validate the device.
 *
 * If present install the relevant sysfs interfaces and input device.
 */
static int mpu6500_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct mpu6500_sensor *sensor;
	struct mpu6500_platform_data *pdata;
	int ret;

	printk(KERN_INFO TAGI "%s start.\n", __func__);
	ret = i2c_check_functionality(client->adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA |
					 I2C_FUNC_I2C);
	if (!ret) {
		printk(KERN_ERR TAGE "Required I2C funcationality does not supported\n");
		return -ENODEV;
	}
	sensor = devm_kzalloc(&client->dev, sizeof(struct mpu6500_sensor),
			GFP_KERNEL);
	if (!sensor) {
		printk(KERN_ERR TAGE "Failed to allocate driver data\n");
		return -ENOMEM;
	}

	sensor->client = client;
	sensor->dev = &client->dev;
	i2c_set_clientdata(client, sensor);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct mpu6500_platform_data), GFP_KERNEL);
		if (!pdata) {
			printk(KERN_ERR TAGE "Failed to allcated memory\n");
			ret = -ENOMEM;
			goto err_free_devmem;
		}
		ret = mpu6500_parse_dt(&client->dev, pdata);
		if (ret) {
			printk(KERN_ERR TAGE "Failed to parse device tree\n");
			ret = -EINVAL;
			goto err_free_devmem;
		}
	} else {
		pdata = client->dev.platform_data;
	}
	
	mpu6500_config_regulator(sensor,true);

	if (!pdata) {
		printk(KERN_ERR TAGE "Cannot get device platform data\n");
		ret = -EINVAL;
		goto err_free_devmem;
	}

	mutex_init(&sensor->op_lock);
	sensor->pdata = pdata;

	ret = mpu_check_chip_type(sensor, id);
	if (ret) {
		printk(KERN_ERR TAGE "Cannot get invalid chip type\n");
		goto err_free_enable_gpio;
	}

	ret = mpu6500_init_engine(sensor);
	if (ret) {
		printk(KERN_ERR TAGE "Failed to init chip engine\n");
		goto err_power_off_device;
	}

	ret = mpu6500_set_lpa_freq(sensor, MPU6050_LPA_5HZ);
	if (ret) {
		printk(KERN_ERR TAGE "Failed to set lpa frequency\n");
		goto err_power_off_device;
	}

	sensor->cfg.is_asleep = false;
	atomic_set(&sensor->accel_en, 0);
	atomic_set(&sensor->gyro_en, 0);
	ret = mpu6500_init_config(sensor);
	if (ret) {
		printk(KERN_ERR TAGE "Failed to set default config\n");
		goto err_power_off_device;
	}

	sensor->accel_dev = devm_input_allocate_device(&client->dev);
	if (!sensor->accel_dev) {
		printk(KERN_ERR TAGE "Failed to allocate accelerometer input device\n");
		ret = -ENOMEM;
		goto err_power_off_device;
	}

	sensor->gyro_dev = devm_input_allocate_device(&client->dev);
	if (!sensor->gyro_dev) {
		printk(KERN_ERR TAGE "Failed to allocate gyroscope input device\n");
		ret = -ENOMEM;
		goto err_power_off_device;
	}

	sensor->accel_dev->name = MPU6500_DEV_NAME_ACCEL;
	sensor->gyro_dev->name = MPU6500_DEV_NAME_GYRO;
	sensor->accel_dev->id.bustype = BUS_I2C;
	sensor->gyro_dev->id.bustype = BUS_I2C;
	sensor->accel_poll_ms = MPU6500_ACCEL_DEFAULT_POLL_INTERVAL_MS;
	sensor->gyro_poll_ms = MPU6500_GYRO_DEFAULT_POLL_INTERVAL_MS;

	input_set_capability(sensor->accel_dev, EV_ABS, ABS_MISC);
	input_set_capability(sensor->gyro_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(sensor->accel_dev, ABS_X,
			MPU6500_ACCEL_MIN_VALUE, MPU6500_ACCEL_MAX_VALUE,
			0, 0);
	input_set_abs_params(sensor->accel_dev, ABS_Y,
			MPU6500_ACCEL_MIN_VALUE, MPU6500_ACCEL_MAX_VALUE,
			0, 0);
	input_set_abs_params(sensor->accel_dev, ABS_Z,
			MPU6500_ACCEL_MIN_VALUE, MPU6500_ACCEL_MAX_VALUE,
			0, 0);
	input_set_abs_params(sensor->gyro_dev, ABS_X,
			     MPU6500_GYRO_MIN_VALUE, MPU6500_GYRO_MAX_VALUE,
			     0, 0);
	input_set_abs_params(sensor->gyro_dev, ABS_Y,
			     MPU6500_GYRO_MIN_VALUE, MPU6500_GYRO_MAX_VALUE,
			     0, 0);
	input_set_abs_params(sensor->gyro_dev, ABS_Z,
			     MPU6500_GYRO_MIN_VALUE, MPU6500_GYRO_MAX_VALUE,
			     0, 0);
	sensor->accel_dev->dev.parent = &client->dev;
	sensor->gyro_dev->dev.parent = &client->dev;
	input_set_drvdata(sensor->accel_dev, sensor);
	input_set_drvdata(sensor->gyro_dev, sensor);

	sensor->use_poll = 1;

	sensor->data_wq = create_freezable_workqueue("mpu6500_data_work");
	if (!sensor->data_wq) {
		printk(KERN_ERR TAGE "Cannot create workqueue!\n");
		goto err_power_off_device;
	}

	INIT_WORK(&sensor->accel_poll_work, mpu6500_accel_work_fn);
	INIT_WORK(&sensor->gyro_poll_work, mpu6500_gyro_work_fn);

	ret = input_register_device(sensor->accel_dev);
	if (ret) {
		printk(KERN_ERR TAGE "Failed to register input device\n");
		goto err_destroy_workqueue;
	}
	ret = input_register_device(sensor->gyro_dev);
	if (ret) {
		printk(KERN_ERR TAGE "Failed to register input device\n");
		goto err_destroy_workqueue;
	}

	input_set_events_per_packet(sensor->accel_dev, 100);
	input_set_events_per_packet(sensor->gyro_dev, 100);
	hrtimer_init(&sensor->timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sensor->timer_acc.function = mpu6500_acc_timer_handle;
	hrtimer_init(&sensor->timer_gyro, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sensor->timer_gyro.function = mpu6500_gyro_timer_handle;
	
	ret = create_accel_sysfs_interfaces(&sensor->accel_dev->dev);
	if (ret < 0) {
		printk(KERN_ERR TAGE "failed to create sysfs for accel\n");
		goto err_destroy_workqueue;
	}
	ret = create_gyro_sysfs_interfaces(&sensor->gyro_dev->dev);
	if (ret < 0) {
		printk(KERN_ERR TAGE "failed to create sysfs for gyro\n");
		goto err_remove_accel_sysfs;
	}

	sensor->accel_cdev = mpu6500_acc_cdev;
	sensor->accel_cdev.delay_msec = sensor->accel_poll_ms;
	sensor->accel_cdev.sensors_enable = mpu6500_accel_cdev_enable;
	sensor->accel_cdev.sensors_poll_delay = mpu6500_accel_cdev_poll_delay;

	ret = sensors_classdev_register(&client->dev, &sensor->accel_cdev);
	if (ret) {
		printk(KERN_ERR TAGE "create accel class device file failed!\n");
		ret = -EINVAL;
		goto err_remove_gyro_sysfs;
	}

	sensor->gyro_cdev = mpu6500_gyro_cdev;
	sensor->gyro_cdev.delay_msec = sensor->gyro_poll_ms;
	sensor->gyro_cdev.sensors_enable = mpu6500_gyro_cdev_enable;
	sensor->gyro_cdev.sensors_poll_delay = mpu6500_gyro_cdev_poll_delay;

	ret = sensors_classdev_register(&client->dev, &sensor->gyro_cdev);
	if (ret) {
		printk(KERN_ERR TAGE "create gyro class device file failed!\n");
		ret = -EINVAL;
		goto err_remove_accel_cdev;
	}
	
	if((ret = platform_driver_register(&mpu6500_accel_pd_driver)))
	{
		printk(KERN_ERR TAGE "failed to register mpu6500_accel_driver err %d\n", ret);
		goto err_remove_gyro_cdev;
	}

    if((ret = platform_device_register(&mpu6500_accel_pd_device)))
    {
		printk(KERN_ERR TAGE "failed to register mpu6500_accel_device err %d\n", ret);
		goto err_free_acc_driver;
    }

	if((ret = mpu6500_accel_create_attr(&mpu6500_accel_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "mpu6500 create acc attribute err = %d\n", ret);
		goto err_free_acc_device;
	}
	
	if((ret = platform_driver_register(&mpu6500_gyro_pd_driver)))
	{
		printk(KERN_ERR TAGE "failed to register mpu6500_gyro_pd_driver err %d\n", ret);
		goto err_free_acc_device;
	}

    if((ret = platform_device_register(&mpu6500_gyro_pd_device)))
    {
		printk(KERN_ERR TAGE "failed to register mpu6500_gyro_pd_device err %d\n", ret);
		goto err_free_gyro_driver;
    }

	if((ret = mpu6500_gyro_create_attr(&mpu6500_gyro_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "mpu6500 create gyro attribute err = %d\n", ret);
		goto err_free_gyro_device;
	}
	
	 printk(KERN_ERR TAGE "%s %d\n", __func__, sensor->pdata->gpio_int);
    if((0<=sensor->pdata->gpio_int))
    {
        ret = gpio_request(sensor->pdata->gpio_int, "gpio-int");
        if (ret < 0)
            goto err_free_gpio;
        
        ret = gpio_direction_input(sensor->pdata->gpio_int);
        if (ret < 0)
            goto err_free_gpio;
        INIT_WORK(&sensor->irq_work, mpu6500_gsensor_irq_work);
        
        sensor->irq_gsensor = gpio_to_irq(sensor->pdata->gpio_int);
        if (sensor->irq_gsensor < 0) {
            ret = sensor->irq_gsensor;
            goto err_free_gpio;
        }
        /*IRQF_TRIGGER_HIGH IRQF_TRIGGER_LOW */
        ret = request_irq( sensor->irq_gsensor, mpu6500_gsensor_irq_handler,
            IRQF_TRIGGER_RISING, "gpio-irq", sensor);
        if (ret < 0)
            goto err_free_gpio;
    	ret = enable_irq_wake(sensor->irq_gsensor);
    	if (ret < 0)
    		goto err_free_gpio;
    }
	
	mpu6500_i2c_client = client;
	/* weitianlei add */
	wake_lock_init(&ts_judge_phone_direction_wakelock, WAKE_LOCK_SUSPEND, "Ts_judge_dir");
	acc_for_ts_judge_dir = mpu6500_for_ts_judge_dir;
	
	printk(KERN_INFO TAGI "%s  is ok!!!\n", __func__);
	return 0;

err_free_gpio:
	gpio_free(sensor->pdata->gpio_int);	
err_free_gyro_device:
	platform_device_unregister(&mpu6500_gyro_pd_device);
err_free_gyro_driver:
	platform_driver_unregister(&mpu6500_gyro_pd_driver);
err_free_acc_device:
	platform_device_unregister(&mpu6500_accel_pd_device);
err_free_acc_driver:
	platform_driver_unregister(&mpu6500_accel_pd_driver);
err_remove_gyro_cdev:
	sensors_classdev_unregister(&sensor->gyro_cdev);
err_remove_accel_cdev:
	 sensors_classdev_unregister(&sensor->accel_cdev);
err_remove_gyro_sysfs:
	remove_accel_sysfs_interfaces(&sensor->gyro_dev->dev);
err_remove_accel_sysfs:
	remove_accel_sysfs_interfaces(&sensor->accel_dev->dev);
err_destroy_workqueue:
	destroy_workqueue(sensor->data_wq);
	if (client->irq > 0)
		free_irq(client->irq, sensor);
err_power_off_device:

err_free_enable_gpio:
//	if (gpio_is_valid(sensor->enable_gpio))
//		gpio_free(sensor->enable_gpio);
err_free_devmem:
	devm_kfree(&client->dev, sensor);
	printk(KERN_ERR TAGE "Probe device return error %d!\n", ret);
	return ret;
}

/**
 * mpu6500_remove() - remove a sensor
 * @client: i2c client of sensor being removed
 *
 * Our sensor is going away, clean up the resources.
 */
static int mpu6500_remove(struct i2c_client *client)
{
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	printk(KERN_INFO TAGI "%s begin!\n",__func__);
	
	sensors_classdev_unregister(&sensor->accel_cdev);
	sensors_classdev_unregister(&sensor->gyro_cdev);
	remove_gyro_sysfs_interfaces(&sensor->gyro_dev->dev);
	remove_accel_sysfs_interfaces(&sensor->accel_dev->dev);
	destroy_workqueue(sensor->data_wq);
	if (gpio_is_valid(sensor->pdata->gpio_int))
		gpio_free(sensor->pdata->gpio_int);
	devm_kfree(&client->dev, sensor);

	return 0;
}

#ifdef CONFIG_PM
/**
 * mpu6500_suspend() - called on device suspend
 * @dev: device being suspended
 *
 * Put the device into sleep mode before we suspend the machine.
 */
static int mpu6500_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);

	printk(KERN_INFO TAGI "%s start!\n",__func__);
	mutex_lock(&sensor->op_lock);

	if (sensor->cfg.gyro_enable)
		hrtimer_cancel(&sensor->timer_gyro);
	//	cancel_delayed_work_sync(&sensor->gyro_poll_work);

	if (sensor->cfg.accel_enable)
		hrtimer_cancel(&sensor->timer_acc);
	//	cancel_delayed_work_sync(&sensor->accel_poll_work);
    if(!atomic_read(&sensor->sensor_int))
		mpu6500_set_power_mode(sensor, false);

	mutex_unlock(&sensor->op_lock);
//	printk(KERN_INFO TAGI "%s Suspend completed!\n",__func__);

	return 0;
}

/**
 * mpu6500_resume() - called on device resume
 * @dev: device being resumed
 *
 * Put the device into powered mode on resume.
 */
static int mpu6500_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mpu6500_sensor *sensor = i2c_get_clientdata(client);
	int ret = 0;
	ktime_t ktime;
	
//	printk(KERN_INFO TAGI "%s start!\n",__func__);
	mutex_lock(&sensor->op_lock);

	/* Reset sensor to recovery from unexpected state */
	mpu6500_reset_chip(sensor);

	ret = mpu6500_restore_context(sensor);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s Failed to restore context\n",__func__);
		goto exit;
	}

	/* Enter sleep mode if both accel and gyro are not enabled */
	ret = mpu6500_set_power_mode(sensor, sensor->cfg.enable);
	if (ret < 0) {
		printk(KERN_ERR TAGE "Failed to set power mode enable=%d\n",sensor->cfg.enable);
		goto exit;
	}

	if (sensor->cfg.gyro_enable) {
		ret = mpu6500_gyro_enable(sensor, true);
		if (ret < 0) {
			printk(KERN_ERR TAGE "%s Failed to enable gyro\n",__func__);
			goto exit;
		}

		if (sensor->use_poll) {
//			queue_delayed_work(sensor->data_wq,&sensor->gyro_poll_work,
//				msecs_to_jiffies(sensor->gyro_poll_ms));
			ktime = ktime_set(0,sensor->gyro_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->timer_gyro, ktime, HRTIMER_MODE_REL);
		}
	}

	if (sensor->cfg.accel_enable) {
		ret = mpu6500_accel_enable(sensor, true);
		if (ret < 0) {
			printk(KERN_ERR TAGE "%s Failed to enable accel\n",__func__);
			goto exit;
		}

		if (sensor->use_poll) {
//			queue_delayed_work(sensor->data_wq,&sensor->accel_poll_work,
//				msecs_to_jiffies(sensor->accel_poll_ms));
			ktime = ktime_set(0,sensor->accel_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->timer_acc, ktime, HRTIMER_MODE_REL);
		}
	}

exit:
	mutex_unlock(&sensor->op_lock);
	printk(KERN_INFO TAGI "%s Resume complete, ret = %d\n",__func__, ret);
	return ret;
}
#endif

static UNIVERSAL_DEV_PM_OPS(mpu6500_pm, mpu6500_suspend, mpu6500_resume, NULL);

static const struct i2c_device_id mpu6500_ids[] = {
	{ "mpu6500", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu6500_ids);

static const struct of_device_id mpu6500_of_match[] = {
	{ .compatible = "invn,mpu6500", },
	{ },
};
MODULE_DEVICE_TABLE(of, mpu6500_of_match);

static struct i2c_driver mpu6500_i2c_driver = {
	.driver	= {
		.name	= "mpu6500",
		.owner	= THIS_MODULE,
		.pm	= &mpu6500_pm,
		.of_match_table = mpu6500_of_match,
	},
	.probe		= mpu6500_probe,
	.remove		= mpu6500_remove,
	.id_table	= mpu6500_ids,
};

static int __init mpu6500_acc_init(void)
{
	printk(KERN_INFO TAGI "%s accelerometer driver: init\n", MPU6500_NAME);
	return i2c_add_driver(&mpu6500_i2c_driver);
}

static void __exit mpu6500_acc_exit(void)
{
	printk(KERN_INFO TAGI "%s accelerometer driver exit\n", MPU6500_NAME);
	i2c_del_driver(&mpu6500_i2c_driver);
	return;
}

late_initcall(mpu6500_acc_init);
module_exit(mpu6500_acc_exit);

MODULE_DESCRIPTION("MPU6500 Tri-axis gyroscope driver");
MODULE_LICENSE("GPL v2");
