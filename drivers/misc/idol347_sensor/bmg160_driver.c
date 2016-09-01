/*!
 * @section LICENSE
 * (C) Copyright 2013-2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bmg160_driver.c
 * @date     2014/03/11 14:20
 * @id       "7bf4b97"
 * @version  1.5.6
 *
 * @brief    BMG160 Linux Driver
 */
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "bmg160.h"

/* POWER SUPPLY VOLTAGE RANGE */
#define BMG160_VDD_MIN_UV  2600000
#define BMG160_VDD_MAX_UV  3300000
#define BMG160_VIO_MIN_UV  1800000
#define BMG160_VIO_MAX_UV  1800000
/* sensor specific */
#define SENSOR_NAME "bmg160"

#define SENSOR_CHIP_ID_BMG (0x0f)
#define CHECK_CHIP_ID_TIME_MAX   5

#define BMG_REG_NAME(name) BMG160_##name
#define BMG_VAL_NAME(name) BMG160_##name
#define BMG_CALL_API(name) bmg160_##name

#define BMG_I2C_WRITE_DELAY_TIME 1

/* generic */
#define BMG_MAX_RETRY_I2C_XFER (100)
#define BMG_MAX_RETRY_WAKEUP (5)
#define BMG_MAX_RETRY_WAIT_DRDY (100)

#define BMG_DELAY_MIN (10)
#define BMG_DELAY_DEFAULT (100)

#define BMG_VALUE_MAX (32767)
#define BMG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMG_SELF_TEST 0

#define BMG_SOFT_RESET_VALUE                0xB6


#ifdef BMG_USE_FIFO
#define MAX_FIFO_F_LEVEL 100
#define MAX_FIFO_F_BYTES 8
#define BMG160_FIFO_DAT_SEL_X                     1
#define BMG160_FIFO_DAT_SEL_Y                     2
#define BMG160_FIFO_DAT_SEL_Z                     3
#endif

/*!
 * @brief:BMI058 feature
 *  macro definition
*/
#ifdef CONFIG_SENSORS_BMI058
/*! BMI058 X AXIS definition*/
#define BMI058_X_AXIS	BMG160_Y_AXIS
/*! BMI058 Y AXIS definition*/
#define BMI058_Y_AXIS	BMG160_X_AXIS

#define C_BMI058_One_U8X	1
#define C_BMI058_Two_U8X	2
#endif

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8


struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	unsigned int place:3;
	int irq;
	int (*irq_gpio_cfg)(void);
};


/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
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


struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

struct bmg160_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);
	int int1_pin;
	int int2_pin;
	uint32_t int1_flags;
	uint32_t int2_flags;
	int place;/*[BUFFIX]-Add by TCTNB.XQJ, PR-916361,2015/1/30,slove gyrp direction ok*/

};

struct bmg_client_data {
	struct bmg160_t device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct sensors_classdev	cdev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;

	struct bmg160_data_t value;
	u8 enable:1;
	unsigned int fifo_count;
	unsigned char fifo_datasel;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct bosch_sensor_specific *bst_pd;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;
/*TR961495 modify by ZXZ ,2015/03/28  for created it own workquque but need to specify it do not bound to any cpu */
	struct workqueue_struct *data_wq;
	struct bmg160_platform_data *pdata;
};

/*TR961495 modify by ZXZ ,2015/03/28  change min_delay from 1000 to 10000 for framwork get data rate*/
static struct sensors_classdev sensors_cdev = {
	.name = "bmg160",
	.vendor = "BOSCH",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "35.0",
	.resolution = "1.0",
	.sensor_power = "0.2",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,

};
static struct bmg_client_data *pdev_data;
static struct i2c_client *bmg_client;
/* i2c operation for API */
static void bmg_i2c_delay(BMG160_U16 msec);
static int bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static int bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmg_dump_reg(struct i2c_client *client);
static int bmg_check_chip_id(struct i2c_client *client);

static int bmg_pre_suspend(struct i2c_client *client);
static int bmg_post_resume(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler);
static void bmg_late_resume(struct early_suspend *handler);
#endif
/*!
* BMG160 sensor remapping function
* need to give some parameter in BSP files first.
*/
static const struct bosch_sensor_axis_remap
	bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,	 1,    2,	  1,	  1,	  1 }, /* P0 */
	{  1,	 0,    2,	  1,	 -1,	  1 }, /* P1 */
	{  0,	 1,    2,	 -1,	 -1,	  1 }, /* P2 */
	{  1,	 0,    2,	 -1,	  1,	  1 }, /* P3 */

	{  0,	 1,    2,	 -1,	  1,	 -1 }, /* P4 */
	{  1,	 0,    2,	 -1,	 -1,	 -1 }, /* P5 */
	{  0,	 1,    2,	  1,	 -1,	 -1 }, /* P6 */
	{  1,	 0,    2,	  1,	  1,	 -1 }, /* P7 */
};

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
			const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
			int place)
{
/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;
	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bmg160_remap_sensor_data(struct bmg160_data_t *val,
		struct bmg_client_data *client_data)
{
	struct bosch_sensor_data bsd;

	if ((NULL == client_data->bst_pd) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		return;

#ifdef CONFIG_SENSORS_BMI058
/*x,y need to be invesed becase of HW Register for BMI058*/
	bsd.y = val->datax;
	bsd.x = val->datay;
	bsd.z = val->dataz;
#else
	bsd.x = val->datax;
	bsd.y = val->datay;
	bsd.z = val->dataz;
#endif

	bst_remap_sensor_data_dft_tab(&bsd,
			client_data->bst_pd->place);

	val->datax = bsd.x;
	val->datay = bsd.y;
	val->dataz = bsd.z;

}

static int bmg_check_chip_id(struct i2c_client *client)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), &chip_id, 1);
		dev_info(&client->dev, "read chip id result: %#x", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMG) {
			udelay(1000);
		} else {
			err = 0;
			break;
		}
	}
	return err;
}

static void bmg_i2c_delay(BMG160_U16 msec)
{
	udelay(msec*1000);
}

static void bmg_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		snprintf(dbg_buf_str + i * 3, PAGE_SIZE, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		snprintf(dbg_buf_str + i * 3, PAGE_SIZE, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);
}

/*i2c read routine for API*/
static int bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -EINVAL;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			dev_err(&client->dev, "i2c bus read error");
			return -EINVAL;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -EINVAL;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -EINVAL;
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},

		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			udelay(1000);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

#ifdef BMG_USE_FIFO
static int bmg_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},

		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			udelay(1000);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif

/*i2c write routine for */
static int bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
	s32 ret;
	u8 buffer[2];


	if (NULL == client)
		return -EPERM;

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		ret = i2c_master_send(client, (char *)buffer, 2);

		reg_addr++;
		data++;
		if (ret < 0) {
			dev_err(&client->dev, "error writing i2c bus");
			return -EPERM;
		}

	}
	return 0;
}

static int bmg_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_i2c_read(bmg_client, reg_addr, data, len);
	return err;
}

static int bmg_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err;
	err = bmg_i2c_write(bmg_client, reg_addr, data, len);
	return err;
}


static void bmg_work_func(struct work_struct *work)
{
	struct bmg_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmg_client_data, work);

	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));
	struct bmg160_data_t gyro_data;

	//add by huangshenglin@hoperun,2016/1/20 ,sync event->time with hal.+++
	ktime_t timestamp;
	//add by huangshenglin@hoperun,2016/1/20 sync event->time with hal.--

	/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
	#if 0
	schedule_delayed_work(&client_data->work, delay);
	#else
	queue_delayed_work(client_data->data_wq,&client_data->work, delay);
	#endif
	/*TR961495 modify by ZXZ END*/

	BMG_CALL_API(get_dataXYZ)(&gyro_data);
	/*remapping for BMG160 sensor*/
	bmg160_remap_sensor_data(&gyro_data, client_data);

	input_report_abs(client_data->input, ABS_RX, gyro_data.datax);
	input_report_abs(client_data->input, ABS_RY, gyro_data.datay);
	input_report_abs(client_data->input, ABS_RZ, gyro_data.dataz);

	//add by huangshenglin@hoperun,2016/1/20, sync event->time with hal.+++
	timestamp = ktime_get_boottime();
	input_event(client_data->input,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
	input_event(client_data->input,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
	//add by huangshenglin@hoperun,2016/1/20 sync event->time with hal.--

	input_sync(client_data->input);

}

static int bmg_set_soft_reset(struct i2c_client *client)
{
	int err = 0;
	unsigned char data = BMG_SOFT_RESET_VALUE;
	printk("*****[%s] send*****\n",__FUNCTION__);
	err = bmg_i2c_write(client, BMG160_BGW_SOFTRESET_ADDR, &data, 1);
	printk("*****[%s] return*****\n",__FUNCTION__);
	return err;
}

static ssize_t bmg_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", SENSOR_CHIP_ID_BMG);
}

static ssize_t bmg_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&client_data->mutex_op_mode);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", op_mode);

	return ret;
}

static ssize_t bmg_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(op_mode);

	mutex_unlock(&client_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}



static ssize_t bmg_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int count;

	struct bmg160_data_t value_data;
	BMG_CALL_API(get_dataXYZ)(&value_data);
	/*BMG160 sensor raw data remapping*/
	bmg160_remap_sensor_data(&value_data, client_data);

	count = snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
				value_data.datax,
				value_data.datay,
				value_data.dataz);

	return count;
}

static ssize_t bmg_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char range = 0;
	BMG_CALL_API(get_range_reg)(&range);
	err = snprintf(buf, PAGE_SIZE, "%d\n", range);
	return err;
}

static ssize_t bmg_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long range;
	err = kstrtoul(buf, 10, &range);
	if (err)
		return err;
	BMG_CALL_API(set_range_reg)(range);
	return count;
}

static ssize_t bmg_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth = 0;
	BMG_CALL_API(get_bw)(&bandwidth);
	err = snprintf(buf, PAGE_SIZE, "%d\n", bandwidth);
	return err;
}

static ssize_t bmg_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long bandwidth;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	BMG_CALL_API(set_bw)(bandwidth);
	return count;
}

static int bmg160_gyro_cdev_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct bmg_client_data *client_data = container_of(sensors_cdev,
			struct bmg_client_data, cdev);
	struct bmg160_platform_data *pdata = client_data->pdata;

	mutex_lock(&client_data->mutex_enable);
	if (enable != client_data->enable) {
		if (enable) {
			if (pdata->power_on)
				pdata->power_on(true);
			BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));
            BMG_CALL_API(set_bw)(4);/* [BUFFIX]-Add- by TCTNB.ZXZ,PR-1073091, 2015/09/16,set default bandwidth 200hz,default is unfilter*/
		/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
			#if 0
			schedule_delayed_work(&client_data->work,
			msecs_to_jiffies(atomic_read(&client_data->delay)));
			#else
			queue_delayed_work(client_data->data_wq,&client_data->work,
			msecs_to_jiffies(atomic_read(&client_data->delay)));
			#endif
		/*TR961495 modify by ZXZ END*/

		} else {
                         BMG_CALL_API(set_mode)(
                              BMG_VAL_NAME(MODE_SUSPEND));/* [BUFFIX]-Add- by TCTNB.XQJ,PR-906832, 2015/02/5,power consumtion problem,register need be set also*/
			cancel_delayed_work_sync(&client_data->work);
			if (pdata->power_on)
				pdata->power_on(false);
		}
		client_data->enable = enable;
	}
	mutex_unlock(&client_data->mutex_enable);

	return 0;
}

static int bmg160_gyro_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
			unsigned int delay_ms)
{
	struct bmg_client_data *client_data = container_of(sensors_cdev,
			struct bmg_client_data, cdev);

	if (delay_ms < BMG_DELAY_MIN)
		delay_ms = BMG_DELAY_MIN;

	atomic_set(&client_data->delay, delay_ms);

	return 0;

}

static ssize_t bmg_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = snprintf(buf, PAGE_SIZE, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmg_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {

		/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
			#if 0
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
			#else
			BMG_CALL_API(set_bw)(4);/* [BUFFIX]-Add- by TCTNB.ZXZ,PR-1073091, 2015/09/16,set default bandwidth 200hz,default is unfilter*/
		queue_delayed_work(client_data->data_wq,
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
			#endif
		/*TR961495 modify by ZXZ END*/
		} else {
			cancel_delayed_work_sync(&client_data->work);
		}

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t bmg_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		atomic_read(&client_data->delay));

}

static ssize_t bmg_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMG_DELAY_MIN)
		data = BMG_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static ssize_t bmg_store_fastoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fastoffset_en;
	err = kstrtoul(buf, 10, &fastoffset_en);
	if (err)
		return err;
	if (fastoffset_en) {

#ifdef CONFIG_SENSORS_BMI058
		BMG_CALL_API(set_fast_offset_en_ch)(BMI058_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMI058_Y_AXIS, 1);
#else
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Y_AXIS, 1);
#endif

		BMG_CALL_API(set_fast_offset_en_ch)(BMG160_Z_AXIS, 1);
		BMG_CALL_API(enable_fast_offset)();
	}
	return count;
}

static ssize_t bmg_store_slowoffset_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long slowoffset_en;
	err = kstrtoul(buf, 10, &slowoffset_en);
	if (err)
		return err;
	if (slowoffset_en) {
		BMG_CALL_API(set_slow_offset_th)(3);
		BMG_CALL_API(set_slow_offset_dur)(0);
#ifdef CONFIG_SENSORS_BMI058
		BMG_CALL_API(set_slow_offset_en_ch)(BMI058_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMI058_Y_AXIS, 1);
#else
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 1);
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 1);
#endif
		BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 1);
	} else {
#ifdef CONFIG_SENSORS_BMI058
	BMG_CALL_API(set_slow_offset_en_ch)(BMI058_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMI058_Y_AXIS, 0);
#else
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_X_AXIS, 0);
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Y_AXIS, 0);
#endif
	BMG_CALL_API(set_slow_offset_en_ch)(BMG160_Z_AXIS, 0);
	}

	return count;
}

static ssize_t bmg_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char selftest;
	BMG_CALL_API(selftest)(&selftest);
	err = snprintf(buf, PAGE_SIZE, "%d\n", selftest);
	return err;
}

static ssize_t bmg_show_sleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char sleepdur;
	BMG_CALL_API(get_sleepdur)(&sleepdur);
	err = snprintf(buf, PAGE_SIZE, "%d\n", sleepdur);
	return err;
}

static ssize_t bmg_store_sleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long sleepdur;
	err = kstrtoul(buf, 10, &sleepdur);
	if (err)
		return err;
	BMG_CALL_API(set_sleepdur)(sleepdur);
	return count;
}

static ssize_t bmg_show_autosleepdur(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char autosleepdur;
	BMG_CALL_API(get_autosleepdur)(&autosleepdur);
	err = snprintf(buf, PAGE_SIZE, "%d\n", autosleepdur);
	return err;
}

static ssize_t bmg_store_autosleepdur(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long autosleepdur;
	unsigned char bandwidth;
	err = kstrtoul(buf, 10, &autosleepdur);
	if (err)
		return err;
	BMG_CALL_API(get_bw)(&bandwidth);
	BMG_CALL_API(set_autosleepdur)(autosleepdur, bandwidth);
	return count;
}

#ifdef BMG_DEBUG
static ssize_t bmg_store_softreset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long softreset;
	err = kstrtoul(buf, 10, &softreset);
	if (err)
		return err;
	BMG_CALL_API(set_soft_reset)();
	return count;
}

static ssize_t bmg_show_dumpreg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg[0x40];
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	for (i = 0; i < 0x40; i++) {
		bmg_i2c_read(client_data->client, i, reg+i, 1);

		count += snprintf(&buf[count],
			PAGE_SIZE, "0x%x: 0x%x\n", i, reg[i]);
	}
	return count;
}
#endif

#ifdef BMG_USE_FIFO
static ssize_t bmg_show_fifo_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_mode;
	BMG_CALL_API(get_fifo_mode)(&fifo_mode);
	err = snprintf(buf, PAGE_SIZE, "%d\n", fifo_mode);
	return err;
}

static ssize_t bmg_store_fifo_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long fifo_mode;
	err = kstrtoul(buf, 10, &fifo_mode);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_mode)(fifo_mode);
	return count;
}

static ssize_t bmg_show_fifo_framecount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_framecount;
	BMG_CALL_API(get_fifo_framecount)(&fifo_framecount);
	err = snprintf(buf, PAGE_SIZE, "%d\n", fifo_framecount);
	return err;
}

static ssize_t bmg_store_fifo_framecount(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	client_data->fifo_count = (unsigned int) data;

	return count;
}

static ssize_t bmg_show_fifo_overrun(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_overrun;
	BMG_CALL_API(get_fifo_overrun)(&fifo_overrun);
	err = snprintf(buf, PAGE_SIZE, "%d\n", fifo_overrun);
	return err;
}

/*!
 * brief: bmg single axis data remaping
 * @param[i] fifo_datasel   fifo axis data select setting
 * @param[i/o] remap_dir   remapping direction
 * @param[i] client_data   to transfer sensor place
 *
 * @return none
 */
static void bmg_single_axis_remaping(unsigned char fifo_datasel,
		unsigned char *remap_dir, struct bmg_client_data *client_data)
{
	if ((NULL == client_data->bst_pd) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		return;
	else {
		signed char place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place <= 0) ||
			(place >= MAX_AXIS_REMAP_TAB_SZ))
			return;

		if (fifo_datasel < 1 || fifo_datasel > 3)
			return;
		else {
			switch (fifo_datasel) {
			/*P2, P3, P4, P5 X axis(andorid) need to reverse*/
			case BMG160_FIFO_DAT_SEL_X:
				if (-1 == bst_axis_remap_tab_dft[place].sign_x)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			/*P1, P2, P5, P6 Y axis(andorid) need to reverse*/
			case BMG160_FIFO_DAT_SEL_Y:
				if (-1 == bst_axis_remap_tab_dft[place].sign_y)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			case BMG160_FIFO_DAT_SEL_Z:
			/*P4, P5, P6, P7 Z axis(andorid) need to reverse*/
				if (-1 == bst_axis_remap_tab_dft[place].sign_z)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			default:
				break;
			}
		}
	}

	return;
}

static ssize_t bmg_show_fifo_data_frame(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err, i, len;
	signed char fifo_data_out[MAX_FIFO_F_LEVEL * MAX_FIFO_F_BYTES] = {0};
	unsigned char f_len = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	struct bmg160_data_t gyro_lsb;
	unsigned char axis_dir_remap = 0;
	s16 value;

	if (client_data->fifo_count == 0)
		return -ENOENT;

	if (client_data->fifo_datasel)
		/*Select one axis data output for every fifo frame*/
		f_len = 2;
	else
		/*Select X Y Z axis data output for every fifo frame*/
		f_len = 6;

	bmg_i2c_burst_read(client_data->client, BMG160_FIFO_DATA_ADDR,
			fifo_data_out, client_data->fifo_count * f_len);
	err = 0;

	if (f_len == 6) {
		/* Select X Y Z axis data output for every frame */
		for (i = 0; i < client_data->fifo_count; i++) {
			gyro_lsb.datax =
			((unsigned char)fifo_data_out[i * f_len + 1] << 8
				| (unsigned char)fifo_data_out[i * f_len + 0]);

			gyro_lsb.datay =
			((unsigned char)fifo_data_out[i * f_len + 3] << 8
				| (unsigned char)fifo_data_out[i * f_len + 2]);

			gyro_lsb.dataz =
			((unsigned char)fifo_data_out[i * f_len + 5] << 8
				| (unsigned char)fifo_data_out[i * f_len + 4]);

			bmg160_remap_sensor_data(&gyro_lsb, client_data);
			len = snprintf(buf, PAGE_SIZE, "%d %d %d ",
				gyro_lsb.datax, gyro_lsb.datay, gyro_lsb.dataz);
			buf += len;
			err += len;
		}
	} else {
		/* single axis data output for every frame */
		bmg_single_axis_remaping(client_data->fifo_datasel,
				&axis_dir_remap, client_data);
		for (i = 0; i < client_data->fifo_count * f_len / 2; i++) {
			value = ((unsigned char)fifo_data_out[2 * i + 1] << 8 |
					(unsigned char)fifo_data_out[2 * i]);
			if (axis_dir_remap)
				value = 0 - value;
			len = snprintf(buf, PAGE_SIZE, "%d ", value);
			buf += len;
			err += len;
		}
	}

	return err;
}

/*!
 * @brief show fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_show_fifo_data_sel(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_data_sel;
	struct i2c_client *client = to_i2c_client(dev);
	struct bmg_client_data *client_data = i2c_get_clientdata(client);
	signed char place = BOSCH_SENSOR_PLACE_UNKNOWN;

	BMG_CALL_API(get_fifo_data_sel)(&fifo_data_sel);

	/*remapping fifo_dat_sel if define virtual place in BSP files*/
	if ((NULL != client_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != client_data->bst_pd->place)) {
		place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
				*bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
				*so we need to +1*/
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}

	}

	err = snprintf(buf, PAGE_SIZE, "%d\n", fifo_data_sel);
	return err;
}

/*!
 * @brief store fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bmg_store_fifo_data_sel(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_data_sel;

	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	signed char place;

	err = kstrtoul(buf, 10, &fifo_data_sel);
	if (err)
		return err;

	/*save fifo_data_sel(android axis definition)*/
	client_data->fifo_datasel = (unsigned char) fifo_data_sel;

	/*remaping fifo_dat_sel if define virtual place*/
	if ((NULL != client_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != client_data->bst_pd->place)) {
		place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			/*Need X Y axis revesal sensor place: P1, P3, P5, P7 */
			/* BMG160_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
			  * but bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
			  * so we need to +1*/
			if (BMG160_FIFO_DAT_SEL_X == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_x + 1;

			else if (BMG160_FIFO_DAT_SEL_Y == fifo_data_sel)
				fifo_data_sel =
					bst_axis_remap_tab_dft[place].src_y + 1;
		}
	}

	if (BMG_CALL_API(set_fifo_data_sel)(fifo_data_sel) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_fifo_tag(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char fifo_tag;
	BMG_CALL_API(get_fifo_tag)(&fifo_tag);
	err = snprintf(buf, PAGE_SIZE, "%d\n", fifo_tag);
	return err;
}

static ssize_t bmg_store_fifo_tag(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)

{
	int err;
	unsigned long fifo_tag;
	err = kstrtoul(buf, 10, &fifo_tag);
	if (err)
		return err;
	BMG_CALL_API(set_fifo_tag)(fifo_tag);
	return count;
}
#endif

static DEVICE_ATTR(chip_id, S_IRUGO,
		bmg_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
		bmg_show_op_mode, bmg_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
		bmg_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR,
		bmg_show_range, bmg_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR,
		bmg_show_bandwidth, bmg_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		bmg_show_enable, bmg_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
		bmg_show_delay, bmg_store_delay);
static DEVICE_ATTR(fastoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_fastoffset_en);
static DEVICE_ATTR(slowoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_slowoffset_en);
static DEVICE_ATTR(selftest, S_IRUGO,
		bmg_show_selftest, NULL);
static DEVICE_ATTR(sleepdur, S_IRUGO|S_IWUSR,
		bmg_show_sleepdur, bmg_store_sleepdur);
static DEVICE_ATTR(autosleepdur, S_IRUGO|S_IWUSR,
		bmg_show_autosleepdur, bmg_store_autosleepdur);
#ifdef BMG_DEBUG
static DEVICE_ATTR(softreset, S_IRUGO|S_IWUSR,
		NULL, bmg_store_softreset);
static DEVICE_ATTR(regdump, S_IRUGO,
		bmg_show_dumpreg, NULL);
#endif
#ifdef BMG_USE_FIFO
static DEVICE_ATTR(fifo_mode, S_IRUGO|S_IWUSR,
		bmg_show_fifo_mode, bmg_store_fifo_mode);
static DEVICE_ATTR(fifo_framecount, S_IRUGO|S_IWUSR,
		bmg_show_fifo_framecount, bmg_store_fifo_framecount);
static DEVICE_ATTR(fifo_overrun, S_IRUGO|S_IWUSR,
		bmg_show_fifo_overrun, NULL);
static DEVICE_ATTR(fifo_data_frame, S_IRUGO|S_IWUSR,
		bmg_show_fifo_data_frame, NULL);
static DEVICE_ATTR(fifo_data_sel, S_IRUGO|S_IWUSR,
		bmg_show_fifo_data_sel, bmg_store_fifo_data_sel);
static DEVICE_ATTR(fifo_tag, S_IRUGO|S_IWUSR,
		bmg_show_fifo_tag, bmg_store_fifo_tag);
#endif

static struct attribute *bmg_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_fastoffset_en.attr,
	&dev_attr_slowoffset_en.attr,
	&dev_attr_selftest.attr,
	&dev_attr_sleepdur.attr,
	&dev_attr_autosleepdur.attr,
#ifdef BMG_DEBUG
	&dev_attr_softreset.attr,
	&dev_attr_regdump.attr,
#endif
#ifdef BMG_USE_FIFO
	&dev_attr_fifo_mode.attr,
	&dev_attr_fifo_framecount.attr,
	&dev_attr_fifo_overrun.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_fifo_data_sel.attr,
	&dev_attr_fifo_tag.attr,
#endif
	NULL
};

static struct attribute_group bmg_attribute_group = {
	.attrs = bmg_attributes
};

static struct device_attribute bmg160_attrs[] = {
	__ATTR(selftest, 0444, bmg_show_selftest, NULL),
};
static int bmg_input_init(struct bmg_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = "gyroscope";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_RX, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_RY, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_RZ, BMG_VALUE_MIN, BMG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void bmg_input_destroy(struct bmg_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int sensor_regulator_configure(struct bmg_client_data *data, bool on)
{
	int rc;

	if (!on) {

		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				BMG160_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				BMG160_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				BMG160_VDD_MIN_UV, BMG160_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				BMG160_VIO_MIN_UV, BMG160_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, BMG160_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int sensor_regulator_power_on(struct bmg_client_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);

			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	}

enable_delay:
//	msleep(130);
	dev_dbg(&data->client->dev,
		"Sensor regulator power on =%d\n", on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on)
{
	struct bmg_client_data *data;
	int err = 0;

	if (pdev_data == NULL)
		return -ENODEV;

	data = pdev_data;
	if (data->power_on != on) {

		err = sensor_regulator_power_on(data, on);
		if (err)
			dev_err(&data->client->dev,
					"Can't configure regulator!\n");
		else
			data->power_on = on;
	}

	return err;
}

static int sensor_platform_hw_init(void)
{
	struct i2c_client *client;
	struct bmg_client_data *data;
	int error;

	if (pdev_data == NULL)
		return -ENODEV;

	data = pdev_data;
	client = data->client;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		dev_err(&client->dev, "unable to configure regulator\n");
		return error;
	}
	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct bmg_client_data *data = pdev_data;

	if (data == NULL)
		return;

	sensor_regulator_configure(data, false);

}

static int bmg_parse_dt(struct device *dev,
				struct bmg160_platform_data *pdata)
{
/* MODIFIED-BEGIN by long.chen.hr, 2016-03-22, BUG-1748726 */
//#ifdef CONFIG_TCT_8X16_IDOL3 /*[BUFFIX]-Add by TCTNB.XQJ, PR-916361,2015/1/30,slove gyrp direction */
    struct device_node *np = dev->of_node;
    u32 temp_val;
    int rc;

    rc = of_property_read_u32(np, "bmg,place", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read bmg160,place\n");
	} else {
		pdata->place = temp_val;
	}
//#endif
/* MODIFIED-END by long.chen.hr,BUG-1748726 */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	return 0;
}

static int bmg_parse_config_gpio(struct device *dev,
		struct bmg160_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->int1_pin = of_get_named_gpio_flags(np, "bmg160,gpio-int1", 0, &pdata->int1_flags);
	if (pdata->int1_pin < 0) {
		dev_err(dev, "Unable to read int1-gpio\n");
		return pdata->int1_pin;
	}

	pdata->int2_pin = of_get_named_gpio_flags(np, "bmg160,gpio-int2", 0, &pdata->int2_flags);
	if (pdata->int2_pin < 0) {
		dev_err(dev, "Unable to read int2-gpio\n");
		return pdata->int2_pin;
	}

	if (gpio_request(pdata->int1_pin, "bmg160_cpio_int1") < 0) {
		dev_err(dev, "gpio err %d\n", pdata->int1_pin);
		return -1;
	}
	gpio_direction_input(pdata->int1_pin);

	if (gpio_request(pdata->int2_pin, "bmg160_cpio_int2") < 0) {
		dev_err(dev, "gpio err %d\n", pdata->int2_pin);
		return -1;
	}
	gpio_direction_input(pdata->int2_pin);

	return 0;
}


static int bmg_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct bmg_client_data *client_data = NULL;
	struct bmg160_platform_data *pdata;
	dev_info(&client->dev, "function entrance");

	printk("*******%s******[start]\n",__FUNCTION__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error!");
		err = -EIO;
		goto exit_err_clean;
	}

	if (NULL == bmg_client) {
		bmg_client = client;
	} else {
		dev_err(&client->dev,
			"this driver does not support multiple clients");
		err = -EINVAL;
		goto exit_err_clean;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct bmg160_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
		err = bmg_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse platfrom data err=%d\n", err);
			return err;
		}

		err = bmg_parse_config_gpio(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev,
				"Unable to config gpio err=%d\n", err);
			return err;
		}

	}  else {
		pdata = client->dev.platform_data;

		if (!pdata) {
			dev_err(&client->dev,
				"platform data is NULL. Abort.\n");
			return -EINVAL;
		}
	}

	client_data = kzalloc(sizeof(struct bmg_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		dev_err(&client->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}
	client_data->pdata = pdata;

	pdev_data = client_data;

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);


	printk("*****power init*****!\n");
	if (pdata->init)
		err = pdata->init();

	if(err < 0){
		dev_err(&client->dev,
			"*****power init error,err = %d*****!\n",err);
		err = -EINVAL;
		goto exit_err_clean;
	}

	printk("*****power on*****!\n");
	if (pdata->power_on)
		err = pdata->power_on(true);

	if(err < 0){
		dev_err(&client->dev,
			"*****power on error,err = %d*****!\n",err);
		goto exit_err_clean;
	}

	usleep(300000);//delay 300ms
	err = bmg_set_soft_reset(client);

	if (err < 0) {
		dev_err(&client->dev,
			"erro soft reset!\n");
		goto exit_err_clean;
	}

	/* check chip id */
	err = bmg_check_chip_id(client);
	if (!err) {
		dev_notice(&client->dev,
			"Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		dev_err(&client->dev,
			"Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	/* input device init */
	err = bmg_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&bmg_attribute_group);
        err = device_create_file(&client->dev,bmg160_attrs); /* [BUFFIX]-Mod- by TCTNB.XQJ,PR-798607, 2014/12/11,add interface interface for selftestin given directory*/

	if (err < 0)
		goto exit_err_sysfs;

	if (NULL != client->dev.platform_data) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);

		if (NULL != client_data->bst_pd) {
			memcpy(client_data->bst_pd, client->dev.platform_data,
					sizeof(*client_data->bst_pd));
			dev_notice(&client->dev, "%s sensor driver set place: p%d",
					SENSOR_NAME,
					client_data->bst_pd->place);
		}
	}

/* MODIFIED-BEGIN by long.chen.hr, 2016-03-22, BUG-1748726 */
//#ifdef CONFIG_TCT_8X16_IDOL3 /*[BUFFIX]-Add by TCTNB.XQJ, PR-916361,2015/1/30,slove gyrp direction */

	  client_data->bst_pd->place=pdata->place;
	  dev_notice(&client->dev, " %s sensor driver set place: p%d",
					SENSOR_NAME,
					client_data->bst_pd->place);
//#endif
/* MODIFIED-END by long.chen.hr,BUG-1748726 */
	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bmg_work_func);
/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
#if 1
	client_data->data_wq = create_freezable_workqueue("bmg160_data_work");
	if (!client_data->data_wq) {
		dev_err(&client->dev, "Cannot get create workqueue!\n");
		goto out_register_classdev;
	}
#endif
/*TR961495 modify by ZXZ END*/
	atomic_set(&client_data->delay, BMG_DELAY_DEFAULT);

	client_data->cdev = sensors_cdev;
	client_data->cdev.sensors_enable = bmg160_gyro_cdev_enable;
	client_data->cdev.sensors_poll_delay = bmg160_gyro_cdev_poll_delay;
	err = sensors_classdev_register(&client->dev, &client_data->cdev);
	if (err) {
		dev_err(&client->dev, "sensors class register failed.\n");
/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
		#if 0
		goto out_register_classdev;
		#else
		goto destroy_workqueue_exit;
		#endif
/*TR961495 modify by ZXZ END*/
	}

	/* h/w init */
	client_data->device.bus_read = bmg_i2c_read_wrapper;
	client_data->device.bus_write = bmg_i2c_write_wrapper;
	client_data->device.delay_msec = bmg_i2c_delay;
	BMG_CALL_API(init)(&client_data->device);

	bmg_dump_reg(client);

	client_data->enable = 0;
	client_data->fifo_datasel = 0;
	client_data->fifo_count = 0;

	/* now it's power on which is considered as resuming from suspend */
	err = BMG_CALL_API(set_mode)(
			BMG_VAL_NAME(MODE_SUSPEND));

	if (err < 0)
		goto exit_err_sysfs;


#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.suspend = bmg_early_suspend;
	client_data->early_suspend_handler.resume = bmg_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif

	dev_notice(&client->dev, "sensor %s probed successfully", SENSOR_NAME);

	dev_dbg(&client->dev,
		"i2c_client: %p client_data: %p i2c_device: %p input: %p",
		client, client_data, &client->dev, client_data->input);

	return 0;
/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
#if 1
destroy_workqueue_exit:
	destroy_workqueue(client_data->data_wq);
#endif
/*TR961495 modify by ZXZ END*/

out_register_classdev:
	input_unregister_device(client_data->input);

exit_err_sysfs:
	if (err)
		bmg_input_destroy(client_data);

exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			kfree(client_data);
			client_data = NULL;
		}

		bmg_client = NULL;
	}

	return err;
}

static int bmg_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);
	dev_info(&client->dev, "function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		dev_info(&client->dev, "cancel work");
	}
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

static int bmg_post_resume(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	dev_info(&client->dev, "function entrance");
	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
/*TR961495 modify by ZXZ BEGIN,2015/03/28 for created it own workquque but need to specify it do not bound to any cpu */
	#if 0
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	#else
		queue_delayed_work(client_data->data_wq,&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	#endif
/*TR961495 modify by ZXZ END*/

	}
	mutex_unlock(&client_data->mutex_enable);

	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmg_early_suspend(struct early_suspend *handler)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	dev_info(&client->dev, "function entrance");

	mutex_lock(&client_data->mutex_op_mode);
	if (client_data->enable) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	mutex_unlock(&client_data->mutex_op_mode);
}

static void bmg_late_resume(struct early_suspend *handler)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)container_of(handler,
			struct bmg_client_data, early_suspend_handler);
	struct i2c_client *client = client_data->client;

	dev_info(&client->dev, "function entrance");

	mutex_lock(&client_data->mutex_op_mode);

	if (client_data->enable)
		err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
}
#else
static int bmg_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);
	struct bmg160_platform_data *pdata = client_data->pdata;
	dev_info(&client->dev, "function entrance");
	mutex_lock(&client_data->mutex_op_mode);
	if (client_data->enable) {
		err = bmg_pre_suspend(client);
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
	}
	if (pdata->power_on)
		pdata->power_on(false);
	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}

static int bmg_resume(struct i2c_client *client)
{

	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);
	struct bmg160_platform_data *pdata = client_data->pdata;
	dev_info(&client->dev, "function entrance");
	mutex_lock(&client_data->mutex_op_mode);
	if (pdata->power_on)
		pdata->power_on(true);
	if (client_data->enable)
		err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));
     else
     {
            if (regulator_count_voltages(client_data->vdd) > 0)//for avoid some error i2c log
               err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_SUSPEND));
      }

	/* post resume operation */
	bmg_post_resume(client);

	mutex_unlock(&client_data->mutex_op_mode);
	return err;
}
#endif

void bmg_shutdown(struct i2c_client *client)
{
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(set_mode)(
		BMG_VAL_NAME(MODE_DEEPSUSPEND));
	mutex_unlock(&client_data->mutex_op_mode);
}

static int bmg_remove(struct i2c_client *client)
{
	int err = 0;
	u8 op_mode;

	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&client_data->early_suspend_handler);
#endif
		mutex_lock(&client_data->mutex_op_mode);
		BMG_CALL_API(get_mode)(&op_mode);
		if (BMG_VAL_NAME(MODE_NORMAL) == op_mode) {
			cancel_delayed_work_sync(&client_data->work);
			dev_info(&client->dev, "cancel work");
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		udelay(1000);

		sysfs_remove_group(&client_data->input->dev.kobj,
				&bmg_attribute_group);
		bmg_input_destroy(client_data);
		kfree(client_data);

		bmg_client = NULL;
	}

	return err;
}

static const struct i2c_device_id bmg_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bmg_id);

static const struct of_device_id bmg160_of_match[] = {
	{ .compatible = "bosch, bmg160", },
	{ },
};
MODULE_DEVICE_TABLE(of, bmg160_of_match);

static struct i2c_driver bmg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = bmg160_of_match,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmg_id,
	.probe = bmg_probe,
	.remove = bmg_remove,
	.shutdown = bmg_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = bmg_suspend,
	.resume = bmg_resume,
#endif
};

static int __init BMG_init(void)
{
	return i2c_add_driver(&bmg_driver);
}

static void __exit BMG_exit(void)
{
	i2c_del_driver(&bmg_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMG GYROSCOPE SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMG_init);
module_exit(BMG_exit);
