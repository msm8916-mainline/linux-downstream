/*
 * Copyright (c) 2014 Yamaha Corporation
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

#include "yas.h"

#define YAS_INPUT_DEVICE_NAME   "compass"

#define TAG  "Msensor "       //"Gsensor" "PAsensor" "GYsensor"
#define TAGI "Msensor.I "     //KERN_INFO 
#define TAGE "Msensor.E "     //KERN_ERR 

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
#define YAS_MSM_NAME		"yas-533"
#define YAS_MSM_VENDOR		"Yamaha"
#define YAS_MSM_VERSION		(1)
#define YAS_MSM_HANDLE		(1)
#define YAS_MSM_TYPE		(2)
#define YAS_MSM_MIN_DELAY	(10000)
#define YAS_MSM_MAX_RANGE	(1200)
#define YAS_MSM_RESOLUTION	"1"
#define YAS_MSM_SENSOR_POWER	"0.40"
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
#define YAS_MSM_NAME		"yas-537"
#define YAS_MSM_VENDOR		"Yamaha"
#define YAS_MSM_VERSION		(1)
#define YAS_MSM_HANDLE		(1)
#define YAS_MSM_TYPE		(2)
#define YAS_MSM_MIN_DELAY	(10000)
#define YAS_MSM_MAX_RANGE	(2000)
#define YAS_MSM_RESOLUTION	"1"
#define YAS_MSM_SENSOR_POWER	"0.28"
#endif

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
	uint8_t data_valid;
	struct	regulator		*vdd;
	struct	regulator		*vio;
};

/* POWER SUPPLY VOLTAGE RANGE */
#define YAS_VDD_MIN_UV	2850000
#define YAS_VDD_MAX_UV	2850000
#define YAS_VIO_MIN_UV	1800000
#define YAS_VIO_MAX_UV	1800000

static struct sensors_classdev sensors_cdev = {
	.name = "yas533-mag",
	.vendor = "Yamaha",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1200",
	.resolution = "1",
	.sensor_power = "0.40",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10000,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

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
	int error;

	if (sizeof(tmp) - 1 < len)
		return -EPERM;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	error = i2c_master_send(this_client, tmp, len + 1);

	if (unlikely(error < 0)) {
		printk(KERN_ERR TAGE "I2C send error: %d\n", error);
		return error;
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
		if(st->poll_delay == 1000)
			st->poll_delay = 10;
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
	sscanf(buf, "%d\n", &position);
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
		st->poll_delay = delay_ms;
	mutex_unlock(&st->lock);

	return 0;

}


#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static ssize_t yas_hard_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t hard_offset[3];
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_GET_HW_OFFSET, hard_offset);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_GET_HW_OFFSET, hard_offset);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return snprintf(buf, sizeof(buf)/sizeof(buf[0]), "%d %d %d\n",
			hard_offset[0], hard_offset[1], hard_offset[2]);
}
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
static ssize_t yas_hard_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp[3];
	int8_t hard_offset[3];
	int ret, i;
	sscanf(buf, "%d %d %d\n", &tmp[0], &tmp[1], &tmp[2]);
	for (i = 0; i < 3; i++)
		hard_offset[i] = (int8_t)tmp[i];
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS532_SET_HW_OFFSET, hard_offset);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537


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
	return snprintf(buf, sizeof(buf), "%d\n", mag_average_sample);
}

static ssize_t yas_mag_average_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp;
	int8_t mag_average_sample;
	int ret;
	sscanf(buf, "%d\n", &tmp);
	mag_average_sample = (int8_t)tmp;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}
#endif

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
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
static DEVICE_ATTR(hard_offset, S_IRUGO|S_IWUSR, yas_hard_offset_show,
		yas_hard_offset_store);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537

#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static DEVICE_ATTR(hard_offset, S_IRUGO|S_IWUSR, yas_hard_offset_show, NULL);
static DEVICE_ATTR(mag_average_sample, S_IRUGO|S_IWUSR,
		yas_mag_average_sample_show, yas_mag_average_sample_store);
#endif

static struct attribute *yas_attributes[] = {

	&dev_attr_data.attr,
	&dev_attr_position.attr,
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	&dev_attr_hard_offset.attr,

#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	&dev_attr_mag_average_sample.attr,
#endif
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
	ktime_t timestamp;
	timestamp = ktime_get_boottime();
	time_before = yas_current_time();
	mutex_lock(&st->lock);
	ret = st->mag.measure(mag, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++)
			st->compass_data[i] = mag[0].xyz.v[i];
		st->data_valid=0x7f;
	}
	poll_delay = st->poll_delay;
	mutex_unlock(&st->lock);
	if (ret == 1) {
		/* report magnetic data in [nT] */
		input_report_abs(st->input_dev, ABS_X, mag[0].xyz.v[0]);
		input_report_abs(st->input_dev, ABS_Y, mag[0].xyz.v[1]);
		input_report_abs(st->input_dev, ABS_Z, mag[0].xyz.v[2]);
		input_event(st->input_dev,EV_SYN,SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
		input_event(st->input_dev,EV_SYN,SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
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

struct platform_device yas_mag_pd_device = {
    .name = "msensor",
    .id   = -1,
};

static ssize_t yas_mag_shipment_test(struct device_driver *ddri, char *buf)
{
	struct yas_state *st=NULL;
	int ret;
	struct yas532_self_test_result self_test_p;
	if(this_client)st=i2c_get_clientdata(this_client);

	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS532_SELF_TEST, (void *)&self_test_p);
	mutex_unlock(&st->lock);
	if(ret)
	{
		ret = -1;
		printk("%s ret %d\n", __func__, ret);
	}
	else
	{
		ret = 0x0F;
	}
	YAS_REM_PRINTK("%s id %d dir %d sx %d sy %d\n", __func__, 
		self_test_p.id, self_test_p.dir, self_test_p.sx, self_test_p.sy);
	YAS_REM_PRINTK("%s x %d y1 %d y2 %d\n", __func__, 
		self_test_p.xy1y2[0], self_test_p.xy1y2[1], self_test_p.xy1y2[2]);
	YAS_REM_PRINTK("%s x %d y %d z %d\n", __func__, 
		self_test_p.xyz[0], self_test_p.xyz[1], self_test_p.xyz[2]);

    return sprintf(buf, "%d\n", ret);
	
}
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	/*char sensordata[SENSOR_DATA_SIZE];*/
	char strbuf[0x20];
	struct yas_state *st=NULL;
	int ret,i;
	struct yas_data mag[1];
	if(this_client)st=i2c_get_clientdata(this_client);

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
static int yas_compass_parse_dt(struct device *dev, struct yas_state *st)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "ak,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read yas,layout\n");
		return rc;
	} else {
		rc  = st->mag.set_position(temp_val);;
	}

	return 0;
}
#else
static int yas_compass_parse_dt(struct device *dev, struct yas_state *st)
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

static int yas_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct yas_state *st = NULL;
	struct input_dev *input_dev = NULL;
	int ret, i;
	int flag_inputdev_unregistered = 0;

	this_client = i2c;
	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st = kzalloc(sizeof(struct yas_state), GFP_KERNEL);
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

	input_set_drvdata(input_dev, st);
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
	if (ret)
		goto error_free;

	st->cdev = sensors_cdev;
	st->cdev.sensors_enable = yas_enable_set;
	st->cdev.sensors_poll_delay = yas_poll_delay_set;

	ret = sensors_classdev_register(&i2c->dev, &st->cdev);
	if (ret) {
		printk(KERN_ERR TAGE "class device create failed: %d\n", ret);
		goto err_unregister_input_device;
	}

	st->dev = st->cdev.dev;

	ret = sysfs_create_group(&st->dev->kobj, &yas_attribute_group);
	if (ret)
		goto error_classdev_unregister;
	ret = yas_mag_driver_init(&st->mag);
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}

	ret = yas_compass_power_init(st, true);
	if (ret/*err < 0*/)
		goto error_remove_sysfs;

	ret = st->mag.init();
	if (ret < 0) {
		ret = -EFAULT;
		goto err_compass_pwr_init;
	}

	ret = yas_compass_parse_dt(&i2c->dev, st);

    if((ret = platform_driver_register(&yas_mag_pd_driver)))
	{
		printk(KERN_ERR "%s failed to register yas_mag_driver err %d\n", __func__, ret);
		goto err_compass_pwr_init/*err_free_irq2*/;
	}

    if((ret = platform_device_register(&yas_mag_pd_device)))
    {
		printk(KERN_ERR "%s failed to register yas_mag_device err %d\n", __func__, ret);
		goto err_free_driver;
    }

	if((ret = yas_mag_create_attr(&yas_mag_pd_driver.driver)))
	{
		printk("%s lis3dh create attribute err = %d\n", __func__, ret);
		goto err_free_device;
	}

	return 0;

err_free_device:
	platform_device_unregister(&yas_mag_pd_device);
err_free_driver:
	platform_driver_unregister(&yas_mag_pd_driver);
err_compass_pwr_init:
	yas_compass_power_init(st, false);
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
		printk("%s null pointer\n", __func__);
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
		printk("%s null pointer\n", __func__);
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
	{YAS_MSM_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);


static struct of_device_id yas_match_table[] = {
	{ .compatible = "yas533_mag", },
	{ },
};

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_MSM_NAME,
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
MODULE_VERSION("1.6.2.1020d");
