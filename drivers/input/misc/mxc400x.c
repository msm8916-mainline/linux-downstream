/*
 * Copyright (c) 2014, Linux Foundation. All rights reserved.
 * Linux Foundation chooses to take subject only to the GPLv2 license
 * terms, and distributes only under these terms.
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/sensors.h>
#include <asm/uaccess.h>

#include "mxc400x.h"

#define MXC400X_DEFAULT_INTERVAL_MS	100

#define MXC400X_MIN_DELAY               10
#define MXC400X_MAX_DELAY               1000

/* POWER SUPPLY VOLTAGE RANGE */
#define MXC400X_VDD_MIN_UV	2000000
#define MXC400X_VDD_MAX_UV	3300000
#define MXC400X_VIO_MIN_UV	1750000
#define MXC400X_VIO_MAX_UV	1950000

//yhj add for debug 20150911
#define YHJ_DBG 0
#if YHJ_DBG
#define Y_DBG(s,args...)	{printk("////yhj : func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define Y_DBG(s,args...) {}
#endif
//yhj add end  20150911

enum {
	OBVERSE_X_AXIS_FORWARD = 0,
	OBVERSE_X_AXIS_RIGHTWARD,
	OBVERSE_X_AXIS_BACKWARD,
	OBVERSE_X_AXIS_LEFTWARD,
	REVERSE_X_AXIS_FORWARD,
	REVERSE_X_AXIS_RIGHTWARD,
	REVERSE_X_AXIS_BACKWARD,
	REVERSE_X_AXIS_LEFTWARD,
	MXC400X_DIR_COUNT,
};

static char *mxc400x_dir[MXC400X_DIR_COUNT] = {
	[OBVERSE_X_AXIS_FORWARD] = "obverse-x-axis-forward",
	[OBVERSE_X_AXIS_RIGHTWARD] = "obverse-x-axis-rightward",
	[OBVERSE_X_AXIS_BACKWARD] = "obverse-x-axis-backward",
	[OBVERSE_X_AXIS_LEFTWARD] = "obverse-x-axis-leftward",
	[REVERSE_X_AXIS_FORWARD] = "reverse-x-axis-forward",
	[REVERSE_X_AXIS_RIGHTWARD] = "reverse-x-axis-rightward",
	[REVERSE_X_AXIS_BACKWARD] = "reverse-x-axis-backward",
	[REVERSE_X_AXIS_LEFTWARD] = "reverse-x-axis-leftward",
};

static s8 mxc400x_rotation_matrix[MXC400X_DIR_COUNT][9] = {
	[OBVERSE_X_AXIS_FORWARD] = {0, -1, 0, 1, 0, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_RIGHTWARD] = {1, 0, 0, 0, 1, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_BACKWARD] = {0, 1, 0, -1, 0, 0, 0, 0, 1},
	[OBVERSE_X_AXIS_LEFTWARD] = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
	[REVERSE_X_AXIS_FORWARD] = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	[REVERSE_X_AXIS_RIGHTWARD] = {1, 0, 0, 0, -1, 0, 0, 0, -1},
	[REVERSE_X_AXIS_BACKWARD] = {0, -1, 0, -1, 0, 0, 0, 0, -1},
	[REVERSE_X_AXIS_LEFTWARD] = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
};

struct mxc400x_vec {
	int x;
	int y;
	int z;
	unsigned char temp;
};

struct mxc400x_data {
	struct mutex		ecompass_lock;
	struct mutex		ops_lock;
	struct delayed_work	dwork;
	struct sensors_classdev	cdev;
	struct mxc400x_vec	last;
	struct workqueue_struct *data_wq;
	struct i2c_client	*i2c;
	struct input_dev	*idev;
	struct regulator	*vdd;
	struct regulator	*vio;
	struct regmap		*regmap;

	int			dir;
//	int			auto_report;
	int			enable;
	int			poll_interval;
	int			power_enabled;
	unsigned long		timeout;
};

static struct sensors_classdev sensors_cdev = {
	.name = "mxc400x-acc",
	.vendor = "MEMSIC, Inc",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "78.4",
	.resolution = "0.0488228125",
	.sensor_power = "0.35",
	.min_delay = MXC400X_MIN_DELAY * 1000,   //10ms
	.max_delay = MXC400X_MAX_DELAY * 1000,  
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = MXC400X_DEFAULT_INTERVAL_MS,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

//static struct i2c_client *this_client;
//static struct regmap *this_regmap;
//static DEFINE_MUTEX(ecompass_ioctl_lock);
static struct mxc400x_data *mxc400x_data_struct;

static int mxc400x_read_xyz(struct mxc400x_data *memsic,
		struct mxc400x_vec *vec)
{
//	int count = 0;
	unsigned char data[6];
//	unsigned char temp;
	unsigned int temp;
//	unsigned int status;
	struct mxc400x_vec tmp;
	int rc = 0;

	mutex_lock(&memsic->ecompass_lock);
	
	/* read xyz raw data */
	rc = regmap_bulk_read(memsic->regmap, MXC400X_REG_DATA, data, 6);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MXC400X_REG_DATA, __LINE__, rc);
		goto exit;
	}
	rc = regmap_read(memsic->regmap, MXC400X_REG_TEMP, &temp);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed at %d.(%d)\n",
				MXC400X_REG_TEMP, __LINE__, rc);
		}
	tmp.x = (s16)(data[0] << 8 | data[1]) >> 4;
	tmp.y = (s16)(data[2] << 8 | data[3]) >> 4;
	tmp.z = (s16)(data[4] << 8 | data[5]) >> 4;

	dev_dbg(&memsic->i2c->dev, "raw data:%d %d %d %d %d %d",
			data[0], data[1], data[2], data[3], data[4], data[5]);
	dev_dbg(&memsic->i2c->dev, "raw x:%d y:%d z:%d\n", tmp.x, tmp.y, tmp.z);


	//printk("mxc400x raw data:%d %d %d %d %d %d temp: %d\n",
	//		data[0], data[1], data[2], data[3], data[4], data[5], temp);
	//printk("mxc400x out %d %d %d %d \n", vec->x, vec->y, vec->z, vec->temp);

	vec->x = -tmp.y * 4;
	vec->y = tmp.x * 4;
	vec->z = tmp.z * 4;	
	vec->temp = temp;

exit:
	mutex_unlock(&memsic->ecompass_lock);
	return rc;
}

static void mxc400x_poll(struct work_struct *work)
{
	int ret;
	s8 *tmp;
	struct mxc400x_vec vec;
	struct mxc400x_vec report;
	struct mxc400x_data *memsic = container_of((struct delayed_work *)work,
			struct mxc400x_data, dwork);

	vec.x = vec.y = vec.z = vec.temp = 0;

	ret = mxc400x_read_xyz(memsic, &vec);
	if (ret) {
		dev_warn(&memsic->i2c->dev, "read xyz failed\n");
		goto exit;
	}

	tmp = &mxc400x_rotation_matrix[memsic->dir][0];
	report.x = tmp[0] * vec.x + tmp[1] * vec.y + tmp[2] * vec.z;
	report.y = tmp[3] * vec.x + tmp[4] * vec.y + tmp[5] * vec.z;
	report.z = tmp[6] * vec.x + tmp[7] * vec.y + tmp[8] * vec.z;
	report.temp = vec.temp;
	
	input_report_abs(memsic->idev, ABS_X, report.x);
	input_report_abs(memsic->idev, ABS_Y, report.y);
	input_report_abs(memsic->idev, ABS_Z, report.z);
	input_report_abs(memsic->idev, ABS_HAT0X, report.temp);
	input_sync(memsic->idev);

exit:
	/*
	schedule_delayed_work(&memsic->dwork,
			msecs_to_jiffies(memsic->poll_interval));
			*/
	queue_delayed_work(memsic->data_wq, &memsic->dwork, msecs_to_jiffies(memsic->poll_interval));
}

static struct input_dev *mxc400x_init_input(struct i2c_client *client)
{
	int status;
	struct input_dev *input = NULL;

	input = devm_input_allocate_device(&client->dev);
	if (!input)
		return NULL;

	input->name = "accelerometer";
	input->phys = "mxc400x/input0";
	input->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_Z, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_HAT0X, -2047, 2047, 0, 0);

//	input_set_capability(input, EV_REL, REL_X);
//	input_set_capability(input, EV_REL, REL_Y);
//	input_set_capability(input, EV_REL, REL_Z);

	status = input_register_device(input);
	if (status) {
		dev_err(&client->dev,
			"error registering input device\n");
		return NULL;
	}

	return input;
}

static int mxc400x_power_init(struct mxc400x_data *data)
{
	int rc;

	data->vdd = devm_regulator_get(&data->i2c->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->i2c->dev,
				"Regualtor get failed vdd rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd,
				MXC400X_VDD_MIN_UV, MXC400X_VDD_MAX_UV);
		if (rc) {
			dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
			goto exit;
		}
	}

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->i2c->dev,
				"Regulator enable vdd failed rc=%d\n", rc);
		goto exit;
	}
	data->vio = devm_regulator_get(&data->i2c->dev, "vio");
	if (IS_ERR(data->vio)) {
		rc = PTR_ERR(data->vio);
		dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		rc = regulator_set_voltage(data->vio,
				MXC400X_VIO_MIN_UV, MXC400X_VIO_MAX_UV);
		if (rc) {
			dev_err(&data->i2c->dev,
					"Regulator set failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}
	}
	rc = regulator_enable(data->vio);
	if (rc) {
		dev_err(&data->i2c->dev,
				"Regulator enable vio failed rc=%d\n", rc);
		goto reg_vdd_set;
	}

	 /* The minimum time to operate device after VDD valid is 10 ms. */
	//usleep_range(15000, 20000);
	usleep_range(300000, 400000);
	data->power_enabled = true;

	return 0;

reg_vdd_set:
	regulator_disable(data->vdd);
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, MXC400X_VDD_MAX_UV);
exit:
	return rc;

}

static int mxc400x_power_deinit(struct mxc400x_data *data)
{
	if (!IS_ERR_OR_NULL(data->vio)) {
		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
					MXC400X_VIO_MAX_UV);

		regulator_disable(data->vio);
	}

	if (!IS_ERR_OR_NULL(data->vdd)) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
					MXC400X_VDD_MAX_UV);

		regulator_disable(data->vdd);
	}

	data->power_enabled = false;

	return 0;
}

static int mxc400x_power_set(struct mxc400x_data *memsic, bool on)
{
	int rc = 0;

	if (!on && memsic->power_enabled) {
		mutex_lock(&memsic->ecompass_lock);

		rc = regulator_disable(memsic->vdd);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(memsic->vio);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		memsic->power_enabled = false;

		usleep_range(300000, 350000);
		mutex_unlock(&memsic->ecompass_lock);
		return rc;
	} else if (on && !memsic->power_enabled) {
		mutex_lock(&memsic->ecompass_lock);

		rc = regulator_enable(memsic->vdd);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(memsic->vio);
		if (rc) {
			dev_err(&memsic->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		memsic->power_enabled = true;

		mutex_unlock(&memsic->ecompass_lock);

		/* The minimum time to operate after VDD valid is 10 ms */
		usleep_range(20000, 25000);

		return rc;
	} else {
		dev_warn(&memsic->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, memsic->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(memsic->vio);
err_vdd_enable:
	mutex_unlock(&memsic->ecompass_lock);
	return rc;

err_vio_disable:
	if (regulator_enable(memsic->vdd))
		dev_warn(&memsic->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	mutex_unlock(&memsic->ecompass_lock);
	return rc;
}
static int mxc400x_check_device(struct mxc400x_data *memsic)
{
	unsigned int data;
	int rc;

	rc = regmap_read(memsic->regmap, MXC400X_REG_PRODUCTID_0, &data);
	if (rc) {
		dev_err(&memsic->i2c->dev, "read reg %d failed.(%d)\n",
				MXC400X_REG_PRODUCTID_0, rc);
		return rc;

	}

	if (data != MXC400X_ID)
		return -ENODEV;

	return 0;
}

static int mxc400x_parse_dt(struct i2c_client *client,
		struct mxc400x_data *memsic)
{
	struct device_node *np = client->dev.of_node;
	const char *tmp;
	int rc;
	int i;

	rc = of_property_read_string(np, "memsic,dir", &tmp);


	/* does not have a value or the string is not null-terminated */
	if (rc && (rc != -EINVAL)) {
		dev_err(&client->dev, "Unable to read memsic,dir\n");
		return rc;
	}

	for (i = 0; i < ARRAY_SIZE(mxc400x_dir); i++) {
		if (strcmp(mxc400x_dir[i], tmp) == 0)
			break;
	}

	if (i >= ARRAY_SIZE(mxc400x_dir)) {
		dev_err(&client->dev, "Invalid memsic,dir property");
		return -EINVAL;
	}
	Y_DBG(" i= %d.\n",i);
	memsic->dir = i;

	return 0;
}

static int mxc400x_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int rc = 0;
	struct mxc400x_data *memsic = container_of(sensors_cdev,
			struct mxc400x_data, cdev);
	printk("///yhj add in %s,enable=%d.\n",__func__,enable);
	mutex_lock(&memsic->ops_lock);

	if (enable && (!memsic->enable)) {
		rc = mxc400x_power_set(memsic, true);
		if (rc) {
			dev_err(&memsic->i2c->dev, "Power up failed\n");
			goto exit;
		}

		rc = regmap_write(memsic->regmap, MXC400X_REG_CTRL,
				MXC400X_AWAKE);
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed.(%d)\n",
					MXC400X_REG_CTRL, rc);
			goto exit;
		}

		memsic->timeout = jiffies;
		/*
		schedule_delayed_work(&memsic->dwork,
				msecs_to_jiffies(memsic->poll_interval));
		*/
		
		usleep_range(300000, 400000);
		//printk("memsic delay \n");
		queue_delayed_work(memsic->data_wq, &memsic->dwork, msecs_to_jiffies(memsic->poll_interval));
	} 
	else if ((!enable) && memsic->enable) {
		rc = regmap_write(memsic->regmap, MXC400X_REG_CTRL,
				MXC400X_SLEEP);
		if (rc) {
			dev_err(&memsic->i2c->dev, "write reg %d failed.(%d)\n",
					MXC400X_REG_CTRL, rc);
			goto exit;
		}

			cancel_delayed_work_sync(&memsic->dwork);

		if (mxc400x_power_set(memsic, false))
			dev_warn(&memsic->i2c->dev, "Power off failed\n");
	} else {
		dev_warn(&memsic->i2c->dev,
				"ignore enable state change from %d to %d\n",
				memsic->enable, enable);
	}
	memsic->enable = enable;
exit:
	mutex_unlock(&memsic->ops_lock);
	return rc;
}

static int mxc400x_set_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct mxc400x_data *memsic = container_of(sensors_cdev,
			struct mxc400x_data, cdev);
	printk("!!!yhj add %s ,delay_msec=%d.\n",__func__,delay_msec);
	mutex_lock(&memsic->ops_lock);
	if (memsic->poll_interval != delay_msec)
		memsic->poll_interval = delay_msec;
		mod_delayed_work(system_wq, &memsic->dwork,
				msecs_to_jiffies(delay_msec));
	mutex_unlock(&memsic->ops_lock);

	return 0;
}

static struct regmap_config mxc400x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int mxc400x_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int mxc400x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mxc400x_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *pa = (void __user *)arg;
	unsigned char data[16] = {0};
	unsigned char reg_addr;
	//unsigned char reg_num;
	unsigned int reg_value;
	int rc = -1;

	mutex_lock(&mxc400x_data_struct->ops_lock);

	switch (cmd) {
		case MXC400X_IOCTL_READ_REG:
			if (copy_from_user(&reg_addr, pa, sizeof(reg_addr))) {
				mutex_unlock(&mxc400x_data_struct->ecompass_lock);
				return -EFAULT;
			}
			rc = regmap_read(mxc400x_data_struct->regmap, reg_addr, &reg_value);
			if (rc) {
				dev_err(&mxc400x_data_struct->i2c->dev, "read reg %d failed at %d.(%d)\n",
						reg_addr, __LINE__, rc);
			}
			if (copy_to_user(pa, &reg_value, sizeof(reg_value))) {
				mutex_unlock(&mxc400x_data_struct->ecompass_lock);
				return -EFAULT;
			}
			break;
		case MXC400X_IOCTL_WRITE_REG:
			if (copy_from_user(&data, pa, sizeof(data))){
				mutex_unlock(&mxc400x_data_struct->ecompass_lock);
				printk("memsic1 copy from file failed\n");
				return -EFAULT;
			}

			rc = regmap_write(mxc400x_data_struct->regmap, data[0],
					data[1]);
			if (rc) {
				dev_err(&mxc400x_data_struct->i2c->dev, "write reg %d failed at %d.(%d)\n",
						data[1], __LINE__, rc);
				printk("write reg %d failed at %d.(%d)\n",
						data[1], __LINE__, rc);
				mutex_unlock(&mxc400x_data_struct->ops_lock);
				return -EFAULT;
			}
			break;
		default:
			break;
	}

	mutex_unlock(&mxc400x_data_struct->ops_lock);

	return 0;

}

static struct file_operations mxc400x_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc400x_open,
	.release	= mxc400x_release,
	.unlocked_ioctl = mxc400x_ioctl,
};

static struct miscdevice mxc400x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MXC400X_I2C_NAME,
	.fops = &mxc400x_fops,
};


static int mxc400x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
	struct mxc400x_data *memsic;
	dev_dbg(&client->dev, "probing mxc400x\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("mxc400x i2c functionality check failed.\n");
		res = -ENODEV;
		goto out;
	}

	memsic = devm_kzalloc(&client->dev, sizeof(struct mxc400x_data),
			GFP_KERNEL);
	if (!memsic) {
		dev_err(&client->dev, "memory allocation failed.\n");
		res = -ENOMEM;
		goto out;
	}

	mxc400x_data_struct = memsic;
	
	if (client->dev.of_node) {
		res = mxc400x_parse_dt(client, memsic);
		if (res) {
			dev_err(&client->dev,
				"Unable to parse platform data.(%d)", res);
			goto out;
		}
	} else {
		memsic->dir = 0;
//		memsic->auto_report = 1;
	}

	memsic->i2c = client;
	dev_set_drvdata(&client->dev, memsic);

	mutex_init(&memsic->ecompass_lock);
	mutex_init(&memsic->ops_lock);

	memsic->regmap = devm_regmap_init_i2c(client, &mxc400x_regmap_config);
	if (IS_ERR(memsic->regmap)) {
		dev_err(&client->dev, "Init regmap failed.(%ld)",
				PTR_ERR(memsic->regmap));
		res = PTR_ERR(memsic->regmap);
		goto out;
	}
	
	res = mxc400x_power_init(memsic);
	if (res) {
		dev_err(&client->dev, "Power up mxc400x failed\n");
		goto out;
	}

	res = mxc400x_check_device(memsic);
	if (res) {
		dev_err(&client->dev, "Check device failed\n");
//		goto out_check_device;
	}

	memsic->idev = mxc400x_init_input(client);
	if (!memsic->idev) {
		dev_err(&client->dev, "init input device failed\n");
		res = -ENODEV;
		goto out_init_input;
	}

	INIT_DELAYED_WORK(&memsic->dwork, mxc400x_poll);

	memsic->data_wq = create_freezable_workqueue("mxc400x_data_work");
		if (!memsic->data_wq) {
			dev_err(&client->dev, "Cannot get create workqueue!\n");
		goto out_init_input;
		}

	memsic->cdev = sensors_cdev;
	memsic->cdev.sensors_enable = mxc400x_set_enable;
	memsic->cdev.sensors_poll_delay = mxc400x_set_poll_delay;
	res = sensors_classdev_register(&client->dev, &memsic->cdev);
	if (res) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto out_register_classdev;
	}

	res = misc_register(&mxc400x_device);
	if (res) {
		pr_err("%s: mxc400x_device register failed\n", __FUNCTION__);
		goto out_deregister;
	}

	res = mxc400x_power_set(memsic, false);
	if (res) {
		dev_err(&client->dev, "Power off failed\n");
		goto out_power_set;
	}

	memsic->poll_interval = MXC400X_DEFAULT_INTERVAL_MS;

	dev_info(&client->dev, "mxc400x successfully probed\n");

	return 0;

out_power_set:
	sensors_classdev_unregister(&memsic->cdev);
out_deregister:
	misc_deregister(&mxc400x_device);
out_register_classdev:
	destroy_workqueue(memsic->data_wq);
	input_unregister_device(memsic->idev);
out_init_input:
//out_check_device:
	mxc400x_power_deinit(memsic);
out:
	return res;
}

static int mxc400x_remove(struct i2c_client *client)
{
	struct mxc400x_data *memsic = dev_get_drvdata(&client->dev);

	sensors_classdev_unregister(&memsic->cdev);
	misc_deregister(&mxc400x_device);
	destroy_workqueue(memsic->data_wq);
	mxc400x_power_deinit(memsic);

	if (memsic->idev)
		input_unregister_device(memsic->idev);

	return 0;
}

static int mxc400x_suspend(struct device *dev)
{
	int res = 0;
	struct mxc400x_data *memsic = dev_get_drvdata(dev);
	printk("%s ,memsic->enable=%d.\n",__func__,memsic->enable);
	dev_dbg(dev, "suspended\n");

	if (memsic->enable) {
		
		res= regmap_write(memsic->regmap, MXC400X_REG_CTRL,
				MXC400X_SLEEP);
		if (res) {
			dev_err(&memsic->i2c->dev, "write reg %d failed.(%d)\n",
					MXC400X_REG_CTRL, res);
			goto exit;
		}
			cancel_delayed_work_sync(&memsic->dwork);

		res = mxc400x_power_set(memsic, false);
		if (res) {
			dev_err(dev, "failed to suspend mxc400x\n");
			goto exit;
		}
	}
exit:
	return res;
}

static int mxc400x_resume(struct device *dev)
{
	int res = 0;
	struct mxc400x_data *memsic = dev_get_drvdata(dev);
	printk("%s ,memsic->enable=%d.\n",__func__,memsic->enable);
	dev_dbg(dev, "resumed\n");

	if (memsic->enable) {
		res = mxc400x_power_set(memsic, true);
		if (res) {
			dev_err(&memsic->i2c->dev, "!!!!%s Power enable failed\n",__func__);
			goto exit;
		}
		
		res= regmap_write(memsic->regmap, MXC400X_REG_CTRL,
				MXC400X_AWAKE);
		if (res) {
			dev_err(&memsic->i2c->dev, "write reg %d failed.(%d)\n",
					MXC400X_REG_CTRL, res);
			goto exit;
		}
		/*
			schedule_delayed_work(&memsic->dwork,
				msecs_to_jiffies(memsic->poll_interval));
		*/
		queue_delayed_work(memsic->data_wq, &memsic->dwork, msecs_to_jiffies(memsic->poll_interval));
	}

exit:
	return res;
}

static const struct i2c_device_id mxc400x_id[] = {
	{ MXC400X_I2C_NAME, 0 },
	{ }
};

static struct of_device_id mxc400x_match_table[] = {
	{ .compatible = "memsic,mxc400x", },
	{ },
};

static const struct dev_pm_ops mxc400x_pm_ops = {
	.suspend = mxc400x_suspend,
	.resume = mxc400x_resume,
};

static struct i2c_driver mxc400x_driver = {
	.probe 		= mxc400x_probe,
	.remove 	= mxc400x_remove,
	.id_table	= mxc400x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= MXC400X_I2C_NAME,
		.of_match_table = mxc400x_match_table,
		.pm = &mxc400x_pm_ops,
	},
};

static int __init mxc400x_init(void)
{
	return i2c_add_driver(&mxc400x_driver);
}

static void __exit mxc400x_exit(void)
{
	i2c_del_driver(&mxc400x_driver);
}

//module_i2c_driver(mxc400x_driver);
late_initcall(mxc400x_init);
module_exit(mxc400x_exit);

MODULE_DESCRIPTION("MEMSIC MXC400X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");

