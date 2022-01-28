/* drivers/input/misc/hscdtd007.c - hscdtd007 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include "hscdtd007.h"

#define HSCDTD_DEBUG_IF			0
#define HSCDTD_HAS_RESET			0
#define HSCDTD_INPUT_DEVICE_NAME	"compass"
#define HSCDTD_DRDY_TIMEOUT_MS		100
#define HSCDTD_BASE_NUM			10

#define HSCDTD_IS_MAG_DATA_ENABLED() (hscdtd->enable_flag & (1 << MAG_DATA_FLAG))

/* POWER SUPPLY VOLTAGE RANGE */
#define HSCDTD007_VDD_MIN_UV	2000000
#define HSCDTD007_VDD_MAX_UV	3300000
#define HSCDTD007_VIO_MIN_UV	1750000
#define HSCDTD007_VIO_MAX_UV	1950000
#define STATUS_ERROR(st)	(((st) & (HSCDTD007_ST1_DRDY | \
				HSCDTD007_ST1_DOR  | \
				HSCDTD007_ST2_HOLF)) \
				!= HSCDTD007_ST1_DRDY)
#define REG_CNTL1_MODE(reg_cntl1)	(reg_cntl1 & 0x0F)

/* Save last device state for power down */
struct hscdtd_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct hscdtd_compass_data {
	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct device		*class_dev;
	struct class		*compass;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
	struct sensors_classdev	cdev;
	struct delayed_work	dwork;
	struct mutex		op_mutex;


	wait_queue_head_t	drdy_wq;
	wait_queue_head_t	open_wq;

	/* These two buffers are initialized at start up.
	   After that, the value is not changed */
	uint8_t sense_info[HSCDTD_SENSOR_INFO_SIZE];
	uint8_t sense_conf[HSCDTD_SENSOR_CONF_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[HSCDTD_SENSOR_DATA_SIZE];
	struct mutex accel_mutex;
	int16_t accel_data[3];

	/* Positive value means the device is working.
	   0 or negative value means the device is not woking,
	   i.e. in power-down mode. */
	int8_t	is_busy;

	struct mutex	val_mutex;
	uint32_t	enable_flag;
	int32_t		delay[HSCDTD_NUM_SENSORS];

	atomic_t	active;
	atomic_t	drdy;

	int	gpio_rstn;
	bool	power_enabled;
	bool	use_poll;
	struct	regulator		*vdd;
	struct	regulator		*vio;
	struct	hscdtd_sensor_state		state;
	struct	hscdtd007_platform_data	*pdata;
};

static struct sensors_classdev sensors_cdev = {
	.name = "hscdtd007-mag",
	.vendor = "ALPS Electric Co., Ltd",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "4800.0",
	.resolution = "0.15",
	.sensor_power = "0.2",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10000,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct hscdtd_compass_data *s_hscdtd;

struct i2c_client *hscdtd007_mag_i2c_client=NULL;

static int hscdtd_compass_power_set(struct hscdtd_compass_data *data, bool on);

/***** I2C I/O function ***********************************************/
static int hscdtd_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *rxData,
	int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int hscdtd_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
    printk(KERN_ERR "%s %x %x\n", __func__, txData[0], txData[1]);
	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

/***** hscdtd miscdevice functions *************************************/
static int HSCDTD_Set_CNTL(
	struct hscdtd_compass_data *hscdtd,
	uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&hscdtd->sensor_mutex);
	/* Busy check */
	if (hscdtd->is_busy > 0) {
		dev_err(&hscdtd->i2c->dev,
				"%s: device is busy.", __func__);
		err = -EBUSY;
	} else {
		/* Set measure mode */
		buffer[0] = HSCDTD_CTRL1;
		buffer[1] = 0x8a;
		printk(KERN_ERR "%s %x %x\n", __func__, buffer[0], buffer[1]);
		err = hscdtd_i2c_txdata(hscdtd->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&hscdtd->i2c->dev,
					"%s: Can not set CNTL.", __func__);
		} else {
			dev_vdbg(&hscdtd->i2c->dev,
					"Mode is set to (%d).", mode);
			/* Set flag */
			hscdtd->is_busy = 1;
			atomic_set(&hscdtd->drdy, 0);
			/* wait at least 100us after changing mode */
			HSCDTD_DELAY(100);
		}
	}
   {
	u8 buffer = HSCDTD_CTRL1;
	err = hscdtd_i2c_rxdata(hscdtd->i2c, &buffer, 1);
    printk(KERN_ERR "%s %x %x\n", __func__, HSCDTD_CTRL1, buffer);
	}
	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int HSCDTD_Set_PowerDown(
	struct hscdtd_compass_data *hscdtd)
{
	/*uint8_t buffer[2];*/
	int err=0;

	/***** lock *****/
	mutex_lock(&hscdtd->sensor_mutex);


	/* Clear status */
	hscdtd->is_busy = 0;
	atomic_set(&hscdtd->drdy, 0);

	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int HSCDTD_Reset(
	struct hscdtd_compass_data *hscdtd,
	int hard)
{
	int err;

#if HSCDTD_HAS_RESET
	uint8_t buffer[2];

	/***** lock *****/
	mutex_lock(&hscdtd->sensor_mutex);

	if (hard != 0) {
		gpio_set_value(hscdtd->gpio_rstn, 0);
		HSCDTD_DELAY(5);
		gpio_set_value(hscdtd->gpio_rstn, 1);
		/* No error is returned */
		err = 0;
	} else {
		buffer[0] = HSCDTD_CTRL3;
		buffer[1] = HSCDTD_RESET_DATA;
		err = hscdtd_i2c_txdata(hscdtd->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&hscdtd->i2c->dev,
				"%s: Can not set SRST bit.", __func__);
		} else {
			dev_dbg(&hscdtd->i2c->dev, "Soft reset is done.");
		}
	}
	/* Device will be accessible 100 us after */
	HSCDTD_DELAY(5000);
	/* Clear status */
	hscdtd->is_busy = 0;
	atomic_set(&hscdtd->drdy, 0);

	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

#else
	err = HSCDTD_Set_PowerDown(hscdtd);
#endif

	return err;
}

static int HSCDTD_SetMode(
	struct hscdtd_compass_data *hscdtd,
	uint8_t mode)
{
	int err;

	switch (REG_CNTL1_MODE(mode)) {
	case HSCDTD_MODE_SNG_MEASURE:
	case HSCDTD_MODE_SELF_TEST:
	case HSCDTD_MODE_CONT1_MEASURE:
	case HSCDTD_MODE_CONT2_MEASURE:
	case HSCDTD_MODE_EXT_TRIG_MEASURE:
	case HSCDTD_MODE_FUSE_ACCESS:
		err = HSCDTD_Set_CNTL(hscdtd, mode);
		break;
	case HSCDTD_MODE_POWERDOWN:
		err = HSCDTD_Set_PowerDown(hscdtd);
		break;
	default:
		dev_err(&hscdtd->i2c->dev,
				"%s: Unknown mode(%d).", __func__, mode);
		return -EINVAL;
	}
	hscdtd->state.mode = mode;
	return err;
}

static void HSCDTD_SetYPR(
	struct hscdtd_compass_data *hscdtd,
	int *rbuf)
{
	uint32_t ready;
	dev_vdbg(&hscdtd->i2c->dev, "%s: flag =0x%X", __func__, rbuf[0]);
	dev_vdbg(&hscdtd->input->dev, "  Acc [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
	dev_vdbg(&hscdtd->input->dev, "  Geo [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[5], rbuf[6], rbuf[7], rbuf[8]);
	dev_vdbg(&hscdtd->input->dev, "  Orientation : %6d,%6d,%6d",
		rbuf[9], rbuf[10], rbuf[11]);
	dev_vdbg(&hscdtd->input->dev, "  Rotation V  : %6d,%6d,%6d,%6d",
		rbuf[12], rbuf[13], rbuf[14], rbuf[15]);

	/* No events are reported */
	if (!rbuf[0]) {
		dev_dbg(&hscdtd->i2c->dev, "Don't waste a time.");
		return;
	}

	mutex_lock(&hscdtd->val_mutex);
	ready = (hscdtd->enable_flag & (uint32_t)rbuf[0]);
	mutex_unlock(&hscdtd->val_mutex);

	/* Report acceleration sensor information */
	if (ready & ACC_DATA_READY) {
		input_report_abs(hscdtd->input, ABS_X, rbuf[1]);
		input_report_abs(hscdtd->input, ABS_Y, rbuf[2]);
		input_report_abs(hscdtd->input, ABS_Z, rbuf[3]);
		input_report_abs(hscdtd->input, ABS_RX, rbuf[4]);
	}
	/* Report magnetic vector information */
	if (ready & MAG_DATA_READY) {
		input_report_abs(hscdtd->input, ABS_X, rbuf[5]);
		input_report_abs(hscdtd->input, ABS_Y, rbuf[6]);
		input_report_abs(hscdtd->input, ABS_Z, rbuf[7]);
	}
	/* Report fusion sensor information */
	if (ready & FUSION_DATA_READY) {
		/* Orientation */
		input_report_abs(hscdtd->input, ABS_HAT0Y, rbuf[9]);
		input_report_abs(hscdtd->input, ABS_HAT1X, rbuf[10]);
		input_report_abs(hscdtd->input, ABS_HAT1Y, rbuf[11]);
		/* Rotation Vector */
		input_report_abs(hscdtd->input, ABS_TILT_X, rbuf[12]);
		input_report_abs(hscdtd->input, ABS_TILT_Y, rbuf[13]);
		input_report_abs(hscdtd->input, ABS_TOOL_WIDTH, rbuf[14]);
		input_report_abs(hscdtd->input, ABS_VOLUME, rbuf[15]);
	}

	input_sync(hscdtd->input);
}

/* This function will block a process until the latest measurement
 * data is available.
 */
static int HSCDTD_GetData(
	struct hscdtd_compass_data *hscdtd,
	uint8_t *rbuf,
	int size)
{
	int err;

	/* Block! */
	err = wait_event_interruptible_timeout(
			hscdtd->drdy_wq,
			atomic_read(&hscdtd->drdy),
			msecs_to_jiffies(HSCDTD_DRDY_TIMEOUT_MS));

	if (err < 0) {
		dev_err(&hscdtd->i2c->dev,
			"%s: wait_event failed (%d).", __func__, err);
		return err;
	}
	if (!atomic_read(&hscdtd->drdy)) {
		dev_err(&hscdtd->i2c->dev,
			"%s: DRDY is not set.", __func__);
		return -ENODATA;
	}

	/***** lock *****/
	mutex_lock(&hscdtd->sensor_mutex);

	memcpy(rbuf, hscdtd->sense_data, size);
	atomic_set(&hscdtd->drdy, 0);

	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

	return 0;
}

static int HSCDTD_GetData_Poll(
	struct hscdtd_compass_data *hscdtd,
	uint8_t *rbuf,
	int size)
{
	uint8_t buffer[HSCDTD_SENSOR_DATA_SIZE];
	int err;
	static u8 is_first=1;
	int i;

    if(is_first)
    {
		buffer[0] = HSCDTD_CTRL3;
		buffer[1] = 0x40;
		err = hscdtd_i2c_txdata(hscdtd->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&hscdtd->i2c->dev, "%s failed.", __func__);
			return err;
		}
		HSCDTD_DELAY(5000);
		is_first=0;
	}
	buffer[0] = HSCDTD_CTRL3;	
	err = hscdtd_i2c_rxdata(hscdtd->i2c, buffer, 1);
    printk(KERN_ERR "%s %x %x\n", __func__, HSCDTD_CTRL3, buffer[0]);
	buffer[0] = HSCDTD_CTRL1;	
	err = hscdtd_i2c_rxdata(hscdtd->i2c, buffer, 1);
    printk(KERN_ERR "%s %x %x\n", __func__, HSCDTD_CTRL1, buffer[0]);
	buffer[0] = HSCDTD_CTRL4;
	err = hscdtd_i2c_rxdata(hscdtd->i2c, buffer, 1);
    printk(KERN_ERR "%s %x %x\n", __func__, HSCDTD_CTRL4, buffer[0]);
	buffer[0] = HSCDTD_STATUS;
	err = hscdtd_i2c_rxdata(hscdtd->i2c, buffer, 1);
    printk(KERN_ERR "%s %x %x\n", __func__, HSCDTD_STATUS, buffer[0]);


	buffer[0] = HSCDTD_XOUT;	
	err = hscdtd_i2c_rxdata(hscdtd->i2c, buffer, HSCDTD_DATA_ACCESS_NUM);	
	if (err < 0){
		dev_err(&hscdtd->i2c->dev, "%s failed.", __func__);
		return err;	
	}

	for(i=0; i<6; i++)
	{
       printk(KERN_ERR "%s data %x\n", __func__, buffer[i]);
	}

	memcpy(rbuf, buffer, size);
	atomic_set(&hscdtd->drdy, 0);

	buffer[0] = HSCDTD_CTRL3;
	buffer[1] = 0x40;
	err = hscdtd_i2c_txdata(hscdtd->i2c, buffer, 2);
	if (err < 0) {
		dev_err(&hscdtd->i2c->dev, "%s failed.", __func__);
		return err;
	}
	/*HSCDTD_DELAY(10000);*/

	/***** lock *****/
	mutex_lock(&hscdtd->sensor_mutex);
	hscdtd->is_busy = 0;
	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

	return 0;
}

static int HSCDTD_GetOpenStatus(
	struct hscdtd_compass_data *hscdtd)
{
	return wait_event_interruptible(
			hscdtd->open_wq, (atomic_read(&hscdtd->active) > 0));
}

static int HSCDTD_GetCloseStatus(
	struct hscdtd_compass_data *hscdtd)
{
	return wait_event_interruptible(
			hscdtd->open_wq, (atomic_read(&hscdtd->active) <= 0));
}

static int HSCDTD_Open(struct inode *inode, struct file *file)
{
	file->private_data = s_hscdtd;
	return nonseekable_open(inode, file);
}

static int HSCDTD_Release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
HSCDTD_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct hscdtd_compass_data *hscdtd = file->private_data;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	uint8_t i2c_buf[HSCDTD_RWBUF_SIZE];		/* for READ/WRITE */
	uint8_t dat_buf[HSCDTD_SENSOR_DATA_SIZE];/* for GET_DATA */
	int32_t ypr_buf[HSCDTD_YPR_DATA_SIZE];		/* for SET_YPR */
	int32_t delay[HSCDTD_NUM_SENSORS];	/* for GET_DELAY */
	int16_t acc_buf[3];	/* for GET_ACCEL */
	uint8_t mode;			/* for SET_MODE*/
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = 0;		/* Return value. */

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (argp == NULL) {
			dev_err(&hscdtd->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf))) {
			dev_err(&hscdtd->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			dev_err(&hscdtd->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			dev_err(&hscdtd->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			dev_err(&hscdtd->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			dev_err(&hscdtd->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
	case ECS_IOCTL_GET_INFO:
	case ECS_IOCTL_GET_CONF:
	case ECS_IOCTL_GET_DATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
	case ECS_IOCTL_GET_ACCEL:
		/* Check buffer pointer for writing a data later. */
		if (argp == NULL) {
			dev_err(&hscdtd->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_READ called.");
		if ((i2c_buf[0] < 1) || (i2c_buf[0] > (HSCDTD_RWBUF_SIZE-1))) {
			dev_err(&hscdtd->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = hscdtd_i2c_rxdata(hscdtd->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_WRITE called.");
		if ((i2c_buf[0] < 2) || (i2c_buf[0] > (HSCDTD_RWBUF_SIZE-1))) {
			dev_err(&hscdtd->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = hscdtd_i2c_txdata(hscdtd->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_RESET called.");
		ret = HSCDTD_Reset(hscdtd, hscdtd->gpio_rstn);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_SET_MODE called.");
		ret = HSCDTD_SetMode(hscdtd, mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_SET_YPR called.");
		HSCDTD_SetYPR(hscdtd, ypr_buf);
		break;
	case ECS_IOCTL_GET_DATA:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_DATA called.");
		if (hscdtd->i2c->irq)
			ret = HSCDTD_GetData(hscdtd, dat_buf, HSCDTD_SENSOR_DATA_SIZE);
		else
			ret = HSCDTD_GetData_Poll(
					hscdtd, dat_buf, HSCDTD_SENSOR_DATA_SIZE);

		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_OPEN_STATUS called.");
		ret = HSCDTD_GetOpenStatus(hscdtd);
		if (ret < 0) {
			dev_err(&hscdtd->i2c->dev,
				"Get Open returns error (%d).", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_CLOSE_STATUS called.");
		ret = HSCDTD_GetCloseStatus(hscdtd);
		if (ret < 0) {
			dev_err(&hscdtd->i2c->dev,
				"Get Close returns error (%d).", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_DELAY called.");
		mutex_lock(&hscdtd->val_mutex);
		delay[0] = ((hscdtd->enable_flag & ACC_DATA_READY) ?
				hscdtd->delay[0] : -1);
		delay[1] = ((hscdtd->enable_flag & MAG_DATA_READY) ?
				hscdtd->delay[1] : -1);
		delay[2] = ((hscdtd->enable_flag & FUSION_DATA_READY) ?
				hscdtd->delay[2] : -1);
		mutex_unlock(&hscdtd->val_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_INFO called.");
		break;
	case ECS_IOCTL_GET_CONF:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_CONF called.");
		break;
	case ECS_IOCTL_GET_LAYOUT:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_LAYOUT called.");
		break;
	case ECS_IOCTL_GET_ACCEL:
		dev_vdbg(&hscdtd->i2c->dev, "IOCTL_GET_ACCEL called.");
		mutex_lock(&hscdtd->accel_mutex);
		acc_buf[0] = hscdtd->accel_data[0];
		acc_buf[1] = hscdtd->accel_data[1];
		acc_buf[2] = hscdtd->accel_data[2];
		mutex_unlock(&hscdtd->accel_mutex);
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		/* +1  is for the first byte */
		if (copy_to_user(argp, &i2c_buf, i2c_buf[0]+1)) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		if (copy_to_user(argp, &hscdtd->sense_info,
					sizeof(hscdtd->sense_info))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		if (copy_to_user(argp, &hscdtd->sense_conf,
					sizeof(hscdtd->sense_conf))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &dat_buf, sizeof(dat_buf))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = atomic_read(&hscdtd->active);
		if (copy_to_user(argp, &status, sizeof(status))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &hscdtd->pdata->layout,
					sizeof(hscdtd->pdata->layout))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			dev_err(&hscdtd->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations HSCDTD_fops = {
	.owner = THIS_MODULE,
	.open = HSCDTD_Open,
	.release = HSCDTD_Release,
	.unlocked_ioctl = HSCDTD_ioctl,
};

/***** hscdtd sysfs functions ******************************************/
static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		device_remove_file(dev, &attrs[i]);
}

static int create_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;
	int err = 0;

	err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = sysfs_create_bin_file(kobj, &attrs[i]);
		if (0 != err)
			break;
	}

	if (0 != err) {
		for (--i; i >= 0 ; --i)
			sysfs_remove_bin_file(kobj, &attrs[i]);
	}

	return err;
}

static void remove_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		sysfs_remove_bin_file(kobj, &attrs[i]);
}

/*********************************************************************
 *
 * SysFS attribute functions
 *
 * directory : /sys/class/compass/hscdtdXXXX/
 * files :
 *  - enable_acc    [rw] [t] : enable flag for accelerometer
 *  - enable_mag    [rw] [t] : enable flag for magnetometer
 *  - enable_fusion [rw] [t] : enable flag for fusion sensor
 *  - delay_acc     [rw] [t] : delay in nanosecond for accelerometer
 *  - delay_mag     [rw] [t] : delay in nanosecond for magnetometer
 *  - delay_fusion  [rw] [t] : delay in nanosecond for fusion sensor
 *
 * debug :
 *  - mode       [w]  [t] : E-Compass mode
 *  - bdata      [r]  [t] : buffered raw data
 *  - asa        [r]  [t] : FUSEROM data
 *  - regs       [r]  [t] : read all registers
 *
 * [b] = binary format
 * [t] = text format
 */

/***** sysfs enable *************************************************/
static void hscdtd_compass_sysfs_update_status(
	struct hscdtd_compass_data *hscdtd)
{
	uint32_t en;
	mutex_lock(&hscdtd->val_mutex);
	en = hscdtd->enable_flag;
	mutex_unlock(&hscdtd->val_mutex);

	if (en == 0) {
		if (atomic_cmpxchg(&hscdtd->active, 1, 0) == 1) {
			wake_up(&hscdtd->open_wq);
			dev_dbg(hscdtd->class_dev, "Deactivated");
		}
	} else {
		if (atomic_cmpxchg(&hscdtd->active, 0, 1) == 0) {
			wake_up(&hscdtd->open_wq);
			dev_dbg(hscdtd->class_dev, "Activated");
		}
	}
	dev_dbg(&hscdtd->i2c->dev,
		"Status updated: enable=0x%X, active=%d",
		en, atomic_read(&hscdtd->active));
}

static int hscdtd_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	struct hscdtd_compass_data *hscdtd = container_of(sensors_cdev,
			struct hscdtd_compass_data, cdev);

    printk(KERN_ERR "%s enable %d\n", __func__, enable);
	mutex_lock(&hscdtd->val_mutex);
	hscdtd->enable_flag &= ~(1<<MAG_DATA_FLAG);
	hscdtd->enable_flag |= ((uint32_t)(enable))<<MAG_DATA_FLAG;
	mutex_unlock(&hscdtd->val_mutex);

	hscdtd_compass_sysfs_update_status(hscdtd);

	mutex_lock(&hscdtd->op_mutex);
	if (enable) {
		ret = hscdtd_compass_power_set(hscdtd, true);
		if (ret) {
			dev_err(&hscdtd->i2c->dev,
				"Power on fail! ret=%d\n", ret);
			goto exit;
		}
	}
    printk(KERN_ERR "%s use_poll %d auto_report %d\n", __func__, hscdtd->use_poll, hscdtd->pdata->auto_report);
	if (hscdtd->use_poll && hscdtd->pdata->auto_report) {
		if (enable) {
			HSCDTD_SetMode(hscdtd,
				0x1A | 0x10);
			hscdtd->delay[MAG_DATA_FLAG]=200;
			printk(KERN_ERR "%s delay %d\n", __func__, hscdtd->delay[MAG_DATA_FLAG]);
			schedule_delayed_work(&hscdtd->dwork,
					msecs_to_jiffies(
						hscdtd->delay[MAG_DATA_FLAG]));
		} else {
			cancel_delayed_work_sync(&hscdtd->dwork);
			HSCDTD_SetMode(hscdtd, HSCDTD_MODE_POWERDOWN);
		}
	} else {
		if (enable)
			enable_irq(hscdtd->i2c->irq);
		else
			disable_irq(hscdtd->i2c->irq);
	}

	if (!enable) {
		ret = hscdtd_compass_power_set(hscdtd, false);
		if (ret) {
			dev_err(&hscdtd->i2c->dev,
				"Power off fail! ret=%d\n", ret);
			goto exit;
		}
	}

exit:
	mutex_unlock(&hscdtd->op_mutex);
	return ret;
}

static ssize_t hscdtd_compass_sysfs_enable_show(
	struct hscdtd_compass_data *hscdtd, char *buf, int pos)
{
	int flag;

	mutex_lock(&hscdtd->val_mutex);
	flag = ((hscdtd->enable_flag >> pos) & 1);
	mutex_unlock(&hscdtd->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", flag);
}

static ssize_t hscdtd_compass_sysfs_enable_store(
	struct hscdtd_compass_data *hscdtd, char const *buf, size_t count, int pos)
{
	long en = 0;
	int ret = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, HSCDTD_BASE_NUM, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	mutex_lock(&hscdtd->op_mutex);
	ret = hscdtd_compass_power_set(hscdtd, en);
	if (ret) {
		dev_err(&hscdtd->i2c->dev,
			"Fail to configure device power!\n");
		goto exit;
	}

	mutex_lock(&hscdtd->val_mutex);
	hscdtd->enable_flag &= ~(1<<pos);
	hscdtd->enable_flag |= ((uint32_t)(en))<<pos;
	mutex_unlock(&hscdtd->val_mutex);

	hscdtd_compass_sysfs_update_status(hscdtd);

exit:
	mutex_unlock(&hscdtd->op_mutex);
	return ret ? ret : count;
}

/***** Acceleration ***/
static ssize_t hscdtd_enable_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return hscdtd_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t hscdtd_enable_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return hscdtd_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t hscdtd_enable_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return hscdtd_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t hscdtd_enable_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return hscdtd_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t hscdtd_enable_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return hscdtd_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t hscdtd_enable_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return hscdtd_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** sysfs delay **************************************************/
static int hscdtd_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct hscdtd_compass_data *hscdtd = container_of(sensors_cdev,
			struct hscdtd_compass_data, cdev);

	mutex_lock(&hscdtd->val_mutex);
	hscdtd->delay[MAG_DATA_FLAG] = delay_msec;
	mutex_unlock(&hscdtd->val_mutex);

	return 0;
}

static ssize_t hscdtd_compass_sysfs_delay_show(
	struct hscdtd_compass_data *hscdtd, char *buf, int pos)
{
	unsigned long val;

	mutex_lock(&hscdtd->val_mutex);
	val = hscdtd->delay[pos];
	mutex_unlock(&hscdtd->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", val);
}

static ssize_t hscdtd_compass_sysfs_delay_store(
	struct hscdtd_compass_data *hscdtd, char const *buf, size_t count, int pos)
{
	unsigned long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (kstrtoul(buf, HSCDTD_BASE_NUM, &val))
		return -EINVAL;

	mutex_lock(&hscdtd->val_mutex);
	hscdtd->delay[pos] = val;
	mutex_unlock(&hscdtd->val_mutex);

	return count;
}

/***** Accelerometer ***/
static ssize_t hscdtd_delay_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return hscdtd_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t hscdtd_delay_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return hscdtd_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t hscdtd_delay_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return hscdtd_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t hscdtd_delay_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return hscdtd_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t hscdtd_delay_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return hscdtd_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t hscdtd_delay_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return hscdtd_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** accel (binary) ***/
static ssize_t hscdtd_bin_accel_write(
	struct file *file,
	struct kobject *kobj,
	struct bin_attribute *attr,
		char *buf,
		loff_t pos,
		size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	int16_t *accel_data;

	if (size == 0)
		return 0;

	accel_data = (int16_t *)buf;

	mutex_lock(&hscdtd->accel_mutex);
	hscdtd->accel_data[0] = accel_data[0];
	hscdtd->accel_data[1] = accel_data[1];
	hscdtd->accel_data[2] = accel_data[2];
	mutex_unlock(&hscdtd->accel_mutex);

	dev_vdbg(&hscdtd->i2c->dev, "accel:%d,%d,%d\n",
			accel_data[0], accel_data[1], accel_data[2]);

	return size;
}


#if HSCDTD_DEBUG_IF
static ssize_t hscdtd_sysfs_mode_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	long mode = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, HSCDTD_BASE_NUM, &mode))
		return -EINVAL;

	if (HSCDTD_SetMode(hscdtd, (uint8_t)mode) < 0)
		return -EINVAL;

	return 1;
}

static ssize_t hscdtd_buf_print(
	char *buf, uint8_t *data, size_t num)
{
	int sz, i;
	char *cur;
	size_t cur_len;

	cur = buf;
	cur_len = PAGE_SIZE;
	sz = snprintf(cur, cur_len, "(HEX):");
	if (sz < 0)
		return sz;
	cur += sz;
	cur_len -= sz;
	for (i = 0; i < num; i++) {
		sz = snprintf(cur, cur_len, "%02X,", *data);
		if (sz < 0)
			return sz;
		cur += sz;
		cur_len -= sz;
		data++;
	}
	sz = snprintf(cur, cur_len, "\n");
	if (sz < 0)
		return sz;
	cur += sz;

	return (ssize_t)(cur - buf);
}

static ssize_t hscdtd_sysfs_bdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	uint8_t rbuf[HSCDTD_SENSOR_DATA_SIZE];

	mutex_lock(&hscdtd->sensor_mutex);
	memcpy(&rbuf, hscdtd->sense_data, sizeof(rbuf));
	mutex_unlock(&hscdtd->sensor_mutex);

	return hscdtd_buf_print(buf, rbuf, HSCDTD_SENSOR_DATA_SIZE);
}

static ssize_t hscdtd_sysfs_wia_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	int err;
	uint8_t asa[3];

	err = HSCDTD_SetMode(hscdtd, HSCDTD_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	asa[0] = HSCDTD_CTRL1;
	err = hscdtd_i2c_rxdata(hscdtd->i2c, asa, 1);
	if (err < 0)
		return err;

	err = HSCDTD_SetMode(hscdtd, HSCDTD_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return hscdtd_buf_print(buf, asa, 1);
}

static ssize_t hscdtd_sysfs_chipid_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* The total number of registers depends on the device. */
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	int err;
	uint8_t regs[HSCDTD_REGS_SIZE];

	/* This function does not lock mutex obj */
	regs[0] = HSCDTD_CHIP_ID;
	err = hscdtd_i2c_rxdata(hscdtd->i2c, regs, HSCDTD_REGS_SIZE);
	if (err < 0)
		return err;

	return hscdtd_buf_print(buf, regs, 2);
}
#endif

static struct device_attribute hscdtd_compass_attributes[] = {
	__ATTR(enable_acc, 0660, hscdtd_enable_acc_show, hscdtd_enable_acc_store),
	__ATTR(enable_mag, 0660, hscdtd_enable_mag_show, hscdtd_enable_mag_store),
	__ATTR(enable_fusion, 0660, hscdtd_enable_fusion_show,
			hscdtd_enable_fusion_store),
	__ATTR(delay_acc,  0660, hscdtd_delay_acc_show,  hscdtd_delay_acc_store),
	__ATTR(delay_mag,  0660, hscdtd_delay_mag_show,  hscdtd_delay_mag_store),
	__ATTR(delay_fusion, 0660, hscdtd_delay_fusion_show,
			hscdtd_delay_fusion_store),
#if HSCDTD_DEBUG_IF
	__ATTR(mode,  0220, NULL, hscdtd_sysfs_mode_store),
	__ATTR(bdata, 0440, hscdtd_sysfs_bdata_show, NULL),
	__ATTR(wia,   0440, hscdtd_sysfs_wia_show, NULL),
	__ATTR(chipid,  0440, hscdtd_sysfs_chipid_show, NULL),
#endif
	__ATTR_NULL,
};

#define __BIN_ATTR(name_, mode_, size_, private_, read_, write_) \
	{ \
		.attr    = { .name = __stringify(name_), .mode = mode_ }, \
		.size    = size_, \
		.private = private_, \
		.read    = read_, \
		.write   = write_, \
	}

#define __BIN_ATTR_NULL \
	{ \
		.attr   = { .name = NULL }, \
	}

static struct bin_attribute hscdtd_compass_bin_attributes[] = {
	__BIN_ATTR(accel, 0220, 6, NULL,
				NULL, hscdtd_bin_accel_write),
	__BIN_ATTR_NULL
};

static char const *const device_link_name = "i2c";
static dev_t const hscdtd_compass_device_dev_t = MKDEV(MISC_MAJOR, 240);

static int create_sysfs_interfaces(struct hscdtd_compass_data *hscdtd)
{
	int err;

	if (NULL == hscdtd)
		return -EINVAL;

	err = 0;

	hscdtd->compass = class_create(THIS_MODULE, HSCDTD_SYSCLS_NAME);
	if (IS_ERR(hscdtd->compass)) {
		err = PTR_ERR(hscdtd->compass);
		goto exit_class_create_failed;
	}

	hscdtd->class_dev = device_create(
			hscdtd->compass,
			NULL,
			hscdtd_compass_device_dev_t,
			hscdtd,
			HSCDTD_SYSDEV_NAME);
	if (IS_ERR(hscdtd->class_dev)) {
		err = PTR_ERR(hscdtd->class_dev);
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&hscdtd->class_dev->kobj,
			&hscdtd->i2c->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_device_attributes(
			hscdtd->class_dev,
			hscdtd_compass_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	err = create_device_binary_attributes(
			&hscdtd->class_dev->kobj,
			hscdtd_compass_bin_attributes);
	if (0 > err)
		goto exit_device_binary_attributes_create_failed;

	return err;

exit_device_binary_attributes_create_failed:
	remove_device_attributes(hscdtd->class_dev, hscdtd_compass_attributes);
exit_device_attributes_create_failed:
	sysfs_remove_link(&hscdtd->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(hscdtd->compass, hscdtd_compass_device_dev_t);
exit_class_device_create_failed:
	hscdtd->class_dev = NULL;
	class_destroy(hscdtd->compass);
exit_class_create_failed:
	hscdtd->compass = NULL;
	return err;
}

static void remove_sysfs_interfaces(struct hscdtd_compass_data *hscdtd)
{
	if (NULL == hscdtd)
		return;

	if (NULL != hscdtd->class_dev) {
		remove_device_binary_attributes(
			&hscdtd->class_dev->kobj,
			hscdtd_compass_bin_attributes);
		remove_device_attributes(
			hscdtd->class_dev,
			hscdtd_compass_attributes);
		sysfs_remove_link(
			&hscdtd->class_dev->kobj,
			device_link_name);
		hscdtd->class_dev = NULL;
	}
	if (NULL != hscdtd->compass) {
		device_destroy(
			hscdtd->compass,
			hscdtd_compass_device_dev_t);
		class_destroy(hscdtd->compass);
		hscdtd->compass = NULL;
	}
}


/***** hscdtd input device functions ***********************************/
static int hscdtd_compass_input_init(
	struct input_dev **input)
{
	int err = 0;

	/* Declare input device */
	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	/* Setup input device */
	set_bit(EV_ABS, (*input)->evbit);

	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(*input, ABS_X,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_Y,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_Z,
			-32768, 32767, 0, 0);
	/* Set name */
	(*input)->name = HSCDTD_INPUT_DEVICE_NAME;

	/* Register */
	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

/***** hscdtd functions ************************************************/
#if 0
static irqreturn_t hscdtd_compass_irq(int irq, void *handle)
{
	struct hscdtd_compass_data *hscdtd = handle;
	uint8_t buffer[HSCDTD_SENSOR_DATA_SIZE];
	int err;

	memset(buffer, 0, sizeof(buffer));

	/***** lock *****/
	mutex_lock(&hscdtd->sensor_mutex);

	/* Read whole data */
	buffer[0] = HSCDTD_REG_STATUS;
	err = hscdtd_i2c_rxdata(hscdtd->i2c, buffer, HSCDTD_SENSOR_DATA_SIZE);
	if (err < 0) {
		dev_err(&hscdtd->i2c->dev, "IRQ I2C error.");
		hscdtd->is_busy = 0;
		mutex_unlock(&hscdtd->sensor_mutex);
		/***** unlock *****/

		return IRQ_HANDLED;
	}
	/* Check ST bit */
	if (!(HSCDTD_DRDY_IS_HIGH(buffer[0])))
		goto work_func_none;

	memcpy(hscdtd->sense_data, buffer, HSCDTD_SENSOR_DATA_SIZE);
	hscdtd->is_busy = 0;

	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

	atomic_set(&hscdtd->drdy, 1);
	wake_up(&hscdtd->drdy_wq);

	dev_vdbg(&hscdtd->i2c->dev, "IRQ handled.");
	return IRQ_HANDLED;

work_func_none:
	mutex_unlock(&hscdtd->sensor_mutex);
	/***** unlock *****/

	dev_vdbg(&hscdtd->i2c->dev, "IRQ not handled.");
	return IRQ_NONE;
}
#endif
static int hscdtd_compass_suspend(struct device *dev)
{
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	/*int ret = 0;*/

	if (HSCDTD_IS_MAG_DATA_ENABLED() &&
		hscdtd->use_poll &&
		hscdtd->pdata->auto_report)
		cancel_delayed_work_sync(&hscdtd->dwork);

	hscdtd->state.power_on = hscdtd->power_enabled;
	if (hscdtd->state.power_on) {
		hscdtd_compass_power_set(hscdtd, false);
		/* Clear status */
		hscdtd->is_busy = 0;
		atomic_set(&hscdtd->drdy, 0);
	}

	/*ret = pinctrl_select_state(hscdtd->pinctrl, hscdtd->pin_sleep);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");*/

	dev_dbg(&hscdtd->i2c->dev, "suspended\n");

	return 0;
}

static int hscdtd_compass_resume(struct device *dev)
{
	struct hscdtd_compass_data *hscdtd = dev_get_drvdata(dev);
	int ret = 0;

	/*ret = pinctrl_select_state(hscdtd->pinctrl, hscdtd->pin_default);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");*/

	if (hscdtd->state.power_on) {
		ret = hscdtd_compass_power_set(hscdtd, true);
		if (ret) {
			dev_err(dev, "Sensor power resume fail!\n");
			goto exit;
		}

		ret = HSCDTD_SetMode(hscdtd, hscdtd->state.mode);
		if (ret) {
			dev_err(dev, "Sensor state resume fail!\n");
			goto exit;
		}

		if (HSCDTD_IS_MAG_DATA_ENABLED() &&
			hscdtd->use_poll &&
			hscdtd->pdata->auto_report)
			schedule_delayed_work(&hscdtd->dwork,
				(unsigned long)nsecs_to_jiffies64(
				hscdtd->delay[MAG_DATA_FLAG]));
	}
	dev_dbg(&hscdtd->i2c->dev, "resumed\n");

exit:
	return 0;

}

static int hscdtd007_i2c_check_device(
	struct i2c_client *client)
{
	/* HSCDTD specific function */
	struct hscdtd_compass_data *hscdtd = i2c_get_clientdata(client);
	int err;
	uint16_t chip_info;	
	hscdtd->sense_info[0] = HSCDTD_INFO;
	err = hscdtd_i2c_rxdata(client, hscdtd->sense_info, HSCDTD_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	chip_info = (uint16_t)((hscdtd->sense_info[1]<<8) | hscdtd->sense_info[0]);
	/* Check read data */
	if (chip_info != HSCDTD_CHIP_ID) {
		dev_err(&client->dev,
			"%s: The device %x is not HSCDTD Compass.", __func__, chip_info);
		return -ENXIO;
	}

	hscdtd->sense_info[0] = HSCDTD_CTRL3;
	hscdtd->sense_info[1] = 0x80;
	err = hscdtd_i2c_txdata(client, hscdtd->sense_info, HSCDTD_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	hscdtd->sense_info[0] = HSCDTD_CTRL1;
	hscdtd->sense_info[1] = 0x8A;
	err = hscdtd_i2c_txdata(client, hscdtd->sense_info, HSCDTD_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;
   {
	u8 buffer = HSCDTD_CTRL1;	
	err = hscdtd_i2c_rxdata(hscdtd->i2c, &buffer, 1);
    printk(KERN_ERR "%s %x %x\n", __func__, HSCDTD_CTRL1, buffer);

	}
	hscdtd->sense_info[0] = HSCDTD_CTRL4;
	hscdtd->sense_info[1] = 0x90;
	err = hscdtd_i2c_txdata(client, hscdtd->sense_info, HSCDTD_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	return err;
}

static int hscdtd_compass_power_set(struct hscdtd_compass_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		dev_warn(&data->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int hscdtd_compass_power_init(struct hscdtd_compass_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				HSCDTD007_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				HSCDTD007_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				HSCDTD007_VDD_MIN_UV, HSCDTD007_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				HSCDTD007_VIO_MIN_UV, HSCDTD007_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
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
		regulator_set_voltage(data->vdd, 0, HSCDTD007_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

#ifdef CONFIG_OF
static int hscdtd_compass_parse_dt(struct device *dev,
				struct hscdtd007_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "alps,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read hscdtd,layout\n");
		return rc;
	} else {
		pdata->layout = temp_val;
	}

	if (of_property_read_bool(np, "alps,auto-report")) {
		pdata->auto_report = 1;
		pdata->use_int = 0;
	} else {
		pdata->auto_report = 0;
		if (of_property_read_bool(dev->of_node, "alps,use-interrupt")) {
			pdata->use_int = 1;
			/* check gpio_int later, if it is invalid,
			 * just use poll */
			pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
					"alps,gpio-int", 0, &pdata->int_flags);
		} else {
			pdata->use_int = 0;
		}
	}

	pdata->gpio_rstn = of_get_named_gpio_flags(dev->of_node,
			"alps,gpio-rstn", 0, NULL);

	return 0;
}
#else
static int hscdtd_compass_parse_dt(struct device *dev,
				struct hscdtd007_platform_data *pdata)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */

/*
static int hscdtd007_pinctrl_init(struct hscdtd_compass_data *s_hscdtd)
{
	struct i2c_client *client = s_hscdtd->i2c;

	s_hscdtd->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(s_hscdtd->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(s_hscdtd->pinctrl);
	}

	s_hscdtd->pin_default = pinctrl_lookup_state(s_hscdtd->pinctrl,
			"hscdtd007_default");
	if (IS_ERR_OR_NULL(s_hscdtd->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(s_hscdtd->pin_default);
	}

	s_hscdtd->pin_sleep = pinctrl_lookup_state(s_hscdtd->pinctrl,
			"hscdtd007_sleep");
	if (IS_ERR_OR_NULL(s_hscdtd->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(s_hscdtd->pin_sleep);
	}

	return 0;
}
*/
static int hscdtd007_mag_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int hscdtd007_mag_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver hscdtd007_mag_pd_driver = {
	.probe  = hscdtd007_mag_pd_probe,
	.remove = hscdtd007_mag_pd_remove,    
	.driver = {
		.name  = "msensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device hscdtd007_mag_pd_device = {
    .name = "msensor",
    .id   = -1,
};

int hscd_self_test_A(void)
{
    u8 sx[2], cr1[1];

    cr1[0] = 0x1b/*HSCD_CTRL1*/;
    if (hscdtd_i2c_rxdata(hscdtd007_mag_i2c_client, cr1, 1)) return 1;
    mdelay(1);

    /* Move active mode (force state)  */
    sx[0] = 0x1b/*HSCD_CTRL1*/;
    sx[1] = 0x8A;
    if (hscdtd_i2c_txdata(hscdtd007_mag_i2c_client,sx, 2)) 
		return 1;

    /* Get inital value of self-test-A register  */
    sx[0] = 0x0c/*HSCD_STB*/;
    hscdtd_i2c_rxdata(hscdtd007_mag_i2c_client,sx, 1);
    mdelay(1);
    sx[0] = 0x0c/*HSCD_STB*/;
    if (hscdtd_i2c_rxdata(hscdtd007_mag_i2c_client, sx, 1)) return 1;
    if (sx[0] != 0x55) {
        printk("error: self-test-A, initial value is %02X\n", sx[0]);
        return 2;
    }

    /* do self-test*/
    sx[0] = 0x1d/*HSCD_CTRL3*/;
    sx[1] = 0x10;
    if (hscdtd_i2c_txdata(hscdtd007_mag_i2c_client, sx, 2)) return 1;
    mdelay(3);

    /* Get 1st value of self-test-A register  */
    sx[0] = 0x0c/*HSCD_STB*/;
    if (hscdtd_i2c_rxdata(hscdtd007_mag_i2c_client, sx, 1)) 
		return 1;
	if (sx[0] != 0xAA) {
        printk("error: self-test, 1st value is %02X\n", sx[0]);
        return 3;
    }
    mdelay(3);

    /* Get 2nd value of self-test register  */
    sx[0] = 0x0c/*HSCD_STB*/;
    if (hscdtd_i2c_rxdata(hscdtd007_mag_i2c_client, sx, 1)) return 1;
    if (sx[0] != 0x55) {
        printk("error: self-test, 2nd value is %02X\n", sx[0]);
        return 4;
    }

    /* Resume */
    sx[0] = 0x1b/*HSCD_CTRL1*/;
    sx[1] = cr1[0];
    if (hscdtd_i2c_txdata(hscdtd007_mag_i2c_client, sx, 2)) return 1;

    return 0x0f;
}

static ssize_t hscdtd007_mag_shipment_test(struct device_driver *ddri, char *buf)
{
	int value;
	value = hscd_self_test_A();
	return sprintf(buf, "%d\n", value);

}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	/*char sensordata[SENSOR_DATA_SIZE];*/
	struct hscdtd_compass_data *hscdtd=NULL;
	int ret=0;
	uint8_t dat_buf[HSCDTD_SENSOR_DATA_SIZE];
	int mag_x, mag_y, mag_z;
	int data_tmp[HSCDTD_3AXIS_NUM];
	int i;
	u8 buffer[2];
	char strbuf[32];


	if(hscdtd007_mag_i2c_client)hscdtd=i2c_get_clientdata(hscdtd007_mag_i2c_client);

	buffer[0] = HSCDTD_CTRL1;
	buffer[1] = 0x8a;
	printk(KERN_ERR "%s %x %x\n", __func__, buffer[0], buffer[1]);
	ret = hscdtd_i2c_txdata(hscdtd->i2c, buffer, 2);
	if (ret < 0) {
		dev_err(&hscdtd->i2c->dev,
				"%s: Can not set CNTL.", __func__);
	}


	ret = HSCDTD_GetData_Poll(hscdtd, dat_buf, HSCDTD_DATA_ACCESS_NUM);
	if (ret < 0) {
		dev_warn(&s_hscdtd->i2c->dev, "Get data failed\n");
	}
	for (i = 0; i < HSCDTD_3AXIS_NUM; i++)		
		data_tmp[i] = (int) ((short)((dat_buf[2*i+1] << 8) | (dat_buf[2*i])));

	mag_x = data_tmp[0];
	mag_y = data_tmp[1];
	mag_z = data_tmp[2];

	/*sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", sensordata[0],sensordata[1],sensordata[2],
		sensordata[3],sensordata[4],sensordata[5],sensordata[6],sensordata[7],sensordata[8]);*/

	sprintf(strbuf, "%d %d %d\n", mag_x,mag_y,mag_z);

	return sprintf(buf, "%s\n", strbuf);
}

static DRIVER_ATTR(selftest,S_IRUGO | S_IWUSR, hscdtd007_mag_shipment_test, NULL);
static DRIVER_ATTR(data,  S_IRUGO, show_sensordata_value, NULL);

static struct driver_attribute *hscdtd007_mag_attr_list[] = {
	&driver_attr_selftest,
	&driver_attr_data,
};

static int hscdtd007_mag_create_attr(struct device_driver *driver) 
{
	int i;
	for (i = 0; i < ARRAY_SIZE(hscdtd007_mag_attr_list); i++)
		if (driver_create_file(driver, hscdtd007_mag_attr_list[i]))
			goto error;
	return 0;

error:
	for (i--; i >= 0; i--)
		driver_remove_file(driver, hscdtd007_mag_attr_list[i]);
	printk(KERN_ERR "%s:Unable to create interface\n", __func__);
	return -1;
}

static void hscdtd_dev_poll(struct work_struct *work)
{
	struct hscdtd_compass_data *hscdtd;
	uint8_t dat_buf[HSCDTD_SENSOR_DATA_SIZE];/* for GET_DATA */
	int ret, i;
	int mag_x, mag_y, mag_z;
	int data_tmp[HSCDTD_3AXIS_NUM];
	int tmp;

    printk(KERN_ERR "%s\n", __func__);
	hscdtd = container_of((struct delayed_work *)work,
			struct hscdtd_compass_data,  dwork);
	ret = HSCDTD_GetData_Poll(hscdtd, dat_buf, HSCDTD_DATA_ACCESS_NUM);
	if (ret < 0) {
		dev_warn(&s_hscdtd->i2c->dev, "Get data failed\n");
		goto exit;
	}
/*
	tmp = 0xFF & (dat_buf[7] + dat_buf[0]);
	if (STATUS_ERROR(tmp)) {
		dev_warn(&hscdtd->i2c->dev, "Status error(0x%x). Reset...\n",
				tmp);
		HSCDTD_Reset(hscdtd, 0);
		goto exit;
	}

*/
	for (i = 0; i < HSCDTD_3AXIS_NUM; i++)		
		data_tmp[i] = (int) ((short)((dat_buf[2*i+1] << 8) | (dat_buf[2*i])));

	mag_x = data_tmp[0];
	mag_y = data_tmp[1];
	mag_z = data_tmp[2];

	switch (hscdtd->pdata->layout) {
	case 0:
	case 1:
		/* Fall into the default direction */
		break;
	case 2:
		tmp = mag_x;
		mag_x = mag_y;
		mag_y = -tmp;
		break;
	case 3:
		mag_x = -mag_x;
		mag_y = -mag_y;
		break;
	case 4:
		tmp = mag_x;
		mag_x = -mag_y;
		mag_y = tmp;
		break;
	case 5:
		mag_x = -mag_x;
		mag_z = -mag_z;
		break;
	case 6:
		tmp = mag_x;
		mag_x = mag_y;
		mag_y = tmp;
		mag_z = -mag_z;
		break;
	case 7:
		mag_y = -mag_y;
		mag_z = -mag_z;
		break;
	case 8:
		tmp = mag_x;
		mag_x = -mag_y;
		mag_y = -tmp;
		mag_z = -mag_z;
		break;
	}

	input_report_abs(hscdtd->input, ABS_X, mag_x);
	input_report_abs(hscdtd->input, ABS_Y, mag_y);
	input_report_abs(hscdtd->input, ABS_Z, mag_z);
	input_sync(hscdtd->input);
    printk(KERN_ERR "%s report mag_x %d mag_y %d mag_z %d\n", __func__, mag_x, mag_y, mag_z);
	dev_vdbg(&s_hscdtd->i2c->dev,
			"input report: mag_x=%02x, mag_y=%02x, mag_z=%02x",
			mag_x, mag_y, mag_z);

exit:
	ret = HSCDTD_SetMode(hscdtd, HSCDTD_MODE_SNG_MEASURE | 0x10);
	if (ret < 0)
		dev_warn(&hscdtd->i2c->dev, "Failed to set mode\n");
    printk(KERN_ERR "%s delay %d\n", __func__, hscdtd->delay[MAG_DATA_FLAG]);
	if (hscdtd->use_poll)
		schedule_delayed_work(&hscdtd->dwork,
				msecs_to_jiffies(hscdtd->delay[MAG_DATA_FLAG]));
}

int hscdtd007_compass_probe(
		struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct hscdtd007_platform_data *pdata;
	int err = 0;
	int i;

	dev_dbg(&i2c->dev, "start probing.");
	printk(KERN_ERR "%s\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto err_i2c_check;
	}

	/* Allocate memory for driver data */
	s_hscdtd = devm_kzalloc(&i2c->dev, sizeof(struct hscdtd_compass_data),
			GFP_KERNEL);
	if (!s_hscdtd) {
		dev_err(&i2c->dev, "Failed to allocate driver data\n");
		return -ENOMEM;
	}

	/***** I2C initialization *****/
	s_hscdtd->i2c = i2c;
	/* set i2c data */
	i2c_set_clientdata(i2c, s_hscdtd);

	/**** initialize variables in hscdtd_compass_data *****/
	init_waitqueue_head(&s_hscdtd->drdy_wq);
	init_waitqueue_head(&s_hscdtd->open_wq);

	mutex_init(&s_hscdtd->sensor_mutex);
	mutex_init(&s_hscdtd->accel_mutex);
	mutex_init(&s_hscdtd->val_mutex);
	mutex_init(&s_hscdtd->op_mutex);

	atomic_set(&s_hscdtd->active, 0);
	atomic_set(&s_hscdtd->drdy, 0);

	s_hscdtd->is_busy = 0;
	s_hscdtd->enable_flag = 0;

	/* Set to 1G in Android coordination, HSCDTD format */
	s_hscdtd->accel_data[0] = 0;
	s_hscdtd->accel_data[1] = 0;
	s_hscdtd->accel_data[2] = 720;

	for (i = 0; i < HSCDTD_NUM_SENSORS; i++)
		s_hscdtd->delay[i] = 0;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(
				&i2c->dev,
				sizeof(struct hscdtd007_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allcated memory\n");
			err = -ENOMEM;
			goto err_devm;
		}

		err = hscdtd_compass_parse_dt(&i2c->dev, pdata);
		if (err) {
			dev_err(
			&i2c->dev,
			"Unable to parse platfrom data err=%d\n",
			err);
			goto err_devm;
		}
	} else {
		if (i2c->dev.platform_data) {
			/* Copy platform data to local. */
			pdata = i2c->dev.platform_data;
		} else {
			/* Platform data is not available.
			   Layout and information should be
			   set by each application. */
			s_hscdtd->pdata->layout = 0;
			s_hscdtd->gpio_rstn = 0;
			dev_warn(&i2c->dev, "%s: No platform data.",
					__func__);
		}
	}

	s_hscdtd->pdata = pdata;

	/* check connection */
	err = hscdtd_compass_power_init(s_hscdtd, true);
	if (err < 0)
		goto err_devm;

	err = hscdtd_compass_power_set(s_hscdtd, true);
	if (err < 0)
		goto err_compass_pwr_init;

	/* Pull up the reset pin */
	HSCDTD_Reset(s_hscdtd, 1);

	err = hscdtd007_i2c_check_device(i2c);
	if (err < 0)
		goto err_compass_pwr_off;

	/***** input *****/
	err = hscdtd_compass_input_init(&s_hscdtd->input);
	if (err) {
		dev_err(&i2c->dev,
				"%s: input_dev register failed", __func__);
		goto err_compass_pwr_off;
	}
	input_set_drvdata(s_hscdtd->input, s_hscdtd);

	/* initialize pinctrl */
	/*if (!hscdtd007_pinctrl_init(s_hscdtd)) {
		err = pinctrl_select_state(s_hscdtd->pinctrl, s_hscdtd->pin_default);
		if (err) {
			dev_err(&i2c->dev, "Can't select pinctrl state\n");
			goto err_compass_pwr_off;
		}
	}*/
#if 0
	if ((s_hscdtd->pdata->use_int) &&
		gpio_is_valid(s_hscdtd->pdata->gpio_int)) {
		s_hscdtd->use_poll = false;

		/* configure interrupt gpio */
		err = gpio_request(s_hscdtd->pdata->gpio_int,
				"hscdtd007_gpio_int");
		if (err) {
			dev_err(
			&i2c->dev,
			"Unable to request interrupt gpio %d\n",
			s_hscdtd->pdata->gpio_int);
			goto err_unregister_device;
		}

		err = gpio_direction_input(s_hscdtd->pdata->gpio_int);
		if (err) {
			dev_err(
			&i2c->dev,
			"Unable to set direction for gpio %d\n",
			s_hscdtd->pdata->gpio_int);
			goto err_gpio_free;
		}
		i2c->irq = gpio_to_irq(s_hscdtd->pdata->gpio_int);

		/***** IRQ setup *****/
		s_hscdtd->i2c->irq = i2c->irq;

		dev_dbg(&i2c->dev, "%s: IRQ is #%d.",
				__func__, s_hscdtd->i2c->irq);

		err = request_threaded_irq(
				s_hscdtd->i2c->irq,
				NULL,
				hscdtd_compass_irq,
				IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
				dev_name(&i2c->dev),
				s_hscdtd);
		if (err) {
			dev_err(&i2c->dev,
					"%s: request irq failed.", __func__);
			goto err_gpio_free;
		}
	} else if (s_hscdtd->pdata->auto_report) 
#endif

	{
		s_hscdtd->use_poll = true;
		INIT_DELAYED_WORK(&s_hscdtd->dwork, hscdtd_dev_poll);
	}

	/***** sysfs *****/
	err = create_sysfs_interfaces(s_hscdtd);
	if (0 > err) {
		dev_err(&i2c->dev,
				"%s: create sysfs failed.", __func__);
		goto err_unregister_device;
	}

	s_hscdtd->cdev = sensors_cdev;
	s_hscdtd->cdev.sensors_enable = hscdtd_enable_set;
	s_hscdtd->cdev.sensors_poll_delay = hscdtd_poll_delay_set;

	s_hscdtd->delay[MAG_DATA_FLAG] = sensors_cdev.delay_msec;

	err = sensors_classdev_register(&i2c->dev, &s_hscdtd->cdev);
	if (err) {
		dev_err(&i2c->dev, "class device create failed: %d\n", err);
		goto remove_sysfs;
	}

	err = hscdtd_compass_power_set(s_hscdtd, false);
	if (err)
		dev_err(&i2c->dev,
			"Fail to disable power after probe: %d\n", err);
	
    if((err = platform_driver_register(&hscdtd007_mag_pd_driver)))
	{
		printk(KERN_ERR "%s failed to register hscdtd007_mag_driver err %d\n", __func__, err);
		goto err_unreg_sensor_class/*err_free_irq2*/;
	}

    if((err = platform_device_register(&hscdtd007_mag_pd_device)))
    {
		printk(KERN_ERR "%s failed to register hscdtd007_mag_device err %d\n", __func__, err);
		goto err_free_driver;
    }

	if((err = hscdtd007_mag_create_attr(&hscdtd007_mag_pd_driver.driver)))
	{
		printk("%s lis3dh create attribute err = %d\n", __func__, err);
		goto err_free_device;
	}
    hscdtd007_mag_i2c_client=i2c;

	dev_info(&i2c->dev, "successfully probed.");
	return 0;
err_free_device:
	platform_device_unregister(&hscdtd007_mag_pd_device);
err_free_driver:
	platform_driver_unregister(&hscdtd007_mag_pd_driver);
err_unreg_sensor_class:
	sensors_classdev_unregister(&s_hscdtd->cdev);
remove_sysfs:
	remove_sysfs_interfaces(s_hscdtd);
/*err_free_irq:
	if (s_hscdtd->i2c->irq)
		free_irq(s_hscdtd->i2c->irq, s_hscdtd);*/
err_unregister_device:
	input_unregister_device(s_hscdtd->input);
/*err_gpio_free:
	if ((s_hscdtd->pdata->use_int) &&
		(gpio_is_valid(s_hscdtd->pdata->gpio_int)))
		gpio_free(s_hscdtd->pdata->gpio_int);*/
err_compass_pwr_off:
	hscdtd_compass_power_set(s_hscdtd, false);
err_compass_pwr_init:
	hscdtd_compass_power_init(s_hscdtd, false);
err_devm:
	devm_kfree(&i2c->dev, s_hscdtd);
err_i2c_check:
	return err;
}

static int hscdtd007_compass_remove(struct i2c_client *i2c)
{

	struct hscdtd_compass_data *hscdtd = i2c_get_clientdata(i2c);

	if (hscdtd_compass_power_set(hscdtd, false))
		dev_err(&i2c->dev, "power off failed.\n");
	if (hscdtd_compass_power_init(hscdtd, false))
		dev_err(&i2c->dev, "power deinit failed.\n");
	remove_sysfs_interfaces(hscdtd);
	if (hscdtd->i2c->irq)
		free_irq(hscdtd->i2c->irq, hscdtd);
	input_unregister_device(hscdtd->input);
	devm_kfree(&i2c->dev, hscdtd);
	dev_info(&i2c->dev, "successfully removed.");
	
	return 0;
}


static const struct i2c_device_id hscdtd007_compass_id[] = {
	{HSCDTD_I2C_NAME, 0 },
	{ }
};

static const struct dev_pm_ops hscdtd_compass_pm_ops = {
	.suspend	= hscdtd_compass_suspend,
	.resume		= hscdtd_compass_resume,
};

static struct of_device_id hscdtd007_match_table[] = {
	{ .compatible = "alps,hscd007", },
	{ .compatible = "alps,hscdtd007", },
	{ },
};

static struct i2c_driver hscdtd_compass_driver = {
	.probe		= hscdtd007_compass_probe,
	.remove		= hscdtd007_compass_remove,
	.id_table	= hscdtd007_compass_id,
	.driver = {
		.name	= HSCDTD_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = hscdtd007_match_table,
		.pm		= &hscdtd_compass_pm_ops,
	},
};

static int __init hscdtd_compass_init(void)
{
	pr_info("HSCDTD compass driver: initialize.");
	return i2c_add_driver(&hscdtd_compass_driver);
}

static void __exit hscdtd_compass_exit(void)
{
	pr_info("HSCDTD compass driver: release.");
	i2c_del_driver(&hscdtd_compass_driver);
}

module_init(hscdtd_compass_init);
module_exit(hscdtd_compass_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("HSCDTD compass driver");
MODULE_LICENSE("GPL");

