/* drivers/input/misc/akm8963.c - akm8963 compass driver
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
#include "akm8963.h"
#include <linux/delay.h>
#include <linux/platform_device.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#endif
#include <linux/jiffies.h>

#define AKM_DEBUG_IF			0
#define AKM_HAS_RESET			1
#define AKM_INPUT_DEVICE_NAME	"compass"
#define AKM_DRDY_TIMEOUT_MS		100
#define AKM_BASE_NUM			10

#define AKM_IS_MAG_DATA_ENABLED() (akm->enable_flag & (1 << MAG_DATA_FLAG))

/* POWER SUPPLY VOLTAGE RANGE */
#define AKM8963_VDD_MIN_UV	2850000
#define AKM8963_VDD_MAX_UV	2850000
#define AKM8963_VIO_MIN_UV	1800000
#define AKM8963_VIO_MAX_UV	1800000
#define STATUS_ERROR(st)	(((st) & (AKM8963_ST1_DRDY | \
				AKM8963_ST1_DOR  | \
				AKM8963_ST2_HOLF)) \
				!= AKM8963_ST1_DRDY)
#define REG_CNTL1_MODE(reg_cntl1)	(reg_cntl1 & 0x0F)

#define TAG  "Msensor "       //"Gsensor" "PAsensor" "GYsensor"
#define TAGI "Msensor.I "     //KERN_INFO 
#define TAGE "Msensor.E "     //KERN_ERR 
extern void print_vivo_init(const char* fmt, ...);
extern void print_vivo_main(const char* fmt, ...);

#define CONT_INT_TIME     50    //50ms 
#define POLL_DELAY_TIMES  2     //2X

#define CONT_INT_GATE   20 
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0

#define DEBUG 0

/* Save last device state for power down */
struct akm_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct akm_compass_data {
	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct device		*class_dev;
	struct class		*compass;
/* 
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
*/
	struct sensors_classdev	cdev;
	struct delayed_work	dwork;
	/*add qcom patch start*/
	struct workqueue_struct	*work_queue;
	/*add qcom patch end*/
	struct mutex		op_mutex;


	wait_queue_head_t	drdy_wq;
	wait_queue_head_t	open_wq;

	/* These two buffers are initialized at start up.
	   After that, the value is not changed */
	uint8_t sense_info[AKM_SENSOR_INFO_SIZE];
	uint8_t sense_conf[AKM_SENSOR_CONF_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[AKM_SENSOR_DATA_SIZE];
	struct mutex accel_mutex;
	int16_t accel_data[3];
	struct	mutex sensor_data_mutex;
	int sense_rawdata[3];
	struct delayed_work	datawork;
	struct workqueue_struct *data_wq;
	uint8_t data_valid;
	uint8_t work_state;

	/* Positive value means the device is working.
	   0 or negative value means the device is not woking,
	   i.e. in power-down mode. */
	int8_t	is_busy;

	struct mutex	val_mutex;
	uint32_t	enable_flag;
	int32_t		delay[AKM_NUM_SENSORS];

	atomic_t	active;
	atomic_t	drdy;

	int	gpio_rstn;
	bool	power_enabled;
	bool	use_poll;
	struct	regulator		*vdd;
	struct	regulator		*vio;
	struct	akm_sensor_state		state;
	struct	akm8963_platform_data	*pdata;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
};

static struct sensors_classdev sensors_cdev = {
	.name = "akm8963-mag",
	.vendor = "Asahi Kasei Microdevices Corporation",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1228.8",
	.resolution = "0.15",
	.sensor_power = "0.35",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct akm_compass_data *s_akm;
struct i2c_client *akm8963_mag_i2c_client=NULL;
static int poll_delay_num = 0;

static void akm_dev_poll(struct work_struct *work);
static void akm_pull_data(struct work_struct *work);


static int akm_compass_power_set(struct akm_compass_data *data, bool on);

/***** I2C I/O function ***********************************************/
static int akm_i2c_rxdata(
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
		printk(KERN_ERR TAGE "%s: transfer failed.\n", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		printk(KERN_ERR TAGE "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}
    if(DEBUG)
		printk(KERN_INFO TAGI "RxData: len=%02x, addr=%02x, data=%02x\n",
				length, addr, rxData[0]);

	return 0;
}

static int akm_i2c_txdata(
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

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s: transfer failed.\n", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		printk(KERN_ERR TAGE "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	if(DEBUG)
		printk(KERN_INFO TAGI "TxData: len=%02x, addr=%02x data=%02x\n",
			length, txData[0], txData[1]);

	return 0;
}
#define AKI2C_RxData(a,b) akm_i2c_rxdata(akm8963_mag_i2c_client,a,b)
#define AKI2C_TxData(a,b) akm_i2c_txdata(akm8963_mag_i2c_client,a,b)

/***** akm miscdevice functions *************************************/
static int AKECS_Set_CNTL(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);
	/* Busy check */
	if (akm->is_busy > 0) {
		printk(KERN_INFO TAGI
				"%s: device is busy.\n", __func__);
		err = -EBUSY;
	} else {
		/* Set measure mode */
		buffer[0] = AKM_REG_MODE;
		buffer[1] = mode;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			printk(KERN_ERR TAGE
					"%s: Can not set CNTL.\n", __func__);
		} else {
			if(DEBUG)
				printk(KERN_INFO TAGI
					"Mode is set to (%d).\n", mode);
			/* Set flag */
			akm->is_busy = 1;
			atomic_set(&akm->drdy, 0);
			/* wait at least 100us after changing mode */
			udelay(100);
		}
	}

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int AKECS_Set_PowerDown(
	struct akm_compass_data *akm)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	/* Set powerdown mode */
	buffer[0] = AKM_REG_MODE;
	buffer[1] = AKM_MODE_POWERDOWN;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		printk(KERN_ERR TAGE
			"%s: Can not set to powerdown mode.\n", __func__);
	} else {
		printk(KERN_INFO TAGI "Powerdown mode is set.\n");
		/* wait at least 100us after changing mode */
		udelay(100);
	}
	/* Clear status */
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int AKECS_Reset(
	struct akm_compass_data *akm,
	int hard)
{
	int err;

#if AKM_HAS_RESET
	uint8_t buffer[2];

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	if (hard != 0) {
		udelay(5);
		/* No error is returned */
		err = 0;
	} else {
		buffer[0] = AKM_REG_RESET;
		buffer[1] = AKM_RESET_DATA;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			printk(KERN_ERR TAGE
				"%s: Can not set SRST bit.\n", __func__);
		} else {
			printk(KERN_INFO TAGI "Soft reset is done.\n");
		}
	}
	/* Device will be accessible 100 us after */
	udelay(100);
	/* Clear status */
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

#else
	err = AKECS_Set_PowerDown(akm);
#endif

	return err;
}

static int AKECS_SetMode(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	int err;

	switch (REG_CNTL1_MODE(mode)) {
	case AKM_MODE_SNG_MEASURE:
	case AKM_MODE_SELF_TEST:
	case AK8963_MODE_CONT1_MEASURE:
	case AK8963_MODE_CONT2_MEASURE:
	case AK8963_MODE_EXT_TRIG_MEASURE:
	case AKM_MODE_FUSE_ACCESS:
		err = AKECS_Set_CNTL(akm, mode);
		break;
	case AKM_MODE_POWERDOWN:
		err = AKECS_Set_PowerDown(akm);
		break;
	default:
		printk(KERN_ERR TAGE
				"%s: Unknown mode(%d).\n", __func__, mode);
		return -EINVAL;
	}
	akm->state.mode = mode;
	return err;
}

static void AKECS_SetYPR(
	struct akm_compass_data *akm,
	int *rbuf)
{
	uint32_t ready;
	if(DEBUG)
	{
		printk(KERN_INFO TAGI "%s: flag =0x%X\n", __func__, rbuf[0]);
		printk(KERN_INFO TAGI "  Acc [LSB]   : %6d,%6d,%6d stat=%d\n",
			rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
		printk(KERN_INFO TAGI "  Geo [LSB]   : %6d,%6d,%6d stat=%d\n",
			rbuf[5], rbuf[6], rbuf[7], rbuf[8]);
		printk(KERN_INFO TAGI "  Orientation : %6d,%6d,%6d\n",
			rbuf[9], rbuf[10], rbuf[11]);
		printk(KERN_INFO TAGI "  Rotation V  : %6d,%6d,%6d,%6d\n",
			rbuf[12], rbuf[13], rbuf[14], rbuf[15]);
	}
	/* No events are reported */
	if (!rbuf[0]) {
		printk(KERN_ERR TAGE "Don't waste a time.\n");
		return;
	}

	mutex_lock(&akm->val_mutex);
	ready = (akm->enable_flag & (uint32_t)rbuf[0]);
	mutex_unlock(&akm->val_mutex);

	/* Report acceleration sensor information */
	if (ready & ACC_DATA_READY) {
		input_report_abs(akm->input, ABS_X, rbuf[1]);
		input_report_abs(akm->input, ABS_Y, rbuf[2]);
		input_report_abs(akm->input, ABS_Z, rbuf[3]);
		input_report_abs(akm->input, ABS_RX, rbuf[4]);
	}
	/* Report magnetic vector information */
	if (ready & MAG_DATA_READY) {
		input_report_abs(akm->input, ABS_X, rbuf[5]);
		input_report_abs(akm->input, ABS_Y, rbuf[6]);
		input_report_abs(akm->input, ABS_Z, rbuf[7]);
	}
	/* Report fusion sensor information */
	if (ready & FUSION_DATA_READY) {
		/* Orientation */
		input_report_abs(akm->input, ABS_HAT0Y, rbuf[9]);
		input_report_abs(akm->input, ABS_HAT1X, rbuf[10]);
		input_report_abs(akm->input, ABS_HAT1Y, rbuf[11]);
		/* Rotation Vector */
		input_report_abs(akm->input, ABS_TILT_X, rbuf[12]);
		input_report_abs(akm->input, ABS_TILT_Y, rbuf[13]);
		input_report_abs(akm->input, ABS_TOOL_WIDTH, rbuf[14]);
		input_report_abs(akm->input, ABS_VOLUME, rbuf[15]);
	}

	input_sync(akm->input);
}

/* This function will block a process until the latest measurement
 * data is available.
 */
static int AKECS_GetData(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	int err;

	/* Block! */
	err = wait_event_interruptible_timeout(
			akm->drdy_wq,
			atomic_read(&akm->drdy),
			msecs_to_jiffies(AKM_DRDY_TIMEOUT_MS));

	if (err < 0) {
		printk(KERN_ERR TAGE
			"%s: wait_event failed (%d).\n", __func__, err);
		return err;
	}
	if (!atomic_read(&akm->drdy)) {
		printk(KERN_ERR TAGE
			"%s: DRDY is not set.\n", __func__);
		return -ENODATA;
	}

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	memcpy(rbuf, akm->sense_data, size);
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return 0;
}

static int AKECS_GetData_custom(struct akm_compass_data *akm, char *rbuf, int size)
{
	char temp;
	int loop_i,ret;

	if(size < SENSOR_DATA_SIZE)
	{
		printk(KERN_ERR TAGE "%s buff size is too small %d!\n", __func__, size);
		return -1;
	}
	
	memset(rbuf, 0, SENSOR_DATA_SIZE);
	rbuf[0] = AK8963_REG_ST1;

	for(loop_i = 0; loop_i < AKM09911_RETRY_COUNT; loop_i++)
	{
		if((ret = AKI2C_RxData(rbuf, 1)))
		{
			printk(KERN_ERR TAGE "%s read ST1 resigster failed!\n",__func__);
			return -1;
		}
		
		if((rbuf[0] & 0x01) == 0x01)
		{
			break;
		}
		msleep(2);
		rbuf[0] = AK8963_REG_ST1;
	}

	if(loop_i >= AKM09911_RETRY_COUNT)
	{
		printk(KERN_ERR TAGE "%s Data read retry larger the max count!\n",__func__);
		return -1;
	}

	temp = rbuf[0];
	rbuf[1]= AK8963_REG_HXL;
	ret = AKI2C_RxData(&rbuf[1], SENSOR_DATA_SIZE -2);
	if(ret < 0)
	{
		printk(KERN_ERR TAGE "%s AKM8975 akm8975_work_func: I2C failed\n",__func__);
		return -1;
	}
	rbuf[0] = temp;
	rbuf[8]=rbuf[7];
	rbuf[7]=0;

	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	return 0;
}

static int AKECS_GetData_Poll(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	/* Read status */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, 1);
	if (err < 0) {
		printk(KERN_ERR TAGE "%s failed.\n", __func__);
		return err;
	}

	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		return -EAGAIN;

	/* Read rest data */
	buffer[1] = AKM_REG_STATUS + 1;
	err = akm_i2c_rxdata(akm->i2c, &(buffer[1]), AKM_SENSOR_DATA_SIZE-1);
	if (err < 0) {
		printk(KERN_ERR TAGE "%s failed.\n", __func__);
		return err;
	}

	memcpy(rbuf, buffer, size);
	atomic_set(&akm->drdy, 0);

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);
	akm->is_busy = 0;
	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return 0;
}

static int AKECS_GetOpenStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) > 0));
}

static int AKECS_GetCloseStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) <= 0));
}

static int AKECS_Open(struct inode *inode, struct file *file)
{
	file->private_data = s_akm;
	return nonseekable_open(inode, file);
}

static int AKECS_Release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
AKECS_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct akm_compass_data *akm = file->private_data;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	uint8_t i2c_buf[AKM_RWBUF_SIZE];		/* for READ/WRITE */
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int32_t ypr_buf[AKM_YPR_DATA_SIZE];		/* for SET_YPR */
	int32_t delay[AKM_NUM_SENSORS];	/* for GET_DELAY */
	int16_t acc_buf[3];	/* for GET_ACCEL */
	uint8_t mode;			/* for SET_MODE*/
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = 0;		/* Return value. */

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (argp == NULL) {
			printk(KERN_ERR TAGE "invalid argument.\n");
			return -EINVAL;
		}
		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf))) {
			printk(KERN_ERR TAGE "copy_from_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			printk(KERN_ERR TAGE "invalid argument.\n");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			printk(KERN_ERR TAGE "copy_from_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			printk(KERN_ERR TAGE "invalid argument.\n");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			printk(KERN_ERR TAGE "copy_from_user failed.\n");
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
			printk(KERN_ERR TAGE "invalid argument.\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if(DEBUG)
			printk(KERN_ERR TAGE "IOCTL_READ called.\n");
		if ((i2c_buf[0] < 1) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			printk(KERN_ERR TAGE "invalid argument.\n");
			return -EINVAL;
		}
		ret = akm_i2c_rxdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		if(DEBUG)
			printk(KERN_ERR TAGE "IOCTL_WRITE called.\n");
		if ((i2c_buf[0] < 2) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			printk(KERN_ERR TAGE "invalid argument.\n");
			return -EINVAL;
		}
		ret = akm_i2c_txdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_RESET called.\n");
		ret = AKECS_Reset(akm, akm->gpio_rstn);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_SET_MODE called.\n");
		ret = AKECS_SetMode(akm, mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_SET_YPR called.\n");
		AKECS_SetYPR(akm, ypr_buf);
		break;
	case ECS_IOCTL_GET_DATA:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_DATA called.\n");
		if (akm->i2c->irq)
			ret = AKECS_GetData(akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		else
			ret = AKECS_GetData_Poll(
					akm, dat_buf, AKM_SENSOR_DATA_SIZE);

		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_OPEN_STATUS called.\n");
		ret = AKECS_GetOpenStatus(akm);
		if (ret < 0) {
			printk(KERN_ERR TAGE
				"Get Open returns error (%d).\n", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_CLOSE_STATUS called.\n");
		ret = AKECS_GetCloseStatus(akm);
		if (ret < 0) {
			printk(KERN_ERR TAGE
				"Get Close returns error (%d).\n", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_DELAY called.\n");
		mutex_lock(&akm->val_mutex);
		delay[0] = ((akm->enable_flag & ACC_DATA_READY) ?
				akm->delay[0] : -1);
		delay[1] = ((akm->enable_flag & MAG_DATA_READY) ?
				akm->delay[1] : -1);
		delay[2] = ((akm->enable_flag & FUSION_DATA_READY) ?
				akm->delay[2] : -1);
		mutex_unlock(&akm->val_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_INFO called.\n");
		break;
	case ECS_IOCTL_GET_CONF:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_CONF called.\n");
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_LAYOUT called.\n");
		break;
	case ECS_IOCTL_GET_ACCEL:
		if(DEBUG)
			printk(KERN_INFO TAGI "IOCTL_GET_ACCEL called.\n");
		mutex_lock(&akm->accel_mutex);
		acc_buf[0] = akm->accel_data[0];
		acc_buf[1] = akm->accel_data[1];
		acc_buf[2] = akm->accel_data[2];
		mutex_unlock(&akm->accel_mutex);
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		/* +1  is for the first byte */
		if (copy_to_user(argp, &i2c_buf, i2c_buf[0]+1)) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		if (copy_to_user(argp, &akm->sense_info,
					sizeof(akm->sense_info))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		if (copy_to_user(argp, &akm->sense_conf,
					sizeof(akm->sense_conf))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &dat_buf, sizeof(dat_buf))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = atomic_read(&akm->active);
		if (copy_to_user(argp, &status, sizeof(status))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &akm->pdata->layout,
					sizeof(akm->pdata->layout))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			printk(KERN_ERR TAGE "copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations AKECS_fops = {
	.owner = THIS_MODULE,
	.open = AKECS_Open,
	.release = AKECS_Release,
	.unlocked_ioctl = AKECS_ioctl,
};

/***** akm sysfs functions ******************************************/
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
 * directory : /sys/class/compass/akmXXXX/
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
static void akm_compass_sysfs_update_status(
	struct akm_compass_data *akm)
{
	uint32_t en;
	mutex_lock(&akm->val_mutex);
	en = akm->enable_flag;
	mutex_unlock(&akm->val_mutex);

	if (en == 0) {
		if (atomic_cmpxchg(&akm->active, 1, 0) == 1) {
			wake_up(&akm->open_wq);
			printk(KERN_INFO TAGI "Deactivated\n");
		}
	} else {
		if (atomic_cmpxchg(&akm->active, 0, 1) == 0) {
			wake_up(&akm->open_wq);
			printk(KERN_INFO TAGI "Activated\n");
		}
	}
	printk(KERN_INFO TAGI
		"Status updated: enable=0x%X, active=%d\n",
		en, atomic_read(&akm->active));
}

static int akm_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<MAG_DATA_FLAG);
	akm->enable_flag |= ((uint32_t)(enable))<<MAG_DATA_FLAG;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);

	mutex_lock(&akm->op_mutex);
	if (enable) {
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			printk(KERN_ERR TAGE
				"Power on fail! ret=%d\n", ret);
			goto exit;
		}
	}

	if (akm->use_poll && akm->pdata->auto_report) {
		if (enable) {
			AKECS_Reset(akm, 0);
			AKECS_SetMode(akm,
				AKM_MODE_SNG_MEASURE | AKM8963_BIT_OP_16);
			poll_delay_num = 0;
				queue_delayed_work(akm->work_queue, &akm->dwork, 
					(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
				queue_delayed_work(akm->data_wq, &akm->datawork,
					(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
			#if 0
			schedule_delayed_work(&akm->dwork,
					msecs_to_jiffies(
						0/*akm->delay[MAG_DATA_FLAG]*/));
			#endif
			printk(KERN_INFO TAGI "%s %d\n", __func__, akm->delay[MAG_DATA_FLAG]);
		} else {
			cancel_delayed_work_sync(&akm->dwork);
			cancel_delayed_work(&akm->datawork);
			AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
			akm->work_state=0;
		}
	} else {
		if (enable)
			enable_irq(akm->i2c->irq);
		else
			disable_irq(akm->i2c->irq);
	}

	if (!enable) {
		ret = akm_compass_power_set(akm, false);
		if (ret) {
			printk(KERN_ERR TAGE
				"Power off fail! ret=%d\n", ret);
			goto exit;
		}
	}

exit:
	mutex_unlock(&akm->op_mutex);
	return ret;
}

static ssize_t akm_compass_sysfs_enable_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int flag;

	mutex_lock(&akm->val_mutex);
	flag = ((akm->enable_flag >> pos) & 1);
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", flag);
}

static ssize_t akm_compass_sysfs_enable_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long en = 0;
	int ret = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	mutex_lock(&akm->op_mutex);
	ret = akm_compass_power_set(akm, en);
	if (ret) {
		printk(KERN_ERR TAGE
			"Fail to configure device power!\n");
		goto exit;
	}

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<pos);
	akm->enable_flag |= ((uint32_t)(en))<<pos;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);

exit:
	mutex_unlock(&akm->op_mutex);
	return ret ? ret : count;
}

/***** Acceleration ***/
static ssize_t akm_enable_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t akm_enable_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t akm_enable_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t akm_enable_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t akm_enable_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t akm_enable_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** sysfs delay **************************************************/
static int akm_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);
	mutex_lock(&akm->val_mutex);
	akm->delay[MAG_DATA_FLAG] = delay_msec;
	mutex_unlock(&akm->val_mutex);

	return 0;
}

static ssize_t akm_compass_sysfs_delay_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	unsigned long val;

	mutex_lock(&akm->val_mutex);
	val = akm->delay[pos];
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", val);
}

static ssize_t akm_compass_sysfs_delay_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	unsigned long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (kstrtoul(buf, AKM_BASE_NUM, &val))
		return -EINVAL;

	mutex_lock(&akm->val_mutex);
	akm->delay[pos] = val;
	mutex_unlock(&akm->val_mutex);

	return count;
}

/***** Accelerometer ***/
static ssize_t akm_delay_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t akm_delay_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t akm_delay_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t akm_delay_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t akm_delay_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t akm_delay_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** accel (binary) ***/
static ssize_t akm_bin_accel_write(
	struct file *file,
	struct kobject *kobj,
	struct bin_attribute *attr,
		char *buf,
		loff_t pos,
		size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int16_t *accel_data;

	if (size == 0)
		return 0;

	accel_data = (int16_t *)buf;

	mutex_lock(&akm->accel_mutex);
	akm->accel_data[0] = accel_data[0];
	akm->accel_data[1] = accel_data[1];
	akm->accel_data[2] = accel_data[2];
	mutex_unlock(&akm->accel_mutex);

	if(DEBUG)
		printk(KERN_INFO TAGI "accel:%d,%d,%d\n",
			accel_data[0], accel_data[1], accel_data[2]);

	return size;
}


#if AKM_DEBUG_IF
static ssize_t akm_sysfs_mode_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	long mode = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &mode))
		return -EINVAL;

	if (AKECS_SetMode(akm, (uint8_t)mode) < 0)
		return -EINVAL;

	return 1;
}

static ssize_t akm_buf_print(
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

static ssize_t akm_sysfs_bdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	uint8_t rbuf[AKM_SENSOR_DATA_SIZE];

	mutex_lock(&akm->sensor_mutex);
	memcpy(&rbuf, akm->sense_data, sizeof(rbuf));
	mutex_unlock(&akm->sensor_mutex);

	return akm_buf_print(buf, rbuf, AKM_SENSOR_DATA_SIZE);
}

static ssize_t akm_sysfs_asa_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t asa[3];

	err = AKECS_SetMode(akm, AKM_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	asa[0] = AKM_FUSE_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, asa, 3);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return akm_buf_print(buf, asa, 3);
}
#endif

static ssize_t akm_sysfs_regs_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* The total number of registers depends on the device. */
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err,i;
	uint8_t regs[AKM_REGS_SIZE];

	/* This function does not lock mutex obj */
	regs[0] = AKM_REGS_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, regs, AKM_REGS_SIZE);
	if (err < 0)
		return err;
	for(i=0;i<AKM_REGS_SIZE;i++)
		printk(KERN_INFO TAGI "%s reg_add:%x  reg_val:%x \n",
				__func__, i+AKM_REGS_1ST_ADDR, regs[i]); 
	return scnprintf(buf, PAGE_SIZE, "%d\n", err);
}

static struct device_attribute akm_compass_attributes[] = {
	__ATTR(enable_acc, 0660, akm_enable_acc_show, akm_enable_acc_store),
	__ATTR(enable_mag, 0660, akm_enable_mag_show, akm_enable_mag_store),
	__ATTR(enable_fusion, 0660, akm_enable_fusion_show,
			akm_enable_fusion_store),
	__ATTR(delay_acc,  0660, akm_delay_acc_show,  akm_delay_acc_store),
	__ATTR(delay_mag,  0660, akm_delay_mag_show,  akm_delay_mag_store),
	__ATTR(delay_fusion, 0660, akm_delay_fusion_show,
			akm_delay_fusion_store),
#if AKM_DEBUG_IF
	__ATTR(mode,  0220, NULL, akm_sysfs_mode_store),
	__ATTR(bdata, 0440, akm_sysfs_bdata_show, NULL),
	__ATTR(asa,   0440, akm_sysfs_asa_show, NULL),
#endif
	__ATTR(regs,  0440, akm_sysfs_regs_show, NULL),

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

static struct bin_attribute akm_compass_bin_attributes[] = {
	__BIN_ATTR(accel, 0220, 6, NULL,
				NULL, akm_bin_accel_write),
	__BIN_ATTR_NULL
};

static char const *const device_link_name = "i2c";
static dev_t const akm_compass_device_dev_t = MKDEV(MISC_MAJOR, 240);

static int create_sysfs_interfaces(struct akm_compass_data *akm)
{
	int err;

	if (NULL == akm)
		return -EINVAL;

	err = 0;

	akm->compass = class_create(THIS_MODULE, AKM_SYSCLS_NAME);
	if (IS_ERR(akm->compass)) {
		err = PTR_ERR(akm->compass);
		goto exit_class_create_failed;
	}

	akm->class_dev = device_create(
			akm->compass,
			NULL,
			akm_compass_device_dev_t,
			akm,
			AKM_SYSDEV_NAME);
	if (IS_ERR(akm->class_dev)) {
		err = PTR_ERR(akm->class_dev);
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&akm->class_dev->kobj,
			&akm->i2c->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	err = create_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
	if (0 > err)
		goto exit_device_binary_attributes_create_failed;

	return err;

exit_device_binary_attributes_create_failed:
	remove_device_attributes(akm->class_dev, akm_compass_attributes);
exit_device_attributes_create_failed:
	sysfs_remove_link(&akm->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(akm->compass, akm_compass_device_dev_t);
exit_class_device_create_failed:
	akm->class_dev = NULL;
	class_destroy(akm->compass);
exit_class_create_failed:
	akm->compass = NULL;
	return err;
}

static void remove_sysfs_interfaces(struct akm_compass_data *akm)
{
	if (NULL == akm)
		return;

	if (NULL != akm->class_dev) {
		remove_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
		remove_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
		sysfs_remove_link(
			&akm->class_dev->kobj,
			device_link_name);
		akm->class_dev = NULL;
	}
	if (NULL != akm->compass) {
		device_destroy(
			akm->compass,
			akm_compass_device_dev_t);
		class_destroy(akm->compass);
		akm->compass = NULL;
	}
}


/***** akm input device functions ***********************************/
static int akm_compass_input_init(
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
	(*input)->name = AKM_INPUT_DEVICE_NAME;

	/* Register */
	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

/***** akm functions ************************************************/
static irqreturn_t akm_compass_irq(int irq, void *handle)
{
	struct akm_compass_data *akm = handle;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;
    static int cont_int_num = 0;
    static unsigned long last_int_time = 0;
    unsigned long int_time;
    unsigned long diff;

	memset(buffer, 0, sizeof(buffer));

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	/* Read whole data */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0) {
		if(DEBUG)
			printk(KERN_ERR TAGE "IRQ I2C error.\n");
		akm->is_busy = 0;
		mutex_unlock(&akm->sensor_mutex);
		/***** unlock *****/

		return IRQ_HANDLED;
	}
	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		goto work_func_none;

	memcpy(akm->sense_data, buffer, AKM_SENSOR_DATA_SIZE);
	akm->is_busy = 0;

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	atomic_set(&akm->drdy, 1);
	wake_up(&akm->drdy_wq);

	if(DEBUG)
		printk(KERN_INFO TAGI "IRQ handled.\n");
    int_time = jiffies;
    diff = (long)int_time - (long)last_int_time;
    if((diff *1000)/HZ < CONT_INT_TIME)
    {
        if(cont_int_num < CONT_INT_GATE)
            cont_int_num++;
        else
            printk(KERN_ERR TAGE "continuous interrupts error\n"); 
    }
    else
    {
        cont_int_num = 0;
    }
    last_int_time = int_time;
	return IRQ_HANDLED;

work_func_none:
	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	printk(KERN_ERR TAGE "IRQ not handled.\n");
	return IRQ_NONE;
}

static int akm_compass_suspend(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	//int ret = 0;
	
	if (AKM_IS_MAG_DATA_ENABLED() &&
		akm->use_poll &&
		akm->pdata->auto_report)
	{
		cancel_delayed_work_sync(&akm->dwork);
		cancel_delayed_work(&akm->datawork);
	}

	akm->state.power_on = akm->power_enabled;
	if (akm->state.power_on) {
		akm_compass_power_set(akm, false);
		/* Clear status */
		akm->is_busy = 0;
		atomic_set(&akm->drdy, 0);
	}
	

/* 
	ret = pinctrl_select_state(akm->pinctrl, akm->pin_sleep);
	if (ret)
		printk(KERN_ERR TAGE "Can't select pinctrl state\n");
*/
	printk(KERN_INFO TAGI "suspended\n");

	return 0;
}

static int akm_compass_resume(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;
	int flag;

/* 
	ret = pinctrl_select_state(akm->pinctrl, akm->pin_default);
	if (ret)
		printk(KERN_ERR TAGE "Can't select pinctrl state\n");
*/
	mutex_lock(&akm->val_mutex);
	flag = ((akm->enable_flag >> MAG_DATA_FLAG) & 1);
	mutex_unlock(&akm->val_mutex);
    if(flag) 
		printk(KERN_INFO TAGI "sensor on after suspend\n");


	if (akm->state.power_on) {
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			printk(KERN_ERR TAGE "Sensor power resume fail!\n");
			goto exit;
		}

		ret = AKECS_SetMode(akm, akm->state.mode);
		if (ret) {
			printk(KERN_ERR TAGE "Sensor state resume fail!\n");
			goto exit;
		}

		if (AKM_IS_MAG_DATA_ENABLED() &&
			akm->use_poll &&
			akm->pdata->auto_report)
		    /*add qcom patch start*/
			poll_delay_num = 0;
				queue_delayed_work(akm->work_queue, &akm->dwork, 
					(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
				queue_delayed_work(akm->data_wq, &akm->datawork,
					(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
			/*
			schedule_delayed_work(&akm->dwork,
				(unsigned long)nsecs_to_jiffies64(
				akm->delay[MAG_DATA_FLAG]));
			*/
			/*add qcom patch end*/
	}
	printk(KERN_INFO TAGI "resumed\n");

exit:
	return 0;

}

static int akm8963_i2c_check_device(
	struct i2c_client *client)
{
	/* AK8963 specific function */
	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int err;

	akm->sense_info[0] = AK8963_REG_WIA;
	err = akm_i2c_rxdata(client, akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	/* Set FUSE access mode */
	err = AKECS_SetMode(akm, AK8963_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	akm->sense_conf[0] = AK8963_FUSE_ASAX;
	err = akm_i2c_rxdata(client, akm->sense_conf, AKM_SENSOR_CONF_SIZE);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AK8963_MODE_POWERDOWN);
	if (err < 0)
		return err;

	/* Check read data */
	if (akm->sense_info[0] != AK8963_WIA_VALUE) {
		printk(KERN_ERR TAGE
			"%s: The device is not AKM Compass.\n", __func__);
		return -ENXIO;
	}

	return err;
}

static int akm_compass_power_set(struct akm_compass_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		/*rc = regulator_disable(data->vdd);
		if (rc) {
			printk(KERN_ERR TAGE
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			printk(KERN_ERR TAGE
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}*/
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		/*rc = regulator_enable(data->vdd);
		if (rc) {
			printk(KERN_ERR TAGE
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			printk(KERN_ERR TAGE
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}*/
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		printk(KERN_INFO TAGI
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}
/*err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		printk(KERN_ERR TAGE "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;*/

}

static int akm_compass_power_init(struct akm_compass_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				AKM8963_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				AKM8963_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			printk(KERN_ERR TAGE
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				AKM8963_VDD_MIN_UV, AKM8963_VDD_MAX_UV);
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
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;

		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			printk(KERN_ERR TAGE
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				AKM8963_VIO_MIN_UV, AKM8963_VIO_MAX_UV);
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
		regulator_set_voltage(data->vio, 0, AKM8963_VIO_MAX_UV);
reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AKM8963_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

#ifdef CONFIG_OF
static int akm_compass_parse_dt(struct device *dev,
				struct akm8963_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "ak,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read akm,layout\n");
		return rc;
	} else {
		pdata->layout = temp_val;
	}

	if (of_property_read_bool(np, "ak,auto-report")) {
		pdata->auto_report = 1;
		pdata->use_int = 0;
	} else {
		pdata->auto_report = 0;
		if (of_property_read_bool(dev->of_node, "ak,use-interrupt")) {
			pdata->use_int = 1;
			/* check gpio_int later, if it is invalid,
			 * just use poll */
			pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
					"ak,gpio-int", 0, &pdata->int_flags);
		} else {
			pdata->use_int = 0;
		}
	}

	/*pdata->gpio_rstn = of_get_named_gpio_flags(dev->of_node,
			"ak,gpio-rstn", 0, NULL);*/
	pdata->gpio_rstn = 0;

	rc = of_property_read_u32(np, "iic-address", &temp_val);
	if (rc) {
		printk(KERN_INFO TAGI "Unable to read iic-address\n");
		pdata->iic_add = -2;
	} else {
		pdata->iic_add = temp_val;
	}
	printk(KERN_INFO TAGI "%s rc %d temp_val %d\n", __func__, rc, temp_val);

	return 0;
}
#else
static int akm_compass_parse_dt(struct device *dev,
				struct akm8963_platform_data *pdata)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */
/* 
static int akm8963_pinctrl_init(struct akm_compass_data *s_akm)
{
	struct i2c_client *client = s_akm->i2c;

	s_akm->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(s_akm->pinctrl)) {
		printk(KERN_ERR TAGE "Failed to get pinctrl\n");
		return PTR_ERR(s_akm->pinctrl);
	}

	s_akm->pin_default = pinctrl_lookup_state(s_akm->pinctrl,
			"ak8963_default");
	if (IS_ERR_OR_NULL(s_akm->pin_default)) {
		printk(KERN_ERR TAGE "Failed to look up default state\n");
		return PTR_ERR(s_akm->pin_default);
	}

	s_akm->pin_sleep = pinctrl_lookup_state(s_akm->pinctrl,
			"ak8963_sleep");
	if (IS_ERR_OR_NULL(s_akm->pin_sleep)) {
		printk(KERN_ERR TAGE "Failed to look up sleep state\n");
		return PTR_ERR(s_akm->pin_sleep);
	}

	return 0;
}
*/
static int akm8963_mag_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int akm8963_mag_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver akm8963_mag_pd_driver = {
	.probe  = akm8963_mag_pd_probe,
	.remove = akm8963_mag_pd_remove,    
	.driver = {
		.name  = "msensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device akm8963_mag_pd_device = {
    .name = "msensor",
    .id   = -1,
};

int
TEST_DATA(const char testno[],
		  const char testname[],
          const int testdata,
		  const int lolimit,
		  const int hilimit,
          int * pf_total)
{
	int pf;                     //Pass;1, Fail;-1

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		// Display header
		printk(KERN_INFO TAGI "--------------------------------------------------------------------\n");
		printk(KERN_INFO TAGI " Test No. Test Name    Fail    Test Data    [      Low         High]\n");
		printk(KERN_INFO TAGI "--------------------------------------------------------------------\n");

		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		// Display result
		printk(KERN_INFO TAGI "--------------------------------------------------------------------\n");
		if (*pf_total == 1) {
			printk(KERN_INFO TAGI "Factory shipment test was passed.\n\n");
		} else {
			printk(KERN_ERR TAGE "Factory shipment test was failed.\n\n");
		}

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit)) {
			//Pass
			pf = 1;
		} else {
			//Fail
			pf = -1;
		}

		//display result
		printk(KERN_INFO TAGI " %7s  %-10s      %c    %9d    [%9d    %9d]\n",
				 testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
				 lolimit, hilimit);
	}

	//Pass/Fail check
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1)) {
			*pf_total = 1;            //Pass
		} else {
			*pf_total = -1;           //Fail
		}
	}
	return pf;
}


int FST_AK8963(void)
{
	int   pf_total;  //p/f flag for this subtest
	char	i2cData[16];
	int   hdata[3];
	int   asax;
	int   asay;
	int   asaz;
	struct akm_compass_data *akm;
	if(akm8963_mag_i2c_client)akm=i2c_get_clientdata(akm8963_mag_i2c_client);

	if(akm->enable_flag & (1<<MAG_DATA_FLAG) || akm->work_state) {
		cancel_delayed_work_sync(&akm->dwork);
		cancel_delayed_work(&akm->datawork);
		AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	} 
	//***********************************************
	//	Reset Test Result
	//***********************************************
	pf_total = 1;
	printk(KERN_INFO TAGI "%s 1 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	//***********************************************
	//	Step1
	//***********************************************
	
	// Set to PowerDown mode 
	//if (AKECS_SetMode(AK8963_MODE_POWERDOWN) < 0) {
	//	printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
	//	return 0;
	//}
	AKECS_Reset(akm,0);
	msleep(1);
	
	// When the serial interface is SPI,
	// write "00011011" to I2CDIS register(to disable I2C,).
	if(CSPEC_SPI_USE == 1){
		i2cData[0] = AK8963_REG_I2CDIS;
		i2cData[1] = 0x1B;
		if (AKI2C_TxData(i2cData, 2) < 0) {
			printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
			goto error_exit;
		}
	}
	
	// Read values from WIA to ASTC.
	i2cData[0] = AK8963_REG_WIA;
	if (AKI2C_RxData(i2cData, 7) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	
	// TEST
	TEST_DATA(TLIMIT_NO_RST_WIA,  TLIMIT_TN_RST_WIA,  (int)i2cData[0],	TLIMIT_LO_RST_WIA,	TLIMIT_HI_RST_WIA,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_INFO, TLIMIT_TN_RST_INFO, (int)i2cData[1],	TLIMIT_LO_RST_INFO, TLIMIT_HI_RST_INFO, &pf_total);
	TEST_DATA(TLIMIT_NO_RST_ST1,  TLIMIT_TN_RST_ST1,  (int)i2cData[2],	TLIMIT_LO_RST_ST1,	TLIMIT_HI_RST_ST1,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_HXL,  TLIMIT_TN_RST_HXL,  (int)i2cData[3],	TLIMIT_LO_RST_HXL,	TLIMIT_HI_RST_HXL,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_HXH,  TLIMIT_TN_RST_HXH,  (int)i2cData[4],	TLIMIT_LO_RST_HXH,	TLIMIT_HI_RST_HXH,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_HYL,  TLIMIT_TN_RST_HYL,  (int)i2cData[5],	TLIMIT_LO_RST_HYL,	TLIMIT_HI_RST_HYL,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_HYH,  TLIMIT_TN_RST_HYH,  (int)i2cData[6],	TLIMIT_LO_RST_HYH,	TLIMIT_HI_RST_HYH,	&pf_total);
	// our i2c only most can read 8 byte  at one time ,
	i2cData[7]= AK8963_REG_HZL;
	if (AKI2C_RxData((i2cData+7), 6) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	TEST_DATA(TLIMIT_NO_RST_HZL,  TLIMIT_TN_RST_HZL,  (int)i2cData[7],	TLIMIT_LO_RST_HZL,	TLIMIT_HI_RST_HZL,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_HZH,  TLIMIT_TN_RST_HZH,  (int)i2cData[8],	TLIMIT_LO_RST_HZH,	TLIMIT_HI_RST_HZH,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_ST2,  TLIMIT_TN_RST_ST2,  (int)i2cData[9],	TLIMIT_LO_RST_ST2,	TLIMIT_HI_RST_ST2,	&pf_total);
	TEST_DATA(TLIMIT_NO_RST_CNTL, TLIMIT_TN_RST_CNTL, (int)i2cData[10], TLIMIT_LO_RST_CNTL, TLIMIT_HI_RST_CNTL, &pf_total);
	// i2cData[11] is BLANK.
	TEST_DATA(TLIMIT_NO_RST_ASTC, TLIMIT_TN_RST_ASTC, (int)i2cData[12], TLIMIT_LO_RST_ASTC, TLIMIT_HI_RST_ASTC, &pf_total);
	
	// Read values from I2CDIS.
	i2cData[0] = AK8963_REG_I2CDIS;
	if (AKI2C_RxData(i2cData, 1) < 0 ) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	if(CSPEC_SPI_USE == 1){
		TEST_DATA(TLIMIT_NO_RST_I2CDIS, TLIMIT_TN_RST_I2CDIS, (int)i2cData[0], TLIMIT_LO_RST_I2CDIS_USESPI, TLIMIT_HI_RST_I2CDIS_USESPI, &pf_total);
	}else{
		TEST_DATA(TLIMIT_NO_RST_I2CDIS, TLIMIT_TN_RST_I2CDIS, (int)i2cData[0], TLIMIT_LO_RST_I2CDIS_USEI2C, TLIMIT_HI_RST_I2CDIS_USEI2C, &pf_total);
	}
	
	// Set to FUSE ROM access mode
	if (AKECS_SetMode(akm, AK8963_MODE_FUSE_ACCESS) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	
	// Read values from ASAX to ASAZ
	i2cData[0] = AK8963_FUSE_ASAX;
	if (AKI2C_RxData(i2cData, 3) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	asax = (int)i2cData[0];
	asay = (int)i2cData[1];
	asaz = (int)i2cData[2];
	
	// TEST
	TEST_DATA(TLIMIT_NO_ASAX, TLIMIT_TN_ASAX, asax, TLIMIT_LO_ASAX, TLIMIT_HI_ASAX, &pf_total);
	TEST_DATA(TLIMIT_NO_ASAY, TLIMIT_TN_ASAY, asay, TLIMIT_LO_ASAY, TLIMIT_HI_ASAY, &pf_total);
	TEST_DATA(TLIMIT_NO_ASAZ, TLIMIT_TN_ASAZ, asaz, TLIMIT_LO_ASAZ, TLIMIT_HI_ASAZ, &pf_total);
	
	// Read values. CNTL
	i2cData[0] = AK8963_REG_CNTL1;
	if (AKI2C_RxData(i2cData, 1)< 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	
	// Set to PowerDown mode 
	if (AKECS_SetMode(akm, AK8963_MODE_POWERDOWN) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	
	// TEST
	TEST_DATA(TLIMIT_NO_WR_CNTL, TLIMIT_TN_WR_CNTL, (int)i2cData[0], TLIMIT_LO_WR_CNTL, TLIMIT_HI_WR_CNTL, &pf_total);

	
	//***********************************************
	//	Step2
	//***********************************************
	printk(KERN_INFO TAGI "%s 2 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Set to SNG measurement pattern (Set CNTL register) 
	if (AKECS_SetMode(akm, AK8963_MODE_SNG_MEASURE) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	printk(KERN_INFO TAGI "%s 3 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Wait for DRDY pin changes to HIGH.
	msleep(10);
	printk(KERN_INFO TAGI "%s 4 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Get measurement data from AK8963
	// ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + ST2
	// = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 = 8 bytes
	if (AKECS_GetData_custom(akm, i2cData,SENSOR_DATA_SIZE) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}

	hdata[0] = (s16)(i2cData[1] | (i2cData[2] << 8));
	hdata[1] = (s16)(i2cData[3] | (i2cData[4] << 8));
	hdata[2] = (s16)(i2cData[5] | (i2cData[6] << 8));
	// AK8963 @ 14 BIT
	hdata[0] <<= 2;
	hdata[1] <<= 2;
	hdata[2] <<= 2;

	
	// TEST
	TEST_DATA(TLIMIT_NO_SNG_ST1, TLIMIT_TN_SNG_ST1, (int)i2cData[0], TLIMIT_LO_SNG_ST1, TLIMIT_HI_SNG_ST1, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HX, TLIMIT_TN_SNG_HX, hdata[0], TLIMIT_LO_SNG_HX, TLIMIT_HI_SNG_HX, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HY, TLIMIT_TN_SNG_HY, hdata[1], TLIMIT_LO_SNG_HY, TLIMIT_HI_SNG_HY, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HZ, TLIMIT_TN_SNG_HZ, hdata[2], TLIMIT_LO_SNG_HZ, TLIMIT_HI_SNG_HZ, &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_ST2, TLIMIT_TN_SNG_ST2, (int)i2cData[8], TLIMIT_LO_SNG_ST2, TLIMIT_HI_SNG_ST2, &pf_total);
	printk(KERN_INFO TAGI "%s 5 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Generate magnetic field for self-test (Set ASTC register)
	i2cData[0] = AK8963_REG_ASTC;
	i2cData[1] = 0x40;
	if (AKI2C_TxData(i2cData, 2) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	printk(KERN_INFO TAGI "%s 6 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Set to Self-test mode (Set CNTL register)
	if (AKECS_SetMode(akm, AK8963_MODE_SELF_TEST) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
	printk(KERN_INFO TAGI "%s 7 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Wait for DRDY pin changes to HIGH.
	msleep(10);
	printk(KERN_INFO TAGI "%s 8 is_busy %d\n", __func__, akm->is_busy);/*laizhilong add*/
	// Get measurement data from AK8963
	// ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + ST2
	// = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 = 8Byte
	if (AKECS_GetData_custom(akm, i2cData,SENSOR_DATA_SIZE) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}
		
	// TEST
	TEST_DATA(TLIMIT_NO_SLF_ST1, TLIMIT_TN_SLF_ST1, (int)i2cData[0], TLIMIT_LO_SLF_ST1, TLIMIT_HI_SLF_ST1, &pf_total);

	hdata[0] = (s16)(i2cData[1] | (i2cData[2] << 8));
	hdata[1] = (s16)(i2cData[3] | (i2cData[4] << 8));
	hdata[2] = (s16)(i2cData[5] | (i2cData[6] << 8));

	// AK8963 @ 14 BIT
	hdata[0] <<= 2;
	hdata[1] <<= 2;
	hdata[2] <<= 2;
	
	printk(KERN_INFO TAGI "hdata[0] = %d\n",hdata[0] );
	printk(KERN_INFO TAGI "asax = %d\n",asax );
	TEST_DATA(
			  TLIMIT_NO_SLF_RVHX, 
			  TLIMIT_TN_SLF_RVHX, 
			  (hdata[0])*((asax - 128)/2/128 + 1),
			  TLIMIT_LO_SLF_RVHX,
			  TLIMIT_HI_SLF_RVHX,
			  &pf_total
			  );
	
	TEST_DATA(
			  TLIMIT_NO_SLF_RVHY,
			  TLIMIT_TN_SLF_RVHY,
			  (hdata[1])*((asay - 128)/2/128 + 1),
			  TLIMIT_LO_SLF_RVHY,
			  TLIMIT_HI_SLF_RVHY,
			  &pf_total
			  );
	
	TEST_DATA(
			  TLIMIT_NO_SLF_RVHZ,
			  TLIMIT_TN_SLF_RVHZ,
			  (hdata[2])*((asaz - 128)/2/128 + 1),
			  TLIMIT_LO_SLF_RVHZ,
			  TLIMIT_HI_SLF_RVHZ,
			  &pf_total
			  );
	// TEST
	TEST_DATA(TLIMIT_NO_SLF_ST2, TLIMIT_TN_SLF_ST2, (int)i2cData[8], TLIMIT_LO_SLF_ST2, TLIMIT_HI_SLF_ST2, &pf_total);
	
	// Set to Normal mode for self-test.
	i2cData[0] = AK8963_REG_ASTC;
	i2cData[1] = 0x00;
	if (AKI2C_TxData(i2cData, 2) < 0) {
		printk(KERN_ERR TAGE "%s:%d Error.\n", __FUNCTION__, __LINE__);
		goto error_exit;
	}

	printk(KERN_INFO TAGI "pf_total = %d\n",pf_total );
	goto noerror_return;
error_exit:
	pf_total=0;
noerror_return:
    if(akm->enable_flag & (1<<MAG_DATA_FLAG)) {
    	AKECS_SetMode(akm,
    		AKM_MODE_SNG_MEASURE | AKM8963_BIT_OP_16);
	    /*add qcom patch start*/
    
			queue_delayed_work(akm->work_queue, &akm->dwork, 
				(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
			queue_delayed_work(akm->data_wq, &akm->datawork,
				(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
			
		/*
    	schedule_delayed_work(&akm->dwork,
    			msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
    		*/
    	/*add qcom patch end*/
    	printk(KERN_INFO TAGI "%s %d\n", __func__, akm->delay[MAG_DATA_FLAG]);
    }
	else
	{
        akm->work_state=0;
	}
    return pf_total;
}

int FctShipmntTestProcess_Body(void)
{
	int pf_total = 1;

	//***********************************************
	//    Reset Test Result
	//***********************************************
	TEST_DATA(NULL, "START", 0, 0, 0, &pf_total);

	//***********************************************
	//    Step 1 to 2
	//***********************************************
	pf_total = FST_AK8963();


	//***********************************************
	//    Judge Test Result
	//***********************************************
	TEST_DATA(NULL, "END", 0, 0, 0, &pf_total);

	return pf_total;
}

static ssize_t akm8963_mag_shipment_test(struct device_driver *ddri, char *buf)
{
	/*char result[10];*/
	int ret=0;
	int res = 0;
	res = FctShipmntTestProcess_Body();
	if(1 == res)
	{
	   printk(KERN_INFO TAGI "shipment_test pass\n");
	   //strcpy(result,"y");
	   ret = 0x0F;
	   
	}
	else if(-1 == res)
	{
	   printk(KERN_ERR TAGE "shipment_test fail\n");
	   //strcpy(result,"n");
	   ret = -1;
	}
	else
	{
	  printk(KERN_INFO TAGI "shipment_test NaN\n");
	 // strcpy(result,"NaN");
	  ret = -1;
	}
	return sprintf(buf, "%d\n", ret); 
	//return sprintf(buf, "%s\n", result);        
}
/*
static long AKECS_SetMode_SngMeasure(void)
{
	char buffer[2];

	buffer[0] = AK8963_REG_CNTL1;
	buffer[1] = AK8963_MODE_SNG_MEASURE;
	
	return AKI2C_TxData(buffer, 2);
}
*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	/*char sensordata[SENSOR_DATA_SIZE];*/
	char strbuf[AKM09911_BUFSIZE];
	struct akm_compass_data *akm=NULL;
	if(akm8963_mag_i2c_client)akm=i2c_get_clientdata(akm8963_mag_i2c_client);

    if(akm)
    {
		if(!akm->data_valid)
		{
			/*
			memcpy(sensordata, akm->sense_rawdata, sizeof(akm->sense_rawdata)); 
			mutex_unlock(&akm->sensor_data_mutex);*/
			AKECS_Reset(akm, 0);
			AKECS_SetMode(akm,	AKM_MODE_SNG_MEASURE | AKM8963_BIT_OP_16);
			/*AKECS_SetMode_SngMeasure();*/
			msleep(10);
			/*AKECS_GetData(akm, sensordata, SENSOR_DATA_SIZE);*/
			akm_dev_poll((struct work_struct *)&akm->dwork);
			if(!AKM_IS_MAG_DATA_ENABLED())
			{
				cancel_delayed_work_sync(&akm->dwork);
				cancel_delayed_work(&akm->datawork);
				AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
			}
			else
			{
				akm->work_state=1;
			}	
		}
	}
	
	/*sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", sensordata[0],sensordata[1],sensordata[2],
		sensordata[3],sensordata[4],sensordata[5],sensordata[6],sensordata[7],sensordata[8]);*/
	mutex_lock(&akm->sensor_data_mutex);
	sprintf(strbuf, "%d %d %d\n", akm->sense_rawdata[0],akm->sense_rawdata[1],akm->sense_rawdata[2]);
	akm->data_valid=0x00;
	mutex_unlock(&akm->sensor_data_mutex);

	return sprintf(buf, "%s\n", strbuf);
}

static DRIVER_ATTR(selftest,S_IRUGO | S_IWUSR, akm8963_mag_shipment_test, NULL);
static DRIVER_ATTR(data,  S_IRUGO, show_sensordata_value, NULL);

static struct driver_attribute *akm8963_mag_attr_list[] = {
	&driver_attr_selftest,
	&driver_attr_data,
};

static int akm8963_mag_create_attr(struct device_driver *driver) 
{
	int i;
	for (i = 0; i < ARRAY_SIZE(akm8963_mag_attr_list); i++)
		if (driver_create_file(driver, akm8963_mag_attr_list[i]))
			goto error;
	return 0;

error:
	for (i--; i >= 0; i--)
		driver_remove_file(driver, akm8963_mag_attr_list[i]);
	printk(KERN_ERR TAGE "%s:Unable to create interface\n", __func__);
	return -1;
}
/*
static int akm8963_mag_remove_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(akm8963_mag_attr_list)/sizeof(akm8963_mag_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, akm8963_mag_attr_list[idx]);           
		printk(KERN_INFO TAGI "%s (%s) = %d\n", __func__, akm8963_mag_attr_list[idx]->attr.name, err);
	}    
	return err;
}
*/

static void akm_dev_poll(struct work_struct *work)
{
	struct akm_compass_data *akm;
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int ret;
	int mag_x, mag_y, mag_z;
	int tmp;
	/*add qcom patch start*/
	int count = 10;
//	ktime_t timestamp;
	/*add qcom patch end*/
	static int same_num_x = 0;
	static int same_num_y = 0;
	static int same_num_z = 0;
    static unsigned long last_poll_time = 0;
    unsigned long poll_time;
    unsigned long diff;

	akm = container_of((struct delayed_work *)work,
			struct akm_compass_data,  dwork);
    /*add qcom patch start*/
	do {
		/* The typical time for single measurement is 7.2ms */
	ret = AKECS_GetData_Poll(akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		if (ret == -EAGAIN)
			udelay(1000);
	} while ((ret == -EAGAIN) && (--count));
	if (!count) {
		printk(KERN_ERR TAGE "Get data failed\n");
		goto exit;
	}
//	timestamp = ktime_get_boottime();
	/*add qcom patch end*/

	tmp = 0xFF & (dat_buf[7] + dat_buf[0]);
	if (STATUS_ERROR(tmp)) {
		/*printk(KERN_ERR TAGE "Status error(0x%x). Reset...\n",
				tmp);*/
		AKECS_Reset(akm, 0);
		goto exit;
	}

	tmp = (int)((int16_t)(dat_buf[2]<<8)+((int16_t)dat_buf[1]));
	tmp = tmp * akm->sense_conf[0] / 256 + tmp / 2;
	mag_x = tmp;

	tmp = (int)((int16_t)(dat_buf[4]<<8)+((int16_t)dat_buf[3]));
	tmp = tmp * akm->sense_conf[1] / 256 + tmp / 2;
	mag_y = tmp;

	tmp = (int)((int16_t)(dat_buf[6]<<8)+((int16_t)dat_buf[5]));
	tmp = tmp * akm->sense_conf[2] / 256 + tmp / 2;
	mag_z = tmp;

	switch (akm->pdata->layout) {
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

//	input_report_abs(akm->input, ABS_X, mag_x);
//	input_report_abs(akm->input, ABS_Y, mag_y);
//	input_report_abs(akm->input, ABS_Z, mag_z);
//	input_sync(akm->input);

	mutex_lock(&akm->sensor_data_mutex);
    if(akm->sense_rawdata[0]==mag_x)
    {
        if(same_num_x < SAME_DATA_GATE)
            same_num_x++;
        else
            printk(KERN_ERR TAGE "same data x error, x = %d\n",mag_x);
    }
    else
    {
        same_num_x = 0;
    }
	if(akm->sense_rawdata[1]==mag_y)
    {
        if(same_num_y < SAME_DATA_GATE)
            same_num_y++;
        else
            printk(KERN_ERR TAGE "same data y error, y = %d\n",mag_y);
    }
    else
    {
        same_num_y = 0;
    }
	if(akm->sense_rawdata[2]==mag_z)
    {
        if(same_num_z < SAME_DATA_GATE)
            same_num_z++;
        else
            printk(KERN_ERR TAGE "same data z error, z = %d\n",mag_z);
    }
    else
    {
        same_num_z = 0;
    }
	akm->sense_rawdata[0]=mag_x;
	akm->sense_rawdata[1]=mag_y;
	akm->sense_rawdata[2]=mag_z;
	akm->data_valid=0x7f;
	mutex_unlock(&akm->sensor_data_mutex);
	/*printk(KERN_INFO TAGI "%s is_busy %d mag_x=%d, mag_y=%d, mag_z=%d\n", 
		__func__, akm->is_busy, mag_x, mag_y, mag_z);*//*laizhilong add*/
	if(DEBUG)
		printk(KERN_INFO TAGI 
			"input report: mag_x=%02x, mag_y=%02x, mag_z=%02x\n",
			mag_x, mag_y, mag_z);

exit:
	ret = AKECS_SetMode(akm, AKM_MODE_SNG_MEASURE | AKM8963_BIT_OP_16);
	if (ret < 0)
		printk(KERN_ERR TAGE "Failed to set mode\n");
	/*printk(KERN_INFO TAGI "%s %d\n", __func__, akm->delay[MAG_DATA_FLAG]);*//*laizhilong add*/

    /*add qcom patch start*/
	if (akm->use_poll)
	{
		queue_delayed_work(akm->work_queue, &akm->dwork, 
			(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
	}
    poll_time = jiffies;
    diff = (long)poll_time - (long)last_poll_time;
    if((diff *1000)/HZ > akm->delay[MAG_DATA_FLAG] * POLL_DELAY_TIMES)
    {
        if(poll_delay_num < POLL_DELAY_GATE)
            poll_delay_num++;
        else
            printk(KERN_ERR TAGE "poll delay long error:delay = %d,time = %ld\n",
					akm->delay[MAG_DATA_FLAG],(diff *1000)/HZ); 
    }
    else
    {
        poll_delay_num = 0;
    }
    last_poll_time = poll_time;
	
}

static void akm_pull_data(struct work_struct *work)
{
	struct akm_compass_data *akm;
	ktime_t timestamp;
	akm = container_of((struct delayed_work *)work,
			struct akm_compass_data,  datawork);
	timestamp = ktime_get_boottime();
	mutex_lock(&akm->sensor_data_mutex);
	input_report_abs(akm->input, ABS_X, akm->sense_rawdata[0]);
	input_report_abs(akm->input, ABS_Y, akm->sense_rawdata[1]);
	input_report_abs(akm->input, ABS_Z, akm->sense_rawdata[2]);
	input_event(akm->input,EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(akm->input,EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
//	printk(KERN_INFO TAGI "%s read mag x=%d ,y=%d ,z=%d ,time_s=%ld, time_ns=%ld \n",
//		__func__, akm->sense_rawdata[0],akm->sense_rawdata[1],akm->sense_rawdata[2], 
//		   ktime_to_timespec(timestamp).tv_sec,ktime_to_timespec(timestamp).tv_nsec);
	input_sync(akm->input);
	mutex_unlock(&akm->sensor_data_mutex);
	if (akm->use_poll)
	{
		queue_delayed_work(akm->data_wq, &akm->datawork,
				(unsigned long)msecs_to_jiffies(akm->delay[MAG_DATA_FLAG]));
	}
}

#if defined(CONFIG_FB)
static int akm8963_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
	int flag;
    struct akm_compass_data *akm = container_of(self, struct akm_compass_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
        if (*blank == FB_BLANK_POWERDOWN)
        {
			mutex_lock(&akm->val_mutex);
			flag = ((akm->enable_flag >> MAG_DATA_FLAG) & 1);
			mutex_unlock(&akm->val_mutex);
            if(flag) 
                printk(KERN_INFO TAGI "sensor on after screen off\n");
        }
	}
	return 0;
}
#endif

int akm8963_compass_probe(
		struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct akm8963_platform_data *pdata;
	int err = 0;
	int i;

	printk(KERN_INFO TAGI "start probing.\n");

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR TAGE
				"%s: check_functionality failed.\n", __func__);
		err = -ENODEV;
		goto err_i2c_check;
	}

	/* Allocate memory for driver data */
	s_akm = devm_kzalloc(&i2c->dev, sizeof(struct akm_compass_data),
			GFP_KERNEL);
	if (!s_akm) {
		printk(KERN_ERR TAGE "Failed to allocate driver data\n");
		return -ENOMEM;
	}

	print_vivo_init(TAGI "init 1\n");
	/***** I2C initialization *****/
	s_akm->i2c = i2c;
	/* set i2c data */
	i2c_set_clientdata(i2c, s_akm);

	/**** initialize variables in akm_compass_data *****/
	init_waitqueue_head(&s_akm->drdy_wq);
	init_waitqueue_head(&s_akm->open_wq);

	mutex_init(&s_akm->sensor_mutex);
	mutex_init(&s_akm->accel_mutex);
	mutex_init(&s_akm->val_mutex);
	mutex_init(&s_akm->op_mutex);
	mutex_init(&s_akm->sensor_data_mutex);

	atomic_set(&s_akm->active, 0);
	atomic_set(&s_akm->drdy, 0);

	s_akm->is_busy = 0;
	s_akm->enable_flag = 0;

	/* Set to 1G in Android coordination, AKSC format */
	s_akm->accel_data[0] = 0;
	s_akm->accel_data[1] = 0;
	s_akm->accel_data[2] = 720;

	for (i = 0; i < AKM_NUM_SENSORS; i++)
		s_akm->delay[i] = 0;

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(
				&i2c->dev,
				sizeof(struct akm8963_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			printk(KERN_ERR TAGE "Failed to allcated memory\n");
			err = -ENOMEM;
			goto err_devm;
		}

		err = akm_compass_parse_dt(&i2c->dev, pdata);
		if (err) {
			printk(KERN_ERR TAGE
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
			s_akm->pdata->layout = 0;
			s_akm->gpio_rstn = 0;
			printk(KERN_ERR TAGE "%s: No platform data.\n",
					__func__);
		}
	}

	s_akm->pdata = pdata;

	print_vivo_init(TAGI "init 2\n");
	/* check connection */
	if(0<=pdata->iic_add && 0xff>=pdata->iic_add)
	{
		printk(KERN_INFO TAGI "%s change the iic address from %x to %x\n",
		__func__, i2c->addr, pdata->iic_add);
		i2c->addr=pdata->iic_add;
	}
	err = akm_compass_power_init(s_akm, true);
	if (err/*err < 0*/)
		goto err_devm;

	err = akm_compass_power_set(s_akm, true);
	if (err < 0)
		goto err_compass_pwr_init;

	/* Pull up the reset pin */
	AKECS_Reset(s_akm, 0/*1*/);
	
	print_vivo_init(TAGI "init 3\n");

	err = akm8963_i2c_check_device(i2c);
	if (err < 0)
		goto err_compass_pwr_off;

	/***** input *****/
	err = akm_compass_input_init(&s_akm->input);
	if (err) {
		printk(KERN_ERR TAGE
				"%s: input_dev register failed\n", __func__);
		goto err_compass_pwr_off;
	}
	input_set_drvdata(s_akm->input, s_akm);

	print_vivo_init(TAGI "init 4\n");
	/* initialize pinctrl */
/* 
	if (!akm8963_pinctrl_init(s_akm)) {
		err = pinctrl_select_state(s_akm->pinctrl, s_akm->pin_default);
		if (err) {
			printk(KERN_ERR TAGE "Can't select pinctrl state\n");
			goto err_compass_pwr_off;
		}
	}
*/

	if ((s_akm->pdata->use_int) &&
		gpio_is_valid(s_akm->pdata->gpio_int)) {
		s_akm->use_poll = false;

		/* configure interrupt gpio */
		err = gpio_request(s_akm->pdata->gpio_int,
				"akm8963_gpio_int");
		if (err) {
			printk(KERN_ERR TAGE
			"Unable to request interrupt gpio %d\n",
			s_akm->pdata->gpio_int);
			goto err_unregister_device;
		}

		err = gpio_direction_input(s_akm->pdata->gpio_int);
		if (err) {
			printk(KERN_ERR TAGE
			"Unable to set direction for gpio %d\n",
			s_akm->pdata->gpio_int);
			goto err_gpio_free;
		}
		i2c->irq = gpio_to_irq(s_akm->pdata->gpio_int);

		/***** IRQ setup *****/
		s_akm->i2c->irq = i2c->irq;

		printk(KERN_INFO TAGI "%s: IRQ is #%d.\n",
				__func__, s_akm->i2c->irq);

		err = request_threaded_irq(
				s_akm->i2c->irq,
				NULL,
				akm_compass_irq,
				IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
				dev_name(&i2c->dev),
				s_akm);
		if (err) {
			printk(KERN_ERR TAGE
					"%s: request irq failed.\n", __func__);
			goto err_gpio_free;
		}
	} else if (s_akm->pdata->auto_report) {
		s_akm->use_poll = true;
		INIT_DELAYED_WORK(&s_akm->dwork, akm_dev_poll);
		/*add qcom patch start*/
		s_akm->work_queue = alloc_workqueue("akm_poll_work", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
        /*add qcom patch end*/
		INIT_DELAYED_WORK(&s_akm->datawork, akm_pull_data);
		s_akm->data_wq = NULL;
		s_akm->data_wq = create_singlethread_workqueue("akm_pull_data_work");
		if (!s_akm->data_wq) {
			printk(KERN_ERR TAGE "create workquque failed\n");
			goto err_free_irq;
		}
	}
	print_vivo_init(TAGI "init 5\n");

	/***** sysfs *****/
	err = create_sysfs_interfaces(s_akm);
	if (0 > err) {
		printk(KERN_ERR TAGE
				"%s: create sysfs failed.\n", __func__);
		goto err_free_irq;
	}

	s_akm->cdev = sensors_cdev;
	s_akm->cdev.sensors_enable = akm_enable_set;
	s_akm->cdev.sensors_poll_delay = akm_poll_delay_set;

	s_akm->delay[MAG_DATA_FLAG] = sensors_cdev.delay_msec;

	print_vivo_init(TAGI "init 6\n");
	err = sensors_classdev_register(&i2c->dev, &s_akm->cdev);
	if (err) {
		printk(KERN_ERR TAGE "class device create failed: %d\n", err);
		goto remove_sysfs;
	}

    if((err = platform_driver_register(&akm8963_mag_pd_driver)))
	{
		printk(KERN_ERR TAGE "%s failed to register akm8963_mag_driver err %d\n", __func__, err);
		goto err_unreg_sensor_class/*err_free_irq2*/;
	}

    if((err = platform_device_register(&akm8963_mag_pd_device)))
    {
		printk(KERN_ERR TAGE "%s failed to register akm8963_mag_device err %d\n", __func__, err);
		goto err_free_driver;
    }

	if((err = akm8963_mag_create_attr(&akm8963_mag_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "%s akm8963 create attribute err = %d\n", __func__, err);
		goto err_free_device;
	}
    akm8963_mag_i2c_client=i2c;
	
	print_vivo_init(TAGI "init 7\n");
#if defined(CONFIG_FB)
    s_akm->fb_notif.notifier_call = akm8963_fb_notifier_callback;
	fb_register_client(&s_akm->fb_notif);
#endif

/*
	err = akm_compass_power_set(s_akm, false);
	if (err)
		printk(KERN_ERR TAGE
			"Fail to disable power after probe: %d\n", err);
*/
	printk(KERN_INFO TAGI "successfully probed.\n");
	return 0;
err_free_device:
	platform_device_unregister(&akm8963_mag_pd_device);
err_free_driver:
	platform_driver_unregister(&akm8963_mag_pd_driver);
err_unreg_sensor_class:
	sensors_classdev_unregister(&s_akm->cdev);
remove_sysfs:
	remove_sysfs_interfaces(s_akm);
err_free_irq:
	if (s_akm->data_wq)
		destroy_workqueue(s_akm->data_wq);
	if (s_akm->i2c->irq)
		free_irq(s_akm->i2c->irq, s_akm);
err_unregister_device:
	input_unregister_device(s_akm->input);
err_gpio_free:
	if ((s_akm->pdata->use_int) &&
		(gpio_is_valid(s_akm->pdata->gpio_int)))
		gpio_free(s_akm->pdata->gpio_int);
err_compass_pwr_off:
	akm_compass_power_set(s_akm, false);
err_compass_pwr_init:
	akm_compass_power_init(s_akm, false);
err_devm:
	devm_kfree(&i2c->dev, s_akm);
err_i2c_check:
	printk(KERN_ERR TAGE " probed error.\n");
	return err;
}

static int akm8963_compass_remove(struct i2c_client *i2c)
{
	struct akm_compass_data *akm = i2c_get_clientdata(i2c);

	if (akm_compass_power_set(akm, false))
		printk(KERN_ERR TAGE "power off failed.\n");
	if (akm_compass_power_init(akm, false))
		printk(KERN_ERR TAGE "power deinit failed.\n");
	remove_sysfs_interfaces(akm);
	if (akm->data_wq)
		destroy_workqueue(akm->data_wq);
	if (akm->i2c->irq)
		free_irq(akm->i2c->irq, akm);
	input_unregister_device(akm->input);
	devm_kfree(&i2c->dev, akm);
	printk(KERN_INFO TAGI "successfully removed.\n");
	return 0;
}

static const struct i2c_device_id akm8963_compass_id[] = {
	{AKM_I2C_NAME, 0 },
	{ }
};

static const struct dev_pm_ops akm_compass_pm_ops = {
	.suspend	= akm_compass_suspend,
	.resume		= akm_compass_resume,
};

static struct of_device_id akm8963_match_table[] = {
	{ .compatible = "ak,ak8963", },
	{ .compatible = "akm,akm8963", },
	{ },
};

static struct i2c_driver akm_compass_driver = {
	.probe		= akm8963_compass_probe,
	.remove		= akm8963_compass_remove,
	.id_table	= akm8963_compass_id,
	.driver = {
		.name	= AKM_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = akm8963_match_table,
		.pm		= &akm_compass_pm_ops,
	},
};

static int __init akm_compass_init(void)
{
	printk(KERN_INFO TAGI "AKM8963 compass driver: initialize.\n");
	return i2c_add_driver(&akm_compass_driver);
}

static void __exit akm_compass_exit(void)
{
	printk(KERN_INFO TAGI "AKM compass driver: release.\n");
	i2c_del_driver(&akm_compass_driver);
}

late_initcall(akm_compass_init);
module_exit(akm_compass_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("AKM compass driver");
MODULE_LICENSE("GPL");

