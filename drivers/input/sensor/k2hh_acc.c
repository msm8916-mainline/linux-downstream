/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name          : k2hh_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.1.0
 * Date               : 2013/Mar/28
 * Description        : K2HH accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 ******************************************************************************
 Revision 1.0.0 25/Feb/2013
  first revision
  supports sysfs;
 Revision 1.1.0 28/Mar/2013
  introduces hr_timers for polling;
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>
#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>

#include	"k2hh.h"
#define DEBUG		1

#define LGE_ACCELEROMETER_NAME		"lge_accelerometer"

#define K2HH_ACCEL_CALIBRATION  1
#ifdef K2HH_ACCEL_CALIBRATION
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#endif

#define G_MAX			7995148 /* (SENSITIVITY_8G*(2^15-1)) */
#define G_MIN			- 7995392 /* (-SENSITIVITY_8G*(2^15)   */
#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define I2C_AUTO_INCREMENT	(0x00)

/* calibration result */
#define FAIL 0
#define SUCCESS 1
#define FLAT_FAIL 2

#define MS_TO_NS(x)			(x*1000000L)

#define SENSITIVITY_2G		 61	/**	ug/LSB	*/
#define SENSITIVITY_4G		122	/**	ug/LSB	*/
#define SENSITIVITY_8G		244	/**	ug/LSB	*/

#define K2HH_AXIS_X             0
#define K2HH_AXIS_Y             1
#define K2HH_AXIS_Z             2

#ifdef K2HH_ACCEL_CALIBRATION
#define K2HH_SHAKING_DETECT_THRESHOLD 160
#define CALIBRATION_DATA_AMOUNT 10
#define TESTLIMIT_XY            (215)
#define TESTLIMIT_Z_USL_LSB     (1270)
#define TESTLIMIT_Z_LSL_LSB     (778)
#endif

/* Accelerometer Sensor Operating Mode */
#define K2HH_ACC_ENABLE	(0x01)
#define K2HH_ACC_DISABLE	(0x00)

#define AXISDATA_REG		(0x28)
#define WHOAMI_K2HH_ACC	(0x41)	/*	Expctd content for WAI	*/
#define ALL_ZEROES		(0x00)
#define K2HH_ACC_PM_OFF	(0x00)
#define ACC_ENABLE_ALL_AXES	(0x07)

/*	CONTROL REGISTERS	*/
#define TEMP_L			(0x0B)
#define TEMP_H			(0x0C)
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define ACT_THS			(0x1E)	/*	Activity Threshold	*/
#define ACT_DUR			(0x1F)	/*	Activity Duration	*/
/* ctrl 1: HR ODR2 ODR1 ODR0 BDU Zenable Yenable Xenable */
#define CTRL1			(0x20)	/*	control reg 1		*/
#define CTRL2			(0x21)	/*	control reg 2		*/
#define CTRL3			(0x22)	/*	control reg 3		*/
#define CTRL4			(0x23)	/*	control reg 4		*/
#define CTRL5			(0x24)	/*	control reg 5		*/
#define CTRL6			(0x25)	/*	control reg 6		*/
#define CTRL7			(0x26)	/*	control reg 7		*/

#define FIFO_CTRL		(0x2E)	/*	fifo control reg	*/

#define INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define INT_THSX1		(0x32)	/*	interrupt 1 threshold x	*/
#define INT_THSY1		(0x33)	/*	interrupt 1 threshold y	*/
#define INT_THSZ1		(0x34)	/*	interrupt 1 threshold z	*/
#define INT_DUR1		(0x35)	/*	interrupt 1 duration	*/

#define INT_CFG2		(0x36)	/*	interrupt 2 config	*/
#define INT_SRC2		(0x37)	/*	interrupt 2 source	*/
#define INT_THS2		(0x38)	/*	interrupt 2 threshold	*/
#define INT_DUR2		(0x39)	/*	interrupt 2 duration	*/

#define REF_XL			(0x3A)	/*	reference_l_x		*/
#define REF_XH			(0x3B)	/*	reference_h_x		*/
#define REF_YL			(0x3C)	/*	reference_l_y		*/
#define REF_YH			(0x3D)	/*	reference_h_y		*/
#define REF_ZL			(0x3E)	/*	reference_l_z		*/
#define REF_ZH			(0x3F)	/*	reference_h_z		*/
/*	end CONTROL REGISTRES	*/



#define ACC_ODR10		(0x10)	/*   10Hz output data rate */
#define ACC_ODR50		(0x20)	/*   50Hz output data rate */
#define ACC_ODR100		(0x30)	/*  100Hz output data rate */
#define ACC_ODR200		(0x40)	/*  200Hz output data rate */
#define ACC_ODR400		(0x50)	/*  400Hz output data rate */
#define ACC_ODR800		(0x60)	/*  800Hz output data rate */
#define ACC_ODR_MASK		(0X70)

/* Registers configuration Mask and settings */
/* CTRL1 */
#define CTRL1_HR_DISABLE	(0x00)
#define CTRL1_HR_ENABLE		(0x80)
#define CTRL1_HR_MASK		(0x80)
#define CTRL1_BDU_ENABLE	(0x08)
#define CTRL1_BDU_MASK		(0x08)

/* CTRL2 */
#define CTRL2_IG1_INT1		(0x08)

/* CTRL3 */
#define CTRL3_IG1_INT1		(0x08)
#define CTRL3_DRDY_INT1

/* CTRL4 */
#define CTRL4_IF_ADD_INC_EN	(0x04)
#define CTRL4_BW_SCALE_ODR_AUT	(0x00)
#define CTRL4_BW_SCALE_ODR_SEL	(0x08)
#define CTRL4_ANTALIAS_BW_400	(0x00)
#define CTRL4_ANTALIAS_BW_200	(0x40)
#define CTRL4_ANTALIAS_BW_100	(0x80)
#define CTRL4_ANTALIAS_BW_50	(0xC0)
#define CTRL4_ANTALIAS_BW_MASK	(0xC0)

/* CTRL5 */
#define CTRL5_HLACTIVE_L	(0x02)
#define CTRL5_HLACTIVE_H	(0x00)

/* CTRL6 */
#define CTRL6_IG2_INT2		(0x10)
#define CTRL6_DRDY_INT2		(0x01)

/* CTRL7 */
#define CTRL7_LIR2		(0x08)
#define CTRL7_LIR1		(0x04)
/* */

#define NO_MASK			(0xFF)

#define INT1_DURATION_MASK	(0x7F)
#define INT1_THRESHOLD_MASK	(0x7F)



/* RESUME STATE INDICES */
#define RES_CTRL1		0
#define RES_CTRL2		1
#define RES_CTRL3		2
#define RES_CTRL4		3
#define RES_CTRL5		4
#define RES_CTRL6		5
#define RES_CTRL7		6

#define RES_INT_CFG1		7
#define RES_INT_THSX1		8
#define RES_INT_THSY1		9
#define RES_INT_THSZ1		10
#define RES_INT_DUR1		11


#define RES_INT_CFG2		12
#define RES_INT_THS2		13
#define RES_INT_DUR2		14

#define RES_TEMP_CFG_REG	15
#define RES_REFERENCE_REG	16
#define RES_FIFO_CTRL	17

#define RESUME_ENTRIES		18
/* end RESUME STATE INDICES */

#define OUTPUT_ALWAYS_ANTI_ALIASED 1

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} k2hh_acc_odr_table[] = {
		{    2, ACC_ODR800 },
		{    3, ACC_ODR400  },
		{    5, ACC_ODR200  },
		{   10, ACC_ODR100  },
#if(!OUTPUT_ALWAYS_ANTI_ALIASED)
		{   20, ACC_ODR50   },
		{  100, ACC_ODR10   },
#endif
};

static int int1_gpio = K2HH_ACC_DEFAULT_INT1_GPIO;
#ifdef USE_ACC_IRQ2
static int int2_gpio = K2HH_ACC_DEFAULT_INT2_GPIO;
#endif
module_param(int1_gpio, int, S_IRUGO);
#ifdef USE_ACC_IRQ2
module_param(int2_gpio, int, S_IRUGO);
#endif
/* k2hh acceleration data */
struct k2hh_acc {
	s16 x;
	s16 y;
	s16 z;
	int bCalLoaded;
};

struct k2hh_acc_data {
	struct i2c_client *client;
	struct k2hh_acc_platform_data *pdata;

	struct mutex lock;
	struct work_struct input_poll_work;
	struct hrtimer hr_timer_poll;
	ktime_t polling_ktime;
	struct workqueue_struct *hr_timer_poll_work_queue;

	struct input_dev *input_dev;
#ifdef K2HH_ACCEL_CALIBRATION
	struct k2hh_acc cal_data;
#endif
	int hw_initialized;
	atomic_t selftest_rslt;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
#ifdef USE_ACC_IRQ2
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
#endif

#ifdef K2HH_ACCEL_CALIBRATION
	atomic_t fast_calib_x_rslt;
	atomic_t fast_calib_y_rslt;
	atomic_t fast_calib_z_rslt;
	atomic_t fast_calib_rslt;
#endif

#ifdef DEBUG
	u8 reg_addr;
#endif
};

static struct k2hh_acc_platform_data default_k2hh_acc_pdata = {
	.fs_range = K2HH_ACC_FS_4G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 1,
	.poll_interval = 100,
	.min_interval = K2HH_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = K2HH_ACC_DEFAULT_INT1_GPIO,
#ifdef USE_ACC_IRQ2
	.gpio_int2 = K2HH_ACC_DEFAULT_INT2_GPIO,
#endif
};

/* sets default init values to be written in registers at probe stage */
static void k2hh_acc_set_init_register_values(struct k2hh_acc_data *acc)
{

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	acc->resume_state[RES_CTRL1] = (ALL_ZEROES | CTRL1_HR_MASK | CTRL1_BDU_ENABLE | ACC_ENABLE_ALL_AXES);
	/* Low-pass cutoff frequency in high resolution mode */
	acc->resume_state[RES_CTRL2] = ALL_ZEROES;

	if(acc->pdata->gpio_int1 >= 0)
		acc->resume_state[RES_CTRL3] = (acc->resume_state[RES_CTRL3] | CTRL3_IG1_INT1);

	acc->resume_state[RES_CTRL4] = (ALL_ZEROES | CTRL4_IF_ADD_INC_EN);

	acc->resume_state[RES_CTRL5] = (ALL_ZEROES | CTRL5_HLACTIVE_H);
#ifdef USE_ACC_IRQ2
	if(acc->pdata->gpio_int2 >= 0)
		acc->resume_state[RES_CTRL6] =(acc->resume_state[RES_CTRL6] | CTRL6_IG2_INT2);
#endif
	acc->resume_state[RES_CTRL7] = (ALL_ZEROES | CTRL7_LIR2 | CTRL7_LIR1);

}

static int k2hh_acc_i2c_read(struct k2hh_acc_data *acc, u8 *buf,
									int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;
#ifdef DEBUG
	unsigned int ii;
#endif

	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (acc->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(acc->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&acc->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(acc->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&acc->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&acc->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(acc->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(acc->client, buf, len);
}

static int k2hh_acc_i2c_write(struct k2hh_acc_data *acc, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;
#ifdef DEBUG
	unsigned int ii;
#endif
	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (acc->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(acc->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&acc->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(acc->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&acc->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(acc->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int k2hh_acc_hw_init(struct k2hh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", K2HH_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = k2hh_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
		dev_warn(&acc->client->dev, "Error reading WHO_AM_I:"
				" is device available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_K2HH_ACC) {
		dev_err(&acc->client->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_K2HH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = FIFO_CTRL;
	buf[1] = acc->resume_state[RES_FIFO_CTRL];
	err = k2hh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THSX1;
	buf[1] = acc->resume_state[RES_INT_THSX1];
	buf[2] = acc->resume_state[RES_INT_THSY1];
	buf[3] = acc->resume_state[RES_INT_THSZ1];
	buf[4] = acc->resume_state[RES_INT_DUR1];
	err = k2hh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = k2hh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = CTRL2;
	buf[1] = acc->resume_state[RES_CTRL2];
	buf[2] = acc->resume_state[RES_CTRL3];
	buf[3] = acc->resume_state[RES_CTRL4];
	buf[4] = acc->resume_state[RES_CTRL5];
	buf[5] = acc->resume_state[RES_CTRL6];
	buf[6] = acc->resume_state[RES_CTRL7];
	err = k2hh_acc_i2c_write(acc, buf, 6);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL1;
	buf[1] = acc->resume_state[RES_CTRL1];
	err = k2hh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", K2HH_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void k2hh_acc_device_power_off(struct k2hh_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL1, K2HH_ACC_PM_OFF };

	err = k2hh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if (acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
#ifdef USE_ACC_IRQ2
		if (acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
#endif
		acc->pdata->power_off(acc->pdata);
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if (acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
#ifdef USE_ACC_IRQ2
		if (acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
#endif
		acc->hw_initialized = 0;
	}

		dev_err(&acc->client->dev, "power off\n");
}

static int k2hh_acc_device_power_on(struct k2hh_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on(acc->pdata);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
#ifdef USE_ACC_IRQ2
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
#endif
	}

	if (!acc->hw_initialized) {
		err = k2hh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			k2hh_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		/* for Warning Message Remove */
		//if (acc->pdata->gpio_int1 >= 0)
		//	enable_irq(acc->irq1);

#ifdef USE_ACC_IRQ2
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
#endif
	}

	
	return 0;
}


static int k2hh_acc_update_fs_range(struct k2hh_acc_data *acc,
							u8 new_fs_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = K2HH_ACC_FS_MASK;

	switch (new_fs_range) {
	case K2HH_ACC_FS_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case K2HH_ACC_FS_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case K2HH_ACC_FS_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	buf[0] = CTRL4;
	err = k2hh_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	acc->resume_state[RES_CTRL4] = init_val;
	new_val = new_fs_range;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL4;
	err = k2hh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error;
	acc->resume_state[RES_CTRL4] = updated_val;
	acc->sensitivity = sensitivity;

	return err;
error:
	dev_err(&acc->client->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int k2hh_acc_update_odr(struct k2hh_acc_data *acc,
							int poll_interval_ms)
{
	int err;
	int i;
	u8 config[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = ACC_ODR_MASK;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(k2hh_acc_odr_table) - 1; i >= 0; i--) {
		if ((k2hh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}
	new_val = k2hh_acc_odr_table[i].mask;

	/* Updates configuration register 1,
	* which contains odr range setting if enabled,
	* otherwise updates RES_CTRL1 for when it will */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL1;
		err = k2hh_acc_i2c_read(acc, config, 1);
		if (err < 0)
			goto error;
		init_val = config[0];
		acc->resume_state[RES_CTRL1] = init_val;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		config[1] = updated_val;
		config[0] = CTRL1;
		err = k2hh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL1] = updated_val;
		return err;
	} else {
		init_val = acc->resume_state[RES_CTRL1];
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		acc->resume_state[RES_CTRL1] = updated_val;
		return 0;
	}

error:
	dev_err(&acc->client->dev,
			"update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int k2hh_acc_register_write(struct k2hh_acc_data *acc,
					u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = k2hh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	return err;
}

/*
static int k2hh_acc_get_data(
				struct k2hh_acc_data *acc, int *xyz)
{
	int err = -1;
	// Data bytes from hardware xL, xH, yL, yH, zL, zH
	u8 acc_data[6];
	// x,y,z hardware data
	s32 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = k2hh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG

	dev_dbg(&acc->client->dev,"%s read x=%d, y=%d, z=%d\n",
			K2HH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

#endif
	return err;
}
*/

static int k2hh_acc_get_acceleration_raw_data(struct k2hh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	// Data bytes from hardware xL, xH, yL, yH, zL, zH
	u8 acc_data[6];
	// x,y,z hardware data
	s32 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);

	mutex_lock(&acc->lock);
	err = k2hh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0) {
		mutex_unlock(&acc->lock);
		return err;
	}
	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = (hw_d[0] * acc->sensitivity) / 1000;
	hw_d[1] = (hw_d[1] * acc->sensitivity) / 1000;
	hw_d[2] = (hw_d[2] * acc->sensitivity) / 1000;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));
	mutex_unlock(&acc->lock);
	#if DEBUG
	//	pr_info("%s read raw x=%d, y=%d, z=%d\n",
	//		K2HH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif
	return err;
}

static int k2hh_acc_get_acceleration_data(struct k2hh_acc_data *acc,
		int *xyz)
{

	int err = -1;

	err = k2hh_acc_get_acceleration_raw_data(acc,xyz);
	if (err < 0) {
		pr_info("k2hh_read_accel_xyz() failed \n");
		return err;
	}

	xyz[0] -= acc->cal_data.x;
	xyz[1] -= acc->cal_data.y;
	xyz[2] -= acc->cal_data.z;

	#if DEBUG
	//	pr_info("%s read calibrated x=%d, y=%d, z=%d\n",
	//		K2HH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif

	return err;
}

static void k2hh_acc_report_values(struct k2hh_acc_data *acc,
					int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static void k2hh_acc_report_triple(struct k2hh_acc_data *acc)
{
	int err;
	int xyz[3];

	err = k2hh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_data failed\n");
	else
		k2hh_acc_report_values(acc, xyz);
}
#ifdef K2HH_ACCEL_CALIBRATION
static int k2hh_read_Calibration_data(struct i2c_client *client)
{
	int fd_offset_x, fd_offset_y, fd_offset_z;
	char offset_x_src[5],offset_y_src[5],offset_z_src[5];
	long offset_x,offset_y,offset_z;

	struct k2hh_acc_data *k2hh = i2c_get_clientdata(client);

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	memset(offset_x_src,0x00,sizeof(offset_x_src));
	memset(offset_y_src,0x00,sizeof(offset_y_src));
	memset(offset_z_src,0x00,sizeof(offset_z_src));

	fd_offset_x = sys_open("/sns/offset_x.dat",O_RDONLY, 0);
	fd_offset_y = sys_open("/sns/offset_y.dat",O_RDONLY, 0);
	fd_offset_z = sys_open("/sns/offset_z.dat",O_RDONLY, 0);

	if ((fd_offset_x < 0) || (fd_offset_y < 0) || (fd_offset_z < 0))
		return -EINVAL;

	if ((sys_read(fd_offset_x, offset_x_src, sizeof(offset_x_src)) < 0)
		||	(sys_read(fd_offset_y ,offset_y_src, sizeof(offset_y_src)) <0)
		||	(sys_read(fd_offset_z, offset_z_src, sizeof(offset_z_src)) <0 ))
		return -EINVAL;

	if ((strict_strtol(offset_x_src, 10, &offset_x))
		||	(strict_strtol(offset_y_src, 10, &offset_y))
		||	(strict_strtol(offset_z_src, 10, &offset_z)))
		return -EINVAL;

	if (offset_x > 1000 || offset_x < -1000
		|| offset_y > 1000 || offset_y < -1000
		|| offset_z > 1000 || offset_z < -1000 ){
		dev_err(&client->dev,"Abnormal Calibration Data");
		offset_x = 0;
		offset_y = 0;
		offset_z = 0;
	}

	k2hh->cal_data.x = offset_x;
	k2hh->cal_data.y = offset_y;
	k2hh->cal_data.z = offset_z;

	sys_close(fd_offset_x);
	sys_close(fd_offset_y);
	sys_close(fd_offset_z);
	set_fs(old_fs);

	return 0;
}
#endif

static irqreturn_t k2hh_acc_isr1(int irq, void *dev)
{
	struct k2hh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	pr_debug("%s: isr1 queued\n", K2HH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}
#ifdef USE_ACC_IRQ2
static irqreturn_t k2hh_acc_isr2(int irq, void *dev)
{
	struct k2hh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	pr_debug("%s: isr2 queued\n", K2HH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}
#endif
static void k2hh_acc_irq1_work_func(struct work_struct *work)
{

	struct k2hh_acc_data *acc =
	container_of(work, struct k2hh_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:k2hh_acc_get_int1_source(acc); */
	/* ; */
	pr_debug("%s: IRQ1 served\n", K2HH_ACC_DEV_NAME);
/* exit: */
	enable_irq(acc->irq1);
}
#ifdef USE_ACC_IRQ2
static void k2hh_acc_irq2_work_func(struct work_struct *work)
{

	struct k2hh_acc_data *acc =
	container_of(work, struct k2hh_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:k2hh_acc_get_tap_source(acc); */
	/* ; */
	pr_debug("%s: IRQ2 served\n", K2HH_ACC_DEV_NAME);
/* exit: */
	enable_irq(acc->irq2);
}
#endif
static int k2hh_acc_enable(struct k2hh_acc_data *acc)
{
	int err;
	pr_info("K2HH cal_data.bCalLoaded = %d\n",acc->cal_data.bCalLoaded);
    if( !(acc->cal_data.bCalLoaded ) ) {
		err = k2hh_read_Calibration_data(acc->client);
		if (err)
			pr_info("k2hh Calibration data is not found\n");
		else
			acc->cal_data.bCalLoaded = 1;
	}
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = k2hh_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		acc->polling_ktime = ktime_set(acc->pdata->poll_interval / 1000,
				MS_TO_NS(acc->pdata->poll_interval % 1000));
		hrtimer_start(&acc->hr_timer_poll,
					acc->polling_ktime, HRTIMER_MODE_REL);
	}

	// Setting scale range : +-/ 4g
	mutex_lock(&acc->lock);
	err = k2hh_acc_update_fs_range(acc, K2HH_ACC_FS_4G);
	if (err < 0) {
		mutex_unlock(&acc->lock);
		return err;
	}
	acc->pdata->fs_range = K2HH_ACC_FS_4G;
	mutex_unlock(&acc->lock);
	dev_info(&acc->client->dev, "scale range set to: 0x%x (0x00 = 2G, 0x20 = 4G, 0x30 = 8G)\n", K2HH_ACC_FS_4G);
	
	return 0;
}

static int k2hh_acc_disable(struct k2hh_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_work_sync(&acc->input_poll_work);
		k2hh_acc_device_power_off(acc);
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = k2hh_acc_i2c_read(acc, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = k2hh_acc_register_write(acc, x, reg, new_val);
	if (err < 0)
		return err;
	acc->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_set_flush(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);

	input_report_rel(acc->input_dev, REL_MISC, 0x96);
	input_sync(acc->input_dev);

	pr_info("attr_set_flush() \n");
	return size;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms, acc->pdata->min_interval);
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	err = k2hh_acc_update_odr(acc, interval_ms);
	if(err >= 0) {
		acc->pdata->poll_interval = interval_ms;
		acc->polling_ktime = ktime_set(acc->pdata->poll_interval / 1000,
				MS_TO_NS(acc->pdata->poll_interval % 1000));
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch (val) {
	case K2HH_ACC_FS_2G:
		range = 2;
		break;
	case K2HH_ACC_FS_4G:
		range = 4;
		break;
	case K2HH_ACC_FS_8G:
		range = 8;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = K2HH_ACC_FS_2G;
		break;
	case 4:
		range = K2HH_ACC_FS_4G;
		break;
	case 8:
		range = K2HH_ACC_FS_8G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&acc->lock);
	err = k2hh_acc_update_fs_range(acc, range);
	if (err < 0) {
		mutex_unlock(&acc->lock);
		return err;
	}
	acc->pdata->fs_range = range;
	mutex_unlock(&acc->lock);
	dev_info(&acc->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		k2hh_acc_enable(acc);
	else
		k2hh_acc_disable(acc);

	return size;
}

static ssize_t attr_get_raw_data(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	int data[3] = {0, 0, 0};

	if (val > 0) {
		k2hh_acc_get_acceleration_raw_data(acc, data);
	}

	return sprintf(buf, "data = %d, %d, %d\n", data[0], data[1], data[2]);
}

static ssize_t attr_get_sensor_cal_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int xyz[3] = { 0, };
	int err = 0;
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);
	err = k2hh_acc_get_acceleration_data(k2hh,xyz);
	if (err < 0)
		return sprintf(buf, "Read Calibrated Data failed");
	return sprintf(buf, "Read Calibrated Data x=%d y=%d z=%d \n", xyz[K2HH_AXIS_X], xyz[K2HH_AXIS_Y], xyz[K2HH_AXIS_Z]);
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_threshx1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSX1, INT1_THRESHOLD_MASK, RES_INT_THSX1);
}

static ssize_t attr_get_threshx1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSX1);
}

static ssize_t attr_set_threshy1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSY1, INT1_THRESHOLD_MASK, RES_INT_THSY1);
}

static ssize_t attr_get_threshy1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSY1);
}

static ssize_t attr_set_threshz1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSZ1, INT1_THRESHOLD_MASK, RES_INT_THSZ1);
}

static ssize_t attr_get_threshz1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSZ1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}


#define SELF_TEST_2G_MAX_LSB	(24576)
#define SELF_TEST_2G_MIN_LSB	(1146)

static int k2hh_acc_get_selftest(struct k2hh_acc_data *acc, char *buf)
{
	int val, i, en_state = 0;

	u8 x[8];
	s32 NO_ST[3] = {0, 0, 0};
	s32 ST[3] = {0, 0, 0};
	
	en_state = atomic_read(&acc->enabled);
	k2hh_acc_disable(acc);

	k2hh_acc_device_power_on(acc);

	x[0] = CTRL1;
	x[1] = 0x3f;
	k2hh_acc_i2c_write(acc, x, 1);
	x[0] = CTRL4;
	x[1] = 0x04;
	x[2] = 0x00;
	x[3] = 0x00;
	k2hh_acc_i2c_write(acc, x, 3);

	mdelay(80);

	x[0] = AXISDATA_REG;
	k2hh_acc_i2c_read(acc, x, 6);

	for (i = 0; i < 5; i++) {
		while (1) {
			x[0] = 0x27;
			val = k2hh_acc_i2c_read(acc, x, 1);
			if (val < 0) {
				pr_info("K2HH I2C fail. (%d)\n", val);
				goto ST_EXIT;
			}
			if (x[0] & 0x08)
				break;
		}
		x[0] = AXISDATA_REG;
		k2hh_acc_i2c_read(acc, x, 6);
		NO_ST[0] += (s16)(x[1] << 8 | x[0]);
		NO_ST[1] += (s16)(x[3] << 8 | x[2]);
		NO_ST[2] += (s16)(x[5] << 8 | x[4]);
	}
	NO_ST[0] /= 5;
	NO_ST[1] /= 5;
	NO_ST[2] /= 5;

	x[0] = CTRL5;
	x[1] = 0x04;
	k2hh_acc_i2c_write(acc, x, 1);

	mdelay(80);

	x[0] = AXISDATA_REG;
	k2hh_acc_i2c_read(acc, x, 6);	

	for (i = 0; i < 5; i++) {
		while (1) {
			x[0] = 0x27;
			val = k2hh_acc_i2c_read(acc, x, 1);
			if (val < 0) {
				pr_info("K2HH I2C fail. (%d)\n", val);
				goto ST_EXIT;
			}
			if (x[0] & 0x08)
				break;
		}
		x[0] = AXISDATA_REG;
		k2hh_acc_i2c_read(acc, x, 6);
		ST[0] += (s16)(x[1] << 8 | x[0]);
		ST[1] += (s16)(x[3] << 8 | x[2]);
		ST[2] += (s16)(x[5] << 8 | x[4]);
	}
	ST[0] /= 5;
	ST[1] /= 5;
	ST[2] /= 5;

	for (val = 1, i = 0; i < 3; i++) {
		ST[i] -= NO_ST[i];
		ST[i] = abs(ST[i]);

		if ((SELF_TEST_2G_MIN_LSB > ST[i]) || (ST[i] > SELF_TEST_2G_MAX_LSB)) {
			pr_info("ST[%d]: Out of range!! (%d)\n", i, ST[i]);
			val = 0;
		}
	}

	if (val)
		pr_info("K2HH Self test: OK (%d, %d, %d)\n", ST[0], ST[1], ST[2]);
	else
		pr_info("K2HH Self test: NG (%d, %d, %d)\n", ST[0], ST[1], ST[2]);

ST_EXIT:
	x[0] = CTRL1;
	x[1] = 0x00;
	k2hh_acc_i2c_write(acc, x, 1);
	x[0] = CTRL5;
	x[1] = 0x00;
	k2hh_acc_i2c_write(acc, x, 1);

	k2hh_acc_device_power_off(acc);

	if (en_state) k2hh_acc_enable(acc);

	return val;
}

static ssize_t attr_get_selftest(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	return sprintf(buf,"%s\n",atomic_read(&k2hh->selftest_rslt) ? "Pass" : "Fail");
}

static ssize_t attr_set_selftest(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long data = 0;
	int error = -1;
	u8 buf_error[50];
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if(data == 1) {
		if(k2hh_acc_get_selftest(k2hh,buf_error))
			atomic_set(&k2hh->selftest_rslt, 1);
		else
			atomic_set(&k2hh->selftest_rslt, 0);
	}
	else {
		pr_info("Selftest is failed\n");
		return -EINVAL;
	}

	return size;
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = k2hh_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = k2hh_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct k2hh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static void k2hh_acc_input_poll_work_func(struct work_struct *work)
{
	struct k2hh_acc_data *acc;

	acc = container_of((struct work_struct *) work,
			struct k2hh_acc_data, input_poll_work);

	//mutex_lock(&acc->lock);
	k2hh_acc_report_triple(acc);
	//mutex_unlock(&acc->lock);

	if (atomic_read(&acc->enabled))
		hrtimer_start(&acc->hr_timer_poll, acc->polling_ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart k2hh_acc_hr_timer_poll_function(struct hrtimer *timer)
{
	struct k2hh_acc_data *acc;

	acc = container_of((struct hrtimer *)timer,
				struct k2hh_acc_data, hr_timer_poll);

	queue_work(acc->hr_timer_poll_work_queue, &acc->input_poll_work);
	return HRTIMER_NORESTART;
}

int k2hh_acc_input_open(struct input_dev *input)
{
	struct k2hh_acc_data *acc = input_get_drvdata(input);
	dev_dbg(&acc->client->dev, "%s\n", __func__);
	return 0;
}

void k2hh_acc_input_close(struct input_dev *dev)
{
	struct k2hh_acc_data *acc = input_get_drvdata(dev);
	dev_dbg(&acc->client->dev, "%s\n", __func__);
}

static int k2hh_acc_validate_pdata(struct k2hh_acc_data *acc)
{
	/* checks for correctness of minimal polling period */
	acc->pdata->min_interval =
		max((unsigned int)K2HH_ACC_MIN_POLL_PERIOD_MS,
						acc->pdata->min_interval);

	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z:%u\n", acc->pdata->axis_map_x,
					acc->pdata->axis_map_y,
						acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

/* power on/off function*/
int k2hh_power(struct k2hh_acc_platform_data *pdata, int on)
{
	int ret;

	if (on) {
		ret = regulator_enable(pdata->vdd_supply);
		ret += regulator_enable(pdata->vcc_i2c_supply);
		pr_info("k2hh power on [status : %d ]\n", ret);
		mdelay(20);
	} else {
		ret = regulator_disable(pdata->vdd_supply);
		ret += regulator_disable(pdata->vcc_i2c_supply);
		pr_info("k2hh power off [status : %d ]\n", ret);
	}
	return ret;
}

int k2hh_power_on(struct k2hh_acc_platform_data *pdata)
{
	int ret;
	ret =  k2hh_power(pdata, 1);
	return ret;
}

int k2hh_power_off(struct k2hh_acc_platform_data *pdata)
{
	int ret;
	ret =  k2hh_power(pdata, 0);
	return ret;
}

static int k2hh_gpio_config(struct k2hh_acc_data *data)
{
	int ret;
	/* configure irq gpio */
	ret = gpio_request(data->pdata->gpio_int1, "k2hh_irq_gpio");
	if (ret) {
		dev_err(&data->client->dev,
				"unable to request gpio [%d]\n",
				data->pdata->gpio_int1);
	}
	ret = gpio_direction_input(data->pdata->gpio_int1);
	if (ret) {
		dev_err(&data->client->dev,
				"unable to set direction for gpio [%d]\n",
				data->pdata->gpio_int1);
	}
	return ret;
}

/* LGE Add start DTS parse function */
#ifdef CONFIG_OF
static int k2hh_parse_dt(struct device *dev, struct k2hh_acc_platform_data *pdata)
{
	int rc;
	int ret, err = 0;
	struct device_node *np = dev->of_node;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		{"st,gpio-int",     &pdata->gpio_int1,      DT_REQUIRED,        DT_GPIO,    0},
#ifdef USE_ACC_IRQ2
		{"st_gpio_int2",    &pdata->gpio_int2,      DT_REQUIRED,        DT_GPIO,    0},
#endif
		{"fs_range",         &pdata->fs_range,        DT_SUGGESTED,       DT_U32,     0},
		{"axis_map_x",      &pdata->axis_map_x,     DT_SUGGESTED,       DT_U32,     0},
		{"axis_map_y",      &pdata->axis_map_y,     DT_SUGGESTED,       DT_U32,     0},
		{"axis_map_z",      &pdata->axis_map_z,     DT_SUGGESTED,       DT_U32,     0},
		{"negate_x",        &pdata->negate_x,       DT_SUGGESTED,       DT_U32,     0},
		{"negate_y",        &pdata->negate_y,       DT_SUGGESTED,       DT_U32,     0},
		{"negate_z",        &pdata->negate_z,       DT_SUGGESTED,       DT_U32,     0},
		{"poll_interval",   &pdata->poll_interval,  DT_SUGGESTED,       DT_U32,     0},
		{"min_interval",    &pdata->min_interval,   DT_SUGGESTED,       DT_U32,     0},
		{NULL,              NULL,                   0,                  0,          0},
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
				ret = of_property_read_u32(np, itr->dt_name,
						(u32 *) itr->ptr_data);
				break;
			case DT_BOOL:
				*((bool *) itr->ptr_data) =
					of_property_read_bool(np, itr->dt_name);
				ret = 0;
				break;
			default:
				dev_err(dev,"%d is an unknown DT entry type\n",itr->type);
				ret = -EBADE;
		}
		dev_info(dev,"DT entry ret:%d name:%s val:%d\n",
				ret,itr->dt_name, *((int *)itr->ptr_data));
		
		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;
			if (itr->status < DT_OPTIONAL) {
				dev_err(dev,"Missing '%s' DT entry\n",itr->dt_name);
				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}
	
	pdata->vdd_supply = regulator_get(dev, "stm,sensor_vdd");
	if (IS_ERR(pdata->vdd_supply)) {
		rc = PTR_ERR(pdata->vdd_supply);
		dev_err(dev, "Regulator get failed sensor_vdd-supply rc=%d\n", rc);
		return rc;
	}
	pdata->vcc_i2c_supply = regulator_get(dev, "stm,sensor_vcc_i2c");
	if (IS_ERR(pdata->vcc_i2c_supply)) {
		rc = PTR_ERR(pdata->vcc_i2c_supply);
		dev_err(dev, "Regulator get failed vcc_i2c_supply rc=%d\n", rc);
		return rc;
	}
	/* debug print disable */
	/*
	   dev_info(dev, "parse_dt data [gpio_int = %d]\n", pdata->gpio_int);
	 */
return 0;
}
#else
static int k2hh_parse_dt(struct device *dev, struct k2hh_acc_platform_data *pdata)
{
	    return -ENODEV;
}
#endif
/*LGE Add end */

static int k2hh_store_Calibration_data(struct device *dev)
{
	unsigned char offset_x_src[5],offset_y_src[5],offset_z_src[5];
	int fd_offset_x, fd_offset_y, fd_offset_z;
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	memset(offset_x_src,0x00,sizeof(offset_x_src));
	memset(offset_y_src,0x00,sizeof(offset_y_src));
	memset(offset_z_src,0x00,sizeof(offset_z_src));

	fd_offset_x = sys_open("/sns/offset_x.dat",O_WRONLY|O_CREAT, 0666);
	fd_offset_y = sys_open("/sns/offset_y.dat",O_WRONLY|O_CREAT, 0666);
	fd_offset_z = sys_open("/sns/offset_z.dat",O_WRONLY|O_CREAT, 0666);

#ifdef DEBUG
	dev_info(dev, "[fd_offset_x] writing %d", fd_offset_x);
	dev_info(dev, "[fd_offset_y] writing %d", fd_offset_y );
	dev_info(dev, "[fd_offset_z] writing %d", fd_offset_z);
#endif

	if ((fd_offset_x < 0) || (fd_offset_y < 0) || (fd_offset_z < 0))
		return -EINVAL;

	sprintf(offset_x_src, "%d", k2hh->cal_data.x);
	sprintf(offset_y_src, "%d", k2hh->cal_data.y);
	sprintf(offset_z_src, "%d", k2hh->cal_data.z);

#ifdef DEBUG
	dev_info(dev, "[offset_x] writing %s", offset_x_src);
	dev_info(dev, "[offset_y] writing %s", offset_y_src);
	dev_info(dev, "[offset_z] writing %s", offset_z_src);
#endif

	if ((sys_write(fd_offset_x, offset_x_src, sizeof(offset_x_src)) < 0)
			||	(sys_write(fd_offset_y, offset_y_src,  sizeof(offset_y_src)) < 0)
				||	(sys_write(fd_offset_z, offset_z_src,  sizeof(offset_z_src)) < 0))
		return -EINVAL;

	atomic_set(&k2hh->fast_calib_rslt, SUCCESS);

	sys_fsync(fd_offset_x);
	sys_fsync(fd_offset_y);
	sys_fsync(fd_offset_z);

	sys_close(fd_offset_x);
	sys_close(fd_offset_y);
	sys_close(fd_offset_z);
        sys_chmod("/sns/offset_x.dat", 0664);
        sys_chmod("/sns/offset_y.dat", 0664);
        sys_chmod("/sns/offset_z.dat", 0664);
	set_fs(old_fs);

#ifdef DEBUG
	dev_info(dev, "[CLOSE FILE]!!!!! ");
#endif

	return 0;
}

static int k2hh_do_calibration(struct device *dev, char *axis)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);
	unsigned int timeout_shaking = 0;
	int acc_cal_pre[3] = { 0, };
	int acc_cal[3] = { 0, };
	int sum[3] = { 0, };

	k2hh_acc_get_acceleration_raw_data(k2hh, acc_cal_pre);
	do{
		mdelay(20);
		k2hh_acc_get_acceleration_raw_data(k2hh, acc_cal);
		dev_info(dev, "===============moved %s =============== timeout = %d", axis, timeout_shaking);
		dev_info(dev, "(%d, %d, %d) (%d, %d, %d)", acc_cal_pre[K2HH_AXIS_X], acc_cal_pre[K2HH_AXIS_Y], acc_cal_pre[K2HH_AXIS_Z], acc_cal[K2HH_AXIS_X], acc_cal[K2HH_AXIS_Y], acc_cal[K2HH_AXIS_Z]);

		if((abs(acc_cal[K2HH_AXIS_X] - acc_cal_pre[K2HH_AXIS_X]) > K2HH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K2HH_AXIS_Y] - acc_cal_pre[K2HH_AXIS_Y])) > K2HH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K2HH_AXIS_Z] - acc_cal_pre[K2HH_AXIS_Z])) > K2HH_SHAKING_DETECT_THRESHOLD)){
				atomic_set(&k2hh->fast_calib_rslt, FAIL);
				dev_info(dev, "===============shaking %s ===============", axis);
				return -EINVAL;
		}
		else {
			sum[K2HH_AXIS_X] += acc_cal[K2HH_AXIS_X];
			sum[K2HH_AXIS_Y] += acc_cal[K2HH_AXIS_Y];
			sum[K2HH_AXIS_Z] += acc_cal[K2HH_AXIS_Z];

			acc_cal_pre[K2HH_AXIS_X] = acc_cal[K2HH_AXIS_X];
			acc_cal_pre[K2HH_AXIS_Y] = acc_cal[K2HH_AXIS_Y];
			acc_cal_pre[K2HH_AXIS_Z] = acc_cal[K2HH_AXIS_Z];
		}
			timeout_shaking++;
	}while(timeout_shaking < CALIBRATION_DATA_AMOUNT);
	dev_info(dev, "===============complete shaking %s check===============", axis);

	/* check zero-g offset */
	if ((abs(sum[K2HH_AXIS_X]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		(abs(sum[K2HH_AXIS_Y]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		 ((abs(sum[K2HH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB) || (abs(sum[K2HH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB))) {
			dev_err(dev, "Calibration zero-g offset check failed (%d, %d, %d)\n",
					sum[K2HH_AXIS_X]/CALIBRATION_DATA_AMOUNT, sum[K2HH_AXIS_Y]/CALIBRATION_DATA_AMOUNT, sum[K2HH_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
			atomic_set(&k2hh->fast_calib_rslt, FLAT_FAIL);

			return -EINVAL;
	}

	k2hh->cal_data.x = sum[0] / CALIBRATION_DATA_AMOUNT;  /* k2hh(12bit) 0+-154 */
	k2hh->cal_data.y = sum[1] / CALIBRATION_DATA_AMOUNT;  /* k2hh(12bit) 0+-154 */

	if (sum[2] >= 0) {
		k2hh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 1024; /* k2hh(12bit) 1024 +-226 */
	} else {
		k2hh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 1024; /* k2hh(12bit) 1024 +-226 */
	}

	dev_info(dev, "=============== %s fast calibration finished===============", axis);

	return 0;
}

static ssize_t attr_get_fast_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k2hh->fast_calib_rslt));
}

static ssize_t attr_set_fast_calibration(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);
	unsigned int timeout_shaking = 0;
	unsigned long data;
	int acc_cal_pre[3] = { 0, };
	int acc_cal[3] = { 0, };
	int sum[3] = { 0, };
	int error = 0;

	atomic_set(&k2hh->fast_calib_rslt, FAIL);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	k2hh_acc_get_acceleration_raw_data(k2hh, acc_cal_pre);
	do{
		mdelay(20);
		k2hh_acc_get_acceleration_raw_data(k2hh, acc_cal);
		dev_info(dev, "===============moved x,y,z=============== timeout = %d",timeout_shaking);
		dev_info(dev, "(%d, %d, %d) (%d, %d, %d)", acc_cal_pre[K2HH_AXIS_X], acc_cal_pre[K2HH_AXIS_Y], acc_cal_pre[K2HH_AXIS_Z], acc_cal[K2HH_AXIS_X], acc_cal[K2HH_AXIS_Y], acc_cal[K2HH_AXIS_Z]);

		if((abs(acc_cal[K2HH_AXIS_X] - acc_cal_pre[K2HH_AXIS_X]) > K2HH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K2HH_AXIS_Y] - acc_cal_pre[K2HH_AXIS_Y])) > K2HH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K2HH_AXIS_Z] - acc_cal_pre[K2HH_AXIS_Z])) > K2HH_SHAKING_DETECT_THRESHOLD)){
				atomic_set(&k2hh->fast_calib_rslt, FAIL);
				dev_info(dev, "===============shaking x,y,z===============");
				return -EINVAL;
		} else {
			sum[K2HH_AXIS_X] += acc_cal[K2HH_AXIS_X];
			sum[K2HH_AXIS_Y] += acc_cal[K2HH_AXIS_Y];
			sum[K2HH_AXIS_Z] += acc_cal[K2HH_AXIS_Z];

			acc_cal_pre[K2HH_AXIS_X] = acc_cal[K2HH_AXIS_X];
			acc_cal_pre[K2HH_AXIS_Y] = acc_cal[K2HH_AXIS_Y];
			acc_cal_pre[K2HH_AXIS_Z] = acc_cal[K2HH_AXIS_Z];
		}
			timeout_shaking++;
	}while(timeout_shaking < CALIBRATION_DATA_AMOUNT);
	dev_info(dev, "===============complete shaking x,y,z check===============");

	/* check zero-g offset */
	if ((abs(sum[K2HH_AXIS_X]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		(abs(sum[K2HH_AXIS_Y]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		 ((abs(sum[K2HH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB) || (abs(sum[K2HH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB))) {
			dev_err(dev, "Calibration zero-g offset check failed (%d, %d, %d)\n",
					sum[K2HH_AXIS_X]/CALIBRATION_DATA_AMOUNT, sum[K2HH_AXIS_Y]/CALIBRATION_DATA_AMOUNT, sum[K2HH_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
			atomic_set(&k2hh->fast_calib_rslt, FLAT_FAIL);

			return -EINVAL;
	}

	k2hh->cal_data.x = sum[0] / CALIBRATION_DATA_AMOUNT;  /* k2hh(12bit) 0+-154 */
	k2hh->cal_data.y = sum[1] / CALIBRATION_DATA_AMOUNT;  /* k2hh(12bit) 0+-154 */

	if (sum[2] >= 0) {
		k2hh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 1024; /* k2hh(12bit) 1024 +-226 */
	} else {
		k2hh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 1024; /* k2hh(12bit) 1024 +-226 */
	}

	dev_info(dev, "===============x,y,z fast calibration finished===============");


	error = k2hh_store_Calibration_data(dev);
	if (error) {
		dev_err(dev,"k2hh_fast_calibration_store failed");
		return error;
	}
	atomic_set(&k2hh->fast_calib_rslt, SUCCESS);

	return size;

}

static ssize_t attr_get_eeprom_writing(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k2hh->fast_calib_x_rslt));
}

static ssize_t attr_set_eeprom_writing(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error = 0;
	error = k2hh_store_Calibration_data(dev);
	if (error)
		return error;

	return count;
}

static ssize_t attr_get_fast_calibration_x(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k2hh->fast_calib_x_rslt));
}

static ssize_t attr_set_fast_calibration_x(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	char axis = 'x';
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	atomic_set(&k2hh->fast_calib_rslt, FAIL);

	error = k2hh_do_calibration(dev,&axis);
	if (error) {
		dev_err(dev,"[k2hh_fast_calibration_x_store] do_calibration failed");
		return error;
	}
	error = k2hh_store_Calibration_data(dev);
	if (error) {
		dev_err(dev,"[k2hh_fast_calibration_x_store] store_Calibration_data failed");
		return error;
	}
	atomic_set(&k2hh->fast_calib_x_rslt, SUCCESS);
	return size;
}

static ssize_t attr_get_fast_calibration_y(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k2hh->fast_calib_y_rslt));
}

static ssize_t attr_set_fast_calibration_y(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	char axis = 'y';
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	atomic_set(&k2hh->fast_calib_y_rslt, FAIL);

	error = k2hh_do_calibration(dev,&axis);
	if (error) {
		dev_err(dev,"[k2hh_fast_calibration_y_store] do_calibration failed");
		return error;
	}
	error = k2hh_store_Calibration_data(dev);
	if (error) {
		dev_err(dev,"[k2hh_fast_calibration_y_store] store_Calibration_data failed");
		return error;
	}
	atomic_set(&k2hh->fast_calib_y_rslt, SUCCESS);
	return size;
}

static ssize_t attr_get_fast_calibration_z(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k2hh->fast_calib_z_rslt));
}

static ssize_t attr_set_fast_calibration_z(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	char axis = 'z';
	struct k2hh_acc_data *k2hh = dev_get_drvdata(dev);

	atomic_set(&k2hh->fast_calib_z_rslt, FAIL);

	error = k2hh_do_calibration(dev,&axis);
	if (error) {
		dev_err(dev,"[k2hh_fast_calibration_z_store] do_calibration failed");
		return error;
	}
	error = k2hh_store_Calibration_data(dev);
	if (error) {
		dev_err(dev,"[k2hh_fast_calibration_z_store] store_Calibration_data failed");
		return error;
	}
	atomic_set(&k2hh->fast_calib_z_rslt, SUCCESS);
	return size;
}




static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_polling_rate, attr_set_polling_rate);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_range, attr_set_range);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_enable, attr_set_enable);
static DEVICE_ATTR(value, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_sensor_cal_data, NULL); //LGE Add
static DEVICE_ATTR(raw, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_raw_data, NULL);
static DEVICE_ATTR(int1_config, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_intconfig1, attr_set_intconfig1);
static DEVICE_ATTR(int1_duration, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_duration1, attr_set_duration1);
static DEVICE_ATTR(int1_thresholdx, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_threshx1, attr_set_threshx1);
static DEVICE_ATTR(int1_thresholdy, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_threshy1, attr_set_threshy1);
static DEVICE_ATTR(int1_thresholdz, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_threshz1, attr_set_threshz1);
static DEVICE_ATTR(int1_source, 0444, attr_get_source1, NULL);
static DEVICE_ATTR(flush, S_IRUGO|S_IWUSR |S_IWGRP, NULL, attr_set_flush);
static DEVICE_ATTR(run_fast_calibration, S_IRUGO|S_IWUSR |S_IWGRP,   attr_get_fast_calibration, attr_set_fast_calibration);  //LGE Add
static DEVICE_ATTR(run_calibration, S_IRUGO|S_IWUSR |S_IWGRP,    attr_get_eeprom_writing, attr_set_eeprom_writing);          //LGE Add
static DEVICE_ATTR(fast_calibration_x, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_fast_calibration_x, attr_set_fast_calibration_x);  //LGE Add
static DEVICE_ATTR(fast_calibration_y, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_fast_calibration_y, attr_set_fast_calibration_y);  //LGE Add
static DEVICE_ATTR(fast_calibration_z, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_fast_calibration_z, attr_set_fast_calibration_z);  //LGE Add
//SGS
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR |S_IWGRP, attr_get_selftest, attr_set_selftest);
#ifdef DEBUG
static DEVICE_ATTR(reg_value, 0600, attr_reg_get, attr_reg_set);
static DEVICE_ATTR(reg_addr, 0200, NULL, attr_addr_set);
#endif
static struct attribute *acc_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_value.attr,
	&dev_attr_raw.attr,
	&dev_attr_range.attr,
	&dev_attr_selftest.attr,
	&dev_attr_int1_config.attr,
	&dev_attr_int1_duration.attr,
	&dev_attr_int1_thresholdx.attr,
	&dev_attr_int1_thresholdy.attr,
	&dev_attr_int1_thresholdz.attr,
	&dev_attr_int1_source.attr,
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,
#ifdef K2HH_ACCEL_CALIBRATION
	&dev_attr_run_fast_calibration.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	&dev_attr_flush.attr,
#endif
	NULL
};

static struct attribute_group acc_attribute_group = {
	.attrs = acc_sysfs_attrs,
};

/*
static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
*/

static int k2hh_acc_input_init(struct k2hh_acc_data *acc)
{
	int err;

	INIT_WORK(&acc->input_poll_work, k2hh_acc_input_poll_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = k2hh_acc_input_open;
	acc->input_dev->close = k2hh_acc_input_close;
	/*acc->input_dev->name = K2HH_ACC_DEV_NAME;*/
	acc->input_dev->name = "accelerometer";
	acc->input_dev->uniq = "k2hh";
	acc->input_dev->dev.init_name = LGE_ACCELEROMETER_NAME;
	//acc->input_dev->id.bustype = BUS_I2C;
	//acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);

	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, G_MIN, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, G_MIN, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, G_MIN, G_MAX, FUZZ, FLAT);

	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN,
								INT_MAX, 0, 0);

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void k2hh_acc_input_cleanup(struct k2hh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int k2hh_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct k2hh_acc_data *acc;

	u32 smbus_func = (I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	int err = -1;

	dev_err(&client->dev, "probe start.\n");

	acc = kzalloc(sizeof(struct k2hh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}
	
	/* Support for both I2C and SMBUS adapter interfaces. */
	acc->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			acc->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}


	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}


	if(client->dev.of_node) {
		/* device tree type */
		err = k2hh_parse_dt(&client->dev,acc->pdata);

	} else {
		if(client->dev.platform_data == NULL) {
			default_k2hh_acc_pdata.gpio_int1 = int1_gpio;
#ifdef USE_ACC_IRQ2
			default_k2hh_acc_pdata.gpio_int2 = int2_gpio;
#endif
			memcpy(acc->pdata, &default_k2hh_acc_pdata,
					sizeof(*acc->pdata));
			dev_info(&client->dev, "using default plaform_data\n");
		} else {
			memcpy(acc->pdata, client->dev.platform_data,
					sizeof(*acc->pdata));
		}
	}
	acc->pdata->power_on = k2hh_power_on;
	acc->pdata->power_off = k2hh_power_off;
	err = k2hh_gpio_config(acc);
	if (err) {
		pr_info("k2hh_gpio_config ERROR  = %d\n", err);
	}
	acc->hr_timer_poll_work_queue = 0;

	err = k2hh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

/* remove this code to prevent  kernel crash while factory reset
	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}
*/
	if (acc->pdata->gpio_int1 >= 0) {
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, "
							"mapped on gpio:%d\n",
			K2HH_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}
#ifdef USE_ACC_IRQ2
	if (acc->pdata->gpio_int2 >= 0) {
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, "
							"mapped on gpio:%d\n",
			K2HH_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}
#endif
	k2hh_acc_set_init_register_values(acc);

#ifdef K2HH_ACCEL_CALIBRATION
	acc->cal_data.bCalLoaded = 0;
#endif
	err = k2hh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = k2hh_acc_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = k2hh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	acc->hr_timer_poll_work_queue = create_workqueue("k2hh_acc_hr_timer_poll_wq");
	hrtimer_init(&acc->hr_timer_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acc->hr_timer_poll.function = &k2hh_acc_hr_timer_poll_function;

	err = k2hh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_remove_hr_work_queue;
	}

	err = sysfs_create_group(&acc->input_dev->dev.kobj, &acc_attribute_group);
	//err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device K2HH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}


	k2hh_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if (acc->pdata->gpio_int1 >= 0) {
		INIT_WORK(&acc->irq1_work, k2hh_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("k2hh_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_mutexunlock;
		}
		err = request_irq(acc->irq1, k2hh_acc_isr1,
			IRQF_TRIGGER_RISING, "k2hh_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}
#ifdef USE_ACC_IRQ2
	if (acc->pdata->gpio_int2 >= 0) {
		INIT_WORK(&acc->irq2_work, k2hh_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("k2hh_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, k2hh_acc_isr2,
			IRQF_TRIGGER_RISING, "k2hh_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}
#endif


	mutex_unlock(&acc->lock);

	dev_err(&client->dev, "%s: probed\n", K2HH_ACC_DEV_NAME);

	return 0;

#ifdef USE_ACC_IRQ2
err_destoyworkqueue2:
	if (acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);

err_free_irq1:
	free_irq(acc->irq1, acc);
#endif
err_destoyworkqueue1:
	if (acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
/*
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
*/
err_input_cleanup:
	k2hh_acc_input_cleanup(acc);
err_remove_hr_work_queue:
	if(!acc->hr_timer_poll_work_queue) {
			flush_workqueue(acc->hr_timer_poll_work_queue);
			destroy_workqueue(acc->hr_timer_poll_work_queue);
	}
err_power_off:
	k2hh_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
/* err_freedata: */
	kfree(acc);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", K2HH_ACC_DEV_NAME);
	return err;
}

static int k2hh_acc_remove(struct i2c_client *client)
{

	struct k2hh_acc_data *acc = i2c_get_clientdata(client);

	dev_info(&acc->client->dev, "driver removing\n");

	if (acc->pdata->gpio_int1 >= 0) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}
#ifdef USE_ACC_IRQ2
	if (acc->pdata->gpio_int2 >= 0) {
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}
#endif
	k2hh_acc_disable(acc);
	k2hh_acc_input_cleanup(acc);

	//remove_sysfs_interfaces(&client->dev);

	if(!acc->hr_timer_poll_work_queue) {
			flush_workqueue(acc->hr_timer_poll_work_queue);
			destroy_workqueue(acc->hr_timer_poll_work_queue);
	}

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int k2hh_acc_resume(struct i2c_client *client)
{
	struct k2hh_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return k2hh_acc_enable(acc);
	return 0;
}

static int k2hh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct k2hh_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return k2hh_acc_disable(acc);
}
#else
#define k2hh_acc_suspend	NULL
#define k2hh_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id k2hh_acc_id[]
		= { { K2HH_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, k2hh_acc_id);

#ifdef CONFIG_OF
static struct of_device_id k2hh_match_table[] = {
	    { .compatible = "st,k2hh",},
		{ },
};
#else
#define k2hh_match_table NULL
#endif

static struct i2c_driver k2hh_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = K2HH_ACC_DEV_NAME,
			.of_match_table = k2hh_match_table,
		  },
	.probe = k2hh_acc_probe,
	.remove = k2hh_acc_remove,
	.suspend = k2hh_acc_suspend,
	.resume = k2hh_acc_resume,
	.id_table = k2hh_acc_id,
};

static int __init k2hh_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n",
						K2HH_ACC_DEV_NAME);
	return i2c_add_driver(&k2hh_acc_driver);
}

static void __exit k2hh_acc_exit(void)
{

	pr_info("%s accelerometer driver exit\n",
						K2HH_ACC_DEV_NAME);

	i2c_del_driver(&k2hh_acc_driver);
	return;
}

module_init(k2hh_acc_init);
module_exit(k2hh_acc_exit);

MODULE_DESCRIPTION("k2hh accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

