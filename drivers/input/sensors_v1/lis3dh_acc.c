/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : lis3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *                    : Samuel Huo (samuel.huo@st.com)
 * Version            : V.1.1.0
 * Date               : 07/10/2012
 * Description        : LIS3DH accelerometer sensor driver
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
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 05/11/09
 First Release;
 Revision 1.0.3 22/01/2010
  Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
  modified _get_acceleration_data function;
  modified _update_odr function;
  manages 2 interrupts;
 Revision 1.0.6 15/11/2010
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9 07/25/2011
  Romove several unused functions,add 5ms delay in init,change sysfs attributes.
 Revision 1.1.0 07/10/2012
  To replace some deprecated functions for 3.4 kernel; to pass the checkpatch's formatting requirement;
  To add regulator request;

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
#include	<linux/pm.h>
#include	<linux/module.h>
#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>
#include	<linux/sensors.h>
#include    <linux/platform_device.h>
#include 	<linux/wakelock.h>
//weitianlei:add for vivo log
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#endif
#include <linux/jiffies.h>
//weitianlei:add end

#include  "lis3dh_acc.h"

#define	DEBUG	0

#define	G_MAX		16000


#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/




/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE	0x01
#define LIS3DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS3DH_ACC	0x33	/*	Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

/* add by flyshine */
#define lis3dh_CTRL_REG4 	CTRL_REG4
#define lis3dh_CR4_BDU_ON 	0x80
#define lis3dh_CR4_BLE_LE  	0x00
#define lis3dh_CR4_BLE_BE  	0x40
#define lis3dh_CR4_FS_2G 	0x00
#define lis3dh_DEFAULT_DELAY 200

/* end of flyshine */


#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/


#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1

#define LIS3DH_ACC_PM_OFF		0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES	0x07


#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */



#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */
#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */

#define LIS3DH_FIFO_CTL_REG   0x2e
#define LIS3DH_BW_25HZ				0x30
#define LIS3DH_FIFO_CTL_REG   0x2e
#define LIS3DH_BUFSIZE				256

#define GRAVITY_EARTH                   981
/* add by flyshine */
#define SELFTEST_X_TYPICAL 276
#define SELFTEST_Y_TYPICAL 276
#define SELFTEST_Z_TYPICAL 984

#define SELFTEST_X_MIN (abs())
#define SELFTEST_X_MAX ()
#define SELFTEST_Y_MIN ()
#define SELFTEST_Y_MAX ()
#define SELFTEST_Z_MIN ()
#define SELFTEST_Z_MAX ()

//weitianlei:add for vivo log
#define TAG  "Gsensor "       //"Msensor" "PAsensor" "GYsensor"
#define TAGI "Gsensor.I "     //KERN_INFO 
#define TAGE "Gsensor.E "     //KERN_ERR 
extern void print_vivo_init(const char* fmt, ...);
extern void print_vivo_main(const char* fmt, ...);

#define CONT_INT_TIME     50    //50ms 
#define POLL_DELAY_TIMES  2     //2X

#define CONT_INT_GATE   20 
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0
//weitianlei:add end
#define FORCE_REPORT_DATA_THRESHOLD 80
/* end of flyshine */
#define PHONE_MOVING_THRESHOLD 30

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
		{    1, ODR1250 },
		{    3, ODR400  },
		{    5, ODR200  },
		{   10, ODR100  },
		{   20, ODR50   },
		{   40, ODR25   },
		{  100, ODR10   },
		{ 1000, ODR1    },
};

struct acceleration {
	int x;
	int y;
	int z;
};

struct calibrate_data {
	int x;
	int y;
	int z;
	int self_test;
};

struct lis3dh_acc_data {
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;
	struct sensors_classdev cdev;

	struct mutex lock;
	struct mutex data_mutex;
	struct delayed_work input_work;
	struct workqueue_struct *data_wq;
	struct input_dev *input_dev;

	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	int	calibrate_process;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	struct acceleration lastdata;       /* last measured data */
	struct acceleration lastraw;       /* last measured data */
	u8 data_valid;
	struct calibrate_data calibration_data;
	atomic_t eintstatus;
#ifdef DEBUG
	u8 reg_addr;
#endif
    //weitianlei:add for check if sensor on after 
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
    //weitianlei:add end
	unsigned char power_delay;

    unsigned char force_report;
};

static struct sensors_classdev lis3dh_acc_cdev = {
	.name = "lis3dh-accel",
	.vendor = "STMicroelectronics",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 10000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator lis3dh_acc_vreg[] = {
	{NULL, "vdd", 2850000/*1700000*/, 2850000/*3600000*/},
	{NULL, "vddio", 1800000/*1700000*/, 1800000/*3600000*/},
};

void gpio_switch_setstate(int state);
static struct i2c_client *lis3dh_acc_i2c_client;
static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,int *xyz);
static int poll_delay_num = 0;
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)

static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc,
				u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		printk(KERN_ERR TAGE "%s i2c_transfer error: (%d %p %d) %d\n",__func__, acc->client->addr, buf, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		printk(KERN_ERR TAGE "%s i2c_transfer error: (%d %p %d) %d\n",__func__, acc->client->addr, buf, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}
/*fix the bug: the eint would be cleared if acc sensor is still working while screen off*/
static int flag_enit_set = 0;
static ssize_t set_acc_eint(const char *buffer, size_t count);
/*fix the bug end*/
static int lis3dh_acc_hw_init(struct lis3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];
	char temp_buffer = '1';
	
	printk(KERN_INFO TAGI "%s start\n", __func__);
	buf[0] = WHO_AM_I;
	err = lis3dh_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
		printk(KERN_ERR TAGE "Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_LIS3DH_ACC) {
		printk(KERN_ERR TAGE "device unknown. Expected: 0x%x, Replies: 0x%x\n",WHOAMI_LIS3DH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = lis3dh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = lis3dh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto err_resume_state;
	
	/*fix the bug: the eint would be cleared if acc sensor is still working while screen off*/
	if(flag_enit_set)
	{
		printk(KERN_INFO TAGI "%s reset enit\n", __func__);
		set_acc_eint(&temp_buffer, 1);
	}
	/*fix the bug end*/
	
	acc->hw_initialized = 1;
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	printk(KERN_ERR TAGE "lis3dh hw init error 0x%x,0x%x: %d\n", buf[0],buf[1], err);
	return err;
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
static struct wake_lock ts_judge_phone_direction_wakelock;
extern bool (*acc_for_ts_judge_dir)(void);
bool lis3dh_for_ts_judge_dir(void)
{
	int result = 0;
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *priv; 
	int acc[3] = {0};

    if(!client)
		return false;
	priv = i2c_get_clientdata(client); 
	wake_lock_timeout(&ts_judge_phone_direction_wakelock, HZ/2); 

    mutex_lock(&priv->lock);
	result = lis3dh_acc_get_acceleration_data(priv, acc);
	mutex_unlock(&priv->lock);
	if (result < 0)
	{
		printk(KERN_ERR TAGE "<<-GTP->>%s get_acceleration_data failed\n", __func__);
		return false;
	}	

	printk(KERN_INFO TAGI "<<-GTP-INFO->>%s data is X(%d) Y(%d) Z(%d)\n", __func__, acc[0],acc[1], acc[2]);

	if (acc[1] < Y_MAX_THRESHOLD && (acc[2] > Z_MIN_THRESHOLD && acc[2] < Z_MAX_THRESHOLD)) 
	{
		printk(KERN_INFO TAGI "<<-GTP-INFO->>%s The phone is handstand\n", __func__);
		return true;
	} else {
		printk(KERN_INFO TAGI "<<-GTP-INFO->>%s The phone is NOT handstand\n", __func__);
		return false;
	}

}

static void lis3dh_acc_device_power_off(struct lis3dh_acc_data *acc)
{
	/*int err;
	u8 buf[2] = { CTRL_REG1, LIS3DH_ACC_PM_OFF };

	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);*/
	printk(KERN_INFO TAGI "%s\n", __func__);

	/*if (gpio_is_valid(acc->pdata->gpio_int1))
		disable_irq_nosync(acc->irq1);
	if (gpio_is_valid(acc->pdata->gpio_int2))
		disable_irq_nosync(acc->irq2);*/

	/*lis3dh_acc_config_regulator(acc, false);*/

	if (acc->hw_initialized) {
		/*if (gpio_is_valid(acc->pdata->gpio_int1))
			disable_irq_nosync(acc->irq1);
		if (gpio_is_valid(acc->pdata->gpio_int2))
			disable_irq_nosync(acc->irq2);*/
		acc->hw_initialized = 0;
	}
}

static int lis3dh_acc_device_power_on(struct lis3dh_acc_data *acc)
{
	int err = -1;

	/*err = lis3dh_acc_config_regulator(acc, true);
	if (err < 0) {
		dev_err(&acc->client->dev,
				"power_on failed: %d\n", err);
		return err;
	}*/

	/*if (gpio_is_valid(acc->pdata->gpio_int1))
		enable_irq(acc->irq1);
	if (gpio_is_valid(acc->pdata->gpio_int2))
		enable_irq(acc->irq2);*/
    if(acc->power_delay)
	msleep(20);

	if (!acc->hw_initialized) {
		err = lis3dh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		/*if (gpio_is_valid(acc->pdata->gpio_int1))
			enable_irq(acc->irq1);
		if (gpio_is_valid(acc->pdata->gpio_int2))
			enable_irq(acc->irq2);*/
	}
	return 0;
}
/*
static irqreturn_t lis3dh_acc_isr1(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	return IRQ_HANDLED;
}

static irqreturn_t lis3dh_acc_isr2(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);

	return IRQ_HANDLED;
}

static void lis3dh_acc_irq1_work_func(struct work_struct *work)
{

	struct lis3dh_acc_data *acc =
	container_of(work, struct lis3dh_acc_data, irq1_work);

	goto exit;
exit:
	enable_irq(acc->irq1);
}

static void lis3dh_acc_irq2_work_func(struct work_struct *work)
{

	struct lis3dh_acc_data *acc =
	container_of(work, struct lis3dh_acc_data, irq2_work);

	goto exit;
exit:
	enable_irq(acc->irq2);
}
*/
int lis3dh_acc_update_g_range(struct lis3dh_acc_data *acc, u8 new_g_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
	case LIS3DH_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LIS3DH_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LIS3DH_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case LIS3DH_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		printk(KERN_ERR TAGE "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 4,
		* which contains g range setting */
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return err;
error:
	printk(KERN_ERR TAGE "update g range failed 0x%x,0x%x: %d\n",buf[0], buf[1], err);

	return err;
}

int lis3dh_acc_update_odr(struct lis3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = lis3dh_acc_odr_table[i].mask;

	config[1] |= LIS3DH_ACC_ENABLE_ALL_AXES;
    printk(KERN_INFO TAGI "%s poll_interval_ms %d register %x\n", __func__, poll_interval_ms, config[1]);
	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
//	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = lis3dh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
//	}

	return err;

error:
	printk(KERN_ERR TAGE "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lis3dh_acc_register_write(struct lis3dh_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	return err;
}


static int lis3dh_acc_get_acceleration_rawdata(struct lis3dh_acc_data *acc,int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
	
	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));
/*
	xyz[0] = xyz[0] * acc->sensitivity;    
 	xyz[1] = xyz[1] * acc->sensitivity;
	xyz[2] = xyz[2] * acc->sensitivity;
*/


	#ifdef DEBUG
	/*
		printk(KERN_INFO TAGI "%s read x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	*/
	#endif
	return err;
}
static bool is_in_fifo_mode = false;
static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
		
	static int same_num_x = 0;
	static int same_num_y = 0;
	static int same_num_z = 0;

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);
/*
	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;
*/
	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

	//weitianlei: add for same data detect
    if(acc->lastraw.x == xyz[0])
    {
        if(same_num_x < SAME_DATA_GATE)
            same_num_x++;
        else
            printk(KERN_ERR TAGE "same data x error, x = %d\n",xyz[0]);
    }
    else
    {
        same_num_x = 0;
    }
	if(acc->lastraw.y == xyz[1])
    {
        if(same_num_y < SAME_DATA_GATE)
            same_num_y++;
        else
            printk(KERN_ERR TAGE "same data y error, y = %d\n",xyz[1]);
    }
    else
    {
        same_num_y = 0;
    }
	if(acc->lastraw.z == xyz[2])
    {
        if(same_num_z < SAME_DATA_GATE)
            same_num_z++;
        else
            printk(KERN_ERR TAGE "same data z error, z = %d\n",xyz[2]);
    }
    else
    {
        same_num_z = 0;
    }
    //weitianlei: add end
    acc->lastraw.x = xyz[0];
    acc->lastraw.y = xyz[1];
    acc->lastraw.z = xyz[2];
	acc->data_valid|=0xf0;

    acc->lastdata.x = xyz[0] * acc->sensitivity * GRAVITY_EARTH/100;
    acc->lastdata.y = xyz[1] * acc->sensitivity * GRAVITY_EARTH/100;
    acc->lastdata.z = xyz[2] * acc->sensitivity * GRAVITY_EARTH/100;
	acc->data_valid|=0x0f;

	xyz[0] = xyz[0] * acc->sensitivity;     //   data * 100000  by xiaot
 	xyz[1] = xyz[1] * acc->sensitivity;     //   data * 100000  by xiaot
	xyz[2] = xyz[2] * acc->sensitivity;     //   data * 100000  by xiaot

//	#ifdef DEBUG
//	printk(KERN_INFO TAGI "%s read x=%d, y=%d, z=%d\n",
//			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
//	#endif
	return 0;
}

static void lis3dh_acc_report_values(struct lis3dh_acc_data *acc,
					int *xyz)
{
	ktime_t timestamp;
	int i_final_x=xyz[0] - acc->calibration_data.x;
	int i_final_y=xyz[1] - acc->calibration_data.y;
	int i_final_z=xyz[2] - acc->calibration_data.z;
	
	timestamp = ktime_get_boottime();
	input_report_abs(acc->input_dev, ABS_X, i_final_x);
	input_report_abs(acc->input_dev, ABS_Y, i_final_y);
	input_report_abs(acc->input_dev, ABS_Z, i_final_z);
	input_event(acc->input_dev,EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(acc->input_dev,EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
//	printk(KERN_INFO TAGI "%s read time_s=%ld, time_ns=%ld \n",
//		LIS3DH_ACC_DEV_NAME, ktime_to_timespec(timestamp).tv_sec,ktime_to_timespec(timestamp).tv_nsec);
	input_sync(acc->input_dev);
}

static int lis3dh_acc_enable(struct lis3dh_acc_data *acc)
{
	int err;

    printk(KERN_INFO TAGI "%s\n", __func__);
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lis3dh_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		poll_delay_num = 0;

        acc->force_report=1;
		queue_delayed_work(acc->data_wq, &acc->input_work,
				msecs_to_jiffies(/*acc->pdata->poll_interval*/1));
//		schedule_delayed_work(&acc->input_work,
//			msecs_to_jiffies(acc->pdata->poll_interval));
	}
    acc->data_valid=0x00;

	return 0;
}

static int lis3dh_acc_disable(struct lis3dh_acc_data *acc)
{
    printk(KERN_INFO TAGI "%s\n", __func__);
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		lis3dh_acc_device_power_off(acc);
	}
    acc->data_valid=0x00;

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lis3dh_acc_i2c_read(acc, &data, 1);
	if (err < 0)
		return err;
	ret = snprintf(buf, 4, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis3dh_acc_register_write(acc, x, reg, new_val);
	if (err < 0)
		return err;
	acc->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return snprintf(buf, 8, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	lis3dh_acc_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->g_range;
	switch (val) {
	case LIS3DH_ACC_G_2G:
		range = 2;
		break;
	case LIS3DH_ACC_G_4G:
		range = 4;
		break;
	case LIS3DH_ACC_G_8G:
		range = 8;
		break;
	case LIS3DH_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return snprintf(buf, 4, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	lis3dh_acc_update_g_range(acc, val);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
    printk(KERN_INFO TAGI "%s enable=%ld\n", __func__,val);
	if (val)
		lis3dh_acc_enable(acc);
	else
		lis3dh_acc_disable(acc);

	return size;
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

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lis3dh_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lis3dh_acc_i2c_read(acc, &data, 1);
	/* TODO: error need to be managed */
	ret = snprintf(buf, 8, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static void dumpReg(struct lis3dh_acc_data *acc)
{
    int i=0;
    u8 addr = 0x00;
    for(i=0x07; i<0x3e; i++)
    {
        //dump all
		addr = i;
        lis3dh_acc_i2c_read(acc, &addr, 1);
	    printk(KERN_INFO TAGI "lis3dh Reg addr=%x regdata=%x\n",i,addr);
		mdelay(5);
    }
}

static ssize_t sensor_readreg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{    
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	dumpReg(acc);   
	return 1;
}

static ssize_t sensor_writereg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{	
	u8 data[2];
	int tem;	
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);	
	tem = simple_strtoul(buf, NULL, 16);	
	data[0] = (u8)((tem&0xff00)>>8);	
	data[1]= (u8)(tem&0x00ff);
    lis3dh_acc_i2c_write(acc, data, 1);
	printk(KERN_INFO TAGI "lis3dh reg==>[%x]/[%x]\n",data[0],data[1]);
	return 1;
}

static int lis3dh_at_get_data(struct lis3dh_acc_data *acc, u8 wh_data)
{
	int err;
	int xyz[3] = { 0 };
	#ifdef WRITE_BACK_THE_REGISTER
	u8 reg_back = CTRL_REG1;
	#endif
	u8 buf[2];

	if(!atomic_read(&acc->enabled)||!(wh_data&acc->data_valid))
	{
	    #ifdef WRITE_BACK_THE_REGISTER
		err = lis3dh_acc_i2c_read(acc, &reg_back, 1);
		if (err < 0)
		{
			printk(KERN_ERR TAGE "%s read failed\n", __func__);
		}
		else
		#endif
		{
		    buf[0] = CTRL_REG1;
			buf[1] = 0x57;
			err = lis3dh_acc_i2c_write(acc, buf, 1);
			if (err < 0)
			{
			   printk(KERN_ERR TAGE "%s set register failed\n", __func__);
			}
			else
			{
				err = lis3dh_acc_get_acceleration_data(acc, xyz);
				if (err < 0)
					printk(KERN_ERR TAGE "%s get_acceleration_data failed\n", __func__);
			}
		}
		#ifdef WRITE_BACK_THE_REGISTER
		buf[0] = CTRL_REG1;
		buf[1] = reg_back;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
		{
		   printk(KERN_ERR TAGE "%s reset register failed\n", __func__);
		}
		#endif

	}
	return 0;
}

static ssize_t lis3dh_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	struct acceleration accel;

	mutex_lock(&acc->lock);
	lis3dh_at_get_data(acc, 0xf0);
	accel = acc->lastraw;
	acc->data_valid&=0x0f;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}
static ssize_t lis3dh_final_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	struct acceleration accel;

	mutex_lock(&acc->lock);
	lis3dh_at_get_data(acc, 0x0f);
	accel = acc->lastdata;
	acc->data_valid&=0xf0;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d %d %d\n",accel.x, accel.y, accel.z);
}

static ssize_t sensor_calibration_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
	// /*
		struct i2c_client *client = lis3dh_acc_i2c_client;
		struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);
		struct calibrate_data value = {5,5,6,6}; 
		int i,j=0;
		char bufnew[30];
		
	printk(KERN_INFO TAGI "sensor_calibration_store\n");
	for(i=0;i<=30;i++)
	{	bufnew[i] = *(buf+i);
	//	printk(KERN_INFO TAGI "%c",*(buf+i));
		if(*(buf+i)=='\0')
			break;
	}
	for(i=0;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.x = simple_strtol(bufnew, NULL, 10);
			*(bufnew+i)=' ';
			j=i;
			break;
		}
	}
	i++;
	for(;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.y = simple_strtol(bufnew+j+1, NULL, 10);
			*(bufnew+i)=' ';
			j=i;
			break;
		}
	}
	i++;
	for(;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.z = simple_strtol(bufnew+j+1, NULL, 10);
			*(bufnew+i)=' ';
			j=i;	
			break;
		}
	}
	i++;
	for(;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.self_test = simple_strtol(bufnew+j+1, NULL, 10);
			*(bufnew+i)=' ';
			j=i;	
			break;
		}
	}	

//	printk(KERN_INFO TAGI "  sensor_calibration_show	 x=%d	\n",value.x);
//	printk(KERN_INFO TAGI "  sensor_calibration_show	 y=%d	\n",value.y);
//	printk(KERN_INFO TAGI "  sensor_calibration_show	 z=%d	\n",value.z);
//	printk(KERN_INFO TAGI "  sensor_calibration_show	 self_test=%d	\n",value.self_test);
	if((value.x > 120000 || value.x < -120000) || (value.y > 120000 || value.y < -120000) || (value.z > 120000 || value.z < -120000))
	{
		value.x = 0;
		value.y = 0;
		value.z = 0;
	}
	/*as accelerator IC has already been over through sleftest ,so ignore it. flyshine*/
	//value.self_test =  lis3dh->calibration_data.self_test;
	/*end flyshine*/
    mutex_lock(&lis3dh->data_mutex);

    lis3dh->calibration_data = value;

    mutex_unlock(&lis3dh->data_mutex);

    return count;
//	*/
//     return 0;
}
static ssize_t sensor_calibration_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);
    struct calibrate_data value;
	printk(KERN_INFO TAGI "  sensor_calibration_show	\n");

    mutex_lock(&lis3dh->data_mutex);

    value = lis3dh->calibration_data;

    mutex_unlock(&lis3dh->data_mutex);
	printk(KERN_INFO TAGI "  sensor_calibration_show   x=%d	%d\n",value.x, lis3dh->calibration_data.x);
	printk(KERN_INFO TAGI "  sensor_calibration_show   y=%d	%d\n",value.y, lis3dh->calibration_data.y);
	printk(KERN_INFO TAGI "  sensor_calibration_show   z=%d	%d\n",value.z, lis3dh->calibration_data.z);
	printk(KERN_INFO TAGI "  sensor_calibration_show   self_test=%d	%d\n",value.self_test, lis3dh->calibration_data.self_test);

    return sprintf(buf, "%d %d %d %d  ov\n", value.x,value.y,value.z,value.self_test);
}
static ssize_t sensor_calibratecmd_xy(struct lis3dh_acc_data *lis3dh)
{	
	int i,sumofx,sumofy,sumofz;
	struct acceleration accelnew;
	int * p_accelnew=(int*)&accelnew;
	int data_valid=0;
	unsigned long delay = 20;
	sumofx = 0;
	sumofy = 0;
	sumofz = 0;
//	printk(KERN_INFO TAGI "  sensor_calibratecmd_xy 	\n");

	for (i=0;i<30;i++)
	{
		//lis3dh_measure(lis3dh, &accelnew);
		if(!lis3dh_acc_get_acceleration_data(lis3dh,p_accelnew))
		{
			sumofx = sumofx + accelnew.x;
			sumofy = sumofy + accelnew.y;
			sumofz = sumofz + accelnew.z - 1024;
			data_valid++;
			//printk(KERN_INFO TAGI "get calibratecmd data : x=%d,y=%d,z=%d\n",accelnew.x,accelnew.y,accelnew.z);
		}
		mdelay(delay + 1);
	}
//	printk(KERN_INFO TAGI "  sensor_calibratecmd_xy   mdelay  = %lu	\n",delay);
	sumofx = sumofx / data_valid;
	sumofy = sumofy / data_valid;
	sumofz = sumofz / data_valid;
	if (sumofx > -100000 && sumofx < 100000)
		if (sumofy > -100000 && sumofy < 100000)
		{
			lis3dh->calibration_data.x = sumofx;
			lis3dh->calibration_data.y = sumofy;
			lis3dh->calibration_data.z = sumofz;
		}
	printk(KERN_INFO TAGI "  sensor_calibratecmd_x   =%d	%d\n",sumofx, lis3dh->calibration_data.x);
	printk(KERN_INFO TAGI "  sensor_calibratecmd_y   =%d	%d\n",sumofy, lis3dh->calibration_data.y);
	printk(KERN_INFO TAGI "  sensor_calibratecmd_z   =%d	%d\n",sumofz, lis3dh->calibration_data.z);
	
	return 0;
}

static ssize_t sensor_calibratecmd_z(struct lis3dh_acc_data *lis3dh)
{
	int i,sumofz;
	struct acceleration accelnew;
	int * p_accelnew =(int*)&accelnew;
	unsigned long delay = 20;
	sumofz = 0;
	printk(KERN_INFO TAGI "  sensor_calibratecmd_z\n");

	for (i=0;i<20;i++)
	{
		//lis3dh_measure(lis3dh, &accelnew);
		lis3dh_acc_get_acceleration_data(lis3dh,p_accelnew);
		sumofz = sumofz + accelnew.z;
		mdelay(delay + 1);
	}
	//printk(KERN_INFO TAGI "  sensor_calibratecmd_Z   mdelay  = %lu	\n",delay);
	sumofz = sumofz / 20;
	if (sumofz > -100000 && sumofz < 100000)
		lis3dh->calibration_data.z = sumofz;
	printk(KERN_INFO TAGI "  sensor_calibratecmd_z   =%d  \n",sumofz);

	
	return 0;
}

static ssize_t sensor_calibratecmd_selftest(struct lis3dh_acc_data *lis3dh)
{
	struct acceleration accelold;
	int *p_accelold=(int*)&accelold;
	struct acceleration accelnew;
	int *p_accelnew=(int*)&accelnew;
	u8 data;
	//unsigned long delay = delay_to_jiffies(atomic_read(&lis3dh->delay));
	int i,x_old_sum,y_old_sum,z_old_sum;
	int x_new_sum,y_new_sum,z_new_sum;
	int x_change,y_change,z_change;
	
	x_old_sum=y_old_sum=z_old_sum=0;
	x_new_sum=y_new_sum=z_new_sum=0;
	x_change=y_change=z_change=0;
	
	data = lis3dh_CR4_FS_2G;
	data |= lis3dh_CR4_BDU_ON | lis3dh_CR4_BLE_LE ;
	lis3dh_acc_register_write(lis3dh, &data,lis3dh_CTRL_REG4,data);
	mdelay(500);
	for ( i=0; i < 5; i++ ){
		lis3dh_acc_get_acceleration_rawdata(lis3dh,p_accelold);
		x_old_sum +=accelold.x;
		y_old_sum +=accelold.y;
		z_old_sum +=accelold.z;
		mdelay(5);
	}

	
	data = lis3dh_CR4_FS_2G;
	data |= lis3dh_CR4_BDU_ON | lis3dh_CR4_BLE_LE  | 0x02 ;

	lis3dh_acc_register_write(lis3dh, &data,lis3dh_CTRL_REG4,data);
	mdelay(500);

	for( i=0;i < 5; i++){
		lis3dh_acc_get_acceleration_rawdata(lis3dh,p_accelnew);	
		x_new_sum +=accelnew.x;
		y_new_sum +=accelnew.y;
		z_new_sum +=accelnew.z;
		mdelay(5);
	}
	
	x_change=abs( x_new_sum/5 - x_old_sum/5);
	y_change=abs( y_new_sum/5 - y_old_sum/5);
	z_change=abs( z_new_sum/5 - z_old_sum/5);
	data = lis3dh_CR4_FS_2G;
	data |= lis3dh_CR4_BDU_ON | lis3dh_CR4_BLE_LE;
	lis3dh_acc_register_write(lis3dh, &data,lis3dh_CTRL_REG4,data);
	mdelay(200);

	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest accelnew +++ x=%d     y=%d   z=%d\n",
			x_new_sum/5,y_new_sum/5,z_new_sum/5);
	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest accelold --- x=%d     y=%d   z=%d\n",
			x_old_sum/5,y_old_sum/5,z_old_sum/5);


	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest  x=%d \n",x_change);
	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest  y=%d \n",y_change);
	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest  z=%d \n",z_change);
			
	lis3dh->calibration_data.self_test = 2;
	lis3dh->calibrate_process=2;
	if ( ( (x_change > 60) && (x_change < 1700) ) && 
		 ( (y_change > 60) && (y_change < 1700) ) &&
		 ( (z_change > 40) && (z_change < 1400) )	)
		 {
				lis3dh->calibration_data.self_test = 1;
				//lis3dh->calibrate_process=2;
			}

	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest  self_test=%d \n",lis3dh->calibration_data.self_test);

	return 0;
}

static ssize_t sensor_calibratecmd_store(struct device_driver *dev_driver, const char *buf, size_t count)
{

	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);
    int delay = atomic_read(&lis3dh->delay);
 //  printk(KERN_INFO TAGI "  sensor_calibratecmd_store  \n");
	unsigned long cmd = simple_strtoul(buf, NULL, 10);
    int err;
    u8 reg_back = CTRL_REG1;
    u8 reg_buf[2];

	printk(KERN_INFO TAGI "sensor_calibratecmd_store cmd= %d enable %d\n",(int)cmd, atomic_read(&lis3dh->enabled));

	if (atomic_read(&lis3dh->enabled)) 
	{                   
		cancel_delayed_work_sync(&lis3dh->input_work);
	}

	err = lis3dh_acc_i2c_read(lis3dh, &reg_back, 1);
	if (err < 0)
	{
		printk(KERN_ERR TAGE "%s read failed\n", __func__);
		goto ERROR_EXIT;
	}
	else
	{
	    reg_buf[0] = CTRL_REG1;
		reg_buf[1] = 0x47;
		err = lis3dh_acc_i2c_write(lis3dh, reg_buf, 1);
		if (err < 0)
		{
		   printk(KERN_ERR TAGE "%s set register failed\n", __func__);
		   goto ERROR_EXIT;
		}
	}

	if (cmd == 1)
	{	lis3dh->calibrate_process = 1;
		sensor_calibratecmd_xy(lis3dh);
		lis3dh->calibrate_process = 0;
	}
	else if (cmd == 2) 
	{	lis3dh->calibrate_process = 1;
		sensor_calibratecmd_z(lis3dh);
		lis3dh->calibrate_process = 0;
	}
        else if (cmd == 3) 
	{	lis3dh->calibrate_process = 1;
		sensor_calibratecmd_selftest(lis3dh);
		lis3dh->calibrate_process = 0;
	}
	else
	{
	}
	printk(KERN_INFO TAGI "sensor_calibratecmd_store cmd= %d enable %d\n",(int)cmd, atomic_read(&lis3dh->enabled));

	reg_buf[0] = CTRL_REG1;
	reg_buf[1] = reg_back;
	err = lis3dh_acc_i2c_write(lis3dh, reg_buf, 1);
	if (err < 0)
	{
	   printk(KERN_ERR TAGE "%s reset register failed\n", __func__);
	}
	
	if (atomic_read(&lis3dh->enabled)) 
	{ 
		poll_delay_num = 0;
		queue_delayed_work(lis3dh->data_wq, &lis3dh->input_work, delay_to_jiffies(delay) + 1);
//		schedule_delayed_work(&lis3dh->input_work, delay_to_jiffies(delay) + 1);                  
	}
	


ERROR_EXIT:
	return count;
}

static ssize_t sensor_calibratecmd_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);
    int value;
//	printk(KERN_INFO TAGI " sensor_calibratecmd_show  \n");

	mutex_lock(&lis3dh->data_mutex);
	
    value = lis3dh->calibrate_process;

    mutex_unlock(&lis3dh->data_mutex);

    return sprintf(buf, "%d\n", value);

}


static ssize_t sensor_eint_store(struct device_driver *dev_driver, const char *buffer, size_t count)
{
	return set_acc_eint(buffer, count);
}
static ssize_t set_acc_eint(const char *buffer, size_t count)
{
	int err;
	u8 buf[2];
	int v_max;
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);
 //   int delay = atomic_read(&lis3dh->delay);
 //  printk(KERN_INFO TAGI "sensor_calibratecmd_store  \n");
	unsigned int eint_state = simple_strtoul(buffer, NULL, 10);
//	printk(KERN_INFO TAGI "sensor_calibratecmd_store cmd= %d\n",(int)cmd);
    printk(KERN_INFO TAGI "start:sensor_eint_store eint_state = %x\n",eint_state);
	
	if (eint_state == 1)
	{
//		disable_irq_nosync(lis3dh->irq1);	
//		enable_irq(lis3dh->irq1);
		gpio_switch_setstate(0);
		msleep(20);
		buf[0] = CTRL_REG3;
		buf[1] = 0x40;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1);  //enable INT1
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
		
		buf[0] = CTRL_REG5;
		buf[1] = 0x08;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1); //latch INT1
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);	
		
	   v_max = abs(lis3dh->lastraw.x);
	   if(v_max < abs(lis3dh->lastraw.y))
	   {
		  v_max = abs(lis3dh->lastraw.y);
	   }
	   if(v_max < abs(lis3dh->lastraw.z))
	   {
		  v_max = abs(lis3dh->lastraw.z);
	   }
	   printk(KERN_INFO TAGI "sensor_eint_enable[%d],[%d],[%d],[%d]\n",lis3dh->lastraw.x,lis3dh->lastraw.y,lis3dh->lastraw.z,v_max);
	   v_max += 128;
	   v_max >>= 4;	
		if(v_max>=0xff)
		{
			v_max=0xff;
		}
		
		buf[0] = INT_THS1;
		buf[1] = v_max;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1); //set threshold
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
		
		buf[0] = INT_DUR1;
		buf[1] = 0x00;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1); //set duration
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
			
		buf[0] = INT_CFG1;
		buf[1] = 0x7f;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1); //set triger source
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);	
	//	enable_irq(lis3dh->irq1);
	    flag_enit_set = 1;
	}
	else if (eint_state == 0)
	{		
//		disable_irq_nosync(lis3dh->irq1);	
	       printk(KERN_INFO TAGI "sensor_eint_disable\n");
		buf[0] = INT_SRC1;
		err = lis3dh_acc_i2c_read(lis3dh, buf, 1);   //clear eint flag
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_read error %ud\n",buf[0]);
			
		buf[0] = INT_CFG1;
		buf[1] = 0x00;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1); //clear triger source
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);	
					
		buf[0] = CTRL_REG3;
		buf[1] = 0x00;
		err = lis3dh_acc_i2c_write(lis3dh, buf, 1);  //disable INT1
		if (err < 0)
			printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
		gpio_switch_setstate(0);
		
		flag_enit_set = 0;
	}
	atomic_set(&(lis3dh->eintstatus),eint_state);  

	return count;
}

static ssize_t sensor_eint__show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);
    int value;
	printk(KERN_INFO TAGI "  sensor_calibratecmd_show  \n");

    value = atomic_read(&(lis3dh->eintstatus));  
    return sprintf(buf, "%d\n", value);
}

static int is_pock_mode_perhaps = 0;
static bool fifo_tag = false;
static int sample_cnt = 0;

static void sensor_fifo_enable(struct lis3dh_acc_data *lis3dh,bool flag)
{
    int err;
    u8 buf[2];
    if(is_in_fifo_mode == flag)
    {
    		printk(KERN_INFO TAGI "sensor_fifo_enable %d state already\n",flag);
    		return;
    }
    if(flag)	
    {
    	  if(!fifo_tag)
    	  {
    	  	  printk(KERN_INFO TAGI "[gsensor]No need to set fifo mode\n");
    	  	  return;
    	  }
	  	  is_pock_mode_perhaps = 0;
	  	  lis3dh_acc_device_power_on(lis3dh);
	  	  lis3dh->pdata->poll_interval = 40;//25HZ
	  	  lis3dh_acc_update_odr(lis3dh, lis3dh->pdata->poll_interval);
	  	  //LIS3DH_SetIntEnable(obj->client,true);
	  	  //sensor_set_reg_bit(obj->client, LIS3DH_REG_CTL_REG3, 0x04);
	  	  buf[0] = CTRL_REG5;
		    buf[1] = 0x40;
		    err = lis3dh_acc_i2c_write(lis3dh, buf, 1);
		    if (err < 0)
			  printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
			  
			  buf[0] = LIS3DH_FIFO_CTL_REG;
		    buf[1] = 0x9f;
		    err = lis3dh_acc_i2c_write(lis3dh, buf, 1);
		    if (err < 0)
			  printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
    }
    else
    {
	  	  buf[0] = LIS3DH_FIFO_CTL_REG;
		    buf[1] = 0x00;
		    err = lis3dh_acc_i2c_write(lis3dh, buf, 1);
		    if (err < 0)
			  printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
			  
			  buf[0] = CTRL_REG5;
		    buf[1] = 0x00;
		    err = lis3dh_acc_i2c_write(lis3dh, buf, 1);
		    if (err < 0)
			  printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
			    	 
        lis3dh->pdata->poll_interval = 20;//50HZ
	  	  lis3dh_acc_update_odr(lis3dh, lis3dh->pdata->poll_interval);
	  	  lis3dh_acc_device_power_off(lis3dh);
	  	  sample_cnt = 0;
    }
    printk(KERN_INFO TAGI "sensor_fifo_enable %d\n",flag);
    is_in_fifo_mode = flag;
    return;
}
struct st_fifo_data
{
	  int x_value;
	  int y_value;
	  int z_value;
};
static struct st_fifo_data gsensor_fifo_data[32];
#define STEP1_CNT_THR 2
#define STEP2_CNT_THR 3
static ssize_t
sensor_pockmode_show(struct device_driver *dev_driver, char *buf)
{
    int idx;
    int err;
    u8 buffer[2];
    int xyz[3] = { 0 };
    int xyTotalValue=0,authCnt=0,authStep=0;
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);

	  if(is_in_fifo_mode)
	  {
	      buffer[0] = 0x2f;
		    err = lis3dh_acc_i2c_read(lis3dh, buffer, 1);
		    if (err < 0)
			  printk(KERN_ERR TAGE "lis3dh_acc_i2c_read error %ud\n",buffer[0]);
		    sample_cnt = (int)buffer[0];
		    printk(KERN_INFO TAGI "+++++++++++++++sample_cnt: %d+++++++++++++\n",sample_cnt);
	      if(sample_cnt&0x40)
	      {
	      	  sample_cnt = 0x20;
	      }
	      else
	      {
	      		sample_cnt &= 0x1f;
	      }
	      memset(gsensor_fifo_data,0,sizeof(gsensor_fifo_data));
	      for(idx=0;idx<sample_cnt;idx++)
	      {
			      lis3dh_acc_get_acceleration_data(lis3dh, xyz);
			      gsensor_fifo_data[idx].x_value = xyz[0];
			      gsensor_fifo_data[idx].y_value = xyz[1];
			      gsensor_fifo_data[idx].z_value = xyz[2];
			      //sscanf(buff, "%x %x %x", &gsensor_fifo_data[idx].x_value,&gsensor_fifo_data[idx].y_value, &gsensor_fifo_data[idx].z_value);	
				  printk(KERN_INFO TAGI "[fifo]%d,%d,%d\n",gsensor_fifo_data[idx].x_value,gsensor_fifo_data[idx].y_value,gsensor_fifo_data[idx].z_value);
				}
			  for(idx=0;idx<sample_cnt;idx++)
			  {
			  	  printk(KERN_INFO TAGI "gsensor_fifo_data[%d].x_value[ %d ]gsensor_fifo_data[%d].y_value[ %d ]gsensor_fifo_data[%d].z_value[ %d ]\n",idx,gsensor_fifo_data[idx].x_value,idx,gsensor_fifo_data[idx].y_value,idx,gsensor_fifo_data[idx].z_value);
			  	  xyTotalValue = gsensor_fifo_data[idx].x_value*gsensor_fifo_data[idx].x_value + gsensor_fifo_data[idx].y_value*gsensor_fifo_data[idx].y_value;
			  	  switch(authStep) 
			  	  {
					  	  case 0:
							  	  //if(xyTotalValue > 30000000 && xyTotalValue < 120000000 && abs(gsensor_fifo_data[idx].z_value)<10000)  //x,y 5000:5000~10000:5000
							  	  if(xyTotalValue > 25000000 && xyTotalValue < 225000000 && abs(gsensor_fifo_data[idx].z_value)<10000)
							  	  {
							  	  	  authCnt ++;
							  	  }
							  	  else
							  	  { 
							  	  	  authCnt = 0;
							  	  }
							  	  printk(KERN_INFO TAGI "fifo auth:step[%d],cnt[%d]\n",authStep,authCnt);
							  	  if(authCnt>=STEP1_CNT_THR)
							  	  {
							  	  	  authStep = 1;
							  	  	  authCnt = 0;
							  	  }
							  	  break;
							  case 1:
							  	  //if(xyTotalValue > 110000000
							  	  	//&&  abs(gsensor_fifo_data[idx].z_value) < 15000 && (gsensor_fifo_data[idx].z_value > gsensor_fifo_data[idx-1].z_value))  //x,y 10000:7000
							  	  //if(xyTotalValue > 60000000
							  	  	//&&  abs(gsensor_fifo_data[idx].z_value) < 15000)  //x,y 10000:7000
							  	  if(abs(gsensor_fifo_data[idx].z_value) < 15000)  //x,y 10000:7000
							  	  {
							  	  	  authCnt ++;
							  	  }
							  	  else 
							  	  { 
							  	  	  authCnt = 0;
							  	  }
							  	  printk(KERN_INFO TAGI "fifo auth:step[%d],cnt[%d]\n",authStep,authCnt);
							  	  if(authCnt>=STEP2_CNT_THR)
							  	  {
							  	  	  authStep = 2;
							  	  	  authCnt = 0;
							  	  }
							  	  break;
			  	  }
			  }
			  is_pock_mode_perhaps = authStep;
			  //wake_lock_timeout(&gs_suspend_lock, 5 * HZ); 
	  }
	  printk(KERN_INFO TAGI "sample_cnt[%d]is_in_fifo_mode[%d]authStep[%d]\n",sample_cnt,is_in_fifo_mode,authStep);
	  sensor_fifo_enable(lis3dh,false);
    return sprintf(buf, "%d\n", is_pock_mode_perhaps);
}
static ssize_t sensor_pockmode_store(struct device_driver *dev_driver, const char *buf, size_t count)
{	
	int tem;
	struct i2c_client *client = lis3dh_acc_i2c_client;
	struct lis3dh_acc_data *lis3dh = i2c_get_clientdata(client);

	tem = simple_strtoul(buf, NULL, 10);	
	printk(KERN_INFO TAGI "%s tem %d\n", __func__, tem);
	if(tem == 1)
	    fifo_tag = true;
	else if(tem == 2)
  	  sensor_fifo_enable(lis3dh,true);
	else
	    fifo_tag = false;	
	printk(KERN_INFO TAGI "gsensor:set fifo tag to [%d]",tem);
	return count;
}
static struct device_attribute attributes[] = {

	__ATTR(poll_delay, 0644, attr_get_polling_rate,
			attr_set_polling_rate),
	__ATTR(range, 0644, attr_get_range, attr_set_range),
	__ATTR(enable, 0644, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0644, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0644, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0644, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0644, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0644, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0644, attr_get_click_tlim,
			attr_set_click_tlim),
	__ATTR(click_timelatency, 0644, attr_get_click_tlat,
							attr_set_click_tlat),
	__ATTR(click_timewindow, 0644, attr_get_click_tw, attr_set_click_tw),

#ifdef DEBUG
	__ATTR(reg_value, 0644, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0220, NULL, attr_addr_set),
#endif
	__ATTR(get_reg, 0644, sensor_readreg_get,sensor_writereg_set),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	printk(KERN_ERR TAGE "%s:Unable to create interface\n", __func__);
	return err;
}

static irqreturn_t lis3dh_acc_irq_handler(int irq, void *dev_id)
{
	struct lis3dh_acc_data *acc =(struct lis3dh_acc_data *)dev_id;

    printk(KERN_ERR TAGI "%s\n", __func__);
    disable_irq_nosync(acc->irq1);
    schedule_work(&acc->irq1_work);
	return IRQ_HANDLED;
}

static void lis3dh_acc_irq_work(struct work_struct *work)
{
    struct lis3dh_acc_data *acc =container_of(work, struct lis3dh_acc_data, irq1_work);
	int state,err,last_x,last_y,ths_flag = 0;
	u8 buf[2];
	int xyz[3] = { 0 };
	
    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        goto exit_err;
    }
	last_x = acc->lastraw.x;
	last_y = acc->lastraw.y;
	err = lis3dh_acc_get_acceleration_data(acc, xyz);
	if(PHONE_MOVING_THRESHOLD<abs(xyz[0]-last_x) ||
	    PHONE_MOVING_THRESHOLD<abs(xyz[1]-last_y))
	{
		ths_flag = 1;
	}	
	flag_enit_set = 0;
	mutex_lock(&acc->lock);
    state = gpio_get_value(acc->pdata->gpio_int1);
	if(ths_flag == 1)
	{	gpio_switch_setstate(state);            
		printk(KERN_ERR TAGI "%s gpio state %d\n", __func__, state);
	}
	buf[0] = INT_SRC1;
	err = lis3dh_acc_i2c_read(acc, buf, 1);   //clear eint flag
	if (err < 0)
		printk(KERN_ERR TAGE "lis3dh_acc_i2c_read error %ud\n",buf[0]);
		
	buf[0] = INT_CFG1;
	buf[1] = 0x00;
	err = lis3dh_acc_i2c_write(acc, buf, 1); //clear triger source
	if (err < 0)
		printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);	
				
	buf[0] = CTRL_REG3;
	buf[1] = 0x00;
	err = lis3dh_acc_i2c_write(acc, buf, 1);  //disable INT1
	if (err < 0)
		printk(KERN_ERR TAGE "lis3dh_acc_i2c_write error %ud\n",buf[0]);
	printk(KERN_INFO TAGI "sensor_eint_disable\n");
	if(ths_flag == 1)
		gpio_switch_setstate(0);
	mutex_unlock(&acc->lock);
	enable_irq(acc->irq1);
	if(ths_flag == 0)
	{
		mdelay(500);		
		set_acc_eint("1",1);
	}
exit_err:
    ;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int lis3dh_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
		struct lis3dh_acc_data, cdev);
	int err;
	printk(KERN_INFO TAGI "%s delay = %d\n", __func__,delay_msec);

	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = delay_msec;
	err = lis3dh_acc_update_odr(acc, delay_msec);
	mutex_unlock(&acc->lock);
	return err;
}

static int lis3dh_acc_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
		struct lis3dh_acc_data, cdev);
	int err;
	printk(KERN_INFO TAGI "%s enable = %d\n", __func__,enable);

	if (enable)
		err = lis3dh_acc_enable(acc);
	else
		err = lis3dh_acc_disable(acc);
	return err;
}

static void lis3dh_acc_input_work_func(struct work_struct *work)
{
	struct lis3dh_acc_data *acc;

	int xyz[3] = { 0 };
	int err;
	//weitianlei: add for check poll delay long
    static unsigned long last_poll_time = 0;
    unsigned long poll_time;
    unsigned long diff;
    //weitianlei: add end

	acc = container_of((struct delayed_work *)work,
			struct lis3dh_acc_data,	input_work);

	mutex_lock(&acc->lock);
	err = lis3dh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		printk(KERN_ERR TAGE "get_acceleration_data failed\n");
	else
		lis3dh_acc_report_values(acc, xyz);

    if(acc->force_report && FORCE_REPORT_DATA_THRESHOLD<acc->pdata->poll_interval)
    {
        queue_delayed_work(acc->data_wq, &acc->input_work,
                    msecs_to_jiffies((acc->pdata->poll_interval)>>1));
        printk(KERN_ERR TAGI "%s force_report %d poll_interval %d\n", __func__, 
            acc->force_report, acc->pdata->poll_interval);
    }
    else
    {
        queue_delayed_work(acc->data_wq, &acc->input_work,
                    msecs_to_jiffies(acc->pdata->poll_interval));
    }
    acc->force_report=0;
//	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
//			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
	
	//weitianlei: add for check poll delay long  
    poll_time = jiffies;
    diff = (long)poll_time - (long)last_poll_time;
    if((diff *1000)/HZ > (acc->pdata->poll_interval) * POLL_DELAY_TIMES)
    {
        if(poll_delay_num < POLL_DELAY_GATE)
            poll_delay_num++;
        else
            printk(KERN_ERR TAGE "poll delay long error:time = %ld\n",(diff *1000)/HZ); 
    }
    else
    {
        poll_delay_num = 0;
    }
    last_poll_time = poll_time;
    //weitianlei: add end
}

int lis3dh_acc_input_open(struct input_dev *input)
{
	return 0;
}

void lis3dh_acc_input_close(struct input_dev *dev)
{

}

static int lis3dh_acc_validate_pdata(struct lis3dh_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		printk(KERN_ERR TAGE "invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x,acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		printk(KERN_ERR TAGE "invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x,acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		printk(KERN_ERR TAGE "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis3dh_acc_input_init(struct lis3dh_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lis3dh_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lis3dh_acc_input_open;
	acc->input_dev->close = lis3dh_acc_input_close;
	acc->input_dev->name = ACCEL_INPUT_DEV_NAME;
	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


	err = input_register_device(acc->input_dev);
	if (err) {
		printk(KERN_ERR TAGE "unable to register input device %s\n",acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lis3dh_acc_input_cleanup(struct lis3dh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

#ifdef CONFIG_OF
static int lis3dh_parse_dt(struct device *dev,
			struct lis3dh_acc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "st,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read min-interval\n");
		return rc;
	} else {
		pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read init-interval\n");
		return rc;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read axis-map_x\n");
		return rc;
	} else {
		pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read axis_map_y\n");
		return rc;
	} else {
		pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read axis-map-z\n");
		return rc;
	} else {
		pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
		case 2:
			pdata->g_range = LIS3DH_ACC_G_2G;
			break;
		case 4:
			pdata->g_range = LIS3DH_ACC_G_4G;
			break;
		case 8:
			pdata->g_range = LIS3DH_ACC_G_8G;
			break;
		case 16:
			pdata->g_range = LIS3DH_ACC_G_16G;
			break;
		default:
			pdata->g_range = LIS3DH_ACC_G_2G;
			break;
		}
	}

	rc = of_property_read_u32(np, "st,negate-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read negate-x\n");
		return rc;
	} else {
		pdata->negate_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,negate-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read negate-y\n");
		return rc;
	} else {
		pdata->negate_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,negate-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read negate-z\n");
		return rc;
	} else {
		pdata->negate_z = (u8)temp_val;
	}	
	/*
	pdata->negate_x = of_property_read_bool(np, "st,negate-x");

	pdata->negate_y = of_property_read_bool(np, "st,negate-y");

	pdata->negate_z = of_property_read_bool(np, "st,negate-z");
	*/
	if(of_property_read_bool(dev->of_node,"sensor,gpio-irq"))
    {
    	rc = of_get_named_gpio_flags(dev->of_node,
    				"sensor,gpio-irq", 0, NULL);
    	if (rc < 0) {
    		printk(KERN_ERR TAGE "%s read interrupt int pin error %d\n", __func__, rc);
            pdata->gpio_int1= -2;
    	}
        else
        {
            pdata->gpio_int1= rc;
        }
	}
	else
	{
        pdata->gpio_int1= -2;
        printk(KERN_ERR TAGE "%s no sensor,gpio-irq\n", __func__);
	}
	/*rc = of_get_named_gpio_flags(dev->of_node,
				"st,gpio-int2", 0, NULL);
	if (rc < 0) {
		printk(KERN_ERR TAGE "%s Unable to read interrupt pin2 number\n", __func__);
		pdata->gpio_int2=-1;
		return 0;
	} else {
		pdata->gpio_int1 = rc;
	}*/

	return 0;
}
#else
static int lis3dh_parse_dt(struct device *dev,
			struct lis3dh_acc_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static int lis3dh_acc_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int lis3dh_acc_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lis3dh_acc_pd_driver = {
	.probe  = lis3dh_acc_pd_probe,
	.remove = lis3dh_acc_pd_remove,    
	.driver = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device lis3dh_acc_device = {
    .name = "gsensor",
    .id   = -1,
};
static DRIVER_ATTR(raw_data, 0644, lis3dh_raw_data_show, NULL);
static DRIVER_ATTR(data, 0644, lis3dh_final_data_show, NULL);
static DRIVER_ATTR(calibration, 0644, sensor_calibration_show, sensor_calibration_store);
static DRIVER_ATTR(calibratecmd, 0644, sensor_calibratecmd_show, sensor_calibratecmd_store);
static DRIVER_ATTR(set_eint, 0644, sensor_eint__show, sensor_eint_store);
static DRIVER_ATTR(pock_mode, 0644, sensor_pockmode_show, sensor_pockmode_store);

static struct driver_attribute *lis3dh_acc_attr_list[] = {
	&driver_attr_raw_data,
	&driver_attr_data,
	&driver_attr_calibration,
    &driver_attr_calibratecmd,
    &driver_attr_set_eint,
    &driver_attr_pock_mode,
};

static int lis3dh_acc_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(lis3dh_acc_attr_list)/sizeof(lis3dh_acc_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, lis3dh_acc_attr_list[idx])))
		{            
			printk(KERN_ERR TAGE "driver_create_file (%s) = %d\n", lis3dh_acc_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int lis3dh_acc_config_regulator(struct lis3dh_acc_data *obj, bool on)
{
	int rc = 0, i;
	struct sensor_regulator *lis3dh_acc_vreg_config=lis3dh_acc_vreg;
	int num_reg = sizeof(lis3dh_acc_vreg) / sizeof(struct sensor_regulator);

    printk(KERN_INFO TAGI "%s num_reg %d on %d\n", __func__, num_reg, on);
	if (on) 
    {
		for (i = 0; i < num_reg; i++) 
        {
			lis3dh_acc_vreg_config[i].vreg = regulator_get(&obj->client->dev, lis3dh_acc_vreg_config[i].name);
            
			if (IS_ERR(lis3dh_acc_vreg_config[i].vreg)) 
            {
				rc = PTR_ERR(lis3dh_acc_vreg_config[i].vreg);
				printk(KERN_ERR TAGE " %s:regulator get failed rc=%d\n", __func__, rc);
				lis3dh_acc_vreg_config[i].vreg = NULL;
			}

			if (regulator_count_voltages(lis3dh_acc_vreg_config[i].vreg) > 0) 
            {
                printk(KERN_INFO TAGI "%s min_uV %d max_uV %d\n", __func__, 
					lis3dh_acc_vreg_config[i].min_uV, lis3dh_acc_vreg_config[i].max_uV);
				rc = regulator_set_voltage(
					lis3dh_acc_vreg_config[i].vreg,
					lis3dh_acc_vreg_config[i].min_uV,
					lis3dh_acc_vreg_config[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE "%s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(lis3dh_acc_vreg_config[i].vreg);
					lis3dh_acc_vreg_config[i].vreg = NULL;
				}
			}

			rc = regulator_enable(lis3dh_acc_vreg_config[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE " %s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(lis3dh_acc_vreg_config[i].vreg) > 0) 
                {
					regulator_set_voltage(
						lis3dh_acc_vreg_config[i].vreg, 0,
						lis3dh_acc_vreg_config[i].max_uV);
				}
				regulator_put(lis3dh_acc_vreg_config[i].vreg);
				lis3dh_acc_vreg_config[i].vreg = NULL;
			}
		}
	} 
    else 
    {
		i = num_reg;
	}
    
	return rc;
}

//weitianlei:add for check if sensor on after screen off
#if defined(CONFIG_FB)
static int lis3dh_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct lis3dh_acc_data *acc = container_of(self, struct lis3dh_acc_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
        if (*blank == FB_BLANK_POWERDOWN)
        {
            if(atomic_read(&acc->enabled)) 
                printk(KERN_INFO TAGI "sensor on after screen off\n");
        }
	}
	return 0;
}
#endif
//weitianlei:add end

static int lis3dh_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lis3dh_acc_data *acc;
	int err = -1;

	printk(KERN_INFO TAGI "%s start!!\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR TAGE "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	acc = kzalloc(sizeof(struct lis3dh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE "failed to allocate memory for module data: %d\n", err);
		goto exit_check_functionality_failed;
	}
	print_vivo_init(TAGI "init 1\n");

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	mutex_init(&acc->data_mutex);


	acc->client = client;
	acc->power_delay=1;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE "failed to allocate memory for pdata: %d\n",err);
		goto err_mutexunlock;
	}
	memset(acc->pdata, 0 , sizeof(*acc->pdata));

	if (client->dev.of_node) {
		err = lis3dh_parse_dt(&client->dev, acc->pdata);
		if (err) {
			printk(KERN_ERR TAGE "Failed to parse device tree\n");
			err = -EINVAL;
			goto exit_kfree_pdata;
		}
	} else if (client->dev.platform_data != NULL) {
		memcpy(acc->pdata, client->dev.platform_data,
			sizeof(*acc->pdata));
	} else {
		printk(KERN_ERR TAGE "No valid platform data. exiting.\n");
		err = -ENODEV;
		goto exit_kfree_pdata;
	}

	lis3dh_acc_config_regulator(acc, true);
	msleep(3);
	
	print_vivo_init(TAGI "init 2\n");
	err = lis3dh_acc_validate_pdata(acc);
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			printk(KERN_ERR TAGE "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	/*if (gpio_is_valid(acc->pdata->gpio_int1))
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);

	if (gpio_is_valid(acc->pdata->gpio_int2))
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);*/
	printk(KERN_INFO TAGI "%s %d %d %d %d\n", __func__, acc->pdata->gpio_int1, acc->pdata->gpio_int2,
		acc->irq1, acc->irq2);

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;

	err = lis3dh_acc_device_power_on(acc);
	if (err < 0) {
		printk(KERN_ERR TAGE "power on failed: %d\n", err);
		goto err_pdata_init;
	}
	print_vivo_init(TAGI "init 3\n");
	atomic_set(&acc->enabled, 1);

	err = lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		printk(KERN_ERR TAGE "update_g_range failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		printk(KERN_ERR TAGE "update_odr failed\n");
		goto  err_power_off;
	}
	
	acc->data_wq = NULL;
	acc->data_wq = create_singlethread_workqueue("lis3dh_data_work");
	if (!acc->data_wq) {
		printk(KERN_ERR TAGE "create workquque failed\n");
		goto err_power_off;
	}
	err = lis3dh_acc_input_init(acc);
	if (err < 0) {
		printk(KERN_ERR TAGE "input init failed\n");
		goto err_destroy_workqueue;
	}

	print_vivo_init(TAGI "init 4\n");
	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		printk(KERN_ERR TAGE "device LIS3DH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	acc->cdev = lis3dh_acc_cdev;
	acc->cdev.sensors_enable = lis3dh_acc_enable_set;
	acc->cdev.sensors_poll_delay = lis3dh_acc_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &acc->cdev);
	if (err) {
		printk(KERN_ERR TAGE "class device create failed: %d\n", err);
		goto err_remove_sysfs_int;
	}	
	
	lis3dh_acc_device_power_off(acc);
	print_vivo_init(TAGI "init 5\n");
	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);
/*
	if (gpio_is_valid(acc->pdata->gpio_int1)) {
		INIT_WORK(&acc->irq1_work, lis3dh_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lis3dh_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			printk(KERN_ERR TAGE "cannot create work queue1: %d\n", err);
			goto err_unreg_sensor_class;
		}
		err = request_irq(acc->irq1, lis3dh_acc_isr1,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"lis3dh_acc_irq1", acc);
		if (err < 0) {
			printk(KERN_ERR TAGE "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if (gpio_is_valid(acc->pdata->gpio_int2)) {
		INIT_WORK(&acc->irq2_work, lis3dh_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lis3dh_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			printk(KERN_ERR TAGE "cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lis3dh_acc_isr2,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"lis3dh_acc_irq2", acc);
		if (err < 0) {
			printk(KERN_ERR TAGE "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}
*/
    if((err = platform_driver_register(&lis3dh_acc_pd_driver)))
	{
		printk(KERN_ERR TAGE "%s failed to register lis3dh_acc_driver err %d\n", __func__, err);
		goto err_unreg_sensor_class/*err_free_irq2*/;
	}

    if((err = platform_device_register(&lis3dh_acc_device)))
    {
		printk(KERN_ERR TAGE "%s failed to register lis3dh_acc_device err %d\n", __func__, err);
		goto err_free_driver;
    }

	if((err = lis3dh_acc_create_attr(&lis3dh_acc_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "%s lis3dh create attribute err = %d\n", __func__, err);
		goto err_free_device;
	}
    lis3dh_acc_i2c_client=client;
	mutex_unlock(&acc->lock);

    printk(KERN_INFO TAGI "%s %d\n", __func__, acc->pdata->gpio_int1);
    if((0<=acc->pdata->gpio_int1))
    {
        err = gpio_request(acc->pdata->gpio_int1, "gpio-int");
        if (err < 0)
            goto err_free_gpio;
        
        err = gpio_direction_input(acc->pdata->gpio_int1);
        if (err < 0)
            goto err_free_gpio;
        INIT_WORK(&acc->irq1_work, lis3dh_acc_irq_work);
        
        acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
        if (acc->irq1 < 0) {
            err = acc->irq1;
            goto err_free_gpio;
        }
        /*IRQF_TRIGGER_HIGH IRQF_TRIGGER_LOW */
        err = request_irq( acc->irq1, lis3dh_acc_irq_handler,
            IRQF_TRIGGER_RISING|IRQF_ONESHOT, "gpio-irq", acc);
        if (err < 0)
            goto err_free_gpio;
    	err = enable_irq_wake(acc->irq1);
    	if (err < 0)
    		goto err_free_gpio;
    }
	
	printk(KERN_INFO TAGI "%s: probed\n", LIS3DH_ACC_DEV_NAME);

	//weitianlei:add for check if sensor on after 
#if defined(CONFIG_FB)
    acc->fb_notif.notifier_call = lis3dh_fb_notifier_callback;
	fb_register_client(&acc->fb_notif);
#endif
	wake_lock_init(&ts_judge_phone_direction_wakelock, WAKE_LOCK_SUSPEND, "Ts_judge_dir");
	acc_for_ts_judge_dir = lis3dh_for_ts_judge_dir;
	acc->power_delay=0;
    //weitianlei:add end	
    acc->force_report=0;
	print_vivo_init(TAGI "init 6\n");
	return 0;

err_free_gpio:
	gpio_free(acc->pdata->gpio_int1);	
err_free_device:
	platform_device_unregister(&lis3dh_acc_device);
err_free_driver:
	platform_driver_unregister(&lis3dh_acc_pd_driver);
/*err_free_irq2:
	free_irq(acc->irq2, acc);
err_destoyworkqueue2:
	if (gpio_is_valid(acc->pdata->gpio_int2))
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if (gpio_is_valid(acc->pdata->gpio_int1))
		destroy_workqueue(acc->irq1_work_queue);*/
err_unreg_sensor_class:
	sensors_classdev_unregister(&acc->cdev);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lis3dh_acc_input_cleanup(acc);
err_destroy_workqueue:
	if (acc->data_wq)
		destroy_workqueue(acc->data_wq);
err_power_off:
	lis3dh_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	kfree(acc);
exit_check_functionality_failed:
	printk(KERN_ERR TAGE "%s: Driver Init failed\n", LIS3DH_ACC_DEV_NAME);
	return err;
}

static int lis3dh_acc_remove(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	if (gpio_is_valid(acc->pdata->gpio_int1)) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
	//	destroy_workqueue(acc->irq1_work_queue);
	}

	/*if (gpio_is_valid(acc->pdata->gpio_int2)) {
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}*/

	sensors_classdev_unregister(&acc->cdev);
	if (acc->data_wq)
		destroy_workqueue(acc->data_wq);
	lis3dh_acc_input_cleanup(acc);
	lis3dh_acc_device_power_off(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lis3dh_acc_resume(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);
    printk(KERN_INFO TAGI "%s acc->on_before_suspend %d\n", __func__, acc->on_before_suspend);
	//weitianlei:add for check if sensor on after screen off
    if(atomic_read(&acc->enabled)) 
        printk(KERN_ERR TAGE "sensor on after suspend\n");
    //weitianlei:add end

	if (acc->on_before_suspend)
		return lis3dh_acc_enable(acc);
	return 0;
}

static int lis3dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	printk(KERN_INFO TAGI "%s on_before_suspend %d\n", __func__, acc->on_before_suspend);

	return lis3dh_acc_disable(acc);
}
#else
#define lis3dh_acc_suspend	NULL
#define lis3dh_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis3dh_acc_id[]
		= { { LIS3DH_ACC_DEV_NAME, 0 }, { }, };

static struct of_device_id lis3dh_acc_match_table[] = {
	{ .compatible = "lis3dh_acc", },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);

static struct i2c_driver lis3dh_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LIS3DH_ACC_DEV_NAME,
			.of_match_table = lis3dh_acc_match_table,
		  },
	.probe = lis3dh_acc_probe,
	.remove = lis3dh_acc_remove,
	.suspend = lis3dh_acc_suspend,
	.resume = lis3dh_acc_resume,
	.id_table = lis3dh_acc_id,
};

static int __init lis3dh_acc_init(void)
{
	return i2c_add_driver(&lis3dh_acc_driver);
}

static void __exit lis3dh_acc_exit(void)
{
	i2c_del_driver(&lis3dh_acc_driver);
	return;
}

late_initcall(lis3dh_acc_init);
module_exit(lis3dh_acc_exit);

MODULE_DESCRIPTION("lis3dh digital accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Samuel Huo, STMicroelectronics");
MODULE_LICENSE("GPL");

