/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : kx023_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *                    : Samuel Huo (samuel.huo@st.com)
 * Version            : V.1.0
 * Date               : 11/08/2014
 * Description        : KX023 accelerometer sensor driver
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
 Revision 1.0 11/08/2014
 First Release;
 

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

#include "kx023_acc.h"

#define DEBUG

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES			5
#define	I2C_AUTO_INCREMENT	0x80

#define G_MAX				8000
#define GRAVITY_EARTH       981
//#define SELFTEST_MAX 		666
//#define SELFTEST_MIN 		358
#define SELFTEST_MIN 		256
#define SELFTEST_MAX 		768

#define SENSITIVITY_2G		1024	/**	mg/LSB	*/
#define SENSITIVITY_4G		512		/**	mg/LSB	*/
#define SENSITIVITY_8G		256		/**	mg/LSB	*/

#define WHOAMI_KX023_ACC	0x15	/*	Expected content for WAI */
#define SELFTEST_KX023_ACC  0xCA    /*  MEMS self-test enable    */
#define PC1_OFF				0x7F
#define PC1_ON				(1 << 7)
#define	HIGH_CURRENT		0x40


/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F       /*	WhoAmI register		*/
/* CONTROL REGISTERS */
#define CNTL1			0x18
#define CNTL2			0x19
#define CNTL3			0x1A
#define ODCNTL			0x1B
#define SELF_TEST       0x60
/* CONTROL REGISTER 1 BITS */

#define INS1			0x12
#define STATUS_REG   	0x15
#define	INT_REL			0x17	/*	interrupt source information	*/
#define	INC1			0x1C	/*	interrupt 1 enable control	*/
#define	INC2			0x1D	/*	interrupt 1 source	*/
#define	INC4			0x1F	/*	interrupt 1 routing	*/
#define WUFC			0x23
#define	ATH				0x30	/*	motion detect threshold	*/

/* Output Data Rates ODR */
#define ODR6_25F        0x0B
#define ODR12_5F		0x00
#define ODR25F			0x01
#define ODR50F			0x02
#define ODR100F			0x03
#define ODR200F			0x04
#define ODR400F			0x05
#define ODR800F			0x06
#define ODR1600F    	0x07

/* RESUME STATE INDICES */
#define	RES_WHO_AM_I	0
#define	RES_CNTL1		1
#define	RES_CNTL2		2
#define	RES_CNTL3		3
#define	RES_ODCNTL		4

#define	RES_INC1		5
#define	RES_INC2		6
#define	RES_INC3		7
#define	RES_INC4		8
#define	RES_INC5		9
#define	RES_INC6		10

#define	RES_TILT_TIMER		11
#define	RES_WUFC		12
#define	RES_TDTRC		13
#define	RES_TDTC		14
#define	RES_TTH			15
#define	RES_TTL			16
#define	RES_FTD			17
#define	RES_STD			18
#define	RES_TLT			19
#define	RES_TWS			20

#define	RES_ATH			21

#define	RES_TILT_ANGLE_LL		22
#define	RES_TILT_ANGLE_HL		23
#define	RES_HYST_SET			24
#define	RES_LP_CNTL				25

#define	RES_BUF_CNTL1		26
#define	RES_BUF_CNTL2		27
#define	RES_BUF_CLEAR		28
#define	RES_SELF_TEST		29

#define	RESUME_ENTRIES		30
/* end RESUME STATE INDICES */


#define KX023_PRINTK
#ifdef KX023_PRINTK	
#define KX023_REM_PRINTK(fmt,args...) printk(KERN_ERR fmt,##args)
#else	
#define KX023_REM_PRINTK(fmt,args...) //
#endif

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} kx023_acc_odr_table[] = {
    { 1,    ODR1600F},
	{ 2,	ODR800F },
	{ 3,	ODR400F },
	{ 5,	ODR200F },
	{ 10,	ODR100F },
	{ 20,	ODR50F  },
	{ 40,	ODR25F  },
	{ 80,	ODR12_5F},
	{ 160,	ODR6_25F}
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

struct kx023_acc_data {
	struct i2c_client *client;
	struct kx023_acc_platform_data *pdata;


	struct mutex lock;
	struct mutex data_mutex;
	
	struct sensors_classdev cdev;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	int	calibrate_process;

	int hw_initialized;				/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u16 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
//	int irq2;
//	struct work_struct irq2_work;
//	struct workqueue_struct *irq2_work_queue;

	struct acceleration lastdata;       /* last measured data */
	struct acceleration lastraw;       /* last measured data */
	u8 data_valid;
	struct calibrate_data calibration_data;
	atomic_t eintstatus;
#ifdef DEBUG
	u8 reg_addr;
#endif
};

static struct sensors_classdev kx023_acc_cdev = {
	.name = "kx023-accel",
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "78.4",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 5000,
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

struct sensor_regulator kx023_acc_vreg[] = {
	{NULL, "vdd", 2850000/*1700000*/, 2850000/*3600000*/},
	{NULL, "vddio", 1800000/*1700000*/, 1800000/*3600000*/},
};

static struct i2c_client *kx023_acc_i2c_client;
void gpio_switch_setstate(int state);

static int kx023_acc_i2c_read(struct kx023_acc_data *acc,
				u8 * buf, int len)
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
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int kx023_acc_i2c_write(struct kx023_acc_data *acc, u8 * buf,
								int len)
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
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int kx023_acc_register_write(struct kx023_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;
	u8 write_buf[2];

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		write_buf[0] = reg_address;
		write_buf[1] = new_value;
		err = kx023_acc_i2c_write(acc, write_buf, 1);
		if (err < 0)
			return err;
	return err;
}

/*static int kx023_acc_register_read(struct kx023_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	KX023_REM_PRINTK("%s used in kx023\n", __func__);
	buf[0] = (reg_address);
	err = kx023_acc_i2c_read(acc, buf, 1);
	return err;
}*/

static int kx023_acc_hw_init(struct kx023_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	KX023_REM_PRINTK("%s\n", __func__);
	buf[0] = WHO_AM_I;
	err = kx023_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
		dev_warn(&acc->client->dev,
		"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_KX023_ACC) {
		dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x, Replies: 0x%x\n",
		WHOAMI_KX023_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}
	
	buf[0] = CNTL1;
	buf[1] = acc->resume_state[RES_CNTL1]&(PC1_OFF);
	err = kx023_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | CNTL2);
	buf[1] = acc->resume_state[RES_CNTL2];
	buf[2] = acc->resume_state[RES_CNTL3];
	buf[3] = acc->resume_state[RES_ODCNTL];
	err = kx023_acc_i2c_write(acc, buf, 3);
	if (err < 0)
		goto err_resume_state;
		
	buf[0] = (I2C_AUTO_INCREMENT | INC1);
	buf[1] = acc->resume_state[RES_INC1];
	buf[2] = acc->resume_state[RES_INC2];
	buf[3] = acc->resume_state[RES_INC3];
	buf[4] = acc->resume_state[RES_INC4];
	err = kx023_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;
		
	buf[0] = ATH;
	buf[1] = acc->resume_state[RES_ATH];
	err = kx023_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = CNTL1;
	buf[1] = acc->resume_state[RES_CNTL1]|(PC1_ON);
	err = kx023_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;
		
	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", KX023_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}


static void kx023_acc_device_power_off(struct kx023_acc_data *acc)
{
	/*int err;
	u8 buf[2];
	buf[0] = CNTL1;
	buf[1] = acc->resume_state[RES_CNTL1]&(PC1_OFF);
	err = kx023_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed\n");*/
	
	KX023_REM_PRINTK("%s\n", __func__);
	
	if (acc->hw_initialized) {
		acc->hw_initialized = 0;
	}
}

static int kx023_acc_device_power_on(struct kx023_acc_data *acc)
{
	int err = -1;

	msleep(20);

	if (!acc->hw_initialized) {
		err = kx023_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			kx023_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {

	}

	return 0;
}

int kx023_acc_update_g_range(struct kx023_acc_data *acc, u8 new_g_range)
{
	int err = -1;

	u16 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = KX023_ACC_FS_MASK | HIGH_CURRENT;

	switch (new_g_range) {
	case KX023_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case KX023_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case KX023_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 1,
		* which contains g range setting */
		buf[0] = CNTL1;
		err = kx023_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CNTL1] = init_val;
		new_val = new_g_range | HIGH_CURRENT;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		
		buf[1] = updated_val & PC1_OFF;
		buf[0] = CNTL1;
		err = kx023_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
			
		buf[1] = updated_val | PC1_ON;
		buf[0] = CNTL1;
		err = kx023_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;

		acc->resume_state[RES_CNTL1] = buf[1];
		acc->sensitivity = sensitivity;
	}


	return err;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
			buf[0], buf[1], err);

	return err;
}

int kx023_acc_update_odr(struct kx023_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];
	
	u8 buf[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(kx023_acc_odr_table) - 1; i >= 0; i--) {
		if (kx023_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = kx023_acc_odr_table[i].mask;


    KX023_REM_PRINTK("%s poll_interval_ms %d register %x\n", __func__, poll_interval_ms, config[1]);
	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		buf[0] = CNTL1 ;
		buf[1] = acc->resume_state[RES_CNTL1]&PC1_OFF ;
		err = kx023_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
			
		config[0] = ODCNTL;
		err = kx023_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_ODCNTL] = config[1];
		
		buf[0] = CNTL1 ;
		buf[1] = acc->resume_state[RES_CNTL1]|(PC1_ON) ;
		err = kx023_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
	}

	return err;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}

static int kx023_acc_get_acceleration_rawdata(struct kx023_acc_data *acc,int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | XOUT_L);
	err = kx023_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]))>>4 ;
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]))>>4 ;
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]))>>4 ;

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

	#ifdef DEBUG
		printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
			KX023_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif
	
	return err;
}

s16 last_hw_d[3];
char lastflag;
static int kx023_acc_get_acceleration_data(struct kx023_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | XOUT_L);
	err = kx023_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	/*hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]))>>4 ;
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]))>>4 ;
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]))>>4 ;*/
	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0])) ;
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2])) ;
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4])) ;
	
	if(( hw_d[0] == last_hw_d[0])||( hw_d[1] == last_hw_d[1])||( hw_d[2] == last_hw_d[2]))
	    lastflag = 0 ;
	else 
		lastflag++ ;
		
	last_hw_d[0] = hw_d[0] ;
	last_hw_d[1] = hw_d[1] ;
	last_hw_d[2] = hw_d[2] ;
	
	hw_d[0] = last_hw_d[0]>>4 ;
	hw_d[1] = last_hw_d[1]>>4 ;
	hw_d[2] = last_hw_d[2]>>4 ;

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

    acc->lastraw.x = xyz[0]+lastflag%2;
    acc->lastraw.y = xyz[1]+lastflag%2;
    acc->lastraw.z = xyz[2]+lastflag%2;
	acc->data_valid|=0xf0;

    acc->lastdata.x = xyz[0] * 1000/acc->sensitivity * GRAVITY_EARTH/100;
    acc->lastdata.y = xyz[1] * 1000/acc->sensitivity * GRAVITY_EARTH/100;
    acc->lastdata.z = xyz[2] * 1000/acc->sensitivity * GRAVITY_EARTH/100;
	acc->data_valid|=0x0f;

	xyz[0] = xyz[0] * 1000/acc->sensitivity;     //   data * 100000  by xiaot
 	xyz[1] = xyz[1] * 1000/acc->sensitivity;     //   data * 100000  by xiaot
	xyz[2] = xyz[2] * 1000/acc->sensitivity;     //   data * 100000  by xiaot

	#ifdef DEBUG
	dev_dbg(&acc->client->dev, "%s read x=%d, y=%d, z=%d\n",
			KX023_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif
	return 0;
}

static void kx023_acc_report_values(struct kx023_acc_data *acc,
					int *xyz)
{
	int i_final_x=xyz[0] - acc->calibration_data.x;
	int i_final_y=xyz[1] - acc->calibration_data.y;
	int i_final_z=xyz[2] - acc->calibration_data.z;

	input_report_abs(acc->input_dev, ABS_X, i_final_x);
	input_report_abs(acc->input_dev, ABS_Y, i_final_y);
	input_report_abs(acc->input_dev, ABS_Z, i_final_z);
	input_sync(acc->input_dev);
}

/*  add for ts use acc data 
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
bool kx023_for_ts_judge_dir(void)
{
	int result = 0;
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *priv; 
	int acc[3] = {0};

    if(!client)
		return false;
	priv = i2c_get_clientdata(client); 
	wake_lock_timeout(&ts_judge_phone_direction_wakelock, HZ/2); 

    mutex_lock(&priv->lock);
	result = kx023_acc_get_acceleration_data(priv, acc);
	mutex_unlock(&priv->lock);
	if (result < 0)
	{
		dev_err(&priv->client->dev, "<<-GTP->>%s get_acceleration_data failed\n", __func__);
		return false;
	}	

	printk(KERN_ERR "<<-GTP-INFO->>%s data is X(%d) Y(%d) Z(%d)\n", __func__, acc[0],acc[1], acc[2]);

	if (acc[1] < Y_MAX_THRESHOLD && (acc[2] > Z_MIN_THRESHOLD && acc[2] < Z_MAX_THRESHOLD)) 
	{
		printk(KERN_ERR "<<-GTP-INFO->>%s The phone is handstand\n", __func__);
		return true;
	} else {
		printk(KERN_ERR "<<-GTP-INFO->>%s The phone is NOT handstand\n", __func__);
		return false;
	}

}

static int kx023_acc_enable(struct kx023_acc_data *acc)
{
	int err;

    KX023_REM_PRINTK("%s\n", __func__);
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = kx023_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}
    acc->data_valid=0;

	return 0;
}

static int kx023_acc_disable(struct kx023_acc_data *acc)
{
    KX023_REM_PRINTK("%s\n", __func__);
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		kx023_acc_device_power_off(acc);
	}
    acc->data_valid=0;

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return snprintf(buf, 8, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	err = kx023_acc_update_odr(acc, interval_ms);
	if(err >= 0)
	{
		acc->pdata->poll_interval = interval_ms;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	int range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->g_range ;
	switch(val) {
	case KX023_ACC_G_2G:
		range = 2;
		break;
	case KX023_ACC_G_4G:
		range = 4;
		break;
	case KX023_ACC_G_8G:
		range = 8;
		break;
	}
	mutex_unlock(&acc->lock);
	return snprintf(buf, 4, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	int range =2 ;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = KX023_ACC_G_2G;
			break;
		case 4:
			range = KX023_ACC_G_4G;
			break;
		case 8:
			range = KX023_ACC_G_8G;
			break;
		default:
			dev_err(&acc->client->dev, "invalid g range requested!\n");
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = kx023_acc_update_g_range(acc, range);
	if(err >= 0)
	{
		acc->pdata->g_range = range;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	KX023_REM_PRINTK("%s\n", __func__);
	if (val)
		kx023_acc_enable(acc);
	else
		kx023_acc_disable(acc);

	return size;
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = kx023_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = kx023_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = snprintf(buf, 8, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct kx023_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(poll_delay, 0664, attr_get_polling_rate,
			attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
	
#ifdef DEBUG
	__ATTR(reg_value, 0664, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0220, NULL, attr_addr_set),
#endif

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
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static irqreturn_t kx023_acc_irq_handler(int irq, void *dev_id)
{
	struct kx023_acc_data *acc =(struct kx023_acc_data *)dev_id;

    printk(KERN_ERR "%s\n", __func__);
    disable_irq_nosync(acc->irq1);
    schedule_work(&acc->irq1_work);
	return IRQ_HANDLED;
}

static void kx023_acc_irq_work(struct work_struct *work)
{
    struct kx023_acc_data *acc =container_of(work, struct kx023_acc_data, irq1_work);
	int state;
	//u8 buf[2];
	
    if(NULL==acc)
    {
        printk(KERN_ERR "%s null pointer\n", __func__);
        goto exit_err;
    }
	mutex_lock(&acc->lock);
    state = gpio_get_value(acc->pdata->gpio_int1);
    gpio_switch_setstate(state);            
    printk(KERN_ERR "%s gpio state %d\n", __func__, state);

	//gpio_switch_setstate(0);
	mutex_unlock(&acc->lock);
exit_err:
    ;
}

static void kx023_acc_input_work_func(struct work_struct *work)
{
	struct kx023_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work, struct kx023_acc_data, input_work);

	mutex_lock(&acc->lock);
	err = kx023_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		kx023_acc_report_values(acc, xyz);
  //  KX023_REM_PRINTK("%s poll_interval %d\n", __func__, acc->pdata->poll_interval);
	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

int kx023_acc_input_open(struct input_dev *input)
{
	struct kx023_acc_data *acc = input_get_drvdata(input);

	return kx023_acc_enable(acc);
}

void kx023_acc_input_close(struct input_dev *dev)
{
	struct kx023_acc_data *acc = input_get_drvdata(dev);

	kx023_acc_disable(acc);
}

static int kx023_acc_input_init(struct kx023_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, kx023_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = kx023_acc_input_open;
	acc->input_dev->close = kx023_acc_input_close;
	acc->input_dev->name = ACCEL_INPUT_DEV_NAME;

	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	/*	next is used for interruptA sources data if the case */
    input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
    input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


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

static void kx023_acc_input_cleanup(struct kx023_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int kx023_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct kx023_acc_data *acc = container_of(sensors_cdev,
		struct kx023_acc_data, cdev);
	int err;
    KX023_REM_PRINTK("%s delay_msec %d\n", __func__, delay_msec);
	
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = delay_msec;
	err = kx023_acc_update_odr(acc, delay_msec);
	mutex_unlock(&acc->lock);
	return err;
}

static int kx023_acc_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct kx023_acc_data *acc = container_of(sensors_cdev,
		struct kx023_acc_data, cdev);
	int err;
    KX023_REM_PRINTK("%s enable %d\n", __func__, enable);
	if (enable)
		err = kx023_acc_enable(acc);
	else
		err = kx023_acc_disable(acc);
	return err;
}

static int kx023_acc_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int kx023_acc_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver kx023_acc_pd_driver = {
	.probe  = kx023_acc_pd_probe,
	.remove = kx023_acc_pd_remove,    
	.driver = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device kx023_acc_pd_device = {
    .name = "gsensor",
    .id   = -1,
};

static int kx023_at_get_data(struct kx023_acc_data *acc, u8 wh_data)
{
	int err;
	int xyz[3] = { 0 };
	u8 buf[2];

	if(!atomic_read(&acc->enabled)||!(wh_data&acc->data_valid))
	{
	  
		buf[0] = CNTL1;
		buf[1] = acc->resume_state[RES_CNTL1]|PC1_ON|HIGH_CURRENT;
		err = kx023_acc_i2c_write(acc, buf, 1);
		if (err < 0)
		{
			dev_err(&acc->client->dev, "%s set register failed\n", __func__);
		}
		else
		{
			err = kx023_acc_get_acceleration_data(acc, xyz);
			if (err < 0)
				dev_err(&acc->client->dev, "%s get_acceleration_data failed\n", __func__);
		}

		#ifdef WRITE_BACK_THE_REGISTER
		buf[0] = CNTL1;
		buf[1] = acc->resume_state[RES_CNTL1];
		err = kx023_acc_i2c_write(acc, buf, 1);
		if (err < 0)
		{
		   dev_err(&acc->client->dev, "%s reset register failed\n", __func__);
		}
		#endif

	}
	return 0;
}

static ssize_t kx023_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *acc = i2c_get_clientdata(client);

	struct acceleration accel;

	mutex_lock(&acc->lock);
	kx023_at_get_data(acc, 0xf0);
	accel = acc->lastraw;
	acc->data_valid&=0x0f;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

static ssize_t kx023_final_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *acc = i2c_get_clientdata(client);

	struct acceleration accel;

	mutex_lock(&acc->lock);
	kx023_at_get_data(acc, 0x0f);
	accel = acc->lastdata;
	acc->data_valid&=0xf0;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d %d %d\n",accel.x, accel.y, accel.z);
}

static ssize_t sensor_calibration_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
// /*
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *kx023 = i2c_get_clientdata(client);
    struct calibrate_data value = {5,5,6,6}; 
	int i,j=0;
	char bufnew[30];
	
	//printk(KERN_INFO"  sensor_calibration\n");
	for(i=0;i<=30;i++)
	{	bufnew[i] = *(buf+i);
	//	printk(KERN_INFO"%c",*(buf+i));
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

//	printk(KERN_INFO"  sensor_calibration_show	 x=%d	\n",value.x);
//	printk(KERN_INFO"  sensor_calibration_show	 y=%d	\n",value.y);
//	printk(KERN_INFO"  sensor_calibration_show	 z=%d	\n",value.z);
//	printk(KERN_INFO"  sensor_calibration_show	 self_test=%d	\n",value.self_test);
	if((value.x > 120000 || value.x < -120000) || (value.y > 120000 || value.y < -120000) || (value.z > 120000 || value.z < -120000))
	{
		value.x = 0;
		value.y = 0;
		value.z = 0;
	}
	/*as accelerator IC has already been over through sleftest ,so ignore it. flyshine*/
	//value.self_test =  kx023->calibration_data.self_test;
	/*end flyshine*/
    mutex_lock(&kx023->data_mutex);

    kx023->calibration_data = value;

    mutex_unlock(&kx023->data_mutex);

    return count;
//	*/
//     return 0;
}

static ssize_t sensor_calibration_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *kx023 = i2c_get_clientdata(client);
    struct calibrate_data value;
	printk(KERN_INFO"  sensor_calibration_show	\n");

    mutex_lock(&kx023->data_mutex);

    value = kx023->calibration_data;

    mutex_unlock(&kx023->data_mutex);
	printk(KERN_INFO"  sensor_calibration_show   x=%d	%d\n",value.x, kx023->calibration_data.x);
		printk(KERN_INFO"  sensor_calibration_show   y=%d	%d\n",value.y, kx023->calibration_data.y);
		printk(KERN_INFO"  sensor_calibration_show   z=%d	%d\n",value.z, kx023->calibration_data.z);
		printk(KERN_INFO"  sensor_calibration_show   self_test=%d	%d\n",value.self_test, kx023->calibration_data.self_test);

    return sprintf(buf, "%d %d %d %d  ov\n", value.x,value.y,value.z,value.self_test);
}

static ssize_t sensor_calibratecmd_xy(struct kx023_acc_data *kx023)
{	
	int i,sumofx,sumofy,sumofz;
	struct acceleration accelnew;
	int * p_accelnew=(int*)&accelnew;
	int data_valid=0;
	unsigned long delay = delay_to_jiffies(atomic_read(&kx023->delay));
	sumofx = 0;
	sumofy = 0;
	sumofz = 0;
//	printk(KERN_INFO"  sensor_calibratecmd_xy 	\n");

	for (i=0;i<50;i++)
	{
		//kx023_measure(kx023, &accelnew);
		if(!kx023_acc_get_acceleration_data(kx023,p_accelnew))
		{
			sumofx = sumofx + accelnew.x;
			sumofy = sumofy + accelnew.y;
			sumofz = sumofz + accelnew.z - 1024;
			data_valid++;
		}
		mdelay(delay + 1);
	}
//	printk(KERN_INFO"  sensor_calibratecmd_xy   mdelay  = %lu	\n",delay);
	sumofx = sumofx / data_valid;
	sumofy = sumofy / data_valid;
	sumofz = sumofz / data_valid;
	if (sumofx > -100000 && sumofx < 100000)
		if (sumofy > -100000 && sumofy < 100000)
		{
			kx023->calibration_data.x = sumofx;
			kx023->calibration_data.y = sumofy;
			kx023->calibration_data.z = sumofz;
		}
	printk(KERN_INFO"  sensor_calibratecmd_x   =%d	%d\n",sumofx, kx023->calibration_data.x);
	printk(KERN_INFO"  sensor_calibratecmd_y   =%d	%d\n",sumofy, kx023->calibration_data.y);
	printk(KERN_INFO"  sensor_calibratecmd_z   =%d	%d\n",sumofz, kx023->calibration_data.z);
	
	return 0;
}

static ssize_t sensor_calibratecmd_z(struct kx023_acc_data *kx023)
{
	int i,sumofz;
	struct acceleration accelnew;
	int * p_accelnew =(int*)&accelnew;
	unsigned long delay = delay_to_jiffies(atomic_read(&kx023->delay));
	sumofz = 0;
//	printk(KERN_INFO"  sensor_calibratecmd_z\n");

	for (i=0;i<20;i++)
	{
		//kx023_measure(kx023, &accelnew);
		kx023_acc_get_acceleration_data(kx023,p_accelnew);
		sumofz = sumofz + accelnew.z;
		mdelay(delay + 1);
	}
	//printk(KERN_INFO"  sensor_calibratecmd_Z   mdelay  = %lu	\n",delay);
	sumofz = sumofz / 20;
	if (sumofz > -100000 && sumofz < 100000)
		kx023->calibration_data.z = sumofz;
	printk(KERN_INFO"  sensor_calibratecmd_z   =%d  \n",sumofz);

	
	return 0;
}

static ssize_t sensor_calibratecmd_selftest(struct kx023_acc_data *kx023)
{
	struct acceleration accel;
	int *p_accel=(int*)&accel;
	u8 data;
	int i,x_off_sum,y_off_sum,z_off_sum;
	int x_neg_sum,y_neg_sum,z_neg_sum;
	int x_pos_sum,y_pos_sum,z_pos_sum;
	int x_neg_change,y_neg_change,z_neg_change;
	int x_pos_change,y_pos_change,z_pos_change;
	
	x_off_sum=y_off_sum=z_off_sum=0;
	x_neg_sum=y_neg_sum=z_neg_sum=0;
	x_pos_sum=y_pos_sum=z_pos_sum=0;
	x_neg_change=y_neg_change=z_neg_change=0;
	x_pos_change=y_pos_change=z_pos_change=0;
	
	//self-test off
	data=0x00;          //disable self-test
	kx023_acc_register_write(kx023, &data,SELF_TEST,data);
	
	data=0x00;			//disable device
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	mdelay(40);
	
	data=ODR50F;			//config odr
	kx023_acc_register_write(kx023, &data,ODCNTL,data);
	
	data=0x10;			//config self-test
	kx023_acc_register_write(kx023, &data,INC1,data);
	
	data = HIGH_CURRENT ;   //enable device
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	data |= PC1_ON;
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	mdelay(40);
	
	for ( i=0; i < 5; i++ ){
		kx023_acc_get_acceleration_rawdata(kx023,p_accel);
		x_off_sum +=accel.x;
		y_off_sum +=accel.y;
		z_off_sum +=accel.z;
		mdelay(5);
	}
	
	//self-test pos
	data=0x00;          //disable self-test
	kx023_acc_register_write(kx023, &data,SELF_TEST,data);
	
	data=0x00;			//disable device
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	mdelay(40);
	
	data=ODR50F;			//config odr
	kx023_acc_register_write(kx023, &data,ODCNTL,data);
	
	data=0x12;			//config self-test
	kx023_acc_register_write(kx023, &data,INC1,data);
	
	data = HIGH_CURRENT ;   //enable device
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	
	data |= PC1_ON;
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	
	data = SELFTEST_KX023_ACC;   //enable self-test
	kx023_acc_register_write(kx023, &data,SELF_TEST,data);
	mdelay(40);
	
	for ( i=0; i < 5; i++ ){
		kx023_acc_get_acceleration_rawdata(kx023,p_accel);
		x_pos_sum +=accel.x;
		y_pos_sum +=accel.y;
		z_pos_sum +=accel.z;
		mdelay(5);
	}
	
	//self-test neg
	data=0x00;          //disable self-test
	kx023_acc_register_write(kx023, &data,SELF_TEST,data);
	
	data=0x00;			//disable device
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	mdelay(40);
	
	data=ODR50F;			//config odr
	kx023_acc_register_write(kx023, &data,ODCNTL,data);
	
	data=0x10;			//config self-test
	kx023_acc_register_write(kx023, &data,INC1,data);
	
	data = HIGH_CURRENT ;   //enable device
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	
	data |= PC1_ON;
	kx023_acc_register_write(kx023, &data,CNTL1,data);
	
	data = SELFTEST_KX023_ACC;   //enable self-test
	kx023_acc_register_write(kx023, &data,SELF_TEST,data);
	mdelay(40);
	
	for ( i=0; i < 5; i++ ){
		kx023_acc_get_acceleration_rawdata(kx023,p_accel);
		x_neg_sum +=accel.x;
		y_neg_sum +=accel.y;
		z_neg_sum +=accel.z;
		mdelay(5);
	}
	
	x_pos_change=abs( x_pos_sum/5 - x_off_sum/5);
	y_pos_change=abs( y_pos_sum/5 - y_off_sum/5);
	z_pos_change=abs( z_pos_sum/5 - z_off_sum/5);
	
	x_neg_change=abs( x_neg_sum/5 - x_off_sum/5);
	y_neg_change=abs( y_neg_sum/5 - y_off_sum/5);
	z_neg_change=abs( z_neg_sum/5 - z_off_sum/5);
	
	data = 0x00;
	kx023_acc_register_write(kx023, &data,SELF_TEST,data);
	mdelay(40);

	printk(KERN_INFO"sensor_calibratecmd_selftest off *** x=%d     y=%d   z=%d\n",
x_off_sum/5,y_off_sum/5,z_off_sum/5);
	printk(KERN_INFO"sensor_calibratecmd_selftest pos +++ x=%d     y=%d   z=%d\n",
x_pos_sum/5,y_pos_sum/5,z_pos_sum/5);
	printk(KERN_INFO"sensor_calibratecmd_selftest neg --- x=%d     y=%d   z=%d\n",
x_neg_sum/5,y_neg_sum/5,z_neg_sum/5);

	printk(KERN_INFO"sensor_calibratecmd_selftest  x_pos_change=%d \n",x_pos_change);
	printk(KERN_INFO"sensor_calibratecmd_selftest  y_pos_change=%d \n",y_pos_change);
	printk(KERN_INFO"sensor_calibratecmd_selftest  z_pos_change=%d \n",z_pos_change);
	printk(KERN_INFO"sensor_calibratecmd_selftest  x_neg_change=%d \n",x_neg_change);
	printk(KERN_INFO"sensor_calibratecmd_selftest  y_neg_change=%d \n",y_neg_change);
	printk(KERN_INFO"sensor_calibratecmd_selftest  z_neg_change=%d \n",z_neg_change);
			
	kx023->calibration_data.self_test = 2;
	kx023->calibrate_process=2;

	if ( ( (x_pos_change > SELFTEST_MIN) && (x_pos_change < SELFTEST_MAX) ) && 
		 ( (y_pos_change > SELFTEST_MIN) && (y_pos_change < SELFTEST_MAX) ) &&
		 ( (z_pos_change > SELFTEST_MIN) && (z_pos_change < SELFTEST_MAX) ) &&
		 ( (x_neg_change > SELFTEST_MIN) && (x_neg_change < SELFTEST_MAX) ) && 
		 ( (y_neg_change > SELFTEST_MIN) && (y_neg_change < SELFTEST_MAX) ) &&
		 ( (z_neg_change > SELFTEST_MIN) && (z_neg_change < SELFTEST_MAX) )		 )
		 {
				kx023->calibration_data.self_test = 1;
				//kx023->calibrate_process=2;
			}

	printk(KERN_INFO"sensor_calibratecmd_selftest  self_test=%d \n",kx023->calibration_data.self_test);

	return 0;
}

static ssize_t sensor_calibratecmd_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
// /*
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *kx023 = i2c_get_clientdata(client);
    int delay = atomic_read(&kx023->delay);
 //  printk(KERN_INFO"  sensor_calibratecmd_store  \n");
	unsigned long cmd = simple_strtoul(buf, NULL, 10);
    int err;
    u8 reg_back = CNTL1;
    u8 reg_buf[2];

	printk(KERN_ERR "  sensor_calibratecmd_store cmd= %d enable %d\n",(int)cmd, atomic_read(&kx023->enabled));

	if (atomic_read(&kx023->enabled)) 
	{                   
		cancel_delayed_work_sync(&kx023->input_work);
	}
	else
	{
		err = kx023_acc_i2c_read(kx023, &reg_back, 1);
		if (err < 0)
		{
			dev_warn(&kx023->client->dev,	"%s read failed\n", __func__);
			goto ERROR_EXIT;
		}
		else
		{
			reg_buf[0] = CNTL1;
			reg_buf[1] = reg_back & PC1_OFF;
			err = kx023_acc_i2c_write(kx023, reg_buf, 1);
			if (err < 0)
			{
			   dev_err(&kx023->client->dev, "%s set register failed\n", __func__);
			   goto ERROR_EXIT;
			}
			reg_buf[0] = CNTL1;
			reg_buf[1] = reg_back|HIGH_CURRENT;
			err = kx023_acc_i2c_write(kx023, reg_buf, 1);
			if (err < 0)
			{
			   dev_err(&kx023->client->dev, "%s set register failed\n", __func__);
			   goto ERROR_EXIT;
			}
		    reg_buf[0] = CNTL1;
			reg_buf[1] = reg_back|PC1_ON;
			err = kx023_acc_i2c_write(kx023, reg_buf, 1);
			if (err < 0)
			{
			   dev_err(&kx023->client->dev, "%s set register failed\n", __func__);
			   goto ERROR_EXIT;
			}
		}

				
	}


	if (cmd == 1)
	{	kx023->calibrate_process = 1;
		sensor_calibratecmd_xy(kx023);
		kx023->calibrate_process = 0;
	}
	else if (cmd == 2) 
	{	kx023->calibrate_process = 1;
		sensor_calibratecmd_z(kx023);
		kx023->calibrate_process = 0;
	}
        else if (cmd == 3) 
	{	kx023->calibrate_process = 1;
		sensor_calibratecmd_selftest(kx023);
		kx023->calibrate_process = 0;
	}
	else
	{
	}
	printk(KERN_ERR "  sensor_calibratecmd_store cmd= %d enable %d\n",(int)cmd, atomic_read(&kx023->enabled));

	if (atomic_read(&kx023->enabled)) 
	{ 
		
		schedule_delayed_work(&kx023->input_work, delay_to_jiffies(delay) + 1);                  
	}
	else
	{
		reg_buf[0] = CNTL1;
		reg_buf[1] = reg_back;
		err = kx023_acc_i2c_write(kx023, reg_buf, 1);
		if (err < 0)
		{
		   dev_err(&kx023->client->dev, "%s reset register failed\n", __func__);
		}
	}
ERROR_EXIT:
	return count;
//*/
//    return 0;
}

static ssize_t sensor_calibratecmd_show(struct device_driver *dev_driver, char *buf)
{///*
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *kx023 = i2c_get_clientdata(client);
    int value;
//	printk(KERN_INFO"  sensor_calibratecmd_show  \n");

	mutex_lock(&kx023->data_mutex);
	
    value = kx023->calibrate_process;

    mutex_unlock(&kx023->data_mutex);

    return sprintf(buf, "%d\n", value);
//*/
//    return 0;
}

static ssize_t sensor_eint_store(struct device_driver *dev_driver, const char *buffer, size_t count)
{

	int err;
	u8 buf[2];
	u8 temp;
//	int v_max;
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *kx023 = i2c_get_clientdata(client);

	unsigned int eint_state = simple_strtoul(buffer, NULL, 10);
	printk(KERN_INFO "sensor_eint_store eint_state = %x\n",eint_state);
	
	if (eint_state == 1)
	{
//		disable_irq_nosync(kx023->irq1);	
//		enable_irq(kx023->irq1);
		//gpio_switch_setstate(0);
		buf[0] = CNTL1;
		buf[1] = 0x00;
		
		err = kx023_acc_i2c_write(kx023, buf, 1);
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
			
		buf[1] = 0x42;
		err = kx023_acc_i2c_write(kx023, buf, 1);//enable INT1
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
		
		buf[0] = CNTL3;
		buf[1] = 0x06;
		err = kx023_acc_i2c_write(kx023, buf, 1); //sets the output data rate
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);											
		
		buf[0] = INC4;
		buf[1] = 0x02;
		err = kx023_acc_i2c_write(kx023, buf, 1); //set routing
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
		
		buf[0] = INC2;
		buf[1] = 0x3f;
		err = kx023_acc_i2c_write(kx023, buf, 1); //set triger source
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);			
		
		buf[0] = WUFC;
		buf[1] = 0x00;
		err = kx023_acc_i2c_write(kx023, buf, 1); //set duration
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
		
	 /*  v_max = abs(kx023->lastraw.x);
	   if(v_max < abs(kx023->lastraw.y))
	   {
		  v_max = abs(kx023->lastraw.y);
	   }
	   if(v_max < abs(kx023->lastraw.z))
	   {
		  v_max = abs(kx023->lastraw.z);
	   }
	   printk("[gsensor]sensor_eint_enable[%d],[%d],[%d],[%d]\n",kx023->lastraw.x,kx023->lastraw.y,kx023->lastraw.z,v_max);
	   v_max >>= 6;
	   v_max += 2;	
		if(v_max>=0xff)
		{
			v_max=0xff;
		}*/
		buf[0] = ATH;
		buf[1] = 2;
		err = kx023_acc_i2c_write(kx023, buf, 1); //set threshold
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
			
		buf[0] = INC1;
		buf[1] = 0x30;
		//buf[1] = 0x20;
		err = kx023_acc_i2c_write(kx023, buf, 1);  //latch INT1
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);	
		
		buf[0] = CNTL1;
		buf[1] = 0x42 | 0x80;
		err = kx023_acc_i2c_write(kx023, buf, 1);
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
        
        enable_irq(kx023->irq1);		
	}
	else if (eint_state == 0)
	{						
		buf[0] = INT_REL;
		err = kx023_acc_i2c_read(kx023, buf, 1);   //clear eint flag
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_read error %ud\n",buf[0]);	
		
		buf[0] = CNTL1;
		err = kx023_acc_i2c_read(kx023, buf, 1);
		if (err < 0)
		{
			printk(KERN_INFO "kx023_acc_i2c_read error %ud\n",buf[0]);
			buf[0] = CNTL1;
			buf[1] = 0x42;
		}
		else
        {
		    buf[1] = buf[0] & 0x7F;
			buf[0] = CNTL1;
		}
        temp = 	buf[1];
		
		err = kx023_acc_i2c_write(kx023, buf, 1);
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
		
		buf[0] = INC4;
		buf[1] = 0x00;
		err = kx023_acc_i2c_write(kx023, buf, 1); //clear routing
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);	
		
		buf[0] = INC2;
		buf[1] = 0x00;
		err = kx023_acc_i2c_write(kx023, buf, 1); //clear triger source
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);	
					
		buf[0] = CNTL1;
		buf[1] = temp & (~0x02);
		temp = buf[1];
		err = kx023_acc_i2c_write(kx023, buf, 1);  //disable INT1
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
		
		buf[1] = temp | 0x80;
		err = kx023_acc_i2c_write(kx023, buf, 1);
		if (err < 0)
			printk(KERN_INFO "kx023_acc_i2c_write error %ud\n",buf[0]);
			
		gpio_switch_setstate(0);
	}
	atomic_set(&(kx023->eintstatus),eint_state);  

	return count;
}

static ssize_t sensor_eint_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = kx023_acc_i2c_client;
	struct kx023_acc_data *kx023 = i2c_get_clientdata(client);
    int value;
//	printk(KERN_INFO"  sensor_calibratecmd_show  \n");
    value = atomic_read(&(kx023->eintstatus));  
    return sprintf(buf, "%d\n", value);
}

static DRIVER_ATTR(raw_data, 0644, kx023_raw_data_show, NULL);
static DRIVER_ATTR(data, 0644, kx023_final_data_show, NULL);
static DRIVER_ATTR(calibration, 0644, sensor_calibration_show, sensor_calibration_store);
static DRIVER_ATTR(calibratecmd, 0644, sensor_calibratecmd_show, sensor_calibratecmd_store);
static DRIVER_ATTR(set_eint, 0644, sensor_eint_show, sensor_eint_store);
//static DRIVER_ATTR(pock_mode, 0777, sensor_pockmode_show, sensor_pockmode_store);

static struct driver_attribute *kx023_acc_attr_list[] = {
	&driver_attr_raw_data,
	&driver_attr_data,
	&driver_attr_calibration,
    &driver_attr_calibratecmd,
    &driver_attr_set_eint,
//    &driver_attr_pock_mode,

};

static int kx023_acc_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(kx023_acc_attr_list)/sizeof(kx023_acc_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, kx023_acc_attr_list[idx])))
		{            
			printk("driver_create_file (%s) = %d\n", kx023_acc_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}




#ifdef CONFIG_OF
static int kx023_parse_dt(struct device *dev,
			struct kx023_acc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "kx,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "kx,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "kx,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kx,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis_map_y\n");
		return rc;
	} else {
		pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kx,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map-z\n");
		return rc;
	} else {
		pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kx,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
		case 2:
			pdata->g_range = KX023_ACC_G_2G;
			break;
		case 4:
			pdata->g_range = KX023_ACC_G_4G;
			break;
		case 8:
			pdata->g_range = KX023_ACC_G_8G;
			break;
		default:
			pdata->g_range = KX023_ACC_G_2G;
			break;
		}
	}

	rc = of_property_read_u32(np, "kx,negate-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read negate-x\n");
		return rc;
	} else {
		pdata->negate_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kx,negate-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read negate-y\n");
		return rc;
	} else {
		pdata->negate_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "kx,negate-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read negate-z\n");
		return rc;
	} else {
		pdata->negate_z = (u8)temp_val;
	}	
	if(of_property_read_bool(dev->of_node,"kx,gpio-irq"))
	{	
		printk(KERN_INFO "%s of_property_read_bool\n", __func__);
		rc = of_get_named_gpio_flags(dev->of_node,
				"kx,gpio-irq", 0, NULL);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to read interrupt pin1 number\n", __func__);
			pdata->gpio_int1 = -2;
		} else {
			pdata->gpio_int1 = rc;
		}
	}
	else
	{
        pdata->gpio_int1= -2;
        printk(KERN_ERR "%s no sensor,gpio-irq\n", __func__);
	}
	return 0;
}
#else
static int kx023_parse_dt(struct device *dev,
			struct kx023_acc_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static int kx023_acc_validate_pdata(struct kx023_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x,
			acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x,
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

static int kx023_acc_config_regulator(struct kx023_acc_data *obj, bool on)
{
	int rc = 0, i;
	struct sensor_regulator *kx023_acc_vreg_config=kx023_acc_vreg;
	int num_reg = sizeof(kx023_acc_vreg) / sizeof(struct sensor_regulator);
    KX023_REM_PRINTK("%s num_reg %d on %d\n", __func__, num_reg, on);
	if (on) 
    {
		for (i = 0; i < num_reg; i++) 
        {
			kx023_acc_vreg_config[i].vreg = regulator_get(&obj->client->dev, kx023_acc_vreg_config[i].name);
            
			if (IS_ERR(kx023_acc_vreg_config[i].vreg)) 
            {
				rc = PTR_ERR(kx023_acc_vreg_config[i].vreg);
				printk(KERN_ERR" %s:regulator get failed rc=%d\n", __func__, rc);
				kx023_acc_vreg_config[i].vreg = NULL;
			}

			if (regulator_count_voltages(kx023_acc_vreg_config[i].vreg) > 0) 
            {
                KX023_REM_PRINTK("%s min_uV %d max_uV %d\n", __func__, 
					kx023_acc_vreg_config[i].min_uV, kx023_acc_vreg_config[i].max_uV);
				rc = regulator_set_voltage(
					kx023_acc_vreg_config[i].vreg,
					kx023_acc_vreg_config[i].min_uV,
					kx023_acc_vreg_config[i].max_uV);
				if(rc) {
					printk(KERN_ERR" %s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(kx023_acc_vreg_config[i].vreg);
					kx023_acc_vreg_config[i].vreg = NULL;
				}
			}

			rc = regulator_enable(kx023_acc_vreg_config[i].vreg);
			if (rc) {
				printk(KERN_ERR" %s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(kx023_acc_vreg_config[i].vreg) > 0) 
                {
					regulator_set_voltage(
						kx023_acc_vreg_config[i].vreg, 0,
						kx023_acc_vreg_config[i].max_uV);
				}
				regulator_put(kx023_acc_vreg_config[i].vreg);
				kx023_acc_vreg_config[i].vreg = NULL;
			}
		}
	} 
    else 
    {
		i = num_reg;
	}
 
	return rc;
}

static int kx023_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct kx023_acc_data *acc;
	int err = -1;
	pr_info("%s: probe start.\n", KX023_ACC_DEV_NAME);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	acc = kzalloc(sizeof(struct kx023_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}


	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	mutex_init(&acc->data_mutex);


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
	memset(acc->pdata, 0 , sizeof(*acc->pdata));

	if (client->dev.of_node) {
		err = kx023_parse_dt(&client->dev, acc->pdata);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto exit_kfree_pdata;
		}
	} else if (client->dev.platform_data != NULL) {
		memcpy(acc->pdata, client->dev.platform_data,
			sizeof(*acc->pdata));
	} else {
		dev_err(&client->dev, "No valid platform data. exiting.\n");
		err = -ENODEV;
		goto exit_kfree_pdata;
	}
	
	err = kx023_acc_validate_pdata(acc);
		if (err < 0) {
			dev_err(&client->dev, "failed to validate platform data\n");
			goto exit_kfree_pdata;
		}
		
	kx023_acc_config_regulator(acc, true);
	msleep(3);

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}
	
	printk(KERN_INFO "%s %d %d\n", __func__, acc->pdata->gpio_int1, acc->irq1);
		
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	
	err = kx023_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);
	last_hw_d[2] = last_hw_d[1] = last_hw_d[0] = 0 ;
	lastflag = 0 ;
	
	err = kx023_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = kx023_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}
	
	err = kx023_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device KX023_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	
	acc->cdev = kx023_acc_cdev;
	acc->cdev.sensors_enable = kx023_acc_enable_set;
	acc->cdev.sensors_poll_delay = kx023_acc_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &acc->cdev);
	if (err) {
		dev_err(&client->dev,
			"class device create failed: %d\n", err);
	//	goto err_irq;
		goto err_remove_sysfs_int;
	}
	
	kx023_acc_device_power_off(acc);
	
	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

    if((err = platform_driver_register(&kx023_acc_pd_driver)))
	{
		printk(KERN_ERR "%s failed to register kx023_acc_driver err %d\n", __func__, err);
		goto err_unreg_sensor_class;
	}

    if((err = platform_device_register(&kx023_acc_pd_device)))
    {
		printk(KERN_ERR "%s failed to register kx023_acc_device err %d\n", __func__, err);
		goto err_free_driver;
    }

	if((err = kx023_acc_create_attr(&kx023_acc_pd_driver.driver)))
	{
		printk("%s KX023 create attribute err = %d\n", __func__, err);
		goto err_free_device;
	}
    kx023_acc_i2c_client=client;

	
	mutex_unlock(&acc->lock);
	
	printk(KERN_INFO "%s %d\n", __func__, acc->pdata->gpio_int1);
    if((0<=acc->pdata->gpio_int1))
    {
        err = gpio_request(acc->pdata->gpio_int1, "gpio-int");
        if (err < 0)
            goto err_free_gpio;
        
        err = gpio_direction_input(acc->pdata->gpio_int1);
        if (err < 0)
            goto err_free_gpio;
        INIT_WORK(&acc->irq1_work, kx023_acc_irq_work);
        
        acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
        if (acc->irq1 < 0) {
            err = acc->irq1;
            goto err_free_gpio;
        }
        /*IRQF_TRIGGER_HIGH IRQF_TRIGGER_LOW */
        err = request_irq( acc->irq1, kx023_acc_irq_handler,
            IRQF_TRIGGER_RISING|IRQF_ONESHOT, "gpio-irq", acc);
        if (err < 0)
            goto err_free_gpio;
    	err = enable_irq_wake(acc->irq1);
    	if (err < 0)
    		goto err_free_gpio;
    }
	
	pr_info("%s: probe haha.\n", KX023_ACC_DEV_NAME);
	
	wake_lock_init(&ts_judge_phone_direction_wakelock, WAKE_LOCK_SUSPEND, "Ts_judge_dir");
	acc_for_ts_judge_dir = kx023_for_ts_judge_dir;
	
	return 0;
	
err_free_gpio:
	gpio_free(acc->pdata->gpio_int1);
err_free_device:
	platform_device_unregister(&kx023_acc_pd_device);
err_free_driver:
	platform_driver_unregister(&kx023_acc_pd_driver);
err_unreg_sensor_class:
	sensors_classdev_unregister(&acc->cdev);
/*err_irq:
    if(acc->pdata->gpio_int2 >= 0)free_irq(acc->irq2, acc);
err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);*/
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	kx023_acc_input_cleanup(acc);
err_power_off:
	kx023_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	kfree(acc);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", KX023_ACC_DEV_NAME);
	return err;
}

static int kx023_acc_remove(struct i2c_client *client)
{
	struct kx023_acc_data *acc = i2c_get_clientdata(client);

	if (gpio_is_valid(acc->pdata->gpio_int1)) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
	//	destroy_workqueue(acc->irq1_work_queue);
	}
	
	sensors_classdev_unregister(&acc->cdev);
	kx023_acc_input_cleanup(acc);
	kx023_acc_device_power_off(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int kx023_acc_resume(struct i2c_client *client)
{
	struct kx023_acc_data *acc = i2c_get_clientdata(client);
    KX023_REM_PRINTK("%s acc->on_before_suspend %d\n", __func__, acc->on_before_suspend);
	if (acc->on_before_suspend)
		return kx023_acc_enable(acc);
	return 0;
}

static int kx023_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct kx023_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	KX023_REM_PRINTK("%s on_before_suspend %d\n", __func__, acc->on_before_suspend);
    

	return kx023_acc_disable(acc);
}
#else
#define kx023_acc_suspend	NULL
#define kx023_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id kx023_acc_id[]
		= { { KX023_ACC_DEV_NAME, 0 }, { }, };

static struct of_device_id kx023_acc_match_table[] = {
	{ .compatible = "kx023_acc", },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kx023_acc_id);

static struct i2c_driver kx023_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = KX023_ACC_DEV_NAME,
			.of_match_table = kx023_acc_match_table,
		  },
	.probe = kx023_acc_probe,
	.remove = kx023_acc_remove,
	.suspend = kx023_acc_suspend,
	.resume = kx023_acc_resume,
	.id_table = kx023_acc_id,
};

static int __init kx023_acc_init(void)
{
	return i2c_add_driver(&kx023_acc_driver);
}

static void __exit kx023_acc_exit(void)
{
	i2c_del_driver(&kx023_acc_driver);
	return;
}

module_init(kx023_acc_init);
module_exit(kx023_acc_exit);

MODULE_DESCRIPTION("kx023 digital accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Samuel Huo, STMicroelectronics");
MODULE_LICENSE("GPL");

