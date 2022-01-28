/* drivers/input/misc/hscdtd007a.c
 *
 * GeoMagneticField device driver (HSCDTD007A)
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
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
#define DEBUG                   //cqf  
#define VERBOSE_DEBUG           //cqf    
#ifdef ALPS_MAG_DEBUG
#define DEBUG 1
#endif
#include <linux/device.h>        //cqf add for log
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sensors.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include "hscdtd.h"
#include <linux/jiffies.h>


#define HSCDTD_DRIVER_NAME		"hscdtd007a"
#define HSCDTD_SENSOE_CLASS_NAME	"hscdtd007a-mag"
#define HSCDTD_INPUT_DEVICE_NAME	"compass"
#define HSCDTD_SNS_KIND			4
#define HSCDTD_LOG_TAG			"[HSCDTD], "
#define HSCDTD_LOG_TAGI			"Msensor.I [HSCDTD], "
#define HSCDTD_LOG_TAGE			"Msensor.E [HSCDTD], "


#define I2C_RETRIES		5


#define HSCDTD_CHIP_ID		0x1511

#define HSCDTD_STBA		0x0C
#define HSCDTD_INFO		0x0D
#define HSCDTD_XOUT		0x10
#define HSCDTD_YOUT		0x12
#define HSCDTD_ZOUT		0x14
#define HSCDTD_XOUT_H		0x11
#define HSCDTD_XOUT_L		0x10
#define HSCDTD_YOUT_H		0x13
#define HSCDTD_YOUT_L		0x12
#define HSCDTD_ZOUT_H		0x15
#define HSCDTD_ZOUT_L		0x14

#define HSCDTD_STATUS		0x18
#define HSCDTD_CTRL1		0x1B
#define HSCDTD_CTRL2		0x1C
#define HSCDTD_CTRL3		0x1D
#define HSCDTD_CTRL4		0x1E

#define HSCDTD_TCS_TIME		10000	/* Measure temp. of every 10 sec */
#define HSCDTD_DATA_ACCESS_NUM	6
#define HSCDTD_3AXIS_NUM	3
#define HSCDTD_INITIALL_DELAY	20
#define STBB_OUTV_THR		15000

#define HSCDTD_DELAY(us)	usleep_range(us, us)

/* Self-test resiter value */
#define HSCDTD_ST_REG_DEF	0x55
#define HSCDTD_ST_REG_PASS	0xAA
#define HSCDTD_ST_REG_X		0x01
#define HSCDTD_ST_REG_Y		0x02
#define HSCDTD_ST_REG_Z		0x04
#define HSCDTD_ST_REG_XYZ	0x07

/* Self-test error number */
#define HSCDTD_ST_OK		0x00
#define HSCDTD_ST_ERR_I2C	0x01
#define HSCDTD_ST_ERR_INIT	0x02
#define HSCDTD_ST_ERR_1ST	0x03
#define HSCDTD_ST_ERR_2ND	0x04
#define HSCDTD_ST_ERR_VAL	0x10

/* POWER SUPPLY VOLTAGE RANGE */
#define HSCDTD_VDD_MIN_UV	2850000
#define HSCDTD_VDD_MAX_UV	2850000
#define HSCDTD_VIO_MIN_UV	1800000
#define HSCDTD_VIO_MAX_UV	1800000

#define TAG  "Msensor "       //"Gsensor" "PAsensor" "GYsensor"
#define TAGI "Msensor.I "     //KERN_INFO 
#define TAGE "Msensor.E "     //KERN_ERR 
#define CONT_INT_TIME     50    //50ms 
#define POLL_DELAY_TIMES  2     //2X

#define CONT_INT_GATE   20 
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0
static int poll_delay_num = 0;
struct hscdtd_data {
	struct input_dev	*input;
	struct i2c_client	*i2c;
	struct delayed_work	work_read;
	struct mutex		lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend_h;
#endif
	struct sensors_classdev	cdev;
	unsigned int		kind;
	unsigned int		delay_msec;
	unsigned int		tcs_thr;
	unsigned int		tcs_cnt;
	bool			factive;
	bool			fsuspend;
	bool			fskip;
	int			matrix[9];

	struct	regulator		*vdd;
	struct	regulator		*vio;

    struct workqueue_struct	*wq_read;
    struct workqueue_struct *wq_report;
    struct delayed_work	work_report;
    struct	mutex sensor_data_mutex;
    int mag_rawdata[3];
    u8 data_valid;
    unsigned int		delay_report;
};

static struct sensors_classdev sensors_cdev = {
	.name = HSCDTD_SENSOE_CLASS_NAME,
	.vendor = "ALPS ELECTRIC CO., LTD.",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "4800.0",
	.resolution = "0.15",
	.sensor_power = "0.6",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10000,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct i2c_client *hscdtd007_mag_i2c_client=NULL;

/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int hscdtd_i2c_read(struct i2c_client *i2c, u8 *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxData,
		},
		{
			.addr	= i2c->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		 },
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->adapter->dev, HSCDTD_LOG_TAGE "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int hscdtd_i2c_write(struct i2c_client *i2c, u8 *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->adapter->dev, HSCDTD_LOG_TAGE "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


/*--------------------------------------------------------------------------
 * hscdtd function
 *--------------------------------------------------------------------------*/
static void hscdtd_convert_mount(
			struct hscdtd_data *hscdtd, int *xyz)
{
	volatile int i, j;
	int tmp[3];
	memcpy(tmp, xyz, sizeof(tmp));
	for (i = 0; i < 3; i++){
		xyz[i] = 0;
		for (j = 0; j < 3; j++)
           xyz[i] += hscdtd->matrix[j + i * 3] * tmp[j];
		}
}

static int hscdtd_get_magnetic_field_data(
			struct hscdtd_data *hscdtd, int *xyz)
{
	int err = -1;
	int i;
	u8  sx[HSCDTD_DATA_ACCESS_NUM];

	static int last_mag_x = 0;
	static int last_mag_y = 0;
	static int last_mag_z = 0;
	static int same_num_x = 0;
	static int same_num_y = 0;
	static int same_num_z = 0;
	
	sx[0] = HSCDTD_XOUT;
	err = hscdtd_i2c_read(hscdtd->i2c, sx,
	    HSCDTD_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	for (i = 0; i < HSCDTD_3AXIS_NUM; i++)
		xyz[i] = (int) ((short)((sx[2*i+1] << 8) | (sx[2*i])));

	hscdtd_convert_mount(hscdtd, xyz);

	/*dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);*/
	 //laiyangming: add for same data detect
    if(xyz[0]==last_mag_x)
    {
        if(same_num_x < SAME_DATA_GATE)
            same_num_x++;
        else
            printk(KERN_ERR TAGE "same data x error, x = %d\n",last_mag_x);
    }
    else
    {
        same_num_x = 0;
    }
	
	if(xyz[1]==last_mag_y)
    {
        if(same_num_y < SAME_DATA_GATE)
            same_num_y++;
        else
            printk(KERN_ERR TAGE "same data y error, y = %d\n",last_mag_y);
    }
    else
    {
        same_num_y = 0;
    }
	
	if(xyz[2]==last_mag_z)
    {
        if(same_num_z < SAME_DATA_GATE)
            same_num_z++;
        else
            printk(KERN_ERR TAGE "same data z error, z = %d\n",last_mag_z);
    }
    else
    {
        same_num_z = 0;
    }
	
	last_mag_x = xyz[0];
	last_mag_y = xyz[1];
	last_mag_z = xyz[2];

	return err;
}

static int hscdtd_soft_reset(struct hscdtd_data *hscdtd)
{
	int rc;
	u8 buf[2];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x80;
	rc = hscdtd_i2c_write(hscdtd->i2c, buf, 2);
	HSCDTD_DELAY(5000);

	return rc;
}

static int hscdtd_tcs_setup(struct hscdtd_data *hscdtd)
{
	int rc;
	u8 buf[2];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x02;
	rc = hscdtd_i2c_write(hscdtd->i2c, buf, 2);
	HSCDTD_DELAY(600);
	hscdtd->tcs_thr = HSCDTD_TCS_TIME / hscdtd->delay_msec;
	hscdtd->tcs_cnt = 0;

	return rc;
}

static int hscdtd_force_setup(struct hscdtd_data *hscdtd)
{
	u8 buf[2];

	buf[0] = HSCDTD_CTRL3;
	buf[1] = 0x40;

	return hscdtd_i2c_write(hscdtd->i2c, buf, 2);
}

static void hscdtd_measure_setup(
			struct hscdtd_data *hscdtd, bool en)
{
	u8 buf[2];

	if (en) {
		buf[0] = HSCDTD_CTRL1;
		buf[1] = 0x8A;
		hscdtd_i2c_write(hscdtd->i2c, buf, 2);
		HSCDTD_DELAY(10);

		buf[0] = HSCDTD_CTRL4;
		buf[1] = 0x90;
		hscdtd_i2c_write(hscdtd->i2c, buf, 2);
		HSCDTD_DELAY(10);

		hscdtd_tcs_setup(hscdtd);
		hscdtd_force_setup(hscdtd);
	} else
		hscdtd_soft_reset(hscdtd);
}

static void hscdtd_schedule_setup(
			struct hscdtd_data *hscdtd, bool en)
{
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s, en = %d\n", __func__, en);

	if (en)
    {   
    	queue_delayed_work(hscdtd->wq_read, &hscdtd->work_read,
    			(unsigned long)msecs_to_jiffies(hscdtd->delay_msec));
    	queue_delayed_work(hscdtd->wq_report, &hscdtd->work_report,
    			(unsigned long)msecs_to_jiffies(hscdtd->delay_report));
    }
	else
    {
		cancel_delayed_work(&hscdtd->work_read);
        cancel_delayed_work(&hscdtd->work_report);
    }   
}

static int hscdtd_get_hardware_data(
			struct hscdtd_data *hscdtd, int *xyz)
{
	int ret = 0;
	hscdtd_measure_setup(hscdtd, true);
	HSCDTD_DELAY(5000);
	ret = hscdtd_get_magnetic_field_data(hscdtd, xyz);
	hscdtd_measure_setup(hscdtd, false);
	if (ret)
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAGE
		    "measurement error.\n");
	return ret;
}

static int hscdtd_self_test_A(struct hscdtd_data *hscdtd)
{
	int rc = HSCDTD_ST_OK;
	u8 sx[2], cr1[1];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);

	/* Control resister1 backup  */
	cr1[0] = HSCDTD_CTRL1;
	if (hscdtd_i2c_read(hscdtd->i2c, cr1, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "Control resister1 value, %02X\n", cr1[0]);

	/* Move active mode (force state)  */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = 0x8A;
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	/* Get inital value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	hscdtd_i2c_read(hscdtd->i2c, sx, 1);
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(hscdtd->i2c, sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "STB reg. initial value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_DEF) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAGE
		    "Err: Initial value of STB reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_INIT;
		goto err_STBA;
	}

	/* do self-test-A  */
	sx[0] = HSCDTD_CTRL3;
	sx[1] = 0x10;
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(3000);

	/* Get 1st value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(hscdtd->i2c, sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "STB reg. 1st value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_PASS) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAGE
		    "Err: 1st value of STB reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_1ST;
		goto err_STBA;
	}
	HSCDTD_DELAY(3000);

	/* Get 2nd value of self-test-A register  */
	sx[0] = HSCDTD_STBA;
	if (hscdtd_i2c_read(hscdtd->i2c, sx, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "STB reg. 2nd value, %02X\n", sx[0]);
	if (sx[0] != HSCDTD_ST_REG_DEF) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAGE
		    "Err: 2nd value of STB reg. is %02X\n", sx[0]);
		rc = HSCDTD_ST_ERR_2ND;
	}

err_STBA:
	/* Resume */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = cr1[0];
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	return rc;
}

static int hscdtd_self_test_B(struct hscdtd_data *hscdtd)
{
	int rc = HSCDTD_ST_OK, xyz[3];
	u8 sx[2], cr1[1], cr4[1];

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);
	/* Control resister1 backup  */
	cr1[0] = HSCDTD_CTRL1;
	if (hscdtd_i2c_read(hscdtd->i2c, cr1, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "Control resister1 value, %02X\n", cr1[0]);

	/* Control resister4 backup  */
	cr4[0] = HSCDTD_CTRL4;
	if (hscdtd_i2c_read(hscdtd->i2c, cr4, 1))
		return HSCDTD_ST_ERR_I2C;
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAG "Control resister4 value, %02X\n", cr4[0]);

	/* Measurement sensor value */
	if (hscdtd_get_hardware_data(hscdtd, xyz))
		return HSCDTD_ST_ERR_I2C;

	/* Check output value */
	if ((xyz[0] <= -STBB_OUTV_THR) || (xyz[0] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_X;
	if ((xyz[1] <= -STBB_OUTV_THR) || (xyz[1] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_Y;
	if ((xyz[2] <= -STBB_OUTV_THR) || (xyz[2] >= STBB_OUTV_THR))
		rc |= HSCDTD_ST_REG_Z;
	if (rc)
		rc |= HSCDTD_ST_ERR_VAL;

	/* Resume */
	sx[0] = HSCDTD_CTRL1;
	sx[1] = cr1[0];
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	sx[0] = HSCDTD_CTRL4;
	sx[1] = cr4[0];
	if (hscdtd_i2c_write(hscdtd->i2c, sx, 2))
		return HSCDTD_ST_ERR_I2C;
	HSCDTD_DELAY(10);

	printk(KERN_INFO TAGI "hscdtd_self_test_B resume register\n");
	
	return rc;
}

static int hscdtd_register_init(struct hscdtd_data *hscdtd)
{
	int v[HSCDTD_3AXIS_NUM], ret = 0;
	u8  buf[2];
	u16 chip_info;

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);

	if (hscdtd_soft_reset(hscdtd)) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAGE
		    "Err. Can't execute software reset");
		return -1;
	}

	buf[0] = HSCDTD_INFO;
	ret = hscdtd_i2c_read(hscdtd->i2c, buf, 2);
	if (ret < 0)
		return ret;

	chip_info = (u16)((buf[1]<<8) | buf[0]);
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "chip_info, 0x%04X\n", chip_info);
	if (chip_info != HSCDTD_CHIP_ID) {
		dev_err(&hscdtd->i2c->adapter->dev, HSCDTD_LOG_TAGE
		    "chipID error(0x%04X).\n", chip_info);
		return -1;
	}

	mutex_lock(&hscdtd->lock);
	ret = hscdtd_get_hardware_data(hscdtd, v);
	hscdtd->kind = HSCDTD_SNS_KIND;
	mutex_unlock(&hscdtd->lock);
	dev_info(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "x:%d y:%d z:%d\n", v[0], v[1], v[2]);

	return ret;
}

static void hscdtd_enable_set(struct hscdtd_data *hscdtd, int en)
{
	mutex_lock(&hscdtd->lock);
	hscdtd->fskip = true;
	if (en) {
		if (!hscdtd->factive) {
			hscdtd_measure_setup(hscdtd, true);
			hscdtd_schedule_setup(hscdtd, true);
            hscdtd->data_valid=0;
		}
		hscdtd->factive = true;
		poll_delay_num = 0;
	} else {
		hscdtd_schedule_setup(hscdtd, false);
		hscdtd_measure_setup(hscdtd, false);
		hscdtd->factive = false;
	}
	mutex_unlock(&hscdtd->lock);
	
	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s, enable = %d\n", __func__, en);
}

static void hscdtd_delay_set(struct hscdtd_data *hscdtd, int delay)
{
    hscdtd->delay_report=delay;
	if (delay < 10)
		hscdtd->delay_report=delay = 10;
	else if (delay > 200)
		delay = 200;
	mutex_lock(&hscdtd->lock);
	hscdtd->tcs_cnt =
	    hscdtd->tcs_cnt * hscdtd->delay_msec / delay;
	hscdtd->tcs_thr = HSCDTD_TCS_TIME / delay;
	hscdtd->delay_msec = delay;
	mutex_unlock(&hscdtd->lock);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s, rate = %d (msec)\n",
	    __func__, hscdtd->delay_msec);
}

/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int hscdtd_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct hscdtd_data *hscdtd = i2c_get_clientdata(client);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);
		
	mutex_lock(&hscdtd->lock);
	hscdtd->fsuspend = true;
	hscdtd->fskip = true;
	hscdtd_schedule_setup(hscdtd, false);
	hscdtd_measure_setup(hscdtd, false);
	mutex_unlock(&hscdtd->lock);

	return 0;
}

static int hscdtd_resume(struct i2c_client *client)
{
	struct hscdtd_data *hscdtd = i2c_get_clientdata(client);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);
	if(hscdtd->factive) 
		printk(KERN_INFO TAGI "sensor on after suspend\n");

	mutex_lock(&hscdtd->lock);
	hscdtd->fskip = true;
	if (hscdtd->factive)
		hscdtd_measure_setup(hscdtd, true);
	if (hscdtd->factive)
		hscdtd_schedule_setup(hscdtd, true);
	hscdtd->fsuspend = false;
	mutex_unlock(&hscdtd->lock);

	poll_delay_num = 0;
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hscdtd_early_suspend(struct early_suspend *handler)
{
	struct hscdtd_data *hscdtd =
	    container_of(handler, struct hscdtd_data, early_suspend_h);
	hscdtd_suspend(hscdtd->i2c, PMSG_SUSPEND);
}

static void hscdtd_early_resume(struct early_suspend *handler)
{
	struct hscdtd_data *hscdtd =
	    container_of(handler, struct hscdtd_data, early_suspend_h);
	hscdtd_resume(hscdtd->i2c);
}
#endif


/*--------------------------------------------------------------------------
 * sysfs
 *--------------------------------------------------------------------------*/
static ssize_t hscdtd_kind_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (int)hscdtd->kind);
}

static ssize_t hscdtd_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (hscdtd->factive) ? 1 : 0);
}

static ssize_t hscdtd_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	int new_value;

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please resume device\n");
		return size;
	}

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "%s: invalid value %d\n",
		    __func__, *buf);
		return -EINVAL;
	}

	hscdtd_enable_set(hscdtd, new_value);

	return size;
}

static ssize_t hscdtd_axis_matrix_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		hscdtd->matrix[0], hscdtd->matrix[1], hscdtd->matrix[2],
		hscdtd->matrix[3], hscdtd->matrix[4], hscdtd->matrix[5],
		hscdtd->matrix[6], hscdtd->matrix[7], hscdtd->matrix[8]);
}

static ssize_t hscdtd_delay_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", hscdtd->delay_msec);
}

static ssize_t hscdtd_delay_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	int err;
	long new_delay;
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	err = strict_strtol(buf, 10, &new_delay);
	if (err < 0)
		return err;

	hscdtd_delay_set(hscdtd, (int)new_delay);

	return size;
}

static ssize_t hscdtd_self_test_A_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please resume device\n");
		return sprintf(buf, "%d\n", ret);
	}

	if (!hscdtd->factive) {
		mutex_lock(&hscdtd->lock);
		ret = hscdtd_self_test_A(hscdtd);
		mutex_unlock(&hscdtd->lock);
		dev_dbg(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGI "Self test-A result : %d\n", ret);
	} else
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please turn off sensor\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t hscdtd_self_test_B_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s\n", __func__);

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please resume device\n");
		return sprintf(buf, "%d\n", ret);
	}

	if (!hscdtd->factive) {
		mutex_lock(&hscdtd->lock);
		ret = hscdtd_self_test_B(hscdtd);
		mutex_unlock(&hscdtd->lock);
		dev_dbg(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGI "Self test-B result : %d\n", ret);
	} else
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please turn off sensor\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t hscdtd_get_hw_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int xyz[] = {0, 0, 0};
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);

	if (hscdtd->fsuspend) {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please resume device\n");
		return sprintf(buf, "%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
	}

	if (!hscdtd->factive) {
		mutex_lock(&hscdtd->lock);
		hscdtd_get_hardware_data(hscdtd, xyz);
		mutex_unlock(&hscdtd->lock);
		dev_dbg(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGI "%s: %d, %d, %d\n",
		    __func__, xyz[0], xyz[1], xyz[2]);
	} else
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "Error: Please turn off sensor\n");

	return sprintf(buf, "%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
}

#if 0
static ssize_t hscdtd_suspend_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (hscdtd->fsuspend) ? 1 : 0);
}

static ssize_t hscdtd_suspend_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct hscdtd_data *hscdtd = dev_get_drvdata(dev);
	int new_value;

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		dev_err(&hscdtd->i2c->adapter->dev,
		    HSCDTD_LOG_TAGE "%s: invalid value %d\n",
		    __func__, *buf);
		return -EINVAL;
	}

	dev_dbg(&hscdtd->i2c->adapter->dev,
	    HSCDTD_LOG_TAGI "%s, suspend = %d\n", __func__, new_value);

	if (new_value)
		hscdtd_suspend(hscdtd->i2c, PMSG_SUSPEND);
	else
		hscdtd_resume(hscdtd->i2c);

	return size;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(kind, S_IRUGO,
		hscdtd_kind_show, NULL),
	__ATTR(axis, S_IRUGO,
		hscdtd_axis_matrix_show, NULL),
	__ATTR(enable, S_IWUGO | S_IRUGO,
		hscdtd_enable_show, hscdtd_enable_store),
	__ATTR(delay, S_IWUGO | S_IRUGO,
		hscdtd_delay_show, hscdtd_delay_store),
#if 0
	__ATTR(suspend, S_IWUGO | S_IRUGO,
		hscdtd_suspend_show, hscdtd_suspend_store),
#endif
	__ATTR(self_test_A, S_IRUGO,
		hscdtd_self_test_A_show, NULL),
	__ATTR(self_test_B, S_IRUGO,
		hscdtd_self_test_B_show, NULL),
	__ATTR(get_hw_data, S_IRUGO,
		hscdtd_get_hw_data_show, NULL)
};

static int hscdtd_enable_sensors_class(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct hscdtd_data *hscdtd = container_of(sensors_cdev,
			struct hscdtd_data, cdev);

	hscdtd_enable_set(hscdtd, (int)enable);

	return 0;
}

static int hscdtd_delay_sensors_class(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct hscdtd_data *hscdtd = container_of(sensors_cdev,
			struct hscdtd_data, cdev);
	hscdtd_delay_set(hscdtd, (int)delay_msec);

	return 0;
}

static int hscdtd_create_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto out_sysfs;

	return 0;

out_sysfs:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, HSCDTD_LOG_TAGE "Unable to create interface\n");

	return -EIO;
}

static void hscdtd_remove_sysfs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static void hscdtd_pull_data(struct work_struct *work)
{
	ktime_t timestamp;
	struct hscdtd_data *hscdtd =
	    container_of(work, struct hscdtd_data, work_report.work);

	timestamp = ktime_get_boottime();
	mutex_lock(&hscdtd->sensor_data_mutex);
    if(hscdtd->data_valid)
    {
    	input_report_abs(hscdtd->input, ABS_X, hscdtd->mag_rawdata[0]);
    	input_report_abs(hscdtd->input, ABS_Y, hscdtd->mag_rawdata[1]);
    	input_report_abs(hscdtd->input, ABS_Z, hscdtd->mag_rawdata[2]);
    	input_event(hscdtd->input,EV_SYN, SYN_TIME_SEC,
    		ktime_to_timespec(timestamp).tv_sec);
    	input_event(hscdtd->input,EV_SYN, SYN_TIME_NSEC,
    		ktime_to_timespec(timestamp).tv_nsec);
    //	printk(KERN_INFO TAGI "%s read mag x=%d ,y=%d ,z=%d ,time_s=%ld, time_ns=%ld \n",
    //		__func__, akm->mag_rawdata[0],akm->mag_rawdata[1],akm->mag_rawdata[2], 
    //		   ktime_to_timespec(timestamp).tv_sec,ktime_to_timespec(timestamp).tv_nsec);
    	input_sync(hscdtd->input);    	
    }
    mutex_unlock(&hscdtd->sensor_data_mutex);
    if(hscdtd->factive)
	queue_delayed_work(hscdtd->wq_report, &hscdtd->work_report,
			(unsigned long)msecs_to_jiffies(hscdtd->delay_report));
}

/*--------------------------------------------------------------------------
 * work function
 *--------------------------------------------------------------------------*/
static void hscdtd_polling(struct work_struct *work)
{
	int xyz[HSCDTD_3AXIS_NUM];
	struct hscdtd_data *hscdtd =
	    container_of(work, struct hscdtd_data, work_read.work);
	
	//laiyangming: add for check poll delay long
    static unsigned long last_poll_time = 0;
    unsigned long poll_time;
    unsigned long diff;
    //laiyangming: add end
	
	mutex_lock(&hscdtd->lock);
	if (hscdtd->fsuspend) {
		mutex_unlock(&hscdtd->lock);
		return;
	}
	if (hscdtd->fskip)
		hscdtd->fskip = false;
	else {
		if (hscdtd->factive) {
			if (!hscdtd_get_magnetic_field_data(hscdtd, xyz)) {
                mutex_lock(&hscdtd->sensor_data_mutex);
                hscdtd->mag_rawdata[0]=xyz[0];
                hscdtd->mag_rawdata[1]=xyz[1];
                hscdtd->mag_rawdata[2]=xyz[2];
                hscdtd->data_valid=1;
                mutex_unlock(&hscdtd->sensor_data_mutex);
			}
			if (++hscdtd->tcs_cnt > hscdtd->tcs_thr)
				hscdtd_tcs_setup(hscdtd);
			hscdtd_force_setup(hscdtd);	/* For next polling */
		}
	}
	if (hscdtd->factive) {
    	queue_delayed_work(hscdtd->wq_read, &hscdtd->work_read,
    			(unsigned long)msecs_to_jiffies(hscdtd->delay_msec));
	}
	
	//laiyangming: add for check poll delay long  
    poll_time = jiffies;
    diff = (long)poll_time - (long)last_poll_time;
    if((diff *1000)/HZ > hscdtd->delay_msec * POLL_DELAY_TIMES)
    {
        if(poll_delay_num < POLL_DELAY_GATE)
            poll_delay_num++;
        else
            printk(KERN_ERR TAGE "poll delay long error:delay = %d,time = %ld\n",
					hscdtd->delay_msec,(diff *1000)/HZ); 
    }
    else
    {
        poll_delay_num = 0;
    }
    last_poll_time = poll_time;
	//laiyangming: add end
	
	mutex_unlock(&hscdtd->lock);
}


#ifdef CONFIG_OF
static int alps_compass_parse_dt(struct device *dev,
				struct hscdtd_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "alps,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, HSCDTD_LOG_TAGE "Unable to read hscdtd,layout\n");
		return rc;
	} else {
		pdata->layout = (char)temp_val;
	}

	/*if (of_property_read_bool(np, "alps,auto-report")) {
		pdata->auto_report = 1;
		pdata->use_int = 0;
	} else {
		pdata->auto_report = 0;
		if (of_property_read_bool(dev->of_node, "hscdtd,use-interrupt")) {
			pdata->use_int = 1;
			pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
					"hscdtd,gpio-int", 0, &pdata->int_flags);
		} else {
			pdata->use_int = 0;
		}
	}*/

	/*pdata->gpio_rstn = of_get_named_gpio_flags(dev->of_node,
			"hscdtd,gpio-rstn", 0, NULL);*/
	/*pdata->gpio_rstn = 0;*/

	return 0;
}
#else
static int alps_compass_parse_dt(struct device *dev,
				struct hscdtd_platform_data *pdata)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */

static int hscdtd_mag_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int hscdtd_mag_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver hscdtd_mag_pd_driver = {
	.probe  = hscdtd_mag_pd_probe,
	.remove = hscdtd_mag_pd_remove,    
	.driver = {
		.name  = "msensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device hscdtd_mag_pd_device = {
    .name = "msensor",
    .id   = -1,
};

#if 0
int hscd_self_test_A(void)
{
    u8 sx[2], cr1[1];

    cr1[0] = 0x1b/*HSCD_CTRL1*/;
    if (hscdtd_i2c_read(hscdtd007_mag_i2c_client, cr1, 1)) return 1;
    mdelay(1);

    /* Move active mode (force state)  */
    sx[0] = 0x1b/*HSCD_CTRL1*/;
    sx[1] = 0x8A;
    if (hscdtd_i2c_write(hscdtd007_mag_i2c_client,sx, 2)) 
		return 1;

    /* Get inital value of self-test-A register  */
    sx[0] = 0x0c/*HSCD_STB*/;
    hscdtd_i2c_read(hscdtd007_mag_i2c_client,sx, 1);
    mdelay(1);
    sx[0] = 0x0c/*HSCD_STB*/;
    if (hscdtd_i2c_read(hscdtd007_mag_i2c_client, sx, 1)) return 1;
    if (sx[0] != 0x55) {
        printk(KERN_ERR TAGE "error: self-test-A, initial value is %02X\n", sx[0]);
        return 2;
    }

    /* do self-test*/
    sx[0] = 0x1d/*HSCD_CTRL3*/;
    sx[1] = 0x10;
    if (hscdtd_i2c_write(hscdtd007_mag_i2c_client, sx, 2)) return 1;
    mdelay(3);

    /* Get 1st value of self-test-A register  */
    sx[0] = 0x0c/*HSCD_STB*/;
    if (hscdtd_i2c_read(hscdtd007_mag_i2c_client, sx, 1)) 
		return 1;
	if (sx[0] != 0xAA) {
        printk(KERN_ERR TAGE "error: self-test, 1st value is %02X\n", sx[0]);
        return 3;
    }
    mdelay(3);

    /* Get 2nd value of self-test register  */
    sx[0] = 0x0c/*HSCD_STB*/;
    if (hscdtd_i2c_read(hscdtd007_mag_i2c_client, sx, 1)) return 1;
    if (sx[0] != 0x55) {
        printk(KERN_ERR TAGE "error: self-test, 2nd value is %02X\n", sx[0]);
        return 4;
    }

    /* Resume */
    sx[0] = 0x1b/*HSCD_CTRL1*/;
    sx[1] = cr1[0];
    if (hscdtd_i2c_write(hscdtd007_mag_i2c_client, sx, 2)) return 1;

    return 0x0f;
}
#endif

static ssize_t hscdtd_mag_shipment_test(struct device_driver *ddri, char *buf)
{
	int value=0;
	struct hscdtd_data *hscdtd=NULL;

	if(hscdtd007_mag_i2c_client)hscdtd=i2c_get_clientdata(hscdtd007_mag_i2c_client);
	value=hscdtd_self_test_A(hscdtd);
	printk(KERN_ERR TAGE "%s value %d\n", __func__, value);
	value+=hscdtd_self_test_B(hscdtd);
	printk(KERN_ERR TAGE "%s value %d\n", __func__, value);
	/*value = hscd_self_test_A();
	printk(KERN_ERR "%s value %d\n", __func__, value);*/

	if(value)value=1;
	else value=0x0F;
	return sprintf(buf, "%d\n", value);
}

static ssize_t show_rawdata_value(struct device_driver *ddri, char *buf)
{

	/*char sensordata[SENSOR_DATA_SIZE];*/
	struct hscdtd_data *hscdtd=NULL;
	int data_tmp[HSCDTD_3AXIS_NUM];
	char strbuf[32];

	if(hscdtd007_mag_i2c_client)hscdtd=i2c_get_clientdata(hscdtd007_mag_i2c_client);

	hscdtd_get_hardware_data(hscdtd, data_tmp);
	
	/*sprintf(strbuf, "%d %d %d %d %d %d %d %d %d\n", sensordata[0],sensordata[1],sensordata[2],
		sensordata[3],sensordata[4],sensordata[5],sensordata[6],sensordata[7],sensordata[8]);*/

	sprintf(strbuf, "%d %d %d\n", data_tmp[0],data_tmp[1],data_tmp[2]);

	return sprintf(buf, "%s\n", strbuf);
}

static DRIVER_ATTR(selftest,S_IRUGO | S_IWUSR, hscdtd_mag_shipment_test, NULL);
static DRIVER_ATTR(data,  S_IRUGO, show_rawdata_value, NULL);

static struct driver_attribute *hscdtd_mag_attr_list[] = {
	&driver_attr_selftest,
	&driver_attr_data,
};

static int hscdtd_mag_create_attr(struct device_driver *driver) 
{
	int i;
	for (i = 0; i < ARRAY_SIZE(hscdtd_mag_attr_list); i++)
		if (driver_create_file(driver, hscdtd_mag_attr_list[i]))
			goto error;
	return 0;

error:
	for (i--; i >= 0; i--)
		driver_remove_file(driver, hscdtd_mag_attr_list[i]);
	printk(KERN_ERR TAGE "%s:Unable to create interface\n", __func__);
	return -1;
}

static int alps_matrix_config(struct hscdtd_platform_data *pdata, struct hscdtd_data *hscdtd)
{
	switch(pdata->layout)
    {
        case 0:
			hscdtd->matrix[0] = 1;
			hscdtd->matrix[4] = 1;
			hscdtd->matrix[8] = 1;
			break;
        case 1:
			hscdtd->matrix[1] = 1;
			hscdtd->matrix[3] = -1;
			hscdtd->matrix[8] = 1;
			break;
        case 2:
			hscdtd->matrix[0] = -1;
			hscdtd->matrix[4] = -1;
			hscdtd->matrix[8] = 1;
			break;
        case 3:
			hscdtd->matrix[1] = -1;
			hscdtd->matrix[3] = 1;
			hscdtd->matrix[8] = 1;
			break;
        case 4:
			hscdtd->matrix[0] = -1;
			hscdtd->matrix[4] = 1;
			hscdtd->matrix[8] = -1;
			break;
        case 5:
			hscdtd->matrix[1] = -1;
			hscdtd->matrix[3] = -1;
			hscdtd->matrix[8] = -1;
			break;
        case 6:
			hscdtd->matrix[0] = 1;
			hscdtd->matrix[4] = -1;
			hscdtd->matrix[8] = -1;
			break;
        case 7:
			hscdtd->matrix[1] = 1;
			hscdtd->matrix[3] = 1;
			hscdtd->matrix[8] = -1;
			break;
		default:
			hscdtd->matrix[0] = 1;
			hscdtd->matrix[4] = 1;
			hscdtd->matrix[8] = 1;

	}
	return 0;
}

#if 1 /*#ifdef HSCDTD_POWERON_CODE*/
static int hscdtd_compass_power_init(struct hscdtd_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				HSCDTD_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				HSCDTD_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev, HSCDTD_LOG_TAGE
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				HSCDTD_VDD_MIN_UV, HSCDTD_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev, HSCDTD_LOG_TAGE
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		rc = regulator_enable(data->vdd);
		if(rc)
		{
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev, HSCDTD_LOG_TAGE
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;

		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev, HSCDTD_LOG_TAGE
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				HSCDTD_VIO_MIN_UV, HSCDTD_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev, HSCDTD_LOG_TAGE
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}

		rc = regulator_enable(data->vio);
		if(rc)
		{
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev, HSCDTD_LOG_TAGE
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vio_set;
		
		}

	}

	return 0;

reg_vio_set:
	if (regulator_count_voltages(data->vio) > 0)
		regulator_set_voltage(data->vio, 0, HSCDTD_VIO_MAX_UV);
reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, HSCDTD_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
#endif

/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int hscdtd_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct hscdtd_data *hscdtd;
	struct hscdtd_platform_data *pdata;

	dev_dbg(&client->adapter->dev,
		HSCDTD_LOG_TAGI "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE "client not i2c capable\n");
		rc = -EIO;
		goto out_region;
	}

	hscdtd = kzalloc(sizeof(struct hscdtd_data), GFP_KERNEL);
	if (!hscdtd) {
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE
		    "failed to allocate memory for module data\n");
		rc = -ENOMEM;
		goto out_region;
	}
	hscdtd->i2c = client;
	i2c_set_clientdata(client, hscdtd);

	pdata = (struct hscdtd_platform_data *) client->dev.platform_data;
	if (!pdata) {
		dev_warn(&client->adapter->dev,
			 "Warning no platform data for hscdtd\n");
		memset(hscdtd->matrix, 0, sizeof(hscdtd->matrix));
		/*
		hscdtd->matrix[0] = 1;
		hscdtd->matrix[4] = 1;
		hscdtd->matrix[8] = 1;
		*/
		if (client->dev.of_node) {
			pdata = devm_kzalloc(
					&client->dev,
					sizeof(struct hscdtd_platform_data),
					GFP_KERNEL);
			if (!pdata) {
				dev_err(&client->dev, HSCDTD_LOG_TAGE "Failed to allcated memory\n");
				rc = -ENOMEM;
				goto err_devm;
			}
		}

		rc = alps_compass_parse_dt(&client->dev, pdata);
		if (rc) {
			dev_err(
			&client->dev, HSCDTD_LOG_TAGE
			"Unable to parse platform data err=%d\n",
			rc);
		}
		alps_matrix_config(pdata, hscdtd);
	} else {
		/*memcpy(hscdtd->matrix, pdata->axis,
			sizeof(hscdtd->matrix));*/

		dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "get platform_data");
	}

	mutex_init(&hscdtd->lock);

	hscdtd->factive = false;
	hscdtd->fsuspend = false;
	hscdtd->delay_msec = HSCDTD_INITIALL_DELAY;
	hscdtd->tcs_thr = HSCDTD_TCS_TIME / hscdtd->delay_msec;
	hscdtd->tcs_cnt = 0;

    #if 1 /*#ifdef HSCDTD_POWERON_CODE*/
    rc = hscdtd_compass_power_init(hscdtd, true);
	if (rc)
		goto out_kzalloc;
	msleep(3);
	#endif

	rc = hscdtd_register_init(hscdtd);
	if (rc) {
		rc = -EIO;
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE "hscdtd_register_init\n");
		goto out_compass_pwr_init;
	}
	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI
	    "initialize %s sensor\n", HSCDTD_DRIVER_NAME);

	hscdtd->input = input_allocate_device();
	if (!hscdtd->input) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE "input_allocate_device\n");
		goto out_compass_pwr_init;
	}
	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "input_allocate_device\n");

	input_set_drvdata(hscdtd->input, hscdtd);

	hscdtd->input->name		= HSCDTD_INPUT_DEVICE_NAME;
	hscdtd->input->id.bustype	= BUS_I2C;
	hscdtd->input->evbit[0]		= BIT_MASK(EV_ABS);

	input_set_abs_params(hscdtd->input, ABS_X, -16384, 16383, 0, 0);
	input_set_abs_params(hscdtd->input, ABS_Y, -16384, 16383, 0, 0);
	input_set_abs_params(hscdtd->input, ABS_Z, -16384, 16383, 0, 0);

	rc = input_register_device(hscdtd->input);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE "input_register_device\n");
		goto out_idev_allc;
	}
	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "input_register_device\n");

	INIT_DELAYED_WORK(&hscdtd->work_read, hscdtd_polling);
    hscdtd->wq_read=NULL;
    hscdtd->wq_read = alloc_workqueue("hscdtd_poll_work", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
    if(!hscdtd->wq_read)
    {
        printk(KERN_ERR TAGE "create read workquque failed\n");
        goto out_idev_reg;
    }

    INIT_DELAYED_WORK(&hscdtd->work_report, hscdtd_pull_data);
    hscdtd->wq_report = NULL;
    hscdtd->wq_report = create_singlethread_workqueue("hscdtd_pull_data_work");
    if (!hscdtd->wq_report) {
        printk(KERN_ERR TAGE "create report workquque failed\n");
        goto out_destroy_wq_read;
    }

    mutex_init(&hscdtd->sensor_data_mutex);
    
	rc = hscdtd_create_sysfs(&hscdtd->input->dev);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE "hscdtd_create_sysfs\n");
		goto out_data_mutex;
	}
	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "hscdtd_create_sysfs\n");

	hscdtd->cdev = sensors_cdev;
	hscdtd->cdev.sensors_enable = hscdtd_enable_sensors_class;
	hscdtd->cdev.sensors_poll_delay = hscdtd_delay_sensors_class;
	rc = sensors_classdev_register(&client->dev, &hscdtd->cdev);
	if (rc) {
		dev_err(&client->adapter->dev, HSCDTD_LOG_TAGE
		    "sensors_classdev_register\n");
		goto out_sysfs;
	}
	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "sensors_classdev_register\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	hscdtd->early_suspend_h.suspend = hscdtd_early_suspend;
	hscdtd->early_suspend_h.resume  = hscdtd_early_resume;
	register_early_suspend(&hscdtd->early_suspend_h);
	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "register_early_suspend\n");
#endif

    if((rc  = platform_driver_register(&hscdtd_mag_pd_driver)))
	{
		printk(KERN_ERR TAGE "%s failed to register hscdtd007_mag_driver err %d\n", __func__, rc);
		goto err_unreg_sensor_class/*err_free_irq2*/;
	}

    if((rc  = platform_device_register(&hscdtd_mag_pd_device)))
    {
		printk(KERN_ERR TAGE "%s failed to register hscdtd007_mag_device err %d\n", __func__, rc);
		goto err_free_driver;
    }

	if((rc  = hscdtd_mag_create_attr(&hscdtd_mag_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "%s lis3dh create attribute err = %d\n", __func__, rc);
		goto err_free_device;
	}
    hscdtd007_mag_i2c_client=client;
    hscdtd->data_valid=0;
    hscdtd->mag_rawdata[0]=hscdtd->mag_rawdata[1]=hscdtd->mag_rawdata[2]=0;

	dev_info(&client->adapter->dev,
	    HSCDTD_LOG_TAGI "detected %s geomagnetic field sensor\n",
	    HSCDTD_DRIVER_NAME);

	return 0;
err_free_device:
	platform_device_unregister(&hscdtd_mag_pd_device);
err_free_driver:
	platform_driver_unregister(&hscdtd_mag_pd_driver);
err_unreg_sensor_class:
	sensors_classdev_unregister(&hscdtd->cdev);
out_sysfs:
	hscdtd_remove_sysfs(&hscdtd->input->dev);
out_data_mutex:
    destroy_workqueue(hscdtd->wq_report);
    mutex_destroy(&hscdtd->sensor_data_mutex);
out_destroy_wq_read:
    destroy_workqueue(hscdtd->wq_read);
out_idev_reg:
	input_unregister_device(hscdtd->input);
out_idev_allc:
	input_free_device(hscdtd->input);
out_compass_pwr_init:
	hscdtd_compass_power_init(hscdtd, false);
out_kzalloc:
	mutex_destroy(&hscdtd->lock);
	devm_kfree(&client->dev, pdata);/*kfree(pdata);*/
err_devm:
	i2c_set_clientdata(client, NULL);
	kfree(hscdtd);
out_region:

	return rc;
}

static int hscdtd_remove(struct i2c_client *client)
{
	struct hscdtd_data *hscdtd = i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, HSCDTD_LOG_TAGI "%s\n", __func__);

	hscdtd_measure_setup(hscdtd, false);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&hscdtd->early_suspend_h);
#endif
	sensors_classdev_unregister(&hscdtd->cdev);
	hscdtd_remove_sysfs(&hscdtd->input->dev);
	input_unregister_device(hscdtd->input);
	input_free_device(hscdtd->input);
	mutex_destroy(&hscdtd->lock);
	kfree(hscdtd);

	return 0;
}


/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static const struct i2c_device_id HSCDTD_id[] = {
	{ HSCDTD_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id hscdtd007a_match_table[] = {
	{ .compatible = "alsp,hscdtd007a", },
	{ },
};

static struct i2c_driver hscdtd_driver = {
	.probe		= hscdtd_probe,
	.remove		= hscdtd_remove,
	.id_table	= HSCDTD_id,
	.driver		= {
		.owner	= THIS_MODULE,
		.name = HSCDTD_DRIVER_NAME,
		.of_match_table = hscdtd007a_match_table,
	},
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= hscdtd_suspend,
	.resume		= hscdtd_resume,
#endif
};

static int __init hscdtd_init(void)
{
	pr_debug(HSCDTD_LOG_TAGI "%s\n", __func__);
	return i2c_add_driver(&hscdtd_driver);
}

static void __exit hscdtd_exit(void)
{
	pr_debug(HSCDTD_LOG_TAGI "%s\n", __func__);
	i2c_del_driver(&hscdtd_driver);
}

module_init(hscdtd_init);
module_exit(hscdtd_exit);

MODULE_DESCRIPTION("ALPS Geomagnetic Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
