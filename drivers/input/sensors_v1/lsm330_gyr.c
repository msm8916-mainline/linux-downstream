/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name		: lsm330_gyr_sysfs.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Both authors are willing to be considered the
*			: contact and update points for the driver.
* Version		: V.1.0.2
* Date			: 2012/Oct/15
* Description		: LSM330 gyroscope driver
*
********************************************************************************
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
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include "lsm330.h"
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#endif
#include <linux/jiffies.h>

#define LSM330_GYR_ENABLED		1
#define LSM330_GYR_DISABLED		0

/** Maximum polled-device-reported rot speed value value in dps*/
#define FS_MAX				32768

/* lsm330 gyroscope registers */
#define WHO_AM_I			(0x0F)

#define SENSITIVITY_250			8750		/*	udps/LSB */
#define SENSITIVITY_500			17500		/*	udps/LSB */
#define SENSITIVITY_2000		70000		/*	udps/LSB */

#define CTRL_REG1			(0x20)    /* CTRL REG1 */
#define CTRL_REG2			(0x21)    /* CTRL REG2 */
#define CTRL_REG3			(0x22)    /* CTRL_REG3 */
#define CTRL_REG4			(0x23)    /* CTRL_REG4 */
#define CTRL_REG5			(0x24)    /* CTRL_REG5 */
#define	REFERENCE			(0x25)    /* REFERENCE REG */
#define	FIFO_CTRL_REG			(0x2E)    /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG			(0x2F)    /* FIFO SOURCE REGISTER */
#define	OUT_X_L				(0x28)    /* 1st AXIS OUT REG of 6 */

#define AXISDATA_REG			OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES			(0x00)
#define PM_OFF				(0x00)
#define PM_NORMAL			(0x08)
#define ENABLE_ALL_AXES			(0x07)
#define ENABLE_NO_AXES			(0x00)
#define BW00				(0x00)
#define BW01				(0x10)
#define BW10				(0x20)
#define BW11				(0x30)
#define ODR095				(0x00)  /* ODR =  95Hz */
#define ODR190				(0x40)  /* ODR = 190Hz */
#define ODR380				(0x80)  /* ODR = 380Hz */
#define ODR760				(0xC0)  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY				(0x08)
#define	I2_WTM				(0x04)
#define	I2_OVRUN			(0x02)
#define	I2_EMPTY			(0x01)
#define	I2_NONE				(0x00)
#define	I2_MASK				(0x0F)

/* CTRL_REG4 bits */
#define	FS_MASK				(0x30)
#define	BDU_ENABLE			(0x80)

/* CTRL_REG5 bits */
#define	FIFO_ENABLE			(0x40)
#define HPF_ENALBE			(0x11)

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK			(0xE0)
#define	FIFO_MODE_BYPASS		(0x00)
#define	FIFO_MODE_FIFO			(0x20)
#define	FIFO_MODE_STREAM		(0x40)
#define	FIFO_MODE_STR2FIFO		(0x60)
#define	FIFO_MODE_BYPASS2STR		(0x80)
#define	FIFO_WATERMARK_MASK		(0x1F)

#define FIFO_STORED_DATA_MASK		(0x1F)

#define I2C_AUTO_INCREMENT		(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1			0
#define	RES_CTRL_REG2			1
#define	RES_CTRL_REG3			2
#define	RES_CTRL_REG4			3
#define	RES_CTRL_REG5			4
#define	RES_FIFO_CTRL_REG		5
#define	RESUME_ENTRIES			6

/** Registers Contents */
#define WHOAMI_LSM330_GYR		0xD4  /* Expected content for WAI */

#define TAG  "GYsensor "       //"Msensor" "PAsensor" "Gsensor"
#define TAGI "GYsensor.I "     //KERN_INFO 
#define TAGE "GYsensor.E "     //KERN_ERR 
extern void print_vivo_init(const char* fmt, ...);
extern void print_vivo_main(const char* fmt, ...);

#define CONT_INT_TIME     50    //50ms 
#define POLL_DELAY_TIMES  10     //2X

#define CONT_INT_GATE   20 
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0
#define DEBUG 0

static int int1_gpio = LSM330_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = LSM330_GYR_DEFAULT_INT2_GPIO;
/* module_param(int1_gpio, int, S_IRUGO); */
static int poll_delay_num = 0;

struct lsm330_gyr_triple {
	short	x,	/* x-axis angular rate data. */
			y,	/* y-axis angluar rate data. */
			z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	2,	ODR760|BW10},
	{	3,	ODR380|BW01},
	{	6,	ODR190|BW00},
	{	11,	ODR095|BW00},
};

static struct lsm330_gyr_platform_data default_lsm330_gyr_pdata = {
	.fs_range = LSM330_GYR_FS_2000DPS,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 0,

	.poll_interval = 100,
	.min_interval = LSM330_GYR_MIN_POLL_PERIOD_MS, /* 2ms */

	.gpio_int1 = LSM330_GYR_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */

};

#define L3G4200D_DEFUALT_DELAY   10
#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)

static struct sensors_classdev lsm330_gyr_cdev = {
	.name = "lsm330_gyr",
	.vendor = "LSM",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
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

struct calibrate_data {
	short x;
	short y;
	short z;
	short self_test;
};

struct lsm330_gyr_status {
	struct i2c_client *client;
	struct lsm330_gyr_platform_data *pdata;

	struct mutex lock;

	ktime_t ktime_gyro;
	struct hrtimer timer_gyro;
	struct input_dev *input_dev;
//	struct delayed_work input_work;
	struct work_struct input_work;
	struct workqueue_struct *data_wq;

	int hw_initialized;
	atomic_t enabled;
	int use_smbus;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];

	u32 sensitivity;

	/* interrupt related */
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	/* fifo related */
	u8 watermark;
	u8 fifomode;

	struct sensors_classdev cdev;
	struct lsm330_gyr_triple last;
	struct calibrate_data calibration_data;
	int calibrate_process;
	struct mutex get_data_lock;
	u8 data_valid;
	u8 suspend_st;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
};


static int lsm330_gyr_i2c_read(struct lsm330_gyr_status *stat, u8 *buf,
								int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;
	unsigned int ii;

	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
			if (DEBUG)
				printk(KERN_INFO TAGI
					"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
					"command=0x%02x, buf[0]=0x%02x\n",
					ret, len, cmd , buf[0]);
		} 
		else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
			if (DEBUG)
			{
					printk(KERN_INFO TAGI
						"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
						"command=0x%02x\n",
						ret, len, cmd);
					for (ii = 0; ii < len; ii++)
						printk(KERN_INFO TAGI "buf[%d]=0x%02x,",
										ii, buf[ii]);

					printk(KERN_INFO "\n");
			}

		} else
			ret = -1;

		if (ret < 0) {
			printk(KERN_ERR TAGE
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lsm330_gyr_i2c_write(struct lsm330_gyr_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;
	unsigned int ii;
	
	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
		if (DEBUG)
			printk(KERN_INFO TAGI
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);

			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
			if (DEBUG)
			{
				printk(KERN_INFO TAGI
					"i2c_smbus_write_i2c_block_data: ret=%d, "
					"len:%d, command=0x%02x\n",
					ret, len, reg);

				for (ii = 0; ii < (len + 1); ii++)
					printk(KERN_INFO TAGI "value[%d]=0x%02x,",
									ii, buf[ii]);

				printk(KERN_INFO "\n");
			}
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}


static int lsm330_gyr_register_write(struct lsm330_gyr_status *stat, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lsm330_gyr_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

	return err;
}

static int lsm330_gyr_register_read(struct lsm330_gyr_status *stat, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lsm330_gyr_i2c_read(stat, buf, 1);
	return err;
}

static int lsm330_gyr_register_update(struct lsm330_gyr_status *stat, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330_gyr_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330_gyr_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}


static int lsm330_gyr_update_watermark(struct lsm330_gyr_status *stat,
								u8 watermark)
{
	int res = 0;
	u8 buf[2];
	u8 new_value;

	mutex_lock(&stat->lock);
	new_value = (watermark % 0x20);
	res = lsm330_gyr_register_update(stat, buf, FIFO_CTRL_REG,
			 FIFO_WATERMARK_MASK, new_value);
	if (res < 0) {
		printk(KERN_ERR TAGE "failed to update watermark\n");
		return res;
	}
	if(DEBUG)
		printk(KERN_INFO TAGI "%s new_value:0x%02x,watermark:0x%02x\n",
						__func__, new_value, watermark);

	stat->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_WATERMARK_MASK & new_value) |
		(~FIFO_WATERMARK_MASK &
				stat->resume_state[RES_FIFO_CTRL_REG]));
	stat->watermark = new_value;
	mutex_unlock(&stat->lock);
	return res;
}

static int lsm330_gyr_update_fifomode(struct lsm330_gyr_status *stat,
								u8 fifomode)
{
	int res;
	u8 buf[2];
	u8 new_value;

	new_value = fifomode;
	res = lsm330_gyr_register_update(stat, buf, FIFO_CTRL_REG,
					FIFO_MODE_MASK, new_value);
	if (res < 0) {
		printk(KERN_ERR TAGE "failed to update fifoMode\n");
		return res;
	}
	/*
	if(DEBUG)
		printk(KERN_INFO TAGI "new_value:0x%02x,prev fifomode:0x%02x\n",
				__func__, new_value, stat->fifomode);
	 */
	stat->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_MODE_MASK & new_value) |
		(~FIFO_MODE_MASK &
				stat->resume_state[RES_FIFO_CTRL_REG]));
	stat->fifomode = new_value;

	return res;
}

static int lsm330_gyr_fifo_reset(struct lsm330_gyr_status *stat)
{
	u8 oldmode;
	int res;

	oldmode = stat->fifomode;
	res = lsm330_gyr_update_fifomode(stat, FIFO_MODE_BYPASS);
	if (res < 0)
		return res;
	res = lsm330_gyr_update_fifomode(stat, oldmode);
	if (res >= 0)
	{
		if(DEBUG)
		printk(KERN_INFO TAGI "%s fifo reset to: 0x%02x\n",
							__func__, oldmode);
	}
	return res;
}

static int lsm330_gyr_fifo_hwenable(struct lsm330_gyr_status *stat,
								u8 enable)
{
	int res;
	u8 buf[2];
	u8 set = 0x00;
	if (enable)
		set = FIFO_ENABLE;
	res = lsm330_gyr_register_update(stat, buf, CTRL_REG5,
			FIFO_ENABLE, set);
	if (res < 0) {
		printk(KERN_ERR TAGE "fifo_hw switch to:0x%02x failed\n",
									set);
		return res;
	}
	stat->resume_state[RES_CTRL_REG5] =
		((FIFO_ENABLE & set) |
		(~FIFO_ENABLE & stat->resume_state[RES_CTRL_REG5]));
	if(DEBUG)
		printk(KERN_INFO TAGI "%s set to:0x%02x\n", __func__, set);
	return res;
}

static int lsm330_gyr_manage_int2settings(struct lsm330_gyr_status *stat,
								u8 fifomode)
{
	int res;
	u8 buf[2];
	bool enable_fifo_hw;
	bool recognized_mode = false;
	u8 int2bits = I2_NONE;

	switch (fifomode) {
	case FIFO_MODE_FIFO:
		recognized_mode = true;


		int2bits = (I2_WTM | I2_OVRUN);
		enable_fifo_hw = true;

		res = lsm330_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, int2bits);
		if (res < 0) {
			printk(KERN_ERR TAGE "%s : failed to update "
							"CTRL_REG3:0x%02x\n",
							__func__, fifomode);
			goto err_mutex_unlock;
		}
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			(~(I2_MASK) & stat->resume_state[RES_CTRL_REG3]));
		/* enable_fifo_hw = true; */
		break;

	case FIFO_MODE_BYPASS:
		recognized_mode = true;


		int2bits = I2_DRDY;

		res = lsm330_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, int2bits);
		if (res < 0) {
			printk(KERN_ERR TAGE "%s : failed to update"
						" to CTRL_REG3:0x%02x\n",
							__func__, fifomode);
			goto err_mutex_unlock;
		}
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
		enable_fifo_hw = false;
		break;

	default:
		recognized_mode = false;
		res = lsm330_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, I2_NONE);
		if (res < 0) {
			printk(KERN_ERR TAGE "%s : failed to update "
						"CTRL_REG3:0x%02x\n",
						__func__, fifomode);
			goto err_mutex_unlock;
		}
		enable_fifo_hw = false;
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & 0x00) |
			(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
		break;

	}
	if (recognized_mode) {
		res = lsm330_gyr_update_fifomode(stat, fifomode);
		if (res < 0) {
			printk(KERN_ERR TAGE "%s : failed to "
						"set fifoMode\n", __func__);
			goto err_mutex_unlock;
		}
	}
	res = lsm330_gyr_fifo_hwenable(stat, enable_fifo_hw);

err_mutex_unlock:

	return res;
}


static int lsm330_gyr_update_fs_range(struct lsm330_gyr_status *stat,
							u8 new_fs)
{
	int res ;
	u8 buf[2];

	u32 sensitivity;

	switch(new_fs) {
		case LSM330_GYR_FS_250DPS:
			sensitivity = SENSITIVITY_250;
			break;
		case LSM330_GYR_FS_500DPS:
			sensitivity = SENSITIVITY_500;
			break;
		case LSM330_GYR_FS_2000DPS:
			sensitivity = SENSITIVITY_2000;
			break;
		default:
			printk(KERN_ERR TAGE "invalid g range "
						"requested: %u\n", new_fs);
			return -EINVAL;
	}


	buf[0] = CTRL_REG4;

	res = lsm330_gyr_register_update(stat, buf, CTRL_REG4,
							FS_MASK, new_fs);

	if (res < 0) {
		printk(KERN_ERR TAGE "%s : failed to update fs:0x%02x\n",
							__func__, new_fs);
		return res;
	}
	stat->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		(~FS_MASK & stat->resume_state[RES_CTRL_REG4]));

	stat->sensitivity = sensitivity;
	return res;
}


static int lsm330_gyr_update_odr(struct lsm330_gyr_status *stat,
			unsigned int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2], buf[2];

	for(i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if(odr_table[i].poll_rate_ms <= poll_interval_ms)
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

    for(i=0;i<3;i++)
    {
	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	//if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lsm330_gyr_i2c_write(stat, config, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CTRL_REG1] = config[1];
	//}
    	buf[0] = CTRL_REG1;
    	lsm330_gyr_i2c_read(stat,buf,1);
        if(buf[0]==stat->resume_state[RES_CTRL_REG1])
        {
            break;
        }
        else
        {
            printk(KERN_ERR TAGI "%s rewrite odr register\n",__func__);
        }
    }

	return err;
}

/* gyroscope data readout */
static int lsm330_gyr_get_data(struct lsm330_gyr_status *stat,
			     struct lsm330_gyr_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s16 hw_d[3] = { 0 };
	static int same_num_x = 0;
	static int same_num_y = 0;
	static int same_num_z = 0;

	gyro_out[0] = (AXISDATA_REG);

	err = lsm330_gyr_i2c_read(stat, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] = (s16) (((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s16) (((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s16) (((gyro_out[5]) << 8) | gyro_out[4]);

	//hw_d[0] = hw_d[0] * stat->sensitivity;
	//hw_d[1] = hw_d[1] * stat->sensitivity;
	//hw_d[2] = hw_d[2] * stat->sensitivity;

	data->x = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	data->y = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	data->z = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));
	
    if(stat->last.x == data->x)
    {
		same_num_x++;
        if(same_num_x < SAME_DATA_GATE)
		{
         //   same_num_x++;
		}
        else if(same_num_x%10 == 1)
            printk(KERN_ERR TAGE "same data x error, x = %d,count = %d\n",data->x,same_num_x);
    }
    else
    {
        same_num_x = 0;
    }
	if(stat->last.y == data->y)
    {
		same_num_y++;
        if(same_num_y < SAME_DATA_GATE)
		{
         //   same_num_y++;
		}
        else if(same_num_y%10 == 1)
            printk(KERN_ERR TAGE "same data y error, y = %d,count = %d\n",data->y,same_num_y);
    }
    else
    {
        same_num_y = 0;
    }
	if(stat->last.z == data->z)
    {
		same_num_z++;
        if(same_num_z < SAME_DATA_GATE)
		{
         //   same_num_z++;
		}
        else if(same_num_z%10 == 1)
            printk(KERN_ERR TAGE "same data z error, z = %d,count = %d\n",data->z,same_num_z);
    }
    else
    {
        same_num_z = 0;
    }
	stat->last.x = data->x;
	stat->last.y = data->y;
	stat->last.z = data->z;
	stat->data_valid|=0xff;

	/*if(DEBUG)
		 printk(KERN_INFO TAGI "gyro_out: x = %d, y = %d, z = %d\n",
		data->x, data->y, data->z); */

	return err;
}

static void lsm330_gyr_report_values(struct lsm330_gyr_status *stat,
						struct lsm330_gyr_triple *data)
{
//	ktime_t timestamp;
//	timestamp = ktime_get_boottime();
	input_report_abs(stat->input_dev, ABS_X, data->x-stat->calibration_data.x);
	input_report_abs(stat->input_dev, ABS_Y, data->y-stat->calibration_data.y);
	input_report_abs(stat->input_dev, ABS_Z, data->z-stat->calibration_data.z);
	input_event(stat->input_dev,EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(stat->ktime_gyro).tv_sec);
	input_event(stat->input_dev,EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(stat->ktime_gyro).tv_nsec);
//	printk(KERN_INFO TAGI "%s read time_s=%ld, time_ns=%ld \n",__func__, 
//	     ktime_to_timespec(stat->ktime_gyro).tv_sec,ktime_to_timespec(stat->ktime_gyro).tv_nsec);
	input_sync(stat->input_dev);             
}

static int lsm330_gyr_hw_init(struct lsm330_gyr_status *stat)
{
	int err;
	u8 buf[6],i=0;

	printk(KERN_INFO TAGI "hw init\n");

	buf[0] = (CTRL_REG1);
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	buf[2] = stat->resume_state[RES_CTRL_REG2];
	buf[3] = stat->resume_state[RES_CTRL_REG3];
	buf[4] = stat->resume_state[RES_CTRL_REG4];
	buf[5] = stat->resume_state[RES_CTRL_REG5];

	err = lsm330_gyr_i2c_write(stat, buf, 5);
	if (err < 0)
		return err;

	buf[0] = (FIFO_CTRL_REG);
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lsm330_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
			return err;

    for(i=0;i<3;i++)
    {
        buf[0] = CTRL_REG1;
        lsm330_gyr_i2c_read(stat,buf,1);
        if(buf[0]==stat->resume_state[RES_CTRL_REG1])
        {
            break;
        }
        else
        {
            buf[0] = CTRL_REG1;
            buf[1] = stat->resume_state[RES_CTRL_REG1];
            err = lsm330_gyr_i2c_write(stat, buf, 1);
            printk(KERN_ERR TAGI "%s rewrite odr register\n",__func__);
        }
    }
	stat->hw_initialized = 1;

	return err;
}

static void lsm330_gyr_device_power_off(struct lsm330_gyr_status *stat)
{
	int err;
	u8 buf[2];

	printk(KERN_INFO TAGI "power off\n");

	buf[0] = (CTRL_REG1);
	buf[1] = (PM_OFF);
	err = lsm330_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
		printk(KERN_ERR TAGE "soft power off failed\n");

	if (stat->pdata->power_off) {
		/* disable_irq_nosync(state->irq1); */
		disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized) {
		/*if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);*/
		if (stat->pdata->gpio_int2 >= 0) {
			disable_irq_nosync(stat->irq2);
			printk(KERN_INFO TAGI
					"power off: irq2 disabled\n");
		}
		stat->hw_initialized = 0;
	}
}

static int lsm330_gyr_device_power_on(struct lsm330_gyr_status *stat)
{
	int err;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0)
			return err;
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}


	if (!stat->hw_initialized) {
		err = lsm330_gyr_hw_init(stat);
		if (err < 0) {
			lsm330_gyr_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		/*if (stat->pdata->gpio_int1 >= 0) {
			enable_irq(stat->irq1);
			printk(KERN_INFO TAGI
						"power on: irq1 enabled\n");
		} */
		printk(KERN_INFO TAGI "stat->pdata->gpio_int2 = %d\n",
						stat->pdata->gpio_int2);
		if (stat->pdata->gpio_int2 >= 0) {
			enable_irq(stat->irq2);
			printk(KERN_INFO TAGI
					"power on: irq2 enabled\n");
		}
	}

	return 0;
}

static int lsm330_gyr_enable(struct lsm330_gyr_status *stat)
{
	int err;
	ktime_t ktime;
	
	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {

		err = lsm330_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
        poll_delay_num = 0;
//		schedule_delayed_work(&stat->input_work,
//				msecs_to_jiffies(stat->pdata->poll_interval));
		ktime = ktime_set(0,stat->pdata->poll_interval * NSEC_PER_MSEC);
		hrtimer_start(&stat->timer_gyro, ktime, HRTIMER_MODE_REL);
	}

	return 0;
}

static int lsm330_gyr_disable(struct lsm330_gyr_status *stat)
{
	int err=0;
	if(DEBUG)
		printk(KERN_INFO TAGI "%s: stat->enabled = %d\n", __func__,
						atomic_read(&stat->enabled));

	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		hrtimer_cancel(&stat->timer_gyro);
	//	cancel_delayed_work_sync(&stat->input_work);
		lsm330_gyr_device_power_off(stat);
		if(DEBUG)
			printk(KERN_INFO TAGI "%s: cancel_delayed_work_sync "
						"result: %d", __func__, err);
	}
	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&stat->lock);
	err = lsm330_gyr_update_odr(stat, interval_ms);
	if(err >= 0)
		stat->pdata->poll_interval = interval_ms;
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int range = 0;
	u8 val;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;

	switch (val) {
	case LSM330_GYR_FS_250DPS:
		range = 250;
		break;
	case LSM330_GYR_FS_500DPS:
		range = 500;
		break;
	case LSM330_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&stat->lock);
	/* return sprintf(buf, "0x%02x\n", val); */
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 250:
		range = LSM330_GYR_FS_250DPS;
		break;
	case 500:
		range = LSM330_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM330_GYR_FS_2000DPS;
		break;
	default:
		printk(KERN_ERR TAGE "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm330_gyr_update_fs_range(stat, range);
	if (err >= 0)
		stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	printk(KERN_INFO TAGI "range set to: %lu dps\n", val);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_gyr_enable(stat);
	else
		lsm330_gyr_disable(stat);

	return size;
}

static ssize_t attr_watermark_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long watermark;
	int res;

	if (strict_strtoul(buf, 16, &watermark))
		return -EINVAL;

	res = lsm330_gyr_update_watermark(stat, watermark);
	if (res < 0)
		return res;

	return size;
}

static ssize_t attr_watermark_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int val = stat->watermark;
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_fifomode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long fifomode;
	int res;

	if (strict_strtoul(buf, 16, &fifomode))
		return -EINVAL;
	/* if (!fifomode)
		return -EINVAL; */

	if(DEBUG)
		printk(KERN_INFO TAGI "%s, got value:0x%02x\n", __func__, (u8)fifomode);

	mutex_lock(&stat->lock);
	res = lsm330_gyr_manage_int2settings(stat, (u8) fifomode);
	mutex_unlock(&stat->lock);

	if (res < 0)
		return res;
	return size;
}

static ssize_t attr_fifomode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	u8 val = stat->fifomode;
	return sprintf(buf, "0x%02x\n", val);
}

#if DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm330_gyr_i2c_write(stat, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm330_gyr_i2c_read(stat, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);

	stat->reg_addr = val;

	mutex_unlock(&stat->lock);

	return size;
}
#endif /* DEBUG */

static ssize_t sensor_writereg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);
	u8 addr,data;
	
	for(addr=0x0f;addr<0x39;addr++)
	{
		data = addr;
		lsm330_gyr_i2c_read(stat, &data, 1);		
		printk(KERN_INFO TAGI "%s reg_add:%x  reg_val:%x \n", __func__, addr, data);
	}
	/*TODO: error need to be managed */
	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t sensor_writereg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{	
	u8 data[2];
	int tem;	
	struct lsm330_gyr_status *stat = dev_get_drvdata(dev);	
	tem = simple_strtoul(buf, NULL, 16);	
	data[0] = (u8)((tem&0xff00)>>8);	
	data[1] = (u8)(tem&0x00ff);
	if(!lsm330_gyr_i2c_write(stat, data, 1))	
	{      
		printk(KERN_INFO TAGI "lsm330 reg==>[%x]/[%x]\n",data[0],data[1]);
	}	
	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0644, attr_polling_rate_show,
						attr_polling_rate_store),
	__ATTR(range, 0644, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0644, attr_enable_show, attr_enable_store),
	__ATTR(fifo_samples, 0644, attr_watermark_show, attr_watermark_store),
	__ATTR(fifo_mode, 0644, attr_fifomode_show, attr_fifomode_store),
#if DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
	__ATTR(get_reg, 0644, sensor_writereg_get,sensor_writereg_set),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	printk(KERN_ERR TAGE "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm330_gyr_report_triple(struct lsm330_gyr_status *stat)
{
	int err;
	struct lsm330_gyr_triple data_out;

	err = lsm330_gyr_get_data(stat, &data_out);
	if (err < 0)
		printk(KERN_ERR TAGE "get_gyroscope_data failed\n");
	else
		lsm330_gyr_report_values(stat, &data_out);
}

static void lsm330_gyr_irq2_fifo(struct lsm330_gyr_status *stat)
{
	int err;
	u8 buf[2];
	u8 int_source;
	u8 samples;
	u8 workingmode;
	u8 stored_samples;

	mutex_lock(&stat->lock);

	workingmode = stat->fifomode;


	if(DEBUG)
		printk(KERN_INFO TAGI "%s : fifomode:0x%02x\n", __func__,
								workingmode);


	switch (workingmode) {
	case FIFO_MODE_BYPASS:
	{
		if(DEBUG)
			printk(KERN_INFO TAGI "%s : fifomode:0x%02x\n", __func__,
							stat->fifomode);
		lsm330_gyr_report_triple(stat);
		break;
	}
	case FIFO_MODE_FIFO:
		samples = (stat->watermark)+1;
		if(DEBUG)
			printk(KERN_INFO TAGI 
			"%s : FIFO_SRC_REG init samples:%d\n",
							__func__, samples);
		err = lsm330_gyr_register_read(stat, buf, FIFO_SRC_REG);
		if (err < 0)
			printk(KERN_ERR TAGE
					"error reading fifo source reg\n");

		int_source = buf[0];
		if(DEBUG)
			printk(KERN_INFO TAGI "%s :FIFO_SRC_REG content:0x%02x\n",
							__func__, int_source);

		stored_samples = int_source & FIFO_STORED_DATA_MASK;
		if(DEBUG)
		{
			printk(KERN_INFO TAGI "%s : fifomode:0x%02x\n", __func__,
						stat->fifomode);
			printk(KERN_INFO TAGI "%s : samples:%d stored:%d\n",
				__func__, samples, stored_samples);
		}
		for (; samples > 0; samples--) {
			if(DEBUG)
				printk(KERN_INFO TAGI "%s : current sample:%d\n",
							__func__, samples);

			lsm330_gyr_report_triple(stat);

		}
		lsm330_gyr_fifo_reset(stat);
		break;
	}

	mutex_unlock(&stat->lock);
}

static irqreturn_t lsm330_gyr_isr2(int irq, void *dev)
{
	struct lsm330_gyr_status *stat = dev;
    static int cont_int_num = 0;
    static unsigned long last_int_time = 0;
    unsigned long int_time;
    unsigned long diff;
	disable_irq_nosync(irq);
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
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
	printk(KERN_INFO TAGI "%s %s: isr2 queued\n", LSM330_GYR_DEV_NAME, __func__);

	return IRQ_HANDLED;
}

static void lsm330_gyr_irq2_work_func(struct work_struct *work)
{

	struct lsm330_gyr_status *stat;
	stat = container_of(work, struct lsm330_gyr_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_gyr_irq2_XXX(stat); */
	lsm330_gyr_irq2_fifo(stat);
	/*  */
	printk(KERN_INFO TAGI"%s %s: IRQ2 served\n", LSM330_GYR_DEV_NAME, __func__);
	enable_irq(stat->irq2);
}

static void lsm330_gyr_input_work_func(struct work_struct *work)
{
	struct lsm330_gyr_status *stat;
	struct lsm330_gyr_triple data_out;
	int err;
    static unsigned long last_poll_time = 0;
    unsigned long poll_time;
    unsigned long diff;

	stat = container_of(work, struct lsm330_gyr_status, input_work);
	if(stat->suspend_st)
		goto suspend_exit;

	mutex_lock(&stat->lock);
	err = lsm330_gyr_get_data(stat, &data_out);
	if (err < 0)
		printk(KERN_ERR TAGE "get_gyroscope_data failed\n");
	else
		lsm330_gyr_report_values(stat, &data_out);

//	schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
//			stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);
    poll_time = jiffies;
    diff = (long)poll_time - (long)last_poll_time;
    if((diff *1000)/HZ > (stat->pdata->poll_interval) * POLL_DELAY_TIMES)
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
suspend_exit:
	;
}

/**
 * lsm330_gyro_timer_handle() - timer_gyro handle to schedule workerthread
 * @hrtimer: the hrtimer struct
 *
 * handle gets called based on the poll time to schedule worker thread
 */
static enum hrtimer_restart lsm330_gyro_timer_handle(struct hrtimer *hrtimer)
{
	ktime_t ktime;
	struct lsm330_gyr_status *stat;
	stat = container_of(hrtimer, struct lsm330_gyr_status, timer_gyro);

	ktime = stat->ktime_gyro =ktime_get_boottime();
	queue_work(stat->data_wq, &stat->input_work);
	ktime = ktime_set(0,stat->pdata->poll_interval * NSEC_PER_MSEC);
	hrtimer_start(&stat->timer_gyro, ktime, HRTIMER_MODE_REL);

       return HRTIMER_NORESTART;
}
 
int lsm330_gyr_input_open(struct input_dev *input)
{
	struct lsm330_gyr_status *stat = input_get_drvdata(input);
	printk(KERN_INFO TAGI "%s\n", __func__);
	return lsm330_gyr_enable(stat);
}

void lsm330_gyr_input_close(struct input_dev *dev)
{
	struct lsm330_gyr_status *stat = input_get_drvdata(dev);
	printk(KERN_INFO TAGI "%s\n", __func__);
	lsm330_gyr_disable(stat);
}

static int lsm330_gyr_validate_pdata(struct lsm330_gyr_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int) LSM330_GYR_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
	    stat->pdata->axis_map_y > 2 ||
	    stat->pdata->axis_map_z > 2) {
		printk(KERN_ERR TAGE
			"invalid axis_map value x:%u y:%u z%u\n",
			stat->pdata->axis_map_x,
			stat->pdata->axis_map_y,
			stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 ||
	    stat->pdata->negate_y > 1 ||
	    stat->pdata->negate_z > 1) {
		printk(KERN_ERR TAGE
			"invalid negate value x:%u y:%u z:%u\n",
			stat->pdata->negate_x,
			stat->pdata->negate_y,
			stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		printk(KERN_ERR TAGE
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int lsm330_gyr_input_init(struct lsm330_gyr_status *stat)
{
	int err = -1;
	printk(KERN_INFO TAGI "%s\n", __func__);

	INIT_WORK(&stat->input_work, lsm330_gyr_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE
			"input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lsm330_gyr_input_open;
	stat->input_dev->close = lsm330_gyr_input_close;
	stat->input_dev->name = LSM330_GYR_DEV_NAME;

	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);


	input_set_abs_params(stat->input_dev, ABS_X, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Y, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Z, -FS_MAX-1, FS_MAX, 0, 0);


	err = input_register_device(stat->input_dev);
	if (err) {
		printk(KERN_ERR TAGE
			"unable to register input polled device %s\n",
			stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm330_gyr_input_cleanup(struct lsm330_gyr_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static struct lsm330_gyr_status *gyro_data;
static int lsm330_gyr_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct lsm330_gyr_status *stat = container_of(sensors_cdev,
		struct lsm330_gyr_status, cdev);
	int err;
	
	if (enable)
		err = lsm330_gyr_enable(stat);
	else
		err = lsm330_gyr_disable(stat);

	return err;
}

static int lsm330_gyr_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct lsm330_gyr_status *stat = container_of(sensors_cdev,
		struct lsm330_gyr_status, cdev);
	int err;
	printk(KERN_INFO TAGI "%s %d\n", __func__, delay_msec);

	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = delay_msec;
	err = lsm330_gyr_update_odr(stat, delay_msec);
	mutex_unlock(&stat->lock);
	return err;
}

static int lsm330_gyr_at_get_data(struct lsm330_gyr_status *state, u8 wh_data)
{
	int err;
	struct lsm330_gyr_triple xyz = { 0 };
	u8 reg_back = CTRL_REG1;
	u8 buf[2];

    printk(KERN_INFO TAGI "%s\n", __func__);
	if(!atomic_read(&state->enabled)&&!(wh_data&state->data_valid))
	{
		err = lsm330_gyr_i2c_read(state, &reg_back, 1);
		if (err < 0)
		{
			printk(KERN_ERR TAGE "%s read failed\n", __func__);
		}
		else
		{
		    buf[0] = CTRL_REG1;
			buf[1] = (ENABLE_ALL_AXES|PM_NORMAL) | (odr_table[2].mask&0xF0);
			err = lsm330_gyr_i2c_write(state, buf, 1);
			if (err < 0)
			{
			   printk(KERN_ERR TAGE "%s set register failed\n", __func__);
			}
			else
			{
			    mdelay(10);
				err = lsm330_gyr_get_data(state, &xyz);
				if (err < 0)
					printk(KERN_ERR TAGE "%s get_acceleration_data failed\n", __func__);
			}
		}
		buf[0] = CTRL_REG1;
		buf[1] = reg_back;
		err = lsm330_gyr_i2c_write(state, buf, 1);
		if (err < 0)
		{
		   printk(KERN_ERR TAGE "%s reset register failed\n", __func__);
		}
	}
	return 0;
}

static ssize_t lsm330_gyro_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct lsm330_gyr_triple lsm330_raw;
	struct lsm330_gyr_status *stat = gyro_data;
    mutex_lock(&stat->lock);
	lsm330_gyr_at_get_data(stat, 0xf0);
    lsm330_raw = stat->last;
	stat->data_valid&=0x0f;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d %d %d\n", lsm330_raw.x, lsm330_raw.y, lsm330_raw.z);

}

static ssize_t lsm330_gyro_xyz_dps_show(struct device_driver *dev_driver, char *buf)
{
	struct lsm330_gyr_triple lsm330_dps;
	struct lsm330_gyr_status *stat = gyro_data;
	
	/*lsm330_gyr_get_data(stat,&lsm330_dps);*/
	printk(KERN_INFO TAGI "%s 1 %d, %d, %d\n",__func__, stat->last.x,stat->last.y,stat->last.z);
	mutex_lock(&stat->lock);
	lsm330_gyr_at_get_data(stat, 0x0f);
	lsm330_dps.x = stat->last.x*7/100;
	lsm330_dps.y = stat->last.y*7/100;
	lsm330_dps.z = stat->last.z*7/100;
	stat->data_valid&=0xf0;
	mutex_unlock(&stat->lock);
	printk(KERN_INFO TAGI "%s 2 %d, %d, %d\n",__func__, lsm330_dps.x,lsm330_dps.y,lsm330_dps.z);
	return sprintf(buf, "%d %d %d\n", lsm330_dps.x, lsm330_dps.y, lsm330_dps.z);
}

static ssize_t
lsm330_gyro_calibratexyz(struct lsm330_gyr_status *stat)
{
	int i,sumofx=0,sumofy=0,sumofz=0;
	int err=0;
	struct lsm330_gyr_triple lsm330new;
	unsigned long delay = delay_to_jiffies(L3G4200D_DEFUALT_DELAY);
	unsigned char data_valid=0;

	for (i=0;i<20;i++)
	{
		err=lsm330_gyr_get_data(stat,&lsm330new);
		if (err < 0)
			continue;
			
		printk(KERN_INFO TAGI "%d, %d, %d\n",lsm330new.x,lsm330new.y,lsm330new.z);
		sumofx += lsm330new.x;
		sumofy += lsm330new.y;
		sumofz += lsm330new.z;
		data_valid++;
		mdelay(delay + 1);
	}

    if(10<data_valid)
    {
		stat->calibration_data.x = sumofx / data_valid;
		stat->calibration_data.y = sumofy / data_valid;
		stat->calibration_data.z = sumofz / data_valid;
	}
	else
	{
        printk(KERN_ERR TAGE "%s read data failed %d\n", __func__, data_valid);
	}

	printk(KERN_INFO TAGI "%s lsm330_calibratecmd  x=%d y=%d z=%d\n",__func__,
		stat->calibration_data.x,
		stat->calibration_data.y,
		stat->calibration_data.z);
	
	return 0;

}


static ssize_t
lsm330_gyro_calibratecmd_selftest(struct lsm330_gyr_status *state)
{
	struct lsm330_gyr_triple lsm330old;
	struct lsm330_gyr_triple lsm330new;
	struct lsm330_gyr_triple lsm330tmp;
	int i,sumofx,sumofy,sumofz;
	u8 buf[2];
	u8 regbuf[5];
	unsigned long delay = delay_to_jiffies(L3G4200D_DEFUALT_DELAY);
	int err=0;
	unsigned char data_valid=0;

	sumofx = 0;
	sumofy = 0;
	sumofz = 0;
	//printk(KERN_INFO TAGI "lsm330_calibratecmd_selftest================\n");
	buf[0] = CTRL_REG1;
	lsm330_gyr_i2c_read(state,buf,1);
	regbuf[0] = buf[0];
	mdelay(2);
	 buf[0] = CTRL_REG2;
	lsm330_gyr_i2c_read(state,buf,1);
	regbuf[1] = buf[0];
	mdelay(2);
	 buf[0] = CTRL_REG3;
	lsm330_gyr_i2c_read(state,buf,1);
	regbuf[2] = buf[0];
	mdelay(2);
	 buf[0] = CTRL_REG4;
	lsm330_gyr_i2c_read(state,buf,1);
	regbuf[3] = buf[0];
	mdelay(2);
	 buf[0] = CTRL_REG5;
	lsm330_gyr_i2c_read(state,buf,1);
	regbuf[4] = buf[0];
	mdelay(2);
	
	buf[0] = CTRL_REG1;
	buf[1] = 0x6f;
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG2;
	buf[1] = 0x00;
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG3;
	buf[1] = 0x00;
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG4;
	buf[1] = 0xa0;
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG5;
	buf[1] = 0x02;
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(50);
	for (i=0;i<5;i++)
	{
		err=lsm330_gyr_get_data(state, &lsm330tmp);
		if (err < 0)
			continue;		
//		printk(KERN_INFO TAGI "Disable selftest: %d, %d, %d\n",lsm330tmp.x,lsm330tmp.y,lsm330tmp.z);
		sumofx = sumofx + lsm330tmp.x;
		sumofy = sumofy + lsm330tmp.y;
		sumofz = sumofz + lsm330tmp.z;
		data_valid++;
		mdelay(delay + 1);
	}
//	printk(KERN_INFO TAGI "sumo: %d, %d, %d\n",sumofx,sumofy,sumofx);
	lsm330old.x = sumofx / data_valid;
	lsm330old.y = sumofy / data_valid;
	lsm330old.z = sumofz / data_valid;
	mdelay(delay+10);
	printk(KERN_INFO TAGI "Disable selftest avg: %d, %d, %d\n",lsm330old.x,lsm330old.y,lsm330old.z);
	buf[0] = CTRL_REG4;
	buf[1] = 0xa2;
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(delay+50);
	sumofx = 0;
	sumofy = 0;
	sumofz = 0;
	data_valid=0;
	for (i=0;i<5;i++)
	{
		err=lsm330_gyr_get_data(state, &lsm330tmp);
		if (err < 0)
			continue;
	//	printk(KERN_INFO TAGI "Enable selftest: %d, %d, %d\n",lsm330tmp.x,lsm330tmp.y,lsm330tmp.z);
		sumofx = sumofx + lsm330tmp.x;
		sumofy = sumofy + lsm330tmp.y;
		sumofz = sumofz + lsm330tmp.z;
		data_valid++;
		mdelay(delay + 1);
	}
	lsm330new.x = sumofx / data_valid;
	lsm330new.y = sumofy / data_valid;
	lsm330new.z = sumofz / data_valid;
	printk(KERN_INFO TAGI "avg selftest: %d, %d, %d\n",lsm330new.x,lsm330new.y,lsm330new.z);
//	printk(KERN_INFO TAGI "sumo: %d, %d, %d\n",sumofx,sumofy,sumofx);
	mdelay(delay+10);

	buf[0] = CTRL_REG1;
	buf[1] = regbuf[0];
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG2;
	buf[1] = regbuf[1];
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG3;
	buf[1] = regbuf[2];
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG4;
	buf[1] = regbuf[3];
	lsm330_gyr_i2c_write(state,buf,1);
	mdelay(2);
	buf[0] = CTRL_REG5;
	buf[1] = regbuf[4];
	lsm330_gyr_i2c_write(state,buf,1);

	mdelay(delay+10);

	printk(KERN_INFO TAGI "jacken:=================x=%d,y=%d,z=%d==================\n",
		(lsm330new.x - lsm330old.x),(lsm330new.y - lsm330old.y),(lsm330new.z - lsm330old.z));
			
	state->calibration_data.self_test = 8;
	if (  (abs(lsm330new.x - lsm330old.x) > 2500) && (abs(lsm330new.x - lsm330old.x) < 12500)	)
		if (  (abs(lsm330new.y - lsm330old.y)> 2500) && (abs(lsm330new.y - lsm330old.y) < 12500)   )
			if (  (abs(lsm330new.z - lsm330old.z) > 2500) && (abs(lsm330new.z - lsm330old.z) < 12500)	)
				state->calibration_data.self_test = 1;

	return 0;
}


static ssize_t
lsm330_gyro_calibratecmd_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
	unsigned long cmd = simple_strtoul(buf, NULL, 10);
	struct lsm330_gyr_status *stat = gyro_data;
//	printk(KERN_INFO TAGI "  sensor_calibratecmd_store cmd= %d\n",(int)cmd);

    mutex_lock(&stat->lock);
	lsm330_gyr_device_power_on(gyro_data);
	/*lsm330_gyr_enable(stat);*/

	mdelay(400);
	
	if (cmd == 1)
	{	stat->calibrate_process = 1;
		lsm330_gyro_calibratexyz(stat);
		stat->calibrate_process = 0;
	}
	else if (cmd == 2) 
	{	stat->calibrate_process = 1;
		lsm330_gyro_calibratecmd_selftest(stat);
		stat->calibrate_process = 0;
	}
	else
	{
	}

	if(!atomic_read(&stat->enabled))lsm330_gyr_device_power_off(gyro_data);		 
	/*lsm330_gyr_disable(stat);*/
	mutex_unlock(&stat->lock);

	return count;

}



static ssize_t
lsm330_gyro_calibratecmd_show(struct device_driver *dev_driver, char *buf)
{
    int value;
	struct lsm330_gyr_status *stat = gyro_data;
	mutex_lock(&stat->lock);
    value = stat->calibrate_process;
    mutex_unlock(&stat->lock);
    return sprintf(buf, "%d\n", value);
}


static ssize_t
lsm330_gyro_calibration_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
	struct calibrate_data value = {5,5,6,6}; 
	int i,j=0;
	char bufnew[30];
	struct lsm330_gyr_status *stat = gyro_data;
	
//printk(KERN_INFO TAGI "  sensor_calibration\n");
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

//	printk(KERN_INFO TAGI "  sensor_calibration_show	 x=%d	\n",value.x);
//	printk(KERN_INFO TAGI "  sensor_calibration_show	 y=%d	\n",value.y);
//	printk(KERN_INFO TAGI "  sensor_calibration_show	 z=%d	\n",value.z);
//	printk(KERN_INFO TAGI "  sensor_calibration_show	 self_test=%d	\n",value.self_test);

	mutex_lock(&stat->lock);

	stat->calibration_data = value;

	mutex_unlock(&stat->lock);

	return count;

}



static ssize_t
lsm330_gyro_calibration_show(struct device_driver *dev_driver, char *buf)
{
	 struct calibrate_data value;
	 struct lsm330_gyr_status *stat = gyro_data;
	 //printk(KERN_INFO TAGI "  sensor_calibration_show	 \n");
	
	 mutex_lock(&stat->lock);	 
	 value = stat->calibration_data;
	 mutex_unlock(&stat->lock);
	 printk(KERN_INFO TAGI "lsm330_calibration_show %d,%d,%d\n",value.x,value.y,value.z);
	 return sprintf(buf, "%d %d %d %d  ov\n", value.x ,value.y	,value.z ,value.self_test);
}

static DRIVER_ATTR(rawdata, 0666, lsm330_gyro_raw_data_show, NULL);
static DRIVER_ATTR(get_dps, 0666, lsm330_gyro_xyz_dps_show, NULL);
static DRIVER_ATTR(calibration, 0666, lsm330_gyro_calibration_show, lsm330_gyro_calibration_store);
static DRIVER_ATTR(calibratecmd, 0666, lsm330_gyro_calibratecmd_show, lsm330_gyro_calibratecmd_store);

static struct driver_attribute *lsm330_gyro_attr_list[] = {
	&driver_attr_rawdata,
	&driver_attr_get_dps,
	&driver_attr_calibration,
    &driver_attr_calibratecmd,
};

static int lsm330_gyro_create_attr(struct device_driver *driver) 
{
	int i;
	for (i = 0; i < ARRAY_SIZE(lsm330_gyro_attr_list); i++)
		if (driver_create_file(driver, lsm330_gyro_attr_list[i]))
			goto error;
	return 0;

error:
	for (i--; i >= 0; i--)
		driver_remove_file(driver, lsm330_gyro_attr_list[i]);
	printk(KERN_ERR TAGE "%s:Unable to create interface\n", __func__);
	return -1;
}
/*
static int lsm330_gyro_remove_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(lsm330_gyro_attr_list)/sizeof(lsm330_gyro_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, lsm330_gyro_attr_list[idx]);           
		printk(KERN_INFO TAGI "%s (%s) = %d\n", __func__, lsm330_gyro_attr_list[idx]->attr.name, err);
	}    
	return err;
}
*/
static int lsm330_gyro_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int lsm330_gyro_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lsm330_gyro_pd_driver = {
	.probe  = lsm330_gyro_pd_probe,
	.remove = lsm330_gyro_pd_remove,    
	.driver = {
		.name  = "gyroscope",
		.owner = THIS_MODULE,
	}
};

struct platform_device lsm330_gyro_pd_device = {
    .name = "gyroscope",
    .id   = -1,
};

static struct sensor_regulator lsm330_vreg[] = {
	{NULL, "vdd", 2850000, 2850000},
	{NULL, "vddio", 1800000, 1800000},
};

#ifdef CONFIG_OF
static int lsm330_gyro_parse_dt(struct device *dev,
			struct lsm330_gyr_platform_data *pdata)
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
			pdata->fs_range = LSM330_GYR_FS_250DPS;
			break;
		case 4:
			pdata->fs_range = LSM330_GYR_FS_500DPS;
			break;
		case 8:
			pdata->fs_range = LSM330_GYR_FS_2000DPS;
			break;
		default:
			pdata->fs_range = LSM330_GYR_FS_2000DPS;
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

	pdata->gpio_int1=LSM330_GYR_DEFAULT_INT1_GPIO;
	pdata->gpio_int2=LSM330_GYR_DEFAULT_INT2_GPIO;
	pdata->init=NULL;
	pdata->exit=NULL;
	pdata->power_on=NULL;
	pdata->power_off=NULL;

	return 0;
}
#else
static int lsm330_gyro_parse_dt(struct device *dev,
			struct lsm330_gyr_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static int lsm330_gyro_config_regulator(struct lsm330_gyr_status *obj, bool on)
{
	int rc = 0, i;
	struct sensor_regulator *lsm330_acc_vreg_config=lsm330_vreg;
	int num_reg = sizeof(lsm330_acc_vreg_config) / sizeof(struct sensor_regulator);

	if (on) 
    {
		for (i = 0; i < num_reg; i++) 
        {
			lsm330_acc_vreg_config[i].vreg = regulator_get(&obj->client->dev, lsm330_acc_vreg_config[i].name);
            
			if (IS_ERR(lsm330_acc_vreg_config[i].vreg)) 
            {
				rc = PTR_ERR(lsm330_acc_vreg_config[i].vreg);
				printk(KERN_ERR TAGE " %s:regulator get failed rc=%d\n", __func__, rc);
				lsm330_acc_vreg_config[i].vreg = NULL;
			}

			if (regulator_count_voltages(lsm330_acc_vreg_config[i].vreg) > 0) 
            {
				rc = regulator_set_voltage(
					lsm330_acc_vreg_config[i].vreg,
					lsm330_acc_vreg_config[i].min_uV,
					lsm330_acc_vreg_config[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE " %s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(lsm330_acc_vreg_config[i].vreg);
					lsm330_acc_vreg_config[i].vreg = NULL;
				}
			}

			rc = regulator_enable(lsm330_acc_vreg_config[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE " %s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(lsm330_acc_vreg_config[i].vreg) > 0) 
                {
					regulator_set_voltage(
						lsm330_acc_vreg_config[i].vreg, 0,
						lsm330_acc_vreg_config[i].max_uV);
				}
				regulator_put(lsm330_acc_vreg_config[i].vreg);
				lsm330_acc_vreg_config[i].vreg = NULL;
			}
		}
	} 
    else 
    {
		i = num_reg;
	}
    
	return rc;
}


#if defined(CONFIG_FB)
static int lsm330_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct lsm330_gyr_status *stat = container_of(self, struct lsm330_gyr_status, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
        if (*blank == FB_BLANK_POWERDOWN)
        {
            if(atomic_read(&stat->enabled)) 
                printk(KERN_INFO TAGI "sensor on after screen off\n");
        }
	}
	return 0;
}
#endif

static int lsm330_gyr_probe(struct i2c_client *client,
					const struct i2c_device_id *devid)
{
	struct lsm330_gyr_status *stat;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;

	int err = -1;

	printk(KERN_INFO TAGI "probe start.\n");


	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (stat == NULL) {
		printk(KERN_ERR TAGE
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}


	/* Support for both I2C and SMBUS adapter interfaces. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR TAGE "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			printk(KERN_ERR TAGE "client using SMBUS\n");
		} else {
			err = -ENODEV;
			printk(KERN_ERR TAGE "client nor SMBUS capable\n");
			stat->use_smbus = 0;
			goto err0;
		}
	}
	
	print_vivo_init(TAGI "init 1\n");
	
	mutex_init(&stat->lock);
	mutex_init(&stat->get_data_lock);
	mutex_lock(&stat->lock);
	stat->client = client;

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		printk(KERN_ERR TAGE
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}

	if (client->dev.of_node) {
		err = lsm330_gyro_parse_dt(&client->dev, stat->pdata);
		if (err) {
			printk(KERN_ERR TAGE "Failed to parse device tree\n");
			err = -EINVAL;
			goto err1_1;
		}
	} else if (client->dev.platform_data == NULL) {
		default_lsm330_gyr_pdata.gpio_int1 = int1_gpio;
		default_lsm330_gyr_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_lsm330_gyr_pdata,
							sizeof(*stat->pdata));
		printk(KERN_ERR TAGE "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
						sizeof(*stat->pdata));
	}

	err = lsm330_gyr_validate_pdata(stat);
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to validate platform data\n");
		goto err1_1;
	}
	
	i2c_set_clientdata(client, stat);

    lsm330_gyro_config_regulator(stat, true);
	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			printk(KERN_ERR TAGE "init failed: %d\n", err);
			goto err1_1;
		}
	}
	
	print_vivo_init(TAGI "init 2\n");

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES
								| PM_NORMAL;
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

	err = lsm330_gyr_device_power_on(stat);
	if (err < 0) {
		printk(KERN_ERR TAGE "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm330_gyr_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		printk(KERN_ERR TAGE "update_fs_range failed\n");
		goto err2;
	}

	err = lsm330_gyr_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		printk(KERN_ERR TAGE "update_odr failed\n");
		goto err2;
	}
	
	print_vivo_init(TAGI "init 3\n");
	
	stat->data_wq = NULL;
	stat->data_wq = create_singlethread_workqueue("lsm330_gyr_data_work");
	if (!stat->data_wq) {
		printk(KERN_ERR TAGE "create workquque failed\n");
		goto err3;
	}
	err = lsm330_gyr_input_init(stat);
	if (err < 0)
		goto err3;
	
	printk(KERN_ERR TAGI  "hrtimer set start\n");
	hrtimer_init(&stat->timer_gyro, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	stat->timer_gyro.function = lsm330_gyro_timer_handle;
	input_set_events_per_packet(stat->input_dev, 100);
//	printk(KERN_ERR TAGI  "hrtimer set end\n");
	
	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		printk(KERN_ERR TAGE
			"%s device register failed\n", LSM330_GYR_DEV_NAME);
		goto err4;
	}

	lsm330_gyr_device_power_off(stat);

		
	print_vivo_init(TAGI "init 4\n");
	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);


	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		printk(KERN_INFO TAGI "%s: %s has set irq2 to irq:"
						" %d mapped on gpio:%d\n",
			LSM330_GYR_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);

		INIT_WORK(&stat->irq2_work, lsm330_gyr_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("lsm330_gyr_irq2_wq");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			printk(KERN_ERR TAGE "cannot create "
						"work queue2: %d\n", err);
			goto err5;
		}

		err = request_irq(stat->irq2, lsm330_gyr_isr2,
				IRQF_TRIGGER_HIGH, "lsm330_gyr_irq2", stat);

		if (err < 0) {
			printk(KERN_ERR TAGE "request irq2 failed: %d\n", err);
			goto err6;
		}
		disable_irq_nosync(stat->irq2);
	}

	stat->cdev = lsm330_gyr_cdev;
	stat->cdev.sensors_enable = lsm330_gyr_enable_set;
	stat->cdev.sensors_poll_delay = lsm330_gyr_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &stat->cdev);
	if (err) {
		printk(KERN_ERR TAGE
			"class device create failed: %d\n", err);
		goto err7;
	}

		
	print_vivo_init(TAGI "init 5\n");
	
    if((err = platform_driver_register(&lsm330_gyro_pd_driver)))
	{
		printk(KERN_ERR TAGE "%s failed to register lsm330_driver err %d\n", __func__, err);
		goto err_unreg_sensor_class;
	}

    if((err = platform_device_register(&lsm330_gyro_pd_device)))
    {
		printk(KERN_ERR TAGE "%s failed to register lsm330_device err %d\n", __func__, err);
		goto err_free_driver;
    }

	if((err = lsm330_gyro_create_attr(&lsm330_gyro_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "%s lsm330 create attribute err = %d\n", __func__, err);
		goto err_free_device;
	}
	gyro_data=stat;
	stat->suspend_st=0x00;

	mutex_unlock(&stat->lock);	
		
	print_vivo_init(TAGI "init 6\n");
	
#if defined(CONFIG_FB)
    stat->fb_notif.notifier_call = lsm330_fb_notifier_callback;
	fb_register_client(&stat->fb_notif);
#endif

	printk(KERN_INFO TAGI "%s haha\n", __func__);

	return 0;

/*err_destroy_sysfs:
	lsm330_gyro_remove_attr(&lsm330_gyro_pd_driver.driver);
*/
err_free_device:
	platform_device_unregister(&lsm330_gyro_pd_device);
err_free_driver:
	platform_driver_unregister(&lsm330_gyro_pd_driver);
err_unreg_sensor_class:
	sensors_classdev_unregister(&stat->cdev);
err7:
	if(stat->pdata->gpio_int2 >= 0)free_irq(stat->irq2, stat);
err6:
	destroy_workqueue(stat->irq2_work_queue);
err5:
	lsm330_gyr_device_power_off(stat);
	remove_sysfs_interfaces(&client->dev);
err4:
	lsm330_gyr_input_cleanup(stat);
err3:
	if (stat->data_wq)
		destroy_workqueue(stat->data_wq);
	lsm330_gyr_device_power_off(stat);
err2:
	if (stat->pdata->exit)
		stat->pdata->exit();
err1_1:
	mutex_unlock(&stat->lock);
	kfree(stat->pdata);
err1:
	kfree(stat);
err0:
		printk(KERN_ERR TAGE "%s: Driver Initialization failed\n",
							LSM330_GYR_DEV_NAME);
		return err;
}

static int lsm330_gyr_remove(struct i2c_client *client)
{
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);

	printk(KERN_INFO TAGI "driver removing\n");

/*
	if (stat->pdata->gpio_int1 >= 0)
	{
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}
*/
	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	if (atomic_cmpxchg(&stat->enabled, 1, 0))
		hrtimer_cancel(&stat->timer_gyro);
	//			cancel_delayed_work_sync(&stat->input_work);

	lsm330_gyr_disable(stat);
	if (stat->data_wq)
		destroy_workqueue(stat->data_wq);
	lsm330_gyr_input_cleanup(stat);

	remove_sysfs_interfaces(&client->dev);

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}

static int lsm330_gyr_suspend(struct device *dev)
{
	int err = 0;
#define SLEEP
#ifdef CONFIG_PM
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);
	u8 buf[2];
	
	stat->suspend_st=0xf0;
	printk(KERN_INFO TAGI "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);		

	//	cancel_delayed_work_sync(&stat->input_work);
	hrtimer_cancel(&stat->timer_gyro);

#ifdef SLEEP
		err = lsm330_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_NO_AXES | PM_NORMAL));
#else
		err = lsm330_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_OFF);
#endif /*SLEEP*/
		mutex_unlock(&stat->lock);
	}
#endif /*CONFIG_PM*/
	return err;
}

static int lsm330_gyr_resume(struct device *dev)
{
	int err = 0;
#ifdef CONFIG_PM
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm330_gyr_status *stat = i2c_get_clientdata(client);
	u8 buf[2];
	ktime_t ktime;
	
	printk(KERN_INFO TAGI "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
        printk(KERN_INFO TAGE "sensor on after suspend\n");
		mutex_lock(&stat->lock);
        poll_delay_num = 0;
//			schedule_delayed_work(&stat->input_work,
//					msecs_to_jiffies(stat->
//							pdata->poll_interval));
		ktime = ktime_set(0,stat->pdata->poll_interval * NSEC_PER_MSEC);
		hrtimer_start(&stat->timer_gyro, ktime, HRTIMER_MODE_REL);

#ifdef SLEEP
		err = lsm330_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_ALL_AXES | PM_NORMAL));
#else
		err = lsm330_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_NORMAL);
#endif
		mutex_unlock(&stat->lock);

	}
	stat->suspend_st=0x00;
#endif /*CONFIG_PM*/
	return err;
}


static const struct i2c_device_id lsm330_gyr_id[] = {
	{ LSM330_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, lsm330_gyr_id);

static struct of_device_id lsm330gyro_match_table[] = {
	{ .compatible = "lsm330_gyro", },
	{ },
};

static const struct dev_pm_ops lsm330_gyr_pm = {
	.suspend = lsm330_gyr_suspend,
	.resume = lsm330_gyr_resume,
};

static struct i2c_driver lsm330_gyr_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM330_GYR_DEV_NAME,
			.pm = &lsm330_gyr_pm,
			.of_match_table = lsm330gyro_match_table,
	},
	.probe = lsm330_gyr_probe,
	.remove = lsm330_gyr_remove,
	.id_table = lsm330_gyr_id,

};

static int __init lsm330_gyr_init(void)
{

	printk(KERN_INFO TAGI "%s: gyroscope sysfs driver init\n", LSM330_GYR_DEV_NAME);

	return i2c_add_driver(&lsm330_gyr_driver);
}

static void __exit lsm330_gyr_exit(void)
{

	printk(KERN_INFO TAGI "%s exit\n", LSM330_GYR_DEV_NAME);
	i2c_del_driver(&lsm330_gyr_driver);
	return;
}

late_initcall(lsm330_gyr_init);
module_exit(lsm330_gyr_exit);

MODULE_DESCRIPTION("lsm330 gyroscope driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
