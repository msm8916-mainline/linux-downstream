/******************************************************************************
 * isl29147.h - Linux kernel module for Intersil isl29147 ambient light sensor
 *				and proximity sensor
 *
 * Copyright 2008-2012 Intersil Inc..
 *
 * DESCRIPTION:
 *	- This is the linux driver for isl29147.
 *		Kernel version 3.0.8
 *
 * modification history
 * --------------------
 * V1.3	  2013/05/06, Shouxian Chen(Simon Chen) for isl29147.
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * For sys/class/sensors class
 */
#include <linux/sensors.h>
/* QCI End: ALT Sensor Daniel Wang 20140709 */

//#define DRIVER_DEBUG 
#ifdef DRIVER_DEBUG 
# define LOGD(fmt, args...) printk(KERN_ERR "[Shengwei] %s: " fmt, __FUNCTION__, ## args)
#else 
# define LOGD(fmt, args...) 
#endif 

#define LIGHT_CONF_FILE "/info2/sensors/light"
#define PROX_CONF_FILE "/info2/sensors/proximity"
#define LUX_AVG	5

/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * Sensor messages
 */
#define SENSOR_MSG_ACTION(M, ...) printk("[qsensor][%s][%05d][act]: " M "\n", __func__, __LINE__, ##__VA_ARGS__);
#define SENSOR_MSG_ERROR(M, ...) printk("[qsensor][%s][%05d][err]: " M "\n", __func__, __LINE__, ##__VA_ARGS__);
#define SENSOR_MSG_INFO(M, ...) printk("[qsensor][%s][%05d][inf]: " M "\n", __func__, __LINE__, ##__VA_ARGS__);
#define SENSOR_MSG_FUNCIN printk("[qsensor][%s][%05d][act]: ++++++++++.\n", __func__, __LINE__);
#define SENSOR_MSG_FUNCOUT printk("[qsensor][%s][%05d][act]: ----------.\n", __func__, __LINE__);
/* QCI End: ALT Sensor Daniel Wang 20140709 */


/*ISL29147 power supply VDD 2.25V-3.63V */
#define ISL29147_VDD_MIN_UV       2400000
#define ISL29147_VDD_MAX_UV       3500000


/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * Chip config struct
 */
struct isl29147_cfg_t {
	int glass_factor;	/* glass factor for als, percent */
	u8 als_ir_comp;		/* als ir comp */
	u8 als_range;		/* als range, 0: 125 Lux, 1: 250, 2:2000, 3:4000 */
	u8 ps_ht;			/* ps high limit */
	u8 ps_led_drv_cur;	/* led driver current, 0:31.25mA, 1:62.5mA, 2:125mA, 3:250mA*/
	u8 ps_lt;			/* ps low limit */
	u8 ps_offset;		/* ps offset comp */
	u8 ps_sleep_time;	/* ps sleep time, 0: 400ms, 1: 100ms, 2: 50ms, 3: 25ms, 4: 12.5ms , 5: 6.25ms, 6: 3.125ms, 7: 0ms*/
};
/* QCI End: ALT Sensor Daniel Wang 20140709 */

#define ISL_IOCTL_MAGIC		0xCF
#define ISL_IOCTL_ALS_ON	_IO(ISL_IOCTL_MAGIC, 1)
#define ISL_IOCTL_ALS_OFF		_IO(ISL_IOCTL_MAGIC, 2)
#define ISL_IOCTL_ALS_DATA		_IOR(ISL_IOCTL_MAGIC, 3, short)
#define ISL_IOCTL_ALS_CALIBRATE	_IO(ISL_IOCTL_MAGIC, 4)
#define ISL_IOCTL_CONFIG_GET	_IOR(ISL_IOCTL_MAGIC, 5, struct isl29147_cfg_t)
#define ISL_IOCTL_CONFIG_SET	_IOW(ISL_IOCTL_MAGIC, 6, struct isl29147_cfg_t)
#define ISL_IOCTL_PROX_ON		_IO(ISL_IOCTL_MAGIC, 7)
#define ISL_IOCTL_PROX_OFF		_IO(ISL_IOCTL_MAGIC, 8)
#define ISL_IOCTL_PROX_DATA		_IO(ISL_IOCTL_MAGIC, 9)
#define ISL_IOCTL_PROX_EVENT	_IO(ISL_IOCTL_MAGIC, 10)
#define ISL_IOCTL_PROX_CALIBRATE	_IO(ISL_IOCTL_MAGIC, 11)
//#define ISL_IOCTL_PROX_STARTUP_CALIBRATE	_IO(ISL_IOCTL_MAGIC, 12)
#define ISL_IOCTL_PROX_GET_ENABLED	_IO(ISL_IOCTL_MAGIC, 13)
#define ISL_IOCTL_ALS_GET_ENABLED	_IO(ISL_IOCTL_MAGIC, 14)

#define ISL29147_ADDR	0x44
#define	DEVICE_NAME		"light"
#define	DRIVER_VERSION	"1.3"

#define ALS_EN_MSK		(1 << 0)
#define PS_EN_MSK		(1 << 1)

#define PS_POLL_TIME	100	 	/* unit is ms */

#define PROX_THRESHOLD_DELTA_LO		15
#define PROX_THRESHOLD_DELTA_HI		15
#define MAX_STARTUP_CALIB_OVER_RANGE	50
#define IS_DO_START_UP_CALIB		1

/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * /sys/class name
 */
#define CLASS_NAME "isl"
/* QCI End: ALT Sensor Daniel Wang 20140709 */

/* char device number */
dev_t dev_num;

/* class structure for this device */
struct class *isl_class;

/* calibaration output data */
struct isl29147_calib_out_t {
	int ret;
	u8 ps_offset;
	u8 ps_lt;
	u8 ps_ht;
};

/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * /sys/class/sensors/ device
 */
static struct sensors_classdev isl29147_ps_classdev = {
	.name = "proximity",
	.vendor = "intersil",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
/* QCI End: ALT Sensor Daniel Wang 20140709 */

/* Riven add Lisght sensor /sys/class/sensor/ entry */
static struct sensors_classdev isl29147_als_classdev = {
    .name = "isl29147-light",
    .vendor = "intersil",
    .version = 1,
    .handle = SENSORS_LIGHT_HANDLE,
    .type = SENSOR_TYPE_LIGHT,
    .max_range = "1",
    .resolution = "5.0",
    .sensor_power = "3",
    .min_delay = 1000,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = 100,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
};
/* Riven add Lisght sensor /sys/class/sensor/ entry */

/* Each client has this additional data */
struct isl29147_data_t {
	struct i2c_client* client;
	struct isl29147_cfg_t* cfg;
	u8 als_pwr_status;
	u8 ps_pwr_status;
	int poll_delay;		/* poll delay set by hal */
	u8 show_als_raw;	/* show als raw data flag, used for debug */
	u8 show_ps_raw;	/* show als raw data flag, used for debug */
	struct timer_list als_timer;	/* als poll timer */
	struct timer_list ps_timer;	/* ps poll timer */
	spinlock_t als_timer_lock;
	spinlock_t ps_timer_lock;
	struct work_struct als_work;
	struct work_struct ps_work;
	struct work_struct calib_work;
	struct delayed_work restore_sensordata_work;
	struct workqueue_struct *als_wq;
	struct workqueue_struct *ps_wq;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	int last_ps;
	u8 als_range_using;		/* the als range using now */
	u8 als_pwr_before_suspend;
	u8 ps_pwr_before_suspend;
	u8 ps_filter_cnt;
	int last_lux; 
	int last_ps_raw;
	int lux_self;
	int lux_meter;
	bool is_lux_file;
	bool is_prox_file;
	
	int als_chg_range_delay_cnt;
	u8 is_do_factory_calib;
	struct cdev cdev;
	/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
	struct sensors_classdev ps_classdev;
	/* QCI End: ALT Sensor Daniel Wang 20140709 */
	/* Riven add Lisght sensor /sys/class/sensor/ entry */
	struct sensors_classdev als_classdev;
	/* Riven add Lisght sensor /sys/class/sensor/ entry */
	/*Added by Shengwei 20140819*/
	int (*power_on)(struct isl29147_data_t *, bool);
	/* regulator data */
	struct regulator *vdd;
	/*Added by Shengwei 20140819*/
};

/* Do not scan isl29147 automatic */
static const unsigned short normal_i2c[] = {ISL29147_ADDR, I2C_CLIENT_END };

/* board and address info */
struct i2c_board_info isl_board_info[] = {
	{I2C_BOARD_INFO(DEVICE_NAME, ISL29147_ADDR)}
};

/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * data struct for isl29147 device
 */
static struct isl29147_cfg_t isl29147_cfg = {
	.als_ir_comp = 0,
	.als_range = 2,
	.glass_factor = 10,
	.ps_ht = 220,			/* FIXME: These are temp values. */
	.ps_led_drv_cur = 3,
	.ps_lt = 170,			/* FIXME: These are temp values. */
	.ps_offset = 0,
	.ps_sleep_time = 0
};
/* QCI End: ALT Sensor Daniel Wang 20140709 */

static struct isl29147_data_t isl29147_data = {
	.cfg = &isl29147_cfg,
	.client = NULL,
	.als_pwr_status = 0,
	.ps_pwr_status = 0,
	.als_input_dev = NULL,
	.ps_input_dev = NULL,
	.lux_meter = 1,
	.lux_self = 1,
	.is_lux_file = false,
	.is_prox_file =  false

};

/* gobal var */
struct isl29147_calib_out_t calib_out;
static atomic_t is_calib_done = ATOMIC_INIT(0);

#if 0
static int do_check_brown_out(struct isl29147_data_t *dev_dat;)
{
	int ret;
	u8 tmp;

	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x04);
	if(ret < 0) goto err_rd;
	tmp = (u8)ret;
	if(tmp & 0x10)
	{
		/* brown out is check */
		ret = i2c_smbus_write_byte_data(dev_dat->client, 0x0e, 0x38);
		return set_sensor_reg(dev_dat);
	}

	return 0;
	
err_rd:
	printk(KERN_ERR "soft reset failed, write sensor reg error, ret = %d\n", ret);
	return -1;
	
err_wr:
	printk(KERN_ERR "soft reset failed, write sensor reg error, ret = %d\n", ret);
	return -1;
}
#endif
	
static void do_als_timer(unsigned long arg)
{
	struct isl29147_data_t *dev_dat;

	dev_dat = (struct isl29147_data_t *)arg;

	/* timer handler is atomic context, so don't need sinp_lock() */
	if(dev_dat->als_pwr_status == 0)
	{
		return ;
	}

	/* start a work queue, I cannot do i2c oepration in timer context for
	   this context is atomic and i2c function maybe sleep. */
	queue_work(dev_dat->als_wq, &dev_dat->als_work);
}

static void do_ps_timer(unsigned long arg)
{
	struct isl29147_data_t *dev_dat;

	dev_dat = (struct isl29147_data_t *)arg;

	if(dev_dat->ps_pwr_status == 0)
	{
		return ;
	}
	
	/* start a work queue, I cannot do i2c oepration in timer context for
	   this context is atomic and i2c function maybe sleep. */
	//schedule_work(&dev_dat->ps_work);
	queue_work(dev_dat->ps_wq, &dev_dat->ps_work);
}

static int do_factory_calib(struct isl29147_data_t *dev_dat)
{
	int i;
	u8 ir_data;
	u8 ps_data;
	int ret;
	u8 offset = 0;
	u8 reg;
	
	printk(KERN_DEBUG "start isl29147 factory mode calibration...");

	for(i = 0; i < 25; i++)
	{
		msleep(20);
		
		ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0a);
		if(ret < 0) goto err_rd;
		ps_data = (u8)ret;

		ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0d);
		if(ret < 0) goto err_rd;
		ir_data = (u8)ret >> 1;

		if(ir_data >= 0x7f) continue;
		if(ps_data > 40)
		{
			/* ps data is too large, I will add the ps offset comp */
			offset = offset + (ps_data - 15) / 15;
			if(offset > 15) offset = 15; /* offset max is 15, 4 bits in reg */
		}
		else
		{
			/* if ps_data is too small, but offset comp is not 0, we can 
				decrease the offset */
			if((ps_data < 5) && (offset > 0))
			{
				offset--;
			}
			else
			{
				/* I find the right comp offset */
				return offset;
			}
		}

		reg = (offset & 0xf) << 3;
		ret = i2c_smbus_write_byte_data(dev_dat->client, 0x02, offset);
		if(ret < 0) goto err_wr;
	}

	printk(KERN_ERR "calibration ps offset comp failed");
	return -2;

err_rd:
	printk(KERN_ERR "calibration failed, read sensor reg error, ret = %d\n", ret);
	return -1;	

err_wr:
	printk(KERN_ERR "calibration failed, write sensor reg error, ret = %d\n", ret);
	return -1;

}

static void do_calib_work(struct work_struct *work)
{
	struct isl29147_data_t *dev_dat;
	struct isl29147_cfg_t *cfg;
	int ret;
	u8 reg;
	int calib_cnt = 10;
	int i;
	u16 sum = 0;
	u8 ps_dat;
	u16 ps_lt, ps_ht;

	printk(KERN_DEBUG "start isl29147 calibration...");
	
	dev_dat = container_of(work, struct isl29147_data_t, calib_work);
	cfg = dev_dat->cfg;

	/* set register config */
	if(dev_dat->is_do_factory_calib)
	{
		reg = 0;
	}
	else
	{
		reg = (dev_dat->cfg->ps_offset & 0xf) << 3;
	}
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x02, reg);
	if(ret < 0) goto err_wr;

	reg = 0;
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x05, reg);
	if(ret < 0) goto err_wr;
	
	reg = 0xff;
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x06, reg);
	if(ret < 0) goto err_wr;

	/* set ps mode */
	reg = 0x30; /* the ps sleep time is 12.5ms */
	reg |= dev_dat->cfg->ps_led_drv_cur & 0x3;
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg);
	if(ret < 0) goto err_wr;

	if(dev_dat->is_do_factory_calib)
	{
		ret = do_factory_calib(dev_dat);
		if(ret < 0)
		{
			/* stop ps */
			reg = 0x0;
			i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg);
			calib_out.ret = ret;
			atomic_set(&is_calib_done, 1);
			return ;
		}
		calib_out.ps_offset = (u8)ret;
	}

	for(i = 0; i < calib_cnt; i++)
	{
		msleep(20);
		ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0a);
		if(ret < 0) goto err_rd;
		sum = sum + (u8)ret;
	}

	/* stop ps */
	reg = 0x0;
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg);
	if(ret < 0) goto err_wr;

	ps_dat = sum / i;
	ps_lt = ps_dat + PROX_THRESHOLD_DELTA_LO;
	ps_ht = ps_lt + PROX_THRESHOLD_DELTA_HI;
	if(ps_lt > 255) ps_lt = 255;
	if(ps_ht > 255) ps_ht = 255;

	calib_out.ret = 0;
	calib_out.ps_ht = ps_ht;
	calib_out.ps_lt = ps_lt;

	atomic_set(&is_calib_done, 1);

	printk(KERN_INFO "end isl29147 calibration, ps_lt=%d, ps_ht=%d", ps_lt, ps_ht);
	return ;
	
err_rd:
	calib_out.ret = -1;
	printk(KERN_ERR "calibration failed, read sensor reg error, ret = %d\n", ret);
	atomic_set(&is_calib_done, 1);
	return ;
	
err_wr:
	calib_out.ret = -1;
	printk(KERN_ERR "calibration failed, write sensor reg error, ret = %d\n", ret);
	atomic_set(&is_calib_done, 1);
	return ;	
}

static void do_calib(struct isl29147_data_t *dev_dat)
{
	if(dev_dat->als_pwr_status || dev_dat->ps_pwr_status)
	{
		calib_out.ret = -2;
		printk(KERN_WARNING "Cannot do calibration when als or ps is working");
		return ;
	}

	atomic_set(&is_calib_done, 0);
	dev_dat->is_do_factory_calib = 1;
	queue_work(dev_dat->ps_wq, &dev_dat->calib_work);

	while(1)
	{
		msleep(100);
		if(atomic_read(&is_calib_done) > 0) break;
	}

	if(calib_out.ret >= 0)
	{
		dev_dat->cfg->ps_lt = calib_out.ps_lt;
		dev_dat->cfg->ps_ht = calib_out.ps_ht;
		dev_dat->cfg->ps_offset = calib_out.ps_offset;
	}
}

static int do_startup_calib(struct isl29147_data_t *dev_dat)
{
	if(dev_dat->als_pwr_status || dev_dat->ps_pwr_status)
	{
		calib_out.ret = -2;
		printk(KERN_WARNING "Cannot do calibration when als or ps is working");
		return -1;
	}

	atomic_set(&is_calib_done, 0);
	dev_dat->is_do_factory_calib = 0;
	queue_work(dev_dat->ps_wq, &dev_dat->calib_work);

	while(1)
	{
		msleep(100);
		if(atomic_read(&is_calib_done) > 0) break;
	}

	if(calib_out.ret < 0) return -1;
	
	if((calib_out.ps_lt > (dev_dat->cfg->ps_lt + MAX_STARTUP_CALIB_OVER_RANGE)) || 
		(calib_out.ps_ht > (dev_dat->cfg->ps_ht + MAX_STARTUP_CALIB_OVER_RANGE)))
	{
		printk(KERN_WARNING "startup calibration result is too large, use "
			"default value");
		return -1;
	}
	else
	{
		dev_dat->cfg->ps_lt = calib_out.ps_lt;
		dev_dat->cfg->ps_ht = calib_out.ps_ht;
		return 0;
	}
}

static void do_als_work(struct work_struct *work)
{
	static int count = 0;
	static int lux_sum=0;
	static int lux_arr[10]={0};
	struct isl29147_data_t *dev_dat;
	struct isl29147_cfg_t *cfg;
	int ret;
	static int als_dat;
	u8 show_raw_dat;
	int lux, lux_report;
	u8 als_range;
	int is_chg_range = 0;
	u8 new_range;
	//int i;
	
	dev_dat = container_of(work, struct isl29147_data_t, als_work);
	cfg = dev_dat->cfg;
	
	spin_lock(&dev_dat->ps_timer_lock);
	show_raw_dat = dev_dat->show_als_raw;
	spin_unlock(&dev_dat->ps_timer_lock);

	//als_range = dev_dat->als_range_using;
	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x02);
	if(ret < 0) goto err_rd;

	als_range = (u8)ret;
	als_range &= 0x03;

	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0c);
	if(ret < 0) goto err_rd;
	als_dat = (u8)ret;
	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0b);
	if(ret < 0) goto err_rd;
	als_dat = als_dat + ( ((u8)ret & 0x0f) << 8 );
	if(dev_dat->als_chg_range_delay_cnt == 0)
	{
		/* als measurment is done */
		if(als_range == 0)
		{
			//lux = (als_dat * 125) / 4096;
			lux = (als_dat * 1) * dev_dat->lux_meter / dev_dat->lux_self;
LOGD("als_range = 0, als_dat = %d\n", als_dat);			
		}
		else if(als_range == 1)
		{
			//lux = (als_dat * 250) / 4096;
			lux = (als_dat * 2) * dev_dat->lux_meter / dev_dat->lux_self;
LOGD("als_range = 1, als_dat = %d\n", als_dat*2);			
		}
		else if(als_range == 2)
		{
			//lux = (als_dat * 2000) / 4096;
			lux = (als_dat * 16) * dev_dat->lux_meter / dev_dat->lux_self;
LOGD("als_range = 2, als_dat = %d\n", als_dat*16);			
		}
		else
		{
			//lux = (als_dat * 4000) / 4096;
			lux = (als_dat * 32) * dev_dat->lux_meter / dev_dat->lux_self;
LOGD("als_range = 3, als_dat = %d\n", als_dat*32);			
		}
		
#if 0
LOGD("read lux = %d\n", lux);
for(i = 0; i < 10; i++)
	LOGD("lux_arr[%d] = %d\n", i, lux_arr[i]);
#endif 
		lux_sum -= lux_arr[count];
		lux_arr[count] = lux;
		count++;
		lux_sum += lux;
#if 0
LOGD("Update...\n");

for(i = 0; i < 10; i++)
	LOGD("lux_arr[%d] = %d\n", i, lux_arr[i]);
#endif 		
		/* change range */	
		if(als_range == 0)
		{
			if(als_dat > 3600) 
			{
				cfg->als_range = 1;
				is_chg_range = 1;
			}
		}
		else if(als_range == 1)
		{
			if(als_dat < 1500) 
			{
				cfg->als_range = 0;
				is_chg_range = 1;
			}
			else if(als_dat > 3600)
			{	
				cfg->als_range = 2;
				is_chg_range = 1;
			}
		}
		else if(als_range == 2)
		{
			if(als_dat < 400) 
			{		
				cfg->als_range = 1;
				is_chg_range = 1;
			}
			else if(als_dat > 3600)
			{	
				cfg->als_range = 3;
				is_chg_range = 1;
			}
		}
		else
		{
			if(als_dat < 1500) 
			{
				cfg->als_range = 2;
				is_chg_range = 1;
			}
		}
		
		if(is_chg_range)
		{
			ret = i2c_smbus_read_byte_data(dev_dat->client, 0x02);
			if(ret < 0) goto err_rd;

			new_range = (u8)ret;
			new_range &= ~(0x03);

			new_range |= cfg->als_range & 0x3;	
			ret = i2c_smbus_write_byte_data(dev_dat->client, 0x02, new_range);
			if(ret < 0) goto err_wr;

			dev_dat->als_chg_range_delay_cnt = 2;

		}

		//lux = lux * 100 / cfg->glass_factor;
		//lux = lux * dev_dat->lux_meter / dev_dat->lux_self;
			
		lux = lux_sum / LUX_AVG;
		if(count == LUX_AVG)
			count = 0;

		if(lux <=100)
			lux_report = 50;
		else if(lux>100 && lux<=2600)
			lux_report = 1300;
		else if(lux>2600 && lux<=5000)
			lux_report = 3800;
		else
			lux_report = 10000;
		
		
LOGD("report lux = %d\n", lux);			
		input_report_abs(dev_dat->als_input_dev, ABS_MISC, lux_report);
		input_sync(dev_dat->als_input_dev);	
		
		if(show_raw_dat)
		{
			printk(KERN_INFO "now als raw data is = %d, LUX = %d\n", als_dat, lux);
		}
		dev_dat->last_lux = lux;
	}
	else if(dev_dat->als_chg_range_delay_cnt > 0)
	{
//LOGD("dev_dat->als_chg_range_delay_cnt--\n");
		dev_dat->als_chg_range_delay_cnt--;
	}

	/* restart timer */			
	spin_lock(&dev_dat->als_timer_lock);
	if(dev_dat->als_pwr_status == 0)
	{
		spin_unlock(&dev_dat->als_timer_lock);
		return ;
	}
	dev_dat->als_timer.expires = jiffies + (HZ * dev_dat->poll_delay) / 1000;
	spin_unlock(&dev_dat->als_timer_lock);
	add_timer(&dev_dat->als_timer);	

	return ;

err_rd:
	printk(KERN_ERR "Read als sensor error, ret = %d\n", ret);
	return ;

err_wr:
	printk(KERN_ERR "write sensor reg error, ret = %d\n", ret);
	return ;
}

static void do_ps_work(struct work_struct *work)
{
	struct isl29147_data_t *dev_dat;
	struct isl29147_cfg_t *cfg;
	int last_ps;
	int ret;
	u8 show_raw_dat;

	dev_dat = container_of(work, struct isl29147_data_t, ps_work);
	cfg = dev_dat->cfg;

	spin_lock(&dev_dat->ps_timer_lock);
	show_raw_dat = dev_dat->show_ps_raw;
	spin_unlock(&dev_dat->ps_timer_lock);

	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0a);
	if(ret < 0) goto err_rd;

	last_ps = dev_dat->last_ps;

	if(ret > cfg->ps_ht) dev_dat->last_ps = 0;
	else if(ret < cfg->ps_lt) dev_dat->last_ps = 1;
	else if(dev_dat->last_ps == -1) dev_dat->last_ps = 1;
	
	dev_dat->last_ps_raw = ret;

	if(show_raw_dat)
	{
		printk(KERN_INFO "ps raw data = %d\n", dev_dat->last_ps_raw);
	}

	if(last_ps != dev_dat->last_ps)
	{
		if(dev_dat->last_ps == 0)
		{
			input_report_abs(dev_dat->ps_input_dev, ABS_DISTANCE, 0);
		}
		else
		{
			input_report_abs(dev_dat->ps_input_dev, ABS_DISTANCE, 1);
		}
		input_sync(dev_dat->ps_input_dev);
		if(show_raw_dat)
		{
			printk(KERN_INFO "ps status changed, now = %d\n",dev_dat->last_ps);
		}
	}

	/* restart timer */
	spin_lock(&dev_dat->ps_timer_lock);
	if(dev_dat->ps_pwr_status == 0)
	{
		spin_unlock(&dev_dat->ps_timer_lock);
		return ;
	}
	dev_dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
	spin_unlock(&dev_dat->ps_timer_lock);
	add_timer(&dev_dat->ps_timer);

	return ;

err_rd:
	printk(KERN_ERR "Read ps sensor error, ret = %d\n", ret);
	return ;
}


/* Shengwei 201 load calibration data */
static void do_restore_sensordata_work(struct work_struct *work)
{
	struct isl29147_data_t *dev_dat;
	char lcontent[256] = {0};
	char pcontent[256] = {0};
	int error = 0;
	loff_t lposition = 0;
	loff_t pposition = 0;
	mm_segment_t lfs = {0};
	mm_segment_t pfs = {0};
	struct file* lfile = NULL;
	struct file* pfile = NULL;

	dev_dat = container_of(work, struct isl29147_data_t, restore_sensordata_work.work);
	/* load light data */
	LOGD("Get light data from file(%s).", LIGHT_CONF_FILE);
	if(!dev_dat->is_lux_file)
	{
		LOGD("Open file(%s).", LIGHT_CONF_FILE);
		lfile = filp_open(LIGHT_CONF_FILE, O_RDONLY, 0);
		error = (int)IS_ERR(lfile);
		if(error)
		{
			LOGD("Failed to open file(%s) with error(%d)!", LIGHT_CONF_FILE, error);
		}
		else
		{
			LOGD("Read file(%s).", LIGHT_CONF_FILE);
			lfs = get_fs();
			set_fs(get_ds());
			vfs_read(lfile, lcontent, sizeof(lcontent), &lposition);
			set_fs(lfs);
			LOGD("light Sscanf offset from content(%s).", lcontent);
			error = sscanf(lcontent, "%d %d", &dev_dat->lux_self, &dev_dat->lux_meter);
			if(error != 2)
			{
				LOGD("Light sensor fails to sscanf offset!");
			}
			LOGD("Sscanfed offset(%d,%d).", dev_dat->lux_self, dev_dat->lux_meter);
			LOGD("Close file(%s).", LIGHT_CONF_FILE);
			filp_close(lfile, NULL);
		}
	}
	else
	{
		LOGD("Gotten light data from file(%s) already.", LIGHT_CONF_FILE);
	}
	/* load proximity data */
	LOGD("Get proximity data from file(%s).", PROX_CONF_FILE);
	if(!dev_dat->is_prox_file)
	{
		LOGD("Open file(%s).", PROX_CONF_FILE);
		pfile = filp_open(PROX_CONF_FILE, O_RDONLY, 0);
		error = (int)IS_ERR(pfile);
		if(error)
		{
			LOGD("Failed to open file(%s) with error(%d)!", PROX_CONF_FILE, error);
		}
		else
		{
		LOGD("Read file(%s).", PROX_CONF_FILE);
			pfs = get_fs();
			set_fs(get_ds());
			vfs_read(pfile, pcontent, sizeof(pcontent), &pposition);
			set_fs(pfs);
			LOGD("proximity Sscanf offset from content(%s).", pcontent);
			error = sscanf(pcontent, "%hhu %hhu", &dev_dat->cfg->ps_lt, &dev_dat->cfg->ps_ht);
			if(error != 2)
			{
				LOGD("Proximity sensor fails to sscanf offset!");
			}
			LOGD("Sscanfed offset(%hhu,%hhu).", dev_dat->cfg->ps_lt, dev_dat->cfg->ps_ht);
			LOGD("Close file(%s).", PROX_CONF_FILE);
			filp_close(pfile, NULL);
		}
	}
	else
	{
		LOGD("Gotten proximity data from file(%s) already.", PROX_CONF_FILE);
	}
}

/* enable to run als */
static int set_sensor_reg(struct isl29147_data_t *dev_dat)
{
	struct isl29147_cfg_t* cfg;
	u8 reg_dat[16];
	int i, ret;

	cfg = dev_dat->cfg;
	
	reg_dat[4] = 0x22;
	//reg_dat[3] = cfg->ps_lt;
	//reg_dat[4] = cfg->ps_ht;
	reg_dat[5] = 0;
	reg_dat[6] = 255;

	/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
	/**
	 * Set reg 0x01
	 */
	reg_dat[1] = 0x00;
	
	/* set ps led driver current to 250mA */
	reg_dat[1] |= cfg->ps_led_drv_cur & 0x3;

	/* set ps sleep time to 400ms */
	reg_dat[1] |= (cfg->ps_sleep_time & 0x7) << 2;
	/* QCI End: ALT Sensor Daniel Wang 20140709 */

	reg_dat[2] = 0x80;	/* ps int is hystersis */
	spin_lock(&dev_dat->als_timer_lock);
	reg_dat[2] |= cfg->als_range & 0x3;
	reg_dat[2] |= (cfg->ps_offset & 0xf) << 3;
	if(dev_dat->als_pwr_status)
	{
		/* measurement als */
		reg_dat[2] |= 0x04;
	}
	reg_dat[3] = cfg->als_ir_comp & 0xf;
	spin_unlock(&dev_dat->als_timer_lock);
	
	spin_lock(&dev_dat->ps_timer_lock);
	if(dev_dat->ps_pwr_status) reg_dat[1] |= 0x20;
	reg_dat[1] |= cfg->ps_led_drv_cur & 0x3;
	spin_unlock(&dev_dat->ps_timer_lock);

	/* first, do a software reset for if there is a brown out */
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x0e, 0x38);
	if(ret < 0) return ret;

	for(i = 3; i <= 6; i++)
	{
		ret = i2c_smbus_write_byte_data(dev_dat->client, i, reg_dat[i]);
		if(ret < 0) 
		{
			return ret;
		}
	}

	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x02, reg_dat[2]);
	if(ret < 0) 
	{
		return ret;	
	}
	
	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg_dat[1]);
	if(ret < 0) 
	{
		return ret;	
	}
	
	return 0;
}

/* set power status */
static int set_als_pwr_st(u8 state, struct isl29147_data_t *dat)
{
	int ret = 0;
		
	if(state)
	{
		spin_lock(&dat->als_timer_lock);
		if(dat->als_pwr_status)
		{
			spin_unlock(&dat->als_timer_lock);
			return ret;
		}
		dat->als_pwr_status = 1;
		spin_unlock(&dat->als_timer_lock);
		ret = set_sensor_reg(dat);
		if(ret < 0)
		{
			printk(KERN_ERR "set light sensor reg error, ret = %d\n", ret);
			return ret;
		}
		
		/* start timer */
		dat->als_timer.function = &do_als_timer;
		dat->als_timer.data = (unsigned long)dat;
		spin_lock(&dat->als_timer_lock);
		dat->als_timer.expires = jiffies + (HZ * 250) / 1000;
		spin_unlock(&dat->als_timer_lock);
		dat->als_range_using = dat->cfg->als_range;
//LOGD("dat->cfg->als_range = %d\n", dat->cfg->als_range);
		add_timer(&dat->als_timer);
	}
	else
	{
		spin_lock(&dat->als_timer_lock);
		if(dat->als_pwr_status == 0)
		{
			spin_unlock(&dat->als_timer_lock);
			return ret;
		}
		dat->als_pwr_status = 0;
		spin_unlock(&dat->als_timer_lock);
		ret = set_sensor_reg(dat);

		/* delete timer */
		del_timer_sync(&dat->als_timer);
	}

	return ret;
}

static int set_ps_pwr_st(u8 state, struct isl29147_data_t *dat)
{
	int ret = 0;
	
	if(state)
	{		
		spin_lock(&dat->ps_timer_lock);
		if(dat->ps_pwr_status)
		{
			spin_unlock(&dat->ps_timer_lock);
			return ret;
		}
		dat->ps_pwr_status = 1;
		spin_unlock(&dat->ps_timer_lock);
		
		dat->last_ps = -1;
		dat->ps_filter_cnt = 0;
		ret = set_sensor_reg(dat);
		if(ret < 0)
		{
			printk(KERN_ERR "set proximity sensor reg error, ret = %d\n", ret);
			return ret;
		}	
		
		/* start timer */
		dat->ps_timer.function = &do_ps_timer;
		dat->ps_timer.data = (unsigned long)dat;
		dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
		add_timer(&dat->ps_timer);
	}
	else
	{
		spin_lock(&dat->ps_timer_lock);
		if(dat->ps_pwr_status == 0)
		{
			spin_unlock(&dat->ps_timer_lock);
			return ret;
		}
		dat->ps_pwr_status = 0;
		spin_unlock(&dat->ps_timer_lock);

		ret = set_sensor_reg(dat);
		
		/* delete timer */
		del_timer_sync(&dat->ps_timer);
	}

	return ret;
}

/* device attribute */
/* enable als attribute */
static ssize_t show_enable_als_sensor(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 pwr_status;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	pwr_status = dat->als_pwr_status;
	spin_unlock(&dat->als_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", pwr_status);
}
static ssize_t store_enable_als_sensor(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	ssize_t ret;
	unsigned long val;

	dat = (struct isl29147_data_t *)dev->platform_data;
		
	val = simple_strtoul(buf, NULL, 10);
	ret = set_als_pwr_st(val, dat);

	if(ret == 0) ret = count;
	return ret;
}
static DEVICE_ATTR(enable_als_sensor, S_IWUSR|S_IWGRP|S_IRUGO, show_enable_als_sensor,
	store_enable_als_sensor);

/* enable ps attribute */
static ssize_t show_enable_ps_sensor(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 pwr_status;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->ps_timer_lock);
	pwr_status = dat->ps_pwr_status;
	spin_unlock(&dat->ps_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", pwr_status);
}
static ssize_t store_enable_ps_sensor(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	ssize_t ret;
	unsigned long val;

	dat = (struct isl29147_data_t *)dev->platform_data;

	val = simple_strtoul(buf, NULL, 10);
	ret = set_ps_pwr_st(val, dat);

	if(ret == 0) ret = count;

	return ret;
}
static DEVICE_ATTR(enable_ps_sensor, S_IWUSR|S_IWGRP|S_IRUGO, show_enable_ps_sensor,
	store_enable_ps_sensor);

/* ps led driver current attribute */
static ssize_t show_ps_led_drv(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;

	dat = (struct isl29147_data_t *)dev->platform_data;
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->cfg->ps_led_drv_cur);
}
static ssize_t store_ps_led_drv(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int val;

	if(sscanf(buf, "%d", &val) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;
	dat->cfg->ps_led_drv_cur = val & 0x3;
	
	return count;
}
static DEVICE_ATTR(ps_led_driver_current, S_IWUSR|S_IWGRP|S_IRUGO, show_ps_led_drv,
	store_ps_led_drv);

/* als range attribute */
static ssize_t show_als_range(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 range;
	
	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	range = dat->cfg->als_range;
	spin_unlock(&dat->als_timer_lock);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", range);
}
static ssize_t store_als_range(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int val;

	if(sscanf(buf, "%d", &val) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;
	
	spin_lock(&dat->als_timer_lock);
	dat->cfg->als_range = val & 0x3;
	spin_unlock(&dat->als_timer_lock);
	
	return count;
}
static DEVICE_ATTR(als_range, S_IWUSR|S_IWGRP|S_IRUGO, show_als_range, store_als_range);

/* ps limit range attribute */
static ssize_t show_ps_limit(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;

	dat = (struct isl29147_data_t *)dev->platform_data;
	return snprintf(buf, PAGE_SIZE, "%d %d\n", dat->cfg->ps_lt, dat->cfg->ps_ht);
}
static ssize_t store_ps_limit(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int lt, ht;

	if(sscanf(buf, "%d %d", &lt, &ht) != 2)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;
	
	if(lt > 255) dat->cfg->ps_lt = 255;
	else if(lt < 0) dat->cfg->ps_lt = 0;
	else  dat->cfg->ps_lt = lt;
	
	if(ht > 255) dat->cfg->ps_ht = 255;
	else if(ht < 0) dat->cfg->ps_ht = 0;
	else  dat->cfg->ps_ht = ht;
	
	return count;
}
static DEVICE_ATTR(ps_limit, S_IWUSR|S_IWGRP|S_IRUGO, show_ps_limit, store_ps_limit);

/* poll delay attribute */
static ssize_t show_poll_delay (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	int delay;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	delay = dat->poll_delay;
	spin_unlock(&dat->als_timer_lock);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", delay);
}
static ssize_t store_poll_delay (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int delay;

	if(sscanf(buf, "%d", &delay) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(delay  < 120) dat->poll_delay = 120;
	else if(delay > 65535) dat->poll_delay = 65535;
	else dat->poll_delay = delay;
	spin_unlock(&dat->als_timer_lock);
	
	return count;
}
static DEVICE_ATTR(poll_delay, S_IWUSR|S_IWGRP|S_IRUGO, show_poll_delay, 
	store_poll_delay);

/* show als raw data attribute */
static ssize_t show_als_show_raw (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 flag;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	flag = dat->show_als_raw;
	spin_unlock(&dat->als_timer_lock);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", flag);
}
static ssize_t store_als_show_raw (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int flag;

	if(sscanf(buf, "%d", &flag) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(flag == 0) dat->show_als_raw = 0;
	else dat->show_als_raw = 1;
	spin_unlock(&dat->als_timer_lock);
	
	return count;
}
static DEVICE_ATTR(als_show_raw, S_IWUSR|S_IWGRP|S_IRUGO, show_als_show_raw, 
	store_als_show_raw);

/* show ps raw data attribute */
static ssize_t show_ps_show_raw (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 flag;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	flag = dat->show_ps_raw;
	spin_unlock(&dat->als_timer_lock);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", flag);
}
static ssize_t store_ps_show_raw (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int flag;

	if(sscanf(buf, "%d", &flag) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(flag == 0) dat->show_ps_raw = 0;
	else dat->show_ps_raw = 1;
	spin_unlock(&dat->als_timer_lock);
	
	return count;
}
static DEVICE_ATTR(ps_show_raw, S_IWUSR|S_IWGRP|S_IRUGO, show_ps_show_raw, 
	store_ps_show_raw);

/* show ps offset comp data attribute */
static ssize_t show_ps_offset_comp (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 offset;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	offset = dat->cfg->ps_offset;
	spin_unlock(&dat->als_timer_lock);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", offset);
}
static ssize_t store_ps_offset_comp (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int offset;

	if(sscanf(buf, "%d", &offset) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	dat->cfg->ps_offset = offset & 0xf;
	spin_unlock(&dat->als_timer_lock);
	
	return count;
}
static DEVICE_ATTR(ps_offset_comp, S_IWUSR|S_IWGRP|S_IRUGO, show_ps_offset_comp, 
	store_ps_offset_comp);

/* Shengwei 20140901 calibration data */

static ssize_t show_ps_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	dat = (struct isl29147_data_t *)dev->platform_data;

	LOGD("dat->last_ps_raw = %d\n", dat->last_ps_raw);
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->last_ps_raw);
}

static DEVICE_ATTR(ps_pdata, S_IRUGO, show_ps_pdata, NULL);


static ssize_t show_als_ldata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	dat = (struct isl29147_data_t *)dev->platform_data;

	LOGD("dat->last_lux = %d\n", dat->last_lux);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->last_lux);
}

static DEVICE_ATTR(als_ldata, S_IRUGO, show_als_ldata, NULL);


static ssize_t show_als_lux_self (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	dat = (struct isl29147_data_t *)dev->platform_data;
	LOGD("dat->lux_self = %d\n", dat->lux_self);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->lux_self);
}
static ssize_t store_als_lux_self (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int lux_self;

	if(sscanf(buf, "%d", &lux_self) != 1)
	{
		return -EINVAL;
	}
	dat = (struct isl29147_data_t *)dev->platform_data;
	dat->lux_self = lux_self;
	
	return count;
}
static DEVICE_ATTR(lux_self, S_IWUSR|S_IWGRP|S_IRUGO, show_als_lux_self, store_als_lux_self);

static ssize_t show_als_lux_meter (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	dat = (struct isl29147_data_t *)dev->platform_data;

	LOGD("dat->lux_meter = %d\n", dat->lux_meter);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->lux_meter);
}
static ssize_t store_als_lux_meter (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int lux_meter;

	if(sscanf(buf, "%d", &lux_meter) != 1)
	{
		return -EINVAL;
	}
	dat = (struct isl29147_data_t *)dev->platform_data;
	dat->lux_meter = lux_meter;
	
	return count;
}
static DEVICE_ATTR(lux_meter, S_IWUSR|S_IWGRP|S_IRUGO, show_als_lux_meter, 	store_als_lux_meter);

/* show als ir comp data attribute */
static ssize_t show_als_ir_comp (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29147_data_t *dat;
	u8 comp;

	dat = (struct isl29147_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	comp = dat->cfg->als_ir_comp;
	spin_unlock(&dat->als_timer_lock);
	
	return snprintf(buf, PAGE_SIZE, "%d\n", comp);
}
static ssize_t store_als_ir_comp (struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29147_data_t *dat;
	int comp;

	if(sscanf(buf, "%d", &comp) != 1)
	{
		return -EINVAL;
	}
	
	dat = (struct isl29147_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	dat->cfg->als_ir_comp = comp & 0xf;
	spin_unlock(&dat->als_timer_lock);
	
	return count;
}
static DEVICE_ATTR(als_ir_comp, S_IWUSR|S_IWGRP|S_IRUGO, show_als_ir_comp, 
	store_als_ir_comp);

static struct attribute *als_attr[] = {
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_range.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_als_show_raw.attr,
	&dev_attr_als_ir_comp.attr,
	&dev_attr_lux_self.attr,
	&dev_attr_lux_meter.attr,
	&dev_attr_als_ldata.attr,
	NULL
};

static struct attribute_group als_attr_grp = {
	.name = "light sensor",
	.attrs = als_attr
};

static struct attribute *ps_attr[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_ps_led_driver_current.attr,
	&dev_attr_ps_limit.attr,
	&dev_attr_ps_show_raw.attr,
	&dev_attr_ps_offset_comp.attr,
	&dev_attr_ps_pdata.attr,
	NULL
};

static struct attribute_group ps_attr_grp = {
	.name = "proximity sensor",
	.attrs = ps_attr
};

/* initial and register a input device for sensor */
static int init_input_dev(struct isl29147_data_t *dev_dat)
{
	int err;
	struct input_dev *als_dev;
	struct input_dev *ps_dev;
	
	als_dev = input_allocate_device();
	if (!als_dev)
	{
		return -ENOMEM;
	}

	ps_dev = input_allocate_device();
	if (!ps_dev)
	{
		err = -ENOMEM;	
		goto err_free_als;
	}

	als_dev->name = "light";
	als_dev->id.bustype = BUS_I2C;
	als_dev->id.vendor  = 0x0001;
	als_dev->id.product = 0x0001;
	als_dev->id.version = 0x0100;
	als_dev->evbit[0] = BIT_MASK(EV_ABS);
	als_dev->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);
	als_dev->dev.platform_data = &isl29147_data;
	input_set_abs_params(als_dev, ABS_MISC, 0, 65535, 0, 0);
	
	ps_dev->name = "proximity";
	ps_dev->id.bustype = BUS_I2C;
	ps_dev->id.vendor  = 0x0001;
	ps_dev->id.product = 0x0002;
	ps_dev->id.version = 0x0100;
	ps_dev->evbit[0] = BIT_MASK(EV_ABS);
	ps_dev->absbit[BIT_WORD(ABS_DISTANCE)] |= BIT_MASK(ABS_DISTANCE);
	ps_dev->dev.platform_data = &isl29147_data;
	input_set_abs_params(ps_dev, ABS_DISTANCE, 0, 65535, 0, 0);
	
	err = input_register_device(als_dev);
	if (err)
	{
		goto err_free_ps;
	}

	err = input_register_device(ps_dev);
	if (err)
	{
		goto err_free_ps;
	}	

	err = sysfs_create_group(&als_dev->dev.kobj, &als_attr_grp);
	if (err) {
		dev_err(&als_dev->dev, "isl29147: device create als file failed\n");
		goto err_free_ps;
	}

	err = sysfs_create_group(&ps_dev->dev.kobj, &ps_attr_grp);
	if (err) {
		dev_err(&ps_dev->dev, "isl29147: device create ps file failed\n");
		goto err_free_ps;
	}

	dev_dat->als_input_dev = als_dev;
	dev_dat->ps_input_dev = ps_dev;
	
	return 0;

err_free_ps:
	input_free_device(ps_dev);
err_free_als:
	input_free_device(als_dev);

	return err;
}

/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
static int isl29147_ps_classdev_sensors_enable(struct sensors_classdev *classdev, unsigned int enable)
{
	int ret = 0;
	struct isl29147_data_t *data = NULL;

	if(classdev == NULL)
	{
		SENSOR_MSG_ERROR("Failed for null point variable!");
		return -1;
	}

	data = container_of(classdev, struct isl29147_data_t, ps_classdev);
	if(data == NULL)
	{
		SENSOR_MSG_ERROR("Failed for null point variable!");
		return -1;
	}

	ret = set_ps_pwr_st(enable, data);

	return ret;
}
/* QCI End: ALT Sensor Daniel Wang 20140709 */

/* Riven add Lisght sensor /sys/class/sensor/ entry */
static int isl29147_als_classdev_sensors_enable(struct sensors_classdev *classdev, unsigned int enable)
{
    int ret = 0;
    struct isl29147_data_t *data = NULL;
    
    if(classdev == NULL)
    {
	printk(KERN_ERR "isl29147-light: Failed for null point variable!\n");
	return -1;
    }
    
    data = container_of(classdev, struct isl29147_data_t, als_classdev);
    if(data == NULL)
    {
	printk(KERN_ERR "isl29147-light: Failed for null point variable!\n");
	return -1;
    }
    
    ret = set_als_pwr_st(enable, data);
    
    return ret;
}
/* Riven add Lisght sensor /sys/class/sensor/ entry */
/* Return 0 if detection is successful, -ENODEV otherwise */
static int isl29147_detect(struct i2c_client *client,
	struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	printk(KERN_DEBUG "In isl29147_detect()\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
	{
		printk(KERN_WARNING "I2c adapter don't support ISL29147\n");
		return -ENODEV;
	}

	/* probe that if isl29147 is at the i2 address */
	if (i2c_smbus_xfer(adapter, client->addr, 0,I2C_SMBUS_WRITE,
		0,I2C_SMBUS_QUICK,NULL) < 0)
		return -ENODEV;

	strlcpy(info->type, "isl29147", I2C_NAME_SIZE);
	printk(KERN_INFO "%s is found at i2c device address %d\n", 
		info->type, client->addr);

	return 0;
}

static int sensor_regulator_power_on(struct isl29147_data_t *client_data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(client_data->vdd);
		if (rc) {
			dev_err(&client_data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		return rc;
	} else {
		rc = regulator_enable(client_data->vdd);
		if (rc) {
			dev_err(&client_data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	}
	return rc;
}


static int sensor_platform_hw_power_on(struct isl29147_data_t *client_data, bool on)
{
	int ret = 0;

	ret = sensor_regulator_power_on(client_data, on);
	if (ret)
		dev_err(&client_data->client->dev,
			"Can't configure regulator!\n");

	return ret;
}

static int sensor_regulator_configure(struct isl29147_data_t *client_data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(client_data->vdd) > 0)
			regulator_set_voltage(client_data->vdd, 0, ISL29147_VDD_MAX_UV);
		regulator_put(client_data->vdd);
	} else {
		client_data->vdd = regulator_get(&client_data->client->dev, "vdd");
		if (IS_ERR(client_data->vdd)) {
			rc = PTR_ERR(client_data->vdd);
			dev_err(&client_data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(client_data->vdd) > 0) {
			rc = regulator_set_voltage(client_data->vdd,
				ISL29147_VDD_MIN_UV, ISL29147_VDD_MAX_UV);
			if (rc) {
				dev_err(&client_data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(client_data->vdd);
	return rc;
}


static int sensor_platform_hw_init(struct isl29147_data_t *client_data)
{
	int ret = 0;
	/* h/w init */
	client_data->power_on = sensor_platform_hw_power_on;
	
	ret = sensor_regulator_configure(client_data, true);	
	
	return ret;
}

/* isl29147 probed */
static int isl29147_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err, i;
	u8 reg_dat[16];

	printk(KERN_DEBUG "In isl29147_probe()\n");
	
	/* initial device data struct */	
	isl29147_data.cfg->als_range = 2;
	isl29147_data.client = client;
	isl29147_data.als_pwr_status = 0;
	isl29147_data.ps_pwr_status = 0;
	isl29147_data.poll_delay = 200;
	isl29147_data.show_als_raw = 0;
	isl29147_data.show_ps_raw = 0;
	isl29147_data.ps_filter_cnt = 0;
	isl29147_data.last_lux = 10; 
	isl29147_data.last_ps_raw = 255;
	isl29147_data.als_chg_range_delay_cnt = 0;
	isl29147_data.lux_meter = 1;
	isl29147_data.lux_self = 1;
	spin_lock_init(&isl29147_data.als_timer_lock);
	spin_lock_init(&isl29147_data.ps_timer_lock);
	INIT_WORK(&isl29147_data.als_work, &do_als_work);
	INIT_WORK(&isl29147_data.ps_work, &do_ps_work);
	INIT_WORK(&isl29147_data.calib_work, &do_calib_work);
	INIT_DELAYED_WORK(&isl29147_data.restore_sensordata_work, &do_restore_sensordata_work);
	init_timer(&isl29147_data.als_timer);
	init_timer(&isl29147_data.ps_timer);
		
	isl29147_data.als_wq = create_workqueue("als wq");
	if(!isl29147_data.als_wq) 
	{
		err= -ENOMEM;
		return err;
	}

	isl29147_data.ps_wq = create_workqueue("ps wq");
	if(!isl29147_data.ps_wq) 
	{
		destroy_workqueue(isl29147_data.als_wq);
		err= -ENOMEM;
		return err;
	}
	
	i2c_set_clientdata(client,&isl29147_data);

	err = sensor_platform_hw_init(&isl29147_data);
	
	if(isl29147_data.power_on)
		isl29147_data.power_on(&isl29147_data, true);
	
	/* initial isl29147 */
	err = set_sensor_reg(&isl29147_data);
	if(err < 0) return err;
	
	/* initial als interrupt limit to low = 0, high = 4095, so als cannot
	   trigger a interrupt. We use ps interrupt only */
	reg_dat[7] = 0x00;
	reg_dat[8] = 0x0f;
	reg_dat[9] = 0xff;
	for(i = 7; i <= 9; i++)
	{
		err = i2c_smbus_write_byte_data(client, i, reg_dat[i]);
		if(err < 0) return err;
	}

	schedule_delayed_work(&isl29147_data.restore_sensordata_work, 10*HZ);

	/* Add input device register here */
	err = init_input_dev(&isl29147_data);
	if(err < 0)
	{
		destroy_workqueue(isl29147_data.als_wq);
		destroy_workqueue(isl29147_data.ps_wq);
		return err;
	}

	/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
	/**
	 * Init /sys/class/sensors/psensor device
	 */
	isl29147_data.ps_classdev= isl29147_ps_classdev;
	isl29147_data.ps_classdev.sensors_enable = isl29147_ps_classdev_sensors_enable;
	err = sensors_classdev_register(&client->dev, &isl29147_data.ps_classdev);
	if(err != 0)
	{
		SENSOR_MSG_ERROR("Failed to register isl29147 ps class device!");
		err = -EINVAL;
		return err;
	}
	/* QCI End: ALT Sensor Daniel Wang 20140709 */
	/* Riven add Lisght sensor /sys/class/sensor/ entry */
	isl29147_data.als_classdev= isl29147_als_classdev;
	isl29147_data.als_classdev.sensors_enable = isl29147_als_classdev_sensors_enable;
	err = sensors_classdev_register(&client->dev, &isl29147_data.als_classdev);
	if(err != 0)
	{
	    dev_err(&client->dev, "isl29147-light: Failed to register isl29147 als class device!\n");
	    err = -EINVAL;
	    return err;
	}
	/* Riven add Lisght sensor /sys/class/sensor/ entry */

	return err;
}

static int isl29147_remove(struct i2c_client *client)
{
	struct input_dev *als_dev;
	struct input_dev *ps_dev;

	printk(KERN_INFO "%s at address %d is removed\n",client->name,client->addr);

	/* clean the isl29147 data struct when isl29147 device remove */
	isl29147_data.client = NULL;
	isl29147_data.als_pwr_status = 0;
	isl29147_data.ps_pwr_status = 0;

	als_dev = isl29147_data.als_input_dev;
	ps_dev = isl29147_data.ps_input_dev;

	sysfs_remove_group(&als_dev->dev.kobj, &als_attr_grp);
	sysfs_remove_group(&ps_dev->dev.kobj, &ps_attr_grp);

	input_unregister_device(als_dev);
	input_unregister_device(ps_dev);
	
	destroy_workqueue(isl29147_data.ps_wq);
	destroy_workqueue(isl29147_data.als_wq);

	isl29147_data.als_input_dev = NULL;
	isl29147_data.ps_input_dev = NULL;

	return 0;
}

#ifdef CONFIG_PM	
/* if define power manager, define suspend and resume function */
static int isl29147_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct isl29147_data_t *dat;
	int ret;

	dat = i2c_get_clientdata(client);
	
	spin_lock(&dat->als_timer_lock);
	dat->als_pwr_before_suspend = dat->als_pwr_status;
	spin_unlock(&dat->als_timer_lock);
	ret = set_als_pwr_st(0, dat);
	if(ret < 0) return ret;
	
	spin_lock(&dat->ps_timer_lock);
	dat->ps_pwr_before_suspend = dat->ps_pwr_status;
	spin_unlock(&dat->ps_timer_lock);
	ret = set_ps_pwr_st(0, dat);
	if(ret < 0) return ret;

	if(dat->power_on)
			dat->power_on(dat, false);

	return 0;
}

static int isl29147_resume(struct i2c_client *client)
{
	struct isl29147_data_t *dat;
	int ret;

	dat = i2c_get_clientdata(client);

	if(dat->power_on)
		dat->power_on(dat, true);

//	schedule_delayed_work(&isl29147_data.restore_sensordata_work, 2*HZ);

	ret = set_als_pwr_st(dat->als_pwr_before_suspend, dat);
	if(ret < 0) return ret;
	
	ret = set_ps_pwr_st(dat->ps_pwr_before_suspend, dat);
	if(ret < 0) return ret;
	
	return 0;
}
#else
#define	isl29147_suspend 	NULL
#define isl29147_resume		NULL
#endif		/*ifdef CONFIG_PM end*/

static const struct i2c_device_id isl29147_id[] = {
	{DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isl29147_id);

/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
/**
 * Of match table
 */
static const struct of_device_id isl29147_of_match_table[] = {
	{ .compatible = "isl,isl29147", },
	{ },
};
/* QCI End: ALT Sensor Daniel Wang 20140709 */

static struct i2c_driver isl29147_driver = {
	.driver = {
		.name	= "isl29147",
		/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
		.of_match_table = isl29147_of_match_table,
		/* QCI End: ALT Sensor Daniel Wang 20140709 */
	},
	.probe			= isl29147_probe,
	.remove			= isl29147_remove,
	.id_table		= isl29147_id,
	.detect			= isl29147_detect,
	//.address_list	= normal_i2c,
	.suspend		= isl29147_suspend,
	.resume			= isl29147_resume
};

/* define IOCTL interface */
static int isl29147_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int isl29147_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long isl29147_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct isl29147_data_t *dat;
	struct isl29147_cfg_t *cfg;
	dat = container_of(filp->f_dentry->d_inode->i_cdev, struct isl29147_data_t, 
		cdev);
	cfg = dat->cfg;

	switch (cmd) 
	{
		case ISL_IOCTL_ALS_ON:
			ret = set_als_pwr_st(1, dat);
			return (ret);
			break;
			
        case ISL_IOCTL_ALS_OFF:	
			ret = set_als_pwr_st(0, dat);
			return (ret);
			break;
			
		case ISL_IOCTL_ALS_DATA:
			return (dat->last_lux);
			break;
			
		case ISL_IOCTL_ALS_CALIBRATE:
			/* don't support als calbrate now */
			return 0;
			break;

		case ISL_IOCTL_CONFIG_GET:			
			ret = copy_to_user((struct isl29147_cfg_t *)arg, cfg, 
				sizeof(struct isl29147_cfg_t));
			if (ret) 
			{
				printk(KERN_ERR "ISL: copy_to_user failed in ioctl config_get\n");
				return -ENODATA;
			}
			return (ret);
			break;
			
        case ISL_IOCTL_CONFIG_SET:
            ret = copy_from_user(cfg, (struct isl29147_cfg_t *)arg,
				sizeof(struct isl29147_cfg_t));

#if(IS_DO_START_UP_CALIB)
			do_startup_calib(dat);
#endif
			return ret;
           	break;
			
		case ISL_IOCTL_PROX_ON:				
			ret = set_ps_pwr_st(1, dat);
			return (ret);
			break;
			
        case ISL_IOCTL_PROX_OFF:
			ret = set_ps_pwr_st(0, dat);
			return (ret);
			break;

		case ISL_IOCTL_PROX_DATA:
			return dat->last_ps_raw;
            break;
			
        case ISL_IOCTL_PROX_EVENT:
			if(dat->last_ps > 0) return 1;
			else return 0;
	        break;
			
		case ISL_IOCTL_PROX_CALIBRATE:
			do_calib(dat);
			ret = copy_to_user((struct isl29147_calib_out_t *)arg, &calib_out, 
				sizeof(struct isl29147_calib_out_t));
			if (ret) 
			{
				printk(KERN_ERR "ISL: copy_to_user failed in ioctl config_get\n");
				return -ENODATA;
			}
			return (ret);			
			break;

/*
		case ISL_IOCTL_PROX_STARTUP_CALIBRATE:
			ret = do_startup_calib(dat);
			ret = put_user(ret, (unsigned int __user *)arg);
			return (ret);
			break;
*/

		case ISL_IOCTL_PROX_GET_ENABLED:
			return put_user(dat->ps_pwr_status, (unsigned long __user *)arg);
			break;
			
		case ISL_IOCTL_ALS_GET_ENABLED:
			return put_user(dat->als_pwr_status, (unsigned long __user *)arg);
			break;


		default:
			return -EINVAL;
			break;
	}
	
	return ret;
}

static struct file_operations isl29147_fops = {
	.owner = THIS_MODULE,
	.open = isl29147_open,
	.release = isl29147_close,
	.unlocked_ioctl = isl29147_ioctl
};


struct i2c_client *isl29147_client;

static int __init isl29147_init(void)
{
	int ret;
		
	printk(KERN_ERR "ISL: comes into isl29147_init\n");

	/* register char device */
	if ((ret = (alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME))) < 0) 
	{
		printk(KERN_ERR "ISL: alloc_chrdev_region() failed in isl29147_init()\n");
		return (ret);
	}

	/* QCI Begin: ALT Sensor Daniel Wang 20140709 */
	isl_class = class_create(THIS_MODULE, CLASS_NAME);
	/* QCI End: ALT Sensor Daniel Wang 20140709 */
	cdev_init(&isl29147_data.cdev, &isl29147_fops);
	isl29147_data.cdev.owner = THIS_MODULE;
	if ((ret = (cdev_add(&isl29147_data.cdev, dev_num, 1))) < 0) 
	{
		printk(KERN_ERR "ISL: cdev_add() failed in isl29147_init()\n");
		return (ret);
	}

	/* register the i2c driver for isl29147 */
	ret = i2c_add_driver(&isl29147_driver);
	if(ret < 0) printk(KERN_ERR "Add isl29147 driver error, ret = %d\n", ret);
	printk(KERN_DEBUG "init isl29147 module\n");
	
	return ret;
}

static void __exit isl29147_exit(void)
{
	printk(KERN_DEBUG "exit isl29147 module\n");
	i2c_del_driver(&isl29147_driver);
	cdev_del(&isl29147_data.cdev);
	unregister_chrdev_region(dev_num, 1);
}


MODULE_AUTHOR("Chen Shouxian");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("isl29147 ambient light sensor driver");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl29147_init);
module_exit(isl29147_exit);
