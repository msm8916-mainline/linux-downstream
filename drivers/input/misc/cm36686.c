/* drivers/input/misc/cm36686.c - cm36686 optical sensors driver
 *
 * Copyright (C) 2014 Capella Microsystems Inc.
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

#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include "lightsensor.h"
#include <linux/slab.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include "cm36686.h"
#include "capella_cm3602.h"
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>

#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <asm/setup.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02

//add by yhj
/* POWER SUPPLY VOLTAGE RANGE */
#define CM36686_VDD_MIN_UV	2700000
#define CM36686_VDD_MAX_UV	3300000
#define CM36686_VI2C_MIN_UV	1750000
#define CM36686_VI2C_MAX_UV	1950000

/* cm36686 polling rate in ms */
#define CM36686_LS_MIN_POLL_DELAY		1
#define CM36686_LS_MAX_POLL_DELAY	1000
#define CM36686_LS_DEFAULT_POLL_DELAY	100

#define CM36686_PS_MIN_POLL_DELAY		1
#define CM36686_PS_MAX_POLL_DELAY	1000
#define CM36686_PS_DEFAULT_POLL_DELAY	100


#define CALIBRATION_FILE_PATH	"/efs/cal_data"
#define CHANGE_SENSITIVITY 20 // in percent

//yhj add for debug 20150911
#define YHJ_DBG 0
#if YHJ_DBG
#define Y_DBG(s,args...)	{printk("////yhj : func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define Y_DBG(s,args...) {}
#endif
//yhj add end  20150911

static int record_init_fail = 0;
//static bool ps_num=0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

static int g_ps_cal_2cm = -1;
static int g_ps_cal_4cm = -1;
static struct sensors_classdev sensors_light_cdev = {
	.name = "cm36686-light",
	.vendor = "Capella",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "5240",
	.resolution = "0.01",
	.sensor_power = "0.26",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	//.delay_msec = CM36283_LS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "cm36686-proximity",
	.vendor = "Capella",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.20",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	//.delay_msec = CM36283_PS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct cm36686_info {
	struct class *cm36686_class;
	struct device *ls_dev;
	struct device *ps_dev;
	struct cm36686_platform_data *pdata;
	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

	//struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;
	int ps_enable;
	int ps_irq_flag;

	int irq;
	
	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_resolution; // represented using a fixed 10(-5) notation
	uint32_t cal_data; // represented using a fixed 10(-5) notation

	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int lightsensor_opened;
	uint8_t slave_addr;

	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	uint32_t current_lux_level;
	uint16_t current_adc;
    uint16_t inte_cancel_set;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;

	uint16_t ls_cmd;

	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work ldwork;
	struct delayed_work pdwork;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
};
struct cm36686_info *lp_info;
bool cal_data_retrieved = false;
static uint16_t ps_cancel_set;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex ps_enable_mutex, ps_disable_mutex, ps_get_adc_mutex;
static struct mutex CM36686_control_mutex;
static int lightsensor_enable(struct cm36686_info *lpi);
static int lightsensor_disable(struct cm36686_info *lpi);
static int initial_cm36686(struct cm36686_info *lpi);
static void psensor_initial_cmd(struct cm36686_info *lpi);
//static int cm36686_power_set(struct cm36686_info *info, bool on);
static int control_and_report(struct cm36686_info *lpi, uint8_t mode, uint16_t param);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm36686_info *lpi = lp_info;
		
	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = &cmd,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },		 
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			Y_DBG("[PS][CM36686 error] , i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				 slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM36686 error]  retry over %d\n",I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm36686_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			Y_DBG("[PS][CM36686 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ALS+PS_ERR][CM36686 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm36686_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[ALS+PS_ERR][CM36686 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM36686] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm36686_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM36686] %s: _cm36686_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[ALS+PS_ERR][CM36686 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm36686_info *lpi = lp_info;
	
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_DATA, als_step);
	if (ret < 0) {
		pr_err(
			"[LS][CM36686 error]%s: _cm36686_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

  
	Y_DBG("[LS][CM36686] : raw adc = 0x%X\n",*als_step);

	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_THDH, high_thd);
	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_THDL, low_thd);

	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	if (data == NULL)
		return -EFAULT;	

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, PS_DATA, data);
	
		
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: _cm36686_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	} else {
		Y_DBG(
			"[PS][CM36686 OK]: _cm36686_I2C_Read_Word OK 0x%04x\n", *data);
	}

	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++)
		for (j = (i + 1); j < size; j++)
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	struct cm36686_info *lpi = lp_info;
	for (i = 0; i < 3; i++) {
		/*wait interrupt GPIO high*/
		while (gpio_get_value(lpi->intr_pin) == 0) {
			msleep(10);
			wait_count++;
			if (wait_count > 12) {
				pr_err("[PS_ERR][CM36686 error]%s: interrupt GPIO low,"
					" get_ps_adc_value\n", __func__);
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0) {
			pr_err("[PS_ERR][CM36686 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}

		if (wait_count < 60/10) {/*wait gpio less than 60ms*/
			msleep(60 - (10*wait_count));
		}
		wait_count = 0;
	}

	/*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);*/
	mid_val = mid_value(value, 3);
	Y_DBG("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);
	*ps_adc=mid_val;
	
	return 0;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm36686_info *lpi = lp_info;
	uint16_t intFlag;
  _cm36686_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag);  
	  
	enable_irq(lpi->irq);
}

static irqreturn_t cm36686_irq_handler(int irq, void *data)
{
	struct cm36686_info *lpi = data;
	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm36686_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static int lightsensor_get_cal_data(struct cm36686_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open calibration data file\n", __func__);
		set_fs(old_fs);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: cal_data = %d\n",
		__func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static void ls_initial_cmd(struct cm36686_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= CM36686_ALS_INT_MASK;
  lpi->ls_cmd |= CM36686_ALS_SD;
  _cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
}

static void psensor_initial_cmd(struct cm36686_info *lpi)
{
	/*must disable p-sensor interrupt befrore IST create*//*disable PS func*/	
  lpi->ps_conf1_val |= CM36686_PS_SD;
  lpi->ps_conf1_val &= CM36686_PS_INT_MASK;  
  _cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);   
  _cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
  _cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_away_thd_set);
  _cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDH,  lpi->ps_close_thd_set);
}

static int psensor_enable(struct cm36686_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_enable_mutex);
	//ps_num=0;
	if ( lpi->ps_enable ) {
		D("[PS][CM36686] %s: already enabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_PS, 1);
	
	mutex_unlock(&ps_enable_mutex);
	return ret;
}

static int psensor_disable(struct cm36686_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&ps_disable_mutex);
	if ( lpi->ps_enable == 0 ) {
		D("[PS][CM36686] %s: already disabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_PS,0);
	/*
      if(ps_num){
	  	D("[PS][CM36686] ////yhj add here  %s: power set 0 ,ps_num=%d\n", __func__,ps_num);
              cm36686_power_set(lpi, 0);
               ps_num=0;
      	}
      	*/
	mutex_unlock(&ps_disable_mutex);
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct cm36686_info *lpi = lp_info;
	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct cm36686_info *lpi = lp_info;
	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct cm36686_info *lpi = lp_info;

	D("[PS][CM36686] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val)
			return psensor_enable(lpi);
		else
			return psensor_disable(lpi);
		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[PS][CM36686 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity",
	.fops = &psensor_fops
};


static int lightsensor_enable(struct cm36686_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	if (lpi->als_enable) {
		D("[LS][CM36686] %s: already enabled\n", __func__);
		ret = 0;
	} else
	
	if (!cal_data_retrieved)
		{
			/* get calibration data */		
			ret = lightsensor_get_cal_data(lpi);
			if (ret < 0 && ret != -ENOENT)
			{
				pr_err("%s: lightsensor_get_cal_data() failed\n",
					__func__);
			}
			else
			{
				cal_data_retrieved = true;
			}
		}
	
  	ret = control_and_report(lpi, CONTROL_ALS, 1);
	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm36686_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	if ( lpi->als_enable == 0 ) {
		D("[LS][CM36686] %s: already disabled\n", __func__);
		ret = 0;
	} else
    ret = control_and_report(lpi, CONTROL_ALS, 0);
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm36686_info *lpi = lp_info;
	int rc = 0;
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM36686 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm36686_info *lpi = lp_info;
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm36686_info *lpi = lp_info;

	/*D("[CM36686] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][CM36686] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][CM36686] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LS][CM36686 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};


static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct cm36686_info *lpi = lp_info;
	int intr_val;

	intr_val = gpio_get_value(lpi->intr_pin);

	get_ps_adc_value(&value);

	ret = sprintf(buf, "DEC ADC[%d], ENABLE = %d, intr_pin = %d\n", value, lpi->ps_enable, intr_val);

	return ret;
}


static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct cm36686_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
		&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	if (ps_en) {
		D("[PS][CM36686] %s: ps_en=%d\n",
			__func__, ps_en);
		psensor_enable(lpi);
	} else
		psensor_disable(lpi);

	return count;
}

//add by yhj
static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{

	struct cm36686_info *lpi = container_of(sensors_cdev,
			struct cm36686_info, ps_cdev);
	int ret;
	if (enable)
		ret = psensor_enable(lpi);
	else
		ret = psensor_disable(lpi);

	return ret;
}
static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

unsigned PS_cmd_test_value;
static ssize_t ps_parameters_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value);

	return ret;
}

static ssize_t ps_parameters_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{

	struct cm36686_info *lpi = lp_info;
	char *token[10];
	int i;

	printk(KERN_INFO "[PS][CM36686] %s\n", buf);
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	lpi->ps_close_thd_set = simple_strtoul(token[0], NULL, 16);
	lpi->ps_away_thd_set = simple_strtoul(token[1], NULL, 16);	
	PS_cmd_test_value = simple_strtoul(token[2], NULL, 16);
	printk(KERN_INFO
		"[PS][CM36686]Set PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		lpi->ps_close_thd_set, lpi->ps_away_thd_set, PS_cmd_test_value);

	D("[PS][CM36686] %s\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_parameters, 0664,
	ps_parameters_show, ps_parameters_store);


static ssize_t ps_conf_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return sprintf(buf, "PS_CONF1 = 0x%04x, PS_CONF3 = 0x%04x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}
static ssize_t ps_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	struct cm36686_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	D("[PS]%s: store value PS conf1 reg = 0x%04x PS conf3 reg = 0x%04x\n", __func__, code1, code2);

  lpi->ps_conf1_val = code1;
  lpi->ps_conf3_val = code2;

	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val );  
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val );

	return count;
}
static DEVICE_ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store);

static ssize_t ps_thd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;
  ret = sprintf(buf, "[PS][CM36686]PS Hi/Low THD ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
  return ret;	
}
static ssize_t ps_thd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	struct cm36686_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	lpi->ps_close_thd_set = code1;	
	lpi->ps_away_thd_set = code2;
	
      _cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set );
	  _cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set );
	
	D("[PS][CM36686]%s: ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", __func__, lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
    
	return count;
}


static DEVICE_ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);

static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "[PS][CM36686]PS_CANC = 0x%04x(%d)\n", lpi->inte_cancel_set,lpi->inte_cancel_set);

	return ret;
}
static ssize_t ps_canc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	struct cm36686_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS][CM36686]PS_CANC: store value = 0x%04x(%d)\n", code,code);
	
	lpi->inte_cancel_set = code;	
	_cm36686_I2C_Write_Word(lpi->slave_addr, PS_CANC, lpi->inte_cancel_set );
	
	return count;
}
static DEVICE_ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);

static ssize_t ps_hw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",
		lpi->ps_conf1_val, lpi->ps_conf3_val, lpi->ps_close_thd_set, lpi->ps_away_thd_set);

	return ret;
}
static ssize_t ps_hw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	
	sscanf(buf, "0x%x", &code);

	D("[PS]%s: store value = 0x%x\n", __func__, code);

	return count;
}
static DEVICE_ATTR(ps_hw, 0664, ps_hw_show, ps_hw_store);

static ssize_t ps_cal_2cm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "0x%x\n", g_ps_cal_2cm);

    printk("Now in %s: g_ps_cal_2cm = 0x%x\n", __func__, g_ps_cal_2cm);

	return ret;
}
static ssize_t ps_cal_2cm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
    struct cm36686_info *lpi = lp_info;

    g_ps_cal_2cm = simple_strtol(buf, NULL, 0);
    
	printk("Now in %s: g_ps_cal_2cm = 0x%x\n", __func__, g_ps_cal_2cm);
    lpi->ps_close_thd_set = (uint16_t)g_ps_cal_2cm ;
    _cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_close_thd_set);

	return count;
}
static DEVICE_ATTR(ps_cal_2cm, 0664, ps_cal_2cm_show, ps_cal_2cm_store);

static ssize_t ps_cal_4cm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "0x%x\n", g_ps_cal_4cm);

    printk("Now in %s: g_ps_cal_4cm = 0x%x\n", __func__, g_ps_cal_4cm);

	return ret;
}
static ssize_t ps_cal_4cm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
    struct cm36686_info *lpi = lp_info;

    g_ps_cal_4cm = simple_strtol(buf, NULL, 0);
    lpi->ps_away_thd_set = (uint16_t)g_ps_cal_4cm ;
	printk("Now in %s: g_ps_cal_4cm = 0x%x\n", __func__, g_ps_cal_4cm);
    _cm36686_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_away_thd_set);

	return count;
}
static DEVICE_ATTR(ps_cal_4cm, 0664, ps_cal_4cm_show, ps_cal_4cm_store);
static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;

	D("[LS][CM36686] %s: ADC = 0x%04X, Lux Level = %d \n",
		__func__, lpi->current_adc, lpi->current_lux_level);
	ret = sprintf(buf, "ADC[0x%04X] => Lux Level %d\n",
		lpi->current_adc, lpi->current_lux_level);

	return ret;
}

static ssize_t ls_adb_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	
	sscanf(buf, "0x%x", &code);

	D("[LS]%s: store value = 0x%x\n", __func__, code);

	return count;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, ls_adb_store);
//add by yhj
static int ls_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct cm36686_info *lpi = container_of(sensors_cdev,
			struct cm36686_info, als_cdev);
	int ret;
	if (enable)
		ret = lightsensor_enable(lpi);
	else
		ret = lightsensor_disable(lpi);

	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: set auto light sensor fail\n",
				__func__);
		return -EIO;
	}

	return 0;
}
static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm36686_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM36686] %s: lpi->als_enable = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM36686 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0664,
	ls_enable_show, ls_enable_store);

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return sprintf(buf, "ALS_CONF = %x\n", lpi->ls_cmd);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	printk(KERN_INFO "[LS]set ALS_CONF = %x\n", lpi->ls_cmd);
	
	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
	return count;
}
static DEVICE_ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store);

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct cm36686_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = 100000;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}
static DEVICE_ATTR(ls_cali, 0664, ls_cal_data_show, ls_cal_data_store);
static int lightsensor_setup(struct cm36686_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][CM36686 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "light";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
	lpi->ls_input_dev = NULL;
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static void cm36686_unregister_lsensor_device(struct i2c_client *client, struct cm36686_info *lpi)
{
    input_unregister_device(lpi->ls_input_dev);
}

static int psensor_setup(struct cm36686_info *lpi)
{
	int ret;
	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err(
			"[PS][CM36686 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}
	
	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
	lpi->ps_input_dev = NULL;
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

static void cm36686_unregister_psensor_device(struct i2c_client *client, struct cm36686_info *lpi)
{
    input_unregister_device(lpi->ps_input_dev);
}

static int initial_cm36686(struct cm36686_info *lpi)
{
	int val, ret;
	uint16_t idReg;

	val = gpio_get_value(lpi->intr_pin);
	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
	idReg &= 0xFF;
	if ((ret < 0) || (idReg != 0x0086)) {
  		if (record_init_fail == 0)
  			record_init_fail = 1;
  		return -ENOMEM;/*If devices without cm36686 chip and did not probe driver*/	
  }
  
	return 0;
}

static int cm36686_setup(struct cm36686_info *lpi)
{
	int ret = 0;
	als_power(1);
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_cm36686_intr");
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}
	//add by yhj 
	lpi->irq = gpio_to_irq(lpi->intr_pin);

	ret = initial_cm36686(lpi);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][CM36686 error]%s: fail to initial cm36686 (%d)\n",
			__func__, ret);
		goto fail_free_intr_pin;
	}
	
	/*Default disable P sensor and L sensor*/
  	ls_initial_cmd(lpi);
	psensor_initial_cmd(lpi);
	
	ret = request_threaded_irq(lpi->irq,
				     NULL, cm36686_irq_handler,
				     IRQF_TRIGGER_LOW |IRQF_ONESHOT,
				     "cm36686", lpi);

	
	/*
	ret = request_any_context_irq(lpi->irq,
			cm36686_irq_handler,
			IRQF_TRIGGER_LOW,
			"cm36686",
			lpi);
	*/
	if (ret < 0) {
		pr_err(
			"[PS][CM36686 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

/*
static void cm36686_early_suspend(struct early_suspend *h)
{
	struct cm36686_info *lpi = lp_info;

	D("[LS][CM36686] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
	
	if (lpi->ps_enable)
		psensor_disable(lpi);
} */

/*static void cm36686_late_resume(struct early_suspend *h)
{
	struct cm36686_info *lpi = lp_info;

	D("[LS][CM36686] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
		
		if (!lpi->ps_enable)
		psensor_enable(lpi);
}
*/
	
//add by yhj
static int cm36686_parse_dt(struct device *dev,
				struct cm36686_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	Y_DBG("[PS][CM36686] %s: Probe start!\n", __func__);
	
	rc = of_get_named_gpio_flags(np, "capella,interrupt-gpio",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} else {
		pdata->intr = rc;
	}
	
	rc = of_property_read_u32(np, "capella,ps_close_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_close_thd_set\n");
		return rc;
	} else {
		pdata->ps_close_thd_set = (u8)temp_val;
	}
	
	rc = of_property_read_u32(np, "capella,ps_away_thd_set", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_away_thd_set\n");
		return rc;
	} else {
		pdata->ps_away_thd_set = (u8)temp_val;
	}
	
	rc = of_property_read_u32(np, "capella,ls_cmd", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ls_cmd\n");
		return rc;
	} else {
		pdata->ls_cmd = (u16)temp_val;
	}
	
	rc = of_property_read_u32(np, "capella,ps_conf1_val", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_conf1_val\n");
		return rc;
	} else {
		pdata->ps_conf1_val = (u16)temp_val;
	}
	
	rc = of_property_read_u32(np, "capella,ps_conf3_val", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ps_conf3_val\n");
		return rc;
	} else {
		pdata->ps_conf3_val = (u16)temp_val;
	}

	return 0;
}


static int cm36686_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm36686_info *lpi;
	struct cm36686_platform_data *pdata;

	printk("///yhj add in %s start.\n",__func__);
	
	lpi = kzalloc(sizeof(struct cm36686_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/*D("[CM36686] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory for pdata\n");
			ret = -ENOMEM;
			goto err_platform_data_null;
		}

		ret = cm36686_parse_dt(&client->dev, pdata);
		pdata->slave_addr = client->addr;
		if (ret) {
			dev_err(&client->dev, "Failed to get pdata from device tree\n");
			goto err_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "%s: Assign platform_data error!!\n",
					__func__);
			ret = -EBUSY;
			goto err_platform_data_null;
		}
	}
	
	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);
	
 	lpi->intr_pin = pdata->intr;
	lpi->power = pdata->power;
	
	lpi->slave_addr = pdata->slave_addr;
	
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	ps_cancel_set = lpi->inte_cancel_set;
	
	lpi->ls_cmd  = pdata->ls_cmd;
	
		
	D("[PS][CM36686] %s: ls_cmd 0x%x\n",
		__func__, lpi->ls_cmd);
	
	if (pdata->ls_cmd == 0) {
		lpi->ls_cmd  = CM36686_ALS_IT_160MS | CM36686_ALS_PERS_2;
	}

	lp_info = lpi;

	mutex_init(&CM36686_control_mutex);
	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM36686 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	mutex_init(&ps_enable_mutex);
	mutex_init(&ps_disable_mutex);
	mutex_init(&ps_get_adc_mutex);

	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][CM36686 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}

	ret = lightsensor_get_cal_data(lpi);

		//set the default ALS cal data, no lens condition
			lpi->cal_data = 100000;
				
		//set the chip resoultion, please refer to datasheet	
		lpi->als_resolution = 4000;

	lpi->lp_wq = create_singlethread_workqueue("cm36686_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][CM36686 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
	/*
	//add by yhj 
	ret = cm36686_power_set(lpi, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s:cm36686 power on error!\n", __func__);
		goto err_cm36686_power_on;
	}
	*/

	ret = cm36686_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM36686 error]%s: cm36686_setup error!\n", __func__);
		goto err_cm36686_setup;
	}
	lpi->cm36686_class = class_create(THIS_MODULE, "capella_sensors");
	if (IS_ERR(lpi->cm36686_class)) {
		ret = PTR_ERR(lpi->cm36686_class);
		lpi->cm36686_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm36686_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf);
	if (ret)
		goto err_create_ls_device_file;

	/* register the attributes */	
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_cali);
	if (ret)
		goto err_create_ls_device_file;	

	lpi->ps_dev = device_create(lpi->cm36686_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ps_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);
	if (ret)
		goto err_create_ps_device_file;

	ret = device_create_file(lpi->ps_dev,
		&dev_attr_ps_parameters);
	if (ret)
		goto err_create_ps_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf);
	if (ret)
		goto err_create_ps_device_file;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_thd);
	if (ret)
		goto err_create_ps_device_file;
		
	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_canc);
	if (ret)
		goto err_create_ps_device_file;
		

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);
	if (ret)
		goto err_create_ps_device_file;


    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_cal_2cm);
    if (ret)
        goto err_create_ps_device_file;
            
    ret = device_create_file(lpi->ps_dev, &dev_attr_ps_cal_4cm);
    if (ret)
        goto err_create_ps_device_file;

	//lpi->early_suspend.level =
	//		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	//lpi->early_suspend.suspend = cm36686_early_suspend;
	//lpi->early_suspend.resume = cm36686_late_resume;
	//register_early_suspend(&lpi->early_suspend);
	
	lpi->als_cdev = sensors_light_cdev;
	lpi->als_cdev.sensors_enable = ls_enable_set;
	//lpi->als_cdev.sensors_poll_delay = ls_poll_delay_set;
	//lpi->als_cdev.min_delay = CM36686_LS_MIN_POLL_DELAY * 1000;

	lpi->ps_cdev = sensors_proximity_cdev;
	lpi->ps_cdev.sensors_enable = ps_enable_set;
	//lpi->ps_cdev.sensors_poll_delay = ps_poll_delay_set;
	//lpi->ps_cdev.min_delay = CM36686_PS_MIN_POLL_DELAY * 1000;

	ret = sensors_classdev_register(&client->dev, &lpi->als_cdev);
	if (ret)
		goto err_remove_als_cdev;

	ret = sensors_classdev_register(&client->dev, &lpi->ps_cdev);
	if (ret)
		goto err_remove_ps_cdev;

	D("[PS][CM36686] %s: Probe success!\n", __func__);

	return ret;
	
err_remove_als_cdev:
	sensors_classdev_unregister(&lpi->als_cdev);
err_remove_ps_cdev:
	 sensors_classdev_unregister(&lpi->ps_cdev);
err_create_ps_device_file:
err_create_ps_device:
	device_unregister(lpi->ps_dev);
err_create_ls_device_file:
err_create_ls_device:
	device_unregister(lpi->ls_dev);
err_create_class:
	class_destroy(lpi->cm36686_class);
err_cm36686_setup:
/*
//add by yhj
err_cm36686_power_on:
	cm36686_power_set(lpi, false);
//add end
*/
	wake_lock_destroy(&(lpi->ps_wake_lock));
err_create_singlethread_workqueue:
	destroy_workqueue(lpi->lp_wq);
err_psensor_setup:
	cm36686_unregister_psensor_device(client,lpi);
	mutex_destroy(&ps_enable_mutex);
	mutex_destroy(&ps_disable_mutex);
	mutex_destroy(&ps_get_adc_mutex);
err_lightsensor_setup:
	cm36686_unregister_lsensor_device(client,lpi);
	mutex_destroy(&CM36686_control_mutex);
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
//add by yhj
err_parse_dt:
	if (client->dev.of_node && (pdata != NULL))
		devm_kfree(&client->dev, pdata);
err_platform_data_null:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct cm36686_info *lpi, uint8_t mode, uint16_t param ) {
	int ret=0;
	uint16_t adc_value = 0;
	uint16_t ps_data = 0;
	uint32_t lux_level = 0;
	uint16_t low_thd;
	uint32_t high_thd;	
	uint16_t sdata;
	int val;
  mutex_lock(&CM36686_control_mutex);
   if( mode == CONTROL_ALS ){
    if(param){
      lpi->ls_cmd &= CM36686_ALS_SD_MASK;      
    } else {
      lpi->ls_cmd |= CM36686_ALS_SD;
    }
    _cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
    lpi->als_enable=param;
  } else if( mode == CONTROL_PS ){
    if(param){ 
      lpi->ps_conf1_val &= CM36686_PS_SD_MASK;
      lpi->ps_conf1_val |= CM36686_PS_INT_IN_AND_OUT;      
    } else {
      lpi->ps_conf1_val |= CM36686_PS_SD;
      lpi->ps_conf1_val &= CM36686_PS_INT_MASK;
    }
    _cm36686_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val); 
	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, PS_CONF1, &sdata);
    lpi->ps_enable=param;  
  }
  if((mode == CONTROL_ALS)||(mode == CONTROL_PS)){  
    if( param==1 ){
		  msleep(100);  
    }
  }
     	
  if(lpi->als_enable){
    if( mode == CONTROL_ALS ||
      ( mode == CONTROL_INT_ISR_REPORT && 
      ((param&INT_FLAG_ALS_IF_L)||(param&INT_FLAG_ALS_IF_H)))){
    
    	  lpi->ls_cmd &= CM36686_ALS_INT_MASK;
    	  ret = _cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
      
        get_ls_adc_value(&adc_value, 0);
		lux_level = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);

		// set interrupt high/low threshold		
		low_thd = (uint16_t)((uint32_t)adc_value * (100 - CHANGE_SENSITIVITY) / 100);
		high_thd = (uint32_t)adc_value * (100 + CHANGE_SENSITIVITY) / 100;
		if (high_thd > 65535)
		{
			high_thd = 65535;
		}
      
    
		ret = set_lsensor_range(low_thd, (uint16_t)high_thd);	
    		   	    	  
        lpi->ls_cmd |= CM36686_ALS_INT_EN;
    	  
        ret = _cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
    	  
    		
    	Y_DBG("[LS][CM36686] %s: ADC=0x%03X, Lux Level=%d, l_thd = 0x%x, h_thd = 0x%x \n",
			__func__, adc_value, lux_level, low_thd, high_thd);
    		lpi->current_lux_level = lux_level;
			lpi->current_adc = adc_value;       
        input_report_abs(lpi->ls_input_dev, ABS_MISC, adc_value);
        input_sync(lpi->ls_input_dev);
    }
  }

#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY
   if(lpi->ps_enable){
    int ps_status = 0;
    if( mode == CONTROL_PS )
      ps_status = PS_CLOSE_AND_AWAY;   
    else if(mode == CONTROL_INT_ISR_REPORT ){  
      if ( param & INT_FLAG_PS_IF_CLOSE )
        ps_status |= PS_CLOSE;      
      if ( param & INT_FLAG_PS_IF_AWAY )
        ps_status |= PS_AWAY;
    }
	Y_DBG("[PS][CM36686] %s,ps_status=%d\n", __func__,ps_status);      
    if (ps_status!=0){
      switch(ps_status){
        case PS_CLOSE_AND_AWAY:
		  get_stable_ps_adc_value(&ps_data);   
          val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
          break;
        case PS_AWAY:
          val = 1;
		  D("[PS][CM36686] proximity detected object away\n");
		  break;
        case PS_CLOSE:
          val = 0;
		  D("[PS][CM36686] proximity detected object close\n");
          break;
        };
	  Y_DBG("[PS][CM36686] %s,ps_val=%d\n", __func__,val);      
      input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);         
      input_sync(lpi->ps_input_dev);        
    }
  }

  mutex_unlock(&CM36686_control_mutex);
  return ret;
}
/*
//add by yhj
static int cm36686_power_set(struct cm36686_info *info, bool on)
{
	int rc;
	info->vdd = regulator_get(&info->i2c_client->dev, "vdd");
	info->vio = regulator_get(&info->i2c_client->dev, "vio");
	if (on) {
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}

		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,
					CM36686_VDD_MIN_UV, CM36686_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}

		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				CM36686_VI2C_MIN_UV, CM36686_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}

		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}

		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}

	} else {
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0, CM36686_VDD_MAX_UV);

		regulator_put(info->vdd);

		rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0,
					CM36686_VI2C_MAX_UV);

		regulator_put(info->vio);
	}

	return 0;

err_vio_ena:
	regulator_disable(info->vdd);
err_vdd_ena:
	if (regulator_count_voltages(info->vio) > 0)
		regulator_set_voltage(info->vio, 0, CM36686_VI2C_MAX_UV);
err_vio_set_vtg:
	regulator_put(info->vio);
err_vio_get:
	if (regulator_count_voltages(info->vdd) > 0)
		regulator_set_voltage(info->vdd, 0, CM36686_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(info->vdd);
err_vdd_get:
	return 1;
}
*/
//add by yhj
#ifdef CONFIG_PM_SLEEP
static int cm36686_suspend(struct device *dev)
{
	struct cm36686_info *lpi = lp_info;
	 D("[LS][CM36686] %s ,als_enable=%d,ps_enable=%d\n", __func__,lpi->als_enable,lpi->ps_enable);
	if (lpi->als_enable) {
		if (lightsensor_disable(lpi))
			goto out;
		lpi->als_enable = 1;
	}
	/*
	if(lpi->ps_enable){
		ps_num=1;
		D("[LS][CM36686]%s ,ps_num=%d.\n", __func__,ps_num);
		return 0;
	}else{
		disable_irq(lpi->irq);
	}

	if (cm36686_power_set(lpi, 0))
		goto out;
	*/
	return 0;

out:
	dev_err(&lpi->i2c_client->dev, "%s:failed during resume operation.\n",
			__func__);
	return -EIO;
}

static int cm36686_resume(struct device *dev)
{
	struct cm36686_info *lpi = lp_info;
	D("[LS][CM36686] %s ,als_enable=%d,ps_enable=%d\n", __func__,lpi->als_enable,lpi->ps_enable);
	/*
	if(!lpi->ps_enable){
		if (cm36686_power_set(lpi, 1))
			goto out;
		enable_irq(lpi->irq);	
	}
	*/

	if (lpi->als_enable) {
		ls_initial_cmd(lpi);
		psensor_initial_cmd(lpi);
		if (lightsensor_enable(lpi))
			goto out;
	}
	return 0;

out:
	dev_err(&lpi->i2c_client->dev, "%s:failed during resume operation.\n",
			__func__);
	return -EIO;
}
#endif
static UNIVERSAL_DEV_PM_OPS(cm36686_pm, cm36686_suspend, cm36686_resume, NULL);
static const struct i2c_device_id cm36686_i2c_id[] = {
	{CM36686_I2C_NAME, 0},
	{}
};
//add by yhj
static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "capella,cm36686",},
	{ },
};
static struct i2c_driver cm36686_driver = {
	.id_table = cm36686_i2c_id,
	.probe = cm36686_probe,
	.driver = {
		.name = CM36686_I2C_NAME,
		.owner = THIS_MODULE,
		//add by yhj
		.pm = &cm36686_pm,
		.of_match_table = cm36686_match_table,
	},
};

static int __init cm36686_init(void)
{
	return i2c_add_driver(&cm36686_driver);
}

static void __exit cm36686_exit(void)
{
	i2c_del_driver(&cm36686_driver);
}

module_init(cm36686_init);
module_exit(cm36686_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36686 Driver");
