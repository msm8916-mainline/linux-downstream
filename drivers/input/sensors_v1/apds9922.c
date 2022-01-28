/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* vivo Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/apds9920.c - APDS9920 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#endif
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#if defined(CONFIG_BBK_DEVICE_INFO)
#include <linux/bbk_drivers_info.h>
#endif

#include <linux/regulator/consumer.h>
#include <linux/sensors_io.h>
#include <linux/bbk_alsps_para.h>
#include <linux/sensors.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <asm/io.h>
#include <linux/jiffies.h>
#include <linux/vivo_sensor_config.h>
#include "apds9922.h"

#define APDS9922_DEV_NAME     "APDS9922"
#define TAG  "PAsensor "
#define TAGI "PAsensor.I "
#define TAGE "PAsensor.E "
extern void print_vivo_init(const char *fmt, ...);
#define CONT_INT_TIME     50    //50ms  	
#define POLL_DELAY_TIMES  2     //2X 
#define CONT_INT_GATE   20
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0

#define DEFAULT_STRONG_SUNLIGHT   10000
#define ALS_LOW_LUX_MODE		0
#define ALS_NORMAL_LUX_MODE		1
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
    CMC_BIT_ENG_ALS = 3,
    CMC_BIT_ENG_PS  = 4,
} CMC_BIT;

#define I2C_FLAG_WRITE 0
#define I2C_FLAG_READ	1
static int debug = 0;

#define ALS_MIN_POLL_DELAY      150
#define ALS_DEFAULT_POLL_DELAY	120

#define PS_MIN_POLL_DELAY       10
#define PS_DEFAULT_POLL_DELAY   120

static struct i2c_client *apds9922_i2c_client = NULL;
extern int qup_i2c_suspended; // add for i2c timeout   
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id apds9922_i2c_id[] = {{APDS9922_DEV_NAME,0},{}};


/*----------------------------------------------------------------------------*/
struct apds9922_i2c_addr { /*define a series of i2c slave address*/
	u8 write_addr;  
	u8 ps_thd; /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct apds9922_priv {
	struct i2c_client *client;
	struct workqueue_struct *eint_queue;
	struct work_struct eint_work;

	/*i2c address group*/
	struct apds9922_i2c_addr addr;

	/*misc*/
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t ps_suspend;

	/*data*/
	u16 als;
	u16 ps;
	ulong enable;            /*enable mask*/

	int first_int_enable;
	u32 		strong_sunlight;
	atomic_t    pulse_value;
	atomic_t	sys_prox_status;          //current prox state,  1: far away 0:
	atomic_t	ps_in_threshold;          //the threshold used now
	atomic_t    ps_out_threshold;

	atomic_t	ps_status;                //current prox state,  1: far away 0:close
	atomic_t    ps_base_value; 
	atomic_t    ps_first_int;
	int			last_ps_data;
	int         last_als_data;
      atomic_t    suspend;
	char 	write_register[20];
	char		read_register[20];
	struct wake_lock wakelock;
   /*early suspend*/
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif

    struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

    struct input_dev *input_dev_als;	
	struct input_dev *input_dev_ps; 

    struct workqueue_struct *poll_queue;
    struct delayed_work		 ps_report_work;
    struct delayed_work		 als_report_work;

    int gpio_int;
    int irq;
    atomic_t als_poll_delay;
    atomic_t ps_poll_delay;
};

static int apds9922_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int apds9922_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int apds9922_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9922_i2c_resume(struct i2c_client *client);

int apds9922_enable_eint(struct apds9922_priv *obj,int enable);
static int apds9922_init_client(struct i2c_client *client);

int apds9922_read_ps(struct i2c_client *client, u16 *data);

static long apds9922_enable_als(struct i2c_client *client, int enable);
static long apds9922_enable_ps(struct i2c_client *client, int enable);

static int ps_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable);
static int als_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable);
static void apds9922_ps_data_report(struct work_struct *work);

static struct i2c_driver apds9922_i2c_driver = {	
	.probe      = apds9922_i2c_probe,
	.remove     = apds9922_i2c_remove,
	//.detect     = apds9922_i2c_detect,
	.suspend    = apds9922_i2c_suspend,
	.resume     = apds9922_i2c_resume,
	.id_table   = apds9922_i2c_id,
	.driver = {
		.name     = APDS9922_DEV_NAME,
	},
};

static struct apds9922_priv *apds9922_obj = NULL;
static struct platform_driver apds9922_alsps_driver;
static struct mutex apds9922_lock;


static struct sensors_classdev sensors_light_cdev0 = {
	.name = "APDS9922-light",
	.vendor = "Avago",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6553",
	.resolution = "0.0125",
	.sensor_power = "0.15",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = ALS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev0 = {
	.name = "APDS9922-proximity",
	.vendor = "Avago",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.18",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = PS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct ps_para bbk_ps_paras_default[] = bbk_ps_para();
static struct als_para bbk_als_paras_default[] = bbk_als_para();
struct ps_para  *apds9922_ps_para = NULL;
struct als_para *apds9922_als_para = NULL;
static int ps_para_num  = 1;          //the value must select early
static int als_para_num = 1;
//static unsigned pulse = 0x04;

static int nAlsReadCount = 0;
static unsigned short stronglight_flag = 0;
static int debug_flag=0;
static int alsCount = 0;
static int PS_ESD_TIME=10;
static int unenable_and_read_count=0;
static struct mutex ps_report_lock = __MUTEX_INITIALIZER(ps_report_lock);



static DEFINE_MUTEX(apds9922_mutex);
static struct apds9922_priv *g_apds9922_ptr = NULL;
static struct wake_lock ps_suspend_lock;

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator apds9922_vreg[] = {
	{NULL, "vdd", 2850000, 2850000},
	{NULL, "vddio", 1800000, 1800000},
};

/******************************************************************************
* local code
******************************************************************************/
static void set_ps_para_num(int para)
{
	int ps_para_count = bbk_ps_para_count;
	if(ps_para_count == 0)
		ps_para_count = ARRAY_SIZE(bbk_ps_paras_default);
	if( para >= 0 && para<= (ps_para_count -1))
		ps_para_num=para;
	else
		printk(KERN_ERR TAGE "%d is wrong ps index, 0, %d \n", para, ps_para_count -1);
}

static void set_als_para_num(int para)
{
	int als_para_count = bbk_als_para_count;
	if(als_para_count == 0)
		als_para_count = ARRAY_SIZE(bbk_als_paras_default);
	if( para >= 0 && para <= (als_para_count -1 ))
		als_para_num=para;
	else
		printk(KERN_ERR TAGE "%d is wrong als index, 0, %d \n", para, als_para_count -1);
	
}

static int apds9922_read_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
    u8 temp_address;
    u8 *temp_data;
    int res = 0;
    if(qup_i2c_suspended > 0){
        printk(KERN_INFO TAGI" apds9922 read reg when i2c bus is suspended.\n");
        return -1;		
    }
    mutex_lock(&apds9922_lock); 

    temp_address = addr;
    temp_data = data;

    res = i2c_master_send(client, &temp_address, 0x1);
    if(res < 0)
    {
        if(temp_address != APDS9922_DD_PART_ID_ADDR)
           printk(KERN_ERR TAGE "apds9920 read 0x%x fail %d\n", temp_address, res);
        mutex_unlock(&apds9922_lock); 
        return res;
    }
    
    res = i2c_master_recv(client, temp_data, count);
    if(res < 0)
    {
	  if(temp_address != APDS9922_DD_PART_ID_ADDR)
            printk(KERN_ERR TAGE"apds9920 read 0x%x fail %d\n", temp_address, res);
        mutex_unlock(&apds9922_lock); 
        return res;
    }
    
    mutex_unlock(&apds9922_lock); 
    
    return count;
}


static int apds9922_write_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
    u8 buffer[10] = {0};
    int res = 0;
    if(qup_i2c_suspended > 0){
        printk(KERN_INFO TAGI" apds9922 write reg when i2c bus is suspended.\n");
        return -1;		
    }
    mutex_lock(&apds9922_lock); 

    buffer[0] = addr;
    if(data != NULL && count > 0 && count < 5)
    {
        memcpy(&buffer[1], data, count);
    }

    res = i2c_master_send(client, buffer, (0x01 + count));
    if(res < 0)
    {
        printk(KERN_ERR TAGE "apds9920 write 0x%x fail %d\n", buffer[0], res);
        mutex_unlock(&apds9922_lock); 
        return res;
    }

    mutex_unlock(&apds9922_lock); 
    
    return (count + 1);
}

#if 0
int apds9922_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	
	mutex_lock(&apds9922_lock);

	switch (i2c_flag) {
		case I2C_FLAG_WRITE:
			client->addr &=I2C_MASK_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		case I2C_FLAG_READ:
			client->addr &=I2C_MASK_FLAG;
			client->addr |=I2C_WR_FLAG;
			client->addr |=I2C_RS_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		default:
			printk(KERN_INFO TAGI "apds9922_i2c_master_operate i2c_flag command not support!\n");
			break;
	}

	if (res <= 0) {
		goto EXIT_ERR;
	}

	mutex_unlock(&apds9922_lock);
	return res;

EXIT_ERR:
	mutex_unlock(&apds9922_lock);
	printk("apds9922_i2c_transfer fail\n");
	return res;
}

#endif

int apds9922_read_als(struct i2c_client *client, u16 *data)
{	 
//	struct apds9922_priv *obj = i2c_get_clientdata(client);
	u32 clr_value, als_value,sf;	 
	u8 buffer[3];
	u16 als_time;
	u16 als_gain;
	u64 als_tmp_lux;
	int res = 0;

	if(nAlsReadCount < 10) {
		nAlsReadCount += 1;
	}
	
	if (client == NULL) {
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	buffer[0]=APDS9922_DD_CLEAR_DATA_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x3);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	clr_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	//APS_LOG("clr_value=%d\n", clr_value);
	//  printk(KERN_INFO TAGI "clr_value=%d\n", clr_value);

	buffer[0]=APDS9922_DD_ALS_DATA_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x3);
	if (res <= 0){ 
		goto EXIT_ERR;
	}
	
	als_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	if(debug)
	    printk(KERN_INFO TAGI "als_value=%d\n", als_value);
       
	buffer[0]=APDS9922_DD_ALS_MEAS_RATE_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	als_time = (buffer[0]>>4)&0x07;

	if (als_time == 0)
		als_time = 400;
	else if (als_time == 1)
		als_time = 200;
	else if (als_time == 2)
		als_time = 100;
	else if (als_time == 3)
		als_time = 50;
	else
		als_time = 25;

	buffer[0]=APDS9922_DD_ALS_GAIN_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	als_gain = buffer[0]&0x07;

	if (als_gain == 0)
		als_gain = 1;
	else if (als_gain == 1)
		als_gain = 3;
	else if (als_gain == 2)
		als_gain = 6;
	else if (als_gain == 3)
		als_gain = 9;
	else
		als_gain = 18;
	sf = als_time*als_gain;
       if(als_value <5000)
       {  
             als_tmp_lux = (als_value*APDS9922_DD_LUX_FACTOR*apds9922_als_para[als_para_num].base_value*10);
             if(debug)
		    printk(KERN_INFO TAGI "als_tmp_lux = %lld als_value = %d  base = %d \r\n",als_tmp_lux,als_value,
		    apds9922_als_para[als_para_num].base_value);
		
		do_div(als_tmp_lux,sf);
		if(als_tmp_lux > 5 && als_tmp_lux < 1000) {
		      als_tmp_lux = 1010;
	      }
		//*data = als_tmp_lux/1000;
		do_div(als_tmp_lux,1000);
		*data = (u16)als_tmp_lux;
       }
       else{
	   als_tmp_lux = als_value*APDS9922_DD_LUX_FACTOR;
	   do_div(als_tmp_lux,sf);
	   als_tmp_lux = apds9922_als_para[als_para_num].base_value * als_tmp_lux;
	   do_div(als_tmp_lux,100);
	   if(als_tmp_lux > 30000)
	   	als_tmp_lux = 30000;
	   *data = (u16)als_tmp_lux;
	 }
	
	if(debug)
	    printk(KERN_INFO TAGI "apds9922_read_als als_value_lux = %d\n", *data);
	return 0;	 

EXIT_ERR:
	printk(KERN_ERR TAGE"apds9922_read_als fail\n");
	return res;
}

int apds9922_read_als0(struct i2c_client *client, u32 *data)
{	 
	//struct apds9922_priv *obj = i2c_get_clientdata(client);
	u32 clr_value, als_value;	 
	u8 buffer[3];
	//u16 als_time;
//	u16 als_gain;
	int res = 0;

	if (client == NULL) {
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	buffer[0]=APDS9922_DD_CLEAR_DATA_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x3);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	clr_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	//APS_LOG("clr_value=%d\n", clr_value);

	buffer[0]=APDS9922_DD_ALS_DATA_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x3);
	if (res <= 0){ 
		goto EXIT_ERR;
	}
	
	als_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	*data = als_value;
	if(debug)
	    printk(KERN_INFO TAGI "apds9922_read_als0 als_value = %d\n", *data);
	return 0;	 

EXIT_ERR:
	printk(KERN_ERR TAGE"apds9922_read_als fail\n");
	return res;
}

int apds9922_read_ps(struct i2c_client *client, u16 *data)
{
//	struct apds9922_priv *obj = i2c_get_clientdata(client);	
	u8 buffer[2];
	u16 ps_data;
	long res = 0;

	if (client == NULL) {
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=APDS9922_DD_PRX_DATA_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x2);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	ps_data = (buffer[0] | (buffer[1]<<8))&0x7FF;
	*data = ps_data;
	if(debug)
		printk(KERN_INFO TAGI "%s data = %d\r\n",__func__,*data);
	return 0;    

EXIT_ERR:
	printk(KERN_ERR TAGE "apds9921_read_ps fail\n");
	return res;
}

static int apds9922_check_intr(struct i2c_client *client) 
{
	int res, intp, intl;
	u8 buffer[2];

	buffer[0] = APDS9922_DD_MAIN_STATUS_ADDR;
	//res = apds9922_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	res = apds9922_read_reg(client, buffer[0], buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	res = 0;
	intp = 0;
	intl = 0;

	if (0 != (buffer[0] & APDS9922_DD_PRX_INT_STATUS)) {
		res = 0;
		intp = 1;
	}
	
	if(0 != (buffer[0] & APDS9922_DD_ALS_INT_STATUS)) {
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	printk(KERN_INFO TAGI "apds9922_check_intr fail\n");
	return 1;
}

static int apds9922_ps_select_para(struct apds9922_priv *obj)         //
{
    int out_threshold, in_threshold;   
    int base = atomic_read(&obj->ps_base_value);
    int step_num;
    const struct ps_step *step;
    const struct ps_para *para;
    
    //TODO: get different parameter here
    para = &apds9922_ps_para[ps_para_num];
    step = &para->step[0];
    
    //count the the threshold from base value
    if(!(base >= para->base_value_min && base <= para->base_value_max))
        base = para->base_value;
    
    for(step_num = 0; step_num < para->step_num; step_num++, step++)
    {
        if(base >= step->min && base <= step->max)
            break;
    }
    
    if(PS_OPERATION)
    {
        in_threshold  = base + step->in_coefficient;
        out_threshold = base + step->out_coefficient;
    }
    else
    {
        in_threshold  = base * (step->in_coefficient) / 100;
        out_threshold = base * (step->out_coefficient) / 100;
    }
    printk(KERN_INFO TAGI "%s ,select_threshold in_coefficient %d out_coefficient %d\n",__func__, step->in_coefficient, step->out_coefficient);


    printk(KERN_INFO TAGI "select_threshold in %d out %d base %d\n", in_threshold, out_threshold, base);

    atomic_set(&obj->ps_in_threshold,  in_threshold);
    atomic_set(&obj->ps_out_threshold, out_threshold);
    atomic_set(&obj->ps_base_value,    base);           //reset base value
	printk(KERN_INFO TAGI "ps_in_threshold %d ps_out_threshold %d\n", atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
	return 0; 
}

/*-----------------------------------------------------------------------------*/
static int apds9922_als_select_para(struct apds9922_priv *obj)
{
    //TODO: get different parameter here
    //als_para_num = 0; 

    return 0; 
}

static int apds9922_get_ps_value(struct apds9922_priv *obj, u16 ps)
{
	int val;
	if( ps > atomic_read(&obj->ps_in_threshold) ) {
		val = 0;
	} else if( ps < atomic_read(&obj->ps_out_threshold) ) {
		val = 1;
	} else {
		val = atomic_read(&obj->ps_status);
	}
	if(debug) {
		printk(KERN_INFO TAGI "%s,ps_value = %d  ps_status=%d \n", __func__, ps, atomic_read(&obj->ps_status));
	}
	if( val != atomic_read(&obj->ps_status) ) {

		printk(KERN_INFO TAGI "%s change ps(%d) status from %d to %d,in=%d out=%d\n",
		       __func__, ps, atomic_read(&obj->ps_status), val, atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
		atomic_set(&obj->ps_status, val);
		if(test_bit(CMC_BIT_PS, &obj->enable) || test_bit(CMC_BIT_ENG_PS, &obj->enable) ) {
			apds9922_enable_eint(obj, 1);
		}
	}
	return val;

}
static int apds9922_enable_als_moniter_eint(struct i2c_client *client, int enable)      //the eint will be triggered in strong lux
{
	struct apds9922_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;
	//u8 buffer[1];
	//u8 reg_value[1];

	databuf[0] = APDS9922_DD_ALS_THRES_LOW_0_ADDR;
	databuf[1] = (u8)(0 & 0x00FF);
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_ALS_THRES_LOW_1_ADDR;
	databuf[1] = (u8)((0 & 0xFF00) >> 8);
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}

	
	databuf[0] = APDS9922_DD_ALS_THRES_LOW_2_ADDR;
	databuf[1] = (u8)((0 & 0xFF0000) >> 16);
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}

	
	databuf[0] = APDS9922_DD_ALS_THRES_UP_0_ADDR;
	databuf[1] = (u8)(obj->strong_sunlight & 0x00FF);
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_ALS_THRES_UP_1_ADDR;
	databuf[1] = (u8)((obj->strong_sunlight & 0xFF00) >> 8);
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_ALS_THRES_UP_2_ADDR;
	databuf[1] = (u8)((obj->strong_sunlight & 0xFF0000) >> 16);
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}
	return 0;

EXIT_ERR:
	return 1;
}

static irqreturn_t apds9922_irq(int irq, void *data)
{
	struct apds9922_priv *obj = data;
	static int cont_int_num = 0;
    	static unsigned long last_int_time = 0;
    	unsigned long int_time;
    	unsigned long diff;    
	disable_irq_nosync(obj->irq);
    	queue_work(obj->eint_queue, &obj->eint_work);
	int_time = jiffies;
	diff = (long)int_time - (long)last_int_time;
    	if((diff *1000)/HZ < CONT_INT_TIME)
    	{
    		if(cont_int_num < CONT_INT_GATE)
            		cont_int_num++;
        	else
		{
			if(stronglight_flag == 0)
			{
            			printk(KERN_ERR TAGE "continuous interrupts error\n");
			}
			else
			{
				printk(KERN_INFO TAGI "continuous interrupts error\n");
			}
			debug_flag = 1;
			debug = 1;
			printk(KERN_INFO TAGI "%s, ps_status= %d, stronglight_flag = %d\n", __func__,atomic_read(&obj->ps_status), stronglight_flag);
		}
	}
    	else
    	{
    		cont_int_num = 0;
	}
	last_int_time = int_time;

	return IRQ_HANDLED;
}

static void apds9922_eint_work(struct work_struct *work)
{
	struct apds9922_priv *obj = (struct apds9922_priv *)container_of(work, struct apds9922_priv, eint_work);
	
	printk(KERN_INFO TAGI "apds9922_eint_work\n");

	if(!test_bit(CMC_BIT_PS, &obj->enable) || test_bit(CMC_BIT_ENG_PS, &obj->enable)) {
		apds9922_check_intr(obj->client);
        	enable_irq(obj->irq);
        	return;
	}
	/* Now it is only a wake source */
	wake_lock_timeout(&ps_suspend_lock, 2 * HZ);    //wake up for 2s
	
	apds9922_check_intr(obj->client);
	if(atomic_read(&obj->ps_suspend))
       	msleep(10);     //10ms 
     	apds9922_ps_data_report(&(obj->ps_report_work.work));

    	enable_irq(obj->irq);    

       queue_delayed_work(obj->poll_queue, &obj->ps_report_work, 0);    //report ps data now
	
}

int apds9922_setup_eint(struct i2c_client *client)
{
    struct apds9922_priv *obj = i2c_get_clientdata(client);        
    int err;
    g_apds9922_ptr = obj;
  
    if (!obj->irq)
    {
        if(gpio_is_valid(obj->gpio_int)) 
        {
            err = gpio_request(obj->gpio_int, "apds9922_int");
            if(err < 0) 
            {
                printk(KERN_ERR TAGE "unable to request gpio [%d]\n", obj->gpio_int);
                return err;
            }   

            err = gpio_direction_input(obj->gpio_int);
            printk(KERN_INFO TAGI "%s, err = %d", __func__, err);
            if(err < 0) 
            {
                printk(KERN_ERR TAGE "unable to set direction for gpio [%d]\n", obj->gpio_int);
                goto err_irq_gpio_req;
            }
            err = gpio_set_debounce(obj->gpio_int, 2000);    //2ms debounce
            if(err < 0) 
            {
                printk(KERN_INFO TAGI "unable to set debounce for err = %d, gpio [%d]\n", err, obj->gpio_int);
            }
        } 
        else
        {
            printk(KERN_ERR TAGE "irq gpio not provided\n");
            return -EINVAL;
        }
    
        obj->irq = gpio_to_irq(obj->gpio_int);
        if(obj->irq)
        {
            err = request_any_context_irq(obj->irq, apds9922_irq, (IRQF_TRIGGER_LOW | IRQF_ONESHOT), 
                        "APDS9922_i2c", obj);
            if (err < 0) {
                printk(KERN_ERR TAGE "request irq failed: %d\n", err);
                return err;
            }

            enable_irq_wake(obj->irq);
        }
    }
       
    return 0;

err_irq_gpio_req:
	gpio_free(obj->gpio_int);
	return err;
}

static long apds9922_enable_als(struct i2c_client *client, int enable)
{
	struct apds9922_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	long res = 0;

	databuf[0]= APDS9922_DD_MAIN_CTRL_ADDR;
	//res = apds9922_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	res = apds9922_read_reg(client, databuf[0], databuf, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	//APS_LOG("APDS9921_CMM_ENABLE als value = %x\n", databuf[0]);
	
	if (enable) {
		databuf[1] = databuf[0]|APDS9922_DD_ALS_EN;
		databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
		if ( test_bit(CMC_BIT_PS , &obj->enable ) || test_bit(CMC_BIT_ENG_PS, &obj->enable) ) {
			databuf[1] = databuf[1] | APDS9922_DD_PRX_EN;    //ensure ps be enabled stutas
		} else {
			databuf[1] = databuf[1] & (~APDS9922_DD_PRX_EN);    //ensure ps be enabled stutas
		}
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}
             apds9922_als_select_para(obj);
	} else {
	       databuf[1] = databuf[0]&~APDS9922_DD_ALS_EN;
		if (test_bit(CMC_BIT_PS, &obj->enable))
			databuf[1] = databuf[1] | APDS9922_DD_PRX_EN;
		else
			databuf[1] = databuf[1]& (~APDS9922_DD_PRX_EN);

		databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}
	}
       alsCount = 0;
	return 0;

EXIT_ERR:
	printk(KERN_ERR TAGE "apds9922_enable_als fail\n");
	return res;
}

static long apds9922_enable_ps(struct i2c_client *client, int enable)
{
	struct apds9922_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;
	u8 main_ctrl=0;

       stronglight_flag = 0;
	databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
	//res = apds9922_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	res = apds9922_read_reg(client, databuf[0], databuf, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	if (enable) {
             apds9922_init_client(client);    
		main_ctrl = databuf[0]|APDS9922_DD_PRX_EN |APDS9922_DD_ALS_EN ;

		databuf[0] = APDS9922_DD_PRX_THRES_LOW_0_ADDR;	
		databuf[1] = 0;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		
		databuf[0] = APDS9922_DD_PRX_THRES_LOW_1_ADDR;	
		databuf[1] = 0;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		
		databuf[0] = APDS9922_DD_PRX_THRES_UP_0_ADDR;	
		databuf[1] = 0;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		databuf[0] = APDS9922_DD_PRX_THRES_LOW_1_ADDR;	
		databuf[1] = 0;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0) {
			goto EXIT_ERR;
		}

		databuf[1] = main_ctrl;
		databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		apds9922_ps_select_para(obj);
             atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
		atomic_set(&obj->ps_status, 1); //1: stand for: far away
		apds9922_enable_eint(obj, 0);
		apds9922_enable_eint(obj, 1);
	} else {
	       databuf[1] = databuf[0]&~APDS9922_DD_PRX_EN;
		if(test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[1] | APDS9922_DD_ALS_EN;
		else
			databuf[1] = databuf[1]& (~APDS9922_DD_ALS_EN);
		
		databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_status, 1);          //default is faraway
		atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
		
		apds9922_enable_eint(obj, 0);
	}
	
	return 0;
	
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9922_enable_ps fail\n");
	return res;
}

static int apds9922_enable(struct i2c_client *client, int enable)
{
//	struct apds9922_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;

	if(client == NULL)
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	if(enable)
	{
	      
		databuf[1] = 0x03;
		databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	
		if(res <= 0)
		{
		    printk(KERN_ERR TAGE "APDS9922_enable fail\n");
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI " apds9922_enable on\n");
	}
	else
	{
	      databuf[1] = 0x00;
		databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
		{
		    printk(KERN_ERR TAGE "APDS9922_enable fail\n");
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI "apds9922_enable  off\n");
	}		

	return 0;
	
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9922_enable fail\n");
	return res;
}

static u8 apds9922_read_id(struct i2c_client *client)
{
	u8 device_id;
	int res = 0;
	res = apds9922_read_reg(client,APDS9922_DD_PART_ID_ADDR, &device_id, 0x1);
	if(res <= 0)
	{
		return 0;
	}
	return device_id;
}
static int apds9922_init_client(struct i2c_client *client)
{
	//struct apds9922_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;

	databuf[0] = APDS9922_DD_MAIN_CTRL_ADDR;
	databuf[1] = 0;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_PRX_LED_ADDR;
	databuf[1] = APDS9922_DD_LED_FREQ_100_KHZ|APDS9922_DD_PRX_DEFAULT_LED_CURRENT;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_PRX_PULSES_ADDR;
	databuf[1] = APDS9922_DD_PRX_DEFAULT_PULSE;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
       
	databuf[0] = APDS9922_DD_PRX_MEAS_RATE_ADDR;
	databuf[1] = 0x40|APDS9922_DD_PRX_DEFAULT_RES|APDS9922_DD_PRX_MEAS_RATE_50_MS;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
      
	databuf[0] = APDS9922_DD_ALS_MEAS_RATE_ADDR;
	databuf[1] = APDS9922_DD_ALS_MEAS_RES_17_BIT|APDS9922_DD_ALS_MEAS_RATE_50_MS;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_ALS_GAIN_ADDR;
	databuf[1] = APDS9922_DD_ALS_DEFAULT_GAIN;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_INT_PERSISTENCE_ADDR;
	databuf[1] = APDS9922_DD_PRX_PERS_1;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_INT_PERSISTENCE_ADDR;
	databuf[1] = APDS9922_DD_PRX_PERS_1;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9922_DD_INT_CFG_ADDR;
	databuf[1] = 0x15;
	//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	if ((res = apds9922_setup_eint(client))!=0) {
		printk(KERN_ERR TAGE "setup eint: %d\n", res);
		return res;
	}

	 if ((res = apds9922_check_intr(client))) {
		printk(KERN_ERR TAGE "check/clear intr: %d\n", res);
		return res;
	}
	return APDS9922_SUCCESS;

EXIT_ERR:
	printk(KERN_INFO TAGI "init dev: %d\n", res);
	return res;
}
/*----------------------------------------------------------------------------*/
static int apds9922_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9922_i2c_client;

	if (!file->private_data)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int apds9922_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long apds9922_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct apds9922_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    printk(KERN_INFO TAGI "%s-----enable ps\n",__func__);
			    if(!test_bit(CMC_BIT_ENG_PS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable) ){
					apds9922_enable_als(obj->client,1);
					if((err = apds9922_enable_ps(obj->client, 1)))
					{
						printk(KERN_ERR TAGE "enable ps fail: %d\n", err); 
						goto err_out;
					}
					apds9922_enable_eint(obj,0);
					apds9922_enable_eint(obj,1);
				}
				else{
					printk(KERN_INFO TAGI "not enable ps as CMC_BIT_ENG_PS=%d CMC_BIT_PS=%d\n",test_bit(CMC_BIT_ENG_PS,&obj->enable)?(1):(0),
						test_bit(CMC_BIT_PS,&obj->enable)?(1):(0));
				}
				set_bit(CMC_BIT_ENG_PS, &obj->enable);
                          printk(KERN_INFO TAGI "%s, set ps enable bit here\n",__func__);
			}
			else
			{
				printk(KERN_INFO TAGI "%s-----disable ps\n",__func__);
				if(!test_bit(CMC_BIT_PS,&obj->enable)){
					if((err = apds9922_enable_ps(obj->client, 0)))
					{
						printk(KERN_ERR TAGE "disable ps fail: %d\n", err); 
						goto err_out;
					}
					apds9922_enable_eint(obj,0);
				}
				if(!test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable)){
					apds9922_enable_als(obj->client,0);
				}
				
				clear_bit(CMC_BIT_ENG_PS, &obj->enable);
                          printk(KERN_INFO TAGI "%s, clear ps enable bit here--2 \n",__func__);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_ENG_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = apds9922_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}	
			dat = apds9922_get_ps_value(obj, obj->ps);			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = apds9922_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(!test_bit(CMC_BIT_ENG_ALS,&obj->enable) && !test_bit(CMC_BIT_ALS,&obj->enable)){
					if((err = apds9922_enable_als(obj->client, 1)))
					{
						printk(KERN_ERR TAGE "enable als fail: %d\n", err); 
						goto err_out;
					}
				}
				set_bit(CMC_BIT_ENG_ALS, &obj->enable);// 2015.4.23, the only place to set CMC_BIT_ENG_ALS
			}
			else
			{
				if( !test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable) && !test_bit(CMC_BIT_ENG_PS,&obj->enable) ) {
					if((err = apds9922_enable_als(obj->client, 0)))
					{
						printk(KERN_ERR TAGE "disable als fail: %d\n", err); 
						goto err_out;
					}
				}
				clear_bit(CMC_BIT_ENG_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ENG_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = apds9922_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}	
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = apds9922_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		case ALSPS_IOCTL_GET_CALI:

			dat = 0x00;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					 goto err_out;
				}
			break;

		default:
			printk(KERN_ERR TAGE "%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations apds9922_fops = {
	.owner = THIS_MODULE,
	.open = apds9922_open,
	.release = apds9922_release,
	.unlocked_ioctl = apds9922_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice apds9922_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &apds9922_fops,
};
int apds9922_enable_eint(struct apds9922_priv *obj, int enable)
{
	int res;
	unsigned char databuf[2];
	if(enable) {
		if(!atomic_read(&obj->ps_status)) {      //note: intr_flag_value only chenged in apds9920_get_ps_value
			printk(KERN_INFO TAGI "wait far away\n");
			databuf[0] = APDS9922_DD_PRX_THRES_LOW_0_ADDR;
			databuf[1] = (u8)((atomic_read(&obj->ps_out_threshold)) & 0x00FF);
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9922_DD_PRX_THRES_LOW_1_ADDR;
			databuf[1] = (u8)(((atomic_read(&obj->ps_out_threshold)) & 0xFF00) >> 8);
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9922_DD_PRX_THRES_UP_0_ADDR;
			databuf[1] = (u8)(0x00FF);
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9922_DD_PRX_THRES_UP_1_ADDR;
			databuf[1] = (u8)((0xFF00) >> 8);;
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			//if is in close state must enable als moniter
			apds9922_enable_als_moniter_eint(obj->client, 1);
		} else {
			printk(KERN_INFO TAGI "wait close\n");
			databuf[0] = APDS9922_DD_PRX_THRES_LOW_0_ADDR;
			databuf[1] = (u8)(0 & 0x00FF);
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9922_DD_PRX_THRES_LOW_1_ADDR;
			databuf[1] = (u8)((0 & 0xFF00) >> 8);
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9922_DD_PRX_THRES_UP_0_ADDR;
			databuf[1] = (u8)((atomic_read(&obj->ps_in_threshold)) & 0x00FF);
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9922_DD_PRX_THRES_UP_1_ADDR;
			databuf[1] = (u8)(((atomic_read(&obj->ps_in_threshold)) & 0xFF00) >> 8);;
			//res = apds9922_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
			res =  apds9922_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			apds9922_enable_als_moniter_eint(obj->client, 0);
		}
		apds9922_check_intr(obj->client);
		return 0;
		} else {
		printk(KERN_INFO TAGI "disabling apds9921 eint\n");
		apds9922_enable_als_moniter_eint(obj->client, 0);
		apds9922_check_intr(obj->client);
	
	}
	
	return 0;

}


static int apds9922_i2c_suspend(struct i2c_client *client, pm_message_t msg) 

{
	
	struct apds9922_priv *obj = i2c_get_clientdata(client);    
	
	int err;
	
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);


	if(!obj)
	
	{
	
		printk(KERN_ERR TAGE "null pointer!!\n");
	
		return -EINVAL;
	
	}
		
	printk(KERN_INFO TAGI "%s CMC_BIT_PS = %d\n ",__func__,test_bit(CMC_BIT_PS, &obj->enable));
	
	if(test_bit(CMC_BIT_PS, &obj->enable))
	
	{
	
		atomic_set(&obj->ps_suspend, 1);         //the obj->ps_status can not change
	
		cancel_delayed_work(&obj->ps_report_work);     //stop report timer here
	
		return 0;
	
	}
	
	if(debug_flag == 1)
	
	{
	
		debug = 0;
	
		debug_flag = 0;
	
		printk(KERN_INFO TAGI " [%s]:clear debug and debug_flag.\n", __func__);
	
	}
	
	if((err = apds9922_enable_als(client, 0)))
	
	{
	
		printk(KERN_ERR TAGE "disable als: %d\n", err);
	
		return err;
	
	}


	printk(KERN_INFO TAGI "%s-----disable ps\n",__func__);
	
	if((err = apds9922_enable_ps(client, 0)))
	
	{
	
		printk(KERN_ERR TAGE "disable ps:  %d\n", err);
	
		return err;
	
	}
	
	apds9922_enable(client, 0);
	
	return 0;

}



/*----------------------------------------------------------------------------*/
static int apds9922_i2c_resume(struct i2c_client *client)
{

	
	struct apds9922_priv *obj = i2c_get_clientdata(client);        
	int err;
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);

	if(!obj)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return -EINVAL;
	}
	if (test_bit(CMC_BIT_PS, &obj->enable))// interrupt
	{
		queue_delayed_work(obj->poll_queue, &obj->ps_report_work, msecs_to_jiffies(10));     //10ms
		atomic_set(&obj->ps_suspend, 0);
		printk(KERN_INFO TAGI"%s----queue_delay_work", __func__);
		return 0;
	}
	if((err = apds9922_init_client(client)))
	{
		printk(KERN_ERR TAGE "initialize client fail!!\n");
		return err;
	}

	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{

		if((err = apds9922_enable_als(client, 1)))
		{
			printk(KERN_ERR TAGE "enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		printk(KERN_INFO TAGI "%s-----enable ps\n",__func__);
		if((err = apds9922_enable_ps(client, 1)))
		{
			printk(KERN_ERR TAGE "enable ps fail: %d\n", err);                
		}
	}

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
/*----------------------------------------------------------------------------*/
static void apds9922_early_suspend(struct early_suspend *h)
{ 
	struct apds9922_priv *obj = container_of(h, struct apds9922_priv, early_drv);
	int err;
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);

	wake_lock_timeout(&(obj->wakelock), 3*HZ);
	if(!obj)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return;
	}
	printk(KERN_INFO TAGI " %s have wakelock for 3HZ.\n",__func__);
}

/*----------------------------------------------------------------------------*/
static void apds9922_late_resume(struct early_suspend *h)
{
	struct apds9922_priv *obj = container_of(h, struct apds9922_priv, early_drv);
	int err;
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);
	
	if(!obj)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return;
	}
	printk(KERN_INFO TAGI " %s do nothing.\n",__func__);  
}
#elif defined(CONFIG_FB)
static int apds9922_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct apds9922_priv *obj = container_of(self, struct apds9922_priv, fb_notif);

	printk(KERN_INFO TAGI "apds9922_fb_notifier_callback \n");

	if (evdata && evdata->data && event == FB_EVENT_BLANK && obj) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
		{
			//do nothing
		}
		else if (*blank == FB_BLANK_POWERDOWN)
		{
			wake_lock_timeout(&(obj->wakelock), 3*HZ); 
		}
	}

	return 0;
}
#endif
static ssize_t show_deviceid(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	int deviceid = 0;
	if(NULL == client)
	{
		printk(KERN_ERR TAGE "i2c client is null!!\n");
		return 0;
	}
	deviceid = 0;
    
	return sprintf(buf, "%d\n", deviceid);
}

static DRIVER_ATTR(deviceid,   S_IWUSR | S_IRUGO, show_deviceid,      NULL);
/*----------------------------------------------------------------------------*/
static ssize_t store_ps_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);	 
	int num =0;
	num=simple_strtoul(buf, NULL, 10);
	set_ps_para_num(num);
	apds9922_ps_select_para(obj);
	return count;
}
static ssize_t show_ps_para_index(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", ps_para_num);
}
static DRIVER_ATTR(ps_para_index,   S_IWUSR | S_IRUGO, show_ps_para_index,store_ps_para_index);

static ssize_t store_ps_esd_time(struct device_driver *ddri,const char *buf, size_t count)
{
	int num =0;
	num=simple_strtoul(buf, NULL, 10);
	if(num >=2 )
	PS_ESD_TIME=num;
	return count;
}
static ssize_t show_ps_esd_time(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", PS_ESD_TIME);
}
static DRIVER_ATTR(ps_esd_time,   S_IWUSR | S_IRUGO, show_ps_esd_time,store_ps_esd_time);

static ssize_t store_debug(struct device_driver *ddri, const char *buf, size_t count)
{
	int num =0;
	num=simple_strtoul(buf, NULL, 10);
	if(num==0){
		debug=0;
	}
	else if(num==1){
		debug=1;
	}
	else{
		printk(KERN_INFO "wrong arg of debug %d\n",num);
	}
	return count;
}
static ssize_t show_debug(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", debug);
}
static DRIVER_ATTR(debug,   S_IWUSR | S_IRUGO, show_debug,store_debug);
/*----------------------------------------------------------------------------*/
static ssize_t show_read_register(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->read_register);
}

static ssize_t store_read_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);	 
	u8 register_value[1];
	u8 register_addr[1];
	int res = 0;

	if(client == NULL )
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		sprintf(obj->read_register,"CLIENT=NULL");
		return -1;
	}

	register_addr[0]=simple_strtoul(buf, NULL, 16)&0xff;
	//res = apds9922_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	res = apds9922_read_reg(client,register_addr[0], register_value, 0x1);
	if(res <= 0)
	{
		goto error_fs;
	}
	sprintf(obj->read_register,"0x%02x=0x%02x",register_addr[0],register_value[0]);
	return count;

	error_fs:
		sprintf(obj->read_register,"I2C error");
	return count;
}
static DRIVER_ATTR(read_register,   S_IWUSR | S_IRUGO, show_read_register,store_read_register);

/*----------------------------------------------------------------------------*/
static ssize_t store_prox_status(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);	 
	int val_tmp = 0;

	if(client == NULL )
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	val_tmp = simple_strtoul(buf, NULL, 10);
	if ( val_tmp == 0 || val_tmp == 1 )
	{
		atomic_set(&obj->sys_prox_status, val_tmp);
		printk(KERN_ERR TAGE "DDDDDDD set sys_prox_status=%d\n",val_tmp);
		return count;
	}
	else 
	{
		goto error_fs;
	}

	error_fs:
		printk(KERN_INFO TAGI "store_prox_status wrong value!value=%d\n",val_tmp);
	return count;
}

static ssize_t show_prox_status(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->sys_prox_status));
}
static DRIVER_ATTR(sys_prox_status,   S_IWUSR | S_IRUGO,show_prox_status,store_prox_status);

/*----------------------------------------------------------------------------*/
static ssize_t show_write_register(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->write_register);
}

static ssize_t store_write_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	//unsigned long reg_value = simple_strtoul(buf, NULL, 16);
	unsigned char databuf[2];
	int res;
	char* const delim = " ";
	char str_command[20];
	char *token=0;
	char *cur;
	char str_reg_addr[10];
	char str_reg_val[10];
	int  reg_addr,reg_value;
	sprintf(str_command,"%s",buf);
	cur=str_command;
	if((token=strsep(&cur,delim)))
	sprintf(str_reg_addr,"%s",token);
	else
	goto error_fs;
	if((token=strsep(&cur,delim)))
	sprintf(str_reg_val,"%s",token);
	else
	goto error_fs;

	reg_addr=simple_strtoul(str_reg_addr, NULL, 16)&0xff;
	reg_value=simple_strtoul(str_reg_val, NULL, 16)&0xff;

	if( obj != NULL && client != NULL ){
		databuf[0] = reg_addr;
		databuf[1] = reg_value;
		//res = apds9922_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		res =  apds9922_write_reg(client, databuf[0], &databuf[1], 0x1);
		printk(KERN_INFO TAGI "set APDS9922 ps write reg:0x%02x=0x%02x\n",databuf[0],databuf[1]);
		if(res <= 0)
		{
			printk(KERN_INFO TAGI "%s--%d set register fail \n",__func__,__LINE__);
			sprintf(obj->write_register,"0x%02x:FAIL",databuf[0]);
		}
		else
		{
			sprintf(obj->write_register,"0x%02x=0x%02x",databuf[0],databuf[1]);
		}
	}
	else {
		printk(KERN_INFO TAGI "set APDS9922 ps register reg FAIL!!\n");
	}
	return count;

	error_fs:
		sprintf(obj->write_register,"WRONG ARGS");
	return count;
}
static DRIVER_ATTR(write_register,   S_IWUSR | S_IRUGO, show_write_register,store_write_register);

static ssize_t show_als_para_index(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);

	apds9922_als_select_para(obj);

	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", als_para_num);
}

static ssize_t store_als_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	int num =0;
	num=simple_strtoul(buf, NULL, 10);
	set_als_para_num(num);
	return count;
}
static DRIVER_ATTR(als_para_index,   S_IWUSR | S_IRUGO, show_als_para_index, store_als_para_index);

/*----------------------------------------------------------------------------*/
static ssize_t show_ps_base_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);

	apds9922_ps_select_para(obj);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->ps_base_value));
}

static ssize_t store_ps_base_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	u16 thd_value = (u16)simple_strtoul(buf, NULL, 10);

	if( obj != NULL && client != NULL && thd_value >= 0 )
	{
		const struct ps_para *para;

		//check the value
		para = &apds9922_ps_para[ps_para_num];
		if(!(thd_value >= para->base_value_min && thd_value <= para->base_value_max))    //if the value is error, do nothing.
		return count;

		printk(KERN_INFO TAGI "mean to set ps_base_value=%d\n", thd_value);
		atomic_set(&obj->ps_base_value, thd_value);

		apds9922_ps_select_para(obj);     //reselect the threshold agian 
		//enable 
		if(test_bit(CMC_BIT_PS,&obj->enable) || test_bit(CMC_BIT_ENG_PS,&obj->enable)){
			apds9922_enable_eint(obj,0);
			apds9922_enable_eint(obj,1);
		}


		printk(KERN_INFO TAGI "%s finnal ps_base_value=%d\n",__func__,atomic_read(&obj->ps_base_value));
	}
	else 
	{
		printk(KERN_INFO TAGI "set apds9922 ps threshold FAIL!!\n");
	}

	return count;
}
static DRIVER_ATTR(ps_base_value,   S_IWUSR | S_IRUGO, show_ps_base_value, store_ps_base_value);

/*----------------------------------------------------------------------------*/
static ssize_t show_strong_sunlight(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", obj->strong_sunlight);
}

static ssize_t store_strong_sunlight(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9922_i2c_client;
	struct apds9922_priv *obj=i2c_get_clientdata(client);
	unsigned light_value = simple_strtoul(buf, NULL, 10);
	if( obj != NULL && client != NULL && light_value > 0 ){
		obj->strong_sunlight=light_value;
	}
	else {
		printk(KERN_ERR TAGE "set LTR558 ps strong_sunlight FAIL!!\n");
	}
	return count;
}
static DRIVER_ATTR(strong_sunlight,   S_IWUSR | S_IRUGO, show_strong_sunlight,store_strong_sunlight);

static struct driver_attribute *apds9922_attr_list[] = {
	&driver_attr_deviceid,
	&driver_attr_als_para_index,
	&driver_attr_ps_base_value,
	&driver_attr_strong_sunlight,
	&driver_attr_read_register,
	&driver_attr_write_register,
	&driver_attr_ps_para_index,
	&driver_attr_sys_prox_status,
	&driver_attr_ps_esd_time,
	&driver_attr_debug,
};

/*----------------------------------------------------------------------------*/
static int apds9922_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(apds9922_attr_list)/sizeof(apds9922_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)	
	{
		if((err = driver_create_file(driver, apds9922_attr_list[idx])))
		{
			printk(KERN_ERR TAGE "driver_create_file (%s) = %d\n", apds9922_attr_list[idx]->attr.name, err);
			break;
		}
	} 
	return err;
}

static int apds9922_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(apds9922_attr_list)/sizeof(apds9922_attr_list[0]));
	if(driver == NULL)
	{
		return -EINVAL;	
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, apds9922_attr_list[idx]);
	}
	return err;
}

//enable attr for input devices 
static ssize_t ps_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if(ps_en != 0 && ps_en != 1)
		return -EINVAL;

	if(ps_en)
	{
		ps_enable_set(&apds9922_obj->ps_cdev, 1);
	}
	else
	{
		ps_enable_set(&apds9922_obj->ps_cdev, 0);
	}

	return count;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9922_priv *obj = apds9922_obj;
	u32 delay = atomic_read(&obj->ps_poll_delay);
	int i;
	int err = 0;

	if (enable)
	{
		for(i=0; i<3; i++)
		{
			if((err = apds9922_enable_ps(obj->client, 1)))
			{
				if(i < 2)
				{
					continue;
				}
				else
				{
					printk(KERN_ERR TAGE "%s enable ps fail: %d i = %d \n", __func__,err,i); 
					return -1;
				}                                    
			}
			else
			{
				printk(KERN_INFO TAGI "%s  enable ps success\n",__func__);
				break;
			}   
		}

		//enable eint
		apds9922_enable_eint(obj,0);
		apds9922_enable_eint(obj,1);

		// add 40ms delay as ps data not stable when it's just init
		//msleep(40);
		set_bit(CMC_BIT_PS, &obj->enable);
		printk(KERN_INFO TAGI "%s, set ps enable bit here",__func__);

		obj->last_ps_data=-1;

		if(!test_bit(CMC_BIT_ALS, &obj->enable) && !test_bit(CMC_BIT_ENG_ALS, &obj->enable)) {
			if(!apds9922_enable_als(obj->client, 1)) {
				printk(KERN_INFO TAGI "---flyshine----- Enable ALS by PS SUCCESS !!\n");
			}
			else {
				printk(KERN_INFO TAGI "---flyshine----- Enable ALS by PS FAIL !!\n");
			}
		}
		//start ps report work
		queue_delayed_work(obj->poll_queue, &obj->ps_report_work, msecs_to_jiffies(delay));
	}
	else
	{
		unenable_and_read_count=0;
		printk(KERN_INFO TAGI "%s-----disable ps\n",__func__);
		clear_bit(CMC_BIT_PS, &obj->enable);
		if(!test_bit(CMC_BIT_ENG_PS,&obj->enable)) 
		{
			apds9922_enable_eint(obj,0);
			if((err = apds9922_enable_ps(obj->client, 0)))
			{
				printk(KERN_ERR TAGE "disable ps fail: %d\n", err); 
				return -1;
			}
		}      		    
		//mt_eint_mask(CUST_EINT_ALS_NUM);//clear ps eint mask
		if(!test_bit(CMC_BIT_ENG_PS,&obj->enable) && !test_bit(CMC_BIT_ENG_ALS,&obj->enable) && !test_bit(CMC_BIT_ALS,&obj->enable) ) 
		{
			if((err = apds9922_enable_als(obj->client, 0)))
			{
				printk(KERN_ERR TAGE "disable als fail: %d\n", err); 
			//return -1;
			}
		}
		if(!test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable))
		cancel_delayed_work(&obj->ps_report_work);     //stop report timer here
	}
	nAlsReadCount = 0;
	return err;
}
static ssize_t als_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int als_en;
//	struct apds9922_priv *obj = apds9922_obj;
	als_en = -1;
	sscanf(buf, "%d", &als_en);

	if(als_en != 0 && als_en != 1)
		return -EINVAL;

	if(als_en)
	{
		als_enable_set(&apds9922_obj->als_cdev, 1);
	}
	else
	{
		als_enable_set(&apds9922_obj->als_cdev, 0);
	}

	return count;
}

static int als_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9922_priv *obj = apds9922_obj;
	u32 delay = atomic_read(&obj->als_poll_delay);
	int err = 0;
	int i;

	if (enable)
	{
		for(i=0; i<3; i++)
		{
			if((err = apds9922_enable_als(obj->client, 1)))
			{
				if(i < 2)
				{
					continue;
				}
				else
				{
					printk(KERN_ERR TAGE "%s enable als fail: %d i = %d \n", __func__,err,i); 
					return -1;
				}                              
			}
			else
			{
				printk(KERN_INFO TAGI "%s  enable als success\n",__func__);
				break;
			}                          
		}

		set_bit(CMC_BIT_ALS, &obj->enable);
		obj->last_als_data = -1;
		nAlsReadCount = 0;

		//start als report work
		queue_delayed_work(obj->poll_queue, &obj->ps_report_work, msecs_to_jiffies(delay));
	}
	else
	{
		printk(KERN_INFO TAGI "%s: LIANG add to test bit: CMC_BIT_PS= %d, CMC_BIT_ENG_PS= %d, CMC_BIT_ENG_ALS= %d\n", __func__,
		test_bit(CMC_BIT_PS, &obj->enable), test_bit(CMC_BIT_ENG_PS, &obj->enable), test_bit(CMC_BIT_ENG_ALS,&obj->enable));
		if(!test_bit(CMC_BIT_PS, &obj->enable) && !test_bit(CMC_BIT_ENG_PS, &obj->enable)
		&& !test_bit(CMC_BIT_ENG_ALS,&obj->enable))
		{
			if((err = apds9922_enable_als(obj->client, 0)))
			{
			printk(KERN_ERR TAGE "disable als fail: %d\n", err); 
			}
			printk(KERN_INFO TAGI "%s: check als-- disable\n", __func__);
		}

		clear_bit(CMC_BIT_ALS, &obj->enable);

		//stop als reprot work
		if(!test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable))
		{
			cancel_delayed_work(&obj->ps_report_work);     //stop report timer her
		}
		//if we run the cts test before , reset the als polling time
		if(atomic_read(&obj->als_poll_delay) == 10)	
			atomic_set(&obj->als_poll_delay,ALS_DEFAULT_POLL_DELAY);
	}

	return err;
}

static int als_poll_delay_set(struct sensors_classdev *sensors_cdev,
    unsigned int delay_msec){
	struct apds9922_priv *obj = apds9922_obj; 
	if(delay_msec == ALS_MIN_POLL_DELAY){
		atomic_set(&obj->als_poll_delay,10);
		printk(KERN_INFO TAGI "%s set min_als_poll_delay to 10ms for cts pass\n", __func__);
	}
	else
	{
		atomic_set(&obj->als_poll_delay,ALS_DEFAULT_POLL_DELAY);
		printk(KERN_INFO TAGI "%s set any poll delay to default 50 ms.cos we didn't want to change als polling time\n", __func__);
	}
	return 0;
}

static struct device_attribute light_attr[] = {
	__ATTR(enable, 0664, als_enable_show, als_enable_store),
};

static struct device_attribute proximity_attr[] = {
	__ATTR(enable, 0664, ps_enable_show, ps_enable_store),
};

static int create_sysfs_interfaces(struct device *dev, struct device_attribute *attributes, int len)
{
	int i;
	int err;
	for (i = 0; i < len; i++) {
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


static int als_ps_probe(struct platform_device *pdev) 
{
	return 0;
}
static int als_ps_remove(struct platform_device *pdev)
{
    return 0;
}
static struct platform_driver als_ps_driver = {
	.probe      = als_ps_probe,
	.remove     =als_ps_remove,    
	.driver     = {
		.name  = "als_ps",
		.owner = THIS_MODULE,
	}
};


/*----------------------------------------------------------------------------*/
static int apds9922_parse_dt(struct device *dev, struct apds9922_priv *obj)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_get_named_gpio_flags(np, "irq-gpio",
	0, NULL);
	if (rc < 0) {
		printk(KERN_ERR TAGE "Unable to read interrupt pin number\n");
		return rc;
	} else {
		obj->gpio_int = rc;
	}

	printk(KERN_INFO TAGI "read interrupt pin number %d\n", obj->gpio_int);
	return 0;
}

static int apds9922_lightsensor_setup(struct apds9922_priv *obj)
{
	int ret;

	obj->input_dev_als = input_allocate_device();
	if (!obj->input_dev_als) {
		printk(KERN_ERR TAGE "%s: could not allocate als input device\n",__func__);
		return -ENOMEM;
	}
	obj->input_dev_als->name = "light";
	obj->input_dev_als->id.bustype = BUS_I2C;
	set_bit(EV_ABS, obj->input_dev_als->evbit);
	input_set_abs_params(obj->input_dev_als, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(obj->input_dev_als, ABS_Y, 0, 1, 0, 0);          //fake index

	ret = input_register_device(obj->input_dev_als);
	if (ret < 0) {
		printk(KERN_ERR TAGE "%s: can not register als input device\n",__func__);
		goto err_free_als_input_device;
	}

	return ret;

	err_free_als_input_device:
	input_free_device(obj->input_dev_als);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int apds9922_psensor_setup(struct apds9922_priv *obj)
	{
	int ret;

	obj->input_dev_ps = input_allocate_device();
	if (!obj->input_dev_ps) {
		printk(KERN_ERR TAGE "%s: could not allocate ps input device\n",	__func__);
		return -ENOMEM;
	}
	obj->input_dev_ps->name = "proximity";
	obj->input_dev_ps->id.bustype = BUS_I2C;
	obj->input_dev_ps->evbit[0] = BIT_MASK(EV_REL);
	obj->input_dev_ps->relbit[0] = BIT_MASK(REL_X);

	ret = input_register_device(obj->input_dev_ps);
	if (ret < 0) {
	printk(KERN_ERR TAGE "%s: could not register ps input device\n",__func__);
	goto err_free_ps_input_device;
	}

	return ret;

	err_free_ps_input_device:
		input_free_device(obj->input_dev_ps);
	return ret;
	}

/*----------------------------------------------------------------------------*/
static int apds9922_config_regulator(struct apds9922_priv *obj, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(apds9922_vreg) / sizeof(struct sensor_regulator);

	if (on) 
	{
		for (i = 0; i < num_reg; i++) 
		{
			apds9922_vreg[i].vreg = regulator_get(&obj->client->dev, apds9922_vreg[i].name);

			if (IS_ERR(apds9922_vreg[i].vreg)) 
			{
				rc = PTR_ERR(apds9922_vreg[i].vreg);
				printk(KERN_ERR TAGE "%s:regulator get failed rc=%d\n", __func__, rc);
				apds9922_vreg[i].vreg = NULL;
			}

			if (regulator_count_voltages(apds9922_vreg[i].vreg) > 0) 
			{
				rc = regulator_set_voltage(
				apds9922_vreg[i].vreg,
				apds9922_vreg[i].min_uV,
				apds9922_vreg[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE "%s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(apds9922_vreg[i].vreg);
					apds9922_vreg[i].vreg = NULL;
				}
			}

			rc = regulator_enable(apds9922_vreg[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE "%s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(apds9922_vreg[i].vreg) > 0) 
				{
					regulator_set_voltage(
					apds9922_vreg[i].vreg, 0,
					apds9922_vreg[i].max_uV);
				}
				regulator_put(apds9922_vreg[i].vreg);
				apds9922_vreg[i].vreg = NULL;
			}
		}
	} 
	else 
	{
		i = num_reg;
	}

	return rc;
}

/*----------------------------------------------------------------------------*/
static void apds9922_ps_data_report(struct work_struct *work)
{
	struct apds9922_priv *data = container_of((struct delayed_work *)work, struct apds9922_priv, ps_report_work);
	static u32 index = 0;
	u32 delay = atomic_read(&data->ps_poll_delay);
	u32 als_c0_dat;
	int err = 0;

	mutex_lock(&ps_report_lock);

	if(data == NULL)
	goto restart_poll;
	if(test_bit(CMC_BIT_ALS, &data->enable)){
		if(alsCount < 20)
		{
		alsCount += 1;
		delay = 35;
		}
		if((err = apds9922_read_als(data->client, &data->als)))
		{
			printk(KERN_ERR TAGE "apds9920_read_als error %d\n", err);
			goto exit_als;
		}		

		if(nAlsReadCount < 2)
		{
			apds9922_read_ps(data->client, &data->ps);
			apds9922_get_ps_value(data, data->ps);
			if(atomic_read(&data->ps_status) == 0)
			{
				printk(KERN_INFO TAGI "apds9922_read_als fisrt data not report\n");
				goto exit_als;
			}	
		}

		if(data->als < 0)
		data->als = 0;

		input_report_abs(data->input_dev_als, ABS_X, data->als);
		input_report_abs(data->input_dev_als, ABS_Y, (index%2)); 
		input_sync(data->input_dev_als);
		index++;

		data->last_als_data = data->als;
	}
	exit_als:
	if(!test_bit(CMC_BIT_PS, &data->enable))
	{
		if(unenable_and_read_count < 10)
		{
			unenable_and_read_count += 1;
			printk(KERN_INFO TAGI "ALS_PS unenable_and_read_count=%d times\n", unenable_and_read_count);
		}
		else
		{
			unenable_and_read_count = 0;
			printk(KERN_INFO TAGI "ALS_PS unenable_and_read_count=%d re-enable ps\n", unenable_and_read_count);

			apds9922_enable_ps(data->client, 0);
			if(apds9922_enable_ps(data->client, 1) == 0)
			{
				set_bit(CMC_BIT_PS, &data->enable);
				printk(KERN_INFO TAGI "ALS_PS re-enable SUCCESS\n");
			}
			else
			{
				printk(KERN_INFO TAGI "ALS_PS re-enable FAIL\n");
			}
		}
		err =-1;
		printk(KERN_INFO TAGI "SENSOR_GET_DATA:can't get ps_data when ic not enabled\n");
		goto restart_poll;
	}


	if((err = apds9922_read_ps(data->client, &data->ps)))
	{
		printk(KERN_ERR TAGE "apds9920_read_ps error %d\n", err);
		goto restart_poll;
	}

	if(data->ps > 1023) 
	{
		err = -1;
		printk(KERN_ERR TAGE "SENSOR_GET_DATA:ps_data overflow\n");
		goto restart_poll;
	}

	apds9922_read_als0(data->client, &als_c0_dat);
	if(als_c0_dat > data->strong_sunlight) 
	{
		data->ps = 4;
		stronglight_flag = 1;
		if(debug)
		printk(KERN_INFO TAGI "als_c0_dat is = %d this is too strong,report ps=4\n", als_c0_dat);
	}
	else
	{
		stronglight_flag = 0;
	}
	if(apds9922_get_ps_value(data, data->ps) < 0)
	{
		printk(KERN_ERR TAGE "apds9920_get_ps_value fail\n");
		goto restart_poll;
	}
	input_report_rel(data->input_dev_ps, REL_X, (atomic_read(&data->ps_status) + 1));
	input_sync(data->input_dev_ps);

	restart_poll:

	mutex_unlock(&ps_report_lock);
	if((test_bit(CMC_BIT_PS, &data->enable) || test_bit(CMC_BIT_ALS, &data->enable)) && !atomic_read(&data->ps_suspend))
		queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(delay));
}

static int apds9922_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct apds9922_priv *obj;
	int err = 0;

	//init locks
	mutex_init(&apds9922_lock);
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	apds9922_obj = obj;

	//get the platform data from device tree
	if (client->dev.of_node) {
		err = apds9922_parse_dt(&client->dev, obj);
	}

	//power on
	obj->client = client;
	i2c_set_clientdata(client, obj);
	apds9922_config_regulator(obj, true);
	msleep(5);

	if(apds9922_read_id(client) != DEVICE_ID)
	{
	       err = -1;
		goto exit_kfree;
	}

	if((err = apds9922_init_client(client)))
	{
		goto exit_kfree;
	}       
	print_vivo_init(TAGI "apds9922_init_client() OK!\n");

	platform_driver_register(&als_ps_driver);

	obj->enable = 0;
	obj->strong_sunlight=DEFAULT_STRONG_SUNLIGHT;

	//setup input_dev
	err = apds9922_lightsensor_setup(apds9922_obj);
	if (err < 0) {
		printk(KERN_ERR TAGE "%s: apds9922_lightsensor_setup error!\n", __func__);
		goto exit_init_failed;
	}

	err = apds9922_psensor_setup(apds9922_obj);
	if (err < 0) {
		printk(KERN_ERR TAGE "apds9920_psensor_setup error!!\n");
		goto exit_init_failed;
	}

	err = create_sysfs_interfaces(&apds9922_obj->input_dev_als->dev, light_attr,
	ARRAY_SIZE(light_attr));
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to create input_dev_als sysfs\n");
		goto exit_init_failed;
	}

	err = create_sysfs_interfaces(&apds9922_obj->input_dev_ps->dev, proximity_attr,
	ARRAY_SIZE(proximity_attr));
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to create input_dev_ps sysfs\n");
		goto exit_init_failed;
	}

	if(bbk_ps_para_count != 0)
		apds9922_ps_para = bbk_ps_paras;
	else
		apds9922_ps_para = bbk_ps_paras_default;
	if(bbk_als_para_count != 0)
		apds9922_als_para = bbk_als_paras;
	else
		apds9922_als_para = bbk_als_paras_default;

	obj->eint_queue = create_singlethread_workqueue("apds9922_eint");
	INIT_WORK(&obj->eint_work, apds9922_eint_work);        

	obj->poll_queue = create_singlethread_workqueue("apds9922_report_workqueue");
	INIT_DELAYED_WORK(&obj->ps_report_work, apds9922_ps_data_report);

	atomic_set(&obj->ps_first_int, 1);
	atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
	//atomic_set(&obj->ps_base_value,65535); //comment by dengweicheng ,set ps in out threshold when setup driver
	atomic_set(&obj->ps_base_value, apds9922_ps_para[ps_para_num].base_value);
	apds9922_ps_select_para(apds9922_obj);

	atomic_set(&obj->i2c_retry, 3);
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
	clear_bit(CMC_BIT_ENG_ALS, &obj->enable);
	clear_bit(CMC_BIT_ENG_PS, &obj->enable);

	//wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");	
	apds9922_i2c_client = client;

	//regsister sensor class
	obj->als_cdev = sensors_light_cdev0;
	obj->als_cdev.sensors_enable = als_enable_set;
	obj->als_cdev.sensors_poll_delay = als_poll_delay_set;
	obj->als_cdev.min_delay = ALS_MIN_POLL_DELAY * 1000;

	obj->ps_cdev = sensors_proximity_cdev0;
	obj->ps_cdev.sensors_enable = ps_enable_set;
	obj->ps_cdev.sensors_poll_delay = NULL;
	obj->ps_cdev.min_delay = PS_MIN_POLL_DELAY * 1000;

	err = sensors_classdev_register(&client->dev, &obj->als_cdev);
	if(err)
		goto exit_init_failed;

	err = sensors_classdev_register(&client->dev, &obj->ps_cdev);
	if(err)
		goto exit_init_failed;

	//init default poll delay
	atomic_set(&obj->ps_poll_delay, PS_DEFAULT_POLL_DELAY);
	atomic_set(&obj->als_poll_delay, ALS_DEFAULT_POLL_DELAY);

	
	if((err = misc_register(&apds9922_device)))
	{
		printk(KERN_ERR TAGE "apds9922_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = apds9922_create_attr(&als_ps_driver.driver)))
	{
		printk(KERN_ERR TAGE "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = apds9922_early_suspend,
	obj->early_drv.resume   = apds9922_late_resume,    
	register_early_suspend(&obj->early_drv);
#else
	obj->fb_notif.notifier_call = apds9922_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif

	wake_lock_init(&(obj->wakelock), WAKE_LOCK_SUSPEND, "bbkapds9922");//liyuchu:prevent system suspend

#if defined(CONFIG_BBK_DEVICE_INFO)
	err = bbk_driver_register_device_info("ALS/PS", "0885159");
	if (err < 0) {
	printk(KERN_ERR TAGE "Failed to register ALS/PS device info\n");
	}
#endif

	print_vivo_init(TAGI "%s: OK in line %d\n", __func__,__LINE__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&apds9922_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	apds9922_i2c_client = NULL;           
	printk(KERN_ERR TAGE "%s: err = %d\n", __func__, err);
	return err;
}

static int apds9922_i2c_remove(struct i2c_client *client)
{
	struct apds9922_priv* obj = i2c_get_clientdata(client);
	int err;	
	if((err = apds9922_delete_attr(&als_ps_driver.driver)))
	{
		printk(KERN_ERR TAGE "apds9922_delete_attr fail: %d\n", err);
	}

	if((err = misc_deregister(&apds9922_device)))
	{
		printk(KERN_ERR TAGE "misc_deregister fail: %d\n", err);    
	}
	
	platform_driver_unregister(&als_ps_driver);
	apds9922_i2c_client = NULL;
	i2c_unregister_device(client);
	wake_lock_destroy(&obj->wakelock);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9922_probe(struct platform_device *pdev) 
{
	if(i2c_add_driver(&apds9922_i2c_driver))
	{
		printk(KERN_ERR TAGE "add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9922_remove(struct platform_device *pdev)
{	
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);    
	i2c_del_driver(&apds9922_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver apds9922_alsps_driver = {
	.probe      = apds9922_probe,
	.remove     = apds9922_remove,    
	.driver     = {
		.name  = "apds9922_als_ps",
		.owner = THIS_MODULE,
	}
};

struct platform_device sensor_apds9922_alsps = {	
	.name 	= "apds9922_als_ps",	
	.id		= -1,
};
/*----------------------------------------------------------------------------*/
static int __init apds9922_init(void)
{
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);

	if(platform_driver_register(&apds9922_alsps_driver))
	{
		printk(KERN_ERR TAGE "failed to register apds9922_als_ps driver");
		return -ENODEV;
	}
	if(platform_device_register(&sensor_apds9922_alsps))
	{
		printk(KERN_ERR TAGE "failed to register apds9922_als_ps device\n");
		return -ENODEV;
	}
	wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit apds9922_exit(void)
{
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);
	platform_driver_unregister(&apds9922_alsps_driver);
}
/*----------------------------------------------------------------------------*/
late_initcall(apds9922_init);
module_exit(apds9922_exit);
/*----------------------------------------------------------------------------*/

MODULE_DESCRIPTION("APDS9922 mbient light + proximity sensor driver");
MODULE_AUTHOR("james<james@vivo.com.cn>");
MODULE_LICENSE("GPL");


