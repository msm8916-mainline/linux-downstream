/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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

/* apds9960.c - APDS9960 ALS/PS driver
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/sensors.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#endif
#include <linux/bbk_alsps_para.h>
#include <linux/sensors_io.h>
#include <linux/rtc.h>
#include <asm/div64.h>
#include <linux/vivo_sensor_config.h>


#include "apds9960.h"
 
#define APDS9960_DEV_NAME     "APDS9960" 
#define APDS9960_RESTART_FILTER_TIME    100   //100MS
#define MAX_DELTA_BUFFER_NUM  40        //40 * 70 = 2800ms

#define APDS9960_ALS_MIN_POLL_DELAY     120
#define APDS9960_ALS_DEFAULT_POLL_DELAY	120

#define APDS9960_PS_MIN_POLL_DELAY      10
#define APDS9960_PS_DEFAULT_POLL_DELAY	120

#define APDS9960_GS_MIN_POLL_DELAY      10
#define APDS9960_GS_DEFAULT_POLL_DELAY	100      //not used now

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		    1

/******************************************************************************
 * extern functions
*******************************************************************************/

//extern unsigned int mt_eint_ack(unsigned int eint_num);
//extern void mt_eint_unmask(unsigned int line);
//extern void mt_eint_mask(unsigned int line);
//extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
//extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
//extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
//extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
//extern void mt_eint_disable_debounce(unsigned int eint_num);
extern int module_check(void);


/******************************************************************************
 * local functions
*******************************************************************************/
static int apds9960_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int apds9960_i2c_remove(struct i2c_client *client);
//static int apds9960_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int apds9960_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9960_i2c_resume(struct i2c_client *client);

static int apds9960_enable_gesture_calibration(struct i2c_client *client, struct apds9960_data *data);
static int apds9960_check_runtime_gesture_calibration(struct i2c_client *client, struct apds9960_data *data, unsigned char* gfifo_data, int gfifo_num);

static unsigned int GetTickCount(void);
static int FilterGestureRawData(gesture_data_type *, gesture_data_type *);
static int GestureDataProcessing(void);
static int DecodeGesture(int gesture_mode);
static void ResetGestureParameters(void);

int ps_enable_for_apds9960 = 0;
int ps_status_for_apds9960 = 1;
int ps_ic_apds9960_working = 0;
EXPORT_SYMBOL_GPL(ps_enable_for_apds9960);
EXPORT_SYMBOL_GPL(ps_status_for_apds9960);
EXPORT_SYMBOL_GPL(ps_ic_apds9960_working);


/******************************************************************************
 * local Variable
*******************************************************************************/
static const struct i2c_device_id apds9960_i2c_id[] = {{APDS9960_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_apds9960 = {I2C_BOARD_INFO("APDS9960", 0x39)};

static struct sensors_classdev sensors_light_cdev0 = {
	.name = "APDS9960-light",
	.vendor = "Avago",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65553",
	.resolution = "0.0125",
	.sensor_power = "0.15",
	.min_delay = 120000, 
	//min_delay == 0 equals SENSOR_FLAG_ON_CHANGE_MODE
	//see in frameworks/native/libs/gui/Sensor.cpp's constructor -- mMinDelay
	//so do not set it as 0
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = APDS9960_ALS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev0 = {
	.name = "APDS9960-proximity",
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
	.delay_msec = APDS9960_PS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_gesture_cdev0 = {
	.name = "APDS9960-gesture",
	.vendor = "Avago",
	.version = 1,
	.handle = SENSORS_GESTURE_HANDLE,
	.type = SENSOR_TYPE_GESTURE,
	.max_range = "6553",
	.resolution = "5.0",
	.sensor_power = "0.18",
	.min_delay = 10,
	//min_delay == 0 equals SENSOR_FLAG_ON_CHANGE_MODE
	//see in frameworks/native/libs/gui/Sensor.cpp's constructor -- mMinDelay
	///so do not set it as 0
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = APDS9960_GS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_light_cdev1 = {
	.name = "TMG399X-light",
	.vendor = "Taos",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6553",
	.resolution = "0.0125",
	.sensor_power = "0.15",
	.min_delay = 120000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = APDS9960_ALS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev1 = {
	.name = "TMG399X-proximity",
	.vendor = "Taos",
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
	.delay_msec = APDS9960_PS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_gesture_cdev1 = {
	.name = "TMG399X-gesture",
	.vendor = "Taos",
	.version = 1,
	.handle = SENSORS_GESTURE_HANDLE,
	.type = SENSOR_TYPE_GESTURE,
	.max_range = "6553",
	.resolution = "5.0",
	.sensor_power = "0.18",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = APDS9960_GS_DEFAULT_POLL_DELAY,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

//TODO: sensor_regulator for power control
struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator apds9960_vreg[] = {
	{NULL, "vdd", 2850000, 2850000},
	{NULL, "vddio", 1800000, 1800000},
};

/*
 * Global data
 */
static struct i2c_client *apds9960_i2c_client; /* global i2c_client to support ioctl */
//static struct workqueue_struct *apds_workqueue;

//static unsigned int als_filter = 0;
static unsigned char  apds9960_als_atime_tb[] = { 0xF6, 0xEE, 0xDC };           /* 27.8ms,  50.04ms , 100.08ms */
static unsigned char  apds9960_wtime_tb[] = { 0xFF, 0xF6, 0xEE };               /* 2.78ms,  27.8ms,  50.04ms */
//static unsigned short apds9960_als_integration_tb[] = { 240, 240, 240 };        // DO NOT use beyond 100.8ms
static unsigned int   apds9960_als_res_tb[] = { 0x2801, 0x4801, 0x9001 };
static unsigned int   apds9960_als_atime_us_tb[] = { 27800, 50040, 100080 };    /* 27.8ms,  50.04ms , 100.08ms */
static unsigned char  apds9960_als_again_tb[] = { 1, 4, 16, 64 };
static unsigned char  apds9960_als_again_bit_tb[] = { 0x00, 0x01, 0x02, 0x03 };

static int32_t RGB_COE_X[3] = {-77, 740, -544}; //{-1882, 10240, -8173}; // {-1.8816, 10.24, -8.173};
static int32_t RGB_COE_Y[3] = {-96, 765, -537}; //{-2100, 10130, -7708}; // {-2.0998, 10.13, -7.708};
static int32_t RGB_COE_Z[3] = {-119, 493, -259};//{-1937, 5201, -2435};  // {-1.937, 5.201, -2.435};

//static int RGB_CIE_N1 = 332; // 0.332;
//static int RGB_CIE_N2 = 186; // 0.1858;

//static int RGB_CIE_CCT1 = 449;  // 449.0;
//static int RGB_CIE_CCT2 = 3525; // 3525.0;
//static int RGB_CIE_CCT3 = 6823; // 6823.3;
//static int RGB_CIE_CCT4 = 5520; // 5520.33;

/* Gesture data storage */
static gesture_data_type gesture_data;

static int gesture_motion=DIR_NONE;
static int gesture_prev_motion=DIR_NONE;
static int fMotionMapped=0;

static int gesture_ud_delta=0;
static int gesture_lr_delta=0;
static int gesture_ud_delta_buffer[MAX_DELTA_BUFFER_NUM];
static int gesture_lr_delta_buffer[MAX_DELTA_BUFFER_NUM];
static int gesture_delta_count=0;
static int gesture_state=0;

static int negative_ud_delta=0;
static int positive_ud_delta=0;
static int negative_lr_delta=0;
static int positive_lr_delta=0;
static int ud_delta_positive_negative=0;	// 1 = positive then negative, 2 = negative then positive
static int lr_delta_positive_negative=0;

static int gesture_circle_state=0;
static int gesture_circle_cw_count=0;
static int gesture_circle_acw_count=0;

static int gesture_ud_count=0;
static int gesture_lr_count=0;

static int gesture_near_count=0;
static int gesture_far_count=0;

static int gesture_fundamentals=0;	// 0 = fundamentals, 1 = extra

//time check
static unsigned int gesture_start_time=0;
static unsigned int gesture_end_time=0;
static unsigned int gesture_time=0;
static unsigned int gesture_speed=0;

static unsigned int gesture_interval=0;
static unsigned int gesture_curr_time=0;
static unsigned int gesture_last_time=0;


static struct apds9960_data *apds9960_obj        = NULL;
static struct platform_driver apds9960_psgs_driver;
static struct platform_driver apds9960_als_driver;
static int chip_id = 0xff;
static int debug = 0;
static struct ps_para bbk_ps_paras_default[] = bbk_ps_para();
static struct als_para bbk_als_paras_default[] = bbk_als_para();
struct ps_para  *apds9960_ps_para  = NULL;
struct als_para *apds9960_als_para = NULL;
static int ps_para_num  = 0;          //the value must select early
static int als_para_num = 0;
static int ps_count = 0;

//static int disturbance =0;

typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS	   = 2,
	CMC_BIT_GS	   = 3,
	CMC_BIT_ENG_PS = 4,
}CMC_BIT;

static struct mutex apds9960_lock;
static struct mutex apds9960_ps_lock;
static struct mutex apds9960_gs_lock;
//static u8 nGMdata = 0;
static int alsCount = 0;

//weitianlei add start 
static int ps_eint_poll = 0;
//weitianlei add end 

static struct wake_lock ps_suspend_lock;

static int apds9960_i2c_read_block(char *buf, u8 addr, int length)
{
    int err;
	int tries = 0;
    
    struct i2c_client *this_client = apds9960_obj->client;
    struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,  //write
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = this_client->addr,
			.flags = 1,   //read
			.len = length,
			.buf = buf,
		},
	};
    
    mutex_lock(&apds9960_lock);
    
    do {
		err = i2c_transfer(this_client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));
	
	mutex_unlock(&apds9960_lock);
    
    if(err == 2)
	    return length;
    else
        return err;
}

static int apds9960_i2c_write_block(char *txData, int length)
{
    int err;
	int tries = 0;

	struct i2c_client *this_client = apds9960_obj->client;
    struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
    };
	
    mutex_lock(&apds9960_lock);

	do {
		err = i2c_transfer(this_client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));
    
    mutex_unlock(&apds9960_lock);
    
    return err;
}

static int apds9960_write_reg(u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	
    buf[0] = regaddr;
	buf[1] = regvalue;
    return  apds9960_i2c_write_block(buf, sizeof(buf));
}

static int apds9960_read_reg(u8 regaddr, u8 *regvalue)
{
	return apds9960_i2c_read_block(regvalue, regaddr, 1);
}

static struct i2c_driver apds9960_i2c_driver = {	
	.probe      = apds9960_i2c_probe,
	.remove     = apds9960_i2c_remove,
	//.detect     = apds9960_i2c_detect,
	.suspend    = apds9960_i2c_suspend,
	.resume     = apds9960_i2c_resume,
	.id_table   = apds9960_i2c_id,
    //.address_data = &tmd2771_addr_data,
	.driver = {
        //.owner        = THIS_MODULE,
		.name           = APDS9960_DEV_NAME,
	},
};

/******************  Management functions ************************/
static int apds9960_clear_interrupt(struct i2c_client *client, u8 command)
{
	int ret;

	ret = apds9960_write_reg(command, 1);
	return ret;
}

static int apds9960_set_enable(struct i2c_client *client, u8 enable)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;

    ret = apds9960_write_reg(APDS9960_ENABLE_REG, enable);
    if(ret < 0)
        printk("[APDS9960] apds9960_set_enable enable = %d\r\n",enable);
    
	return ret;
}
/******************  Management functions ************************/
static int apds9960_set_atime(struct i2c_client *client, u8 atime)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
    ret = apds9960_write_reg(APDS9960_ATIME_REG,atime);
	//data->atime = atime;
     if(ret < 0)
        printk("[APDS9960] apds9960_set_atime atime = %d\r\n",atime);
    
	return ret;
}

static int apds9960_set_wtime(struct i2c_client *client, u8 wtime)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
      ret = apds9960_write_reg(APDS9960_WTIME_REG,wtime);
	//data->wtime = wtime;
      if(ret < 0)
        printk("[APDS9960] apds9960_set_wtime wtime = %d\r\n",wtime);
	return ret;
}

static int apds9960_set_ailt(struct i2c_client *client, u16 threshold)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_AILTL_REG, (threshold & 0xFF));	
      ret = apds9960_write_reg(APDS9960_AILTH_REG, ((threshold >> 8) & 0xFF));	
      if(ret < 0)
        printk("[APDS9960] apds9960_set_ailt threshold = %d\r\n",threshold);
	//data->ailt = threshold;

	return ret;
}

static int apds9960_set_aiht(struct i2c_client *client, u16 threshold)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_AIHTL_REG, (threshold & 0xFF));
      ret = apds9960_write_reg(APDS9960_AIHTH_REG, ((threshold >> 8) & 0xFF));
	if(ret < 0)
        printk("[APDS9960] apds9960_set_aiht threshold = %d\r\n",threshold);
	//data->aiht = threshold;

	return ret;
}

static int apds9960_set_pilt(struct i2c_client *client, u8 threshold)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_PITLO_REG, threshold);	
	//data->pilt = threshold;
      if(ret < 0)
        printk("[APDS9960] apds9960_set_pilt  threshold = %d\r\n",threshold);
	return ret;
}

static int apds9960_set_piht(struct i2c_client *client, u8 threshold)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_PITHI_REG, threshold);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_piht  threshold = %d\r\n",threshold);
	//data->piht = threshold;

	return ret;
}

static int apds9960_set_pers(struct i2c_client *client, u8 pers)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_PERS_REG, pers);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_pers  pers = %d\r\n",pers);
	//data->pers = pers;

	return ret;
}

static int apds9960_set_config(struct i2c_client *client, u8 config)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_CONFIG_REG, config);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_config  config = %d\r\n",config);
	//data->config = config;

	return ret;
}

static int apds9960_set_ppulse(struct i2c_client *client, u8 ppulse)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_PPULSE_REG, ppulse);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_ppulse  ppulse = %d\r\n",ppulse);
	//data->ppulse = ppulse;

	return ret;
}

static int apds9960_set_control(struct i2c_client *client, u8 control)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_CONTROL_REG, control);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_control  control = %d\r\n",control);
	//data->control = control;

	return ret;
}

static int apds9960_set_aux(struct i2c_client *client, u8 aux)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_AUX_REG, aux);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_aux  aux = %d\r\n",aux);
	//data->aux = aux;

	return ret;
}

static int apds9960_set_poffset_ur(struct i2c_client *client, u8 poffset_ur)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_POFFSET_UR_REG, poffset_ur);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_poffset_ur  poffset_ur = %d\r\n",poffset_ur);
	//data->poffset_ur = poffset_ur;

	return ret;
}

static int apds9960_set_poffset_dl(struct i2c_client *client, u8 poffset_dl)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_POFFSET_DL_REG, poffset_dl);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_poffset_dl  poffset_dl = %d\r\n",poffset_dl);
	//data->poffset_dl = poffset_dl;

	return ret;
}

static int apds9960_set_pthd(struct i2c_client *client, u8 lowthd, u8 highthd)
{
    int err;
    int i;

    u8 lowthd_check, highthd_check;

    for(i=0; i<5; i++)
    {
        err = apds9960_set_pilt(client, lowthd);
	    err = apds9960_set_piht(client, highthd);
        err = apds9960_read_reg(APDS9960_PITLO_REG, &lowthd_check);
        err = apds9960_read_reg(APDS9960_PITHI_REG, &highthd_check);
        
        if(lowthd == lowthd_check && highthd == highthd_check && err >=0)
            break;
    }
    
    printk("[APDS9960] ps thd set %d[%d] : %d[%d] i %d err %d\r\n", lowthd, lowthd_check, highthd, highthd_check, i, err);

    return err;
}

/****************** Gesture related registers ************************/
static int apds9960_set_config2(struct i2c_client *client, u8 config2)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_CONFIG2_REG, config2);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_config2  config2 = %d\r\n",config2);
	//data->config2= config2;

	return ret;
}

static int apds9960_set_gthr_in(struct i2c_client *client, u8 gthr_in)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GTHR_IN_REG, gthr_in);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gthr_in  gthr_in = %d\r\n",gthr_in);
	//data->gthr_in = gthr_in;

	return ret;
}

static int apds9960_set_gthr_out(struct i2c_client *client, u8 gthr_out)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GTHR_OUT_REG, gthr_out);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gthr_out  gthr_out = %d\r\n",gthr_out);
	//data->gthr_out = gthr_out;

	return ret;
}

static int apds9960_set_gconf1(struct i2c_client *client, u8 gconf1)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GCONF1_REG, gconf1);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gconf1  gconf1 = %d\r\n",gconf1);
	//data->gconf1 = gconf1;

	return ret;
}

static int apds9960_set_gconf2(struct i2c_client *client, u8 gconf2)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GCONF2_REG, gconf2);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gconf2  gconf2 = %d\r\n",gconf2);
	//data->gconf2 = gconf2;

	return ret;
}

static int apds9960_set_goffset_u(struct i2c_client *client, u8 goffset_u)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GOFFSET_U_REG, goffset_u);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_goffset_u  goffset_u = %d\r\n",goffset_u);
	//data->goffset_u = goffset_u;

	return ret;
}

static int apds9960_set_goffset_d(struct i2c_client *client, u8 goffset_d)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GOFFSET_D_REG, goffset_d);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_goffset_d  goffset_d = %d\r\n",goffset_d);
	//data->goffset_d = goffset_d;

	return ret;
}

static int apds9960_set_gpulse(struct i2c_client *client, u8 gpulse)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GPULSE_REG, gpulse);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gpulse  gpulse = %d\r\n",gpulse);
	//data->gpulse = gpulse;

	return ret;
}

static int apds9960_set_goffset_l(struct i2c_client *client, u8 goffset_l)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GOFFSET_L_REG, goffset_l);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_goffset_l  goffset_l = %d\r\n",goffset_l);
	//data->gpulse = gpulse;
	//data->goffset_l = goffset_l;

	return ret;
}

static int apds9960_set_goffset_r(struct i2c_client *client, u8 goffset_r)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GOFFSET_R_REG, goffset_r);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_goffset_r  goffset_r = %d\r\n",goffset_r);
	//data->goffset_r = goffset_r;

	return ret;
}

static int apds9960_set_gconf3(struct i2c_client *client, u8 gconf3)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GCONF3_REG, gconf3);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gconf3  gconf3 = %d\r\n",gconf3);
	//data->gconf3 = gconf3;

	return ret;
}

static int apds9960_set_gctrl(struct i2c_client *client, u8 gctrl)
{
	//struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = apds9960_write_reg(APDS9960_GCTRL_REG, gctrl);
	if(ret < 0)
          printk("[APDS9960] apds9960_set_gctrl  gctrl = %d\r\n",gctrl);
	//data->gctrl = gctrl;

	return ret;
}

/******************   BBK parameters ************************/
static void set_ps_para_num(int para)
{
	int ps_para_count = bbk_ps_para_count;
	if(ps_para_count == 0)
		ps_para_count = ARRAY_SIZE(bbk_ps_paras_default);
	if(para >= 0 && para<= ps_para_count - 1)
		ps_para_num = para;
	else
	{
		//printk(KERN_ERR"%d is wrong ps index,[0,%d]\n", para, (sizeof(apds9960_ps_para)/sizeof(struct ps_para)-1));
		printk(KERN_ERR"%d is wrong ps index,", para);
		printk(KERN_ERR"[0,%d]\n", ps_para_count-1);
	}
}

static void set_als_para_num(int para)
{
	int als_para_count = bbk_als_para_count;
	if(als_para_count == 0)
		als_para_count = ARRAY_SIZE(bbk_als_paras_default);
	if( para >= 0 && para <= als_para_count - 1)
		als_para_num = para;
	else
		printk(KERN_ERR"%d is wrong als index,[0,%d]\n", para, als_para_count-1);
	
}

static int apds9960_ps_select_para(struct apds9960_data *obj)         //
{
    int out_threshold, in_threshold;   
    int base = atomic_read(&obj->ps_base_value);
    int step_num;
    const struct ps_step *step;
    const struct ps_para *para;
    
    //TODO: get different parameter here
    para = &apds9960_ps_para[ps_para_num];
    step = &para->step[0];
    printk("[APDS9960] base=%d\r\n", base);
    
    //count the the threshold from base value
    if(!(base >= para->base_value_min && base <= para->base_value_max))
        base = para->base_value;

    for(step_num = 0; step_num < para->step_num; step_num++, step++)
    {
        if(base >= step->min && base <= step->max)
            break;
    }

    if(debug)
        printk("[APDS9960] after base=%d\r\n",base);
    
    in_threshold  = base + step->in_coefficient;
    out_threshold = base + step->out_coefficient;
    printk("[APDS9960] select_threshold in_coefficient %d out_coefficient %d\n", step->in_coefficient, step->out_coefficient);

    atomic_set(&obj->ps_in_threshold,  in_threshold);
    atomic_set(&obj->ps_out_threshold, out_threshold);
    atomic_set(&obj->ps_base_value,    base);           //reset base value
    
    //NOTE: APDS9960 Proximity adc is U8
    //apds9960_set_pilt(obj->client, in_threshold);		
    //apds9960_set_piht(obj->client, out_threshold);
    
    printk("[APDS9960]ps_in_threshold %d ps_out_threshold %d\n", atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
	
    return 0; 
}

static int apds9960_als_select_para(struct apds9960_data *obj)
{
    //TODO: get different parameter here
    //als_para_num = 0; 
    
    return 0; 
}

static int apds9960_get_ps_value(struct apds9960_data *obj, unsigned int ps)
{
    int val;
    static int val_temp = 1;    //default is far away
    int ps_status_label = atomic_read(&obj->ps_status);
    
    if(obj->suspended) {
        printk(KERN_ERR"[APDS9960]%s, ps_suspend return !!!\n", __func__);
        return -1;
	}
    
    mutex_lock(&apds9960_ps_lock);

    //printk("[apds9960] in = %d out =%d", atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
    //first eint
    if(atomic_read(&obj->ps_first_int))
    {
        if((ps > atomic_read(&obj->ps_in_threshold)))
        {
            val = 0;          //close
            val_temp = 0;
            atomic_set(&obj->ps_status, 0);
        }
        else
        {
            val = 1;          //far away
		    val_temp = 1;
            atomic_set(&obj->ps_status, 1);
    	  } 
    }
    else
    {
        if((ps > atomic_read(&obj->ps_in_threshold)))
        {
            val = 0;          //close 
            val_temp = 0;
            atomic_set(&obj->ps_status, 0);
    	  }
	    else if((ps < atomic_read(&obj->ps_out_threshold)))
	    {
            val = 1;          //far away
		    val_temp = 1;
		    atomic_set(&obj->ps_status, 1);
    	    }
	    else
		    val = val_temp;	  //keep last status
    }
    
    if(ps_status_label != atomic_read(&obj->ps_status) || atomic_read(&obj->ps_first_int))
    {
    	   struct timespec ts;
	    struct rtc_time tm;

	    getnstimeofday(&ts);
	    rtc_time_to_tm(ts.tv_sec, &tm);
	    printk("change ps: %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
            tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		    tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

        printk("[APDS9960]%s,ps_status_label = %d  ps_status=%d ps_value= %d in_threshold=%d out_threshold=%d\n", __func__,
            	ps_status_label, atomic_read(&obj->ps_status), ps, atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
         ps_status_for_apds9960 = atomic_read(&obj->ps_status);
        //set the threshold to regsister
        if(atomic_read(&obj->ps_status))      //far away wait close
            apds9960_set_pthd(obj->client, 0, atomic_read(&obj->ps_in_threshold)); 
        else
            apds9960_set_pthd(obj->client, atomic_read(&obj->ps_out_threshold), 0xff);
	   if(ps_eint_poll <= 0 && obj->suspended == 0)
	   {
	        ps_eint_poll = 10;
	        queue_delayed_work(obj->poll_queue, &obj->ps_report_work, msecs_to_jiffies(50));
	   }
	   	
    }

    atomic_set(&obj->ps_first_int, 0);

    if(debug)
        printk("[APDS9960]%s, ps_value = %d  ps_status=%d \n",__func__, ps, atomic_read(&obj->ps_status));  
    
    mutex_unlock(&apds9960_ps_lock);

	return atomic_read(&obj->ps_status);
}

/******************   ALS PS GS data get ************************/
static int tmg399x_set_als_gain(struct apds9960_data *obj, int gain)
{
	int ret;
	u8 ctrl_reg;
    
	ret = apds9960_read_reg(APDS9960_CONTROL_REG, &ctrl_reg);
	if(ret < 0)
	{	
		printk("[APDS9960] tmg399x_set_als_gain APDS9960_CONTROL_REG \r\n");
	}
	ctrl_reg = ctrl_reg & (~TMG399X_ALS_GAIN_MASK); 
    switch (gain)
    {
	    case 1:
		    ctrl_reg |= APDS9960_ALS_GAIN_1X;
            obj->als_again_index = 0; 
		    break;
	    case 4:
		    ctrl_reg |= APDS9960_ALS_GAIN_4X;
            obj->als_again_index = 1; 
		    break;  
        case 16:
		    ctrl_reg |= APDS9960_ALS_GAIN_16X;
            obj->als_again_index = 2; 
		    break;
	    case 64:
		    ctrl_reg |= APDS9960_ALS_GAIN_64X;
            obj->als_again_index = 3; 
		    break;
	    default:
		    break;
    }
    
    ret = apds9960_set_control(obj->client, ctrl_reg);
    
	return ret;
}

static void tmg399x_calc_cpl(struct apds9960_data *data)
{
	unsigned int cpl;
	unsigned int sat;
    unsigned int tmp1;
    unsigned int tmp2;
	uint8_t atime = apds9960_als_atime_tb[data->als_atime_index];
    
	cpl = 256 - atime;
	cpl *= TMG399X_ATIME_PER_100;                  //2.78ms*100
	cpl /= 100;     //ms
	cpl *= als_gains[data->als_again_index];
	
    tmp1 = MAX_ALS_VALUE;
	tmp2 = (unsigned int)(256 - atime) << 10;
	sat  = (tmp1 < tmp2)? tmp1 : tmp2;
	sat  = sat * 8 / 10;
	
    data->cpl = cpl;
	data->saturation = sat;
}

static unsigned int tmg399x_get_lux(struct apds9960_data *data)
{
	int rp1, gp1, bp1, cp1;
	int64_t  lux = 0;
	unsigned int cct;
	unsigned int sat;
	unsigned int sf;
    unsigned char gain = als_gains[data->als_again_index];
    unsigned char change_gain = 0;
    int ret = 0;

	tmg399x_calc_cpl(data);
	sat = data->saturation;	
    if(gain == 16 && data->cdata >= sat) 
    {
        ret = tmg399x_set_als_gain(data, 1);
        change_gain = true;
	} 
    else if(gain == 16 && data->cdata < GAIN_SWITCH_LEVEL) 
    {
        ret = tmg399x_set_als_gain(data, 64);
        change_gain = true;
	}
    else if((gain == 64 && data->cdata >= (sat - GAIN_SWITCH_LEVEL)) ||
			(gain == 1  && data->cdata < GAIN_SWITCH_LEVEL)) 
    {
        ret = tmg399x_set_als_gain(data, 16);
        change_gain = true;
	}
    
	if (change_gain) 
    {
        //tmg399x_calc_cpl(dd_ptr);
        lux = data->lux;            //keep the last value
        ret = -1;
        goto exit;
	}
		
    if(data->cdata < MIN_ALS_VALUE) 
    {
        if(data->cdata < 1)
            lux = 0;
		else
			lux = 1;
        printk(KERN_ERR"[APDS9960]%s, cdata limited %d!!!\n", __func__, data->cdata);
		goto exit;
	} 
    else if(data->cdata >= sat) 
    {
        lux = data->lux; 
        printk(KERN_ERR"[APDS9960]%s, cdata saturation %d!!!\n", __func__, data->cdata);
        goto exit;
   }
		
	rp1 = data->rdata - data->ir;
	gp1 = data->gdata - data->ir;
	bp1 = data->bdata - data->ir;
	cp1 = data->cdata - data->ir;
    
    if(!data->cpl)
		data->cpl = 1;

	if(data->rdata > data->ir)
	{
	    	lux += data->segment.r_coef * rp1;
//		printk("red tmg399x_get_lux: lux rp1 = %lld\r\n", lux);
	}
      else
		printk("red tmg399x_get_lux: lux rp1 = %d\r\n", (data->segment.r_coef * rp1));
    
	if(data->gdata > data->ir)
	{
		lux += data->segment.g_coef * gp1;
//		printk("green tmg399x_get_lux: lux rp1 = %lld\r\n", lux);
	}
	else 
		printk("green tmg399x_get_lux: lux rp1 = %d\r\n", (data->segment.g_coef * gp1));			

	if(data->bdata > data->ir)
	{
		lux += data->segment.b_coef * bp1;
//		printk("blue tmg399x_get_lux: lux rp1 = %lld\r\n",lux);
	}
	else 
		printk("blue tmg399x_get_lux: lux rp1 = %d\r\n", (data->segment.b_coef * bp1));

    if(lux < 0)
      lux=0;

	sf = data->cpl;
	if(debug)
	{
		printk("tmg399x sf = %d\r\n",sf);
	}
	if (sf > 131072)
		goto error;
     /*
	if(lux > 100000)
	{
	   
	      	lux /= sf;
		lux *= data->segment.d_factor;
		  // printk("tmg399x11 lux  = %ld\r\n",lux);
	}
	else
	{
		lux *= data->segment.d_factor;
		lux /= sf;
	     // printk("tmg399x22 lux  = %ld\r\n",lux);
	}
	*/
/*   if(lux > (Maxvalue_OF_32BIT/data->segment.d_factor))
     {
        lux = Maxvalue_OF_32BIT/data->segment.d_factor -1;
     }*/
     lux *= data->segment.d_factor;
//     lux /= sf;
     do_div(lux, sf);
     if(lux > 50000000) 
     {
     	    lux = 50000000;
     }
	//lux += 500;     TODO:must check if this add is needed?
    // printk("tmg399x before als lux = %ld mlux\n", lux);
    if(lux >= 100 && lux < 1000) lux = 1010;
    
    if(lux >= 100 && lux < 2000){
//        lux = (lux * 100) / 100000;
        lux = (lux * 100);
        do_div(lux,100000);
    }
    else{
//        lux = (lux * apds9960_als_para[als_para_num].base_value) / 100000;     //als_factor = 100
        lux = lux * apds9960_als_para[als_para_num].base_value;
        do_div(lux,100000);
    }

   if(debug)
        printk("tmg399x als lux = %d mlux\n", (unsigned int)lux);

    if (lux > 30000)
		lux = 30000;
    
	if ( bp1 > 0 && rp1 > 0) 
      {
        cct = ((data->segment.ct_coef * bp1) / rp1) + data->segment.ct_offset;
		data->cct = cct;
	}
    
exit:
	data->lux = (uint16_t) lux;
	return 0;

error:
	return -1;
}

static int tmg399x_read_als(struct i2c_client *client, unsigned int *databuf)
{
    struct apds9960_data *data = i2c_get_clientdata(client);
   // unsigned int lux;
    unsigned char i2c_data[10] = {0};
    int status;
    if(atomic_read(&apds9960_obj->gesture_calibration)
        || atomic_read(&apds9960_obj->enabling_ps) 
        || atomic_read(&apds9960_obj->enabling_gesture))
    {
        printk(KERN_ERR"[APDS9960]apds9960_read_als als data not ready for gs_cal=%d ps_en=%d gs_en=%d!!!\r\n",
                        atomic_read(&apds9960_obj->gesture_calibration),
                        atomic_read(&apds9960_obj->enabling_ps),
                        atomic_read(&apds9960_obj->enabling_gesture));
        return -1; 
    }
    status = apds9960_i2c_read_block(i2c_data, APDS9960_CDATAL_REG, 8);
    if(status < 0) 
    {
        printk("tmg399x_read_als read data error\n");
        return -1; 
    }  
    
	data->cdata = ((i2c_data[1] << 8) | i2c_data[0]);    
	data->rdata = ((i2c_data[3] << 8) | i2c_data[2]); 
    data->gdata = ((i2c_data[5] << 8) | i2c_data[4]);  
    data->bdata = ((i2c_data[7] << 8) | i2c_data[6]); 
        
	//data->craw = ((i2c_data[1] << 8) | i2c_data[0]);
	//data->rraw = ((i2c_data[3] << 8) | i2c_data[2]);  
	//data->graw = ((i2c_data[5] << 8) | i2c_data[4]);  
	//data->braw = ((i2c_data[7] << 8) | i2c_data[6]);
#if defined(PD1420L) || defined(PD1420LG4) || defined(PD1420F) || defined(PD1420V)|| defined(PD1420F_EX)
	//as the R channel in this projcet is apparently higher than others,so we decrease it
	//to avoid calculation exception.(negative or small data maybe)
    if(data->rdata > (data->cdata)*4/10)    
       data->rdata = (data->cdata)*4/10;
#endif

      if((data->rdata + data->gdata + data->bdata) < data->cdata)
      	{
      		data->ir  = 0;
      	}
	else
	{
		data->ir = (int)(data->rdata + data->gdata + data->bdata - data->cdata + 1) / 2;
	}

    if(debug)
    {
	    printk("tmg399x cdata = %d rdata = %d gdata = %d bdata = %d\n", data->cdata, data->rdata, data->gdata, data->bdata);
	    printk("tmg399x irdata = %d\n", data->ir);
    }
   
    
    status = tmg399x_get_lux(data);
    if(status < 0)
    {
        printk("tmg399x_read_als get lux error\n");
        return -1; 
    }

    databuf[0] = data->lux;
    databuf[1] = data->cct; 

    return status;
}

static int LuxCalculation(struct i2c_client *client)
{
    struct apds9960_data *data = i2c_get_clientdata(client);
    unsigned long mlux, cal10kf;
    unsigned long lux;
    //u8 n = 0;
    
    if(debug)
        printk("[APDS9960] LuxCalculation cdata %d atime %d again %d\r\n", 
                    data->cdata,
                    apds9960_als_atime_us_tb[data->als_atime_index],
                    apds9960_als_again_tb[data->als_again_index]); 
    
    /*if(data->cdata < 200)
    {	
        n = 200 - data->cdata;
        cdata = cdata * powf(1.011, n);
    }*/
    
    cal10kf = apds9960_als_atime_us_tb[data->als_atime_index] * apds9960_als_again_tb[data->als_again_index];
    cal10kf = (100080*10000) / cal10kf;
    mlux = (data->cdata*cal10kf) / 10; 

    if (data->cdata > 0) 
    {
		if(((data->rdata * 100) / data->cdata) > 80)	            // 0.8
        {
			mlux=(mlux * APDS9960_LUX_GA2) / 100;
        }
		else if(((data->rdata  * 100) / data->cdata) > 40)	        // 0.4
		{
			mlux=(mlux * APDS9960_LUX_GA3) / 100;
		}
        else 
        {
			mlux=(mlux * APDS9960_LUX_GA1) / 100;
        }
    }

    //TODO: for ALS calibration
    //lux = (lux * dd_ptr->common.nv_db.als_factor)/100;    //als_factor = 100
	//mlux = (mlux * 1) / 100;
     
    //luanxinlin add
    if(mlux >= 100 && mlux < 1000) mlux = 1010;

    /* convert to lux */
    //TODO: for ALS calibration
    //data_lux = (dd_ptr->als.data_mlux *dd_ptr->als_base_value)/100000;
    if(mlux >= 100 && mlux < 2000)
        lux = (mlux * 100) / 100000;
    else
        lux = (mlux * apds9960_als_para[als_para_num].base_value) / 100000;       //als_factor = 100
    lux = (lux > 10000)? 10000 : lux;
    data->lux = lux;
    
    if(debug)
        printk("[APDS9960] Lux = %d\r\n", (uint32_t)lux);
    
    return lux;
}

static int apds9960_read_ps(struct i2c_client *client, unsigned char *data)
{
	u8 mode = 0;
	u8 err =0;
	//u8 buf[4] = {0};
	//unsigned int nProData = 0;
    if(atomic_read(&apds9960_obj->gesture_calibration))
    {
          if(ps_count < 20)
               ps_count++;
	    else
	        atomic_set(&apds9960_obj->gesture_calibration, 0);
    }
    if(!atomic_read(&apds9960_obj->ps_data_ready) 
        || atomic_read(&apds9960_obj->gesture_calibration)
        || atomic_read(&apds9960_obj->enabling_ps)
        || atomic_read(&apds9960_obj->enabling_gesture)) 
    	{
        printk(KERN_ERR"[APDS9960]apds9960_read_ps ps data not ready for ready=%d gs_cal=%d ps_en=%d gs_en=%d!!!\r\n",
                        atomic_read(&apds9960_obj->ps_data_ready),
                        atomic_read(&apds9960_obj->gesture_calibration),
                        atomic_read(&apds9960_obj->enabling_ps),
                        atomic_read(&apds9960_obj->enabling_gesture));
        return -1;
    	}
       ps_count = 0;
	err = apds9960_read_reg(APDS9960_GCTRL_REG, &mode);
      if(err < 0)
      	{
      		printk(KERN_ERR"[APDS9960] apds9960_read_ps APDS9960_GCTRL_REG \r\n");
      	}
	if((mode & 0x01) == 0x01 && apds9960_obj->enable_gesture_sensor == APDS_ENABLE_GESTURE)
	{
        printk(KERN_ERR"[APDS9960]apds9960_read_ps error this can not be happended !!!\r\n");
        //apds9960_set_gctrl(client, APDS9960_GFIFO_CLR); 
        return -1;
        /*apds9960_i2c_read_block(buf, APDS9960_GFIFO0_REG, 4);
		nProData = (buf[0] + buf[1] + buf[2] + buf[3]) / 2;
		if(nProData > 255)
			data[0] = 255;
		else
			data[0] = nProData &0xff;*/
	}
	else
	{
		err = apds9960_read_reg(APDS9960_PDATA_REG, data);
		 if(err < 0)
      		{
      			printk(KERN_ERR"[APDS9960] apds9960_read_ps APDS9960_PDATA_REG \r\n");
      		}
        if(debug)
		    printk("[APDS9960]apds9960_read_ps %d\r\n", data[0]);
	}
    
	return 0;
}

static void ResetGestureParameters()
{
	gesture_data.index = 0;
	gesture_data.total_gestures = 0;

	gesture_ud_delta = 0;
	gesture_lr_delta = 0;
    memset(gesture_ud_delta_buffer, 0, sizeof(gesture_ud_delta_buffer));
    memset(gesture_lr_delta_buffer, 0, sizeof(gesture_lr_delta_buffer));
    gesture_delta_count = 0;
	gesture_state    = 0;

	negative_ud_delta = 0;
	positive_ud_delta = 0;
	negative_lr_delta = 0;
	positive_lr_delta = 0;
	ud_delta_positive_negative = 0;
	lr_delta_positive_negative = 0;

	gesture_circle_state     = 0;
	gesture_circle_cw_count  = 0;
	gesture_circle_acw_count = 0;

	gesture_ud_count = 0;
	gesture_lr_count = 0;

	gesture_near_count = 0;
	gesture_far_count  = 0;

	gesture_prev_motion = DIR_NONE;
      gesture_motion = DIR_NONE;

	gesture_start_time = 0;
	gesture_end_time   = 0;
	gesture_time       = 0;
	gesture_speed      = 0;

    //gesture_interval   = 0;
    //gesture_curr_time  = 0;
    //gesture_last_time  = 0;
}

/*static void SaveToFile(void)       //dump to file
{
    struct file* fd;
	int i;
	char dummyc[100];

	fd = filp_open("/sdcard/gesture.csv", O_CREAT|O_APPEND|O_WRONLY, 0644);
    if (IS_ERR (fd))
    {
        printk(KERN_ERR "Failed to open file.\n");
    }
    else
    {
        for (i=0; i<gesture_data.total_gestures; i++) {
			sprintf(dummyc, "%d, %d, %d, %d\r\n", gesture_data.u_data[i], gesture_data.d_data[i], gesture_data.l_data[i], gesture_data.r_data[i]);
	    	fd->f_op->write(fd, dummyc, strlen(dummyc), &fd->f_pos);
			fd->f_pos += strlen(dummyc);
		}
    }
	filp_close(fd, NULL);
}*/

static unsigned int GetTickCount(void)     //ms
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return ((tv.tv_sec * 1000) + (tv.tv_usec / 1000));       // return in msec
}

static int FilterGestureRawData(gesture_data_type *gesture_in_data, gesture_data_type *gesture_out_data)
{
    int u_delta=0, d_delta=0, l_delta=0, r_delta=0;
	int i;

	if (gesture_in_data->total_gestures > 32 || gesture_in_data->total_gestures <= 0) 
    {
        gesture_out_data->total_gestures = 0;
        return -1;
    }

    for(i=1; i<gesture_in_data->total_gestures; i++)
    {
        u_delta += abs(gesture_in_data->u_data[i] - gesture_in_data->u_data[i-1]);
        d_delta += abs(gesture_in_data->d_data[i] - gesture_in_data->d_data[i-1]);
        l_delta += abs(gesture_in_data->l_data[i] - gesture_in_data->l_data[i-1]);
        r_delta += abs(gesture_in_data->r_data[i] - gesture_in_data->r_data[i-1]);
    }

    
    if((u_delta==0 || d_delta==0 || l_delta==0 || r_delta==0))
    {
        gesture_out_data->total_gestures = 0;
        return -1;
    }

	gesture_out_data->total_gestures = 0;
	for (i=0; i<gesture_in_data->total_gestures; i++) {
		if (gesture_in_data->u_data[i] > gesture_in_data->out_threshold &&
			gesture_in_data->d_data[i] > gesture_in_data->out_threshold &&
			gesture_in_data->l_data[i] > gesture_in_data->out_threshold &&
			gesture_in_data->r_data[i] > gesture_in_data->out_threshold ) {		
                
                gesture_out_data->u_data[gesture_out_data->total_gestures] = gesture_in_data->u_data[i];
			    gesture_out_data->d_data[gesture_out_data->total_gestures] = gesture_in_data->d_data[i];
			    gesture_out_data->l_data[gesture_out_data->total_gestures] = gesture_in_data->l_data[i];
			    gesture_out_data->r_data[gesture_out_data->total_gestures] = gesture_in_data->r_data[i];

			    gesture_out_data->total_gestures++;
		}
	}

	if (gesture_out_data->total_gestures == 0) 
        return -1;

    if(gesture_out_data->total_gestures > 4) {
	    for (i=1; i<gesture_out_data->total_gestures-1; i++) {
		    gesture_out_data->u_data[i] = (gesture_out_data->u_data[i] + gesture_out_data->u_data[i-1] + gesture_out_data->u_data[i+1])/3;
		    gesture_out_data->d_data[i] = (gesture_out_data->d_data[i] + gesture_out_data->d_data[i-1] + gesture_out_data->d_data[i+1])/3;
		    gesture_out_data->l_data[i] = (gesture_out_data->l_data[i] + gesture_out_data->l_data[i-1] + gesture_out_data->l_data[i+1])/3;
		    gesture_out_data->r_data[i] = (gesture_out_data->r_data[i] + gesture_out_data->r_data[i-1] + gesture_out_data->r_data[i+1])/3;
	    }
    }

	return 1;
}

/*static void AveragingRawData(gesture_data_type *gesture_data)
{
	int i, j;
	int loop;

	loop = (gesture_data->total_gestures - 4);

	for (i=0; i<loop; i++) {
		for (j=0; j<4; j++) {
			gesture_data->u_data[i] += gesture_data->u_data[i+j+1];
			gesture_data->d_data[i] += gesture_data->d_data[i+j+1];
			gesture_data->l_data[i] += gesture_data->l_data[i+j+1];
			gesture_data->r_data[i] += gesture_data->r_data[i+j+1];
		}

		gesture_data->u_data[i] /= 5;
		gesture_data->d_data[i] /= 5;
		gesture_data->l_data[i] /= 5;
		gesture_data->r_data[i] /= 5;
	}

	for (i=loop; i<(gesture_data->total_gestures-1); i++) {
		for (j=0; j<(gesture_data->total_gestures-i-1); j++) {
			gesture_data->u_data[i] += gesture_data->u_data[i+j+1];
			gesture_data->d_data[i] += gesture_data->d_data[i+j+1];
			gesture_data->l_data[i] += gesture_data->l_data[i+j+1];
			gesture_data->r_data[i] += gesture_data->r_data[i+j+1];
		}

		if ((gesture_data->total_gestures-i) > 0)
		{
			gesture_data->u_data[i] /= (gesture_data->total_gestures-i);
			gesture_data->d_data[i] /= (gesture_data->total_gestures-i);
			gesture_data->l_data[i] /= (gesture_data->total_gestures-i);
			gesture_data->r_data[i] /= (gesture_data->total_gestures-i);
		}
	}
}

static int GestureZone(int ud, int lr)
{
	if((ud<0 && lr>0) || (ud==0 && lr>0)) {
		return GESTURE_ZONE_1;
	}
	else if (ud>0 && lr>0) {
		return GESTURE_ZONE_2;
	}
	else if ((ud>0 && lr<0) || (ud>0 && lr==0)) {
		return GESTURE_ZONE_3;
	}
	else if (ud<0 && lr<0) {
		return GESTURE_ZONE_4;
	}
	else
		return GESTURE_ZONE_UNKNOWN;
}

static int DecodeGestureZone(int mapped, int start, int end)
{
	if ((start==2 && end==1) || (start==3 && end==4)) {
		if (!mapped) {
			return DIR_UP;
		}
		else {	
			return DIR_RIGHT;	// mapped
		}		
	}
	else if ((start==1 && end==2) || (start==4 && end==3)) {
		if (!mapped) {
			return DIR_DOWN;
		}
		else {	
			return DIR_LEFT;	// mapped
		}		
	}
	else if ((start==1 && end==4) || (start==2 && end==3)) {
		if (!mapped) {
			return DIR_LEFT;
		}
		else {	
			return DIR_UP;	    // mapped
		}		
	}
	else if ((start==4 && end==1) || (start==3 && end==2)) {
		if (!mapped) {
			return DIR_RIGHT;
		}
		else {	
			return DIR_DOWN;	// mapped
		}		
	}
	else if (start==1 && end==3) {
		if (!mapped) {
			return DIR_DOWN_LEFT;
		}
		else {	
			return DIR_LEFT_UP;	// mapped
		}		
	}
	else if (start==3 && end==1) {
		if (!mapped) {
			return DIR_UP_RIGHT;
		}
		else {	
			return DIR_RIGHT_DOWN;	// mapped
		}		
	}
	else if (start==4 && end==2) {
		if (!mapped) {
			return DIR_RIGHT_DOWN;
		}
		else {	
			return DIR_DOWN_LEFT;	// mapped
		}		
	}
	else if (start==2 && end==4) {
		if (!mapped) {
			return DIR_LEFT_UP;
		}
		else {	
			return DIR_UP_RIGHT;	// mapped
		}		
	}

	return DIR_NONE;
}*/

static int DecodeMappedGesture(int mapped, int motion) 
{
	if (!mapped) {
		return motion;
	}
	else {
		switch (motion)
		{
		    case DIR_UP:
			    return DIR_RIGHT;
		    case DIR_DOWN:
			    return DIR_LEFT;
		    case DIR_LEFT:
			    return DIR_UP;
		    case DIR_RIGHT:
			    return DIR_DOWN;
		    case DIR_LEFT_UP:
			    return DIR_UP_RIGHT;
		    case DIR_RIGHT_DOWN:
			    return DIR_DOWN_LEFT;
		    case DIR_UP_RIGHT:
			    return DIR_RIGHT_DOWN;
		    case DIR_DOWN_LEFT:
			    return DIR_LEFT_UP;
		    case DIR_UP_U_TURN:
			    return DIR_RIGHT_U_TURN;
		    case DIR_DOWN_U_TURN:
			    return DIR_LEFT_U_TURN;
		    case DIR_LEFT_U_TURN:
			    return DIR_UP_U_TURN;
		    case DIR_RIGHT_U_TURN:
			    return DIR_DOWN_U_TURN;
		    default:
			    return DIR_NONE;
		}
	}
}

static int DecodeGesture(int gesture_mode)
{
	/* check timing */
	/*if ( (gesture_end_time - gesture_start_time) <= 100) {
		gesture_speed = 2; // fast
	}
	else if ((gesture_end_time - gesture_start_time) <= 300) {
		gesture_speed = 1; // medium
	}
	else {
		gesture_speed = 0; // slow
	}*/
    

	if (gesture_ud_count == -1 &&
		gesture_lr_count == 0 ) 
    {
        gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_UP); 
	}
	else if ( gesture_ud_count == 1 &&
			  gesture_lr_count == 0 ) 
    {
        gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_DOWN);
	}
	else if ( gesture_ud_count == 0 &&
			  gesture_lr_count == 1 ) 
    {
        gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_RIGHT); 
	}
	else if ( gesture_ud_count == 0 &&
			  gesture_lr_count == -1 ) 
    {
        gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_LEFT); 
	}
	else if ( gesture_ud_count == -1 &&
			  gesture_lr_count == 1 ) 
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
            gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_UP); 
		else 
			gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_RIGHT); 

	}
	else if ( gesture_ud_count == 1 &&
			  gesture_lr_count == -1 ) 
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
            gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_DOWN);
		else 
			gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_LEFT);
	}
	else if ( gesture_ud_count == -1 &&
			  gesture_lr_count == -1 ) 
    {
		if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
            gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_UP); 
		else 
			gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_LEFT); 
	}
	else if ( gesture_ud_count == 1 &&
			  gesture_lr_count == 1 ) 
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
            gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_DOWN); 
		else 
            gesture_motion = DecodeMappedGesture(fMotionMapped, DIR_RIGHT); 
	}
	else 
    {
        gesture_motion = DIR_NONE; 
		printk("[APDS9960] decode gesture error !!!\n");
		return -1;
	}

	return 1;
}

static int GestureDataProcessing()
{
	gesture_data_type gesture_out_data;
	int ud_delta, lr_delta;
	int sensitivity1_threshold=GESTURE_SENSITIVITY_LEVEL1;
	//int sensitivity2_threshold=GESTURE_SENSITIVITY_LEVEL2;
	int gesture_u_d_ratio_first, gesture_u_d_ratio_last;
	int gesture_l_r_ratio_first, gesture_l_r_ratio_last;
    int i,error=0;

	if(gesture_data.total_gestures < 4)
    {
        printk(KERN_ERR"[APDS9960]too limited data(%d) to processce\r\n", gesture_data.total_gestures);
        ud_delta = 0;   lr_delta = 0;
        error = 1;
        goto process;
    }
    
	/************** This is to detect fundamentals gesture ****************************/
	gesture_data.in_threshold  = GESTURE_GTHR_IN;
	gesture_data.out_threshold = GESTURE_GTHR_OUT-10;

	FilterGestureRawData(&gesture_data, &gesture_out_data);

	if (gesture_out_data.total_gestures < 4)
    {
        printk(KERN_ERR"[APDS9960]too limited data(%d) after filter\r\n", gesture_out_data.total_gestures);
        ud_delta = 0;   lr_delta = 0;
        error = 1;
        goto process;
    }
    
    if(debug)
        printk("[APDS9960]total gestures %d\r\n", gesture_out_data.total_gestures);

	if (gesture_out_data.u_data[0]==0) gesture_out_data.u_data[0] = 1;
	if (gesture_out_data.d_data[0]==0) gesture_out_data.d_data[0] = 1;
	if (gesture_out_data.l_data[0]==0) gesture_out_data.l_data[0] = 1;
	if (gesture_out_data.r_data[0]==0) gesture_out_data.r_data[0] = 1;

	if (gesture_out_data.u_data[gesture_out_data.total_gestures-1]==0) gesture_out_data.u_data[gesture_out_data.total_gestures-1] = 1;
	if (gesture_out_data.d_data[gesture_out_data.total_gestures-1]==0) gesture_out_data.d_data[gesture_out_data.total_gestures-1] = 1;
	if (gesture_out_data.l_data[gesture_out_data.total_gestures-1]==0) gesture_out_data.l_data[gesture_out_data.total_gestures-1] = 1;
	if (gesture_out_data.r_data[gesture_out_data.total_gestures-1]==0) gesture_out_data.r_data[gesture_out_data.total_gestures-1] = 1;

	gesture_u_d_ratio_first = (gesture_out_data.u_data[0] - gesture_out_data.d_data[0])*100/(gesture_out_data.u_data[0]+gesture_out_data.d_data[0]);
	gesture_l_r_ratio_first = (gesture_out_data.l_data[0] - gesture_out_data.r_data[0])*100/(gesture_out_data.l_data[0]+gesture_out_data.r_data[0]);
	gesture_u_d_ratio_last = (gesture_out_data.u_data[gesture_out_data.total_gestures-1] - gesture_out_data.d_data[gesture_out_data.total_gestures-1])*100/(gesture_out_data.u_data[gesture_out_data.total_gestures-1]+gesture_out_data.d_data[gesture_out_data.total_gestures-1]);
	gesture_l_r_ratio_last = (gesture_out_data.l_data[gesture_out_data.total_gestures-1] - gesture_out_data.r_data[gesture_out_data.total_gestures-1])*100/(gesture_out_data.l_data[gesture_out_data.total_gestures-1]+gesture_out_data.r_data[gesture_out_data.total_gestures-1]);

	ud_delta = (gesture_u_d_ratio_last - gesture_u_d_ratio_first);
	lr_delta = (gesture_l_r_ratio_last - gesture_l_r_ratio_first);
     

process: 
    if(gesture_delta_count < MAX_DELTA_BUFFER_NUM)
    {
        gesture_ud_delta_buffer[gesture_delta_count] = ud_delta;
        gesture_lr_delta_buffer[gesture_delta_count] = lr_delta;
        gesture_delta_count++;
    }
    
    if(gesture_delta_count == MAX_DELTA_BUFFER_NUM)
    {
        gesture_ud_delta = 0;
        gesture_lr_delta = 0;
    }
    else
    {
        gesture_ud_delta = 0;
        gesture_lr_delta = 0;

        if(gesture_delta_count < 4)
        {
            for(i=0; i<gesture_delta_count; i++)
            {
                gesture_ud_delta += gesture_ud_delta_buffer[i];
                gesture_lr_delta += gesture_lr_delta_buffer[i];
            }
        }
        else
        {
            gesture_ud_delta += gesture_ud_delta_buffer[0];
            gesture_ud_delta += gesture_ud_delta_buffer[1];
            gesture_ud_delta += gesture_ud_delta_buffer[gesture_delta_count - 1];
            gesture_ud_delta += gesture_ud_delta_buffer[gesture_delta_count - 2];

            gesture_lr_delta += gesture_lr_delta_buffer[0];
            gesture_lr_delta += gesture_lr_delta_buffer[1];
            gesture_lr_delta += gesture_lr_delta_buffer[gesture_delta_count - 1];
            gesture_lr_delta += gesture_lr_delta_buffer[gesture_delta_count - 2];
        }
    }

	/**************** for Left/Right/Up/Down start****************/
	if (gesture_ud_delta >= sensitivity1_threshold) {
		gesture_ud_count = 1;
	}
	else if (gesture_ud_delta <= -sensitivity1_threshold) {
		gesture_ud_count = -1;
	}
	else {
		gesture_ud_count = 0;
	}

	if (gesture_lr_delta >= sensitivity1_threshold) {
		gesture_lr_count = 1;
	}
	else if (gesture_lr_delta <= -sensitivity1_threshold) {
		gesture_lr_count = -1;
	}
	else
		gesture_lr_count = 0;

    if(gesture_ud_count != 0 || gesture_lr_count != 0)
    {
  	    if(gesture_ud_count == 0)
	    {
            if(gesture_ud_delta > 0) 
    			gesture_ud_count = 1;
  		    else if(gesture_ud_delta < 0) 
    			gesture_ud_count = -1;
  	    }
	    
        if(gesture_lr_count == 0)
	    {
		    if(gesture_lr_delta > 0) 
    			gesture_lr_count = 1;
  		    else if(gesture_lr_delta < 0) 
    			gesture_lr_count = -1;
	    }
    }
	/**************** for Left/Right/Up/Down end****************/

	return 1;
}


static void apds9960_gs_data_report(struct work_struct *work)
{
    struct apds9960_data *data = container_of((struct delayed_work *)work, struct apds9960_data, gs_report_work); 
    int gst_event = 0;
    int res;
    u8 enable_reg;

    printk("[APDS9960]report gs data\n");
   
    mutex_lock(&apds9960_gs_lock);
    
    if (gesture_motion == DIR_UP)
		gst_event = DIRECTION_TOP;
    else if (gesture_motion == DIR_DOWN)
		gst_event = DIRECTION_BOTTOM;
    else if (gesture_motion == DIR_LEFT)
		gst_event = DIRECTION_LEFT;
    else if (gesture_motion == DIR_RIGHT)
		gst_event = DIRECTION_RIGHT;
    else if (gesture_motion == DIR_NONE)
        gst_event = 0;
    
    if(gst_event && (test_bit(CMC_BIT_GS, (const volatile void *)&data->enable)) && data->enable_gesture_sensor == APDS_ENABLE_GESTURE)
    {
        input_report_abs(data->input_dev_gs, ABS_X, gst_event);
	    input_report_abs(data->input_dev_gs, ABS_Y, gesture_lr_delta);
	    input_report_abs(data->input_dev_gs, ABS_Z, gesture_ud_delta);
	    input_sync(data->input_dev_gs);
    }
     
    //if(data->enable_als_sensor == APDS_ENABLE_ALS_NO_INT)
    //{		
        res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable_reg);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]enable reg read error %d !!!\n", res);
            
        enable_reg = enable_reg | APDS9960_ALS_ENABLE;
            
        res = apds9960_write_reg(APDS9960_ENABLE_REG, enable_reg);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]enable reg write error %d !!!\n", res);
	//} 
    
    ResetGestureParameters();

    if(test_bit(CMC_BIT_GS, (const volatile void *)&data->enable))
        apds9960_set_gctrl(data->client, (APDS9960_GFIFO_CLR | APDS9960_GIEN));
  
    atomic_set(&data->gesture_start_flag, 0);
    //apds9960_clear_interrupt(data->client, CMD_CLR_ALL_INT);

    mutex_unlock(&apds9960_gs_lock);
}

static void apds9960_gs_resume(struct work_struct *work)
{
	struct apds9960_data *data = container_of((struct delayed_work *)work, struct apds9960_data, gs_resume_work);
    int res;
   
    mutex_lock(&apds9960_gs_lock);
    
    //check if gs enabel
    if(test_bit(CMC_BIT_GS, (const volatile void *)&data->enable))
    {
        res = apds9960_set_gthr_in(data->client, data->gthr_in);
        if(res < 0) 
            printk(KERN_ERR"[APDS9960] gesture resume gthr_in write error %d !!!\n", res); 
    
        res = apds9960_set_gctrl(data->client, (APDS9960_GFIFO_CLR | APDS9960_GIEN));
        if(res < 0) 
            printk(KERN_ERR"[APDS9960] gesture resume gctrl write error %d !!!\n", res);

        printk("[APDS9960] gesture resume with in=%d out=%d\n", data->gthr_in, data->gthr_out);
    }
    
    mutex_unlock(&apds9960_gs_lock);
}

static void apds9960_ps_monitor(struct work_struct *work)
{
	struct apds9960_data *data = container_of((struct delayed_work *)work, struct apds9960_data, gs_ps_monitor);
    u8 ps_data;
    u8 enable;
    int res;
   
    mutex_lock(&apds9960_gs_lock);
    
    //check if gs enabel
    if(test_bit(CMC_BIT_GS, (const volatile void *)&data->enable))
    {
        res = apds9960_read_reg(APDS9960_PDATA_REG, &ps_data);
        if(res < 0)
		    printk(KERN_ERR"[APDS9960] guesture ps monitor pdata read error %d\r\n", res);
        printk("[APDS9960] guesture ps monitor pdata=%d\r\n", ps_data);

        if(ps_data == 0 || ps_data > 40)
            data->gesture_ps_excp_cont++;
        else
            data->gesture_ps_excp_cont = 0;

        if(data->gesture_ps_excp_cont > 6)     //3s     //500ms sample
        {
            res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable);
            if(res < 0)
		        printk(KERN_ERR"[APDS9960] guesture ps monitor enable read error %d\r\n", res);

            //TODO:may need use run time calibration
			if((enable & (APDS9960_GESTURE_ENABLE | APDS9960_PS_ENABLE)) == (APDS9960_GESTURE_ENABLE | APDS9960_PS_ENABLE))
                apds9960_enable_gesture_calibration(data->client, data); 
            
            apds9960_set_gctrl(data->client, APDS9960_GFIFO_CLR | APDS9960_GIEN);
            enable_irq(data->irq);

			data->gesture_ps_excp_cont = 0;
		}

        queue_delayed_work(data->gs_workqueue, &data->gs_ps_monitor, (HZ/2));    //500ms
    }
    
    mutex_unlock(&apds9960_gs_lock);
}

static void apds9960_gs_data_polling(struct apds9960_data *data)
{
	u8 gstatus = 0;
	u8 pdata   = 0;
    u8 gctrl   = 0;
	u8 gfifo_level;
	u8 gfifo_read = 0;
    //u8 buf[4] = {0};
	//u8 nMode = 0;
    //unsigned int nProData = 0;
    unsigned char gfifo_data[128] = {0};
	//int count;
	int i;
	int err;
     
    mutex_lock(&apds9960_gs_lock); 

    err = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
    if(err < 0)
        printk(KERN_ERR"[APDS9960] polling get pdata error %d\n", err);
    //gesture_start_time = GetTickCount();
    
    //NOTE: wait fifo data 40ms
    mdelay(38);	    // hope to collect more fifo - depends on GTIME 
					// use 30ms if GTIME = 0, use 60ms if GTIME = 2.8ms    //GTIM now is 0ms

	err = apds9960_read_reg(APDS9960_GSTATUS_REG, &gstatus);
	//printk("[APDS9960]%s: gstatus = %d\n", __func__, gstatus);
    if(err < 0) 
    {
        printk("[APDS9960] %s read gstatus error(%d): exit 1\n", __func__, err);
        mutex_unlock(&apds9960_gs_lock);
        
        //NOTE: need resume the als
        gesture_motion = DIR_NONE;
        cancel_delayed_work(&data->gs_report_work);
        queue_delayed_work(data->gs_workqueue, &data->gs_report_work, (HZ/100)*12);    //120ms
        return;
    }

    if((gstatus & APDS9960_GVALID) == APDS9960_GVALID) 
    {
        //TODO:force to exit gesture ???
        //apds9960_set_gctrl(data->client, APDS9960_GIEN);

		err = apds9960_read_reg(APDS9960_GFIFO_LVL_REG, &gfifo_level);
		if(err < 0)
            printk(KERN_ERR"[APDS9960] polling get gfifo level error %d\n", err);

		if(gfifo_level > 0) 
        {
            gfifo_read = apds9960_i2c_read_block(gfifo_data, APDS9960_GFIFO0_REG, 4*gfifo_level);
            printk("[APDS9960] gfifo_level = %d gfifo_read = %d\r\n", gfifo_level, gfifo_read);

            if(gfifo_read >= 4 && gfifo_read <= 128) 
            {
                for(i=0; i<gfifo_read; i+=4) 
                {
                    if(ROTATE_180_DEGREE == 1)
                    {
                        gesture_data.u_data[gesture_data.index] = gfifo_data[i+3]; //gfifo_data[i+0];
                        gesture_data.d_data[gesture_data.index] = gfifo_data[i+2]; //gfifo_data[i+1];
                        gesture_data.l_data[gesture_data.index] = gfifo_data[i+0]; //gfifo_data[i+2];
                        gesture_data.r_data[gesture_data.index] = gfifo_data[i+1]; //gfifo_data[i+3];
                    }
                    else
                    {
                    	  gesture_data.u_data[gesture_data.index] = gfifo_data[i+2]; //gfifo_data[i+0];
	                gesture_data.d_data[gesture_data.index] = gfifo_data[i+3]; //gfifo_data[i+1];
                    gesture_data.l_data[gesture_data.index] = gfifo_data[i+1]; //gfifo_data[i+2];
                    gesture_data.r_data[gesture_data.index] = gfifo_data[i+0]; //gfifo_data[i+3];
                    }
                  
                    gesture_data.index++;
                    gesture_data.total_gestures++;
                    
                    if(debug)
                    {
			            printk("[APDS9960] gfifo read  U = %d\r\n", gfifo_data[i+2]);
			            printk("[APDS9960] gfifo read  D = %d\r\n", gfifo_data[i+3]);
			            printk("[APDS9960] gfifo read  L = %d\r\n", gfifo_data[i+1]);
			            printk("[APDS9960] gfifo read  R = %d\r\n", gfifo_data[i+0]);
                    }
				}
				
                if(GestureDataProcessing() > 0) 
                {
                    if(DecodeGesture(gesture_fundamentals) > 0)     
                    {
                        gesture_prev_motion = gesture_motion;
			 }
                    else
                    {
                        gesture_motion = DIR_NONE;
                    }
                    
                    err = apds9960_read_reg(APDS9960_GCTRL_REG, &gctrl);
                    if(err < 0)
                        printk(KERN_ERR"[APDS9960] polling get gctrl error %d\n", err);
                    printk("[APDS9960] polling get gctrl 0x%d\n", gctrl);

                    //NOTE:check if need force out gmod for als can run
                    if((gctrl & 0x01) == 0x01)        //Gmod is on
				    {
                        data->gesture_int_cont++;
                        printk("[APDS9960] gesture_int_cont %d\n", data->gesture_int_cont);
                        
					    if(data->gesture_int_cont == 8)
					    { 
                            //start a timer to resume the guesture
							cancel_delayed_work(&data->gs_resume_work);
                            queue_delayed_work(data->gs_workqueue, &data->gs_resume_work, (HZ/100)*30);    //300ms
						    
                            apds9960_set_gctrl(data->client, 0x00);	
						    apds9960_set_gthr_in(data->client, 0xff);
						    
                            //update flags
                            atomic_set(&data->gesture_exit_flag, 1);
						    data->gesture_int_cont = 0;//NOTE:start timer to report gesture, then resume the als
					    }
				    }
                    else
                    {
                        data->gesture_int_cont = 0;
                    }
                    
                    //NOTE:check if is continuous int, if need do runtime calibration
                    gesture_curr_time = GetTickCount();     //in ms 
                    gesture_interval  = gesture_curr_time - gesture_last_time;    //first in this will be a huge num
                    gesture_last_time = gesture_curr_time;
                    
                    if(gesture_interval < 150) {            //150ms
                        data->gesture_cont_int_cont++;
                    }
                    else if(atomic_read(&data->gesture_exit_flag)) {
                        atomic_set(&data->gesture_exit_flag, 0);
                    }
                    else {
                        data->gesture_cont_int_cont=0;
                    }
                       
                    printk("[APDS9960] gesture_cont_int_cont %d gesture_interval %d\n", data->gesture_cont_int_cont, gesture_interval);
                    if(data->gesture_cont_int_cont > 30)     //at least 3S
				    {
					    apds9960_check_runtime_gesture_calibration(data->client, data, gfifo_data, gfifo_read);
					    data->gesture_cont_int_cont = 0;
				    }
				}
                else
                {
                    gesture_motion = DIR_NONE;
                }
                 
                //NOTE:reset data buffer index
                gesture_data.index = 0;
                gesture_data.total_gestures = 0;
			}
        } 
    }
    else
    {
        printk(KERN_ERR"[APDS9960] no gfifo data valid !!!\n");
    }

    //NOTE:start timer to report gesture, then resume the als
    cancel_delayed_work(&data->gs_report_work);
    queue_delayed_work(data->gs_workqueue, &data->gs_report_work, (HZ/100)*12);    //120ms

    mutex_unlock(&apds9960_gs_lock);
}

/*static enum hrtimer_restart apds9960_gs_report_timer_func(struct hrtimer *timer)
{
	struct apds9960_data *data = container_of(timer, struct apds9960_data, gs_polling_timer) ;

	printk("[APDS9960] report timer\n");
	if(data != NULL)
	{
		//queue_work(data->gs_polling_workqueue, &data->gs_polling_work);
		//hrtimer_forward_now(&data->gs_polling_timer, data->gs_polling_delay) ;
	}
	return HRTIMER_NORESTART;

}*/

static enum hrtimer_restart apds9960_ps_timer_func(struct hrtimer *timer)
{
	struct apds9960_data  *data = container_of( timer, struct apds9960_data, ps_timer ) ;

	printk(KERN_ERR "apds9960_ps_timer_func\n" ) ;

	if( data != NULL )
	{
	    atomic_set(&data->ps_data_ready, 1);         //data ready
	}

	return HRTIMER_NORESTART ;
}
/* ALS polling routine */
static int apds9960_read_als(struct i2c_client *client, unsigned int *databuf)
{
	struct apds9960_data *data = i2c_get_clientdata(client);
	unsigned char change_gain  = 0;
	unsigned char control_data = 0;	
    unsigned int adc_saturation_data;
    unsigned int adc_low_data;
    unsigned char i2c_data[10] = {0};
	int status;
   
    if(atomic_read(&apds9960_obj->gesture_calibration)
        || atomic_read(&apds9960_obj->enabling_ps) 
        || atomic_read(&apds9960_obj->enabling_gesture))
    {
        printk(KERN_ERR"[APDS9960]apds9960_read_als als data not ready for gs_cal=%d ps_en=%d gs_en=%d!!!\r\n",
                        atomic_read(&apds9960_obj->gesture_calibration),
                        atomic_read(&apds9960_obj->enabling_ps),
                        atomic_read(&apds9960_obj->enabling_gesture));
        return -1; 
    }

    status = apds9960_i2c_read_block(i2c_data, APDS9960_CDATAL_REG, 8);
	if (status < 0) {
        printk("apds9960_read_als read data error");
        return -1;
    }
    
	data->cdata = (i2c_data[1]<<8) | i2c_data[0];
	data->rdata = (i2c_data[3]<<8) | i2c_data[2];
	data->gdata = (i2c_data[5]<<8) | i2c_data[4];
	data->bdata = (i2c_data[7]<<8) | i2c_data[6];

	LuxCalculation(client);
   
    adc_saturation_data = apds9960_als_res_tb[data->als_atime_index] * 99;
    adc_saturation_data = adc_saturation_data /100;
    
    adc_low_data = apds9960_als_res_tb[data->als_atime_index] * 1;
    adc_low_data = adc_low_data /100;

    if(data->cdata >= adc_saturation_data) 
    {
        if(data->als_again_index != APDS9960_ALS_GAIN_1X) 
        {
            data->als_again_index--;
            change_gain = true;

            printk("gain decrease to %d\r\n", apds9960_als_again_tb[data->als_again_index]);      
        }
    }
    else if(data->cdata < adc_low_data) 
    {
        if (data->als_again_index != APDS9960_ALS_GAIN_64X) 
        {
            data->als_again_index++;
            change_gain = true;

            printk("gain increase to %d\r\n", apds9960_als_again_tb[data->als_again_index]);  
        }
    }
    
    if (change_gain) 
    {
        status = apds9960_read_reg(APDS9960_CONTROL_REG, &control_data);
	  if(status < 0)
	  	printk(KERN_ERR"[APDS9960] apds9960_read_als APDS9960_CONTROL_REG \r\n");
        control_data  = (control_data & 0xFC);
        control_data  = (control_data | apds9960_als_again_bit_tb[data->als_again_index]);
        status = apds9960_set_control(client, control_data);
       
        return -1;	// changing GAIN, don't update SMGR first, wait for next cycle
    }
    else
    {
        databuf[0] = data->lux;
        databuf[1] = data->cct;
    }

    return status;
}

static int read_als(struct i2c_client *client, unsigned int *databuf)
{
    int ret = 0;
    if(ams_support == 1){//tmg399x only
        ret = tmg399x_read_als(client, databuf);
    }
    else{ 
            ret = apds9960_read_als(client, databuf);
    }
    return ret;
}

/*
static void apds9960_als_data_report(struct work_struct *work)
{
	struct apds9960_data *data = container_of((struct delayed_work *)work, struct apds9960_data, als_report_work);
    u32 databuf[2] = {0};
    u32 delay = atomic_read(&data->als_poll_delay);
    static u32 index = 0;

    if(als_filter < 2)
    {
        als_filter++;
    }
    else
    {
        if(read_als(data->client, databuf) >= 0)
        {
            input_report_abs(data->input_dev_als, ABS_X, databuf[0]);
            input_report_abs(data->input_dev_als, ABS_Y, (index%2)); 
            input_sync(data->input_dev_als);

            index++;
        }
    }

    if(data->enable_als_sensor && data->suspended == 0)
        queue_delayed_work(data->poll_queue, &data->als_report_work, msecs_to_jiffies(delay));
}
*/
/******************   ALS PS GS control ************************/
static int apds9960_enable_als_sensor(struct i2c_client *client, int val)
{
    struct apds9960_data *data = i2c_get_clientdata(client);
    u32 delay = atomic_read(&data->als_poll_delay);
    int res = 0;
	u8 buf = 0;
    unsigned char control_data = 0;	
	
    printk("[APDS9960] als enable=%d with enable_ps_sensor=%d enable_gesture_sensor %d\n", 
                                                             val, data->enable_ps_sensor, data->enable_gesture_sensor);
	if(val) 
    {
		if (data->enable_als_sensor == 0) 
             {
			res = apds9960_read_reg(APDS9960_ENABLE_REG, &buf);
            if(res < 0)
                printk("[APDS9960]enable buf read err=%d\r\n", res);

            printk("[APDS9960]enable buf = 0x%x\r\n", buf);
            
            data->als_again_index=APDS9960_ALS_GAIN_64X;
			res = apds9960_read_reg(APDS9960_CONTROL_REG, &control_data);
			control_data  = (control_data & 0xFC);
			control_data  = (control_data | apds9960_als_again_bit_tb[data->als_again_index]);
			res = apds9960_set_control(client, control_data);
            if(res < 0)
            {
                printk("[APDS9960] enable apds9960_set_control err=%d\r\n", res);
                return res;
            }
            
            buf = buf | APDS9960_PWR_ON | APDS9960_ALS_ENABLE | APDS9960_WAIT_ENABLE | APDS9960_PS_ENABLE;
			apds9960_set_enable(client, buf);
            
            data->enable_als_sensor = APDS_ENABLE_ALS_NO_INT;
            delay = 35;
            //start als report work
            //queue_delayed_work(data->poll_queue, &data->als_report_work, msecs_to_jiffies(delay));
            queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(delay));

            //TODO:may need to set ATIM AGAIN
            
            //refresh the filter
            //als_filter = 0;
		}
	}
	else 
    {
        if(data->enable_als_sensor == APDS_DISABLE_ALS)
            return 0;

        data->enable_als_sensor = APDS_DISABLE_ALS;
        
        //stop als reprot work
	  if(data->enable_ps_sensor == APDS_DISABLE_PS)
             cancel_delayed_work(&data->ps_report_work);     //stop report timer here

        //ps als open together
		if ((data->enable_ps_sensor == APDS_DISABLE_PS) &&
			(data->enable_gesture_sensor == APDS_DISABLE_GESTURE))  
        	{
			apds9960_set_enable(client, 0);     //power off
		}
	}
	alsCount = 0;
	return 0;
}

#if 0       //do not used now
static int apds9960_set_als_poll_delay(struct i2c_client *client, unsigned int val)
{
	struct apds9960_data *data = i2c_get_clientdata(client);
	int ret;
	int atime_index=0;
 	
	printk("[APDS9960] %s : %d\n", __func__, val);
    
	if (val <=20) {
		atime_index = APDS9960_ALS_RES_24MS;
	}
	else if (val <=80) {
		atime_index = APDS9960_ALS_RES_50MS;
	}
	else {	// APDS_ALS_POLL_SLOW
		atime_index = APDS9960_ALS_RES_100MS;
	}

	ret = apds9960_set_atime(client, apds9960_als_atime_tb[atime_index]);
	if (ret >= 0) {
		data->als_atime_index = atime_index;
		printk("[APDS9960] poll delay %d, atime_index %d\n", data->als_poll_delay, data->als_atime_index);
	}
	else
		return -1;
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//__cancel_delayed_work(&data->als_dwork);
	//flush_delayed_work(&data->als_dwork);
	//queue_delayed_work(apds_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
    printk("[APDS9960] %s: enable als sensor (%d)\n", __func__, val);
	return 0;
}
#endif

static void apds9960_ps_data_report(struct work_struct *work)
{
	struct apds9960_data *data = container_of((struct delayed_work *)work, struct apds9960_data, ps_report_work);
    u32 databuf[2] = {0};
    u32 delay = atomic_read(&data->ps_poll_delay);
    static u32 index = 0;
    if(data->enable_als_sensor)
    {
        if(alsCount < 20)
        {
        	alsCount += 1;
		delay = 35;
        }
        if(read_als(data->client, databuf) >= 0)
        {
            input_report_abs(data->input_dev_als, ABS_X, databuf[0]);
            input_report_abs(data->input_dev_als, ABS_Y, (index%2)); 
            input_sync(data->input_dev_als);
            index++;
        }
    }
	//weitianlei add ps_eint_poll
    if(data->enable_ps_sensor && (ps_eint_poll > 0)){
        if(apds9960_read_ps(data->client, (unsigned char *)&data->ps_data) >= 0)
        {
            apds9960_get_ps_value(data, data->ps_data);
        
            input_report_rel(data->input_dev_ps, REL_X, (atomic_read(&data->ps_status) + 1));
            //input_report_rel(data->input_dev_ps, REL_Y, (index%2));
            input_sync(data->input_dev_ps);
			//weitianlei add start
			ps_eint_poll--;
			//weitianlei add end
           //index++;
        }
    }
   

    if(((data->enable_ps_sensor && (ps_eint_poll > 0)) || data->enable_als_sensor) && data->suspended == 0)
        queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(delay));
}

static int apds9960_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds9960_data *data = i2c_get_clientdata(client);
    u32 delay = atomic_read(&data->ps_poll_delay);
    int res = 0;
	u8 buf = 0;
	
    printk("[APDS9960] ps enable=%d with enable_als_sensor=%d enable_gesture_sensor=%d\n", 
                                                                   val, data->enable_als_sensor, data->enable_gesture_sensor);
    
    //ps and gst can't work together, the only specail case in phone call 
	if(val) 
    {
		if(data->enable_ps_sensor == APDS_DISABLE_PS) 
        {
            atomic_set(&data->enabling_ps, 1);
            hrtimer_start( &data->ps_timer, data->ps_delay, HRTIMER_MODE_REL ) ;
            res = apds9960_read_reg(APDS9960_ENABLE_REG, &buf);
            if(res < 0)
                printk("[APDS9960]enable buf read err=%d\r\n", res);
            printk("[APDS9960]enable buf = 0x%x\r\n", buf);
           
            if(!data->enable_gesture_sensor)      //NOTE: if gesture on, used gesture parameters 
            {
                //need close first to reset the parameters
                
                //when ALS is in use, do not turn it off, otherwise will cause C0 data is 0;
                //While the moment turn on AutoBacklight will make brightness very low;
                //modified by vivo dengweicheng
                if(!data->enable_als_sensor){
                	apds9960_set_enable(client, 0);
                }
                mdelay(5);

			    apds9960_set_ppulse(client, data->ps_ppulse); 
                //set all to 0
                apds9960_set_poffset_ur(client, data->ps_poffset_ur);
			    apds9960_set_poffset_dl(client, data->ps_poffset_dl);
                apds9960_set_goffset_u(client, 0);
		        apds9960_set_goffset_d(client, 0);
		        apds9960_set_goffset_l(client, 0);
		        apds9960_set_goffset_r(client, 0);
                apds9960_set_aux(client, APDS9960_PS_LED_BOOST | 0x01);
                apds9960_set_control(client, APDS9960_PDRVIE_FOR_PS | APDS9960_PGAIN_FOR_PS | apds9960_als_again_bit_tb[data->als_again_index]);

                //close gst 
                buf &= (~APDS9960_GESTURE_ENABLE);
            } 
            
            //als ps always open together
            buf = buf | APDS9960_PWR_ON | APDS9960_ALS_ENABLE | APDS9960_WAIT_ENABLE | APDS9960_PS_ENABLE | APDS9960_PS_INT_ENABLE; 
            apds9960_set_enable(client, buf);
            
            if(data->enable_gesture_sensor)
                apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GIEN);
            else
            {
                apds9960_set_gctrl(client, APDS9960_GFIFO_CLR);     //clear GFIFO
                apds9960_set_gctrl(client, 0);
            }
            
		    //select the threshold
            atomic_set(&data->ps_first_int, 1);
            atomic_set(&data->ps_status, 1);            //default is faraway
		    atomic_set(&data->sys_prox_status, 1);      //1: stand for status far
            atomic_set(&data->ps_data_ready, 0);
			apds9960_ps_select_para(data);

            data->enable_ps_sensor = APDS_ENABLE_PS;
             ps_enable_for_apds9960 = 1;
		ps_status_for_apds9960 = atomic_read(&data->ps_status);
            //start als report work
            queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(delay));
            
            enable_irq(data->irq);

            //eint set up
            //apds9960_set_pthd(client, 0xff, 0);                 //first in force ps interrupt 
            apds9960_set_pers(client, APDS9960_PPERS_0 | APDS9960_APERS_60);

            atomic_set(&data->enabling_ps, 0);
		}
		ps_eint_poll = 10;
	} 
	else 
    {
        if(data->enable_ps_sensor == APDS_DISABLE_PS)
            return 0;

        data->enable_ps_sensor = APDS_DISABLE_PS;
	  ps_enable_for_apds9960 = 0;
        
        //stop als reprot work
        if(data->enable_als_sensor == APDS_DISABLE_ALS)
            cancel_delayed_work(&data->ps_report_work);     //stop report timer here

		if((data->enable_als_sensor == APDS_DISABLE_ALS) &&
                (data->enable_gesture_sensor == APDS_DISABLE_GESTURE)) 
        {
            disable_irq_nosync(data->irq);
			apds9960_set_enable(client, 0);	        //power off sensor		
		}
		else if(data->enable_gesture_sensor == APDS_ENABLE_GESTURE)
		{
            atomic_set(&data->enabling_gesture, 1); 
        
            res = apds9960_read_reg(APDS9960_ENABLE_REG, &buf);
            if(res < 0)
                printk("[APDS9960]enable buf read err=%d\r\n", res);
            printk("[APDS9960]enable buf = 0x%x\r\n", buf);

            //need close first to reset the parameters
            apds9960_set_enable(client, 0);
            mdelay(5);

            //reset parameters for gst 
            apds9960_set_ppulse(client, data->gesture_ppulse); 
            apds9960_set_poffset_ur(client, data->gesture_poffset_ur);
			apds9960_set_poffset_dl(client, data->gesture_poffset_dl);
            apds9960_set_goffset_u(client, data->gesture_goffset_u);
		    apds9960_set_goffset_d(client, data->gesture_goffset_d);
		    apds9960_set_goffset_l(client, data->gesture_goffset_l);
		    apds9960_set_goffset_r(client, data->gesture_goffset_r);
            
            apds9960_set_gthr_in(client, data->gthr_in);	
	        apds9960_set_gthr_out(client, data->gthr_out);
            apds9960_set_gconf1(client, APDS9960_GFIFO_8_LEVEL);

            apds9960_set_aux(client, APDS9960_GESTURE_LED_BOOST | 0x01);
            apds9960_set_control(client, APDS9960_PDRVIE_FOR_GESTURE | APDS9960_PGAIN_FOR_GESTURE | apds9960_als_again_bit_tb[data->als_again_index]); 
            
            //do calibration here
            //apds9960_enable_gesture_calibration(client, data); 
            
            if(data->enable_ps_sensor == APDS_DISABLE_PS)  
                buf = buf & (~APDS9960_PS_INT_ENABLE);                //disable ps int
            
            buf = buf | APDS9960_PWR_ON | APDS9960_ALS_ENABLE | APDS9960_WAIT_ENABLE | APDS9960_PS_ENABLE | APDS9960_GESTURE_ENABLE;
            apds9960_set_enable(client, buf); 

            apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GIEN);
            
            enable_irq(data->irq);

            atomic_set(&data->enabling_gesture, 0);

            //start ps poll timer
            cancel_delayed_work(&data->gs_ps_monitor);
            queue_delayed_work(data->gs_workqueue, &data->gs_ps_monitor, (HZ/2));    //500ms
		}
	}
	
	return 0;
}

static int apds9960_enable_gesture_sensor(struct i2c_client *client, int val)
{
	struct apds9960_data *data = i2c_get_clientdata(client);
    u32 delay = atomic_read(&data->ps_poll_delay);
 	int res = 0;
	u8 buf = 0;
	
    printk("[APDS9960] gs enable=%d with enable_als_sensor=%d enable_ps_sensor=%d\n", 
                                                        val, data->enable_als_sensor, data->enable_ps_sensor);
	if (val == APDS_ENABLE_GESTURE)
    	{
		if(data->enable_gesture_sensor == APDS_DISABLE_GESTURE) 
        	{
			data->enable_gesture_sensor = APDS_ENABLE_GESTURE;
			ResetGestureParameters();
            
            //if(data->enable_ps_sensor == APDS_ENABLE_PS)
            //{
                //printk("[APDS9960] ps already on !!!\n");
                //return 0;
            //}
            
            atomic_set(&data->enabling_gesture, 1); 

            res = apds9960_read_reg(APDS9960_ENABLE_REG, &buf);
            if(res < 0)
                printk("[APDS9960]enable buf read err=%d\r\n", res);
            printk("[APDS9960]enable buf = 0x%x\r\n", buf);

             //need close first to reset the parameters
            apds9960_set_enable(client, 0);
            mdelay(5);

			apds9960_set_ppulse(client, data->gesture_ppulse); 
            apds9960_set_poffset_ur(client, data->gesture_poffset_ur);
			apds9960_set_poffset_dl(client, data->gesture_poffset_dl);
            apds9960_set_goffset_u(client, data->gesture_goffset_u);
		    apds9960_set_goffset_d(client, data->gesture_goffset_d);
		    apds9960_set_goffset_l(client, data->gesture_goffset_l);
		    apds9960_set_goffset_r(client, data->gesture_goffset_r);
           
            apds9960_set_gthr_in(client, data->gthr_in);	
	        apds9960_set_gthr_out(client, data->gthr_out);
            apds9960_set_gconf1(client, APDS9960_GFIFO_8_LEVEL);

            apds9960_set_aux(client, APDS9960_GESTURE_LED_BOOST | 0x01);
            apds9960_set_control(client, APDS9960_PDRVIE_FOR_GESTURE | APDS9960_PGAIN_FOR_GESTURE | apds9960_als_again_bit_tb[data->als_again_index]);
            
            //do calibration here
            apds9960_enable_gesture_calibration(client, data); 
            
            if(data->enable_ps_sensor == APDS_DISABLE_PS)
                buf = buf & (~APDS9960_PS_INT_ENABLE);                //disable ps int
            buf = buf | APDS9960_PWR_ON | APDS9960_ALS_ENABLE | APDS9960_WAIT_ENABLE | APDS9960_PS_ENABLE | APDS9960_GESTURE_ENABLE;
            apds9960_set_enable(client, buf);
             
            apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GIEN);
            
            enable_irq(data->irq);

            atomic_set(&data->enabling_gesture, 0);
            
            //start ps poll timer
            cancel_delayed_work(&data->gs_ps_monitor);
            queue_delayed_work(data->gs_workqueue, &data->gs_ps_monitor, (HZ/2));    //500ms
        }
	} 
	else 
    {
        if(data->enable_gesture_sensor == APDS_DISABLE_GESTURE)
            return 0;

		data->enable_gesture_sensor = APDS_DISABLE_GESTURE;

        ResetGestureParameters();
        
        apds9960_set_gctrl(client, APDS9960_GFIFO_CLR);
        
        //force out of gesture mode
        apds9960_set_gctrl(client, 0x00);	
		apds9960_set_gthr_in(client, 0xff);
        
        //cancel ps poll timer
        cancel_delayed_work(&data->gs_ps_monitor);
        //cancel gs report timer
        cancel_delayed_work(&data->gs_report_work);          //stop report timer here

		if ( (data->enable_als_sensor == APDS_DISABLE_ALS) && 	  
            (data->enable_ps_sensor == APDS_DISABLE_PS) ) 
        {
            disable_irq_nosync(data->irq);
			apds9960_set_enable(client, 0);
		}
		else if(data->enable_ps_sensor == APDS_ENABLE_PS)
		{
            atomic_set(&data->enabling_ps, 1);

            res = apds9960_read_reg(APDS9960_ENABLE_REG, &buf);
            if(res < 0)
                printk("[APDS9960]enable buf read err=%d\r\n", res);
            printk("[APDS9960]enable buf = 0x%x\r\n", buf);
            
            if(!data->enable_gesture_sensor)      //NOTE: if gesture on, used gesture parameters 
            {
                //need close first to reset the parameters
                apds9960_set_enable(client, 0);
                mdelay(5);

                apds9960_set_ppulse(client, data->ps_ppulse); 
                //set all to 0
                apds9960_set_poffset_ur(client, data->ps_poffset_ur);
			    apds9960_set_poffset_dl(client, data->ps_poffset_dl);
                apds9960_set_goffset_u(client, 0);
		        apds9960_set_goffset_d(client, 0);
		        apds9960_set_goffset_l(client, 0);
		        apds9960_set_goffset_r(client, 0);
                apds9960_set_aux(client, APDS9960_PS_LED_BOOST | 0x01);
                apds9960_set_control(client, APDS9960_PDRVIE_FOR_PS | APDS9960_PGAIN_FOR_PS | apds9960_als_again_bit_tb[data->als_again_index]);

                //close gst
                buf &= (~APDS9960_GESTURE_ENABLE);
            }
            
            //als ps always open together
            buf = buf | APDS9960_PWR_ON | APDS9960_ALS_ENABLE | APDS9960_WAIT_ENABLE | APDS9960_PS_ENABLE | APDS9960_PS_INT_ENABLE;
	     
            apds9960_set_enable(client, buf);
           
            if(data->enable_gesture_sensor)
                apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GIEN);
            else
            {
                apds9960_set_gctrl(client, APDS9960_GFIFO_CLR);
                apds9960_set_gctrl(client, 0);
            }

		    //select the threshold
            atomic_set(&data->ps_first_int, 1);
            atomic_set(&data->ps_status, 1);            //default is faraway
		    atomic_set(&data->sys_prox_status, 1);      //1: stand for status far
            atomic_set(&data->ps_data_ready, 0);
			apds9960_ps_select_para(data);

            data->enable_ps_sensor= APDS_ENABLE_PS;

            //start als report work
            queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(delay));
            
            enable_irq(data->irq);

            //eint set up
            //apds9960_set_pthd(client, 0xff, 0);                 //first in force ps interrupt 
            apds9960_set_pers(client, APDS9960_PPERS_0 | APDS9960_APERS_60);

            atomic_set(&data->enabling_ps, 0);
        }
		
        //hrtimer_cancel( &data->gs_polling_timer ) ;
	}
	
	return 0;
}

//calibration resume delay
static void apds9960_resume_delay(struct work_struct *work)
{
    struct apds9960_data *data = container_of((struct delayed_work *)work, struct apds9960_data, gs_resume_delay);
   
    //mutex_lock(&apds9960_gs_lock);
    
    atomic_set(&data->gesture_calibration, 0);
    
    //mutex_unlock(&apds9960_gs_lock);
}

//enable calibration
static int apds9960_enable_gesture_calibration(struct i2c_client *client, struct apds9960_data *data)
{
    int i, j;
    u8 i2c_read_data[4];
    int res;
    
    u8 u_done, d_done, l_done, r_done;
    u8 n_u, n_d, n_l, n_r;
    u8 n_ur, n_dl;
    u8 enable;
    u8 status;
    u8 gthr_in, gthr_out;
    u8 gconf1;
    u8 gstatus;
    u8 udata, ddata, ldata, rdata;
    u8 pdata;
    u8 ur_flag, dl_flag, u_flag, d_flag, l_flag, r_flag;
    u8 ur_poffset, dl_poffset, u_goffset, d_goffset, l_goffset, r_goffset;
     
    printk("[APDS9960] apds9960_enable_calibration");
    if(!test_bit(CMC_BIT_GS, (const volatile void *)&data->enable))
    {
       printk("[APDS9960] apds9960_enable_calibration exit");
    	 return 0;
    }
    //set up calibration flag
    atomic_set(&data->gesture_calibration, 1);

	//disable enable
    res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration enable read err=%d\r\n", res);
	apds9960_set_enable(client, 0x00);		
	
	//force it into gesture mode and read data
    res = apds9960_read_reg(APDS9960_GTHR_IN_REG, &gthr_in);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gesture in read err=%d\r\n", res);
	apds9960_set_gthr_in(client, 0x00);		  //force into gesture mode 

    res = apds9960_read_reg(APDS9960_GTHR_OUT_REG, &gthr_out);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gesture out read err=%d\r\n", res);
	apds9960_set_gthr_out(client, 0x00);       //force into gesture mode 
     
    //TODO maybe check with default
	printk("[APDS9960]enable gesture calibration gthr_in = %d  gthr_out = %d\n", gthr_in, gthr_out);
     
	res = apds9960_read_reg(APDS9960_GCONF1_REG, &gconf1);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gconf1 read err=%d\r\n", res);
	apds9960_set_gconf1(client, APDS9960_GFIFO_1_LEVEL);
	
    res = apds9960_read_reg(APDS9960_GOFFSET_U_REG, &u_goffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration u_goffset read err=%d\r\n", res);	
		
	res = apds9960_read_reg(APDS9960_GOFFSET_D_REG, &d_goffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration d_goffset read err=%d\r\n", res);	
		
	res = apds9960_read_reg(APDS9960_GOFFSET_L_REG, &l_goffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration l_goffset read err=%d\r\n", res);	
		
	res = apds9960_read_reg(APDS9960_GOFFSET_R_REG, &r_goffset);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration r_goffset read err=%d\r\n", res);	
	
	res = apds9960_read_reg(APDS9960_POFFSET_UR_REG, &ur_poffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration ur_poffset read err=%d\r\n", res);
    
    res = apds9960_read_reg(APDS9960_POFFSET_DL_REG, &dl_poffset);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration dl_poffset read err=%d\r\n", res);
    
    printk("[APDS9960]enable gesture calibration goffset u:%d d:%d l:%d r:%d\n", 
                                                        u_goffset, d_goffset, l_goffset, r_goffset);
    printk("[APDS9960]enable gesture calibration poffset ur:%d dl:%d\n", 
                                                        ur_poffset, dl_poffset);

    apds9960_set_goffset_u(client, 0x00);
    apds9960_set_goffset_d(client, 0x00);
	apds9960_set_goffset_l(client, 0x00);
	apds9960_set_goffset_r(client, 0x00);
	apds9960_set_poffset_ur(client, 0);
	apds9960_set_poffset_dl(client, 0x00);

    ur_flag = 0;
	dl_flag = 0;
	u_flag = 0;
	d_flag = 0;
	l_flag = 0;
	r_flag = 0;

	//calibration U&R
	apds9960_set_config2(client, 0x26);    //NOTE: D and L are masked !!!
	apds9960_set_enable(client, 0x05);	

    for(j=0; j<10; j++)    //wait data ready
	{
        res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration status read err=%d\r\n", res);
        
        if ((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
            break;
        
        mdelay(1);         //1ms	
	}

	res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
    if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration pdata read err=%d\r\n", res);
	printk("enable calibration UR before: pdata = %d\n", pdata);
    
	if(pdata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		ur_flag = 1;

	n_ur = 128;
    data->gesture_poffset_ur = 0;
	
    for (i=0; i<7; i++)
	{
		n_ur /= 2;
		apds9960_set_enable(client, 0x05);		
		for(j=0; j<10; j++)    //wait data ready
		{
            mdelay(1);         //1ms
            res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
		    if(res < 0)
                printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration status read err=%d\r\n", res);

            if ((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
                break;
		}

		res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration pdata read err=%d\r\n", res);
        
        if(debug)
		    printk("pdata = %d (%d)\n", pdata, j);

		if (pdata > APDS9960_PS_CALIBRATED_XTALK)
		{
            if(ur_flag == 1)
			{
                if(data->gesture_poffset_ur < 127)
				{
					data->gesture_poffset_ur += n_ur;
					if (data->gesture_poffset_ur > 127)
						data->gesture_poffset_ur = 127;
				}
			}
			else
            {
                if(data->gesture_poffset_ur >= n_ur && data->gesture_poffset_ur < 127)
                    data->gesture_poffset_ur -= n_ur;						
            }
        }
    	else if (pdata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{ 
 			if(ur_flag == 1)
 			{
 				if(data->gesture_poffset_ur >=n_ur && data->gesture_poffset_ur < 127)
					data->gesture_poffset_ur -= n_ur;		
 			}
			else
			{	
				if (data->gesture_poffset_ur < 127)
				{
					data->gesture_poffset_ur += n_ur;
					if (data->gesture_poffset_ur > 127)
						data->gesture_poffset_ur = 127;
				}
			}					
		}
		else
		{
			printk("[APDS9960] Done poffset-ur = %d (%d) flag %d\n", data->gesture_poffset_ur, i, ur_flag);
			break;
		}
        
	    if(ur_flag == 1)
            apds9960_set_poffset_ur(client, data->gesture_poffset_ur);
	    else
	       	apds9960_set_poffset_ur(client, data->gesture_poffset_ur + 128);

		apds9960_set_enable(client, 0x00);	
	}
    
    if(ur_flag != 1)
       	 data->gesture_poffset_ur += 128;      //change to negative 
   
    //calibration D&L
    apds9960_set_enable(client, 0x00);		
    apds9960_set_config2(client, 0x29);     //NOTE: U and R are masked !!!
	apds9960_set_enable(client, 0x05);		

	for(j=0; j<10; j++)    //wait data ready
	{
        res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration status read err=%d\r\n", res);
        
        if ((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
            break;
        
        mdelay(1);         //1ms	
	}

	res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
    if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration pdata read err=%d\r\n", res);
	printk("enable calibration DL before: pdata = %d\n", pdata);
        
	if(pdata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		dl_flag = 1;
	
	n_dl = 128;
	data->gesture_poffset_dl = 0;
    
	for(i=0; i<7; i++)
	{
		n_dl /= 2;
		apds9960_set_enable(client, 0x05);		
		for(j=0; j<10; j++)    //wait data ready
		{
            mdelay(1);         //1ms
            res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
		    if(res < 0)
                printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration status read err=%d\r\n", res);

            if ((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
                break;
		}

		res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration pdata read err=%d\r\n", res);
		
        if(debug)
            printk("pdata = %d (%d)\n", pdata, j);
        
		if (pdata > APDS9960_PS_CALIBRATED_XTALK)
		{
            if(dl_flag == 1)
	        {
                if (data->gesture_poffset_dl < 127)
				{
                    data->gesture_poffset_dl += n_dl;
					if (data->gesture_poffset_dl > 127)
						data->gesture_poffset_dl = 127;
				}
		    }
			else
			{
				if(data->gesture_poffset_dl >=n_dl && data->gesture_poffset_dl < 127)
					data->gesture_poffset_dl -= n_dl;		
			}
			
		}
    	else if (pdata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
            if(dl_flag == 1)
		    {
                if(data->gesture_poffset_dl >=n_dl && data->gesture_poffset_dl < 127)
                   data->gesture_poffset_dl -= n_dl;	
		    }
			else
			{
                if(data->gesture_poffset_dl < 127)
				{
					data->gesture_poffset_dl += n_dl;
					if (data->gesture_poffset_dl > 127)
						data->gesture_poffset_dl = 127;					
				}
			}					
		}
		else
		{
            printk("[APDS9960] Done poffset-dl = %d (%d) flag %d\n", data->gesture_poffset_dl, i, ur_flag);
		    break;
		}
		
		if(dl_flag == 1)
			apds9960_set_poffset_dl(client, data->gesture_poffset_dl);	
		else
			apds9960_set_poffset_dl(client, data->gesture_poffset_dl + 128);	

		apds9960_set_enable(client, 0x00);		
	}
    
    if(dl_flag != 1)
       	data->gesture_poffset_dl += 128;  //change to negative 

    //calibration u & d & l & r
	apds9960_set_enable(client, 0x00);		
	apds9960_set_config2(client, 0x0);
	apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GMODE);	
	apds9960_set_enable(client, 0x41);

	n_u = 128;
	n_d = 128;
	n_l = 128;
	n_r = 128;
	data->gesture_goffset_u = 0;
	data->gesture_goffset_d = 0;
	data->gesture_goffset_l = 0;
	data->gesture_goffset_r = 0;
	u_done = 0;
	d_done = 0;
	l_done = 0;
	r_done = 0;
    
	for (j=0; j<10; j++)
	{
		res = apds9960_read_reg(APDS9960_GSTATUS_REG, &gstatus);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gstatus read err=%d\r\n", res);	

		if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID) 
            break;
        
		mdelay(1);      //1ms
	}

    res = apds9960_i2c_read_block(i2c_read_data, APDS9960_GFIFO0_REG, 4);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gfifo read err=%d\r\n", res);

	udata = i2c_read_data[0];
	ddata = i2c_read_data[1];
	ldata = i2c_read_data[2];
	rdata = i2c_read_data[3];
	apds9960_set_enable(client, 0x00);
	printk("enable calibration before: gdata_u=%d, gdata_d=%d\r\n", i2c_read_data[0], i2c_read_data[1]);
	printk("enable calibration before: gdata_l=%d, gdata_r=%d\r\n", i2c_read_data[2], i2c_read_data[3]);
    
	if(udata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		u_flag = 1;
	if(ddata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		d_flag = 1;
	if(ldata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		l_flag = 1;
	if(rdata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		r_flag = 1;

	for(i=0; i <7; i++)
	{	
		n_u /= 2;
		n_d /= 2;
		n_l /= 2;
		n_r /= 2;
		apds9960_set_enable(client, 0x41);
        
		for (j=0; j<10; j++)
		{
			res = apds9960_read_reg(APDS9960_GSTATUS_REG, &gstatus);
            	   if(res < 0)
                   printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gstatus read err=%d\r\n", res);

			if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID) 
				break;
            
			mdelay(1);      //1ms
		}
        
		res = apds9960_i2c_read_block(i2c_read_data, APDS9960_GFIFO0_REG, 4);
	    if(res < 0)
             printk(KERN_ERR"[APDS9960]apds9960_enable_gesture_calibration gfifo read err=%d\r\n", res);
		udata = i2c_read_data[0];
		ddata = i2c_read_data[1];
		ldata = i2c_read_data[2];
		rdata = i2c_read_data[3];
		printk("gdata_u=%d, gdata_d=%d\r\n", i2c_read_data[0], i2c_read_data[1]);
		printk("gdata_l=%d, gdata_r=%d\r\n", i2c_read_data[2], i2c_read_data[3]);
        
        //u offset
		if (udata > APDS9960_PS_CALIBRATED_XTALK)
		{
			if(u_flag == 1)
			{
				if (data->gesture_goffset_u < 127)
				{
					data->gesture_goffset_u += n_u;
					if (data->gesture_goffset_u > 127)
						data->gesture_goffset_u = 127;					
				}
			}
			else
			{
				if(data->gesture_goffset_u >=n_u && data->gesture_goffset_u < 127)
				    data->gesture_goffset_u -= n_u;		
			}
			
		}
		else if (udata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
            if(u_flag == 1)
            {
                if(data->gesture_goffset_u >=n_u && data->gesture_goffset_u < 127)
                    data->gesture_goffset_u -= n_u;	
            }
	     	else
	     	{
	     		if (data->gesture_goffset_u < 127)
				{
					data->gesture_goffset_u += n_u;
					if (data->gesture_goffset_u > 127)
						data->gesture_goffset_u = 127;					
				}
	     	}
		}
		else
		{
			u_done = 1;
			printk("[APDS9960] u_done %d (%d) u_flag %d\r\n", data->gesture_goffset_u, i, u_flag);
		}
        
        if(u_flag == 1)
            apds9960_set_goffset_u(client, data->gesture_goffset_u);
		else
			apds9960_set_goffset_u(client, data->gesture_goffset_u + 128);

        
        //d offset
		if(ddata > APDS9960_PS_CALIBRATED_XTALK)
		{
            if(d_flag == 1)
		    {
                if(data->gesture_goffset_d < 127)
				{
					data->gesture_goffset_d += n_d;
					if (data->gesture_goffset_d > 127)
						data->gesture_goffset_d = 127;					
				}
		    }
			else
			{
				if(data->gesture_goffset_d >= n_u && data->gesture_goffset_d < 127)
				    data->gesture_goffset_d -= n_d;	
			}	
		}
		else if (ddata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{ 
			if(d_flag == 1)
			{
				if(data->gesture_goffset_d >=n_u && data->gesture_goffset_d < 127)
				    data->gesture_goffset_d -= n_d;
			}
			else
			{
				if (data->gesture_goffset_d < 127)
				{
					data->gesture_goffset_d += n_d;
					if(data->gesture_goffset_d > 127)
						data->gesture_goffset_d = 127;			
				}
			}
	      								
		}
		else
		{
			d_done = 1;
			printk("[APDS9960] d_done %d (%d) d_flag %d\r\n", data->gesture_goffset_d, i, d_flag);
		}

		if(d_flag == 1)
			apds9960_set_goffset_d(client, data->gesture_goffset_d);
		else
			apds9960_set_goffset_d(client, data->gesture_goffset_d + 128);

		//l offset
		if(ldata > APDS9960_PS_CALIBRATED_XTALK)
		{
			if(l_flag == 1)
			{
				if (data->gesture_goffset_l < 127)
				{
					data->gesture_goffset_l += n_l;
					if (data->gesture_goffset_l > 127)
						data->gesture_goffset_l = 127;						
				}
			}
			else
			{
				if(data->gesture_goffset_l >= n_l && data->gesture_goffset_l < 127)
				    data->gesture_goffset_l -= n_l;		
			}
		}
		else if (ldata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
			if(l_flag == 1)
			{
				if(data->gesture_goffset_l >= n_l && data->gesture_goffset_l < 127)
				    data->gesture_goffset_l -= n_l;
			}
	      	else
	      	{
	      		if (data->gesture_goffset_l < 127)
				{
					data->gesture_goffset_l += n_l;
					if(data->gesture_goffset_l > 127)
					{
						data->gesture_goffset_l = 127;
					}							
				}
	      	}
		}
		else
		{
			l_done = 1;
			printk("[APDS9960] l_done %d (%d) l_flag %d\r\n", data->gesture_goffset_l, i, l_flag);
		}

        if(l_flag == 1)
            apds9960_set_goffset_l(client, data->gesture_goffset_l);
		else
			apds9960_set_goffset_l(client, data->gesture_goffset_l + 128);
        
        //r offset
		if(rdata > APDS9960_PS_CALIBRATED_XTALK)
		{
			if(r_flag == 1)
			{
				if(data->gesture_goffset_r < 127)
				{
					data->gesture_goffset_r += n_r;
					if(data->gesture_goffset_r > 127)
					    data->gesture_goffset_r = 127;						
				}
			}
			else
			{
				if(data->gesture_goffset_r >= n_r && data->gesture_goffset_r < 127)
					data->gesture_goffset_r -= n_r;
			}
		}
		else if(rdata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
			if(r_flag == 1)
			{
				if(data->gesture_goffset_r >= n_r && data->gesture_goffset_r < 127)
				    data->gesture_goffset_r -= n_r;	
			}
			else
			{
				if(data->gesture_goffset_r < 127)
				{
					data->gesture_goffset_r += n_r;
					if(data->gesture_goffset_r > 127)
					    data->gesture_goffset_r = 127;						
				}
			}					
		}
		else
		{
			r_done = 1;
			printk("[APDS9960] r_done %d (%d) r_flag %d\r\n", data->gesture_goffset_r, i, r_flag);
		}

        if(r_flag == 1)
            apds9960_set_goffset_r(client, data->gesture_goffset_r);
		else
			apds9960_set_goffset_r(client, data->gesture_goffset_r + 128);


		if(u_done && d_done && l_done && r_done)
		{
			break;
		}
             
		apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GMODE);
		apds9960_set_enable(client, 0x00);
    }

    if(u_flag != 1)
       	data->gesture_goffset_u += 128;
	if(d_flag != 1)
	 	data->gesture_goffset_d += 128;
	if(l_flag != 1)
	 	data->gesture_goffset_l += 128;
	if(r_flag != 1)
	 	data->gesture_goffset_r += 128;

    if(data->gesture_goffset_u == 127 && data->gesture_goffset_d == 127 &&
	 	data->gesture_goffset_l == 127 && data->gesture_goffset_r == 127 &&
	 	data->gesture_poffset_ur == 127 && data->gesture_poffset_dl == 127)
    {
	    apds9960_set_goffset_u(client, u_goffset);
		apds9960_set_goffset_d(client, d_goffset);
		apds9960_set_goffset_l(client, l_goffset);
		apds9960_set_goffset_r(client, r_goffset);
		apds9960_set_poffset_ur(client, ur_poffset);
		apds9960_set_poffset_dl(client, dl_poffset);
        
		data->gesture_poffset_ur = ur_poffset;
		data->gesture_poffset_dl = dl_poffset;
		data->gesture_goffset_u = u_goffset;
		data->gesture_goffset_d = d_goffset;
		data->gesture_goffset_l = l_goffset;
		data->gesture_goffset_r = r_goffset;
		printk(KERN_ERR"[APDS9960]==============enable cal fail==============");
	}
    printk("[APDS9960]enable cal u = %d r = %d ur = %d\r\n", data->gesture_goffset_u, data->gesture_goffset_r, data->gesture_poffset_ur);
	printk("[APDS9960]enable cal d = %d l = %d dl = %d\r\n", data->gesture_goffset_d, data->gesture_goffset_l, data->gesture_poffset_dl);

    apds9960_set_enable(client, 0x0);
	apds9960_set_enable(client, enable);            //TODO:more check here
	apds9960_set_gconf1(client, gconf1);	
	apds9960_set_gthr_in(client, gthr_in);	
	apds9960_set_gthr_out(client, gthr_out);

    //clear flag after 100ms
    //atomic_set(&data->gesture_calibration, 0);
    queue_delayed_work(data->gs_workqueue, &data->gs_resume_delay, (HZ/10));    //100ms

    return 0;
}

static int apds9960_get_offset_diff(u8 offset1, u8 offset2)
{
    int diff = 0;

    if(offset1 < 128 && offset2 < 128)
        diff = abs(offset1 - offset2);
    else if(offset1 < 128 && offset2 > 128)
        diff = offset1 + (offset2 - 128);
    else if(offset1 > 128 && offset2 < 128)
        diff = (offset1 - 128) + offset2;
    else if(offset1 > 128 && offset2 > 128)
        diff = abs(offset1 - offset2);

    return diff;
}

//runtime calibration
static int apds9960_runtime_gesture_calibration(struct i2c_client *client, struct apds9960_data *data)
{
    int i, j;
    u8 i2c_read_data[4];
    int res;
    
    u8 u_done, d_done, l_done, r_done;
    u8 n_u, n_d, n_l, n_r;
    u8 n_ur, n_dl;
    u8 enable;
    u8 status;
    u8 gthr_in, gthr_out;
    u8 gconf1;
    u8 gstatus;
    u8 udata, ddata, ldata, rdata;
    u8 pdata;
    u8 ur_poffset, dl_poffset, u_goffset, d_goffset, l_goffset, r_goffset;
    u8 offset_ur,  offset_dl,  offset_u,  offset_d,  offset_l,  offset_r;
    u8 ur_flag, dl_flag, u_flag, d_flag, l_flag, r_flag;
    int ur_diff, dl_diff, u_diff, d_diff, l_diff, r_diff;

    printk("[APDS9960] apds9960_runtime_calibration\n");

    if(!test_bit(CMC_BIT_GS, (const volatile void *)&data->enable))
    {
       printk("[APDS9960] apds9960_enable_calibration exit");
    	 return 0;
    }
    //set up calibration flag
    atomic_set(&data->gesture_calibration, 1);

	// disable enable		
    res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration enable read err=%d\r\n", res);
    apds9960_set_enable(client, 0x00);
    	
	// force it into gesture mode and read data	
	res = apds9960_read_reg(APDS9960_GTHR_IN_REG, &gthr_in);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gthr_in read err=%d\r\n", res);
	apds9960_set_gthr_in(client, 0x00);		

	res = apds9960_read_reg(APDS9960_GTHR_OUT_REG, &gthr_out);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gthr_out read err=%d\r\n", res);
	apds9960_set_gthr_out(client, 0x00);	

	printk("[APDS9960] runtime gesture calibration gthr_in = %d  gthr_out = %d", gthr_in, gthr_out);
    
	res = apds9960_read_reg(APDS9960_GCONF1_REG, &gconf1);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gconf1 read err=%d\r\n", res);
	apds9960_set_gconf1(client, APDS9960_GFIFO_1_LEVEL);	
    
	res = apds9960_read_reg(APDS9960_GOFFSET_U_REG, &u_goffset);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration u_goffset read err=%d\r\n", res);

	res = apds9960_read_reg(APDS9960_GOFFSET_D_REG, &d_goffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration d_goffset read err=%d\r\n", res);
    
    res = apds9960_read_reg(APDS9960_GOFFSET_L_REG, &l_goffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration l_goffset read err=%d\r\n", res);
	
	res = apds9960_read_reg(APDS9960_GOFFSET_R_REG, &r_goffset);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration r_goffset read err=%d\r\n", res);
    
	res = apds9960_read_reg(APDS9960_POFFSET_UR_REG, &ur_poffset);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration ur_poffset read err=%d\r\n", res);

	res = apds9960_read_reg(APDS9960_POFFSET_DL_REG, &dl_poffset);
	if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration dl_poffset read err=%d\r\n", res);
   
    printk("[APDS9960]runtime gesture calibration goffset u:%d d:%d l:%d r:%d\n", 
                                                        u_goffset, d_goffset, l_goffset, r_goffset);
    printk("[APDS9960]runtime gesture calibration poffset ur:%d dl:%d\n", 
                                                        ur_poffset, dl_poffset);

    apds9960_set_goffset_u(client, 0x00);
    apds9960_set_goffset_d(client, 0x00);
	apds9960_set_goffset_l(client, 0x00);
	apds9960_set_goffset_r(client, 0x00);
	apds9960_set_poffset_ur(client, 0x00);
	apds9960_set_poffset_dl(client, 0x00);

    ur_flag = 0;
	dl_flag = 0;
	u_flag = 0;
	d_flag = 0;
	l_flag = 0;
	r_flag = 0;
    
	//calibration U&R
	apds9960_set_config2(client, 0x26);  // D and L are masked	
	apds9960_set_enable(client, 0x05);		
	
    for(j=0; j<10; j++)
	{
        res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration status read err=%d\r\n", res);
		
		if ((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
            break;

		mdelay(1);     //1ms	
	}

	res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration pdata read err=%d\r\n", res); 
	printk("runtime calibration UR before: pdata = %d\n", pdata);
    
	if(pdata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		ur_flag = 1;

	n_ur = 128;
	offset_ur = 0;
	for(i=0; i<7; i++)
	{
        n_ur /= 2;
		apds9960_set_enable(client, 0x05);
        
		for(j=0; j<10; j++)
		{
            mdelay(1);   // 1ms	

			res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
            if(res < 0)
                printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration status read err=%d\r\n", res);
            
            if((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
                break;
		}

		res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration pdata read err=%d\r\n", res);
		
        if(debug)
            printk("pdata = %d\r\n", pdata);
        
		if(pdata > APDS9960_PS_CALIBRATED_XTALK)
		{
			if(ur_flag == 1)
			{
                if(offset_ur < 127)
				{
					offset_ur += n_ur;
					if(offset_ur > 127)
						offset_ur = 127;		
				}
            }
            else
            {
                if(offset_ur >= n_ur && offset_ur < 127)
                    offset_ur -= n_ur;						
            }
		}
        else if(pdata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{ 
 			if(ur_flag == 1)
 			{
                if(offset_ur >=n_ur && offset_ur < 127)
                    offset_ur -= n_ur;		
 			}
			else
			{	
				if(offset_ur < 127)
				{
					offset_ur += n_ur;
					if(offset_ur > 127)
						offset_ur = 127;			
				}
			}					
		}
		else
		{
			printk("[APDS9960] Done poffset-ur=%d (%d) ur_flag %d\r\n", offset_ur, i, ur_flag);
			break;
		}
		
        if(ur_flag == 1)
            apds9960_set_poffset_ur(client, offset_ur);
        else
	       	apds9960_set_poffset_ur(client, offset_ur + 128);
        
		apds9960_set_enable(client, 0x00);	
	}
       
    if(ur_flag != 1)
       	offset_ur += 128;
    
    //calibration D&L
	apds9960_set_enable(client, 0x00);		
	apds9960_set_config2(client, 0x29);  // U and R are masked	
	apds9960_set_enable(client, 0x05);		
      
    for(j=0; j<10; j++)
	{
        res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration status read err=%d\r\n", res);
		
        if((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
            break;
        
        mdelay(1);       // 1ms	
	}

	res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
    if(res < 0)
        printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration pdata read err=%d\r\n", res);
	printk("runtime calibration DL before: pdata = %d\n", pdata);
    
	if(pdata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
        dl_flag = 1;
    
	n_dl = 128;
	offset_dl = 0;
	for(i=0; i<7; i++)
	{
		n_dl /= 2;
		apds9960_set_enable(client, 0x05);
        
		for(j=0; j<10; j++)
		{
			res = apds9960_read_reg(APDS9960_STATUS_REG, &status);
            if(res < 0)
                printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration status read err=%d\r\n", res);

            if((status & APDS9960_STATUS_PVALID) == APDS9960_STATUS_PVALID) 
                break;

			mdelay(1);   // 1ms	
		}

		res = apds9960_read_reg(APDS9960_PDATA_REG, &pdata);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration pdata read err=%d\r\n", res);
		
        if(debug)
            printk("pdata = %d\r\n", pdata);
        
		if(pdata > APDS9960_PS_CALIBRATED_XTALK)
		{
            if(dl_flag == 1)
            {
                if(offset_dl < 127)
				{
					offset_dl += n_dl;
					if(offset_dl > 127)
                        offset_dl = 127;		
				}
		    }
			else
			{
                if(offset_dl >= n_dl && offset_dl < 127)
                    offset_dl -= n_dl;		
			}
		}
    	else if(pdata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
            if(dl_flag == 1)
            {
                if(offset_dl >= n_dl && offset_dl < 127)
				    offset_dl -= n_dl;	
		    }
			else
			{
                if(offset_dl < 127)
				{
					offset_dl += n_dl;
					if (offset_dl > 127)
					    offset_dl = 127;						
				}
			}			
		}
		else
		{
			printk("[APDS9960] Done poffset-dl =%d (%d) dl_flag %d\r\n", offset_dl, i, dl_flag);
            break;
		}
        
		if(dl_flag == 1)
			apds9960_set_poffset_dl(client, offset_dl);
		else
			apds9960_set_poffset_dl(client, offset_dl + 128);	
        
		apds9960_set_enable(client, 0x00);		
    }
    
    if(dl_flag != 1)
       	offset_dl += 128;
    
    //calibration gesture offset
	apds9960_set_enable(client, 0x00);		
	apds9960_set_config2(client, 0x0);
	apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GMODE);	
	apds9960_set_enable(client, 0x41);

	n_u = 128;
	n_d = 128;
	n_l = 128;
	n_r = 128;
	offset_u = 0;
	offset_d = 0;
	offset_l = 0;
	offset_r = 0;
	u_done = 0;
	d_done = 0;
	l_done = 0;
	r_done = 0;
    
	for(j=0; j<10; j++)
	{
		res = apds9960_read_reg(APDS9960_GSTATUS_REG, &gstatus);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gstatus read err=%d\r\n", res);
        
		if((gstatus & APDS9960_GVALID) == APDS9960_GVALID) 
				break;

		mdelay(1);   // 1ms	
	}
    
    res = apds9960_i2c_read_block(i2c_read_data, APDS9960_GFIFO0_REG, 4);
    if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gfifo read err=%d\r\n", res);

	udata = i2c_read_data[0];
	ddata = i2c_read_data[1];
	ldata = i2c_read_data[2];
	rdata = i2c_read_data[3];
	printk("runtime calibration before: gdata_u=%d, gdata_d=%d\r\n", i2c_read_data[0], i2c_read_data[1]);
	printk("runtime calibration before: gdata_l=%d, gdata_r=%d\r\n", i2c_read_data[2], i2c_read_data[3]);
    
	if(udata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		u_flag = 1;
	if(ddata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		d_flag = 1;
	if(ldata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		l_flag = 1;
	if(rdata > APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		r_flag = 1;

	for(i = 0; i <7 ;i++)
	{	
		n_u /= 2;
		n_d /= 2;
		n_l /= 2;
		n_r /= 2;
		apds9960_set_enable(client, 0x41);
        
		for(j=0; j<10; j++)
		{
			res = apds9960_read_reg(APDS9960_GSTATUS_REG, &gstatus);
			if(res < 0)
                printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gstatus read err=%d\r\n", res);
            
			if((gstatus & APDS9960_GVALID) == APDS9960_GVALID) 
				break;
			
            mdelay(1);   // 1ms	
		}
        
		res = apds9960_i2c_read_block(i2c_read_data, APDS9960_GFIFO0_REG, 4);
        if(res < 0)
            printk(KERN_ERR"[APDS9960]apds9960_runtime_gesture_calibration gfifo read err=%d\r\n", res);
        
		udata = i2c_read_data[0];
		ddata = i2c_read_data[1];
		ldata = i2c_read_data[2];
		rdata = i2c_read_data[3];
		printk("gdata_u=%d, gdata_d=%d\r\n", i2c_read_data[0], i2c_read_data[1]);
		printk("gdata_l=%d, gdata_r=%d\r\n", i2c_read_data[2], i2c_read_data[3]);

		//u offset
		if(udata > APDS9960_PS_CALIBRATED_XTALK)
		{
            if(u_flag == 1)
			{
				if(offset_u < 127)
				{
					offset_u += n_u;
					if (offset_u > 127)
						offset_u = 127;			
				}
			}
			else
			{
                if(offset_u >= n_u && offset_u < 127)
                    offset_u -= n_u;		
			}
		}
		else if(udata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
            if(u_flag == 1)
            {
                if(offset_u >= n_u && offset_u < 127)
					offset_u -= n_u;	
            }
	     	else
	     	{
                if(offset_u < 127)
				{
					offset_u += n_u;
					if (offset_u > 127)
						offset_u = 127;			
				}
            }
		}
		else
		{
			u_done = 1;
			printk("[APDS9960] u_done %d (%d) u_flag %d\r\n", offset_u, i, u_flag);
		}
        
        if(u_flag == 1)
            apds9960_set_goffset_u(client, offset_u);
		else
			apds9960_set_goffset_u(client, offset_u + 128);

		//d offset
        if(ddata > APDS9960_PS_CALIBRATED_XTALK)
		{
            if(d_flag == 1)
            {
                if (offset_d < 127)
				{
					offset_d += n_d;
					if(offset_d > 127)
						offset_d = 127;							
				}
		    }
			else
			{
				if(offset_d >= n_u && offset_d < 127)
					offset_d -= n_d;	
			}
		}
		else if(ddata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{ 
			if(d_flag == 1)
			{
				if(offset_d >= n_u && offset_d < 127)
					offset_d -= n_d;
			}
			else
			{
				if (offset_d < 127)
				{
					offset_d += n_d;
					if(offset_d > 127)
						offset_d = 127;						
				}
			}
	      								
		}
		else
		{
			d_done = 1;
			printk("[APDS9960] d_done %d (%d) d_flag %d\r\n", offset_d, i, d_flag);
		}

		if(d_flag == 1)
			apds9960_set_goffset_d(client, offset_d);
		else
			apds9960_set_goffset_d(client, offset_d + 128);

		//l offset
		if(ldata > APDS9960_PS_CALIBRATED_XTALK)
		{
			if(l_flag == 1)
			{
				if(offset_l < 127)
				{
					offset_l += n_l;
					if (offset_l > 127)
					{
						offset_l = 127;
					}							
				}
			}
			else
			{
				if(offset_l >= n_l && offset_l < 127)
					offset_l -= n_l;		
			}
		}
		else if(ldata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
			if(l_flag == 1)
			{
				if(offset_l >= n_l && offset_l < 127)
					offset_l -= n_l;
			}
	      	else
	      	{
	      		if (offset_l < 127)
				{
					offset_l += n_l;
					if (offset_l > 127)
						offset_l = 127;						
				}
	      	}
		}
		else
		{
			l_done = 1;
			printk("[APDS9960] l_done %d (%d) l_flag %d\r\n", offset_l, i, l_flag);
		}

        if(l_flag == 1)
            apds9960_set_goffset_l(client, offset_l);
		else
			apds9960_set_goffset_l(client, offset_l + 128);

        //r offset
		if(rdata > APDS9960_PS_CALIBRATED_XTALK)
		{
			if(r_flag == 1)
			{
				if(offset_r < 127)
				{
                    offset_r += n_r;
					if(offset_r > 127)
						offset_r = 127;						
				}
			}
			else
			{
				if(offset_r >= n_r && offset_r < 127)
					offset_r -= n_r;
			}
			
		}
		else if (rdata < APDS9960_PS_CALIBRATED_XTALK_BASELINE)
		{
			if(r_flag == 1)
			{
				if(offset_r >= n_r && offset_r < 127)
					offset_r -= n_r;	
			}
			else
			{
				if(offset_r < 127)
				{
					offset_r += n_r;
					if(offset_r > 127)
						offset_r = 127;						
				}
			}
	     	 						
		}
		else
		{
			r_done = 1;
			printk("[APDS9960] r_done %d (%d) r_flag %d\r\n", offset_r, i, r_flag);
		}
        
        if(r_flag == 1)
            apds9960_set_goffset_r(client, offset_r);
		else
			apds9960_set_goffset_r(client, offset_r + 128);

		if(u_done && d_done && l_done && r_done)
		{
            printk("[APDS9960] runtime calibration i = %d\r\n", i);
			break;
		}
             
		apds9960_set_gctrl(client, APDS9960_GFIFO_CLR | APDS9960_GMODE);
		apds9960_set_enable(client, 0x00);
    }

    if(u_flag != 1)
       	offset_u += 128;
    if(d_flag != 1)
	 	offset_d += 128;
    if(l_flag != 1)
	 	offset_l += 128;
    if(r_flag != 1)
	 	offset_r += 128;
    
    ur_diff = apds9960_get_offset_diff(data->gesture_poffset_ur, offset_ur);
    dl_diff = apds9960_get_offset_diff(data->gesture_poffset_dl, offset_dl);
    u_diff  = apds9960_get_offset_diff(data->gesture_goffset_u,  offset_u);
    d_diff  = apds9960_get_offset_diff(data->gesture_goffset_d,  offset_d);
    l_diff  = apds9960_get_offset_diff(data->gesture_goffset_l,  offset_l);
    r_diff  = apds9960_get_offset_diff(data->gesture_goffset_r,  offset_r); 
	 
    if(debug)
    {
        printk("[APDS9960] runtime u: cal = %d raw = %d\r\n",  offset_u,  data->gesture_goffset_u);
        printk("[APDS9960] runtime d: cal = %d raw = %d\r\n",  offset_d,  data->gesture_goffset_d);
	    printk("[APDS9960] runtime l: cal = %d raw = %d\r\n",  offset_l,  data->gesture_goffset_l);
	    printk("[APDS9960] runtime r: cal = %d raw = %d\r\n",  offset_r,  data->gesture_goffset_r);
	    printk("[APDS9960] runtime ur: cal = %d raw = %d\r\n", offset_ur, data->gesture_poffset_ur);
	    printk("[APDS9960] runtime dl: cal = %d raw = %d\r\n", offset_dl, data->gesture_poffset_dl);
    }

    //check diff
    if(ur_diff< 20 && dl_diff < 20 &&
	  	u_diff < 20 && d_diff < 20 &&
	  	l_diff < 20 && r_diff < 20)
    {  
        printk("[APDS9960] runtime calbration OK\r\n");
      	
        data->gesture_poffset_ur = offset_ur;
		data->gesture_poffset_dl = offset_dl;
		data->gesture_goffset_u  = offset_u;
		data->gesture_goffset_d  = offset_d;
		data->gesture_goffset_l  = offset_l;
		data->gesture_goffset_r  = offset_r;
    }
    else
    {
        printk("[APDS9960] runtime calbration fail !!!\r\n");
        
       	apds9960_set_poffset_ur(client, data->gesture_poffset_ur);
		apds9960_set_poffset_dl(client, data->gesture_poffset_dl);
		apds9960_set_goffset_u(client,  data->gesture_goffset_u);
		apds9960_set_goffset_d(client,  data->gesture_goffset_d);
		apds9960_set_goffset_l(client,  data->gesture_goffset_l);
		apds9960_set_goffset_r(client,  data->gesture_goffset_r);
    }

	apds9960_set_enable(client, enable);
	apds9960_set_gconf1(client, gconf1);	
	apds9960_set_gthr_in(client, gthr_in);	
	apds9960_set_gthr_out(client, gthr_out);

    //clear flag after 100ms
    //atomic_set(&data->gesture_calibration, 0);
    queue_delayed_work(data->gs_workqueue, &data->gs_resume_delay, (HZ/10));    //100ms

    return 0;
}

//used for check if can do runtime calibration
static int apds9960_check_runtime_gesture_calibration
(
    struct i2c_client *client, 
    struct apds9960_data *data,
    unsigned char* gfifo_data,
    int gfifo_num
)
{
    u8 avg_data[4] = {0};
    u8 min_data[4] = {0xff,0xff,0xff,0xff};
    u8 max_data[4] = {0};
    int sum_data[4] = {0};
    int i = 0;
    int count = 0;
    
    const int data_offset = 10;

    if(gfifo_num < 32)
        return -1;

    for(i=0; i <gfifo_num; i+=4)
    {
        sum_data[0] += gfifo_data[i];
		sum_data[1] += gfifo_data[i+1];
		sum_data[2] += gfifo_data[i+2];
		sum_data[3] += gfifo_data[i+3];
        
		min_data[0] = (min_data[0] > gfifo_data[i])   ? gfifo_data[i]   : min_data[0];
		min_data[1] = (min_data[1] > gfifo_data[i+1]) ? gfifo_data[i+1] : min_data[1];
		min_data[2] = (min_data[2] > gfifo_data[i+2]) ? gfifo_data[i+2] : min_data[2];
		min_data[3] = (min_data[3] > gfifo_data[i+3]) ? gfifo_data[i+3] : min_data[3];
        
		max_data[0] = (max_data[0] < gfifo_data[i])   ? gfifo_data[i]   : max_data[0];
		max_data[1] = (max_data[1] < gfifo_data[i+1]) ? gfifo_data[i+1] : max_data[1];
		max_data[2] = (max_data[2] < gfifo_data[i+2]) ? gfifo_data[i+2] : max_data[2];
		max_data[3] = (max_data[3] < gfifo_data[i+3]) ? gfifo_data[i+3] : max_data[3];
		
        //printk("check runtime calibration %d gfifo read  U = %d\r\n", i,   raw_data[i+0]);
		//printk("check runtime calibration %d gfifo read  D = %d\r\n", i+1, raw_data[i+1]);
		//printk("check runtime calibration %d gfifo read  R = %d\r\n", i+2, raw_data[i+2]);
		//printk("check runtime calibration %d gfifo read  L = %d\r\n", i+3, raw_data[i+3]);
    }
    
    if(debug)
    {
        printk("[APDS9960] check runtime calibration data0 = %d data1 %d\r\n", sum_data[0], sum_data[1]);
        printk("[APDS9960] check runtime calibration data2 = %d data3 %d\r\n", sum_data[2], sum_data[3]);
    }

    for(i=0; i < 4; i++)
    {
        sum_data[i] = sum_data[i] - min_data[i] - max_data[i];
	    avg_data[i] = sum_data[i] / (gfifo_num/4 -2);

        printk("[APDS9960] check runtime calibration avg=%d min=%d max=%d\r\n", avg_data[i], min_data[i], max_data[i]);
        if((max_data[i] < avg_data[i] + data_offset) && (min_data[i] + data_offset > avg_data[i]))
        {
            count += 1;
        }
    }

    if(count == 4)     //fifo data is steady
    {
        printk("[APDS9960] offset ur=%d dl=%d\n", data->gesture_poffset_ur, data->gesture_poffset_dl);
        printk("[APDS9960] offset u=%d d=%d l=%d r=%d\n", data->gesture_goffset_u, data->gesture_goffset_d,
                                                          data->gesture_goffset_l, data->gesture_goffset_r);
        
        apds9960_runtime_gesture_calibration(client, data);
    }

    return 0;
}

#if 0
static int apds9960_calibration(struct i2c_client *client, int val)
#endif

/*----------------------------------------------------------------------------*/
static int apds9960_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9960_i2c_client;

	if (!file->private_data)
	{
		printk("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int apds9960_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/******************************************************************************
//NOTE: used for engineer mode. 
//check with vivo/source/common/external/mtk_sensors_engineermode_interface/vivo_sensors/src/als_ps.c
*******************************************************************************/
static long apds9960_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
    struct apds9960_data *data;
    struct i2c_client *client;
    u32 databuf[2];
    unsigned int ps_status;
   	int enable;
    int ret = 0;
    
    if (arg == 0) 
        return -1;

    if(apds9960_i2c_client == NULL) {
		printk("apds9960_ps_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

    client = apds9960_i2c_client;   
    data = i2c_get_clientdata(apds9960_i2c_client);

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9960_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}
            data->eng_ps_sensor = enable;
            if(enable)
			    ret = apds9960_enable_ps_sensor(client, enable);
            else if(!test_bit(CMC_BIT_PS, (const volatile void *)&data->enable))
                ret = apds9960_enable_ps_sensor(client, enable);
            mdelay(30);   //30ms
			if(ret < 0)
				return ret;
		    break;

		case ALSPS_GET_PS_MODE:
			if (copy_to_user((void __user *)arg, (const void *)&data->enable_ps_sensor, sizeof(data->enable_ps_sensor))) {
				printk("apds9960_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		    break;

		case ALSPS_GET_PS_DATA:    
            ret = apds9960_read_ps(client, (unsigned char *)&data->ps_data);
            if(ret < 0)
                return -EFAULT;
			ps_status = apds9960_get_ps_value(data, data->ps_data);
			if (copy_to_user((void __user *)arg, (const void *)&ps_status, sizeof(unsigned int))) {
				printk("apds9960_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		    break;

		case ALSPS_GET_PS_RAW_DATA:    
            ret = apds9960_read_ps(client, (unsigned char *)&data->ps_data);
            if(ret < 0)
                return -EFAULT;
            apds9960_get_ps_value(data, data->ps_data);
			if (copy_to_user((void __user *)arg, (const void *)&data->ps_data, sizeof(data->ps_data))) {
				printk("apds9960_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;     

		case ALSPS_SET_ALS_MODE:
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9960_ioctl: copy_from_user failed\n");
				return -EFAULT;
			} 
            data->eng_als_sensor = enable;
            if(enable)
			    ret = apds9960_enable_als_sensor(client, enable);
            else if(!test_bit(CMC_BIT_ALS, (const volatile void *)&data->enable))
                ret = apds9960_enable_als_sensor(client, enable);
			if(ret < 0)
				return ret; 
			break;

		case ALSPS_GET_ALS_MODE:
			if (copy_to_user((void __user *)arg, (const void *)&data->enable_als_sensor, sizeof(data->enable_als_sensor))) {
				printk("apds9960_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ALSPS_GET_ALS_DATA:        //report lux 
			ret = read_als(client, databuf);
            if(ret < 0)
                return -EFAULT;
            if (copy_to_user((void __user *)arg, (const void *)&databuf[0], sizeof(u32))) {
				printk("apds9960_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:    //report lux  
			ret = read_als(client, databuf);
            if(ret < 0)
                return -EFAULT;
            if (copy_to_user((void __user *)arg, (const void *)&databuf[0], sizeof(u32))) {
				printk("apds9960_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ALSPS_IOCTL_GET_CALI:
			break;

		default:
			printk("%s not supported = 0x%04x", __FUNCTION__, cmd);
			ret = -ENOIOCTLCMD;
			break;
	}
    
	return ret;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations apds9960_fops = {
	.owner = THIS_MODULE,
	.open = apds9960_open,
	.release = apds9960_release,
	.unlocked_ioctl = apds9960_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice apds9960_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "psgs",
	.fops = &apds9960_fops,
};

static int apds9960_als_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9960_i2c_client;

	if (!file->private_data)
	{
		printk("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int apds9960_als_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
//NOTE: this function is same with apds9960_ioctl 
static long apds9960_als_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{
    struct apds9960_data *data;
    struct i2c_client *client;
    u32 databuf[2];
    unsigned int ps_status;
   	int enable;
    int ret = 0;
    
    if (arg == 0) 
        return -1;

    if(apds9960_i2c_client == NULL) {
		printk("apds9960_ps_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

    client = apds9960_i2c_client;   
    data = i2c_get_clientdata(apds9960_i2c_client);

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9960_als_ioctl: copy_from_user failed\n");
				return -EFAULT;
			}
            data->eng_ps_sensor = enable;
            if(enable)
			    ret = apds9960_enable_ps_sensor(client, enable);
            else if(!test_bit(CMC_BIT_PS, (const volatile void *)&data->enable))
                ret = apds9960_enable_ps_sensor(client, enable);
            mdelay(30);   //30ms
			if(ret < 0)
				return ret;
		    break;

		case ALSPS_GET_PS_MODE:
			if (copy_to_user((void __user *)arg, (const void *)&data->enable_ps_sensor, sizeof(data->enable_ps_sensor))) {
				printk("apds9960_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		    break;

		case ALSPS_GET_PS_DATA:    
            ret = apds9960_read_ps(client, (unsigned char *)&data->ps_data);
            if(ret < 0)
                return -EFAULT;
			ps_status = apds9960_get_ps_value(data,data->ps_data);
			if (copy_to_user((void __user *)arg, (const void *)&ps_status, sizeof(unsigned int))) {
				printk("apds9960_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
		    break;

		case ALSPS_GET_PS_RAW_DATA:    
            ret = apds9960_read_ps(client, (unsigned char *)&data->ps_data);
            if(ret < 0)
                return -EFAULT;
            apds9960_get_ps_value(data,data->ps_data);
			if (copy_to_user((void __user *)arg, (const void *)&data->ps_data, sizeof(data->ps_data))) {
				printk("apds9960_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;     

		case ALSPS_SET_ALS_MODE:
			if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
				printk("apds9960_als_ioctl: copy_from_user failed\n");
				return -EFAULT;
			} 
            data->eng_als_sensor = enable;
            if(enable)
            	{
			 ret = apds9960_enable_als_sensor(client, enable);
		       msleep(50);
            	}
            else if(!test_bit(CMC_BIT_ALS, (const volatile void *)&data->enable))
                ret = apds9960_enable_als_sensor(client, enable);
			if(ret < 0)
				return ret; 
			break;

		case ALSPS_GET_ALS_MODE:
			if (copy_to_user((void __user *)arg, (const void *)&data->enable_als_sensor, sizeof(data->enable_als_sensor))) {
				printk("apds9960_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ALSPS_GET_ALS_DATA:        //report lux 
			ret = read_als(client, databuf);
            if(ret < 0)
                return -EFAULT;
            if (copy_to_user((void __user *)arg, (const void *)databuf, sizeof(u32))) {
				printk("apds9960_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:    //report lux  
			ret = read_als(client, databuf);
			if(ret >= 0)
				ret = 0;
            if(ret < 0)
                return -EFAULT;
            if (copy_to_user((void __user *)arg, (const void *)databuf, sizeof(u32))) {
				printk("apds9960_als_ioctl: copy_to_user failed\n");
				return -EFAULT;
			}
			break;

		case ALSPS_IOCTL_GET_CALI:
			break;

		default:
			printk("%s not supported = 0x%04x", __FUNCTION__, cmd);
			ret = -ENOIOCTLCMD;
			break;
	}
    
	return ret;    
}

static struct file_operations apds9960_als_fops = {
	.owner = THIS_MODULE,
	.open = apds9960_als_open,
	.release = apds9960_als_release,
	.unlocked_ioctl = apds9960_als_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice apds9960_als_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als",
	.fops = &apds9960_als_fops,
};


/*----------------------------------------------------------------------------*/
static int apds9960_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{

	struct apds9960_data *data = i2c_get_clientdata(client);

	printk("apds9960_suspend\n");

	data->suspended = 1;
	data->enable_suspended_value = data->enable;
	
    //disable poll work
    if(data->enable_ps_sensor || data->enable_als_sensor)
        cancel_delayed_work(&data->ps_report_work);     //stop report timer here
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9960_i2c_resume(struct i2c_client *client)
{
	struct apds9960_data *data = i2c_get_clientdata(client);

	printk("apds9960_resume (enable=%d)\n", data->enable_suspended_value);
    
    //enable poll work
    if(data->enable_ps_sensor || data->enable_als_sensor)
        queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(10));     //10ms
    
	data->suspended = 0;
    
	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void apds9960_early_suspend(struct early_suspend *h)
{ 
    struct apds9960_data *obj = container_of(h, struct apds9960_data, early_drv);
	u8 buf = 0;
	u8 res = 0;
	
    printk("apds9960_early_suspend\n");

    /*res = apds9960_read_reg(APDS9960_ENABLE_REG, &buf);
	if(res == 1)
	{
		buf = buf & 0xdf;
		apds9960_set_enable(obj->client, buf);
	}*/

	if(test_bit(CMC_BIT_ALS, (const volatile void *)&obj->enable))
		apds9960_enable_als_sensor(obj->client, 0); 

	if(test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable))
		apds9960_enable_gesture_sensor(obj->client, 0); 
}

/*----------------------------------------------------------------------------*/
static void apds9960_late_resume(struct early_suspend *h)
{
    struct apds9960_data *obj = container_of(h, struct apds9960_data, early_drv);
	
    printk("apds9960_late_resume\n");  
    
	if(test_bit(CMC_BIT_ALS, (const volatile void *)&obj->enable))
		apds9960_enable_als_sensor(obj->client,1);

	if(test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable))
		apds9960_enable_gesture_sensor(obj->client, 1); 
}
#elif defined(CONFIG_FB)
static int apds9960_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
	struct apds9960_data *obj = container_of(self, struct apds9960_data, fb_notif);
	
	//printk("[APDS9960]apds9960_fb_notifier_callback %d\n", evdata->data);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && obj) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
        {
			if(test_bit(CMC_BIT_ALS, (const volatile void *)&obj->enable))
		        apds9960_enable_als_sensor(obj->client,1);

	    //    if(test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable))
		//        apds9960_enable_gesture_sensor(obj->client, 1); 
        }
        else if (*blank == FB_BLANK_POWERDOWN)
        {
            if(test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable))
		        apds9960_enable_gesture_sensor(obj->client, 0); 

			if(test_bit(CMC_BIT_ALS, (const volatile void *)&obj->enable))
                apds9960_enable_als_sensor(obj->client, 0); 
        }
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
//int apds9960_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
//{
    //return 0;
//}

/************************* SysFS support **************************/
static void apds9960_dumpReg(struct i2c_client *client)
{ 
    int i=0;
	u8 addr = 0x80;  
	u8 regdata=0;
    
	for(i=0; i<128 ; i++)  
	{   
        apds9960_read_reg(addr,&regdata);
        printk("[APDS9960] Reg addr=%x regdata=%x\n",addr,regdata);	
        addr++;	  
	}
}

static ssize_t sensor_readreg_show(struct device_driver *driver, char *buf)
{    
	struct i2c_client *client = apds9960_i2c_client;   
	apds9960_dumpReg(client);   
	return 1;
}

static ssize_t sensors_writereg_store(struct device_driver *driver, const char *buf, size_t count)
{	
	int addr;
	u8 value;	
	int tem;	
	//struct i2c_client *client = apds9960_i2c_client;	
    
	tem = simple_strtoul(buf, NULL, 16);	
	addr = (tem&0xff00)>>8;	
	value = tem&0x00ff;	
	if(!apds9960_write_reg(addr,value))
	{      
		printk("apds9960 reg==>[%x]/[%x]\n",addr,value);	
	}	
	return count;
}

static ssize_t store_debug(struct device_driver *driver, const char *buf, size_t count)
{
	//struct i2c_client *client = apds9960_i2c_client;
	//struct apds9960_data*obj=i2c_get_clientdata(client);	 
	int num =0;

    num=simple_strtoul(buf, NULL, 10);
	if(num==0)
		debug=0;
	else if(num==1)
		debug=1;
	else
		printk(KERN_INFO "wrong arg of debug %d\n",num);
    
	return count;
}

static ssize_t show_debug(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "%d\n", debug);
}


static ssize_t store_ps_delay(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);
	int num =0;

    num=simple_strtoul(buf, NULL, 10);
	if(num > 0){
		if(NULL != obj){
			printk(KERN_INFO "update ps_delay from %d to %d\n",atomic_read(&obj->ps_poll_delay),num);
			atomic_set(&obj->ps_poll_delay,num);
		}
		else {
			printk(KERN_INFO "Can't update ps_delay as obj is NULL.\n");
			return 0;
		}
	}
	else {
		printk(KERN_INFO "wrong arg of ps_delay %d\n",num);
		return 0;
	}
    
	return count;
}

static ssize_t show_ps_delay(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);
	if( NULL != obj ){
		return sprintf(buf, "%d\n", atomic_read(&obj->ps_poll_delay));
	}
	else {
		return 0;
	}
}


static ssize_t show_deviceid(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	int deviceid = 0;
	if(NULL == client)
	{
		printk("i2c client is null!!\n");
		return 0;
	}
	deviceid = chip_id;
	return sprintf(buf, "%d\n", deviceid);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_ps_para_index(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);	 
	int num =0;
      
    num=simple_strtoul(buf, NULL, 10);
    set_ps_para_num(num);
    apds9960_ps_select_para(obj);
	
    return count;
}

static ssize_t show_ps_para_index(struct device_driver *driver, char *buf)
{
	return sprintf(buf, "%d\n", ps_para_num);
}

static ssize_t show_als_para_index(struct device_driver *driver, char *buf)
{
	//struct als_para *para;
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);
    //int base;

    apds9960_als_select_para(obj);

	if(NULL == obj)
	{
		printk("i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", als_para_num);
}

static ssize_t store_als_para_index(struct device_driver *driver, const char *buf, size_t count)
{
	//do nothing
	//struct i2c_client *client = apds9960_i2c_client;
	//struct apds9960_data *obj=i2c_get_clientdata(client);	 
	int num =0;
    
    num=simple_strtoul(buf, NULL, 10);
    set_als_para_num(num);

	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_ps_base_value(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);

    apds9960_ps_select_para(obj);
	if(NULL == obj)
	{
		printk("i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->ps_base_value));
}

static ssize_t store_ps_base_value(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);
	u16 thd_value = (u16)simple_strtoul(buf, NULL, 10);
	//unsigned char databuf[2];
	//int res;
    
	if( obj != NULL && client != NULL && thd_value >= 0 )
    {
        const struct ps_para *para;
    
        //check the value
        para = &apds9960_ps_para[ps_para_num];
        if(!(thd_value >= para->base_value_min && thd_value <= para->base_value_max))    //if the value is error, do nothing.
            return count;
        
        atomic_set(&obj->ps_base_value, thd_value);
        apds9960_ps_select_para(obj);     //reselect the threshold agian 

        printk("%s finnal ps_base_value=%d\n", __func__, atomic_read(&obj->ps_base_value));
	}
	else 
    {
		printk("set adps9960 ps threshold FAIL!!    mean to set threshold value=%d\n", thd_value);
	}
    
	return count;
}

static ssize_t store_prox_status(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);	 
	int val_tmp = 0;
	
	if(client == NULL )
	{
		printk("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	val_tmp = simple_strtoul(buf, NULL, 10);
	if ( val_tmp == 0 || val_tmp == 1 )
    {
        atomic_set(&obj->sys_prox_status, val_tmp);
		printk(KERN_ERR "DDDDDDD set sys_prox_status=%d\n",val_tmp);
		return count;
	}
	else 
    {
		goto error_fs;
	}
    
error_fs:
    printk("store_prox_status wrong value!value=%d\n",val_tmp);
	return count;
}

static ssize_t show_prox_status(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk("i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->sys_prox_status));
}

static ssize_t apds9960_show_calibration(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_gesture_sensor);
}

static ssize_t apds9960_store_calibration(struct device_driver *driver, const char *buf, size_t count)
{
    struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *obj = i2c_get_clientdata(client);	 
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: calibration (%ld)\n", __func__, val);
	
	if ((val != APDS_ALS_CALIBRATION) && (val != APDS_PS_CALIBRATION) && (val != APDS_PS_GESTURE_CALIBRATION))
	{
		printk("**%s: store invalid valeu=%ld\n", __func__, val);
		return count;
	}

	apds9960_enable_gesture_calibration(client, obj);
    
	return count;
}
static ssize_t apds9960_show_proximity_enable(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9960_store_proximity_enable(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable ps senosr (%ld)\n", __func__, val);
	
	if ((val != APDS_DISABLE_PS) && (val != APDS_ENABLE_PS)) {
		printk("**%s:store invalid value=%ld\n", __func__, val);
		return count;
	}

	apds9960_enable_ps_sensor(client, val);	
	
	return count;
}

static ssize_t apds9960_show_light_enable(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds9960_store_light_enable(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable als sensor (%ld)\n", __func__, val);
	
	if ((val != APDS_DISABLE_ALS) && (val != APDS_ENABLE_ALS_WITH_INT) && (val != APDS_ENABLE_ALS_NO_INT))
	{
		printk("**%s: store invalid valeu=%ld\n", __func__, val);
		return count;
	}

	apds9960_enable_als_sensor(client, val); 
	
	return count;
}

static ssize_t apds9960_show_gesture_enable(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_gesture_sensor);
}

static ssize_t apds9960_store_gesture_enable(struct device_driver *driver, const char *buf, size_t count)
{
	struct i2c_client *client = apds9960_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable gesture sensor (%ld)\n", __func__, val);
	
	if ((val != APDS_DISABLE_GESTURE) && (val != APDS_ENABLE_GESTURE))
	{
		printk("**%s: store invalid valeu=%ld\n", __func__, val);
		return count;
	}

	apds9960_enable_gesture_sensor(client, val); 
	
	return count;
}

static ssize_t apds9960_store_lcd_state(struct device_driver *driver,const char *buf,size_t count)
{
      struct i2c_client *client = apds9960_i2c_client;
	struct apds9960_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	if(val == 1)
	{
		atomic_set(&data->ps_poll_delay, APDS9960_PS_DEFAULT_POLL_DELAY);
		printk("%s val = %ld\r\n",__FUNCTION__,val);
	}
	else
	{
	    atomic_set(&data->ps_poll_delay, 200);
	    printk("%s val = %ld\r\n",__FUNCTION__,val);
	}
	return count;
}

static DRIVER_ATTR(calibration, 0644, apds9960_show_calibration, apds9960_store_calibration);
static DRIVER_ATTR(sys_prox_status,   S_IWUSR | S_IRUGO, show_prox_status,store_prox_status);
static DRIVER_ATTR(get_reg, 0644, sensor_readreg_show, sensors_writereg_store);
static DRIVER_ATTR(debug,   S_IWUSR | S_IRUGO, show_debug,store_debug);
static DRIVER_ATTR(ps_para_index,   S_IWUSR | S_IRUGO, show_ps_para_index, store_ps_para_index);
static DRIVER_ATTR(als_para_index,   S_IWUSR | S_IRUGO, show_als_para_index, store_als_para_index);
static DRIVER_ATTR(ps_base_value,   S_IWUSR | S_IRUGO, show_ps_base_value, store_ps_base_value);
static DRIVER_ATTR(deviceid,   S_IWUSR | S_IRUGO, show_deviceid,      NULL);
static DRIVER_ATTR(gesture_enable, 0644,apds9960_show_gesture_enable, apds9960_store_gesture_enable);
static DRIVER_ATTR(light_enable, 0644,apds9960_show_light_enable, apds9960_store_light_enable);
static DRIVER_ATTR(proximity_enable, 0644,apds9960_show_proximity_enable, apds9960_store_proximity_enable);
static DRIVER_ATTR(ps_delay,0644,show_ps_delay,store_ps_delay);
static DRIVER_ATTR(lcd_state,S_IWUSR | S_IRUGO,NULL,apds9960_store_lcd_state);

static struct driver_attribute *apds9960_attr_list[] = {
	&driver_attr_get_reg,
	&driver_attr_debug,
	&driver_attr_als_para_index,
    &driver_attr_ps_base_value,
    &driver_attr_ps_para_index,
    &driver_attr_deviceid,
    &driver_attr_sys_prox_status,
    &driver_attr_calibration,
    &driver_attr_gesture_enable,
    &driver_attr_light_enable,
    &driver_attr_proximity_enable,
    &driver_attr_ps_delay,
	&driver_attr_lcd_state,
};

/*----------------------------------------------------------------------------*/
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
	struct apds9960_data *data = apds9960_obj;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if(ps_en != 0 && ps_en != 1)
		return -EINVAL;

	if(ps_en)
    {
        set_bit(CMC_BIT_PS, (volatile void *)&data->enable);
		apds9960_enable_ps_sensor(data->client, 1);
    }
    else
    {
		clear_bit(CMC_BIT_PS, (volatile void *)&data->enable);
        apds9960_enable_ps_sensor(data->client, 0);
    }

	return count;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9960_data *data = apds9960_obj;
	int ret = 0;

	if (enable)
    {
        set_bit(CMC_BIT_PS, (volatile void *)&data->enable);
		apds9960_enable_ps_sensor(data->client, 1);
    }
	else
    {
	    clear_bit(CMC_BIT_PS, (volatile void *)&data->enable);
        apds9960_enable_ps_sensor(data->client, 0);
    }

	return ret;
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
	struct apds9960_data *data = apds9960_obj;

	als_en = -1;
	sscanf(buf, "%d", &als_en);

	if(als_en != 0 && als_en != 1)
		return -EINVAL;

	if(als_en)
    {
        set_bit(CMC_BIT_ALS, (volatile void *)&data->enable);
		apds9960_enable_als_sensor(data->client, 1);
        msleep(25);     //wait for 25ms to let IC prepare ALS data. //as we set  ADC Integration Time Register (0x81) as 0XF6, it takes 25.2ms.
    }
    else
    {
        clear_bit(CMC_BIT_ALS, (volatile void *)&data->enable);
		apds9960_enable_als_sensor(data->client, 0);
    }

	return count;
}

static int als_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9960_data *data = apds9960_obj;
	int ret = 0;

	if (enable)
    {
        set_bit(CMC_BIT_ALS, (volatile void *)&data->enable);
		apds9960_enable_als_sensor(data->client, 1);
    }
    else
    {
        clear_bit(CMC_BIT_ALS, (volatile void *)&data->enable);
		apds9960_enable_als_sensor(data->client, 0);
    }

	return ret;
}

static ssize_t gs_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t gs_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
    int gs_en;
	struct apds9960_data *data = apds9960_obj;

	gs_en = -1;
	sscanf(buf, "%d", &gs_en);

	if(gs_en != 0 && gs_en != 1)
		return -EINVAL;

	if(gs_en)
    {
        set_bit(CMC_BIT_GS, (volatile void *)&data->enable);
		apds9960_enable_gesture_sensor(data->client, 1);
    }
    else
    {
        clear_bit(CMC_BIT_GS, (volatile void *)&data->enable);
		apds9960_enable_gesture_sensor(data->client, 0);
    }

	return count;
}

static int gs_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9960_data *data = apds9960_obj;
	int ret = 0;

	if (enable)
    {
        set_bit(CMC_BIT_GS, (volatile void *)&data->enable);
		apds9960_enable_gesture_sensor(data->client, 1);
    }
    else
    {
        clear_bit(CMC_BIT_GS, (volatile void *)&data->enable);
		apds9960_enable_gesture_sensor(data->client, 0);
    }

	return ret;
}

//TODO: add set poll delay attr


static struct device_attribute light_attr[] = {
	//__ATTR(poll_delay, 0664, ls_poll_delay_show, ls_poll_delay_store),
	__ATTR(enable, 0664, als_enable_show, als_enable_store),
};

static struct device_attribute proximity_attr[] = {
    //__ATTR(poll_delay, 0664, ps_poll_delay_show, ps_poll_delay_store),
	__ATTR(enable, 0664, ps_enable_show, ps_enable_store),
};

static struct device_attribute gesture_attr[] = {
    //__ATTR(poll_delay, 0664, ps_poll_delay_show, ps_poll_delay_store),
	__ATTR(enable, 0664, gs_enable_show, gs_enable_store),
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

/*----------------------------------------------------------------------------*/
static int apds9960_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(apds9960_attr_list)/sizeof(apds9960_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, apds9960_attr_list[idx])))
		{            
			printk("driver_create_file (%s) = %d\n", apds9960_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int apds9960_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(apds9960_attr_list)/sizeof(apds9960_attr_list[0]));

	if(driver == NULL)
		return -EINVAL;
	
	for(idx = 0; idx < num; idx++)
		driver_remove_file(driver, apds9960_attr_list[idx]);
	
	return err;
}

/*----------------------------------------------------------------------------*/
static void apds9960_eint_work(struct work_struct *work)
{
	struct apds9960_data *obj = (struct apds9960_data *)container_of(work, struct apds9960_data, eint_work);
	//hwm_sensor_data sensor_data;
	int res = 0;
    u8 status_reg = 0;
    u8 enable_reg = 0;
	//weitianlei add start 
	u32 delay = atomic_read(&obj->ps_poll_delay);
	//weitianlei add end 

    //printk("[APDS9960]apds9960_eint_work\n");
    
    if(!test_bit(CMC_BIT_PS, (const volatile void *)&obj->enable) && !test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable) 
        && !obj->eng_ps_sensor && !obj->eng_gesture_sensor)
    {
        printk(KERN_ERR"[APDS9960]no function on!!!\n");
        res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable_reg);
        res =  apds9960_read_reg(APDS9960_STATUS_REG, &status_reg);
	  printk(KERN_ERR"[APDS9960] enable = %d, status_reg = %d\r\n",enable_reg,status_reg);
        apds9960_clear_interrupt(obj->client, CMD_CLR_ALL_INT);
        enable_irq(obj->irq);
        return; 
    }
    
	wake_lock_timeout(&ps_suspend_lock, 2 * HZ);    //wake up for 2s
    if(obj->suspended)
        msleep(10);     //10ms

    if((res = apds9960_read_reg(APDS9960_STATUS_REG, &status_reg)) < 0) {
        printk(KERN_ERR"[APDS9960]status reg read error !!!\n");
        apds9960_clear_interrupt(obj->client, CMD_CLR_ALL_INT);
        enable_irq(obj->irq);
        return;
    }
   
    printk("[APDS9960]status_reg 0x%x\n", status_reg);

    /*********************************************************************
     * NOTE gesture & ps can not work together
     * ******************************************************************/
    //check saturation
    if(status_reg & (APDS9960_STATUS_ASAT|APDS9960_STATUS_PSAT))
    {
        printk(KERN_ERR"[APDS9960]status reg saturation error!!!\n");
       // apds9960_set_pers(obj->client, APDS9960_PPERS_1 | APDS9960_APERS_60);
        apds9960_clear_interrupt(obj->client, CMD_CLR_ALL_INT);
	//    enable_irq(obj->irq);
    }
    //check ps interrupt
    if((status_reg & APDS9960_STATUS_PINT) && ((test_bit(CMC_BIT_PS, (const volatile void *)&obj->enable)) || obj->eng_ps_sensor)) 
    {
        //hwm_sensor_data sensor_data;
         
        printk("[APDS9960]ps int\n");
        //if(!atomic_read(&obj->ps_data_ready)) {
            atomic_set(&obj->ps_data_ready, 1);         //data ready
            hrtimer_cancel( &obj->ps_timer ) ;
            apds9960_set_pers(obj->client, APDS9960_PPERS_1 | APDS9960_APERS_60);
        //}

        //if(!atomic_read(&obj->gesture_calibration) /*|| atomic_read(&obj->gesture_start_flag)*/ 
            //&& !atomic_read(&obj->enabling_ps) && !atomic_read(&obj->enabling_gesture))
        //{
			//weitianlei add start 
			ps_eint_poll = (int)(2000/delay);
			//weitianlei add end 
	      if(apds9960_read_ps(obj->client, (unsigned char *)&obj->ps_data) >= 0)
             {
                 apds9960_get_ps_value(obj, obj->ps_data);
                 input_report_rel(obj->input_dev_ps, REL_X, (atomic_read(&obj->ps_status) + 1));
                 //input_report_rel(data->input_dev_ps, REL_Y, (index%2));
                 input_sync(obj->input_dev_ps);
		  //  printk("[APDS9960] ps report\n");
            }
            queue_delayed_work(obj->poll_queue, &obj->ps_report_work, 40);    //report ps data now
        //}
        
        //apds9960_clear_interrupt(obj->client, CMD_CLR_ALL_INT);
        //enable_irq(obj->irq);
        //return;
    }

    //check with Gesture interrupt
    if((status_reg & APDS9960_STATUS_GINT) && test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable))
    {
        atomic_set(&obj->gesture_start_flag, 1);
        cancel_delayed_work(&obj->gs_report_work);     //stop report timer here

        printk("[APDS9960]gestrue int\n");
        
        //NOTE: first disable als then must resume in gs report worker
        //if(obj->enable_als_sensor == APDS_ENABLE_ALS_NO_INT)
		//{		
            res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable_reg);
            if(res < 0)
                printk(KERN_ERR"[APDS9960]enable reg read error %d !!!\n", res);
            
            enable_reg = enable_reg & (~APDS9960_ALS_ENABLE);
            
            res = apds9960_write_reg(APDS9960_ENABLE_REG, enable_reg);
            if(res < 0)
                printk(KERN_ERR"[APDS9960]enable reg write error %d !!!\n", res);
		//} 

        apds9960_gs_data_polling(obj);
        apds9960_clear_interrupt(obj->client, CMD_CLR_ALL_INT);
	    enable_irq(obj->irq);
        return;
    } 
    if((status_reg & APDS9960_STATUS_GINT) && !test_bit(CMC_BIT_GS, (const volatile void *)&obj->enable))
    {
         apds9960_set_gctrl(obj->client, 0x00);
	   res = apds9960_read_reg(APDS9960_ENABLE_REG, &enable_reg);
         if(res < 0)
             printk(KERN_ERR"[APDS9960]enable reg read error %d !!!\n", res);      
         enable_reg = enable_reg & (~APDS9960_GESTURE_ENABLE);        
         res = apds9960_write_reg(APDS9960_ENABLE_REG, enable_reg);
         if(res < 0)
             printk(KERN_ERR"[APDS9960]enable reg write error %d !!!\n", res);
    }
    //printk(KERN_ERR"[APDS9960] unexpected interrupt type = %d !!!\n", status_reg);
    apds9960_clear_interrupt(obj->client, CMD_CLR_ALL_INT);
    enable_irq(obj->irq);
}

/*----------------------------------------------------------------------------*/
static irqreturn_t apds9960_irq(int irq, void *data)
{
    struct apds9960_data *obj = data;
    
    //printk(KERN_ERR"[APDS9960] apds9960_irq\n");
    disable_irq_nosync(obj->irq);
    queue_work(obj->eint_queue, &obj->eint_work);

    return IRQ_HANDLED;
}

int apds9960_setup_eint(struct i2c_client *client)
{
    struct apds9960_data *obj = i2c_get_clientdata(client); 
    int err;
   
    if(gpio_is_valid(obj->gpio_int)) 
    {
		err = gpio_request(obj->gpio_int, "apds9960_int");
		if(err < 0) 
        {
			printk(KERN_ERR"[APDS9960] unable to request gpio [%d]\n", obj->gpio_int);
			return err;
		}
        
		err = gpio_direction_input(obj->gpio_int);
		if(err < 0) 
        {
			printk(KERN_ERR"[APDS9960 unable to set direction for gpio [%d]\n", obj->gpio_int);
			goto err_irq_gpio_req;
		}

        err = gpio_set_debounce(obj->gpio_int, 2 * 1000);    //2ms debounce
        if(err < 0) 
        {
			printk(KERN_ERR"[APDS9960 unable to set debounce for gpio [%d]\n", obj->gpio_int);
		}
	} 
    else
    {
		printk(KERN_ERR"[APDS9960] irq gpio not provided\n");
		return -EINVAL;
	}
   
    //gpio_free(obj->gpio_int);

    obj->irq = gpio_to_irq(obj->gpio_int);
    if(obj->irq)
    {
        err = request_any_context_irq(obj->irq, apds9960_irq, (IRQF_TRIGGER_LOW | IRQF_ONESHOT), 
                        "apds9960_i2c", obj);
        if (err < 0) {
			printk(KERN_ERR"[APDS9960] request irq failed: %d\n", err);
            return err;
		}

        enable_irq_wake(obj->irq);
    }
    //disable_irq_nosync(obj->irq);
    
    return 0;

err_irq_gpio_req:
	gpio_free(obj->gpio_int);
	return err;
}

/******************************* Initialization function *************************/
static int apds9960_init_client(struct i2c_client *client)
{
	struct apds9960_data *data = i2c_get_clientdata(client);
	int err;
    int i;
    
    //power down sensor
	err = apds9960_set_enable(client, 0);
	if (err < 0)
		return err;
    
    //check id
    for(i=0; i<3; i++) {
        apds9960_read_reg(APDS9960_ID_REG, (u8 *)&chip_id);
        printk("[APDS9960] id=0x%x\r\n", chip_id);
	    if (chip_id == 0xAB || chip_id == 0xAA)
            break;
	}
    if(i >= 3) {
        printk("[APDS9960] id error\r\n");
        return -1;
    }

    //init the regsister
	err = apds9960_set_atime(client, apds9960_als_atime_tb[data->als_atime_index]);	
	if (err < 0) return err;

	err = apds9960_set_wtime(client, apds9960_wtime_tb[data->als_atime_index]);	 // 2.78ms Wait time
	if (err < 0) return err;

	err = apds9960_set_ppulse(client, data->ps_ppulse);	
	if (err < 0) return err;

	err = apds9960_set_poffset_ur(client, data->ps_poffset_ur);	
	if (err < 0) return err;

	err = apds9960_set_poffset_dl(client, data->ps_poffset_dl);	
	if (err < 0) return err;

	err = apds9960_set_config(client, 0x60); // no long wait
	if (err < 0) return err;
    
    //TODO:als_again_index???
	err = apds9960_set_control(client, APDS9960_PDRVIE_FOR_PS | APDS9960_PGAIN_FOR_PS | apds9960_als_again_bit_tb[data->als_again_index]);
	if (err < 0) return err;

	//err = apds9960_set_pilt(client, 0);	 // init threshold in apds9960_ps_select_para
	//if (err < 0) return err;

	//err = apds9960_set_piht(client, 255);
	//if (err < 0) return err;
    
	err = apds9960_set_ailt(client, 0);
	if (err < 0) return err;

	err = apds9960_set_aiht(client, 0xffff);
	if (err < 0) return err;

	err = apds9960_set_pers(client, APDS9960_PPERS_1 | APDS9960_APERS_60);
	if (err < 0) return err;

	// gesture register
	err = apds9960_set_aux(client, 0x01);
	if (err < 0) return err;

	err = apds9960_set_config2(client, 0);
	if (err < 0) return err;

	err = apds9960_set_gthr_in(client, data->gthr_in);
	if (err < 0) return err;

	err = apds9960_set_gthr_out(client, data->gthr_out);
	if (err < 0) return err;

	err = apds9960_set_gconf1(client, APDS9960_GESTURE_FIFO);
	if (err < 0) return err;

	err = apds9960_set_gconf2(client, APDS9960_GDRIVE | APDS9960_GGAIN | APDS9960_GTIME);
	if (err < 0) return err;

	err = apds9960_set_goffset_u(client, 0);
	if (err < 0) return err;

	err = apds9960_set_goffset_d(client, 0);
	if (err < 0) return err;

    err = apds9960_set_goffset_l(client, 0);
	if (err < 0) return err;

	err = apds9960_set_goffset_r(client, 0);
	if (err < 0) return err;

	err = apds9960_set_gpulse(client, (APDS9960_GPULSE-1) | APDS9960_GPULSE_LEN);
	if (err < 0) return err;

	err = apds9960_set_gconf3(client, 0);
	if (err < 0) return err;

	err = apds9960_set_gctrl(client, 0x04);     //Clears the GFIFO, GINT, GVALID, GFIFO_OV and GFIFO_LVL
	if (err < 0) return err;

	// sensor is in disabled mode but all the configurations are preset
    // clear eint
    apds9960_clear_interrupt(client, CMD_CLR_ALL_INT);

    // set up the eint gpio, unmask the eint when ps os gst on 
    apds9960_setup_eint(client);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9960_parse_dt(struct device *dev, struct apds9960_data *obj)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_get_named_gpio_flags(np, "irq-gpio",
			0, NULL);
	if (rc < 0) {
		printk(KERN_ERR"[APDS9960] Unable to read interrupt pin number\n");
		return rc;
	} else {
		obj->gpio_int = rc;
	}
    
    printk(KERN_ERR"[APDS9960] read interrupt pin number %d\n", obj->gpio_int);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9960_lightsensor_setup(struct apds9960_data *obj)
{
	int ret;

	obj->input_dev_als = input_allocate_device();
	if (!obj->input_dev_als) {
		printk(KERN_ERR
			"[APDS9960] %s: could not allocate als input device\n",
			__func__);
		return -ENOMEM;
	}
	obj->input_dev_als->name = "light";
	obj->input_dev_als->id.bustype = BUS_I2C;
	set_bit(EV_ABS, obj->input_dev_als->evbit);
    input_set_abs_params(obj->input_dev_als, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(obj->input_dev_als, ABS_Y, 0, 1, 0, 0);          //fake index

	ret = input_register_device(obj->input_dev_als);
	if (ret < 0) {
		printk(KERN_ERR"[APDS9960] %s: can not register als input device\n",
				__func__);
		goto err_free_als_input_device;
	}

	return ret;

err_free_als_input_device:
	input_free_device(obj->input_dev_als);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int apds9960_psensor_setup(struct apds9960_data *obj)
{
	int ret;

	obj->input_dev_ps = input_allocate_device();
	if (!obj->input_dev_ps) {
		printk(KERN_ERR
			"[APDS9960] %s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	obj->input_dev_ps->name = "proximity";
	obj->input_dev_ps->id.bustype = BUS_I2C;
	
    //set_bit(EV_ABS, obj->input_dev_ps->evbit);
    //input_set_abs_params(obj->input_dev_ps, ABS_X, 0, 1, 0, 0);
	//input_set_abs_params(obj->input_dev_ps, ABS_Y, 0, 1, 0, 0);          //fake index
    
    obj->input_dev_ps->evbit[0] = BIT_MASK(EV_REL);
	obj->input_dev_ps->relbit[0] = BIT_MASK(REL_X); 

	ret = input_register_device(obj->input_dev_ps);
	if (ret < 0) {
		printk(KERN_ERR
			"[APDS9960] %s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	return ret;

err_free_ps_input_device:
	input_free_device(obj->input_dev_ps);
	return ret;
}

/*----------------------------------------------------------------------------*/
//TODO: gesture input device regsister
static int apds9960_gssensor_setup(struct apds9960_data *obj)
{
	int ret;

	obj->input_dev_gs = input_allocate_device();
	if (!obj->input_dev_gs) {
		printk(KERN_ERR
			"[APDS9960] %s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	obj->input_dev_gs->name = "ir_gesture";
	obj->input_dev_gs->id.bustype = BUS_I2C;
	set_bit(EV_ABS, obj->input_dev_gs->evbit);
	input_set_abs_params(obj->input_dev_gs, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(obj->input_dev_gs, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(obj->input_dev_gs, ABS_Z, INT_MIN, INT_MAX, 0, 0);

	ret = input_register_device(obj->input_dev_gs);
	if (ret < 0) {
		printk(KERN_ERR
			"[APDS9960] %s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	return ret;

err_free_ps_input_device:
	input_free_device(obj->input_dev_gs);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int apds9960_config_regulator(struct apds9960_data *obj, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(apds9960_vreg) / sizeof(struct sensor_regulator);

	if (on) 
    {
		for (i = 0; i < num_reg; i++) 
        {
			apds9960_vreg[i].vreg = regulator_get(&obj->client->dev, apds9960_vreg[i].name);
            
			if (IS_ERR(apds9960_vreg[i].vreg)) 
            {
				rc = PTR_ERR(apds9960_vreg[i].vreg);
				printk(KERN_ERR"[APDS9960] %s:regulator get failed rc=%d\n", __func__, rc);
				apds9960_vreg[i].vreg = NULL;
			}

			if (regulator_count_voltages(apds9960_vreg[i].vreg) > 0) 
            {
				rc = regulator_set_voltage(
					apds9960_vreg[i].vreg,
					apds9960_vreg[i].min_uV,
					apds9960_vreg[i].max_uV);
				if(rc) {
					printk(KERN_ERR"[APDS9960] %s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(apds9960_vreg[i].vreg);
					apds9960_vreg[i].vreg = NULL;
				}
			}

			rc = regulator_enable(apds9960_vreg[i].vreg);
			if (rc) {
				printk(KERN_ERR"[APDS9960] %s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(apds9960_vreg[i].vreg) > 0) 
                {
					regulator_set_voltage(
						apds9960_vreg[i].vreg, 0,
						apds9960_vreg[i].max_uV);
				}
				regulator_put(apds9960_vreg[i].vreg);
				apds9960_vreg[i].vreg = NULL;
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
static int apds9960_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct hwmsen_object obj_ps, obj_als,obj_gs;
    //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    //struct i2c_client *new_client;
	struct apds9960_data *obj;
	int err = 0;
	
    //init locks
    mutex_init(&apds9960_lock);
    mutex_init(&apds9960_ps_lock);
    mutex_init(&apds9960_gs_lock);

	obj = kzalloc(sizeof(struct apds9960_data), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
    memset(obj, 0, sizeof(*obj));
	apds9960_obj = obj;

    //TODO:get the platform data from device tree
    if (client->dev.of_node) {
		err = apds9960_parse_dt(&client->dev, obj);
		/*if (ret) {
			goto err_parse_dt;
		}*/
	} 
    
    //init the i2c client
	obj->client = client;
	apds9960_i2c_client = client;

	i2c_set_clientdata(client, obj);

    //power on
    apds9960_config_regulator(obj, true);
    msleep(5);
	
	if(bbk_ps_para_count != 0)
		apds9960_ps_para = bbk_ps_paras;
	else
		apds9960_ps_para = bbk_ps_paras_default;

	if(bbk_als_para_count != 0)
		apds9960_als_para = bbk_als_paras;
	else
		apds9960_als_para = bbk_als_paras_default;
		
    //init parameters
	obj->enable = 0;
	obj->enable_als_sensor = 0;	    // default to 0
	obj->enable_ps_sensor  = 0;	    // default to 0
	obj->enable_gesture_sensor = 0;	// default to 0
    obj->suspended = 0;
	obj->enable_suspended_value = 0;
    
    //als parameters
	obj->als_atime_index = APDS9960_ALS_RES_24MS;	// 24ms ATIME
	obj->als_again_index = APDS9960_ALS_GAIN_64X;	// 64x AGAIN
	obj->als_prev_lux = 0;

	obj->RGB_COE_X[0] = RGB_COE_X[0];
	obj->RGB_COE_X[1] = RGB_COE_X[1];
	obj->RGB_COE_X[2] = RGB_COE_X[2];

	obj->RGB_COE_Y[0] = RGB_COE_Y[0];
	obj->RGB_COE_Y[1] = RGB_COE_Y[1];
	obj->RGB_COE_Y[2] = RGB_COE_Y[2];

	obj->RGB_COE_Z[0] = RGB_COE_Z[0];
	obj->RGB_COE_Z[1] = RGB_COE_Z[1];
	obj->RGB_COE_Z[2] = RGB_COE_Z[2];

	obj->lux_GA1 = APDS9960_LUX_GA1;
	obj->lux_GA2 = APDS9960_LUX_GA2;
	obj->lux_GA3 = APDS9960_LUX_GA3;
	
	obj->cct_GA1 = APDS9960_CCT_GA1;
	obj->cct_GA2 = APDS9960_CCT_GA2;
	obj->cct_GA3 = APDS9960_CCT_GA3;
    
    //gs parameters
	obj->gesture_ppulse = (APDS9960_PPULSE_FOR_GESTURE - 1) | APDS9960_PPULSE_LEN_FOR_GESTURE;
      obj->gthr_in  = GESTURE_GTHR_IN;
	obj->gthr_out = GESTURE_GTHR_OUT;
	obj->gesture_poffset_dl = 0;
	obj->gesture_poffset_ur = 0;
	obj->gesture_goffset_u = 0;
	obj->gesture_goffset_d = 0;
	obj->gesture_goffset_l = 0;
	obj->gesture_goffset_r = 0;
	obj->segment.d_factor = D_Factor1;
      obj->segment.r_coef = R_Coef1;
      obj->segment.g_coef = G_Coef1;
      obj->segment.b_coef = B_Coef1;
      obj->segment.ct_coef = CT_Coef1;
      obj->segment.ct_offset = CT_Offset1;

    //ps parameters
	obj->ps_ppulse = (APDS9960_PPULSE_FOR_PS - 1) | APDS9960_PPULSE_LEN_FOR_PS;
	obj->ps_poffset_ur = 0;
    obj->ps_poffset_dl = 0;
    atomic_set(&obj->ps_first_int, 1);
    atomic_set(&obj->sys_prox_status, 1);
    atomic_set(&obj->ps_base_value, apds9960_ps_para[ps_para_num].base_value);
  
    atomic_set(&obj->enabling_ps, 0);
    atomic_set(&obj->enabling_gesture, 0);
    
    atomic_set(&obj->ps_poll_delay, APDS9960_PS_DEFAULT_POLL_DELAY);
    atomic_set(&obj->gs_poll_delay, APDS9960_GS_DEFAULT_POLL_DELAY);
    atomic_set(&obj->als_poll_delay, APDS9960_ALS_DEFAULT_POLL_DELAY);

		
	apds9960_ps_select_para(apds9960_obj);
    
    //input deives setup
    //TODO: power set up
    err = apds9960_lightsensor_setup(apds9960_obj);
	if (err < 0) {
		printk(KERN_ERR"[APDS9960] %s: apds9960_lightsensor_setup error!\n", __func__);
		goto exit_init_failed;
	}

	err = apds9960_psensor_setup(apds9960_obj);
	if (err < 0) {
		printk(KERN_ERR"[APDS9960] apds9960_psensor_setup error!!\n");
		goto exit_init_failed;
	}

    err = apds9960_gssensor_setup(apds9960_obj);
	if (err < 0) {
		printk(KERN_ERR"[APDS9960] apds9960_gssensor_setup error!!\n");
		goto exit_init_failed;
	}

    err = create_sysfs_interfaces(&apds9960_obj->input_dev_als->dev, light_attr,
			ARRAY_SIZE(light_attr));
	if (err < 0) {
		printk(KERN_ERR"[APDS9960] failed to create input_dev_als sysfs\n");
		goto exit_init_failed;
	}

	err = create_sysfs_interfaces(&apds9960_obj->input_dev_ps->dev, proximity_attr,
			ARRAY_SIZE(proximity_attr));
	if (err < 0) {
		printk(KERN_ERR"[APDS9960] failed to create input_dev_gs sysfs\n");
		goto exit_init_failed;
	}

    err = create_sysfs_interfaces(&apds9960_obj->input_dev_gs->dev, gesture_attr,
			ARRAY_SIZE(gesture_attr));
	if (err < 0) {
		printk(KERN_ERR"[APDS9960] failed to create input_dev_gs sysfs\n");
		goto exit_init_failed;
	}

    //init eint work queue and work
    obj->eint_queue = create_singlethread_workqueue("apds9960_eint");
    INIT_WORK(&obj->eint_work, apds9960_eint_work);

	//init the chip
	err = apds9960_init_client(client);
	if (err)
		goto exit_init_failed;
    printk("[APDS9960]apds9960_i2c_probe 222222\r\n");
   
    //regsister sensor class
    if(chip_id == 0xAA)
    {
        apds9960_obj->als_cdev = sensors_light_cdev1;
	    apds9960_obj->als_cdev.sensors_enable = als_enable_set;
	    apds9960_obj->als_cdev.sensors_poll_delay = NULL;
	    apds9960_obj->als_cdev.min_delay = APDS9960_ALS_MIN_POLL_DELAY * 1000;

	    apds9960_obj->ps_cdev = sensors_proximity_cdev1;
	    apds9960_obj->ps_cdev.sensors_enable = ps_enable_set;
	    apds9960_obj->ps_cdev.sensors_poll_delay = NULL;
	  //  apds9960_obj->ps_cdev.min_delay = APDS9960_PS_MIN_POLL_DELAY * 1000;
    
        apds9960_obj->gs_cdev = sensors_gesture_cdev1;
	    apds9960_obj->gs_cdev.sensors_enable = gs_enable_set;
	    apds9960_obj->gs_cdev.sensors_poll_delay = NULL;
	    apds9960_obj->gs_cdev.min_delay = APDS9960_GS_MIN_POLL_DELAY * 1000;
    }
    else
    {
        apds9960_obj->als_cdev = sensors_light_cdev0;
	    apds9960_obj->als_cdev.sensors_enable = als_enable_set;
	    apds9960_obj->als_cdev.sensors_poll_delay = NULL;
	    apds9960_obj->als_cdev.min_delay = APDS9960_ALS_MIN_POLL_DELAY * 1000;

	    apds9960_obj->ps_cdev = sensors_proximity_cdev0;
	    apds9960_obj->ps_cdev.sensors_enable = ps_enable_set;
	    apds9960_obj->ps_cdev.sensors_poll_delay = NULL;
	 //   apds9960_obj->ps_cdev.min_delay = APDS9960_PS_MIN_POLL_DELAY * 1000;
    
        apds9960_obj->gs_cdev = sensors_gesture_cdev0;
	    apds9960_obj->gs_cdev.sensors_enable = gs_enable_set;
	    apds9960_obj->gs_cdev.sensors_poll_delay = NULL;
	    apds9960_obj->gs_cdev.min_delay = APDS9960_GS_MIN_POLL_DELAY * 1000;
    }

    err = sensors_classdev_register(&client->dev, &apds9960_obj->als_cdev);
	if(err)
		goto exit_init_failed;

	err = sensors_classdev_register(&client->dev, &apds9960_obj->ps_cdev);
	if(err)
		goto exit_init_failed;

    err = sensors_classdev_register(&client->dev, &apds9960_obj->gs_cdev);
	if(err)
		goto exit_init_failed;
    
    /* Register for sensor ioctl */
    err = misc_register(&apds9960_device);
	if (err) {
		printk("Unalbe to register ps ioctl: %d", err);
		goto exit_misc_device_register_failed;
	}
	err = misc_register(&apds9960_als_device);
	if (err) {
		printk("Unalbe to register apds9960_als_device: %d", err);
		goto exit_misc_device_register_failed;
	}
    
    printk("[APDS9960]apds9960_i2c_probe 333333\r\n");
    
	if((err = apds9960_create_attr(&apds9960_psgs_driver.driver)))
	{
		printk("apds9960_create_attr create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	if((err = apds9960_create_attr(&apds9960_als_driver.driver)))
	{
		printk("apds9960_create_attr apds9960_als_driver err = %d\n", err);
		goto exit_create_attr_failed;
	}
    
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = apds9960_early_suspend,
	obj->early_drv.resume   = apds9960_late_resume,    
	register_early_suspend(&obj->early_drv);
#else
    obj->fb_notif.notifier_call = apds9960_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif
    
    obj->gs_workqueue = create_singlethread_workqueue("gesture_workqueue");
	//hrtimer_init(&obj->gs_polling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_init(&obj->ps_timer,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	obj->ps_delay = ns_to_ktime(200*1000 * NSEC_PER_USEC);
	obj->ps_timer.function = apds9960_ps_timer_func;
	//obj->gs_polling_delay = ns_to_ktime(obj->gs_delay * 100 * NSEC_PER_USEC);
	//obj->gs_polling_timer.function = apds9960_gs_report_timer_func;
	INIT_DELAYED_WORK(&obj->gs_report_work, apds9960_gs_data_report);
    INIT_DELAYED_WORK(&obj->gs_resume_work, apds9960_gs_resume);
    INIT_DELAYED_WORK(&obj->gs_ps_monitor,  apds9960_ps_monitor);
    INIT_DELAYED_WORK(&obj->gs_resume_delay,  apds9960_resume_delay);
   
    obj->poll_queue = create_singlethread_workqueue("apds9960_report_workqueue");
    INIT_DELAYED_WORK(&obj->ps_report_work, apds9960_ps_data_report);
    //INIT_DELAYED_WORK(&obj->als_report_work, apds9960_als_data_report);


	printk("[APDS9960] %s: OK in line %d\n", __func__, __LINE__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&apds9960_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(client);
//exit_kfree:
	kfree(obj);
exit:
	apds9960_i2c_client = NULL;           
	
	printk("[APDS9960] %s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int apds9960_i2c_remove(struct i2c_client *client)
{
    //struct apds9960_data *data = i2c_get_clientdata(client);
	int err;	
    
    if((err = apds9960_delete_attr(&apds9960_psgs_driver.driver)))
		printk("apds9960_delete_attr fail: %d\n", err);
	
	if((err = apds9960_delete_attr(&apds9960_als_driver.driver)))
		printk("apds9960_delete_attr apds9960_als_driverfail: %d\n", err);
	
	if((err = misc_deregister(&apds9960_device)))
		printk("misc_deregister fail: %d\n", err);    
	
	if(apds9960_obj != NULL){
		flush_workqueue(apds9960_obj->gs_workqueue);
		destroy_workqueue(apds9960_obj->gs_workqueue);
	}

	apds9960_i2c_client = NULL;
	i2c_unregister_device(client);

	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9960_probe(struct platform_device *pdev) 
{
    //struct alsps_hw *hw = get_cust_alsps_hw();
    
	//tmd2771_power(hw, 1); 
	if(i2c_add_driver(&apds9960_i2c_driver))
	{
		printk("add driver error\n");
		return -1;
	} 
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9960_remove(struct platform_device *pdev)
{
	//struct alsps_hw *hw = get_cust_alsps_hw();
	
	//tmd2771_power(hw, 0);    
	i2c_del_driver(&apds9960_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static struct platform_driver apds9960_psgs_driver = {
	.probe      = apds9960_probe,
	.remove     =apds9960_remove,    
	.driver     = {
		.name  = "psgs",
		.owner = THIS_MODULE,
	}
};

struct platform_device sensor_psgs = {
    .name = "psgs",
    .id   = -1,
};

static int apds9960_als_probe(struct platform_device *pdev) 
{
	//struct alsps_hw *hw = get_cust_alsps_hw();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9960_als_remove(struct platform_device *pdev)
{
	//struct alsps_hw *hw = get_cust_alsps_hw();
	return 0;
}

static struct platform_driver apds9960_als_driver = {
	.probe  = apds9960_als_probe,
	.remove = apds9960_als_remove,    
	.driver = {
		.name  = "als",
		.owner = THIS_MODULE,
	}
};

struct platform_device sensor_als = {
    .name = "als",
    .id   = -1,
};

static int __init apds9960_init(void)
{
    //register platform driver
    if(platform_driver_register(&apds9960_als_driver))
	{
		printk("failed to register apds9960_als_driver");
		return -ENODEV;
	}
    
    if(platform_driver_register(&apds9960_psgs_driver))
	{
		printk("failed to register apds9960_psgs_driver");
		return -ENODEV;
	}	
   
    //register platform device
    if(platform_device_register(&sensor_als))
    {
        printk("failed to register sensor_als");
		return -ENODEV;
    }

    if(platform_device_register(&sensor_psgs))
    {
		printk("failed to register sensor_psgs");
		return -ENODEV;
    }

	wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");

	return 0;
}

static void __exit apds9960_exit(void)
{
	platform_driver_unregister(&apds9960_psgs_driver);
	platform_driver_unregister(&apds9960_als_driver);
}

module_init(apds9960_init);
module_exit(apds9960_exit);

MODULE_AUTHOR("Ling Xu <xuling@vivo.com.cn>");
MODULE_DESCRIPTION("APDS9960 gesture + RGB + ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

