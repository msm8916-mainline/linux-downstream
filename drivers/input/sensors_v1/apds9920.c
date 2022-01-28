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
//#include <linux/timer.h>
//#include <linux/timex.h>
//#include <linux/rtc.h>
//#include <mach/devs.h>
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

#include "apds9920.h"

/******************************************************************************
 * configuration
*******************************************************************************/
#define APDS9920_DEV_NAME     "APDS9920" 
#define APDS9920_CHIP          0
#define APDS9920_DEVICE 1


#define DEFAULT_STRONG_SUNLIGHT   7000 //modified from 15059 PD1216[B121109-260] //45176 //the atime is changed from 150ms to 50ms
#define DEFAULT_PS_FILTER_DELAY   50
#define DEFAULT_ATIME             50

#define ALS_LOW_LUX_MODE		0
#define ALS_NORMAL_LUX_MODE		1

#define ALS_LOW_LUX_SETTING		0x02        //16X
#define ALS_NORMAL_LUX_SETTING 	0x00        //1x

#define ALS_LUX_HIGH_THRESHOLD		((85*(0X100-0xED)*1024)/100) // 85% of max als ch0 count
#define ALS_LUX_LOW_THRESHOLD		((5*(0X100-0xED)*1024)/100)  // 5%  of max als ch0 count

#define ALS_MIN_POLL_DELAY      150
#define ALS_DEFAULT_POLL_DELAY	120

#define PS_MIN_POLL_DELAY       10
#define PS_DEFAULT_POLL_DELAY   120

#if defined(PD1403L) || defined(PD1403LG4) || defined(PD1403F) || defined(PD1403V)
#define DGF    182600L      //X1000
#define BCOEF  1810L        //X1000
#define CCOEF  236L          //X1000
#define DCOEF  363L         //X1000

#elif defined(PD1419L) || defined(PD1419LG4) || defined(PD1419F) || defined(PD1419V)
#define DGF    524800L      //X1000
#define BCOEF  1860L        //X1000
#define CCOEF  750L          //X1000
#define DCOEF  1290L         //X1000
#else
#define DGF    456200L      //X1000
#define BCOEF  1820L        //X1000
#define CCOEF  99L          //X1000
#define DCOEF  152L         //X1000
#endif

//#define APS_TAG                  "[ALS/PS] "
//#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
//#define APS_ERR(KERN_ERR TAGE fmt, args...)    printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
//#define APS_TAG(KERN_INFO TAGI fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)     
//#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define TAG					"PAsensor"
#define TAGI					"PAsensor.I"	//KERN_INFO
#define TAGE					"PAsensor.E"	//KERN_ERR
#define CONT_INT_TIME     50    //50ms
#define CONT_INT_GATE     20
#define POLL_DELAY_TIMES  2     //2X
#define POLL_DELAY_GATE 5
#define SAME_DATA_GATE  20
#define STATUS_PVALID	0x02
#define STATUS_AVALID 	0x01
extern void print_vivo_init(const char* fmt, ...);
extern void print_vivo_main(const char* fmt, ...);
extern int qup_i2c_suspended; // add for i2c timeout   


/******************************************************************************
 * local struct
*******************************************************************************/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
    CMC_BIT_ENG_ALS = 3,
    CMC_BIT_ENG_PS  =4,
} CMC_BIT;


struct apds9920_i2c_addr {
    u8  write_addr;  
    u8  ps_thd;
};

struct apds9920_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;

    struct workqueue_struct *eint_queue;
    struct work_struct eint_work;
    struct workqueue_struct *apds9920_fb_workqueue;
    struct work_struct apds9920_fb_work;
    //struct workqueue_struct *eint_workqueue;

    /*i2c address group*/
    struct apds9920_i2c_addr  addr;
    
    /*misc*/
    u32		    als_modulus;
    u32         als_mode;
    atomic_t    i2c_retry;
//    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/

    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
    
	//u32 		enable_als_by_ps;
	//u32 		enable_als_by_system;#if defined(CONFIG_HAS_EARLYSUSPEND)
	u32 		strong_sunlight;
    atomic_t    pulse_value;
	//adb by dengweicheng bug[]
	atomic_t	sys_prox_status;          //current prox state,  1: far away 0:

	atomic_t	ps_in_threshold;          //the threshold used now
    atomic_t    ps_out_threshold;

    atomic_t	ps_status;                //current prox state,  1: far away 0:close
    atomic_t    ps_base_value;            //the base value is from app  
    
    atomic_t    ps_first_int;
	u32			ps_filter_delay;
	int			last_ps_data;
      int         last_als_data;
	int			count_ps_over_in;
	char 		write_register[20];
	char		read_register[20];
	unsigned char		prox_offset;
	unsigned char als_atime_index;
    unsigned char als_last_atime_index;
    
    //atmic_t     cal_mode;                //0: nomal 1: calibration
    
    /*early suspend*/
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
    struct wake_lock wakelock;

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
static unsigned char apds9920_als_atime_tb[] = { 0xED, 0xc0, 0xF6 };
static unsigned short apds9920_als_integration_tb[] = { 5168, 17408, 2720};
static unsigned short apds9920_als_res_tb[] = { 19456, 65535, 10239 };
static unsigned short is_short_atime_mode = 0;
static unsigned short is_prox_across_enable =0;
static unsigned short stronglight_flag = 0;
static int alsCount = 0;
static int ps_read_count = 0;
static int als_read_count = 0;
static u8 change = 0;
static int *blank;
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
//extern void mt_eint_dis_debounce(unsigned int eint_num);
extern int module_check(void);
/******************************************************************************
 * local functions
*******************************************************************************/
static int apds9920_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int apds9920_i2c_remove(struct i2c_client *client);
//static int apds9920_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int apds9920_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9920_i2c_resume(struct i2c_client *client);
int apds9920_enable_eint(struct apds9920_priv *obj,int enable);
static int apds9920_init_client(struct i2c_client *client);

int apds9920_read_ps(struct i2c_client *client, u16 *data);
int apds9920_read_als_c0(struct i2c_client *client, u16 *data);

int apds9920_enable_als(struct i2c_client *client, int enable);
int apds9920_enable_ps(struct i2c_client *client, int enable);

static int ps_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable);
static int als_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable);
static void apds9920_ps_data_report(struct work_struct *work);

int enable_als_short_atime_mode(struct apds9920_priv *obj, int enable);
/******************************************************************************
 * local Variable
*******************************************************************************/
static const struct i2c_device_id apds9920_i2c_id[] = {{APDS9920_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_apds9920 = {I2C_BOARD_INFO("APDS9920", 0x39)};

static struct mutex ps_value_lock = __MUTEX_INITIALIZER(ps_value_lock);   //irq handle in eint_queue, poll handle in poll_queue
static struct mutex ps_report_lock = __MUTEX_INITIALIZER(ps_report_lock);
static struct wake_lock ps_suspend_lock;

static struct i2c_driver apds9920_i2c_driver = {	
	.probe      = apds9920_i2c_probe,
	.remove     = apds9920_i2c_remove,
	//.detect     = apds9920_i2c_detect,
	.suspend    = apds9920_i2c_suspend,
	.resume     = apds9920_i2c_resume,
	.id_table   = apds9920_i2c_id,
    //.address_data = &apds9920_addr_data,
	.driver = {
        //.owner        = THIS_MODULE,
		.name           = APDS9920_DEV_NAME,
	},
};
static struct i2c_client   *apds9920_i2c_client = NULL;
static struct apds9920_priv *g_apds9920_ptr      = NULL;
static struct apds9920_priv *apds9920_obj        = NULL;

static struct platform_driver apds9920_alsps_driver;

static int chip_id = 0xff;
static int device_id = 0xff;
static int restart_ps_automatic=0;
static int debug=0;
static int debug_flag=0;
//parameters
static struct ps_para bbk_ps_paras_default[] = bbk_ps_para();
static struct als_para bbk_als_paras_default[] = bbk_als_para();
struct ps_para  *apds9920_ps_para = NULL;
struct als_para *apds9920_als_para = NULL;
static int ps_para_num  = 0;          //the value must select early
static int als_para_num = 0;
static unsigned pulse = 0x04;
static int last_ps_data=-1;
static int ps_data_same_count=0;
static int PS_ESD_TIME=10;
static int unenable_and_read_count=0;

static int nAlsReadCount=0;


static struct mutex apds9920_lock;

static struct sensors_classdev sensors_light_cdev0 = {
	.name = "APDS9920-light",
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
	.name = "APDS9920-proximity",
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
struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator apds9920_vreg[] = {
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

static int apds9920_read_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
    u8 temp_address;
    u8 *temp_data;
    int res = 0;
    if(qup_i2c_suspended > 0){
        printk(KERN_INFO TAGI" apds9920 read reg when i2c bus is suspended.\n");
        return -1;		
    }

    mutex_lock(&apds9920_lock); 

    temp_address = addr;
    temp_data = data;

    res = i2c_master_send(client, &temp_address, 0x1);
    if(res < 0)
    {
        if(temp_address != 0x92)
            printk(KERN_ERR TAGE "apds9920 read 0x%x fail %d\n", temp_address, res);
        mutex_unlock(&apds9920_lock); 
        return res;
    }
    
    res = i2c_master_recv(client, temp_data, count);
    if(res < 0)
    {
	 if(temp_address != 0x92)
            printk(KERN_ERR TAGE"apds9920 read 0x%x fail %d\n", temp_address, res);
        mutex_unlock(&apds9920_lock); 
        return res;
    }
    
    mutex_unlock(&apds9920_lock); 
    
    return count;
}


static int apds9920_write_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
    u8 buffer[10] = {0};
    int res = 0;
    if(qup_i2c_suspended > 0){
        printk(KERN_INFO TAGI" apds9920 write reg when i2c bus is suspended.\n");
        return -1;		
    }
    mutex_lock(&apds9920_lock); 

    buffer[0] = addr;
    if(data != NULL && count > 0 && count < 5)
    {
        memcpy(&buffer[1], data, count);
    }

    res = i2c_master_send(client, buffer, (0x01 + count));
    if(res < 0)
    {
        printk(KERN_ERR TAGE "apds9920 write 0x%x fail %d\n", buffer[0], res);
        mutex_unlock(&apds9920_lock); 
        return res;
    }

    mutex_unlock(&apds9920_lock); 
    
    return (count + 1);
}

/*----------------------------------------------------------------------------*/
/*** enable: 1 for change to short ATIME,0 for out for ***/
int enable_als_short_atime_mode(struct apds9920_priv *obj, int enable){
   u8 buffer[2];
   if(obj==NULL){
          printk(KERN_ERR TAGE "%s obj is null.\n",__FUNCTION__);
          return -1;
   }

   if(enable != is_short_atime_mode){
          if(enable){
                 //change ATIME to 0xF6, index to 0;GAIN TO 1(als_modulus);
                 is_short_atime_mode=1;
                 buffer[0] = APDS9920_CMM_ATIME;
                 buffer[1] = apds9920_als_atime_tb[2];
                 apds9920_write_reg(obj->client, buffer[0], &buffer[1], 0x1);
				 obj->als_atime_index=2;
                 printk(KERN_INFO TAGI "%s enable=%d change ATIME to 0x%x \n",__FUNCTION__,enable,
				 apds9920_als_atime_tb[obj->als_atime_index]);
          } else {
                 //change ATIME to 0xED
                 buffer[0] = APDS9920_CMM_ATIME;
                 buffer[1] = apds9920_als_atime_tb[0];
                 apds9920_write_reg(obj->client, buffer[0], &buffer[1], 0x1);
				 obj->als_atime_index=0;
                 is_short_atime_mode=0;
                 obj->als_last_atime_index = obj->als_atime_index;
                 printk(KERN_INFO TAGI "%s enable=%d change ATIME to 0x%x \n",__FUNCTION__,enable,
                 apds9920_als_atime_tb[obj->als_atime_index]);
          }
   } else {
          printk(KERN_INFO TAGI "%s same atime mode = %d\n",__FUNCTION__,is_short_atime_mode);
   }
   return 0;
}

#if 1
int apds9920_read_als_c0(struct i2c_client *client, u16 *data)
{
//	struct apds9920_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value;	 
//	u32 c0_nf;
	//u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL || data == NULL )
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
    buffer[0] = APDS9920_CMM_C0DATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)&c0_value, 0x2);
    if(res <= 0) {
        goto EXIT_ERR;
    }
    
    *data = c0_value;
     if(debug){
         printk(KERN_INFO TAGI "apds9920_read_als_c0 c0_value = %d\r\n",*data);
     }
/*    
    buffer[0]=APDS9920_CMM_C1DATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)(&c1_value), 0x2);
	if(res <= 0)
	{
        goto EXIT_ERR;
	}
    printk(KERN_INFO TAGI "c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);
*/
	return 0;	 	
	
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_read_als_c0 fail\n");
	return res;
}
#endif

/*  update lux convert algo from avago FAE --liaoxl.lenovo start */
#define APDS9920_COE_B1		195	/* 2.23 without glass window */
#define APDS9920_COE_C1		73	/* 0.70 without glass window */
#define APDS9920_COE_D1		129	/* 1.42 without glass window */
#define APDS9920_COE_GA1		85		/* 0.48 without glass window */
#define APDS9920_DF		52
#define APDS9920_MAX_LUXVALUE (30000*1000)

/*----------------------------------------------------------------------------*/
static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
	/*  update lux convert algo from avago FAE --20130817 molg1.lenovo begin*/
	struct apds9920_priv *data = i2c_get_clientdata(client);
	int luxValue = 0;
	int IAC1 = 0;
	int IAC2 = 0;
	int IAC = 0;
	u64 data1;
	u32 data2;
	int ch1ch0_ratio = 0;
	int APDS9920_COE_B, APDS9920_COE_C, APDS9920_COE_D, APDS9920_GA;
//	int APDS9920_OUTDOOR_GA = 250; //125;
	if ( (ch0data >= apds9920_als_res_tb[data->als_atime_index]) && (ch1data >= apds9920_als_res_tb[data->als_atime_index]) ) {

		luxValue = APDS9920_MAX_LUXVALUE;
		return (luxValue);
	}
	if (ch0data == 0) {
		luxValue = 0;
		return (luxValue);
	}
	ch1ch0_ratio = (ch1data * 100) / ch0data;	// scale up by 100
	/*
	if ( (ch1ch0_ratio >= 30) && (ch1ch0_ratio <= 45) ) {
		// sunglight
		luxValue = (ch0data*100)/((apds9920_als_integration_tb[data->als_atime_index]/100)*data->als_modulus);
		luxValue = (luxValue * APDS9920_OUTDOOR_GA)/100;
		// DO NOT USE ALS_REDUCE

		return luxValue;
	}
	else */
//	{
		// white light/Incand
		APDS9920_COE_B = APDS9920_COE_B1;
		APDS9920_COE_C = APDS9920_COE_C1;
		APDS9920_COE_D = APDS9920_COE_D1;
		APDS9920_GA = APDS9920_COE_GA1;
//	}

	IAC1 = (ch0data * 100 - (APDS9920_COE_B * ch1data));			// re-adjust COE_B to avoid 2 decimal point
	IAC2 = ((APDS9920_COE_C * ch0data) - (APDS9920_COE_D * ch1data));	// re-adjust COE_C and COE_D to void 2 decimal point
	if (IAC1 > IAC2) {
		IAC = IAC1;
	} else if (IAC1 <= IAC2) {
		IAC = IAC2;
	} else {
		IAC = 0;
	}

	if ((IAC1 < 0) && (IAC2 < 0)) {
		if (ch0data < (apds9920_als_res_tb[data->als_atime_index] / 2)) {
			//printk("apds9920 IAC = %d\r\n",IAC);
			IAC = 0;	// cdata and irdata saturated
		} else {
			//printk("apds9920 luxValue1 = %d\r\n",luxValue);
			luxValue = APDS9920_MAX_LUXVALUE;
			return (luxValue);
		}
	}
	data1 = (u64)IAC;
	data1 = (data1 * APDS9920_GA * APDS9920_DF);
	//data1 = div_u64(data1, 10);
	data2 = ((apds9920_als_integration_tb[data->als_atime_index] / 100) * data->als_modulus);

	luxValue = div_u64(data1, data2);
	if(debug){
		printk(KERN_INFO TAGI "%s als_modulus = %d, als_atime_index = %d, final luxValue = %d\r\n",__func__, data->als_modulus,data->als_atime_index,luxValue);
		}
	return luxValue;
}

int apds9920_read_als0(struct i2c_client *client, u16 *data)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);
	u16 c1_value,c0_value;
	int luxValue;
	u8 buffer[2];
//    u8 reg_value[1];
//	u8 change_again = 0;
//	unsigned long als_value;
//	int dark_light = 0;
	int res = 0;

	if(nAlsReadCount <10)
	{
		nAlsReadCount +=1;
	}
	if(client == NULL || data == NULL )
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	res = apds9920_read_reg(client, APDS9920_CMM_STATUS, buffer, 0x1);
	if(res <= 0)
	{
		    goto EXIT_ERR;
	}
	 
	res = 1;
	if(!(buffer[0] & STATUS_AVALID))	//make sure ALS valid;
	{
		als_read_count++;
		if(als_read_count < 20)
		{ 
			printk(KERN_INFO TAGI " %s, als is invalid!\n", __func__);
		}
		else
		{
			als_read_count = 0;
			printk(KERN_ERR TAGE " %s, als is invalid!\n", __func__);
		}
		return -1;
	}
	als_read_count = 0;

    //get adc channel 0 value
	buffer[0]=APDS9920_CMM_C0DATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)(&c0_value), 0x2);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
	if(debug)
		printk(KERN_INFO TAGI "c0_value=%d\n", c0_value);

	//get adc channel 1 value
    buffer[0]=APDS9920_CMM_C1DATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)(&c1_value), 0x2);
    if(res <= 0)
    {
        goto EXIT_ERR;
	}
    if(debug)
		printk(KERN_INFO TAGI "c1_value=%d\n", c1_value);

    //als mode switch
	if(obj->als_mode == ALS_LOW_LUX_MODE && c0_value >= (apds9920_als_res_tb[obj->als_atime_index] * 85 / 100)) {
		if(res <= 0) {
			res = APDS9920_ERR_I2C;
			goto EXIT_ERR;
		}
		buffer[0] = APDS9920_CMM_CONTROL;
		buffer[1] = 0x20 | ALS_NORMAL_LUX_SETTING;
		obj->als_modulus = 1;
		res = apds9920_write_reg(client, buffer[0], &buffer[1], 0x1);
		if(res <= 0) {
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI "apds9920_read_als switch to normal mode c0_value=%d prv_lux=%d\n", c0_value, obj->last_als_data);

		obj->als_mode = ALS_NORMAL_LUX_MODE;
		if(obj->last_als_data == -1)
			return -1;
		else{
                 *data = obj->last_als_data;
			return 0;
		}
	} else if(obj->als_mode == ALS_NORMAL_LUX_MODE && c0_value <= (apds9920_als_res_tb[obj->als_atime_index] * 5 / 100)) {
		buffer[0] = APDS9920_CMM_CONTROL;
		if(c0_value < 10) {
			buffer[1] = 0x20 | 0x03;
			obj->als_modulus = 120;
		} else {
			buffer[1] = 0x20 | ALS_LOW_LUX_SETTING;
			obj->als_modulus = 16;
		}
		res = apds9920_write_reg(client, buffer[0], &buffer[1], 0x1);
		if(res <= 0) {
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI "apds9920_read_als switch to low mode c0_value=%d prv_lux=%d\n", c0_value, obj->last_als_data);
		obj->als_mode = ALS_LOW_LUX_MODE;
		if(obj->last_als_data == -1)
			return -1;
		else{
                 *data = obj->last_als_data;
			return 0;
		}
		
	}
#if 1
	if(!is_short_atime_mode){
		if(c0_value >= (apds9920_als_res_tb[obj->als_atime_index] * 85 / 100) && obj->als_last_atime_index == 1) {
			obj->als_atime_index = 0;
			obj->als_last_atime_index = obj->als_atime_index;
			buffer[0] = APDS9920_CMM_ATIME;
			buffer[1] = apds9920_als_atime_tb[obj->als_atime_index];               //set to ms
			res = apds9920_write_reg(client, buffer[0], &buffer[1], 0x1);
			if(res <= 0) {
				res = APDS9920_ERR_I2C;
				goto EXIT_ERR;
			}
			if(obj->last_als_data == -1)
				return -1;
			else{
                 		*data = obj->last_als_data;
				return 0;
			}
		} else if(c0_value <= (apds9920_als_res_tb[obj->als_atime_index] * 5 / 100) &&  obj->als_last_atime_index == 0 ) {
			obj->als_atime_index = 1;
			obj->als_last_atime_index = obj->als_atime_index;
			buffer[0] = APDS9920_CMM_ATIME;
			buffer[1] = apds9920_als_atime_tb[obj->als_atime_index];               //set to ms
			res = apds9920_write_reg(client, buffer[0], &buffer[1], 0x1);
			if(debug)
				printk(KERN_INFO TAGI "apds9920 atime = %d res = %d\r\n",apds9920_als_atime_tb[obj->als_atime_index],res);
			if(res <= 0) {
				res = APDS9920_ERR_I2C;
				goto EXIT_ERR;
			}
			if(obj->last_als_data == -1)
				return -1;
			else{
                 		*data = obj->last_als_data;
				change = 2;
				return 0;
			}
		}
	}
#endif
	if(change > 0)
	{
		*data = obj->last_als_data;
		change--;
		return 0; 
	}

	luxValue = LuxCalculation(client, c0_value, c1_value);
	if (luxValue >= 0) {
		luxValue = (luxValue < APDS9920_MAX_LUXVALUE) ? luxValue : APDS9920_MAX_LUXVALUE;
	}
	if(luxValue > 5 && luxValue < 1000) {
		luxValue = 1010;
	}
	*data = luxValue / 1000;
	*data = (apds9920_als_para[als_para_num].base_value * ( *data) ) / 100;
	if(debug) {
		printk(KERN_INFO TAGI "apds9920 als final data = %d\r\n", *data);
	}

	return 0;	 
    
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_read_als fail\n");
	return res;
}


#if 0
int apds9920_read_als1(struct i2c_client *client, u16 *data)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);	 
	u16 c1_value,c0_value;	 
	u32 c0_nf, c1_nf;
	//u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
    u8 databuf[2];
	u16 ratio;
	unsigned long als_value;
    int dark_light=0;
	int res = 0;

	if(nAlsReadCount <10)
	{
		nAlsReadCount +=1;
	}
	buffer[0]=APDS9920_CMM_C0DATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)(&c0_value), 0x2);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    c0_nf = obj->als_modulus*c0_value;
	if(debug)
		printk(KERN_INFO TAGI "c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

	//get adc channel 1 value
    buffer[0]=APDS9920_CMM_C1DATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)(&c1_value), 0x2);
    if(res <= 0)
    {
        goto EXIT_ERR;
	}
    c1_nf = obj->als_modulus*c1_value;	
    if(debug)
		printk(KERN_INFO TAGI "c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

    //als mode switch
    if(obj->als_mode == ALS_LOW_LUX_MODE && c0_value >= ALS_LUX_HIGH_THRESHOLD){
		databuf[0] = APDS9920_CMM_CONTROL; 
        databuf[1] = 0x20 | ALS_NORMAL_LUX_SETTING;
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
            goto EXIT_ERR;
        
		printk(KERN_ERR TAGE "apds9920_read_als switch to normal mode c0_value=%d prv_lux=%d\n", c0_value, obj->last_als_data);
		
        obj->als_mode = ALS_NORMAL_LUX_MODE;
        obj->als_modulus = (400*100)/(1*DEFAULT_ATIME);
		*data = obj->last_als_data;
        
		return 0;
	}
	else if(obj->als_mode == ALS_NORMAL_LUX_MODE && c0_value <= ALS_LUX_LOW_THRESHOLD){
        databuf[0] = APDS9920_CMM_CONTROL; 
		databuf[1] = 0x20 | ALS_LOW_LUX_SETTING;
		res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
            goto EXIT_ERR;

		printk(KERN_ERR TAGE "apds9920_read_als switch to low mode c0_value=%d prv_lux=%d\n", c0_value, obj->last_als_data);

		obj->als_mode = ALS_LOW_LUX_MODE;
		obj->als_modulus = (400*100)/(16*DEFAULT_ATIME);
		*data = obj->last_als_data;
        
		return 0;
	}


	//caculate the light value lux
	/*begin liujie 2011-11-5 modify*/
    if (c0_value < c1_value)
	{
        printk(KERN_INFO TAGI "APDS9920_CHIP als_value is invalid!!\n");
        return -1;
    }
	else if (c0_value > 17500)   //19456 * 0.9
	{
        obj->last_als_data = 65535;
        *data =  obj->last_als_data;//65535;
		return 0;
	}
	
    if ( 0 == c0_nf )
    {
        *data = 0;
        printk(KERN_INFO TAGI "0==c0_nf so output 0 \n");
	    return 0;
	}

	ratio = (c1_nf*100) / c0_nf;
	if(ratio<30)
	{
        als_value = (13*c0_nf - 24*c1_nf)/10;     //mlux

		if(als_value > 5 && als_value < 800) {
			als_value = 1010;
			dark_light=1;
		}
		else if(als_value >= 800 && als_value < 1000)
		{
			als_value = 1010;
		}
		
		als_value /= 1000;
	}
    else if(ratio>=30 && ratio<38)
    { 
        als_value = (16*c0_nf - 35*c1_nf)/10;     //mlux

        if(als_value > 5 && als_value < 800) {
			als_value = 1010;
			dark_light=1;
		}
		else if(als_value >= 800 && als_value < 1000)
		{
			als_value = 1010;
		}
		als_value /= 1000;
    }
	else if(ratio>=38 && ratio<45)
	{ 
        als_value = (9*c0_nf - 17*c1_nf)/10;      //mlux

        if(als_value > 5 && als_value < 800) {
			als_value = 1010;
			dark_light=1;
		}
		else if(als_value >= 800 && als_value < 1000)
		{
			als_value = 1010;
		}
		als_value /= 1000;
	}
	else if(ratio>=45 && ratio<=58) 
	{ 
		als_value = (6*c0_nf - 10*c1_nf)/10;      //mlux

        if(als_value > 5 && als_value < 800) {
			als_value = 1010;
			dark_light=1;
		}
		else if(als_value >= 800 && als_value < 1000)
		{
			als_value = 1010;
		}
		als_value /= 1000;
	}
	else
	{
        als_value = obj->last_als_data;
        if(debug)
			printk(KERN_INFO TAGI "als_data is abnormal so output 0 ratio=%d \n",ratio);
	}
    *data = als_value;
    
	if(debug)
		printk(KERN_INFO TAGI "APDS9920_CHIP als_value_lux(0) = %d ratio=%d\n", *data,ratio);	//no ci

    if(*data == 1 && dark_light == 1){
		obj->last_als_data = *data;
	}
	else{
        if(*data == 1)
            obj->last_als_data = (100 * ( *data) )/100;
        else
		    obj->last_als_data = (apds9920_als_para[als_para_num].base_value * ( *data) )/100;
		*data = (apds9920_als_para[als_para_num].base_value * ( *data) )/100;
	}
	
    if(debug)
		printk(KERN_INFO TAGI "APDS9920_CHIP als_value_lux(1) = %d\n", *data);//no ci
	return 0;	 
    
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_read_als fail\n");
	return res;
}
#endif

int apds9920_read_als(struct i2c_client *client, u16 *data)
{ 
	int res = 0;
	// #if defined(PD1304CL) || defined(PD1304CLG4) || defined(PD1304CV) || defined(PD1304CF)
      //   	res = apds9920_read_als1(client,data);
	// #else
		res = apds9920_read_als0(client,data);
	// #endif
	 return res;
}
/*----------------------------------------------------------------------------*/
/*static int apds9920_get_als_value(struct apds9920_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		printk(KERN_ERR TAGE "exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//printk(KERN_INFO TAGI "ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		printk(KERN_ERR TAGE "ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}*/
int apds9920_get_offset(struct i2c_client *client,u8 *data)
{
	// struct apds9920_priv *obj = i2c_get_clientdata(client);	
	u16 ps_value;	 
	u8 i = 0;
	u8 buffer[1];
	u8 databuf[2];
	int res = 0;
	
	if(client == NULL)
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	buffer[0]=APDS9920_CMM_PDATA_L;
	  databuf[0] = APDS9920_CMM_OFFSET;
	  for (i = 0; i < 127; i ++ )
		{
			   res = apds9920_read_reg(client, buffer[0], (u8 *)&ps_value, 0x2);
		   printk(KERN_INFO TAGI "apds9920 offset ps_value = %d\r\n",ps_value);
		   if(res <= 0)
		   {
			goto EXIT_ERR;
		   }
		 if(ps_value > 450)
		 {
			   databuf[1] = i +1;
			 res = apds9920_write_reg(client, databuf[0], (u8 *)&databuf[1], 0x1);
			   printk(KERN_INFO TAGI "set APDS9920 prx offset reg:0x%02x=0x%02x\n",databuf[0],databuf[1]);
			   if(res <= 0)
			   {
					 goto EXIT_ERR;
			   }
			 msleep(1);
		 }
		 else if (ps_value < 15)
		 {
			  databuf[1] = i +128;
			 res = apds9920_write_reg(client, databuf[0], (u8 *)&databuf[1], 0x1);
			   printk(KERN_INFO TAGI "set APDS9920 prx offset reg:0x%02x=0x%02x\n",databuf[0],databuf[1]);
			   if(res <= 0)
			   {
					 goto EXIT_ERR;
			   }
			 msleep(1);
		 }
		 else
		 {
			  break;
		 }
		 *data = databuf[1];
		 printk(KERN_INFO TAGI "apds9920 offset = %d\r\n",*data);
		}
	
	return 0;
	EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/

int apds9920_read_ps(struct i2c_client *client, u16 *data)
{
	//struct apds9920_priv *obj = i2c_get_clientdata(client);    
	u16 ps_value;    
	//u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	//static int count = 0;
	int res = 0;
	
	if(client == NULL)
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	res = apds9920_read_reg(client, APDS9920_CMM_STATUS, buffer, 0x1);
	if(res <= 0)
	{
		    goto EXIT_ERR;
	}
	 
	res = 1;
	if(!(buffer[0] & STATUS_PVALID))	//make sure PS valid;
	{
		ps_read_count++;
		if(ps_read_count < 20)
		{
			printk(KERN_INFO TAGI " %s, ps is invalid!\n", __func__);
		}
		else
		{
			ps_read_count = 0;
			printk(KERN_ERR TAGE " %s, ps is invalid!\n", __func__);
		}
		return -1;
	}
	ps_read_count = 0;

	buffer[0]=APDS9920_CMM_PDATA_L;
    res = apds9920_read_reg(client, buffer[0], (u8 *)(&ps_value), 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	/*
	buffer[0]=APDS9920_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	*/
	*data=ps_value;



	if ( ps_value != 0 && ps_value != 1023 )
	{
		if( ps_value== last_ps_data ){
			ps_data_same_count +=1;
		}
		else {
			ps_data_same_count =0;
		}
		if (ps_data_same_count==PS_ESD_TIME){
			ps_data_same_count = 0;
			restart_ps_automatic=1;
			apds9920_enable_ps(client,0);
			restart_ps_automatic=0;
			mdelay(1);
			apds9920_enable_ps(client,1);
			printk(KERN_INFO TAGI "apds9920_read_ps init client obj->ps = 0\n");
		}
	}
	//equal 0 or 1023
	else  {
		if(last_ps_data!=ps_value){
			ps_data_same_count =0;
		}
	}
	last_ps_data=ps_value;
	
	if(debug)
	printk(KERN_INFO TAGI "apds9920_read_ps  obj->ps = %d \n",*data);
	return 0;    

EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9920_check_and_clear_intr(struct i2c_client *client) 
{
	int res,intp = 0, intl = 0;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	    //return 0;

	//buffer[0] = APDS9920_CMM_STATUS;
    res = apds9920_read_reg(client, APDS9920_CMM_STATUS, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
    
	//printk(KERN_INFO TAGI "yucong apds9920_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	if(0 == res)
	{
        //clear the eint
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}
		
        res = apds9920_write_reg(client, buffer[0], NULL, 0x0);
		if(res <= 0)
			goto EXIT_ERR;
		else
			res = 0;
	}
    
    res = 0;
	return res;

EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_check_and_clear_intr fail\n");
	return 1;
}

/*----------------------------------------------------------------------------*/
/*static int apds9920_check_intr(struct i2c_client *client) 
{
    //struct apds9920_priv *obj = i2c_get_clientdata(client);
	int res, intp = 0, intl = 0;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1)
	    //return 0;

	//buffer[0] = APDS9920_CMM_STATUS;
    res = apds9920_read_reg(client, APDS9920_CMM_STATUS, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk(KERN_ERR TAGE "apds9920_check_and_clear_intr status=0x%x\n", buffer[0]);
    
	res = 1;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}
    
    res = 0;
	return res;

EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_check_intr fail\n");
	return 1;
}*/

/*----------------------------------------------------------------------------*/
static int apds9920_clear_intr(struct i2c_client *client) 
{
    //struct apds9920_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];
    
    //clear all the eint
    
	buffer[0] = (TAOS_TRITON_CMD_REG |TAOS_TRITON_CMD_SPL_FN |0x07);
    
    res = apds9920_write_reg(client, buffer[0], NULL, 0); 
	if(res <= 0)
		goto EXIT_ERR;
	else
		res = 0;
        printk(KERN_INFO TAGI "%s success \n",__func__);
	return res;

EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_check_and_clear_intr fail\n");
	return 1;
}

/*----------------------------------------------------------------------------*/
static int apds9920_ps_select_para(struct apds9920_priv *obj)         //
{
    int out_threshold, in_threshold;   
    int base = atomic_read(&obj->ps_base_value);
    int step_num;
    const struct ps_step *step;
    const struct ps_para *para;
    
    //TODO: get different parameter here
    para = &apds9920_ps_para[ps_para_num];
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
static int apds9920_als_select_para(struct apds9920_priv *obj)
{
    //TODO: get different parameter here
    //als_para_num = 0; 

    return 0; 
}

/*-----------------------------------------------------------------------------*/
static int apds9920_get_ps_value(struct apds9920_priv *obj, u16 ps)  
{
	int val;
	//static int val_temp = 1;   //default is far away

    //printk(KERN_INFO TAGI "ps_in_threshold %d ps_out_threshold %d\n", atomic_read(&obj->ps_in_threshold), atomic_read(&obj->ps_out_threshold));
	//int ps_status_label = atomic_read(&obj->ps_status);
    
    mutex_lock(&ps_value_lock);
    
	if( ps > atomic_read(&obj->ps_in_threshold) ){
		val = 0;
	}
	else if( ps < atomic_read(&obj->ps_out_threshold) ){
		val = 1;
	}
	else{
		val=atomic_read(&obj->ps_status);
	}
	
    /*
    if(atomic_read(&obj->ps_suspend))
	{
	printk(KERN_INFO TAGI "%s,ps_suspend return \n",__func__);
        return -1;
	}

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

        atomic_set(&obj->ps_first_int, 0);
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
    if(ps_status_label !=atomic_read(&obj->ps_status))
    {
        printk(KERN_INFO TAGI "%s,ps_status_label = %d  ps_status=%d ps_value= %d in_threshold=%d out_threshold=%d\n",__func__,
            ps_status_label,atomic_read(&obj->ps_status),ps,atomic_read(&obj->ps_in_threshold),atomic_read(&obj->ps_out_threshold));
    }
    */
    if(debug)
    {
        printk(KERN_INFO TAGI "%s,ps_value = %d  ps_status=%d \n",__func__,ps,atomic_read(&obj->ps_status));
    }
	if( val != atomic_read(&obj->ps_status) )
	{
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		printk(KERN_INFO TAGI"change ps: %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
		printk(KERN_INFO TAGI "%s change ps(%d) status from %d to %d,in=%d out=%d\n",
		    __func__,ps,atomic_read(&obj->ps_status),val,atomic_read(&obj->ps_in_threshold),atomic_read(&obj->ps_out_threshold));
		    atomic_set(&obj->ps_status,val);



		if(test_bit(CMC_BIT_PS,&obj->enable) || test_bit(CMC_BIT_ENG_PS,&obj->enable) ){
			apds9920_enable_eint(obj,1);
		}
	}

    mutex_unlock(&ps_value_lock);
	return val;

}

/*-----------------------------------------------------------------------------*/
static int apds9920_enable_als_moniter_eint(struct i2c_client *client, int enable)      //the eint will be triggered in strong lux
{
    struct apds9920_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];
    
    databuf[0] = 0x84;	
	databuf[1] = (u8)(0 & 0x00FF);
    res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
        goto EXIT_ERR;

	databuf[0] = 0x85;	
	databuf[1] = (u8)((0 & 0xFF00) >> 8);
    res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
    if(res <= 0)
        goto EXIT_ERR;
				
    databuf[0] = 0x86;	
    databuf[1] = (u8)(obj->strong_sunlight & 0x00FF);
    res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
    if(res <= 0)
        goto EXIT_ERR;

    databuf[0] = 0x87; 
    databuf[1] = (u8)((obj->strong_sunlight & 0xFF00) >> 8);
    res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
    if(res <= 0)
        goto EXIT_ERR;
		
    //read the enable reg agian
    buffer[0]=APDS9920_CMM_ENABLE;
    res = apds9920_read_reg(client, buffer[0], reg_value, 0x1);
    if(res <= 0)
        goto EXIT_ERR;
            
    reg_value[0] = reg_value[0] & 0x3f;
    printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n",__func__,__LINE__,reg_value[0]);
    //enable als eint
    if(enable)
    {
        databuf[0] = APDS9920_CMM_ENABLE;
        //databuf[1] = reg_value[0] | 0x10;
        databuf[1] = reg_value[0] | 0x3f;
        printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n",__func__,__LINE__,databuf[1]);
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
        if(res <= 0)
            goto EXIT_ERR;
    }
    else
    {
      	databuf[0] = APDS9920_CMM_ENABLE;
		if(test_bit(CMC_BIT_ALS,&obj->enable) || test_bit(CMC_BIT_ENG_ALS, &obj->enable) ){

//			printk(KERN_INFO TAGI "CMC_BIT_ALS = %d, CMC_BIT_ENG_ALS = %d\n", test_bit(CMC_BIT_ALS,&obj->enable), test_bit(CMC_BIT_ENG_ALS, &obj->enable));
			databuf[1] = reg_value[0] | 0x0b; // 00001011 assert WEN AEN PON 
		}
		if(test_bit(CMC_BIT_PS,&obj->enable) || test_bit(CMC_BIT_ENG_PS, &obj->enable) ){

//			printk(KERN_INFO TAGI "CMC_BIT_PS = %d, CMC_BIT_ENG_PS = %d\n",test_bit(CMC_BIT_PS,&obj->enable), test_bit(CMC_BIT_ENG_PS, &obj->enable)); 
			databuf[1] = reg_value[0] | 0x2d; // 00101101 assert PIEN WEN PEN PON
		}

//		printk(KERN_INFO TAGI "databuf[1] = %x\n", databuf[1]);
		databuf[1] = databuf[1] & 0xEF; //clear AIEN

        printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n",__func__,__LINE__,databuf[1]);
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
        if(res <= 0)
            goto EXIT_ERR;
    }

    return 0;

EXIT_ERR:
    return 1;
}

/*-----------------------------------------------------------------------------*/
static irqreturn_t apds9920_irq(int irq, void *data)
{
	struct apds9920_priv *obj = data;
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

/*----------------------------------------------------------------------------*/
static void apds9920_eint_work(struct work_struct *work)
{
	struct apds9920_priv *obj = (struct apds9920_priv *)container_of(work, struct apds9920_priv, eint_work);
    //hwm_sensor_data sensor_data;
	printk(KERN_INFO TAGI "apds9920_eint_work\n");

    if(!test_bit(CMC_BIT_PS, &obj->enable)|| test_bit(CMC_BIT_ENG_PS,&obj->enable)) {
        apds9920_clear_intr(obj->client);
        enable_irq(obj->irq);
        return;
	}
    
    wake_lock_timeout(&ps_suspend_lock, 2 * HZ);    //wake up for 3s
//	printk(KERN_INFO TAGI "apds9920_eint_work\n");

	apds9920_clear_intr(obj->client);

    if(atomic_read(&obj->ps_suspend))
        msleep(10);     //10ms 
    
    apds9920_ps_data_report(&(obj->ps_report_work.work));

	enable_irq(obj->irq);

	queue_delayed_work(obj->poll_queue, &obj->ps_report_work, 0);
}

/*----------------------------------------------------------------------------*/
int apds9920_setup_eint(struct i2c_client *client)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);        
    int err;

	g_apds9920_ptr = obj;
    if (!obj->irq)
    {
        if(gpio_is_valid(obj->gpio_int)) 
        {
		    err = gpio_request(obj->gpio_int, "apds9920_int");
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
    
	    //gpio_free(obj->gpio_int);
        obj->irq = gpio_to_irq(obj->gpio_int);
        if(obj->irq)
        {
            err = request_any_context_irq(obj->irq, apds9920_irq, (IRQF_TRIGGER_LOW | IRQF_ONESHOT), 
                            "APDS9920_i2c", obj);
            if (err < 0) {
			    printk(KERN_ERR TAGE "request irq failed: %d\n", err);
                return err;
		    }

            enable_irq_wake(obj->irq);

            //disable_irq_nosync(obj->irq); 
        }
    }
         

    return 0;

err_irq_gpio_req:
	gpio_free(obj->gpio_int);
	return err;
}

/*----------------------------------------------------------------------------*/
int apds9920_get_addr(struct alsps_hw *hw, struct apds9920_i2c_addr *addr,int type)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	if (type!=0 && type!=1){
	    printk(KERN_INFO TAGI "[%s] type =%d this must wrong!\n",__func__,type);
	}
	
	addr->write_addr = hw->i2c_addr[type];
	printk(KERN_INFO TAGI "[%s] addr->write_addr = 0x%2x,type = %d\n",__func__,addr->write_addr,type);
	return 0;
}

/*----------------------------------------------------------------------------*/
/*static int apds9920_set_enable(struct i2c_client *client, int enable)
{
	//struct apds9920_priv *obj = i2c_get_clientdata(client);
	int ret=0;
	u8 databuf[2];
	u8 reg_value[1];

	databuf[0] = APDS9920_CMM_ENABLE;
	databuf[1] = enable;
    ret = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if ( ret < 0 )
	{
		printk(KERN_INFO TAGI "%s -- %d \n",__func__,__LINE__);
	}

    //read CTL register
	databuf[0] = APDS9920_CMM_ENABLE;
    ret = apds9920_read_reg(client, databuf[0], reg_value, 0x1);
	if(ret <= 0)
	{
		printk(KERN_INFO TAGI "%s -- %d \n",__func__,__LINE__);
	}
	printk(KERN_INFO TAGI "%s -- %d reg_value[0] = %d\n",__func__,__LINE__,reg_value[0]);		

	return ret;
}*/

/*----------------------------------------------------------------------------*/
static void apds9920_power(struct alsps_hw *hw, unsigned int on) 
{
	//static unsigned int power_on = 0;

	printk(KERN_INFO TAGI "power %s\n", on ? "on" : "off");
}

/*----------------------------------------------------------------------------*/
int apds9920_enable_als(struct i2c_client *client, int enable)
{
    struct apds9920_priv *obj = i2c_get_clientdata(client);
    u8 databuf[2];	  
    int res = 0;
    u8 buffer[1];
    u8 reg_value[1];
	
    if(client == NULL)
    {
        printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    buffer[0] = APDS9920_CMM_ENABLE;
    res = apds9920_read_reg(client, buffer[0], reg_value, 0x1);
	if(res <= 0)
	{
        goto EXIT_ERR;
	}
    reg_value[0]=reg_value[0]&0x3f;
    printk(KERN_INFO TAGI "apds9920_enable_als  reg_value = %d chip_id = %d \n",reg_value[0],chip_id);
    if(enable)
    {
        databuf[0] = APDS9920_CMM_ENABLE;
        databuf[1] = reg_value[0] |0x0B;
	    if ( test_bit(CMC_BIT_PS , &obj->enable ) || test_bit(CMC_BIT_ENG_PS,&obj->enable) )
            databuf[1] = databuf[1] | 0x04;//ensure ps be enabled stutas
        else
	        databuf[1] = databuf[1] & 0xfb;//ensure ps be enabled stutas
		printk(KERN_INFO TAGI "apds9920_enable_als enable databuf[1] = 0x%02x \n",databuf[1]);	
            printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n",__func__,__LINE__,databuf[1]);
        
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
		{
            goto EXIT_ERR;
        }
        atomic_set(&obj->als_deb_on, 1);
        atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
        printk(KERN_INFO TAGI "apds9920 enable als\n");	

        apds9920_als_select_para(obj);
	}
	else
    {
        databuf[0] = APDS9920_CMM_ENABLE;	
        databuf[1] = reg_value[0] &0xFD;
        if ( test_bit(CMC_BIT_PS , &obj->enable ) )
            databuf[1] = databuf[1] | 0x04;//ensure ps be enable stutas
        else {
			//changed 0xfb to 0xfa: clear bit PON ,as it mey  interference FM : PD1216
            databuf[1] = databuf[1] & 0xfa;//ensure ps be disable stutas
        }
        printk(KERN_INFO TAGI "apds9920_enable_als disable databuf[1] = 0x%02x \n",databuf[1]);
        printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n",__func__,__LINE__,databuf[1]);

        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }			
        atomic_set(&obj->als_deb_on, 0);
        printk(KERN_INFO TAGI "apds9920 disable als\n");
	}
	alsCount = 0;
	return 0;
		
	EXIT_ERR:
		printk(KERN_ERR TAGE "apds9920_enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
int apds9920_enable_ps(struct i2c_client *client, int enable)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];
    //static int first_in = 1;

	printk(KERN_INFO TAGI " apds9920_enable_ps begin\n");
	if(client == NULL)
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	stronglight_flag = 0;
	buffer[0]=APDS9920_CMM_ENABLE;
    res = apds9920_read_reg(client, buffer[0], reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	printk(KERN_INFO TAGI " apds9920_enable_ps reg_value = %d chip_id = %d \n",reg_value[0],chip_id);	
    
    reg_value[0]=reg_value[0]&0x3f;
	if(enable)
	{
        /*ÉèÖÃÒ»ÏÂPON*/
        apds9920_init_client(client);    //set pon again 
        
		databuf[0] = APDS9920_CMM_ENABLE;
		databuf[1] = reg_value[0] | 0x0d;
		//if ( test_bit(CMC_BIT_ALS , &obj->enable ) || obj->enable_als_by_ps )
			databuf[1] = databuf[1] | 0x02;//ensure als be enabled stutas
		//else
            //databuf[1] = databuf[1] & 0xfd;//ensure als be disabled stutas
		printk(KERN_INFO TAGI "apds9920_enable_ps enalbe databuf[1] = 0x%02x \n",databuf[1]);
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		printk(KERN_INFO TAGI "apds9920 power on\n");

        //int the eint
	    if(0 == obj->hw->polling_mode_ps)
		{
            databuf[0] = APDS9920_CMM_INT_LOW_THD_LOW;	
	        databuf[1] = (u8)((atomic_read(&obj->ps_in_threshold) - 1) & 0x00FF);
			res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
			if(res <= 0)
                goto EXIT_ERR;

			databuf[0] = APDS9920_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)(((atomic_read(&obj->ps_in_threshold) - 1) & 0xFF00) >> 8);
			res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
			if(res <= 0)
				goto EXIT_ERR;
				
			databuf[0] = APDS9920_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(atomic_read(&obj->ps_in_threshold) & 0x00FF);
			res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
			if(res <= 0)
				goto EXIT_ERR;

			databuf[0] = APDS9920_CMM_INT_HIGH_THD_HIGH; 
			databuf[1] = (u8)((atomic_read(&obj->ps_in_threshold) & 0xFF00) >> 8);;
			res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
			if(res <= 0)
				goto EXIT_ERR;
		
			databuf[0] = APDS9920_CMM_Persistence;
			databuf[1] = 0x11;
			res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
			if(res <= 0)
				goto EXIT_ERR;
            
            //read the enable reg agian
            buffer[0]=APDS9920_CMM_ENABLE;
            res = apds9920_read_reg(client, buffer[0], reg_value, 0x1);
	        if(res <= 0)
		        goto EXIT_ERR;
            
            reg_value[0] = reg_value[0] & 0x3f;
            printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n",__func__,__LINE__,reg_value[0] );

            //enable eint
            databuf[0] = APDS9920_CMM_ENABLE;
		    databuf[1] = reg_value[0] | 0x2f;//must set f ,make sure ps enabled
            printk(KERN_INFO TAGI "%s--%d--APDS9920_CMM_ENABLE value = 0x%02x\n", __func__,__LINE__,databuf[1] );
            res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		    if(res <= 0)
			    goto EXIT_ERR;
            
            //slect the threshold here
            apds9920_ps_select_para(obj);
			atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
			atomic_set(&obj->ps_status,1); //1: stand for: far away
			apds9920_enable_eint(obj,0);
			apds9920_enable_eint(obj,1);
		}	
	}
	else
	{
		databuf[0] = APDS9920_CMM_ENABLE;
		databuf[1] = reg_value[0] &0xfb;
		if ( test_bit(CMC_BIT_ALS , &obj->enable ) || restart_ps_automatic==1 )
			databuf[1] = databuf[1] | 0x02;//ensure als be enabled stutas
		else {
			//change 0xfd to 0xfc : set poweroff . as bit PON interference FM :PD1216 dengweicheng
			databuf[1] = databuf[1] & 0xfc;//ensure als be disabled stutas 

		}
        databuf[1] = databuf[1] & 0xcf;

		printk(KERN_INFO TAGI "apds9920_enable_ps disable databuf[1] = 0x%02x \n",databuf[1]);
		res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		printk(KERN_INFO TAGI "apds9920 power off\n");
        
        atomic_set(&obj->ps_first_int, 1);
        atomic_set(&obj->ps_status, 1);          //default is faraway
		atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
        if(0 == obj->hw->polling_mode_ps)
		{
			/*disable_irq_nosync(ob->irq);*/
        }
        apds9920_enable_eint(obj,0);
	}
	return 0;
	
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_enable_ps fail\n");
	return -1;
}

/*----------------------------------------------------------------------------*/
static int apds9920_enable(struct i2c_client *client, int enable)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	//u8 buffer[1];
	//u8 reg_value[1];

	if(client == NULL)
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	if(enable)
	{
		databuf[0] = APDS9920_CMM_ENABLE;    
		databuf[1] = 0x01;
		res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
		{
		    printk(KERN_ERR TAGE "APDS9920_enable fail\n");
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI "apds9920 power on\n");
	}
	else
	{
		databuf[0] = APDS9920_CMM_ENABLE;    
		databuf[1] = 0x00;
		res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		if(res <= 0)
		{
		    printk(KERN_ERR TAGE "APDS9920_enable fail\n");
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		printk(KERN_INFO TAGI "apds9920 power off\n");
	}		

	return 0;
	
EXIT_ERR:
	printk(KERN_ERR TAGE "apds9920_enable fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9920_init_client(struct i2c_client *client)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;

    printk(KERN_INFO TAGI "[%s] --%d\n",__func__,__LINE__);
	databuf[0] = APDS9920_CMM_ENABLE;    
	databuf[1] = 0x01;
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9920_CMM_ATIME;    
	databuf[1] = apds9920_als_atime_tb[obj->als_atime_index];              //set to 50ms
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;

	}

	databuf[0] = APDS9920_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;

	}

	databuf[0] = APDS9920_CMM_WTIME;    
	databuf[1] = 0xFF;                //set to 2.73ms
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;

	}

	databuf[0] = APDS9920_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;

	}
    
	databuf[0] = APDS9920_CMM_PPCOUNT;   
	if(strstr(bbk_product_model_for_alsps,"PD1421CA"))
	{ 
		databuf[1] = 0x08;
	}
	else if(strstr(bbk_product_model_for_alsps,"PD1523A"))
	{
		databuf[1] = 0x04;
	}
	else
	{
		databuf[1] = 0x04;
	}
    
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;

	}

	databuf[0] = APDS9920_CMM_CONTROL;    
	databuf[1] = 0x20 | 0x02;   // 100mA TMD2772_AGAIN_16x
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9920_CMM_OFFSET;    
	databuf[1] = obj->prox_offset;   
	res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
	if(res <= 0)
	{
        res = APDS9920_ERR_I2C;
		goto EXIT_ERR;
	}		

    obj->als_modulus = 16;//(400*100)/(16*DEFAULT_ATIME);      //(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value 
    obj->als_mode =  ALS_LOW_LUX_MODE; 

	if((res = apds9920_setup_eint(client)) != 0)
	{
		printk(KERN_ERR TAGE "setup eint: %d\n", res);
		return res;
	}

	if((res = apds9920_check_and_clear_intr(client)))
	{
		printk(KERN_ERR TAGE "check/clear intr: %d\n", res);
		return res;
	}
    
	return APDS9920_SUCCESS;

EXIT_ERR:
	printk(KERN_ERR TAGE "init dev: %d\n", res);
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9920_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9920_i2c_client;

	if (!file->private_data)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int apds9920_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long apds9920_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct apds9920_priv *obj = i2c_get_clientdata(client);  
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
					apds9920_enable_als(obj->client,1);
					if((err = apds9920_enable_ps(obj->client, 1)))
					{
						printk(KERN_ERR TAGE "enable ps fail: %d\n", err); 
						goto err_out;
					}
					apds9920_enable_eint(obj,0);
					apds9920_enable_eint(obj,1);
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
					if((err = apds9920_enable_ps(obj->client, 0)))
					{
						printk(KERN_ERR TAGE "disable ps fail: %d\n", err); 
						goto err_out;
					}
					apds9920_enable_eint(obj,0);
				}
				if(!test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable)){
					apds9920_enable_als(obj->client,0);
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
			if((err = apds9920_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
	/*begin liujie 2011-11-16 modify*/		
			dat = apds9920_get_ps_value(obj, obj->ps);
	       // dat = obj->ps;
	/*end liujie 2011-11-16 modify*/			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = apds9920_read_ps(obj->client, &obj->ps)))
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
					if((err = apds9920_enable_als(obj->client, 1)))
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
					if((err = apds9920_enable_als(obj->client, 0)))
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
			if((err = apds9920_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}
/*begin liujie 2011-11-16 modify*/
	//		dat = apds9920_get_als_value(obj, obj->als);			
/*end liujie 2011-11-16 modify*/	
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = apds9920_read_als(obj->client, &obj->als)))
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
			if((err = apds9920_get_offset(obj->client, &obj->prox_offset)))
				{
					goto err_out;
				}
			dat = obj->prox_offset;
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
static struct file_operations apds9920_fops = {
	.owner = THIS_MODULE,
	.open = apds9920_open,
	.release = apds9920_release,
	.unlocked_ioctl = apds9920_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice apds9920_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &apds9920_fops,
};
int apds9920_enable_eint(struct apds9920_priv *obj,int enable){
	int res;
	unsigned char databuf[2];
	if(enable){
        if(!atomic_read(&obj->ps_status))        //note: intr_flag_value only chenged in apds9920_get_ps_value
        {
            printk(KERN_INFO TAGI "wait far away\n");
            databuf[0] = APDS9920_CMM_INT_LOW_THD_LOW;	
            databuf[1] = (u8)((atomic_read(&obj->ps_out_threshold)) & 0x00FF);
            res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9920_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)(((atomic_read(&obj->ps_out_threshold)) & 0xFF00) >> 8);
			res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9920_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(0x00FF);
			res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9920_CMM_INT_HIGH_THD_HIGH; 
			databuf[1] = (u8)((0xFF00) >> 8);;
			res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);
                
            //if is in close state must enable als moniter
            apds9920_enable_als_moniter_eint(obj->client, 1);
		}
		else
        {	
            printk(KERN_INFO TAGI "wait close\n");
            databuf[0] = APDS9920_CMM_INT_LOW_THD_LOW;	
            databuf[1] = (u8)(0 & 0x00FF);
            res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9920_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((0 & 0xFF00) >> 8);
			res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9920_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)((atomic_read(&obj->ps_in_threshold)) & 0x00FF);
			res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

			databuf[0] = APDS9920_CMM_INT_HIGH_THD_HIGH; 
			databuf[1] = (u8)(((atomic_read(&obj->ps_in_threshold)) & 0xFF00) >> 8);;
			res = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

            apds9920_enable_als_moniter_eint(obj->client, 0);
		}
        
		apds9920_clear_intr(obj->client);
        enable_irq(obj->irq);
		return 0;
	}
	else
	{
		apds9920_clear_intr(obj->client);//first, clear ps interrupt;
		printk(KERN_INFO TAGI "disabling apds9920 eint\n");
        //disable_irq_nosync(obj->irq);
        apds9920_enable_als_moniter_eint(obj->client, 0);
        return 0;
	}

    return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9920_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);    
	int err;
	//int ps_temp;
	//bool curren_prox_status;
	//unsigned char databuf[2];
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);

	if(!obj)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return -EINVAL;
	}
    printk(KERN_INFO TAGI "%s CMC_BIT_PS = %d\n ",__func__,test_bit(CMC_BIT_PS, &obj->enable));
    if(test_bit(CMC_BIT_PS, &obj->enable) && !obj->hw->polling_mode_ps)
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
//	if(msg.event == PM_EVENT_SUSPEND)
//	{   

		
//		atomic_set(&obj->als_suspend, 1);
		if((err = apds9920_enable_als(client, 0)))
		{
			printk(KERN_ERR TAGE "disable als: %d\n", err);
			return err;
		}

//		atomic_set(&obj->ps_suspend, 1);
        	printk(KERN_INFO TAGI "%s-----disable ps\n",__func__);
		if((err = apds9920_enable_ps(client, 0)))
		{
			printk(KERN_ERR TAGE "disable ps:  %d\n", err);
			return err;
		}

		//force power_down mode for sleep. modifed by xiaot. 
		apds9920_enable(client, 0);
		//xiaot_end

		apds9920_power(obj->hw, 0);
//	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9920_i2c_resume(struct i2c_client *client)
{
	struct apds9920_priv *obj = i2c_get_clientdata(client);        
	int err;
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);
    
	if(!obj)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return -EINVAL;
	}
//    atomic_set(&obj->als_suspend, 0);
    if (test_bit(CMC_BIT_PS, &obj->enable) && !obj->hw->polling_mode_ps)// interrupt
    {
        queue_delayed_work(obj->poll_queue, &obj->ps_report_work, msecs_to_jiffies(10));     //10ms
        atomic_set(&obj->ps_suspend, 0);
		printk(KERN_INFO TAGI"%s----queue_delay_work", __func__);
	    return 0;
	}
    
	apds9920_power(obj->hw, 1);
	if((err = apds9920_init_client(client)))
	{
		printk(KERN_ERR TAGE "initialize client fail!!\n");
		return err;
	}

//	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{

		if((err = apds9920_enable_als(client, 1)))
		{
			printk(KERN_ERR TAGE "enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
        printk(KERN_INFO TAGI "%s-----enable ps\n",__func__);
		if((err = apds9920_enable_ps(client, 1)))
		{
			printk(KERN_ERR TAGE "enable ps fail: %d\n", err);                
		}
	}

	return 0;
}

static void apds9920_fb_workfunc(struct work_struct *work)
{
	struct apds9920_priv *obj = apds9920_obj;
	if(blank == NULL)
	   return;
	printk(KERN_INFO TAGI"%s blank:%d\n",__func__,*blank);
	if (*blank == FB_BLANK_UNBLANK)
	{
		printk(KERN_INFO TAGI "apds9920_fb_workfunc resume\n");
		if( !is_prox_across_enable )
		{
			enable_als_short_atime_mode(obj,0);
		}
	}else if (*blank == FB_BLANK_POWERDOWN)
	{
		printk(KERN_INFO TAGI "apds9920_fb_workfunc suspend\n");
		if(test_bit(CMC_BIT_PS, &obj->enable))
		{
			enable_als_short_atime_mode(obj,1);
		}
	}
}
//replace by mohuifu 20110630
#if defined(CONFIG_HAS_EARLYSUSPEND)
/*----------------------------------------------------------------------------*/
static void apds9920_early_suspend(struct early_suspend *h)
{ 
	struct apds9920_priv *obj = container_of(h, struct apds9920_priv, early_drv);
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
static void apds9920_late_resume(struct early_suspend *h)
{
	struct apds9920_priv *obj = container_of(h, struct apds9920_priv, early_drv);
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
static int apds9920_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	//int *blank;
	struct apds9920_priv *obj = container_of(self, struct apds9920_priv, fb_notif);

	printk(KERN_INFO TAGI "apds9920_fb_notifier_callback \n");

	if (evdata && evdata->data && event == FB_EVENT_BLANK && obj) {
		blank = evdata->data;
		queue_work(obj->apds9920_fb_workqueue,&obj->apds9920_fb_work); 
		if (*blank == FB_BLANK_UNBLANK)
		{
			printk(KERN_INFO TAGI "apds9920_fb_notifier_callback resume\n");
		}
		else if (*blank == FB_BLANK_POWERDOWN)
		{
			printk(KERN_INFO TAGI "apds9920_fb_notifier_callback suspend\n");
			wake_lock_timeout(&(obj->wakelock), 3*HZ); 
		}
	}

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
/*static int apds9920_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, APDS9920_DEV_NAME);
	return 0;
}*/

/*----------------------------------------------------------------------------*/
static ssize_t show_deviceid(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9920_i2c_client;
	int deviceid = 0;
	if(NULL == client)
	{
		printk(KERN_ERR TAGE "i2c client is null!!\n");
		return 0;
	}
	deviceid = device_id;
    
	return sprintf(buf, "%d\n", deviceid);
}
static DRIVER_ATTR(deviceid,   S_IWUSR | S_IRUGO, show_deviceid,      NULL);
/*----------------------------------------------------------------------------*/
static ssize_t store_ps_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);	 
	int num =0;
    num=simple_strtoul(buf, NULL, 10);
    set_ps_para_num(num);
    apds9920_ps_select_para(obj);
	return count;
}
static ssize_t show_ps_para_index(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", ps_para_num);
}
static DRIVER_ATTR(ps_para_index,   S_IWUSR | S_IRUGO, show_ps_para_index,store_ps_para_index);

static ssize_t store_ps_esd_time(struct device_driver *ddri,const char *buf, size_t count)
{
	//struct i2c_client *client = apds9920_i2c_client;
	//struct apds9920_priv *obj=i2c_get_clientdata(client);	 
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
	//struct i2c_client *client = apds9920_i2c_client;
	//struct apds9920_priv *obj=i2c_get_clientdata(client);	 
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
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->read_register);
}

static ssize_t store_read_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);	 
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
    res = apds9920_read_reg(client,register_addr[0], register_value, 0x1);
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
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);	 
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
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
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
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->write_register);
}

static ssize_t store_write_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
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
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		printk(KERN_INFO TAGI "set APDS9920 ps write reg:0x%02x=0x%02x\n",databuf[0],databuf[1]);
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
		printk(KERN_INFO TAGI "set APDS9920 ps register reg FAIL!!\n");
	}
	return count;
    
error_fs:
	sprintf(obj->write_register,"WRONG ARGS");
	return count;
}
static DRIVER_ATTR(write_register,   S_IWUSR | S_IRUGO, show_write_register,store_write_register);


/*----------------------------------------------------------------------------*/
static ssize_t show_offset(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "0x%02x\n", obj->prox_offset);
}

static ssize_t store_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	//unsigned long reg_value = simple_strtoul(buf, NULL, 16);
	unsigned char databuf[2];
	int res;
	unsigned char off_value=0;

	off_value=simple_strtoul(buf, NULL, 10);
	if(off_value<0 || off_value>255){
		printk(KERN_INFO TAGI "set APDS9920 wrong offset value=%d\n",off_value);
	}
	
	
	if( obj != NULL && client != NULL ){
		databuf[0] = APDS9920_CMM_OFFSET;
		databuf[1] = off_value;
		obj->prox_offset=off_value;
        res = apds9920_write_reg(client, databuf[0], &databuf[1], 0x1);
		printk(KERN_INFO TAGI "set APDS9920 prx offset reg:0x%02x=0x%02x\n",databuf[0],databuf[1]);
		if(res <= 0)
		{
			printk(KERN_INFO TAGI "%s--%d set register fail \n",__func__,__LINE__);
			sprintf(obj->write_register,"offset 0x%02x:FAIL",databuf[0]);
		}
		else
		{
			sprintf(obj->write_register,"offset 0x%02x=0x%02x",databuf[0],databuf[1]);
		}
	}
	else {
		printk(KERN_INFO TAGI "set APDS9920 prox offset fail\n");
	}
	return count;
}
static DRIVER_ATTR(offset,   S_IWUSR | S_IRUGO, show_offset,store_offset);
/*----------------------------------------------------------------------------*/
static ssize_t show_als_para_index(struct device_driver *ddri, char *buf)
{
    //struct als_para *para;
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
    //int base;

    apds9920_als_select_para(obj);

	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", als_para_num);
}

static ssize_t store_als_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	//do nothing
	//struct i2c_client *client = apds9920_i2c_client;
	//struct apds9920_priv *obj=i2c_get_clientdata(client);	 
	int num =0;
    num=simple_strtoul(buf, NULL, 10);
    set_als_para_num(num);
	return count;
}
static DRIVER_ATTR(als_para_index,   S_IWUSR | S_IRUGO, show_als_para_index, store_als_para_index);

/*----------------------------------------------------------------------------*/
static ssize_t show_ps_base_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);

    apds9920_ps_select_para(obj);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->ps_base_value));
}

static ssize_t store_ps_base_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	u16 thd_value = (u16)simple_strtoul(buf, NULL, 10);
	//unsigned char databuf[2];
	//int res;
	if( obj != NULL && client != NULL && thd_value >= 0 )
    {
        const struct ps_para *para;
    
        //check the value
        para = &apds9920_ps_para[ps_para_num];
        if(!(thd_value >= para->base_value_min && thd_value <= para->base_value_max))    //if the value is error, do nothing.
            return count;

        printk(KERN_INFO TAGI "mean to set ps_base_value=%d\n", thd_value);
        atomic_set(&obj->ps_base_value, thd_value);
        //comment by dengweicheng :store ps_base_value anyway

    //    {
            apds9920_ps_select_para(obj);     //reselect the threshold agian 
			//enable 
			if(test_bit(CMC_BIT_PS,&obj->enable) || test_bit(CMC_BIT_ENG_PS,&obj->enable)){
				apds9920_enable_eint(obj,0);
				apds9920_enable_eint(obj,1);
			}
            //apds9920_enable_ps(client,0);
			//mdelay(5);
			//apds9920_enable_ps(client,1);
    //    }

        printk(KERN_INFO TAGI "%s finnal ps_base_value=%d\n",__func__,atomic_read(&obj->ps_base_value));
	}
	else 
      {
		printk(KERN_INFO TAGI "set LTR558 ps threshold FAIL!!\n");
	}
    
	return count;
}
static DRIVER_ATTR(ps_base_value,   S_IWUSR | S_IRUGO, show_ps_base_value, store_ps_base_value);

/*----------------------------------------------------------------------------*/
static ssize_t show_strong_sunlight(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", obj->strong_sunlight);
}

static ssize_t store_strong_sunlight(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	unsigned light_value = simple_strtoul(buf, NULL, 10);
	//unsigned char databuf[2];
	//int res;
	if( obj != NULL && client != NULL && light_value > 0 ){
		obj->strong_sunlight=light_value;
	}
	else {
		printk(KERN_ERR TAGE "set LTR558 ps strong_sunlight FAIL!!\n");
	}
	return count;
}
static DRIVER_ATTR(strong_sunlight,   S_IWUSR | S_IRUGO, show_strong_sunlight,store_strong_sunlight);

/*----------------------------------------------------------------------------*/
static ssize_t show_ps_filter_delay(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", obj->ps_filter_delay);
}

static ssize_t store_ps_filter_delay(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	unsigned delay_value = simple_strtoul(buf, NULL, 10);
	//unsigned char databuf[2];
	//int res;
	if( obj != NULL && client != NULL && delay_value > 0 ){
		obj->ps_filter_delay=delay_value;
	}
	else {
		printk(KERN_INFO TAGI "set LTR558 ps_filter_delay FAIL!!\n");
	}
	return count;
}
static DRIVER_ATTR(ps_filter_delay,   S_IWUSR | S_IRUGO, show_ps_filter_delay,store_ps_filter_delay);

/*----------------------------------------------------------------------------*/
static ssize_t show_pulse_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->pulse_value));
}

static ssize_t store_pulse_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = apds9920_i2c_client;
	struct apds9920_priv *obj=i2c_get_clientdata(client);
	unsigned pulse_value = simple_strtoul(buf, NULL, 10);//6?
    
	//int res;
	if( obj != NULL && client != NULL && pulse_value >0){
        atomic_set(&obj->pulse_value, pulse_value);
        pulse = pulse_value;
	}
	else {
		printk(KERN_INFO TAGI "set apds9920 ps pulse_value FAIL!!\n");
	}
	return count;
}
static DRIVER_ATTR(pulse_value,   S_IWUSR | S_IRUGO, show_pulse_value,store_pulse_value);
/*----------------------------------------------------------------------------*/
static ssize_t show_prox_across(struct device_driver *ddri, char *buf)
{
   return sprintf(buf, "%d\n", is_prox_across_enable);
}

static ssize_t store_prox_across(struct device_driver *ddri, const char *buf, size_t count)
{
   struct i2c_client *client = apds9920_i2c_client;
   struct apds9920_priv *obj = i2c_get_clientdata(client);
   unsigned enable_prox = simple_strtoul(buf, NULL, 10);//6?

//   int res;
   if( obj != NULL && client != NULL && enable_prox >= 0) {
          printk("%s enable_prox=%d\n",__FUNCTION__,enable_prox);
          if(enable_prox){
                 enable_als_short_atime_mode(obj,1);
                 is_prox_across_enable = enable_prox;
          } else {
                 enable_als_short_atime_mode(obj,0);
                 is_prox_across_enable = enable_prox;
          }
   } else {
          printk(KERN_ERR TAGE "set apds9920 prox_across FAIL!!\n");
   }
   return count;
}
static DRIVER_ATTR(prox_across,   S_IWUSR | S_IRUGO, show_prox_across, store_prox_across);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *apds9920_attr_list[] = {
    &driver_attr_deviceid,
    &driver_attr_als_para_index,
    &driver_attr_ps_base_value,
    &driver_attr_strong_sunlight,
    &driver_attr_ps_filter_delay,
	&driver_attr_read_register,
    &driver_attr_write_register,
    &driver_attr_ps_para_index,
    &driver_attr_pulse_value,
	&driver_attr_sys_prox_status,
	&driver_attr_ps_esd_time,
	&driver_attr_offset,
	&driver_attr_debug,
	&driver_attr_prox_across,
};

/*----------------------------------------------------------------------------*/
static int apds9920_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(apds9920_attr_list)/sizeof(apds9920_attr_list[0]));
//	printk(KERN_INFO TAGI "[%s-%d]num = %d!\n",__func__,__LINE__,num);
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)	
	{
		if((err = driver_create_file(driver, apds9920_attr_list[idx])))
		{
			printk(KERN_ERR TAGE "driver_create_file (%s) = %d\n", apds9920_attr_list[idx]->attr.name, err);
			break;
		}
	} 
	return err;
}

static int apds9920_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(apds9920_attr_list)/sizeof(apds9920_attr_list[0]));
	if(driver == NULL)
	{
		return -EINVAL;	
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, apds9920_attr_list[idx]);
	}
	return err;
}

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
	//struct apds9920_priv *data = apds9920_obj;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if(ps_en != 0 && ps_en != 1)
		return -EINVAL;

	if(ps_en)
    {
        ps_enable_set(&apds9920_obj->ps_cdev, 1);
    }
    else
    {
        ps_enable_set(&apds9920_obj->ps_cdev, 0);
    }

	return count;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9920_priv *obj = apds9920_obj;
    u32 delay = atomic_read(&obj->ps_poll_delay);
    int i;
	int err = 0;

	if (enable)
    {
        for(i=0; i<3; i++)
        {
            if((err = apds9920_enable_ps(obj->client, 1)))
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
		apds9920_enable_eint(obj,0);
		apds9920_enable_eint(obj,1);
					
        // add 40ms delay as ps data not stable when it's just init
        //msleep(40);
        set_bit(CMC_BIT_PS, &obj->enable);
        printk(KERN_INFO TAGI "%s, set ps enable bit here",__func__);
					
        restart_ps_automatic=0;
        //obj->enable_als_by_ps = 1;
        obj->last_ps_data=-1;
        obj->count_ps_over_in=0;
        
		if(!test_bit(CMC_BIT_ALS, &obj->enable) && !test_bit(CMC_BIT_ENG_ALS, &obj->enable)) {
            if(!apds9920_enable_als(obj->client, 1)) {
                //obj->enable_als_by_ps=1;
                printk(KERN_INFO TAGI "---flyshine----- Enable ALS by PS SUCCESS !!\n");
			}
			else {
                //obj->enable_als_by_ps=0;
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
            apds9920_enable_eint(obj,0);
            if((err = apds9920_enable_ps(obj->client, 0)))
			{
                printk(KERN_ERR TAGE "disable ps fail: %d\n", err); 
				return -1;
			}
		}      		    
        //mt_eint_mask(CUST_EINT_ALS_NUM);//clear ps eint mask
		if(!test_bit(CMC_BIT_ENG_PS,&obj->enable) && !test_bit(CMC_BIT_ENG_ALS,&obj->enable) && !test_bit(CMC_BIT_ALS,&obj->enable) ) 
		{
            if((err = apds9920_enable_als(obj->client, 0)))
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
    //int i;
    //int err = 0;
	struct apds9920_priv *obj = apds9920_obj;
	als_en = -1;
	sscanf(buf, "%d", &als_en);

	if(als_en != 0 && als_en != 1)
		return -EINVAL;

	if(als_en)
    {
		if(is_short_atime_mode == 1)
		{
			enable_als_short_atime_mode(obj,0);
		}
        als_enable_set(&apds9920_obj->als_cdev, 1);
    }
    else
    {
        als_enable_set(&apds9920_obj->als_cdev, 0);
    }

	return count;
}

static int als_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds9920_priv *obj = apds9920_obj;
    u32 delay = atomic_read(&obj->als_poll_delay);
	unsigned char databuf[2];
	int err = 0;
    int i;

	if (enable)
    {
        //obj->enable_als_by_system = 1;
        for(i=0; i<3; i++)
        {
            if((err = apds9920_enable_als(obj->client, 1)))
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
		databuf[0] = APDS9920_CMM_CONTROL;
        databuf[1] = 0x20 | 0x02;   //50mA 1*
        err = apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);
		if(err <= 0)
            printk(KERN_ERR TAGE "set als parameter 1x FAIL!!!!!!!!\n");
        obj->als_modulus =16; //(400*100)/(16*DEFAULT_ATIME);       //(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
        obj->als_mode =  ALS_LOW_LUX_MODE;
        printk(KERN_INFO TAGI "Update als_modulus=%d\n", obj->als_modulus);
        
        if(err == 2)
            err = 0;
        else
            err = -1;
		
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
        //lxder M
        if(!test_bit(CMC_BIT_PS, &obj->enable) && !test_bit(CMC_BIT_ENG_PS, &obj->enable)
           && !test_bit(CMC_BIT_ENG_ALS,&obj->enable))
        {
            if((err = apds9920_enable_als(obj->client, 0)))
            {
                printk(KERN_ERR TAGE "disable als fail: %d\n", err); 
				//return -1;
			}
		}
		databuf[0] = APDS9920_CMM_ATIME;
		databuf[1] = apds9920_als_atime_tb[2];
		apds9920_write_reg(obj->client, databuf[0], &databuf[1], 0x1);
		obj->als_atime_index=2;
		clear_bit(CMC_BIT_ALS, &obj->enable);
		//obj->enable_als_by_system = 0;
        
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
	struct apds9920_priv *obj = apds9920_obj; 
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
	//__ATTR(poll_delay, 0664, ls_poll_delay_show, ls_poll_delay_store),
	__ATTR(enable, 0664, als_enable_show, als_enable_store),
};

static struct device_attribute proximity_attr[] = {
    //__ATTR(poll_delay, 0664, ps_poll_delay_show, ps_poll_delay_store),
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

/*----------------------------------------------------------------------------*/
static int check_sensor_type(struct i2c_client *client)
{
    int res = 0;
	u16 i = 0;
	u8 buffer[0];
	u8 device_ID=0xff;
	
	buffer[0] = APDS9920_CMM_DEVICEID;
	printk(KERN_INFO TAGI "[%s]client->addr = 0x%2x\n",__func__,client->addr);
	for (i=0; i<5; i++)
	{
        res = apds9920_read_reg(client, buffer[0], &device_ID, 0x1);
	    if(res <= 0)
		{
			continue;
		}
		break;
	}
    
	printk(KERN_INFO TAGI "[%s] i = %d,device_ID = %d,res=%d\n",__func__,i,device_ID,res);

	if(device_ID==0x39||device_ID==0x30){
    	device_id = APDS9920_DEVICE;         
		return APDS9920_CHIP;
    }
	return 0xff;
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
static int apds9920_parse_dt(struct device *dev, struct apds9920_priv *obj)
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

/*----------------------------------------------------------------------------*/
static int apds9920_lightsensor_setup(struct apds9920_priv *obj)
{
	int ret;

	obj->input_dev_als = input_allocate_device();
	if (!obj->input_dev_als) {
		printk(KERN_ERR TAGE 
			"%s: could not allocate als input device\n",
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
		printk(KERN_ERR TAGE 
            "%s: can not register als input device\n",
			__func__);
		goto err_free_als_input_device;
	}

	return ret;

err_free_als_input_device:
	input_free_device(obj->input_dev_als);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int apds9920_psensor_setup(struct apds9920_priv *obj)
{
	int ret;

	obj->input_dev_ps = input_allocate_device();
	if (!obj->input_dev_ps) {
		printk(KERN_ERR TAGE 
			"%s: could not allocate ps input device\n",
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
		printk(KERN_ERR TAGE 
			"%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	return ret;

err_free_ps_input_device:
	input_free_device(obj->input_dev_ps);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int apds9920_config_regulator(struct apds9920_priv *obj, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(apds9920_vreg) / sizeof(struct sensor_regulator);

	if (on) 
      {
		for (i = 0; i < num_reg; i++) 
        	{
			apds9920_vreg[i].vreg = regulator_get(&obj->client->dev, apds9920_vreg[i].name);
            
			if (IS_ERR(apds9920_vreg[i].vreg)) 
            		{
				rc = PTR_ERR(apds9920_vreg[i].vreg);
				printk(KERN_ERR TAGE "%s:regulator get failed rc=%d\n", __func__, rc);
				apds9920_vreg[i].vreg = NULL;
			}

			if (regulator_count_voltages(apds9920_vreg[i].vreg) > 0) 
            		{
				rc = regulator_set_voltage(
					apds9920_vreg[i].vreg,
					apds9920_vreg[i].min_uV,
					apds9920_vreg[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE "%s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(apds9920_vreg[i].vreg);
					apds9920_vreg[i].vreg = NULL;
				}
			}

			rc = regulator_enable(apds9920_vreg[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE "%s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(apds9920_vreg[i].vreg) > 0) 
                		{
					regulator_set_voltage(
						apds9920_vreg[i].vreg, 0,
						apds9920_vreg[i].max_uV);
				}
				regulator_put(apds9920_vreg[i].vreg);
				apds9920_vreg[i].vreg = NULL;
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
static void apds9920_ps_data_report(struct work_struct *work)
{
	struct apds9920_priv *data = container_of((struct delayed_work *)work, struct apds9920_priv, ps_report_work);
    static u32 index = 0;
    u32 delay = atomic_read(&data->ps_poll_delay);
    u16 als_c0_dat;
    int err = 0;
    
    mutex_lock(&ps_report_lock);

    if(data == NULL)
         goto restart_poll;
	if(test_bit(CMC_BIT_ALS, &data->enable)){
		if(alsCount < 20)
        {
        	alsCount += 1;
			delay = 66;
        }
    	if((err = apds9920_read_als(data->client, &data->als)))
    	{
    		printk(KERN_ERR TAGE "apds9920_read_als error %d\n", err);
    		goto exit_als;
    	}		
		
		if(nAlsReadCount < 2)
    	{
         	apds9920_read_ps(data->client, &data->ps);
         	apds9920_get_ps_value(data, data->ps);
	   		if(atomic_read(&data->ps_status) == 0)
	   		{
	       		printk(KERN_INFO TAGI "apds9920_read_als fisrt data not report\n");
	       		goto exit_als;
	   		}	
    	}

		if(data->als < 0)
        	data->als = 0;
		if(data->als > 65535)
        	data->als = 65533;		
                
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
			
            apds9920_enable_ps(data->client, 0);
            if(apds9920_enable_ps(data->client, 1) == 0)
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

    
    if((err = apds9920_read_ps(data->client, &data->ps)))
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

    apds9920_read_als_c0(data->client, &als_c0_dat);
//    apds9920_read_als(data->client, &als_c0_dat);
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

    if(apds9920_get_ps_value(data, data->ps) < 0)
    {
        printk(KERN_ERR TAGE "apds9920_get_ps_value fail\n");
        goto restart_poll;
    }

    input_report_rel(data->input_dev_ps, REL_X, (atomic_read(&data->ps_status) + 1));
    //input_report_rel(data->input_dev_ps, REL_Y, (index%2));
    input_sync(data->input_dev_ps);
    //index++;

restart_poll:

    mutex_unlock(&ps_report_lock);
    if((test_bit(CMC_BIT_PS, &data->enable) || test_bit(CMC_BIT_ALS, &data->enable)) && !atomic_read(&data->ps_suspend))
        queue_delayed_work(data->poll_queue, &data->ps_report_work, msecs_to_jiffies(delay));
}

/*----------------------------------------------------------------------------*/
/*static void apds9920_als_data_report(struct work_struct *work)
{
	struct apds9920_priv *data = container_of((struct delayed_work *)work, struct apds9920_priv, als_report_work);
    u32 delay = atomic_read(&data->als_poll_delay);
    static u32 index = 0;
    int err = 0;
    if(test_bit(CMC_BIT_ALS, &data->enable)){
    	if((err = apds9920_read_als(data->client, &data->als)))
    	{
    		printk(KERN_ERR TAGE "apds9920_read_als error %d\n", err);
    		goto restart_poll;
    	}

    if(nAlsReadCount < 3)
    {
         apds9920_read_ps(data->client, &data->ps);
         apds9920_get_ps_value(data, data->ps);
	   if(atomic_read(&data->ps_status) == 0)
	   {
	       printk(KERN_INFO TAGI "apds9920_read_als fisrt data not report\n");
	       goto restart_poll;
	   }	
    }

	if(data->als < 0)
        data->als = 0;
	if(data->als > 65535)
        data->als = 65533;		
                
    input_report_abs(data->input_dev_als, ABS_X, data->als);
    input_report_abs(data->input_dev_als, ABS_Y, (index%2)); 
    input_sync(data->input_dev_als);
    index++;
    
    data->last_als_data = data->als;
    }
restart_poll:
//	if(debug)
    if(test_bit(CMC_BIT_ALS, &data->enable) && atomic_read(&data->als_suspend) == 0)
        queue_delayed_work(data->poll_queue, &data->als_report_work, msecs_to_jiffies(delay));
}
*/

/*----------------------------------------------------------------------------*/
static int apds9920_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct apds9920_priv *obj;
	int err = 0;

    //init locks
    mutex_init(&apds9920_lock);
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	apds9920_obj = obj;

    //get the platform data from device tree
    if (client->dev.of_node) {
		err = apds9920_parse_dt(&client->dev, obj);
		/*if (ret) {
			goto err_parse_dt;
		}*/
	}

    //power on
    obj->client = client;
	i2c_set_clientdata(client, obj);
    apds9920_config_regulator(obj, true);
    msleep(5);
	
    chip_id = check_sensor_type(client);
	if(chip_id == 0xff)
	{
		err = -1;
		goto exit_kfree;
	}
	else
	{
		platform_driver_register(&als_ps_driver);
	}
	obj->hw = get_cust_alsps_hw();
	apds9920_get_addr(obj->hw, &obj->addr,chip_id);
	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
//	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);

	obj->enable = 0;
	obj->pending_intr = 0;
	//obj->enable_als_by_ps=0;
	//obj->enable_als_by_system=0;
	obj->strong_sunlight=DEFAULT_STRONG_SUNLIGHT;
	obj->prox_offset=0;

    //setup input_dev
    err = apds9920_lightsensor_setup(apds9920_obj);
	if (err < 0) {
		printk(KERN_ERR TAGE "%s: apds9920_lightsensor_setup error!\n", __func__);
		goto exit_init_failed;
	}

	err = apds9920_psensor_setup(apds9920_obj);
	if (err < 0) {
		printk(KERN_ERR TAGE "apds9920_psensor_setup error!!\n");
		goto exit_init_failed;
	}
    
    err = create_sysfs_interfaces(&apds9920_obj->input_dev_als->dev, light_attr,
			ARRAY_SIZE(light_attr));
	if (err < 0) {
	    printk(KERN_ERR TAGE "failed to create input_dev_als sysfs\n");
		goto exit_init_failed;
	}

    err = create_sysfs_interfaces(&apds9920_obj->input_dev_ps->dev, proximity_attr,
			ARRAY_SIZE(proximity_attr));
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to create input_dev_ps sysfs\n");
		goto exit_init_failed;
	}
	
	if(bbk_ps_para_count != 0)
		apds9920_ps_para = bbk_ps_paras;
	else
		apds9920_ps_para = bbk_ps_paras_default;
	if(bbk_als_para_count != 0)
		apds9920_als_para = bbk_als_paras;
	else
		apds9920_als_para = bbk_als_paras_default;

    obj->eint_queue = create_singlethread_workqueue("apds9920_eint");
    INIT_WORK(&obj->eint_work, apds9920_eint_work);

    obj->apds9920_fb_workqueue = create_singlethread_workqueue("apds9920_fb_workqueue");
    INIT_WORK(&obj->apds9920_fb_work,apds9920_fb_workfunc);
    
    obj->poll_queue = create_singlethread_workqueue("apds9920_report_workqueue");
    INIT_DELAYED_WORK(&obj->ps_report_work, apds9920_ps_data_report);
//    INIT_DELAYED_WORK(&obj->als_report_work, apds9920_als_data_report);
   
    atomic_set(&obj->ps_first_int, 1);
	atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
	ps_data_same_count=0;
	last_ps_data=-1;
    //atomic_set(&obj->ps_base_value,65535); //comment by dengweicheng ,set ps in out threshold when setup driver
    atomic_set(&obj->ps_base_value, apds9920_ps_para[ps_para_num].base_value);
    apds9920_ps_select_para(apds9920_obj);


    obj->ps_filter_delay=DEFAULT_PS_FILTER_DELAY;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = 16;//(400*100)/(16*DEFAULT_ATIME);   //(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										               //(400)/16*2.72 here is amplify *100
    obj->als_mode =  ALS_LOW_LUX_MODE;
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
	clear_bit(CMC_BIT_ENG_ALS, &obj->enable);
	clear_bit(CMC_BIT_ENG_PS, &obj->enable);
    
    //wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");	
	apds9920_i2c_client = client;

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
	
	if((err = apds9920_init_client(client)))
	{
		goto exit_init_failed;
	}       
	print_vivo_init(TAGI "apds9920_init_client() OK!\n");

	if((err = misc_register(&apds9920_device)))
	{
		printk(KERN_ERR TAGE "apds9920_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = apds9920_create_attr(&als_ps_driver.driver)))
	{
		printk(KERN_ERR TAGE "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
    
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = apds9920_early_suspend,
	obj->early_drv.resume   = apds9920_late_resume,    
	register_early_suspend(&obj->early_drv);
#else
    obj->fb_notif.notifier_call = apds9920_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif
    
    wake_lock_init(&(obj->wakelock), WAKE_LOCK_SUSPEND, "bbkapds9920");//liyuchu:prevent system suspend

#if defined(CONFIG_BBK_DEVICE_INFO)
	err = bbk_driver_register_device_info("ALS/PS", "0885159");
	if (err < 0) {
		printk(KERN_ERR TAGE "Failed to register ALS/PS device info\n");
	}
#endif
    
	print_vivo_init(TAGI "%s: OK in line %d\n", __func__,__LINE__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&apds9920_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(client);
exit_kfree:
	kfree(obj);
exit:
	apds9920_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	printk(KERN_INFO TAGI "%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int apds9920_i2c_remove(struct i2c_client *client)
{
        struct apds9920_priv* obj = i2c_get_clientdata(client);
	int err;	
	if((err = apds9920_delete_attr(&als_ps_driver.driver)))
	{
		printk(KERN_ERR TAGE "lapds9920_delete_attr fail: %d\n", err);
	}

	if((err = misc_deregister(&apds9920_device)))
	{
		printk(KERN_ERR TAGE "misc_deregister fail: %d\n", err);    
	}
	  if(chip_id != 0xff)
       {
       	platform_driver_unregister(&als_ps_driver);
       }
	apds9920_i2c_client = NULL;
	i2c_unregister_device(client);
    	wake_lock_destroy(&obj->wakelock);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9920_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	apds9920_power(hw, 1); 
	if(i2c_add_driver(&apds9920_i2c_driver))
	{
		printk(KERN_ERR TAGE "add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9920_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);    
	apds9920_power(hw, 0);    
	i2c_del_driver(&apds9920_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver apds9920_alsps_driver = {
	.probe      = apds9920_probe,
	.remove     = apds9920_remove,    
	.driver     = {
		.name  = "apds9920_als_ps",
		.owner = THIS_MODULE,
	}
};

struct platform_device sensor_apds9920_alsps = {	
	.name 	= "apds9920_als_ps",	
	.id		= -1,
};
/*----------------------------------------------------------------------------*/
static int __init apds9920_init(void)
{
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);
    
	if(platform_driver_register(&apds9920_alsps_driver))
	{
		printk(KERN_ERR TAGE "failed to register apds9920_als_ps driver");
		return -ENODEV;
	}
	if(platform_device_register(&sensor_apds9920_alsps))
	{
		printk(KERN_ERR TAGE "failed to register apds9920_als_ps device\n");
		return -ENODEV;
	}

    wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");
    
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit apds9920_exit(void)
{
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);
	platform_driver_unregister(&apds9920_alsps_driver);
}
/*----------------------------------------------------------------------------*/
late_initcall(apds9920_init);
module_exit(apds9920_exit);
/*----------------------------------------------------------------------------*/

MODULE_DESCRIPTION("APDS9920 mbient light + proximity sensor driver");
MODULE_AUTHOR("Ling Xu <xuling@vivo.com.cn>");
MODULE_LICENSE("GPL");

