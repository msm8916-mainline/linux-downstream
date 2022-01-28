/* Copyright Statement:
 *
 * This software/firmware and related documentation ("vivo Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to vivo Inc. and/or its licensors.
 * Without the prior written permission of vivo inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of vivo Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* vivo Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("vivo SOFTWARE")
 * RECEIVED FROM vivo AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. vivo EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES vivo PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE vivo SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN vivo SOFTWARE. vivo SHALL ALSO NOT BE RESPONSIBLE FOR ANY vivo
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND vivo'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE vivo SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT vivo'S OPTION, TO REVISE OR REPLACE THE vivo SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * vivo FOR SUCH vivo SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("vivo Software")
 * have been modified by vivo Inc. All revisions are subject to any receiver's
 * applicable license agreements with vivo Inc.
 */

/* drivers/hwmon/mt6516/amit/apds9920.c - APDS9920 ALS/PS driver
 * 
 * Author: james<james@vivo.com.cn>
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
#include <linux/sensors.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <asm/io.h>
#include <linux/jiffies.h>
#include <linux/vivo_sensor_config.h>
#include <asm/div64.h>

#include "gp2ap008t.h"

#define GP2AP008T_DEV_NAME     "GP2AP008T"
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

static struct i2c_client *gp2ap008t_i2c_client = NULL;
extern int qup_i2c_suspended; // add for i2c timeout   
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id gp2ap008t_i2c_id[] = {{GP2AP008T_DEV_NAME,0},{}};

#define alfa1					4490//0.00585
#define alfa2					9896//0.008525
#define alfa3					14022//0.001853
#define alfa4					1061//0.001853
//#define alfa5					2214//0.001853
#define beta1					-898
#define beta2					12617//-0.013425
#define beta3					19065//-0.002305
#define beta4					1310//-0.002305
//#define beta5					2214//-0.002305
#define CALC_OFFSET1			100000
#define CALC_OFFSET2			100000
#define CALC_OFFSET3			100000
#define CALC_OFFSET4			100000
//#define CALC_OFFSET5			1000000
#define RATIO_FIRST_BOUND		40//0.2
#define RATIO_SECOND_BOUND	 	64//0.6
#define RATIO_THIRD_BOUND		73//0.8
#define RATIO_FOURTH_BOUND      81

#define MAX_LUX_VALUE				0xFFFFFFFF//Indicate Overflow
#define OVER_FLOW_COUNT		 		50000
#define ZERO_LUX_TH					50

/* ALS mode change */
#define LOW_LUX_MODE				( 0 )
#define	MIDDLE_LUX_MODE				( 1 )
#define	HIGH_LUX_MODE				( 2 )

#define LOW_LUX_RANGE	 			( RANGE_A_1 )
#define MIDDLE_LUX_RANGE 			( RANGE_A_16 )
#define HIGH_LUX_RANGE	 			( RANGE_A_555 )

#define ALS_L_to_M_counts		 	50000
#define ALS_M_to_H_counts		 	50000

#define ALS_H_to_M_counts		 	1000
#define ALS_M_to_L_counts		 	2000

/* coefficient for adjusting the difference in RANGE */
#define GAMMA_LOW_LUX_MODE			1			/*   1/ 1  = 1    basis  */
#define GAMMA_MIDDLE_LUX_MODE		16			/*   16/1  = 16	         */
#define GAMMA_HIGH_LUX_MODE			555			/*   555/1 = 555         */

#define LUX_MEDIAN					3


struct gp2ap008t_priv
{
	struct i2c_client *client;
	struct workqueue_struct *eint_queue;
	struct work_struct eint_work;
	u8			regData[15] ;
	int			als_mode ;
	u32			als_lux_prev ;
	u8			i ;
	u32			lux_median[LUX_MEDIAN] ;

	/*i2c address group*/
	//struct apds9922_i2c_addr addr;

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
} ;

static int gp2ap008t_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int gp2ap008t_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/
static int gp2ap008t_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int gp2ap008t_i2c_resume(struct i2c_client *client);

int gp2ap008t_enable_eint(struct gp2ap008t_priv *obj,int enable);

int gp2ap008t_read_ps(struct i2c_client *client, u16 *data);

static long gp2ap008t_enable_als(struct i2c_client *client, int enable);
static long gp2ap008t_enable_ps(struct i2c_client *client, int enable);

static int ps_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable);
static int als_enable_set(struct sensors_classdev *sensors_cdev, unsigned int enable);
static void gp2ap008t_ps_data_report(struct work_struct *work);


static struct i2c_driver gp2ap008t_i2c_driver = {	
	.probe      = gp2ap008t_i2c_probe,
	.remove     = gp2ap008t_i2c_remove,
	.suspend    = gp2ap008t_i2c_suspend,
	.resume     = gp2ap008t_i2c_resume,
	.id_table   = gp2ap008t_i2c_id,
	.driver = {
		.name     = GP2AP008T_DEV_NAME,
	},
};

static struct gp2ap008t_priv *gp2ap008t_obj = NULL;
static struct platform_driver gp2ap008t_alsps_driver;
static struct mutex gp2ap008t_lock;


static struct sensors_classdev sensors_light_cdev0 = {
	.name = "GP2AP008T-light",
	.vendor = "SHARP",
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
	.name = "GP2AP008T-proximity",
	.vendor = "SHARP",
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
struct ps_para  *gp2ap008t_ps_para = NULL;
struct als_para *gp2ap008t_als_para = NULL;
static int ps_para_num  = 2;          //the value must select early
static int als_para_num = 2;

static int nAlsReadCount = 0;
static unsigned short stronglight_flag = 0;
static int debug_flag=0;
static int alsCount = 0;
static int unenable_and_read_count=0;
static struct mutex ps_report_lock = __MUTEX_INITIALIZER(ps_report_lock);



static DEFINE_MUTEX(gp2ap008t_mutex);
static struct gp2ap008t_priv *g_gp2ap008t_ptr = NULL;
static struct wake_lock ps_suspend_lock;

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator gp2ap008t_vreg[] = {
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

static int gp2ap008t_read_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
	u8 temp_address;
	u8 *temp_data;
	int res = 0;
	if(qup_i2c_suspended > 0){
		printk(KERN_INFO TAGI" gp2ap008t read reg when i2c bus is suspended.\n");
		return -1;		
	}
	mutex_lock(&gp2ap008t_lock);

	temp_address = addr;
	temp_data = data;

	res = i2c_master_send(client, &temp_address, 0x1);
	if(res < 0) {
		if(temp_address != REG_ADR_20)
		    printk("KERN_ERR TAGE gp2ap008t read 0x%x fail %d\n", temp_address, res);
		mutex_unlock(&gp2ap008t_lock);
		return res;
	}

	res = i2c_master_recv(client, temp_data, count);
	if(res < 0) {
		if(temp_address != REG_ADR_20)
		     printk(KERN_ERR TAGE "gp2ap008t read 0x%x fail %d\n", temp_address, res);
		mutex_unlock(&gp2ap008t_lock);
		return res;
	}

	mutex_unlock(&gp2ap008t_lock);

	return count;
}


static int gp2ap008t_write_reg(struct i2c_client *client, u8 addr, u8 *data, int count)
{
	u8 buffer[10] = {0};
	int res = 0;
      	if(qup_i2c_suspended > 0){
		printk(KERN_INFO TAGI" gp2ap008t write reg when i2c bus is suspended.\n");
		return -1;		
	}
	mutex_lock(&gp2ap008t_lock);

	buffer[0] = addr;
	if(data != NULL && count > 0 && count < 5) {
		memcpy(&buffer[1], data, count);
	}

	res = i2c_master_send(client, buffer, (0x01 + count));
	if(res < 0) {
		printk(KERN_ERR TAGE "gp2ap008t write 0x%x fail %d\n", buffer[0], res);
		mutex_unlock(&gp2ap008t_lock);
		return res;
	}

	mutex_unlock(&gp2ap008t_lock);

	return (count + 1);
}

static int als_onoff_simplified( u8 onoff, struct gp2ap008t_priv *data )
{
	u8		value ;

	pr_debug( "light_sensor onoff = %d\n", onoff ) ;

	if( onoff )
	{
		if( !(test_bit(CMC_BIT_PS , &data->enable ) || test_bit(CMC_BIT_ENG_PS, &data->enable))  )
		{
			
			data->regData[REG_ADR_03] = ( INTVAL_0 );
			value = data->regData[REG_ADR_03] ;
			//gp2ap_i2c_write( REG_ADR_03, &value, data->client ) ;
			gp2ap008t_write_reg(data->client, REG_ADR_03, &value, 0x01);
			value = ( OP_RUN | OP_ALS ) ;			// ALS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
		else 
		{
			value = ( OP_SHUTDOWN ) ; 				// Shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
			data->regData[REG_ADR_03] = ( INTVAL_0 );
			value = data->regData[REG_ADR_03] ;
			gp2ap008t_write_reg(data->client, REG_ADR_03, &value, 0x01);
			value = ( OP_RUN | OP_PS_ALS ) ;		// PS & ALS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}

	}
	else
	{
		if( !(test_bit(CMC_BIT_PS , &data->enable ) || test_bit(CMC_BIT_ENG_PS, &data->enable) ) )
		{
			value = ( OP_SHUTDOWN ) ; // shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
		else
		{
			value = ( OP_SHUTDOWN ) ; // shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
			data->regData[REG_ADR_03] = ( INTVAL_25 );
			value = data->regData[REG_ADR_03] ;
			gp2ap008t_write_reg(data->client, REG_ADR_03, &value, 0x01);
			value = ( OP_RUN | OP_PS ) ;	// PS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
	}
	return 0 ;
}

int gp2ap008t_read_als(struct i2c_client *client, u16 *data)
{	 
	struct gp2ap008t_priv *obj = i2c_get_clientdata(client);
	u32		data_als0 ;
	u32		data_als1 ;
	u64       lux  = 0;
	u32 	     ratio ;
	u8		value ;
	u8		rdata[4] ;
	int 	     gamma=1 ;
       u8 		a, b ;	
       u32		temp ;
       u32		sub_data[3] ;
	int		adjust_offset;
  
  if (client == NULL) {
	  printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	/* get data_als0(clear), data_als1(ir) */
	gp2ap008t_read_reg(client,REG_ADR_04, rdata, 1);
	if(debug)
	     printk(KERN_INFO TAGI " data_als0  reg_0x04 =  %d\r\n",rdata[0]);
	gp2ap008t_read_reg(client,REG_ADR_13, rdata, sizeof( rdata ));
	
	data_als0 = rdata[0]+rdata[1]*256;
	data_als1 = rdata[2]+rdata[3]*256;
	if(debug)
           printk(KERN_INFO TAGI " als value data_als0 = %d data_als1 = %d\r\n",data_als0,data_als1);
	/* calculate the ratio */	
	if( data_als0 == 0 )
	{
		ratio = 100 ;
	}
	else
	{
		ratio = ( data_als1 * 100 ) / data_als0 ;
	}
    
	/* lux calculation */
	
	/* calculate lux without adjusting parameters */
	if(ratio<RATIO_FIRST_BOUND)
	{
		lux = (alfa1 * data_als0 - beta1 * data_als1) ;
		adjust_offset = CALC_OFFSET1 ;
	}
	else if (ratio<RATIO_SECOND_BOUND)
	{
		lux = (alfa2 * data_als0 - beta2 * data_als1) ;
		adjust_offset = CALC_OFFSET2 ;
	}
	else if (ratio<RATIO_THIRD_BOUND)
	{
		lux = (alfa3 * data_als0 - beta3 * data_als1) ;
		adjust_offset = CALC_OFFSET3 ;
	}
	else if(ratio < RATIO_FOURTH_BOUND)
	{
		lux = (alfa4* data_als0 - beta4 * data_als1) ;
		adjust_offset = CALC_OFFSET4 ;
	}
	
	
	/* coefficient for adjusting the difference in RANGE */
	if( obj->als_mode == HIGH_LUX_MODE )
		gamma = GAMMA_HIGH_LUX_MODE;		// HIGH_LUX_RANGE   :ALS1_RANGE X512
	else if( obj->als_mode == MIDDLE_LUX_MODE )
		gamma = GAMMA_MIDDLE_LUX_MODE;		// MIDDLE_LUX_RANGE :ALS1_RANGE X16
	else 
		gamma = GAMMA_LOW_LUX_MODE;			// LOW_LUX_RANGE    :ALS1_RANGE X1
	//printk(KERN_INFO TAGI "data_als0 obj->als_mode = %d\r\n", obj->als_mode);
	/* Exceptional process and precaution for calculation overflow */
	 if ( ( data_als0 > OVER_FLOW_COUNT ) &&
				 ( obj->als_mode == HIGH_LUX_MODE ) )
	{/* raw data overflow */
		lux = MAX_LUX_VALUE ;
	}
	else if ( ratio>RATIO_FOURTH_BOUND )
	{/* Illegal data */
		lux = obj->als_lux_prev;
	}
	else
	{/* Gamma calculation with precaution for overflow */
		if ( lux >= 1000000 )
		{
			adjust_offset /= 1000 ;
		       do_div(lux,1000);
			 lux = lux*gamma;
			 do_div(lux,adjust_offset);
			//lux = ( lux /1000 ) * gamma / adjust_offset ;
		}
		else			
		{    
		     // lux = gamma * lux / adjust_offset ;
			//lux = gamma * lux /100;
			lux = gamma*lux;
			do_div(lux,100);
			if(lux >5 && lux < 1000)
				lux = 1010;
			//lux /=1000;
			do_div(lux,1000);	
		}
	}
	
	/*  Lux mode (Range) change */
	
	/*  LOW ---> MIDDLE */
	if( ( data_als0 >= ALS_L_to_M_counts ) && ( obj->als_mode == LOW_LUX_MODE ) )
	{
		obj->als_mode = MIDDLE_LUX_MODE ;
		
		printk( KERN_INFO TAGI"change lux mode. MIDDLE_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, obj ) ;
		
		/* change Range of ALS */
		obj->regData[REG_ADR_04] = ( obj->regData[REG_ADR_04] & 0xF0 ) | MIDDLE_LUX_RANGE;
		value = obj->regData[REG_ADR_04];
		//gp2ap_i2c_write( REG_ADR_04, &value, obj->client ) ;
		gp2ap008t_write_reg(obj->client, REG_ADR_04, &value, 0x01);

		/* Start */
		als_onoff_simplified( 1, obj ) ;

		lux = obj->als_lux_prev ; 
	}
	/*  MIDDLE ---> HIGH */
	else if( ( data_als0 > ALS_M_to_H_counts ) && ( obj->als_mode == MIDDLE_LUX_MODE ) )
	{
		obj->als_mode = HIGH_LUX_MODE ;
		
		printk( KERN_INFO TAGI"change lux mode. HIGH_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, obj ) ;
		
		/* change Range of ALS */
		obj->regData[REG_ADR_04] = ( obj->regData[REG_ADR_04] & 0xF0 ) | HIGH_LUX_RANGE;
		value = obj->regData[REG_ADR_04];
		//gp2ap_i2c_write( REG_ADR_04, &value, obj->client ) ;
		gp2ap008t_write_reg(obj->client, REG_ADR_04, &value, 0x01);

		/* Start */
		als_onoff_simplified( 1, obj ) ;
		
		lux = obj->als_lux_prev ; 
	}
	/*  HIGH ---> MIDDLE */
	else if( ( data_als0 < ALS_H_to_M_counts ) && ( obj->als_mode == HIGH_LUX_MODE ) )
	{
		obj->als_mode = MIDDLE_LUX_MODE ;
		
		printk( KERN_INFO "change lux mode. MIDDLE_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, obj ) ;
		
		/* change Range of ALS */
		obj->regData[REG_ADR_04] = ( obj->regData[REG_ADR_04] & 0xF0 ) | MIDDLE_LUX_RANGE;
		value = obj->regData[REG_ADR_04];
		//gp2ap_i2c_write( REG_ADR_04, &value, obj->client ) ;
		gp2ap008t_write_reg(obj->client, REG_ADR_04, &value, 0x01);

		/* Start */
		als_onoff_simplified( 1, obj ) ;
		
		lux = obj->als_lux_prev ; 
	}
	/*  MIDDLE ---> LOW */
	else if( ( data_als0 < ALS_M_to_L_counts ) && ( obj->als_mode == MIDDLE_LUX_MODE ) )    
	{
		obj->als_mode = LOW_LUX_MODE ;
		
		printk( KERN_INFO "change lux mode. LOW_LUX_MODE!! \n" ) ;
		
		/* ALS Shutdown */
		als_onoff_simplified( 0, obj ) ;
		
		/* change Range of ALS */
		obj->regData[REG_ADR_04] = ( obj->regData[REG_ADR_04] & 0xF0 ) | LOW_LUX_RANGE;
		value = obj->regData[REG_ADR_04];
		//gp2ap_i2c_write( REG_ADR_04, &value, obj->client ) ;
		gp2ap008t_write_reg(obj->client, REG_ADR_04, &value, 0x01);

		/* Start */
		als_onoff_simplified( 1, obj ) ;
		
		lux = obj->als_lux_prev ; 
	}

	//printk( KERN_INFO "LUX Median. lux = %d\n", lux ) ;	

	/*  the median of 3 times of measured value */
	if(obj->i >= 3)
	{
		obj->i = 0;
	}
	obj->lux_median[obj->i] = lux;
		
 	for(a=0; a<3; a++){
		sub_data[a] = obj->lux_median[a];
	}
 	//printk( KERN_INFO "LUX Median. lux = %d, sub_data0,1,2 = %d, %d, %d \n", lux, sub_data[0], sub_data[1], sub_data[2] ) ;
  	for(a=0; a<3-1; a++){
		for(b=3-1; b>a; b--){
			if(sub_data[b-1] > sub_data[b]){
				temp = sub_data[b];
				sub_data[b] = sub_data[b-1];
				sub_data[b-1] = temp;
			}
		}
	}
	lux = sub_data[1];
	obj->i++;
	//printk( KERN_INFO "LUX Median. lux = %d, sub_data0,1,2 = %d, %d, %d \n", lux, sub_data[0], sub_data[1], sub_data[2] ) ;

	if(lux > 65535)
		lux = 65535;
	obj->als_lux_prev = (u32)lux ; 

	*data = (u16)lux;
	if(*data > 1)
	   *data =  (gp2ap008t_als_para[als_para_num].base_value * ( *data) ) / 100;
	if(debug)
	    printk(KERN_INFO TAGI "data_als data = %d lux = %lld base_value = %d\r\n ",*data,lux,gp2ap008t_als_para[als_para_num].base_value);
	return 0 ;
}

int gp2ap008t_read_als_c0(struct i2c_client *client, u16 *data)
{
    //struct gp2ap008t_priv *obj = i2c_get_clientdata(client);
     u32		data_als0 ;	
     u8		rdata[4] ;
  
     if (client == NULL) {
	  printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
     }
	/* get data_als0(clear), data_als1(ir) */
     // gp2ap_i2c_read( REG_ADR_13, rdata, sizeof( rdata ),client ) ;
	gp2ap008t_read_reg(client,REG_ADR_13, rdata, sizeof( rdata ));
	
	data_als0 = rdata[0]+rdata[1]*256;
	*data = data_als0;
      return 0;
}

int gp2ap008t_read_ps(struct i2c_client *client, u16 *data)
{
	//struct gp2ap008t_priv*obj = i2c_get_clientdata(client);	
	u8 buffer[2];
	u16 ps_data;
	long res = 0;
	int i = 0;
	if (client == NULL) {
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	for(i = 0; i < 5; i++)
	{
	      res = gp2ap008t_read_reg(client,REG_ADR_11, buffer, sizeof( buffer ));
		if (res <= 0) {
			goto EXIT_ERR;
		}
		ps_data = (buffer[0] | (buffer[1]<<8));
		if(ps_data != 0)
			break;
	}
	
	
	
	*data = ps_data;
	if(debug)
		printk(KERN_INFO TAGI "%s data = %d\r\n",__func__,*data);
	return 0;    

EXIT_ERR:
	printk(KERN_ERR TAGE "gp2ap008_read_ps fail\n");
	return res;
}

static int gp2ap008t_clear_intr(struct i2c_client *client) 
{
    //struct gp2ap008t_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];
	u8 reg_val;	
       gp2ap008t_read_reg(client,REG_ADR_01, &reg_val, 1);
	printk(KERN_INFO TAGI "reg_val = 0x%dx\r\n",reg_val);
	buffer[0] = REG_ADR_01;
	buffer[1] = reg_val & 0xfb;
	res = gp2ap008t_write_reg(client, buffer[0],&buffer[1], 0x1);
	return 1;
}

static int gp2ap008t_check_intr(struct i2c_client *client) 
{
       //struct gp2ap008t_priv *obj = i2c_get_clientdata(client);
	int res, intp = 0, intl = 0;
	u8 rdata[2];

	res = gp2ap008t_read_reg(client,REG_ADR_01, rdata, 1);
	printk(KERN_INFO TAGI "gp2ap008_check_intr value = 0x%x\r\n",rdata[0]);
	if(0 != (rdata[0] & 0x04))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (rdata[0] & 0x02))
	{
		res = 0;
		intl = 1;		
	}
	return 1;
}

static int gp2ap008t_ps_select_para(struct gp2ap008t_priv *obj)         //
{
    int out_threshold, in_threshold;   
    int base = atomic_read(&obj->ps_base_value);
    int step_num;
    const struct ps_step *step;
    const struct ps_para *para;
    
    //TODO: get different parameter here
    para = &gp2ap008t_ps_para[ps_para_num];
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
static int gp2ap008t_als_select_para(struct gp2ap008t_priv *obj)
{
    //TODO: get different parameter here
    //als_para_num = 0; 

    return 0; 
}

static int gp2ap008t_get_ps_value(struct gp2ap008t_priv *obj, u16 ps)
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
			//gp2ap008t_enable_eint(obj, 1);
		}
	}
	return val;

}

static irqreturn_t gp2ap008t_irq(int irq, void *data)
{
	struct gp2ap008t_priv *obj = data;
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

static void gp2ap008t_eint_work(struct work_struct *work)
{
	struct gp2ap008t_priv *obj = (struct gp2ap008t_priv *)container_of(work, struct gp2ap008t_priv, eint_work);
	
	printk(KERN_INFO TAGI "agp2ap008t_eint_work\n");

	if(!test_bit(CMC_BIT_PS, &obj->enable) || test_bit(CMC_BIT_ENG_PS, &obj->enable)) {
		gp2ap008t_clear_intr(obj->client);
		gp2ap008t_check_intr(obj->client);
        	enable_irq(obj->irq);
        	return;
	}
	/* Now it is only a wake source */
	wake_lock_timeout(&ps_suspend_lock, 2 * HZ);    //wake up for 2s
	gp2ap008t_clear_intr(obj->client);
	
	if(atomic_read(&obj->ps_suspend))
       	msleep(10);     //10ms 
     	gp2ap008t_ps_data_report(&(obj->ps_report_work.work));
 
    	enable_irq(obj->irq);    
       gp2ap008t_check_intr(obj->client);
       queue_delayed_work(obj->poll_queue, &obj->ps_report_work, 0);    //report ps data now
	
}

int gp2ap008t_setup_eint(struct i2c_client *client)
{
    struct gp2ap008t_priv *obj = i2c_get_clientdata(client);        
    int err;
    g_gp2ap008t_ptr = obj;
  
    if (!obj->irq)
    {
        if(gpio_is_valid(obj->gpio_int)) 
        {
            err = gpio_request(obj->gpio_int, "gp2ap008t_int");
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
            err = request_any_context_irq(obj->irq, gp2ap008t_irq, (IRQF_TRIGGER_LOW | IRQF_ONESHOT), 
                        "GP2AP008T_i2c", obj);
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

static u8 gp2ap_init_data[15] = {
	/* Reg0 shutdown */
	0x00,
	/* Clear Flags */
	( 0x00 ),
	/* Reg2 PIN:00 INTTYPE:0  */
	( PIN_INT_PS),
	/* Reg3 INTVAL:0 IS:11 PIN:11 FREQ:0 */
	( INTVAL_0 ),
	/* Reg.4 RES_A:01 RANGE_A:0011 */
	( ALS_AREA_ALL | RES_A_16 | MIDDLE_LUX_RANGE ),
	/* Reg.5  */
	( RES_P_12 ),
	/* Reg.6 IS:11 SUM:011 */
	( IS_75 | SUM_X11 ),
	/* Reg.7 PRST:100 */
	( PRST_1 | 0x03 ),
	/* Reg.8 PL[7:0]:0x90 */
	0x00,
	/* Reg.9 PL[15:8]:0x01 */
	0x00,
	/* Reg.A PH[7:0]:0x58 */
	0xff,
	/* Reg.B PH[15:8]:0x02 */
	0x3f,
	/* Reg.C OS[7:0]:0x00 */
	0x00,
	/* Reg.D OS[15:8]:0x00 */
	0x00,
	/* Reg.E PANEL:0x00 */
	0x00
};
static int gp2ap008t_init_device( struct gp2ap008t_priv *data )
{
	int		i ;
      int res = 0;
	for( i = 1 ; i < sizeof( data->regData ) ; i++ )
	{
		res =gp2ap008t_write_reg(data->client, i, &data->regData[i], 0x01);
	}
	if((res = gp2ap008t_setup_eint(data->client)) != 0) {
		printk(KERN_ERR TAGE"setup eint: %d\n", res);
	}
	return res;
}


static long gp2ap008t_enable_als(struct i2c_client *client, int enable)
{
	struct gp2ap008t_priv *data = i2c_get_clientdata(client);
	u8		value ;

	//pr_debug( "light_sensor onoff = %d\n", onoff ) ;

	if( enable )
	{
		if( !(test_bit(CMC_BIT_PS , &data->enable ) || test_bit(CMC_BIT_ENG_PS, &data->enable) ))
		{
			data->regData[REG_ADR_03] = ( INTVAL_0 );
			gp2ap008t_init_device( data );
			data->als_mode = MIDDLE_LUX_MODE;
			value = ( OP_RUN | OP_ALS ) ;		// ALS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
		else
		{
			value = ( OP_SHUTDOWN ) ; // shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
			data->regData[REG_ADR_03] = ( INTVAL_0 );
			value = data->regData[REG_ADR_03] ;
			//gp2ap_i2c_write( REG_ADR_03, &value, data->client ) ;
			gp2ap008t_write_reg(data->client, REG_ADR_03, &value, 0x01);
			value = ( OP_RUN | OP_PS_ALS ) ;	// ALS & PS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
	}
	else
	{
		if( !(test_bit(CMC_BIT_PS , &data->enable ) || test_bit(CMC_BIT_ENG_PS, &data->enable) ) )
		{
			value = ( OP_SHUTDOWN ) ; // shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
		else
		{
			value = ( OP_SHUTDOWN ) ; // shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
			data->regData[REG_ADR_03] = ( INTVAL_25 );
			value = data->regData[REG_ADR_03] ;
			gp2ap008t_write_reg(data->client, REG_ADR_03, &value, 0x01);
			value = ( OP_RUN | OP_PS ) ;	// PS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
	}
	return 0 ;
}

static long gp2ap008t_enable_ps(struct i2c_client *client, int enable)
{
	struct gp2ap008t_priv *data = i2c_get_clientdata(client);
	u8			value ;

	if( enable )
	{
		value = ( OP_SHUTDOWN ) ;	// shutdown
		gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);

		data->regData[REG_ADR_03] = (INTVAL_0 ) ;
		value = data->regData[REG_ADR_03] ;
		gp2ap008t_write_reg(data->client, REG_ADR_03, &value, 0x01);

		value = ( OP_RUN | OP_PS_ALS ) ;	// ALS & PS mode
		gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		//slect the threshold here
		gp2ap008t_ps_select_para(data);
		atomic_set(&data->sys_prox_status, 1);         //1: stand for status far
		atomic_set(&data->ps_status, 1); //1: stand for: far away
		gp2ap008t_enable_eint(data, 0);
		gp2ap008t_enable_eint(data, 1);
	}
	else
	{
		if( !( test_bit(CMC_BIT_ALS , &data->enable) ))
		{
			value = ( OP_SHUTDOWN ) ;	// shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
		else
		{
			value = ( OP_SHUTDOWN ) ;	// shutdown
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
			value = ( OP_RUN | OP_ALS ) ;		// ALS mode
			gp2ap008t_write_reg(data->client, REG_ADR_00, &value, 0x01);
		}
		//atomic_set(&data->ps_deb_on, 0);
		printk(KERN_INFO TAGI "gp2ap008 power off\n");

		atomic_set(&data->ps_first_int, 1);
		atomic_set(&data->ps_status, 1);          //default is faraway
		atomic_set(&data->sys_prox_status, 1);         //1: stand for status far
		
		gp2ap008t_enable_eint(data, 0);
	}

	return 0 ;
}

static int gp2ap008t_enable(struct i2c_client *client, int enable)
{
//	struct apds9922_priv *obj = i2c_get_clientdata(client);
	u8	value ;    
	int res = 0;

	if(client == NULL)
	{
		printk(KERN_ERR TAGE "CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	if(enable)
	{
	      
		value = ( OP_RUN | OP_PS_ALS ) ;	// ALS & PS mode
		res = gp2ap008t_write_reg(client, REG_ADR_00, &value, 0x01);
	
		if(res <= 0)
		{
		    printk(KERN_ERR TAGE "GP2AP008T_enable fail\n");
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI " GP2AP008T_enable on\n");
	}
	else
	{
	      value = ( OP_SHUTDOWN ) ;	// shutdown
		gp2ap008t_write_reg(client, REG_ADR_00, &value, 0x01);
		if(res <= 0)
		{
		    printk(KERN_ERR TAGE "GP2AP008T_enable fail\n");
			goto EXIT_ERR;
		}
		printk(KERN_INFO TAGI "GP2AP008T_enable  off\n");
	}		

	return 0;
	
EXIT_ERR:
	printk(KERN_ERR TAGE "GP2AP008T_enable fail\n");
	return res;
}

static int gp2ap008t_open(struct inode *inode, struct file *file)
{
	file->private_data = gp2ap008t_i2c_client;

	if (!file->private_data)
	{
		printk(KERN_ERR TAGE "null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int gp2ap008t_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long gp2ap008t_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct gp2ap008t_priv *obj = i2c_get_clientdata(client);  
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
					gp2ap008t_enable_als(obj->client,1);
					if((err = gp2ap008t_enable_ps(obj->client, 1)))
					{
						printk(KERN_ERR TAGE "enable ps fail: %d\n", err); 
						goto err_out;
					}
					gp2ap008t_enable_eint(obj,0);
					gp2ap008t_enable_eint(obj,1);
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
					if((err = gp2ap008t_enable_ps(obj->client, 0)))
					{
						printk(KERN_ERR TAGE "disable ps fail: %d\n", err); 
						goto err_out;
					}
					gp2ap008t_enable_eint(obj,0);
				}
				if(!test_bit(CMC_BIT_ALS,&obj->enable) && !test_bit(CMC_BIT_PS,&obj->enable)){
					gp2ap008t_enable_als(obj->client,0);
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
			if((err = gp2ap008t_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}	
			dat = gp2ap008t_get_ps_value(obj, obj->ps);			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = gp2ap008t_read_ps(obj->client, &obj->ps)))
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
					if((err = gp2ap008t_enable_als(obj->client, 1)))
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
					if((err = gp2ap008t_enable_als(obj->client, 0)))
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
			if((err = gp2ap008t_read_als(obj->client, &obj->als)))
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
			if((err = gp2ap008t_read_als(obj->client, &obj->als)))
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
static struct file_operations gp2ap008t_fops = {
	.owner = THIS_MODULE,
	.open = gp2ap008t_open,
	.release = gp2ap008t_release,
	.unlocked_ioctl = gp2ap008t_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice gp2ap008t_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &gp2ap008t_fops,
};

int gp2ap008t_enable_eint(struct gp2ap008t_priv *obj, int enable)
{
	int res;
	unsigned char databuf[2];
	printk(KERN_INFO TAGI " gp2ap008t_enable_eint enable = %d\n",enable);

       if(enable) {
		databuf[0] = REG_ADR_08;
		databuf[1] = (u8)((atomic_read(&obj->ps_out_threshold)) & 0x00FF);
		res = gp2ap008t_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

		databuf[0] = REG_ADR_09;
		databuf[1] = (u8)(((atomic_read(&obj->ps_out_threshold)) & 0xFF00) >> 8);
		res = gp2ap008t_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

		databuf[0] = REG_ADR_0A;
		databuf[1] = (u8)((atomic_read(&obj->ps_in_threshold)) & 0x00FF);
		res = gp2ap008t_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

		databuf[0] = REG_ADR_0B;
		databuf[1] = (u8)(((atomic_read(&obj->ps_in_threshold)) & 0xFF00) >> 8);;
		res = gp2ap008t_write_reg(obj->client, databuf[0], &databuf[1], 0x1);

		gp2ap008t_clear_intr(obj->client);

		return 0;
	} else {
		printk(KERN_INFO TAGI "disabling gp2ap008 eint\n");
		
		gp2ap008t_clear_intr(obj->client);
	}

	return 0;
}

static int gp2ap008t_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{	
	struct gp2ap008t_priv *obj = i2c_get_clientdata(client);    	
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
	if((err = gp2ap008t_enable_als(client, 0)))
	{	
		printk(KERN_ERR TAGE "disable als: %d\n", err);	
		return err;	
	}
	printk(KERN_INFO TAGI "%s-----disable ps\n",__func__);	
	if((err = gp2ap008t_enable_ps(client, 0)))
	{
		printk(KERN_ERR TAGE "disable ps:  %d\n", err);
		return err;
	}
	gp2ap008t_enable(client, 0);
	
	return 0;

}



/*----------------------------------------------------------------------------*/
static int gp2ap008t_i2c_resume(struct i2c_client *client)
{

	
	struct gp2ap008t_priv *obj = i2c_get_clientdata(client);        
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
	
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{

		if((err = gp2ap008t_enable_als(client, 1)))
		{
			printk(KERN_ERR TAGE "enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		printk(KERN_INFO TAGI "%s-----enable ps\n",__func__);
		if((err = gp2ap008t_enable_ps(client, 1)))
		{
			printk(KERN_ERR TAGE "enable ps fail: %d\n", err);                
		}
	}

	return 0;
}


#if defined(CONFIG_HAS_EARLYSUSPEND)
/*----------------------------------------------------------------------------*/
static void gp2ap008t_early_suspend(struct early_suspend *h)
{ 
	struct gp2ap008t_priv *obj = container_of(h, struct gp2ap008t_priv, early_drv);
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
static void gp2ap008t_late_resume(struct early_suspend *h)
{
	struct gp2ap008t_priv *obj = container_of(h, struct gp2ap008t_priv, early_drv);
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
static int gp2ap008t_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct gp2ap008t_priv *obj = container_of(self, struct gp2ap008t_priv, fb_notif);

	printk(KERN_INFO TAGI "gp2ap008t_fb_notifier_callback \n");

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

static ssize_t store_ps_para_index(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);	 
	int num =0;
	num=simple_strtoul(buf, NULL, 10);
	set_ps_para_num(num);
	gp2ap008t_ps_select_para(obj);
	return count;
}
static ssize_t show_ps_para_index(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", ps_para_num);
}
static DRIVER_ATTR(ps_para_index,   S_IWUSR | S_IRUGO, show_ps_para_index,store_ps_para_index);

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
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->read_register);
}

static ssize_t store_read_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);	 
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
	res = gp2ap008t_read_reg(client,register_addr[0], register_value, 0x1);
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
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);	 
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
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
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
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "REG:%s\n", obj->write_register);
}

static ssize_t store_write_register(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
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
		res =  gp2ap008t_write_reg(client, databuf[0], &databuf[1], 0x1);
		printk(KERN_INFO TAGI "set GP2AP008T ps write reg:0x%02x=0x%02x\n",databuf[0],databuf[1]);
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
		printk(KERN_INFO TAGI "set GP2AP008T ps register reg FAIL!!\n");
	}
	return count;

	error_fs:
		sprintf(obj->write_register,"WRONG ARGS");
	return count;
}
static DRIVER_ATTR(write_register,   S_IWUSR | S_IRUGO, show_write_register,store_write_register);

static ssize_t show_als_para_index(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);

	gp2ap008t_als_select_para(obj);

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
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);

	gp2ap008t_ps_select_para(obj);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", atomic_read(&obj->ps_base_value));
}

static ssize_t store_ps_base_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
	u16 thd_value = (u16)simple_strtoul(buf, NULL, 10);

	if( obj != NULL && client != NULL && thd_value >= 0 )
	{
		const struct ps_para *para;

		//check the value
		para = &gp2ap008t_ps_para[ps_para_num];
		if(!(thd_value >= para->base_value_min && thd_value <= para->base_value_max))    //if the value is error, do nothing.
		return count;

		printk(KERN_INFO TAGI "mean to set ps_base_value=%d\n", thd_value);
		atomic_set(&obj->ps_base_value, thd_value);

		gp2ap008t_ps_select_para(obj);     //reselect the threshold agian 
		//enable 
		if(test_bit(CMC_BIT_PS,&obj->enable) || test_bit(CMC_BIT_ENG_PS,&obj->enable)){
			gp2ap008t_enable_eint(obj,0);
			gp2ap008t_enable_eint(obj,1);
		}


		printk(KERN_INFO TAGI "%s finnal ps_base_value=%d\n",__func__,atomic_read(&obj->ps_base_value));
	}
	else 
	{
		printk(KERN_INFO TAGI "set gp2ap008t ps threshold FAIL!!\n");
	}

	return count;
}
static DRIVER_ATTR(ps_base_value,   S_IWUSR | S_IRUGO, show_ps_base_value, store_ps_base_value);

/*----------------------------------------------------------------------------*/
static ssize_t show_strong_sunlight(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
	if(NULL == obj)
	{
		printk(KERN_ERR TAGE "i2c clientdata is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", obj->strong_sunlight);
}

static ssize_t store_strong_sunlight(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = gp2ap008t_i2c_client;
	struct gp2ap008t_priv *obj=i2c_get_clientdata(client);
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

static struct driver_attribute *gp2ap008t_attr_list[] = {
	&driver_attr_als_para_index,
	&driver_attr_ps_base_value,
	&driver_attr_strong_sunlight,
	&driver_attr_read_register,
	&driver_attr_write_register,
	&driver_attr_ps_para_index,
	&driver_attr_sys_prox_status,
	&driver_attr_debug,
};

/*----------------------------------------------------------------------------*/
static int gp2ap008t_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(gp2ap008t_attr_list)/sizeof(gp2ap008t_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}
	for(idx = 0; idx < num; idx++)	
	{
		if((err = driver_create_file(driver, gp2ap008t_attr_list[idx])))
		{
			printk(KERN_ERR TAGE "driver_create_file (%s) = %d\n", gp2ap008t_attr_list[idx]->attr.name, err);
			break;
		}
	} 
	return err;
}

static int gp2ap008t_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(gp2ap008t_attr_list)/sizeof(gp2ap008t_attr_list[0]));
	if(driver == NULL)
	{
		return -EINVAL;	
	}
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, gp2ap008t_attr_list[idx]);
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
		ps_enable_set(&gp2ap008t_obj->ps_cdev, 1);
	}
	else
	{
		ps_enable_set(&gp2ap008t_obj->ps_cdev, 0);
	}

	return count;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct gp2ap008t_priv *obj = gp2ap008t_obj;
	u32 delay = atomic_read(&obj->ps_poll_delay);
	int i;
	int err = 0;

	if (enable)
	{
		for(i=0; i<3; i++)
		{
			if((err = gp2ap008t_enable_ps(obj->client, 1)))
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
		gp2ap008t_enable_eint(obj,0);
		gp2ap008t_enable_eint(obj,1);

		// add 40ms delay as ps data not stable when it's just init
		//msleep(40);
		set_bit(CMC_BIT_PS, &obj->enable);
		printk(KERN_INFO TAGI "%s, set ps enable bit here",__func__);

		obj->last_ps_data=-1;

		if(!test_bit(CMC_BIT_ALS, &obj->enable) && !test_bit(CMC_BIT_ENG_ALS, &obj->enable)) {
			if(!gp2ap008t_enable_als(obj->client, 1)) {
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
			gp2ap008t_enable_eint(obj,0);
			if((err = gp2ap008t_enable_ps(obj->client, 0)))
			{
				printk(KERN_ERR TAGE "disable ps fail: %d\n", err); 
				return -1;
			}
		}      		    
		//mt_eint_mask(CUST_EINT_ALS_NUM);//clear ps eint mask
		if(!test_bit(CMC_BIT_ENG_PS,&obj->enable) && !test_bit(CMC_BIT_ENG_ALS,&obj->enable) && !test_bit(CMC_BIT_ALS,&obj->enable) ) 
		{
			if((err = gp2ap008t_enable_als(obj->client, 0)))
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
		als_enable_set(&gp2ap008t_obj->als_cdev, 1);
	}
	else
	{
		als_enable_set(&gp2ap008t_obj->als_cdev, 0);
	}

	return count;
}

static int als_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct gp2ap008t_priv *obj = gp2ap008t_obj;
	u32 delay = atomic_read(&obj->als_poll_delay);
	int err = 0;
	int i;

	if (enable)
	{
		for(i=0; i<3; i++)
		{
			if((err = gp2ap008t_enable_als(obj->client, 1)))
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
			if((err = gp2ap008t_enable_als(obj->client, 0)))
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
	struct gp2ap008t_priv *obj = gp2ap008t_obj; 
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
static int gp2ap008t_parse_dt(struct device *dev, struct gp2ap008t_priv *obj)
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

static int gp2ap008t_lightsensor_setup(struct gp2ap008t_priv *obj)
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
static int gp2ap008t_psensor_setup(struct gp2ap008t_priv *obj)
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
static int gp2ap008t_config_regulator(struct gp2ap008t_priv *obj, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(gp2ap008t_vreg) / sizeof(struct sensor_regulator);

	if (on) 
	{
		for (i = 0; i < num_reg; i++) 
		{
			gp2ap008t_vreg[i].vreg = regulator_get(&obj->client->dev, gp2ap008t_vreg[i].name);

			if (IS_ERR(gp2ap008t_vreg[i].vreg)) 
			{
				rc = PTR_ERR(gp2ap008t_vreg[i].vreg);
				printk(KERN_ERR TAGE "%s:regulator get failed rc=%d\n", __func__, rc);
				gp2ap008t_vreg[i].vreg = NULL;
			}

			if (regulator_count_voltages(gp2ap008t_vreg[i].vreg) > 0) 
			{
				rc = regulator_set_voltage(
				gp2ap008t_vreg[i].vreg,
				gp2ap008t_vreg[i].min_uV,
				gp2ap008t_vreg[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE "%s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(gp2ap008t_vreg[i].vreg);
					gp2ap008t_vreg[i].vreg = NULL;
				}
			}

			rc = regulator_enable(gp2ap008t_vreg[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE "%s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(gp2ap008t_vreg[i].vreg) > 0) 
				{
					regulator_set_voltage(
					gp2ap008t_vreg[i].vreg, 0,
					gp2ap008t_vreg[i].max_uV);
				}
				regulator_put(gp2ap008t_vreg[i].vreg);
				gp2ap008t_vreg[i].vreg = NULL;
			}
		}
	} 
	else 
	{
		i = num_reg;
	}

	return rc;
}

static void gp2ap008t_ps_data_report(struct work_struct *work)
{
	struct gp2ap008t_priv *data = container_of((struct delayed_work *)work, struct gp2ap008t_priv, ps_report_work);
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
		delay = 50;
		}
		if((err = gp2ap008t_read_als(data->client, &data->als)))
		{
			printk(KERN_ERR TAGE "gp2ap008t_read_als error %d\n", err);
			goto exit_als;
		}		

		if(nAlsReadCount < 2)
		{
			gp2ap008t_read_ps(data->client, &data->ps);
			gp2ap008t_get_ps_value(data, data->ps);
			if(atomic_read(&data->ps_status) == 0)
			{
				printk(KERN_INFO TAGI "gp2ap008t_read_als fisrt data not report\n");
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

			gp2ap008t_enable_ps(data->client, 0);
			if(gp2ap008t_enable_ps(data->client, 1) == 0)
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


	if((err =gp2ap008t_read_ps(data->client, &data->ps)))
	{
		printk(KERN_ERR TAGE "apds9920_read_ps error %d\n", err);
		goto restart_poll;
	}

	gp2ap008t_read_als_c0(data->client, &als_c0_dat);
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
	if(gp2ap008t_get_ps_value(data, data->ps) < 0)
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

static u8 gp2ap008t_read_id(struct i2c_client *client)
{
	u8 device_id;
	int res = 0;
	res = gp2ap008t_read_reg(client,REG_ADR_20, &device_id, 0x1);
	if(res <= 0)
	{
		return 0;
	}
	return device_id;
}
static int gp2ap008t_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gp2ap008t_priv *obj;
	int err = 0;
      int i = 0;
	//init locks
	mutex_init(&gp2ap008t_lock);
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	gp2ap008t_obj = obj;

	//get the platform data from device tree
	if (client->dev.of_node) {
		err = gp2ap008t_parse_dt(&client->dev, obj);
	}
       client->addr = 0x39;
	//power on
	obj->client = client;
	i2c_set_clientdata(client, obj);
	gp2ap008t_config_regulator(obj, true);
	msleep(5);

	for( i = 0 ; i < sizeof( gp2ap_init_data ) ; i++ )	
	{	
	     obj->regData[i] = gp2ap_init_data[i] ;	
	}
	if(gp2ap008t_read_id(client) != DEVICE_ID)
	{
	       err = -1;
		goto exit_kfree;
	}
	gp2ap008t_init_device( obj );
	print_vivo_init(TAGI "gp2ap008t_init_client() OK!\n");
      
	platform_driver_register(&als_ps_driver);

	obj->enable = 0;
	obj->strong_sunlight=DEFAULT_STRONG_SUNLIGHT;

	//setup input_dev
	err = gp2ap008t_lightsensor_setup(gp2ap008t_obj);
	if (err < 0) {
		printk(KERN_ERR TAGE "%s: gp2ap008t_lightsensor_setup error!\n", __func__);
		goto exit_init_failed;
	}

	err = gp2ap008t_psensor_setup(gp2ap008t_obj);
	if (err < 0) {
		printk(KERN_ERR TAGE "apds9920_psensor_setup error!!\n");
		goto exit_init_failed;
	}

	err = create_sysfs_interfaces(&gp2ap008t_obj->input_dev_als->dev, light_attr,
	ARRAY_SIZE(light_attr));
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to create input_dev_als sysfs\n");
		goto exit_init_failed;
	}

	err = create_sysfs_interfaces(&gp2ap008t_obj->input_dev_ps->dev, proximity_attr,
	ARRAY_SIZE(proximity_attr));
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to create input_dev_ps sysfs\n");
		goto exit_init_failed;
	}

	if(bbk_ps_para_count != 0)
		gp2ap008t_ps_para = bbk_ps_paras;
	else
		gp2ap008t_ps_para = bbk_ps_paras_default;
	if(bbk_als_para_count != 0)
		gp2ap008t_als_para = bbk_als_paras;
	else
		gp2ap008t_als_para = bbk_als_paras_default;

	obj->eint_queue = create_singlethread_workqueue("gp2ap008t_eint");
	INIT_WORK(&obj->eint_work, gp2ap008t_eint_work);        

	obj->poll_queue = create_singlethread_workqueue("gp2ap008t_report_workqueue");
	INIT_DELAYED_WORK(&obj->ps_report_work, gp2ap008t_ps_data_report);

	atomic_set(&obj->ps_first_int, 1);
	atomic_set(&obj->sys_prox_status, 1);         //1: stand for status far
	//atomic_set(&obj->ps_base_value,65535); //comment by dengweicheng ,set ps in out threshold when setup driver
	atomic_set(&obj->ps_base_value, gp2ap008t_ps_para[ps_para_num].base_value);
	gp2ap008t_ps_select_para(gp2ap008t_obj);

	atomic_set(&obj->i2c_retry, 3);
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
	clear_bit(CMC_BIT_ENG_ALS, &obj->enable);
	clear_bit(CMC_BIT_ENG_PS, &obj->enable);
	obj->als_mode = MIDDLE_LUX_MODE ;

	//wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");	
	gp2ap008t_i2c_client = client;

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

	
	if((err = misc_register(&gp2ap008t_device)))
	{
		printk(KERN_ERR TAGE "gp2ap008t_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = gp2ap008t_create_attr(&als_ps_driver.driver)))
	{
		printk(KERN_ERR TAGE "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = gp2ap008t_early_suspend,
	obj->early_drv.resume   = gp2ap008t_late_resume,    
	register_early_suspend(&obj->early_drv);
#else
	obj->fb_notif.notifier_call = gp2ap008t_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif

	wake_lock_init(&(obj->wakelock), WAKE_LOCK_SUSPEND, "bbkgp2ap008t");//liyuchu:prevent system suspend

#if defined(CONFIG_BBK_DEVICE_INFO)
	err = bbk_driver_register_device_info("ALS/PS", "0885159");
	if (err < 0) {
	printk(KERN_ERR TAGE "Failed to register ALS/PS device info\n");
	}
#endif

	print_vivo_init(TAGI "%s: OK in line %d\n", __func__,__LINE__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&gp2ap008t_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	gp2ap008t_i2c_client = NULL;           
	printk(KERN_ERR TAGE "%s: err = %d\n", __func__, err);
	return err;
}

static int gp2ap008t_i2c_remove(struct i2c_client *client)
{
	struct gp2ap008t_priv* obj = i2c_get_clientdata(client);
	int err;	
	if((err = gp2ap008t_delete_attr(&als_ps_driver.driver)))
	{
		printk(KERN_ERR TAGE "gp2ap008t_delete_attr fail: %d\n", err);
	}

	if((err = misc_deregister(&gp2ap008t_device)))
	{
		printk(KERN_ERR TAGE "misc_deregister fail: %d\n", err);    
	}
	
	platform_driver_unregister(&als_ps_driver);
	gp2ap008t_i2c_client = NULL;
	i2c_unregister_device(client);
	wake_lock_destroy(&obj->wakelock);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int gp2ap008t_probe(struct platform_device *pdev) 
{
	if(i2c_add_driver(&gp2ap008t_i2c_driver))
	{
		printk(KERN_ERR TAGE "add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int gp2ap008t_remove(struct platform_device *pdev)
{	
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);    
	i2c_del_driver(&gp2ap008t_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver gp2ap008t_alsps_driver = {
	.probe      = gp2ap008t_probe,
	.remove     = gp2ap008t_remove,    
	.driver     = {
		.name  = "gp2ap008t_als_ps",
		.owner = THIS_MODULE,
	}
};

struct platform_device sensor_gp2ap008t_alsps = {	
	.name 	= "gp2ap008t_als_ps",	
	.id		= -1,
};
/*----------------------------------------------------------------------------*/
static int __init gp2ap008t_init(void)
{
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);

	if(platform_driver_register(&gp2ap008t_alsps_driver))
	{
		printk(KERN_ERR TAGE "failed to register gp2ap008t_alsps_driver driver");
		return -ENODEV;
	}
	if(platform_device_register(&sensor_gp2ap008t_alsps))
	{
		printk(KERN_ERR TAGE "failed to register gp2ap008t_alsps_driver device\n");
		return -ENODEV;
	}
	wake_lock_init(&ps_suspend_lock, WAKE_LOCK_SUSPEND, "PS wakelock");

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit gp2ap008t_exit(void)
{
	printk(KERN_INFO TAGI "%s\n", __FUNCTION__);
	platform_driver_unregister(&gp2ap008t_alsps_driver);
}
/*----------------------------------------------------------------------------*/
late_initcall(gp2ap008t_init);
module_exit(gp2ap008t_exit);
/*----------------------------------------------------------------------------*/

MODULE_DESCRIPTION("GP2AP008T mbient light + proximity sensor driver");
MODULE_AUTHOR("james<james@vivo.com.cn>");
MODULE_LICENSE("GPL");

