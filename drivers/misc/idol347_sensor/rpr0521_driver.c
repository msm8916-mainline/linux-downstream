/* drivers/i2c/chips/rpr521_driver.c - ROHM RPR521 Linux kernel driver
 *
 * Copyright (C) 2012 
 * Written by Andy Mi <andy-mi@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  This is Linux kernel modules for ambient light + proximity sensor
 *  Revision History
 *  2012-7-19:	Ver. 1.0	New release together with a porting guide.
 *  2012-8-14:	Ver. 1.1	Added calibration and set thresholds methods. Besides, the thresholds are automatically changed if a ps int is triggered to avoid constant interrupts.
 *  2014-1-09:	Ver. 1.2	Modified some functions for rpr521
 */
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/sensors.h>
//#include <linux/input/rpr0521_driver.h>
#include "rpr0521_driver.h"
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

/* add by xx for sensor class */
#define DEVICE_NAME		"rpr521"
#define ALS_NAME 		"rpr521-als"
#define PS_NAME 			"rpr521-ps"

/*add by xx for double click improvement*/
#ifdef CONFIG_TCT_8X16_IDOL347
static struct i2c_client *g_client;
struct class *rpr_prx_misoperation_class = NULL;
#endif

/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
const unsigned long data0_coefficient[COEFFICIENT] = {9647, 4974, 3113, 4050};
const unsigned long data1_coefficient[COEFFICIENT] = {6523, 1889,  1128,  1440};
const unsigned long judge_coefficient[COEFFICIENT] = {1008,  1631,  2757, 2810}; 


#define _AUTO_THRESHOLD_CHANGE_
//#define _INIT_CALIB_  //zero modify in 20140505
//#define _ENABLE_CALIB_  //grace modify in 2014.12.12
u8 init_calib_flag = 0;   //grace modify in 2014.12.12
u8 init_ps_high = 0;  //grace modify in 2014.12.12
u8 init_ps_low = 0;  //grace modify in 2014.12.12
u8 calib_status = 0;
//#define ROHM_CALIBRATE //grace modify in 2015.2.16


/* mode control table */
#define MODE_CTL_FACTOR (16)
static const struct MCTL_TABLE {
    short ALS;
    short PS;
} mode_table[MODE_CTL_FACTOR] = {
    {  0,   0},   /*  0 */
    {  0,  10},   /*  1 */
    {  0,  40},   /*  2 */
    {  0, 100},   /*  3 */
    {  0, 400},   /*  4 */
    {100,  50},   /*  5 */
    {100, 100},   /*  6 */
    {100, 400},   /*  7 */
    {400,   0},   /*  8 */
    {400, 100},   /*  9 */
    {400,   0},   /* 10 */
    {400, 400},   /* 11 */
    {  50,  50},   /* 12 */
    {  0,   0},   /* 13 */
    {  0,   0},   /* 14 */
    {  0,   0}    /* 15 */
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    /*char DATA0;
    char DATA1;*/
    unsigned char DATA0; //grace modify in 2014.5.7
    unsigned char DATA1; //grace modify in 2014.5.7
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  0,   0},   /*  1 */
    {  0,   0},   /*  2 */
    {  0,   0},   /*  3 */
    {  2,   1},   /*  4 */
    {  2,   2},   /*  5 */
    {  0,   0},   /*  6 */
    {  0,   0},   /*  7 */
    {  0,   0},   /*  8 */
    {  0,   0},   /*  9 */
    { 64,  64},   /* 10 */
    {  0,   0},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};

/* add by xx for sensor class */
static int rpr521_prox_enable(struct ALS_PS_DATA *als_ps, int on);
static int rpr521_als_enable(struct ALS_PS_DATA *als_ps, int on);

static struct sensors_classdev sensors_light_cdev = {
	.name = ALS_NAME,
	.vendor = "rpr521",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "60000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = PS_NAME,
	.vendor = "rpr521",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static int rpr521_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ALS_PS_DATA *als_ps =
		container_of(sensors_cdev, struct ALS_PS_DATA, als_cdev);
	
	return rpr521_als_enable(als_ps, enable);
}

static int rpr521_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ALS_PS_DATA *als_ps =
		container_of(sensors_cdev, struct ALS_PS_DATA, ps_cdev);

	return rpr521_prox_enable(als_ps, enable);
}
/* add by xx for sensor class end*/

/*************** Functions ******************/
/******************************************************************************
 * NAME       : rpr521_set_enable
 * FUNCTION   : set measurement time according to enable
 * REMARKS    : this function will overwrite the work mode. if it is called improperly, 
 *			   you may shutdown some part unexpectedly. please check als_ps->enable first.
 *			   I assume it is run in normal mode. If you want low noise mode, the code should be modified.
 *****************************************************************************/
static int rpr521_set_enable(struct i2c_client *client, int enable)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;

	if(enable > 0xFb)
	{
		printk(KERN_ERR "%s: invalid measurement time setting.\n", __func__);
		return -EINVAL;
	}
	else
	{
		mutex_lock(&als_ps->update_lock);
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL, enable);
		mutex_unlock(&als_ps->update_lock);

		als_ps->enable = enable;
		als_ps->als_time = mode_table[(enable & 0xF)].ALS;
		als_ps->ps_time = mode_table[(enable & 0xF)].PS;

		return ret;
	}
}

static int rpr521_set_ps_threshold_low(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold > als_ps->ps_th_h)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}*/ //grace modify in 2014.5.6
	
    /* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTL, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_l = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr521_set_ps_threshold_high(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold < als_ps->ps_th_l)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}*///grace modify in 2014.5.6
	
    /* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTH, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_h = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr521_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int average;
 	unsigned int i, tmp, ps_th_h, ps_th_l;	
	u8 infrared_data; //grace modify in 2014.12.12
//	u16 judge_ps = 0; //grace modify in 2014.12.12
//	u16 judge_ps_high = 0, judge_ps_low = 0; //grace modify in 2014.12.12

	average = 0;
	calib_status = 0;

	//rpr521_set_enable(client, 0x41);	//PS 10ms
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR&0XFE); //disable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}	
	
	rpr521_set_enable(client, PS_EN|PS_DOUBLE_PULSE|PS10MS);        //PS 10ms

	//grace modify in 2014.12.12 begin	
	 mdelay(20); 
	 tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
	 if(tmp < 0)	
	 {		
		goto err_exit;	
	 }	
	 infrared_data = tmp;	

	 if(infrared_data>>6)  
	 {		
		goto err_exit;
	 }
	 //grace modify in 2014.12.12 end

	//for(i = 0; i < 20; i ++)
	for(i = 0; i < 10; i ++) //grace modify in 2014.12.16
	{
		mdelay(20);
		
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			goto err_exit;
		}
		average += tmp & 0xFFF;	// 12 bit data
	}
	//average /= 20;
	average /= 10; //grace modify in 2014.12.16

//	ps_th_h = average + PS_ALS_SET_PS_TH;
//	ps_th_l = average + PS_ALS_SET_PS_TL;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if(ps_th_h < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_h > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}

#if 0
	//grace modify in 2014.12.12 begin
#if 1
	judge_ps = PS_CROSSTALK;
	if ((average > judge_ps * 15/10) || (average < judge_ps * 5/10 ))
	{
		if(init_calib_flag)
		{
			ps_th_h = init_ps_high;
			ps_th_l = init_ps_low;
		}
		else
		{
			ps_th_h = PS_ALS_SET_PS_TH;
			ps_th_l = PS_ALS_SET_PS_TL;
		}		
	}
#else
	judge_ps_high =  PS_ALS_SET_PS_TH;
	judge_ps_low = PS_ALS_SET_PS_TL;
	if ((ps_th_h > (judge_ps_high*14/10) ) || (ps_th_h < (judge_ps_high*6/10)) ||(ps_th_l > (judge_ps_low*14/10))||( ps_th_l < (judge_ps_low*6/10)))
	{
		if(init_calib_flag)
		{
			ps_th_h = init_ps_high;
			ps_th_l = init_ps_low;
		}
		else
		{
			ps_th_h = PS_ALS_SET_PS_TH;
			ps_th_l = PS_ALS_SET_PS_TL;
		}		
	}
#endif
//grace modify in 2014.12.12 end	
#endif

	//grace modify in 2014.12.16 begin
	if (average > PS_CROSSTALK)
	{
		if(init_calib_flag)
		{
			ps_th_h = init_ps_high;
			ps_th_l = init_ps_low;
		}
		else
		{
			ps_th_h = PS_ALS_SET_PS_TH;
			ps_th_l = PS_ALS_SET_PS_TL;
		}		
	}
	else
	{
		calib_status = 1;
	}
	//grace modify in 2014.12.16 end
	
	if(!(rpr521_set_ps_threshold_high(client, ps_th_h)))
		als_ps->ps_th_h_back = ps_th_h;
	else 
		goto err_exit;
	if(!(rpr521_set_ps_threshold_low(client, ps_th_l)))
		als_ps->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	//rpr521_set_enable(client, 0);	//disable ps
  rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.6 
  tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}	
	return 0;		

err_exit:
	//rpr521_set_enable(client, 0);	//disable ps
	rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.6 
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt

	return -1;
	
}

#if _FUNCTION_USED_	//masked because they are claimed but not used, which may cause error when compilling if the warning level is high enough. These functions provides some methods.
static int rpr521_set_persist(struct i2c_client *client, int persist)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	
    /* check whether the parameter is valid */
	if(persist > PERSISTENCE_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to rpr521 via i2c */
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, persist);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->persistence = persist;	//Update the value after successful i2c write to avoid difference. 

	return 0;
}

static int rpr521_set_control(struct i2c_client *client, int control)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned char gain, led_current;
	
	if(control > REG_ALSPSCTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
	gain = (control & 0x3C) >> 2;	//gain setting values
	led_current = control & 0x03;		//led current setting value

	if(!((gain == ALSGAIN_X1X1) || (gain == ALSGAIN_X1X2) || (gain == ALSGAIN_X2X2) || (gain == ALSGAIN_X64X64)
		|| (gain == ALSGAIN_X128X64) || (gain == ALSGAIN_X128X128)))
	{
		printk(KERN_ERR "%s: invalid gain setting. \n", __func__);
		return -EINVAL;
	}
	
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, control);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->control = control;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = led_current;

	return ret;
}
#endif

#if 0
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0)	// . If data MSB is 1, it may go to endless loop. 
    	{
    	data /= 2;	//0xFFFFFFFFFFFFFFFF / 2 = 0x7FFFFFFFFFFFFFFF
	base_divier /= 2;
    	}
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

#else
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile long long divier;
    volatile long      unit_sft;

    if ((data < 0) || (base_divier == 0)) {
        *answer   = 0;
        *overplus = 0;
        return (CALC_ERROR);
    }

    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while ((data > divier) && (divier > 0)) {
            unit_sft++;
            divier = divier << 1;
        }
        //while ((data > base_divier) && (unit_sft > 0)) {
        while ((data > base_divier) && (unit_sft >= 0)) { //grace modify in 2014.8.19
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }

    return (0);
}
#endif

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
//static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned char time)
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned short time) //grace modify in 2014.5.7
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE    (65535)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                result; //grace modified in 2014.4.2
	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
#if 0  //grace modified in 2014.4.2
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus); 
#else
			result = long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
      if (result == CALC_ERROR)
      {
         return (result);
      }
#endif
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
#if 0 //grace modified in 2014.4.2
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
#else
					result = long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					if (result == CALC_ERROR)
      		{
         		return (result);
      		}
#endif
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/******************************************************************************
 * NAME       : calculate_ps_data
 * FUNCTION   : calculate proximity data for rpr521
 * REMARKS    : 12 bit output
 *****************************************************************************/
static int calc_rohm_ps_data(unsigned short ps_reg_data)
{
    return (ps_reg_data & 0xFFF);
}

static unsigned int rpr521_als_data_to_level(unsigned int als_data)
{
#if 0
#define ALS_LEVEL_NUM 15
	int als_level[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
	int als_value[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
    	unsigned char idx;

	for(idx = 0; idx < ALS_LEVEL_NUM; idx ++)
	{
		if(als_data < als_value[idx])
		{
			break;
		}
	}
	if(idx >= ALS_LEVEL_NUM)
	{
		printk(KERN_ERR "rpr521 als data to level: exceed range.\n");
		idx = ALS_LEVEL_NUM - 1;
	}
	
	return als_level[idx];
#undef ALS_LEVEL_NUM
#else
	return als_data;
#endif
}

#ifdef ROHM_CALIBRATE

int rpr521_read_ps(struct i2c_client *client, u16 *data)
{
   
	int tmp = 0;

	if(client == NULL)
	{
		return -1;
	}

	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
		return tmp;
	}
	*data = (unsigned short)(tmp&0xFFF);

	return 0;    

}

int rpr521_judge_infra(struct i2c_client *client)
{
   
	int tmp = 0;
	u8 infrared_data;
	

	if(client == NULL)
	{
		return -1;
	}

	tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read ps infra data fail. \n", __func__);
		return tmp;
	}

	infrared_data = tmp;

	if(infrared_data>>6)  
	 {		
		return 0;
	 }

	return 1;

}


static int rpr_get_offset(struct ALS_PS_DATA *als_ps , uint16_t ct_value)
{
	if(ct_value <= 100)
	{
		als_ps->rpr_max_min_diff = 100;
		als_ps->rpr_ht_n_ct = 130;
		als_ps->rpr_lt_n_ct = 50;
	}
	else if(ct_value <= 300)
	{
		als_ps->rpr_max_min_diff = 200;
		als_ps->rpr_ht_n_ct = 130;
		als_ps->rpr_lt_n_ct = 50;
	}
	else if(ct_value <= 500)
	{
		als_ps->rpr_max_min_diff = 200;
		als_ps->rpr_ht_n_ct = 170;
		als_ps->rpr_lt_n_ct = 60;
	}
	else
	{
		als_ps->rpr_max_min_diff = 200;
		als_ps->rpr_ht_n_ct = 300;
		als_ps->rpr_lt_n_ct = 100;
	}
	printk(KERN_INFO "%s: change diff=%d, htnct=%d, ltnct=%d\n", __func__, als_ps->rpr_max_min_diff, als_ps->rpr_ht_n_ct,  als_ps->rpr_lt_n_ct);	
	printk("%s: change diff=%d, htnct=%d, ltnct=%d\n", __func__, als_ps->rpr_max_min_diff, als_ps->rpr_ht_n_ct,  als_ps->rpr_lt_n_ct);
	return 0;
}


static int rpr_ps_tune_zero_final(struct ALS_PS_DATA *als_ps)
{
	int res = 0;

	als_ps->tune_zero_init_proc = false;

	res = i2c_smbus_write_byte_data(als_ps->client, REG_MODECONTROL, PS_ALS_SET_MODE_CONTROL); //disable ps interrupt
	if(res< 0)	
	{	
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return res;	
	}	


	res = i2c_smbus_write_byte_data(als_ps->client, REG_INTERRUPT, PS_ALS_SET_INTR | MODE_PROXIMITY); //disable ps interrupt
	if(res< 0)	
	{	
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return res;	
	}	
		
	if(als_ps->data_count == -1)
	{
		als_ps->ps_th_h_boot = PS_ALS_SET_PS_TH;
		als_ps->ps_th_l_boot = PS_ALS_SET_PS_TL;
		als_ps->ps_th_h = als_ps->ps_th_h_boot;
		als_ps->ps_th_l = als_ps->ps_th_l_boot;
		als_ps->boot_cali = 0;
		printk(KERN_INFO "%s: exceed limit\n", __func__);
		hrtimer_cancel(&als_ps->ps_tune0_timer);	
		return 0;
	}
	
	als_ps->psa = als_ps->ps_stat_data[0];
	als_ps->psi = als_ps->ps_stat_data[2];	
	als_ps->boot_ct = als_ps->ps_stat_data[1];
	
	rpr_get_offset(als_ps, als_ps->ps_stat_data[1]);


	als_ps->ps_th_h_boot = als_ps->ps_stat_data[1] + als_ps->rpr_ht_n_ct;
	als_ps->ps_th_l_boot = als_ps->ps_stat_data[1] + als_ps->rpr_lt_n_ct;
	als_ps->boot_cali = 40;
	
	als_ps->ps_th_h = als_ps->ps_th_h_boot;
	als_ps->ps_th_l = als_ps->ps_th_l_boot;

	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;

	rpr521_set_ps_threshold_high(als_ps->client, als_ps->ps_th_h_boot);
	rpr521_set_ps_threshold_low(als_ps->client, als_ps->ps_th_l_boot);
			
	printk(KERN_INFO "%s: set HT=%d,LT=%d\n", __func__, als_ps->ps_th_h,  als_ps->ps_th_l);		
	hrtimer_cancel(&als_ps->ps_tune0_timer);					
	return 0;
}


static int rpr_tune_zero_get_ps_data(struct ALS_PS_DATA *als_ps)
{
	uint16_t ps_adc;
	int ret;
	u8 infra_flag= 0;
	
	ret = rpr521_read_ps(als_ps->client, &als_ps->ps);
	if(ret < 0)
	{		
		als_ps->data_count = -1;
		rpr_ps_tune_zero_final(als_ps);
		return 0;
	}	
	
	ps_adc = als_ps->ps;
	printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, als_ps->data_count, ps_adc);
	//if(ps_adc < 0)		
		//return ps_adc;		

	infra_flag = rpr521_judge_infra(als_ps->client);
	if(infra_flag != 1)
	{		
		als_ps->data_count = -1;
		rpr_ps_tune_zero_final(als_ps);
		return 0;
	}	
	
	als_ps->ps_stat_data[1]  +=  ps_adc;			
	if(ps_adc > als_ps->ps_stat_data[0])
		als_ps->ps_stat_data[0] = ps_adc;
	if(ps_adc < als_ps->ps_stat_data[2])
		als_ps->ps_stat_data[2] = ps_adc;						
	als_ps->data_count++;	
	
	if(als_ps->data_count == 5)
	{
		als_ps->ps_stat_data[1]  /= als_ps->data_count;			
		rpr_ps_tune_zero_final(als_ps);
	}		
	
	return 0;
}


static int rpr_ps_tune_zero_func_fae(struct ALS_PS_DATA *als_ps)
{
	uint16_t word_data;
	int ret, diff = 0;
	u8 infra_flag= 0;
			
	if(!als_ps->enable_ps_sensor)
	{
		return 0;
	}	

	ret = rpr521_read_ps(als_ps->client, &als_ps->ps);
	if(ret < 0)
	{		
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
	}	

	word_data = als_ps->ps;

	if(word_data == 0)
	{
		//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
		return 0xFFFF;
	}
	
	infra_flag = rpr521_judge_infra(als_ps->client);
	if(infra_flag != 1)
	{		
		printk(KERN_ERR "%sinvalid  infra data , infra_flag=0x%x", __func__, infra_flag);
		return 0xFFFF;
	}	

	if(word_data > als_ps->psa)
	{
		als_ps->psa = word_data;
		printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, als_ps->psa, als_ps->psi);
	}
	if(word_data < als_ps->psi)
	{			
		als_ps->psi = word_data;
		rpr_get_offset( als_ps, als_ps->psi);
		printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, als_ps->psa, als_ps->psi);	
	}	
	
	diff = als_ps->psa - als_ps->psi;
	if((als_ps->psa - als_ps->psi) > als_ps->rpr_max_min_diff)
	{
	
		als_ps->psi_set = als_ps->psi;
		
//		rpr_get_offset( als_ps, als_ps->psi);
		
		als_ps->ps_th_h = als_ps->psi + als_ps->rpr_ht_n_ct;
		als_ps->ps_th_l = als_ps->psi + als_ps->rpr_lt_n_ct;				
	
		if(als_ps->ps_th_h < als_ps->ps_th_h_boot)
		{
			als_ps->ps_th_h_boot = als_ps->ps_th_h;
			als_ps->ps_th_l_boot = als_ps->ps_th_l;

			als_ps->ps_th_h_back = als_ps->ps_th_h;
			als_ps->ps_th_l_back = als_ps->ps_th_l;

			rpr521_set_ps_threshold_high(als_ps->client, als_ps->ps_th_h_boot);
			rpr521_set_ps_threshold_low(als_ps->client, als_ps->ps_th_l_boot);

			printk(KERN_INFO "%s: update boot HT=%d, LT=%d\n", __func__,als_ps->ps_th_h_boot, als_ps->ps_th_l_boot);
		}	

		//rpr521_set_ps_threshold_high(als_ps->client, als_ps->ps_th_h_boot);
		//rpr521_set_ps_threshold_low(als_ps->client, als_ps->ps_th_l_boot);

		printk("tune0 %s: update HT=%d, LT=%d\n", __func__, als_ps->ps_th_h, als_ps->ps_th_l);		

		hrtimer_cancel(&als_ps->ps_tune0_timer);

	}
#ifdef RPR_DEBUG_PRINTF	
		printk("tune0 finsh %s: boot HT=%d, LT=%d, boot_cali=%d\n", __func__, als_ps->ps_th_h_boot, als_ps->ps_th_l_boot, als_ps->boot_cali);
#endif	
	return 0;
}

static void rpr_ps_tune0_work_func(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, rpr_ps_tune0_work);		
	if(als_ps->tune_zero_init_proc)
		rpr_tune_zero_get_ps_data(als_ps);
	else
		rpr_ps_tune_zero_func_fae(als_ps);
	return;
}	


static enum hrtimer_restart rpr_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct ALS_PS_DATA *als_ps = container_of(timer, struct ALS_PS_DATA, ps_tune0_timer);
	queue_work(als_ps->rpr_ps_tune0_wq, &als_ps->rpr_ps_tune0_work);	
	hrtimer_forward_now(&als_ps->ps_tune0_timer, als_ps->ps_tune0_delay);
	return HRTIMER_RESTART;	
}

#endif

static void rpr521_reschedule_work(struct ALS_PS_DATA *als_ps,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&als_ps->dwork);
	schedule_delayed_work(&als_ps->dwork, delay);

	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
}

/* ALS polling routine */
static void rpr521_als_polling_work_handler(struct work_struct *work)
{
	ktime_t timestamp;
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, als_dwork.work);
	struct i2c_client *client=als_ps->client;
	int tmp = 0;
	
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// restart timer
	
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data0 fail. \n", __func__);
		//return tmp;
	}
	als_ps->als_data0_raw = (unsigned short)tmp;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data1 fail. \n", __func__);
		//return tmp;
	}
	als_ps->als_data1_raw = (unsigned short)tmp;

// Theorically it is not necesssary to do so, but I just want to avoid any potential error.  -- Andy 2012.6.6
	tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read gain fail. \n", __func__);
		//return tmp;
	}
	tmp = (tmp & 0x3C) >> 2;
	als_ps->gain0 = gain_table[tmp].DATA0;
	als_ps->gain1 = gain_table[tmp].DATA1;	
	
	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	printk("REG_MODECONTROL=%x\n", tmp);

	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read time fail. \n", __func__);
		//return tmp;
	}
	tmp = tmp & 0xF;
	als_ps->als_time = mode_table[tmp].ALS;	
	
	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw, als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time);	
	if(als_ps->als_data == 0)
		als_ps->als_data ++;

	als_ps->als_level = rpr521_als_data_to_level(als_ps->als_data);

      printk(KERN_INFO "rpr521 als report: data0 = %d, data1 = %d, gain0 = %d, gain1 = %d, time = %d, lux = %d, level = %d.\n", als_ps->als_data0_raw, 
        als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time, als_ps->als_data, als_ps->als_level);

	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	printk("REG_PSDATA=%x\n", tmp);
	//tmp = i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	//printk("REG_INTERRUPT=%x\n", tmp);
	if(als_ps->als_data != CALC_ERROR)
	{
		input_report_abs(als_ps->input_dev_als, ABS_MISC, als_ps->als_level); // report als data. maybe necessary to convert to lux level
		timestamp = ktime_get_boottime();
		input_event(als_ps->input_dev_als,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
		input_event(als_ps->input_dev_als,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
		input_sync(als_ps->input_dev_als);	
	}	
}


/* PS interrupt routine */
static void rpr521_ps_int_work_handler(struct work_struct *work)
{
	ktime_t timestamp;
	struct ALS_PS_DATA *als_ps = container_of((struct delayed_work *)work, struct ALS_PS_DATA, dwork);
	struct i2c_client *client=als_ps->client;
	int tmp;

	printk(" \n\n ********** rpr521_ps_int_work_handler ********* \n\n ");

	tmp =  i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read interrupt status fail. \n", __func__);
		//return;
		goto err_exit; //grace modify in 2014.9.5
	}
	if(tmp & PS_INT_MASK)	//Interrupt is caused by PS
	{
		tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.5.7
		}
		als_ps->ledcurrent = tmp & 0x3;
		
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.5.7
		}
		als_ps->ps_data_raw = (unsigned short)tmp;

		als_ps->ps_data = calc_rohm_ps_data(als_ps->ps_data_raw);

		if(als_ps->ps_data > als_ps->ps_th_h)
		{		
			//grace modify in 2014.5.7 begin	
			tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
			if(tmp < 0)
			{
				printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
				goto err_exit; 
			}
			if ((tmp>>6) == INFRARED_LOW)
			{
				als_ps->ps_direction = 0;
#ifdef _AUTO_THRESHOLD_CHANGE_
				rpr521_set_ps_threshold_high(client, REG_PSTH_MAX);
				rpr521_set_ps_threshold_low(client, als_ps->ps_th_l_back);
#endif
			}
			else
			{
				goto err_exit;
			}
			//grace modify in 2014.5.7 end			
		}
		else if(als_ps->ps_data < als_ps->ps_th_l)
		{
			als_ps->ps_direction = 1;
#ifdef _AUTO_THRESHOLD_CHANGE_
			rpr521_set_ps_threshold_high(client, als_ps->ps_th_h_back);
			rpr521_set_ps_threshold_low(client, 0);
#endif
		}

		tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read persistence fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.9.5
		}

		printk(KERN_INFO "rpr521 ps report: raw_data = %d, data = %d, direction = %d, 0x43 = %d. \n", 
				als_ps->ps_data_raw, als_ps->ps_data, als_ps->ps_direction, tmp);
	         printk(KERN_INFO "rpr521 ps report: ps_high = %d, ps_low = %d, ps_high_back = %d, ps_low_back = %d. \n",
                                i2c_smbus_read_word_data(client, REG_PSTH), i2c_smbus_read_word_data(client, REG_PSTL), als_ps->ps_th_h_back, als_ps->ps_th_l_back );


		input_report_abs(als_ps->input_dev_ps, ABS_DISTANCE, als_ps->ps_direction); 
		timestamp = ktime_get_boottime();
		input_event(als_ps->input_dev_ps,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
		input_event(als_ps->input_dev_ps,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
		input_sync(als_ps->input_dev_ps);	
	}
	else
	{
		printk(KERN_ERR "%s: unknown interrupt source.\n", __func__);
	}
	
	enable_irq(client->irq);
	
//grace modify in 2014.5.7 begin
err_exit:
	enable_irq(client->irq);
//grace modify in 2014.5.7 end
}

/* assume this is ISR */
static irqreturn_t rpr521_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	disable_irq_nosync(client->irq);
	printk("%s\n", __func__);
	rpr521_reschedule_work(als_ps, 0);

	return IRQ_HANDLED;
}

/*************** SysFS Support ******************/
static ssize_t rpr521_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_ps_sensor);
}

static ssize_t rpr521_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
// 	unsigned long flags;
	int tmp;
//	struct input_dev *input_dev = als_ps->input_dev_ps;

	printk(KERN_INFO "rpr521 enable PS sensor -> %ld \n", val);
		
	if ((val != 0) && (val != 1)) 
	{
		printk("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}
	
	//mutex_lock(&input_dev->mutex);
	if(val == 1) 
	{
		//turn on p sensor
		//wake_lock(&ps_lock);

		if (als_ps->enable_ps_sensor == 0) 
		{
			als_ps->enable_ps_sensor = 1;
		
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			printk("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x40;

#ifdef _ENABLE_CALIB_  //grace modify in 2014.12.12
			rpr521_calibrate(client);
#endif
			i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
			rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor == 1)
		{
			als_ps->enable_ps_sensor = 0;
			
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x40);
			rpr521_set_enable(client, tmp);	//PS on and ALS off
			
			//wake_unlock(&ps_lock);
		}
	}

	//mutex_unlock(&input_dev->mutex);	
	return count;
}

static void rpr521_check_enable(struct i2c_client *client)
{
        int enable_status;

        struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

        enable_status = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
        if(enable_status & PS_EN)
        {
                   enable_status = enable_status | PS_ALS_SET_MODE_CONTROL;
                   rpr521_set_enable(client, enable_status);
                   als_ps->enable_ps_sensor = 1;
        }
        else
        {
                enable_status = enable_status | PS_ALS_SET_MODE_CONTROL | PS_EN;
                rpr521_set_enable(client, enable_status);
                als_ps->enable_ps_sensor = 1;
        }
}


/* add by xx for sensor class */
static int rpr521_prox_enable(struct ALS_PS_DATA *als_ps, int on)
{
	struct i2c_client *client = als_ps->client;
	int tmp = 0;
	int rc = 0;

#ifdef ROHM_CALIBRATE
	if (!on)
	{
		hrtimer_cancel(&als_ps->ps_tune0_timer);					
		cancel_work_sync(&als_ps->rpr_ps_tune0_work);
	}

	if(als_ps->first_boot == true)
	{		
		als_ps->first_boot = false;
	}
#endif
			
	if(on == 1) 
	{
		if (als_ps->enable_ps_sensor == 0) 
		{
                        rpr521_check_enable(client);
			als_ps->enable_ps_sensor = 1;
		
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			printk("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x40;

#ifdef _ENABLE_CALIB_  //grace modify in 2014.12.12
			rpr521_calibrate(client);
#endif
			printk("%s: tmp1=%x\n", __func__, tmp);
			i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
			rc = rpr521_set_enable(client, tmp);	//PS on and ALS off

#ifdef ROHM_CALIBRATE

			als_ps->psi_set = 0;
			als_ps->psa = 0;		
		
			als_ps->ps_th_h = als_ps->ps_th_h_boot;
			als_ps->ps_th_l = als_ps->ps_th_l_boot;	

			als_ps->ps_th_h_back = als_ps->ps_th_h;
			als_ps->ps_th_l_back = als_ps->ps_th_l;
				
			hrtimer_start(&als_ps->ps_tune0_timer, als_ps->ps_tune0_delay, HRTIMER_MODE_REL);					
			rpr521_set_ps_threshold_high(als_ps->client, als_ps->ps_th_h);
			rpr521_set_ps_threshold_low(als_ps->client, als_ps->ps_th_l);
	
#endif			

		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor == 1)
		{
			als_ps->enable_ps_sensor = 0;
			
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x40);
			rc = rpr521_set_enable(client, tmp);	//PS on and ALS off
			
		}
	}	
	return rc;
}
/* add by xx for sensor class end*/

static ssize_t rpr521_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_als_sensor);
}

static ssize_t rpr521_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned long flags;
	int tmp;
	
	printk(KERN_INFO "rpr521 enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			als_ps->enable_als_sensor = 1;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
                        printk("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x80;
			rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
       	cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor == 1)
		{
			als_ps->enable_als_sensor = 0;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x80);
			rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	
	return count;
}

/* add by xx for sensor class */
static int rpr521_als_enable(struct ALS_PS_DATA *als_ps, int on)
{
	struct i2c_client *client = als_ps->client;
	unsigned long flags;
	int tmp = 0;
	int rc = 0;
	
	if(on == 1)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			als_ps->enable_als_sensor = 1;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
                        printk("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x80;
			rc = rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
       		cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor == 1)
		{
			als_ps->enable_als_sensor = 0;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x80);
			rc = rpr521_set_enable(client, tmp);	//PS on and ALS off
		}
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	return rc;	
}
/* add by xx for sensor class end */


static ssize_t rpr521_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->als_poll_delay*1000);	// return in micro-second
}

static ssize_t rpr521_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
//	int ret;
//	int poll_delay = 0;
 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;	
	
	als_ps->als_poll_delay = val /1000;	// convert us => ms
	
	if (als_ps->enable_als_sensor == 1)//grace modified in 2013.10.09
	{
	
	/* we need this polling timer routine for sunlight canellation */
	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&als_ps->als_dwork);
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
			
	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	
	}
	return count;
}

//grace add in 2013.10.09 begin
static ssize_t rpr521_show_als_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp;
	int tmp1;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	tmp1 = i2c_smbus_read_word_data(client, REG_ALSDATA1);

	return sprintf(buf, "%d %d\n", tmp, tmp1);
}

static ssize_t rpr521_show_ps_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp = 0;
	int tmp1 = 0;
        int sys_mode = 0, mode = 0, als_ps_cont = 0, persist = 0, ch0 = 0, ch1 = 0, ps_l = 0, ps_h = 0;

        tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
        tmp1 = i2c_smbus_read_word_data(client, REG_INTERRUPT);
        sys_mode = i2c_smbus_read_byte_data(client, REG_SYSTEMCONTROL);
        mode = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
        als_ps_cont = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
        persist = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
        ch0 = i2c_smbus_read_word_data(client, REG_ALSDATA0);
        ch1 = i2c_smbus_read_word_data(client, REG_ALSDATA1);
        ps_l = i2c_smbus_read_word_data(client, REG_PSTL);
        ps_h = i2c_smbus_read_word_data(client, REG_PSTH);

	return sprintf(buf, "code=%d int=%x sys_mode=%x mode=%x als_ps_cont=%x persist=%x  ch0=%d ch1=%d ps_l=%d ps_h=%d \n", tmp, tmp1, sys_mode , mode, als_ps_cont, persist, ch0, ch1, ps_l, ps_h);
}
//grace add in 2013.10.09 end

static ssize_t rpr521_show_ps_thres_high(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	//grace test in 2013.10.09 begin
	int ps_data = 0, ps_high = 0, ps_low = 0;
	ps_data = i2c_smbus_read_word_data(client, REG_PSDATA);
	if(ps_data < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_high = i2c_smbus_read_word_data(client, REG_PSTH);
	if(ps_high < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_low = i2c_smbus_read_word_data(client, REG_PSTL);
	if(ps_low < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	//return sprintf(buf, "%d\n", als_ps->ps_th_h);	
	return sprintf(buf, "%d %d %d\n", ps_data, ps_high, ps_low);
	//grace test in 2013.10.09 end
}

static ssize_t rpr521_store_ps_thres_high(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	if(!(rpr521_set_ps_threshold_high(client, val)))
		als_ps->ps_th_h_back = als_ps->ps_th_h;
	
	return count;
}

static ssize_t rpr521_show_ps_thres_low(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->ps_th_l);	
}

static ssize_t rpr521_store_ps_thres_low(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	if(!(rpr521_set_ps_threshold_low(client, val)))
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	
	return count;
}

static ssize_t rpr521_show_ps_calib(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\t%d\n", als_ps->ps_th_h, als_ps->ps_th_l);	
}

static ssize_t rpr521_store_ps_calib(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
#define SET_LOW_THRES	1
#define SET_HIGH_THRES	2
#define SET_BOTH_THRES	3

	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned int i, tmp, ps_th_h, ps_th_l;
	int average;	//This should be signed to avoid error.
	
	switch(val)
	{
		case SET_LOW_THRES:		
		//Take 20 average for noise. use noise + THRES_TOLERANCE as low threshold.
		//If high threshold is lower than the new low threshold + THRES_DIFF, make the high one equal low + THRES_DIFF
		//Please make sure that there is NO object above the sensor, otherwise it may cause the high threshold too high to trigger which make the LCD NEVER shutdown.
		//If the noise is too large, larger than 4065, it will return -1. If so, the mechanical design MUST be redo. It is quite unlikely. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average noise
			ps_th_l = average + THRES_TOLERANCE;	
			if(ps_th_l > REG_PSTL_MAX)
			{
			//printk(KERN_ERR "%d in %s: low threshold is too high. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_l < 0)
			{
			//printk(KERN_ERR "%d in %s: low threshold is too low. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_h < ps_th_l + THRES_DIFF)
			{
				ps_th_h = ps_th_l + THRES_DIFF;	//It will not be minus or an error should have occured earlier. 
				if(ps_th_h > REG_PSTH_MAX)
				{
			//printk(KERN_ERR "%d in %s: high threshold is too high. \n", __line__, __func__);
					return -1;
				}
				if(!rpr521_set_ps_threshold_high(client, ps_th_h))
					als_ps->ps_th_h_back = ps_th_h;
				else
					return -1;
			}
			if(!rpr521_set_ps_threshold_low(client, ps_th_l))
				als_ps->ps_th_l_back = ps_th_l;
			else
				return -1;
			break;
		
		case SET_HIGH_THRES:	
		//Take 20 average for signal. use signal -THRES_TOLERANCE as high threshold. 
		//If low threshold is higher than the new high one - THRES_DIFF, make the low one equal high - THRES_DIFF
		//Please make sure that there IS AN object above the sensor, otherwise it will be a disaster. The high threshold will be too low which will cause the panel ALWAYS shutdown
		//Customer can use their own standard to set the test scenario. For example, a 18% grey card @ 2cm, or another example, a 90% white card 4cm, and etc. 
		//If the signal is too weak, less than 30, it will return -1. If so, the mechanical design MUST be redo. It shall not happen very frequently. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average signal
			ps_th_h = average - THRES_TOLERANCE;
			if(ps_th_h > REG_PSTH_MAX)
			{
			//printk(KERN_ERR "%d in %s: high threshold is too high. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_h < 0)
			{
			//printk(KERN_ERR "%d in %s: high threshold is too low. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_l > ps_th_h - THRES_DIFF)
			{
				ps_th_l = ps_th_h - THRES_DIFF;	//Given that REG_PSTH_MAX = REG_PSTL+MAX, it will not be greater than REG_PSTL_MAX or an error should have occured earlier.
				if(ps_th_l < 0)
				{
					//printk(KERN_ERR "%d in %s: low threshold is too low. \n", __line__, __func__);
					return -1;
				}
				if(!rpr521_set_ps_threshold_low(client, ps_th_l))
					als_ps->ps_th_l_back = ps_th_l;
				else
					return -1;
			}
			if(!rpr521_set_ps_threshold_high(client, ps_th_h))
				als_ps->ps_th_h_back = ps_th_h;
			else
				return -1;
			break;
		
		case SET_BOTH_THRES:	//Take 20 average for noise. use noise + PS_ALS_SET_PS_TL as low threshold, noise + PS_ALS_SET_PS_TH as high threshold
			rpr521_calibrate(client);
			break;

		default:
			return -EINVAL;	//NOT supported!
	}
			
	return count;

#undef SET_BOTH_THRES
#undef SET_HIGH_THRES
#undef SET_LOW_THRES
}

static ssize_t rpr521_show_type(struct device *dev,
                struct device_attribute *attr, char *buf){
    struct i2c_client *client = to_i2c_client(dev);
    struct ALS_PS_DATA *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->type);
}
static DEVICE_ATTR(als_poll_delay,  0660,
				    rpr521_show_als_poll_delay, rpr521_store_als_poll_delay);

static DEVICE_ATTR(enable_als_sensor,  0660 ,
				  rpr521_show_enable_als_sensor, rpr521_store_enable_als_sensor);

static DEVICE_ATTR(enable_ps_sensor,  0660 ,
				   rpr521_show_enable_ps_sensor, rpr521_store_enable_ps_sensor);

static DEVICE_ATTR(ps_thres_high,  0660 ,
				  rpr521_show_ps_thres_high, rpr521_store_ps_thres_high);

static DEVICE_ATTR(ps_thres_low,  0660 ,
				   rpr521_show_ps_thres_low, rpr521_store_ps_thres_low);

static DEVICE_ATTR(ps_calib,  0660 ,
				   rpr521_show_ps_calib, rpr521_store_ps_calib);
static DEVICE_ATTR(als_data, S_IRUGO, rpr521_show_als_data, NULL); //grace add in 2013.10.09
static DEVICE_ATTR(ps_data, S_IRUGO, rpr521_show_ps_data, NULL);//grace add in 2013.10.09
static DEVICE_ATTR(type, S_IRUGO, rpr521_show_type, NULL);//Add for EngineerMode

static struct attribute *rpr521_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_ps_thres_high.attr,
	&dev_attr_ps_thres_low.attr,
	&dev_attr_ps_calib.attr,
	&dev_attr_als_data.attr,  //grace add in 2013.10.09
	&dev_attr_ps_data.attr,   //grace add in 2013.10.09
	&dev_attr_type.attr,
	NULL
};

static const struct attribute_group rpr521_attr_group = {
	.attrs = rpr521_attributes,
};

/*************** Initialze Functions ******************/
static int rpr521_init_client(struct i2c_client *client)
{
    struct init_func_write_data {
        unsigned char mode_ctl;
        unsigned char psals_ctl;
        unsigned char persist;
        unsigned char reserved0;
        unsigned char reserved1;
        unsigned char reserved2;
        unsigned char reserved3;
        unsigned char reserved4;
        unsigned char reserved5;
        unsigned char intr;
        unsigned char psth_hl;
        unsigned char psth_hh;
        unsigned char psth_ll;
        unsigned char psth_lh;
        unsigned char alsth_hl;
        unsigned char alsth_hh;
        unsigned char alsth_ll;
        unsigned char alsth_lh;
    } write_data;
    int result;
    unsigned char gain;

    unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
    unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
    unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
    unsigned char intr        = PS_ALS_SET_INTR;
    unsigned short psth_upper  = PS_ALS_SET_PS_TH;
    unsigned short psth_low    = PS_ALS_SET_PS_TL;
    unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
    unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

    struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
    /* execute software reset */
    result =  i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, 0xC0);	//soft-reset
    if (result != 0) {
        return (result);
    }

    write_data.mode_ctl  = mode_ctl;
    write_data.psals_ctl = psals_ctl;
    write_data.persist   = persist;
    write_data.reserved0 = 0;
    write_data.reserved1 = 0;
    write_data.reserved2 = 0;
    write_data.reserved3 = 0;
    write_data.reserved4 = 0;
    write_data.reserved5 = 0;
    write_data.intr      = intr;
    write_data.psth_hl   = CONVERT_TO_BE(psth_upper) & MASK_CHAR;
    write_data.psth_hh   = CONVERT_TO_BE(psth_upper) >> 8;
    write_data.psth_ll   = CONVERT_TO_BE(psth_low) & MASK_CHAR;
    write_data.psth_lh   = CONVERT_TO_BE(psth_low) >> 8;
    write_data.alsth_hl  = CONVERT_TO_BE(alsth_upper) & MASK_CHAR;
    write_data.alsth_hh  = CONVERT_TO_BE(alsth_upper) >> 8;
    write_data.alsth_ll  = CONVERT_TO_BE(alsth_low) & MASK_CHAR;
    write_data.alsth_lh  = CONVERT_TO_BE(alsth_low) >> 8;
    result               = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL, sizeof(write_data), (unsigned char *)&write_data);

	if(result < 0)
	{
		printk(KERN_ERR "%s: i2c write fail. \n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;	//gain setting values
	
	als_ps->enable = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->ps_th_l = psth_low;
	als_ps->ps_th_h = psth_upper;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;
		
	als_ps->enable_als_sensor = 0; //grace add in 2013.10.09
	als_ps->enable_ps_sensor = 0;  //grace add in 2013.10.09

#ifdef _INIT_CALIB_
	
  //grace modify in 2014.12.12 begin
	if ((!rpr521_calibrate(client))&&calib_status)
	{
		init_ps_high = als_ps->ps_th_h;
		init_ps_low = als_ps->ps_th_l;
		init_calib_flag =1;
	}
	else
	{
		als_ps->ps_th_h_back = als_ps->ps_th_h;
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	}
	//grace modify in 2014.12.12 end
#else

#ifdef ROHM_CALIBRATE

	als_ps->psi_set = 0;	
	als_ps->ps_stat_data[0] = 0;
	als_ps->ps_stat_data[2] = 4095;
	als_ps->ps_stat_data[1] = 0;
	als_ps->data_count = 0;
	als_ps->ps_th_h_boot = PS_ALS_SET_PS_TH;
	als_ps->ps_th_l_boot = PS_ALS_SET_PS_TL;
	als_ps->tune_zero_init_proc = true;	
	als_ps->boot_ct = 0xFFF;
	als_ps->ps_nf = 1;
	als_ps->first_boot = true;	

	rpr521_set_ps_threshold_high(client, als_ps->ps_th_h_boot);
	rpr521_set_ps_threshold_low(client, als_ps->ps_th_l_boot);

	result =  i2c_smbus_write_byte_data(client, REG_MODECONTROL, PS_ALS_SET_MODE_CONTROL|PS_EN);	//soft-reset
    	if (result != 0) {
        	return (result);
    	}

	hrtimer_start(&als_ps->ps_tune0_timer, als_ps->ps_tune0_delay, HRTIMER_MODE_REL);	
	
#endif

	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;


#endif

    return (result);
}

static int rpr_power_on(struct ALS_PS_DATA *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}
static int rpr_power_init(struct ALS_PS_DATA *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
	
	data->vcc_i2c = regulator_get(&data->client->dev, "vio");	//vcc-i2c not defined, instead of vio
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}


#ifdef CONFIG_TCT_8X16_IDOL347

static ssize_t rpr_device_prx_detected(struct class *class,
				struct class_attribute *attr, char *buf)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(g_client);
	uint32_t reading;
	
        rpr521_prox_enable(als_ps, 1);
	msleep(10);
	reading = i2c_smbus_read_word_data(g_client, REG_PSDATA);
	rpr521_prox_enable(als_ps, 1);

        if (reading > 200)
                reading = 1;
        else
                reading = 0;
 
	return scnprintf(buf, PAGE_SIZE, "%d\n", reading);	
}

static struct class_attribute prx_status =
	__ATTR(status, 0444, rpr_device_prx_detected, NULL);

static int rpr_prx_misoperation_creat_file(void)
{
	int ret;

	/*  /<sysfs>/class/prx_misoperation/status */

	rpr_prx_misoperation_class = class_create(THIS_MODULE, "prx_misoperation");
	if (IS_ERR(rpr_prx_misoperation_class)) {
		ret = PTR_ERR(rpr_prx_misoperation_class);
		printk(KERN_ERR "prx_misoperation_class: couldn't create prx_misoperation\n");
	}
	ret = class_create_file(rpr_prx_misoperation_class, &prx_status);
	if (ret) {
		printk(KERN_ERR "prx_misoperation: couldn't create status\n");
	}

	return ret;

}

static void rpr_remove_prx_create_file(void)
{
	class_remove_file(rpr_prx_misoperation_class, &prx_status);
}

#endif


static int rpr521_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (4095)

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ALS_PS_DATA *als_ps;
	//unsigned long flags;
	struct device_node *np = client->dev.of_node;
	ktime_t timestamp;
	int err = 0;
	int dev_id;
	//wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");

        #ifdef CONFIG_TCT_8X16_IDOL347
	g_client=client;
	#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	als_ps = kzalloc(sizeof(struct ALS_PS_DATA), GFP_KERNEL);
	if (!als_ps) {
		err = -ENOMEM;
		goto exit;
	}
	als_ps->client = client;
	i2c_set_clientdata(client, als_ps);

	printk("\n\n ***** client is %p ***** \n\n", client);			//debug
	//suc
	
	err = rpr_power_init(als_ps, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
	}

	//suc										
	
	err = rpr_power_on(als_ps, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
	}

	//suc

//	printk("enable = %x\n", als_ps->enable);

	//check whether is rpr521 or apds9930
	dev_id = i2c_smbus_read_byte_data(client, REG_MANUFACT_ID);
	if(dev_id != 0xE0){
	    kfree(als_ps);
	    return -1;
	}
	als_ps->type = 1;
	printk("%s: id(0x%x), this is rpr521!\n", __func__, dev_id);

	mutex_init(&als_ps->update_lock);

	INIT_DELAYED_WORK(&als_ps->dwork, rpr521_ps_int_work_handler);
	INIT_DELAYED_WORK(&als_ps->als_dwork, rpr521_als_polling_work_handler); 

#ifdef ROHM_CALIBRATE
	als_ps->rpr_ps_tune0_wq = create_singlethread_workqueue("rpr_ps_tune0_wq");
	INIT_WORK(&als_ps->rpr_ps_tune0_work, rpr_ps_tune0_work_func);
	hrtimer_init(&als_ps->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	als_ps->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	als_ps->ps_tune0_timer.function = rpr_ps_tune0_timer_func;
#endif
	
//	printk("%s interrupt is hooked\n", __func__);

	// fail
	/* Initialize the RPR400 chip */
	err = rpr521_init_client(client);
	if (err)
		goto exit_kfree;
	
	als_ps->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;	

	/* Register to Input Device */
	als_ps->input_dev_als = input_allocate_device();
	if (!als_ps->input_dev_als) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device als\n", __func__);
		goto exit_free_irq;
	}

	als_ps->input_dev_ps = input_allocate_device();
	if (!als_ps->input_dev_ps) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device ps\n", __func__);
		goto exit_free_dev_als;
	}
	
	input_set_drvdata(als_ps->input_dev_ps, als_ps); //grace modified in 2013.9.24
	input_set_drvdata(als_ps->input_dev_als, als_ps); //grace modified in 2013.9.24
	
	set_bit(EV_ABS, als_ps->input_dev_als->evbit);
	set_bit(EV_ABS, als_ps->input_dev_ps->evbit);

	input_set_abs_params(als_ps->input_dev_als, ABS_MISC, 0, ROHM_PSALS_ALSMAX, 0, 0);
	input_set_abs_params(als_ps->input_dev_ps, ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

	als_ps->input_dev_als->name = "light";
	als_ps->input_dev_ps->name = "proximity";
	//grace modified in 2013.3.22 begin
	als_ps->input_dev_als->id.bustype = BUS_I2C;
  als_ps->input_dev_als->dev.parent =&als_ps->client->dev;
  als_ps->input_dev_ps->id.bustype = BUS_I2C;
  als_ps->input_dev_ps->dev.parent =&als_ps->client->dev;
  //grace modified in 2013.3.22 begin


	err = input_register_device(als_ps->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device als: %s\n", __func__, 
		       als_ps->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(als_ps->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device ps: %s\n", __func__, 
		       als_ps->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rpr521_attr_group);
	if (err)
	{
		printk("%s sysfs_create_groupX\n", __func__);
		goto exit_unregister_dev_ps;
	}

	als_ps->irq_gpio = of_get_named_gpio_flags(np, "rpr,irq-gpio",
				0, &als_ps->irq_gpio_flags);
	if (als_ps->irq_gpio < 0)
		return als_ps->irq_gpio;
	
	if (gpio_is_valid(als_ps->irq_gpio)) {
		err = gpio_request(als_ps->irq_gpio, "rpr_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
		}

		err = gpio_direction_input(als_ps->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
		}
	}
	
	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	if (request_irq(client->irq, rpr521_interrupt, IRQF_TRIGGER_FALLING/*IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING*/,
		RPR521_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate rpr521_INT !\n", __func__);
	
		goto exit_kfree;
	}

	irq_set_irq_wake(client->irq, 1);

	/* add by xx for sensor class */
	/* Register to sensors class */
	als_ps->als_cdev = sensors_light_cdev;
	als_ps->als_cdev.sensors_enable = rpr521_als_set_enable;
	als_ps->als_cdev.sensors_poll_delay = NULL;
	als_ps->ps_cdev = sensors_proximity_cdev;
	als_ps->ps_cdev.sensors_enable = rpr521_ps_set_enable;
	als_ps->ps_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&client->dev, &als_ps->als_cdev);
	if (err) {
		dev_err(&client->dev,"%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto exit_unregister_als_ioctl;
	}

	err = sensors_classdev_register(&client->dev, &als_ps->ps_cdev);
	if (err) {
		dev_err(&client->dev,"%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto exit_unregister_ps_class;
	}
	/* add by xx for sensor class end*/

        #ifdef CONFIG_TCT_8X16_IDOL347
	err = rpr_prx_misoperation_creat_file();
	if(err)
		goto prx_misoperation_creat_fail;
        #endif
		
	input_report_abs(als_ps->input_dev_ps, ABS_DISTANCE, 1); //grace modify in 2014.12.16 to avoid cover in calling period
	timestamp = ktime_get_boottime();
	input_event(als_ps->input_dev_ps,EV_SYN, SYN_TIME_SEC,ktime_to_timespec(timestamp).tv_sec);
	input_event(als_ps->input_dev_ps,EV_SYN, SYN_TIME_NSEC,ktime_to_timespec(timestamp).tv_nsec);
	input_sync(als_ps->input_dev_ps);	//grace modify in 2014.12.16	
	printk(KERN_INFO "%s: INT No. %d", __func__, client->irq);
	return 0;

#ifdef CONFIG_TCT_8X16_IDOL347
prx_misoperation_creat_fail:
	rpr_remove_prx_create_file();
#endif
exit_unregister_ps_class:
	sensors_classdev_unregister(&als_ps->ps_cdev);
exit_unregister_als_ioctl:
	sensors_classdev_unregister(&als_ps->als_cdev);
exit_unregister_dev_ps:
	input_unregister_device(als_ps->input_dev_ps);	
exit_unregister_dev_als:
	printk("%s exit_unregister_dev_als:\n", __func__);
	input_unregister_device(als_ps->input_dev_als);
exit_free_dev_ps:
	input_free_device(als_ps->input_dev_ps);
exit_free_dev_als:
	input_free_device(als_ps->input_dev_als);
exit_free_irq:
	free_irq(client->irq, client);	
exit_kfree:
	kfree(als_ps);
exit:
	return err;

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

static int rpr521_remove(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	input_unregister_device(als_ps->input_dev_als);
	input_unregister_device(als_ps->input_dev_ps);
	
	input_free_device(als_ps->input_dev_als);
	input_free_device(als_ps->input_dev_ps);

	free_irq(client->irq, client);

	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);

	/* Power down the device */
	rpr521_set_enable(client, 0);

	kfree(als_ps);

	return 0;
}

static int rpr521_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s\n", __func__);

	disable_irq(client->irq);
	//wake_unlock(&ps_lock);
	//return rpr521_set_enable(client, 0);
	return rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.7
}

static int rpr521_resume(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int tmp = PS_ALS_SET_MODE_CONTROL;
	
	printk("%s \n", __func__);
	
	enable_irq(client->irq);
	if(als_ps->enable_als_sensor == 1)
		tmp = tmp | ALS_EN;
	if(als_ps->enable_ps_sensor == 1)	
		tmp = tmp | PS_EN;
	return rpr521_set_enable(client, tmp);
}


MODULE_DEVICE_TABLE(i2c, rpr521_id);

static const struct i2c_device_id rpr521_id[] = {
	{ "rpr521", 0 },
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id rpr_match_table[] = {
                { .compatible = "rpr,rpr410",},
                { },
        };
#else
#define rpr_match_table NULL
#endif 

static struct i2c_driver rpr521_driver = {
	.driver = {
		.name	= RPR521_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table =rpr_match_table,
	},
	.suspend = rpr521_suspend,
	.resume	= rpr521_resume,
	.probe	= rpr521_probe,
	.remove	= rpr521_remove,
	.id_table = rpr521_id,
};

static int __init rpr521_init(void)
{
	return i2c_add_driver(&rpr521_driver);
}

static void __exit rpr521_exit(void)
{
	i2c_del_driver(&rpr521_driver);
}

MODULE_AUTHOR("Andy Mi @ ROHM");
MODULE_DESCRIPTION("rpr521 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr521_init);
module_exit(rpr521_exit);
