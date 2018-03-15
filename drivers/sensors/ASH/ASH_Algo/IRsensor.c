/* 
 * Copyright (C) 2014 ASUSTek Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/input/ASH.h>
#include "IRsensor.h"

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME		"IRsensor"

#undef dbg
#ifdef ASH_ALGO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#define err(fmt, args...) do{	\
		printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
		sprintf(g_error_mesg, "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
	}while(0)

/******************************/
/* IR Sensor Global Variables */
/*****************************/
static int ASUS_IR_SENSOR_IRQ;
static int ASUS_IR_SENSOR_INT;
static struct ASUS_light_sensor_data			*g_als_data;
static struct ASUS_proximity_sensor_data	*g_ps_data;
static struct IRsensor_hw						*IRsensor_hw_client;
static struct workqueue_struct 					*IRsensor_workqueue;
static struct workqueue_struct 					*IRsensor_delay_workqueue;
static struct mutex 								g_ir_lock;
static struct wake_lock 							g_ir_wake_lock;
static struct hrtimer 							g_ir_timer;

static int g_als_last_lux = 0;
static char *g_error_mesg;

/***********************/
/* IR Sensor Functions*/
/**********************/
/*Device Layer Part*/
static int 	proximity_turn_onoff(bool bOn);
static int 	proximity_set_threshold(void);
static void proximity_polling_adc(struct work_struct *work);
static int 	light_turn_onoff(bool bOn);
static int 	light_get_lux(int adc);
static int 	light_get_shift(void);
static int 	light_get_accuracy_gain(void);
static void light_polling_lux(struct work_struct *work);

/*Interrupt Service Routine Part*/
static void IRsensor_ist(struct work_struct *work);
static void proximity_autok(struct work_struct *work);

/* Export Functions */
bool proximity_check_status(void);

/*Initialization Part*/
static int init_data(void);

/*Proximity auto calibration*/
static void proximity_autok(struct work_struct *work);

/*Work Queue*/
static 		DECLARE_WORK(IRsensor_ist_work, IRsensor_ist);
static 		DECLARE_WORK(proximity_autok_work, proximity_autok);
static 		DECLARE_DELAYED_WORK(proximity_polling_adc_work, proximity_polling_adc);
static 		DECLARE_DELAYED_WORK(light_polling_lux_work, light_polling_lux);

/*Disable touch for detecting near when phone call*/
extern void ftxxxx_disable_touch(bool flag);
extern int get_audiomode(void);

/*Proximity auto calibration*/
static int proximity_check_minCT(void);

/*******************************/
/* ALS and PS data structure */
/******************************/
struct ASUS_light_sensor_data 
{	
	int g_als_calvalue_200lux;				/* Lightsensor 200lux calibration value(adc) */
	int g_als_calvalue_1000lux;				/* Lightsensor 1000lux calibration value(adc) */
	int g_als_calvalue_shift;					/* Lightsensor Shift calibration value */
	int g_als_change_sensitivity;			/* Lightsensor Change sensitivity */
	int g_als_log_threshold;					/* Lightsensor Log Print Threshold */

	bool HAL_switch_on;						/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */

	int int_counter;
	int event_counter;
};

struct ASUS_proximity_sensor_data 
{	
	int g_ps_calvalue_lo;						/* Proximitysensor setting low calibration value(adc) */
	int g_ps_calvalue_hi;						/* Proximitysensor setting high calibration value(adc) */
	int g_ps_calvalue_inf;						/* Proximitysensor setting inf calibration value(adc) */

	int g_ps_autok_min;
	int g_ps_autok_max;
	
	bool HAL_switch_on;						/* this var. means if HAL is turning on ps or not */
	bool Device_switch_on;					/* this var. means is turning on ps or not */	
	bool polling_mode;							/* Polling for adc of proximity */
	bool autok;							/*auto calibration status*/

	int int_counter;
	int event_counter;
	int crosstalk_diff;
};

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int IRsensor_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;	
	int ret = 0;
	int adc = 0;
	int low_threshold = 0;
	int high_threshold = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff== NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff== NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/* Turn on Proximity and Light Sensor */
	if(!g_ps_data->Device_switch_on) {
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
	}
	if(!g_als_data->Device_switch_on) {
		ret = IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
	}

	/* Proximity i2c read test */
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to get adc\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Proximity i2c write test */
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set high threshold.\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set low threshold. \n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Light Sensor i2c read test */
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	if(adc < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get adc\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	/* Light Sensor Low Threshold */	
	low_threshold = adc * (100 - LIGHT_CHANGE_MID_SENSITIVITY) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + LIGHT_CHANGE_MID_SENSITIVITY) / 100;	
	if (high_threshold > IRsensor_hw_client->mlsensor_hw->light_max_threshold)	
		high_threshold = IRsensor_hw_client->mlsensor_hw->light_max_threshold;	
	
	ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set high threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}
	
	ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set low threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;	
	}

	if(!g_ps_data->HAL_switch_on) {
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
	}
	if(!g_als_data->HAL_switch_on) {
		ret = IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
	}
	
	i2c_log_in_test_case("TestLSensorI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info IRSensorTestCaseInfo[] =	{
	__I2C_STRESS_TEST_CASE_ATTR(IRsensor_I2C_stress_test),
};
#endif

/*====================
 *|| Device Layer Part ||
 *====================
 */ 
static int proximity_turn_onoff(bool bOn)
{
	int ret = 0;	
	ktime_t autok_delay;

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff == NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */
		/*Set Proximity Threshold*/
		ret = proximity_set_threshold();
		if (ret < 0) {	
			err("proximity_set_threshold ERROR\n");
			return ret;
		}

		/*check the min for auto calibration*/
		if(true == g_ps_data->autok){
			g_ps_data->crosstalk_diff = 0;
			/*Stage 1 : check first 6 adc which spend about 50ms~100ms*/
			ret = proximity_check_minCT();
			if (ret < 0) {	
				log("proximity_check_minCT ERROR\n");	
				g_ps_data->autok = false;
			}
		}
		
		/*set turn on register*/
		if(g_ps_data->Device_switch_on == false){
			ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
			if(ret < 0){
				err("proximity_hw_turn_onoff(true) ERROR\n");
				return ret;
			}
				
		}
				
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		/*change the Device Status*/
		g_ps_data->Device_switch_on = true;
		/*check the polling mode*/
		if(g_ps_data->polling_mode == true) {
			queue_delayed_work(IRsensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
			log("[Polling] Proximity polling adc START. \n");
		}

		/*Stage 2 : start polling proximity adc(500ms) to check min value*/
		if(true == g_ps_data->autok && g_ps_data->crosstalk_diff != 0){
			autok_delay = ns_to_ktime( PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
			hrtimer_start(&g_ir_timer, autok_delay, HRTIMER_MODE_REL);
		}
		
	} else	{	/* power off */
		/*set turn off register*/
		if(g_ps_data->Device_switch_on == true){
			/*disable IRQ before switch off*/
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
		
			ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
			if(ret < 0){
				err("proximity_hw_turn_onoff(false) ERROR\n");
			}
			/*change the Device Status*/
			g_ps_data->Device_switch_on = false;			

			/*enable IRQ when light sensor is ON*/
			if (g_als_data->Device_switch_on == true) {
				dbg("[IRQ] Enable irq !! \n");
				enable_irq(ASUS_IR_SENSOR_IRQ);
			}

			/*diable the timer*/
			if(g_ps_data->autok == true){
				hrtimer_cancel(&g_ir_timer);
			}
		}		
	}	
	
	return ret;
}

static int proximity_set_threshold(void)
{
	int ret = 0;	

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/*Set Proximity High Threshold*/
	ret = psensor_sysfs_read_high();
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_hi = ret;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	/*Set Proximity Low Threshold*/
	ret = psensor_sysfs_read_low();	
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	return 0;
}

static void proximity_polling_adc(struct work_struct *work)
{
	int adc = 0;

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}
	
	adc= IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	log("[Polling] Proximity get adc = %d\n", adc);
	
	if(g_ps_data->Device_switch_on == true)
		queue_delayed_work(IRsensor_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(1000));
	else
		log("[Polling] Proximity polling adc STOP. \n");
}

static int light_turn_onoff(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */	
			
		light_get_shift();
		log("[Cal] Light Sensor Set Shift calibration value : %d\n", g_als_data->g_als_calvalue_shift);

		if(g_als_data->Device_switch_on == false) {
			ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(0);
			if(ret < 0){
				err("light_hw_set_hi_threshold ERROR. \n");
				return -ENOENT;
			}
			ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(0);
			if(ret < 0){
				err("light_hw_set_lo_threshold ERROR. \n");
				return -ENOENT;
			}
			ret = IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
			if(ret < 0){
				err("light_hw_turn_onoff(true) ERROR. \n");
				return -ENOENT;
			}
		}
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ASUS_IR_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = true;		
	} else	{	/* power off */			
		/*set turn off register*/
		if(g_als_data->Device_switch_on == true){
			/*disable IRQ before switch off*/		
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
			
			ret = IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
			if(ret < 0){
				err("light_hw_turn_onoff(false) ERROR. \n");
			}else{
				/*change the Device Status*/
				g_als_data->Device_switch_on = false;	
			}

			/*enable IRQ when proximity sensor is ON*/
			if (g_ps_data->Device_switch_on == true) {
				dbg("[IRQ] Enable irq !! \n");
				enable_irq(ASUS_IR_SENSOR_IRQ);
			}
		}
	}
	
	return ret;
}

static int light_get_lux(int adc)
{
	int lux = 0;

	if(adc < 0) {
		err("Light Sensor get Lux ERROR. (adc < 0)\n");
		return 0;
	}	
	
	lux = (adc * 800/(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux)
			+ g_als_data->g_als_calvalue_shift);	
	
	if(lux > LIGHT_MAX_LUX)
		lux = LIGHT_MAX_LUX;
	if(adc < 10 || lux < 0)
		lux = 0;
	
	return lux;
}

static int 	light_get_shift(void)
{
	int ret = 0;
	
	/* Light Sensor Read Calibration*/
	ret = lsensor_sysfs_read_200lux();
	if(ret > 0 ) {
		g_als_data->g_als_calvalue_200lux = ret;
		log("Light Sensor read 200lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_200lux);
	}else{
		err("Light Sensor read DEFAULT 200lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_200lux);
	}
	
	ret = lsensor_sysfs_read_1000lux();
	if(ret > 0 ) {
		g_als_data->g_als_calvalue_1000lux = ret;
		log("Light Sensor read 1000lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_1000lux);
	}else{
		err("Light Sensor read DEFAULT 1000lux Calibration: Cal: %d\n", g_als_data->g_als_calvalue_1000lux);
	}
		
	g_als_data->g_als_calvalue_shift = (1000 - g_als_data->g_als_calvalue_1000lux*800/
				(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux));
	return g_als_data->g_als_calvalue_shift;
}

static int 	light_get_accuracy_gain(void)
{
	int ret = 0;
	int gainvalue = 0;

	/* Light Sensor Read Calibration*/
	ret = lsensor_sysfs_read_200lux();
	if(ret > 0 )
		g_als_data->g_als_calvalue_200lux = ret;
	ret = lsensor_sysfs_read_1000lux();
	if(ret > 0 )
		g_als_data->g_als_calvalue_1000lux = ret;
		
	gainvalue = (800*LIGHT_GAIN_ACCURACY_CALVALUE)/
				(g_als_data->g_als_calvalue_1000lux-g_als_data->g_als_calvalue_200lux);
	
	return gainvalue;
}

static void light_polling_lux(struct work_struct *work)
{
	int adc = 0;
	int lux = 0;

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");		
	}
	
	/* Light Sensor Report the first real event*/			
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[Polling] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	lsensor_report_lux(lux);	
}

/**********************/
/*IR sensor Info Type*/
/*********************/

static IRsensor_info_type mIRsensor_info_type = {{0}};
	
/**********************/
/*Calibration Function*/
/*********************/
int mproximity_show_calibration_hi(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_high();	
	dbg("Proximity show High Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_hi(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store High Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;	
	}
	log("Proximity store High Calibration: %d\n", calvalue);
	psensor_sysfs_write_high(calvalue);
	proximity_set_threshold();
	
	return 0;
}

int mproximity_show_calibration_lo(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_low();
	dbg("Proximity show Low Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_lo(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Low Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Low Calibration: %d\n", calvalue);
	psensor_sysfs_write_low(calvalue);
	proximity_set_threshold();	

	return 0;
}

int mproximity_show_calibration_inf(void)
{
	int calvalue;
	calvalue = psensor_sysfs_read_inf();
	dbg("Proximity show Inf Calibration: %d\n", calvalue);
	return calvalue;
}

int mproximity_store_calibration_inf(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Inf Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Inf Calibration: %d\n", calvalue);
	psensor_sysfs_write_inf(calvalue);
	
	return 0;
}

int mlight_show_calibration_200lux(void)
{
	int calvalue;
	calvalue = lsensor_sysfs_read_200lux();	
	dbg("Light Sensor show 200 lux Calibration: %d\n", calvalue);
	return calvalue;
}

int mlight_store_calibration_200lux(int calvalue)
{
	int adc = 0;
	int lux = 0;
	
	if(calvalue <= 0) {
		err("Light Sensor store 200 lux Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store 200 lux Calibration: %d\n", calvalue);
	lsensor_sysfs_write_200lux(calvalue);
	g_als_data->g_als_calvalue_200lux = calvalue;
	light_get_shift();

	/* Light Sensor Report input event*/			
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	lsensor_report_lux(lux);	

	return 0;
}

int mlight_show_calibration_1000lux(void)
{
	int calvalue;
	calvalue = lsensor_sysfs_read_1000lux();	
	dbg("Light Sensor show 1000 lux Calibration: %d\n", calvalue);
	return calvalue;
}	

int mlight_store_calibration_1000lux(int calvalue)
{
	int adc = 0;
	int lux = 0;
	
	if(calvalue <= 0) {
		err("Light Sensor store 1000 lux Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store 1000 lux Calibration: %d\n", calvalue);
	lsensor_sysfs_write_1000lux(calvalue);
	g_als_data->g_als_calvalue_1000lux = calvalue;
	light_get_shift();

	/* Light Sensor Report input event*/			
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
	lsensor_report_lux(lux);	
			
	return 0;
}

int mlight_show_shift(void)
{
	int shiftvalue;
	shiftvalue = light_get_shift();
	dbg("Light Sensor show Shift Calibration: %d\n", shiftvalue);
	return shiftvalue;
}

int mlight_show_gain(void)
{
	int gainvalue;
	gainvalue = light_get_accuracy_gain();
	dbg("Light Sensor show Gain Calibration: %d.%d\n",
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);

	return gainvalue;
}

int mlight_show_adc(void)
{
	int adc = 0;
	if(IRsensor_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
		
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	dbg("mlight_show_adc : %d \n", adc);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	return adc;
}

int mproximity_show_adc(void)
{
	int adc = 0;
	int ret;
	
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_ir_lock);
	
	if(g_ps_data->Device_switch_on == false){
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			return ret;
		}			
	}

	msleep(PROXIMITY_TURNON_DELAY_TIME);
	
	adc = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("mproximity_show_adc : %d \n", adc);
	
	if(g_ps_data->HAL_switch_on == false){
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
			return ret;
		}			
	}
	
	mutex_unlock(&g_ir_lock);
	
	return adc;
}

static IRsensor_ATTR_Calibration mIRsensor_ATTR_Calibration = {
	.proximity_show_calibration_hi = mproximity_show_calibration_hi,
	.proximity_store_calibration_hi = mproximity_store_calibration_hi,
	.proximity_show_calibration_lo = mproximity_show_calibration_lo,
	.proximity_store_calibration_lo = mproximity_store_calibration_lo,
	.proximity_show_calibration_inf = mproximity_show_calibration_inf ,
	.proximity_store_calibration_inf  = mproximity_store_calibration_inf ,
	.light_show_calibration_200lux = mlight_show_calibration_200lux,
	.light_store_calibration_200lux = mlight_store_calibration_200lux,
	.light_show_calibration_1000lux = mlight_show_calibration_1000lux,
	.light_store_calibration_1000lux = mlight_store_calibration_1000lux,
	.light_show_shift = mlight_show_shift,
	.light_show_gain = mlight_show_gain,
	.light_show_adc = mlight_show_adc,
	.proximity_show_adc = mproximity_show_adc,
};

/******************/
/*BMMI Function*/
/****************/
bool mproximity_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = IRsensor_hw_client->IRsensor_hw_check_ID();
	if(ret < 0){
		err("Proximity ATD test check ID ERROR\n");
		goto proximity_atd_test_fail;
	}	
	
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);		
	if(ret < 0){
		err("Proximity ATD test turn on ERROR\n");
		goto proximity_atd_test_fail;
	}	
	
	for(;round<5; round++){
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
		if(ret < 0){
			err("Proximity ATD test get adc ERROR\n");
			goto proximity_atd_test_fail;
		}
		msleep(100);
	}	

	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
	if(ret < 0){
		err("Proximity ATD test turn off ERROR\n");
		goto proximity_atd_test_fail;
	}

	return true;
proximity_atd_test_fail:
	return false;
}

bool mlight_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = IRsensor_hw_client->IRsensor_hw_check_ID();
	if(ret < 0){
		err("Light Sensor ATD test check ID ERROR\n");
		goto light_atd_test_fail;
	}
	
	ret = IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
	if(ret < 0){
		err("Light Sensor ATD test turn on ERROR\n");
		goto light_atd_test_fail;
	}	
	
	for(; round<5; round++){
		ret = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
		if(ret < 0){
			err("Light Sensor ATD test get adc ERROR\n");
			goto light_atd_test_fail;
		}
		msleep(100);
	}	
	
	ret = IRsensor_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
	if(ret < 0){
		err("Light Sensor ATD test turn off ERROR\n");
		goto light_atd_test_fail;
	}

	return true;
light_atd_test_fail:
	return false;

}

static IRsensor_ATTR_BMMI mIRsensor_ATTR_BMMI = {
	.proximity_show_atd_test = mproximity_show_atd_test,
	.light_show_atd_test = mlight_show_atd_test,
};

/*********************/
/*Hardware Function*/
/********************/
int mIRsensor_show_reg(uint8_t addr)
{
	int value;
	if(IRsensor_hw_client->IRsensor_hw_get_register == NULL) {
		err("IRsensor_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = IRsensor_hw_client->IRsensor_hw_get_register(addr);
	log("mIRsensor_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

int mIRsensor_store_reg(uint8_t addr, int value)
{	
	if(IRsensor_hw_client->IRsensor_hw_set_register == NULL) {
		err("IRsensor_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	IRsensor_hw_client->IRsensor_hw_set_register(addr, value);
	log("mIRsensor_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static IRsensor_ATTR_Hardware mIRsensor_ATTR_Hardware = {
	.IRsensor_show_reg = mIRsensor_show_reg,
	.IRsensor_store_reg = mIRsensor_store_reg,
};

/****************/
/*HAL Function*/
/***************/
bool mproximity_show_switch_onoff(void)
{	
	return g_ps_data->Device_switch_on;
}

int mproximity_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_ir_lock);
	dbg("Proximity switch = %d.\n", bOn);		
	if ((g_ps_data->Device_switch_on != bOn))	{						
		if (bOn == true)	{
			/* Turn on Proxomity */
			g_ps_data->HAL_switch_on = true;
			proximity_turn_onoff(true);
			/* send the init value */
			psensor_report_abs(IRSENSOR_REPORT_PS_AWAY);
			log("Proximity Report First Away abs.\n");
		} else	{
			/* Turn off Proxomity */
			g_ps_data->HAL_switch_on = false;				
			proximity_turn_onoff(false);
			ftxxxx_disable_touch(false);
		}			
	}else{
		log("Proximity is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_ir_lock);
	
	return 0;
}

bool mlight_show_switch_onoff(void)
{
	return g_als_data->Device_switch_on;
}

int mlight_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_ir_lock);
	dbg("Light Sensor switch = %d.\n", bOn);		
	if ((g_als_data->Device_switch_on != bOn)) {					
		if (bOn == true)	{
			/* Turn on Light Sensor */
			g_als_data->HAL_switch_on = true;
			light_turn_onoff(true);
			
			/*light sensor polling the first real event after delayed time. */
			queue_delayed_work(IRsensor_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
		} else	{
			/* Turn off Light Sensor */
			g_als_data->HAL_switch_on = false;				
			light_turn_onoff(false);
			/* Report lux=-1 when turn off */
			lsensor_report_lux(-1);
		}			
	}else{
		log("Light Sensor is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_ir_lock);

	return 0;
}

int mlight_show_lux(void)
{
	int adc = 0;
	int lux = 0;

	mutex_lock(&g_ir_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);
	
	adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
	lux = light_get_lux(adc);
	dbg("mlight_show_lux : %d \n", lux);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_ir_lock);
	
	return lux;	
}

static IRsensor_ATTR_HAL mIRsensor_ATTR_HAL = {
	.proximity_show_switch_onoff = mproximity_show_switch_onoff,
	.proximity_store_switch_onoff = mproximity_store_switch_onoff,
	.proximity_show_status = proximity_check_status,
	.light_show_switch_onoff = mlight_show_switch_onoff,
	.light_store_switch_onoff = mlight_store_switch_onoff,
	.light_show_lux = mlight_show_lux,		
};

/*********************/
/*Extension Function*/
/********************/
bool mIRsensor_show_allreg(void)
{
	if(IRsensor_hw_client->IRsensor_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	IRsensor_hw_client->IRsensor_hw_show_allreg();
	return true;
}

bool mproximity_show_polling_mode(void)
{
	return g_ps_data->polling_mode;
}

int mproximity_store_polling_mode(bool bOn)
{
	g_ps_data->polling_mode = bOn;	
	return 0;
}

bool mproximity_show_autok(void)
{
	return g_ps_data->autok;
}

int mproximity_store_autok(bool bOn)
{
	g_ps_data->autok = bOn;	
	return 0;
}

int mproximity_show_int_count(void)
{
	return g_ps_data->int_counter;
}

int mproximity_show_event_count(void)
{
	return g_ps_data->event_counter;
}

int mproximity_show_autokmin(void)
{
	return g_ps_data->g_ps_autok_min;
}

int mproximity_store_autokmin(int autokmin)
{
	g_ps_data->g_ps_autok_min = autokmin;
	log("Proximity store autokmin: %d\n", autokmin);	
	
	return 0;
}

int mproximity_show_autokmax(void)
{
	return g_ps_data->g_ps_autok_max;
}

int mproximity_store_autokmax(int autokmax)
{
	g_ps_data->g_ps_autok_max = autokmax;
	log("Proximity store autokmax: %d\n", autokmax);	
	
	return 0;
}

int mlight_show_sensitivity(void)
{
	return g_als_data->g_als_change_sensitivity;
}

int mlight_store_sensitivity(int sensitivity)
{
	g_als_data->g_als_change_sensitivity = sensitivity;
	log("Light Sensor store Sensitivity: %d\n", sensitivity);	
	
	return 0;
}

int mlight_show_log_threshold(void)
{
	return g_als_data->g_als_log_threshold;
}

int mlight_store_log_threshold(int log_threshold)
{
	g_als_data->g_als_log_threshold = log_threshold;
	log("Light Sensor store Log Threshold: %d\n", log_threshold);	
	
	return 0;
}

int mlight_show_int_count(void)
{
	return g_als_data->int_counter;
}

int mlight_show_event_count(void)
{
	return g_als_data->event_counter;
}

int mIRsensor_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static IRsensor_ATTR_Extension mATTR_Extension = {
	.IRsensor_show_allreg = mIRsensor_show_allreg,
	.proximity_show_polling_mode = mproximity_show_polling_mode,
	.proximity_store_polling_mode = mproximity_store_polling_mode,
	.proximity_show_autok = mproximity_show_autok,
	.proximity_store_autok = mproximity_store_autok,
	.proximity_show_int_count = mproximity_show_int_count,
	.proximity_show_event_count = mproximity_show_event_count,
	.proximity_show_autokmin = mproximity_show_autokmin,
	.proximity_store_autokmin = mproximity_store_autokmin,
	.proximity_show_autokmax = mproximity_show_autokmax,
	.proximity_store_autokmax = mproximity_store_autokmax,
	.light_show_sensitivity = mlight_show_sensitivity,
	.light_store_sensitivity = mlight_store_sensitivity,
	.light_show_log_threshold = mlight_show_log_threshold,
	.light_store_log_threshold = mlight_store_log_threshold,
	.light_show_int_count = mlight_show_int_count,
	.light_show_event_count = mlight_show_event_count,
	.IRsensor_show_error_mesg = mIRsensor_show_error_mesg,
};

static IRsensor_ATTR mIRsensor_ATTR = {
	.info_type = &mIRsensor_info_type,
	.ATTR_Calibration = &mIRsensor_ATTR_Calibration,
	.ATTR_BMMI = &mIRsensor_ATTR_BMMI,
	.ATTR_Hardware = &mIRsensor_ATTR_Hardware,
	.ATTR_HAL = &mIRsensor_ATTR_HAL,
	.ATTR_Extension = &mATTR_Extension,
};

/*================================
 *|| Interrupt Service Routine Part ||
 *================================
 */
static void proximity_work(int state)
{
	int adc = 0;

	/* Get Proximity adc value */
	adc= IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc < 0){
		err("[ISR] Proximity get adc ERROR\n");	
		return;
	}

	/* Ignore the interrupt when Switch off */
	if(g_ps_data->HAL_switch_on == true)
	{
		/* Check proximity close or away. */
		if(IRSENSOR_INT_PS_AWAY == state) {
			log("[ISR] Proximity Detect Object Away. (adc = %d)\n", adc);
			psensor_report_abs(IRSENSOR_REPORT_PS_AWAY);
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
			if (2 == get_audiomode()) {
				ftxxxx_disable_touch(false);
			}
		} else if (IRSENSOR_INT_PS_CLOSE == state) {
			log("[ISR] Proximity Detect Object Close. (adc = %d)\n", adc);		
			psensor_report_abs(IRSENSOR_REPORT_PS_CLOSE);
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
			if (2 == get_audiomode()) {
				ftxxxx_disable_touch(true);
			}
		} else {
			err("[ISR] Proximity Detect Object ERROR. (adc = %d)\n", adc);
		}
	}
	
}

static void light_work(void)
{	
	int low_threshold = 0;
	int high_threshold = 0;	
	int adc = 0;
	int lux = 0;
	int ret = 0;
	int light_change_sensitivity = 0;	
	int light_log_threshold = 0;

	/* Ignore the interrupt when Switch off */
	if(g_als_data->HAL_switch_on == true)
	{
		adc = IRsensor_hw_client->mlsensor_hw->light_hw_get_adc();
		dbg("[ISR] Light Sensor Get adc : %d\n", adc);
		if(adc < 0){
			err("light_hw_get_adc ERROR\n");
			return;
		}

		/* Set the default sensitivity (3rd priority)*/
		if(adc >= g_als_data->g_als_calvalue_1000lux) {
			light_change_sensitivity = LIGHT_CHANGE_LOW_SENSITIVITY;
		} else if (adc <= g_als_data->g_als_calvalue_200lux) {
			light_change_sensitivity = LIGHT_CHANGE_HI_SENSITIVITY;
		} else {
			light_change_sensitivity = LIGHT_CHANGE_MID_SENSITIVITY;
		}

		/* Set the factory sensitivity (2nd priority) */
#ifdef ASUS_FACTORY_BUILD
		light_change_sensitivity = LIGHT_CHANGE_FACTORY_SENSITIVITY;
#endif

		/* Set the interface sensitivity (1st priority) */
		if(g_als_data->g_als_change_sensitivity >= 0)
			light_change_sensitivity = g_als_data->g_als_change_sensitivity;
		
		dbg("[ISR] Light Sensor Set Sensitivity. (light_change_sensitivity:%d)\n", light_change_sensitivity);	
		
		/* Light Sensor Low Threshold */	
		low_threshold = adc * (100 - light_change_sensitivity) / 100;

		/* Light Sensor High Threshold */
		high_threshold = (adc * (100 + light_change_sensitivity) / 100) + 1;	
		if (high_threshold > IRsensor_hw_client->mlsensor_hw->light_max_threshold)	
			high_threshold = IRsensor_hw_client->mlsensor_hw->light_max_threshold;	
		
		ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set High Threshold ERROR. (High:%d)\n", high_threshold);
		}
		dbg("[ISR] Light Sensor Set High Threshold. (High:%d)\n", high_threshold);	

		ret = IRsensor_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set Low Threshold ERROR. (Low:%d)\n", low_threshold);
		}
		dbg("[ISR] Light Sensor Set Low Threshold. (Low:%d)\n", low_threshold);	
		
		/* Light Sensor Report input event*/
		lux = light_get_lux(adc);

		light_log_threshold = LIGHT_LOG_THRESHOLD;
		
		/* Set the interface log threshold (1st priority) */
		if(g_als_data->g_als_log_threshold >= 0)
			light_log_threshold = g_als_data->g_als_log_threshold;
		
		if(abs(g_als_last_lux - lux) > light_log_threshold)
			log("[ISR] Light Sensor Report lux : %d (adc = %d)\n", lux, adc);
			
		lsensor_report_lux(lux);
		g_als_data->event_counter++;	/* --- For stress test debug --- */
		g_als_last_lux = lux;
	}
}

static void IRsensor_ist(struct work_struct *work)
{
	int irsensor_int_ps, irsensor_int_als;
	
mutex_lock(&g_ir_lock);
	if(g_als_data->HAL_switch_on == false && g_ps_data->HAL_switch_on == false) {
		log("ALSPS are disabled and ignore IST.\n");
		goto ist_err;
	}
	dbg("IRsensor ist +++ \n");
	if(IRsensor_hw_client == NULL)	{
		dbg("IRsensor_hw_client is NULL \n");
		goto ist_err;
	}

	/* Check Proximity Interrupt */
	irsensor_int_ps = ASUS_IR_SENSOR_INT&IRSENSOR_INT_PS_MASK;
	if(irsensor_int_ps == IRSENSOR_INT_PS_CLOSE || irsensor_int_ps == IRSENSOR_INT_PS_AWAY) 
	{
		dbg("Proximity ist \n");
		if(g_ps_data->HAL_switch_on == true)
			g_ps_data->int_counter++;	/* --- For stress test debug --- */
		
		if (irsensor_int_ps == IRSENSOR_INT_PS_AWAY) {
			proximity_work(IRSENSOR_INT_PS_AWAY);
		}
		if (irsensor_int_ps == IRSENSOR_INT_PS_CLOSE) {
			proximity_work(IRSENSOR_INT_PS_CLOSE);
		}
	}

	/* Check Light Sensor Interrupt */
	irsensor_int_als = ASUS_IR_SENSOR_INT&IRSENSOR_INT_ALS_MASK;
	if (irsensor_int_als == IRSENSOR_INT_ALS) {
		dbg("Light Sensor ist \n");
		if(g_als_data->HAL_switch_on == true)
			g_als_data->int_counter++;	/* --- For stress test debug --- */
		
		light_work();
	}
	dbg("IRsensor ist --- \n");
ist_err:	
	wake_unlock(&g_ir_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_IR_SENSOR_IRQ);	
mutex_unlock(&g_ir_lock);

}

void IRsensor_irq_handler(void)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ASUS_IR_SENSOR_IRQ);
	
	if(IRsensor_hw_client->IRsensor_hw_get_interrupt == NULL) {
		err("IRsensor_hw_get_interrupt NOT SUPPORT. \n");
		goto irq_err;
	}

	/* Read INT_FLAG will clean the interrupt */
	ASUS_IR_SENSOR_INT = IRsensor_hw_client->IRsensor_hw_get_interrupt();
	if(ASUS_IR_SENSOR_INT <0){
		err("IRsensor_hw_get_interrupt ERROR\n");
		goto irq_err;
	}

	/*Queue work will enbale IRQ and unlock wake_lock*/
	queue_work(IRsensor_workqueue, &IRsensor_ist_work);
	wake_lock(&g_ir_wake_lock);
	return;
irq_err:
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ASUS_IR_SENSOR_IRQ);
}

static IRsensor_GPIO mIRsensor_GPIO = {
	.IRsensor_isr = IRsensor_irq_handler,
};


/*============================
 *|| For Proximity check status ||
 *============================
 */
bool proximity_check_status(void)
{	
	int adc_value = 0;
	bool status = false;
	int ret=0;
	
	/* check probe status */
	if(IRsensor_hw_client == NULL)
		return status;

	mutex_lock(&g_ir_lock);

	/*Set Proximity Threshold(reset to factory)*/
	ret = proximity_set_threshold();
	if (ret < 0) {	
		err("proximity_set_threshold ERROR\n");
		return status;
	}

	
	if(g_ps_data->Device_switch_on == false){
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			return status;
		}			
	}
	
	msleep(PROXIMITY_TURNON_DELAY_TIME);

	adc_value = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();

	if (adc_value >= g_ps_data->g_ps_calvalue_hi) {
		status = true;
	}else{ 
		status = false;
	}
	log("proximity_check_status : %s , (adc, hi_cal)=(%d, %d)\n", 
		status?"Close":"Away", adc_value, g_ps_data->g_ps_calvalue_hi);
	
	if(g_ps_data->HAL_switch_on == false){
		ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
			return status;
		}			
	}
	
	mutex_unlock(&g_ir_lock);

	return status;
}

EXPORT_SYMBOL(proximity_check_status);

/*===========================
 *|| Proximity Auto Calibration Part ||
 *============================
 */
 static int proximity_set_nowork_threshold(void)
{
	int ret = 0;	

	/* Check Hardware Support First */
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	/*Set Proximity High Threshold*/
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(9999);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	/*Set Proximity Low Threshold*/
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(0);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	return 0;
}

static int proximity_check_minCT(void)
{
	int adc_value = 0;
	int crosstalk_diff;
	int crosstalk_min = 9999;
	int ret;
	int round;
	
	/*check the crosstalk calibration value*/	
	ret = psensor_sysfs_read_inf();	
	if(ret > 0) {
	    	g_ps_data->g_ps_calvalue_inf= ret;
		log("Proximity read INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}else{
		err("Proximity read DEFAULT INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}	

	/*make sure de-asserted INT when cat adc*/
	ret = proximity_set_nowork_threshold();
	if (ret < 0) {	
		err("proximity_set_nowork_threshold ERROR\n");
		return ret;
	}

	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);	
	if(ret < 0){
		err("proximity_hw_turn_onoff(true) ERROR\n");
		return ret;
	}
	/*update the min crosstalk value*/
	for(round=0; round<PROXIMITY_AUTOK_COUNT; round++){	
		mdelay(PROXIMITY_AUTOK_DELAY);
		adc_value = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
		log("proximity auto calibration adc : %d\n", adc_value);
		if(adc_value < crosstalk_min ){
			crosstalk_min = adc_value;
			log("Update the min for crosstalk : %d\n", crosstalk_min);
		}
	}
	ret = IRsensor_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
	if(ret < 0){
		err("proximity_hw_turn_onoff(true) ERROR\n");
		return ret;
	}

	/*Set Proximity Threshold*/
	ret = proximity_set_threshold();
	if (ret < 0) {	
		err("proximity_set_threshold ERROR\n");
		return ret;
	}

	/*update the diff crosstalk value*/
	crosstalk_diff = crosstalk_min -g_ps_data->g_ps_calvalue_inf;
	if(crosstalk_diff>g_ps_data->g_ps_autok_min && crosstalk_diff<g_ps_data->g_ps_autok_max){
		log("Update the diff for crosstalk : %d\n", crosstalk_diff);
		g_ps_data->crosstalk_diff = crosstalk_diff;

		if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
			err("proximity_hw_set_autoK NOT SUPPORT. \n");
			return -1;
		}
		IRsensor_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff);
		g_ps_data->g_ps_calvalue_hi += crosstalk_diff;
		g_ps_data->g_ps_calvalue_lo += crosstalk_diff;
	}else if(crosstalk_diff>=g_ps_data->g_ps_autok_max){
		log("crosstalk diff(%d) >= proximity autok max(%d)\n", crosstalk_diff, g_ps_data->g_ps_autok_max);
		g_ps_data->crosstalk_diff = crosstalk_diff;
	}else{
		log("crosstalk diff(%d) <= proximity autok min(%d)\n", crosstalk_diff, g_ps_data->g_ps_autok_min);
		g_ps_data->crosstalk_diff = 0;
	}
	
	return 0;
}

static void proximity_autok(struct work_struct *work)
{
	int adc_value;
	int crosstalk_diff;

	if(IRsensor_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
		err("proximity_hw_set_autoK NOT SUPPORT. \n");
		return;
	}
	
	adc_value = IRsensor_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("auto calibration polling : %d\n", adc_value);

	crosstalk_diff = adc_value -g_ps_data->g_ps_calvalue_inf;

	if((crosstalk_diff<g_ps_data->crosstalk_diff) &&( g_ps_data->crosstalk_diff!=0)){
		/*last diff of crosstalk does not set to HW, should reset the value to 0.*/
		if(g_ps_data->crosstalk_diff >= g_ps_data->g_ps_autok_max ){
			g_ps_data->crosstalk_diff=0;
		}
		if(crosstalk_diff<=g_ps_data->g_ps_autok_min ){			
			IRsensor_hw_client->mpsensor_hw->proximity_hw_set_autoK(0-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = 0;
			log("Update the diff for crosstalk : %d\n", g_ps_data->crosstalk_diff);
		}else if((crosstalk_diff>g_ps_data->g_ps_autok_min) && (crosstalk_diff<g_ps_data->g_ps_autok_max) ){			
			IRsensor_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = crosstalk_diff;
			log("Update the diff for crosstalk : %d\n", crosstalk_diff);
		}else{
			log("over the autok_max : (adc, inf) = %d(%d, %d) > %d\n", 
				crosstalk_diff, adc_value, g_ps_data->g_ps_calvalue_inf, g_ps_data->g_ps_autok_max);
			g_ps_data->crosstalk_diff = crosstalk_diff;
		}
	}	
}

static enum hrtimer_restart proximity_timer_function(struct hrtimer *timer)
{
	ktime_t autok_delay;
	
	dbg("proximity_timer_function\n");
	queue_work(IRsensor_workqueue, &proximity_autok_work);	

	if(0 == g_ps_data->crosstalk_diff){
		return HRTIMER_NORESTART;
	}else{
		/*needs to be reset in the callback function*/
		autok_delay = ns_to_ktime( PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
		hrtimer_forward_now(&g_ir_timer, autok_delay);
	}
	return HRTIMER_RESTART;
}

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_light_sensor_data */
	g_als_data = kmalloc(sizeof(struct ASUS_light_sensor_data), GFP_KERNEL);
	if (!g_als_data)	{
		err("g_als_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_als_data, 0, sizeof(struct ASUS_light_sensor_data));
	g_als_data->Device_switch_on = false;
	g_als_data->HAL_switch_on = 	false;	
	
	g_als_data->g_als_calvalue_200lux = 	IRsensor_hw_client->mlsensor_hw->light_200lux_default;
	g_als_data->g_als_calvalue_1000lux = 	IRsensor_hw_client->mlsensor_hw->light_1000lux_default;	
	g_als_data->g_als_calvalue_shift = 		IRSENSOR_DEFAULT_VALUE;
	g_als_data->g_als_change_sensitivity = IRSENSOR_DEFAULT_VALUE;
	g_als_data->g_als_log_threshold = 		IRSENSOR_DEFAULT_VALUE;

	g_als_data->int_counter = 0;
	g_als_data->event_counter = 0;
	
	/* Reset ASUS_proximity_sensor_data */
	g_ps_data = kmalloc(sizeof(struct ASUS_proximity_sensor_data), GFP_KERNEL);
	if (!g_ps_data) {
		err("g_ps_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_ps_data, 0, sizeof(struct ASUS_proximity_sensor_data));
	g_ps_data->Device_switch_on = 	false;
	g_ps_data->HAL_switch_on = 	false;	
	g_ps_data->polling_mode = 		false;
	g_ps_data->autok = 			true;
	
	g_ps_data->g_ps_calvalue_hi = IRsensor_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_calvalue_lo = IRsensor_hw_client->mpsensor_hw->proximity_low_threshold_default;	
	g_ps_data->g_ps_calvalue_inf = IRsensor_hw_client->mpsensor_hw->proximity_crosstalk_default;	
	g_ps_data->g_ps_autok_min= IRsensor_hw_client->mpsensor_hw->proximity_autok_min;	
	g_ps_data->g_ps_autok_max = IRsensor_hw_client->mpsensor_hw->proximity_autok_max;	

	g_ps_data->int_counter = 0;
	g_ps_data->event_counter = 0;
	g_ps_data->crosstalk_diff = 0;

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
void mIRsensor_algo_probe(struct i2c_client *client)
{	
	int ret = 0;	
	log("Driver PROBE +++\n");

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		goto probe_err;
	}	

	/*link driver data to i2c client*/
	strlcpy(client->name, SENSOR_TYPE_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_als_data);
	i2c_set_clientdata(client, g_ps_data);	

	/*set i2c Client for of_get_named_gpio*/
	ret = IRsensor_gpio_setI2cClient(client);
	if(ret < 0) {
		err("IRsensor_gpio_setI2cClient is ERROR\n");
		goto probe_err;
	}

	/* I2c stress test */
#ifdef CONFIG_I2C_STRESS_TEST	
	i2c_add_test_case(client, "IRSensorTest", ARRAY_AND_SIZE(IRSensorTestCaseInfo));	
#endif
	
	log("Driver PROBE ---\n");
	return ;
probe_err:
	err("Driver PROBE ERROR ---\n");
	return;

}

void IRsensor_algo_remove(void)
{
	log("Driver REMOVE +++\n");

	IRsensor_gpio_unregister(ASUS_IR_SENSOR_IRQ);

	log("Driver REMOVE ---\n");
	
	return;
}

void mIRsensor_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");

	/* Disable sensor */
	if (g_als_data->Device_switch_on)
		light_turn_onoff(false);
	if (g_ps_data->Device_switch_on)
		proximity_turn_onoff(false);	
	
	log("Driver SHUTDOWN ---\n");
	
	return;
}

void mIRsensor_algo_suspend(void)
{
	log("Driver SUSPEND +++\n");

	/* For keep Proximity can wake_up system */
	if (g_ps_data->Device_switch_on)
		enable_irq_wake(ASUS_IR_SENSOR_IRQ);

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_als_data->Device_switch_on)				
		light_turn_onoff(false);
	
	log("Driver SUSPEND ---\n");
	
	return;
}

void mIRsensor_algo_resume(void)
{
	log("Driver RESUME +++\n");

	if (g_ps_data->Device_switch_on)
		disable_irq_wake(ASUS_IR_SENSOR_IRQ);

	if (false == g_als_data->Device_switch_on && true == g_als_data->HAL_switch_on)
		light_turn_onoff(true);
	
	log("Driver RESUME ---\n");
	
	return;
}

static IRsensor_I2C mIRsensor_I2C = {
	.IRsensor_probe = mIRsensor_algo_probe,
	.IRsensor_remove = IRsensor_algo_remove,
	.IRsensor_shutdown = mIRsensor_algo_shutdown,
	.IRsensor_suspend = mIRsensor_algo_suspend,
	.IRsensor_resume = mIRsensor_algo_resume,
};

static int __init IRsensor_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");
	
	/* Work Queue */
	IRsensor_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");	
	IRsensor_delay_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_delay_wq");	

	/* Initialize the Mutex */
	mutex_init(&g_ir_lock);	

	/* Initialize the wake lock */
	wake_lock_init(&g_ir_wake_lock, WAKE_LOCK_SUSPEND, "IRsensor_wake_lock");

	/*Initialize high resolution timer*/
	hrtimer_init(&g_ir_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_ir_timer.function = proximity_timer_function;
	
	/* i2c Registration for probe/suspend/resume */				
	ret = IRsensor_i2c_register(&mIRsensor_I2C);
	if (ret < 0)
		goto init_err;
	
	/* Hardware Register Initialization */
	IRsensor_hw_client = IRsensor_hw_getHardware();
	if(IRsensor_hw_client == NULL)
		goto init_err;

	/* GPIO */
	ASUS_IR_SENSOR_IRQ = IRsensor_gpio_register(&mIRsensor_GPIO);
	if (ASUS_IR_SENSOR_IRQ < 0)
		goto init_err;	

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto init_err;

	/* string copy the character of vendor and module number */
	strcpy(mIRsensor_ATTR.info_type->vendor, IRsensor_hw_client->vendor);
	strcpy(mIRsensor_ATTR.info_type->module_number, IRsensor_hw_client->module_number);
	
	/* Attribute */
	IRsensor_ATTR_register(&mIRsensor_ATTR);	
	if (ret < 0)
		goto init_err;
	
	/* Input Device */
	ret = IRsensor_report_register();
	if (ret < 0)
		goto init_err;
		
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit IRsensor_exit(void)
{
	log("Driver EXIT +++\n");

	/* i2c Unregistration */	
	IRsensor_i2c_unregister();

	IRsensor_report_unregister();
	IRsensor_ATTR_unregister();	
	
	wake_lock_destroy(&g_ir_wake_lock);
	mutex_destroy(&g_ir_lock);
	kfree(g_ps_data);
	kfree(g_als_data);

	destroy_workqueue(IRsensor_workqueue);
	destroy_workqueue(IRsensor_delay_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(IRsensor_init);
module_exit(IRsensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor");
MODULE_LICENSE("GPL");

