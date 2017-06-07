/*
 * MELFAS MIP4 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * mip4_lge.c : Vendor specific functions for LGE
 *
 *
 * Protocol Version : LGE 1.4
 *
 */

#include "mip4.h"

/**
* Register map
*/
//Info
#define MIP_R1_VENDOR_VERSION					0x00

//Control
#define MIP_R1_LPWG_START							0x10

//Public
#define MIP_R1_LPWG_IDLE_REPORTRATE				0x20
#define MIP_R1_LPWG_ACTIVE_REPORTRATE 			0x21
#define MIP_R1_LPWG_SENSITIVITY 					0x22
#define MIP_R1_LPWG_ACTIVE_AREA_HOR_START_H 	0x23
#define MIP_R1_LPWG_ACTIVE_AREA_HOR_START_L 	0x24
#define MIP_R1_LPWG_ACTIVE_AREA_VER_START_H 	0x25
#define MIP_R1_LPWG_ACTIVE_AREA_VER_START_L 	0x26
#define MIP_R1_LPWG_ACTIVE_AREA_HOR_END_H 		0x27
#define MIP_R1_LPWG_ACTIVE_AREA_HOR_END_L 		0x28
#define MIP_R1_LPWG_ACTIVE_AREA_VER_END_H 		0x29
#define MIP_R1_LPWG_ACTIVE_AREA_VER_END_L	 	0x2A
#define MIP_R1_LPWG_DEBUG_ENABLE 				0x2B
#define MIP_R1_LPWG_FAIL_REASON 					0x2C
#define MIP_R1_LPWG_LCD_STATUS 					0x2D

//Knock On
#define MIP_R1_LPWG_ENABLE 						0x40
#define MIP_R1_LPWG_TOUCH_SLOP 					0x41
#define MIP_R1_LPWG_MIN_INTERTAP_DISTANCE 		0x42
#define MIP_R1_LPWG_MAX_INTERTAP_DISTANCE 		0x43
#define MIP_R1_LPWG_MIN_INTERTAP_TIME 			0x44
#define MIP_R1_LPWG_MAX_INTERTAP_TIME 			0x46
#define MIP_R1_LPWG_TOTAL_TAP_COUNT 				0x48
#define MIP_R1_LPWG_INT_DELAY_TIME 				0x49

//Knock Code
#define MIP_R1_LPWG_ENABLE2						0x50
#define MIP_R1_LPWG_TOUCH_SLOP2					0x51
#define MIP_R1_LPWG_MIN_INTERTAP_DISTANCE2		0x52
#define MIP_R1_LPWG_MAX_INTERTAP_DISTANCE2		0x53
#define MIP_R1_LPWG_MIN_INTERTAP_TIME2			0x54
#define MIP_R1_LPWG_MAX_INTERTAP_TIME2			0x56
#define MIP_R1_LPWG_TOTAL_TAP_COUNT2			0x58
#define MIP_R1_LPWG_INT_DELAY_TIME2				0x59

/**
* Register value
*/
#define MIP_LPWG_EVENT_TYPE_KNOCK_ON	0
#define MIP_LPWG_EVENT_TYPE_KNOCK_CODE	1
#define MIP_LPWG_EVENT_TYPE_FAIL			3

/**
* LPWG event handler
*/
int mip_lpwg_event_handler(struct mip_ts_info *info, u8 *rbuf, u8 size)
{
	u8 event_type = rbuf[1];
	int i;
	int x, y;
	u8 fail_multi_touch, fail_touch_slop, fail_touch_distance, fail_tap_time, fail_delay_time, fail_palm, fail_area, fail_limit;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	printk(KERN_ERR "%s [START]\n", __func__);

	/////////////////////////////////
	// PLEASE MODIFY HERE !!!
	//

	switch(event_type){
		case MIP_LPWG_EVENT_TYPE_KNOCK_ON:
		case MIP_LPWG_EVENT_TYPE_KNOCK_CODE:
			for(i = 2; i < size; i += 3){
				u8 *tmp = &rbuf[i];

				x = tmp[1] | ((tmp[0] & 0xf) << 8);
				y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);

				dev_dbg(&info->client->dev, "%s - event type[%d] x[%d] y[%d]\n", __func__, event_type, x, y);
				printk(KERN_ERR "%s - event type[%d] x[%d] y[%d]\n", __func__, event_type, x, y);
				//Report event
				//......
			}

			break;

		case MIP_LPWG_EVENT_TYPE_FAIL:
			fail_multi_touch = (rbuf[2] >> 7) & 0x01;
			fail_touch_slop = (rbuf[2] >> 6) & 0x01;
			fail_touch_distance = (rbuf[2] >> 5) & 0x01;
			fail_tap_time = (rbuf[2] >> 4) & 0x01;
			fail_delay_time = (rbuf[2] >> 3) & 0x01;
			fail_palm = (rbuf[2] >> 2) & 0x01;
			fail_area = (rbuf[2] >> 1) & 0x01;
			fail_limit = rbuf[2] & 0x01;

			dev_dbg(&info->client->dev, "%s - event type[%d] multi_touch[%d] touch_slop[%d] touch_distance[%d] tap_time[%d] delay_time[%d] palm[%d] area[%d] limit[%d]\n", __func__, event_type, fail_multi_touch, fail_touch_slop, fail_touch_distance, fail_tap_time, fail_delay_time, fail_palm, fail_area, fail_limit);
		  printk(KERN_ERR "%s - event type[%d] multi_touch[%d] touch_slop[%d] touch_distance[%d] tap_time[%d] delay_time[%d] palm[%d] area[%d] limit[%d]\n", __func__, event_type, fail_multi_touch, fail_touch_slop, fail_touch_distance, fail_tap_time, fail_delay_time, fail_palm, fail_area, fail_limit);
			//Report event
			//......

			break;

		default:
			dev_err(&info->client->dev, "%s [ERROR] Unknown event type[%d]\n", __func__, event_type);
			printk(KERN_ERR "%s [ERROR] Unknown event type[%d]\n", __func__, event_type);
			goto ERROR;
			break;
	}

	//
	//
	/////////////////////////////////

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	printk(KERN_ERR "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return 1;
}

/**
* LPWG Config Public
*/
int mip_lpwg_config(struct mip_ts_info *info, u8 *value)
{
	u8 wbuf[32];
	int i;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_VENDOR;
	wbuf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
	for(i = 0; i < 14; i++){
		wbuf[2 + i] = value[i];
	}
	if(mip_i2c_write(info, wbuf, 16)){
		dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
* LPWG Config Knock-on
*/
int mip_lpwg_config_knock_on(struct mip_ts_info *info, u8 *value)
{
	u8 wbuf[32];
	int i;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_VENDOR;
	wbuf[1] = MIP_R1_LPWG_ENABLE;
	for(i = 0; i < 11; i++){
		wbuf[2 + i] = value[i];
	}
	if(mip_i2c_write(info, wbuf, 13)){
		dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
* LPWG Config Knock-code
*/
int mip_lpwg_config_knock_code(struct mip_ts_info *info, u8 *value)
{
	u8 wbuf[32];
	int i;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_VENDOR;
	wbuf[1] = MIP_R1_LPWG_ENABLE2;
	for(i = 0; i < 11; i++){
		wbuf[2 + i] = value[i];
	}
	if(mip_i2c_write(info, wbuf, 13)){
		dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
* LPWG Start
*/
int mip_lpwg_start(struct mip_ts_info *info)
{
	u8 wbuf[4];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_VENDOR;
	wbuf[1] = MIP_R1_LPWG_START;
	wbuf[2] = 1;
	if(mip_i2c_write(info, wbuf, 3)){
		dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

