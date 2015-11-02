/*
 *	IQS253_init.h - IQS253 Registers and Bit Definitions
 *
 *	Copyright (C) 2013 Azoteq (Pty) Ltd
 *	Author: Alwino van der Merwe <alwino.vandermerwe@azoteq.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *	Azoteq (Pty) Ltd does not take responsibility for the use of
 *	this driver.
 *
 *	This header file contains the setup for the IQS253 for this demo code
 *	of the IQS253 SAR Sensor. This file needs to be edited for each
 *	specific implementation of this driver. Use the IQS253 GUI from
 *	www.azoteq.com to change these settings and tune for a specific
 *	implementation.
 */

#ifndef IQS253_INIT_H
#define IQS253_INIT_H

/*	ATI Target - (0xC4)	*/
/*	1600 target	*/
#define ATI_TARGET_VAL						0xC8

/*	Compensation as Placeholder - (0xC5 - 0xC7)	*/
#define COMPENSATION_CH0					0X00
#define COMPENSATION_CH1					0X00
#define COMPENSATION_CH2					0X00

/*	Change the Multipliers & Base value - (0xC8 - 0xCA) */
/*	Base = 100	*/
#define MULTIPLIERS_CH0						0x80
#define MULTIPLIERS_CH1						0x80
#define MULTIPLIERS_CH2						0x80

/*	Change the Thresholds for each channel - (0xCB - 0xD0) */
/*	Prox Threshold 4 Counts	*/
#define PROX_THRESHOLD_CH0					0x05
#define PROX_THRESHOLD_CH1					0x04
#define PROX_THRESHOLD_CH2					0x05
/*	Make Touch 32 Counts = 5/256*1600	*/
#define TOUCH_THRESHOLD_CH0					0x15
#define TOUCH_THRESHOLD_CH1					0x15
#define TOUCH_THRESHOLD_CH2					0x15

/*	Change the Prox Settings or setup of the IQS253 - (0xD1 - 0xD4) */
#define PROXSETTINGS0_VAL					0x27
#define PROXSETTINGS1_VAL					0x00
/*	Always Halt LTA	*/
#define PROXSETTINGS2_VAL					0x03
/*	Xfer 250k, debounce - 4, LTA Adapt 0.78%	*/
#define PROXSETTINGS3_VAL					0x26

/*	Set Active Channels - (0xD5) */
/*	All CH is active	*/
#define ACTIVE_CHS							0x03

/*	Low Power Settings - (0xD6)	*/
#define LOW_POWER_VAL						0x00

/* DYCAL Specific Settings - (0xD7)	*/
#define DYCAL_SETTINGS_VAL					0x00

/*	DYCAL Channels Enable - (0xD8)	*/
/*	DYCAL Disabled	*/
#define DYCAL_CHANNELS_ENABLE				0x00

/*	Event Mode Mask - (0xD9)	*/
/*	ATI Event, Prox touch Event	*/
#define EVENT_MASK_VAL						0x23

/*	Boolean Settings - (0xDA)	*/
#define BOOLEAN_SETTINGS_VAL				0x00

/*	Boolean NOT Mask - (0xDB)	*/
#define BOOLEAN_NOT_MASK_VAL				0x00

/*	Boolean Settings - (0xDD)	*/
/*	Start Reading at (Sys flags) Prox Status Byte for convenience	*/
#define DEFAULT_COMMS_POINTER_VAL			0x10

/*************************************************************/
/*						Register Setup						 */
/*************************************************************/

/**
 *	@brief Register default values, do not edit.
 */
const unsigned char iqs253_default_regs[] = {
	ATI_TARGET_VAL,   /* ATI_TARGET - 0xC4 */
	COMPENSATION_CH0,   /* CH0_COMP */
	COMPENSATION_CH1,   /* CH1_COMP */
	COMPENSATION_CH2,   /* CH2_COMP */
	MULTIPLIERS_CH0,   /* CH0_ATI_BASE */
	MULTIPLIERS_CH1,   /* CH1_ATI_BASE */
	MULTIPLIERS_CH2,   /* CH2_ATI_BASE */
	PROX_THRESHOLD_CH0,   /* PROX_TH_0 */
	PROX_THRESHOLD_CH1,   /* PROX_TH_1 */
	PROX_THRESHOLD_CH2,   /* PROX_TH_2 */
	TOUCH_THRESHOLD_CH0,   /* TOUCH_TH_0 */
	TOUCH_THRESHOLD_CH1,   /* TOUCH_TH_1 */
	TOUCH_THRESHOLD_CH2,   /* TOUCH_TH_2 */
	(PROXSETTINGS0_VAL|ATI_OFF),   /* PROX_SETTINGS0 and ATI Off	*/
	PROXSETTINGS1_VAL,   /* PROX_SETTINGS1 */
	/* PROX_SETTINGS2 Always halt and ACK Reset	*/
	(PROXSETTINGS2_VAL|ACK_RESET),
	PROXSETTINGS3_VAL,	/* PROX_SETTINGS3 */
	ACTIVE_CHS,	/* ACTIVE_CHAN */
	LOW_POWER_VAL,	/* LOW_POWER */
	DYCAL_SETTINGS_VAL,	/* DYCAL_SETTINGS */
	DYCAL_CHANNELS_ENABLE,	/* DYCAL_CHANS */
	EVENT_MASK_VAL,	/* EVENT_MASK */
	BOOLEAN_SETTINGS_VAL,	/* BOOLEAN_SETTINGS */
	BOOLEAN_NOT_MASK_VAL,	/* BOOLEAN_NOT */
	0x07,	/* 0xDC - Padding */
	DEFAULT_COMMS_POINTER_VAL,	/* DEFAULT_COMMS_POINTER - 0xDD*/
};

#endif	/* IQS253_INIT_H */
