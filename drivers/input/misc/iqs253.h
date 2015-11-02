/*
 *	iqs253.h - IQS253 Registers and Bit Definitions
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
 *	This header file contains the register and bit definitions for the
 *	IQS253 SAR Sensor, to be used in the driver. This makes the code
 *	cleaner to read.
 */
#ifndef IQS253_H
#define IQS253_H

/*	i2c slave device address	*/
#define IQS253_ADDR	0x44

/*	Definitions for the driver	*/
#define IQS_MAJOR				248
#define IQS_MINOR				0
#define IQS_DEV_NUMS			1
#define IQS_NUM_RW_REGS         26
#define DEVICE_NAME             "iqs253"

/*	Addresses of Registers on IQS253	*/
/*	Read	*/
#define PROD_NR					0x00
#define SW_NR					0x01
#define SYSFLAGS				0x10
#define PROX_STAT				0x31
#define TOUCH_STAT				0x35
#define DYCAL_TM				0x36
#define DYCAL_OUT				0x37
#define HALT_STAT				0x39
#define CHAN_NUM				0x3D
#define CUR_SAM_HI				0x42
#define CUR_SAM_LO				0x43
#define LTA_HI					0x83
#define LTA_LO					0x84

/*	Read / Write	*/
#define ATI_TARGET				0xC4
#define CH0_COMP				0xC5
#define CH1_COMP				0xC6
#define CH2_COMP				0xC7
#define CH0_ATI_BASE			0xC8
#define CH1_ATI_BASE			0xC9
#define CH2_ATI_BASE			0xCA
#define PROX_TH_0				0xCB
#define PROX_TH_1				0xCC
#define PROX_TH_2				0xCD
#define TOUCH_TH_0				0xCE
#define TOUCH_TH_1				0xCF
#define TOUCH_TH_2				0xD0
#define PROX_SETTINGS0			0xD1
#define PROX_SETTINGS1			0xD2
#define PROX_SETTINGS2			0xD3
#define PROX_SETTINGS3			0xD4
#define ACTIVE_CHAN				0xD5
#define LOW_POWER				0xD6
#define DYCAL_SETTINGS			0xD7
#define DYCAL_CHANS				0xD8
#define EVENT_MASK				0xD9
#define BOOLEAN_SETTINGS		0xDA
#define BOOLEAN_NOT				0xDB
#define DEFAULT_COMMS_POINTER	0xDD

/*	BIT DEFINITIONS FOR IQS253	*/

/*	Bit definitions	*/
/*	System Flags - STATUS	*/
/*	Indicates ATI is busy	*/
#define	ATI_BUSY			0x04
/*	Set IQS253 in Projected mode	*/
#define	PROJ_MODE			0x10
/*	Indicates reset has occurred	*/
#define	SHOW_RESET			0x20

/*	Bit definitions - Touch Bytes	*/
/*	Indicates a Prox and Touch on CH0	*/
#define	CH_0_EVENT			0x01
#define	CH_1_EVENT			0x02
#define	CH_ALL_EVENT			0x03

/*	Bit definitions - Multipliers	*/
/*	Sensitivity Multipliers - P-mirror	*/
#define	SENS_MULT			0x0F
/*	Compensation Multipliers - N-mirror	*/
#define	COMP_MULT			0x30

#define HS_ENABLE    0x10
/*	Bit definitions - ProxSettings0	*/
/*	Reseed the IQS253	*/
#define	RESEED				0x08
/*	Redo ATI - switch on ATI	*/
#define	REDO_ATI			0x10
/*	0 - Partial ATI Off, 1 - Partial ATI On	*/
#define	ATI_PARTIAL			0x40
/*	0 - ATI Enabled, 1 - ATI Disabled	*/
#define	ATI_OFF				0x80

/*	Bit definitions - ProxSettings0	*/
/*	0-Streaming Mode, 1-Event Mode	*/
#define	EVENT_MODE			0x40
/*	Acknowledge that chip was reset	*/
#define ACK_RESET			0x80

/*	Delays	*/
#define HANDSHAKE_DELAY_HOLD	11		/*	11ms	*/
#define HANDSHAKE_DELAY_SET		200		/*	200μs	*/

#endif
