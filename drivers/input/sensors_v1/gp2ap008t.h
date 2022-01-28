#ifndef __GP2AP008_H__
#define __GP2AP008_H__

// Reg.
#define REG_ADR_00	0x00	// Read & Write
#define REG_ADR_01	0x01	// Read & Write
#define REG_ADR_02	0x02	// Read & Write
#define REG_ADR_03	0x03	// Read & Write
#define REG_ADR_04	0x04	// Read & Write
#define REG_ADR_05	0x05	// Read & Write
#define REG_ADR_06	0x06	// Read & Write
#define REG_ADR_07	0x07	// Read & Write
#define REG_ADR_08	0x08	// Read & Write
#define REG_ADR_09	0x09	// Read & Write
#define REG_ADR_0A	0x0A	// Read & Write
#define REG_ADR_0B	0x0B	// Read & Write
#define REG_ADR_0C	0x0C	// Read & Write
#define REG_ADR_0D	0x0D	// Read & Write
#define REG_ADR_0E	0x0E	// Read & Write
#define REG_ADR_11	0x11	// Read  Only
#define REG_ADR_12	0x12	// Read  Only
#define REG_ADR_13	0x13	// Read  Only
#define REG_ADR_14	0x14	// Read  Only
#define REG_ADR_15	0x15	// Read  Only
#define REG_ADR_16	0x16	// Read  Only
#define REG_ADR_20	0x20	// Read  Only

// Reg. 00H
#define	OP_SHUTDOWN		0x00	// OP3:0
#define	OP_RUN			0xC0	// OP3:1
#define	OP_PS_ALS		0x00	// OP01:00
#define	OP_ALS			0x10	// OP01:01
#define	OP_PS			0x20	// OP01:10

// Reg. 01H
#define	INT_NOCLEAR		0x0F	// PROX:1 FLAG_P:1 FLAG_A:1 FLAF_FUSE:1

// Reg. 02H
#define	PIN_PROX		0x00	// PIN:00
#define	PIN_INT_ALS		0x20	// PIN:10
#define	PIN_INT_PS		0x10	// PIN:01
#define	INTTYPE_L		0x00	// INTTYPE:0
#define	INTTYPE_P		0x02	// INTTYPE:1
#define	RST				0x01	// RST:1

// Reg. 03H
#define	INTVAL_0		0x00	// INTVAL:000
#define	INTVAL_1p56		0x01	// INTVAL:001
#define	INTVAL_6p25		0x02	// INTVAL:010
#define	INTVAL_25		0x03	// INTVAL:011
#define	INTVAL_50		0x04	// INTVAL:100
#define	INTVAL_100		0x05	// INTVAL:101
#define	INTVAL_200		0x06	// INTVAL:110
#define	INTVAL_400		0x07	// INTVAL:111

// Reg. 04H
#define	ALS_AREA_ALL	0x80	// ALS_AREA:1
#define	RES_A_18		0x00	// RES_A:00
#define	RES_A_16		0x10	// RES_A:01
#define	RANGE_A_1		0x00	// RANGE_A:0000
#define	RANGE_A_2		0x01	// RANGE_A:0001
#define	RANGE_A_4		0x02	// RANGE_A:0010
#define	RANGE_A_8		0x03	// RANGE_A:0011
#define	RANGE_A_16		0x04	// RANGE_A:0100
#define	RANGE_A_32		0x05	// RANGE_A:0101
#define	RANGE_A_64		0x06	// RANGE_A:0110
#define	RANGE_A_138		0x0C	// RANGE_A:1100
#define	RANGE_A_277		0x0D	// RANGE_A:1101
#define	RANGE_A_555		0x0E	// RANGE_A:1110

// Reg. 05H
#define	RES_P_14		0x00	// RES_P:00
#define	RES_P_12		0x10	// RES_P:01
#define   RES_P_10           0x20       //RES_P:10

// Reg. 06H
#define	IS_19			0x00	// IS:00
#define	IS_38			0x10	// IS:01
#define	IS_75			0x20	// IS:10
#define	IS_150			0x30	// IS:11
#define   SUM_X11                   0x01
#define   SUM_X12                   0x02
#define	SUM_X16			0x03	// SUM:011
#define	SUM_X20			0x04	// SUM:100
#define	SUM_X24			0x05	// SUM:101

// Reg. 07H
#define	PRST_EVERY		0x00	// PRST:000
#define	PRST_1			0x10	// PRST:001
#define	PRST_4			0x40	// PRST:100
#define	PRST_7			0x70	// PRST:111
#define DEVICE_ID            0x60

/* event code */
#define ABS_WAKE				( ABS_BRAKE )
#define ABS_CONTROL_REPORT		( ABS_THROTTLE )
#define ABS_LUX_REPORT			( ABS_MISC )
#define ABS_DISTANCE_REPORT		( ABS_DISTANCE )

/* platform data */
struct gp2ap008_platform_data
{
	int		gpio ;
} ;

#endif

