/****************************************************************************
 *
 *		Copyright(c) 2013-2014 Yamaha Corporation. All rights reserved.
 *
 *		Module		: yad1driver_d.h
 *
 *		Description	: Yamaha Audio engine Device version 1
 *						device driver local definition.
 *
 *		Version		: 1.1.0		2014.07.25
 *
 ****************************************************************************/

#ifndef YAD1DD_D_H
#define YAD1DD_D_H

#include "yad1machdep.h"

#if		(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_SPI )
	#define	YAD1DD_SPI_COM_WRITE		(0x00)
	#define	YAD1DD_SPI_COM_READ			(0x80)
	#define	YAD1DD_SPI_COM_ADR_MASK		(0x3F)
	#define	YAD1DD_SPI_COM_BURST		(0x01)
#elif	(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_I2C )
	#define	YAD1DD_I2C_COM_WRITE		(0x00)
	#define	YAD1DD_I2C_COM_READ			(0x01)
	#define	YAD1DD_I2C_COM_ADR_MASK		(0x3F)
	#define	YAD1DD_I2C_COM_BURST		(0x01)
	#define	YAD1DD_DIGITALBLOCK_SA		(0x11)
	#define	YAD1DD_ANALOGBLOCK_SA		(0x3A)
#endif

#define	YAD1DD_REGADR_A_REG_A		(0x00)
#define	YAD1DD_REGADR_A_REG_D		(0x01)
#define	YAD1DD_REGADR_MA_REG_A		(0x0C)
#define	YAD1DD_REGADR_MA_REG_D		(0x0D)
#define	YAD1DD_REGADR_MB_REG_A		(0x0E)
#define	YAD1DD_REGADR_MB_REG_D		(0x0F)
#define	YAD1DD_REGADR_B_REG_A		(0x10)
#define	YAD1DD_REGADR_B_REG_D		(0x11)
#define	YAD1DD_REGADR_C_REG_A		(0x28)
#define	YAD1DD_REGADR_C_REG_D		(0x29)
#define	YAD1DD_REGADR_E_REG_A		(0x20)
#define	YAD1DD_REGADR_E_REG_D		(0x21)
#define	YAD1DD_REGADR_F_REG_A		(0x30)
#define	YAD1DD_REGADR_F_REG_D		(0x31)
#define	YAD1DD_REGADR_ANA_REG_A		(0x06)
#define	YAD1DD_REGADR_ANA_REG_D		(0x07)
#define	YAD1DD_REGADR_CD_REG_A		(0x08)
#define	YAD1DD_REGADR_CD_REG_D		(0x09)

#define	YAD1DD_REGFLAG_AINC			(0x80)

#define	YAD1DD_COMTYPE_IF_REG_W		(0x01)
#define	YAD1DD_COMTYPE_A_REG_W		(0x02)
#define	YAD1DD_COMTYPE_MA_REG_W		(0x03)
#define	YAD1DD_COMTYPE_MB_REG_W		(0x04)
#define	YAD1DD_COMTYPE_B_REG_W		(0x05)
#define	YAD1DD_COMTYPE_C_REG_W		(0x06)
#define	YAD1DD_COMTYPE_E_REG_W		(0x07)
#define	YAD1DD_COMTYPE_F_REG_W		(0x08)
#define	YAD1DD_COMTYPE_ANA_REG_W	(0x09)
#define	YAD1DD_COMTYPE_CD_REG_W		(0x0A)

#define	YAD1DD_COMTYPE_DIRECT_W		(0x0F)

#define	YAD1DD_COMTYPE_IF_REG_W_V	(0x81)
#define	YAD1DD_COMTYPE_A_REG_W_V	(0x82)
#define	YAD1DD_COMTYPE_MA_REG_W_V	(0x83)
#define	YAD1DD_COMTYPE_MB_REG_W_V	(0x84)
#define	YAD1DD_COMTYPE_B_REG_W_V	(0x85)
#define	YAD1DD_COMTYPE_C_REG_W_V	(0x86)
#define	YAD1DD_COMTYPE_E_REG_W_V	(0x87)
#define	YAD1DD_COMTYPE_F_REG_W_V	(0x88)
#define	YAD1DD_COMTYPE_ANA_REG_W_V	(0x89)
#define	YAD1DD_COMTYPE_CD_REG_W_V	(0x8A)

#define	YAD1DD_COMTYPE_DIRECT_W_V	(0x8F)

#define	YAD1DD_COMTYPE_IF_REG_F		(0x11)
#define	YAD1DD_COMTYPE_A_REG_F		(0x12)
#define	YAD1DD_COMTYPE_MA_REG_F		(0x13)
#define	YAD1DD_COMTYPE_MB_REG_F		(0x14)
#define	YAD1DD_COMTYPE_B_REG_F		(0x15)
#define	YAD1DD_COMTYPE_C_REG_F		(0x16)
#define	YAD1DD_COMTYPE_E_REG_F		(0x17)
#define	YAD1DD_COMTYPE_F_REG_F		(0x18)
#define	YAD1DD_COMTYPE_ANA_REG_F	(0x19)
#define	YAD1DD_COMTYPE_CD_REG_F		(0x1A)

#define	YAD1DD_COMTYPE_WAIT			(0x1F)

#define	YAD1DD_COMTYPE_MEM_W_IF		(0x61)
#define	YAD1DD_COMTYPE_MEM_W_A		(0x62)
#define	YAD1DD_COMTYPE_MEM_W_MA		(0x63)
#define	YAD1DD_COMTYPE_MEM_W_MB		(0x64)
#define	YAD1DD_COMTYPE_MEM_W_B		(0x65)
#define	YAD1DD_COMTYPE_MEM_W_C		(0x66)
#define	YAD1DD_COMTYPE_MEM_W_E		(0x67)
#define	YAD1DD_COMTYPE_MEM_W_F		(0x68)
#define	YAD1DD_COMTYPE_MEM_W_ANA	(0x69)
#define	YAD1DD_COMTYPE_MEM_W_CD		(0x6A)

#define	YAD1DD_COMTYPE_MEM_CTRL		(0x6F)

#define	YAD1DD_COMTYPE_MEM_CP_IF	(0x71)
#define	YAD1DD_COMTYPE_MEM_CP_A		(0x72)
#define	YAD1DD_COMTYPE_MEM_CP_MA	(0x73)
#define	YAD1DD_COMTYPE_MEM_CP_MB	(0x74)
#define	YAD1DD_COMTYPE_MEM_CP_B		(0x75)
#define	YAD1DD_COMTYPE_MEM_CP_C		(0x76)
#define	YAD1DD_COMTYPE_MEM_CP_E		(0x77)
#define	YAD1DD_COMTYPE_MEM_CP_F		(0x78)
#define	YAD1DD_COMTYPE_MEM_CP_ANA	(0x79)
#define	YAD1DD_COMTYPE_MEM_CP_CD	(0x7A)

#define	YAD1DD_MEMCTRL_WRITE		(0x00)
#define	YAD1DD_MEMCTRL_ADD			(0x01)
#define	YAD1DD_MEMCTRL_SUB			(0x02)
#define	YAD1DD_MEMCTRL_MUL			(0x03)
#define	YAD1DD_MEMCTRL_OR			(0x04)
#define	YAD1DD_MEMCTRL_AND			(0x05)
#define	YAD1DD_MEMCTRL_XOR			(0x06)

#endif /* YAD1DD_D_H */
