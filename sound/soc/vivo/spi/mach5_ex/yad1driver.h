/****************************************************************************
 *
 *		Copyright(c) 2013-2014 Yamaha Corporation. All rights reserved.
 *
 *		Module		: yad1driver.h
 *
 *		Description	: Yamaha Audio engine Device version 1
 *						 device driver public definition.
 *
 *		Version		: 1.1.0		2014.07.25
 *
 ****************************************************************************/

#ifndef YAD1DD_H
#define YAD1DD_H

#include "yad1types.h"

#define	YAD1DD_SUCCESS			(0)
#define	YAD1DD_ERROR_SPI		(-1)
#define	YAD1DD_ERROR_I2C		(-1)
#define	YAD1DD_ERROR_ARG		(-2)
#define	YAD1DD_ERROR_SYN		(-3)
#define	YAD1DD_ERROR_TO			(-4)

#define	YAD1DD_REGSEL_IF		(0)
#define	YAD1DD_REGSEL_IM_A		(1)
#define	YAD1DD_REGSEL_IM_MA		(2)
#define	YAD1DD_REGSEL_IM_MB		(3)
#define	YAD1DD_REGSEL_IM_B		(4)
#define	YAD1DD_REGSEL_IM_C		(5)
#define	YAD1DD_REGSEL_IM_E		(6)
#define	YAD1DD_REGSEL_IM_F		(7)
#define	YAD1DD_REGSEL_IM_ANA	(8)
#define	YAD1DD_REGSEL_IM_CD		(9)

#define	YAD1DD_MEM_SIZE			(16)

#ifdef __cplusplus
extern "C" {
#endif

S32		YAD1DD_Write			( U08 u08Page, U08 u08Adr, const U08 * pu08Data, U32 u32Size );
S32		YAD1DD_Read				( U08 u08Page, U08 u08Adr, U08 * pu08Data, U32 u32Size );
void	YAD1DD_Wait				( U32 u32MSec );
S32		YAD1DD_BatchProcess		( const U08 * pu08Script, U32 u32Size );

#ifdef __cplusplus
}
#endif

#endif /* YAD1DD_H */
