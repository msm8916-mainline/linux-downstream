/****************************************************************************
 *
 *		Copyright(c) 2013-2014 Yamaha Corporation. All rights reserved.
 *
 *		Module		: yad1machdep.h
 *
 *		Description	: Yamaha Audio engine Device version 1
 *						device driver machine dependence part public definition.
 *
 *		Version		: 1.1.0		2014.07.25
 *
 ****************************************************************************/

#ifndef YAD1MD_H
#define YAD1MD_H

#include "yad1types.h"
#include "yad1driver.h"

#define	YAD1_DEVICE_IF_SPI		(0)
#define	YAD1_DEVICE_IF_I2C		(1)

#define	YAD1_DEVICE_IF			YAD1_DEVICE_IF_SPI

#ifdef __cplusplus
extern "C" {
#endif

#if		(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_SPI )
S32		YAD1MD_Write			( U08 u08Com, const U08 * pu08Data, U32 u32Size );
S32		YAD1MD_Read				( U08 u08Com, U08 * pu08Data, U32 u32Size );
#elif	(YAD1_DEVICE_IF	== YAD1_DEVICE_IF_I2C )
S32		YAD1MD_WriteI2C			( U08 u08SA, U08 u08IFA, const U08 * pu08Data, U32 u32Size );
S32		YAD1MD_ReadI2C			( U08 u08SA, U08 u08IFA, U08 * pu08Data, U32 u32Size );
#endif
void	YAD1MD_Sleep			( U32 u32MSec );

#ifdef __cplusplus
}
#endif

#endif /* YAD1MD_H */
