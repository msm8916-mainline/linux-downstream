/******************************************************************************
* (c) COPYRIGHT 2012 RAONTECH, Inc. ALL RIGHTS RESERVED.
*
* This software is the property of RAONTECH and is furnished under license
* by RAONTECH.
* This software may be used only in accordance with the terms of said license.
* This copyright noitce may not be remoced, modified or obliterated without
* the prior written permission of RAONTECH, Inc.
*
* This software may not be copied, transmitted, provided to or otherwise
* made available to any other person, company, corporation or other entity
* except as specified in the terms of said license.
*
* No right, title, ownership or other interest in the software is hereby
* granted or transferred.
*
* The information contained herein is subject to change without notice
* and should not be construed as a commitment by RAONTECH, Inc.
*
* TITLE		: MTV319 porting source file.
*
* FILENAME	: mtv319_port.c
*
* DESCRIPTION	: User-supplied Routines for RAONTECH TV Services.
*
******************************************************************************/
/******************************************************************************
* REVISION HISTORY
*
*    DATE         NAME          REMARKS
* ----------  -------------    ------------------------------------------------
* 07/12/2012  Ko, Kevin        Created.
******************************************************************************/

#include "mtv319.h"
#include "mtv319_internal.h"


/* Declares a variable of gurad object if neccessry. */
#if defined(RTV_IF_SPI) || defined(RTV_IF_EBI2)\
|| defined(RTV_FIC_I2C_INTR_ENABLED)
	#if defined(__KERNEL__)
	struct mutex raontv_guard;
	#elif defined(WINCE) || defined(WINDOWS) || defined(WIN32)
	CRITICAL_SECTION raontv_guard;
    #else
	/* non-OS and RTOS. */
	#endif
#endif


void rtvOEM_PowerOn(int on)
{
#if 0
	if (on) {
		/* Set the GPIO of MTV_EN pin to high. */
		gpio_set_value(MTV_PWR_EN, 1);
	
		/* In case of SPI interface or FIC interrupt mode for T-DMB,
		we should lock the register page. */
		RTV_GUARD_INIT;
	} else {
		/* Set the GPIO of MTV_EN pin to low. */
		gpio_set_value(MTV_PWR_EN, 0);

		RTV_GUARD_DEINIT;
	}
#endif
}




