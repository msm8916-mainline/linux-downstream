/**
 * @file Upidef.h
 *
 *  The global header file for all the source files
 *
 *  The header file gathers the following header files
 *  It includes:
 *  1. standard header
 *  2. type definition
 *  3. vender definition
 *  4. register for the system
 *  5. macro definition
 *  6. global variables definition
 *  7. TP command definition
 *  8. memory definition
 *  8. global function definition
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2014, JSoftLab, all rights reserved.
 * @note
*/
#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

#ifndef __UPIDEF_H__
#define __UPIDEF_H__

#ifndef FEATURE_PLAT_LINUX

#include <stdio.h>
#include <string.h>
#include <time.h>

#endif  ///< end of FEATURE_PLAT_LINUX

/// definition for  function and vender features
#include "Vender.h"
/// definition for macros
#include "Define.h"
/// definition for types
#include "TypeDef.h"
/// definition for 
#include "RegDef.h"
/// definition for global variables
#include "VarDef.h"
/// definition for global function
#include "FuncDef.h"
/// definition for the SVN version
#include "Revision.h"

///--------------------------------------------------///
/// Paltform Related
///--------------------------------------------------///
#if defined(FEATURE_PLAT_WINDOWS)
///----------------------------------------///
/// Windows Include file
///---------------------------------------///
#include <windows.h>

#elif defined(FEATURE_PLAT_LINUX)
///----------------------------------------///
/// Linux Include file
///---------------------------------------///
#include <linux/init.h>
#include <linux/platform_device.h>

#if defined(PLATFORM_ASUS_Z370_COVER)
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#elif defined(PLATFORM_ASUS_Z380KL_COVER)
#include <linux/of_gpio.h>
#include <linux/of.h>
#endif	///< for PLATFORM_ASUS_Z370_COVER

#include <linux/kernel.h>
#include <linux/module.h>			///< for Driver Module
#include <linux/delay.h>
#include <linux/fs.h>					///< for file operation
#include <linux/i2c.h> 				///< for I2C
#include <linux/slab.h>			
#include <linux/jiffies.h> 		///< for UpiGetTickCount()
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#elif defined(FEATURE_PLAT_ARM_M0)
#include "evmdef.h"
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
#include "evmdef.h"
#else  ///< for FEATURE_PLAT_WINDOWS
#include <sys/time.h>
#include <stdarg.h>
#endif ///< for FEATURE_PLAT_WINDOWS

#endif ///< for __UPIDEF_H__ 

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus
