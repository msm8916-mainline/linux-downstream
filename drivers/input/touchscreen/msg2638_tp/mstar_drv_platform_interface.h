////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2012 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_interface.h
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

#ifndef __MSTAR_DRV_PLATFORM_INTERFACE_H__
#define __MSTAR_DRV_PLATFORM_INTERFACE_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"
#include <linux/sensors.h>

struct virtualpsensor {
	struct input_dev *proximity_dev;
	int proximity_function;
	int vps_enabled;
	char *vendor;
	int value;
};
extern struct virtualpsensor *vps;
/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
extern int proximity_enable ;
extern int proximity_enable_suspend;
extern int late_disable_proximity;
extern void DrvMainFirmwarePSEnable(int en);
#endif
extern s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId);
extern s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient);
//extern void MsDrvInterfaceTouchDeviceResume(struct early_suspend *pSuspend);        
//extern void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend);
extern void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate);
        
#endif  /* __MSTAR_DRV_PLATFORM_INTERFACE_H__ */
