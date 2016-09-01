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
 * @file    mstar_drv_platform_interface.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_interface.h"
#include "mstar_drv_main.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_porting_layer.h"
#include <linux/wakelock.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
struct notifier_block TPfb_notifier;
#endif
extern int msg2638_pinctrl_init(struct i2c_client *pClient);

/* modify by furong, 2014/12/10, PR-865304, add for sometimes TP can't work after suspend/resume. */
atomic_t tp_suspended;	//a flag to indicate touchpanel is in suspend or resume mode
struct wake_lock tp_wakelock; //furong add 2015.04.09


/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
extern u8 g_GestureWakeupFlag;
extern int register_msc_dev(void);

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#define VPS_NAME "virtual-proximity"
/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

extern struct input_dev *g_InputDevice;
struct virtualpsensor *vps;
extern struct msg2638_sys_data_s *msg2638_sys_data;
bool is_msg2638 = false;	//furong add 
extern struct i2c_client *g_I2cClient;

enum proximity_sensor_vendor
{
	TAOS = 1,
	STK,
	TOTAL,
};
extern void DrvPlatformLyrDisableIrqWakeup(void);
extern void DrvPlatformLyrEnableIrqWakeup(void);

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/
//zxz
#if 1
//void MsDrvInterfaceTouchDeviceSuspend(struct early_suspend *pSuspend)
void MsDrvInterfaceTouchDeviceSuspend(void)
{
    DBG("*** %s() ***\n", __func__);
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
 if( _gIsUpdateFirmware != 0x00)
	return;
#endif

	/* modify by furong, 2014/12/10, PR-865304, add for sometimes TP can't work after suspend/resume. */
	if (atomic_read(&tp_suspended)) {
		DBG("****Already in suspend state*****\n");
		return;
	}
	atomic_set(&tp_suspended, 1);

	printk("[Fu]%s\n", __func__);

	DrvPlatformLyrDisableFingerTouchReport();
	DrvPlatformLyrFingerTouchReleased(0, 0); // Send touch end for clearing point touch
	input_sync(g_InputDevice);
	
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
	if (proximity_enable == 1) {
	    proximity_enable_suspend = 1;
	    DBG("\n%s:proximity! \n",__func__);
		//device_init_wakeup(&g_I2cClient->dev, 1);
		#if 0
		if (device_may_wakeup(&g_I2cClient->dev))
		{	
			enable_irq_wake(g_I2cClient->irq);
		} 
		#else
			DrvPlatformLyrEnableIrqWakeup();
		#endif
		DrvPlatformLyrEnableFingerTouchReport();

	    return;
	}
	/*Avoid 3 times i2c error when incall in sleep,so put it here.zhanghaibin,20140228*/
	if (late_disable_proximity)
	{
		//disable proximity
		DrvMainFirmwarePSEnable(0);
		late_disable_proximity=0;
	}
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
//    g_GestureWakeupMode = 0x1FFF; // Enable all gesture wakeup mode for testing 

    if (g_GestureWakeupMode != 0x0000)
    {	
        //DrvIcFwLyrOpenGestureWakeup(g_GestureWakeupMode);
		DrvFwCtrlOpenGestureWakeup(g_GestureWakeupMode);
		DrvPlatformLyrEnableFingerTouchReport();
        return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP



    //DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrTouchDevicePowerOff(); 
    DrvPlatformLyrTouchDeviceVoltageControl(false);//zxzadd
}

extern bool charger_in;
void MsDrvInterfaceTouchDeviceResume(void)
{
	//Add by Luoxingxing.
	int i = 0;
	
    DBG("*** %s() ***\n", __func__);

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
	if( _gIsUpdateFirmware != 0x00)
		return;
#endif

/* modify by furong, 2014/12/10, PR-865304, add for sometimes TP can't work after suspend/resume.
  because sometimes suspend twice but resume once, this leads to irq disabled twice but enabled only one time,
  irq disabled times not equal to enabled times will cause TP irq not correctly enable and TP can't work. */
	if (!atomic_read(&tp_suspended)) {
		DBG("****Already in awake state*****\n");
		return;
	}
	atomic_set(&tp_suspended, 0);

	printk("[Fu]%s\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
    if (proximity_enable_suspend == 1) { //ps_open  and  face leave
        printk("[Fu]suspend:proximity! \n");
        proximity_enable_suspend = 0;
		#if 0
		if (device_may_wakeup(&g_I2cClient->dev))
		{
			disable_irq_wake(g_I2cClient->irq);
		} 
		#else
			DrvPlatformLyrDisableIrqWakeup();
		#endif
        #if 0
        if(ps_data_state[0] == 1) {
            disable_irq(data->client->irq);
            msleep(10);
            gpio_set_value(data->pdata->reset_gpio, 0);
            msleep(10);
            gpio_set_value(data->pdata->reset_gpio, 1);
            msleep(300);
            enable_irq(data->client->irq);

            ps_data_state[0] = 0;
            enable_ps();
        }
        #endif
        return;
    } 
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
	if (g_GestureWakeupFlag != 0x0000)
	{
		DrvIcFwLyrCloseGestureWakeup();
		DrvPlatformLyrDisableFingerTouchReport(); //modify by furong, 2014.12.10, to make irq enable times are equal to irq disable times
	}
#endif

    DrvPlatformLyrTouchDeviceVoltageControl(true);//zxzadd
    DrvPlatformLyrTouchDevicePowerOn();
	mdelay(50);

	if (proximity_enable == 1 && proximity_enable_suspend == 0) {
		DrvMainFirmwarePSEnable(1);
	}

	if (charger_in == true) {
		DrvIcFwLyrChargerInput();	//furong add 2015.03.10
	}

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
//    DrvIcFwLyrRestoreFirmwareModeToLogDataMode(); // Mark this function call for avoiding device driver may spend longer time to resume from suspend state.
#endif

    DrvPlatformLyrEnableFingerTouchReport(); 

	//Add by Luoxingxing to release all points when resume.START
	for (i = 0; i < MAX_TOUCH_NUM; i ++)
	{
		 input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, -1);
   		 input_mt_sync(g_InputDevice);
	}
	input_sync(g_InputDevice);
	//END
}
#endif


//zxzadd
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			MsDrvInterfaceTouchDeviceResume();
		else if (*blank == FB_BLANK_POWERDOWN)
			MsDrvInterfaceTouchDeviceSuspend();
	}

	return 0;
}
#endif

static int vps_set_enable(int enable)
{
	u16 status;
	vps->vps_enabled = enable ? 1 : 0;
	//DrvMainFirmwareProximityEnable(enable);
	if(enable == 1)
	{
		status = DrvMainFirmwareProximityStatus();
		if(status == 0x40)
		{
			input_report_abs(vps->proximity_dev, ABS_DISTANCE, 1);
			input_sync(vps->proximity_dev);
		}
		else if(status == 0x80)
		{
			input_report_abs(vps->proximity_dev, ABS_DISTANCE, 0);
			input_sync(vps->proximity_dev);
		}
	}

	return 0;
}

ssize_t virtual_proximity_enable_show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
	return sprintf(pBuf, "%d", vps->vps_enabled);
}

ssize_t virtual_proximity_enable_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	int enable;
	if (pBuf != NULL)
	{
		sscanf(pBuf, "%d", &enable);
		vps_set_enable(enable);
	}
	return nSize;
}

static DEVICE_ATTR(enable, SYSFS_AUTHORITY, virtual_proximity_enable_show, virtual_proximity_enable_store);

ssize_t proximity_vendor_show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
	switch(vps->value)
	{
		case TAOS:vps->vendor = "taos";break;
		case STK:vps->vendor = "stk";break;
		default:break;
	}

	if(vps->vendor == NULL)
		return sprintf(pBuf, "%s", "error");

	return sprintf(pBuf, "%s", vps->vendor);
}

ssize_t proximity_vendor_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	extern struct input_dev* get_taos_input_device(void);
	extern struct input_dev* get_stk_input_device(void);

	printk("proximity_vendor_store in!\n");
	if (pBuf != NULL)
	{
		sscanf(pBuf, "%d", &vps->value);
		if(vps->value == 0 || vps->value >= TOTAL)
			printk("wrong vendor value!\n");

		switch(vps->value)
		{
			case TAOS:
				vps->proximity_dev = get_taos_input_device();
				if(vps->proximity_dev == NULL)
					printk("proximity input device is NULL!\n");
				printk("TAOS priximity sensor\n");
				break;
			case STK:
				vps->proximity_dev = get_stk_input_device();
				if(vps->proximity_dev == NULL)
					printk("proximity input device is NULL!\n");
				printk("STK priximity sensor\n");
				break;

			default:
				printk("error proximity vendor!\n");
				break;
		}
	}
	return nSize;
}

static DEVICE_ATTR(vendor, SYSFS_AUTHORITY, proximity_vendor_show, proximity_vendor_store);

ssize_t proximity_function_enable_show(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
	return sprintf(pBuf, "%x", vps->proximity_function);
}

ssize_t proximity_function_enable_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
	u32 nProximityMode;
	if (pBuf != NULL)
	{
		sscanf(pBuf, "%x", &nProximityMode);
		DBG("nWakeupMode = 0x%x\n", nProximityMode);
		vps->proximity_function = nProximityMode;
		DrvMainFirmwareProximityEnable(nProximityMode);
	}
	return nSize;
}

static DEVICE_ATTR(proximity, SYSFS_AUTHORITY, proximity_function_enable_show, proximity_function_enable_store);

static int sys_device_create(void)
{
	struct class *virtual_proximity = NULL;
	struct device *virtual_proximity_device = NULL;

	virtual_proximity = class_create(THIS_MODULE, "virtual-proximity");
	if (IS_ERR(virtual_proximity))
		DBG("Failed to create class(virtual_proximity)!\n");

	virtual_proximity_device = device_create(virtual_proximity, NULL, 0, NULL, "device");
	if (IS_ERR(virtual_proximity_device))
		DBG("Failed to create device(virtual_proximity_device)!\n");

	if (device_create_file(virtual_proximity_device, &dev_attr_enable) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

	if (device_create_file(virtual_proximity_device, &dev_attr_vendor) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

	if (device_create_file(virtual_proximity_device, &dev_attr_proximity) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

	return 0;
}

/* [PLATFORM]-Add-BEGIN by TCTNB.ZXZ, FR-810275, 2014/10/14, Add I2C test before add driver */
#if 0
static int MsDrvI2cTest(struct i2c_client *client)
{
	int rc;
	char temp;
	struct i2c_msg msgs[] =
	{
		{
		.addr = SLAVE_I2C_ID_DBBUS,//client->addr,
		.flags = 0,//I2C_M_RD,
		.len = 1,
		.buf = &temp,
		},
	};
	printk("MsDrvI2cTest --client addr : %x\n", client->addr);

	rc = i2c_transfer(client->adapter, msgs, 1);
	DBG("MsDrvI2cTest-- i2c transfer rc = %d\n", rc);
	if( rc < 0 )
	{
		printk("MsDrvI2cTest--ReadI2CSeq error %d\n", rc);
		return rc;
	}

	msleep(10);
	return 0;
}
#endif
/* [PLATFORM]-Add-END by TCTNB.ZXZ */

/* probe function is used for matching and initializing input device */
s32 /*__devinit*/ MsDrvInterfaceTouchDeviceProbe(struct i2c_client *pClient, const struct i2c_device_id *pDeviceId)
{
    s32 nRetVal = 0;
    int err = 0;

    DBG("*** %s() ***\n", __func__);
/* [PLATFORM]-Add-BEGIN by TCTNB.ZXZ, FR-810275, 2014/10/14, Add I2C test before add driver */
    //err=MsDrvI2cTest(pClient);
    //if(err<0)
	//return err;
  /* [PLATFORM]-Add-END by TCTNB.ZXZ */
	mutex_initialize();

#if defined(CONFIG_ENABLE_GESTURE_WAKEUP)
	register_msc_dev();  //add by furong to support TP Gesture
#endif
	
    DrvPlatformLyrTouchDeviceVoltageInit(pClient,true);//zxzadd
    DrvPlatformLyrTouchDeviceVoltageControl(true);//zxzadd

	msg2638_pinctrl_init(pClient);//furong add 2014/12/31

    DrvPlatformLyrTouchDeviceRequestGPIO();

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    DrvPlatformLyrTouchDeviceRegulatorPowerOn();
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

    DrvPlatformLyrTouchDevicePowerOn();

    err = DrvMainTouchDeviceInitialize();
	if(err < 0) {
		pr_err("[Liu]%s: failed\n", __func__);
		return err;
	} else {
		is_msg2638 = true;	//furong add for distinguish TP 1st source and 2nd source
	}
	DrvPlatformLyrInputDeviceInitialize(pClient);
	vps = kzalloc(sizeof(struct virtualpsensor), GFP_KERNEL);
	sys_device_create();
	wake_lock_init(&tp_wakelock,WAKE_LOCK_SUSPEND, "mstar_wakelock"); //furong add 2015.04.09

    DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler();
//zxz
    //vPlatformLyrTouchDeviceRegisterEarlySuspend();

#if defined(CONFIG_FB)
	TPfb_notifier.notifier_call = fb_notifier_callback;

	err = fb_register_client(&TPfb_notifier);

	if (err)
		dev_err(&pClient->dev, "Unable to register fb_notifier: %d\n",
			err);
#endif

    DBG("*** MStar touch driver registered ***\n");
    tp_dbg = 0;
    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 /*__devexit*/ MsDrvInterfaceTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

    return DrvPlatformLyrTouchDeviceRemove(pClient);
}

void MsDrvInterfaceTouchDeviceSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() ***\n", __func__);

    DrvPlatformLyrSetIicDataRate(pClient, nIicDataRate);
}    
