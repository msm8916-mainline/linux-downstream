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
 * @file    mstar_drv_platform_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_interface.h"
#include <linux/regulator/consumer.h>
/*=============================================================*/
// EXTREN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kset *g_TouchKSet;
extern struct kobject *g_TouchKObj;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
extern struct regulator *g_ReguVdd;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

struct mutex g_Mutex;
static struct work_struct _gFingerTouchWork;
//zxz
#if 0
static struct early_suspend _gEarlySuspend;
#endif
 /* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
u8 irq_enabled = 1;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
const int g_TpVirtualKey[] = {TOUCH_KEY_BACK, TOUCH_KEY_HOME, TOUCH_KEY_MENU, TOUCH_KEY_SEARCH};

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#define BUTTON_W (100)
#define BUTTON_H (100)

const int g_TpVirtualKeyDimLocal[MAX_KEY_NUM][4] = {{BUTTON_W/2*1,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*3,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*5,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*7,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H}};
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

struct input_dev *g_InputDevice = NULL;
static int _gIrq = -1;

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

/* read data through I2C then report data to input sub-system when interrupt occurred */
static void _DrvPlatformLyrFingerTouchDoWork(struct work_struct *pWork)
{
    DBG("*** %s() ***\n", __func__);
    
    mutex_lock(&g_Mutex);

    DrvIcFwLyrHandleFingerTouch(NULL, 0);
    enable_irq(_gIrq);
    mutex_unlock(&g_Mutex);
}

/* The interrupt service routine will be triggered when interrupt occurred */
static irqreturn_t _DrvPlatformLyrFingerTouchInterruptHandler(s32 nIrq, void *pDeviceId)
{
    DBG("*** %s() ***\n", __func__);

    disable_irq_nosync(_gIrq);
    schedule_work(&_gFingerTouchWork);

    return IRQ_HANDLED;
}

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/
//zxzadd
int DrvPlatformLyrTouchDeviceVoltageInit(struct i2c_client *client,bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	TPvdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(TPvdd)) {
		rc = PTR_ERR(TPvdd);
		dev_err(&client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(TPvdd) > 0) {
		rc = regulator_set_voltage(TPvdd, MSG_VTG_MIN_UV,
					   MSG_VTG_MAX_UV);
		if (rc) {
			dev_err(&client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	TPvcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(TPvcc_i2c)) {
		rc = PTR_ERR(TPvcc_i2c);
		dev_err(&client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(TPvcc_i2c) > 0) {
		rc = regulator_set_voltage(TPvcc_i2c, MSG_I2C_VTG_MIN_UV,
					   MSG_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	return 0;

reg_vcc_i2c_put:
	regulator_put(TPvcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(TPvdd) > 0)
		regulator_set_voltage(TPvdd, 0, MSG_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(TPvdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(TPvdd) > 0)
		regulator_set_voltage(TPvdd, 0, MSG_VTG_MAX_UV);

	regulator_put(TPvdd);

	if (regulator_count_voltages(TPvcc_i2c) > 0)
		regulator_set_voltage(TPvcc_i2c, 0, MSG_I2C_VTG_MAX_UV);

	regulator_put(TPvcc_i2c);
	return 0;
}


int DrvPlatformLyrTouchDeviceVoltageControl(bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(TPvdd);
	if (rc) {
		pr_err("Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(TPvcc_i2c);
	if (rc) {
		pr_err("Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(TPvdd);
	}

	return rc;

power_off:
	rc = regulator_disable(TPvdd);
	if (rc) {
		pr_err("Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(TPvcc_i2c);
	if (rc) {
		pr_err("Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(TPvdd);
		if (rc) {
			pr_err("Regulator vdd enable failed rc=%d\n", rc);
		}
	}
	
	return rc;
}

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
void DrvPlatformLyrTouchDeviceRegulatorPowerOn(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
    nRetVal = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); // For specific SPRD BB chip(ex. SC7715) or QCOM BB chip(ex. MSM8610), need to enable this function call for correctly power on Touch IC.

    if (nRetVal)
    {
        DBG("Could not set to 2800mv.\n");
    }
    regulator_enable(g_ReguVdd);

    mdelay(20); //mdelay(100);
}
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

void DrvPlatformLyrTouchDevicePowerOn(void)
{
    DBG("*** %s() ***\n", __func__);
    
    gpio_direction_output(global_reset_gpio, 1);
//    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
//    mdelay(100);
    gpio_set_value(global_reset_gpio, 0);
    mdelay(10);
    gpio_set_value(global_reset_gpio, 1);
    mdelay(300);

}

void DrvPlatformLyrTouchDevicePowerOff(void)
{
    DBG("*** %s() ***\n", __func__);
//    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 0);
    gpio_set_value(global_reset_gpio, 0);

}

void DrvPlatformLyrTouchDeviceResetHw(void)
{
    DBG("*** %s() ***\n", __func__);
        gpio_direction_output(global_reset_gpio, 1);
//    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
    gpio_set_value(global_reset_gpio, 0);
    mdelay(100); 
    gpio_set_value(global_reset_gpio, 1);
    mdelay(100); 

}

void DrvPlatformLyrDisableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

//    disable_irq(MS_TS_MSG_IC_GPIO_RST);
/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
	if(1==irq_enabled)
	{
		disable_irq(_gIrq);
		irq_enabled = 0;
	}

}

void DrvPlatformLyrEnableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

//    enable_irq(MS_TS_MSG_IC_GPIO_RST);
/* [PLATFORM]-Mod- by TCTNB.ZXZ, PR-844006, 2014/11/20, add for sometimes TP can't work after work because disabled irq  many times when suspend */
	if(0==irq_enabled)
	{
		enable_irq(_gIrq);
		irq_enabled = 1;
	}

}

void DrvPlatformLyrFingerTouchPressed(s32 nX, s32 nY, s32 nPressure, s32 nId)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch pressed\n");
#ifdef TP_DEBUG_ON
    if(tp_debug_on)
    {
        printk(KERN_ERR "point touch pressed (%d,%d)\n", nX, nY);
    }
#endif

    input_report_key(g_InputDevice, BTN_TOUCH, 1);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, nId);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_report_abs(g_InputDevice, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_X, nX);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_Y, nY);

    input_mt_sync(g_InputDevice);

}

void DrvPlatformLyrFingerTouchReleased(s32 nX, s32 nY)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch released\n");
#ifdef TP_DEBUG_ON
    if(tp_debug_on)
    {
        printk(KERN_ERR "point touch released");
    }
#endif

    input_report_key(g_InputDevice, BTN_TOUCH, 0);
    input_mt_sync(g_InputDevice);

}

s32 DrvPlatformLyrInputDeviceInitialize(struct i2c_client *pClient)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    mutex_init(&g_Mutex);

    /* allocate an input device */
    g_InputDevice = input_allocate_device();
    if (g_InputDevice == NULL)
    {
        DBG("*** input device allocation failed ***\n");
        return -ENOMEM;
    }

    g_InputDevice->name = pClient->name;
    g_InputDevice->phys = "I2C";
    g_InputDevice->dev.parent = &pClient->dev;
    g_InputDevice->id.bustype = BUS_I2C;
    
    /* set the supported event type for input device */
    set_bit(EV_ABS, g_InputDevice->evbit);
    set_bit(EV_SYN, g_InputDevice->evbit);
    set_bit(EV_KEY, g_InputDevice->evbit);
    set_bit(BTN_TOUCH, g_InputDevice->keybit);
    set_bit(INPUT_PROP_DIRECT, g_InputDevice->propbit);

#ifdef CONFIG_TP_HAVE_KEY
    {
        u32 i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(g_InputDevice, EV_KEY, g_TpVirtualKey[i]);
        }
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(g_InputDevice, EV_KEY, KEY_POWER);
    input_set_capability(g_InputDevice, EV_KEY, KEY_UP);
    input_set_capability(g_InputDevice, EV_KEY, KEY_DOWN);
    input_set_capability(g_InputDevice, EV_KEY, KEY_LEFT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_RIGHT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_W);
    input_set_capability(g_InputDevice, EV_KEY, KEY_Z);
    input_set_capability(g_InputDevice, EV_KEY, KEY_V);
    input_set_capability(g_InputDevice, EV_KEY, KEY_O);
    input_set_capability(g_InputDevice, EV_KEY, KEY_M);
    input_set_capability(g_InputDevice, EV_KEY, KEY_C);
    input_set_capability(g_InputDevice, EV_KEY, KEY_E);
    input_set_capability(g_InputDevice, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*
#ifdef CONFIG_TP_HAVE_KEY
    set_bit(TOUCH_KEY_MENU, g_InputDevice->keybit); //Menu
    set_bit(TOUCH_KEY_HOME, g_InputDevice->keybit); //Home
    set_bit(TOUCH_KEY_BACK, g_InputDevice->keybit); //Back
    set_bit(TOUCH_KEY_SEARCH, g_InputDevice->keybit); //Search
#endif
*/

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_set_abs_params(g_InputDevice, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_set_abs_params(g_InputDevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX, 0, 0);

    /* register the input device to input sub-system */
    nRetVal = input_register_device(g_InputDevice);
    if (nRetVal < 0)
    {
        DBG("*** Unable to register touch input device ***\n");
    }


    return nRetVal;    
}

s32 DrvPlatformLyrTouchDeviceRequestGPIO(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
    nRetVal = gpio_request(global_reset_gpio, "C_TP_RST");     
    if (nRetVal < 0)
    {	
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_RST, nRetVal);
		goto err_request_tprst;
    }
	gpio_direction_output(global_reset_gpio, 1);

    nRetVal = gpio_request(global_irq_gpio, "C_TP_INT");    
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
		goto err_request_tpirq;
    }
	gpio_direction_input(global_irq_gpio);
    return nRetVal;    

err_request_tpirq:
	gpio_free(global_irq_gpio);
err_request_tprst:
	gpio_free(global_reset_gpio);
	return -1;
}

s32 DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    if (DrvIcFwLyrIsRegisterFingerTouchInterruptHandler())
    {    	
        /* initialize the finger touch work queue */ 
        INIT_WORK(&_gFingerTouchWork, _DrvPlatformLyrFingerTouchDoWork);

        _gIrq = gpio_to_irq(global_irq_gpio);

        /* request an irq and register the isr */
        nRetVal = request_irq(_gIrq/*MS_TS_MSG_IC_GPIO_INT*/, _DrvPlatformLyrFingerTouchInterruptHandler,
                      IRQF_TRIGGER_RISING /* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING */,
                      "msg2xxx", NULL);
        if (nRetVal != 0)
        {
            DBG("*** Unable to claim irq %d; error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
        }

    }
    
    return nRetVal;    
}	
//zxz 
#if 0
void DrvPlatformLyrTouchDeviceRegisterEarlySuspend(void)
{
    DBG("*** %s() ***\n", __func__);

    _gEarlySuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    _gEarlySuspend.suspend = MsDrvInterfaceTouchDeviceSuspend;
    _gEarlySuspend.resume = MsDrvInterfaceTouchDeviceResume;
    register_early_suspend(&_gEarlySuspend);
}
#endif

/* remove function is triggered when the input device is removed from input sub-system */
s32 DrvPlatformLyrTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

//    free_irq(MS_TS_MSG_IC_GPIO_INT, g_InputDevice);
    free_irq(_gIrq, g_InputDevice);
    gpio_free(global_irq_gpio);
    gpio_free(global_reset_gpio);
	
   DrvPlatformLyrTouchDeviceVoltageControl(false);//zxzadd
   DrvPlatformLyrTouchDeviceVoltageInit(pClient,false);//zxzadd

    input_unregister_device(g_InputDevice);
	
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    kset_unregister(g_TouchKSet);
    kobject_put(g_TouchKObj);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    return 0;
}

void DrvPlatformLyrSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() nIicDataRate = %d ***\n", __func__, nIicDataRate);
    // TODO : Please FAE colleage to confirm with customer device driver engineer
}
