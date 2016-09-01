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
 * @file    mstar_drv_main.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_main.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"

#include <asm/uaccess.h>

#ifdef  CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
//#include <linux/input/vir_ps.h>
#endif

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/


/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern FirmwareInfo_t g_FirmwareInfo;
extern u8 *g_LogModePacket;
extern u16 g_FirmwareMode;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

extern u8 g_ChipType;
extern u16 g_GestureWakeupMode;

#ifdef CONFIG_ENABLE_ITO_MP_TEST
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
extern TestScopeInfo_t g_TestScopeInfo;
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
#endif //CONFIG_ENABLE_ITO_MP_TEST

extern struct mutex g_Mutex;
extern atomic_t tp_suspended;
extern u8 g_GestureWakeupFlag;
extern void DrvPlatformLyrDisableIrqWakeup(void);

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

static u16 _gDebugReg[MAX_DEBUG_REGISTER_NUM] = {0};
static u32 _gDebugRegCount = 0;

static u8 *_gPlatformFwVersion = NULL; // internal use firmware version for MStar

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
char ps_data_state[1] = {1};
int proximity_enable = 0;
int proximity_enable_suspend = 0;
int tp_suspend=0;
int late_enable_proximity=0;
int late_disable_proximity=0;
#endif

#ifdef CONFIG_ENABLE_ITO_MP_TEST
static ItoTestMode_e _gItoTestMode = 0;
#endif //CONFIG_ENABLE_ITO_MP_TEST

extern struct i2c_client *g_I2cClient;

#ifdef TP_PRINT
static void tp_print_create_entry(void);
#endif

#ifdef TP_DEBUG_ON
 int tp_debug_on=0;
static void tp_debug_create_entry(void);
#endif

static u32 _gIsUpdateComplete = 0;

static u8 *_gFwVersion = NULL; // customer firmware version

static struct class *_gFirmwareClass = NULL;
static struct device *_gFirmwareCmdDev = NULL;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
struct kset *g_TouchKSet = NULL;
struct kobject *g_TouchKObj = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

u8 g_FwData[94][1024];
u32 g_FwDataCount = 0;

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/


/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

ssize_t DrvMainFirmwareChipTypeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", g_ChipType);
}

ssize_t DrvMainFirmwareChipTypeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

//    g_ChipType = DrvIcFwLyrGetChipType();

    return nSize;
}

static DEVICE_ATTR(chip_type, SYSFS_AUTHORITY, DrvMainFirmwareChipTypeShow, DrvMainFirmwareChipTypeStore);

ssize_t DrvMainFirmwareDriverVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%s", DEVICE_DRIVER_RELEASE_VERSION);
}

ssize_t DrvMainFirmwareDriverVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(driver_version, SYSFS_AUTHORITY, DrvMainFirmwareDriverVersionShow, DrvMainFirmwareDriverVersionStore);

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwareUpdateShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return sprintf(pBuf, "%s\n", _gFwVersion);
}

ssize_t DrvMainFirmwareUpdateStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DrvPlatformLyrDisableFingerTouchReport();

    DBG("*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);

    if (0 != DrvIcFwLyrUpdateFirmware(g_FwData, EMEM_ALL))
    {
        _gIsUpdateComplete = 0;
        DBG("Update FAILED\n");
    }
    else
    {
        _gIsUpdateComplete = 1;
        DBG("Update SUCCESS\n");
    }

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DrvPlatformLyrEnableFingerTouchReport();
  
    return nSize;
}

static DEVICE_ATTR(update, SYSFS_AUTHORITY, DrvMainFirmwareUpdateShow, DrvMainFirmwareUpdateStore);

ssize_t DrvMainFirmwareVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u16 nMajor = 0, nMinor = 0;
    
    DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);

    DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return sprintf(pBuf, "%s\n", _gFwVersion);
}

ssize_t DrvMainFirmwareVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    //u16 nMajor = 0, nMinor = 0;
    
    //DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);

    //DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return nSize;
}

static DEVICE_ATTR(version, SYSFS_AUTHORITY, DrvMainFirmwareVersionShow, DrvMainFirmwareVersionStore);

ssize_t DrvMainFirmwareDataShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);
    
    return g_FwDataCount;
}

ssize_t DrvMainFirmwareDataStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nCount = nSize / 1024;
    u32 i;

    DBG("*** %s() ***\n", __func__);

    if (nCount > 0) // nSize >= 1024
   	{
        for (i = 0; i < nCount; i ++)
        {
            memcpy(g_FwData[g_FwDataCount], pBuf+(i*1024), 1024);

            g_FwDataCount ++;
        }
    }
    else // nSize < 1024
    {
    		if (nSize > 0)
    		{
            memcpy(g_FwData[g_FwDataCount], pBuf, nSize);

            g_FwDataCount ++;
    		}
    }

    DBG("*** g_FwDataCount = %d ***\n", g_FwDataCount);

    if (pBuf != NULL)
    {
        DBG("*** buf[0] = %c ***\n", pBuf[0]);
    }
   
    return nSize;
}

static DEVICE_ATTR(data, SYSFS_AUTHORITY, DrvMainFirmwareDataShow, DrvMainFirmwareDataStore);

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_ITO_MP_TEST
ssize_t DrvMainFirmwareTestShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("*** ctp mp test status = %d ***\n", DrvIcFwLyrGetMpTestResult());
    
    return sprintf(pBuf, "%d", DrvIcFwLyrGetMpTestResult());
}

ssize_t DrvMainFirmwareTestStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nMode = 0;

    DBG("*** %s() ***\n", __func__);
    
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nMode);   

        DBG("Mp Test Mode = 0x%x\n", nMode);

        if (nMode == ITO_TEST_MODE_OPEN_TEST) //open test
        {
            _gItoTestMode = ITO_TEST_MODE_OPEN_TEST;
            DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_OPEN_TEST);
        }
        else if (nMode == ITO_TEST_MODE_SHORT_TEST) //short test
        {
            _gItoTestMode = ITO_TEST_MODE_SHORT_TEST;
            DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_SHORT_TEST);
        }
        else
        {
            DBG("*** Undefined MP Test Mode ***\n");
        }
    }

    return nSize;
}

static DEVICE_ATTR(test, SYSFS_AUTHORITY, DrvMainFirmwareTestShow, DrvMainFirmwareTestStore);

ssize_t DrvMainFirmwareTestLogShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 nLength = 0;
    
    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrGetMpTestDataLog(_gItoTestMode, pBuf, &nLength);
    
    return nLength;
}

ssize_t DrvMainFirmwareTestLogStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_log, SYSFS_AUTHORITY, DrvMainFirmwareTestLogShow, DrvMainFirmwareTestLogStore);

ssize_t DrvMainFirmwareTestFailChannelShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 nCount = 0;

    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrGetMpTestFailChannel(_gItoTestMode, pBuf, &nCount);
    
    return nCount;
}

ssize_t DrvMainFirmwareTestFailChannelStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_fail_channel, SYSFS_AUTHORITY, DrvMainFirmwareTestFailChannelShow, DrvMainFirmwareTestFailChannelStore);

ssize_t DrvMainFirmwareTestScopeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM) || defined(CONFIG_ENABLE_CHIP_MSG28XX)
    DrvIcFwLyrGetMpTestScope(&g_TestScopeInfo);
    
    return sprintf(pBuf, "%d,%d", g_TestScopeInfo.nMx, g_TestScopeInfo.nMy);
#else
    return 0;    
#endif //CONFIG_ENABLE_CHIP_MSG26XXM || CONFIG_ENABLE_CHIP_MSG28XX
}

ssize_t DrvMainFirmwareTestScopeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_scope, SYSFS_AUTHORITY, DrvMainFirmwareTestScopeShow, DrvMainFirmwareTestScopeStore);
#endif //CONFIG_ENABLE_ITO_MP_TEST

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
void DrvMainFirmwarePSEnable(int en)
{
    u8 ps_store_data[4];
    u8 ps_rev_data[2] = {0};
    u8 i = 0;

    if(!en) {
	  mdelay(200);
        for(i=0;i<3;i++)
        {
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x47;
            ps_store_data[3] = 0xa1;		
            IicWriteData(SLAVE_I2C_ID_DWI2C, &ps_store_data[0],4);
            mdelay(20);
            ps_store_data[0] = 0x53;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x47;
            IicWriteData(SLAVE_I2C_ID_DWI2C, &ps_store_data[0],3);
            mdelay(20);
            IicReadData(SLAVE_I2C_ID_DWI2C, &ps_rev_data[0], 2);
            mdelay(20);
            printk("[Fu]disable ps_rev_data[0]=%x(0xa1),i=%d\n",ps_rev_data[0],i);
            if(0xa1==ps_rev_data[0])
            {
				proximity_enable = 0;
				if ((atomic_read(&tp_suspended)) && (g_GestureWakeupFlag == 0)) //furong add 2014.04.20, pr987220
				{
					DrvPlatformLyrDisableIrqWakeup();
				}

                break;
            }
            else if(i<2)
            {
                mdelay(100);
            }
            else
            {
                	break;
            }
        }
    } else {
        ps_data_state[0] = 1;
		printk("[Fu]%s reset pin=%d,gesture=%d\n ", __func__, gpio_get_value(global_reset_gpio), g_GestureWakeupMode);
		
		if ((0 == gpio_get_value(global_reset_gpio)) || (g_GestureWakeupFlag != 0x0000)) { //2015.05.27 rong.fu
			proximity_enable = 1;
			device_init_wakeup(&g_I2cClient->dev, 1);
			return;
		}
	
        for(i=0;i<5;i++)
        {
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x47;
            ps_store_data[3] = 0xa0;		
            IicWriteData(SLAVE_I2C_ID_DWI2C, &ps_store_data[0],4);
            mdelay(20);
            ps_store_data[0] = 0x53;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x47;
            IicWriteData(SLAVE_I2C_ID_DWI2C, &ps_store_data[0],3);
            mdelay(20);
            IicReadData(SLAVE_I2C_ID_DWI2C, &ps_rev_data[0], 2);
            mdelay(20);
            printk("[Fu]enable ps_rev_data[0]=%x(0xa0),i=%d\n",ps_rev_data[0],i);
	
            if(0xa0==ps_rev_data[0])
            {
                proximity_enable = 1;
				device_init_wakeup(&g_I2cClient->dev, 1);
                break;
            }
            else if(i<2)
            {
                mdelay(100);
            }
            else
            {
		    gpio_direction_output(global_reset_gpio, 1);
		    gpio_set_value(global_reset_gpio, 0);
		    mdelay(100); 
		    gpio_set_value(global_reset_gpio, 1);
		    mdelay(100); 
            }

        }
    }
}

int DrvMainFirmwareProximityData(void)
{
    DBG("%d ",ps_data_state[0]);
    return ps_data_state[0];
}
void DrvMainFirmwareProximityEnable(int en)
{
	DBG("\n%s en = %d\n",__func__,en);
	//Modified by zhanghaibin for ps cannt take effect when tp is suspended.
	if ( en)
	{
		if (tp_suspend)
		{
			late_enable_proximity=1;//late enable ps
			return;
		}
	}
	else
	{
		if (tp_suspend)
		{
			late_disable_proximity=1;//late disable ps
			return;
		}
	}
	DrvMainFirmwarePSEnable(en);
}

//ps_status:  0x40 -- leave,  0x80 -- close
u16 DrvMainFirmwareProximityStatus(void)
{
    u8 ps_store_data[4];
    u8 ps_rev_data[2] = {0};
	u16 ps_status;
    ps_store_data[0] = 0x53;
    ps_store_data[1] = 0x00;
    ps_store_data[2] = 0x4C;
    IicWriteData(SLAVE_I2C_ID_DWI2C, &ps_store_data[0],3);
    mdelay(50);
    IicReadData(SLAVE_I2C_ID_DWI2C, &ps_rev_data[0], 2);
    mdelay(50);
    DBG("DrvMainFirmwareProximityStatus ps_rev_data[0]=%x\n",ps_rev_data[0]);
	ps_status = ps_rev_data[0];
	return ps_status;
}

ssize_t DrvMainFirmwareProximityFaceShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    //int cnt= 0;
    u8 ps_store_data[3];
    u8 ps_rev_data[2] = {0};
    DBG("proximity_enable=%d;ps_data_state[0]=%d;proximity_enable_suspend=%d;\n",proximity_enable,ps_data_state[0],proximity_enable_suspend);
    ps_store_data[0] = 0x53;
    ps_store_data[1] = 0x00;
    ps_store_data[2] = 0x47;
    IicWriteData(SLAVE_I2C_ID_DWI2C, &ps_store_data[0],3);
    mdelay(50);
    IicReadData(SLAVE_I2C_ID_DWI2C, &ps_rev_data[0], 2);
    mdelay(50);
    DBG("ps_rev_data[0]=%x,ps_rev_data[1]=%x\n",ps_rev_data[0],ps_rev_data[1]);
	DrvMainFirmwareProximityStatus();
    return sprintf(pBuf, "%x", ps_rev_data[0]);
}

ssize_t DrvMainFirmwareProximityFaceStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nProximityMode;
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nProximityMode);   
        DBG("nWakeupMode = 0x%x\n", nProximityMode);
        DrvMainFirmwareProximityEnable(nProximityMode);
   }
    return nSize;
}

static DEVICE_ATTR(proximity, SYSFS_AUTHORITY, DrvMainFirmwareProximityFaceShow, DrvMainFirmwareProximityFaceStore);

#endif //CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

ssize_t DrvMainFirmwareGestureWakeupModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode);

    return sprintf(pBuf, "%x", g_GestureWakeupMode);
}

ssize_t DrvMainFirmwareGestureWakeupModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nLength, nWakeupMode;
    
    DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nWakeupMode);   
        DBG("nWakeupMode = 0x%x\n", nWakeupMode);

        nLength = nSize;
        DBG("nLength = %d\n", nLength);

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }
        
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
        }
       
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
        }
		//add for phone in sleep can't wakeup
		if(g_GestureWakeupMode!=0)
				device_init_wakeup(&g_I2cClient->dev, 1);
		else
				device_init_wakeup(&g_I2cClient->dev, 0);

        DBG("g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode);
    }
        
    return nSize;
}

static DEVICE_ATTR(gesture_wakeup_mode, 0664, DrvMainFirmwareGestureWakeupModeShow, DrvMainFirmwareGestureWakeupModeStore);

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwareDebugShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 i;
    u8 nBank, nAddr;
    u16 szRegData[MAX_DEBUG_REGISTER_NUM] = {0};
    u8 szOut[MAX_DEBUG_REGISTER_NUM*25] = {0}, szValue[10] = {0};

    DBG("*** %s() ***\n", __func__);
    
    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
    mdelay(300);
    
    for (i = 0; i < _gDebugRegCount; i ++)
    {
        szRegData[i] = RegGet16BitValue(_gDebugReg[i]);
    }

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();

    for (i = 0; i < _gDebugRegCount; i ++)
    {
    	  nBank = (_gDebugReg[i] >> 8) & 0xFF;
    	  nAddr = _gDebugReg[i] & 0xFF;
    	  
        DBG("reg(0x%X,0x%X)=0x%04X\n", nBank, nAddr, szRegData[i]);

        strcat(szOut, "reg(");
        sprintf(szValue, "0x%X", nBank);
        strcat(szOut, szValue);
        strcat(szOut, ",");
        sprintf(szValue, "0x%X", nAddr);
        strcat(szOut, szValue);
        strcat(szOut, ")=");
        sprintf(szValue, "0x%04X", szRegData[i]);
        strcat(szOut, szValue);
        strcat(szOut, "\n");
    }

    return sprintf(pBuf, "%s\n", szOut);
}

ssize_t DrvMainFirmwareDebugStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 i;
    char *pCh;

    DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        DBG("*** %s() pBuf[0] = %c ***\n", __func__, pBuf[0]);
        DBG("*** %s() pBuf[1] = %c ***\n", __func__, pBuf[1]);
        DBG("*** %s() pBuf[2] = %c ***\n", __func__, pBuf[2]);
        DBG("*** %s() pBuf[3] = %c ***\n", __func__, pBuf[3]);
        DBG("*** %s() pBuf[4] = %c ***\n", __func__, pBuf[4]);
        DBG("*** %s() pBuf[5] = %c ***\n", __func__, pBuf[5]);

        DBG("nSize = %d\n", (int)nSize);
       
        i = 0;
        while ((pCh = strsep((char **)&pBuf, " ,")) && (i < MAX_DEBUG_REGISTER_NUM))
        {
            DBG("pCh = %s\n", pCh);
            
            _gDebugReg[i] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

            DBG("_gDebugReg[%d] = 0x%04X\n", i, _gDebugReg[i]);
            i ++;
        }
        _gDebugRegCount = i;
        
        DBG("_gDebugRegCount = %d\n", _gDebugRegCount);
    }

    return nSize;
}

static DEVICE_ATTR(debug, SYSFS_AUTHORITY, DrvMainFirmwareDebugShow, DrvMainFirmwareDebugStore);

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwarePlatformVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

    return sprintf(pBuf, "%s\n", _gPlatformFwVersion);
}

ssize_t DrvMainFirmwarePlatformVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DrvIcFwLyrGetPlatformFirmwareVersion(&_gPlatformFwVersion);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

    return nSize;
}

static DEVICE_ATTR(platform_version, SYSFS_AUTHORITY, DrvMainFirmwarePlatformVersionShow, DrvMainFirmwarePlatformVersionStore);

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
ssize_t DrvMainFirmwareModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    g_FirmwareMode = DrvIcFwLyrGetFirmwareMode();
    
    DBG("%s() firmware mode = 0x%x\n", __func__, g_FirmwareMode);

    return sprintf(pBuf, "%x", g_FirmwareMode);
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;

    DBG("%s() firmware mode = 0x%x, can change firmware mode = %d\n", __func__, g_FirmwareInfo.nFirmwareMode, g_FirmwareInfo.nIsCanChangeFirmwareMode);

    return sprintf(pBuf, "%x,%d", g_FirmwareInfo.nFirmwareMode, g_FirmwareInfo.nIsCanChangeFirmwareMode);
#endif
}

ssize_t DrvMainFirmwareModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    u32 nMode;
    
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nMode);   
        DBG("firmware mode = 0x%x\n", nMode);

        if (nMode == FIRMWARE_MODE_DEMO_MODE) //demo mode
        {
            g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_DEMO_MODE);
        }
        else if (nMode == FIRMWARE_MODE_DEBUG_MODE) //debug mode
        {
            g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_DEBUG_MODE);
        }
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
        else if (nMode == FIRMWARE_MODE_RAW_DATA_MODE) //raw data mode
        {
            g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_RAW_DATA_MODE);
        }
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
        else
        {
            DBG("*** Undefined Firmware Mode ***\n");
        }
    }

    DBG("*** g_FirmwareMode = 0x%x ***\n", g_FirmwareMode);

    return nSize;
}

static DEVICE_ATTR(mode, SYSFS_AUTHORITY, DrvMainFirmwareModeShow, DrvMainFirmwareModeStore);
/*
ssize_t DrvMainFirmwarePacketShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;
    
    DBG("*** %s() ***\n", __func__);
    
    if (g_LogModePacket != NULL)
    {
        DBG("g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode, g_LogModePacket[0], g_LogModePacket[1]);
        DBG("g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n", g_LogModePacket[2], g_LogModePacket[3]);
        DBG("g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n", g_LogModePacket[4], g_LogModePacket[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
        if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5 || g_LogModePacket[0] == 0xAB || g_LogModePacket[0] == 0xA7))
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
        if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE) && (g_LogModePacket[0] == 0x62))
#endif
        {
            for (i = 0; i < g_FirmwareInfo.nLogModePacketLength; i ++)
            {
                pBuf[i] = g_LogModePacket[i];
            }

            nLength = g_FirmwareInfo.nLogModePacketLength;
            DBG("nLength = %d\n", nLength);
        }
        else
        {
            DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
//        nLength = 0;
        }
    }
    else
    {
        DBG("g_LogModePacket is NULL\n");
//        nLength = 0;
    }

    return nLength;
}

ssize_t DrvMainFirmwarePacketStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(packet, SYSFS_AUTHORITY, DrvMainFirmwarePacketShow, DrvMainFirmwarePacketStore);
*/
ssize_t DrvMainFirmwareSensorShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    if (g_FirmwareInfo.nLogModePacketHeader == 0xA5 || g_FirmwareInfo.nLogModePacketHeader == 0xAB)
    {
        return sprintf(pBuf, "%d,%d", g_FirmwareInfo.nMx, g_FirmwareInfo.nMy);
    }
    else if (g_FirmwareInfo.nLogModePacketHeader == 0xA7)
    {
        return sprintf(pBuf, "%d,%d,%d,%d", g_FirmwareInfo.nMx, g_FirmwareInfo.nMy, g_FirmwareInfo.nSs, g_FirmwareInfo.nSd);
    }
    else
    {
        DBG("Undefined debug mode packet format : 0x%x\n", g_FirmwareInfo.nLogModePacketHeader);
        return 0;
    }
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    return sprintf(pBuf, "%d", g_FirmwareInfo.nLogModePacketLength);
#endif
}

ssize_t DrvMainFirmwareSensorStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);
/*
    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
*/    
    return nSize;
}

static DEVICE_ATTR(sensor, SYSFS_AUTHORITY, DrvMainFirmwareSensorShow, DrvMainFirmwareSensorStore);

ssize_t DrvMainFirmwareHeaderShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", g_FirmwareInfo.nLogModePacketHeader);
}

ssize_t DrvMainFirmwareHeaderStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);
/*
    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
*/    
    return nSize;
}

static DEVICE_ATTR(header, SYSFS_AUTHORITY, DrvMainFirmwareHeaderShow, DrvMainFirmwareHeaderStore);

//------------------------------------------------------------------------------//

ssize_t DrvMainKObjectPacketShow(struct kobject *pKObj, struct kobj_attribute *pAttr, char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;

    DBG("*** %s() ***\n", __func__);

    if (strcmp(pAttr->attr.name, "packet") == 0)
    {
        if (g_LogModePacket != NULL)
        {
            DBG("g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode, g_LogModePacket[0], g_LogModePacket[1]);
            DBG("g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n", g_LogModePacket[2], g_LogModePacket[3]);
            DBG("g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n", g_LogModePacket[4], g_LogModePacket[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
            if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5 || g_LogModePacket[0] == 0xAB || g_LogModePacket[0] == 0xA7))
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
            if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE) && (g_LogModePacket[0] == 0x62))
#endif
            {
                for (i = 0; i < g_FirmwareInfo.nLogModePacketLength; i ++)
                {
                    pBuf[i] = g_LogModePacket[i];
                }

                nLength = g_FirmwareInfo.nLogModePacketLength;
                DBG("nLength = %d\n", nLength);
            }
            else
            {
                DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
//            nLength = 0;
            }
        }
        else
        {
            DBG("g_LogModePacket is NULL\n");
//            nLength = 0;
        }
    }
    else
    {
        DBG("pAttr->attr.name = %s \n", pAttr->attr.name);
//        nLength = 0;
    }

    return nLength;
}

ssize_t DrvMainKObjectPacketStore(struct kobject *pKObj, struct kobj_attribute *pAttr, const char *pBuf, size_t nCount)
{
    DBG("*** %s() ***\n", __func__);
/*
    if (strcmp(pAttr->attr.name, "packet") == 0)
    {

    }
*/    	
    return nCount;
}

static struct kobj_attribute packet_attr = __ATTR(packet, 0666, DrvMainKObjectPacketShow, DrvMainKObjectPacketStore);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *attrs[] = {
    &packet_attr.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory. If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
struct attribute_group attr_group = {
    .attrs = attrs,
};
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG


//Bug814306
static ssize_t tp_fw_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

u16 nMajor = 0, nMinor = 0;

    u8 szDbBusTxData[3] = {0};
    u8 szDbBusRxData[4] = {0};

    DBG("*** %s() ***\n", __func__);

    szDbBusTxData[0] = 0x53;
    szDbBusTxData[1] = 0x00;
    szDbBusTxData[2] = 0x2A;
       pr_err("zxz====tp_fw_version_show=========11111111\n");
 
    mutex_lock(&g_Mutex);

    IicWriteData(SLAVE_I2C_ID_DWI2C, &szDbBusTxData[0], 3);
    IicReadData(SLAVE_I2C_ID_DWI2C, &szDbBusRxData[0], 4);
    pr_err("zxz===tp_fw_version_show=========222222\n");

    mutex_unlock(&g_Mutex);

    nMajor = (szDbBusRxData[1]<<8) + szDbBusRxData[0];
   nMinor = (szDbBusRxData[3]<<8) + szDbBusRxData[2];

    pr_err("zxz==== ===========major = %d ***\n", nMajor);
    pr_err("zxz================ minor = %d ***\n", nMinor);
    return sprintf(buf,"nMajor=%03d;nMinor=%03d", nMajor, nMinor);
}


static DEVICE_ATTR(tp_fw_version, 0444, tp_fw_version_show, NULL);
//------------------------------------------------------------------------------//

extern s32 _gCtpMpTestStatus;
extern s32 _gResult[MAX_MUTUAL_NUM];

static ssize_t ctp_rawdata_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	char *s = buf;
	int i = 0, j = 0;

	for (i = 0; i < 13; i ++)
	{
    	s += sprintf(s, "\nSense[%02d]\t", i);
        
        for (j = 0; j < 22; j ++)
        {
			s += sprintf(s, "%d\t", _gResult[i * 22 + j]);
        }
    }
	
	s += sprintf(s, "\n");

	return (s - buf);
}

extern unsigned long mstar_irq_times;

static ssize_t rd_result_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u16 r70,r71,r72,r60,r6f, r50, r0f00;
	char *s = buf;

	DBG("*** %s() ***\n", __func__);

	s += sprintf(s, "mstar_irq_times = %ld\n", mstar_irq_times);
	DBG("mstar_irq_times = %ld\n", mstar_irq_times);

	// Erase TP Flash first
	DbBusEnterSerialDebugMode();
	DbBusStopMCU();
	DbBusIICUseBus();
	DbBusIICReshape();
	mdelay(50);   //(300);
	
	// Stop MCU
	//RegSetLByteValue(0x0FE6, 0x01);
	
	// Disable watchdog
	//RegSet16BitValue(0x3C60, 0xAA55);

	r70 = RegGet16BitValue((0x3C<<8)|(0x70<<1));
	r71 = RegGet16BitValue((0x3C<<8)|(0x71<<1));
	r72 = RegGet16BitValue((0x3C<<8)|(0x72<<1));	
	r60 = RegGet16BitValue((0x3C<<8)|(0x60<<1));
	r6f = RegGet16BitValue((0x3C<<8)|(0x6f<<1));
	r50 = RegGet16BitValue((0x3C<<8)|(0x50<<1));
	r0f00 = RegGet16BitValue((0x0f<<8)|(0x00<<1));
	s += sprintf(s, "Face cmd: r70 = 0x%x\n", r70);
	DBG("Face cmd: r70 = 0x%x\n", r70);
	s += sprintf(s, "Face state: r71 = 0x%x\n", r71);
	DBG("Face state:r71 = 0x%x\n", r71);	
	s += sprintf(s, "Water state: r72 = 0x%x\n", r72);
	DBG("Water state: r72 = 0x%x\n", r72);
	s += sprintf(s, "Palm state: r60 = 0x%x\n", r60);
	DBG("Palm state: r60 = 0x%x\n", r60);
	s += sprintf(s, "TP state: r6f = 0x%x\n", r6f);
	DBG("TP state: r6f = 0x%x\n", r6f);
	s += sprintf(s, "strike: r50 = 0x%x\n", r50);
	DBG("strike: r50 = 0x%x\n", r50);
	s += sprintf(s, "r0f00 = 0x%x\n", r0f00);
	DBG("r0f00 = 0x%x\n", r0f00);


	//nChipType = RegGet16BitValue(0x1ECC) & 0xFF;
	//DrvPlatformLyrTouchDeviceResetHw();

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();

	return (s - buf);
}

static ssize_t tp_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "tp_dbg = %d\n", tp_dbg);
}


static ssize_t tp_debug_store(struct device *dev, 
				struct device_attribute *attr, const char *buf, size_t nSize)
{
	if (buf != NULL) 
		sscanf(buf, "%d", &tp_dbg);	
	
	return nSize;	
}

static DEVICE_ATTR(rawdata, 0444, ctp_rawdata_show, NULL);
static DEVICE_ATTR(rd_result, 0444, rd_result_show, NULL);
static DEVICE_ATTR(ctp_debug, SYSFS_AUTHORITY, tp_debug_show, tp_debug_store);


//------------------------------------------------------------------------------//

s32 DrvMainTouchDeviceInitialize(void)
{
    s32 nRetVal = 0;
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    u8 *pDevicePath = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** %s() ***\n", __func__);

    /* set sysfs for firmware */
    _gFirmwareClass = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(_gFirmwareClass))
        DBG("Failed to create class(firmware)!\n");

    _gFirmwareCmdDev = device_create(_gFirmwareClass, NULL, 0, NULL, "device");
    if (IS_ERR(_gFirmwareCmdDev))
        DBG("Failed to create device(_gFirmwareCmdDev)!\n");

    // version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_update) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_data) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

    // debug
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_debug) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
    // platform version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_platform_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_platform_version.attr.name);

    // chip type
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_chip_type) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_chip_type.attr.name);
    // driver version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_driver_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_driver_version.attr.name);


//Bug814306
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_tp_fw_version) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_tp_fw_version.attr.name);

    if (device_create_file(_gFirmwareCmdDev, &dev_attr_rawdata) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_rawdata.attr.name);

    if (device_create_file(_gFirmwareCmdDev, &dev_attr_rd_result) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_rd_result.attr.name);

    if (device_create_file(_gFirmwareCmdDev, &dev_attr_ctp_debug) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_ctp_debug.attr.name);


    dev_set_drvdata(_gFirmwareCmdDev, NULL);

#ifdef  CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
    //tsps_assist_register_callback("msg26xxa",&DrvMainFirmwareProximityEnable,&DrvMainFirmwareProximityData);
#endif

#ifdef CONFIG_ENABLE_ITO_MP_TEST
    DrvIcFwLyrCreateMpTestWorkQueue();
#endif //CONFIG_ENABLE_ITO_MP_TEST

#ifdef TP_PRINT
    tp_print_create_entry();
#endif

#ifdef TP_DEBUG_ON
    tp_debug_create_entry();
#endif

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    /* create a kset with the name of "kset_example" which is located under /sys/kernel/ */
    g_TouchKSet = kset_create_and_add("kset_example", NULL, kernel_kobj);
    if (!g_TouchKSet)
    {
        DBG("*** kset_create_and_add() failed, nRetVal = %d ***\n", nRetVal);

        nRetVal = -ENOMEM;
    }

    g_TouchKObj = kobject_create();
    if (!g_TouchKObj)
    {
        DBG("*** kobject_create() failed, nRetVal = %d ***\n", nRetVal);

        nRetVal = -ENOMEM;
		    kset_unregister(g_TouchKSet);
		    g_TouchKSet = NULL;
    }

    g_TouchKObj->kset = g_TouchKSet;

    nRetVal = kobject_add(g_TouchKObj, NULL, "%s", "kobject_example");
    if (nRetVal != 0)
    {
        DBG("*** kobject_add() failed, nRetVal = %d ***\n", nRetVal);

		    kobject_put(g_TouchKObj);
		    g_TouchKObj = NULL;
		    kset_unregister(g_TouchKSet);
		    g_TouchKSet = NULL;
    }
    
    /* create the files associated with this kobject */
    nRetVal = sysfs_create_group(g_TouchKObj, &attr_group);
    if (nRetVal != 0)
    {
        DBG("*** sysfs_create_file() failed, nRetVal = %d ***\n", nRetVal);

        kobject_put(g_TouchKObj);
		    g_TouchKObj = NULL;
		    kset_unregister(g_TouchKSet);
		    g_TouchKSet = NULL;
    }
    
    pDevicePath = kobject_get_path(g_TouchKObj, GFP_KERNEL);
    DBG("DEVPATH = %s\n", pDevicePath);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    g_ChipType = DrvIcFwLyrGetChipType();

	if(!g_ChipType) {
		pr_err("%s: failed to get touch type", __func__);
		return -EINVAL;
	}
	else {
		pr_info("%s: g_ChipType=%d", __func__, g_ChipType);
	}

#ifdef CONFIG_ENABLE_ITO_MP_TEST
	// test
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_test) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_test.attr.name);
	// test_log
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_log) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_test_log.attr.name);
	// test_fail_channel
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_fail_channel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_test_fail_channel.attr.name);
	// test_scope
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_scope) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_test_scope.attr.name);
#endif //CONFIG_ENABLE_ITO_MP_TEST

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	// mode
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_mode) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_mode.attr.name);
	// packet
//	  if (device_create_file(_gFirmwareCmdDev, &dev_attr_packet) < 0)
//		  DBG("Failed to create device file(%s)!\n", dev_attr_packet.attr.name);
	// sensor
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_sensor) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_sensor.attr.name);
	// header
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_header) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_header.attr.name);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

	// proximity
	if (device_create_file(_gFirmwareCmdDev, &dev_attr_proximity) < 0)
		DBG("Failed to create device file(%s)!\n", dev_attr_proximity.attr.name);	
	#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    // gesture wakeup mode
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_gesture_wakeup_mode) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_gesture_wakeup_mode.attr.name);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
	
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    // get firmware mode for parsing packet judgement.
    g_FirmwareMode = DrvIcFwLyrGetFirmwareMode();
#endif //CONFIG_ENABLE_CHIP_MSG26XXM

    memset(&g_FirmwareInfo, 0x0, sizeof(FirmwareInfo_t));

    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
    DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

    return nRetVal;
}

#ifdef TP_PRINT
#include <linux/proc_fs.h>

#define TP_PRINT_AUTHORITY 0777
#define PROC_TP_PRINT_MSG   "msgtpp"
#define PROC_TP_PRINT_NODE  "tpp"

static struct proc_dir_entry *msg_tpp = NULL;
static struct proc_dir_entry *tpp = NULL;
static u16 InfoAddr = 0x0F, PoolAddr = 0x10, TransLen = 256;
static u16 cnt, head, tail;
static u8 row, units, update;

static int tp_print_proc_read(struct file *filp,char __user *page, size_t len,loff_t *pos)
{
    u16 i, j;
    u16 left, offset;
    u8 dbbus_tx_data[3] = {0};
    u8 dbbus_rx_data[8] = {0};
    u8 u8Data;
    s16 s16Data;
    s32 s32Data;
    char *buf = NULL;
    int rc = 0;

    if ((InfoAddr != 0x0F) && (PoolAddr != 0x10))
    {
        if (update)
        {
            dbbus_tx_data[0] = 0x53;
            dbbus_tx_data[1] = (InfoAddr >> 8) & 0xFF;
            dbbus_tx_data[2] = InfoAddr & 0xFF;
            mutex_lock(&g_Mutex);
            IicWriteData(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
            IicReadData(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 8);
            mutex_unlock(&g_Mutex);

            units = dbbus_rx_data[0];
            row = dbbus_rx_data[1];
            cnt = dbbus_rx_data[2];
            head = dbbus_rx_data[3];
            tail = dbbus_rx_data[4];
            TransLen = (dbbus_rx_data[7]<<8) + dbbus_rx_data[6];
            printk("tpp: row=%d, units=%d\n", row, units);
            printk("tpp: cnt=%d, head=%d, tail=%d\n", cnt, head, tail);
        }
        else
        {
            printk("tpp: \n");
        }

        offset = 0;
        left = cnt*row*units;
        buf = kmalloc(left, GFP_KERNEL);
        if (buf != NULL)
        {
            while (left > 0)
            {
                dbbus_tx_data[0] = 0x53;
                dbbus_tx_data[1] = ((PoolAddr + offset) >> 8) & 0xFF;
                dbbus_tx_data[2] = (PoolAddr + offset) & 0xFF;
                mutex_lock(&g_Mutex);
                IicWriteData(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
                IicReadData(SLAVE_I2C_ID_DWI2C, &buf[offset], left > TransLen ? TransLen : left);
                mutex_unlock(&g_Mutex);

                if (left > TransLen)
                {
                    left -= TransLen;
                    offset += TransLen;
                }
                else
                {
                    left = 0;
                }
            }

            i = head;
            while ((cnt*row*units > 0) && (rc >= 0))
            {
                if (i < cnt)
                {
                    if (i == tail)
                    {
                        rc = -1;
                    }
                }
                else
                {
                    if (tail >= cnt)
                    {
                        rc = -1;
                    }
                    else
                    {
                        i = 0;
                    }
                }

                if (rc >= 0)
                {
                    printk("tpp: ");
                    for (j = 0; j < row; j++)
                    {
                        if (units == 1)
                        {
                            u8Data = buf[i*row*units + j*units];
                            printk("%d\t", u8Data);
                        }
                        else if (units == 2)
                        {
                            s16Data = buf[i*row*units + j*units] + (buf[i*row*units + j*units + 1] << 8);
                            printk("%d\t", s16Data);
                        }
                        else if (units == 4)
                        {
                            s32Data = buf[i*row*units + j*units] + (buf[i*row*units + j*units + 1] << 8) + (buf[i*row*units + j*units + 2] << 16) + (buf[i*row*units + j*units + 3] << 24);
                            printk("%d\t", s32Data);
                        }
                    }
                    printk("\n");
                    i++;
                }
            }

            kfree(buf);
        }
    }
    return 0;
}

static void tp_print_create_entry(void)
{
    u8 dbbus_tx_data[3] = {0};
    u8 dbbus_rx_data[8] = {0};

    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x56;
    mutex_lock(&g_Mutex);
    IicWriteData(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
    IicReadData(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 4);
    mutex_unlock(&g_Mutex);
    InfoAddr = (dbbus_rx_data[1]<<8) + dbbus_rx_data[0];
    PoolAddr = (dbbus_rx_data[3]<<8) + dbbus_rx_data[2];
    printk("InfoAddr=0x%X\n", InfoAddr);
    printk("PoolAddr=0x%X\n", PoolAddr);

    if ((InfoAddr != 0x0F) && (PoolAddr != 0x10))
    {
	static const struct file_operations debug_ops = {
	.read		= tp_print_proc_read,
	};
        msg_tpp = proc_mkdir(PROC_TP_PRINT_MSG, NULL);
        tpp = proc_create_data(PROC_TP_PRINT_NODE, TP_PRINT_AUTHORITY, msg_tpp,&debug_ops,(void *)g_I2cClient);
        if (NULL == tpp)
        {
            printk("tp_print_create_entry failed\n");
        }
        else
        {
            //tpp->read_proc = tp_print_proc_read;
            //tpp->write_proc = NULL;
            printk("tp_print_create_entry OK\n");
        }

        msleep(10);
        dbbus_tx_data[0] = 0x53;
        dbbus_tx_data[1] = (InfoAddr >> 8) & 0xFF;
        dbbus_tx_data[2] = InfoAddr & 0xFF;
        mutex_lock(&g_Mutex);
        IicWriteData(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
        IicReadData(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 8);
        mutex_unlock(&g_Mutex);

        units = dbbus_rx_data[0];
        row = dbbus_rx_data[1];
        cnt = dbbus_rx_data[2];
        head = dbbus_rx_data[3];
        tail = dbbus_rx_data[4];
        update = dbbus_rx_data[5];
        TransLen = (dbbus_rx_data[7]<<8) + dbbus_rx_data[6];
        printk("tpp: row=%d, units=%d\n", row, units);
        printk("tpp: cnt=%d, head=%d, tail=%d\n", cnt, head, tail);
        printk("tpp: update=%d, TransLen=%d\n", update, TransLen);
    }
}
#endif

#ifdef TP_DEBUG_ON
#include <linux/proc_fs.h>

#define DEBUG_AUTHORITY 0777
#define PROC_TP_DEBUG      "tp_debug"
#define PROC_DEBUG_ON      "debug_on"

static struct proc_dir_entry *tp_debug = NULL;
static struct proc_dir_entry *debug_on = NULL;

static int tp_debug_proc_write(struct file *filp,const char __user *buff, size_t len,loff_t *ops)
{
    char *buf;

    if (len < 1)
    return -EINVAL;

    buf = kmalloc(len, GFP_KERNEL);
    if (!buf)
    return -ENOMEM;

    if (copy_from_user(buf, buff, len))
    {
        kfree(buf);
        return -EFAULT;
    }

    if (buf[0] >= '0' && buf[0] <= '9')
    {
        tp_debug_on = buf[0] - '0';
    }
    else
    {
        kfree(buf);
        return -EINVAL;
    }

    kfree(buf);
    return len;
}

static int tp_debug_proc_read(struct file *filp,char __user *page, size_t len,loff_t *pos)
{
    printk("tp_debug_on=%d\n",tp_debug_on);
    return sprintf(page, "%d\n", tp_debug_on);
}

static void tp_debug_create_entry(void)
{
	static const struct file_operations debug_ops = {
	.read= tp_debug_proc_read,
	.write= tp_debug_proc_write,
	};
    tp_debug = proc_mkdir(PROC_TP_DEBUG, NULL);
    debug_on = proc_create_data(PROC_DEBUG_ON, DEBUG_AUTHORITY, tp_debug,&debug_ops,(void *)g_I2cClient);
    if (NULL==debug_on)
    {
        printk("tp_debug_create_entry failed\n");
    }
    else
    {
        //debug_on->read_proc = tp_debug_proc_read;
        //debug_on->write_proc = tp_debug_proc_write;
        printk("tp_debug_create_entry OK\n");
    }
}
#endif

//------------------------------------------------------------------------------//
