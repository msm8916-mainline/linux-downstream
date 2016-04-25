/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  	: lgtp_device_dummy.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MIT200]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
static const char defaultFirmware[] = "melfas/mit200/c50n_global_com/L0M45P1_00_04.fw";

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/


/****************************************************************************
* Device Specific Functions
****************************************************************************/
static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret += sprintf(buf+ret, "%s\n", "Dummy Device");
	
	return ret;
}

static LGE_TOUCH_ATTR(device_name, S_IRUGO | S_IWUSR, show_device_name, NULL);

static struct attribute *Dummy_attribute_list[] = {
	&lge_touch_attr_device_name.attr,
	NULL,
};



static int Dummy_Initialize(struct i2c_client *client)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */

	return TOUCH_SUCCESS;
}

static void Dummy_Reset(struct i2c_client *client)
{
	TouchResetCtrl(0);
	msleep(5);
	TouchResetCtrl(1);
	msleep(5);
	
	TOUCH_LOG("Device was reset\n");
}


//====================================================================
// Function : Dummy_QueryDeviceConnection
// Description
//   - Check if touch IC was connected
//   - will be called at module init ( to select proper device driver function )
//   - implement using "Maker ID Pin" or "Read special register of IC"
//   - In case of using "Read special register of IC", you should be implement it using "touch_i2c_read_for_query()"
//====================================================================
static int Dummy_Connect(void)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device detection function */

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : Dummy_InitRegister
// Description
//   - Initialize touch IC register
//   - will be called after IC reset ( by reset pin )
//====================================================================
static int Dummy_InitRegister(struct i2c_client *client)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Register initialization after reset */

	return TOUCH_SUCCESS;
}

static int Dummy_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	
	/* IMPLEMENT : Interrupt Handler */

	return TOUCH_SUCCESS;

}

static int Dummy_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 0;
	pFwInfo->version = 0;

	return TOUCH_SUCCESS;	
}

static int Dummy_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141117 : */
	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 0;
	pFwInfo->version = 0;

	return TOUCH_SUCCESS;	
#else
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;

	TOUCH_FUNC();
	
	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFilename, &client->dev);
	if( ret )
	{
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);

	/* IMPLEMENT : parse and get firmware information function */
	
	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 0;
	pFwInfo->version = 0;

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;
#endif
		
}


static int Dummy_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141117 : */
	return TOUCH_SUCCESS;
#else
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;

	TOUCH_FUNC();
	
	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFilename, &client->dev);
	if( ret )
	{
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);

	/* IMPLEMENT : firmware update function */

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;
#endif
	
}

static int Dummy_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	TOUCH_FUNC();

	/* IMPLEMENT : each mode change function */

	return TOUCH_SUCCESS;
}


static int Dummy_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	/* IMPLEMENT : self-diagnosis function */

	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	dataLen += sprintf(pBuf, "%s", "========= Additional Information =========\n");
	dataLen += sprintf(pBuf+dataLen, "%s", "Device Name = Dummy\n");

	*pDataLen = dataLen;

	return TOUCH_SUCCESS;
	
}

static int Dummy_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;
	
	switch( cmd )
	{
		case READ_IC_REG:
			ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}

TouchDeviceSpecificFunction MIT200_Func = {

	.Initialize = Dummy_Initialize,
	.Reset = Dummy_Reset,
	.Connect = Dummy_Connect,
	.InitRegister = Dummy_InitRegister,
	.InterruptHandler = Dummy_InterruptHandler,
	.ReadIcFirmwareInfo = Dummy_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = Dummy_GetBinFirmwareInfo,
	.UpdateFirmware = Dummy_UpdateFirmware,
	.SetLpwgMode = Dummy_SetLpwgMode,
	.DoSelfDiagnosis = Dummy_DoSelfDiagnosis,
	.AccessRegister = Dummy_AccessRegister,
	.device_attribute_list = Dummy_attribute_list,

};


/* End Of File */


