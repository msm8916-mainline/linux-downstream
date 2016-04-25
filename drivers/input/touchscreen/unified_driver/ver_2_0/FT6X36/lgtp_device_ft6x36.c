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
 *    File  	: lgtp_device_ft6x36.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[FT6X36]"

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
typedef struct Ft6x36DriverDataTag {

	int test;

} Ft6x36DriverData;

/****************************************************************************
* Variables
****************************************************************************/


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
* Global Functions
****************************************************************************/
int Ft6x36_Initialize(struct i2c_client *client)
{
	Ft6x36DriverData *pDriverData = NULL;

	TOUCH_FUNC();

	pDriverData = devm_kzalloc(&client->dev, sizeof(Ft6x36DriverData), GFP_KERNEL);
	if( pDriverData == NULL )
	{
		TOUCH_ERR("Fail to allocate device specific driver data\n");
		goto earlyReturn;
	}
	
	return TOUCH_SUCCESS;

earlyReturn:

	return TOUCH_FAIL;
	
}

void Ft6x36_Reset(struct i2c_client *client)
{
	TOUCH_FUNC();

	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(200);
}


int Ft6x36_Connect(void)
{
	if( TouchReadMakerId() == 1 )
	{
		TOUCH_LOG("FT6X36 was detected\n");
		return TOUCH_SUCCESS;
	}
	else 
	{
		TOUCH_LOG("FT6X36 was NOT detected\n");
		return TOUCH_FAIL;
	}
}


TouchDeviceSpecificFunction Ft6x36_Func = {

	.Initialize = Ft6x36_Initialize,
	.Reset = Ft6x36_Reset,
	.Connect = Ft6x36_Connect,
	.InitRegister = NULL,
	.InterruptHandler = NULL,
	.ReadIcFirmwareInfo = NULL,
	.GetBinFirmwareInfo = NULL,
	.UpdateFirmware = NULL,
};


/* End Of File */


