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
 *    File  	: lgtp_model_config.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MODEL]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_model_config.h>


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
atomic_t power_state;


#if defined ( TOUCH_DEVICE_LU202X )
extern TouchDeviceSpecificFunction Lu202x_Func;
struct of_device_id  Lu202x_MatchTable[2] = {{ .compatible = "unified_driver_ver2,lu202x", },};
#endif

static TouchDeviceSpecificFunction *pDeviceSpecificFunction[MAX_DEVICE_SUPPORT] = {
    #if defined ( TOUCH_MODEL_CF)
        #if defined ( TOUCH_DEVICE_LU202X )
        &Lu202x_Func,
        #endif
	#else
	#error "Model should be defined"
	#endif
	NULL
};

/* mathch table for Device Tree Model required*/
static struct of_device_id *pDeviceSpecific_MatchTablelist[MAX_DEVICE_SUPPORT] = {

    #if defined ( TOUCH_MODEL_CF )
        #if defined( TOUCH_DEVICE_LU202X)
        &Lu202x_MatchTable[0],
        #endif
	#else
	#error "Model should be defined"
	#endif

	NULL
};


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
#if defined ( TOUCH_MODEL_CF )
static void TouchPowerCF(struct i2c_client *client, int isOn )
{
    static struct regulator *vdd_dd = NULL;
    int error = 0;
	TOUCH_WARN("isOn = %d\n", isOn);

    if( vdd_dd == NULL ) {
        vdd_dd = regulator_get(&client->dev, "vdd");
        if (IS_ERR(vdd_dd))
        {
            error = PTR_ERR(vdd_dd);
            TOUCH_ERR("failed to get regulator ( error = %d )\n",error);
            return;
        }
        error = regulator_set_voltage(vdd_dd, 2800000, 2800000);
        if (error < 0) {
            TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", error);
            return;
        }
    }
    if(isOn == 1)
    {
		if (atomic_read(&power_state) != isOn) {
			error = regulator_enable(vdd_dd);
			if (error < 0) {
				TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
				return;
			}
			TouchResetCtrl(isOn);
			atomic_set(&power_state, isOn);
			msleep(15);
		}
    }
    else
    {
		if (atomic_read(&power_state) != isOn) {
			TouchResetCtrl(isOn);
			error = regulator_disable(vdd_dd);
			if (error < 0) {
				TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
				return;
			}
			atomic_set(&power_state, isOn);
		}
    }
    return;
}
#endif

/****************************************************************************
* Global Functions
****************************************************************************/
void TouchGetDeviceSpecificDriver(TouchDeviceSpecificFunction ***pDeviceFunc)
{
	*pDeviceFunc = pDeviceSpecificFunction;
}

void TouchGetDeviceSpecificMatchTable(struct of_device_id ***pMatchtableList)
{
    *pMatchtableList = pDeviceSpecific_MatchTablelist;
}

void TouchGetModelConfig(TouchDriverData *pDriverData)
{
	TouchModelConfig *pConfig = &pDriverData->mConfig;
    #if defined ( TOUCH_MODEL_CF)
    pConfig->button_support = 0;
	pConfig->number_of_button = 0;
	pConfig->button_name[0] = 158;
	pConfig->button_name[1] = 172;
	pConfig->button_name[2] = 139;
	pConfig->button_name[3] = 249;

    /*TBD - NSM TOUCH DEFAULT SETTING VALUE*/
    pConfig->max_x = 320;
	pConfig->max_y = 480;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 15;
	pConfig->max_orientation = 1;
	pConfig->max_id = 10;

	pConfig->protocol_type = MT_PROTOCOL_B;
    
	#else
	#error "Model should be defined"
	#endif
		
	TOUCH_LOG("======== Model Configuration ( Begin ) ========\n");
	TOUCH_LOG("button_support=%d\n", pConfig->button_support);
	TOUCH_LOG("number_of_button=%d\n", pConfig->number_of_button);
	TOUCH_LOG("button_name[0]=%d\n", pConfig->button_name[0]);
	TOUCH_LOG("button_name[1]=%d\n", pConfig->button_name[1]);
	TOUCH_LOG("button_name[2]=%d\n", pConfig->button_name[2]);
	TOUCH_LOG("button_name[3]=%d\n", pConfig->button_name[3]);
	TOUCH_LOG("max_x=%d\n", pConfig->max_x);
	TOUCH_LOG("max_y=%d\n", pConfig->max_y);
	TOUCH_LOG("max_pressure=%d\n", pConfig->max_pressure);
	TOUCH_LOG("max_width=%d\n", pConfig->max_width);
	TOUCH_LOG("max_orientation=%d\n", pConfig->max_orientation);
	TOUCH_LOG("max_id=%d\n", pConfig->max_id);
	TOUCH_LOG("protocol_type=%s", ( pConfig->protocol_type == MT_PROTOCOL_A ) ? "MT_PROTOCOL_A\n" : "MT_PROTOCOL_B\n");
	TOUCH_LOG("======== Model Configuration ( End ) ========\n");
	
	return;

}

/* this function is for platform api so do not use it in other module */
void TouchPowerModel(struct i2c_client *client, int isOn )
{
	#if defined ( TOUCH_MODEL_CF )
    TouchPowerCF(client, isOn);
	#else
    /* If use Power on before get device Id. use client null */
	#error "Model should be defined"
	#endif
}

/* this function is for platform api so do not use it in other module */
void TouchAssertResetModel( void )
{
    #if defined ( TOUCH_MODEL_CF)
    /* TBD */
	#else
	#error "Model should be defined"
	#endif
	
}

/* End Of File */

