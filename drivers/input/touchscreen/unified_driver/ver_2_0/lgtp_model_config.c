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
#if defined ( TOUCH_DEVICE_DUMMY )
extern TouchDeviceSpecificFunction Dummy_Func;
#endif

#if defined ( TOUCH_DEVICE_LU201X )
extern TouchDeviceSpecificFunction Lu201x_Func;
#endif

#if defined ( TOUCH_DEVICE_LU202X )
extern TouchDeviceSpecificFunction Lu202x_Func;
#endif

#if defined ( TOUCH_DEVICE_FT6X36 )
extern TouchDeviceSpecificFunction Ft6x36_Func;
#endif

#if defined ( TOUCH_DEVICE_S3320 )
extern TouchDeviceSpecificFunction S3320_Func;
#endif

#if defined ( TOUCH_DEVICE_MIT200 )
extern TouchDeviceSpecificFunction MIT200_Func;
#endif

#if defined ( TOUCH_MODEL_LION_3G )
extern TouchDeviceSpecificFunction td4191_Func;
#endif

static TouchDeviceSpecificFunction *pDeviceSpecificFunction[MAX_DEVICE_SUPPORT] = {

	#if defined ( TOUCH_MODEL_Y30 )

		#if defined ( TOUCH_DEVICE_LU201X )
		&Lu201x_Func,
		#endif

		#if defined ( TOUCH_DEVICE_LU202X )
		&Lu202x_Func,
		#endif

		#if defined ( TOUCH_DEVICE_FT6X36 )
		&Ft6x36_Func,
		#endif

		#if defined ( TOUCH_DEVICE_DUMMY )
		&Dummy_Func,
		#endif

	#elif defined ( TOUCH_MODEL_C30 )

		#if defined ( TOUCH_DEVICE_LU202X )
		&Lu202x_Func,
		#endif

		#if defined ( TOUCH_DEVICE_DUMMY )
		&Dummy_Func,
		#endif

	#elif defined ( TOUCH_MODEL_C70 ) || defined ( TOUCH_MODEL_C90 ) || defined ( TOUCH_MODEL_Y90 ) || defined ( TOUCH_MODEL_Y70 ) || defined ( TOUCH_MODEL_C90NAS)\
        || defined( TOUCH_MODEL_P1B ) || defined ( TOUCH_MODEL_P1C) || defined ( TOUCH_MODEL_YG )

		#if defined ( TOUCH_DEVICE_S3320 )
		&S3320_Func,
		#endif

	#elif defined ( TOUCH_MODEL_C50 )

		#if defined ( TOUCH_DEVICE_MIT200 )
		&MIT200_Func,
		#endif
    #elif defined ( TOUCH_MODEL_LION_3G )
        &td4191_Func,
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
#if defined ( TOUCH_MODEL_Y30 )
static void TouchPowerY30( int isOn )
{
	static struct regulator *vdd_io = NULL;
	int error = 0;

	if( vdd_io == NULL ) {
		vdd_io = regulator_get(NULL, "vdd_ana");
		if(IS_ERR(vdd_io))
		{
			error = PTR_ERR(vdd_io);
			TOUCH_ERR("failed to get regulator ( error = %d )\n",error);
			return;
		}
		
		error = regulator_set_voltage(vdd_io, 2950000, 2950000);
		if (error < 0) {
			TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", error);
			return;
		}
	}

	if( vdd_io != NULL )
	{
		if( isOn )
		{
			error = regulator_enable(vdd_io);
			if (error < 0) {
				TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
				return;
			}
			msleep(15);
		}
		else
		{
			error = regulator_disable(vdd_io);
			if (error < 0) {
				TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
				return;
			}
		}
	}

}
#endif
#if defined ( TOUCH_MODEL_C30 )
static void TouchPowerVioC30( int isOn )
{
    static struct regulator *vdd_io = NULL;
    int error = 0;


    if( vdd_io == NULL ) {
        vdd_io = regulator_get(NULL, "vdd_ana");
        if (IS_ERR(vdd_io))
        {
            error = PTR_ERR(vdd_io);
            TOUCH_ERR("failed to get regulator ( error = %d )\n",error);
            return;
        }

        error = regulator_set_voltage(vdd_io, 1800000, 1800000);
        if (error < 0) {
            TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", error);
            return;
        }
    }

    if( vdd_io != NULL )
    {
        if( isOn )
        {
            error = regulator_enable(vdd_io);
            if (error < 0) {
                TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
                return;
            }
            msleep(15);
        }
        else
        {
            error = regulator_disable(vdd_io);
            if (error < 0) {
                TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
                return;
            }
        }
    }
}

static void TouchPowerVddC30( int isOn )
{
    static struct regulator *vdd_dd = NULL;
    int error = 0;

    if( vdd_dd == NULL ) {
        vdd_dd = regulator_get(NULL, "vdd_dd");
        if (IS_ERR(vdd_dd))
        {
            error = PTR_ERR(vdd_dd);
            TOUCH_ERR("failed to get regulator ( error = %d )\n",error);
            return;
        }

        error = regulator_set_voltage(vdd_dd, 3000000, 3000000);
        if (error < 0) {
            TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", error);
            return;
        }
    }

    if( vdd_dd != NULL )
    {
        if( isOn )
        {
            error = regulator_enable(vdd_dd);
            if (error < 0) {
                TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
                return;
            }
            msleep(15);
        }
        else
        {
            error = regulator_disable(vdd_dd);
            if (error < 0) {
                TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
                return;
            }
        }
    }
}
#endif
/****************************************************************************
* Global Functions
****************************************************************************/
void TouchGetDeviceSpecificDriver(TouchDeviceSpecificFunction ***pDeviceFunc)
{
	*pDeviceFunc = pDeviceSpecificFunction;
}

void TouchGetModelConfig(TouchDriverData *pDriverData)
{
	TouchModelConfig *pConfig = &pDriverData->mConfig;
    #if defined ( TOUCH_MODEL_LION_3G )
    pConfig->button_support = 0;
	pConfig->number_of_button = 0;
    pConfig->protocol_type = MT_PROTOCOL_B;
	pConfig->max_x = 720;
	pConfig->max_y = 1280;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 15;
	pConfig->max_orientation = 1;
	pConfig->max_id = 10;
    
	#elif defined ( TOUCH_MODEL_C30 )
	pConfig->button_support = 1;
	pConfig->number_of_button = 3;
	pConfig->button_name[0] = 158;
	pConfig->button_name[1] = 172;
	pConfig->button_name[2] = 139;
	pConfig->button_name[3] = 0;

	pConfig->max_x = 480;
	pConfig->max_y = 800;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 15;
	pConfig->max_orientation = 1;
	pConfig->max_id = 2;

	pConfig->protocol_type = MT_PROTOCOL_B;

	#elif defined ( TOUCH_MODEL_Y30 )
	
	pConfig->button_support = 1;
	pConfig->number_of_button = 4;
	pConfig->button_name[0] = 158;
	pConfig->button_name[1] = 172;
	pConfig->button_name[2] = 139;
	pConfig->button_name[3] = 249;

	pConfig->max_x = 480;
	pConfig->max_y = 800;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 15;
	pConfig->max_orientation = 1;
	pConfig->max_id = 10;

	pConfig->protocol_type = MT_PROTOCOL_B;
	
	#elif defined ( TOUCH_MODEL_C70 ) || defined ( TOUCH_MODEL_C90 ) || defined ( TOUCH_MODEL_Y90 ) || defined ( TOUCH_MODEL_Y70 ) || defined ( TOUCH_MODEL_C90NAS) || defined ( TOUCH_MODEL_P1C) || defined ( TOUCH_MODEL_YG )

	pConfig->button_support = 0;
	pConfig->number_of_button = 0;
	pConfig->button_name[0] = 0;
	pConfig->button_name[1] = 0;
	pConfig->button_name[2] = 0;
	pConfig->button_name[3] = 0;

	pConfig->max_x = 720;
	pConfig->max_y = 1280;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 15;
	pConfig->max_orientation = 1;
	pConfig->max_id = 10;

	pConfig->protocol_type = MT_PROTOCOL_B;
	
	#elif defined ( TOUCH_MODEL_C50 )

	pConfig->button_support = 0;
	pConfig->number_of_button = 0;
	pConfig->button_name[0] = 0;
	pConfig->button_name[1] = 0;
	pConfig->button_name[2] = 0;
	pConfig->button_name[3] = 0;

	pConfig->max_x = 480;
	pConfig->max_y = 854;
	pConfig->max_pressure = 0xff;
	pConfig->max_width = 30;
	pConfig->max_orientation = 1;
	pConfig->max_id = 10;

	pConfig->protocol_type = MT_PROTOCOL_B;

    #elif defined ( TOUCH_MODEL_P1B )
 
	pConfig->button_support = 0;
	pConfig->number_of_button = 0;
	pConfig->button_name[0] = 0;
	pConfig->button_name[1] = 0;
	pConfig->button_name[2] = 0;
	pConfig->button_name[3] = 0;

	pConfig->max_x = 1080;
	pConfig->max_y = 1920;
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
void TouchPowerModel( int isOn )
{
	#if defined ( TOUCH_MODEL_Y30 )

	TouchPowerY30(isOn);

	#elif defined ( TOUCH_MODEL_C30 )

	TouchPowerVddC30(isOn);
	TouchPowerVioC30(isOn);

	
	#elif defined ( TOUCH_MODEL_C70 ) || defined ( TOUCH_MODEL_Y90 )  || defined ( TOUCH_MODEL_Y70 )  || defined ( TOUCH_MODEL_C90 ) || defined ( TOUCH_MODEL_C90NAS) \
    || defined( TOUCH_MODEL_P1B ) || defined ( TOUCH_MODEL_P1C) || defined ( TOUCH_MODEL_YG )

	/* there is no power control */
	
	#elif defined ( TOUCH_MODEL_C50 )

	gpio_direction_output(TOUCH_GPIO_POWER, 1);
	
	if( isOn )
	{
		gpio_set_value(TOUCH_GPIO_POWER, 1);
	}
	else
	{
		gpio_set_value(TOUCH_GPIO_POWER, 0);
	}
	#elif defined ( TOUCH_MODEL_LION_3G )
    
	#else
	#error "Model should be defined"
	#endif
	
}

/* this function is for platform api so do not use it in other module */
void TouchAssertResetModel( void )
{
	#if defined ( TOUCH_MODEL_Y30 ) || defined ( TOUCH_MODEL_C30 )

	gpio_set_value(TOUCH_GPIO_RESET, 0);

	#elif defined ( TOUCH_MODEL_C70 ) || defined ( TOUCH_MODEL_C90NAS) || defined( TOUCH_MODEL_P1B ) || defined ( TOUCH_MODEL_P1C) || defined ( TOUCH_MODEL_YG )

	gpio_set_value(TOUCH_GPIO_RESET, 0);
	
	#elif defined ( TOUCH_MODEL_Y90 ) || defined ( TOUCH_MODEL_Y70 ) || defined ( TOUCH_MODEL_C90 )

	mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
	
	#elif defined ( TOUCH_MODEL_C50 )

    
    #elif defined ( TOUCH_MODEL_LION_3G )

    #else
	#error "Model should be defined"
	#endif
	
}

/* End Of File */

