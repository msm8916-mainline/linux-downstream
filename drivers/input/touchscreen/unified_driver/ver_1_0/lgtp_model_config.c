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
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>

#include <linux/input/lgtp_common_driver.h>
#include <linux/input/lgtp_platform_api.h>
#include <linux/input/lgtp_model_config.h>
#include <linux/input/lgtp_device_s3320.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/
#define GET_PROPERTY_U8(np, string, target)					\
	do {										\
		u32 tmp_val = 0;							\
		if (of_property_read_u32(np, string, &tmp_val) < 0) 	\
			target = 0;							\
		else									\
			target = (u8) tmp_val;		\
	} while(0)
#define GET_PROPERTY_U32(np, string, target)					\
do {										\
	u32 tmp_val = 0;							\
	if (of_property_read_u32(np, string, &tmp_val) < 0)		\
		target = -1;							\
	else									\
		target = tmp_val;						\
} while(0)

#define GET_PROPERTY_U32_ARRAY(np, string, target, size)			\
do {										\
	struct property *prop = of_find_property(np, string, NULL); \
	if (prop && prop->value && prop->length == size)	{		\
		int i = 0;							\
		const u8 *iprop = prop->value;					\
		for (i = 0; i < prop->length; i++)				\
			target[i] = (u32)iprop[i];				\
	}									\
} while(0)

#define GET_PROPERTY_STRING(np, string, target)					\
do {										\
	const char *tmp_val = np->name;						\
	if (of_property_read_string(np, string, &tmp_val) < 0)	\
		strncpy(target, " ", 1);					\
	else {									\
		int len = strlen(tmp_val);					\
		memcpy(target, tmp_val, len);					\
	}									\
} while(0)


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
extern int nInt_Gpio_num;
extern int nRst_Gpio_num;


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static int TouchVerifyModelConfig( TouchModelConfig *pConfig )
{
	int result = LGTC_SUCCESS;

	LGTC_FUN();

	if( pConfig->button_support ) {
		if( ( pConfig->number_of_button <= 0 ) ||( pConfig->number_of_button > MAX_BUTTON ) ) {
			LGTC_ERR("button number is over-range ( %d )\n", pConfig->number_of_button);
			result = LGTC_FAIL;
			goto earlyReturn;
		}
	}

	if( ( pConfig->max_x <= 0  ) ||( pConfig->max_y <= 0 ) ||( pConfig->max_pressure <= 0 )
		||( pConfig->max_width <= 0 ) ||( pConfig->max_orientation <= 0  ) ||( pConfig->max_id <= 0 ) ) {
		LGTC_ERR("input values are not valid ( %d, %d, %d, %d, %d, %d )\n", pConfig->max_x, pConfig->max_y,
			pConfig->max_pressure, pConfig->max_width, pConfig->max_orientation, pConfig->max_id);
		result = LGTC_FAIL;
		goto earlyReturn;
	}

	if( pConfig->max_id > MAX_FINGER ) {
		LGTC_ERR("max_id is over-range ( %d )\n", pConfig->max_id);
		result = LGTC_FAIL;
		goto earlyReturn;
	}

earlyReturn:

	return result;
}


/****************************************************************************
* Global Functions
****************************************************************************/
int TouchGetModelConfig(struct lge_touch_data *pDriverData)
{
	int result = LGTC_SUCCESS;

	TouchModelConfig *pConfig = &pDriverData->mConfig;

	struct i2c_client *client = pDriverData->client;
	struct device_node *np;

	LGTC_FUN();

	if( i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0 ) {
		LGTC_ERR("Failed at i2c_check_functionality()\n");
		result = LGTC_FAIL;
		goto earlyReturn;
	}

	np = client->dev.of_node;
	if( np == NULL ) {
		#if defined ( CONFIG_MTK_TOUCHPANEL )

		pConfig->reset_pin = GPIO_CTP_RST_PIN;
		pConfig->int_pin = GPIO_CTP_EINT_PIN;

		nRst_Gpio_num = pConfig->reset_pin;
		nInt_Gpio_num = pConfig->int_pin;

		pConfig->button_support = 0;
		pConfig->number_of_button = 0;

		pConfig->max_x = 720;
		pConfig->max_y = 1280;
		pConfig->max_pressure = 0xff;
		pConfig->max_width = 15;
		pConfig->max_orientation = 1;
		pConfig->max_id = 10;

		pConfig->protocol_type = MT_PROTOCOL_B;

		//strcpy(pConfig->fw_image,"synaptics/y70/PLG430-V1.08-PR1711225_DS5.2.12.0.1013_40047188.img");
		strcpy(pConfig->fw_image,"synaptics/y70/PLG430-V1.14-PR1741017_DS5.2.12.0.1013_4004718E.img");

		#else

		LGTC_ERR("Null Pointer ( dev->of_node is NULL )\n");
		result = LGTC_FAIL;
		goto earlyReturn;

		#endif

	} else {
		pConfig->reset_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);
		pConfig->int_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);

		nRst_Gpio_num = pConfig->reset_pin;
		nInt_Gpio_num = pConfig->int_pin;

		GET_PROPERTY_U32(np, "irqflags", pConfig->irqflags);

		GET_PROPERTY_U32(np, "button_support", pConfig->button_support);
		GET_PROPERTY_U32(np, "number_of_button", pConfig->number_of_button);
		GET_PROPERTY_U32_ARRAY(np, "button_name", pConfig->button_name, pConfig->number_of_button);

		GET_PROPERTY_U32(np, "max_x", pConfig->max_x);
		GET_PROPERTY_U32(np, "max_y", pConfig->max_y);
		GET_PROPERTY_U32(np, "max_pressure", pConfig->max_pressure);
		GET_PROPERTY_U32(np, "max_width", pConfig->max_width);
		GET_PROPERTY_U32(np, "max_orientation", pConfig->max_orientation);
		GET_PROPERTY_U32(np, "max_id", pConfig->max_id);

		GET_PROPERTY_U32(np, "protocol_type", pConfig->protocol_type);

		GET_PROPERTY_STRING(np, "fw_image", pConfig->fw_image);
	}

	result = TouchVerifyModelConfig( pConfig );
	if( result == LGTC_FAIL ) {
		LGTC_ERR("Model configuration verify fail\n");
	}

earlyReturn:

	return result;
}


/* End Of File */

