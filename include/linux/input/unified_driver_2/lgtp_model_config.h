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
 *    File  	: lgtp_model_config.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_MODEL_CONFIG_H_ )
#define _LGTP_MODEL_CONFIG_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/

/* Hardware(Board) Configuration */
#if defined ( TOUCH_MODEL_CF )
/*TBD - NSM Touch Pin Setting///*/
#define TOUCH_I2C_BUS_NUM 5
#define TOUCH_GPIO_RESET 31 + 911
#define TOUCH_GPIO_INTERRUPT 13 + 911
#define TOUCH_GPIO_MAKER_ID 2 + 911
#define TOUCH_IRQ_FLAGS ( IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND )

#else
#error "Model should be defined"
#endif



/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
void TouchGetDeviceSpecificDriver(TouchDeviceSpecificFunction ***pDeviceFunc);
void TouchGetModelConfig(TouchDriverData *pDriverData);
void TouchPowerModel(struct i2c_client *client, int isOn );
void TouchAssertResetModel( void );
void TouchGetDeviceSpecificMatchTable(struct of_device_id ***pMatchtableList);


#endif /* _LGTP_MODEL_CONFIG_H_ */

/* End Of File */

