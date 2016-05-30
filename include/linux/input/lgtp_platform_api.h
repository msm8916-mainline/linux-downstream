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
 *    File  	: lgtp_platform_api.h
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_PLATFORM_API_H_ )
#define _LGTP_PLATFORM_API_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/


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
#ifdef CONFIG_MTK_TOUCHPANEL
int ts_dma_allocation(void);
#endif

int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf);
int touch_i2c_read_byte(struct i2c_client *client, u8 reg, int len, u8 *buf);
int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 * buf);
int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data);

void TouchReset( void );
void TouchEnableIrq( void );
void TouchDisableIrq( void );
int TouchReadGpioLineInt( void );
int TouchInitialisePlatform ( struct lge_touch_data *pDriverData );



#endif /* _LGTP_PLATFORM_API_H_ */

/* End Of File */

