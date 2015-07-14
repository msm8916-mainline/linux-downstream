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
 *    File  	: lgtp_device_s3320.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_MIT200_H_ )
#define _LGTP_DEVICE_MIT200_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define MIT_REGH_CMD						0x10

#define FINGER_EVENT_SZ						6
#define MIT_FINGER_EVENT_SZ					8
#define MIT_LPWG_EVENT_SZ					3
#define KNOCKON_DELAY   					700
	
#define MAX_NUM_OF_FINGERS					10
#define MIT_ROW_NUM             			0x0B
#define MIT_COL_NUM             			0x0C

/****************************************************************************
* Type Definitions
****************************************************************************/

struct melfas_ts_data {
	struct i2c_client	*client;

	TouchState currState;
	LpwgSetting lpwgSetting;
};

enum TCI_CTRL {
	IDLE_REPORTRATE_CTRL	= 1,
	ACTIVE_REPORTRATE_CTRL,
	SENSITIVITY_CTRL,

	TCI_ENABLE_CTRL			= 11,
	TOUCH_SLOP_CTRL,
	TAP_MIN_DISTANCE_CTRL,
	TAP_MAX_DISTANCE_CTRL,
	MIN_INTERTAP_CTRL,
	MAX_INTERTAP_CTRL,
	TAP_COUNT_CTRL,
	INTERRUPT_DELAY_CTRL,

	TCI_ENABLE_CTRL2		= 21,
	TOUCH_SLOP_CTRL2,
	TAP_MIN_DISTANCE_CTRL2,
	TAP_MAX_DISTANCE_CTRL2,
	MIN_INTERTAP_CTRL2,
	MAX_INTERTAP_CTRL2,
	TAP_COUNT_CTRL2,
	INTERRUPT_DELAY_CTRL2,

	LPWG_STORE_INFO_CTRL   	= 31, 
	LPWG_START_CTRL,
	LPWG_PANEL_DEBUG_CTRL,
	LPWG_FAIL_REASON_CTRL,
};

/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
extern int MIT_FirmwareUpgrade(struct melfas_ts_data* ts, const char* fw_path);

#endif /* _LGTP_DEVICE_MIT200_H_ */

/* End Of File */

