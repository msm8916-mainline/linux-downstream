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
 *    File  	: lgtp_device_lu202x.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_LU202X_H_ )
#define _LGTP_DEVICE_LU202X_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define KNOCK_PALM_FAIL			0x10
#define KNOCK_TIME_OVER_FAIL	0x11
#define KNOCK_THRE_OVER_FAIL	0x12
#define KNOCK_TAP_WAIT_FAIL		0x13
#define KNOCK_TAP_CNT_OVER_FAIL	0xFF

/****************************************************************************
* Type Definitions
****************************************************************************/
/* Debug Mode RawData Minor Type */
enum {
	GET_NONE = 0,
	GET_HISTO,		/* Max channel */
	GET_SUMHISTO,	/* Half channel */
	GET_CURCAP,		/* Max channel */
	GET_REFCAP,		/* Max channel */
	GET_SUMDIVHISTO	/* Half channel */
};

enum {
	GET_REFERENCE = 0,
	GET_JITTER,
	GET_RAW,
};

struct lge_hw_smem_id2_type {
	u32 sbl_log_meta_info;
	u32 sbl_delta_time;
	u32 lcd_maker;
	u32 build_info;		/* build type user:0 userdebug:1 eng:2 */
	int modem_reset;
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



#endif /* _LGTP_DEVICE_LU202X_H_ */

/* End Of File */

