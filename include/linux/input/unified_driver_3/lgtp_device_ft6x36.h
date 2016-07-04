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
 *    File  	: lgtp_device_ft6x36.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description : 
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_FT6X36_H_ )
#define _LGTP_DEVICE_FT6X36_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


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
/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	2
#define MT_MAX_TOUCH_POINTS	9

#define PRESS_MAX	0xFF
#define FT_PRESS		0x7F

#define Proximity_Max	32

#define FT_FACE_DETECT_ON		0xc0
#define FT_FACE_DETECT_OFF		0xe0

#define FT_FACE_DETECT_ENABLE	1
#define FT_FACE_DETECT_DISABLE	0
#define FT_FACE_DETECT_REG		0xB0

#define FT6X06_NAME 	"ft6x06_ts"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_FACE_DETECT_POS		1
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5
#define FT_TOUCH_WEIGHT          7
#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

    /*register address*/
#define FT6x06_REG_FW_VER		0xA6
#define FT6x06_REG_POINT_RATE	0x88
#define FT6x06_REG_THGROUP	0x80

/****************************************************************************
* Global Function Prototypes
****************************************************************************/
void focaltech_knockbaseaddr_set(int sel);
int  focaltech_knockbaseaddr_get(void);

#endif /* _LGTP_DEVICE_FT6X36_H_ */

/* End Of File */

