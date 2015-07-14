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
 *    File  	: lgtp_project_setting.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_PROJECT_SETTING_H_ )
#define _LGTP_PROJECT_SETTING_H_


/****************************************************************************
* Project Setting ( Model )
****************************************************************************/
#if defined ( CONFIG_TOUCHSCREEN_UNIFIED )
#define TOUCH_MODEL_CF
#endif
/****************************************************************************
* Available Feature supported by Unified Driver
* If you want to use it, define it inside of model feature
****************************************************************************/
//#define ENABLE_HOVER_DETECTION
//#define ENABLE_TOUCH_AT_OFF_CHARGING

/****************************************************************************
* Project Setting ( AP Solution / AP Chipset / Touch Device )
****************************************************************************/

#if defined ( TOUCH_MODEL_CF )
/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* Ap Chipset */
#define TOUCH_PLATFORM_MSM8909

/* Touch Device */
#define TOUCH_DEVICE_LU202X

/* Touch Type*/
//#define TOUCH_TYPE_ONCELL

#else
#error "Model should be defined"
#endif

#endif /* _LGTP_PROJECT_SETTING_H_ */

/* End Of File */

