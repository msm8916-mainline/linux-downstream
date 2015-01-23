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
#if defined ( CONFIG_TOUCHSCREEN_LU201X )
#define TOUCH_MODEL_Y30
#endif

#if defined ( CONFIG_TOUCHSCREEN_LGE_SYNAPTICS )
#define TOUCH_MODEL_C70
#endif

#if defined ( CONFIG_MSM8916_C90 )
#define TOUCH_MODEL_C90NAS
#endif

#if defined ( TARGET_MT6582_Y90 )
#define TOUCH_MODEL_Y90
#endif

#if defined ( TARGET_MT6582_Y70 )
#define TOUCH_MODEL_Y70
#endif

#if defined ( TARGET_MT6732_C90 )
#define TOUCH_MODEL_C90
#endif

#if defined ( CONFIG_TOUCHSCREEN_LGE_MELFAS )
#define TOUCH_MODEL_C50
#endif

#if defined ( TARGET_MT6582_LION_3G )
#define TOUCH_MODEL_LION_3G
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
#if defined ( TOUCH_MODEL_Y30 )

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8210

/* Touch Device */
#define TOUCH_DEVICE_LU201X
#define TOUCH_DEVICE_LU202X
#define TOUCH_DEVICE_FT6X36
#define TOUCH_DEVICE_DUMMY

/* Driver Feature */
#define ENABLE_HOVER_DETECTION

#elif defined ( TOUCH_MODEL_C70 )

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_S3320

/* Driver Feature */
#define ENABLE_TOUCH_AT_OFF_CHARGING

/* Swipe mode */
#define ENABLE_SWIPE_MODE

#elif defined ( TOUCH_MODEL_C90NAS )

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_S3320

/* Driver Feature */
#define ENABLE_TOUCH_AT_OFF_CHARGING


#elif defined ( TOUCH_MODEL_Y90 )

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6582

/* Touch Device */
#define TOUCH_DEVICE_S3320

#elif defined ( TOUCH_MODEL_Y70 )

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6582

/* Touch Device */
#define TOUCH_DEVICE_S3320

#elif defined ( TOUCH_MODEL_C90 )

/* AP Solution */
#define TOUCH_PLATFORM_MTK

/* AP Chipset */
#define TOUCH_PLATFORM_MT6732

/* Touch Device */
#define TOUCH_DEVICE_S3320

#elif defined ( TOUCH_MODEL_C50 )

/* AP Solution */
#define TOUCH_PLATFORM_QCT

/* AP Chipset */
#define TOUCH_PLATFORM_MSM8916

/* Touch Device */
#define TOUCH_DEVICE_MIT200

#else
#error "Model should be defined"
#endif

#endif /* _LGTP_PROJECT_SETTING_H_ */

/* End Of File */

