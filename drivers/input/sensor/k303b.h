
/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name	: k303b.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
* Version	: V.1.0.6_ST
* Date		: 2014/Jun/18
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/
/******************************************************************************/

#ifdef CONFIG_LGE_SENSOR
#include	<soc/qcom/lge/lge_board_revision.h>
#endif/*#ifdef CONFIG_LGE_SENSOR*/

#ifndef	__K303B_H__
#define	__K303B_H__

#define K303B_ACC_DEVICE_CUSTOM_NAME		"accelerometer"
#define K303B_MAG_DEVICE_CUSTOM_NAME		"magnetic_field"

#define K303B_ACC_DEV_NAME			"k303b_acc"
#define	K303B_MAG_DEV_NAME			"k303b_mag"

#ifdef __KERNEL__

#define K303B_ACC_I2C_SAD			(0x1D)
#define K303B_MAG_I2C_SAD			(0x1E)

/* to set gpios numb connected to interrupt pins,
 * the unused ones have to be set to -EINVAL
 */
#define K303B_ACC_DEFAULT_INT1_GPIO		(-EINVAL)

#define K303B_MAG_DEFAULT_INT1_GPIO		(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define K303B_ACC_FS_MASK			(0x30)
#define K303B_ACC_FS_2G			(0x00)
#define K303B_ACC_FS_4G			(0x20)
#define K303B_ACC_FS_8G			(0x30)

/* Magnetometer Sensor Full Scale */
#define K303B_MAG_FS_MASK			(0x60)
#define K303B_MAG_FS_4G			(0x00)	/* Full scale 4 G */
#define K303B_MAG_FS_8G			(0x20)	/* Full scale 8 G */
#define K303B_MAG_FS_10G			(0x40)	/* Full scale 10 G */
#define K303B_MAG_FS_16G			(0x60)	/* Full scale 16 G */

#define K303B_ACC_MIN_POLL_PERIOD_MS		2
#define K303B_MAG_MIN_POLL_PERIOD_MS		13
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct sensor_dt_to_pdata_map {
	const char                  *dt_name;
	void                        *ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type   type;
	int                          default_val;
};


struct k303b_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(struct k303b_acc_platform_data *);
	int (*power_off)(struct k303b_acc_platform_data *);

	/* set gpio_int[1] to choose gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;

	struct regulator *vdd_reg;
	struct regulator *vdd_i2c;
};

struct k303b_mag_platform_data {

	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(struct k303b_mag_platform_data *);
	int (*power_off)(struct k303b_mag_platform_data *);

	struct regulator *vdd_reg;
	struct regulator *vdd_i2c;
};

#endif	/* __KERNEL__ */

#endif	/* __K303B_H__ */



