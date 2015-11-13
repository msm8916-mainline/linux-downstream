
/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : lsm6ds3.h
* Authors	     : AMS - Motion Mems Division - Application Team
*		     : Giuseppe Barba (giuseppe.barba@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
* Version            : V.1.0.0
* Date               : 2014/Feb/11
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
********************************************************************************/

#ifndef	__LSM6DS3_H__
#define	__LSM6DS3_H__

#define LSM6DS3_ACC_GYR_DEV_NAME	"lsm6ds3"
#define LSM6DS3_ACC_DEV_NAME		"accelerometer"
#define LSM6DS3_GYR_DEV_NAME    	"gyroscope"
#define LSM6DS3_STEP_COUNTER_DEV_NAME   "lsm6ds3_stc"

#ifdef	__KERNEL__
/**********************************************/
/* 	Accelerometer section defines	 	*/
/**********************************************/
#define LSM6DS3_ACC_MIN_POLL_PERIOD_MS	1

/* Accelerometer Sensor Full Scale */
#define LSM6DS3_ACC_FS_MASK		(0x0c)
#define LSM6DS3_ACC_FS_2G 		(0x00)	/* Full scale 2g */
#define LSM6DS3_ACC_FS_4G 		(0x08)	/* Full scale 4g */
#define LSM6DS3_ACC_FS_8G 		(0x0c)	/* Full scale 8g */
#define LSM6DS3_ACC_FS_16G		(0x04)	/* Full scale 16g */

/* Accelerometer Anti-Aliasing Filter */
#define LSM6DS3_ACC_BW_400		(0X00)
#define LSM6DS3_ACC_BW_200		(0X01)
#define LSM6DS3_ACC_BW_100		(0X02)
#define LSM6DS3_ACC_BW_50		(0X03)
#define LSM6DS3_ACC_BW_MASK		(0X03)

#define LSM6DS3_INT1_GPIO_DEF		(-EINVAL)
#define LSM6DS3_INT2_GPIO_DEF		(-EINVAL)

#define LSM6DS3_ACC_ODR_OFF		(0x00)
#define LSM6DS3_ACC_ODR_MASK		(0xF0)
#define LSM6DS3_ACC_ODR_13		(0x10)
#define LSM6DS3_ACC_ODR_26		(0x20)
#define LSM6DS3_ACC_ODR_52		(0x30)
#define LSM6DS3_ACC_ODR_104		(0x40)
#define LSM6DS3_ACC_ODR_208		(0x50)
#define LSM6DS3_ACC_ODR_416		(0x60)
#define LSM6DS3_ACC_ODR_833		(0x70)
#define LSM6DS3_ACC_ODR_1666		(0x80)
#define LSM6DS3_ACC_ODR_3330		(0x90)
#define LSM6DS3_ACC_ODR_6660		(0xA0)


/**********************************************/
/* 	Gyroscope section defines	 	*/
/**********************************************/
#define LSM6DS3_GYR_MIN_POLL_PERIOD_MS	1

#define LSM6DS3_GYR_FS_MASK		(0x0c)
#define LSM6DS3_GYR_FS_245DPS		(0x00)
#define LSM6DS3_GYR_FS_500DPS		(0x04)
#define LSM6DS3_GYR_FS_1000DPS		(0x08)
#define LSM6DS3_GYR_FS_2000DPS		(0x0c)

#define LSM6DS3_GYR_ODR_OFF		(0x00)
#define LSM6DS3_GYR_ODR_MASK		(0xF0)
#define LSM6DS3_GYR_ODR_13		(0x10)
#define LSM6DS3_GYR_ODR_26		(0x20)
#define LSM6DS3_GYR_ODR_52		(0x30)
#define LSM6DS3_GYR_ODR_104		(0x40)
#define LSM6DS3_GYR_ODR_208		(0x50)
#define LSM6DS3_GYR_ODR_416		(0x60)
#define LSM6DS3_GYR_ODR_833		(0x70)
#define LSM6DS3_GYR_ODR_1666		(0x80)

/*
#define LSM6DS3_GYR_BW_0		(0x00)
#define LSM6DS3_GYR_BW_1		(0x01)
#define LSM6DS3_GYR_BW_2		(0x02)
#define LSM6DS3_GYR_BW_3		(0x03)
*/

#define LSM6DS3_GYR_POLL_INTERVAL_DEF	(100)
#define LSM6DS3_ACC_POLL_INTERVAL_DEF	(100)

#define LSM6DS3_STC_MIN_POLL_PERIOD_MS	(100)
#define LSM6DS3_STC_POLL_INTERVAL_DEF	(100)

struct lsm6ds3_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	u8 aa_filter_bandwidth;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct lsm6ds3_gyr_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

/* add by Jay for the step counter */
struct lsm6ds3_step_counter_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
};
/* end by Jay for the step counter */

struct lsm6ds3_main_platform_data {
	int gpio_int1;
	int gpio_int2;
	short rot_matrix[3][3];
	struct lsm6ds3_acc_platform_data *pdata_acc;
	struct lsm6ds3_gyr_platform_data *pdata_gyr;
	/* add by Jay for the step counter */
	struct lsm6ds3_step_counter_platform_data *pdata_step_counter;
	/* end by Jay for the step counter */
#ifdef CONFIG_OF
	struct device_node	*of_node;
#endif
};

#endif	/* __KERNEL__ */
#endif	/* __LSM6DS3_H__ */
