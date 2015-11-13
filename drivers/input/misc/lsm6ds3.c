/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
*
* File Name          : lsm6ds3.c
* Authors            : MSH - C&I BU - Application Team
*                    : Darren Han (darren.han@st.com)
*                    : Ian Yang (ian.yang@st.com)
*                    : Both authors are willing to be considered the contact
*                    : and update points for the driver.
* Version            : V.2.0.0
* Date               : 2014/Feb/11
* Description        : LSM6DS3 driver
*                      1. add step counter
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
//#define DEBUG

/*
Task list:
	1. Add address atuo inc function---finished
*/

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/delay.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include <linux/sensors.h>
#include "lsm6ds3.h"


#define MS_TO_NS(x)			(x*1000000L)
#define GRAVITY_HAL_UNIT       16384
#define GYR_REG_RANG           0x7FFF
#define GYR_RANG               245
#define GYR_HAL_COMPENSTATE    16
#define GYR_CONTRARY           -1
#define ABS(x)		((x) < 0 ? (-x) : (x))
#define GYRO_CAL_REPT (30)


/* TODO: check the following values */
/* Sensitivity */
#define SENSITIVITY_ACC_2G		(61)	/** ug/LSB */
#define SENSITIVITY_ACC_4G		(122)	/** ug/LSB */
#define SENSITIVITY_ACC_8G		(244)	/** ug/LSB */
#define SENSITIVITY_ACC_16G		(488)	/** ug/LSB */

#define SENSITIVITY_GYR_125		(4375)	/** udps/LSB */
#define SENSITIVITY_GYR_245		(8750)	/** udps/LSB */
#define SENSITIVITY_GYR_500		(17500)	/** udps/LSB */
#define SENSITIVITY_GYR_1000		(35000)	/** udps/LSB */
#define SENSITIVITY_GYR_2000		(70000)	/** udps/LSB */

#define ACC_G_MAX_POS			(1495040)/** max positive value acc [ug] */
#define ACC_G_MAX_NEG			(1495770)/** max negative value acc [ug] */
#define GYR_FS_MAX			(32768)

#define FUZZ				(0)
#define FLAT				(0)

#define FILTER_50			(50)/** Anti-Aliasing 50 Hz */
#define FILTER_100			(100)/** Anti-Aliasing 100 Hz */
#define FILTER_200			(200)/** Anti-Aliasing 200 Hz */
#define FILTER_400			(400)/** Anti-Aliasing 400 Hz */

#define RANGE_125DPS			(125)
#define RANGE_245DPS			(245)
#define RANGE_500DPS			(500)
#define RANGE_1000DPS			(1000)
#define RANGE_2000DPS			(2000)

#define ACT_THS				(0x04)
#define ACT_DUR				(0x05)
#define WHO_AM_I			(0x0F)
#define WHO_AM_I_VAL			(0x69)


/* FIFO Control register */
#define FIFO_CTRL_1 			(0x06)
#define FIFO_CTRL_2 			(0x07)
#define FIFO_CTRL_3 			(0x08)
#define FIFO_CTRL_4 			(0x09)
#define FIFO_CTRL_5 			(0x0a)

/* Angular rate sensor sign and orientation register. */
#define ORIENT_CFG_G			(0x0B)
#define ORIENT_CFG_G_SIGN_X_MASK	(0x20)
#define ORIENT_CFG_G_SIGN_Y_MASK	(0x10)
#define ORIENT_CFG_G_SIGN_Z_MASK	(0x08)
#define ORIENT_CFG_G_SIGN_ORIENT_MASK	(0x07)

/* Linear acceleration sensor Control register 1 */
#define CTRL1_XL			(0x10)

/* Angular rate sensor Control Register 1 */
#define CTRL2_G				(0x11)

#define FS_G_SHIFT			(2)
#define FS_G_MASK			(0x0c) 

/* Control Register 3*/
#define CTRL3_C 			(0x12)
#define ADDRESS_AUTOINC_MASK		(0x04)
#define ADDRESS_AUTOINC_EN		(0x04)

/* Control Register 4*/
#define CTRL4_C 			(0x13)
/* Control Register 5*/
#define CTRL5_C 			(0x14)
/* Angular rate sensor Control Register 6 */
#define CTRL6_G 			(0x15)

/* Angular rate sensor Control Register 7 */
#define CTRL7_G 			(0x16)

/* Linear acceleration sensor Control Register 8 */
#define CTRL8_XL			(0x17)

/* Linear acceleration sensor Control Register 9 */
#define CTRL9_XL			(0x18)

/* Control Register 10*/
#define CTRL10_C 			(0x19)

/* Master configuration register */
#define MASTER_CONFIG			(0x1A)

#define TAP_CFG 			(0x58)
#define TAP_THS_6D			(0X59)
#define INT_DUR2			(0X5a)
#define WAKE_UP_THS			(0x5b)
#define WAKE_UP_DUR			(0x5c)
#define FREE_FALL			(0x5d)
#define MD1_CFG				(0x5e)
#define MD2_CFG				(0x5f)

#define SLV0_ADD			(0x02)
#define SLV0_SUBADD			(0x03)
#define SLAVE0_CONFIG			(0x04)
#define SLV1_ADD			(0x05)
#define SLV1_SUBADD			(0x06)
#define SLAVE1_CONFIG			(0x07)
#define SLV2_ADD			(0x08)
#define SLV2_SUBADD			(0x09)
#define SLAVE2_CONFIG			(0x0a)
#define SLV3_ADD			(0x0b)
#define SLV3_SUBADD			(0x0c)
#define SLAVE3_CONFIG			(0x0d)
#define DATAWRITE_SRC_MODE_SUB_SLV0	(0x0e)
#define CONFIG_PEDO_THS_MIN		(0x0f)
#define CONFIG_TILT_IIR			(0x10)
#define CONFIG_TILT_ACOS		(0x11)
#define CONFIG_TILT_WTIME		(0x12)
#define SM_STEP_THS			(0x13)
#define MAG_SI_AA			(0x24)
#define MAG_SI_BB			(0x25)
#define MAG_SI_CC			(0x26)
#define MAG_SI_DD			(0x27)
#define MAG_SI_EE			(0x28)
#define MAG_SI_FF			(0x29)
#define MAG_SI_GG			(0x2A)
#define MAG_SI_HH			(0x2B)
#define MAG_SI_II			(0x2C)
#define MAG_OFFX_L			(0x2D)
#define MAG_OFFX_H			(0x2E)
#define MAG_OFFY_L			(0x2F)
#define MAG_OFFY_H			(0x30)
#define MAG_OFFZ_L			(0x31)
#define MAG_OFFZ_H			(0x32)


#define FUNC_CFG_ACCESS			(0x01)
#define SENSOR_SYNC_TIME_FRAME		(0x04)
#define SENSOR_SYNC_ENABLE		(0x05)
#define FIFO_CTRL1			(0x06)
#define FIFO_CTRL2			(0x07)
#define FIFO_CTRL3			(0x08)
#define FIFO_CTRL4			(0x09)
#define FIFO_CTRL5			(0x0a)
#define INT1_CTRL			(0x0d)
#define INT2_CTRL			(0x0e)

#define OUTX_L_XL			(0x28)
#define OUTX_L_G			(0x22)

#define CTRL9_XL_ALL_AXES_MASK		(0x38)
#define CTRL10_C_ALL_AXES_MASK		(0x38)
#define CTRL9_XL_ALL_AXES_EN		(0x38)
#define CTRL10_C_ALL_AXES_EN		(0x38)

#define DEF_ZERO			(0x00)
#define UNDEF				(0x00)
#define NDTEMP				(1000)	/* Not Available temperature */

#define GET_BIT(reg,mask)		(((reg & mask) == mask) ? 1 : 0)
#define SET_BIT(reg,mask)		(reg | mask)
#define UNSET_BIT(reg,mask)		(reg & (~mask))

#define STEP_COUNTER_EN_MASK 		(0x40)
#define STEP_COUNTER_EN_ENABLE 		(0x40)
#define STEP_COUNTER_INT1_MASK		(0x80)
#define STEP_COUNTER_INT1_ENABLE	(0x80)
#define STEP_COUNTER_FUNC_EN_MASK	(0x04)
#define STEP_COUNTER_FUNC_EN_ENABLE	(0x04)

static struct kobject *acc_kobj;
static struct kobject *gyr_kobj;
static struct kobject *step_counter_kobj;

struct workqueue_struct *lsm6ds3_workqueue = 0;

#define to_dev(obj) container_of(obj, struct device, kobj)

struct output_rate{
	unsigned int cutoff_ms;
	u8 value;
}; 

static const struct output_rate lsm6ds3_gyr_odr_table[] = {
	{  1, (LSM6DS3_GYR_ODR_1666) },
	{  2, (LSM6DS3_GYR_ODR_833)  },
	{  3, (LSM6DS3_GYR_ODR_416)  },
	{  5, (LSM6DS3_GYR_ODR_208) },
	{ 10, (LSM6DS3_GYR_ODR_104) },
	{ 20, (LSM6DS3_GYR_ODR_52) },
	{ 39, (LSM6DS3_GYR_ODR_26) },
	{ 77, (LSM6DS3_GYR_ODR_13) },
};

static const struct output_rate lsm6ds3_acc_odr_table[] = {
        {  1, (LSM6DS3_ACC_ODR_1666) },
        {  2, (LSM6DS3_ACC_ODR_833) },
	{  3, (LSM6DS3_ACC_ODR_416) },
        {  5, (LSM6DS3_ACC_ODR_208) },
        { 10, (LSM6DS3_ACC_ODR_104) },
        { 20, (LSM6DS3_ACC_ODR_52) },
	{ 39, (LSM6DS3_ACC_ODR_26) },		
        { 77, (LSM6DS3_ACC_ODR_13) },
};

static const struct lsm6ds3_acc_platform_data default_lsm6ds3_acc_pdata = {
	.fs_range = LSM6DS3_ACC_FS_2G,
	.poll_interval = LSM6DS3_ACC_POLL_INTERVAL_DEF,
	.min_interval = LSM6DS3_ACC_MIN_POLL_PERIOD_MS,
	.aa_filter_bandwidth = LSM6DS3_ACC_BW_400,
};

static const struct lsm6ds3_gyr_platform_data default_lsm6ds3_gyr_pdata = {
	.fs_range = LSM6DS3_GYR_FS_2000DPS,
	.poll_interval = LSM6DS3_GYR_POLL_INTERVAL_DEF,
	.min_interval = LSM6DS3_GYR_MIN_POLL_PERIOD_MS,
};

/* add by Jay for the step counter */
static const struct lsm6ds3_step_counter_platform_data default_lsm6ds3_step_counter_pdata = {
	.poll_interval = LSM6DS3_STC_POLL_INTERVAL_DEF,
	.min_interval = LSM6DS3_STC_MIN_POLL_PERIOD_MS,
};
/* end by Jay for the step counter */

struct lsm6ds3_main_platform_data default_lsm6ds3_main_platform_data = {
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
	.gpio_int1 = LSM6DS3_INT1_GPIO_DEF,
	.gpio_int2 = LSM6DS3_INT2_GPIO_DEF,
};

struct interrupt_enable {
	atomic_t enable;
	u8 address;
	u8 mask;
};

struct interrupt_value {
	int value;
	u8 address;
};

struct lsm6ds3_status {
	struct i2c_client *client;
	struct lsm6ds3_main_platform_data *pdata_main;
	struct lsm6ds3_acc_platform_data *pdata_acc;
	struct lsm6ds3_gyr_platform_data *pdata_gyr;

	/* add by Jay for the step counter */
	struct lsm6ds3_step_counter_platform_data *pdata_step_counter;
	struct work_struct input_work_step_counter;
	atomic_t enabled_step_counter;
	struct hrtimer hr_timer_step_counter;
	ktime_t ktime_step_counter;
	struct input_dev *input_dev_step_counter;
	/* end by Jay for the step counter */

	struct mutex lock;
	struct mutex i2c_lock;
	struct work_struct input_work_acc;
	struct work_struct input_work_gyr;

	struct hrtimer hr_timer_acc;
	ktime_t ktime_acc;
	struct hrtimer hr_timer_gyr;
	ktime_t ktime_gyr;

	struct input_dev *input_dev_acc;
	struct input_dev *input_dev_gyr;
	//struct input_dev *input_dev_temp;

	int8_t hw_initialized;
	/* hw_working=-1 means not tested yet */
	int8_t hw_working;

	atomic_t enabled_acc;
	atomic_t enabled_gyr;
	atomic_t enabled_temp;

	int32_t temp_value_dec;
	uint32_t temp_value_flo;

	int32_t on_before_suspend;
	int32_t use_smbus;

	uint32_t sensitivity_acc;
	uint32_t sensitivity_gyr;

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;

	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

    //add by zhou at 2015.2.2
    struct sensors_classdev acc_sysdev;
	struct sensors_classdev gyr_sysdev;
	int accel_cali[3];//acc calibration data
	int32_t gyr_cali[3];//gyr calibration data
};


static struct sensors_classdev lsm6ds3_acc_classdev = {
    .name = "lsm6ds3_accelerometer",
	.vendor = "st",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "19.6",
	.resolution = "0.00061",
	.sensor_power = "4",
	.min_delay = 1000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev lsm6ds3_gyr_classdev = {
    .name = "lsm6ds3_gyroscope",
	.vendor = "st",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "34.906586",
	.resolution = "0.0012217305",
	.sensor_power = "4",
	.min_delay = 1000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};



struct reg_rw {
	u8 address;
	u8 default_val;
	u8 resume_val;
};

struct reg_r {
	u8 address;
	u8 default_val;
};

static struct status_registers {
	struct reg_rw func_cfg_access;
	struct reg_rw sensor_sync_time_frame;
	struct reg_rw sensor_sync_enable;
	struct reg_rw fifo_ctrl1;
	struct reg_rw fifo_ctrl2;
	struct reg_rw fifo_ctrl3;
	struct reg_rw fifo_ctrl4;
	struct reg_rw fifo_ctrl5;
	struct reg_rw orient_cfg_g;
	struct reg_rw int1_ctrl;
	struct reg_rw int2_ctrl;
	struct reg_r who_am_i;
	struct reg_rw ctrl1_xl;
	struct reg_rw ctrl2_g;
	struct reg_rw ctrl3_c;
	struct reg_rw ctrl4_c;
	struct reg_rw ctrl5_c;
	struct reg_rw ctrl6_g;
	struct reg_rw ctrl7_g;
	struct reg_rw ctrl8_xl;
	struct reg_rw ctrl9_xl;
	struct reg_rw ctrl10_c;
	struct reg_rw master_config;
	struct reg_r wake_up_src;
	struct reg_r tap_src;
	struct reg_r d6d_src;
	struct reg_r status_reg;
	struct reg_r fifo_status1;
	struct reg_r fifo_status2;
	struct reg_r fifo_status3;
	struct reg_r fifo_status4;
	struct reg_r step_counter_l;
	struct reg_r step_counter_h;
	struct reg_r func_src;
	struct reg_rw tap_cfg;
	struct reg_rw tap_ths_6d;
	struct reg_rw int_dur2;
	struct reg_rw wake_up_ths;
	struct reg_rw wake_up_dur;
	struct reg_rw free_fall;
	struct reg_rw md1_cfg;
	struct reg_rw md2_cfg;
	
	struct reg_rw slv0_add;
	struct reg_rw slv0_subadd;
	struct reg_rw slave0_config;
	struct reg_rw slv1_add;
	struct reg_rw slv1_subadd;
	struct reg_rw slave1_config;
	struct reg_rw slv2_add;
	struct reg_rw slv2_subadd;
	struct reg_rw slave2_config;
	struct reg_rw slv3_add;
	struct reg_rw slv3_subadd;
	struct reg_rw slave3_config;
	struct reg_rw datawrite_src_mode_sub_slv0;
	struct reg_rw config_pedo_ths_min;
	struct reg_rw config_tilt_iir;
	struct reg_rw config_tilt_acos;
	struct reg_rw config_tilt_wtime;
	struct reg_rw sm_step_ths;
	struct reg_rw mag_si_aa;
	struct reg_rw mag_si_bb;
	struct reg_rw mag_si_cc;
	struct reg_rw mag_si_dd;
	struct reg_rw mag_si_ee;
	struct reg_rw mag_si_ff;
	struct reg_rw mag_si_gg;
	struct reg_rw mag_si_hh;
	struct reg_rw mag_si_ii;
	struct reg_rw mag_offx_l;
	struct reg_rw mag_offx_h;
	struct reg_rw mag_offy_l;
	struct reg_rw mag_offy_h;
	struct reg_rw mag_offz_l;
	struct reg_rw mag_offz_h;
} status_registers = {
	.func_cfg_access =
		{.address = FUNC_CFG_ACCESS, 	.default_val = DEF_ZERO,},
	.sensor_sync_time_frame =
		{.address = SENSOR_SYNC_TIME_FRAME, .default_val = DEF_ZERO,},
	.sensor_sync_enable =
		{.address = SENSOR_SYNC_ENABLE, .default_val = DEF_ZERO,},
	.fifo_ctrl1 =
		{.address = FIFO_CTRL1, 	.default_val = DEF_ZERO,},
	.fifo_ctrl2 =
		{.address = FIFO_CTRL2, 	.default_val = DEF_ZERO,},
	.fifo_ctrl3 =
		{.address = FIFO_CTRL3, 	.default_val = DEF_ZERO,},
	.fifo_ctrl4 = 
		{.address = FIFO_CTRL4, 	.default_val = DEF_ZERO,},
	.fifo_ctrl5 =
		{.address = FIFO_CTRL5, 	.default_val = DEF_ZERO,},
	.orient_cfg_g =
		{.address = ORIENT_CFG_G,	.default_val = DEF_ZERO,},
	.int1_ctrl =
		{.address = INT1_CTRL,		.default_val = DEF_ZERO,},
	.int2_ctrl =
		{.address = INT2_CTRL,		.default_val = DEF_ZERO,},
	.who_am_i =
		{.address = WHO_AM_I,		.default_val = WHO_AM_I_VAL,},
	.ctrl1_xl =
		{.address = CTRL1_XL,	.default_val = DEF_ZERO,},
	.ctrl2_g =
		{.address = CTRL2_G,	.default_val = DEF_ZERO,},
	.ctrl3_c =
		{.address = CTRL3_C,	.default_val = ADDRESS_AUTOINC_EN,},
	.ctrl4_c =
		{.address = CTRL4_C,	.default_val = DEF_ZERO,},
	.ctrl5_c =
		{.address = CTRL5_C,	.default_val = DEF_ZERO,},
	.ctrl6_g =
		{.address = CTRL6_G,	.default_val = DEF_ZERO,},
	.ctrl7_g =
		{.address = CTRL7_G,	.default_val = DEF_ZERO,},
	.ctrl8_xl =
		{.address = CTRL8_XL,	.default_val = DEF_ZERO,},
	.ctrl9_xl =
		{.address = CTRL9_XL,	.default_val = CTRL9_XL_ALL_AXES_EN,},
	.ctrl10_c =
		{.address = CTRL10_C,	.default_val = CTRL10_C_ALL_AXES_EN,},
	.master_config =
		{.address = MASTER_CONFIG, .default_val = DEF_ZERO,},
	.tap_cfg =
		{.address = TAP_CFG,	.default_val = DEF_ZERO,},
	.tap_ths_6d =
		{.address = TAP_THS_6D,	.default_val = DEF_ZERO,},
	.int_dur2 =
		{.address = INT_DUR2,	.default_val = DEF_ZERO,},
	.wake_up_ths =
		{.address = WAKE_UP_THS, .default_val = DEF_ZERO,},
	.wake_up_dur =
		{.address = WAKE_UP_DUR, .default_val = DEF_ZERO,},
	.free_fall =
		{.address = FREE_FALL,	.default_val = DEF_ZERO,},
	.md1_cfg =
		{.address = MD1_CFG,	.default_val = DEF_ZERO,},
	.md2_cfg =
		{.address = MD2_CFG,	.default_val = DEF_ZERO,},
	.slv0_add =
		{.address = SLV0_ADD,	.default_val = DEF_ZERO,},
	.slv0_subadd =
		{.address = SLV0_SUBADD,	.default_val = DEF_ZERO,},
	.slave0_config =
		{.address = SLAVE0_CONFIG,	.default_val = DEF_ZERO,},
	.slv1_add =
		{.address = SLV1_ADD,	.default_val = DEF_ZERO,},
	.slv1_subadd =
		{.address = SLV1_SUBADD,	.default_val = DEF_ZERO,},
	.slave1_config =
		{.address = SLAVE1_CONFIG,	.default_val = DEF_ZERO,},
	.slv2_add =
		{.address = SLV2_ADD,	.default_val = DEF_ZERO,},
	.slv2_subadd =
		{.address = SLV2_SUBADD,	.default_val = DEF_ZERO,},
	.slave2_config =
		{.address = SLAVE2_CONFIG,	.default_val = DEF_ZERO,},
	.slv3_add =
		{.address = SLV3_ADD,	.default_val = DEF_ZERO,},
	.slv3_subadd =
		{.address = SLV3_SUBADD,	.default_val = DEF_ZERO,},
	.slave3_config =
		{.address = SLAVE3_CONFIG,	.default_val = DEF_ZERO,},
	.datawrite_src_mode_sub_slv0 =
		{.address = DATAWRITE_SRC_MODE_SUB_SLV0,	.default_val = DEF_ZERO,},
	.config_pedo_ths_min =
		{.address = CONFIG_PEDO_THS_MIN,	.default_val = 0x11,},
	.config_tilt_iir =
		{.address = CONFIG_TILT_IIR,	.default_val = 0x7a,},
	.config_tilt_acos =
		{.address = CONFIG_TILT_ACOS,	.default_val = 0x69,},
	.config_tilt_wtime =
		{.address = CONFIG_TILT_WTIME,	.default_val = 0x62,},
	.sm_step_ths =
		{.address = SM_STEP_THS,	.default_val = 0x06,},
	.mag_si_aa =
		{.address = MAG_SI_AA,	.default_val = DEF_ZERO,},
	.mag_si_bb =
		{.address = MAG_SI_BB,	.default_val = DEF_ZERO,},
	.mag_si_cc =
		{.address = MAG_SI_CC,	.default_val = DEF_ZERO,},
	.mag_si_dd =
		{.address = MAG_SI_DD,	.default_val = DEF_ZERO,},
	.mag_si_ee =
		{.address = MAG_SI_EE,	.default_val = DEF_ZERO,},
	.mag_si_ff =
		{.address = MAG_SI_FF,	.default_val = DEF_ZERO,},
	.mag_si_gg =
		{.address = MAG_SI_GG,	.default_val = DEF_ZERO,},
	.mag_si_hh =
		{.address = MAG_SI_HH,	.default_val = DEF_ZERO,},
	.mag_si_ii =
		{.address = MAG_SI_II,	.default_val = DEF_ZERO,},
	.mag_offx_l =
		{.address = MAG_OFFX_L,	.default_val = DEF_ZERO,},
	.mag_offx_h =
		{.address = MAG_OFFX_H,	.default_val = DEF_ZERO,},	
	.mag_offy_l =
		{.address = MAG_OFFY_L,	.default_val = DEF_ZERO,},
	.mag_offy_h =
		{.address = MAG_OFFY_H,	.default_val = DEF_ZERO,},	
	.mag_offz_l =
		{.address = MAG_OFFZ_L,	.default_val = DEF_ZERO,},
	.mag_offz_h =
		{.address = MAG_OFFZ_H,	.default_val = DEF_ZERO,},	
};

static u8 ram_code[] = {
	0x00, 0x00, 0x04, 0x00, 0x1F, 0x29, 0xB0, 0xFF, 0x2B, 0x58, 0x4D, 0x04, 0x34,
	0x83, 0xD5, 0x2C, 0x01, 0x2F, 0x1D, 0x40, 0xBD, 0x05, 0x04, 0x20, 0x48, 0x11,
	0x50, 0x23, 0x00, 0xB3, 0x05, 0x2C, 0xF7, 0xA0, 0x6C, 0x01, 0xDB, 0x2A, 0x30,
	0x15, 0x10, 0x03, 0xC0, 0x50, 0x01, 0x0B, 0x26, 0x30, 0x19, 0x00, 0x6F, 0x03,
	0x2C, 0x1C, 0xF0, 0x00, 0xD0, 0x72, 0x40, 0x80, 0x80, 0xE4, 0x41, 0x35, 0x3F,
	0xAF, 0x00, 0x80, 0x8A, 0x05, 0x04, 0x08, 0x48, 0x26, 0x50, 0x0B, 0x00, 0x83,
	0x05, 0x2C, 0xFD, 0xA0, 0x60, 0x01, 0xFB, 0xF3, 0x0A, 0xE0, 0x4B, 0x36, 0x54,
	0x0F, 0x29, 0x51, 0xB4, 0x04, 0x47, 0x4C, 0xF0, 0x2A, 0xDB, 0xBB, 0xAC, 0x2F,
	0xB3, 0xE8, 0x01, 0x0F, 0x29, 0x51, 0xB4, 0x04, 0x47, 0x4C, 0xFC, 0x26, 0x01,
	0xF8, 0x0C, 0x40, 0x00, 0xD6, 0x40, 0x43, 0x0A, 0xF1, 0x27, 0x0A, 0x80, 0x03,
	0x80, 0x4C, 0x17, 0x6D, 0x04, 0xC8, 0x05, 0x80, 0x08, 0x3C, 0x09, 0x0C, 0x06,
	0x01, 0x70, 0x11, 0x0C, 0x19, 0x01, 0x73, 0x01, 0x20, 0x02, 0x4F, 0x02, 0x83,
	0xC1, 0x68, 0x00, 0x20, 0x00, 0x01, 0x1A, 0x00, 0x08, 0x10, 0x1A, 0x84, 0x06,
	0xD2, 0x46, 0x80, 0x5C, 0x00, 0x88, 0x00, 0x4F, 0x02, 0x83, 0x41, 0x00, 0x17,
	0x41, 0x91, 0x11, 0x30, 0x17, 0x00, 0x22, 0xF0, 0x24, 0x30, 0x60, 0x30, 0x1A,
	0x00, 0x08, 0x10, 0xA0, 0x01, 0x80, 0x00, 0xA1, 0x41, 0x6C, 0x30, 0x6D, 0x04,
	0x20, 0x17, 0x00, 0x22, 0xF0, 0x24, 0x30, 0x18, 0x04, 0x70, 0x11, 0x1C, 0x19,
	0x01, 0x73, 0x01, 0x80, 0x08, 0x3C, 0x09, 0x0C, 0x06, 0xA3, 0x01, 0x80, 0x00,
	0x01, 0x1A, 0x00, 0x08, 0x10, 0x1A, 0x10, 0x1C, 0xF0, 0x2A, 0xDB, 0xBB, 0xAC,
	0x2F, 0xB3, 0xE8, 0x01, 0xF3, 0x2A, 0xD1, 0xBB, 0x04, 0xBE, 0x4C, 0xFC, 0x26,
	0x41, 0x9B, 0x94, 0x1D, 0xD6, 0xD8, 0x41, 0x58, 0x03, 0xED, 0x92, 0x14, 0x7A,
	0x5C, 0x63, 0x07, 0x61, 0x0D, 0x34, 0xD0, 0x4B, 0x81, 0x54, 0x63, 0x07, 0x91,
	0x11, 0x14, 0xD0, 0x5C, 0x07, 0x04, 0x66, 0x0D, 0x34, 0xB5, 0x11, 0xA0, 0x17,
	0x00, 0x22, 0xF0, 0x24, 0x30, 0x60, 0x10, 0xC0, 0x45, 0x50, 0x6D, 0x04, 0xEC,
	0x05, 0x80, 0x08, 0x3C, 0x09, 0x0C, 0x06, 0x23, 0xD6, 0x46, 0x00, 0x5F, 0x00,
	0x88, 0xC0, 0x93, 0xC0, 0x60, 0x30, 0x02, 0x86, 0x54, 0x09, 0x35, 0x00, 0x00,
	0x70, 0x02, 0x6D, 0x00, 0x20, 0x40, 0x80, 0x06, 0x00, 0x02, 0x04, 0x68, 0x00,
	0x20, 0x00, 0x01, 0x42, 0x40, 0x70, 0x2D, 0x04, 0xA4, 0xFC, 0x06, 0x02, 0x02,
	0x0C, 0x68, 0x20, 0x20, 0x00, 0x03, 0x1A, 0x08, 0x08, 0x30, 0x20, 0x00, 0x02,
	0x60, 0x04, 0x10, 0x20, 0xB5, 0x0B, 0x35, 0x40, 0x16, 0x00, 0xD1, 0x6F, 0x40,
	0x20, 0x40, 0x69, 0x50, 0x14, 0xC0, 0xD0, 0x6F, 0x40, 0x20, 0x00, 0x05, 0x1A,
	0x10, 0x08, 0x50, 0xA0, 0x01, 0x81, 0x00, 0xA5, 0x41, 0x51, 0x00, 0x50, 0x00,
	0x02, 0xE6, 0x03, 0xD9, 0x09, 0x46, 0x41, 0x00, 0x62, 0xA6, 0x40, 0x03, 0xC1,
	0x6F, 0x00, 0x20, 0x00, 0x01, 0x42, 0x40, 0x80, 0x1D, 0x44, 0xA4, 0x10, 0x7F,
	0xA2, 0x48, 0x0E, 0x01, 0x02, 0x15, 0xC9, 0x5C, 0xC3, 0x44, 0xB5, 0xCC, 0x75,
	0x40, 0x60, 0xD6, 0x40, 0x03, 0x04, 0x60, 0x0D, 0x34, 0x54, 0x43, 0x80, 0x0C,
	0x43, 0x35, 0xEC, 0xF7, 0x28, 0x54, 0xC3, 0x02, 0x04, 0x00, 0xF4, 0x0C, 0xD4,
	0xCA, 0x56, 0x2F, 0x6B, 0xCD, 0xEC, 0x75, 0xB2, 0xD8, 0xC1, 0x46, 0x2B, 0x2B,
	0xBD, 0x2C, 0xD3, 0xCC, 0x02, 0x80, 0x2A, 0x07, 0x04, 0x10, 0x48, 0xE1, 0x54,
	0x67, 0x00, 0x11, 0x00, 0x12, 0x00, 0xB0, 0x6E, 0x01, 0x01, 0x04, 0x12, 0x3C,
	0xD4, 0x04, 0xC0, 0x6C, 0x01, 0xBB, 0x0F, 0x0A, 0x5B, 0xC0, 0x02, 0x00, 0xD0,
	0x0C, 0x2C, 0x06, 0x00, 0xCE, 0xC0, 0x42, 0x00, 0x90, 0x0C, 0x2C, 0xFF, 0x00,
	0x08, 0x03, 0xBB, 0x05, 0x04, 0x08, 0x48, 0xFC, 0x50, 0x0B, 0x00, 0xB3, 0x05,
	0x2C, 0xFD, 0xA0, 0x6C, 0x01, 0x0B, 0x00, 0xC0, 0x12, 0xB0, 0x4C, 0xC0, 0x6A,
	0x03, 0xBB, 0x0D, 0x2C, 0x43, 0x4A, 0x14, 0x2D, 0x71, 0xC4, 0xC4, 0x23, 0x10,
	0x80, 0x20, 0x15, 0x44, 0x4D, 0xA4, 0x04, 0x46, 0x4B, 0x20, 0x31, 0xC1, 0xAB,
	0x6C, 0xEF, 0xB2, 0xBE, 0xCC, 0xA2, 0x07, 0xFC, 0x0B, 0x04, 0xC1, 0xC0, 0x0A,
	0x07, 0xF1, 0x1F, 0x14, 0x44, 0xD5, 0xC2, 0xC0, 0x02, 0x80, 0xFA, 0x2F, 0x00,
	0xC2, 0xC2, 0x42, 0x40, 0x86, 0x00, 0xE0, 0x23, 0x11, 0x40, 0x24, 0xA9, 0x54,
	0x0D, 0x0C, 0x3E, 0x10, 0x10, 0x0C, 0x07, 0x61, 0x2C, 0xC4, 0x30, 0xB0, 0x20,
	0xD0, 0x21, 0x00, 0x48, 0x2C, 0x04, 0xC5, 0x43, 0x10, 0x07, 0x5B, 0x2C, 0xAC,
	0xF1, 0xB0, 0x30, 0xD1, 0x21, 0x04, 0x08, 0x31, 0x01, 0x0A, 0x03, 0xC2, 0x44,
	0x06, 0x01, 0xD9, 0x52, 0xD0, 0xC7, 0x41, 0x28, 0x00, 0x08, 0x30, 0x01, 0x10,
	0x90, 0x01, 0x44, 0x00, 0x20, 0x81, 0x82, 0x80, 0x20, 0x15, 0x28, 0x08, 0x08,
	0x12, 0x1D, 0xC8, 0x42, 0x80, 0x44, 0x76, 0x1C, 0x2C, 0x40, 0x75, 0x30, 0xD5,
	0x21, 0x17, 0x3B, 0x1C, 0x04, 0xC4, 0x42, 0x14, 0x0F, 0xA1, 0x00, 0xE0, 0x30,
	0xB0, 0xC4, 0xC1, 0x16, 0x0B, 0xAB, 0x20, 0x20, 0x0A, 0x02, 0xC2, 0x44, 0xA6,
	0x10, 0x20, 0x84, 0x74, 0x0A, 0x02, 0x1A, 0x07, 0x61, 0x3C, 0x2C, 0x10, 0x90,
	0x81, 0x40, 0xA6, 0x1C, 0xC4, 0xB2, 0x10, 0xCA, 0xC0, 0x2E, 0x07, 0x9B, 0xDC,
	0x04, 0xF0, 0x2D, 0x55, 0x45, 0x0D, 0xD1, 0x58, 0x72, 0xB3, 0x00, 0xA0, 0x3A,
	0x0F, 0xF1, 0x48, 0x04, 0x1F, 0xA4, 0x38, 0x0C, 0xE8, 0x30, 0x20, 0x84, 0x07,
	0xBD, 0x01, 0xC3, 0x78, 0x05, 0xCD, 0x04, 0xCE, 0x43, 0xB0, 0x75, 0x0D, 0x1C,
	0x01, 0x7E, 0x28, 0x08, 0x8C, 0x51, 0x0A, 0x81, 0x2F, 0x0A, 0x20, 0x51, 0xB5,
	0x66, 0x0D, 0x18, 0x00, 0x73, 0xB0, 0x00, 0xAC, 0xEC, 0xB7, 0xD2, 0x1A, 0x34,
	0x07, 0xAC, 0x00, 0x82, 0x03, 0x18, 0x0E, 0xA0, 0x38, 0x10, 0x50, 0xD5, 0x75,
	0x0D, 0x12, 0x15, 0x77, 0x5D, 0xEB, 0x06, 0x0D, 0x12, 0x95, 0x6E, 0xD7, 0xCC,
	0x4E, 0x00, 0x3A, 0xC5, 0x57, 0x35, 0xCC, 0xC1, 0xEA, 0x06, 0x0D, 0xE1, 0x1D,
	0x60, 0xD7, 0xFF, 0xEE, 0x04, 0x38, 0x06, 0xE3, 0x15, 0xAD, 0x59, 0x23, 0x34,
	0x43, 0x09, 0xC4, 0x01, 0x28, 0x00, 0x5C, 0x25, 0x46, 0x1D, 0xC0, 0x18, 0xA8,
	0x41, 0x53, 0x15, 0xB1, 0x14, 0x04, 0x93, 0x10, 0x04, 0x2D, 0x45, 0x56, 0x0D,
	0x10, 0x19, 0x00, 0xA2, 0x11, 0x34, 0x13, 0x10, 0x18, 0x80, 0x68, 0x95, 0x5D,
	0x23, 0x14, 0xB3, 0x14, 0x2C, 0x4C, 0xC2, 0x06, 0x05, 0x21, 0x24, 0x04, 0x04,
	0x04, 0xCF, 0x42, 0x80, 0x44, 0x57, 0x29, 0x04, 0x20, 0x51, 0x91, 0x66, 0x8D,
	0xD0, 0x12, 0x69, 0xD4, 0xF7, 0xAD, 0x00, 0x16, 0xF3, 0x0C, 0x2C, 0x10, 0xC5,
	0x50, 0x15, 0x7B, 0x56, 0x0A, 0x55, 0xB1, 0x80, 0x00, 0x30, 0x03, 0x0B, 0x4E,
	0x01, 0x20, 0x0E, 0xB8, 0x06, 0x0D, 0xE1, 0x40, 0xB3, 0x10, 0x10, 0xE4, 0x03,
	0xC9, 0xD8, 0x0C, 0x04, 0xE0, 0xF0, 0x3B, 0x13, 0x01, 0x4E, 0x3F, 0x04, 0x65,
	0x00, 0xA5, 0x01, 0xC9, 0xD9, 0x2C, 0x2C, 0xCE, 0xC3, 0x42, 0x33, 0x9B, 0xDC,
	0x2C, 0x00, 0xA8, 0x80, 0x2E, 0x01, 0xC0, 0xFA, 0xEF, 0x0E, 0x01, 0x8E, 0x01,
	0x80, 0x9A, 0x01, 0x44, 0x00, 0x48, 0xCA, 0x55, 0x2F, 0x01, 0xA1, 0x0D, 0x2C,
	0x4C, 0x40, 0x6C, 0x03, 0x0B, 0x00, 0x2A, 0x16, 0x10, 0x10, 0x20, 0x59, 0x47,
	0x4D, 0x00, 0x0C, 0x58, 0xC0, 0xEE, 0x83, 0x82, 0x05, 0xEC, 0x12, 0x10, 0xDA,
	0xC0, 0x32, 0x01, 0xB1, 0x0D, 0x2C, 0x00, 0xA0, 0x4E, 0x12, 0xB1, 0x04, 0x04,
	0x53, 0x10, 0xDA, 0x42, 0x6C, 0x0F, 0x01, 0x02, 0x27, 0x40, 0x50, 0x01, 0xE0,
	0x4A, 0x35, 0x04, 0xD0, 0x30, 0x54, 0xC3, 0xFE, 0x8E, 0x42, 0x35, 0x2C, 0x4B,
	0x40, 0x30, 0x05, 0xA1, 0x0D, 0xEC, 0x76, 0xB0, 0x00, 0xA0, 0xF2, 0x02, 0xD1,
	0x1B, 0x04, 0x0A, 0x00, 0x2A, 0x04, 0x08, 0xE0, 0x40, 0x80, 0x00, 0x80, 0x41,
	0xCD, 0x67, 0xFD, 0x1F, 0x0E, 0x01, 0x81, 0xD5, 0x07, 0xFD, 0x0F, 0x4E, 0x00,
	0x60, 0x08, 0x42, 0x21, 0x48, 0x0D, 0x40, 0x01, 0x10, 0x16, 0x04, 0x08, 0xD5,
	0x1F, 0x35, 0x8D, 0xC0, 0xFF, 0x41, 0x03, 0x54, 0x00, 0x71, 0x01, 0x0C, 0x02,
	0x02, 0x02, 0x55, 0xE0, 0x35, 0x54, 0x64, 0x70, 0xD0, 0x1D, 0x08, 0x0D, 0x15,
	0x1D, 0x70, 0x90, 0xD5, 0x07, 0x0D, 0x41, 0x01, 0x40, 0x01, 0x06, 0x04, 0x02,
	0x14, 0x08, 0x64, 0x01, 0x50, 0x17, 0x18, 0x10, 0x08, 0x50, 0x20, 0x10, 0x06,
	0x50, 0x19, 0x18, 0x10, 0x08, 0x50, 0x20, 0x06, 0x04, 0x02, 0x14, 0x68, 0x40,
	0x20, 0x40, 0x81, 0x80, 0x94, 0x41, 0xD6, 0x06, 0x46, 0x19, 0x70, 0xB5, 0x41,
	0x50, 0x06, 0x50, 0x1A, 0x04, 0x61, 0x00, 0xA5, 0x21, 0x20, 0x50, 0x70, 0x00,
	0x00, 0x47, 0xA1, 0x48, 0xED, 0x50, 0x20, 0x02, 0x81, 0x01, 0xC7, 0x8D, 0x08,
	0x1D, 0xE0, 0x0A, 0x31, 0x56, 0x03, 0x45, 0xE0, 0x0B, 0x84, 0x02, 0x80, 0x0E,
	0x01, 0x06, 0x38, 0x10, 0x20, 0x00, 0xF5, 0x41, 0xFF, 0x12, 0xEB, 0xEF, 0x0A,
	0x00, 0xA8, 0x20, 0xF6, 0x1B, 0x10, 0xA8, 0xA3, 0x35, 0x00, 0x14, 0x80, 0xD0,
	0x2F, 0x24, 0x34, 0x00, 0x05, 0xFF, 0xE4, 0x04, 0x10, 0x06, 0x42, 0x3F, 0xFF,
	0xE4, 0xFC, 0x97, 0x13, 0x40, 0x18, 0x40, 0x69, 0x00, 0xA0, 0x1A, 0x18, 0x78,
	0xA4, 0x35, 0x70, 0xF4, 0x03, 0x80, 0xFA, 0x4F, 0x4E, 0x00, 0x61, 0x70, 0xF4,
	0xFF, 0x93, 0xF3, 0x5F, 0x0E, 0x01, 0x84, 0x01, 0x94, 0x06, 0x00, 0x2A, 0x1C,
	0x2E, 0x53, 0x52, 0xFF, 0x9F, 0x13, 0x70, 0x18, 0x80, 0x26, 0x61, 0x49, 0xFD,
	0x6F, 0x4E, 0x80, 0x61, 0x80, 0xE8, 0xC0, 0xD9, 0x0F, 0x88, 0x12, 0x5F, 0x52,
	0xFF, 0x9B, 0xF3, 0x7F, 0x4E, 0x80, 0x61, 0x00, 0xA7, 0x01, 0x80, 0x0A, 0x01,
	0x04, 0x0C, 0xA0, 0xA0, 0x04, 0x30, 0x54, 0x40, 0x4B, 0x50, 0x00, 0xBC, 0x42,
	0x48, 0x11, 0x10, 0x18, 0x10, 0x53, 0xC0, 0x50, 0x10, 0x27, 0x30, 0xD4, 0xB4,
	0x01, 0x85, 0x41, 0x51, 0x1B, 0x10, 0x18, 0x01, 0x85, 0x91, 0x09, 0x0D, 0x00,
	0x54, 0x9D, 0xD5, 0x0A, 0x04, 0x02, 0x80, 0x5A, 0x28, 0x30, 0x06, 0x02, 0x02,
	0x0C, 0x88, 0x00, 0x14, 0x00, 0xAC, 0x30, 0x14, 0x14, 0x0A, 0x6C, 0x20, 0x20,
	0x00, 0x03, 0x32, 0x00, 0x05, 0x00, 0x2B, 0x0C, 0x05, 0x85, 0x02, 0x1B, 0x08,
	0x08, 0x30, 0x20, 0x00, 0xA0, 0x02, 0x0D, 0x05, 0x80, 0x2B, 0xE0, 0x50, 0x8C,
	0x5E, 0xFF, 0x0B, 0xF0, 0x33, 0x00, 0x00, 0xA0, 0x02, 0x0F, 0x05, 0xE9, 0x35,
	0x80, 0x00, 0xC0, 0x03, 0x00, 0x80, 0x0A, 0x00, 0x00, 0x82, 0x01, 0x00, 0x07,
	0x85, 0x29, 0x35, 0x04, 0xB4, 0x01, 0x81, 0x4D, 0x0A, 0x3D, 0x14, 0x00, 0x5A,
	0x41, 0x79, 0x4A, 0x0D, 0x01, 0x6D, 0x40, 0x60, 0x99, 0x42, 0x6F, 0x01, 0xFB,
	0x00, 0x00, 0x8F, 0xC0, 0x16, 0x02, 0x40, 0x09, 0xAC, 0x1E, 0x00, 0x90, 0xC0,
	0xA6, 0x01, 0x10, 0x09, 0x2C, 0x32, 0x00, 0x48, 0x02, 0x6B, 0x00, 0xC0, 0x24,
	0xB0, 0xAD, 0x02, 0x03, 0x80, 0x0A, 0x00, 0x00, 0xC0, 0x01, 0x3C, 0x04, 0x45,
	0x2B, 0x35, 0x04, 0xB4, 0x01, 0x81, 0xBD, 0x0A, 0x0D, 0x08, 0x00, 0xCC, 0xC0,
	0x02, 0x00, 0xD0, 0x0C, 0x2C, 0x06, 0x00, 0xCE, 0xC0, 0x42, 0x00, 0x90, 0x0C,
	0x2C, 0xFF, 0x00, 0x08, 0x03, 0xAB, 0x07, 0x80, 0x3F, 0xB0, 0x3D, 0x00, 0xFC,
	0x03, 0x0B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x30, 0x00, 0x00, 0x01,
	0x00, 0x05, 0x00, 0x18, 0x00, 0x70, 0x00, 0x00, 0x08, 0x00, 0x24, 0x00, 0xA0,
	0x00, 0xC0, 0x02, 0x00, 0x0C, 0x00, 0x34, 0x00, 0xE0, 0x00, 0x00, 0x0F, 0x00,
	0x80, 0x1A, 0xF0, 0x8C, 0x7F, 0x2B, 0x58, 0xCD, 0x06, 0x00, 0x0E, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x30, 0x2D, 0x34
};
/*****************************************************************************/
static int lsm6ds3_acc_update_odr(struct lsm6ds3_status *stat,
						unsigned int poll_interval_ms);
static int lsm6ds3_gyr_update_odr(struct lsm6ds3_status *stat,
						unsigned int poll_interval_ms);


static int lsm6ds3_i2c_write(struct lsm6ds3_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg, value;
#ifdef DEBUG
	unsigned int ii;
#endif
	mutex_lock(&stat->i2c_lock);
	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			mutex_unlock(&stat->i2c_lock);
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			mutex_unlock(&stat->i2c_lock);
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len + 1);
	mutex_unlock(&stat->i2c_lock);
	return (ret == len + 1) ? 0 : ret;
}

static int lsm6ds3_i2c_read(struct lsm6ds3_status *stat, u8 *buf,
								  int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;
#ifdef DEBUG
	unsigned int ii;
#endif
	mutex_lock(&stat->i2c_lock);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			mutex_unlock(&stat->i2c_lock);
			return 0;
		}
		mutex_unlock(&stat->i2c_lock);
		return len;
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd)){
		mutex_unlock(&stat->i2c_lock);
		return ret;
	}

	ret = i2c_master_recv(stat->client, buf, len);
	mutex_unlock(&stat->i2c_lock);
	return ret;
}

static int lsm6ds3_write_with_mask(struct lsm6ds3_status *stat, u8 reg, u8 mask, u8 value)
{
	int err = -1;
	u8 buf[8];

	/* read current value */
	buf[0] = reg;
	err = lsm6ds3_i2c_read(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "%s: step counter write with mask "
							"failed: %d\n", __func__, err);
	buf[0] &= ~mask;
	buf[0] |= mask & value;

	buf[1] = buf[0];
	buf[0] = reg;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter write access "
							"failed: %d\n", err);
	return err;
}

static int lsm6ds3_set_config_access(struct lsm6ds3_status *stat, u8 mask, u8 value)
{
	int err = -1;
	u8 buf[8];

	/* read current value */
	buf[0] = status_registers.func_cfg_access.address;
	err = lsm6ds3_i2c_read(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter read access "
							"failed: %d\n", err);
	buf[0] &= ~mask;
	buf[0] |= mask & value;

	buf[1] = buf[0];
	buf[0] = status_registers.func_cfg_access.address;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter write access "
							"failed: %d\n", err);
	return err;
}

/* Set the RAM address */
static inline void lsm6ds3_set_ram_addr(struct lsm6ds3_status *stat, u16 addr)
{
	int err = -1;
	u8 buf[8];

	buf[0] = 0x62; // low address
	buf[1] = addr & 0xFF;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "set ram low address "
							"failed: %d\n", err);

	buf[0] = 0x63;  // high address
	buf[1] = (addr >> 8) & 0xFF;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "set ram high address "
							"failed: %d\n", err);
}

/* write program to ram */
static void lsm6ds3_program_ram(struct lsm6ds3_status *stat, u8 *ram, int len)
{
	int i = 0;
	int err = -1;
	u8 buf[8];

	/* Enable RAM Access */
	lsm6ds3_set_config_access(stat, 0x01, 0x01);

	for(i=0; i<len; i+=4) {
		/* Set RAM Address */
		lsm6ds3_set_ram_addr(stat, i);

		/* Write program to RAM (multiple writes) */
		buf[0] = 0x64;
		memcpy(buf+1, ram+i, 4);
		err = lsm6ds3_i2c_write(stat, buf, 4);
		if (err < 0)
			dev_err(&stat->client->dev, "step counter program ram code "
								"failed: %d\n", err);
	}

#if 1
	/* Read back ram[4]~ram[7] */
	lsm6ds3_set_ram_addr(stat, 4);
	buf[0] = 0x64;
	err = lsm6ds3_i2c_read(stat, buf, 4);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter read ram code[4-7] "
								"failed: %d\n", err);
	/* Verify */
	if ((buf[0] != ram[4]) || (buf[1] != ram[5]) || (buf[2] != ram[6]) || (buf[3] != ram[7])) {
		dev_err(&stat->client->dev, "step counter verify ram code[4-7] "
								"failed: %d\n", err);
	}

	/* Read back ram[2044]~ram[2047] */
	lsm6ds3_set_ram_addr(stat, 2044);
	buf[0] = 0x64;
	err = lsm6ds3_i2c_read(stat, buf, 4);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter read ram code[2044-2047] "
								"failed: %d\n", err);
	/* Verify */
	if ((buf[0] != ram[2044]) || (buf[1] != ram[2045]) || (buf[2] != ram[2046]) || (buf[3] != ram[2047])) {
		dev_err(&stat->client->dev, "step counter verify ram code[2044-2047] "
								"failed: %d\n", err);
	}
#endif
	/* Disable RAM Access */
	lsm6ds3_set_config_access(stat, 0x01, 0x00);
}

/* Init RAM */
static void lsm6ds3_init_ram(struct lsm6ds3_status *stat, u8 *ram, int len)
{
	/* Force device to exec from ROM */
	lsm6ds3_set_config_access(stat, 0x04, 0x00);

	/* program the RAM with code */
	lsm6ds3_program_ram(stat, ram, len);

	/* Enable device to exec from RAM */
	lsm6ds3_set_config_access(stat, 0x04, 0x04);
}

static int lsm6ds3_acc_device_power_off(struct lsm6ds3_status *stat)
{
	int err;

	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl1_xl.address, LSM6DS3_ACC_ODR_MASK, LSM6DS3_ACC_ODR_OFF);
	if (err < 0)
		dev_err(&stat->client->dev, "accelerometer soft power off "
							"failed: %d\n", err);

	if (stat->pdata_acc->power_off) {
		stat->pdata_acc->power_off();
	}

	atomic_set(&stat->enabled_acc, 0);
	dev_info(&stat->client->dev, "accelerometer switched off.");

	return 0;
}

static int lsm6ds3_gyr_device_power_off(struct lsm6ds3_status *stat)
{
	int err;

	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl2_g.address, LSM6DS3_GYR_ODR_MASK, LSM6DS3_GYR_ODR_OFF);
	if (err < 0)
		dev_err(&stat->client->dev, "gyroscope soft power off "
							"failed: %d\n", err);

	if (stat->pdata_gyr->power_off) {
		stat->pdata_gyr->power_off();
	}

	atomic_set(&stat->enabled_gyr, 0);
	dev_info(&stat->client->dev, "gyroscope switched off.");

	return 0;
}

static int lsm6ds3_step_counter_device_power_off(struct lsm6ds3_status *stat)
{
	int err;

	/* Disable Pedometer */
	err = lsm6ds3_write_with_mask(stat, status_registers.tap_cfg.address, STEP_COUNTER_EN_MASK, 0x00);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter soft power off "
							"failed: %d\n", err);

	/* Disable Pedometer on INT1 */
	err = lsm6ds3_write_with_mask(stat, status_registers.int1_ctrl.address, STEP_COUNTER_INT1_MASK, 0x00);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter soft power off "
							"failed: %d\n", err);

	/* Disable Embedded Functions */
	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl10_c.address, STEP_COUNTER_FUNC_EN_MASK, 0x00);
	if (err < 0)
		dev_err(&stat->client->dev, "step counter soft power off "
							"failed: %d\n", err);

	atomic_set(&stat->enabled_step_counter, 0);
	dev_info(&stat->client->dev, "step counter switched off.");

	return 0;
}

static int lsm6ds3_gyr_disable(struct lsm6ds3_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_gyr, 1, 0)) {
		cancel_work_sync(&stat->input_work_gyr);
		hrtimer_cancel(&stat->hr_timer_gyr);
		lsm6ds3_gyr_device_power_off(stat);
	}

	return 0;
}

static int lsm6ds3_acc_disable(struct lsm6ds3_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_acc, 1, 0)) {
		cancel_work_sync(&stat->input_work_acc);
		hrtimer_cancel(&stat->hr_timer_acc);
		lsm6ds3_acc_device_power_off(stat);
	}

	return 0;
}

static int lsm6ds3_step_counter_disable(struct lsm6ds3_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_step_counter, 1, 0)) {
		cancel_work_sync(&stat->input_work_step_counter);
		hrtimer_cancel(&stat->hr_timer_step_counter);
		lsm6ds3_step_counter_device_power_off(stat);
	}

	return 0;
}

static void lsm6ds3_acc_input_cleanup(struct lsm6ds3_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);
}

static void lsm6ds3_gyr_input_cleanup(struct lsm6ds3_status *stat)
{
	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

static void lsm6ds3_step_counter_input_cleanup(struct lsm6ds3_status *stat)
{
	input_unregister_device(stat->input_dev_step_counter);
	input_free_device(stat->input_dev_step_counter);
}

enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct lsm6ds3_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct lsm6ds3_status, hr_timer_acc);

	queue_work(lsm6ds3_workqueue, &stat->input_work_acc);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart poll_function_read_gyr(struct hrtimer *timer)
{
	struct lsm6ds3_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct lsm6ds3_status, hr_timer_gyr);

	queue_work(lsm6ds3_workqueue, &stat->input_work_gyr);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart poll_function_read_step_counter(struct hrtimer *timer)
{
	struct lsm6ds3_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct lsm6ds3_status, hr_timer_step_counter);

	queue_work(lsm6ds3_workqueue, &stat->input_work_step_counter);
	return HRTIMER_NORESTART;
}

static void lsm6ds3_validate_polling(unsigned int *min_interval,
					unsigned int *poll_interval,
					unsigned int min)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);
}

static int lsm6ds3_acc_validate_pdata(struct lsm6ds3_status *stat)
{
	int res = -EINVAL;

	lsm6ds3_validate_polling(&stat->pdata_acc->min_interval,
				 &stat->pdata_acc->poll_interval,
				(unsigned int)LSM6DS3_ACC_MIN_POLL_PERIOD_MS);

	switch (stat->pdata_acc->aa_filter_bandwidth) {
	case LSM6DS3_ACC_BW_50:
		res = 1;
		break;
	case LSM6DS3_ACC_BW_100:
		res = 1;
		break;
	case LSM6DS3_ACC_BW_200:
		res = 1;
		break;
	case LSM6DS3_ACC_BW_400:
		res = 1;
		break;
	default:
		dev_err(&stat->client->dev, "invalid accelerometer "
			"bandwidth selected: %u\n",
				stat->pdata_acc->aa_filter_bandwidth);
	}

	return res;
}

static int lsm6ds3_gyr_validate_pdata(struct lsm6ds3_status *stat)
{
	/* checks for correctness of minimal polling period */
	lsm6ds3_validate_polling(&stat->pdata_gyr->min_interval, 
				 &stat->pdata_gyr->poll_interval,
				(unsigned int)LSM6DS3_GYR_MIN_POLL_PERIOD_MS);

	/* Enforce minimum polling interval */
	if (stat->pdata_gyr->poll_interval < stat->pdata_gyr->min_interval) {
		dev_err(&stat->client->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int lsm6ds3_acc_gyr_hw_init(struct lsm6ds3_status *stat)
{
	int err = -1;
	u8 buf[1];

	dev_info(&stat->client->dev, "%s: hw init start\n", LSM6DS3_ACC_GYR_DEV_NAME);

	buf[0] = status_registers.who_am_i.address;
	err = lsm6ds3_i2c_read(stat, buf, 1);
	if (err < 0) {
		dev_warn(&stat->client->dev, "Error reading WHO_AM_I: is device"
		" available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != status_registers.who_am_i.default_val) {
	dev_err(&stat->client->dev,
		"device unknown. Expected: 0x%02x,"
		" Replies: 0x%02x\n", status_registers.who_am_i.default_val, 
					buf[0]);
		err = -1;
		goto err_unknown_device;
	}
	
	status_registers.func_cfg_access.resume_val =
				status_registers.func_cfg_access.default_val;
	status_registers.sensor_sync_time_frame.resume_val =
				status_registers.sensor_sync_time_frame.default_val;
	status_registers.sensor_sync_enable.resume_val =
				status_registers.sensor_sync_enable.default_val;
	status_registers.fifo_ctrl1.resume_val =
				status_registers.fifo_ctrl1.default_val;
	status_registers.fifo_ctrl2.resume_val =
				status_registers.fifo_ctrl2.default_val;
	status_registers.fifo_ctrl3.resume_val =
				status_registers.fifo_ctrl3.default_val;
	status_registers.fifo_ctrl4.resume_val =
				status_registers.fifo_ctrl4.default_val;
	status_registers.fifo_ctrl5.resume_val =
				status_registers.fifo_ctrl5.default_val;
	status_registers.orient_cfg_g.resume_val =
				status_registers.orient_cfg_g.default_val;
	status_registers.int1_ctrl.resume_val =
				status_registers.int1_ctrl.default_val;
	status_registers.int2_ctrl.resume_val =
				status_registers.int2_ctrl.default_val;
	status_registers.ctrl1_xl.resume_val = 
				status_registers.ctrl1_xl.default_val;
	status_registers.ctrl2_g.resume_val = 
				status_registers.ctrl2_g.default_val;
	status_registers.ctrl3_c.resume_val =
				status_registers.ctrl3_c.default_val;
	status_registers.ctrl4_c.resume_val =
				status_registers.ctrl4_c.default_val;
	status_registers.ctrl5_c.resume_val =
				status_registers.ctrl5_c.default_val;
	status_registers.ctrl6_g.resume_val =
				status_registers.ctrl6_g.default_val;
	status_registers.ctrl7_g.resume_val =
				status_registers.ctrl7_g.default_val;
	status_registers.ctrl8_xl.resume_val =
				status_registers.ctrl8_xl.default_val;
	status_registers.ctrl9_xl.resume_val =
				status_registers.ctrl9_xl.default_val;
	status_registers.ctrl10_c.resume_val =
				status_registers.ctrl10_c.default_val;
	status_registers.master_config.resume_val =
				status_registers.master_config.default_val;
	status_registers.tap_cfg.resume_val =
				status_registers.tap_cfg.default_val;
	status_registers.tap_ths_6d.resume_val =
				status_registers.tap_ths_6d.default_val;
	status_registers.int_dur2.resume_val =
				status_registers.int_dur2.default_val;
	status_registers.wake_up_ths.resume_val =
				status_registers.wake_up_ths.default_val;
	status_registers.wake_up_dur.resume_val =
				status_registers.wake_up_dur.default_val;
	status_registers.free_fall.resume_val =
				status_registers.free_fall.default_val;
	status_registers.md1_cfg.resume_val =
				status_registers.md1_cfg.default_val;
	status_registers.md2_cfg.resume_val =
				status_registers.md2_cfg.default_val;
				
	status_registers.slv0_add.resume_val =
				status_registers.slv0_add.default_val;
	status_registers.slv0_subadd.resume_val =
				status_registers.slv0_subadd.default_val;
	status_registers.slave0_config.resume_val =
				status_registers.slave0_config.default_val;
	status_registers.slv1_add.resume_val =
				status_registers.slv1_add.default_val;
	status_registers.slv1_subadd.resume_val =
				status_registers.slv1_subadd.default_val;
	status_registers.slave1_config.resume_val =
				status_registers.slave1_config.default_val;
	status_registers.slv2_add.resume_val =
				status_registers.slv2_add.default_val;
	status_registers.slv2_subadd.resume_val =
				status_registers.slv2_subadd.default_val;
	status_registers.slave2_config.resume_val =
				status_registers.slave2_config.default_val;
	status_registers.slv3_add.resume_val =
				status_registers.slv3_add.default_val;
	status_registers.slv3_subadd.resume_val =
				status_registers.slv3_subadd.default_val;
	status_registers.slave3_config.resume_val =
				status_registers.slave3_config.default_val;
	status_registers.datawrite_src_mode_sub_slv0.resume_val =
				status_registers.datawrite_src_mode_sub_slv0.default_val;
	status_registers.config_pedo_ths_min.resume_val =
				status_registers.config_pedo_ths_min.default_val;
	status_registers.config_tilt_iir.resume_val =
				status_registers.config_tilt_iir.default_val;
				
	status_registers.config_tilt_acos.resume_val =
			status_registers.config_tilt_acos.default_val;
	status_registers.config_tilt_wtime.resume_val =
			status_registers.config_tilt_wtime.default_val;
	status_registers.sm_step_ths.resume_val =
			status_registers.sm_step_ths.default_val;
	status_registers.mag_si_aa.resume_val =
			status_registers.mag_si_aa.default_val;
	status_registers.mag_si_bb.resume_val =
			status_registers.mag_si_bb.default_val;
	status_registers.mag_si_cc.resume_val =
			status_registers.mag_si_cc.default_val;
	status_registers.mag_si_dd.resume_val =
			status_registers.mag_si_dd.default_val;
	status_registers.mag_si_ee.resume_val =
			status_registers.mag_si_ee.default_val;
	status_registers.mag_si_ff.resume_val =
			status_registers.mag_si_ff.default_val;
	status_registers.mag_si_gg.resume_val =
			status_registers.mag_si_gg.default_val;
	status_registers.mag_si_hh.resume_val =
			status_registers.mag_si_hh.default_val;
	status_registers.mag_si_ii.resume_val =
			status_registers.mag_si_ii.default_val;

	status_registers.mag_offx_l.resume_val =
			status_registers.mag_offx_l.default_val;
	status_registers.mag_offx_h.resume_val =
			status_registers.mag_offx_h.default_val;
	status_registers.mag_offy_l.resume_val =
			status_registers.mag_offy_l.default_val;
	status_registers.mag_offy_h.resume_val =
			status_registers.mag_offy_h.default_val;
	status_registers.mag_offz_l.resume_val =
			status_registers.mag_offz_l.default_val;
	status_registers.mag_offz_h.resume_val =
			status_registers.mag_offz_h.default_val;	
	
	/* enable address auto inc */
	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl3_c.address, ADDRESS_AUTOINC_MASK, ADDRESS_AUTOINC_EN);
	if (err < 0)
		goto err_auto_inc;

	stat->temp_value_dec = NDTEMP;
	stat->hw_initialized = 1;
	dev_info(&stat->client->dev, "%s: hw init done\n", LSM6DS3_ACC_GYR_DEV_NAME);

	return 0;

err_auto_inc:
err_unknown_device:
err_firstread:
	stat->hw_working = 0;
	stat->hw_initialized = 0;
	return err;
}

static int lsm6ds3_acc_device_power_on(struct lsm6ds3_status *stat)
{
	int err = -1;
	u8 buf[9];

	if (stat->pdata_acc->power_on) {
		err = stat->pdata_acc->power_on();
		if (err < 0) {
			dev_err(&stat->client->dev,
				"accelerometer power_on failed: %d\n", err);
			return err;
		}
	}

#if 0
	buf[0] = status_registers.func_cfg_access.address;
 	buf[1] = status_registers.func_cfg_access.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.address;
 	buf[1] = status_registers.int1_ctrl.resume_val;
	buf[2] = status_registers.int2_ctrl.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl1_xl.address;
 	buf[1] = status_registers.ctrl1_xl.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl3_c.address;
	buf[1] = status_registers.ctrl3_c.resume_val;
	buf[2] = status_registers.ctrl4_c.resume_val;
	buf[3] = status_registers.ctrl5_c.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl8_xl.address;
	buf[1] = status_registers.ctrl8_xl.resume_val;
	buf[2] = status_registers.ctrl9_xl.resume_val;
	buf[3] = status_registers.ctrl10_c.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.fifo_ctrl1.address;
	buf[1] = status_registers.fifo_ctrl1.resume_val;
	buf[2] = status_registers.fifo_ctrl2.resume_val;
	buf[3] = status_registers.fifo_ctrl3.resume_val;
	buf[4] = status_registers.fifo_ctrl4.resume_val;
	buf[5] = status_registers.fifo_ctrl5.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;
#else
	lsm6ds3_acc_update_odr(stat, stat->pdata_acc->poll_interval);
	
	/* ODR and FS  default is 13Hz*/
//	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl1_xl.address, LSM6DS3_ACC_ODR_MASK, LSM6DS3_ACC_ODR_13);
//	if (err < 0)
//		goto err_resume_state;

	/* X/Y/Z axis enable */
	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl9_xl.address, CTRL9_XL_ALL_AXES_MASK, CTRL9_XL_ALL_AXES_EN);
	if (err < 0)
		goto err_resume_state;
#endif
	atomic_set(&stat->enabled_acc, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_acc, 0);
	dev_err(&stat->client->dev, "accelerometer hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int lsm6ds3_gyr_device_power_on(struct lsm6ds3_status *stat)
{
	int err = -1;
	u8 buf[9];

	if (stat->pdata_gyr->power_on) {
		err = stat->pdata_gyr->power_on();
		if (err < 0) {
			dev_err(&stat->client->dev,
				"gyroscope power_on failed: %d\n", err);
			return err;
		}
	}

#if 0
	buf[0] = status_registers.func_cfg_access.address;
 	buf[1] = status_registers.func_cfg_access.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.fifo_ctrl1.address;
	buf[1] = status_registers.fifo_ctrl1.resume_val;
	buf[2] = status_registers.fifo_ctrl2.resume_val;
	buf[3] = status_registers.fifo_ctrl3.resume_val;
	buf[4] = status_registers.fifo_ctrl4.resume_val;
	buf[5] = status_registers.fifo_ctrl5.resume_val;
	buf[6] = status_registers.orient_cfg_g.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 6);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.address;
 	buf[1] = status_registers.int1_ctrl.resume_val;
	buf[2] = status_registers.int2_ctrl.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 2);	
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl2_g.address;
	buf[1] = status_registers.ctrl2_g.resume_val;
 	buf[2] = status_registers.ctrl3_c.resume_val;
	buf[3] = status_registers.ctrl4_c.resume_val;
	buf[4] = status_registers.ctrl5_c.resume_val;
	buf[5] = status_registers.ctrl6_g.resume_val;
	buf[6] = status_registers.ctrl7_g.resume_val;
	err = lsm6ds3_i2c_write(stat, buf, 6);
	if (err < 0)
		goto err_resume_state;
#else
	lsm6ds3_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	/* ODR and FS , default is 13Hz*/
//	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl2_g.address, LSM6DS3_GYR_ODR_MASK, LSM6DS3_GYR_ODR_13);
//	if (err < 0)
//		goto err_resume_state;

	/* X/Y/Z axis enable */
	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl10_c.address, CTRL10_C_ALL_AXES_EN, CTRL10_C_ALL_AXES_EN);
	if (err < 0)
		goto err_resume_state;
#endif
	atomic_set(&stat->enabled_gyr, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_gyr, 0);
	dev_err(&stat->client->dev, "gyroscope hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int lsm6ds3_step_counter_device_power_on(struct lsm6ds3_status *stat)
{
	int err = -1;
	u8 buf[8];

#if 0
	/* Set ACC ODR  */
	response = LSM6DS3_ACC_GYRO_W_ODR_XL(LSM6DS3_ACC_GYRO_ODR_XL_26Hz);
	if(response==MEMS_ERROR) while(1); //manage here comunication error
#endif

	/* Enable Pedometer  */
	err = lsm6ds3_write_with_mask(stat, status_registers.tap_cfg.address, STEP_COUNTER_EN_MASK, STEP_COUNTER_EN_ENABLE);
	if (err < 0)
		goto err_resume_state;

#if 0
	/* Pedometer on INT1 */
	err = lsm6ds3_write_with_mask(stat, status_registers.int1_ctrl.address, STEP_COUNTER_INT1_MASK, STEP_COUNTER_INT1_ENABLE);
	if (err < 0)
		goto err_resume_state;

	/* Clear INT1 routing */
	buf[0] = status_registers.md1_cfg.address;
	buf[1] = 0x00;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;
	status_registers.md1_cfg.resume_val = buf[1];
#endif

	/* Enable Embedded Functions */
	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl10_c.address, STEP_COUNTER_FUNC_EN_MASK, STEP_COUNTER_FUNC_EN_ENABLE);
	if (err < 0)
		goto err_resume_state;
	
	/* Configure pedometer acceleration threshold */
	/* Open Embedded Function Register page */
	err = lsm6ds3_set_config_access(stat, 0x80, 0x80);
	if (err < 0)
		goto err_resume_state;

	/* write new value for threshold */
	err = lsm6ds3_write_with_mask(stat, 0x0F, 0x1F, 0x11);
	if (err < 0)
		goto err_resume_state;

	/* write new value for debouncing */
	buf[0] = 0x14;
	buf[1] = 0x6e; // 0xe6;
	err = lsm6ds3_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	/* Close Embedded Function Register page */
	err = lsm6ds3_set_config_access(stat, 0x80, 0x00);
	if (err < 0)
		goto err_resume_state;

	/* Clear the step counter */
	// LSM6DS3_ACC_GYRO_W_PedoStepReset(LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);

	atomic_set(&stat->enabled_step_counter, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_step_counter, 0);
	dev_err(&stat->client->dev, "step counter hw power on error "
				"%d\n", err);
	return err;
}

static int lsm6ds3_acc_update_fs_range(struct lsm6ds3_status *stat,
								u8 new_fs_range)
{
	int err = -1;

	u16 sensitivity;

	switch (new_fs_range) {
	case LSM6DS3_ACC_FS_2G:
		sensitivity = SENSITIVITY_ACC_2G;
		break;
	case LSM6DS3_ACC_FS_4G:
		sensitivity = SENSITIVITY_ACC_4G;
		break;
	case LSM6DS3_ACC_FS_8G:
		sensitivity = SENSITIVITY_ACC_8G;
		break;
	case LSM6DS3_ACC_FS_16G:
		sensitivity = SENSITIVITY_ACC_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid accelerometer "
				"fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl1_xl.address, LSM6DS3_ACC_FS_MASK, new_fs_range);
	if (err < 0)
		goto error;

	stat->sensitivity_acc = sensitivity;

	return err;

error:
	dev_err(&stat->client->dev, "update accelerometer fs range failed "
		": %d\n", err);
	return err;
}

static int lsm6ds3_gyr_update_fs_range(struct lsm6ds3_status *stat,
								u8 new_fs_range)
{
	int err = -1;
	u32 sensitivity;

	switch(new_fs_range) {
	case LSM6DS3_GYR_FS_245DPS:
		sensitivity = SENSITIVITY_GYR_245;
		break;
	case LSM6DS3_GYR_FS_500DPS:
		sensitivity = SENSITIVITY_GYR_500;
		break;
	case LSM6DS3_GYR_FS_1000DPS:
		sensitivity = SENSITIVITY_GYR_1000;
		break;
	case LSM6DS3_GYR_FS_2000DPS:
		sensitivity = SENSITIVITY_GYR_2000;
		break;
	default:
		dev_err(&stat->client->dev, "invalid g range "
					"requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl2_g.address, LSM6DS3_GYR_FS_MASK, new_fs_range);
	if (err < 0)
		goto error;

	stat->sensitivity_gyr = sensitivity;

error:
	return err;
}

static int lsm6ds3_acc_update_odr(struct lsm6ds3_status *stat,
						unsigned int poll_interval_ms)
{
	int err = -1;
	int i;

	for (i = ARRAY_SIZE(lsm6ds3_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm6ds3_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}

	if (atomic_read(&stat->enabled_acc)) {
	
		err = lsm6ds3_write_with_mask(stat, status_registers.ctrl1_xl.address, LSM6DS3_ACC_ODR_MASK, lsm6ds3_acc_odr_table[i].value);
		if (err < 0)
			goto error;

		stat->ktime_acc = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}

	return err;

error:
	dev_err(&stat->client->dev, "update accelerometer odr failed "
			"%d\n", err);

	return err;
}

static int lsm6ds3_gyr_update_odr(struct lsm6ds3_status *stat,
						unsigned int poll_interval_ms)
{
	int err = -1;
	int i;

	for (i = ARRAY_SIZE(lsm6ds3_gyr_odr_table) - 1; i >= 0; i--) {
		if ((lsm6ds3_gyr_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}

	if (atomic_read(&stat->enabled_gyr)) {
		/* Set ODR value */
		err = lsm6ds3_write_with_mask(stat, status_registers.ctrl2_g.address, LSM6DS3_GYR_ODR_MASK, lsm6ds3_gyr_odr_table[i].value);
		if (err < 0)
			goto error;

		stat->ktime_gyr = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}

	return err;

error:
	dev_err(&stat->client->dev, "update accelerometer odr failed "
			"%d\n", err);

	return err;
}

static int lsm6ds3_acc_update_filter(struct lsm6ds3_status *stat,
							u8 new_bandwidth)
{
	int err = -1;

	switch (new_bandwidth) {
	case LSM6DS3_ACC_BW_50:
		break;
	case LSM6DS3_ACC_BW_100:
		break;
	case LSM6DS3_ACC_BW_200:
		break;
	case LSM6DS3_ACC_BW_400:
		break;
	default:
		dev_err(&stat->client->dev, "invalid accelerometer "
			"update bandwidth requested: %u\n", new_bandwidth);
		return -EINVAL;
	}

	err = lsm6ds3_write_with_mask(stat, status_registers.ctrl1_xl.address, LSM6DS3_ACC_BW_MASK, new_bandwidth);
	if (err < 0)
		goto error;

	return err;

error:
	dev_err(&stat->client->dev, "update accelerometer fs range failed "
		": %d\n", err);
	return err;
}

static int lsm6ds3_step_counter_update_odr(struct lsm6ds3_status *stat,
						unsigned int poll_interval_ms)
{
	int err = 0;

	if (atomic_read(&stat->enabled_step_counter)) {
		stat->ktime_step_counter = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}

	return err;
}

static int lsm6ds3_acc_enable(struct lsm6ds3_status *stat)
{
	int err = -1;

	if (!atomic_cmpxchg(&stat->enabled_acc, 0, 1)) {
		err = lsm6ds3_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_acc, 0);
			dev_err(&stat->client->dev, "enable accelerometer "
				      "failed");
			return err;
		}

		hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc, 
							HRTIMER_MODE_REL);
	}

	return 0;
}

static int lsm6ds3_gyr_enable(struct lsm6ds3_status *stat)
{
	int err = -1;

	if (!atomic_cmpxchg(&stat->enabled_gyr, 0, 1)) {

		err = lsm6ds3_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_gyr, 0);
			return err;
		}

		hrtimer_start(&(stat->hr_timer_gyr), stat->ktime_gyr,
							HRTIMER_MODE_REL);
	}
	return 0;
}

static int lsm6ds3_step_counter_enable(struct lsm6ds3_status *stat)
{
	int err = -1;

	if (!atomic_cmpxchg(&stat->enabled_step_counter, 0, 1)) {
		err = lsm6ds3_step_counter_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_step_counter, 0);
			return err;
		}

		hrtimer_start(&(stat->hr_timer_step_counter), stat->ktime_step_counter,
							HRTIMER_MODE_REL);
	}
	return 0;
}

int lsm6ds3_acc_input_open(struct input_dev *input)
{
	struct lsm6ds3_status *stat = input_get_drvdata(input);
	dev_dbg(&stat->client->dev, "%s\n", __func__);

	//return lsm6ds3_acc_enable(stat);
	return 0;
}

void lsm6ds3_acc_input_close(struct input_dev *dev)
{
	struct lsm6ds3_status *stat = input_get_drvdata(dev);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	//lsm6ds3_acc_disable(stat);
}

int lsm6ds3_gyr_input_open(struct input_dev *input)
{
	struct lsm6ds3_status *stat = input_get_drvdata(input);
	dev_dbg(&stat->client->dev, "%s\n", __func__);

	//return lsm6ds3_gyr_enable(stat);
	return 0;
}

void lsm6ds3_gyr_input_close(struct input_dev *dev)
{
	struct lsm6ds3_status *stat = input_get_drvdata(dev);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	//lsm6ds3_gyr_disable(stat);
}

int lsm6ds3_step_counter_input_open(struct input_dev *input)
{
	struct lsm6ds3_status *stat = input_get_drvdata(input);
	dev_dbg(&stat->client->dev, "%s\n", __func__);

	//return lsm6ds3_step_counter_enable(stat);
	return 0;
}

void lsm6ds3_step_counter_input_close(struct input_dev *dev)
{
	struct lsm6ds3_status *stat = input_get_drvdata(dev);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	//lsm6ds3_step_counter_disable(stat);
}

//add by zhou at 2015.4.23
static void lsm6ds3_swap(int* x, int* y){
	int tmp;
	tmp = *x;
	*x = *y;
	*y = tmp;
}
//end add
static int lsm6ds3_acc_get_data(struct lsm6ds3_status *stat, int *xyz)
{
	int i, err = -1;
	u8 acc_data[6];
	s32 hw_d[3] = { 0 };

	acc_data[0] = OUTX_L_XL;
	err = lsm6ds3_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s32)( (s16)((acc_data[1] << 8) | (acc_data[0]))));
	hw_d[1] = ((s32)( (s16)((acc_data[3] << 8) | (acc_data[2]))));
	hw_d[2] = ((s32)( (s16)((acc_data[5] << 8) | (acc_data[4]))));
/* del by zhou at 2015.4.23
	hw_d[0] = hw_d[0] * stat->sensitivity_acc;
	hw_d[1] = hw_d[1] * stat->sensitivity_acc;
	hw_d[2] = hw_d[2] * stat->sensitivity_acc;
*/
	
	// add by zhou swap x and y, because the hardware orientaion error. set x reverse.
	lsm6ds3_swap(&hw_d[0], &hw_d[1]);
	hw_d[0] = - hw_d[0];
	
	for (i = 0; i < 3; i++) {
		xyz[i] = stat->pdata_main->rot_matrix[0][i] * hw_d[0] +
				stat->pdata_main->rot_matrix[1][i] * hw_d[1] +
				stat->pdata_main->rot_matrix[2][i] * hw_d[2];
	}

	return err;
}

static int lsm6ds3_gyr_get_data(struct lsm6ds3_status *stat, int *xyz)
{
	int i, err = 1;
	u8 gyro_data[6];
	s32 hw_d[3] = { 0 };

	gyro_data[0] = OUTX_L_G;
	err = lsm6ds3_i2c_read(stat, gyro_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (s32) ((s16)((gyro_data[1]) << 8) | gyro_data[0]);
	hw_d[1] = (s32) ((s16)((gyro_data[3]) << 8) | gyro_data[2]);
	hw_d[2] = (s32) ((s16)((gyro_data[5]) << 8) | gyro_data[4]);
	
	hw_d[0] = hw_d[0] * stat->sensitivity_gyr;
	hw_d[1] = hw_d[1] * stat->sensitivity_gyr;
	hw_d[2] = hw_d[2] * stat->sensitivity_gyr;
	
	for (i = 0; i < 3; i++) {
		xyz[i] = stat->pdata_main->rot_matrix[0][i] * hw_d[0] +
				stat->pdata_main->rot_matrix[1][i] * hw_d[1] +
				stat->pdata_main->rot_matrix[2][i] * hw_d[2];
//		xyz[i] = (xyz[i] * GYR_RANG * GYR_HAL_COMPENSTATE) / GYR_REG_RANG;
	}
	//modify by zhou for bug 89878
	lsm6ds3_swap(&xyz[0], &xyz[1]);
	xyz[2] = -xyz[2];
	
	return err;
}

static int lsm6ds3_step_counter_get_data(struct lsm6ds3_status *stat, u32 *steps)
{
	int err = 1;
	u8 buf[2];

	buf[0] = 0x4B; // STEP_COUNTER_L register address
	err = lsm6ds3_i2c_read(stat, buf, 2);
	if (err < 0)
		return err;

	*steps = ((u32)((u16)((buf[1] << 8) | (buf[0]))));

	return 0;
}

static void lsm6ds3_acc_report_values(struct lsm6ds3_status *stat,
								int *xyz)
{
	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(stat->input_dev_acc, ABS_X, xyz[0] + stat->accel_cali[0]);
	input_report_abs(stat->input_dev_acc, ABS_Y, xyz[1] + stat->accel_cali[1]);
	input_report_abs(stat->input_dev_acc, ABS_Z, xyz[2] + stat->accel_cali[2]);

	input_event(stat->input_dev_acc, EV_SYN, SYN_TIME_SEC, 
		ktime_to_timespec(timestamp).tv_sec);
	input_event(stat->input_dev_acc, EV_SYN, SYN_TIME_NSEC, 
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(stat->input_dev_acc);
}

static void lsm6ds3_gyr_report_values(struct lsm6ds3_status *stat,
								int *xyz)
{
	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(stat->input_dev_gyr, ABS_RX, xyz[0] + stat->gyr_cali[0]);
	input_report_abs(stat->input_dev_gyr, ABS_RY, xyz[1] + stat->gyr_cali[1]);
	input_report_abs(stat->input_dev_gyr, ABS_RZ, xyz[2] + stat->gyr_cali[2]);

	input_event(stat->input_dev_gyr, EV_SYN, SYN_TIME_SEC, 
		ktime_to_timespec(timestamp).tv_sec);
	input_event(stat->input_dev_gyr, EV_SYN, SYN_TIME_NSEC, 
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(stat->input_dev_gyr);
}

static void lsm6ds3_step_counter_report_values(struct lsm6ds3_status *stat,
								int steps)
{
	input_report_abs(stat->input_dev_step_counter, ABS_MISC, steps);
	input_sync(stat->input_dev_step_counter);
}

static int lsm6ds3_acc_input_init(struct lsm6ds3_status *stat)
{
	int err;

	stat->input_dev_acc = input_allocate_device();
	if (!stat->input_dev_acc) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "accelerometer "
					"input device allocation failed\n");
		return err;
	}

	stat->input_dev_acc->open = lsm6ds3_acc_input_open;
	stat->input_dev_acc->close = lsm6ds3_acc_input_close;
	stat->input_dev_acc->name = LSM6DS3_ACC_DEV_NAME;
	stat->input_dev_acc->id.bustype = BUS_I2C;
	stat->input_dev_acc->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev_acc, stat);

	set_bit(EV_ABS, stat->input_dev_acc->evbit);

	input_set_abs_params(stat->input_dev_acc, ABS_X, -ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_acc, ABS_Y, -ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_acc, ABS_Z,-ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);

	err = input_register_device(stat->input_dev_acc);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register accelerometer input device %s\n",
				stat->input_dev_acc->name);
		input_free_device(stat->input_dev_acc);
	}

	return err;
}

static int lsm6ds3_gyr_input_init(struct lsm6ds3_status *stat)
{
	int err = -1;

	dev_dbg(&stat->client->dev, "%s\n", __func__);

	stat->input_dev_gyr = input_allocate_device();
	if (!stat->input_dev_gyr) {
		err = -ENOMEM;
		dev_err(&stat->client->dev,
			"input device allocation failed\n");
		return err;
	}

	stat->input_dev_gyr->open = lsm6ds3_gyr_input_open;
	stat->input_dev_gyr->close = lsm6ds3_gyr_input_close;
	stat->input_dev_gyr->name = LSM6DS3_GYR_DEV_NAME;
	stat->input_dev_gyr->id.bustype = BUS_I2C;
	stat->input_dev_gyr->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev_gyr, stat);

	set_bit(EV_ABS, stat->input_dev_gyr->evbit);

	input_set_abs_params(stat->input_dev_gyr, ABS_RX, -GYR_FS_MAX - 1,
							GYR_FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev_gyr, ABS_RY, -GYR_FS_MAX - 1,
							GYR_FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev_gyr, ABS_RZ, -GYR_FS_MAX - 1,
							GYR_FS_MAX, 0, 0);

	err = input_register_device(stat->input_dev_gyr);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register input device %s\n",
			stat->input_dev_gyr->name);
		input_free_device(stat->input_dev_gyr);
	}

	return err;
}

static int lsm6ds3_step_counter_input_init(struct lsm6ds3_status *stat)
{
	int err;

	stat->input_dev_step_counter = input_allocate_device();
	if (!stat->input_dev_step_counter) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "step counter "
					"input device allocation failed\n");
		return err;
	}

	stat->input_dev_step_counter->open = lsm6ds3_step_counter_input_open;
	stat->input_dev_step_counter->close = lsm6ds3_step_counter_input_close;
	stat->input_dev_step_counter->name = LSM6DS3_STEP_COUNTER_DEV_NAME;
	stat->input_dev_step_counter->id.bustype = BUS_I2C;
	stat->input_dev_step_counter->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev_step_counter, stat);
	set_bit(EV_ABS, stat->input_dev_step_counter->evbit);

	input_set_abs_params(stat->input_dev_step_counter, ABS_MISC, -ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);

	err = input_register_device(stat->input_dev_step_counter);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register step counter input device %s\n",
				stat->input_dev_step_counter->name);
		input_free_device(stat->input_dev_step_counter);
	}

	return err;
}

static void lsm6ds3_input_cleanup(struct lsm6ds3_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);

	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
	
	input_unregister_device(stat->input_dev_step_counter);
	input_free_device(stat->input_dev_step_counter);
}

static ssize_t attr_set_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_ms = (unsigned int)max((unsigned int)interval_ms,
						stat->pdata_acc->min_interval);

	mutex_lock(&stat->lock);
	stat->pdata_acc->poll_interval = (unsigned int)interval_ms;
	lsm6ds3_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	unsigned int val;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_get_enable_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);

	int val = (int)atomic_read(&stat->enabled_acc);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6ds3_acc_enable(stat);
	else
		lsm6ds3_acc_disable(stat);

	return size;
}

static ssize_t attr_get_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	u8 val;
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	int range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->fs_range ;
	mutex_unlock(&stat->lock);

	switch (val) {
	case LSM6DS3_ACC_FS_2G:
		range = 2;
		break;
	case LSM6DS3_ACC_FS_4G:
		range = 4;
		break;
	case LSM6DS3_ACC_FS_8G:
		range = 8;
		break;
	case LSM6DS3_ACC_FS_16G:
		range = 16;
		break;
	}

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM6DS3_ACC_FS_2G;
		break;
	case 4:
		range = LSM6DS3_ACC_FS_4G;
		break;
	case 8:
		range = LSM6DS3_ACC_FS_8G;
		break;
	case 16:
		range = LSM6DS3_ACC_FS_16G;
		break;
	default:
		dev_err(&stat->client->dev, "accelerometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&stat->lock);
	err = lsm6ds3_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata_acc->fs_range = range;
	mutex_unlock(&stat->lock);

	dev_info(&stat->client->dev, "accelerometer range set to:"
							" %lu g\n", val);

	return size;
}

static ssize_t attr_get_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	u8 val;
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	int frequency = FILTER_400;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->aa_filter_bandwidth;
	mutex_unlock(&stat->lock);

	switch (val) {
	case LSM6DS3_ACC_BW_50:
		frequency = FILTER_50;
		break;
	case LSM6DS3_ACC_BW_100:
		frequency = FILTER_100;
		break;
	case LSM6DS3_ACC_BW_200:
		frequency = FILTER_200;
		break;
	case LSM6DS3_ACC_BW_400:
		frequency = FILTER_400;
		break;
	}

	return sprintf(buf, "%d\n", frequency);
}

static ssize_t attr_set_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 frequency;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case FILTER_50:
		frequency = LSM6DS3_ACC_BW_50;
		break;
	case FILTER_100:
		frequency = LSM6DS3_ACC_BW_100;
		break;
	case FILTER_200:
		frequency = LSM6DS3_ACC_BW_200;
		break;
	case FILTER_400:
		frequency = LSM6DS3_ACC_BW_400;
		break;
	default:
		dev_err(&stat->client->dev, "accelerometer invalid filter "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&stat->lock);
	err = lsm6ds3_acc_update_filter(stat, frequency);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata_acc->aa_filter_bandwidth = frequency;
	mutex_unlock(&stat->lock);

	dev_info(&stat->client->dev, "accelerometer anti-aliasing filter "
					"set to: %lu Hz\n", val);

	return size;
}

static ssize_t attr_get_polling_rate_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	int val;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	int err;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	
	interval_ms = (unsigned int)max((unsigned int)interval_ms,
					stat->pdata_gyr->min_interval);

	mutex_lock(&stat->lock);
	err = lsm6ds3_gyr_update_odr(stat, interval_ms);
	if(err >= 0)
		stat->pdata_gyr->poll_interval = interval_ms;
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_enable_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled_gyr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6ds3_gyr_enable(stat);
	else
		lsm6ds3_gyr_disable(stat);

	return size;
}

static ssize_t attr_get_range_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	int range = 0;
	u8 val;

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->fs_range;
	switch (val) {
	case LSM6DS3_GYR_FS_245DPS:
		range = RANGE_245DPS;
		break;
	case LSM6DS3_GYR_FS_500DPS:
		range = RANGE_500DPS;
		break;
	case LSM6DS3_GYR_FS_1000DPS:
		range = RANGE_1000DPS;
		break;
	case LSM6DS3_GYR_FS_2000DPS:
		range = RANGE_2000DPS;
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 245:
		range = LSM6DS3_GYR_FS_245DPS;
		break;
	case 500:
		range = LSM6DS3_GYR_FS_500DPS;
		break;
	case 1000:
		range = LSM6DS3_GYR_FS_1000DPS;
		break;
	case 2000:
		range = LSM6DS3_GYR_FS_2000DPS;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}

	mutex_lock(&stat->lock);
	err = lsm6ds3_gyr_update_fs_range(stat, range);
	if (err >= 0)
		stat->pdata_gyr->fs_range = range;
	mutex_unlock(&stat->lock);

	dev_info(&stat->client->dev, "range set to: %lu dps\n", val);

	return size;
}

/* add by Jay for the step counter */
static ssize_t attr_get_polling_rate_step_counter(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	int val;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_step_counter->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate_step_counter(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	int err;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	
	interval_ms = (unsigned int)max((unsigned int)interval_ms,
					stat->pdata_step_counter->min_interval);

	mutex_lock(&stat->lock);
	err = lsm6ds3_step_counter_update_odr(stat, interval_ms);
	if(err >= 0)
		stat->pdata_step_counter->poll_interval = interval_ms;
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_enable_step_counter(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled_step_counter);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_step_counter(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6ds3_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6ds3_step_counter_enable(stat);
	else
		lsm6ds3_step_counter_disable(stat);

	return size;
}
/* end by Jay for the step counter */

static struct kobj_attribute poll_attr_acc =
	__ATTR(pollrate_ms, 0664, attr_get_polling_rate_acc,
						attr_set_polling_rate_acc);
static struct kobj_attribute enable_attr_acc =
	__ATTR(enable_device, 0664, attr_get_enable_acc, attr_set_enable_acc);
static struct kobj_attribute fs_attr_acc =
	__ATTR(range, 0664, attr_get_range_acc, attr_set_range_acc);
static struct kobj_attribute aa_filter_attr  =
	__ATTR(anti_aliasing_frequency, 0664, attr_get_aa_filter,
							attr_set_aa_filter);
static struct kobj_attribute poll_attr_gyr =
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_gyr,
						attr_set_polling_rate_gyr);
static struct kobj_attribute enable_attr_gyr =
	__ATTR(enable_device, 0666, attr_get_enable_gyr, attr_set_enable_gyr);
static struct kobj_attribute range_attr_gyr =
	__ATTR(range, 0666, attr_get_range_gyr, attr_set_range_gyr);
	
/* add by Jay for the step counter */
static struct kobj_attribute poll_attr_step_counter =
	__ATTR(pollrate_ms, 0664, attr_get_polling_rate_step_counter,
						attr_set_polling_rate_step_counter);
static struct kobj_attribute enable_attr_step_counter =
	__ATTR(enable_device, 0664, attr_get_enable_step_counter, attr_set_enable_step_counter);
/* end by Jay for the step counter */

static struct attribute *attributes_acc[] = {
	&poll_attr_acc.attr,
	&enable_attr_acc.attr,
	&fs_attr_acc.attr,
	&aa_filter_attr.attr,
	NULL,
};

static struct attribute *attributes_gyr[] = {
	&poll_attr_gyr.attr,
	&enable_attr_gyr.attr,
	&range_attr_gyr.attr,
	NULL,
};

/* add by Jay for the step counter */
static struct attribute *attributes_step_counter[] = {
	&poll_attr_step_counter.attr,
	&enable_attr_step_counter.attr,
	NULL,
};
/* end by Jay for the step counter */

static struct attribute_group attr_group_acc = {
	.attrs = attributes_acc,
};

static struct attribute_group attr_group_gyr = {
	.attrs = attributes_gyr,
};

/* add by Jay for the step counter */
static struct attribute_group attr_group_step_counter = {
	.attrs = attributes_step_counter,
};
/* end by Jay for the step counter */

static int create_sysfs_interfaces(struct device *dev)
{
	int err;

	acc_kobj = kobject_create_and_add("accelerometer", &dev->kobj);
	if(!acc_kobj)
		return -ENOMEM;

	gyr_kobj = kobject_create_and_add("gyroscope", &dev->kobj);
	if(!gyr_kobj)
		return -ENOMEM;

	step_counter_kobj = kobject_create_and_add("stepcounter", &dev->kobj);
	if(!step_counter_kobj)
		return -ENOMEM;

	err = sysfs_create_group(acc_kobj, &attr_group_acc);
	if (err)
		kobject_put(acc_kobj);

	err = sysfs_create_group(gyr_kobj, &attr_group_gyr);
	if (err)
		kobject_put(gyr_kobj);

	err = sysfs_create_group(step_counter_kobj, &attr_group_step_counter);
	if (err)
		kobject_put(step_counter_kobj);

	return 0;
}

//add by zhou at 2015.2.2

static int	lsm6ds3_acc_devenable(struct sensors_classdev *sensors_cdev,
					unsigned int enabled){
	struct lsm6ds3_status *stat = container_of(sensors_cdev,
			                struct lsm6ds3_status, acc_sysdev);
	if (enabled){
		return	lsm6ds3_acc_enable(stat);
	 } else {
		return lsm6ds3_acc_disable(stat);
	 }
}
static int	lsm6ds3_acc_devdelay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec){
   
	struct lsm6ds3_status *stat = container_of(sensors_cdev,
			                struct lsm6ds3_status, acc_sysdev);
	unsigned long interval_ms;


	if (!delay_msec)
		return -EINVAL;

	interval_ms = (unsigned int)max(delay_msec, stat->pdata_acc->min_interval);

	mutex_lock(&stat->lock);
	stat->pdata_acc->poll_interval = (unsigned int)interval_ms;
	sensors_cdev->delay_msec = (unsigned int)interval_ms;
	lsm6ds3_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);

	return sizeof(delay_msec);
}


//add by zhou at 2015.4.23  add acc calibration function
static int lsm6ds3_acc_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now)
{
	int32_t ret, acc_data[3];
	unsigned int acc_cali_delay = 5;//make the test fastest
	unsigned int old_acc_delay;
	unsigned int acc_cali_enable = 1;
	struct lsm6ds3_status *stat =
		container_of(sensors_cdev, struct lsm6ds3_status, acc_sysdev);

	old_acc_delay = sensors_cdev->delay_msec;
	lsm6ds3_acc_devdelay(sensors_cdev, acc_cali_delay);
	if (!sensors_cdev->enabled)
		lsm6ds3_acc_devenable(sensors_cdev, acc_cali_enable);
	
	memset(acc_data, 0, sizeof(acc_data));
	ret = lsm6ds3_acc_get_data(stat, acc_data);
	if (ret < 0) {
		dev_err(&stat->client->dev, "get acc data error!\n");
		return ret;
	}

	stat->accel_cali[0] = -acc_data[0];
	stat->accel_cali[1] = -acc_data[1];
	stat->accel_cali[2] = GRAVITY_HAL_UNIT - ABS(acc_data[2]);

	dev_dbg(&stat->client->dev, "lsm6ds3 calibrate data, x:%d, y:%d, z:%d\n",
			stat->accel_cali[0], stat->accel_cali[1], stat->accel_cali[2]);

	if (sensors_cdev->params == NULL) {
		sensors_cdev->params = devm_kzalloc(&stat->client->dev, 128, GFP_KERNEL);
		if (sensors_cdev->params == NULL) {
			dev_err(&stat->client->dev, "acc sensor_cdev allocate memory error!\n");
			return -EBUSY;
		}
	}
	snprintf(sensors_cdev->params, 64, "%d,%d,%d", stat->accel_cali[0],
			stat->accel_cali[1], stat->accel_cali[2]);

	lsm6ds3_acc_devdelay(sensors_cdev, old_acc_delay);
	if (!sensors_cdev->enabled)
		lsm6ds3_acc_devenable(sensors_cdev, !acc_cali_enable);

	return 0;
}

static int	lsm6ds3_acc_write_cali_params(struct sensors_classdev
		*sensors_cdev, struct cal_result_t *cal_result)
{
	struct lsm6ds3_status *stat =
		container_of(sensors_cdev, struct lsm6ds3_status, acc_sysdev);

	if (sensors_cdev->params == NULL) {
		sensors_cdev->params = devm_kzalloc(&stat->client->dev, 128, GFP_KERNEL);
		if (sensors_cdev->params == NULL) {
			dev_err(&stat->client->dev, "acc sensor_cdev allocate memory error!\n");
			return -EBUSY;
		}
	}

	stat->accel_cali[0] = cal_result->offset[0];
	stat->accel_cali[1] = cal_result->offset[1];
	stat->accel_cali[2] = cal_result->offset[2];

	snprintf(sensors_cdev->params, 64, "%d,%d,%d", stat->accel_cali[0],
			stat->accel_cali[1], stat->accel_cali[2]);

	return 0;
}
//end add

static int	lsm6ds3_gyr_devenable(struct sensors_classdev *sensors_cdev,
					unsigned int enabled){
    struct lsm6ds3_status *stat = container_of(sensors_cdev,
			                struct lsm6ds3_status, gyr_sysdev);
	if(enabled){
		return lsm6ds3_gyr_enable(stat);
	} else {
		return lsm6ds3_gyr_disable(stat);
	}
}

static int	lsm6ds3_gyr_devdelay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec){
    struct lsm6ds3_status *stat = container_of(sensors_cdev,
			                struct lsm6ds3_status, gyr_sysdev);

	unsigned long interval_ms;

	if (!delay_msec)
		return -EINVAL;
	
	interval_ms = (unsigned int)max(delay_msec, stat->pdata_gyr->min_interval);

	mutex_lock(&stat->lock);
	stat->pdata_gyr->poll_interval = (unsigned int)interval_ms;
	sensors_cdev->delay_msec = (unsigned int)interval_ms;
	lsm6ds3_gyr_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);

	return sizeof(delay_msec);
}

static int lsm6ds3_gyr_calibrate(struct sensors_classdev *sensors_cdev,int axis,int apply_now)
{
       int32_t ret,gyr_data[3] = {0, 0, 0};
       long int sum_x = 0, sum_y = 0, sum_z = 0;
       int32_t i = 0;
	   int rept = GYRO_CAL_REPT;
	   int32_t max_data[3] = {0, 0, 0};
       int32_t min_data[3] = {0, 0, 0};
	   unsigned int gyr_cali_delay = 5;//make the test fastest
	   unsigned int old_gyr_delay;
	   unsigned int gyr_cali_enable = 1;
	   
       struct lsm6ds3_status *stat = container_of(sensors_cdev,struct lsm6ds3_status,gyr_sysdev);

		old_gyr_delay = sensors_cdev->delay_msec;
	   lsm6ds3_gyr_devdelay(sensors_cdev, gyr_cali_delay);
	   if (!sensors_cdev->enabled)
	   		lsm6ds3_gyr_devenable(sensors_cdev, gyr_cali_enable);

       for (i = 0; i < rept; i++)
       {
			msleep(stat->pdata_gyr->poll_interval);
			ret = lsm6ds3_gyr_get_data(stat, gyr_data);
			if (ret < 0){
			   dev_err(&stat->client->dev, "get gyr data error!\n");
			   return ret;
			}

			if (i == 0){
				max_data[0] = gyr_data[0];
				max_data[1] = gyr_data[1];
				max_data[2] = gyr_data[2];
				
				min_data[0] = gyr_data[0];
				min_data[1] = gyr_data[1];
				min_data[2] = gyr_data[2];
			}

			if (max_data[0] < gyr_data[0])
					max_data[0] = gyr_data[0];
			if (max_data[1] < gyr_data[1])
					max_data[1] = gyr_data[1];
			if (max_data[2] < gyr_data[2])
					max_data[2] = gyr_data[2];

			if (min_data[0] > gyr_data[0])
					min_data[0] = gyr_data[0];
			if (min_data[1] > gyr_data[1])
					min_data[1] = gyr_data[1];
			if (min_data[2] > gyr_data[2])
					min_data[2] = gyr_data[2];

			sum_x += gyr_data[0];
			sum_y += gyr_data[1];
			sum_z += gyr_data[2];
			   	
       }
	   
	   sum_x -= (max_data[0] + min_data[0]);
	   sum_y -= (max_data[1] + min_data[1]);
	   sum_z -= (max_data[2] + min_data[2]);
	   
       gyr_data[0] = sum_x / (rept - 2);
       gyr_data[1] = sum_y / (rept - 2);
       gyr_data[2] = sum_z / (rept - 2);
	   
       stat->gyr_cali[0] = -gyr_data[0];
       stat->gyr_cali[1] = -gyr_data[1];
       stat->gyr_cali[2] = -gyr_data[2];
       if (sensors_cdev->params == NULL) {
           sensors_cdev->params = devm_kzalloc(&stat->client->dev, 128, GFP_KERNEL);
           if (sensors_cdev->params == NULL) {
                   dev_err(&stat->client->dev, "acc sensor_cdev allocate memory error!\n");
                   return -EBUSY;
           }
       }
       snprintf(sensors_cdev->params,64,"%d,%d,%d",stat->gyr_cali[0],stat->gyr_cali[1],stat->gyr_cali[2]);
       dev_dbg(&stat->client->dev,"sensor calibration offset:%s\n",sensors_cdev->params);

	   lsm6ds3_gyr_devdelay(sensors_cdev, old_gyr_delay);
	   if (!sensors_cdev->enabled)
	   		lsm6ds3_gyr_devenable(sensors_cdev, !gyr_cali_enable);
       return 0;
}

static int lsm6ds3_gyr_write_cal_params(struct sensors_classdev *sensors_cdev,struct cal_result_t *cal_result)
{
    struct lsm6ds3_status *stat = container_of(sensors_cdev,struct lsm6ds3_status,gyr_sysdev);

       if (sensors_cdev->params == NULL) {
           sensors_cdev->params = devm_kzalloc(&stat->client->dev, 128, GFP_KERNEL);
           if (sensors_cdev->params == NULL) {
                   dev_err(&stat->client->dev, "acc sensor_cdev allocate memory error!\n");
                   return -EBUSY;
           }
       }
       stat->gyr_cali[0] = cal_result->offset[0];
       stat->gyr_cali[1] = cal_result->offset[1];
       stat->gyr_cali[2] = cal_result->offset[2];

       snprintf(sensors_cdev->params, 64, "%d,%d,%d",stat->gyr_cali[0],stat->gyr_cali[1],stat->gyr_cali[2]);
       return 0;
}

static int lsm6ds3_sysclass_register(struct device *dev, 
                                            struct lsm6ds3_status *status){
    int ret;
    status->acc_sysdev = lsm6ds3_acc_classdev;
	status->acc_sysdev.sensors_enable = lsm6ds3_acc_devenable;
	status->acc_sysdev.sensors_poll_delay = lsm6ds3_acc_devdelay;
	status->acc_sysdev.sensors_calibrate = lsm6ds3_acc_calibrate;
	status->acc_sysdev.sensors_write_cal_params = lsm6ds3_acc_write_cali_params;

	ret = sensors_classdev_register(dev, &status->acc_sysdev);
    if(ret){
        return -1;
    }
    
    status->gyr_sysdev = lsm6ds3_gyr_classdev;
	status->gyr_sysdev.sensors_enable = lsm6ds3_gyr_devenable;
	status->gyr_sysdev.sensors_poll_delay = lsm6ds3_gyr_devdelay;
	status->gyr_sysdev.sensors_calibrate = lsm6ds3_gyr_calibrate;
	status->gyr_sysdev.sensors_write_cal_params = lsm6ds3_gyr_write_cal_params;

	ret = sensors_classdev_register(dev, &status->gyr_sysdev);
    if(ret){
       sensors_classdev_unregister(&status->acc_sysdev); 
       return -1;
    }

    return 0;
}

static void lsm6ds3_sysclass_remove(struct lsm6ds3_status *status){
	sensors_classdev_unregister(&status->gyr_sysdev); 
	sensors_classdev_unregister(&status->acc_sysdev); 
}
//end add by zhou

static void remove_sysfs_interfaces(struct device *dev)
{
	kobject_put(acc_kobj);
	kobject_put(gyr_kobj);
	kobject_put(step_counter_kobj);
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
	struct lsm6ds3_status *stat;
	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct work_struct *)input_work_acc,
			struct lsm6ds3_status, input_work_acc);

	err = lsm6ds3_acc_get_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get accelerometer data failed\n");
	else
		lsm6ds3_acc_report_values(stat, xyz);

	hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc, HRTIMER_MODE_REL);
}

static void poll_function_work_gyr(struct work_struct *input_work_gyr)
{
	struct lsm6ds3_status *stat;
	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct work_struct *)input_work_gyr,
			struct lsm6ds3_status, input_work_gyr);

	err = lsm6ds3_gyr_get_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get gyroscope data failed.\n");
	else
		lsm6ds3_gyr_report_values(stat, xyz);

	hrtimer_start(&stat->hr_timer_gyr, stat->ktime_gyr, HRTIMER_MODE_REL);
}

static void poll_function_work_step_counter(struct work_struct *input_work_step_counter)
{
	struct lsm6ds3_status *stat;
	u32 steps = 0;
	int err;

	stat = container_of((struct work_struct *)input_work_step_counter,
			struct lsm6ds3_status, input_work_step_counter);

	err = lsm6ds3_step_counter_get_data(stat, &steps);
	if (err < 0)
		dev_err(&stat->client->dev, "get step counter data failed\n");
	else
		lsm6ds3_step_counter_report_values(stat, steps);

	hrtimer_start(&stat->hr_timer_step_counter, stat->ktime_step_counter, HRTIMER_MODE_REL);
}

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_acc_gyr_dt_id[] = {
	{.compatible = "st,lsm6ds3",},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6ds3_acc_gyr_dt_id);

static int lsm6ds3_acc_gyr_parse_dt(struct lsm6ds3_status *stat,
                                        struct device* dev)
{
	struct device_node *dn;
	uint8_t i, j;
	uint32_t val;
	short vect[9];

	if (of_match_device(lsm6ds3_acc_gyr_dt_id, dev)) {
		dn = dev->of_node;
		stat->pdata_main->of_node = dn;
		
		stat->pdata_main->gpio_int1 = of_get_gpio(dn, 0);
		if (!gpio_is_valid(stat->pdata_main->gpio_int1)) {
			dev_err(dev, "failed to get gpio_int1\n");

			stat->pdata_main->gpio_int1 = LSM6DS3_INT1_GPIO_DEF;
		}

		stat->pdata_main->gpio_int2 = of_get_gpio(dn, 1);
		if (!gpio_is_valid(stat->pdata_main->gpio_int2)) {
			dev_err(dev, "failed to get gpio_int2\n");

			stat->pdata_main->gpio_int2 = LSM6DS3_INT2_GPIO_DEF;
		}

		if (of_property_read_u16_array(dn, "rot-matrix", vect,
			      ARRAY_SIZE(vect)) >= 0) {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					stat->pdata_main->rot_matrix[i][j] =
						(short)vect[3 * j + i];
				}
			}
		} else {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					stat->pdata_main->rot_matrix[i][j] =
			default_lsm6ds3_main_platform_data.rot_matrix[i][j];
				}
			}
		}

		if (!of_property_read_u32(dn, "g-poll-interval", &val)) {
			stat->pdata_gyr->poll_interval = val;
		} else {
			stat->pdata_gyr->poll_interval =
				LSM6DS3_GYR_POLL_INTERVAL_DEF;
		}

		if (!of_property_read_u32(dn, "g-min-interval", &val)) {
			stat->pdata_gyr->min_interval = val;
		} else {
			stat->pdata_gyr->min_interval =
				LSM6DS3_GYR_MIN_POLL_PERIOD_MS;
		}

		if (!of_property_read_u32(dn, "g-fs-range", &val)) {
			stat->pdata_gyr->fs_range = val;
		} else {
			stat->pdata_gyr->fs_range = LSM6DS3_GYR_FS_245DPS;
		}

		if (!of_property_read_u32(dn, "x-poll-interval", &val)) {
			stat->pdata_acc->poll_interval = val;
		} else {
			stat->pdata_acc->poll_interval =
				LSM6DS3_ACC_POLL_INTERVAL_DEF;
		}

		if (!of_property_read_u32(dn, "x-min-interval", &val)) {
			stat->pdata_acc->min_interval = val;
		} else {
			stat->pdata_acc->min_interval =
				LSM6DS3_ACC_MIN_POLL_PERIOD_MS;
		}

		if (!of_property_read_u32(dn, "x-fs-range", &val)) {
			stat->pdata_acc->fs_range = val;
		} else {
			stat->pdata_acc->fs_range = LSM6DS3_ACC_FS_2G;
		}

		if (!of_property_read_u32(dn, "aa-filter-bw", &val)) {
			stat->pdata_acc->aa_filter_bandwidth = val;
		} else {
			stat->pdata_acc->aa_filter_bandwidth = LSM6DS3_ACC_BW_400;
		}

		/* add by Jay for the step counter */
		if (!of_property_read_u32(dn, "step-poll-interval", &val)) {
			stat->pdata_step_counter->poll_interval = val;
		} else {
			stat->pdata_step_counter->poll_interval =
				LSM6DS3_ACC_POLL_INTERVAL_DEF;
		}

		if (!of_property_read_u32(dn, "step-min-interval", &val)) {
			stat->pdata_step_counter->min_interval = val;
		} else {
			stat->pdata_step_counter->min_interval =
				LSM6DS3_ACC_MIN_POLL_PERIOD_MS;
		}
		/* end by Jay for the step counter */
		
		return 0;
	}
	return -1;
}
#else
#endif

//add by zhou at 2015.4.24
static int check_chip_id(struct lsm6ds3_status * stat){
	u8 buf[1];
	int ret;
	
	buf[0] = WHO_AM_I;
	ret = lsm6ds3_i2c_read(stat, buf, 1);
	if(ret < 0){
		dev_err(&stat->client->dev, "%s: Read failure :0x%02X",
		__func__, buf[0]);
		return -EPERM;
	}
	if(buf[0] != WHO_AM_I_VAL){
		dev_err(&stat->client->dev,
		"%s: Part failure miscompare act:0x%02x exp:0x%02x\n",
		__func__, buf[0], WHO_AM_I_VAL);
		return -ENOENT;
	}
	
	dev_info(&stat->client->dev, "Lsm6ds3 chip ID is 0x%02x", buf[0]);
	return 0;
}
//end add

static int lsm6ds3_acc_gyr_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lsm6ds3_status *stat;
	int err = -1;
	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	dev_info(&client->dev, "probe start.\n");
	stat = kzalloc(sizeof(struct lsm6ds3_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)){
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}

	if(lsm6ds3_workqueue == 0)
		lsm6ds3_workqueue = 
			create_workqueue("lsm6ds3_workqueue");

	hrtimer_init(&stat->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_acc.function = &poll_function_read_acc;
	hrtimer_init(&stat->hr_timer_gyr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_gyr.function = &poll_function_read_gyr;
	hrtimer_init(&stat->hr_timer_step_counter, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_step_counter.function = &poll_function_read_step_counter;

	mutex_init(&stat->lock);
	mutex_init(&stat->i2c_lock);
//	mutex_lock(&stat->lock);
	stat->client = client;
	i2c_set_clientdata(client, stat);

	//add chip check else system will be dead
	err = check_chip_id(stat);
	if(err){
		dev_err(&client->dev,"%s: Part ID Read Fail...\n", __func__);
		goto err_mutexunlock;	
	}
	
	stat->pdata_main = kzalloc(sizeof(*stat->pdata_main), GFP_KERNEL);
	stat->pdata_acc = kzalloc(sizeof(*stat->pdata_acc), GFP_KERNEL);
	stat->pdata_gyr = kzalloc(sizeof(*stat->pdata_gyr), GFP_KERNEL);
	stat->pdata_step_counter = kzalloc(sizeof(*stat->pdata_step_counter), GFP_KERNEL);

	if ((stat->pdata_main == NULL) || (stat->pdata_acc == NULL) ||
			(stat->pdata_gyr == NULL) || (stat->pdata_step_counter == NULL)){
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}
	stat->pdata_main->pdata_acc = stat->pdata_acc;
	stat->pdata_main->pdata_gyr = stat->pdata_gyr;
	stat->pdata_main->pdata_step_counter = stat->pdata_step_counter;
#ifdef CONFIG_OF
	lsm6ds3_acc_gyr_parse_dt(stat, &client->dev);
#else
	if (client->dev.platform_data == NULL) {
		memcpy(stat->pdata_main, 
				&default_lsm6ds3_main_platform_data,
				sizeof(*stat->pdata_main));
		memcpy(stat->pdata_acc, &default_lsm6ds3_acc_pdata,
						sizeof(*stat->pdata_acc));
		memcpy(stat->pdata_gyr, &default_lsm6ds3_gyr_pdata,
						sizeof(*stat->pdata_gyr));
		/* add by Jay for the step counter */
		memcpy(stat->pdata_step_counter, &default_lsm6ds3_step_counter_pdata,
						sizeof(*stat->pdata_step_counter));
		/* end by Jay for the step counter */
		dev_info(&client->dev, "using default plaform_data for "
					"accelerometer and gyroscope\n");
	} else {
		struct lsm6ds3_main_platform_data *platform_data;
		platform_data = client->dev.platform_data;

		if(platform_data == NULL) {
			memcpy(stat->pdata_main,
				&default_lsm6ds3_main_platform_data,
				sizeof(*stat->pdata_main));
			dev_info(&client->dev, "using default plaform_data for "
							"accelerometer\n");
		} else {
			memcpy(stat->pdata_main, platform_data,
						sizeof(*stat->pdata_acc));
		}

		if(platform_data->pdata_acc == NULL) {
			memcpy(stat->pdata_acc, &default_lsm6ds3_acc_pdata,
						sizeof(*stat->pdata_acc));
			dev_info(&client->dev, "using default plaform_data for "
							"accelerometer\n");
		} else {
			memcpy(stat->pdata_acc, platform_data->pdata_acc,
						sizeof(*stat->pdata_acc));
		}

		if(platform_data->pdata_gyr == NULL) {
			memcpy(stat->pdata_gyr, &default_lsm6ds3_gyr_pdata,
						sizeof(*stat->pdata_gyr));
			dev_info(&client->dev, "using default plaform_data for "
							"gyroscope\n");
		} else {
			memcpy(stat->pdata_gyr, platform_data->pdata_gyr,
						sizeof(*stat->pdata_gyr));
		}
		/* add by Jay for the step counter */
		if(platform_data->pdata_step_counter == NULL) {
			memcpy(stat->pdata_step_counter, &default_lsm6ds3_step_counter_pdata,
						sizeof(*stat->pdata_step_counter));
			dev_info(&client->dev, "using default plaform_data for "
							"step counter\n");
		} else {
			memcpy(stat->pdata_step_counter, platform_data->pdata_step_counter,
						sizeof(*stat->pdata_step_counter));
		}
		/* end by Jay for the step counter */
	}
#endif

	err = lsm6ds3_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data for "
							"accelerometer \n");
		goto exit_kfree_pdata;
	}

	err = lsm6ds3_gyr_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data for "
							"gyroscope\n");
		goto exit_kfree_pdata;
	}

	if (stat->pdata_acc->init) {
		err = stat->pdata_acc->init();
		if (err < 0) {
			dev_err(&client->dev, "accelerometer init failed: "
								"%d\n", err);
			goto err_pdata_acc_init;
		}
	}
	if (stat->pdata_gyr->init) {
		err = stat->pdata_gyr->init();
		if (err < 0) {
			dev_err(&client->dev, "magnetometer init failed: "
								"%d\n", err);
			goto err_pdata_gyr_init;
		}
	}

	err = lsm6ds3_acc_gyr_hw_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm6ds3_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "accelerometer power on failed: "
								"%d\n", err);
		goto err_pdata_init;
	}

	err = lsm6ds3_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "gyroscope power on failed: "
								"%d\n", err);
		goto err_pdata_init;
	}

	err = lsm6ds3_acc_update_fs_range(stat, stat->pdata_acc->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update accelerometer full scale range "
								"failed\n");
		goto  err_power_off_acc;
	}

	err = lsm6ds3_gyr_update_fs_range(stat, stat->pdata_gyr->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update gyroscope full scale range "
								"failed\n");
		goto  err_power_off_gyr;
	}

	err = lsm6ds3_acc_update_odr(stat, stat->pdata_acc->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update accelerometer ODR failed\n");
		goto  err_power_off;
	}

	err = lsm6ds3_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update gyroscope ODR failed\n");
		goto  err_power_off;
	}
	
	err = lsm6ds3_step_counter_update_odr(stat, stat->pdata_step_counter->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update step counter ODR failed\n");
		goto  err_power_off;
	}

	err = lsm6ds3_acc_update_filter(stat, 
					stat->pdata_acc->aa_filter_bandwidth);
	if (err < 0) {
		dev_err(&client->dev, "update accelerometer filter "
								"failed\n");
		goto  err_power_off;
	}

	err = lsm6ds3_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "accelerometer input init failed\n");
		goto err_power_off;
	}

	err = lsm6ds3_gyr_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "gyroscope input init failed\n");
		goto err_power_off;
	}
	
	err = lsm6ds3_step_counter_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "step counter input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "device %s sysfs register failed\n",
			LSM6DS3_ACC_GYR_DEV_NAME);
		goto err_input_cleanup;
	}

	//add by zhou at 2015.4.1 
	// add standard  sensors sys class interface 
	err = lsm6ds3_sysclass_register(&client->dev, stat);
 	if (err < 0) {
		dev_err(&client->dev, "device %s sysfs register failed\n",
			LSM6DS3_ACC_GYR_DEV_NAME);
		goto err_remove_sysfs;
	}
	//end add by zhou

	lsm6ds3_acc_device_power_off(stat);
	lsm6ds3_gyr_device_power_off(stat);
	lsm6ds3_step_counter_device_power_off(stat);
//	mutex_unlock(&stat->lock);

	INIT_WORK(&stat->input_work_acc, poll_function_work_acc);
	INIT_WORK(&stat->input_work_gyr, poll_function_work_gyr);
	INIT_WORK(&stat->input_work_step_counter, poll_function_work_step_counter);

	/* Initialize the step counter firmware */
	lsm6ds3_init_ram(stat, ram_code, sizeof(ram_code));

	dev_info(&client->dev, "%s: probed\n", LSM6DS3_ACC_GYR_DEV_NAME);
	return 0;
	
//add by zhou at 2015.4.1
err_remove_sysfs:
	remove_sysfs_interfaces(&client->dev);
//end add by zhou
err_input_cleanup:
	lsm6ds3_input_cleanup(stat);
err_power_off:
err_power_off_gyr:
	lsm6ds3_gyr_device_power_off(stat);
err_power_off_acc:
	lsm6ds3_acc_device_power_off(stat);
err_hw_init:
err_pdata_init:
err_pdata_gyr_init:
	if (stat->pdata_gyr->exit)
		stat->pdata_gyr->exit();
err_pdata_acc_init:
	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();
exit_kfree_pdata:
	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_step_counter);
err_mutexunlock:
//	mutex_unlock(&stat->lock);
	kfree(stat);
	if(!lsm6ds3_workqueue) {
		flush_workqueue(lsm6ds3_workqueue);
		destroy_workqueue(lsm6ds3_workqueue);
	}
exit_check_functionality_failed:
	dev_err(&client->dev,"%s: Driver Init failed\n", LSM6DS3_ACC_GYR_DEV_NAME);
	return err;
}

static int lsm6ds3_acc_gyr_remove(struct i2c_client *client)
{
	struct lsm6ds3_status *stat = i2c_get_clientdata(client);

	if (atomic_read(&stat->enabled_gyr)) {
		lsm6ds3_gyr_disable(stat);
		lsm6ds3_gyr_input_cleanup(stat);

		if (stat->pdata_gyr->exit)
			stat->pdata_gyr->exit();
	}
	
	if (atomic_read(&stat->enabled_step_counter)) {
		lsm6ds3_step_counter_disable(stat);
		lsm6ds3_step_counter_input_cleanup(stat);
	}

	lsm6ds3_acc_disable(stat);
	lsm6ds3_acc_input_cleanup(stat);

	remove_sysfs_interfaces(&client->dev);
    lsm6ds3_sysclass_remove(stat);//add by zhou

	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();

	if(!lsm6ds3_workqueue) {
		flush_workqueue(lsm6ds3_workqueue);
		destroy_workqueue(lsm6ds3_workqueue);
	}

	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_step_counter);
	kfree(stat->pdata_main);
	kfree(stat);
	return 0;
}

static const struct i2c_device_id lsm6ds3_acc_gyr_id[]
				= { { LSM6DS3_ACC_GYR_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm6ds3_acc_gyr_id);

static struct i2c_driver lsm6ds3_acc_gyr_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM6DS3_ACC_GYR_DEV_NAME,
#ifdef CONFIG_OF
			.of_match_table = 
				of_match_ptr(lsm6ds3_acc_gyr_dt_id),
#endif
		  },
	.probe = lsm6ds3_acc_gyr_probe,
	.remove = lsm6ds3_acc_gyr_remove,
	.id_table = lsm6ds3_acc_gyr_id,
};

static int __init lsm6ds3_acc_gyr_init(void)
{
	return i2c_add_driver(&lsm6ds3_acc_gyr_driver);
}

static void __exit lsm6ds3_acc_gyr_exit(void)
{
	i2c_del_driver(&lsm6ds3_acc_gyr_driver);
}

module_init(lsm6ds3_acc_gyr_init);
module_exit(lsm6ds3_acc_gyr_exit);

MODULE_DESCRIPTION("lsm6ds3 driver");
MODULE_AUTHOR("Darren HAN,Ian YANG,STMicroelectronics");
MODULE_LICENSE("GPL");
