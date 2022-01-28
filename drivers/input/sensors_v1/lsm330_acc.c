/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
 *
 * File Name		: lsm330_acc.c
 * Authors		: MSH - Motion Mems BU - Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Denis Ciocca (denis.ciocca@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
* Version		: V.1.0.2
* Date			: 2012/Oct/15
 * Description		: LSM330 accelerometer driver
 *
 *******************************************************************************
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
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#endif
#include <linux/jiffies.h>

#include "lsm330.h"


//#define	DEBUG

#define G_MAX			23920640	/* ug */
#define	I2C_RETRY_DELAY		5		/* Waiting for signals [ms] */
#define	I2C_RETRIES		5		/* Number of retries */
#define	I2C_AUTO_INCREMENT	0x80		/* Autoincrement i2c address */

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_6G		3	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/*
#define SENSITIVITY_2G		60	
#define SENSITIVITY_4G		120	
#define SENSITIVITY_6G		180	
#define SENSITIVITY_8G		240	
#define SENSITIVITY_16G		730	
*/
#define	LSM330_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define	LSM330_ODR_MASK		0XF0
#define LSM330_PM_OFF		0x00		/* OFF */
#define	LSM330_ODR3_125		0x10		/*    3.125 Hz */
#define	LSM330_ODR6_25		0x20		/*    6.25  Hz */
#define	LSM330_ODR12_5		0x30		/*   12.5   Hz */
#define	LSM330_ODR25		0x40		/*   25     Hz */
#define	LSM330_ODR50		0x50		/*   50     Hz */
#define	LSM330_ODR100		0x60		/*  100     Hz */
#define	LSM330_ODR400		0x70		/*  400     Hz */
#define	LSM330_ODR800		0x80		/*  800     Hz */
#define	LSM330_ODR1600		0x90		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREG1 */
#define LSM330_INTEN_MASK		0x01
#define LSM330_INTEN_OFF		0x00
#define LSM330_INTEN_ON			0x01

/* CTRLREG2 */
#define LSM330_HIST1_MASK		0xE0
#define LSM330_SM1INT_PIN_MASK		0x08
#define LSM330_SM1INT_PINB		0x08
#define LSM330_SM1INT_PINA		0x00
#define LSM330_SM1_EN_MASK		0x01
#define LSM330_SM1_EN_ON		0x01
#define LSM330_SM1_EN_OFF		0x00
/* */

/* CTRLREG3 */
#define LSM330_HIST2_MASK		0xE0
#define LSM330_SM2INT_PIN_MASK		0x08
#define LSM330_SM2INT_PINB		0x08
#define LSM330_SM2INT_PINA		0x00
#define LSM330_SM2_EN_MASK		0x01
#define LSM330_SM2_EN_ON		0x01
#define LSM330_SM2_EN_OFF		0x00
/* */

/* CTRLREG4 */
#define LSM330_INT_ACT_MASK		(0x01 << 6)
#define LSM330_INT_ACT_H		(0x01 << 6)
#define LSM330_INT_ACT_L		0x00

#define LSM330_INT2_EN_MASK		(0x01 << 4)
#define LSM330_INT2_EN_ON		(0x01 << 4)
#define LSM330_INT2_EN_OFF		0x00

#define LSM330_INT1_EN_MASK		(0x01 << 3)
#define LSM330_INT1_EN_ON		(0x01 << 3)
#define LSM330_INT1_EN_OFF		0x00
/* */

#define	OUT_AXISDATA_REG		LSM330_OUTX_L
#define WHOAMI_LSM330_ACC		0x40	/* Expected content for WAI */
#define WHOAMI_LIS3DSH_ACC		0x3F

/*	CONTROL REGISTERS	*/
#define	LSM330_WHO_AM_I			0x0F	/* WhoAmI register Address */

#define	LSM330_OUTX_L			0x28	/* Output X LSByte */
#define	LSM330_OUTX_H			0x29	/* Output X MSByte */
#define	LSM330_OUTY_L			0x2A	/* Output Y LSByte */
#define	LSM330_OUTY_H			0x2B	/* Output Y MSByte */
#define	LSM330_OUTZ_L			0x2C	/* Output Z LSByte */
#define	LSM330_OUTZ_H			0x2D	/* Output Z MSByte */
#define	LSM330_LC_L			0x16	/* LSByte Long Counter Status */
#define	LSM330_LC_H			0x17	/* MSByte Long Counter Status */
#define	LSM330_STAT_REG			0x18	/*STAT */

#define	LSM330_STATUS_REG		0x27	/* Status */

#define	LSM330_CTRL_REG1		0x21	/* control reg 1 */
#define	LSM330_CTRL_REG2		0x22	/* control reg 2 */
#define	LSM330_CTRL_REG3		0x23	/* control reg 3 */
#define	LSM330_CTRL_REG4		0x20	/* control reg 4 */
#define	LSM330_CTRL_REG5		0x24	/* control reg 3 */
#define	LSM330_CTRL_REG6		0x25	/* control reg 4 */

#ifdef LSM330_BATCH_MODE
#define LSM330_FIFO_CTRL_A		0x2E
#define LSM330_FIFO_SRC_A		0x2F
#define LSM330_FIFO_MODE_MASK 	0xe0
#define LSM330_FIFO_MODE_SHIFT	5

#define LSM330_ACC_ODR_MASK 0xf0
#define LSM330_ACC_ODR_SHIFT 4
#define LSM330_TIME_MS_TO_NS	1000000L

#define LSM330_ACC_FIFO_SRC_DATA_CNT_MASK		0x1F
#define LSM330_ACC_FIFO_MAX_CNT			0x1F
#define LSM330_ACC_FIFO_SRC_OVRN_MASK		0x40

#define LSM330_ACC_FIFO_EN 0x40
#define LSM330_ACC_FIFO_EN_MASK 0x40

#define	FUZZ			0
#define	FLAT			0

enum {
	LSM330_ACC_BYPASS_MODE = 0,
	LSM330_ACC_FIFO_MODE,
	LSM330_ACC_STREAM_MODE,
	LSM330_ACC_STREAM2FIFO_MODE,
	LSM330_ACC_BYPASS2STREAM_MODE,
	LSM330_ACC_FIFO_MODE_NUM
};

/* interrput mode for sensor max delay ms */
#define LSM330_INT_MAX_DELAY	10000

#define LSM330_FIFO_SIZE	32
#define LSM330_FIFO_EMPTY	0
#define LSM330_FIFO_OVERRUN_MASK 0x40
#endif


#define	LSM330_OFF_X			0x10	/* Offset X Corr */
#define	LSM330_OFF_Y			0x11	/* Offset Y Corr */
#define	LSM330_OFF_Z			0x12	/* Offset Z Corr */

#define	LSM330_CS_X			0x13	/* Const Shift X */
#define	LSM330_CS_Y			0x14	/* Const Shift Y */
#define	LSM330_CS_Z			0x15	/* Const Shift Z */

#define	LSM330_VFC_1			0x1B	/* Vect Filter Coeff 1 */
#define	LSM330_VFC_2			0x1C	/* Vect Filter Coeff 2 */
#define	LSM330_VFC_3			0x1D	/* Vect Filter Coeff 3 */
#define	LSM330_VFC_4			0x1E	/* Vect Filter Coeff 4 */

/*	end CONTROL REGISTRES	*/


/* RESUME STATE INDICES */
#define	LSM330_RES_LC_L				0
#define	LSM330_RES_LC_H				1

#define	LSM330_RES_CTRL_REG1			2
#define	LSM330_RES_CTRL_REG2			3
#define	LSM330_RES_CTRL_REG3			4
#define	LSM330_RES_CTRL_REG4			5
#define	LSM330_RES_CTRL_REG5			6
#define	LSM330_RES_CTRL_REG6            7

#define LSM330_RES_FIFO_CTRL_A          16

#define	LSM330_RES_TIM4_1			20
#define	LSM330_RES_TIM3_1			21
#define	LSM330_RES_TIM2_1_L			22
#define	LSM330_RES_TIM2_1_H			23
#define	LSM330_RES_TIM1_1_L			24
#define	LSM330_RES_TIM1_1_H			25

#define	LSM330_RES_THRS2_1			26
#define	LSM330_RES_THRS1_1			27
#define	LSM330_RES_SA_1				28
#define	LSM330_RES_MA_1				29
#define	LSM330_RES_SETT_1			30

#define	LSM330_RES_TIM4_2			31
#define	LSM330_RES_TIM3_2			32
#define	LSM330_RES_TIM2_2_L			33
#define	LSM330_RES_TIM2_2_H			34
#define	LSM330_RES_TIM1_2_L			35
#define	LSM330_RES_TIM1_2_H			36

#define	LSM330_RES_THRS2_2			37
#define	LSM330_RES_THRS1_2			38
#define	LSM330_RES_DES_2			39
#define	LSM330_RES_SA_2				40
#define	LSM330_RES_MA_2				41
#define	LSM330_RES_SETT_2			42

#define	LSM330_RESUME_ENTRIES			43



#define	LSM330_STATE_PR_SIZE			16
/* end RESUME STATE INDICES */

/* STATE PROGRAMS ENABLE CONTROLS */
#define	LSM330_SM1_DIS_SM2_DIS			0x00
#define	LSM330_SM1_DIS_SM2_EN			0x01
#define	LSM330_SM1_EN_SM2_DIS			0x02
#define	LSM330_SM1_EN_SM2_EN			0x03

/* INTERRUPTS ENABLE CONTROLS */
#define	LSM330_INT1_DIS_INT2_DIS		0x00
#define	LSM330_INT1_DIS_INT2_EN			0x01
#define	LSM330_INT1_EN_INT2_DIS			0x02
#define	LSM330_INT1_EN_INT2_EN			0x03

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330_acc_odr_table[] = {
		{    1, LSM330_ODR1600 },
		{    3, LSM330_ODR400  },
		{   10, LSM330_ODR100  },
		{   20, LSM330_ODR50   },
		{   40, LSM330_ODR25   },
		{   80, LSM330_ODR12_5 },
		{  160, LSM330_ODR6_25 },
		{  320, LSM330_ODR3_125},
};

static const struct lsm330_acc_platform_data default_lsm330_acc_pdata = {
	.fs_range = LSM330_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,
	.poll_interval = 100,
	.min_interval = LSM330_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LSM330_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LSM330_ACC_DEFAULT_INT2_GPIO,
};

#define LSM330_AXIS_X          0
#define LSM330_AXIS_Y          1
#define LSM330_AXIS_Z          2
#define LSM330_AXES_NUM        3
#define LSM330_DATA_LEN        6

#define GRAVITY_EARTH                   981

static struct sensors_classdev lsm330_acc_cdev = {
	.name = "lsm330_acc",
	.vendor = "LSM",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 10000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev lsm330_step_counter_cdev = {
	.name = LSM330_SC_DEV_NAME,
	.vendor = "LSM",
	.version = 1,
	.handle = SENSOR_STEP_COUNTER_HANDLE,
	.type = SENSOR_TYPE_STEP_COUNTER,
	.max_range = "255",
	.resolution = "1",
	.sensor_power = "0.01",
	.min_delay = 800000,
	.delay_msec = 800,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#define SG_MAX			23920640
#define REPORT     1
#define NOTREPORT  0
#define BATCH_ENABLE   1
#define BATCH_DISABLE  0
#define STEP_COUNTER_ONLY_WALK  1 
#define STEP_COUNTER_ADD_WALK_DEBOUNCE 2
#define STEP_COUNTER_DETAL_TIME 6750000000L

#define LSM330_LOG_LEVEL_POLL 20

static int lsm330_acc_flush_data(struct sensors_classdev *sensors_cdev, unsigned char isreport);
static struct i2c_client *lsm330_acc_i2c_client;
static int poll_delay_num = 0;

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)

#define TAG  "Gsensor "       //"Msensor" "PAsensor" "GYsensor"
#define TAGI "Gsensor.I "     //KERN_INFO 
#define TAGE "Gsensor.E "     //KERN_ERR 
extern void print_vivo_init(const char* fmt, ...);
extern void print_vivo_main(const char* fmt, ...);

#define CONT_INT_TIME     50    //50ms 
#define POLL_DELAY_TIMES  2     //2X

#define CONT_INT_GATE   20 
#define SAME_DATA_GATE  20
#define POLL_DELAY_GATE 5
#define MAX_DATA_GATE   0
#define MIN_DATA_GATE   0

struct calibrate_data {
	int x;
	int y;
	int z;
	int self_test;
};

struct acceleration {
	int x;
	int y;
	int z;
};

struct lsm330_acc_data {
	struct i2c_client *client;
	struct lsm330_acc_platform_data *pdata;

	struct mutex lock;
//	struct delayed_work input_work;
	struct work_struct input_work;
	struct workqueue_struct *data_wq;
	struct hrtimer timer_acc;
	ktime_t ktime_acc;
	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u16 sensitivity;
	u8 stateprogs_enable_setting;
	u8 interrupts_enable_setting;

	u8 resume_state[LSM330_RESUME_ENTRIES];
	u8 resume_stmach_program1[LSM330_STATE_PR_SIZE];
	u8 resume_stmach_program2[LSM330_STATE_PR_SIZE];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	
	struct sensors_classdev cdev;
	struct acceleration lastdata;       /* last measured data */
	struct acceleration lastraw;       /* last measured data */
    struct acceleration sameraw;       /* last measured data */
	struct calibrate_data calibration_data;
	int	calibrate_process;
	struct mutex data_mutex;
	atomic_t delay;
	atomic_t sensor_int;
	u8 data_valid;
	
	#ifdef LSM330_BATCH_MODE
	/* batch mode is configured */
	bool use_batch;
	unsigned int delay_ms;
	unsigned int batch_mode;
	unsigned int fifo_timeout_ms;
	unsigned int flush_count;
	u8 *acc_fifo;
	#endif
	u32 chip_sample_time;
	unsigned char flush_in_work;
	u64 time_current_ns;


    struct sensors_classdev step_counter_cdev;
    struct input_dev *input_step;
    struct delayed_work step_work;
    struct work_struct irq_work;
    unsigned int sc_poll_interval;
    unsigned int step_counter_overflow;
    unsigned int step_reg_value;
    unsigned int step_reg_raw_value;
    unsigned int step_reg_value_last;
    u8 sc_data_valid;
    unsigned int step_counter; 
    unsigned int step_counter_last; 
    int step_counter_enable;
    int step_counter_ststate;
    int irq_step;
    unsigned char step_walk_run;
    unsigned char force_report;
 //   unsigned char backlight_on;
    unsigned int walk_run_counter;
    u64 time_data_sec;
    unsigned int static_day;
    unsigned int current_day;
    unsigned int moving_detect;
    unsigned int step_counter_rept;
    int step_counter_delta;
    int step_counter_deb;
    int step_counter_deb_counter;
    int sc_deb_mode_counter;
    u64 sc_deb_mode_ns;
    int sc_deb_or_only_walk;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif

#ifdef DEBUG
	u8 reg_addr;
#endif
    u8 debug;
};

struct st_reg_val {
	u8 reg_add;
	u8 reg_val;
};

static struct st_reg_val lsm330_eint_config[]={
	{0x21,0x00},//SM1 Disable
	{0x20,0x57},
	{0x23,0x4C},//Int1 , High, Vector
	{0x24,0xC8},
	{0x25,0x10},
#if 0
	{0x10,0x00},
	{0x11,0x00},
	{0x12,0x00},
	{0x13,0x00},
	{0x14,0x00},
	{0x15,0x00},
	{0x16,0xFF},
	{0x17,0x7F},
#endif
	{0x1B,0x7D},
	{0x1C,0x40},
	{0x1D,0x20},
	{0x1E,0x10},
	{0x1F,0x00},
	{0x2E,0x00},
	{0x19,0x00},
	{0x1A,0x00},
#if 1
	{0x50,0x00},
	{0x51,0x00},
	{0x52,0x00},
	{0x53,0x00},
	{0x54,0x00},
	{0x55,0x00},
	{0x56,0x00},
#endif
	{0x57,0x02},//Threshold
	{0x59,0x00},
	{0x5A,0x03},
	{0x5B,0x21},
	{0x5C,0x00},
#if 1
	{0x70,0x00},
	{0x71,0x00},
	{0x72,0x00},
	{0x73,0x00},
	{0x74,0x00},
	{0x75,0x00},
	{0x76,0x00},
	{0x77,0x00},
	{0x78,0x00},
	{0x79,0x00},
	{0x7A,0x00},
	{0x7B,0x00},
	{0x7C,0x00},
#endif
	{0x40,0x05},//SM1
	{0x41,0x11},
	{0x42,0x00},
	{0x60,0x00},//SM2
	{0x23,0x4C},//Int1 , High, Vector
	{0x22,0x00},//SM2 Disable
	{0x21,0x01} //SM1 Enable
};
static struct st_reg_val lsm330_step_counter_init[]={
	{0x20,0x67},
	{0x23,0x4C},
	{0x24,0x08},
	{0x25,0x10},
//	{0x27,0xFF},
	{0x10,0x00},
	{0x11,0x00},
	{0x12,0x00},
	{0x13,0x00},
	{0x14,0x00},
	{0x15,0x00},
	{0x16,0xFF},
	{0x17,0x7F},
	{0x1B,0x7D},
	{0x1C,0x72},
	{0x1D,0x4C},
	{0x1E,0x26},
	{0x1F,0x00},
	{0x2E,0x00},
};

static struct st_reg_val lsm330_step_counter_walk_det[]={
    {0x19, 0x00},
    {0x1A, 0x00},
    {0x50, 0x00},
    {0x51, 0x00},
    {0x52, 0x2D},
    {0x53, 0x00},
    {0x54, 0x5A},
    {0x55, 0x00},
    {0x56, 0x00},
    {0x57, 0x03},
    {0x59, 0x00},
    {0x5A, 0x03},
    {0x5B, 0x21},
    {0x5C, 0x00},
    {0x5D, 0x21},
    {0x5E, 0x00},
    {0x5F, 0x00},
    {0x70, 0x00},
    {0x71, 0x00},
    {0x72, 0x1C},
    {0x73, 0x00},
    {0x74, 0x12},
    {0x75, 0x00},
    {0x76, 0x00},
    {0x77, 0x0D},
    {0x78, 0x00},
    {0x79, 0x00},
    {0x7A, 0x03},
    {0x7B, 0x21},
    {0x7C, 0x00},
    {0x7D, 0x11},
    {0x7E, 0x00},
    {0x7F, 0x00},
    {0x40, 0x15},
    {0x41, 0x02},
    {0x42, 0x15},
    {0x43, 0x02},
    {0x44, 0x15},
    {0x45, 0x02},
    {0x46, 0x15},
    {0x47, 0x02},
    {0x48, 0x15},
    {0x49, 0x02},
    {0x4A, 0x15},
    {0x4B, 0x02},
    {0x4C, 0x15},
    {0x4D, 0x02},
    {0x4E, 0x15},
    {0x4F, 0x11},
    {0x60, 0x15},
    {0x61, 0x02},
    {0x62, 0x15},
    {0x63, 0x02},
    {0x64, 0x15},
    {0x65, 0x02},
    {0x66, 0x15},
    {0x67, 0x02},
    {0x68, 0x15},
    {0x69, 0x02},
    {0x6A, 0x15},
    {0x6B, 0x02},
    {0x6C, 0x15},
    {0x6D, 0x02},
    {0x6E, 0x11},
    {0x6F, 0x00},
    {0x23, 0x4C},
    {0x21, 0x01},
    {0x22, 0x01},
};

static struct st_reg_val lsm330_step_counter_walk_only[]={
    {0x19,0x00},
    {0x1A,0x00},
    {0x50,0x00},
    {0x51,0x00},
    {0x52,0x2D},
    {0x53,0x00},
    {0x54,0x5A},/*0x52*/
    {0x55,0x00},
    {0x56,0x00},
    {0x57,0x03},
    {0x59,0x00},
    {0x5A,0x03},    
    {0x5B,0x20},
    {0x5C,0x00},
    {0x5D,0x2E},
    {0x5E,0x00},
    {0x5F,0x00},
    {0x70,0x00},
    {0x71,0x00},
    {0x72,0x2D},
    {0x73,0x00},
    {0x74,0x5A},/*0x52*/
    {0x75,0x00},
    {0x76,0x00},
    {0x77,0x03},
    {0x78,0x00},
    {0x79,0x00},
    {0x7A,0x03},
    {0x7B,0x20},
    {0x7C,0x00},
    {0x7D,0x2E},
    {0x7E,0x00},    
    {0x7F,0x00},
    {0x40,0xFF},
    {0x41,0x15},
    {0x42,0x02},
    {0x43,0x15},
    {0x44,0x02},
    {0x45,0x15},
    {0x46,0x02},
    {0x47,0x15},
    {0x48,0x02},
    {0x49,0x15},
    {0x4A,0x02},
    {0X4B,0x15},
    {0x4C,0x02},
    {0x4D,0xBB},
    {0x4E,0x11},
    {0x4F,0x11},    
    {0x60,0x15},
    {0x61,0x02},
    {0x62,0x15},    
    {0x63,0x02},
    {0x64,0x15},
    {0x65,0x02},
    {0x66,0x15},
    {0x67,0x02},
    {0x68,0x15},
    {0x69,0x02},
    {0x6A,0x15},
    {0x6B,0x02},
    {0x6C,0x15},
    {0x6D,0x02},
    {0x6E,0xFF},
    {0x6F,0x11},
    {0x23,0x4C},
    {0x21,0x01},
    {0x22,0x01},
};

static struct st_reg_val lsm330_step_counter_walk_deb[]={
    {0x1A, 0x00},
    {0x50, 0x00},
    {0x51, 0x00},
    {0x52, 0x2D},
    {0x53, 0x00},
    {0x54, 0x5A},
    {0x55, 0x00},
    {0x56, 0x00},
    {0x57, 0x03},
    {0x59, 0x00},
    {0x5A, 0x03},
    {0x5B, 0x21},
    {0x5C, 0x00},
    {0x5D, 0x3E},
    {0x5E, 0x00},
    {0x5F, 0x00},
    {0x70, 0x00},
    {0x71, 0x00},
    {0x72, 0x1C},
    {0x73, 0x00},
    {0x74, 0x12},
    {0x75, 0x00},
    {0x76, 0x00},
    {0x77, 0x0F},
    {0x78, 0x00},
    {0x79, 0x00},
    {0x7A, 0x03},
    {0x7B, 0x21},
    {0x7C, 0x00},
    {0x7D, 0x07},
    {0x7E, 0x00},
    {0x7F, 0x00},
    {0x40, 0x15},
    {0x41, 0x02},
    {0x42, 0x15},
    {0x43, 0x02},
    {0x44, 0x15},
    {0x45, 0x02},
    {0x46, 0x15},
    {0x47, 0x02},
    {0x48, 0x15},
    {0x49, 0x02},
    {0x4A, 0xBB},
    {0x4B, 0x22},
    {0x4C, 0x15},
    {0x4D, 0xE9},
    {0x4E, 0x11},
    {0x4F, 0x0B},
    {0x60, 0x15},
    {0x61, 0x02},
    {0x62, 0x15},
    {0x63, 0x02},
    {0x64, 0x15},
    {0x65, 0x02},
    {0x66, 0x15},
    {0x67, 0x02},
    {0x68, 0x15},
    {0x69, 0x02},
    {0x6A, 0x15},
    {0x6B, 0x02},
    {0x6C, 0x15},
    {0x6D, 0x02},
    {0x6E, 0x11},
    {0x6F, 0x00},
    {0x23, 0x4C},
    {0x21, 0x01},
    {0x22, 0x01},
};

static struct st_reg_val lsm330_step_counter_walk[]={
	{0x19,0x00},
	{0x1A,0x00},
	{0x50,0x00},
	{0x51,0x00},
	{0x52,0x2D},
	{0x53,0x00},
	{0x54,0x5A},
	{0x55,0x00},
	{0x56,0x00},
	{0x57,0x03},
	{0x59,0x00},
	{0x5A,0x03},	
	{0x5B,0x21},
	{0x5C,0x00},
	{0x5D,0x1F},
	{0x5E,0x00},
	{0x5F,0x00},
	{0x70,0x00},
	{0x71,0x00},
	{0x72,0x1C},
	{0x73,0x00},
	{0x74,0x12},
	{0x75,0x00},
	{0x76,0x00},
	{0x77,0x0F/*0x0D*/},
	{0x78,0x00},
	{0x79,0x00},
	{0x7A,0x03},
	{0x7B,0x21},
	{0x7C,0x00},
	{0x7D,0x07},
	{0x7E,0x00},	
	{0x7F,0x00},
	{0x40,0x15},
	{0x41,0x02},
	{0x42,0x15},
	{0x43,0x02},
	{0x44,0x15},
	{0x45,0x02},
	{0x46,0x15},
	{0x47,0x02},
	{0x48,0x15},
	{0x49,0xBB},
	{0x4A,0xBB},
	{0X4B,0xBB},
	{0x4C,0x02},
	{0x4D,0x22},
	{0x4E,0x15},
	{0x4F,0x0B},
	{0x60,0x15},
	{0x61,0x02},
	{0x62,0x15},	
	{0x63,0x02},
	{0x64,0x15},
	{0x65,0x02},
	{0x66,0x15},
	{0x67,0x02},
	{0x68,0x15},
	{0x69,0x02},
	{0x6A,0x15},
	{0x6B,0x02},
	{0x6C,0x15},
	{0x6D,0x02},
	{0x6E,0x11},
	{0x6F,0x00},
	{0x23,0x4C},
	{0x21,0x01},
	{0x22,0x01},
};

static struct st_reg_val lsm330_step_counter_run[]={
	{0x19,0x00},
	{0x1A,0x00},
	{0x50,0x00},
	{0x51,0x00},
	{0x52,0x19},
	{0x53,0x00},
	{0x54,0x52},
	{0x55,0x00},
	{0x56,0x00},
	{0x57,0x0B},
	{0x59,0x00},
	{0x5A,0x03},	
	{0x5B,0x21},
	{0x5C,0x00},
	{0x5D,0x2E},
	{0x5E,0x00},
	{0x5F,0x00},
	{0x70,0x00},
	{0x71,0x00},
	{0x72,0x00},
	{0x73,0x00},
	{0x74,0x3C},
	{0x75,0x00},
	{0x76,0x00},
	{0x77,0x0E},
	{0x78,0x00},
	{0x79,0x00},
	{0x7A,0x03},
	{0x7B,0x21},
	{0x7C,0x00},
	{0x7D,0x41},
	{0x7E,0x00},	
	{0x7F,0x00},
	{0x40,0x15},
	{0x41,0x02},
	{0x42,0x15},
	{0x43,0x02},
	{0x44,0x15},
	{0x45,0x02},
	{0x46,0x15},
	{0x47,0x02},
	{0x48,0x15},
	{0x49,0xBB},
	{0x4A,0xBB},
	{0X4B,0xBB},
	{0x4C,0x02},
	{0x4D,0x22},
	{0x4E,0x15},
	{0x4F,0x0B},
	{0x60,0x51},
	{0x61,0x11},
	{0x62,0x15},	
	{0x63,0x02},
	{0x64,0x15},
	{0x65,0x02},
	{0x66,0x15},
	{0x67,0x02},
	{0x68,0x15},
	{0x69,0x02},
	{0x6A,0x15},
	{0x6B,0x02},
	{0x6C,0x15},
	{0x6D,0x02},
	{0x6E,0x11},
	{0x6F,0x00},
	{0x23,0x4C},
	{0x21,0x01},
	{0x22,0x01},
};

#define STEP_COUNTER_OVERFLOW 0x7FFF
#define STEP_COUNTER_OVERFLOW_MIN 0x0020
#define STEP_COUNTER_OVERFLOW_MAX 0x7FDF

#define STEP_COUNTER_WALK    0x01
#define STEP_COUNTER_RUN     0x02
#define STEP_COUNTER_CHANGE  0x04
#define STEP_COUNTER_WALK_DEB    0x08

#define STEP_COUNTER_PARA_CLEAR 0
#define STEP_COUNTER_PARA_HOLD 1

#define STEP_COUNTER_FIRST_REPORT 1000 /*200ms*/

#define STEP_COUNTER_SM1_INTR 0x08
#define STEP_COUNTER_SM2_INTR 0x04
#define STEP_COUNTER_STEP_COUNTER_OVERFLOW_INTR 0x80

#define INT_COUNTER_THRESHLOD 50

#define STEP_COUNTER_INVERVAL 13

void gpio_switch_setstate(int state);
/*static int lsm330_step_counter_change_config(struct lsm330_acc_data *acc);*/


static unsigned int lsm330_acc_odr_to_interval(struct lsm330_acc_data *acc,
				unsigned int odr);

/* sets default init values to be written in registers at probe stage */
static void lsm330_acc_set_init_register_values(struct lsm330_acc_data *acc)
{
	acc->resume_state[LSM330_RES_LC_L] = 0x00;
	acc->resume_state[LSM330_RES_LC_H] = 0x00;

	acc->resume_state[LSM330_RES_CTRL_REG1] = LSM330_INT_ACT_H;
	acc->resume_state[LSM330_RES_CTRL_REG2] = 0x00;
	acc->resume_state[LSM330_RES_CTRL_REG3] = 0x00;
	acc->resume_state[LSM330_RES_CTRL_REG4] = 0x87;
	acc->resume_state[LSM330_RES_CTRL_REG5] = 0x00;

    acc->resume_state[LSM330_RES_CTRL_REG6] = 0x00;
    acc->resume_state[LSM330_RES_FIFO_CTRL_A] = 0x00;
}

static int lsm330_acc_i2c_read(struct lsm330_acc_data *acc,
				u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		printk(KERN_ERR TAGE "%s i2c_transfer error: (%d %p %d) %d\n",__func__, acc->client->addr, buf, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_write(struct lsm330_acc_data *acc, u8 * buf,
								int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		printk(KERN_ERR TAGE "%s i2c_transfer error: (%d %p %d) %d\n",__func__, acc->client->addr, buf, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lsm330_acc_i2c_update(struct lsm330_acc_data *acc,
				u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 rdbuf[1] = { reg_address };
	u8 wrbuf[2] = { reg_address , 0x00 };

	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_i2c_read(acc, rdbuf, 1);
	if (!(err < 0)) {
		init_val = rdbuf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		wrbuf[1] = updated_val;
		err = lsm330_acc_i2c_write(acc, wrbuf, 1);
	}
	return err;
}

static int lsm330_acc_hw_init(struct lsm330_acc_data *acc)
{
	int err = -1;
	u8 buf[17];

	printk(KERN_INFO TAGI "%s: hw init start\n", LSM330_ACC_DEV_NAME);

	if(acc->moving_detect==1)
	{
        printk(KERN_INFO TAGI "%s eint disable\n ", __func__);
		buf[0]=0X18;
		lsm330_acc_i2c_read(acc, buf, 1);
		printk(KERN_INFO TAGI "%s 0X18 %x\n", __func__, buf[0]);  

    	buf[0]=0X5F;
    	lsm330_acc_i2c_read(acc, buf, 1);
    	printk(KERN_INFO TAGI "%s 0X5F %x\n", __func__, buf[0]);
		
		acc->moving_detect = 0;
	}
	
    if(acc->step_counter_enable)
    {
		printk(KERN_ERR TAGI "%s step counter is on so do nothing \n", __func__);
        return 0;
    }
	buf[0] = LSM330_WHO_AM_I;
	err = lsm330_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
	    printk(KERN_ERR TAGE "Error reading WHO_AM_I: is device "
		    "available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;

	if ((buf[0] != WHOAMI_LSM330_ACC)&&(buf[0] != WHOAMI_LIS3DSH_ACC)) {
	    printk(KERN_ERR TAGE "device unknown. Expected: 0x%x,or 0x%x,"
		    " Replies: 0x%x\n", WHOAMI_LSM330_ACC, WHOAMI_LIS3DSH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}


	buf[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
	buf[1] = acc->resume_state[LSM330_RES_LC_L];
	buf[2] = acc->resume_state[LSM330_RES_LC_H];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG2);
	buf[1] = acc->resume_state[LSM330_RES_CTRL_REG2];
	buf[2] = acc->resume_state[LSM330_RES_CTRL_REG3];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG4);
	buf[1] = acc->resume_state[LSM330_RES_CTRL_REG4];
	buf[2] = acc->resume_state[LSM330_RES_CTRL_REG1];
	err = lsm330_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;

	if(acc->use_batch)
	{
		buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG6);
		buf[1] = acc->resume_state[LSM330_RES_CTRL_REG6];
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto err_resume_state;

		buf[0] = (I2C_AUTO_INCREMENT | LSM330_FIFO_CTRL_A);
		buf[1] = acc->resume_state[LSM330_RES_FIFO_CTRL_A];
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto err_resume_state;
	}
    else
    {
		buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG6);
		buf[1] = 0x10;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto err_resume_state;

		buf[0] = (I2C_AUTO_INCREMENT | LSM330_FIFO_CTRL_A);
		buf[1] = 0x00;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto err_resume_state;
    }
	acc->hw_initialized = 1;
	printk(KERN_INFO TAGI "%s: hw init done\n", LSM330_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	printk(KERN_ERR TAGE "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm330_acc_device_power_off(struct lsm330_acc_data *acc)
{
	int err;
    u8 buf[3];

    if(!acc->step_counter_enable&&!acc->moving_detect)
    {
    	err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG4,
    					LSM330_ODR_MASK, LSM330_PM_OFF);	
    	if (err < 0)
    		printk(KERN_ERR TAGE "soft power off failed: %d\n", err);
    }	

    if(acc->use_batch)
    {
        err=lsm330_acc_flush_data(&acc->cdev, NOTREPORT);
		if (err < 0)
			printk(KERN_ERR TAGE "%s flush data failed\n", __func__);
		buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG6);
		buf[1] = 0x10;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			printk(KERN_ERR TAGE "soft power off  LSM330_CTRL_REG6 failed: %d\n", err);

		buf[0] = (I2C_AUTO_INCREMENT | LSM330_FIFO_CTRL_A);
		buf[1] = 0x00;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			printk(KERN_ERR TAGE "soft power off LSM330_FIFO_CTRL_A failed: %d\n", err);

    }
	if (acc->pdata->power_off) {
		if(acc->pdata->gpio_int1)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lsm330_acc_device_power_on(struct lsm330_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			printk(KERN_ERR TAGE
					"power_on failed: %d\n", err);
			return err;
		}
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = lsm330_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lsm330_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t lsm330_acc_isr1(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;
    static int cont_int_num = 0;
    static unsigned long last_int_time = 0;
    unsigned long int_time;
    unsigned long diff;
     
    
	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	printk(KERN_INFO TAGI "%s: isr1 queued\n", LSM330_ACC_DEV_NAME);

    int_time = jiffies;
    diff = (long)int_time - (long)last_int_time;
    if((diff *1000)/HZ < CONT_INT_TIME)
    {
        if(cont_int_num < CONT_INT_GATE)
            cont_int_num++;
        else
            printk(KERN_ERR TAGE "continuous interrupts error\n"); 
    }
    else
    {
        cont_int_num = 0;
    }
    last_int_time = int_time;

	return IRQ_HANDLED;
}

static irqreturn_t lsm330_acc_isr2(int irq, void *dev)
{
	struct lsm330_acc_data *acc = dev;
    static int cont_int_num = 0;
    static unsigned long last_int_time = 0;
    unsigned long int_time;
    unsigned long diff;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk(KERN_INFO TAGI "%s: isr2 queued\n", LSM330_ACC_DEV_NAME);

    int_time = jiffies;
    diff = (long)int_time - (long)last_int_time;
    if((diff *1000)/HZ < CONT_INT_TIME)
    {
        if(cont_int_num < CONT_INT_GATE)
            cont_int_num++;
        else
            printk(KERN_ERR TAGE "continuous interrupts error\n"); 
    }
    else
    {
        cont_int_num = 0;
    }
    last_int_time = int_time;

	return IRQ_HANDLED;
}

static void lsm330_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm330_acc_data *acc;
	acc = container_of(work, struct lsm330_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_int1_source(acc); */
	;
	/*  */
	printk(KERN_INFO TAGI "%s: IRQ1 triggered\n", LSM330_ACC_DEV_NAME);
	enable_irq(acc->irq1);
}

static void lsm330_acc_irq2_work_func(struct work_struct *work)
{

	struct lsm330_acc_data *acc;
	acc = container_of(work, struct lsm330_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm330_acc_get_tap_source(acc); */
	;
	/*  */

	printk(KERN_INFO TAGI "%s: IRQ2 triggered\n", LSM330_ACC_DEV_NAME);
	enable_irq(acc->irq2);
}

static int lsm330_acc_register_masked_update(struct lsm330_acc_data *acc,
		u8 reg_address, u8 mask, u8 new_bit_values, int resume_index)
{
	u8 config[2] = {0};
	u8 init_val, updated_val;
	int err;
	int step = 0;

	config[0] = reg_address;
	err = lsm330_acc_i2c_read(acc, config, 1);
	if (err < 0)
		goto error;
	init_val = config[0];
	init_val = acc->resume_state[resume_index];
	step = 1;
	updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
	config[0] = reg_address;
	config[1] = updated_val;
	printk(KERN_INFO TAGI "%s reg_address %x %x\n", __func__, reg_address, updated_val);
	err = lsm330_acc_i2c_write(acc, config, 1);
	if (err < 0)
		goto error;
	acc->resume_state[resume_index] = updated_val;

	return err;
	error:
		printk(KERN_ERR TAGE
			"register 0x%x update failed at step %d, error: %d\n",
				config[0], step, err);
	return err;
}

static int lsm330_acc_update_fs_range(struct lsm330_acc_data *acc,
								u8 new_fs_range)
{
	int err=-1;
	u16 sensitivity;

	switch (new_fs_range) {
	case LSM330_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LSM330_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LSM330_ACC_G_6G:
		sensitivity = SENSITIVITY_6G;
		break;
	case LSM330_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LSM330_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		printk(KERN_ERR TAGE "invalid g range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}
	/*if (atomic_read(&acc->enabled))*/ {
		/* Updates configuration register 1,
		* which contains g range setting */
		err = lsm330_acc_register_masked_update(acc, LSM330_CTRL_REG5,
			LSM330_ACC_FS_MASK, new_fs_range, LSM330_RES_CTRL_REG5);
		if(err < 0) {
			printk(KERN_ERR TAGE "update g range failed\n");
			return err;
		}
		else
			acc->sensitivity = sensitivity;
	}

	if(err < 0)
		printk(KERN_ERR TAGE "update g range not executed "
						"because the device is off\n");
	return err;
}


static int lsm330_acc_update_odr(struct lsm330_acc_data *acc,
							int poll_interval_ms)
{
	int err = 0;
	int i;
	u8 new_odr;

	if(poll_interval_ms > 66)
	{
	    poll_interval_ms = 66;
	//	acc->resume_state[LSM330_RES_CTRL_REG4] &= 0xf7;
	}
//	else
//	{
//		acc->resume_state[LSM330_RES_CTRL_REG4] |= 0x08;
//	}
    poll_interval_ms=10;

	acc->resume_state[LSM330_RES_CTRL_REG4] &= 0xf7;
	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i >= 0; i--) {
		if (lsm330_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	new_odr = lsm330_acc_odr_table[i].mask;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	/*if (atomic_read(&acc->enabled))*/ {
		err = lsm330_acc_register_masked_update(acc,
			LSM330_CTRL_REG4, LSM330_ODR_MASK, new_odr,
							LSM330_RES_CTRL_REG4);
	}

	acc->chip_sample_time = lsm330_acc_odr_to_interval(acc, acc->resume_state[LSM330_RES_CTRL_REG4]);

	if(err < 0)
		printk(KERN_ERR TAGE "update odr failed\n");
	return err;
}

static int lsm330_acc_register_write(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;
	u8 write_buf[2];

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		write_buf[0] = reg_address;
		write_buf[1] = new_value;
		err = lsm330_acc_i2c_write(acc, write_buf, 1);
		if (err < 0)
			return err;
	return err;
}


#ifdef DEBUG
static int lsm330_acc_register_read(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lsm330_acc_i2c_read(acc, buf, 1);
	return err;
}

static int lsm330_acc_register_update(struct lsm330_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}
#endif


static int lsm330_acc_get_acceleration_data(struct lsm330_acc_data *acc,
		int *xyz)
{
	int err = -1;
    static int same_num_x = 0;
	static int same_num_y = 0;
	static int same_num_z = 0;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };
    s32 same_raw[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | OUT_AXISDATA_REG);
	err = lsm330_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

    same_raw[0]=((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
    same_raw[1]=((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
    same_raw[2]=((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));
    if(acc->sameraw.x == same_raw[0])
    {
        if(same_num_x < SAME_DATA_GATE)
            same_num_x++;
        else
            printk(KERN_ERR TAGE "same data x error, x = %d\n",same_raw[0]);
    }
    else
    {
        same_num_x = 0;
    }
	if(acc->sameraw.y == same_raw[1])
    {
        if(same_num_y < SAME_DATA_GATE)
            same_num_y++;
        else
            printk(KERN_ERR TAGE "same data y error, y = %d\n",same_raw[1]);
    }
    else
    {
        same_num_y = 0;
    }
	if(acc->sameraw.z == same_raw[2])
    {
        if(same_num_z < SAME_DATA_GATE)
            same_num_z++;
        else
            printk(KERN_ERR TAGE "same data z error, z = %d\n",same_raw[2]);
    }
    else
    {
        same_num_z = 0;
    }

    acc->sameraw.x = same_raw[0];
    acc->sameraw.y = same_raw[1];
    acc->sameraw.z = same_raw[2];

	hw_d[0] >>= 4;
	hw_d[1] >>= 4;
	hw_d[2] >>= 4;

	/*hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;*/
	if(SENSITIVITY_4G==acc->sensitivity)
    {
        hw_d[0] = hw_d[0]<<1;
        hw_d[1] = hw_d[1]<<1;
        hw_d[2] = hw_d[2]<<1;
    }   

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));
   
    acc->lastraw.x = xyz[0];
    acc->lastraw.y = xyz[1];
    acc->lastraw.z = xyz[2];
	acc->data_valid|=0xf0;

    if(SENSITIVITY_4G==acc->sensitivity)
    {
        acc->lastdata.x = (xyz[0]>>1) * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.y = (xyz[1]>>1) * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.z = (xyz[2]>>1) * acc->sensitivity * GRAVITY_EARTH/100;
        acc->data_valid|=0x0f;
    }
    else
    {
        acc->lastdata.x = xyz[0] * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.y = xyz[1] * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.z = xyz[2] * acc->sensitivity * GRAVITY_EARTH/100;
    	acc->data_valid|=0x0f;
    }
    if(LSM330_LOG_LEVEL_POLL==acc->debug)
    {
        printk(KERN_ERR TAGE "%s delay_ms %d %d %d %d\n", __func__, acc->delay_ms, xyz[0], xyz[1], xyz[2]);
    }

#ifdef DEBUG
//	printk(KERN_INFO TAGI" %s read x=%d, y=%d, z=%d\n",
//			LSM330_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	return 0;
}

#define Y_MAX_THRESHOLD	(-500)
#define Z_MAX_THRESHOLD	500
#define Z_MIN_THRESHOLD	(-700)
static struct wake_lock ts_judge_phone_direction_wakelock;
extern bool (*acc_for_ts_judge_dir)(void);
bool lsm330_for_ts_judge_dir(void)
{
	int result = 0;
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *priv; 
	int acc[3] = {0};

    if(!client)
		return false;
	priv = i2c_get_clientdata(client); 
	wake_lock_timeout(&ts_judge_phone_direction_wakelock, HZ/2); 

    mutex_lock(&priv->lock);
    if(!atomic_read(&priv->enabled)&&!priv->step_counter_enable&&!priv->moving_detect) 
    {
        result = lsm330_acc_i2c_update(priv, LSM330_CTRL_REG4,
                    LSM330_ODR_MASK, LSM330_ODR400);
		mdelay(12);
    }
	result = lsm330_acc_get_acceleration_data(priv, acc);
    if(!atomic_read(&priv->enabled)&&!priv->step_counter_enable&&!priv->moving_detect) 
    {
        result = lsm330_acc_i2c_update(priv, LSM330_CTRL_REG4,
                    LSM330_ODR_MASK, LSM330_PM_OFF);
    }
	mutex_unlock(&priv->lock);
	if (result < 0)
	{
		printk(KERN_ERR TAGE "<<-GTP->>%s get_acceleration_data failed\n", __func__);
		return false;
	}

	printk(KERN_INFO TAGI "<<-GTP-INFO->>%s data is X(%d) Y(%d) Z(%d)\n", __func__, acc[0],acc[1], acc[2]);

	if (acc[1] < Y_MAX_THRESHOLD && (acc[2] > Z_MIN_THRESHOLD && acc[2] < Z_MAX_THRESHOLD)) 
	{
		printk(KERN_INFO TAGI "<<-GTP-INFO->>%s The phone is handstand\n", __func__);
		return true;
	} else {
		printk(KERN_INFO TAGI "<<-GTP-INFO->>%s The phone is NOT handstand\n", __func__);
		return false;
	}

}

#ifdef LSM330_BATCH_MODE
/*
 * Turn ON/OFF FIFO by setting the FIFO_En bit,
 * FIFO must be enabled before activate FIFO mode.
 */
static int lsm330_acc_enable_fifo(struct i2c_client *client, bool enable)
{
	unsigned char buf[2];
	int error;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	buf[0] = LSM330_CTRL_REG6;
	error = lsm330_acc_i2c_read(acc, buf, 1);
	if (error < 0) {
		printk(KERN_ERR TAGE "read fifo enable reg error\n");
		return error;
	}
	if (enable)
		buf[1] = buf[0] | LSM330_ACC_FIFO_EN;
	else
		buf[1] = buf[0] & (~LSM330_ACC_FIFO_EN);
	buf[0] = LSM330_CTRL_REG6;
	error = lsm330_acc_i2c_write(acc, buf, 1);
	if (error < 0) {
		printk(KERN_ERR TAGE "write fifo enable reg error\n");
		return error;
	}
	acc->resume_state[LSM330_RES_CTRL_REG6] = buf[1];
	/*printk(KERN_INFO TAGI "lsm330 acc REG5 = %#x\n", buf[1]);*/
	return error;
}

/*
 * Activate FIFO mode by setting FIFO mode bits.
 */
static int lsm330_acc_set_fifo_mode(struct i2c_client *client,
				unsigned char fifo_mode)
{
	unsigned char buf[2];
	int error;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	buf[0] = LSM330_FIFO_CTRL_A;
	error = lsm330_acc_i2c_read(acc, buf, 1);
	if (error < 0) {
		printk(KERN_ERR TAGE "read fifo reg error\n");
		return error;
	}
	buf[1] = buf[0] & (~LSM330_FIFO_MODE_MASK);
	buf[1] |= (unsigned char)(fifo_mode << LSM330_FIFO_MODE_SHIFT);

	buf[0] = LSM330_FIFO_CTRL_A;
	error = lsm330_acc_i2c_write(acc, buf, 1);
	if (error < 0) {
		printk(KERN_ERR TAGE "write fifo mode error\n");
		return error;
	}
	acc->resume_state[LSM330_RES_FIFO_CTRL_A] = buf[1];
	/*printk(KERN_INFO TAGI 
		"lsm330_acc_set_fifo_mode: lis3dh fifo_reg = %#x\n", buf[1]);*/
	return error;
}

static int lsm330_acc_enable_batch(struct lsm330_acc_data *acc, bool en)
{
	struct i2c_client *client = acc->client;
	int err;

	if (en) {
		err = lsm330_acc_enable_fifo(client, true);
		if (err < 0) {
			printk(KERN_ERR TAGE 
				"enable batch: cannot enable FIFO\n");
			goto exit;
		}
		err = lsm330_acc_set_fifo_mode(client, LSM330_ACC_STREAM_MODE);
		if (err < 0) {
			printk(KERN_ERR TAGE 
				"enable batch: cannot set FIFO mode\n");
			goto exit;
		}
	} else {
		err = lsm330_acc_set_fifo_mode(client, LSM330_ACC_BYPASS_MODE);
		if (err < 0) {
			printk(KERN_ERR TAGE 
				"enable batch: cannot set FIFO mode\n");
			goto exit;
		}
		err = lsm330_acc_enable_fifo(client, false);
		if (err < 0) {
			printk(KERN_ERR TAGE 
				"enable batch: cannot enable FIFO\n");
			goto exit;
		}
	}

exit:
	return err;
}

static int lsm330_latency_set(struct sensors_classdev *cdev,
					unsigned int max_latency)
{
	struct lsm330_acc_data *acc = container_of(cdev,
		struct lsm330_acc_data, cdev);
	int ret=0;

    if(!acc->acc_fifo)
    {
    	printk(KERN_ERR TAGE "%s no fifo memory\n", __func__);
		return -EINVAL;
	}
	
	if((0<acc->chip_sample_time)&&(LSM330_FIFO_SIZE<=max_latency/acc->chip_sample_time))
		max_latency=acc->chip_sample_time*LSM330_FIFO_SIZE;
    mutex_lock(&acc->lock);
	acc->fifo_timeout_ms = max_latency;
	acc->use_batch = max_latency ? true : false;
	
	ret = lsm330_acc_enable_batch(acc, max_latency);
	if (ret) {
		printk(KERN_ERR TAGE "enable batch:%d failed\n", max_latency);
	}
	mutex_unlock(&acc->lock);

	return ret;
}

static inline s64 lsm330_acc_get_time_ns(void)
{
	s64 ts;
	s32 time_data_sec,time_data_ns;
	ktime_t timestamp;
	
	timestamp = ktime_get_boottime();
	time_data_sec = ktime_to_timespec(timestamp).tv_sec;
	time_data_ns = ktime_to_timespec(timestamp).tv_nsec;
	ts = 1000000000LL*time_data_sec + time_data_ns;
	return ts;
} 
static unsigned int lsm330_acc_odr_to_interval(struct lsm330_acc_data *acc,
				unsigned int odr)
{
	unsigned int odr_mask;
	unsigned int i;

	odr_mask = odr & LSM330_ACC_ODR_MASK;

	for (i = ARRAY_SIZE(lsm330_acc_odr_table) - 1; i > 0; i--) {
		if (lsm330_acc_odr_table[i].mask == odr_mask)
			break;
	}

	return lsm330_acc_odr_table[i].cutoff_ms;
}
static int lsm330_acc_get_fifo_lvl(struct lsm330_acc_data *acc, u8 *fifo_c)
{
	int error = 0;
	unsigned char buf[2];
	unsigned int fifo_lvl;

	buf[0] = LSM330_FIFO_SRC_A;
	error = lsm330_acc_i2c_read(acc, buf, 1);
	if (error < 0) {
		printk(KERN_ERR TAGE "read fifo level error\n");
		return error;
	}
	fifo_lvl = buf[0] & LSM330_ACC_FIFO_SRC_DATA_CNT_MASK;
	if ((fifo_lvl == LSM330_ACC_FIFO_MAX_CNT)
		&& (buf[0] | LSM330_ACC_FIFO_SRC_OVRN_MASK))
		fifo_lvl = LSM330_ACC_FIFO_MAX_CNT + 1;

	*fifo_c=fifo_lvl;
	return error;
}
static int lsm330_acc_get_acceleration_fifo_data(struct lsm330_acc_data *acc,
		u8 *acc_data, int fifo_count)
{
	int err = 0, bat_err=0;

    if(!acc->acc_fifo)
    {
    	printk(KERN_ERR TAGE "%s no fifo memory\n", __func__);
		return -EINVAL;
	}

	acc_data[0] = (I2C_AUTO_INCREMENT | OUT_AXISDATA_REG);
	err = lsm330_acc_i2c_read(acc, acc_data, 6*fifo_count);

    bat_err = lsm330_acc_set_fifo_mode(acc->client, LSM330_ACC_BYPASS_MODE);
    if (bat_err < 0) {
        printk(KERN_ERR TAGE 
            "%s cannot set bypass mode\n", __func__);
    }
    bat_err = lsm330_acc_set_fifo_mode(acc->client, LSM330_ACC_STREAM_MODE);
    if (bat_err < 0) {
        printk(KERN_ERR TAGE 
            "%s cannot set stream mode\n", __func__);
    }

	return err;
}

static int lsm330_acc_flush_data(struct sensors_classdev *sensors_cdev, unsigned char isreport)
{
	struct lsm330_acc_data *acc = container_of(sensors_cdev,
			struct lsm330_acc_data, cdev);
//#ifdef CONFIG_ARM64
	u64 time_current_ns, time_data_sec, time_data_first, time_one_sample;
//#else
//	u32 time_current_ns, time_data_sec, time_data_first, time_one_sample;
//#endif
	s32 tv_sec , tv_nsec;
	/*u32 time_ms;*/
	int i, err=0, xyz[3] = {0}, hw_d[3] = { 0 };
	u8 fifo_cnt=0;
    s32 same_raw[3] = { 0 };
    static int same_batch_x = 0;
	static int same_batch_y = 0;
	static int same_batch_z = 0;
    static int continue_batch = 0;

    if(!acc->acc_fifo)
    {
    	printk(KERN_ERR TAGE "%s no fifo memory\n", __func__);
		return -EINVAL;
	}

	acc->time_current_ns = time_current_ns = 
	1000000000LL*ktime_to_timespec(acc->ktime_acc).tv_sec + ktime_to_timespec(acc->ktime_acc).tv_nsec;
	/*time_ms = lsm330_acc_odr_to_interval(acc, acc->resume_state[LSM330_RES_CTRL_REG4]);*/
	err = lsm330_acc_get_fifo_lvl(acc, &fifo_cnt);
	if(err)
	{
		printk(KERN_ERR TAGE "%s err %d\n", __func__, err);
		err = -EINVAL;
		goto exit;
	}
	if((LSM330_FIFO_SIZE==fifo_cnt) || (LSM330_FIFO_EMPTY==fifo_cnt))
		printk(KERN_INFO TAGI "%s fifo full or empty %d\n", __func__, fifo_cnt);
    time_one_sample = acc->pdata->poll_interval*LSM330_TIME_MS_TO_NS;
	time_data_first = time_data_sec = time_current_ns -((u64)acc->chip_sample_time * LSM330_TIME_MS_TO_NS * (fifo_cnt-1));
	/*printk(KERN_ERR TAGE "TS: time_current_ns=%llu, interval=%u fifo_cnt=%d\n",
			time_current_ns, acc->chip_sample_time, fifo_cnt);*/

	err = lsm330_acc_get_acceleration_fifo_data(acc, acc->acc_fifo, fifo_cnt);
	if (err < 0) {
		printk(KERN_ERR TAGE 
				"get_acceleration_data failed\n");
		goto exit;
	}
	for (i = 0; i < fifo_cnt; i++)
	{
        hw_d[0] = ((s16) ((*(acc->acc_fifo+i*6+1) << 8) | *(acc->acc_fifo+i*6+0)));
        hw_d[1] = ((s16) ((*(acc->acc_fifo+i*6+3) << 8) | *(acc->acc_fifo+i*6+2)));
        hw_d[2] = ((s16) ((*(acc->acc_fifo+i*6+5) << 8) | *(acc->acc_fifo+i*6+4)));
        
        same_raw[0]=((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
               : (hw_d[acc->pdata->axis_map_x]));
        same_raw[1]=((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
               : (hw_d[acc->pdata->axis_map_y]));
        same_raw[2]=((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
               : (hw_d[acc->pdata->axis_map_z]));
        if(acc->sameraw.x == same_raw[0])
        {
            if(same_batch_x < SAME_DATA_GATE)
                same_batch_x++;
            else
                printk(KERN_ERR TAGE "same data x error, x = %d\n",same_raw[0]);
        }
        else
        {
            same_batch_x = 0;
        }
        if(acc->sameraw.y == same_raw[1])
        {
            if(same_batch_y < SAME_DATA_GATE)
                same_batch_y++;
            else
                printk(KERN_ERR TAGE "same data y error, y = %d\n",same_raw[1]);
        }
        else
        {
            same_batch_y = 0;
        }
        if(acc->sameraw.z == same_raw[2])
        {
            if(same_batch_z < SAME_DATA_GATE)
                same_batch_z++;
            else
                printk(KERN_ERR TAGE "same data z error, z = %d\n",same_raw[2]);
        }
        else
        {
            same_batch_z = 0;
        }

		hw_d[0] >>= 4;
		hw_d[1] >>= 4;
		hw_d[2] >>= 4;
    	if(SENSITIVITY_4G==acc->sensitivity)
        {
            hw_d[0] = hw_d[0]<<1;
            hw_d[1] = hw_d[1]<<1;
            hw_d[2] = hw_d[2]<<1;
        }   
		xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
			   : (hw_d[acc->pdata->axis_map_x]));
		xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
			   : (hw_d[acc->pdata->axis_map_y]));
		xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
			   : (hw_d[acc->pdata->axis_map_z]));

		time_data_sec = time_data_first+acc->chip_sample_time * i *LSM330_TIME_MS_TO_NS;
        if((acc->time_data_sec+time_one_sample)>time_data_sec)
        {
            if(LSM330_FIFO_SIZE<++continue_batch)
            {
//#ifdef CONFIG_ARM64
                printk(KERN_ERR TAGI "%s continue time %d %llu %llu %llu\n", __func__,
                    continue_batch,
                    acc->time_data_sec,
                    time_one_sample,
                    time_data_sec);
/*#else
				printk(KERN_ERR TAGI "%s continue time %d %llu %u %u\n", __func__,
                    continue_batch,
                    acc->time_data_sec,
                    time_one_sample,
                    time_data_sec);
#endif*/
            }
            continue;
        }
        continue_batch=0;
        acc->time_data_sec=time_data_sec;
		if(time_data_sec>time_current_ns)
			printk(KERN_ERR TAGI "%s time err\n", __func__);
	//	printk(KERN_INFO TAGI "%s read time_data_sec=%llu\n",__func__, time_data_sec);
#ifdef CONFIG_ARM64
		tv_sec = (s32)(time_data_sec/1000000000LL);
		tv_nsec = (s32)(time_data_sec%1000000000LL);
#else		
		tv_nsec = do_div(time_data_sec,1000000000);
		tv_sec = (s32)time_data_sec;
#endif
		/*printk(KERN_INFO TAGI "%s time_data_sec %8.1llu \n", __func__, time_data_sec);	*/	
		/*printk(KERN_INFO TAGI "%s x %8.1d y %8.1d z %8.1d\n", __func__, xyz[0], xyz[1], xyz[2]);*/
        if(isreport)
        {
			input_report_abs(acc->input_dev, ABS_X, xyz[0] - acc->calibration_data.x);
			input_report_abs(acc->input_dev, ABS_Y, xyz[1] - acc->calibration_data.y);
			input_report_abs(acc->input_dev, ABS_Z, xyz[2] - acc->calibration_data.z);
			input_event(acc->input_dev,EV_SYN, SYN_TIME_SEC,tv_sec);
			input_event(acc->input_dev,EV_SYN, SYN_TIME_NSEC,tv_nsec);
	//		printk(KERN_INFO TAGI "%s read time_s=%d, time_ns=%d \n",
	//			__func__, tv_sec,tv_nsec);
			input_sync(acc->input_dev);
		     if(LSM330_LOG_LEVEL_POLL==acc->debug)
		    {
		        printk(KERN_ERR TAGE "%s delay_ms %d %d %d %d\n", __func__, acc->delay_ms, xyz[0], xyz[1], xyz[2]);
		    }
        }
	}

    acc->lastraw.x = xyz[0];
    acc->lastraw.y = xyz[1];
    acc->lastraw.z = xyz[2];
	acc->data_valid|=0xf0;

    if(SENSITIVITY_4G==acc->sensitivity)
    {
        acc->lastdata.x = (xyz[0]>>1) * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.y = (xyz[1]>>1) * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.z = (xyz[2]>>1) * acc->sensitivity * GRAVITY_EARTH/100;
        acc->data_valid|=0x0f;
    }
    else
    {    
        acc->lastdata.x = xyz[0] * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.y = xyz[1] * acc->sensitivity * GRAVITY_EARTH/100;
        acc->lastdata.z = xyz[2] * acc->sensitivity * GRAVITY_EARTH/100;
    	acc->data_valid|=0x0f;
    }

	for(i = 0; i < fifo_cnt*6; i++)
	{
		*(acc->acc_fifo+i)=0;
	}
exit:
	return err;
}

static int lsm330_acc_flush(struct sensors_classdev *sensors_cdev)
{
	struct lsm330_acc_data *acc = container_of(sensors_cdev,
			struct lsm330_acc_data, cdev);
	int err=0;
	
	mutex_lock(&acc->lock);
	if(acc->use_batch)err=lsm330_acc_flush_data(sensors_cdev, REPORT);
	mutex_unlock(&acc->lock);

	return err;
}
#endif

static int lsm330_acc_enable(struct lsm330_acc_data *acc)
{
	int err;
	ktime_t ktime;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lsm330_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		if(acc->use_batch)
			acc->delay_ms=acc->fifo_timeout_ms;
		else
			acc->delay_ms=acc->pdata->poll_interval;
		poll_delay_num = 0;
        if(acc->time_data_sec)
            acc->time_data_sec=0;
		ktime = ktime_set(0,acc->delay_ms * NSEC_PER_MSEC);
		hrtimer_start(&acc->timer_acc, ktime, HRTIMER_MODE_REL);
//		queue_delayed_work(acc->data_wq, &acc->input_work,
//		    msecs_to_jiffies(acc->delay_ms));
//		schedule_delayed_work(&acc->input_work,
//			msecs_to_jiffies(acc->delay_ms));
	}
    acc->data_valid=0;
	return 0;
}

static int lsm330_acc_disable(struct lsm330_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
	//	cancel_delayed_work(&acc->input_work);
		hrtimer_cancel(&acc->timer_acc);
		lsm330_acc_device_power_off(acc);
        if(acc->use_batch)acc->use_batch=false;
	}
    acc->data_valid=0;
	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
					struct device_attribute *attr,
								char *buf)
{
	int val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	err = lsm330_acc_update_odr(acc, interval_ms);
	if(err >= 0)
	{
		acc->pdata->poll_interval = interval_ms;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->fs_range ;
	switch(val) {
	case LSM330_ACC_G_2G:
		range = 2;
		break;
	case LSM330_ACC_G_4G:
		range = 4;
		break;
	case LSM330_ACC_G_6G:
		range = 6;
		break;
	case LSM330_ACC_G_8G:
		range = 8;
		break;
	case LSM330_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	int err;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	switch(val) {
		case 2:
			range = LSM330_ACC_G_2G;
			break;
		case 4:
			range = LSM330_ACC_G_4G;
			break;
		case 6:
			range = LSM330_ACC_G_6G;
			break;
		case 8:
			range = LSM330_ACC_G_8G;
			break;
		case 16:
			range = LSM330_ACC_G_16G;
			break;
		default:
			return -1;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_update_fs_range(acc, range);
	if(err >= 0)
	{
		acc->pdata->fs_range = range;
	}
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm330_acc_enable(acc);
	else
		lsm330_acc_disable(acc);

	return size;
}


static int lsm330_acc_interrupt_enable_control(struct lsm330_acc_data *acc,
								u8 settings)
{
	u8 val1;
	u8 val2 = LSM330_INTEN_ON;
	u8 mask1 = (LSM330_INT1_EN_MASK | LSM330_INT2_EN_MASK);
	int err = -1;
	settings = settings & 0x03;

	switch ( settings ) {
	case LSM330_INT1_DIS_INT2_DIS:
		val1 = (LSM330_INT1_EN_OFF | LSM330_INT2_EN_OFF);
		val2 = LSM330_INTEN_OFF;
		break;
	case LSM330_INT1_DIS_INT2_EN:
		val1 = (LSM330_INT1_EN_OFF | LSM330_INT2_EN_ON);
		break;
	case LSM330_INT1_EN_INT2_DIS:
		val1 = (LSM330_INT1_EN_ON | LSM330_INT2_EN_OFF);
		break;
	case LSM330_INT1_EN_INT2_EN:
		val1 = ( LSM330_INT1_EN_ON | LSM330_INT2_EN_ON);
		break;
	default :
		printk(KERN_ERR TAGE "invalid interrupt setting : 0x%02x\n",settings);
		return err;
	}
	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG3, mask1, val1, LSM330_RES_CTRL_REG3);
	if (err < 0 )
		return err;

	err = lsm330_acc_register_masked_update(acc,
		LSM330_CTRL_REG1, LSM330_INTEN_MASK, val2,
							LSM330_RES_CTRL_REG1);
	if (err < 0 )
			return err;
	acc->interrupts_enable_setting = settings;
#ifdef DEBUG
	printk(KERN_INFO TAGI "interrupt setting : 0x%02x\n",acc->interrupts_enable_setting);
#endif
	return err;
}

static ssize_t attr_get_interr_enable(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	u8 val;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->interrupts_enable_setting;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_interr_enable(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int err = -1;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;


	if ( val < 0x00 || val > LSM330_INT1_EN_INT2_EN){
#ifdef DEBUG
		printk(KERN_ERR TAGE "invalid interrupt setting, val: %d\n",val);
#endif
		return -EINVAL;
	}

	mutex_lock(&acc->lock);
	err = lsm330_acc_interrupt_enable_control(acc, val);
	mutex_unlock(&acc->lock);
	if (err < 0)
		return err;
	return size;
}

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lsm330_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lsm330_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static ssize_t
lsm330_step_counter_mode_set_show(struct device_driver *dev_driver, char *buf)
{
    int value;
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);
    value = acc->sc_deb_or_only_walk;  
    return sprintf(buf, "%d\n", value);
}

static ssize_t
lsm330_step_counter_mode_set_store(struct device_driver *dev_driver, const char *buffer, size_t count)
{
	unsigned int val = simple_strtoul(buffer, NULL, 10);
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);
	printk(KERN_INFO TAGI "%s\n", __func__);
    
	/*if (strict_strtoul(buffer, 16, &val))
		return -EINVAL;*/

    if(STEP_COUNTER_ADD_WALK_DEBOUNCE!=val &&
        STEP_COUNTER_ONLY_WALK!=val)
        return -EINVAL;

    printk(KERN_INFO TAGI "%s mode %d \n", __func__, val);
	mutex_lock(&acc->lock);
    acc->sc_deb_or_only_walk = val;
    /*lsm330_step_counter_change_config(acc);*/
	mutex_unlock(&acc->lock);
 
	return count;
}


static struct st_reg_val sensor_reg[]={
	{0x0f,10},
	{0x1a,12},
	{0x27,9},
	{0x40,24},
	{0x59,39}
};

static ssize_t sensor_writereg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);
	//int err;
	u8 data;
	int i,j;
	
	for(i=0;i<5;i++)
	{
		for(j=0;j<sensor_reg[i].reg_val;j++)
		{
			data=sensor_reg[i].reg_add+j;		
			lsm330_acc_i2c_read(acc, &data, 1);		
			printk(KERN_INFO TAGI "%s reg_add:%x  reg_val:%x \n", __func__, sensor_reg[i].reg_add+j, data);
		}
	}
	/*TODO: error need to be managed */
    if(acc->debug)
    {
        printk(KERN_INFO TAGI "apk sample %d chip sample %d batch time %d poll timer %d eanble %d\n",
            acc->pdata->poll_interval, acc->chip_sample_time, acc->fifo_timeout_ms, acc->delay_ms, atomic_read(&acc->enabled));
        printk(KERN_INFO TAGI "batch last reprot data time %llu\n", acc->time_data_sec);
    }
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t sensor_writereg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{	
	u8 data[2];
	int tem;	
	struct lsm330_acc_data *acc = dev_get_drvdata(dev);	
	tem = simple_strtoul(buf, NULL, 16);	
	data[0] = (u8)((tem&0xff00)>>8);	
	data[1] = (u8)(tem&0x00ff);
	if(!lsm330_acc_i2c_write(acc, data, 1))	
	{      
		printk(KERN_INFO TAGI "lsm330 reg==>[%x]/[%x]\n",data[0],data[1]);
	}	
	return size;
}

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0644, attr_get_polling_rate,
							attr_set_polling_rate),
	__ATTR(range, 0644, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0644, attr_get_enable, attr_set_enable),
	__ATTR(enable_interrupt_output, 0644, attr_get_interr_enable,
							attr_set_interr_enable),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
	__ATTR(get_reg, 0644, sensor_writereg_get,sensor_writereg_set),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	printk(KERN_ERR TAGE "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lsm330_acc_input_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

    static unsigned long last_poll_time = 0;
    unsigned long poll_time;
    unsigned long diff;
 //   ktime_t timestamp;

	acc = container_of(work, struct lsm330_acc_data, input_work);

	mutex_lock(&acc->lock);
	if(acc->use_batch)
	{
		err = lsm330_acc_flush_data(&acc->cdev, REPORT);
		if (err < 0)
			printk(KERN_ERR TAGE "lsm330_acc_flush_data failed\n");
	}
	else
	{
		err = lsm330_acc_get_acceleration_data(acc, xyz);
		if (err < 0)
			printk(KERN_ERR TAGE "get_acceleration_data failed\n");
		else
		{
		//	timestamp = ktime_get_boottime();
			input_report_abs(acc->input_dev, ABS_X, xyz[0] - acc->calibration_data.x);
			input_report_abs(acc->input_dev, ABS_Y, xyz[1] - acc->calibration_data.y);
			input_report_abs(acc->input_dev, ABS_Z, xyz[2] - acc->calibration_data.z);
			input_event(acc->input_dev,EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(acc->ktime_acc).tv_sec);
			input_event(acc->input_dev,EV_SYN, SYN_TIME_NSEC,
				ktime_to_timespec(acc->ktime_acc).tv_nsec);
//			printk(KERN_INFO TAGI "%s read time_s=%ld, time_ns=%ld !!!\n",
//			__func__, ktime_to_timespec(acc->ktime_acc).tv_sec,ktime_to_timespec(acc->ktime_acc).tv_nsec);
			input_sync(acc->input_dev);
		}
	}
    /*printk(KERN_INFO TAGI"%s poll_interval %d\n", __func__, acc->pdata->poll_interval);*/
	if(acc->use_batch)
		acc->delay_ms=acc->fifo_timeout_ms;
	else
		acc->delay_ms=acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);

    poll_time = jiffies;
    diff = (long)poll_time - (long)last_poll_time;
    if((diff *1000)/HZ > (acc->delay_ms) * POLL_DELAY_TIMES)
    {
        if(poll_delay_num < POLL_DELAY_GATE)
            poll_delay_num++;
        else
            printk(KERN_ERR TAGE "poll delay long error:time = %ld\n",(diff *1000)/HZ);
    }
    else
    {
        poll_delay_num = 0;
    }
    last_poll_time = poll_time;
}

 /* lsm330_acc_timer_handle() - timer_acc handle to schedule workerthread
 * @hrtimer: the hrtimer struct
 *
 * handle gets called based on the poll time to schedule worker thread
 */
static enum hrtimer_restart lsm330_acc_timer_handle(struct hrtimer *hrtimer)
{
	ktime_t ktime;
	struct lsm330_acc_data *acc;
	acc = container_of(hrtimer, struct lsm330_acc_data, timer_acc);

	ktime = acc->ktime_acc =ktime_get_boottime();
//	printk(KERN_INFO TAGI "%s read time_s=%ld, time_ns=%ld !!!\n",
//			__func__, ktime_to_timespec(acc->ktime_acc).tv_sec,ktime_to_timespec(acc->ktime_acc).tv_nsec);
	if(atomic_read(&acc->enabled))
	{
		queue_work(acc->data_wq, &acc->input_work);
	}
	 
	if(acc->use_batch)
		acc->delay_ms=acc->fifo_timeout_ms;
	else
		acc->delay_ms=acc->pdata->poll_interval;
 
	ktime = ktime_set(0,acc->delay_ms * NSEC_PER_MSEC);
	hrtimer_start(&acc->timer_acc, ktime, HRTIMER_MODE_REL);

    if(LSM330_LOG_LEVEL_POLL==acc->debug)
    {
        printk(KERN_ERR TAGE "%s delay_ms %d", __func__, acc->delay_ms);
    }
    return HRTIMER_NORESTART;
}
 
int lsm330_acc_input_open(struct input_dev *input)
{
	struct lsm330_acc_data *acc = input_get_drvdata(input);

	return lsm330_acc_enable(acc);
}

void lsm330_acc_input_close(struct input_dev *dev)
{
	struct lsm330_acc_data *acc = input_get_drvdata(dev);

	lsm330_acc_disable(acc);
}

static int lsm330_acc_validate_pdata(struct lsm330_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		printk(KERN_ERR TAGE "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		printk(KERN_ERR TAGE "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		printk(KERN_ERR TAGE "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330_acc_input_init(struct lsm330_acc_data *acc)
{
	int err;
	
	INIT_WORK(&acc->input_work, lsm330_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lsm330_acc_input_open;
	acc->input_dev->close = lsm330_acc_input_close;
	acc->input_dev->name = LSM330_ACC_DEV_NAME;

	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	/*	next is used for interruptA sources data if the case */
//input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
//input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


	err = input_register_device(acc->input_dev);
	if (err) {
		printk(KERN_ERR TAGE
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lsm330_acc_input_cleanup(struct lsm330_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int lsm330_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct lsm330_acc_data *acc = container_of(sensors_cdev,
		struct lsm330_acc_data, cdev);
	int err;
    printk(KERN_INFO TAGI "%s delay_msec %d\n", __func__, delay_msec);
	mutex_lock(&acc->lock);
	if(acc->use_batch)
	{
		err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG4,
						LSM330_ODR_MASK, LSM330_PM_OFF);
		lsm330_acc_flush_data(sensors_cdev, REPORT);
	}
	acc->pdata->poll_interval = delay_msec;
	err = lsm330_acc_update_odr(acc, delay_msec);
	mutex_unlock(&acc->lock);
	return err;
}

static int lsm330_acc_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct lsm330_acc_data *acc = container_of(sensors_cdev,
		struct lsm330_acc_data, cdev);
	int err;
    printk(KERN_INFO TAGI "%s enable %d\n", __func__, enable);
    mutex_lock(&acc->lock);
	if (enable)
		err = lsm330_acc_enable(acc);
	else
		err = lsm330_acc_disable(acc);
    mutex_unlock(&acc->lock);
	return err;
}
#if 0
static int lsm330_step_counter_change_config(struct lsm330_acc_data *acc)
{
    int err=0;
    /*unsigned char only_walk_size;*/
    /*unsigned char walk_size=sizeof(lsm330_step_counter_walk_only)/sizeof(struct st_reg_val);*/
    /*unsigned char walk_change_size;*/
    /*struct st_reg_val *walk_p;*/

    if(STEP_COUNTER_ONLY_WALK==acc->sc_deb_or_only_walk)
    {
       lsm330_step_counter_init[1].reg_val=0x44;
    }
    else if(STEP_COUNTER_ADD_WALK_DEBOUNCE==acc->sc_deb_or_only_walk)
    {
       lsm330_step_counter_init[1].reg_val=0x4C;
    }
  
    return err;
}
#endif
static int lsm330_step_counter_init_config(struct lsm330_acc_data *acc)
{
	int i = 0, err=0;
	int size = sizeof(lsm330_step_counter_init)/sizeof(struct st_reg_val);

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }

    if(STEP_COUNTER_ONLY_WALK==acc->sc_deb_or_only_walk)
    {
       lsm330_step_counter_init[1].reg_val=0x44;
    }
    else
    {
       lsm330_step_counter_init[1].reg_val=0x4C;
    }

	for(i=0; i<size; i++)
	{
		err=lsm330_acc_register_write(acc, NULL, 
			lsm330_step_counter_init[i].reg_add, lsm330_step_counter_init[i].reg_val);
        if(err<0)
            break;
	}
    printk(KERN_ERR TAGI "%s counter %u counter_last %u overflow %u reg %u reg_last %u err %d\n", __func__, 
        acc->step_counter,acc->step_counter_last,acc->step_counter_overflow,
        acc->step_reg_value,acc->step_reg_value_last, err);
    if(0<=err)
    {
        if(acc->step_counter>acc->step_counter_last)
            acc->step_counter_last=acc->step_counter;
        else if(STEP_COUNTER_OVERFLOW<acc->step_counter_last)
            acc->step_counter_last+=acc->step_counter;
        acc->step_counter=0;
        acc->step_counter_overflow=0;
        acc->step_reg_value=0;
        acc->step_reg_value_last=0;
        acc->step_counter_delta=0;
        acc->step_counter_deb_counter=0;
        acc->step_counter_deb=0;
    }

    return err;
}

/*
static int lsm330_step_counter_walk_mode(struct lsm330_acc_data *acc)
{
	int i = 0, err=0;
	int size = sizeof(lsm330_step_counter_walk)/sizeof(struct st_reg_val);

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }
    
	for(i=0; i<size; i++)
	{
		lsm330_acc_register_write(acc, NULL, 
			lsm330_step_counter_walk[i].reg_add, lsm330_step_counter_walk[i].reg_val);
        if(err<0)
            break;
    }
    if(0<=err)acc->step_walk_run=STEP_COUNTER_WALK;
    printk(KERN_ERR TAGI "%s okay\n", __func__);
    return err;
}

#ifdef STEP_COUNTER_RUM_MODE
static int lsm330_step_counter_run_mode(struct lsm330_acc_data *acc)
{
	int i = 0, err=0;
	int size = sizeof(lsm330_step_counter_run)/sizeof(struct st_reg_val);

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }

	for(i=0; i<size; i++)
	{
		lsm330_acc_register_write(acc, NULL, 
			lsm330_step_counter_run[i].reg_add, lsm330_step_counter_run[i].reg_val);
        if(err<0)
            break;
    }
    if(0<=err)acc->step_walk_run=STEP_COUNTER_RUN;
    printk(KERN_ERR TAGI "%s okay\n", __func__);
    return err;
}
#endif
*/
static int lsm330_step_counter_set_mode(struct lsm330_acc_data *acc, unsigned char mode)
{
	int i = 0, err=0, size=0;
	int size_run = sizeof(lsm330_step_counter_run)/sizeof(struct st_reg_val);
    int size_walk = sizeof(lsm330_step_counter_walk)/sizeof(struct st_reg_val);
    int size_walk_debounce = sizeof(lsm330_step_counter_walk_det)/sizeof(struct st_reg_val);
    struct st_reg_val *reg_val_p=NULL;

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }

    if((STEP_COUNTER_RUN!=mode)&&(STEP_COUNTER_WALK!=mode)&&(STEP_COUNTER_WALK_DEB!=mode))
    {
        printk(KERN_ERR TAGE "%s no this mode %d\n", __func__, mode);
        return -EINVAL;
    }

    if(STEP_COUNTER_RUN==mode)
    {
        size=size_run;
        reg_val_p=lsm330_step_counter_run;
        acc->step_counter_deb=0;
    }
    else if(STEP_COUNTER_WALK==mode)
    {
        size=size_walk;
        if(STEP_COUNTER_ADD_WALK_DEBOUNCE==acc->sc_deb_or_only_walk)
        {
            reg_val_p=lsm330_step_counter_walk_deb;
            if(STEP_COUNTER_WALK_DEB==acc->step_walk_run)
            {
                acc->step_counter_deb=1;
            }
        }
        else if(STEP_COUNTER_ONLY_WALK==acc->sc_deb_or_only_walk)
        {
            reg_val_p=lsm330_step_counter_walk_only;
        }
        else
        {
            reg_val_p=lsm330_step_counter_walk;
        }

    }
    else if((STEP_COUNTER_ADD_WALK_DEBOUNCE==acc->sc_deb_or_only_walk)&&
        (STEP_COUNTER_WALK_DEB==mode))
    {
        size=size_walk_debounce;
        reg_val_p=lsm330_step_counter_walk_det;

        acc->step_counter_deb=0;
    }
	for(i=0; i<size; i++)
	{
		err=lsm330_acc_register_write(acc, NULL, 
			reg_val_p[i].reg_add, reg_val_p[i].reg_val);
        if(err<0)
            break;
    }

    if(err<0)
    {
        printk(KERN_ERR TAGE "%s set failed\n", __func__);
    }
    else
    {
        acc->step_walk_run=mode;
        acc->step_counter_enable=1;
        printk(KERN_ERR TAGI "%s mode %u okay\n", __func__, mode);
    }
    
    return err;
}

static int lsm330_step_counter_close(struct lsm330_acc_data *acc, unsigned char clear)
{
	int err=0;
    u8 data_buf[3];

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }

	data_buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG1);
	data_buf[1] = 0x00;
	data_buf[2] = 0x00;
	err = lsm330_acc_i2c_write(acc, data_buf, 2);
    mdelay(3);
    acc->step_counter_enable=0;
    /*
    if(STEP_COUNTER_PARA_CLEAR==clear)
    {
        acc->step_counter_last=acc->step_counter;
        acc->step_counter=0;
        acc->step_counter_overflow=0;
        acc->step_reg_value=0;
        acc->step_reg_value_last=0;        
    }
    */
    
    return err;
}


static int lsm330_get_step_counter_data(struct lsm330_acc_data *acc)
{
	int err = -1;
	u8 acc_data[2];
    u64 time_current_ns;
    u64 time_deltal;

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }

	acc_data[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
	err = lsm330_acc_i2c_read(acc, acc_data, 2);
	if (err < 0)
		return err;
    acc->step_reg_raw_value = (acc_data[1] << 8 | acc_data[0]);
    acc->sc_data_valid |= 0xf0;
    if(STEP_COUNTER_ONLY_WALK==acc->sc_deb_or_only_walk)
    {
    	acc->step_reg_value = (STEP_COUNTER_OVERFLOW - (STEP_COUNTER_OVERFLOW&(acc_data[1] << 8 | acc_data[0])))*STEP_COUNTER_INVERVAL;
 //       if((acc->step_reg_value_last!=acc->step_reg_value) && (acc->backlight_on))
		if(acc->step_reg_value_last!=acc->step_reg_value)
        {
           acc->step_counter_delta=STEP_COUNTER_INVERVAL;       
        }
    }
    else
    {
	    acc->step_reg_value = STEP_COUNTER_OVERFLOW - (STEP_COUNTER_OVERFLOW&(acc_data[1] << 8 | acc_data[0]));
    }
    /*
    if((STEP_COUNTER_OVERFLOW_MAX<acc->step_reg_value_last) &&
        (STEP_COUNTER_OVERFLOW_MIN>acc->step_reg_value))acc->step_counter_overflow++;
        */
    if((STEP_COUNTER_ADD_WALK_DEBOUNCE==acc->sc_deb_or_only_walk) && (acc->step_counter_deb))
    {
        if((acc->step_reg_value_last!=acc->step_reg_value))
        {
            acc->step_counter_deb=0;
            acc->step_counter_deb_counter+=11;
            /*acc->sc_deb_mode_counter=0;*/
        }
        else
        {
            time_current_ns = lsm330_acc_get_time_ns();
            /*acc->sc_deb_mode_counter++;*/
            /*if(9<=acc->sc_deb_mode_counter)*/
            if(time_current_ns>acc->sc_deb_mode_ns)
            {
                time_deltal=time_current_ns-acc->sc_deb_mode_ns;
            }
            else
            {
                time_deltal=acc->sc_deb_mode_ns-time_current_ns;
            }
            if(time_deltal>STEP_COUNTER_DETAL_TIME)
            {
                err=lsm330_step_counter_set_mode(acc, STEP_COUNTER_WALK_DEB);
                if(err<0)
                {
                    printk(KERN_ERR TAGE "%s step counter deb mode failed\n", __func__);
                }
            }
        }
    }
    acc->step_reg_value_last=acc->step_reg_value;
    acc->step_counter=acc->step_reg_value+acc->step_counter_overflow*STEP_COUNTER_OVERFLOW+
        acc->step_counter_last+acc->step_counter_deb_counter;
    /*printk(KERN_ERR "%s had read 0x%x 0x%x reg_value %u overflow %u step_counter %u  step_counter_last %u deb_counter %d\n", __func__, 
    acc_data[0], acc_data[1], acc->step_reg_value, acc->step_counter_overflow, acc->step_counter,
    acc->step_counter_last, acc->step_counter_deb_counter);*/

	return 0;
}
/*
static void lsm330_step_counter_irq_limit(struct lsm330_acc_data *acc)
{
    struct timespec ts;
    struct rtc_time tm;
    int err=0;
    int size_walk = sizeof(lsm330_step_counter_walk)/sizeof(struct st_reg_val);
    
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);      
    
    if(!acc->static_day)
    {
        if(24<tm.tm_hour+8)
        {
            acc->static_day=acc->current_day=tm.tm_mday+1;
        }
        else
        {
            acc->static_day=acc->current_day=tm.tm_mday;
        }
    }
    
    if(24<tm.tm_hour+8)
    {
        acc->current_day=tm.tm_mday+1;
    }
    else
    {
        acc->current_day=tm.tm_mday;
    }

    if(acc->current_day!=acc->static_day)
    {
        printk(KERN_ERR TAGI "%s current_day %u static_day %u hour %u step_counter %u\n", __func__, acc->current_day, acc->static_day,
            tm.tm_hour, acc->step_counter);

        acc->walk_run_counter=0;
        acc->static_day=acc->current_day;
        (lsm330_step_counter_walk+(size_walk-1))->reg_val=0x01;
    	err = lsm330_acc_i2c_update(acc, LSM330_CTRL_REG2,
				0x08, 0x00);
    }

}
*/
static int lsm330_step_counter_report_values(struct lsm330_acc_data *acc)
{
    unsigned int step_counter_last_rept;
    unsigned int report_value=0;
    
    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        return -EINVAL;
    }
    /*lsm330_step_counter_irq_limit(acc);*/
 //   if(acc->backlight_on)
 //   {
        if(STEP_COUNTER_ONLY_WALK==acc->sc_deb_or_only_walk)
        {
            if(0<acc->step_counter_delta)
            {     
                acc->step_counter_delta--;
            }

            step_counter_last_rept=acc->step_counter-acc->step_counter_delta;

            if(step_counter_last_rept>acc->step_counter_rept)
            {
                acc->step_counter_rept=step_counter_last_rept;
            }
            report_value=acc->step_counter_rept;
        }
        else
        {
            report_value=acc->step_counter;
        }
        input_report_abs(acc->input_step, ABS_X, (int)report_value);
        input_report_abs(acc->input_step, ABS_Y, (int)acc->force_report);
        /*input_report_abs(acc->input_step, ABS_Z, (int)acc->walk_run_counter);*/
        
        input_sync(acc->input_step);
  //  }
    return 0;
}

static void lsm330_step_counter_input_work_func(struct work_struct *work)
{
	struct lsm330_acc_data *acc;
	int err;

	acc = container_of((struct delayed_work *)work, struct lsm330_acc_data, step_work);

    /*printk(KERN_ERR TAGI "%s %u \n", __func__, acc->sc_poll_interval);*/
	mutex_lock(&acc->lock);
	err = lsm330_get_step_counter_data(acc);
	if (err < 0)
		printk(KERN_ERR TAGE "get step counter failed\n");
	else
		lsm330_step_counter_report_values(acc);

    if(acc->step_counter_enable)
    {
        schedule_delayed_work(&acc->step_work, msecs_to_jiffies(
                acc->sc_poll_interval));
    }
	mutex_unlock(&acc->lock);

}

static int lsm330_step_counter_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct lsm330_acc_data *acc = container_of(sensors_cdev,
		struct lsm330_acc_data, step_counter_cdev);
	int err=0;
    
	mutex_lock(&acc->lock);
    /*if set delay must report*/
    acc->force_report++;
	acc->sc_poll_interval = delay_msec;
	mutex_unlock(&acc->lock);
    printk(KERN_ERR TAGI "%s delay_msec %d sc_poll_interval %u\n", __func__, delay_msec, acc->sc_poll_interval);
	return err;
}

static int lsm330_step_counter_enable_disable(struct lsm330_acc_data *acc, unsigned int enable)
{
    int err=0;
    
    printk(KERN_ERR TAGI "%s enable %u %d sc_poll_interval %u\n", __func__, 
        enable, acc->step_counter_enable, acc->sc_poll_interval);

    acc->step_counter_ststate=enable;
    if (enable)
    {
        /*if enable must report*/
        acc->force_report++;
        /*lsm330_step_counter_report_values(acc);*/
        if(!acc->step_counter_enable)
        {
            err=lsm330_step_counter_init_config(acc);
            if(err<0)
            {
                printk(KERN_ERR TAGE "%s step counter init failed\n", __func__);
            }
            else
            {
                if(STEP_COUNTER_ADD_WALK_DEBOUNCE==acc->sc_deb_or_only_walk)
                {
                err=lsm330_step_counter_set_mode(acc, STEP_COUNTER_WALK_DEB);
                }
                else
                {
                err=lsm330_step_counter_set_mode(acc, STEP_COUNTER_WALK);
                }
                if(err<0)
                {
                    printk(KERN_ERR TAGE "%s step counter walk mode failed\n", __func__);
                }
                else
                {
                    schedule_delayed_work(&acc->step_work,
                        msecs_to_jiffies(STEP_COUNTER_FIRST_REPORT));                    
                }                
            }
            /*if config step counter failed must close SM1 and SM2 and set */
            if(err<0)
            {
                lsm330_step_counter_close(acc, STEP_COUNTER_PARA_HOLD);
            }
        }
    }
    else
    {
        if(acc->step_counter_enable)
        {
            cancel_delayed_work(&acc->step_work);
            err = lsm330_step_counter_close(acc, STEP_COUNTER_PARA_CLEAR);
            if(err < 0)
            {
               printk(KERN_ERR TAGE "%s step counter close failed\n", __func__); 
            }
        }
    }
    if(acc->use_batch)
    {
        err = lsm330_acc_enable_batch(acc, acc->use_batch);
        if (err) {
            printk(KERN_ERR TAGE "%s enable batch:%d failed\n", __func__,acc->fifo_timeout_ms);
        }
    }

    return err;
}


static int lsm330_step_counter_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct lsm330_acc_data *acc = container_of(sensors_cdev,
		struct lsm330_acc_data, step_counter_cdev);
	int err=0;
    
    mutex_lock(&acc->lock);
    lsm330_step_counter_enable_disable(acc, enable);
    mutex_unlock(&acc->lock);
	return err;
}

int lsm330_step_counter_input_open(struct input_dev *input)
{
	return 0;
}

void lsm330_step_counter_input_close(struct input_dev *dev)
{
    ;
}

static int lsm330_step_counter_input_init(struct lsm330_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->step_work, lsm330_step_counter_input_work_func);
	acc->input_step = input_allocate_device();
	if (!acc->input_step) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE "input steop counter device allocation failed\n");
		goto err0;
	}

	acc->input_step->open = lsm330_step_counter_input_open;
	acc->input_step->close = lsm330_step_counter_input_close;
	acc->input_step->name = LSM330_SC_DEV_NAME;

	acc->input_step->id.bustype = BUS_I2C;
	acc->input_step->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_step, acc);

	set_bit(EV_ABS, acc->input_step->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_step->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_step->absbit);

	input_set_abs_params(acc->input_step, ABS_X, -SG_MAX, SG_MAX, 0, 0);
	input_set_abs_params(acc->input_step, ABS_Y, -SG_MAX, SG_MAX, 0, 0);
	input_set_abs_params(acc->input_step, ABS_Z, -SG_MAX, SG_MAX, 0, 0);

	err = input_register_device(acc->input_step);
	if (err) {
		printk(KERN_ERR TAGE
				"unable to register input device %s\n",
				acc->input_step->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_step);
err0:
	return err;
}

static void lsm330_step_counter_input_cleanup(struct lsm330_acc_data *acc)
{
	input_unregister_device(acc->input_step);
	input_free_device(acc->input_step);
}

static ssize_t lsm330_step_counter_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);
	int err = -1;
	u8 acc_data[2];

	unsigned int sc_raw_data = 0;

    if(!dev_driver||!buf)
        return -EINVAL;

	mutex_lock(&acc->lock);
	if(!(0xf0&acc->sc_data_valid))
    {
        acc_data[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
        err = lsm330_acc_i2c_read(acc, acc_data, 2);
        if (0<err)
        {
            acc->step_reg_raw_value = (acc_data[1] << 8 | acc_data[0]);
            acc->sc_data_valid |= 0xf0;
        }
    }   
	sc_raw_data = acc->step_reg_raw_value;
	acc->sc_data_valid&=0x0f;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%u\n", sc_raw_data);
}


static int lsm330_acc_pd_probe(struct platform_device *pdev) 
{
	return 0;
}

static int lsm330_acc_pd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lsm330_acc_pd_driver = {
	.probe  = lsm330_acc_pd_probe,
	.remove = lsm330_acc_pd_remove,    
	.driver = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

struct platform_device lsm330_acc_pd_device = {
    .name = "gsensor",
    .id   = -1,
};

static int lsm330_acc_at_get_data(struct lsm330_acc_data *acc, u8 wh_data)
{
	int err;
	int xyz[3] = { 0 };
	#ifdef WRITE_BACK_THE_REGISTER
	u8 reg_back = LSM330_CTRL_REG4;
	#endif
	u8 buf[2];

	if(!atomic_read(&acc->enabled)||!(wh_data&acc->data_valid))
	{
	    #ifdef WRITE_BACK_THE_REGISTER
		err = lsm330_acc_i2c_read(acc, &reg_back, 1);
		if (err < 0)
		{
			printk(KERN_ERR TAGE "%s read failed\n", __func__);
		}
		else
		#endif
		{
		    buf[0] = LSM330_CTRL_REG4;
			buf[1] = 0x67;
			err = lsm330_acc_i2c_write(acc, buf, 1);
			if (err < 0)
			{
			   printk(KERN_ERR TAGE "%s set register failed\n", __func__);
			}
			else
			{
				err = lsm330_acc_get_acceleration_data(acc, xyz);
				if (err < 0)
					printk(KERN_ERR TAGE "%s get_acceleration_data failed\n", __func__);
			}
		}
		#ifdef WRITE_BACK_THE_REGISTER
		buf[0] = LSM330_CTRL_REG4;
		buf[1] = reg_back;
		err = lsm330_acc_i2c_write(acc, buf, 1);
		if (err < 0)
		{
		   printk(KERN_ERR TAGE "%s reset register failed\n", __func__);
		}
		#endif
	}
	return 0;
}

static ssize_t lsm330_raw_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	struct acceleration accel;

	mutex_lock(&acc->lock);
	lsm330_acc_at_get_data(acc, 0xf0);
	accel = acc->lastraw;
	acc->data_valid&=0x0f;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d %d %d\n", accel.x, accel.y, accel.z);
}

static ssize_t lsm330_final_data_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	struct acceleration accel;

	mutex_lock(&acc->lock);
	lsm330_acc_at_get_data(acc, 0x0f);
	accel = acc->lastdata;
	acc->data_valid&=0xf0;
	mutex_unlock(&acc->lock);


	return sprintf(buf, "%d %d %d\n",accel.x, accel.y, accel.z);
}


static ssize_t
sensor_calibration_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *lsm330 = i2c_get_clientdata(client);
	struct calibrate_data value = {5,5,6,6}; 
	int i,j=0;
	char bufnew[30];
		
	//printk(KERN_INFO TAGI "  sensor_calibration\n");
	for(i=0;i<=30;i++)
	{	bufnew[i] = *(buf+i);
	//printk(KERN_INFO TAGI "%c\n",*(buf+i));
		if(*(buf+i)=='\0')
			break;
	}
	for(i=0;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.x = simple_strtol(bufnew, NULL, 10);
			*(bufnew+i)=' ';
			j=i;
			break;
		}
	}
	i++;
	for(;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.y = simple_strtol(bufnew+j+1, NULL, 10);
			*(bufnew+i)=' ';
			j=i;
			break;
		}
	}
	i++;
	for(;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.z = simple_strtol(bufnew+j+1, NULL, 10);
			*(bufnew+i)=' ';
			j=i;	
			break;
		}
	}
	i++;
	for(;i<=30;i++)
	{	if(*(bufnew+i)==' ')
		{	*(bufnew+i)='\0';
			value.self_test = simple_strtol(bufnew+j+1, NULL, 10);
			*(bufnew+i)=' ';
			j=i;	
			break;
		}
	}	
	
	//	printk(KERN_INFO TAGI "  sensor_calibration_show	 x=%d	\n",value.x);
	//	printk(KERN_INFO TAGI "  sensor_calibration_show	 y=%d	\n",value.y);
	//	printk(KERN_INFO TAGI "  sensor_calibration_show	 z=%d	\n",value.z);
	//	printk(KERN_INFO TAGI "  sensor_calibration_show	 self_test=%d	\n",value.self_test);
	if((value.x > 120000 || value.x < -120000) || (value.y > 120000 || value.y < -120000) || (value.z > 120000 || value.z < -120000))
	{
		value.x = 0;
		value.y = 0;
		value.z = 0;
	}
	/*as accelerator IC has already been over through sleftest ,so ignore it. flyshine*/
	//value.self_test =  lsm330->calibration_data.self_test;
	/*end flyshine*/
	mutex_lock(&lsm330->data_mutex);

	lsm330->calibration_data = value;

	mutex_unlock(&lsm330->data_mutex);

	return count;
}

static ssize_t
sensor_calibration_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *lsm330 = i2c_get_clientdata(client);
	struct calibrate_data value;
	printk(KERN_INFO TAGI "  sensor_calibration_show	\n");

	mutex_lock(&lsm330->data_mutex);

	value = lsm330->calibration_data;

	mutex_unlock(&lsm330->data_mutex);
	printk(KERN_INFO TAGI "  sensor_calibration_show	 x=%d	%d\n",value.x, lsm330->calibration_data.x);
	printk(KERN_INFO TAGI "  sensor_calibration_show	 y=%d	%d\n",value.y, lsm330->calibration_data.y);
	printk(KERN_INFO TAGI "  sensor_calibration_show	 z=%d	%d\n",value.z, lsm330->calibration_data.z);
	printk(KERN_INFO TAGI "  sensor_calibration_show	 self_test=%d	%d\n",value.self_test, lsm330->calibration_data.self_test);

	return sprintf(buf, "%d %d %d %d  ov\n", value.x,value.y ,
value.z,value.self_test);
}


static ssize_t
sensor_calibratecmd_xy(struct lsm330_acc_data *lsm330)
{	
	int i,sumofx,sumofy,sumofz;
	struct acceleration accelnew;
	int * p_accelnew=(int*)&accelnew;
	int data_valid=0;
//	unsigned long delay = delay_to_jiffies(atomic_read(&lsm330->delay));
	sumofx = 0;
	sumofy = 0;
	sumofz = 0;
//	printk(KERN_INFO TAGI "  sensor_calibratecmd_xy	\n");

	for (i=0;i<50;i++)
	{
		//lsm330_measure(lsm330, &accelnew);
		if(!lsm330_acc_get_acceleration_data(lsm330,p_accelnew))
		{
			sumofx = sumofx + accelnew.x;
			sumofy = sumofy + accelnew.y;
			sumofz = sumofz + accelnew.z - 1000;
			data_valid++;
		}
		mdelay(5);
	}
//	printk(KERN_INFO TAGI "  sensor_calibratecmd_xy	mdelay	= %lu	\n",delay);
	sumofx = sumofx / data_valid;
	sumofy = sumofy / data_valid;
	sumofz = sumofz / data_valid;
	if (sumofx > -100000 && sumofx < 100000)
		if (sumofy > -100000 && sumofy < 100000)
		{
			lsm330->calibration_data.x = sumofx;
			lsm330->calibration_data.y = sumofy;
			lsm330->calibration_data.z = sumofz;
		}
	printk(KERN_INFO TAGI "  sensor_calibratecmd_x   =%d	%d\n",sumofx, lsm330->calibration_data.x);
	printk(KERN_INFO TAGI "  sensor_calibratecmd_y   =%d	%d\n",sumofy, lsm330->calibration_data.y);
	printk(KERN_INFO TAGI "  sensor_calibratecmd_z   =%d	%d\n",sumofz, lsm330->calibration_data.z);
	
	return 0;
}


static ssize_t
sensor_calibratecmd_z(struct lsm330_acc_data *lsm330)
{
	int i,sumofz;
	struct acceleration accelnew;
	int * p_accelnew =(int*)&accelnew;
	unsigned long delay = delay_to_jiffies(atomic_read(&lsm330->delay));
	sumofz = 0;
//	printk(KERN_INFO TAGI "  sensor_calibratecmd_z\n");

	for (i=0;i<20;i++)
	{
		//lsm330_measure(lsm330, &accelnew);
		lsm330_acc_get_acceleration_data(lsm330,p_accelnew);
		sumofz = sumofz + accelnew.z;
		mdelay(delay + 1);
	}
	//printk(KERN_INFO TAGI "  sensor_calibratecmd_Z	 mdelay  = %lu	\n",delay);
	sumofz = sumofz / 20;
	if (sumofz > -100000 && sumofz < 100000)
		lsm330->calibration_data.z = sumofz;
	printk(KERN_INFO TAGI "sensor_calibratecmd_z   =%d	\n",sumofz);

	return 0;
}


static ssize_t
sensor_calibratecmd_selftest(struct lsm330_acc_data *lsm330)
{
	struct acceleration accelold;
	struct acceleration accelnew;
	int *p_accelold=(int*)&accelold;
	int *p_accelnew=(int*)&accelnew;
	u8 data;
	int i,x_old_sum,y_old_sum,z_old_sum;
	int x_new_sum,y_new_sum,z_new_sum;
    	x_old_sum=y_old_sum=z_old_sum=0;
	x_new_sum=y_new_sum=z_new_sum=0;
	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest\n");

		data = LSM330_ACC_G_2G;/*LSM330_RANGE_2G*/
		data |= 0XC0;/*LSM330_BW_50HZ*/
		lsm330_acc_register_write(lsm330, &data,LSM330_CTRL_REG5,data);
        lsm330->sensitivity=SENSITIVITY_2G;
		mdelay(500);
        for ( i=0; i < 5; i++ ){
		lsm330_acc_get_acceleration_data(lsm330,p_accelold);
		x_old_sum += accelold.x;
		y_old_sum += accelold.y;
		z_old_sum += accelold.z;
              mdelay(5);
        }
        printk(KERN_INFO TAGI "old_sum_.x= %d, y = %d, z = %d\n",x_old_sum/5 ,y_old_sum/5 ,z_old_sum/5);
		data = LSM330_ACC_G_2G;
		data |= 0XC0|0x02;
		lsm330_acc_register_write(lsm330, &data,LSM330_CTRL_REG5,data);
		mdelay(500);
           for ( i=0; i < 5; i++ ){
		lsm330_acc_get_acceleration_data(lsm330,p_accelnew);
		x_new_sum += accelnew.x;
		y_new_sum += accelnew.y;
		z_new_sum += accelnew.z;
              mdelay(5);
            }
     printk(KERN_INFO TAGI "new_sum.x= %d, y = %d, z = %d\n",x_new_sum/5 ,y_new_sum/5 ,z_new_sum/5);
              data =LSM330_ACC_G_2G;
		lsm330_acc_register_write(lsm330, &data,LSM330_CTRL_REG5,data);
		mdelay(200);
		lsm330->calibration_data.self_test = 2;
              lsm330->calibrate_process = 2;
		if (  (abs(x_new_sum/5  - x_old_sum/5) > 60) && (abs(x_new_sum/5  - x_old_sum/5) < 1700)   )
			if (  (abs(y_new_sum/5 - y_old_sum/5) >60) && (abs(y_new_sum/5 - y_old_sum/5) <1700)   )
				if (  (abs(z_new_sum/5  - z_old_sum/5) > 60) && (abs(z_new_sum/5 - z_old_sum/5) <1700)   )
					lsm330->calibration_data.self_test = 1;
        printk(KERN_INFO TAGI "abs x = %ld,abs y = %ld,abs z = %ld,\n",abs(x_new_sum/5  - x_old_sum/5),
            abs(y_new_sum/5  - y_old_sum/5),abs(z_new_sum/5  - z_old_sum/5));
	printk(KERN_INFO TAGI "sensor_calibratecmd_selftest=%d\n", lsm330->calibration_data.self_test);
    lsm330_acc_update_fs_range(lsm330, lsm330->pdata->fs_range);
	return 0;
}

static ssize_t
sensor_calibratecmd_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *lsm330 = i2c_get_clientdata(client);
	int delay = atomic_read(&lsm330->delay);
 //  printk(KERN_INFO TAGI "	sensor_calibratecmd_store  \n");
	unsigned long cmd = simple_strtoul(buf, NULL, 10);
    int err;
    u8 reg_back = LSM330_CTRL_REG4;
    u8 reg_buf[2];
	ktime_t ktime;

	printk(KERN_INFO TAGI "sensor_calibratecmd_store cmd= %d enable %d\n",(int)cmd, atomic_read(&lsm330->enabled));

	if (atomic_read(&lsm330->enabled)) 
	{					
	//	cancel_delayed_work_sync(&lsm330->input_work);
		hrtimer_cancel(&lsm330->timer_acc);
        if(lsm330->use_batch)
        {
            reg_buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG6);
            reg_buf[1] = 0x10;
            err = lsm330_acc_i2c_write(lsm330, reg_buf, 1);
            if (err < 0)
                printk(KERN_ERR TAGE "soft power off  LSM330_CTRL_REG6 failed: %d\n", err);
        
            reg_buf[0] = (I2C_AUTO_INCREMENT | LSM330_FIFO_CTRL_A);
            reg_buf[1] = 0x00;
            err = lsm330_acc_i2c_write(lsm330, reg_buf, 1);
            if (err < 0)
                printk(KERN_ERR TAGE "soft power off LSM330_FIFO_CTRL_A failed: %d\n", err);
        
        }
	}
    if(lsm330->step_counter_enable)
    {                   
        cancel_delayed_work_sync(&lsm330->step_work);
    }

	if((!atomic_read(&lsm330->enabled))&&(!lsm330->step_counter_enable))
	{
		err = lsm330_acc_i2c_read(lsm330, &reg_back, 1);
		if (err < 0)
		{
			printk(KERN_ERR TAGE "%s read failed\n", __func__);
			goto ERROR_EXIT;
		}
		else
		{
		    reg_buf[0] = LSM330_CTRL_REG4;
			reg_buf[1] = 0x57;
			err = lsm330_acc_i2c_write(lsm330, reg_buf, 1);
			if (err < 0)
			{
			   printk(KERN_ERR TAGE "%s set register failed\n", __func__);
			   goto ERROR_EXIT;
			}
		}			
	}


	if (cmd == 1)
	{	lsm330->calibrate_process = 1;
		mdelay(20);
		sensor_calibratecmd_xy(lsm330);
		lsm330->calibrate_process = 0;
	}
	else if (cmd == 2) 
	{	lsm330->calibrate_process = 1;
		sensor_calibratecmd_z(lsm330);
		lsm330->calibrate_process = 0;
	}
		else if (cmd == 3) 
	{	lsm330->calibrate_process = 1;
		sensor_calibratecmd_selftest(lsm330);
		lsm330->calibrate_process = 0;
	}
	else
	{
	}
	printk(KERN_INFO TAGI "  sensor_calibratecmd_store cmd= %d enable %d\n",(int)cmd, atomic_read(&lsm330->enabled));

	if (atomic_read(&lsm330->enabled)) 
	{ 
		ktime = ktime_set(0,(delay+1) * NSEC_PER_MSEC);
		hrtimer_start(&lsm330->timer_acc, ktime, HRTIMER_MODE_REL);
	//	queue_delayed_work(lsm330->data_wq, &lsm330->input_work, delay_to_jiffies(delay) + 1);
	//	schedule_delayed_work(&lsm330->input_work, delay_to_jiffies(delay) + 1);				  
        if(lsm330->use_batch)
        {
            reg_buf[0] = (I2C_AUTO_INCREMENT | LSM330_CTRL_REG6);
            reg_buf[1] = lsm330->resume_state[LSM330_RES_CTRL_REG6];
            err = lsm330_acc_i2c_write(lsm330, reg_buf, 1);
            if (err < 0)
                printk(KERN_INFO TAGI "%s reset register LSM330_CTRL_REG6 failed\n", __func__);
        
            reg_buf[0] = (I2C_AUTO_INCREMENT | LSM330_FIFO_CTRL_A);
            reg_buf[1] = lsm330->resume_state[LSM330_RES_FIFO_CTRL_A];
            err = lsm330_acc_i2c_write(lsm330, reg_buf, 1);
            if (err < 0)
                printk(KERN_INFO TAGI "%s reset register LSM330_FIFO_CTRL_A failed\n", __func__);
        }
	}
	if (lsm330->step_counter_enable) 
	{ 
    	schedule_delayed_work(&lsm330->step_work, msecs_to_jiffies(
    			lsm330->sc_poll_interval));				  
	}
	if((!atomic_read(&lsm330->enabled))&&(!lsm330->step_counter_enable))
	{
		reg_buf[0] = LSM330_CTRL_REG4;
		reg_buf[1] = reg_back;
		err = lsm330_acc_i2c_write(lsm330, reg_buf, 1);
		if (err < 0)
		{
		   printk(KERN_INFO TAGI "%s reset register failed\n", __func__);
		}
	}
ERROR_EXIT:
	return count;
}


static ssize_t
sensor_calibratecmd_show(struct device_driver *dev_driver, char *buf)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *lsm330 = i2c_get_clientdata(client);
	int value;
//	printk(KERN_INFO TAGI "sensor_calibratecmd_show  \n");

	mutex_lock(&lsm330->data_mutex);
	
	value = lsm330->calibrate_process;

	mutex_unlock(&lsm330->data_mutex);

	return sprintf(buf, "%d\n", value);
//*/
//	  return 0;

}

static void sensor_eint_enable(u8 flag)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);
	int size=sizeof(lsm330_eint_config)/sizeof(struct st_reg_val);
    int i=0;
	u8 buf;

    if(flag)
    {
		/*for(i=0; i<size; i++)
		{
			buf=lsm330_eint_config[i].reg_add;		
			lsm330_acc_i2c_read(acc, &buf, 1);		
			printk(KERN_INFO TAGI "%s %x %x\n", __func__, lsm330_eint_config[i].reg_add, buf);
		}*/
		gpio_switch_setstate(0);
        if(acc->step_counter_enable)
        {
            cancel_delayed_work(&acc->step_work);
        }
        /*lsm330_acc_update_fs_range(acc, LSM330_ACC_G_2G);*/
        lsm330_step_counter_close(acc, STEP_COUNTER_PARA_CLEAR);
		for(i=0; i<size; i++)
		{
			lsm330_acc_register_write(acc, NULL, lsm330_eint_config[i].reg_add, lsm330_eint_config[i].reg_val);
		}
		
		for(i=0; i<size; i++)
		{
			buf=lsm330_eint_config[i].reg_add;		
			lsm330_acc_i2c_read(acc, &buf, 1);		
			printk(KERN_INFO TAGI "%s %x %x\n", __func__, lsm330_eint_config[i].reg_add, buf);
		}
		buf=0X18;
		lsm330_acc_i2c_read(acc, &buf, 1);
		printk(KERN_INFO TAGI "%s 0X18 %x\n", __func__, buf);  
        acc->moving_detect=1;
	}
	else
	{
	    /*u8 buf;*/
		if(acc->moving_detect==1)
		{
			printk(KERN_INFO TAGI "%s int disable\n ", __func__);
			buf=0X18;
			lsm330_acc_i2c_read(acc, &buf, 1);
			printk(KERN_INFO TAGI "%s 0X18 %x\n", __func__, buf);  

			buf=0X5F;
			lsm330_acc_i2c_read(acc, &buf, 1);
			printk(KERN_INFO TAGI "%s 0X5F %x\n", __func__, buf);
		}

		lsm330_acc_register_write(acc, NULL, LSM330_CTRL_REG1, 0x00);
        gpio_switch_setstate(0);
        acc->moving_detect=0;

        if((acc->step_counter_ststate)&&(!acc->step_counter_enable))
        {
            lsm330_step_counter_enable_disable(acc, acc->step_counter_ststate);
        }

	}

}

static ssize_t
sensor_eint_store(struct device_driver *dev_driver, const char *buffer, size_t count)
{
	unsigned int eint_state = simple_strtoul(buffer, NULL, 10);
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);
	printk(KERN_INFO TAGI "[sensor_eint]+++++++++sensor_interrupt_set+++++++++[%d]\n",eint_state);
    mutex_lock(&acc->lock);
    sensor_eint_enable(eint_state);
    mutex_unlock(&acc->lock);
	atomic_set(&acc->sensor_int,eint_state);  
	return count;
}

static ssize_t
sensor_eint__show(struct device_driver *dev_driver, char *buf)
{
    int value;
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);
    value = atomic_read(&acc->sensor_int);  
    return sprintf(buf, "%d\n", value);
}

static ssize_t
sensor_pockmode_show(struct device_driver *dev_driver, char *buf)
{
	return 0;
}

static ssize_t sensor_pockmode_store(struct device_driver *dev_driver, const char *buf, size_t count)
{
	return 0;
}

static ssize_t lsm330_acc_store_debug(struct device_driver *driver, const char *buf, size_t count)
{
	//struct i2c_client *client = apds9960_i2c_client;
	//struct apds9960_data*obj=i2c_get_clientdata(client);	 
	int num =0;
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

    num=simple_strtoul(buf, NULL, 10);
	if(0<=num)
		acc->debug=num;

	printk(KERN_INFO TAGI "%s debug %d\n", __func__, num);
    
	return count;
}

static ssize_t lsm330_acc_show_debug(struct device_driver *driver, char *buf)
{
	struct i2c_client *client = lsm330_acc_i2c_client;
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", acc->debug);
}

static DRIVER_ATTR(raw_data, 0666, lsm330_raw_data_show, NULL);
static DRIVER_ATTR(data, 0666, lsm330_final_data_show, NULL);
static DRIVER_ATTR(calibration, 0666, sensor_calibration_show, sensor_calibration_store);
static DRIVER_ATTR(calibratecmd, 0666, sensor_calibratecmd_show, sensor_calibratecmd_store);
static DRIVER_ATTR(set_eint, 0666, sensor_eint__show, sensor_eint_store);
static DRIVER_ATTR(pock_mode, 0777, sensor_pockmode_show, sensor_pockmode_store);
static DRIVER_ATTR(sc_raw_data, 0644, lsm330_step_counter_raw_data_show, NULL);
static DRIVER_ATTR(sc_mode_set, 0644, lsm330_step_counter_mode_set_show, lsm330_step_counter_mode_set_store);
static DRIVER_ATTR(debug,   0644, lsm330_acc_show_debug,lsm330_acc_store_debug);

static struct driver_attribute *lsm330_acc_attr_list[] = {
	&driver_attr_raw_data,
	&driver_attr_data,
	&driver_attr_calibration,
    &driver_attr_calibratecmd,
    &driver_attr_set_eint,
    &driver_attr_pock_mode,
    &driver_attr_sc_raw_data,
    &driver_attr_sc_mode_set,
    &driver_attr_debug,
};

#ifdef CONFIG_OF
static int lsm330_acc_parse_dt(struct device *dev,
			struct lsm330_acc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
    enum of_gpio_flags flags;


	rc = of_property_read_u32(np, "st,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read min-interval\n");
		return rc;
	} else {
		pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read init-interval\n");
		return rc;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read axis-map_x\n");
		return rc;
	} else {
		pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read axis_map_y\n");
		return rc;
	} else {
		pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read axis-map-z\n");
		return rc;
	} else {
		pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
		case 2:
			pdata->fs_range = LSM330_ACC_G_2G;
			break;
		case 4:
			pdata->fs_range = LSM330_ACC_G_4G;
			break;
		case 8:
			pdata->fs_range = LSM330_ACC_G_8G;
			break;
		case 16:
			pdata->fs_range = LSM330_ACC_G_16G;
			break;
		default:
			pdata->fs_range = LSM330_ACC_G_2G;
			break;
		}
	}

	rc = of_property_read_u32(np, "st,negate-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read negate-x\n");
		return rc;
	} else {
		pdata->negate_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,negate-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read negate-y\n");
		return rc;
	} else {
		pdata->negate_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,negate-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		printk(KERN_ERR TAGE "Unable to read negate-z\n");
		return rc;
	} else {
		pdata->negate_z = (u8)temp_val;
	}	
    if(of_property_read_bool(np,"sensor,step-irq"))
    {
    	rc = of_get_named_gpio_flags(np,
    				"sensor,step-irq", 0, &flags);
    	if (rc < 0) {
    		printk(KERN_ERR TAGE "%s read interrupt int pin error %d\n", __func__, rc);
            pdata->gpio_step= -2;
    	}
        else
        {
            pdata->gpio_step= rc;
        }
	}
	else
	{
        pdata->gpio_step= -2;
        printk(KERN_ERR TAGE "%s no sensor,step-irq\n", __func__);
	}	
	pdata->gpio_int1=LSM330_ACC_DEFAULT_INT1_GPIO;
	pdata->gpio_int2=LSM330_ACC_DEFAULT_INT2_GPIO;
	pdata->init=NULL;
	pdata->exit=NULL;
	pdata->power_on=NULL;
	pdata->power_off=NULL;

	return 0;
}
#else
static int lsm330_acc_parse_dt(struct device *dev,
			struct lsm330_acc_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static struct sensor_regulator lsm330_vreg[] = {
	{NULL, "vdd", 2850000, 2850000},
	{NULL, "vddio", 1800000, 1800000},
};

static int lsm330_acc_config_regulator(struct lsm330_acc_data *obj, bool on)
{
	int rc = 0, i;
	struct sensor_regulator *lsm330_acc_vreg_config=lsm330_vreg;
	int num_reg = sizeof(lsm330_acc_vreg_config) / sizeof(struct sensor_regulator);

	if (on) 
    {
		for (i = 0; i < num_reg; i++) 
        {
			lsm330_acc_vreg_config[i].vreg = regulator_get(&obj->client->dev, lsm330_acc_vreg_config[i].name);
            
			if (IS_ERR(lsm330_acc_vreg_config[i].vreg)) 
            {
				rc = PTR_ERR(lsm330_acc_vreg_config[i].vreg);
				printk(KERN_ERR TAGE " %s:regulator get failed rc=%d\n", __func__, rc);
				lsm330_acc_vreg_config[i].vreg = NULL;
			}

			if (regulator_count_voltages(lsm330_acc_vreg_config[i].vreg) > 0) 
            {
				rc = regulator_set_voltage(
					lsm330_acc_vreg_config[i].vreg,
					lsm330_acc_vreg_config[i].min_uV,
					lsm330_acc_vreg_config[i].max_uV);
				if(rc) {
					printk(KERN_ERR TAGE " %s: set voltage failed rc=%d\n", __func__, rc);
					regulator_put(lsm330_acc_vreg_config[i].vreg);
					lsm330_acc_vreg_config[i].vreg = NULL;
				}
			}

			rc = regulator_enable(lsm330_acc_vreg_config[i].vreg);
			if (rc) {
				printk(KERN_ERR TAGE " %s: regulator_enable failed rc =%d\n", __func__, rc);
				if (regulator_count_voltages(lsm330_acc_vreg_config[i].vreg) > 0) 
                {
					regulator_set_voltage(
						lsm330_acc_vreg_config[i].vreg, 0,
						lsm330_acc_vreg_config[i].max_uV);
				}
				regulator_put(lsm330_acc_vreg_config[i].vreg);
				lsm330_acc_vreg_config[i].vreg = NULL;
			}
		}
	} 
    else 
    {
		i = num_reg;
	}
    
	return rc;
}


static int lsm330_acc_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(lsm330_acc_attr_list)/sizeof(lsm330_acc_attr_list[0]));
	
    if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, lsm330_acc_attr_list[idx])))
		{            
			printk(KERN_ERR TAGE "driver_create_file (%s) = %d\n", lsm330_acc_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*
static int lsm330_acc_remove_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lsm330_acc_attr_list)/sizeof(lsm330_acc_attr_list[0]));
	
    if (NULL == driver)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, lsm330_acc_attr_list[idx]);
	}    
	return err;
}
*/
static irqreturn_t lsm330_step_counter_irq_handler(int irq, void *dev_id)
{
	struct lsm330_acc_data *acc =
	    (struct lsm330_acc_data *)dev_id;

    printk(KERN_ERR TAGI "%s\n", __func__);
    disable_irq_nosync(acc->irq_step);
    schedule_work(&acc->irq_work);
	return IRQ_HANDLED;
}

static void lsm330_step_counter_irq_work(struct work_struct *work)
{
	int err = -1;
	u8 acc_data[3];
    int state;
    int step_mode;
    int size_walk = sizeof(lsm330_step_counter_walk)/sizeof(struct st_reg_val);
    struct lsm330_acc_data *acc=container_of(work, struct lsm330_acc_data, irq_work);

    if(NULL==acc)
    {
        printk(KERN_ERR TAGE "%s null pointer\n", __func__);
        goto exit_err;
    }

    mutex_lock(&acc->lock);
	acc_data[0] = LSM330_STAT_REG;
	err = lsm330_acc_i2c_read(acc, acc_data, 1);
    printk(KERN_ERR TAGI "%s err %d 0x%x %d\n", __func__, err, acc_data[0], acc->step_walk_run); 
	if (err < 0)
    {
		printk(KERN_ERR TAGE "%s err %d\n", __func__, err);
    }
    else
    {
        if(STEP_COUNTER_SM2_INTR&acc_data[0])
        {   
            /*lsm330_step_counter_irq_limit(acc);*/
            acc_data[0]=0X7F;
    	    lsm330_acc_i2c_read(acc, acc_data, 1);
            printk(KERN_ERR TAGI "%s 0X7F is 0x%x\n", __func__, acc_data[0]);  

            err = lsm330_get_step_counter_data(acc);
            if (err < 0)
                printk(KERN_ERR TAGE "get step counter failed\n");
        
            err = lsm330_step_counter_close(acc, STEP_COUNTER_PARA_HOLD);
            if(err < 0)
            {
               printk(KERN_ERR TAGE "%s step counter close failed\n", __func__); 
            }
            /*if(acc->walk_run_counter<INT_COUNTER_THRESHLOD)acc->walk_run_counter++;*/
            if(acc->walk_run_counter<INT_COUNTER_THRESHLOD)
            {
                if(STEP_COUNTER_RUN==acc->step_walk_run)
                {
                    step_mode=STEP_COUNTER_WALK;
                }
                else if(STEP_COUNTER_WALK==acc->step_walk_run)
                {
                    step_mode=STEP_COUNTER_RUN;
                }
                else
                {
                    printk(KERN_ERR TAGE "%s wrong step_walk_run %d\n", __func__, acc->step_walk_run); 
                }
            }
            else
            {
                (lsm330_step_counter_walk+(size_walk-1))->reg_val=0x09;
                step_mode=STEP_COUNTER_WALK;
            }
            lsm330_step_counter_set_mode(acc, step_mode);
        }
        if(STEP_COUNTER_SM1_INTR&acc_data[0])
        {
            if(acc->moving_detect)
            {
            lsm330_step_counter_close(acc, STEP_COUNTER_PARA_CLEAR);
            state = gpio_get_value(acc->pdata->gpio_step);
            gpio_switch_setstate(state);            
            printk(KERN_ERR TAGI "%s gpio state %d\n", __func__, state);
            lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
            if((acc->step_counter_ststate)&&(!acc->step_counter_enable)&&(state))
            {
                lsm330_step_counter_enable_disable(acc, acc->step_counter_ststate);
            }
            }
            else if(STEP_COUNTER_ADD_WALK_DEBOUNCE==acc->sc_deb_or_only_walk)
            {
                if(STEP_COUNTER_WALK_DEB==acc->step_walk_run)
                {
                    step_mode=STEP_COUNTER_WALK;
                    lsm330_step_counter_set_mode(acc, step_mode);
                    acc->sc_deb_mode_ns = lsm330_acc_get_time_ns();
                }
                else if(STEP_COUNTER_WALK==acc->step_walk_run)
                {
                    step_mode=STEP_COUNTER_WALK_DEB;
                    lsm330_step_counter_set_mode(acc, step_mode);
                }
            }
        }
        if(STEP_COUNTER_STEP_COUNTER_OVERFLOW_INTR&acc_data[0])
        {
            acc_data[0]=0X5F;
    	    lsm330_acc_i2c_read(acc, acc_data, 1);
            printk(KERN_ERR TAGI "%s 0x5F is 0x%x\n", __func__, acc_data[0]);  

        	acc_data[0] = (I2C_AUTO_INCREMENT | LSM330_LC_L);
        	acc_data[1] = 0xFF/*STEP_COUNTER_OVERFLOW&&0xFF*/;
        	acc_data[2] = 0x7F/*(STEP_COUNTER_OVERFLOW>>8)&&0xFF*/;
        	err = lsm330_acc_i2c_write(acc, acc_data, 2);
        	if (err < 0)
        		printk(KERN_ERR TAGE "%s set step counter failed\n", __func__); 
            acc->step_counter_overflow++;
        }

    }
        
    

    mutex_unlock(&acc->lock);
    enable_irq(acc->irq_step);

exit_err:
    ;
}

#if defined(CONFIG_FB)
static int lsm330_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct lsm330_acc_data *acc = container_of(self, struct lsm330_acc_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
        if (*blank == FB_BLANK_POWERDOWN)
        {
            if(atomic_read(&acc->enabled)) 
                printk(KERN_INFO TAGI "sensor on after screen off\n");
        //    acc->backlight_on=0;
        }
        else if(FB_BLANK_UNBLANK==*blank)
        {
         //   acc->backlight_on=1;
        }
	}
	return 0;
}
#endif

static int lsm330_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lsm330_acc_data *acc;

	int err = -1;

	printk(KERN_INFO TAGI "%s: probe start.\n", LSM330_ACC_DEV_NAME);

	/*if (client->dev.platform_data == NULL) {
		printk(KERN_ERR TAGE "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}*/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR TAGE "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	acc = kzalloc(sizeof(struct lsm330_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE 
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

    print_vivo_init(TAGI "init 1\n");

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);
	mutex_init(&acc->data_mutex);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR TAGE 
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (client->dev.of_node) {
		err = lsm330_acc_parse_dt(&client->dev, acc->pdata);
		if (err) {
			printk(KERN_ERR TAGE "Failed to parse device tree\n");
			err = -EINVAL;
			goto exit_kfree_pdata;
		}
	} else if(client->dev.platform_data == NULL) {
		printk(KERN_ERR TAGE "using default platform_data for accelerometer\n");
		memcpy(acc->pdata, &default_lsm330_acc_pdata,
							sizeof(*acc->pdata));
	} else {
		memcpy(acc->pdata, client->dev.platform_data,
							sizeof(*acc->pdata));
	}

	err = lsm330_acc_validate_pdata(acc);
	if (err < 0) {
		printk(KERN_ERR TAGE "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

    lsm330_acc_config_regulator(acc, true);

    print_vivo_init(TAGI "init 2\n");

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			printk(KERN_ERR TAGE "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		printk(KERN_INFO TAGI "%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		printk(KERN_INFO TAGI "%s: %s has set irq2 to irq: %d "
							"mapped on gpio:%d\n",
			LSM330_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	/* resume state init config */
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	lsm330_acc_set_init_register_values(acc);

    print_vivo_init(TAGI "init 3\n");

	err = lsm330_acc_device_power_on(acc);
	if (err < 0) {
		printk(KERN_ERR TAGE  "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lsm330_acc_interrupt_enable_control(acc,
						acc->interrupts_enable_setting);
	if (err < 0) {
		printk(KERN_ERR TAGE  "interrupt settings failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
	if (err < 0) {
		printk(KERN_ERR TAGE  "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		printk(KERN_ERR TAGE  "update_odr failed\n");
		goto  err_power_off;
	}

    print_vivo_init(TAGI "init 4\n");

	acc->data_wq = NULL;
	acc->data_wq = create_singlethread_workqueue("lsm330_data_work");
	if (!acc->data_wq) {
		printk(KERN_ERR TAGE "create workquque failed\n");
		goto err_power_off;
	}
	err = lsm330_acc_input_init(acc);
	if (err < 0) {
		printk(KERN_ERR TAGE  "input init failed\n");
		goto err_destroy_workqueue;
	}

	printk(KERN_ERR TAGI  "hrtimer set start\n");
	hrtimer_init(&acc->timer_acc, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	acc->timer_acc.function = lsm330_acc_timer_handle;
	input_set_events_per_packet(acc->input_dev, 100);
//	printk(KERN_ERR TAGI  "hrtimer set end\n");

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		printk(KERN_ERR TAGE 
		   "device LSM330_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm330_acc_device_power_off(acc);

    print_vivo_init(TAGI "init 5\n");

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, lsm330_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			printk(KERN_ERR TAGE 
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lsm330_acc_isr1,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq1", acc);
		if (err < 0) {
			printk(KERN_ERR TAGE  "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, lsm330_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lsm330_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			printk(KERN_ERR TAGE 
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lsm330_acc_isr2,
				IRQF_TRIGGER_RISING, "lsm330_acc_irq2", acc);
		if (err < 0) {
			printk(KERN_ERR TAGE  "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

    print_vivo_init(TAGI "init 6\n");

	acc->cdev = lsm330_acc_cdev;
	acc->cdev.sensors_enable = lsm330_acc_enable_set;
	acc->cdev.sensors_poll_delay = lsm330_acc_poll_delay_set;
	#ifdef LSM330_BATCH_MODE
	acc->cdev.sensors_set_latency = lsm330_latency_set;
	acc->cdev.sensors_flush = lsm330_acc_flush;
	acc->cdev.fifo_reserved_event_count = LSM330_FIFO_SIZE;
	acc->cdev.fifo_max_event_count = LSM330_FIFO_SIZE;
	acc->cdev.max_delay = LSM330_INT_MAX_DELAY;
	#endif

	err = sensors_classdev_register(&client->dev, &acc->cdev);
	if (err) {
		printk(KERN_ERR TAGE 
			"class device create failed: %d\n", err);
		goto err_irq;
	}

    if((err = platform_driver_register(&lsm330_acc_pd_driver)))
	{
		printk(KERN_ERR TAGE  "%s failed to register lsm330_acc_driver err %d\n", __func__, err);
		goto err_unreg_sensor_class/*err_free_irq2*/;
	}

    if((err = platform_device_register(&lsm330_acc_pd_device)))
    {
		printk(KERN_ERR TAGE  "%s failed to register lsm330_acc_device err %d\n", __func__, err);
		goto err_free_driver;
    }

	if((err = lsm330_acc_create_attr(&lsm330_acc_pd_driver.driver)))
	{
		printk(KERN_ERR TAGE "%s lsm330 create attribute err = %d\n", __func__, err);
		goto err_free_device;
	}
	acc->acc_fifo=NULL;
	acc->acc_fifo=(u8 *)kmalloc(LSM330_FIFO_SIZE*6, GFP_KERNEL);
	if(acc->acc_fifo)
	    memset(acc->acc_fifo, 0, LSM330_FIFO_SIZE*6);
    acc->use_batch=false;
    acc->debug=0;
    lsm330_acc_i2c_client=client;

    print_vivo_init(TAGI "init 7\n");

#if defined(CONFIG_FB)
    acc->fb_notif.notifier_call = lsm330_fb_notifier_callback;
	fb_register_client(&acc->fb_notif);
#endif
    
	mutex_unlock(&acc->lock);

	wake_lock_init(&ts_judge_phone_direction_wakelock, WAKE_LOCK_SUSPEND, "Ts_judge_dir");
	acc_for_ts_judge_dir = lsm330_for_ts_judge_dir;

	printk(KERN_INFO TAGI  "%s haha\n", __func__);
    printk(KERN_ERR TAGI "%s %d\n", __func__, acc->pdata->gpio_step);
    if((0<=acc->pdata->gpio_step))
    {
        err = lsm330_step_counter_input_init(acc);
        if (err < 0) {
            printk(KERN_ERR TAGE  "input step counter init failed\n");
            goto err_remove_driver_fs;
        }
        
        acc->step_counter_cdev = lsm330_step_counter_cdev;
        acc->step_counter_cdev.sensors_enable = lsm330_step_counter_enable_set;
        acc->step_counter_cdev.sensors_poll_delay = lsm330_step_counter_poll_delay_set;
        err = sensors_classdev_register(&client->dev, &acc->step_counter_cdev);
        if (err) {
            printk(KERN_ERR TAGE 
                "class device step counter create failed: %d\n", err);
            goto err_input_step_counter_cleanup;
        }

        err = gpio_request(acc->pdata->gpio_step, "step-int");
        if (err < 0)
            goto err_unreg_step_class;
        
        err = gpio_direction_input(acc->pdata->gpio_step);
        if (err < 0)
            goto err_free_gpio;
        INIT_WORK(&acc->irq_work, lsm330_step_counter_irq_work);
        
        acc->irq_step = gpio_to_irq(acc->pdata->gpio_step);
        if (acc->irq_step < 0) {
            err = acc->irq_step;
            goto err_free_gpio;
        }
        /*IRQF_TRIGGER_HIGH IRQF_TRIGGER_LOW */
        err = request_irq( acc->irq_step, lsm330_step_counter_irq_handler,
                 IRQF_TRIGGER_RISING, "step-irq", acc);
        if (err < 0)
            goto err_free_gpio;
    	err = enable_irq_wake(acc->irq_step);
    	if (err < 0)
    		goto err_free_gpio;
        if(LSM330_ACC_G_4G!=acc->pdata->fs_range)
        {
            acc->pdata->fs_range=LSM330_ACC_G_4G;
            lsm330_acc_update_fs_range(acc, acc->pdata->fs_range);
            printk(KERN_ERR TAGI "%s, force change acc range to +-4G\n", __func__);
        }        
    }

    /*
    err=lsm330_step_counter_init_config(acc);
    if(err<0)
    {
        printk(KERN_ERR TAGE "%s step counter init failed\n", __func__);
    }
    */

    acc->sc_poll_interval=acc->step_counter_cdev.delay_msec;
    acc->step_counter_enable=0;
    acc->force_report=0;
    
    acc->step_counter_last=0;
    acc->step_counter=0;
    acc->step_counter_overflow=0;
    acc->step_reg_value=0;
    acc->step_reg_value_last=0;

    acc->walk_run_counter=0;
    acc->static_day=0;
    acc->current_day=0;
    acc->step_counter_delta=0;
    acc->step_counter_rept=0;

    acc->step_counter_deb_counter=0;
    acc->step_counter_deb=0;
    acc->sc_deb_mode_counter=0;

    acc->sc_deb_or_only_walk=STEP_COUNTER_ONLY_WALK;

	return 0;
err_free_gpio:
	gpio_free(acc->pdata->gpio_step);
err_unreg_step_class:
	sensors_classdev_unregister(&acc->step_counter_cdev);
err_input_step_counter_cleanup:
    lsm330_step_counter_input_cleanup(acc);
err_remove_driver_fs:
    /*lsm330_acc_remove_attr(&lsm330_acc_pd_driver.driver);*/
    /*if step counter create failed it only need to remove step counter*/
    return 0;
err_free_device:
	platform_device_unregister(&lsm330_acc_pd_device);
err_free_driver:
	platform_driver_unregister(&lsm330_acc_pd_driver);
err_unreg_sensor_class:
	sensors_classdev_unregister(&acc->cdev);
err_irq:
    if(acc->pdata->gpio_int2 >= 0)free_irq(acc->irq2, acc);
err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lsm330_acc_input_cleanup(acc);
err_destroy_workqueue:
	if (acc->data_wq)
		destroy_workqueue(acc->data_wq);
err_power_off:
	lsm330_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
//err_freedata:
	kfree(acc);
exit_check_functionality_failed:
	printk(KERN_ERR TAGE "%s: Driver Init failed\n", LSM330_ACC_DEV_NAME);
	return err;
}

static int lsm330_acc_remove(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	if(acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if(acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	if (atomic_cmpxchg(&acc->enabled, 1, 0))
		hrtimer_cancel(&acc->timer_acc);
		//	cancel_delayed_work_sync(&acc->input_work);

	lsm330_acc_device_power_off(acc);
	if (acc->data_wq)
		destroy_workqueue(acc->data_wq);
	lsm330_acc_input_cleanup(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();

	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lsm330_acc_resume(struct i2c_client *client)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

    if(atomic_read(&acc->enabled)) 
        printk(KERN_INFO TAGI "sensor on after suspend\n");

	if ((acc->on_before_suspend)&&(!acc->step_counter_enable))
		return lsm330_acc_enable(acc);
    if(acc->step_counter_enable)
    {
        /*if resume must report*/
        int err=0;
        err = lsm330_get_step_counter_data(acc);
        if (err < 0)
            printk(KERN_ERR TAGE "get step counter failed\n");

        acc->force_report++;
        schedule_delayed_work(&acc->step_work,
            msecs_to_jiffies(STEP_COUNTER_FIRST_REPORT));
    }
	return 0;
}

static int lsm330_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm330_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
    
    printk(KERN_ERR TAGI "sensor on after suspend %d\n", acc->step_counter_enable);
    if(acc->step_counter_enable)
        cancel_delayed_work_sync(&acc->step_work);
    else
        lsm330_acc_disable(acc);
	return 0;
}
#else
#define lsm330_acc_suspend	NULL
#define lsm330_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm330_acc_id[]
		= { { LSM330_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lsm330_acc_id);

static struct of_device_id lsm330acc_match_table[] = {
	{ .compatible = "lsm330_acc", },
	{ },
};

static struct i2c_driver lsm330_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM330_ACC_DEV_NAME,
			.of_match_table = lsm330acc_match_table,
		  },
	.probe = lsm330_acc_probe,
	.remove = lsm330_acc_remove,
	.suspend = lsm330_acc_suspend,
	.resume = lsm330_acc_resume,
	.id_table = lsm330_acc_id,
};

static int __init lsm330_acc_init(void)
{
	printk(KERN_INFO TAGI "%s accelerometer driver: init\n", LSM330_ACC_DEV_NAME);
	return i2c_add_driver(&lsm330_acc_driver);
}

static void __exit lsm330_acc_exit(void)
{
#ifdef DEBUG
	printk(KERN_INFO TAGI "%s accelerometer driver exit\n", LSM330_ACC_DEV_NAME);
#endif /* DEBUG */
	i2c_del_driver(&lsm330_acc_driver);
	return;
}

late_initcall(lsm330_acc_init);
module_exit(lsm330_acc_exit);

MODULE_DESCRIPTION("lsm330 accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

