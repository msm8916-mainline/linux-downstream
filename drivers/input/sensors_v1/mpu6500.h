/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*register and associated bit definition*/

#ifndef __MPU6500_H__
#define __MPU6500_H__

#define REG_SAMPLE_RATE_DIV	0x19
#define REG_CONFIG		0x1A

#define REG_GYRO_CONFIG		0x1B
#define BITS_SELF_TEST_EN	0xE0
#define GYRO_CONFIG_FSR_SHIFT	3

#define REG_ACCEL_CONFIG	0x1C
#define REG_ACCEL_MOT_THR	0x1F
#define REG_ACCEL_MOT_DUR	0x20
#define ACCL_CONFIG_FSR_SHIFT	3

#define REG_ACCELMOT_THR	0x1F

#define REG_ACCEL_MOT_DUR	0x20

#define REG_FIFO_EN		0x23
#define FIFO_DISABLE_ALL	0x00
#define BIT_ACCEL_OUT		0x08
#define BITS_GYRO_OUT		0x70

#define REG_INT_PIN_CFG		0x37
#define BIT_INT_ACTIVE_LOW	0x80
#define BIT_INT_OPEN_DRAIN	0x40
#define BIT_INT_LATCH_EN	0x20
#define BIT_INT_RD_CLR		0x10
#define BIT_I2C_BYPASS_EN	0x02
#define BIT_INT_CFG_DEFAULT	(BIT_INT_LATCH_EN | BIT_INT_RD_CLR)

#define REG_INT_ENABLE		0x38
#define BIT_DATA_RDY_EN		0x01
#define BIT_DMP_INT_EN		0x02
#define BIT_FIFO_OVERFLOW	0x10
#define BIT_ZMOT_EN		0x20
#define BIT_MOT_EN		0x40
#define BIT_6500_WOM_EN		0x40

#define REG_DMP_INT_STATUS	0x39

#define REG_INT_STATUS		0x3A
#define BIT_DATA_RDY_INT	0x01
#define BIT_DMP_INT_INT		0x02
#define BIT_FIFO_OVERFLOW	0x10
#define BIT_ZMOT_INT		0x20
#define BIT_MOT_INT		0x40
#define BIT_6500_WOM_INT	0x40

#define REG_RAW_ACCEL		0x3B
#define REG_TEMPERATURE		0x41
#define REG_RAW_GYRO		0x43
#define REG_EXT_SENS_DATA_00	0x49

#define BIT_FIFO_RST		0x04
#define BIT_DMP_RST		0x08
#define BIT_I2C_MST_EN		0x20
#define BIT_FIFO_EN		0x40
#define BIT_DMP_EN		0x80
#define BIT_ACCEL_FIFO		0x08
#define BIT_GYRO_FIFO		0x70

#define REG_DETECT_CTRL		0x69
#define MOT_DET_DELAY_SHIFT	4

#define REG_USER_CTRL		0x6A
#define	BIT_FIFO_RESET		0x04

#define REG_PWR_MGMT_1		0x6B
#define BIT_H_RESET		0x80
#define BIT_SLEEP		0x40
#define BIT_CYCLE		0x20
#define BIT_CLK_MASK		0x07
#define BIT_RESET_ALL		0xCF
#define BIT_WAKEUP_AFTER_RESET	0x00

#define REG_PWR_MGMT_2		0x6C
#define BIT_PWR_ACCEL_STBY_MASK	0x38
#define BIT_PWR_GYRO_STBY_MASK	0x07
#define BIT_LPA_FREQ_MASK	0xC0
#define BITS_PWR_ALL_AXIS_STBY	(BIT_PWR_ACCEL_STBY_MASK |\
				BIT_PWR_GYRO_STBY_MASK)

#define REG_FIFO_COUNT_H	0x72
#define REG_FIFO_R_W		0x74
#define REG_WHOAMI		0x75

#define SAMPLE_DIV_MAX		0xFF
#define ODR_DLPF_DIS		8000
#define ODR_DLPF_ENA		1000

/* Min delay = MSEC_PER_SEC/ODR_DLPF_ENA */
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_ENA/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_DLPF	1
#define DELAY_MS_MAX_DLPF	256

/* Min delay = MSEC_PER_SEC/ODR_DLPF_DIS and round up to 1*/
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_DIS/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_NODLPF	1
#define DELAY_MS_MAX_NODLPF	32

/* device bootup time in millisecond */
#define POWER_UP_TIME_MS	100
/* delay to wait gyro engine stable in millisecond */
#define SENSOR_UP_TIME_MS	30
/* delay between power operation in millisecond */
#define POWER_EN_DELAY_US	10

#define MPU6050_LPA_5HZ		0x40

/* initial configure */
#define INIT_FIFO_RATE		200

#define DEFAULT_MOT_THR		1
#define DEFAULT_MOT_DET_DUR	1
#define DEFAULT_MOT_DET_DELAY	0

/* chip reset wait */
#define MPU6500_RESET_RETRY_CNT	10
#define MPU6500_RESET_WAIT_MS	20

/* FIFO related constant */
#define MPU6500_FIFO_SIZE_BYTE	1024
#define	MPU6500_FIFO_CNT_SIZE	2

// below do not modify	 
#define MPU6500_SUCCESS                     0
#define MPU6500_ERR_I2C                     -1

/* selftest */
#define REG_6500_XG_ST_DATA     0x0
#define REG_6500_XA_ST_DATA     0xD
#define REG_6500_XA_OFFS_H      0x77
#define REG_6500_YA_OFFS_H      0x7A
#define REG_6500_ZA_OFFS_H      0x7D
#define REG_6500_ACCEL_CONFIG2  0x1D
#define BIT_ACCEL_FCHOCIE_B              0x08
#define BIT_FIFO_SIZE_1K                 0x40

/* sample rate */
#define DEF_SELFTEST_SAMPLE_RATE        0
/* full scale setting dps */
#define DEF_SELFTEST_GYRO_FS            (0 << 3)
#define DEF_SELFTEST_ACCEL_FS           (2 << 3)
#define DEF_SELFTEST_GYRO_SENS          (32768 / 250)
/* wait time before collecting data */
#define DEF_GYRO_WAIT_TIME              10
#define DEF_ST_STABLE_TIME              20
#define DEF_ST_6500_STABLE_TIME         20
#define DEF_GYRO_SCALE                  131
#define DEF_ST_PRECISION                1000
#define DEF_ST_ACCEL_FS_MG              8000UL
#define DEF_ST_SCALE                    (1L << 15)
#define DEF_ST_TRY_TIMES                2
/*---- MPU6500 Self Test Pass/Fail Criteria ----*/
/* Gyro Offset Max Value (dps) */
#define DEF_GYRO_OFFSET_MAX             20
/* Gyro Self Test Absolute Limits ST_AL (dps) */
#define DEF_GYRO_ST_AL                  60
/* Accel Self Test Absolute Limits ST_AL (mg) */
#define DEF_ACCEL_ST_AL_MIN             225
#define DEF_ACCEL_ST_AL_MAX             675
#define DEF_6500_ACCEL_ST_SHIFT_DELTA   500
#define DEF_6500_GYRO_CT_SHIFT_DELTA    500
#define DEF_ST_MPU6500_ACCEL_LPF        2
#define DEF_ST_6500_ACCEL_FS_MG         2000UL
#define DEF_SELFTEST_6500_ACCEL_FS      (0 << 3)

#define THREE_AXIS               3
#define BYTES_PER_SENSOR         6

#define ONE_K_HZ                              1000
#define INV_MPU_SAMPLE_RATE_CHANGE_STABLE 50
#define INIT_ST_SAMPLES          200
#define FIFO_COUNT_BYTE          2

enum mpu_device_id {
	MPU6050_ID = 0x68,
	MPU6500_ID = 0x70,
};

enum mpu_fsr {
	MPU_FSR_250DPS = 0,
	MPU_FSR_500DPS,
	MPU_FSR_1000DPS,
	MPU_FSR_2000DPS,
	NUM_FSR
};

enum mpu_filter {
	MPU_DLPF_256HZ_NOLPF2 = 0,
	MPU_DLPF_188HZ,
	MPU_DLPF_98HZ,
	MPU_DLPF_42HZ,
	MPU_DLPF_20HZ,
	MPU_DLPF_10HZ,
	MPU_DLPF_5HZ,
	MPU_DLPF_RESERVED,
	NUM_FILTER
};

enum mpu_clock_source {
	MPU_CLK_INTERNAL = 0,
	MPU_CLK_PLL_X,
	NUM_CLK
};

enum mpu_accl_fs {
	ACCEL_FS_02G = 0,
	ACCEL_FS_04G,
	ACCEL_FS_08G,
	ACCEL_FS_16G,
	NUM_ACCL_FSR
};

/* Sensitivity Scale Factor */
/* Sensor HAL will take 1024 LSB/g */
enum mpu_accel_fs_shift {
	ACCEL_SCALE_SHIFT_02G = 0,
	ACCEL_SCALE_SHIFT_04G = 1,
	ACCEL_SCALE_SHIFT_08G = 2,
	ACCEL_SCALE_SHIFT_16G = 3
};

enum mpu_gyro_fs_shift {
	GYRO_SCALE_SHIFT_FS0 = 3,
	GYRO_SCALE_SHIFT_FS1 = 2,
	GYRO_SCALE_SHIFT_FS2 = 1,
	GYRO_SCALE_SHIFT_FS3 = 0
};

/*device enum */
enum inv_devices {
	INV_MPU6050,
	INV_MPU6500,
	INV_MPU6XXX,
	INV_NUM_PARTS
};


/**
 *  struct mpu_reg_map_s - Notable slave registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal LPF.
 *  @fifo_en:	Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accel_config:	accel config register
 *  @mot_thr:	Motion detection threshold.
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:	FIFO register.
 *  @raw_gyro:	Address of first gyro register.
 *  @raw_accl:	Address of first accel register.
 *  @temperature:	temperature register.
 *  @int_pin_cfg:	Interrupt pin and I2C bypass configuration.
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt flags.
 *  @user_ctrl:	User control.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 */
struct mpu_reg_map {
	u8 sample_rate_div;
	u8 lpf;
	u8 fifo_en;
	u8 gyro_config;
	u8 accel_config;
	u8 fifo_count_h;
	u8 mot_thr;
	u8 mot_dur;
	u8 fifo_r_w;
	u8 raw_gyro;
	u8 raw_accel;
	u8 temperature;
	u8 int_pin_cfg;
	u8 int_enable;
	u8 int_status;
	u8 user_ctrl;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
};

/**
 *  struct mpu_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable to enable output
 *  @accel_enable:		enable accel functionality
 *  @accel_fifo_enable:	enable accel data output
 *  @gyro_enable:		enable gyro functionality
 *  @gyro_fifo_enable:	enable gyro data output
 *  @is_asleep:		1 if chip is powered down.
 *  @lpa_mode:		low power mode.
 *  @tap_on:		tap on/off.
 *  @flick_int_on:		flick interrupt on/off.
 *  @int_enabled:		interrupt is enabled.
 *  @mot_det_on:		motion detection wakeup enabled.
 *  @cfg_fifo_en:		FIFO R/W is enabled in USER_CTRL register.
 *  @int_pin_cfg:		interrupt pin configuration.
 *  @lpa_freq:		frequency of low power accelerometer.
 *  @rate_div:		Sampling rate divider.
 */
struct mpu_chip_config {
	u32 fsr:2;
	u32 lpf:3;
	u32 accel_fs:2;
	u32 enable:1;
	u32 accel_enable:1;
	u32 accel_fifo_enable:1;
	u32 gyro_enable:1;
	u32 gyro_fifo_enable:1;
	u32 is_asleep:1;
	u32 lpa_mode:1;
	u32 tap_on:1;
	u32 flick_int_on:1;
	u32 int_enabled:1;
	u32 mot_det_on;
	u32 cfg_fifo_en:1;
	u8 int_pin_cfg;
	u16 lpa_freq;
	u16 rate_div;
};

/*** for selftest device  ***/
struct inv_selftest_device {
    u8 *name;
    enum inv_devices chip_type;
    u16 sample_rate_div;
    u16 config;
    u16 gyro_config;
    u16 accel_config;
    int accel_bias[3];
    int gyro_bias[3];
    int accel_bias_st[3];
    int gyro_bias_st[3];
    u16 samples;        //selftest sample number
};

/**
 *  struct mpu6500_platform_data - device platform dependent data.
 *  @gpio_en:		enable GPIO.
 *  @gpio_int:		interrupt GPIO.
 *  @int_flags:		interrupt pin control flags.
 *  @use_int:		use interrupt mode instead of polling data.
 *  @place:			sensor place number.
 */
struct mpu6500_platform_data {
    int gpio_int;
	u32 int_flags;
	u8 place;
};

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

static const u16 mpu_6500_st_tb[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};

#endif /* __MPU6500_H__ */
