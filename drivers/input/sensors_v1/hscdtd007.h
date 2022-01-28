/* hscdtd007.h - hscdtd007 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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
 */

#ifndef HSCDTD007_H
#define HSCDTD007_H

#include <linux/delay.h>
#include <linux/ioctl.h>

////////////
#define HSCDTD007_I2C_ADDRESS 0x0c  /*7bit*/
#define HSCDTD_CHIP_ID		0x1511
#define HSCDTD_WIA_VALUE	0x49

#define HSCDTD_STBA			0x0C
#define HSCDTD_INFO			0x0D
#define HSCDTD_WIA 			0x0F
#define HSCDTD_XOUT			0x10
#define HSCDTD_YOUT			0x12
#define HSCDTD_ZOUT			0x14
#define HSCDTD_XOUT_H		0x11
#define HSCDTD_XOUT_L		0x10
#define HSCDTD_YOUT_H		0x13
#define HSCDTD_YOUT_L		0x12
#define HSCDTD_ZOUT_H		0x15
#define HSCDTD_ZOUT_L		0x14

#define HSCDTD_STATUS		0x18
#define HSCDTD_CTRL1		0x1B
#define HSCDTD_CTRL2		0x1C
#define HSCDTD_CTRL3		0x1D
#define HSCDTD_CTRL4		0x1E

#define HSCDTD_TCS_TIME		10000	/* Measure temp. of every 10 sec */
#define HSCDTD_DATA_ACCESS_NUM	6
#define HSCDTD_3AXIS_NUM	3
#define HSCDTD_INITIALL_DELAY	20
#define STBB_OUTV_THR		3838

#define HSCDTD_DELAY(us)	usleep_range(us, us)

/* Self-test resiter value */
#define HSCDTD_ST_REG_DEF	0x55
#define HSCDTD_ST_REG_PASS	0xAA
#define HSCDTD_ST_REG_X		0x01
#define HSCDTD_ST_REG_Y		0x02
#define HSCDTD_ST_REG_Z		0x04
#define HSCDTD_ST_REG_XYZ	0x07

/* Self-test error number */
#define HSCDTD_ST_OK		0x00
#define HSCDTD_ST_ERR_I2C	0x01
#define HSCDTD_ST_ERR_INIT	0x02
#define HSCDTD_ST_ERR_1ST	0x03
#define HSCDTD_ST_ERR_2ND	0x04
#define HSCDTD_ST_ERR_VAL	0x10
#define HSCDTD_ST_ERR_VAL_X	(HSCDTD_ST_REG_X | HSCDTD_ST_ERR_VAL)
#define HSCDTD_ST_ERR_VAL_Y	(HSCDTD_ST_REG_Y | HSCDTD_ST_ERR_VAL)
#define HSCDTD_ST_ERR_VAL_Z	(HSCDTD_ST_REG_Z | HSCDTD_ST_ERR_VAL)

#define HSCDTD_RESET_DATA 		0x80
///////////////////////

/* To avoid device dependency, convert to general name */
#define HSCDTD_I2C_NAME			"hscdtd007"
#define HSCDTD_MISCDEV_NAME		"hscdtd007_dev"
#define HSCDTD_SYSCLS_NAME			"compass"
#define HSCDTD_SYSDEV_NAME			"hscdtd007"

#define HSCDTD_MEASURE_TIME_US		10000
#define HSCDTD_DRDY_IS_HIGH(x)		((x) & 0x01)
#define HSCDTD_SENSOR_INFO_SIZE	2
#define HSCDTD_SENSOR_CONF_SIZE	3
#define HSCDTD_SENSOR_DATA_SIZE	HSCDTD_DATA_ACCESS_NUM

#define HSCDTD_YPR_DATA_SIZE		16
#define HSCDTD_RWBUF_SIZE			16
#define HSCDTD_REGS_SIZE			2

#define HSCDTD_MODE_POWERDOWN			0x00
#define HSCDTD_MODE_SNG_MEASURE		0x01
#define HSCDTD_MODE_CONT1_MEASURE	0x02
#define HSCDTD_MODE_EXT_TRIG_MEASURE	0x04
#define HSCDTD_MODE_CONT2_MEASURE	0x06
#define HSCDTD_MODE_SELF_TEST			0x08
#define HSCDTD_MODE_FUSE_ACCESS		0x0F

#define HSCDTD_ST1_DRDY				0x01
#define HSCDTD_ST1_DOR					0x02
#define HSCDTD_ST2_HOLF				0x08
#define HSCDTD_ST2_BITM				0x10

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define FUSION_DATA_FLAG	2
#define HSCDTD_NUM_SENSORS		3

#define ACC_DATA_READY		(1<<(ACC_DATA_FLAG))
#define MAG_DATA_READY		(1<<(MAG_DATA_FLAG))
#define FUSION_DATA_READY	(1<<(FUSION_DATA_FLAG))

#define ALPSIO				0xAF

/* IOCTLs for HSCDTD library */
#define ECS_IOCTL_READ				_IOWR(ALPSIO, 0x01, char)
#define ECS_IOCTL_WRITE				_IOW(ALPSIO, 0x02, char)
#define ECS_IOCTL_RESET				_IO(ALPSIO, 0x03)
#define ECS_IOCTL_SET_MODE			_IOW(ALPSIO, 0x10, char)
#define ECS_IOCTL_SET_YPR			_IOW(ALPSIO, 0x11, int[HSCDTD_YPR_DATA_SIZE])
#define ECS_IOCTL_GET_INFO			_IOR(ALPSIO, 0x20, unsigned char[HSCDTD_SENSOR_INFO_SIZE])
#define ECS_IOCTL_GET_CONF			_IOR(ALPSIO, 0x21, unsigned char[HSCDTD_SENSOR_CONF_SIZE])
#define ECS_IOCTL_GET_DATA			_IOR(ALPSIO, 0x22, unsigned char[HSCDTD_SENSOR_DATA_SIZE])
#define ECS_IOCTL_GET_OPEN_STATUS	_IOR(ALPSIO, 0x23, int)
#define ECS_IOCTL_GET_CLOSE_STATUS	_IOR(ALPSIO, 0x24, int)
#define ECS_IOCTL_GET_DELAY			_IOR(ALPSIO, 0x25, long long int)
#define ECS_IOCTL_GET_LAYOUT		_IOR(ALPSIO, 0x26, char)
#define ECS_IOCTL_GET_ACCEL			_IOR(ALPSIO, 0x30, short[3])

struct hscdtd007_platform_data {
	char layout;
	int	auto_report;
	int gpio_DRDY;
	int gpio_rstn;
	int gpio_int;
	unsigned int int_flags;
	bool use_int;
};

#endif