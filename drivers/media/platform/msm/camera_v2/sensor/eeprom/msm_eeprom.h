/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#ifndef MSM_EEPROM_H
#define MSM_EEPROM_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_spi.h"
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"

struct msm_eeprom_ctrl_t;

#if defined(CONFIG_MACH_MSM8916_E7IILTE_SPR_US) || \
	defined(CONFIG_MACH_MSM8916_C50N_GLOBAL_COM) || \
	defined(CONFIG_MACH_MSM8916_C50_GLOBAL_COM) || \
	defined(CONFIG_MACH_MSM8916_C50DS_GLOBAL_COM) || \
	defined(CONFIG_MACH_MSM8916_Y50_TRF_US) || \
	defined(CONFIG_MACH_MSM8916_Y50C_TRF_US) || \
	defined(CONFIG_MACH_MSM8916_C50_TRF_US) || \
	defined(CONFIG_MACH_MSM8916_C50_CRK_US) || \
	defined(CONFIG_MACH_MSM8916_C30_TRF_US) || \
	defined(CONFIG_MACH_MSM8916_C30C_TRF_US)

	#define HI544_LGIT_MODULE

#elif defined(CONFIG_MACH_MSM8916_C70_CRK_US) || \
	defined(CONFIG_MACH_MSM8916_C70_RGS_CA)   || \
	defined(CONFIG_MACH_MSM8916_C70_USC_US)   || \
	defined(CONFIG_MACH_MSM8916_C50_MPCS_US)  || \
	defined(CONFIG_MACH_MSM8916_C50_TMO_US)   || \
	defined(CONFIG_MACH_MSM8916_C50_SPR_US)

	#define HI544_COWEL_MODULE

#endif

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

struct msm_eeprom_ctrl_t {
	struct platform_device *pdev;
	struct mutex *eeprom_mutex;

	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *eeprom_v4l2_subdev_ops;
	enum msm_camera_device_type_t eeprom_device_type;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_master;

	struct msm_camera_i2c_client i2c_client;
	struct msm_eeprom_memory_block_t cal_data;
	uint8_t is_supported;
	struct msm_eeprom_board_info *eboard_info;
	uint32_t subdev_id;

#if defined(CONFIG_MACH_LGE)
	struct list_head link;
	enum camb_position_t position;
#endif
};


typedef enum {
	BigEndian,
	LittleEndian,
} Endian;

#define MODULE_VENDOR_ID 0x700

//Module Selector
int32_t msm_eeprom_checksum_imtech(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_cowell(struct msm_eeprom_ctrl_t *e_ctrl);

//Module CheckSum routine
int32_t msm_eeprom_checksum_cowell_hi841(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_cowell_imx258(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_imtech_ov8858(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_imtech_t4kb3(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_imtech_hi841(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_v0d(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_v0d_t4ka3(struct msm_eeprom_ctrl_t *e_ctrl);
int32_t msm_eeprom_checksum_lgit_hi553(struct msm_eeprom_ctrl_t *e_ctrl);

//Helper function for arithmetic shifted addition / just accumulation
uint32_t shiftedSum (struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr, Endian endian);
uint32_t accumulation (struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr);

#endif
