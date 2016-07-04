/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_lg4895.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* production test */
#define tc_test_mode_ctl		(0x0C6E)
#define cmd_test_enter			(0x1)
#define cmd_test_exit			(0x0)
#define tc_tsp_test_ctl			(0xC04)
#define tc_tsp_test_sts			(0x265)
#define tc_tsp_test_pf_result		(0x266)
#define tc_tsp_test_off_info		(0x2FB)
#define PROD_os_result_tune_code_offset (0x2FE)

#define tc_tsp_test_data_offset		(0x07B)
#define tc_tsp_data_access_addr		(0x301)

#define RAWDATA_OFFSET			(0x0B84)
#define OPEN_RESULT_OFFSET		(0xB84)
#define SHORT_RESULT_OFFSET		(0xCC4)
#define OPEN_SHORT_DIFF_RESULT_OFFSET	(0xE04)
#define rawdata_ctl_read		(0x2A4)
#define rawdata_ctl_write		(0xC49)
#define DATA_RAWSIW_BASE_ADDR		(0xD1C)

//psy++ 151129
typedef enum
{
    IT_NONE = 0,
    IT_ALGORITHM_RAW_IMAGE,
    IT_BASELINE_IMAGE,
    IT_DELTA_IMAGE,
    IT_LABEL_IMAGE,
    IT_FILTERED_DELTA_IMAGE,
    IT_WAIT = 0xFF
} eImageType_t;

typedef struct
{
    volatile eImageType_t eImageType;
} tImageControl_t;

typedef enum
{
    RS_READY    = 0xA0,
    RS_NONE     = 0x05,
    RS_LOG      = 0x77,
    RS_IMAGE	= 0xAA
} eProtocolReadyStatus_t;

typedef struct
{
    volatile eProtocolReadyStatus_t eReadyStatus;
    bool bEventReady;
} tProtocolGetter_t;

#define CMD_I2CBASE_ADDR                (0xC00)
#define tc_interrupt_clr				(CMD_I2CBASE_ADDR + 0x2)

#define SWIP_REG_ADDR_CTRL_GETTER       (0x0300)
#define DATA_I2CBASE_ADDR               (0x301)
#define SERIAL_DATA_OFFSET              (0x007B)
#define DATA_SRAM_DELTA_BASE_ADDR 	(0xF80)
#define DATA_LABLE_BASE_ADDR 		(0x10E8)
#define DATA_BASE_ADDR 			(0xE4E)

//psy+++ i2c_write_from_indirect
#define ADDR_CMD_REG_SWIP_ACCESS_START           0x0C9E
#define ADDR_CMD_REG_SWIP_ACCESS_STOP            0x0C9F
#define ADDR_CMD_REG_SWIP_ACCESS_TRANSFER_TX     0x0CA0
#define ADDR_CMD_REG_SWIP_ACCESS_TRANSFER_RX	(0x0CE0)
//psy++ readFrameRnAFL
#define CMD_I2CBASE_ADDR			(0xC00)
#define tc_interrupt_clr				(CMD_I2CBASE_ADDR + 0x2)

#define SWIP_REG_ADDR_IMAGE                      0x0600
#define SWIP_REG_ADDR_IMAGE_INFO                 0x0600
#define SWIP_REG_ADDR_IMAGE_CONTROL              0x0610
#define SWIP_REG_ADDR_IMAGE_DATA_FORMAT          0x0620
#define SWIP_REG_ADDR_IMAGE_FINGER_NUM           0x0630
#define SWIP_REG_ADDR_IMAGE_FINGER_AREA          0x0631

#define ADDR_CMD_REG_SIC_IMAGECTRL_TYPE		0x0C6C
#define ADDR_CMD_REG_SIC_GETTER_READYSTATUS	0x0C64
//-------------------------

struct lg4895_test_off {
	u16 offset0;
	u16 offset1;
} __packed;

struct lg4895_test_off_info {
	struct lg4895_test_off m1_m2_raw;
	struct lg4895_test_off frame0_1;
	struct lg4895_test_off frame2_short;
	struct lg4895_test_off os_result;
} __packed;

/* tune code */
#define tc_tune_code_size		260
#define tc_total_ch_size		34
#define TSP_TUNE_CODE_L_GOFT_OFFSET		0
#define TSP_TUNE_CODE_L_M1_OFT_OFFSET		2
#define TSP_TUNE_CODE_L_G1_OFT_OFFSET		(TSP_TUNE_CODE_L_M1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_L_G2_OFT_OFFSET	(TSP_TUNE_CODE_L_G1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_L_G3_OFT_OFFSET		(TSP_TUNE_CODE_L_G2_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_GOFT_OFFSET		(TSP_TUNE_CODE_L_G3_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_M1_OFT_OFFSET		(TSP_TUNE_CODE_R_GOFT_OFFSET + 2)
#define TSP_TUNE_CODE_R_G1_OFT_OFFSET		(TSP_TUNE_CODE_R_M1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_G2_OFT_OFFSET		(TSP_TUNE_CODE_R_G1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_G3_OFT_OFFSET		(TSP_TUNE_CODE_R_G2_OFT_OFFSET + tc_total_ch_size)
#define PATH_SIZE		64
#define BURST_SIZE		512
#define RAWDATA_SIZE		2
#define ROW_SIZE		34
#define COL_SIZE		18
#define M1_COL_SIZE		2
#define LOG_BUF_SIZE		256
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum {
	CMD_RAWDATA = 1,
	CMD_DELTADATA,
	CMD_LABELDATA,
	CMD_BASEDATA,
	CMD_ALGORITHM,
	CMD_TCMDATA,
};
enum {
	NO_TEST = 0,
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,
	SHORT_NODE_TEST,
	U3_M2_RAWDATA_TEST = 5,
	U3_M1_RAWDATA_TEST = 6,
	U0_M2_RAWDATA_TEST,
	U0_M1_RAWDATA_TEST,
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

extern void touch_msleep(unsigned int msecs);
int lg4895_prd_register_sysfs(struct device *dev);

#endif


