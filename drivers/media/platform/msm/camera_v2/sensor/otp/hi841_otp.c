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

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_otp.h"
#include "msm.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug("%s %d "fmt, __func__, __LINE__, ##args)
#include <linux/device.h>

/**
  * read_xxx_otp_memory() - read map data into buffer
  * @o_ctrl:	otp control struct
  * @block:	block to be read
  *
  * This function iterates through blocks stored in block->map, reads each
  * region and concatenate them into the pre-allocated block->mapdata
  */
static int32_t hi841_read_otp_memory(struct msm_otp_ctrl_t *o_ctrl,
			      struct msm_otp_memory_block_t *block)
{
	int rc = 0;
	int j;
	struct msm_otp_memory_map_t *emap = block->map;
	struct msm_otp_board_info *ob_info;
	uint8_t *memptr = block->mapdata;
	uint16_t chipid = 0;

	if (!o_ctrl) {
		pr_err("%s o_ctrl is NULL", __func__);
		return -EINVAL;
	}

	ob_info = o_ctrl->oboard_info;

	for (j = 0; j < block->num_map; j++) {
		if (emap[j].saddr.addr) {
			ob_info->i2c_slaveaddr = emap[j].saddr.addr;
			o_ctrl->i2c_client.cci_client->sid =
					ob_info->i2c_slaveaddr >> 1;
			pr_err("qcom,slave-addr = 0x%X\n",
				ob_info->i2c_slaveaddr);
		}

		if (emap[j].mem.valid_size) {
			o_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&(o_ctrl->i2c_client), 0x0,
					&chipid, MSM_CAMERA_I2C_WORD_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c fail\n", __func__, __LINE__);
				return rc;
			}

			pr_info("%s %d chipid = 0x%x\n", __func__, __LINE__, chipid);

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x8400,
					0x01, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x9c04,
					(0xa891-0xa000), MSM_CAMERA_I2C_WORD_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(o_ctrl->i2c_client),
				0x9c00,
				0x11, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(o_ctrl->i2c_client), emap[j].mem.addr,
				memptr, emap[j].mem.valid_size);

			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x8400,
					0x00, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}
		}
	}

	return rc;
}

static int32_t hi841_otp_checksum(struct msm_otp_ctrl_t *o_ctrl) {
	uint32_t k, awb_datasum_5k, awb_datasum_3k;
	uint32_t awb_checksum_5k, lsc_checksum_5k, lsc_checksum_4k, awb_checksum_3k;
	uint32_t lsc_datasum_5k, lsc_datasum_4k, lsc_cal_5k, lsc_cal_4k;
	uint32_t total_datasum, total_checksum;
	int32_t rc = -EFAULT;

	lsc_datasum_5k = 0;
	lsc_datasum_4k = 0;
	lsc_cal_5k = 0;
	lsc_cal_4k = 0;
	total_datasum = 0;
	total_checksum = 0;

	awb_datasum_5k = 0;
	for(k = 0; k < 0x06; k++) {
		awb_datasum_5k += o_ctrl->cal_data.mapdata[k];
	}
	awb_checksum_5k = (o_ctrl->cal_data.mapdata[0x0006]*256) + o_ctrl->cal_data.mapdata[0x0007];

	awb_datasum_3k = 0;
	for(k = 0x382; k < 0x388; k++) {
		awb_datasum_3k += o_ctrl->cal_data.mapdata[k];
	}
	awb_checksum_3k = (o_ctrl->cal_data.mapdata[0x388]*256) + o_ctrl->cal_data.mapdata[0x389];

	for (k = 0x0C; k < 0x380; k++) {
		lsc_datasum_5k += o_ctrl->cal_data.mapdata[k];
		lsc_cal_5k = lsc_datasum_5k & 0xffff;
	}

	for (k = 0x38A; k < 0x6fE; k++) {
		lsc_datasum_4k += (o_ctrl->cal_data.mapdata[k]);
		lsc_cal_4k = lsc_datasum_4k & 0xffff;
	}
	lsc_checksum_5k = (o_ctrl->cal_data.mapdata[0x0380]*256) + o_ctrl->cal_data.mapdata[0x0381];
	lsc_checksum_4k = (o_ctrl->cal_data.mapdata[0x06FE]*256) + o_ctrl->cal_data.mapdata[0x06FF];

	for(k = 0x0; k < 0x7fC; k++) {
		total_datasum += o_ctrl->cal_data.mapdata[k];
		total_datasum &= 0xffffff;
	}
	total_checksum = (o_ctrl->cal_data.mapdata[0x07FC] << 24) |
		(o_ctrl->cal_data.mapdata[0x07FD] << 16) |
		(o_ctrl->cal_data.mapdata[0x07FE] << 8) |
		(o_ctrl->cal_data.mapdata[0x07FF] << 0);

	pr_info("%s %d verify otp data, id = 0x%x, name = %s\n",
	 __func__, __LINE__, o_ctrl->cal_data.mapdata[0x700], o_ctrl->oboard_info->otp_name);

	if((awb_datasum_5k == awb_checksum_5k) && (awb_datasum_3k == awb_checksum_3k)
			&& (lsc_cal_5k == lsc_checksum_5k) && (lsc_cal_4k == lsc_checksum_4k)
			&& (total_datasum == total_checksum)){
		pr_info("%s otp checksum success\n", __func__);
		rc = 0;
	} else {
		pr_err("awb_datasum_5k = %d, awb_checksum_5k = %d\n", awb_datasum_5k, awb_checksum_5k);
		pr_err("awb_datasum_3k = %d, awb_checksum_3k = %d\n", awb_datasum_3k, awb_checksum_3k);
		pr_err("lsc_datasum_5k = %d, lsc_cal_5k = %d, lsc_checksum_5k = %d\n", lsc_datasum_5k, lsc_cal_5k, lsc_checksum_5k);
		pr_err("lsc_datasum_4k = %d, lsc_cal_4k = %d, lsc_checksum_4k = %d\n", lsc_datasum_4k, lsc_cal_4k, lsc_checksum_4k);
	}
	return rc;
};

static const struct of_device_id hi841_otp_dt_match[] = {
	{ .compatible = "sk,hi841-otp" },
	{ }
};

MODULE_DEVICE_TABLE(of, hi841_otp_dt_match);

static struct platform_driver hi841_otp_platform_driver = {
	.driver = {
		.name = "hi841,otp",
		.owner = THIS_MODULE,
		.of_match_table = hi841_otp_dt_match,
	},
	.remove = msm_otp_platform_remove,
};

struct msm_otp_fn_t hi841_otp_func = {
	.otp_read = hi841_read_otp_memory,
	.otp_checksum = hi841_otp_checksum,
};

static int32_t hi841_otp_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	CDBG("E\n");
	rc = msm_otp_platform_probe(pdev, &hi841_otp_func);
	CDBG("msm_otp_platform_probe rc = %d\n", rc);
	return rc;
}


static int32_t __init hi841_otp_init_module(void)
{
	int rc = 0;

	CDBG("E\n");
	rc = platform_driver_probe(&hi841_otp_platform_driver,
		hi841_otp_platform_probe);
	CDBG("platform rc %d\n", rc);

	return rc;
}

static void __exit hi841_otp_exit_module(void)
{
	platform_driver_unregister(&hi841_otp_platform_driver);
}

module_init(hi841_otp_init_module);
module_exit(hi841_otp_exit_module);
MODULE_DESCRIPTION("HI841 OTP driver");
MODULE_LICENSE("GPL v2");
