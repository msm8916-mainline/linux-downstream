/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* 
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
/*
 * Definitions for tmd2771 als/ps sensor chip.
 */
#ifndef __TMD2771_H__
#define __TMD2771_H__

#include <linux/ioctl.h>

extern int TMD2771_CMM_PPCOUNT_VALUE;
extern int ZOOM_TIME;

#define TMD2771_CMM_DEVICEID    0x92
#define TMD2771_CMM_ENABLE 		0X80
#define TMD2771_CMM_ATIME 		0X81
#define TMD2771_CMM_PTIME 		0X82
#define TMD2771_CMM_WTIME 		0X83
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
#define TMD2771_CMM_INT_LOW_THD_LOW   0X88
#define TMD2771_CMM_INT_LOW_THD_HIGH  0X89
#define TMD2771_CMM_INT_HIGH_THD_LOW  0X8A
#define TMD2771_CMM_INT_HIGH_THD_HIGH 0X8B
#define TMD2771_CMM_Persistence       0X8C
#define TMD2771_CMM_STATUS            0X93
#define TAOS_TRITON_CMD_REG           0X80
#define TAOS_TRITON_CMD_SPL_FN        0x60

#define TMD2771_CMM_CONFIG 		0X8D
#define TMD2771_CMM_PPCOUNT 		0X8E
#define TMD2771_CMM_CONTROL 		0X8F

#define TMD2771_CMM_PDATA_L 		0X98
#define TMD2771_CMM_PDATA_H 		0X99
#define TMD2771_CMM_C0DATA_L 	0X94
#define TMD2771_CMM_C0DATA_H 	0X95
#define TMD2771_CMM_C1DATA_L 	0X96
#define TMD2771_CMM_C1DATA_H 	0X97
#define TMD2771_CMM_OFFSET		0X9E


#define TMD2771_SUCCESS						0
#define TMD2771_ERR_I2C						-1
#define TMD2771_ERR_STATUS					-3
#define TMD2771_ERR_SETUP_FAILURE				-4
#define TMD2771_ERR_GETGSENSORDATA			-5
#define TMD2771_ERR_IDENTIFICATION			-6

#define C_CUST_ALS_LEVEL    16
#define C_CUST_I2C_ADDR_NUM 4

struct alsps_hw {
    int i2c_num;                                    /*!< the i2c bus used by ALS/PS */
	int polling_mode;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_ps;                               /*!< 1: polling mode ; 0:interrupt mode*/
	int polling_mode_als;                               /*!< 1: polling mode ; 0:interrupt mode*/
    unsigned char   i2c_addr[C_CUST_I2C_ADDR_NUM];  /*!< i2c address list, some chip will have multiple address */
    unsigned int    als_level[C_CUST_ALS_LEVEL-1];  /*!< (C_CUST_ALS_LEVEL-1) levels divides all range into C_CUST_ALS_LEVEL levels*/
    unsigned int    als_value[C_CUST_ALS_LEVEL];    /*!< the value reported in each level */
    unsigned int    ps_threshold;                   /*!< the threshold of proximity sensor */
	unsigned int    als_window_loss;                /*!< the window loss  */
	unsigned int    ps_threshold_high;
	unsigned int    ps_threshold_low;
	unsigned int    als_threshold_high;
	unsigned int    als_threshold_low;
	#if defined(BBK_FEATURE_PROXIMITY_GESTURE)
		int poling_mode_gs;
	#endif
};

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    .als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    .ps_threshold_high = 200,
    .ps_threshold_low = 150,
};

static struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

#endif
