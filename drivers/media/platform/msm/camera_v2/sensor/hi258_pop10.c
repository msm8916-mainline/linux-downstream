/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *         Modify History For This Module
 * When           Who             What,Where,Why
 * --------------------------------------------------------------------------------------
 * 14/05/13      Lu Zhi        correct prview orientation
 * 14/05/26      ZKX           adjust camera subjective params, like, decrease Saturation, denoise
 * 14/05/27      ZKX           add powerdown seq
 * 14/05/29      Lu Zhi        modify cam clk for best match
 * 14/06/10      ZKX           fixed snapshot exposure time is too much
 * --------------------------------------------------------------------------------------
*/

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define HI258_POP10_SENSOR_NAME "hi258_pop10"
#define PLATFORM_DRIVER_NAME "msm_camera_hi258_pop10"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
//#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) printk(fmt, ##args)
//#else
//#define CDBG(fmt, args...) do { } while (0)
//#endif

DEFINE_MSM_MUTEX(hi258_pop10_mut);
static struct msm_sensor_ctrl_t hi258_pop10_s_ctrl;

static struct msm_sensor_power_setting hi258_pop10_power_setting[] = {
{
	.seq_type = SENSOR_VREG,
	.seq_val = CAM_VDIG,
	.config_val = 1,
	.delay = 0,
},
{
	.seq_type = SENSOR_VREG,
	.seq_val = CAM_VIO,	.config_val = 1,
	.delay = 0,
},
{
	.seq_type = SENSOR_VREG,
	.seq_val = CAM_VANA,
	.config_val = 1,
	.delay = 0,
},
{
	.seq_type = SENSOR_GPIO,
	.seq_val = SENSOR_GPIO_RESET,
	.config_val = GPIO_OUT_LOW,
	.delay = 1,
},
{
	.seq_type = SENSOR_GPIO,
	.seq_val = SENSOR_GPIO_RESET,
	.config_val = GPIO_OUT_HIGH,
	.delay = 30,
},
{
	.seq_type = SENSOR_GPIO,
	.seq_val = SENSOR_GPIO_STANDBY,
	.config_val = GPIO_OUT_LOW,
	.delay = 1,
},
{
	.seq_type = SENSOR_GPIO,
	.seq_val = SENSOR_GPIO_STANDBY,
	.config_val = GPIO_OUT_HIGH,
	.delay = 30,
},
{
	.seq_type = SENSOR_CLK,
	.seq_val = SENSOR_CAM_MCLK,
	.config_val = 23880000,
	.delay = 1,
},
{
	.seq_type = SENSOR_I2C_MUX,
	.seq_val = 0,
	.config_val = 0,
	.delay = 1,
},
};

#if 1
/*[BUGFIX]-Mod-BEGIN by TCTSZ.ZKX, Add power down sequence, 2014/05/27 */
static struct msm_sensor_power_setting hi258_pop10_power_down_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,   // this is need double check
		.delay = 3,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO, 
		.config_val = 0,
		.delay = 9,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 9,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 30,
	},
};
/*[BUGFIX]-Mod-END by TCTSZ.ZKX */
#endif


static struct msm_camera_i2c_reg_conf hi258_pop10_preview_settings[] = {
{0x03, 0x00},
{0x01, 0x01}, //HI258_Sleep_Mode

{0x03, 0x20},
{0x10, 0x1c},
{0x03, 0x22},
{0x10, 0x69},

{0x03, 0x00},
{0x10, 0x10}, // preview mode off
{0x12, 0x04},

{0x03, 0x20},
{0x86, 0x01}, //EXPMin 6500.00 fps
{0x87, 0xf4},

{0x8B, 0xea}, //EXP100
{0x8C, 0x60},
{0x8D, 0xc3}, //EXP120
{0x8E, 0x50},

{0x9c, 0x21}, //EXP Limit 1083.33fps
{0x9d, 0x34},
{0x9e, 0x01}, //EXP Unit
{0x9f, 0xf4},

//PLL Setting
{0x03, 0x00},
{0xd0, 0x05}, //PLL pre_div 1/6 = 4 Mhz
{0xd1, 0x30}, //PLL maim_div
{0xd2, 0x05}, //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]
{0xd3, 0x20}, //isp_clk_inv[0]  mipi_4x_inv[1]  mipi_1x_inv[2]
{0xd0, 0x85},
{0xd0, 0x85},
{0xd0, 0x85},
{0xd0, 0x95},

{0x03, 0x00}, //Dummy 750us
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},

// MIPI TX Setting //
{0x03, 0x48},
{0x30, 0x06},
{0x31, 0x40},
{0x01, 0x00}, //HI258_Sleep_Mode

{0x03, 0x20},//page 20
{0x10, 0x9c},//AE ON

{0x03, 0x22},
{0x10, 0xe9},//AWB ON
};



static struct msm_camera_i2c_reg_conf hi258_pop10_snapshot_settings[] = {
/* 2M FULL Mode */
{0x03,0x00},
{0x01, 0x01},  // HI258_Sleep_Mode

{0x03, 0x00},
{0x20, 0x00},
{0x21, 0x0a},
{0x22, 0x00},
{0x23, 0x0a},

{0x40, 0x01},  //Hblank 360
{0x41, 0x78},
{0x42, 0x00},  //Vblank 18
{0x43, 0x14},

// 1600*1200
{0x03, 0x00},
{0x10, 0x00},
{0xd2, 0x01},  //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]

{0x03, 0x48},
{0x72, 0x81},
{0x30, 0x0c},
{0x31, 0x80},

{0x03, 0x00},
{0x12, 0x04},  // CLK_DIV_REG

//{0x03, 0x20},
//{0x83, 0x09},
//{0x84, 0x27},
//{0x85, 0xc0},

{0x03,0x00},
{0x01, 0x00},  // HI258_Sleep_Mode
};

/*[BUGFIX]-Mod-BEGIN by TCTNB.WPL, bug564473, 2013/11/30 */
#if 0
static struct msm_camera_i2c_reg_conf hi258_pop10_start_settings[] = {
	{0x03, 0x00},
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_conf hi258_pop10_stop_settings[] = {
	{0x03, 0x00},
	{0x01, 0x01},
};
#endif
/*[BUGFIX]-Mod-END by TCTNB.WPL */

static struct msm_camera_i2c_reg_conf hi258_pop10_recommend_settings[] = {
{0x01, 0x01}, //sleep on
{0x01, 0x03}, //sleep off
{0x01, 0x01}, //sleep on
// PAGE 20
{0x03, 0x20}, // page 20
{0x10, 0x1c}, // AE off 60hz

// PAGE 22
{0x03, 0x22}, // page 22
{0x10, 0x69}, // AWB off

{0x03, 0x00}, //Dummy 750us
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},

{0x08, 0x00},
{0x09, 0x77},		// pad strength = max
{0x0a, 0x07},		// pad strength = max

//PLL Setting
{0x03, 0x00},
{0xd0, 0x05}, //PLL pre_div 1/6 = 4 Mhz
{0xd1, 0x30}, //PLL maim_div
{0xd2, 0x05}, //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]
{0xd3, 0x20}, //isp_clk_inv[0]  mipi_4x_inv[1]  mipi_1x_inv[2]
{0xd0, 0x85},
{0xd0, 0x85},
{0xd0, 0x85},
{0xd0, 0x95},

{0x03, 0x00}, //Dummy 750us
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},

///// PAGE 20 /////
{0x03, 0x20}, //page 20
{0x10, 0x1c}, //AE off 50hz

///// PAGE 22 /////
{0x03, 0x22}, //page 22
{0x10, 0x69}, //AWB off

///// Initial Start /////
///// PAGE 0 Start /////
{0x03, 0x00}, //page 0
{0x10, 0x10},
{0x11, 0x90}, //Windowing On + 1Frame Skip
{0x12, 0x04}, //Rinsing edge 0x04 // Falling edge 0x00
{0x14, 0x05},

{0x20, 0x00}, //Row H
{0x21, 0x0a}, //Row L
{0x22, 0x00}, //Col H
{0x23, 0x0a}, //Col L

{0x24, 0x04}, //Window height_H //= 1200
{0x25, 0xb0}, //Window height_L //
{0x26, 0x06}, //Window width_H  //= 1600
{0x27, 0x40}, //Window wight_L

{0x28, 0x04}, //Full row start y-flip
{0x29, 0x01}, //Pre1 row start no-flip
{0x2a, 0x02}, //Pre1 row start y-flip

{0x2b, 0x04}, //Full wid start x-flip
{0x2c, 0x04}, //Pre2 wid start no-flip
{0x2d, 0x02}, //Pre2 wid start x-flip

{0x40, 0x01}, //Hblank_456
{0x41, 0x78},
{0x42, 0x00}, //Vblank
{0x43, 0x14}, //Flick Stop

{0x50, 0x00}, //Test Pattern

///// BLC /////
{0x80, 0x2e},
{0x81, 0x7e},
{0x82, 0x90},
{0x83, 0x00},
{0x84, 0xcc}, //20130604 0x0c->0xcc
{0x85, 0x00},
{0x86, 0x00},
{0x87, 0x0f},
{0x88, 0x34},
{0x8a, 0x0b},
{0x8e, 0x80}, //Pga Blc Hold

{0x90, 0x0a}, //BLC_TIME_TH_ON
{0x91, 0x0a}, //BLC_TIME_TH_OFF
{0x92, 0x68}, //BLC_AG_TH_ON
{0x93, 0x60}, //BLC_AG_TH_OFF
{0x96, 0xdc}, //BLC Outdoor Th On
{0x97, 0xfe}, //BLC Outdoor Th Off
{0x98, 0x38},

//OutDoor  BLC
{0x99, 0x43}, //R,Gr,B,Gb Offset

//Dark BLC
{0xa0, 0x01}, //R,Gr,B,Gb Offset

//Normal BLC
{0xa8, 0x43}, //R,Gr,B,Gb Offset
///// PAGE 0 END /////

///// PAGE 2 START /////
{0x03, 0x02},
{0x10, 0x00},
{0x13, 0x00},
{0x14, 0x00},
{0x18, 0xcc},
{0x19, 0x01}, // pmos switch on (for cfpn)
{0x1A, 0x39}, //20130604 0x09->0xcc
{0x1B, 0x00},
{0x1C, 0x1a}, // for ncp
{0x1D, 0x14}, // for ncp
{0x1E, 0x30}, // for ncp
{0x1F, 0x10},

{0x20, 0x77},
{0x21, 0xde},
{0x22, 0xa7},
{0x23, 0x30},
{0x24, 0x77},
{0x25, 0x10},
{0x26, 0x10},
{0x27, 0x3c},
{0x2b, 0x80},
{0x2c, 0x02},
{0x2d, 0x58},
{0x2e, 0x11},//20130604 0xde->0x11
{0x2f, 0x11},//20130604 0xa7->0x11

{0x30, 0x00},
{0x31, 0x99},
{0x32, 0x00},
{0x33, 0x00},
{0x34, 0x22},
{0x36, 0x75},
{0x38, 0x88},
{0x39, 0x88},
{0x3d, 0x03},
{0x3f, 0x02},

{0x49, 0xc1},//20130604 0x87->0xd1 --> mode Change Issue modify -> 0xc1
{0x4a, 0x10},

{0x50, 0x21},
{0x53, 0xb1},
{0x54, 0x10},
{0x55, 0x1c}, // for ncp
{0x56, 0x11},
{0x58, 0x3a},//20130604 add
{0x59, 0x38},//20130604 add
{0x5d, 0xa2},
{0x5e, 0x5a},

{0x60, 0x87},
{0x61, 0x98},
{0x62, 0x88},
{0x63, 0x96},
{0x64, 0x88},
{0x65, 0x96},
{0x67, 0x3f},
{0x68, 0x3f},
{0x69, 0x3f},

{0x72, 0x89},
{0x73, 0x95},
{0x74, 0x89},
{0x75, 0x95},
{0x7C, 0x84},
{0x7D, 0xaf},

{0x80, 0x01},
{0x81, 0x7a},
{0x82, 0x13},
{0x83, 0x24},
{0x84, 0x78},
{0x85, 0x7c},

{0x92, 0x44},
{0x93, 0x59},
{0x94, 0x78},
{0x95, 0x7c},

{0xA0, 0x02},
{0xA1, 0x74},
{0xA4, 0x74},
{0xA5, 0x02},
{0xA8, 0x85},
{0xA9, 0x8c},
{0xAC, 0x10},
{0xAD, 0x16},

{0xB0, 0x99},
{0xB1, 0xa3},
{0xB4, 0x9b},
{0xB5, 0xa2},
{0xB8, 0x9b},
{0xB9, 0x9f},
{0xBC, 0x9b},
{0xBD, 0x9f},

{0xc4, 0x29},
{0xc5, 0x40},
{0xc6, 0x5c},
{0xc7, 0x72},
{0xc8, 0x2a},
{0xc9, 0x3f},
{0xcc, 0x5d},
{0xcd, 0x71},

{0xd0, 0x10},
{0xd1, 0x14},
{0xd2, 0x20},
{0xd3, 0x00},
{0xd4, 0x0a}, //DCDC_TIME_TH_ON
{0xd5, 0x0a}, //DCDC_TIME_TH_OFF 
{0xd6, 0x68}, //DCDC_AG_TH_ON
{0xd7, 0x60}, //DCDC_AG_TH_OFF
{0xdc, 0x00},
{0xdd, 0xa3},
{0xde, 0x00},
{0xdf, 0x84},

{0xe0, 0xa4},
{0xe1, 0xa4},
{0xe2, 0xa4},
{0xe3, 0xa4},
{0xe4, 0xa4},
{0xe5, 0x01},
{0xe8, 0x00},
{0xe9, 0x00},
{0xea, 0x77},

{0xF0, 0x00},
{0xF1, 0x00},
{0xF2, 0x00},
///// PAGE 2 END /////

///// PAGE 10 START /////
{0x03, 0x10}, //page 10
{0x10, 0x03}, //S2D enable _ YUYV Order
{0x11, 0x03},
{0x12, 0x30},
{0x13, 0x03},

{0x20, 0x00},
{0x21, 0x40},
{0x22, 0x0f},
{0x24, 0x20},
{0x25, 0x10},
{0x26, 0x01},
{0x27, 0x02},
{0x28, 0x11},

{0x40, 0x85},
{0x41, 0x00}, //D-YOffset Th
{0x42, 0x00}, //Cb Offset
{0x43, 0x00}, //Cr Offset
{0x44, 0x80},
{0x45, 0x80},
{0x46, 0xf0},
{0x48, 0x86},
{0x4a, 0x80},

{0x50, 0xa0}, //D-YOffset AG

{0x60, 0x4f},
{0x61, 0x93}, //Sat B
{0x62, 0x98}, //Sat R
{0x63, 0x80}, //Auto-De Color

{0x66, 0x42},
{0x67, 0x22},//11
{0x68, 0x78},
{0x69, 0x78},

{0x6a, 0x8a}, //White Protection Offset Dark/Indoor
{0x74, 0x03}, //White Protection Offset Outdoor
{0x76, 0x01}, //White Protection Enable
///// PAGE 10 END /////

///// PAGE 11 START /////
{0x03, 0x11}, //page 11

//LPF Auto Control
{0x20, 0x00},
{0x21, 0x00},
{0x26, 0x5a}, // Double_AG
{0x27, 0x58}, // Double_AG
{0x28, 0x0f},
{0x29, 0x10},
{0x2b, 0x30},
{0x2c, 0x32},

//GBGR
{0x70, 0x2b},
{0x74, 0x30},
{0x75, 0x18},
{0x76, 0x30},
{0x77, 0xff},
{0x78, 0xa0},
{0x79, 0xff}, //Dark GbGr Th
{0x7a, 0x30},
{0x7b, 0x20},
{0x7c, 0xf4}, //Dark Dy Th B[7:4]
{0x7d, 0x02},
{0x7e, 0xb0},
{0x7f, 0x10},
///// PAGE 11 END /////

///// PAGE 12 START /////
{0x03, 0x12}, //page 11

//YC2D
{0x10, 0x03}, //Y DPC Enable
{0x11, 0x08}, //
{0x12, 0x10}, //0x30 -> 0x10
{0x20, 0x52}, //Y_lpf_enable
{0x21, 0x03}, //C_lpf_enable_on
{0x22, 0xe6}, //YC2D_CrCbY_Dy

{0x23, 0x14}, //Outdoor Dy Th
{0x24, 0x20}, //Indoor Dy Th // For reso Limit 0x20
{0x25, 0x30}, //Dark Dy Th

//Outdoor LPF Flat
{0x30, 0xff}, //Y Hi Th
{0x31, 0x00}, //Y Lo Th
{0x32, 0xf0}, //Std Hi Th //Reso Improve Th Low //50
{0x33, 0x00}, //Std Lo Th
{0x34, 0x00}, //Median ratio

//Indoor LPF Flat
{0x35, 0xff}, //Y Hi Th
{0x36, 0x00}, //Y Lo Th
{0x37, 0xff}, //Std Hi Th //Reso Improve Th Low //50
{0x38, 0x00}, //Std Lo Th
{0x39, 0x00}, //Median ratio

//Dark LPF Flat
{0x3a, 0xff}, //Y Hi Th
{0x3b, 0x00}, //Y Lo Th
{0x3c, 0x93}, //Std Hi Th //Reso Improve Th Low //50
{0x3d, 0x00}, //Std Lo Th
{0x3e, 0x00}, //Median ratio

//Outdoor Cindition
{0x46, 0xa0}, //Out Lum Hi
{0x47, 0x40}, //Out Lum Lo

//Indoor Cindition
{0x4c, 0xb0}, //Indoor Lum Hi
{0x4d, 0x40}, //Indoor Lum Lo

//Dark Cindition
{0x52, 0xb0}, //Dark Lum Hi
{0x53, 0x50}, //Dark Lum Lo

//C-Filter
{0x70, 0x10}, //Outdoor(2:1) AWM Th Horizontal
{0x71, 0x0a}, //Outdoor(2:1) Diff Th Vertical
{0x72, 0x10}, //Indoor,Dark1 AWM Th Horizontal
{0x73, 0x0a}, //Indoor,Dark1 Diff Th Vertical
{0x74, 0x18}, //Dark(2:3) AWM Th Horizontal
{0x75, 0x0f}, //Dark(2:3) Diff Th Vertical

//DPC
{0x90, 0x7d},
{0x91, 0x34},
{0x99, 0x28},
{0x9c, 0x1e},
{0x9d, 0x15},
{0x9e, 0x28},
{0x9f, 0x28},
{0xb0, 0x0e}, //Zipper noise Detault change (0x75->0x0e)
{0xb8, 0x44},
{0xb9, 0x15},
///// PAGE 12 END /////

///// PAGE 13 START /////
{0x03, 0x13}, //page 13

{0x80, 0xfd}, //Sharp2D enable _ YUYV Order
{0x81, 0x07}, //Sharp2D Clip/Limit
{0x82, 0x63}, //Sharp2D Filter
{0x83, 0x00}, //Sharp2D Low Clip
{0x85, 0x00},

{0x92, 0x33}, //Sharp2D Slop n/p
{0x93, 0x30}, //Sharp2D LClip
{0x94, 0x02}, //Sharp2D HiClip1 Th
{0x95, 0xf0}, //Sharp2D HiClip2 Th
{0x96, 0x1e}, //Sharp2D HiClip2 Resolution
{0x97, 0x40},
{0x98, 0x80},
{0x99, 0x40},

//Sharp Lclp
{0xa2, 0x03}, //Outdoor Lclip_N
{0xa3, 0x04}, //Outdoor Lclip_P
{0xa4, 0x02}, //Indoor Lclip_N 0x03 For reso Limit 0x0e
{0xa5, 0x03}, //Indoor Lclip_P 0x0f For reso Limit 0x0f
{0xa6, 0x80}, //Dark Lclip_N
{0xa7, 0x80}, //Dark Lclip_P

//Outdoor Slope
{0xb6, 0x28}, //Lum negative Hi
{0xb7, 0x20}, //Lum negative middle
{0xb8, 0x24}, //Lum negative Low
{0xb9, 0x28}, //Lum postive Hi
{0xba, 0x20}, //Lum postive middle
{0xbb, 0x24}, //Lum postive Low

//Indoor Slope
{0xbc, 0x28}, //Lum negative Hi
{0xbd, 0x20}, //Lum negative middle
{0xbe, 0x24}, //Lum negative Low
{0xbf, 0x28}, //Lum postive Hi
{0xc0, 0x20}, //Lum postive middle
{0xc1, 0x24}, //Lum postive Low

//Dark Slope
{0xc2, 0x14}, //Lum negative Hi
{0xc3, 0x24}, //Lum negative middle
{0xc4, 0x1d}, //Lum negative Low
{0xc5, 0x14}, //Lum postive Hi
{0xc6, 0x24}, //Lum postive middle
{0xc7, 0x1d}, //Lum postive Low
///// PAGE 13 END /////

///// PAGE 14 START /////
{0x03, 0x14}, //page 14
{0x10, 0x09},

{0x20, 0x80}, //X-Center
{0x21, 0xc0}, //Y-Center

{0x22, 0x53}, //LSC R 1b->15 20130125   58
{0x23, 0x3b}, //LSC G   3c  37
{0x24, 0x35}, //LSC B   38  36

{0x25, 0xf0}, //LSC Off
{0x26, 0xf0}, //LSC On
///// PAGE 14 END /////

/////// PAGE 15 START ///////
{0x03, 0x15}, //page 15
{0x10, 0x0f},
{0x14, 0x52},
{0x15, 0x30},	//CMCOFSGM
{0x16, 0x20},	//CMCOFSGL
{0x17, 0x2f},

//CMC
{0x30, 0x89},
{0x31, 0x5c},
{0x32, 0x13},
{0x33, 0x17},
{0x34, 0x5c},
{0x35, 0x05},
{0x36, 0x07},
{0x37, 0x45},
{0x38, 0x8c},

//CMC OFS
{0x40, 0x08},
{0x41, 0x84},
{0x42, 0x85},
{0x43, 0x04},
{0x44, 0x05},
{0x45, 0x89},
{0x46, 0x90},
{0x47, 0x8b},
{0x48, 0x1b},

//CMC POFS
{0x50, 0x83},
{0x51, 0x85},
{0x52, 0x08},
{0x53, 0x03},
{0x54, 0x09},
{0x55, 0x8d},
{0x56, 0x00},
{0x57, 0x07},
{0x58, 0x87},
///// PAGE 15 END /////

///// PAGE 16 START /////
{0x03, 0x16}, //page 16
{0x10, 0x31},
{0x18, 0x5a}, //Double_AG 5e->37
{0x19, 0x58}, //Double_AG 5e->36
{0x1a, 0x0e},
{0x1b, 0x01},
{0x1c, 0xdc},
{0x1d, 0xfe},

//Indoor
{0x30, 0x00},
{0x31, 0x0b},
{0x32, 0x19},
{0x33, 0x2e},
{0x34, 0x5c},
{0x35, 0x78},
{0x36, 0x90},
{0x37, 0xa1},
{0x38, 0xaf},
{0x39, 0xb9},
{0x3a, 0xc1},
{0x3b, 0xca},
{0x3c, 0xd5},
{0x3d, 0xde},
{0x3e, 0xe6},
{0x3f, 0xec},
{0x40, 0xf4},
{0x41, 0xfa},
{0x42, 0xff},

//Outdoor
{0x50, 0x00},
{0x51, 0x03},
{0x52, 0x10},
{0x53, 0x26},
{0x54, 0x43},
{0x55, 0x5d},
{0x56, 0x79},
{0x57, 0x8c},
{0x58, 0x9f},
{0x59, 0xaa},
{0x5a, 0xb6},
{0x5b, 0xc3},
{0x5c, 0xce},
{0x5d, 0xd9},
{0x5e, 0xe1},
{0x5f, 0xe9},
{0x60, 0xf0},
{0x61, 0xf4},
{0x62, 0xf5},

//Dark
{0x70, 0x00},
{0x71, 0x10},
{0x72, 0x1c},
{0x73, 0x2e},
{0x74, 0x4e},
{0x75, 0x6c},
{0x76, 0x82},
{0x77, 0x96},
{0x78, 0xa7},
{0x79, 0xb6},
{0x7a, 0xc4},
{0x7b, 0xd0},
{0x7c, 0xda},
{0x7d, 0xe2},
{0x7e, 0xea},
{0x7f, 0xf0},
{0x80, 0xf6},
{0x81, 0xfa},
{0x82, 0xff},
///// PAGE 16 END /////

///// PAGE 17 START /////
{0x03, 0x17}, //page 17
{0xc1, 0x00},
{0xc4, 0x4b},
{0xc5, 0x3f},
{0xc6, 0x02},
{0xc7, 0x20},
///// PAGE 17 END /////

///// PAGE 18 START /////
{0x03, 0x18}, //page 18
{0x10, 0x00},	//Scale Off
{0x11, 0x00},
{0x12, 0x58},
{0x13, 0x01},
{0x14, 0x00}, //Sawtooth
{0x15, 0x00},
{0x16, 0x00},
{0x17, 0x00},
{0x18, 0x00},
{0x19, 0x00},
{0x1a, 0x00},
{0x1b, 0x00},
{0x1c, 0x00},
{0x1d, 0x00},
{0x1e, 0x00},
{0x1f, 0x00},
{0x20, 0x05},	//zoom wid
{0x21, 0x00},
{0x22, 0x01},	//zoom hgt
{0x23, 0xe0},
{0x24, 0x00},	//zoom start x
{0x25, 0x00},
{0x26, 0x00},	//zoom start y
{0x27, 0x00},
{0x28, 0x05},	//zoom end x
{0x29, 0x00},
{0x2a, 0x01},	//zoom end y
{0x2b, 0xe0},
{0x2c, 0x0a},	//zoom step vert
{0x2d, 0x00},
{0x2e, 0x0a},	//zoom step horz
{0x2f, 0x00},
{0x30, 0x44},	//zoom fifo

///// PAGE 18 END /////

{0x03, 0x19}, //Page 0x19
{0x10, 0x7f}, //mcmc_ctl1
{0x11, 0x7f}, //mcmc_ctl2
{0x12, 0x1e}, //mcmc_delta1
{0x13, 0x32}, //mcmc_center1
{0x14, 0x1e}, //mcmc_delta2
{0x15, 0x6e}, //mcmc_center2
{0x16, 0x0a}, //mcmc_delta3
{0x17, 0xb8}, //mcmc_center3
{0x18, 0x1e}, //mcmc_delta4
{0x19, 0xe6}, //mcmc_center4
{0x1a, 0x9e}, //mcmc_delta5
{0x1b, 0x22}, //mcmc_center5
{0x1c, 0x9e}, //mcmc_delta6
{0x1d, 0x5e}, //mcmc_center6
{0x1e, 0x3b}, //mcmc_sat_gain1
{0x1f, 0x3e}, //mcmc_sat_gain2
{0x20, 0x4b}, //mcmc_sat_gain3
{0x21, 0x52}, //mcmc_sat_gain4
{0x22, 0x59}, //mcmc_sat_gain5
{0x23, 0x46}, //mcmc_sat_gain6
{0x24, 0x00}, //mcmc_hue_angle1
{0x25, 0x01}, //mcmc_hue_angle2
{0x26, 0x0e}, //mcmc_hue_angle3
{0x27, 0x04}, //mcmc_hue_angle4
{0x28, 0x00}, //mcmc_hue_angle5
{0x29, 0x8c}, //mcmc_hue_angle6
{0x53, 0x10}, //mcmc_ctl3
{0x6c, 0xff}, //mcmc_lum_ctl1
{0x6d, 0x3f}, //mcmc_lum_ctl2
{0x6e, 0x00}, //mcmc_lum_ctl3
{0x6f, 0x00}, //mcmc_lum_ctl4
{0x70, 0x00}, //mcmc_lum_ctl5
{0x71, 0x3f}, //rg1_lum_gain_wgt_th1
{0x72, 0x3f}, //rg1_lum_gain_wgt_th2
{0x73, 0x3f}, //rg1_lum_gain_wgt_th3
{0x74, 0x3f}, //rg1_lum_gain_wgt_th4
{0x75, 0x30}, //rg1_lum_sp1
{0x76, 0x50}, //rg1_lum_sp2
{0x77, 0x80}, //rg1_lum_sp3
{0x78, 0xb0}, //rg1_lum_sp4
{0x79, 0x3f}, //rg2_gain_wgt_th1
{0x7a, 0x3f}, //rg2_gain_wgt_th2
{0x7b, 0x3f}, //rg2_gain_wgt_th3
{0x7c, 0x3f}, //rg2_gain_wgt_th4
{0x7d, 0x28}, //rg2_lum_sp1
{0x7e, 0x50}, //rg2_lum_sp2
{0x7f, 0x80}, //rg2_lum_sp3
{0x80, 0xb0}, //rg2_lum_sp4
{0x81, 0x28}, //rg3_gain_wgt_th1
{0x82, 0x3f}, //rg3_gain_wgt_th2
{0x83, 0x3f}, //rg3_gain_wgt_th3
{0x84, 0x3f}, //rg3_gain_wgt_th4
{0x85, 0x28}, //rg3_lum_sp1
{0x86, 0x50}, //rg3_lum_sp2
{0x87, 0x80}, //rg3_lum_sp3
{0x88, 0xb0}, //rg3_lum_sp4
{0x89, 0x1a}, //rg4_gain_wgt_th1
{0x8a, 0x28}, //rg4_gain_wgt_th2
{0x8b, 0x3f}, //rg4_gain_wgt_th3
{0x8c, 0x3f}, //rg4_gain_wgt_th4
{0x8d, 0x10}, //rg4_lum_sp1
{0x8e, 0x30}, //rg4_lum_sp2
{0x8f, 0x60}, //rg4_lum_sp3
{0x90, 0x90}, //rg4_lum_sp4
{0x91, 0x1a}, //rg5_gain_wgt_th1
{0x92, 0x28}, //rg5_gain_wgt_th2
{0x93, 0x3f}, //rg5_gain_wgt_th3
{0x94, 0x3f}, //rg5_gain_wgt_th4
{0x95, 0x28}, //rg5_lum_sp1
{0x96, 0x50}, //rg5_lum_sp2
{0x97, 0x80}, //rg5_lum_sp3
{0x98, 0xb0}, //rg5_lum_sp4
{0x99, 0x1a}, //rg6_gain_wgt_th1
{0x9a, 0x28}, //rg6_gain_wgt_th2
{0x9b, 0x3f}, //rg6_gain_wgt_th3
{0x9c, 0x3f}, //rg6_gain_wgt_th4
{0x9d, 0x28}, //rg6_lum_sp1
{0x9e, 0x50}, //rg6_lum_sp2
{0x9f, 0x80}, //rg6_lum_sp3
{0xa0, 0xb0}, //rg6_lum_sp4
{0xa1, 0x00}, //mcmc_allgain_ctl

{0xa2, 0x00},
{0xe5, 0x80}, //add 20120709 Bit[7] On MCMC --> YC2D_LPF

/////// PAGE 20 START ///////
{0x03, 0x20},
{0x10, 0x1c},
{0x11, 0x0c},//14
{0x18, 0x30},
{0x20, 0x25}, //8x8 Ae weight 0~7 Outdoor / Weight Outdoor On B[5]
{0x21, 0x30},
{0x22, 0x10},
{0x23, 0x00},

{0x28, 0xf7},
{0x29, 0x0d},
{0x2a, 0xff},
{0x2b, 0x34}, //Adaptive Off,1/100 Flicker

{0x2c, 0x83}, //AE After CI
{0x2d, 0xe3},
{0x2e, 0x13},
{0x2f, 0x0b},

{0x30, 0x78},
{0x31, 0xd7},
{0x32, 0x10},
{0x33, 0x2e},
{0x34, 0x20},
{0x35, 0xd4},
{0x36, 0xfe},
{0x37, 0x32},
{0x38, 0x04},
{0x39, 0x22},
{0x3a, 0xde},
{0x3b, 0x22},
{0x3c, 0xde},
{0x3d, 0xe1},

{0x3e, 0xc9}, //Option of changing Exp max
{0x41, 0x23}, //Option of changing Exp max

{0x50, 0x45},
{0x51, 0x88},

{0x56, 0x1f}, // for tracking
{0x57, 0xa6}, // for tracking
{0x58, 0x1a}, // for tracking
{0x59, 0x7a}, // for tracking

{0x5a, 0x04},
{0x5b, 0x04},

{0x5e, 0xc7},
{0x5f, 0x95},

{0x62, 0x10},
{0x63, 0xc0},
{0x64, 0x10},
{0x65, 0x8a},
{0x66, 0x58},
{0x67, 0x58},

{0x70, 0x52}, //6c
{0x71, 0x89}, //81(+4),89(-4)

{0x76, 0x32},
{0x77, 0xa1},
{0x78, 0x22}, //24
{0x79, 0x30}, // Y Target 70 => 25, 72 => 26 //
{0x7a, 0x23}, //23
{0x7b, 0x22}, //22
{0x7d, 0x23},

{0x83, 0x09}, //EXP Normal 10.00 fps 
{0x84, 0x27}, 
{0x85, 0xc0}, 
{0x86, 0x01}, //EXPMin 12000.00 fps
{0x87, 0xf4}, 
{0x88, 0x09}, //EXP Max 60hz 10.00 fps 
{0x89, 0x27}, 
{0x8a, 0xc0}, 
{0xa5, 0x09}, //EXP Max 50hz 10.00 fps 
{0xa6, 0x27}, 
{0xa7, 0xc0}, 
{0x8B, 0xea}, //EXP100 
{0x8C, 0x60}, 
{0x8D, 0xc3}, //EXP120 
{0x8E, 0x50}, 
{0x9c, 0x21}, //EXP Limit 857.14 fps 
{0x9d, 0x34}, 
{0x9e, 0x01}, //EXP Unit 
{0x9f, 0xf4}, 
{0xa3, 0x00}, //Outdoor Int 
{0xa4, 0xc3}, 


{0xb0, 0x18},
{0xb1, 0x14},
{0xb2, 0x70},
{0xb3, 0x18},
{0xb4, 0x1a},
{0xb5, 0x44},
{0xb6, 0x2f},
{0xb7, 0x28},
{0xb8, 0x25},
{0xb9, 0x22},
{0xba, 0x21},
{0xbb, 0x20},
{0xbc, 0x1f},
{0xbd, 0x1f},

{0xc0, 0x14},
{0xc1, 0x1f},
{0xc2, 0x1f},
{0xc3, 0x18},
{0xc4, 0x10},

{0xc6, 0x80}, //Exp max 1frame target AG

{0xc8, 0x80},
{0xc9, 0x80},
///// PAGE 20 END /////

///// PAGE 21 START /////
{0x03, 0x21}, //page 21

//Indoor Weight
{0x20, 0x11},
{0x21, 0x11},
{0x22, 0x11},
{0x23, 0x11},
{0x24, 0x13},
{0x25, 0x33},
{0x26, 0x33},
{0x27, 0x31},
{0x28, 0x13},
{0x29, 0x33},
{0x2a, 0x33},
{0x2b, 0x31},
{0x2c, 0x13},
{0x2d, 0x37},
{0x2e, 0x73},
{0x2f, 0x31},
{0x30, 0x13},
{0x31, 0x37},
{0x32, 0x73},
{0x33, 0x31},
{0x34, 0x13},
{0x35, 0x33},
{0x36, 0x33},
{0x37, 0x31},
{0x38, 0x13},
{0x39, 0x33},
{0x3a, 0x33},
{0x3b, 0x31},
{0x3c, 0x11},
{0x3d, 0x11},
{0x3e, 0x11},
{0x3f, 0x11},

//Outdoor Weigh
{0x40, 0x00},
{0x41, 0x00},
{0x42, 0x00},
{0x43, 0x00},
{0x44, 0x02},
{0x45, 0x22},
{0x46, 0x22},
{0x47, 0x20},
{0x48, 0x02},
{0x49, 0x22},
{0x4a, 0x22},
{0x4b, 0x20},
{0x4c, 0x02},
{0x4d, 0x27},
{0x4e, 0x72},
{0x4f, 0x20},
{0x50, 0x02},
{0x51, 0x27},
{0x52, 0x72},
{0x53, 0x20},
{0x54, 0x02},
{0x55, 0x22},
{0x56, 0x22},
{0x57, 0x20},
{0x58, 0x02},
{0x59, 0x22},
{0x5a, 0x22},
{0x5b, 0x20},
{0x5c, 0x00},
{0x5d, 0x00},
{0x5e, 0x00},
{0x5f, 0x00},

///// PAGE 22 START /////
{0x03, 0x22}, //page 22
{0x10, 0xfd},
{0x11, 0x2e},
{0x19, 0x02},
{0x20, 0x30}, //For AWB Speed
{0x21, 0x80},
{0x22, 0x00},
{0x23, 0x00},
{0x24, 0x01},
{0x25, 0x4f}, //2013-09-13 AWB Hunting

{0x30, 0x80},
{0x31, 0x80},
{0x38, 0x11},
{0x39, 0x34},
{0x40, 0xe4}, //Stb Yth
{0x41, 0x33}, //Stb cdiff
{0x42, 0x22}, //Stb csum
{0x43, 0xf3}, //Unstb Yth
{0x44, 0x33}, //Unstb cdiff55
{0x45, 0x33}, //Unstb csum
{0x46, 0x00},
{0x47, 0x09}, //2013-09-13 AWB Hunting
{0x48, 0x00}, //2013-09-13 AWB Hunting
{0x49, 0x0a},

{0x60, 0x04},
{0x61, 0xc4},
{0x62, 0x04},
{0x63, 0x92},
{0x66, 0x04},
{0x67, 0xc4},
{0x68, 0x04},
{0x69, 0x92},

{0x80, 0x36},
{0x81, 0x20},
{0x82, 0x2a},

{0x83, 0x58},
{0x84, 0x15},
{0x85, 0x4f},
{0x86, 0x15},

{0x87, 0x44},
{0x88, 0x2b},
{0x89, 0x2c},
{0x8a, 0x18},

{0x8b, 0x3b},
{0x8c, 0x32},
{0x8d, 0x2a},
{0x8e, 0x1b},

{0x8f, 0x4f},
{0x90, 0x46},
{0x91, 0x41},
{0x92, 0x3b},
{0x93, 0x30},
{0x94, 0x26},
{0x95, 0x1e},
{0x96, 0x19},
{0x97, 0x13},
{0x98, 0x12},
{0x99, 0x11},
{0x9a, 0x10},

{0x9b, 0x88},
{0x9c, 0x77},
{0x9d, 0x48},
{0x9e, 0x38},
{0x9f, 0x30},

{0xa0, 0x70},
{0xa1, 0x54},
{0xa2, 0x6f},
{0xa3, 0xff},

{0xa4, 0x14}, //1536fps
{0xa5, 0x2c}, //698fps
{0xa6, 0xcf}, //148fps

{0xad, 0x2e},
{0xae, 0x28},

{0xaf, 0x28}, //Low temp Rgain
{0xb0, 0x26}, //Low temp Rgain

{0xb1, 0x08},
{0xb4, 0xbf}, //For Tracking AWB Weight
{0xb8, 0x91}, //(0+,1-)High Cb , (0+,1-)Low Cr
{0xb9, 0xb0},
/////// PAGE 22 END ///////

//// MIPI Setting /////
{0x03, 0x48},
{0x39, 0x4f}, //lvds_bias_ctl    [2:0]mipi_tx_bias   [4:3]mipi_vlp_sel   [6:5]mipi_vcm_sel
{0x10, 0x1c}, //lvds_ctl_1       [5]mipi_pad_disable [4]lvds_en [0]serial_data_len
{0x11, 0x10}, //lvds_ctl_2       [4]mipi continous mode setting
//{0x14, 0x00} //ser_out_ctl_1  [2:0]serial_sout_a_phase   [6:4]serial_cout_a_phase

{0x16, 0x00}, //lvds_inout_ctl1  [0]vs_packet_pos_sel [1]data_neg_sel [4]first_vsync_end_opt
{0x18, 0x80}, //lvds_inout_ctl3
{0x19, 0x00}, //lvds_inout_ctl4
{0x1a, 0xf0}, //lvds_time_ctl
{0x24, 0x1e}, //long_packet_id

//====== MIPI Timing Setting =========
{0x36, 0x01}, //clk_tlpx_time_dp
{0x37, 0x05}, //clk_tlpx_time_dn
{0x34, 0x04}, //clk_prepare_time
{0x32, 0x15}, //clk_zero_time
{0x35, 0x04}, //clk_trail_time
{0x33, 0x0d}, //clk_post_time

{0x1c, 0x01}, //tlps_time_l_dp
{0x1d, 0x0b}, //tlps_time_l_dn
{0x1e, 0x06}, //hs_zero_time
{0x1f, 0x09}, //hs_trail_time

//long_packet word count
{0x30, 0x06},
{0x31, 0x40}, //long_packet word count

/////// PAGE 20 ///////
{0x03, 0x20},
{0x10, 0x9c}, //AE On 50hz

/////// PAGE 22 ///////
{0x03, 0x22},
{0x10, 0xe9}, //AWB On

{0x03, 0x00},
{0x01, 0x00},
};

static struct v4l2_subdev_info hi258_pop10_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt	= 1,
		.order	= 0,
	},
};

static const struct i2c_device_id hi258_pop10_i2c_id[] = {
	{HI258_POP10_SENSOR_NAME, (kernel_ulong_t)&hi258_pop10_s_ctrl},
	{ }
};

static int32_t msm_hi258_pop10_i2c_probe(struct i2c_client *client,
	   const struct i2c_device_id *id)
{
	printk("Evan:%s\n",__func__);
	   return msm_sensor_i2c_probe(client, id, &hi258_pop10_s_ctrl);
}

static struct i2c_driver hi258_pop10_i2c_driver = {
	.id_table = hi258_pop10_i2c_id,
	.probe  = msm_hi258_pop10_i2c_probe,
	.driver = {
		.name = HI258_POP10_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi258_pop10_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id hi258_pop10_dt_match[] = {
	{.compatible = "qcom,hi258_pop10", .data = &hi258_pop10_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi258_pop10_dt_match);

static struct platform_driver hi258_pop10_platform_driver = {
	.driver = {
		.name = "qcom,hi258_pop10",
		.owner = THIS_MODULE,
		.of_match_table = hi258_pop10_dt_match,
	},
};

static void hi258_pop10_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}



static int32_t hi258_pop10_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	printk("Evan:%s\n",__func__);
	match = of_match_device(hi258_pop10_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hi258_pop10_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hi258_pop10_platform_driver,
		hi258_pop10_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hi258_pop10_i2c_driver);
}

static void __exit hi258_pop10_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi258_pop10_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi258_pop10_s_ctrl);
		platform_driver_unregister(&hi258_pop10_platform_driver);
	} else
		i2c_del_driver(&hi258_pop10_i2c_driver);
	return;
}

static int32_t hi258_pop10_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	printk("Evan:%s\n",__func__);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: hi258_pop10 read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	printk("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}


int32_t hi258_pop10_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	int32_t rc = 0;
	int32_t i = 0;

	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;

	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		CDBG("=====> INcase: [CFG_GET_SENSOR_INFO]\n");
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_SET_INIT_SETTING:
		CDBG("init setting");
		hi258_pop10_i2c_write_table(s_ctrl,
				&hi258_pop10_recommend_settings[0],
				ARRAY_SIZE(hi258_pop10_recommend_settings));
		CDBG("init setting X");
		break;
	case CFG_SET_RESOLUTION: {
		int val = 0;
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		if (val == 0)
			hi258_pop10_i2c_write_table(s_ctrl, &hi258_pop10_snapshot_settings[0],
				ARRAY_SIZE(hi258_pop10_snapshot_settings));
		else if (val == 1)
			hi258_pop10_i2c_write_table(s_ctrl, &hi258_pop10_preview_settings[0],
				ARRAY_SIZE(hi258_pop10_preview_settings));
		break;
	}
	case CFG_SET_STOP_STREAM:
		CDBG("=====> INcase: [CFG_SET_STOP_STREAM]\n");
/*[BUGFIX]-Mod-BEGIN by TCTNB.WPL, bug564473, 2013/11/30 */
	#if 0
		hi258_pop10_i2c_write_table(s_ctrl,
			&hi258_pop10_stop_settings[0],
			ARRAY_SIZE(hi258_pop10_stop_settings));
	#endif
/*[BUGFIX]-Mod-END by TCTNB.WPL */
		break;
	case CFG_SET_START_STREAM:
/*[BUGFIX]-Mod-BEGIN by TCTNB.WPL, bug564473, 2013/11/30 */
		hi258_pop10_i2c_write_table(s_ctrl,
			&hi258_pop10_snapshot_settings[0],
			ARRAY_SIZE(hi258_pop10_snapshot_settings));
#if 0
		hi258_pop10_i2c_write_table(s_ctrl,
			&hi258_pop10_start_settings[0],
			ARRAY_SIZE(hi258_pop10_start_settings));
#endif
/*[BUGFIX]-Mod-END by TCTNB.WPL */
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;

	case CFG_SET_SLAVE_INFO: {
#if 0
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s sensor id 0x%x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg 0x%x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id 0x%x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
			slave_index,
			power_setting_array->power_setting[slave_index].
			seq_type,
			power_setting_array->power_setting[slave_index].
			seq_val,
			power_setting_array->power_setting[slave_index].
			config_val,
			power_setting_array->power_setting[slave_index].
			delay);
		}
		kfree(power_setting_array->power_setting);
#endif
		break;
	}

	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		CDBG("=====> INcase: [CFG_WRITE_I2C_ARRAY]\n");

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;
		CDBG("=====> INcase: [CFG_WRITE_I2C_SEQ_ARRAY]\n");

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		CDBG("=====> INcase: [CFG_POWER_UP]\n");
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		CDBG("=====> INcase: [CFG_POWER_DOWN]\n");
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		CDBG("=====> INcase: [CFG_SET_STOP_STREAM_SETTING]\n");
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting, stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
		}
		case CFG_SET_SATURATION: {
			int32_t sat_lev;
		CDBG("=====> INcase: [CFG_SET_SATURATION]\n");
			if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);

		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
		CDBG("=====> INcase: [CFG_SET_CONTRAST]\n");
			if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);

		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);

		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: anti-banding mode is %d", __func__,
			antibanding_mode);

		break;
	}


	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		//hi258_pop10_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		//hi258_pop10_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d", __func__, effect_mode);

		break;
	}

	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);

		break;
	}

	case CFG_SET_ISO:

		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

#ifdef CONFIG_COMPAT
int32_t hi258_pop10_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	int32_t rc = 0;
	int32_t i = 0;

	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;

	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		CDBG("=====> INcase: [CFG_GET_SENSOR_INFO]\n");
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_SET_INIT_SETTING:
		CDBG("init setting");
		hi258_pop10_i2c_write_table(s_ctrl,
				&hi258_pop10_recommend_settings[0],
				ARRAY_SIZE(hi258_pop10_recommend_settings));
		CDBG("init setting X");
		break;
	case CFG_SET_RESOLUTION: {
		int val = 0;
		if (copy_from_user(&val,
			(void *)compat_ptr(cdata->cfg.setting), sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		if (val == 0)
			hi258_pop10_i2c_write_table(s_ctrl, &hi258_pop10_snapshot_settings[0],
				ARRAY_SIZE(hi258_pop10_snapshot_settings));
		else if (val == 1)
			hi258_pop10_i2c_write_table(s_ctrl, &hi258_pop10_preview_settings[0],
				ARRAY_SIZE(hi258_pop10_preview_settings));
		break;
	}
	case CFG_SET_STOP_STREAM:
		CDBG("=====> INcase: [CFG_SET_STOP_STREAM]\n");
/*[BUGFIX]-Mod-BEGIN by TCTNB.WPL, bug564473, 2013/11/30 */
	#if 0
		hi258_pop10_i2c_write_table(s_ctrl,
			&hi258_pop10_stop_settings[0],
			ARRAY_SIZE(hi258_pop10_stop_settings));
	#endif
/*[BUGFIX]-Mod-END by TCTNB.WPL */
		break;
	case CFG_SET_START_STREAM:
/*[BUGFIX]-Mod-BEGIN by TCTNB.WPL, bug564473, 2013/11/30 */
		hi258_pop10_i2c_write_table(s_ctrl,
			&hi258_pop10_snapshot_settings[0],
			ARRAY_SIZE(hi258_pop10_snapshot_settings));
#if 0
		hi258_pop10_i2c_write_table(s_ctrl,
			&hi258_pop10_start_settings[0],
			ARRAY_SIZE(hi258_pop10_start_settings));
#endif
/*[BUGFIX]-Mod-END by TCTNB.WPL */
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;

	case CFG_SET_SLAVE_INFO: {
#if 0
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s sensor id 0x%x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg 0x%x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id 0x%x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
			slave_index,
			power_setting_array->power_setting[slave_index].
			seq_type,
			power_setting_array->power_setting[slave_index].
			seq_val,
			power_setting_array->power_setting[slave_index].
			config_val,
			power_setting_array->power_setting[slave_index].
			delay);
		}
		kfree(power_setting_array->power_setting);
#endif
		break;
	}

	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting32 conf_array32;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		CDBG("=====> INcase: [CFG_WRITE_I2C_ARRAY]\n");

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.data_type = conf_array32.data_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);
		
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting32 conf_array32;
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;
		CDBG("=====> INcase: [CFG_WRITE_I2C_SEQ_ARRAY]\n");

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_seq_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		conf_array.addr_type = conf_array32.addr_type;
		//conf_array.data_type = conf_array32.data_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		CDBG("=====> INcase: [CFG_POWER_UP]\n");
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		CDBG("=====> INcase: [CFG_POWER_DOWN]\n");
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting32 *stop_setting32;
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		CDBG("=====> INcase: [CFG_SET_STOP_STREAM_SETTING]\n");
		if (copy_from_user(stop_setting32, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		stop_setting->addr_type = stop_setting32->addr_type;
		stop_setting->data_type = stop_setting32->data_type;
		stop_setting->delay = stop_setting32->delay;
		stop_setting->size = stop_setting32->size;
		stop_setting->reg_setting = compat_ptr(stop_setting32->reg_setting);
		
		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting, stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
		}
		case CFG_SET_SATURATION: {
			int32_t sat_lev;
		CDBG("=====> INcase: [CFG_SET_SATURATION]\n");
			if (copy_from_user(&sat_lev, (void *)compat_ptr(cdata->cfg.setting),
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);

		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
		CDBG("=====> INcase: [CFG_SET_CONTRAST]\n");
			if (copy_from_user(&con_lev, (void *)compat_ptr(cdata->cfg.setting),
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);

		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);

		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: anti-banding mode is %d", __func__,
			antibanding_mode);

		break;
	}


	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		//hi258_pop10_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		//hi258_pop10_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d", __func__, effect_mode);

		break;
	}

	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);

		break;
	}

	case CFG_SET_ISO:

		break;
	default:
		pr_err("Invalid cfgtype func %s line %d cfgtype = %d\n",
			__func__, __LINE__, (int32_t)cdata->cfgtype);
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#endif

static struct msm_sensor_fn_t hi258_pop10_sensor_func_tbl = {
	.sensor_config = hi258_pop10_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = hi258_pop10_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = hi258_pop10_sensor_match_id,
};

static struct msm_sensor_ctrl_t hi258_pop10_s_ctrl = {
	.sensor_i2c_client = &hi258_pop10_sensor_i2c_client,
	.power_setting_array.power_setting = hi258_pop10_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi258_pop10_power_setting),
	#if 1
/*[BUGFIX]-Mod-BEGIN by TCTSZ.ZKX, Add power down sequence, 2014/05/27 */
	.power_setting_array.power_down_setting = hi258_pop10_power_down_setting,
	.power_setting_array.size_down = ARRAY_SIZE(hi258_pop10_power_down_setting),
/*[BUGFIX]-Mod-END by TCTSZ.ZKX*/
	#endif
	.msm_sensor_mutex = &hi258_pop10_mut,
	.sensor_v4l2_subdev_info = hi258_pop10_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi258_pop10_subdev_info),
	.func_tbl = &hi258_pop10_sensor_func_tbl,
};

module_init(hi258_pop10_init_module);
module_exit(hi258_pop10_exit_module);
MODULE_DESCRIPTION("Hi256 2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
