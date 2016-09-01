/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 * 
 * --------------------------------------------------------------------------------------
*/
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define GC2145_POP10_SENSOR_NAME "gc2145_pop10"
#define PLATFORM_DRIVER_NAME "msm_camera_gc2145_pop10"
//#include <linux/productinfo.h>

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MSM_MUTEX(gc2145_pop10_mut);
static struct msm_sensor_ctrl_t gc2145_pop10_s_ctrl;

static struct msm_sensor_power_setting gc2145_pop10_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 1,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 1,
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
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
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

static struct msm_camera_i2c_reg_conf gc2145_pop10_uxga_settings[] = {
//uxga = 1600*1200, 2M
  //  {0xf7 ,  0x1d},
  //  {0xf8 ,  0x84},
  {0xfe,  0x00},
  {0xfa,  0x11},//00
  //{0x1c,  0x05},    ///7.10
                           /////////1600x1200////////////
  {0xfd,  0x00}, //scaler
  {0xfe , 0x00},
  {0x99 , 0x11},
  {0x9a , 0x06},
  {0x9b , 0x00},
  {0x9c , 0x00},
  {0x9d , 0x00},
  {0x9e , 0x00},
  {0x9f , 0x00},
  {0xa0 , 0x00},
  {0xa1 , 0x00},
  {0xa2  ,0x00},
  {0x90 , 0x01},
  {0x91 , 0x00},
  {0x92 , 0x00},
  {0x93 , 0x00},
  {0x94 , 0x00},
  {0x95,  0x04},
  {0x96,  0xb0},
  {0x97,  0x06},
  {0x98,  0x40},

	//// AWB
  {0xfe,  0x00},
  {0xec,  0x06},  //measure window
  {0xed,  0x04},
  {0xee,  0x60}, //16  col
  {0xef,  0x90}, //8  row
  {0xfe,  0x01},
  {0x74,  0x01}, 
                           // blk

	//// AEC
  {0xfe,  0x01},
  {0x01,  0x04},
  {0x02,  0xc0},
  {0x03,  0x04}, 
  {0x04,  0x90}, 
  {0x05,  0x30},
  {0x06,  0x90},
  {0x07,  0x30},
  {0x08,  0x80},
  {0x0a,  0x82},
  {0x21 , 0x04},
  {0xfe , 0x00},
  {0x20 , 0x03},
 //MIPI  
  {0xfe,  0x03},
  {0x12,  0x80},  //LWC[7:0]  //
  {0x13,  0x0c},  //LWC[15:8]
  {0x04 , 0x01},
  {0x05 , 0x00},
  {0xfe,  0x00},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_start_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x94},
	{0xfe, 0x00},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_stop_settings[] = {
	{0xfe, 0x03},
	{0x10, 0x84},
	{0xfe, 0x00},
};

//set sensor init setting
static struct msm_camera_i2c_reg_conf gc2145_pop10_recommend_settings[] = {
//2M init
    {0xfe ,  0xf0},
    {0xfe ,  0xf0},
    {0xfe ,  0xf0},
    {0xfc ,  0x06},
    {0xf6 ,  0x00},
    {0xf7 ,  0x1d}, //37 //17 //37 //1d//05
    {0xf8 ,  0x84}, //87 //83 //82
    {0xfa ,  0x00},//00
    {0xf9 ,  0x8e}, //ff
    {0xf2 ,  0x00},
    {0xfe ,  0x00},
    
    //{0xb3 ,  0x60}, //r add betty
    //{0xb4 ,  0x40}, //g
    //{0xb5 ,  0x60},//b
    //{0xad ,  0x80},	
    //{0xae ,  0x7e},
    //{0xaf ,  0x80},

    {0x03 ,  0x03}, //exp time
    {0x04 ,  0x50}, //exp time
    {0x09 ,  0x00}, //row start
    {0x0a ,  0x00}, //
    {0x0b ,  0x00}, //col start
    {0x0c ,  0x04},//00 betty
    {0x0d ,  0x04}, //height
    {0x0e ,  0xc0},
    {0x0f ,  0x06}, //width
    {0x10 ,  0x52},
    {0x12 ,  0x2e}, //sh_delay 太短 YUV出图异常
    {0x17 ,  0x14}, //CISCTL Mode1 [1:0]mirror flip
    {0x18 ,  0x22}, //sdark mode
    {0x19 ,  0x0e}, // AD pipe number
    {0x1a ,  0x01}, //AD manual switch mode
    {0x1b ,  0x4b}, //48 restg Width,SH width
    {0x1c ,  0x07}, //06  帧率快后，横条纹 //12 //TX Width,Space Width
    {0x1d ,  0x10}, //double reset
    {0x1e ,  0x88},//90//98 //fix  竖线//Analog Mode1,TX high,Coln_r
    {0x1f ,  0x78}, //78 //38 //18 //Analog Mode2,txlow
    {0x20 ,  0x03}, //07 //Analog Mode3,comv,ad_clk mode
    {0x21 ,  0x40},//10//20//40 //fix 灯管横条纹
	{0x22 ,  0xa0},
    {0x24 ,  0x16}, //Pad drv
    {0x25 ,  0x01}, //col sel
    {0x26 ,  0x10}, //Analog PGA gain1
    {0x2d ,  0x60},//40//40 //txl drv mode
    {0x30 ,  0x01}, //Analog Mode4 
    {0x31 ,  0x90},//b0//70 // Analog Mode7 [7:5]rsgh_r灯管横条纹[4:3]isp_g
    {0x33 ,  0x06},//03//02//01 //EQ_hstart_width
    {0x34 ,  0x01},
    /////////////////////////////
    /////ISP reg/////////////////
    ////////////////////////////
	{0xfe ,  0x00},
	{0x80 ,  0x7f},
    {0x81 ,  0x26},//26//24 //BLK dither mode, ll_y_en ,skin_en, edge SA, new_skin_mode, autogray_en,ll_gamma_en,BFF test image
    {0x82 ,  0xfa}, //FA //auto_SA, auto_EE, auto_DN, auto_DD, auto_LSC, ABS_en, AWB_en, NA
    {0x83 ,  0x00}, //special_effect
    {0x84 ,  0x02}, //output format
    {0x86 ,  0x02}, //c2 //46 //c2 //sync mode
    {0x88 ,  0x03}, //[1]ctl_auto_gating [0]out_auto_gating
    {0x89 ,  0x03}, //bypass disable
	{0x85 ,  0x08},
    {0x8a ,  0x00}, //ISP_quiet_mode,close aaa pclk,BLK gate mode,exception,close first pipe clock,close dndd clock,close intp clock,DIV_gatedclk_en
    {0x8b ,  0x00}, //[7:6]BFF_gate_mode,[5]BLK switch gain,[4]protect exp,[3:2]pipe gate mode,[1]not split sram,[0]dark current update
    {0xb0 ,  0x55},//60 //global gain
    {0xc3 ,  0x00}, //[7:4]auto_exp_gamma_th1[11:8],[3:0]auto_exp_gamma_th2[11:8]
    {0xc4 ,  0x80}, //auto_exp_gamma_th1[7:0] into 
    {0xc5 ,  0x90}, //auto_exp_gamma_th2[7:0] out //outdoor gamma
    {0xc6 ,  0x3b}, //auto_gamma_th1   38
    {0xc7 ,  0x46}, //auto_gamma_th2   40
    {0xec ,  0x06},  //measure window
    {0xed ,  0x04},
    {0xee ,  0x60}, //16  col
    {0xef ,  0x90}, //8  row
    {0xb6 ,  0x01}, //[0]aec en
    {0x90 ,  0x01}, //crop
    {0x91 ,  0x00},//y1
    {0x92 ,  0x00},//y1
    {0x93 ,  0x00},//x1 
    {0x94 ,  0x04}, //08 x1 00 betty
    {0x95 ,  0x04},
    {0x96 ,  0xb0},
    {0x97 ,  0x06},
    {0x98 ,  0x40},
	{0xfe ,  0x00},
    {0x40 ,  0x42}, //2b //27
    {0x41 ,  0x00}, //80 //dark row sel
	{0x43 ,  0x5b},
    {0x5e ,  0x00},//00//10 //18
    {0x5f ,  0x00},//00//10 //18
    {0x60 ,  0x00},//00//10 //18
    {0x61 ,  0x00},//00///10 //18
    {0x62 ,  0x00},//00//10 //18
    {0x63 ,  0x00},//00//10 //18
    {0x64 ,  0x00},//00/10 //18
    {0x65 ,  0x00},//00//10 //18
    {0x66 ,  0x20},//1e//20
    {0x67 ,  0x20},//1e //20
    {0x68 ,  0x20},//1e //20
    {0x69 ,  0x20},//1e//20
    {0x76 ,  0x00},//0f
	{0x6a ,  0x08},
	{0x6b ,  0x08},
	{0x6c ,  0x08},
	{0x6d ,  0x08},
	{0x6e ,  0x08},
	{0x6f ,  0x08},
	{0x70 ,  0x08},
	{0x71 ,  0x08},
    {0x76 ,  0x00},//1f//add offset
    {0x72 ,  0xf0}, //[7:4]BLK DD th [3:0]BLK various th
    {0x7e ,  0x3c}, //ndark
    {0x7f ,  0x00}, 
    {0xfe ,  0x02},
    {0x48 ,  0x15},
    {0x49 ,  0x00},//04//04 //ASDE OFFSET SLOPE 
    {0x4b ,  0x0b}, //ASDE y OFFSET SLOPE
    {0xfe ,  0x00},
    ///////////////////////
    ////////AEC///////////
    //////////////////////
    {0xfe ,  0x01},
    {0x01 ,  0x01}, //AEC X1
    {0x02 ,  0x80}, //AEC X2
    {0x03 ,  0x00}, //AEC Y1
    {0x04 ,  0x60}, //AEC Y2
    {0x05 ,  0x28}, //20 //AEC center X1
    {0x06 ,  0x3c}, //40 //AEC center X2
    {0x07 ,  0x1d}, //30 //AEC center Y1
    {0x08 ,  0x2e}, //60 //AEC center Y2
    {0x09 ,  0x00}, //AEC show mode
    {0x0a ,  0x82}, //[7]col gain enable//c2
    {0x0b ,  0x11}, //AEC every N
    {0x0c ,  0x70}, //AEC_mode3 center weight 
    {0x11 ,  0x10},
    {0x13 ,  0x7b},//2a //AEC Y target 30 betty
    {0x17 ,  0x00}, //AEC ignore mode
    {0x1c ,  0x11}, //
    {0x1e ,  0x61}, //
    {0x1f ,  0x28},//40//50 //max pre gain
    {0x20 ,  0x30},//60//40 //max post gain
	{0x22 , 0x40},
    {0x23 ,  0x20}, //target_Y_low_limit
    {0xfe ,  0x02},
	{0x0f , 0x04},
    {0xfe ,  0x01},
    {0x12 ,  0x30}, //35 //[5:4]group_size [3]slope_disable [2]outdoor_enable [0]histogram_enable//00
    {0x15 ,  0xb0}, //target_Y_high_limit//0x50
    {0x10 ,  0x31}, //num_thd_high
    {0x3e ,  0x28}, //num_thd_low
    {0x3f ,  0xb0}, //luma_thd//e0
    {0x40 ,  0x90}, //luma_slope//e0
    {0x41 ,  0x0f}, //color_diff
    ///////////////////////
    ////////INTPEE/////////
    //////////////////////
    {0xfe ,  0x02}, //page2
    {0x90 ,  0x6c}, //ac //eeintp mode1
    {0x91 ,  0x03},//02 ////eeintp mode2 
    {0x92 ,  0xcb},//44 //low criteria for direction//0xc8
    {0x94 ,  0x33},
    {0x95 ,  0x84},
    {0x97 ,  0x78},//76 ////edge effect  64 betty
    {0xa2 ,  0x11}, //fix direction
    {0xfe ,  0x00},
    /////////////////////
    ///////DNDD/////////
    ////////////////////
    {0xfe ,  0x02},
    {0x80 ,  0xc1}, //c1 //[7]share mode [6]skin mode  [5]is 5x5 mode [1:0]noise value select 0:2  1:2.5  2:3  3:4
    {0x81 ,  0x08}, //
    {0x82 ,  0x0f}, //signal a 0.6 08 betty
    {0x83 ,  0x08},//04 //signal b 2.5
    {0x84 ,  0x0a}, //10 //05 dark_DD_TH
    {0x86 ,  0xf0}, //a0 Y_value_dd_th2//50
    {0x87 ,  0x50}, //90 Y_value_dd_th3//30
    {0x88 ,  0x15}, //60 Y_value_dd_th4
	{0x89 , 0xb0},
    {0x8a ,  0x30}, //60  // asde th3//60
    {0x8b ,  0x10}, //30  // asde th4//30
    ///////////////////////
    ///////////ASDE///////
    //////////////////////
    {0xfe ,  0x01}, //page 1
    {0x21 ,  0x04}, //cj 0x14//luma_value_div_sel(分频，与0xef呈2倍关系，增大1，0xef的值减小1倍)//12
    {0xfe ,  0x02}, //page2
    {0xa3 ,  0x50}, //ASDE_low_luma_value_LSC_th_H
    {0xa4 ,  0x20}, //ASDE_low_luma_value_LSC_th_L
    {0xa5 ,  0x40}, //80 //ASDE_LSC_gain_dec_slope_H
    {0xa6 ,  0x80}, // 80 //ASDE_LSC_gain_dec_slope_L
    {0xab ,  0x40},//cj 0x30//50 //ASDE_low_luma_value_OT_th   40 betty
    {0xae ,  0x0c}, //[3]EE1_effect_inc_or_dec_high,[2]EE2_effect_inc_or_dec_high,
    {0xb3 ,  0x46},//44 //ASDE_EE1_effect_slope_low,ASDE_EE2_effect_slope_low
    {0xb4 ,  0x64}, //12 //ASDE_EE1_effect_slope_high,ASDE_EE2_effect_slope_high
    {0xb6 ,  0x38},//40//40 //ASDE_auto_saturation_dec_slope 
	{0xb7 , 0x01},
	{0xb9 , 0x2b},
	{0x3c , 0x04},
	{0x3d , 0x15},
	{0x4b , 0x06},
    {0x4c ,  0x20},//y offset limit
    {0xfe ,  0x00},
/////////////Gamma1/////////////
	{0xfe , 0x02},
	{0x10 , 0x09},
	{0x11 , 0x0d},
	{0x12 , 0x13},
	{0x13 , 0x19},
	{0x14 , 0x27},
	{0x15 , 0x37},
	{0x16 , 0x45},
	{0x17 , 0x53},
	{0x18 , 0x69},
	{0x19 , 0x7d},
	{0x1a , 0x8f},
	{0x1b , 0x9d},
	{0x1c , 0xa9},
	{0x1d , 0xbd},
	{0x1e , 0xcd},
	{0x1f , 0xd9},
	{0x20 , 0xe3},
	{0x21 , 0xea},
	{0x22 , 0xef},
	{0x23 , 0xf5},
	{0x24 , 0xf9},
	{0x25 , 0xff},


	{0xfe , 0x00},
	{0xc6 , 0x20},
	{0xc7 , 0x2b},
///////Gamma2//////////
    {0xfe ,  0x02},
    {0x26 ,  0x0f},
    {0x27 ,  0x14},
    {0x28 ,  0x19},
    {0x29 ,  0x1e},
    {0x2a ,  0x27},
    {0x2b ,  0x33},
    {0x2c ,  0x3b},
    {0x2d ,  0x45},
    {0x2e ,  0x59},
    {0x2f ,  0x69},
    {0x30 ,  0x7c},
    {0x31 ,  0x89},
    {0x32 ,  0x98},
    {0x33 ,  0xae},
    {0x34 ,  0xc0},
    {0x35 ,  0xcf},
    {0x36 ,  0xda},
    {0x37 ,  0xe2},
    {0x38 ,  0xe9},
    {0x39 ,  0xf3},
    {0x3a ,  0xf9},
    {0x3b ,  0xff},
    {0xfe ,  0x00},
    {0xc6 ,  0x20},
    {0xc7 ,  0x2b},

//////////////////////////
//////////YCP////////////
/////////////////////////
    
    {0xfe ,  0x02},
    {0xd1 ,  0x38},//32
    {0xd2 ,  0x38},//32
    {0xd3 ,  0x40},//45 betty
   // {0xd5 ,  0x00},//0x00//45 betty
    {0xd6 ,  0xf0},
    {0xd7 ,  0x10},
    {0xd8 ,  0xda},
    {0xdd ,  0x14}, //edge sa
    {0xde ,  0x86}, //asde auto gray
	{0xed ,  0x80},
	{0xee ,  0x00},
    {0xef ,  0x3f}, //30
    {0xd8 ,  0xd8},//autogray protecy

////////////abs/////////////
    {0xfe ,  0x01},
    {0x9f ,  0x40},

////////////lsc start///////////

	//gc2135 Alight lsc reg setting list
	////Record date: 2014-03-19 09:55:16
	//gc2135 Alight lsc reg setting list
	////Record date: 2014-03-19 09:55:16
	#if  1
	  {0xfe, 0x01},                    //test
	  {0xa1, 0x80},
	  {0xa2, 0x80},
	  {0xa4, 0x76},
	  {0xa5, 0x77},
	  {0xa6, 0x67},
	  {0xa7, 0x77},
	  {0xa8, 0x20},
	  {0xa9, 0x76},
	  {0xaa, 0x15},
	  {0xab, 0x12},
	  {0xac, 0x14},
	  {0xad, 0x01},
	  {0xae, 0x01},
	  {0xaf, 0x09},
	  {0xb0, 0x07},
	  {0xb1, 0x0b},
	  {0xb2, 0x11},
	  {0xb3, 0x16},
	  {0xb4, 0x15},
	  {0xb5, 0x15},
	  {0xb6, 0x42},
	  {0xb7, 0x31},
	  {0xb8, 0x24},
	  {0xb9, 0x0e},
	  {0xba, 0x0d},
	  {0xbb, 0x01},
	  {0xbc, 0x62},
	  {0xbd, 0x53},
	  {0xbe, 0x4c},
	  {0xbf, 0x2f},
	  {0xc0, 0x34},
	  {0xc1, 0x31},
	  {0xc2, 0x2e},
	  {0xc3, 0x27},
	  {0xc4, 0x27},
	  {0xc5, 0x10},
	  {0xc6, 0x0d},
	  {0xc7, 0x17},
	  {0xc8, 0x35},
	  {0xc9, 0x26},
	  {0xca, 0x1f},
	  {0xcb, 0x2e},
	  {0xcc, 0x19},
	  {0xcd, 0x20},
	  {0xd0, 0x02},
	  {0xd1, 0x14},
	  {0xd2, 0x00},
	  {0xd3, 0x0f},
	  {0xd4, 0x02},
	  {0xd5, 0x27},
	  {0xd6, 0x2a},
	  {0xd7, 0x3b},
	  {0xd8, 0x43},
	  {0xd9, 0x17},
	  {0xda, 0x22},
	  {0xdb, 0x07},
	  #else
	  {0xfe, 0x01},
	  {0xa1, 0x80},
	  {0xa2, 0x80},
	  {0xa4, 0x77},
	  {0xa5, 0x76},
	  {0xa6, 0x76},
	  {0xa7, 0x77},
	  {0xa8, 0x67},
	  {0xa9, 0x02},
	  {0xaa, 0x06},
	  {0xab, 0x0b},
	  {0xac, 0x10},
	  {0xad, 0x16},
	  {0xae, 0x16},
	  {0xaf, 0x14},
	  {0xb0, 0x14},
	  {0xb1, 0x12},
	  {0xb2, 0x13},
	  {0xb3, 0x01},
	  {0xb4, 0x01},
	  {0xb5, 0x0a},
	  {0xb6, 0x61},
	  {0xb7, 0x53},
	  {0xb8, 0x4b},
	  {0xb9, 0x2e},
	  {0xba, 0x34},
	  {0xbb, 0x30},
	  {0xbc, 0x42},
	  {0xbd, 0x31},
	  {0xbe, 0x24},
	  {0xbf, 0x0d},
	  {0xc0, 0x0d},
	  {0xc1, 0x02},
	  {0xc2, 0x34},
	  {0xc3, 0x27},
	  {0xc4, 0x1f},
	  {0xc5, 0x2e},
	  {0xc6, 0x1a},
	  {0xc7, 0x1f},
	  {0xc8, 0x2e},
	  {0xc9, 0x27},
	  {0xca, 0x26},
	  {0xcb, 0x0f},
	  {0xcc, 0x0d},
	  {0xcd, 0x15},
	  {0xd0, 0x17},
	  {0xd1, 0x22},
	  {0xd2, 0x07},
	  {0xd3, 0x2a},
	  {0xd4, 0x3b},
	  {0xd5, 0x43},
	  {0xd6, 0x0e},
	  {0xd7, 0x02},
	  {0xd8, 0x27},
	  {0xd9, 0x01},
	  {0xda, 0x14},
	  {0xdb, 0x00},
	  #endif

////////lsc end///////////
    {0xfe ,  0x01},
    {0xdf ,  0x0d},//0c//00
    {0xdc ,  0x25},//80
    {0xdd ,  0x50},//30
    {0xe0 ,  0x74},//70
    {0xe1 ,  0x7d},//80
    {0xe2 ,  0x6c},//80
    {0xe3 ,  0x80},//80
    {0xe6 ,  0x90},//90
    {0xe7 ,  0x80},//50
    {0xe8 ,  0x98},//90/
    {0xe9 ,  0x80},//60
    {0xfe ,  0x00},
    
//////////awb/////////
    ///////d75//////             
    {0xfe, 0x01},                   
    {0x4f, 0x00},                   
    {0x4f, 0x00},                   
    {0x4b, 0x01},                   
    {0x4f, 0x00},                   
    {0x4c, 0x01},  // D75           
    {0x4d, 0x71},                   
    {0x4e, 0x01},                   
    {0x4c, 0x01},                   
    {0x4d, 0x91},                   
    {0x4e, 0x01},                   
    {0x4c, 0x01},                   
    {0x4d, 0x70},                   
    {0x4e, 0x01},                   
                    
  /////D65 ///////////   
    {0x4c, 0x01},  // D65           
    {0x4d, 0x90},                   
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x8f},
    {0x4e, 0x02},
    {0x4c, 0x01},                   
    {0x4d, 0x8e},
    {0x4e, 0x02},
    {0x4c, 0x01},                   
    {0x4d, 0x9f},                   
    {0x4e, 0x02},
    {0x4c, 0x01},                   
    {0x4d, 0xaf},                   
    {0x4e, 0x02},                   
    {0x4c, 0x01},                   
    {0x4d, 0xb0},                   
    {0x4e, 0x02},                   
    {0x4c, 0x01},                   
    {0x4d, 0xaf},                   
    {0x4e, 0x02},                   
    {0x4c, 0x01},
    {0x4d, 0x6f},
    {0x4e, 0x02},
                
   // {0x4c, 0x01},
    //{0x4d, 0x71},
    //{0x4e, 0x02},
    //{0x4c, 0x01},
    //{0x4d, 0x70},
    //{0x4e, 0x02},
    //{0x4c, 0x01},
    //{0x4d, 0x90},
    //{0x4e, 0x02},
    //{0x4c, 0x01},
    //{0x4d, 0x8f},
    //{0x4e, 0x02},
                
  ///// D50           
    {0x4c, 0x01},                   
    {0x4d, 0xcd},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0xed},                   
    {0x4e, 0x03},                   
              
    {0x4c, 0x01},  // D50           
    {0x4d, 0xad},                   
    {0x4e, 0x33},                   
    {0x4c, 0x01},                   
    {0x4d, 0xae},                   
    {0x4e, 0x33},                   
    {0x4c, 0x01},                   
    {0x4d, 0x8c},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0xac},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0xcd},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x8e},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x8d},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x8b},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x6a},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x6b},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x6c},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x6d},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0x6e},                   
    {0x4e, 0x03},                   
    {0x4c, 0x01},                   
    {0x4d, 0xab},                   
    {0x4e, 0x03},                   
                    
  ////////CWF ////// 
    {0x4c, 0x01},//  // CWF         
    {0x4d, 0xaa},//                 
    {0x4e, 0x04},//                 
    {0x4c, 0x01},//  // CWF         
    {0x4d, 0xa9},//                 
    {0x4e, 0x04},//                 
    {0x4c, 0x01},//                 
    {0x4d, 0xca},//                 
    {0x4e, 0x04},//                 
    {0x4c, 0x01},//                 
    {0x4d, 0xc9},//                 
    {0x4e, 0x04},//                 
    {0x4c, 0x01},//                 
    {0x4d, 0x8a},//                 
    {0x4e, 0x04},//                 
    {0x4c, 0x01},//                 
    {0x4d, 0x89},//                 
    {0x4e, 0x04},//                 
    {0x4c, 0x01},                   
    {0x4d, 0xcb},                   
    {0x4e, 0x04},                   
                  
  ///////TL84//////////////////           
    {0x4c, 0x02},// TL84            
    {0x4d, 0x0a},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},                   
    {0x4d, 0x4B},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x08},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x09},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x0B},                   
    {0x4e, 0x05},                   
    {0x4c, 0x01},	                 
    {0x4d, 0xE7},                   
    {0x4e, 0x05},                   
    {0x4c, 0x01},	                 
    {0x4d, 0xE8},                   
    {0x4e, 0x05},                   
    {0x4c, 0x01},	                 
    {0x4d, 0xE9},                   
    {0x4e, 0x05},                   
    {0x4c, 0x01},	                 
    {0x4d, 0xEb},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x2a},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x29},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x28},                   
    {0x4e, 0x05},                   
    {0x4c, 0x02},	                 
    {0x4d, 0x2b},                   
    {0x4e, 0x05},                   
                  
  ////A  ////////              
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x8a},                   
    {0x4e, 0x06},                   
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x89},                   
    {0x4e, 0x06},//06               
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x88},                   
    {0x4e, 0x06},                   
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x69},                   
    {0x4e, 0x06},                   
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x6a},                   
    {0x4e, 0x06},                   
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x6b},                   
    {0x4e, 0x06},                   
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x4a},                   
    {0x4e, 0x06},                   
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x49},                   
    {0x4e, 0x00},//06               
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x48},                   
    {0x4e, 0x00},//06               
    {0x4c, 0x02},//add hgj a        
    {0x4d, 0x4A},                   
    {0x4e, 0x06},//                 
                 
                 
  ///////H /////////////        
    {0x4c, 0x02},///H               
    {0x4d, 0xca},                   
    {0x4e, 0x07},//07               
    {0x4c, 0x02},//H                
    {0x4d, 0xc9},                   
    {0x4e, 0x07},//07               
    {0x4c, 0x02},                   
    {0x4d, 0xc8},                   
    {0x4e, 0x07},                   
    {0x4c, 0x02},                   
    {0x4d, 0xEa},                   
    {0x4e, 0x07},                   
    {0x4c, 0x02},                   
    {0x4d, 0xE9},                   
    {0x4e, 0x07},                   
    {0x4c, 0x02},                   
    {0x4d, 0xE8},                   
    {0x4e, 0x07},                   
             
    {0x4f, 0x01},//                 
    {0x50, 0x80},//                 
    {0x51, 0xa8},//                 
    {0x52, 0x57},//                 
    {0x53, 0x38},//                 
    {0x54, 0xc7},//                 
    {0x56, 0x0e},//                 
    {0x58, 0x09},//                 
    {0x5b, 0x00},//                 
    {0x5c, 0x74},//                 
    {0x5d, 0x8b},//                 
    {0x61, 0xa7},//                 
    {0x62, 0xb5},//                 
    {0x63, 0xaa},//                 
    {0x64, 0x80},//                 
    {0x65, 0x04},//                 
    {0x67, 0xa4},//                 
    {0x68, 0xb0},//                 
    {0x69, 0x00},//                 
    {0x6a, 0xa4},//                 
    {0x6b, 0xb0},//                 
    {0x6c, 0xb2},//                 
    {0x6d, 0xac},//                 
    {0x6e, 0x60},//                 
    {0x6f, 0x15},//                 
    {0x73, 0xf0},//  0f             
    {0x70, 0x10},//                 
    {0x71, 0xe8},//                 
    {0x72, 0xc0},//                 
    {0x74, 0x01},//                 
    {0x75, 0x00},//                 
    {0x7f, 0x08},//                 
    {0x76, 0x70},//                 
    {0x77, 0x58},//                 
    {0x78, 0xd0},//                 
    {0xfe, 0x00},//         
 
    ///////CC////////////
    {0xfe , 0x02},
    {0xc0 , 0x01},
	{0xc1 , 0x44},
	{0xc2 , 0xfd},
    {0xc3 , 0x04},
	{0xc4 , 0xfa},
	{0xc5 , 0x43},
	{0xc6 , 0xfd},
	{0xc7 , 0x46},
	{0xc8 , 0xfd},
	{0xc9 , 0x02},
	{0xca , 0xe0},
    {0xcb , 0x45},
    {0xcc , 0xec},//0x45
    {0xcd , 0x48},
    {0xce , 0xf0},
    {0xcf , 0xf0},
    {0xe3 , 0x0c},//0x50
    {0xe4 , 0x4b},  
    {0xe5 , 0xe0},      
	//////////////////////////////////////////
	///////////ABS ////////////////////
	//////////////////////////////////////////
	{0xfe , 0x01},
	{0x9f , 0x40},
	{0xfe , 0x00},
	//////////////////////////////////////
	///////////  OUTPUT   ////////////////
	//////////////////////////////////////
	{0xfe, 0x00},
	{0xf2, 0x00},
	//////////////frame rate 50Hz/////////
	/*
    {0xfe ,  0x00},
    {0x05 ,  0x01},//hb
    {0x06 ,  0x56},
    {0x07 ,  0x00},//Vb
    {0x08 ,  0x32},
    {0xfe ,  0x01},
    {0x25 ,  0x00},
    {0x26 ,  0xfa}, //step
    {0x27 ,  0x06}, //20fps
    {0x28 ,  0xd6},
    {0x29 ,  0x07}, //16.7fps
    {0x2a ,  0xd0},
    {0x2b ,  0x09}, //12.5fps
    {0x2c ,  0xc4},
    {0x2d ,  0x0b}, //night mode 8.3fps
    {0x2e ,  0xb8},
    */
           {0xfe , 0x00},
           {0x05 , 0x02},
           {0x06 , 0x2d},
           {0x07 , 0x00},
           {0x08 , 0x48},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xd4}, //step    250
           {0x27 , 0x04},
           {0x28 , 0xf8}, //20  fps 00
           {0x29 , 0x06},
           {0x2a , 0xa0}, //14  fps 01
           {0x2b , 0x09},
           {0x2c , 0xf0}, //10  fps 10
           {0x2d , 0x0c},
           {0x2e , 0x6c}, //6.6 fps 11 bb8-8
           {0xfe , 0x00},

    /////////////dark sun///////////
    {0xfe ,  0x00},
    {0x18 ,  0x22}, 
    {0xfe ,  0x02},
    {0x40 ,  0xbf},
    {0x46 ,  0xcf},
    {0xfe ,  0x00},
 
    //******MIPI*******************
    {0xfe ,  0x03},  //page 3
    {0x01 ,  0x83}, //07 
    {0x02 ,  0x55},  //37//07
    {0x03 ,  0x10},  //10
    {0x04 ,  0x20}, //01  // fifo_prog 
    {0x05 ,  0x00},  //fifo_prog 
    {0x06 ,  0x88},  //YUV ISP data  
    {0x10 ,  0x84}, //95 //94 // last bit  lane num 
    {0x11 ,  0x1e},  //LDI set YUV422
    {0x12 ,  0x80}, //04 //00 //04//00 //LWC[7:0]  //
    {0x13 ,  0x0c}, //05 //LWC[15:8]
    {0x15 , 0x10},  //DPHYY_MODE read_ready 12  
    {0x17 ,  0xf0},  //ff  01wdiv set 
//    {0xfe ,  0x00},

     {0x21 , 0x03},//LPX         4 * 16.7 =67 ns
     {0x22 , 0x05},
     {0x23 , 0x03},
    ///{0x24 , 0x10},
     {0x29 , 0x05},//H-prepare   4 * 16.7  = 67    //T = 200
     {0x2a , 0x03},//H-zero      8 * 16.7  = 133   //settel = 100
//   {0x21, 0x01},
//   {0x22, 0x02},
//   {0x23, 0x01},
//   {0x29, 0x02},//(2+2)x23.8 =95ns  H-prepare  //    T=214ns
//   {0x2a, 0x02},//(2+3)x23.8 =119ns    H-zero          //settle=100
     {0xfe , 0x00}, 

};

static struct v4l2_subdev_info gc2145_pop10_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};



//for preview setting
static struct msm_camera_i2c_reg_conf gc2145_pop10_svga_settings[] = {
    {0xfe ,  0x00},
	{0xfd ,  0x01},
    {0xfa ,  0x00},
     {0xb6 ,  0x01},  /////add   handy 2014/8/22
	//// crop window
	{0xfe , 0x00},
	{0x99 , 0x11},
	{0x9a , 0x06},
	{0x9b , 0x00},
	{0x9c , 0x00},
	{0x9d , 0x00},
	{0x9e , 0x00},
	{0x9f , 0x00},
	{0xa0 , 0x00},
	{0xa1 , 0x00},
	{0xa2  ,0x00},
	{0x90 , 0x01},
	{0x91 , 0x00},
	{0x92 , 0x00},
	{0x93 , 0x00},
	{0x94 , 0x00},
	{0x95 , 0x02},
	{0x96 , 0x58},
	{0x97 , 0x03},
	{0x98 , 0x20},

	//// AWB
	{0xfe , 0x00},
	{0xec , 0x02},
	{0xed , 0x02},
	{0xee , 0x30},
	{0xef , 0x48},
	{0xfe , 0x02},
	{0x9d , 0x0b},
	{0xfe , 0x01},
	{0x74 , 0x00},
	//// AEC
	{0xfe , 0x01},
	{0x01 , 0x04},
	{0x02 , 0x60},
	{0x03 , 0x02},
	{0x04 , 0x48},
	{0x05 , 0x18},
	{0x06 , 0x50},
	{0x07 , 0x10},
	{0x08 , 0x38},
	{0x0a , 0x80},
	{0x21 , 0x04},
	{0xfe , 0x00},
	{0x20 , 0x03},

	//// mipi
	{0xfe , 0x03},
	{0x12 , 0x40},
	{0x13 , 0x06},
	{0x04 , 0x01},
	{0x05 , 0x00},
	{0xfe , 0x00},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_sleep_settings[] = {

};


static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_saturation[11][4] = {
	{
{0xfe, 0x02},{0xd1, 0x1a},{0xd2, 0x1a},{0xfe, 0x00},//0
	},
	{
{0xfe, 0x02},{0xd1, 0x1e},{0xd2, 0x1e},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x22},{0xd2, 0x22},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x26},{0xd2, 0x26},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x2a},{0xd2, 0x2a},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x38},{0xd2, 0x38},{0xfe, 0x00},//deault
	},
	{
{0xfe, 0x02},{0xd1, 0x3a},{0xd2, 0x3a},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x40},{0xd2, 0x40},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x46},{0xd2, 0x46},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x4c},{0xd2, 0x4c},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd1, 0x54},{0xd2, 0x54},{0xfe, 0x00},//10
	},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_contrast[11][3] = {
	{
{0xfe, 0x02},{0xd3, 0x18},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x20},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x28},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x30},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x38},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x40},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x48},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x50},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x58},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x60},{0xfe, 0x00},
	},
	{
{0xfe, 0x02},{0xd3, 0x68},{0xfe, 0x00},
	},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_sharpness[7][3] = {
	{{0xfe, 0x02},{0x97, 0x22},{0xfe, 0x00}},//Sharpness -3
	{{0xfe, 0x02},{0x97, 0x44},{0xfe, 0x00}},//Sharpness -2
	{{0xfe, 0x02},{0x97, 0x65},{0xfe, 0x00}},//Sharpness -1
	{{0xfe, 0x02},{0x97, 0x78},{0xfe, 0x00}},//Sharpness
	{{0xfe, 0x02},{0x97, 0x88},{0xfe, 0x00}},//Sharpness +1
	{{0xfe, 0x02},{0x97, 0x99},{0xfe, 0x00}},//Sharpness +2
	{{0xfe, 0x02},{0x97, 0xaa},{0xfe, 0x00}},//Sharpness +3
};
static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_iso[7][2] = {
//not supported
	/* auto */
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
	/* auto hjt */  
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
	/* iso 100 */
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
	/* iso 200 */
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
	/* iso 400 */
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
	/* iso 800 */
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
	/* iso 1600 */
	{
{0xfe, 0x00},
{0xfe, 0x00},
	},
};
static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_exposure_compensation[5][3] = {
	{{0xfe, 0x01},{0x13, 0x5b},{0xfe, 0x00}},//Exposure -2
	{{0xfe, 0x01},{0x13, 0x6b},{0xfe, 0x00}},//Exposure -1
	{{0xfe, 0x01},{0x13, 0x7b},{0xfe, 0x00}},//Exposure     //20 betty
	{{0xfe, 0x01},{0x13, 0x8b},{0xfe, 0x00}},//Exposure +1
	{{0xfe, 0x01},{0x13, 0x9b},{0xfe, 0x00}},//Exposure +2
};
static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_antibanding[4][17] = {
	/* OFF */  //60-1  50-2   auto-off NC
	{
           {0xfe , 0x00},
           {0x05 , 0x02},
           {0x06 , 0x2d},
           {0x07 , 0x00},
           {0x08 , 0x48},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xd4}, //step    250
           {0x27 , 0x04},
           {0x28 , 0xf8}, //20  fps 00
           {0x29 , 0x06},
           {0x2a , 0xa0}, //14  fps 01
           {0x2b , 0x09},
           {0x2c , 0xf0}, //10  fps 10
           {0x2d , 0x0c},
           {0x2e , 0x6c}, //6.6 fps 11 bb8-8
           {0xfe , 0x00},
	/*
           {0xfe , 0x00},
           {0x05 , 0x01},
           {0x06 , 0x56},
           {0x07 , 0x00},
           {0x08 , 0x32},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xfa}, //step    250
           {0x27 , 0x06},
           {0x28 , 0xd6}, //20  fps 00
           {0x29 , 0x07},
           {0x2a , 0xd0}, //14  fps 01
           {0x2b , 0x09},
           {0x2c , 0xc4}, //10  fps 10
           {0x2d , 0x0b},
           {0x2e , 0xb8}, //6.6 fps 11 bb8-8
           {0xfe , 0x00},
           */
	}, /*ANTIBANDING 60HZ*/
	
	/* 60Hz */
	{

           {0xfe , 0x00},
           {0x05 , 0x02},
           {0x06 , 0x43},
           {0x07 , 0x00},
           {0x08 , 0xc0},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xae}, //step    250
           {0x27 , 0x05},
           {0x28 , 0x70}, //20  fps 00
           {0x29 , 0x07},
           {0x2a , 0x7a}, //14  fps 01
           {0x2b , 0x09},
           {0x2c , 0x84}, //10  fps 10
           {0x2d , 0x0c},
           {0x2e , 0x3c}, //6.6 fps 11 bb8-8
           {0xfe , 0x00},

	/*
           {0xfe , 0x00},
           {0x05 , 0x01},
           {0x06 , 0x56},
           {0x07 , 0x00},
           {0x08 , 0x32},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xd0}, //step     208  0x3c->0x40
           {0x27 , 0x04},
           {0x28 , 0xe0}, //19fps 00 
           {0x29 , 0x07},            
           {0x2a , 0x50}, //13.3fps 01 
           {0x2b , 0x09},
           {0x2c , 0xc0}, //10fps 10 
           {0x2d , 0x0d},
           {0x2e , 0xd0}, //7fps 11   
           {0xfe , 0x00},
           */
	}, /*ANTIBANDING 50HZ*/

	/* 50Hz */
	{
           {0xfe , 0x00},
           {0x05 , 0x02},
           {0x06 , 0x2d},
           {0x07 , 0x00},
           {0x08 , 0x48},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xd4}, //step    250
           {0x27 , 0x04},
           {0x28 , 0xf8}, //20  fps 00
           {0x29 , 0x06},
           {0x2a , 0xa0}, //14  fps 01
           {0x2b , 0x09},
           {0x2c , 0xf0}, //10  fps 10
           {0x2d , 0x0c},
           {0x2e , 0x6c}, //6.6 fps 11 bb8-8
           {0xfe , 0x00},
	/*
           {0xfe , 0x00},
           {0x05 , 0x01},
           {0x06 , 0x56},
           {0x07 , 0x00},
           {0x08 , 0x32},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xfa}, //step    250
           {0x27 , 0x06},
           {0x28 , 0xd6}, //20fps 00
           {0x29 , 0x07},
           {0x2a , 0xd0}, //14fps 01
           {0x2b , 0x09},
           {0x2c , 0xc4}, //10fps 10
           {0x2d , 0x0b},
           {0x2e , 0xb8}, //8fps 11
           {0xfe , 0x00},
	*/
	},
	/*ANTIBANDING 60HZ*/
	
	/* AUTO */
	{

	           {0xfe , 0x00},
           {0x05 , 0x02},
           {0x06 , 0x2d},
           {0x07 , 0x00},
           {0x08 , 0x48},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xd4}, //step    250
           {0x27 , 0x04},
           {0x28 , 0xf8}, //20  fps 00
           {0x29 , 0x06},
           {0x2a , 0xa0}, //14  fps 01
           {0x2b , 0x09},
           {0x2c , 0xf0}, //10  fps 10
           {0x2d , 0x0c},
           {0x2e , 0x6c}, //6.6 fps 11 bb8-8
           {0xfe , 0x00},
           /*
           {0xfe , 0x00},
           {0x05 , 0x01},
           {0x06 , 0x56},
           {0x07 , 0x00},
           {0x08 , 0x32},
           {0xfe , 0x01},
           {0x25 , 0x00},
           {0x26 , 0xfa}, //step     
           {0x27 , 0x06},
           {0x28 , 0xd6}, //20fps 00
           {0x29 , 0x07},
           {0x2a , 0xd0}, //14fps 01
           {0x2b , 0x09},
           {0x2c , 0xc4}, //10fps 10
           {0x2d , 0x0b},
           {0x2e , 0xb8}, //8fps 11
           {0xfe , 0x00},
*/
	},/*ANTIBANDING 50HZ*/
};

//begin effect
static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_effect_normal[] = {
	/* normal: */
	{0x83, 0x00},//0xe0
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_effect_black_white[] = {
	/* B&W: */
        {0x83, 0x12},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_effect_negative[] = {
	/* Negative: */
        {0x83, 0x01},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_effect_old_movie[] = {
	/* Sepia(antique): */
        {0x83, 0x82},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_effect_solarize[] = {
	{0xfe, 0x00},
};
// end effect


//begin scene, not realised
static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_scene_auto[] = {
	/* <SCENE_auto> */
		{0xfe, 0x00},

};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */
		{0xfe, 0x00},

};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */
		{0xfe, 0x00},

};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_scene_night[] = {
	/* <SCENE_NIGHT> */
		{0xfe, 0x00},

};
//end scene


//begin white balance
static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_wb_auto[] = {
	/* Auto: */
{0xb3, 0x61},{0xb4, 0x40},{0xb5, 0x61},{0x82, 0xfa},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_wb_sunny[] = {
	/* Sunny: */
{0x82, 0xf8},{0xb3, 0x58},{0xb4, 0x40},{0xb5, 0x50},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_wb_cloudy[] = {
	/* Cloudy: */
{0x82, 0xf8},{0xb3, 0x8c},{0xb4, 0x50},{0xb5, 0x40},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_wb_office[] = {
	/* Office: */
{0x82, 0xf8},{0xb3, 0x72},{0xb4, 0x40},{0xb5, 0x5b},
};

static struct msm_camera_i2c_reg_conf gc2145_pop10_reg_wb_home[] = {
	/* Home: */
{0x82, 0xf8},{0xb3, 0x50},{0xb4, 0x40},{0xb5, 0xa8},
};
//end white balance


static const struct i2c_device_id gc2145_pop10_i2c_id[] = {
	{GC2145_POP10_SENSOR_NAME, (kernel_ulong_t)&gc2145_pop10_s_ctrl},
	{ }
};

static int32_t msm_gc2145_pop10_i2c_probe(struct i2c_client *client,
	   const struct i2c_device_id *id)
{
	int rc;
	rc = msm_sensor_i2c_probe(client, id, &gc2145_pop10_s_ctrl);
/*	if(rc==0)
	{
		productinfo_register(PRODUCTINFO_FRONT_CAMERA_ID,"gc2145_pop10",NULL);
	}
*/
	return rc;
	
}

static struct i2c_driver gc2145_pop10_i2c_driver = {
	.id_table = gc2145_pop10_i2c_id,
	.probe  = msm_gc2145_pop10_i2c_probe,
	.driver = {
		.name = GC2145_POP10_SENSOR_NAME,
	},
};
static struct msm_camera_i2c_client gc2145_pop10_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id gc2145_pop10_dt_match[] = {
	{.compatible = "qcom,gc2145_pop10", .data = &gc2145_pop10_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gc2145_pop10_dt_match);

static struct platform_driver gc2145_pop10_platform_driver = {
	.driver = {
		.name = "qcom,gc2145_pop10",
		.owner = THIS_MODULE,
		.of_match_table = gc2145_pop10_dt_match,
	},
};

static void gc2145_pop10_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
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

static int32_t gc2145_pop10_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_sleep_settings[0],
		ARRAY_SIZE(gc2145_pop10_sleep_settings));
	return msm_sensor_power_down(s_ctrl);
}

static int32_t gc2145_pop10_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(gc2145_pop10_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gc2145_pop10_init_module(void)
{
	int32_t rc;
	CDBG("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&gc2145_pop10_platform_driver,
		gc2145_pop10_platform_probe);
	if (!rc)
		return rc;
	CDBG("%s:%d\n", __func__, __LINE__);
		
	return i2c_add_driver(&gc2145_pop10_i2c_driver);

}

static void __exit gc2145_pop10_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gc2145_pop10_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc2145_pop10_s_ctrl);
		platform_driver_unregister(&gc2145_pop10_platform_driver);
	} else
		i2c_del_driver(&gc2145_pop10_i2c_driver);
	return;
}

static int32_t gc2145_pop10_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid_H = 0;
	uint16_t chipid_L = 0;
	uint16_t chipid = 0;

	CDBG("%s, E. calling i2c_read:, i2c_addr:%d, id_reg_addr:%d\n",
		__func__,
		s_ctrl->sensordata->slave_info->sensor_slave_addr,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0xf0,
			&chipid_H, MSM_CAMERA_I2C_BYTE_DATA);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0xf1,
			&chipid_L, MSM_CAMERA_I2C_BYTE_DATA);
			
		chipid = (chipid_H << 8) | (chipid_L & 0xff) ;
	
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	//CDBG("%s:  read id: %x expected id gc2145_pop10:\n", __func__, chipid);
	      CDBG("gc2145_pop10_PETER-21-read chipid = 0x%x" , chipid);
	
		  
	if (chipid != 0x2145) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	return rc;
}

static void gc2145_pop10_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{

	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_saturation[value][0],
		ARRAY_SIZE(gc2145_pop10_reg_saturation[value]));


		
}

static void gc2145_pop10_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_contrast[value][0],
		ARRAY_SIZE(gc2145_pop10_reg_contrast[value]));
}

static void gc2145_pop10_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 7;
	CDBG("%s %d\n", __func__, value);
	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_sharpness[val][0],
		ARRAY_SIZE(gc2145_pop10_reg_sharpness[val]));
}


static void gc2145_pop10_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_iso[value][0],
		ARRAY_SIZE(gc2145_pop10_reg_iso[value]));
}

static void gc2145_pop10_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{

	int val = (value + 12) / 6;
	CDBG("%s %d\n", __func__, value);
	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_exposure_compensation[val][0],
		ARRAY_SIZE(gc2145_pop10_reg_exposure_compensation[val]));
		

		   
		
}

static void gc2145_pop10_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_effect_normal[0],
			ARRAY_SIZE(gc2145_pop10_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_effect_black_white[0],
			ARRAY_SIZE(gc2145_pop10_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_effect_negative[0],
			ARRAY_SIZE(gc2145_pop10_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_effect_old_movie[0],
			ARRAY_SIZE(gc2145_pop10_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_effect_solarize[0],
			ARRAY_SIZE(gc2145_pop10_reg_effect_solarize));
		break;
	}
	default:
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_effect_normal[0],
			ARRAY_SIZE(gc2145_pop10_reg_effect_normal));
	}
}

static void gc2145_pop10_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	   CDBG("gc2145_pop10_PETER，gc2145_pop10_set_antibanding = %x" , value);
	gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_antibanding[value][0],
		ARRAY_SIZE(gc2145_pop10_reg_antibanding[value]));
}

static void gc2145_pop10_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_scene_auto[0],
			ARRAY_SIZE(gc2145_pop10_reg_scene_auto));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_scene_night[0],
			ARRAY_SIZE(gc2145_pop10_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_scene_landscape[0],
			ARRAY_SIZE(gc2145_pop10_reg_scene_landscape));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_scene_portrait[0],
			ARRAY_SIZE(gc2145_pop10_reg_scene_portrait));
					break;
	}
	default:
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_scene_auto[0],
			ARRAY_SIZE(gc2145_pop10_reg_scene_auto));
	}
}

static void gc2145_pop10_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_wb_auto[0],
			ARRAY_SIZE(gc2145_pop10_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_wb_home[0],
			ARRAY_SIZE(gc2145_pop10_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_wb_sunny[0],
			ARRAY_SIZE(gc2145_pop10_reg_wb_sunny));
					break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_wb_office[0],
			ARRAY_SIZE(gc2145_pop10_reg_wb_office));
					break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_wb_cloudy[0],
			ARRAY_SIZE(gc2145_pop10_reg_wb_cloudy));
					break;
	}
	default:
		gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_reg_wb_auto[0],
		ARRAY_SIZE(gc2145_pop10_reg_wb_auto));
	}
}




void gc2145_pop10_set_shutter(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t temp = 1;
	uint16_t shutter = 1;
	uint16_t shutter_h = 1,shutter_L = 1 ;



	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0xfe,
		0x00, MSM_CAMERA_I2C_BYTE_DATA); 


	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0xb6,
		0x00, MSM_CAMERA_I2C_BYTE_DATA);  //AEC off


	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x03,
			&shutter_h, MSM_CAMERA_I2C_BYTE_DATA);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x04,
			&shutter_L, MSM_CAMERA_I2C_BYTE_DATA);
			
        shutter = (shutter_h << 8) | (shutter_L & 0xff) ;
     //   CDBG("=====>preview_shutter=  %x  \n", shutter);
        	   CDBG("gc2145_pop10_PETER，preview_shutter = %d" , shutter);
       	shutter = shutter / 2;
	if (shutter < 1)
		shutter = 1;
	shutter = shutter & 0x1fff;

	

        temp = shutter & 0xff;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x04,
		temp, MSM_CAMERA_I2C_BYTE_DATA);
	
	temp = (shutter >> 8) & 0xff;		
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x03,
		temp, MSM_CAMERA_I2C_BYTE_DATA);		

}


int32_t gc2145_pop10_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
    
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
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
		CDBG("init setting\n");
		gc2145_pop10_i2c_write_table(s_ctrl,
				&gc2145_pop10_recommend_settings[0],
				ARRAY_SIZE(gc2145_pop10_recommend_settings));
		CDBG("init setting X\n");
		break;
	case CFG_SET_RESOLUTION:  //peter
	{
		int val = 0;
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
	
	    	   CDBG("gc2145_pop10_PETER-preview/capture-VAL = %d" , val);
	    	   printk(KERN_INFO "JackChen:gc2145_pop10_PETER-preview/capture-VAL = %d\n" , val);
	
	if (val == 0)
		{
			gc2145_pop10_set_shutter(s_ctrl);//ppp
		
			gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_uxga_settings[0],
				ARRAY_SIZE(gc2145_pop10_uxga_settings));
					msleep(250);
		}
	else if (val == 1)
	        {
			gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_svga_settings[0],
				ARRAY_SIZE(gc2145_pop10_svga_settings));
			        msleep(200);//add betty
		}
		}
		break;
	case CFG_SET_STOP_STREAM:
		gc2145_pop10_i2c_write_table(s_ctrl,
			&gc2145_pop10_stop_settings[0],
			ARRAY_SIZE(gc2145_pop10_stop_settings));
		break;
	case CFG_SET_START_STREAM:
		gc2145_pop10_i2c_write_table(s_ctrl,
			&gc2145_pop10_start_settings[0],
			ARRAY_SIZE(gc2145_pop10_start_settings));
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

		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
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
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:

		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;

		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
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
			if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
	
		gc2145_pop10_set_stauration(s_ctrl, sat_lev);

		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
			if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);

		gc2145_pop10_set_contrast(s_ctrl, con_lev);

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
	
		gc2145_pop10_set_sharpness(s_ctrl, shp_lev);

		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: ISO Value is %d\n", __func__, iso_lev);

		gc2145_pop10_set_iso(s_ctrl, iso_lev);
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
				//if(0)
		gc2145_pop10_set_exposure_compensation(s_ctrl, ec_lev);

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
		CDBG("%s: Effect mode is %d\n", __func__, effect_mode);
	
		gc2145_pop10_set_effect(s_ctrl, effect_mode);
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
		CDBG("%s: anti-banding mode is %d\n", __func__,
			antibanding_mode);

		gc2145_pop10_set_antibanding(s_ctrl, antibanding_mode);
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
		CDBG("%s: best shot mode is %d\n", __func__, bs_mode);
	
		gc2145_pop10_set_scene_mode(s_ctrl, bs_mode);
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
		CDBG("%s: white balance is %d\n", __func__, wb_mode);
	
		gc2145_pop10_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

#ifdef CONFIG_COMPAT
int32_t gc2145_pop10_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
	long rc = 0;
	int32_t i = 0;
    
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
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
		CDBG("init setting\n");
		gc2145_pop10_i2c_write_table(s_ctrl,
				&gc2145_pop10_recommend_settings[0],
				ARRAY_SIZE(gc2145_pop10_recommend_settings));
		CDBG("init setting X\n");
		break;
	case CFG_SET_RESOLUTION:  //peter
	{
		int val = 0;
		if (copy_from_user(&val,
			(void *)compat_ptr(cdata->cfg.setting), sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
	
	    	   CDBG("gc2145_pop10_PETER-preview/capture-VAL = %d" , val);
	    	   printk(KERN_INFO "JackChen:gc2145_pop10_PETER-preview/capture-VAL = %d\n" , val);
	
	if (val == 0)
		{
			gc2145_pop10_set_shutter(s_ctrl);//ppp
		
			gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_uxga_settings[0],
				ARRAY_SIZE(gc2145_pop10_uxga_settings));
					msleep(250);
		}
	else if (val == 1)
	        {
			gc2145_pop10_i2c_write_table(s_ctrl, &gc2145_pop10_svga_settings[0],
				ARRAY_SIZE(gc2145_pop10_svga_settings));
			        msleep(200);//add betty
		}
		}
		break;
	case CFG_SET_STOP_STREAM:
		gc2145_pop10_i2c_write_table(s_ctrl,
			&gc2145_pop10_stop_settings[0],
			ARRAY_SIZE(gc2145_pop10_stop_settings));
		break;
	case CFG_SET_START_STREAM:
		gc2145_pop10_i2c_write_table(s_ctrl,
			&gc2145_pop10_start_settings[0],
			ARRAY_SIZE(gc2145_pop10_start_settings));
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

		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
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
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:

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
		if (copy_from_user(stop_setting32, (void *)compat_ptr(cdata->cfg.setting),
		    sizeof(struct msm_camera_i2c_reg_setting))) {
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
			if (copy_from_user(&sat_lev, (void *)compat_ptr(cdata->cfg.setting),
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
	
		gc2145_pop10_set_stauration(s_ctrl, sat_lev);

		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
			if (copy_from_user(&con_lev, (void *)compat_ptr(cdata->cfg.setting),
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);

		gc2145_pop10_set_contrast(s_ctrl, con_lev);

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
	
		gc2145_pop10_set_sharpness(s_ctrl, shp_lev);

		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)compat_ptr(cdata->cfg.setting),
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: ISO Value is %d\n", __func__, iso_lev);

		gc2145_pop10_set_iso(s_ctrl, iso_lev);
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
				//if(0)
		gc2145_pop10_set_exposure_compensation(s_ctrl, ec_lev);

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
		CDBG("%s: Effect mode is %d\n", __func__, effect_mode);
	
		gc2145_pop10_set_effect(s_ctrl, effect_mode);
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
		CDBG("%s: anti-banding mode is %d\n", __func__,
			antibanding_mode);

		gc2145_pop10_set_antibanding(s_ctrl, antibanding_mode);
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
		CDBG("%s: best shot mode is %d\n", __func__, bs_mode);
	
		gc2145_pop10_set_scene_mode(s_ctrl, bs_mode);
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
		CDBG("%s: white balance is %d\n", __func__, wb_mode);
	
		gc2145_pop10_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
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

static struct msm_sensor_fn_t gc2145_pop10_sensor_func_tbl = {
	.sensor_config = gc2145_pop10_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = gc2145_pop10_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = gc2145_pop10_sensor_power_down,
	.sensor_match_id = gc2145_pop10_sensor_match_id,
};

static struct msm_sensor_ctrl_t gc2145_pop10_s_ctrl = {
	.sensor_i2c_client = &gc2145_pop10_sensor_i2c_client,
	.power_setting_array.power_setting = gc2145_pop10_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gc2145_pop10_power_setting),
	.msm_sensor_mutex = &gc2145_pop10_mut,
	.sensor_v4l2_subdev_info = gc2145_pop10_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc2145_pop10_subdev_info),
	.func_tbl = &gc2145_pop10_sensor_func_tbl,
};

module_init(gc2145_pop10_init_module);
module_exit(gc2145_pop10_exit_module);
MODULE_DESCRIPTION("GC2145_POP10 2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");



