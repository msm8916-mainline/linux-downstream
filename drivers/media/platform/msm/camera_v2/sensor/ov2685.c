/* Copyritht (C) 2013 Intrinsyc Software International Inc.
 * Copyright (c) 2011-2013, Code Aurora Forum. All rights reserved.
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
 */

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm.h"
#define OV2685_SENSOR_NAME "ov2685"
#define PLATFORM_DRIVER_NAME "msm_camera_ov2685"
#define ov2685_obj ov2685_##obj
#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MUTEX(ov2685_mut);
static struct msm_sensor_ctrl_t ov2685_s_ctrl;
static struct class *camera_class = NULL;
char slave_camera_info[7] = OV2685_SENSOR_NAME;

static ssize_t slave_camera_info_show(struct class *class,
                     struct class_attribute *attr, char *buf){
	if (slave_camera_info != NULL)
		return snprintf(buf, 32, "%s\n", slave_camera_info);
	return 0;
}
static CLASS_ATTR(slave_camera_info, 0644, slave_camera_info_show, NULL);

static struct msm_sensor_power_setting ov2685_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},

    {
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},


/*
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
*/
	/*{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
/*
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
*/
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_camera_i2c_reg_conf ov2685_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf ov2685_stop_settings[] = {
	{0x0100, 0x00},
};

/// ALTEK_IQM >>>
static struct msm_camera_i2c_reg_conf ov2685_hw_frame_sync_settings[] = {
    {0x3826, 0x05}, // r, calibrated by Altek, 11us, n=0x0610-0x05d4=60
    {0x3827, 0xd4},

};
/// ALTEK_IQM <<<

static struct msm_camera_i2c_reg_conf ov2685_1600x1200p30_settings[] = {
	{0x0103 , 0x01},
	{0x3002 , 0x00},
	{0x3016 , 0x1c},
	{0x3018 , 0x84},
	{0x301d , 0xf0},
	{0x3020 , 0x00},
	{0x3082 , 0x2c},
	{0x3083 , 0x03},
	{0x3084 , 0x07},
	{0x3085 , 0x03},
	{0x3086 , 0x00},
	{0x3087 , 0x00},
	{0x3501 , 0x4e},
	{0x3502 , 0xe0},
	{0x3503 , 0x03},
	{0x350b , 0x36},
	{0x3600 , 0xb4},
	{0x3603 , 0x35},
	{0x3604 , 0x24},
	{0x3605 , 0x00},
	{0x3620 , 0x24},
	{0x3621 , 0x34},
	{0x3622 , 0x03},
	{0x3628 , 0x10},
	{0x3705 , 0x3c},
	{0x370a , 0x21},
	{0x370c , 0x50},
	{0x370d , 0xc0},
	{0x3717 , 0x58},
	{0x3718 , 0x80},
	{0x3720 , 0x00},
	{0x3721 , 0x09},
	{0x3722 , 0x06},
	{0x3723 , 0x59},
	{0x3738 , 0x99},
	{0x3781 , 0x80},
	{0x3784 , 0x0c},
	{0x3789 , 0x60},
	{0x3800 , 0x00},
	{0x3801 , 0x00},
	{0x3802 , 0x00},
	{0x3803 , 0x00},
	{0x3804 , 0x06},
	{0x3805 , 0x4f},
	{0x3806 , 0x04},
	{0x3807 , 0xbf},
	{0x3808 , 0x06},
	{0x3809 , 0x40},
	{0x380a , 0x04},
	{0x380b , 0xb0},
	{0x380c , 0x06},
	{0x380d , 0xa4},
	{0x380e , 0x05},
	{0x380f , 0x0e},
	{0x3810 , 0x00},
	{0x3811 , 0x08},
	{0x3812 , 0x00},
	{0x3813 , 0x08},
	{0x3814 , 0x11},
	{0x3815 , 0x11},
	{0x3819 , 0x04},
	{0x3820 , 0xc0},
    {0x3821 , 0x00},
	{0x3a06 , 0x01},
	{0x3a07 , 0x84},
	{0x3a08 , 0x01},
	{0x3a09 , 0x43},
	{0x3a0a , 0x24},
	{0x3a0b , 0x60},
	{0x3a0c , 0x28},
	{0x3a0d , 0x60},
	{0x3a0e , 0x04},
	{0x3a0f , 0x8c},
	{0x3a10 , 0x05},
	{0x3a11 , 0x0c},
	{0x4000 , 0x81},
	{0x4001 , 0x40},
	{0x4008 , 0x02},
	{0x4009 , 0x09},
	{0x4300 , 0x30},
	{0x430e , 0x00},
	{0x4602 , 0x02},
	{0x481b , 0x40},
	{0x481f , 0x40},
	{0x4837 , 0x1e},
	{0x5000 , 0xff},
	{0x5001 , 0x05},
	{0x5002 , 0x32},
	{0x5003 , 0x04},
	{0x5004 , 0xfd},
	{0x5005 , 0x12},
	{0x0100 , 0x00},
	{0x5180 , 0xf4},
	{0x5181 , 0x11},
	{0x5182 , 0x41},
	{0x5183 , 0x42},
	{0x5184 , 0x75},
	{0x5185 , 0x55},
	{0x5186 , 0xa5},
	{0x5187 , 0xa0},
	{0x5188 , 0x0a},
	{0x5189 , 0x0e},
	{0x518a , 0x0c},
	{0x518b , 0x40},
	{0x518c , 0x2c},
	{0x518d , 0xf8},
	{0x518e , 0x04},
	{0x518f , 0x7f},
	{0x5190 , 0x40},
	{0x5191 , 0x5f},
	{0x5192 , 0x40},
	{0x5193 , 0xff},
	{0x5194 , 0x40},
	{0x5195 , 0x06},
	{0x5196 , 0x43},
	{0x5197 , 0x04},
	{0x5198 , 0x00},
	{0x5199 , 0x05},
	{0x519a , 0xd3},
	{0x519b , 0x04},
	{0x5200 , 0x09},
	{0x5201 , 0x00},
	{0x5202 , 0x06},
	{0x5203 , 0x20},
	{0x5204 , 0x41},
	{0x5205 , 0x16},
	{0x5206 , 0x00},
	{0x5207 , 0x05},
	{0x520b , 0x30},
	{0x520c , 0x75},
	{0x520d , 0x00},
	{0x520e , 0x30},
	{0x520f , 0x75},
	{0x5210 , 0x00},
	{0x5280 , 0x14},
	{0x5281 , 0x02},
	{0x5282 , 0x02},
	{0x5283 , 0x04},
	{0x5284 , 0x06},
	{0x5285 , 0x08},
	{0x5286 , 0x0c},
	{0x5287 , 0x10},
	{0x5300 , 0xc5},
	{0x5301 , 0xa0},
	{0x5302 , 0x06},
	{0x5303 , 0x08},
	{0x5304 , 0x18},
	{0x5305 , 0x30},
	{0x5306 , 0x60},
	{0x5307 , 0xc0},
	{0x5308 , 0x82},
	{0x5309 , 0x00},
	{0x530a , 0x0c},
	{0x530b , 0x02},
	{0x530c , 0x02},
	{0x530d , 0x00},
	{0x530e , 0x0c},
	{0x530f , 0x14},
	{0x5310 , 0x1a},
	{0x5311 , 0x20},
	{0x5312 , 0x80},
	{0x5313 , 0x4b},
	{0x5380 , 0x01},
	{0x5381 , 0x6e},
	{0x5382 , 0x00},
	{0x5383 , 0x6d},
	{0x5384 , 0x00},
	{0x5385 , 0x72},
	{0x5386 , 0x00},
	{0x5387 , 0x5f},
	{0x5388 , 0x00},
	{0x5389 , 0x4f},
	{0x538a , 0x01},
	{0x538b , 0x89},
	{0x538c , 0x00},
	{0x5400 , 0x05},
	{0x5401 , 0x0e},
	{0x5402 , 0x25},
	{0x5403 , 0x4e},
	{0x5404 , 0x60},
	{0x5405 , 0x6e},
	{0x5406 , 0x7b},
	{0x5407 , 0x86},
	{0x5408 , 0x90},
	{0x5409 , 0x99},
	{0x540a , 0xa7},
	{0x540b , 0xb3},
	{0x540c , 0xc8},
	{0x540d , 0xdb},
	{0x540e , 0xeb},
	{0x540f , 0xa0},
	{0x5410 , 0x6e},
	{0x5411 , 0x06},
	{0x5480 , 0x19},
	{0x5481 , 0x00},
	{0x5482 , 0x09},
	{0x5483 , 0x12},
	{0x5484 , 0x04},
	{0x5485 , 0x06},
	{0x5486 , 0x08},
	{0x5487 , 0x0c},
	{0x5488 , 0x10},
	{0x5489 , 0x18},
	{0x5500 , 0x02},
	{0x5501 , 0x03},
	{0x5502 , 0x04},
	{0x5503 , 0x05},
	{0x5504 , 0x06},
	{0x5505 , 0x08},
	{0x5506 , 0x00},
	{0x5600 , 0x02},
	{0x5603 , 0x40},
	{0x5604 , 0x28},
	{0x5609 , 0x20},
	{0x560a , 0x60},
	{0x5780 , 0x3e},
	{0x5781 , 0x0f},
	{0x5782 , 0x04},
	{0x5783 , 0x02},
	{0x5784 , 0x01},
	{0x5785 , 0x01},
	{0x5786 , 0x00},
	{0x5787 , 0x04},
	{0x5788 , 0x02},
	{0x5789 , 0x00},
	{0x578a , 0x01},
	{0x578b , 0x02},
	{0x578c , 0x03},
	{0x578d , 0x03},
	{0x578e , 0x08},
	{0x578f , 0x0c},
	{0x5790 , 0x08},
	{0x5791 , 0x04},
	{0x5792 , 0x00},
	{0x5793 , 0x00},
	{0x5794 , 0x03},
	{0x5800 , 0x03},
	{0x5801 , 0x1d},
	{0x5802 , 0x02},
	{0x5803 , 0x49},
	{0x5804 , 0x4b},
	{0x5805 , 0x05},
	{0x5806 , 0x97},
	{0x5807 , 0x05},
	{0x5808 , 0x03},
	{0x5809 , 0x25},
	{0x580a , 0x02},
	{0x580b , 0x4b},
	{0x580c , 0x3d},
	{0x580d , 0x05},
	{0x580e , 0x92},
	{0x580f , 0x05},
	{0x5810 , 0x03},
	{0x5811 , 0x14},
	{0x5812 , 0x02},
	{0x5813 , 0x48},
	{0x5814 , 0x38},
	{0x5815 , 0x05},
	{0x5816 , 0x99},
	{0x5817 , 0x05},
	{0x5818 , 0x0d},
	{0x5819 , 0x40},
	{0x581a , 0x04},
	{0x581b , 0x0c},
	{0x3a03 , 0x42},
	{0x3a04 , 0x36},
	{0x3503 , 0x00},
	{0x3080 , 0x00},  
	{0x3018 , 0x44},  
	{0x3084 , 0x0f},  
	{0x3085 , 0x07},  
	{0x4837 , 0x0f},  
	{0x380e , 0x06},  
	{0x380f , 0x10},  	

/// ALTEK_IQM >>>
    // HW frame sync, OV2685 slave setting
    {0x3002, 0x00},
    {0x3823, 0x30},

    {0x3824, 0x00}, // cs
    {0x3825, 0x20},

    {0x3826, 0x00}, // r
    {0x3827, 0x06},

    {0x3503, 0x03}, // AE manual
    {0x5180, 0x03}, // MWB
/// ALTEK_IQM <<<
};

static struct msm_camera_i2c_reg_conf ov2685_720p60_settings[] = {
  {0x0103, 0x01},
  {0x3002, 0x00},
  {0x3016, 0x1c},
  {0x3018, 0x84},
  {0x301d, 0xf0},
  {0x3020, 0x00},
  {0x3082, 0x2c},
  {0x3083, 0x03},
  {0x3084, 0x07},
  {0x3085, 0x03},
  {0x3086, 0x00},
  {0x3087, 0x00},
  {0x3501, 0x2d},
  {0x3502, 0x80},
  {0x3503, 0x03},
  {0x350b, 0x36},
  {0x3600, 0xb4},
  {0x3603, 0x35},
  {0x3604, 0x24},
  {0x3605, 0x00},
  {0x3620, 0x26},
  {0x3621, 0x37},
  {0x3622, 0x04},
  {0x3628, 0x10},
  {0x3705, 0x3c},
  {0x370a, 0x21},
  {0x370c, 0x50},
  {0x370d, 0xc0},
  {0x3717, 0x58},
  {0x3718, 0x88},
  {0x3720, 0x00},
  {0x3721, 0x00},
  {0x3722, 0x00},
  {0x3723, 0x00},
  {0x3738, 0x00},
  {0x3781, 0x80},
  {0x3784, 0x0c},
  {0x3789, 0x60},
  {0x3800, 0x00},
  {0x3801, 0xa0},
  {0x3802, 0x00},
  {0x3803, 0xf2},
  {0x3804, 0x05},
  {0x3805, 0xaf},
  {0x3806, 0x03},
  {0x3807, 0xcd},
  {0x3808, 0x05},
  {0x3809, 0x00},
  {0x380a, 0x02},
  {0x380b, 0xd0},
  {0x380c, 0x05},
  {0x380d, 0xa6},
  {0x380e, 0x02},
  {0x380f, 0xf8},
  {0x3810, 0x00},
  {0x3811, 0x08},
  {0x3812, 0x00},
  {0x3813, 0x06},
  {0x3814, 0x11},
  {0x3815, 0x11},
  {0x3819, 0x04},
  {0x3820, 0xc0},
  {0x3821, 0x00},
  {0x3a06, 0x01},
  {0x3a07, 0xc8},
  {0x3a08, 0x01},
  {0x3a09, 0x7c},
  {0x3a0a, 0x0e},
  {0x3a0b, 0x40},
  {0x3a0c, 0x17},
  {0x3a0d, 0xc0},
  {0x3a0e, 0x01},
  {0x3a0f, 0xc8},
  {0x3a10, 0x02},
  {0x3a11, 0xf8},
  {0x4000, 0x81},
  {0x4001, 0x40},
  {0x4008, 0x02},
  {0x4009, 0x09},
  {0x4300, 0x30},
  {0x430e, 0x00},
  {0x4602, 0x02},
  {0x481b, 0x40},
  {0x481f, 0x40},
  {0x4837, 0x1e},
  {0x5000, 0xff},
  {0x5001, 0x05},
  {0x5002, 0x32},
  {0x5003, 0x04},
  {0x5004, 0xfd},
  {0x5005, 0x12},
  {0x5180, 0xf4},
  {0x5181, 0x11},
  {0x5182, 0x41},
  {0x5183, 0x42},
  {0x5184, 0x75},
  {0x5185, 0x55},
  {0x5186, 0xa5},
  {0x5187, 0xa0},
  {0x5188, 0x0a},
  {0x5189, 0x0e},
  {0x518a, 0x0c},
  {0x518b, 0x40},
  {0x518c, 0x2c},
  {0x518d, 0xf8},
  {0x518e, 0x04},
  {0x518f, 0x7f},
  {0x5190, 0x40},
  {0x5191, 0x5f},
  {0x5192, 0x40},
  {0x5193, 0xff},
  {0x5194, 0x40},
  {0x5195, 0x06},
  {0x5196, 0x43},
  {0x5197, 0x04},
  {0x5198, 0x00},
  {0x5199, 0x05},
  {0x519a, 0xd3},
  {0x519b, 0x04},
  {0x5200, 0x09},
  {0x5201, 0x00},
  {0x5202, 0x06},
  {0x5203, 0x20},
  {0x5204, 0x41},
  {0x5205, 0x16},
  {0x5206, 0x00},
  {0x5207, 0x05},
  {0x520b, 0x30},
  {0x520c, 0x75},
  {0x520d, 0x00},
  {0x520e, 0x30},
  {0x520f, 0x75},
  {0x5210, 0x00},
  {0x5280, 0x14},
  {0x5281, 0x02},
  {0x5282, 0x02},
  {0x5283, 0x04},
  {0x5284, 0x06},
  {0x5285, 0x08},
  {0x5286, 0x0c},
  {0x5287, 0x10},
  {0x5300, 0xc5},
  {0x5301, 0xa0},
  {0x5302, 0x06},
  {0x5303, 0x08},
  {0x5304, 0x18},
  {0x5305, 0x30},
  {0x5306, 0x60},
  {0x5307, 0xc0},
  {0x5308, 0x82},
  {0x5309, 0x00},
  {0x530a, 0x0c},
  {0x530b, 0x02},
  {0x530c, 0x02},
  {0x530d, 0x00},
  {0x530e, 0x0c},
  {0x530f, 0x14},
  {0x5310, 0x1a},
  {0x5311, 0x20},
  {0x5312, 0x80},
  {0x5313, 0x4b},
  {0x5380, 0x01},
  {0x5381, 0x6e},
  {0x5382, 0x00},
  {0x5383, 0x6d},
  {0x5384, 0x00},
  {0x5385, 0x72},
  {0x5386, 0x00},
  {0x5387, 0x5f},
  {0x5388, 0x00},
  {0x5389, 0x4f},
  {0x538a, 0x01},
  {0x538b, 0x89},
  {0x538c, 0x00},
  {0x5400, 0x05},
  {0x5401, 0x0e},
  {0x5402, 0x25},
  {0x5403, 0x4e},
  {0x5404, 0x60},
  {0x5405, 0x6e},
  {0x5406, 0x7b},
  {0x5407, 0x86},
  {0x5408, 0x90},
  {0x5409, 0x99},
  {0x540a, 0xa7},
  {0x540b, 0xb3},
  {0x540c, 0xc8},
  {0x540d, 0xdb},
  {0x540e, 0xeb},
  {0x540f, 0xa0},
  {0x5410, 0x6e},
  {0x5411, 0x06},
  {0x5480, 0x19},
  {0x5481, 0x00},
  {0x5482, 0x09},
  {0x5483, 0x12},
  {0x5484, 0x04},
  {0x5485, 0x06},
  {0x5486, 0x08},
  {0x5487, 0x0c},
  {0x5488, 0x10},
  {0x5489, 0x18},
  {0x5500, 0x02},
  {0x5501, 0x03},
  {0x5502, 0x04},
  {0x5503, 0x05},
  {0x5504, 0x06},
  {0x5505, 0x08},
  {0x5506, 0x00},
  {0x5600, 0x02},
  {0x5603, 0x40},
  {0x5604, 0x28},
  {0x5609, 0x20},
  {0x560a, 0x60},
  {0x5780, 0x3e},
  {0x5781, 0x0f},
  {0x5782, 0x04},
  {0x5783, 0x02},
  {0x5784, 0x01},
  {0x5785, 0x01},
  {0x5786, 0x00},
  {0x5787, 0x04},
  {0x5788, 0x02},
  {0x5789, 0x00},
  {0x578a, 0x01},
  {0x578b, 0x02},
  {0x578c, 0x03},
  {0x578d, 0x03},
  {0x578e, 0x08},
  {0x578f, 0x0c},
  {0x5790, 0x08},
  {0x5791, 0x04},
  {0x5792, 0x00},
  {0x5793, 0x00},
  {0x5794, 0x03},
  {0x5800, 0x03},
  {0x5801, 0x1d},
  {0x5802, 0x02},
  {0x5803, 0x49},
  {0x5804, 0x4b},
  {0x5805, 0x05},
  {0x5806, 0x97},
  {0x5807, 0x05},
  {0x5808, 0x03},
  {0x5809, 0x25},
  {0x580a, 0x02},
  {0x580b, 0x4b},
  {0x580c, 0x3d},
  {0x580d, 0x05},
  {0x580e, 0x92},
  {0x580f, 0x05},
  {0x5810, 0x03},
  {0x5811, 0x14},
  {0x5812, 0x02},
  {0x5813, 0x48},
  {0x5814, 0x38},
  {0x5815, 0x05},
  {0x5816, 0x99},
  {0x5817, 0x05},
  {0x5818, 0x0d},
  {0x5819, 0x40},
  {0x581a, 0x04},
  {0x581b, 0x0c},
  {0x3a03, 0x42},
  {0x3a04, 0x36},
  {0x3503, 0x00},

/// ALTEK_IQM >>>
    {0x3503, 0x03}, // AE manual
    {0x5180, 0x03}, // MWB
/// ALTEK_IQM <<<
};

struct ov2685_resolution_table_t {
	char *name;
	struct msm_camera_i2c_reg_conf *settings;
	unsigned int size; /* ARRAY_SIZE(settings) */
};
/* This table has to be in the same order as they are in the sensor lib */
static struct ov2685_resolution_table_t ov2685_resolutions[] = {
	{"2MP 30fps", ov2685_1600x1200p30_settings, ARRAY_SIZE(ov2685_1600x1200p30_settings)},
	{"720p 60fps",  ov2685_720p60_settings,  ARRAY_SIZE(ov2685_720p60_settings)},
};

int32_t ov2685_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = -EINVAL;
	CDBG("Power Up");

	msleep(20);

	rc = msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s: msm_sensor_power_up failed\n", __func__);
		return rc;
	}

	return rc;
}

/* FIXME: Stop stream null for now, use VFE stop */
/* static void ov2685_stop_stream(struct msm_sensor_ctrl_t *s_ctrl) {} */

int32_t ov2685_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG("Power Down");

	msleep(20);
	msm_sensor_power_down(s_ctrl);

	return 0;
}

static struct v4l2_subdev_info ov2685_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order  = 0,
	},
	/* more can be supported, to be added later */
};

static const struct i2c_device_id ov2685_i2c_id[] = {
	{OV2685_SENSOR_NAME, (kernel_ulong_t)&ov2685_s_ctrl},
	{ }
};

static int32_t msm_ov2685_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov2685_s_ctrl);
}

static struct i2c_driver ov2685_i2c_driver = {
	.id_table = ov2685_i2c_id,
	.probe  = msm_ov2685_i2c_probe,
	.driver = {
		.name = OV2685_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2685_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov2685_dt_match[] = {
	{.compatible = "ovti,ov2685", .data = &ov2685_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov2685_dt_match);

static struct platform_driver ov2685_platform_driver = {
	.driver = {
		.name = "ovti,ov2685",
		.owner = THIS_MODULE,
		.of_match_table = ov2685_dt_match,
	},
};

static int32_t ov2685_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(ov2685_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov2685_init_module(void)
{
	int32_t rc;
	int sys_ret = 0;
	pr_info("jerry-%s:%d\n", __func__, __LINE__);
	CDBG("jerry-afasfadfsdsdfs");
    rc = platform_driver_probe(&ov2685_platform_driver,
		ov2685_platform_probe);
	if (!rc){
		camera_class = class_create(THIS_MODULE, "yuv_camera");
		sys_ret = class_create_file(camera_class, &class_attr_slave_camera_info);
		return rc;
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov2685_i2c_driver);
}

static void __exit ov2685_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov2685_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov2685_s_ctrl);
		platform_driver_unregister(&ov2685_platform_driver);
	} else
		i2c_del_driver(&ov2685_i2c_driver);
	return;
}

int32_t ov2685_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	long rc = 0;
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t i = 0;
	enum msm_sensor_resolution_t res;
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
		/* 1. Write Recommend settings */
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_1600x1200p30_settings,
			ARRAY_SIZE(ov2685_1600x1200p30_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		/* 2. Write change settings */
		CDBG("%s:%d:CFG_SET_INIT_SETTING %ld\n", __func__, __LINE__, rc);
		break;
	case CFG_SET_RESOLUTION:
		{ int val = 0;
		CDBG(">>%s:%d<<\n",__func__, __LINE__);
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		res = val;
		CDBG(">>%s:%d<<\n", __func__, __LINE__);
		if ( val > 1) { // TODO
			rc = -EFAULT;
			CDBG(">>%s:%d<<\n", __func__, __LINE__);
			CDBG(">>No Good Resolution<<\n");
			break;
		}
		}
		CDBG("CFG_SET_RESOLUTION - picking %s\n", ov2685_resolutions[res].name);
		CDBG(">>%s:%d<<\n", __func__, __LINE__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_resolutions[res].settings,
			ov2685_resolutions[res].size,
			MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d:CFG_SET_RESOLUTION res=%d, rc=%ld", __func__, __LINE__, res, rc);
		break;
	case CFG_SET_STOP_STREAM:
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_stop_settings,
			ARRAY_SIZE(ov2685_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d:CFG_SET_STOP_STREAM %ld\n", __func__, __LINE__, rc);
		break;
	case CFG_SET_START_STREAM:
/// ALTEK_IQM >>>
        msleep(50); // make sure sub camera stream on after main camera
/// ALTEK_IQM <<<

        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_start_settings,
			ARRAY_SIZE(ov2685_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d:CFG_SET_START_STREAM %ld\n", __func__, __LINE__, rc);
/// ALTEK_IQM >>>
        msleep(20);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_hw_frame_sync_settings,
			ARRAY_SIZE(ov2685_hw_frame_sync_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
/// ALTEK_IQM <<<        
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
		static struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
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
		p_ctrl = &s_ctrl->sensordata->power_info;
		size = sensor_slave_info.power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(struct msm_sensor_power_setting)
				      * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;

		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info.power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
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
			p_ctrl->power_setting_size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				p_ctrl->power_setting[slave_index].seq_type,
				p_ctrl->power_setting[slave_index].seq_val,
				p_ctrl->power_setting[slave_index].config_val,
				p_ctrl->power_setting[slave_index].delay);
		}
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
                struct msm_camera_i2c_reg_setting32 conf_array32;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
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
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
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
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

#ifdef CONFIG_COMPAT
	int32_t ov2685_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	long rc = 0;
	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
	int32_t i = 0;
	enum msm_sensor_resolution_t res;
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
		/* 1. Write Recommend settings */
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_1600x1200p30_settings,
			ARRAY_SIZE(ov2685_1600x1200p30_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		/* 2. Write change settings */
		CDBG("%s:%d:CFG_SET_INIT_SETTING %ld\n", __func__, __LINE__, rc);
		break;
	case CFG_SET_RESOLUTION:
		{ int val = 0;
		CDBG(">>%s:%d<<\n",__func__, __LINE__);
		if (copy_from_user(&val,
			(void *)compat_ptr(cdata->cfg.setting), sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		res = val;
		CDBG(">>%s:%d<<\n", __func__, __LINE__);
		if ( val > 1) { // TODO
			rc = -EFAULT;
			CDBG(">>%s:%d<<\n", __func__, __LINE__);
			CDBG(">>No Good Resolution<<\n");
			break;
		}
		}
		CDBG("CFG_SET_RESOLUTION - picking %s\n", ov2685_resolutions[res].name);
		CDBG(">>%s:%d<<\n", __func__, __LINE__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_resolutions[res].settings,
			ov2685_resolutions[res].size,
			MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d:CFG_SET_RESOLUTION res=%d, rc=%ld", __func__, __LINE__, res, rc);
		break;
	case CFG_SET_STOP_STREAM:
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_stop_settings,
			ARRAY_SIZE(ov2685_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d:CFG_SET_STOP_STREAM %ld\n", __func__, __LINE__, rc);
		break;
	case CFG_SET_START_STREAM:
/// ALTEK_IQM >>>
        msleep(50); // make sure sub camera stream on after main camera
/// ALTEK_IQM <<<
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_start_settings,
			ARRAY_SIZE(ov2685_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		CDBG("%s:%d:CFG_SET_START_STREAM %ld\n", __func__, __LINE__, rc);
/// ALTEK_IQM >>>
        msleep(20);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ov2685_hw_frame_sync_settings,
			ARRAY_SIZE(ov2685_hw_frame_sync_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
/// ALTEK_IQM <<<        
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
		static struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)compat_ptr(cdata->cfg.setting),
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
		p_ctrl = &s_ctrl->sensordata->power_info;
		size = sensor_slave_info.power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(struct msm_sensor_power_setting)
				      * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;

		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info.power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
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
			p_ctrl->power_setting_size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				p_ctrl->power_setting[slave_index].seq_type,
				p_ctrl->power_setting[slave_index].seq_val,
				p_ctrl->power_setting[slave_index].config_val,
				p_ctrl->power_setting[slave_index].delay);
		}
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
/// ALTEK_IQM >>>
#if 1 // To fix copy data wrong bug
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

/*
        CDBG("%s:%d kernel CFG_WRITE_I2C_ARRAY size [%d] reg [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x] [0x%x=0x%x]\n", __func__,
			__LINE__, conf_array.size, 
            conf_array.reg_setting[0].reg_addr, conf_array.reg_setting[0].reg_data, 
            conf_array.reg_setting[1].reg_addr, conf_array.reg_setting[1].reg_data,
            conf_array.reg_setting[2].reg_addr, conf_array.reg_setting[2].reg_data,
            conf_array.reg_setting[3].reg_addr, conf_array.reg_setting[3].reg_data,
            conf_array.reg_setting[4].reg_addr, conf_array.reg_setting[4].reg_data,
            conf_array.reg_setting[5].reg_addr, conf_array.reg_setting[5].reg_data,
            conf_array.reg_setting[6].reg_addr, conf_array.reg_setting[6].reg_data,
            conf_array.reg_setting[7].reg_addr, conf_array.reg_setting[7].reg_data,
            conf_array.reg_setting[8].reg_addr, conf_array.reg_setting[8].reg_data,
            conf_array.reg_setting[9].reg_addr, conf_array.reg_setting[9].reg_data,
            conf_array.reg_setting[10].reg_addr, conf_array.reg_setting[10].reg_data,
            conf_array.reg_setting[11].reg_addr, conf_array.reg_setting[11].reg_data);
*/
        if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}            
#else
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
#endif
/// ALTEK_IQM <<<

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
			(void *)compat_ptr(cdata->cfg.setting),
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
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)compat_ptr(cdata->cfg.setting),
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
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#endif

static struct msm_sensor_fn_t ov2685_func_tbl = {
	.sensor_config = ov2685_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = ov2685_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t ov2685_s_ctrl = {
	.sensor_i2c_client = &ov2685_sensor_i2c_client,
	.power_setting_array.power_setting = ov2685_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov2685_power_setting),
	.msm_sensor_mutex = &ov2685_mut,
	.sensor_v4l2_subdev_info = ov2685_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2685_subdev_info),
	.func_tbl = &ov2685_func_tbl,
};

module_init(ov2685_init_module);
module_exit(ov2685_exit_module);
MODULE_AUTHOR("Jerry Yuan <jerry.yuan@ovt.com>");
MODULE_DESCRIPTION("Omnivision OV2685 2MP YUV driver");
MODULE_LICENSE("GPL v2");
