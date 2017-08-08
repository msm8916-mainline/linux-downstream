/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#define BF3905_SENSOR_NAME "bf3905"
#define PLATFORM_DRIVER_NAME "msm_camera_bf3905"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


#define SENSOR_REG_PAGE_ADDR 0x03
#define SENSOR_REG_PAGE_0 0x00
#define SENSOR_REG_PAGE_20 0x20

#define SENSOR_PREVIEW_WIDTH 640
#define SENSOR_PREVIEW_HEIGHT 480

#define AWB_LOCK_ON 1
#define AWB_LOCK_OFF 0
#define AEC_LOCK_ON 1
#define AEC_LOCK_OFF 0

#define AEC_ROI_DX (192) // (128)
#define AEC_ROI_DY (192) // (128) // (96)

static int PREV_SOC_AEC_LOCK = -1;	/*LGE_CHANGE, to prevent duplicated setting, 2013-01-07, kwangsik83.kim@lge.com*/
static int PREV_SOC_AWB_LOCK = -1;  /*LGE_CHANGE, to prevent duplicated setting, 2013-01-07, kwangsik83.kim@lge.com*/

DEFINE_MSM_MUTEX(bf3905_mut);
static struct msm_sensor_ctrl_t bf3905_s_ctrl;

static struct msm_sensor_power_setting bf3905_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#if 0
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
		{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},

	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_camera_i2c_reg_conf bf3905_start_settings[] = {
	
};

static struct msm_camera_i2c_reg_conf bf3905_stop_settings[] = {
	
};
static struct msm_camera_i2c_reg_conf bf3905_recommend_settings[] = {
	{0x15, 0x12},
	{0x09, 0x01},
	{0x12, 0x00},
	{0x3a, 0x00},
	{0x1e, 0x70}, //0x40}, mirror and vetically flip
	{0x1b, 0x0e},
	{0x2a, 0x00},
	{0x2b, 0x10},
	{0x92, 0x09},
	{0x93, 0x00},
	{0x8a, 0x96},
	{0x8b, 0x7d},
	{0x13, 0x00},
	{0x01, 0x15},
	{0x02, 0x23},
	{0x9d, 0x20},
	{0x8c, 0x02},
	{0x8d, 0xee},
	{0x13, 0x07},
	{0x5d, 0xb3},
	{0xbf, 0x08},
	{0xc3, 0x08},
	{0xca, 0x10},
	{0x62, 0x00},
	{0x63, 0x00},
	{0xb9, 0x00},
	{0x64, 0x00},
	{0xbb, 0x10},
	{0x08, 0x02},
	{0x20, 0x09},
	{0x21, 0x4f},
	{0x3e, 0x83},
	{0x2f, 0x04},
	{0x16, 0xa3},//0xaf
	{0x6c, 0xc2},
	{0x27, 0x98},
	{0x71, 0x0f},
	{0x7e, 0x84},
	{0x7f, 0x3c},
	{0x60, 0xe5},
	{0x61, 0xf2},
	{0x6d, 0xc0},
	{0xd9, 0x25},
	{0xdf, 0x26},
	{0x17, 0x00},
	{0x18, 0xa0},
	{0x19, 0x00},
	{0x1a, 0x78},
	{0x03, 0xa0},
	{0x4a, 0x0c},
	{0xda, 0x00},
	{0xdb, 0xa2},
	{0xdc, 0x00},
	{0xdd, 0x7a},
	{0xde, 0x00},
	{0x34, 0x1d},
	{0x36, 0x45},
	{0x6e, 0x20},
	{0xbc, 0x0d},
	{0x35, 0x50},
	{0x65, 0x42},
	{0x66, 0x42},
	{0xbd, 0xf4},
	{0xbe, 0x44},
	{0x9b, 0xf4},
	{0x9c, 0x44},
	{0x37, 0xf4},
	{0x38, 0x44},
	{0x70, 0x0b},
	{0x73, 0x27},
	{0x79, 0x24},
	{0x7a, 0x12},
	{0x75, 0x8a},
	{0x76, 0x98},
	{0x77, 0x2a},
	{0x7b, 0x58},
	{0x7d, 0x00},
	{0x13, 0x07},
	{0x24, 0x4a},
	{0x25, 0x88},
	{0x97, 0x3c},
	{0x98, 0x8a},
	{0x80, 0x92},
	{0x81, 0x00},
	{0x82, 0x2a},
	{0x83, 0x54},
	{0x84, 0x39},
	{0x85, 0x5d},
	{0x86, 0x88},
	{0x89, 0x63},
	{0x94, 0x82},
	{0x96, 0xb3},
	{0x9a, 0x50},
	{0x99, 0x10},
	{0x9f, 0x64},
	{0x39, 0x9c},
	{0x3f, 0x9c},
	{0x90, 0x20},
	{0x91, 0xd0},
	{0x40, 0x3b},
	{0x41, 0x36},
	{0x42, 0x2b},
	{0x43, 0x1d},
	{0x44, 0x1a},
	{0x45, 0x14},
	{0x46, 0x11},
	{0x47, 0x0f},
	{0x48, 0x0e},
	{0x49, 0x0d},
	{0x4b, 0x0c},
	{0x4c, 0x0b},
	{0x4e, 0x0a},
	{0x4f, 0x09},
	{0x50, 0x09},
	{0x5a, 0x56},
	{0x51, 0x13},
	{0x52, 0x05},
	{0x53, 0x91},
	{0x54, 0x72},
	{0x57, 0x96},
	{0x58, 0x35},
	{0x5a, 0xd6},
	{0x51, 0x28},
	{0x52, 0x35},
	{0x53, 0x9e},
	{0x54, 0x7d},
	{0x57, 0x50},
	{0x58, 0x15},
	{0x5c, 0x26},
	{0x6a, 0x81},
	{0x23, 0x55},
	{0xa1, 0x31},
	{0xa2, 0x0d},
	{0xa3, 0x27},
	{0xa4, 0x0a},
	{0xa5, 0x2c},
	{0xa6, 0x04},
	{0xa7, 0x1a},
	{0xa8, 0x18},
	{0xa9, 0x13},
	{0xaa, 0x18},
	{0xab, 0x1c},
	{0xac, 0x3c},
	{0xad, 0xf0},
	{0xae, 0x57},
	{0xd0, 0x92},
	{0xd1, 0x00},
	{0xd2, 0x58},
	{0xc5, 0xaa},
	{0xc6, 0x88},
	{0xc7, 0x10},
	{0xc8, 0x0d},
	{0xc9, 0x10},
	{0xd3, 0x09},
	{0xd4, 0x24},
	{0xee, 0x30},
	{0xb0, 0xe0},
	{0xb3, 0x48},
	{0xb4, 0xe3},
	{0xb1, 0xf0},
	{0xb2, 0xa0},
	{0xb4, 0x63},
	{0xb1, 0xb0},
	{0xb2, 0xa0},
	{0x55, 0x00},
	{0x56, 0x40},
};

static struct msm_camera_i2c_reg_conf bf3905_reg_effect_off[] = {
	/* OFF */
};

static struct msm_camera_i2c_reg_conf bf3905_reg_effect_mono[] = {
	/* MONO */
};

static struct msm_camera_i2c_reg_conf bf3905_reg_effect_negative[] = {
	/* Negative: */
};

static struct msm_camera_i2c_reg_conf bf3905_reg_effect_sepia[] = {
	/* SEPIA  */
};

static struct msm_camera_i2c_reg_conf bf3905_reg_exposure_compensation[13][2] = {
	/* -6 */
	{
		{0x03, 0x10},
		{0x40, 0xBC},
	},
	/* -5 */
	{
		{0x03, 0x10},
		{0x40, 0xB2},
	},
	/* -4 */
	{
		{0x03, 0x10},
		{0x40, 0xA8},
	},
	/* -3 */
	{
		{0x03, 0x10},
		{0x40, 0x9E},
	},
	/* -2 */
	{
		{0x03, 0x10},
		{0x40, 0x94},
	},
	/* -1 */
	{
		{0x03, 0x10},
		{0x40, 0x8A},
	},
	/* 0 */
	{
		{0x03, 0x10},
		{0x40, 0x80},
	},
	/* 1 */
	{
		{0x03, 0x10},
		{0x40, 0x0A},
	},
	/* 2 */
	{
		{0x03, 0x10},
		{0x40, 0x14},
	},
	/* 3 */
	{
		{0x03, 0x10},
		{0x40, 0x1E},
	},
	/* 4 */
	{
		{0x03, 0x10},
		{0x40, 0x28},
	},
	/* 5 */
	{
		{0x03, 0x10},
		{0x40, 0x32},
	},
	/* 6 */
	{
		{0x03, 0x10},
		{0x40, 0x3C},
	},
};

static struct msm_camera_i2c_reg_conf bf3905_reg_scene_auto[] = {
	/* SCENE_auto: 10~30fps */
	
};

static struct msm_camera_i2c_reg_conf bf3905_reg_wb_auto[] = {
	/* Auto */
};

static struct msm_camera_i2c_reg_conf bf3905_reg_wb_incandescent[] = {
	
};

static struct msm_camera_i2c_reg_conf bf3905_reg_wb_fluorescent[] = {
	
};

static struct msm_camera_i2c_reg_conf bf3905_reg_wb_sunny[] = {
	
};

static struct msm_camera_i2c_reg_conf bf3905_reg_wb_cloudy[] = {
	
};

static struct msm_camera_i2c_reg_conf bf3905_AE_weight[] = {

};

static struct msm_camera_i2c_reg_conf bf3905_AE_window[] = {

};

static struct v4l2_subdev_info bf3905_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,/* For YUV type sensor (YUV422) */
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id bf3905_i2c_id[] = {
	{BF3905_SENSOR_NAME, (kernel_ulong_t)&bf3905_s_ctrl},
	{ }
};

static int32_t msm_bf3905_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &bf3905_s_ctrl);
}

static struct i2c_driver bf3905_i2c_driver = {
	.id_table = bf3905_i2c_id,
	.probe  = msm_bf3905_i2c_probe,
	.driver = {
		.name = BF3905_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client bf3905_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id bf3905_dt_match[] = {
	{.compatible = "qcom,bf3905", .data = &bf3905_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, bf3905_dt_match);

static struct platform_driver bf3905_platform_driver = {
	.driver = {
		.name = "qcom,bf3905",
		.owner = THIS_MODULE,
		.of_match_table = bf3905_dt_match,
	},
};

static void bf3905_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
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

static int32_t bf3905_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(bf3905_dt_match, &pdev->dev);
/* LGE_CHANGE_S, WBT issue fix, 2013-11-25, hyunuk.park@lge.com */
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	}
/* LGE_CHANGE_E, WBT issue fix, 2013-11-25, hyunuk.park@lge.com */
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init bf3905_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&bf3905_platform_driver,
		bf3905_platform_probe);
	if (!rc)
		return rc;

	return i2c_add_driver(&bf3905_i2c_driver);
}

static void __exit bf3905_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (bf3905_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&bf3905_s_ctrl);
		platform_driver_unregister(&bf3905_platform_driver);
	} else
		i2c_del_driver(&bf3905_i2c_driver);
	return;
}

static void bf3905_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = 0;

	switch (value) {
		case -12 :{ val = 0; break;}
		case -10 :{ val = 1; break;}
		case -8  :{ val = 2; break;}
		case -6  :{ val = 3; break;}
		case -4  :{ val = 4; break;}
		case -2  :{ val = 5; break;}
		case 0   :{ val = 6; break;}
		case 2   :{ val = 7; break;}
		case 4   :{ val = 8; break;}
		case 6   :{ val = 9; break;}
		case 8   :{ val = 10; break;}
		case 10  :{ val = 11; break;}
		case 12  :{ val = 12; break;}
		default  :{ val = 6; break;}
	}

	CDBG("%s %d", __func__, val);
	bf3905_i2c_write_table(s_ctrl, &bf3905_reg_exposure_compensation[val][0],
		ARRAY_SIZE(bf3905_reg_exposure_compensation[val]));
}

static void bf3905_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_effect_off[0],
			ARRAY_SIZE(bf3905_reg_effect_off));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_effect_mono[0],
			ARRAY_SIZE(bf3905_reg_effect_mono));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_effect_negative[0],
			ARRAY_SIZE(bf3905_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_effect_sepia[0],
			ARRAY_SIZE(bf3905_reg_effect_sepia));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE:
	case MSM_CAMERA_EFFECT_MODE_POSTERIZE:
	case MSM_CAMERA_EFFECT_MODE_WHITEBOARD:
	case MSM_CAMERA_EFFECT_MODE_BLACKBOARD:
	case MSM_CAMERA_EFFECT_MODE_AQUA:
	case MSM_CAMERA_EFFECT_MODE_EMBOSS:
	case MSM_CAMERA_EFFECT_MODE_SKETCH:
	case MSM_CAMERA_EFFECT_MODE_NEON:
	default:
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_effect_off[0],
			ARRAY_SIZE(bf3905_reg_effect_off));
		break;
	}
}

static void bf3905_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_scene_auto[0],
			ARRAY_SIZE(bf3905_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE:
	case MSM_CAMERA_SCENE_MODE_SNOW:
	case MSM_CAMERA_SCENE_MODE_BEACH:
	case MSM_CAMERA_SCENE_MODE_SUNSET:
	case MSM_CAMERA_SCENE_MODE_PORTRAIT:
	case MSM_CAMERA_SCENE_MODE_BACKLIGHT:
	case MSM_CAMERA_SCENE_MODE_SPORTS:
	case MSM_CAMERA_SCENE_MODE_ANTISHAKE:
	case MSM_CAMERA_SCENE_MODE_FLOWERS:
	case MSM_CAMERA_SCENE_MODE_CANDLELIGHT:
	case MSM_CAMERA_SCENE_MODE_FIREWORKS:
	case MSM_CAMERA_SCENE_MODE_PARTY:
	case MSM_CAMERA_SCENE_MODE_NIGHT_PORTRAIT:
	case MSM_CAMERA_SCENE_MODE_THEATRE:
	case MSM_CAMERA_SCENE_MODE_ACTION:
	case MSM_CAMERA_SCENE_MODE_AR:
	case MSM_CAMERA_SCENE_MODE_FACE_PRIORITY:
	case MSM_CAMERA_SCENE_MODE_BARCODE:
	case MSM_CAMERA_SCENE_MODE_HDR:
	default:
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_scene_auto[0],
			ARRAY_SIZE(bf3905_reg_scene_auto));
		break;
	}
}

static void bf3905_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_wb_auto[0],
			ARRAY_SIZE(bf3905_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_wb_incandescent[0],
			ARRAY_SIZE(bf3905_reg_wb_incandescent));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_wb_fluorescent[0],
			ARRAY_SIZE(bf3905_reg_wb_fluorescent));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_wb_sunny[0],
			ARRAY_SIZE(bf3905_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_wb_cloudy[0],
			ARRAY_SIZE(bf3905_reg_wb_cloudy));
		break;
	}
	case MSM_CAMERA_WB_MODE_CUSTOM:
	case MSM_CAMERA_WB_MODE_WARM_FLUORESCENT:
	case MSM_CAMERA_WB_MODE_TWILIGHT:
	case MSM_CAMERA_WB_MODE_SHADE:
	case MSM_CAMERA_WB_MODE_OFF:
	default:
		bf3905_i2c_write_table(s_ctrl, &bf3905_reg_wb_auto[0],
		ARRAY_SIZE(bf3905_reg_wb_auto));
		break;
	}
}




static int32_t bf3905_set_aec_roi_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int32_t rc = 0;
	int16_t coordinate_x, coordinate_y;
	int16_t x_start, x_end, y_start, y_end;

	coordinate_x = ((value >> 16) & 0xFFFF);
	coordinate_y = value & 0xFFFF;

	CDBG("%s: roi point (%d, %d)\n", __func__, coordinate_x, coordinate_y);
	if(coordinate_x == 0 && coordinate_y == 0)
	{
	pr_err("%s: roi zero value",__func__);
	return rc;
	}

	if(coordinate_x == -1 && coordinate_y == -1) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
						SENSOR_REG_PAGE_ADDR,
						SENSOR_REG_PAGE_20,
						MSM_CAMERA_I2C_BYTE_DATA);
		/* AE weight */
	    bf3905_i2c_write_table(s_ctrl,&bf3905_AE_weight[0],
				ARRAY_SIZE(bf3905_AE_weight));
		/* AE window */
	    bf3905_i2c_write_table(s_ctrl,&bf3905_AE_window[0],
				ARRAY_SIZE(bf3905_AE_window));


		if (rc < 0) {
			pr_err("%s: %s: failed\n", __func__,s_ctrl->sensordata->sensor_name);
			return rc;
		}
	}
	else {

#ifdef CONFIG_BF3905_ROT_180
		coordinate_x = SENSOR_PREVIEW_WIDTH - coordinate_x;
		coordinate_y = SENSOR_PREVIEW_HEIGHT -coordinate_y;
#endif

		x_start = ((coordinate_x - (AEC_ROI_DX/2) > 0)? coordinate_x - (AEC_ROI_DX/2) : 0)/4;
		x_end = ((coordinate_x + (AEC_ROI_DX/2) < SENSOR_PREVIEW_WIDTH)? coordinate_x + (AEC_ROI_DX/2) : SENSOR_PREVIEW_WIDTH)/4;

		y_start = ((coordinate_y - (AEC_ROI_DY/2) > 0)? coordinate_y - (AEC_ROI_DY/2) : 0)/2;
		y_end = ((coordinate_y + (AEC_ROI_DY/2) < SENSOR_PREVIEW_HEIGHT)? coordinate_y + (AEC_ROI_DY/2) : SENSOR_PREVIEW_HEIGHT)/2;

		CDBG("%s : (%d, %d), (%d, %d)\n", __func__, x_start, y_start, x_end, y_end);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, SENSOR_REG_PAGE_ADDR, SENSOR_REG_PAGE_20, MSM_CAMERA_I2C_BYTE_DATA);

		/* AE weight */
	    bf3905_i2c_write_table(s_ctrl,&bf3905_AE_weight[0],
				ARRAY_SIZE(bf3905_AE_weight));

		/* AE window */
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x68, x_start, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x69, x_end, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6a, y_start, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6b, y_end, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: %s: failed\n", __func__,s_ctrl->sensordata->sensor_name);
			return rc;
		}
	}

	return rc;
}

static int32_t bf3905_set_awb_lock_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int32_t rc = 0;
	int16_t temp = 0;
	CDBG("%s %d", __func__, value);

	if(PREV_SOC_AWB_LOCK != value){

		PREV_SOC_AWB_LOCK = value;

		if(PREV_SOC_AWB_LOCK == AWB_LOCK_ON){
			CDBG("%s soc_awb_lock %d\n", __func__, PREV_SOC_AWB_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x22, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp & 0x7f; //[7]bit set as '0'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
		else if(PREV_SOC_AWB_LOCK == AWB_LOCK_OFF){
			CDBG("%s soc_awb_unlock %d\n", __func__, PREV_SOC_AWB_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x22, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,	0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp | 0x80; //[7]bit set as '1'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
	}

	return rc;
}

static int32_t bf3905_set_aec_lock_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int32_t rc = 0;
	int16_t temp = 0;
	CDBG("%s %d", __func__, value);

	if(PREV_SOC_AEC_LOCK != value){

		PREV_SOC_AEC_LOCK = value;

		if(PREV_SOC_AEC_LOCK == AEC_LOCK_ON){
			CDBG("%s soc_aec_lock %d\n", __func__, PREV_SOC_AEC_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x20, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,	0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp & 0x7f; //[7]bit set as '0'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
		else if(PREV_SOC_AEC_LOCK == AEC_LOCK_OFF){
			CDBG("%s soc_aec_unlock %d\n", __func__, PREV_SOC_AEC_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x20, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,	0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp | 0x80; //[7]bit set as '1'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
	}
	return rc;
}

int32_t bf3905_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
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
/*LGE_CHANGE_S, set sensor position, 2014-06-13, jeognda.lee@lge.com*/
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		CDBG("%s:%d sensor position %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.position);
/*LGE_CHANGE_E, set sensor position, 2014-06-13, jeognda.lee@lge.com*/
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
		pr_err("set init");
		CDBG("init setting");
		bf3905_i2c_write_table(s_ctrl,
				&bf3905_recommend_settings[0],
				ARRAY_SIZE(bf3905_recommend_settings));
		CDBG("init setting X");
		break;
	case CFG_SET_RESOLUTION:
		break;

	case CFG_SET_STOP_STREAM:
		pr_err("%s - stop stream",__func__);
		CDBG("STOP_STREAM\n");
		bf3905_i2c_write_table(s_ctrl,
			&bf3905_stop_settings[0],
			ARRAY_SIZE(bf3905_stop_settings));
		CDBG("STOP_STREAM X\n");
		break;
	case CFG_SET_START_STREAM:
		pr_err("%s - start stream",__func__);
		CDBG("START_STREAM\n");

		bf3905_i2c_write_table(s_ctrl, &bf3905_start_settings[0], ARRAY_SIZE(bf3905_start_settings));
		pr_err("[WX] %s : normal start stream\n", __func__);

		CDBG("START_STREAM X\n");
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
/*LGE_CHANGE_S, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/
	case CFG_PAGE_MODE_READ_I2C_ARRAY:{
		int16_t size=0;
		uint16_t read_data_size = 0;
		uint16_t *read_data;
		uint16_t *read_data_head;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

//		pr_err("[WX] %s CFG_PAGE_MODE_READ_I2C_ARRAY\n", __func__);

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		size = conf_array.size;		//size for write(page_mode) and read
		read_data_size = size - 1;	//size for read

		pr_err("[WX] %s: size : %d rsize : %d\n", __func__, size, read_data_size);

		if (!size || !read_data_size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(size *(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			size * sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		read_data = kzalloc(read_data_size * (sizeof(uint16_t)), GFP_KERNEL);
		if (!read_data) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}

		//check if this code is needed;;;
		if (copy_from_user(read_data, (void *)conf_array.value,
			read_data_size * sizeof(uint16_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		//

		conf_array.reg_setting = reg_setting;
		read_data_head = read_data;

		for(i = 0; i < size; i++){
			if(i == 0){
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, conf_array.reg_setting->reg_addr, conf_array.reg_setting->reg_data, conf_array.data_type);
			}
			else{
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, conf_array.reg_setting->reg_addr, read_data, conf_array.data_type);
				pr_err("[WX] %s read_data : %d\n", __func__, *read_data);
				read_data++;
			}
			conf_array.reg_setting++;
		}

		read_data = read_data_head;

		if (copy_to_user((void *)conf_array.value, read_data, read_data_size * sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		kfree(reg_setting);
		kfree(read_data);

		reg_setting = NULL;
		read_data = NULL;
		read_data_head = NULL;

		pr_err("[WX] %s done\n", __func__);

		break;
	}
/*LGE_CHANGE_E, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/
/*LGE_CHANGE_S, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
	case CFG_POWER_UP:{

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__, __LINE__,
					s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_up) {
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);

			if (rc < 0) {
				pr_err("%s POWER_UP failed\n", __func__);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
					s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	}
	case CFG_POWER_DOWN:{

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_down){
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);

			if (rc < 0) {
				pr_err("%s POWER_DOWN failed\n", __func__);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);

		}else{
			rc = -EFAULT;
		}
		break;
	}
/*LGE_CHANGE_E, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
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
		break;
	}
	case CFG_SET_CONTRAST: {
		break;
	}
	case CFG_SET_SHARPNESS: {
		break;
	}
	case CFG_SET_ISO: {
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
		CDBG("%s: Exposure compensation Value is %d",__func__, ec_lev);
		bf3905_set_exposure_compensation(s_ctrl, ec_lev);
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
		CDBG("%s: Effect mode is %d", __func__, effect_mode);
		bf3905_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
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
		CDBG("%s: best shot mode is %d", __func__, bs_mode);
		bf3905_set_scene_mode(s_ctrl, bs_mode);
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
		CDBG("%s: white balance is %d", __func__, wb_mode);
		bf3905_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
	case CFG_SET_AEC_ROI:{
		int32_t coordinate;
		if (copy_from_user(&coordinate, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: coordinate is %d", __func__, coordinate);
		rc = bf3905_set_aec_roi_mode(s_ctrl, coordinate);
		break;
	}
	case CFG_SET_AWB_LOCK: {
		int32_t awb_lock;
		if (copy_from_user(&awb_lock, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: awb_lock is %d", __func__, awb_lock);
		rc = bf3905_set_awb_lock_mode(s_ctrl, awb_lock);
		break;
	}
    case CFG_SET_AEC_LOCK:{
    	int32_t aec_lock;
		if (copy_from_user(&aec_lock, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: aec_lock is %d", __func__, aec_lock);
		rc = bf3905_set_aec_lock_mode(s_ctrl, aec_lock);
		break;
	}

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t bf3905_sensor_func_tbl = {
	.sensor_config = bf3905_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t bf3905_s_ctrl = {
	.sensor_i2c_client = &bf3905_sensor_i2c_client,
	.power_setting_array.power_setting = bf3905_power_setting,
	.power_setting_array.size = ARRAY_SIZE(bf3905_power_setting),
	.msm_sensor_mutex = &bf3905_mut,
	.sensor_v4l2_subdev_info = bf3905_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(bf3905_subdev_info),
	.func_tbl = &bf3905_sensor_func_tbl,
};

module_init(bf3905_init_module);
module_exit(bf3905_exit_module);
MODULE_DESCRIPTION("BYD VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");

