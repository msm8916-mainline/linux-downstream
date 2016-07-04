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
 */
#include "msm_sensor.h"
#define IMX258_SENSOR_NAME "imx258"
#include <mach/board_lge.h>
DEFINE_MSM_MUTEX(imx258_mut);

static struct msm_sensor_ctrl_t imx258_s_ctrl;

static struct msm_sensor_power_setting imx258_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 23880000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info imx258_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx258_i2c_id[] = {
	{IMX258_SENSOR_NAME, (kernel_ulong_t)&imx258_s_ctrl},
	{ }
};

static int32_t msm_imx258_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx258_s_ctrl);
}

static struct i2c_driver imx258_i2c_driver = {
	.id_table = imx258_i2c_id,
	.probe  = msm_imx258_i2c_probe,
	.driver = {
		.name = IMX258_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx258_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx258_dt_match[] = {
	{.compatible = "qcom,imx258", .data = &imx258_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx258_dt_match);

static struct platform_driver imx258_platform_driver = {
	.driver = {
		.name = "qcom,imx258",
		.owner = THIS_MODULE,
		.of_match_table = imx258_dt_match,
	},
};

static int32_t imx258_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx258_dt_match, &pdev->dev);
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	 }
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx258_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);

	rc = platform_driver_probe(&imx258_platform_driver,
		imx258_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx258_i2c_driver);
}

static struct msm_sensor_fn_t imx258_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static void __exit imx258_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx258_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx258_s_ctrl);
		platform_driver_unregister(&imx258_platform_driver);
	} else
		i2c_del_driver(&imx258_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t imx258_s_ctrl = {
	.sensor_i2c_client = &imx258_sensor_i2c_client,
	.power_setting_array.power_setting = imx258_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx258_power_setting),
	.msm_sensor_mutex = &imx258_mut,
	.sensor_v4l2_subdev_info = imx258_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx258_subdev_info),
	.func_tbl = &imx258_sensor_func_tbl,
};

module_init(imx258_init_module);
module_exit(imx258_exit_module);
MODULE_DESCRIPTION("imx258");
MODULE_LICENSE("GPL v2");

