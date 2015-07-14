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
#define HI544_SENSOR_NAME "hi544"
DEFINE_MSM_MUTEX(hi544_mut);

static struct msm_sensor_ctrl_t hi544_s_ctrl;

/////////////////////////////////////////////////////////////////
// USE Model Feature: If you want to change the follow setting
/////////////////////////////////////////////////////////////////
static struct msm_sensor_power_setting hi544_power_setting[] = {
//USE Vendor Probe

};

static struct v4l2_subdev_info hi544_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hi544_i2c_id[] = {
	{HI544_SENSOR_NAME, (kernel_ulong_t)&hi544_s_ctrl},
	{ }
};

static int32_t msm_hi544_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hi544_s_ctrl);
}

static struct i2c_driver hi544_i2c_driver = {
	.id_table = hi544_i2c_id,
	.probe  = msm_hi544_i2c_probe,
	.driver = {
		.name = HI544_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi544_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hi544_dt_match[] = {
	{.compatible = "qcom,hi544", .data = &hi544_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi544_dt_match);

static struct platform_driver hi544_platform_driver = {
	.driver = {
		.name = "qcom,hi544",
		.owner = THIS_MODULE,
		.of_match_table = hi544_dt_match,
	},
};

static int32_t hi544_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(hi544_dt_match, &pdev->dev);
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	 }
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hi544_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);

	rc = platform_driver_probe(&hi544_platform_driver,
		hi544_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hi544_i2c_driver);
}

static struct msm_sensor_fn_t hi544_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static void __exit hi544_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi544_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi544_s_ctrl);
		platform_driver_unregister(&hi544_platform_driver);
	} else
		i2c_del_driver(&hi544_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hi544_s_ctrl = {
	.sensor_i2c_client = &hi544_sensor_i2c_client,
	.power_setting_array.power_setting = hi544_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi544_power_setting),
	.msm_sensor_mutex = &hi544_mut,
	.sensor_v4l2_subdev_info = hi544_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi544_subdev_info),
	.func_tbl = &hi544_sensor_func_tbl,
};

module_init(hi544_init_module);
module_exit(hi544_exit_module);
MODULE_DESCRIPTION("hi544");
MODULE_LICENSE("GPL v2");

