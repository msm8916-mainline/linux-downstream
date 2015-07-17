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
 */
#include "msm_sensor.h"
#define GC0339_SENSOR_NAME "gc0339_alto5"
DEFINE_MSM_MUTEX(gc0339_mut);
static struct msm_sensor_ctrl_t gc0339_s_ctrl;

static struct msm_sensor_power_setting gc0339_power_setting[] = {
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VANA,
        .config_val = GPIO_OUT_LOW,
        .delay = 0,
    },
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VDIG,
        .config_val = 1,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VANA,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_CLK,
        .seq_val = SENSOR_CAM_MCLK,
        .config_val = 24000000,
        .delay = 2,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_STANDBY,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
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
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
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
        .delay = 2,
    },
};

static struct v4l2_subdev_info gc0339_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR8_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};



static const struct i2c_device_id gc0339_i2c_id[] = {
	{GC0339_SENSOR_NAME,
		(kernel_ulong_t)&gc0339_s_ctrl},
	{ }
};

static int32_t msm_gc0339_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gc0339_s_ctrl);
}

static struct i2c_driver gc0339_i2c_driver = {
	.id_table = gc0339_i2c_id,
	.probe  = msm_gc0339_i2c_probe,
	.driver = {
		.name = GC0339_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc0339_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};



static const struct of_device_id gc0339_dt_match[] = {
	{
		.compatible = "qcom,gc0339_alto5",
		.data = &gc0339_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, gc0339_dt_match);

static struct platform_driver gc0339_platform_driver = {
	.driver = {
		.name = "qcom,gc0339_alto5",
		.owner = THIS_MODULE,
		.of_match_table = gc0339_dt_match,
	},
};

static int32_t gc0339_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(gc0339_dt_match, &pdev->dev);
	if (match)
		rc = msm_sensor_platform_probe(pdev, match->data);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	return rc;
}

static int __init gc0339_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&gc0339_platform_driver,
		gc0339_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&gc0339_i2c_driver);
}

static void __exit gc0339_exit_module(void)
{
	if (gc0339_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc0339_s_ctrl);
		platform_driver_unregister(&gc0339_platform_driver);
	} else
		i2c_del_driver(&gc0339_i2c_driver);
	return;
}

static int32_t gc0339_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	pr_err("%s %d Enter.\n", __func__, __LINE__);
	
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0xfc, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: %d: read id = %d\n", __func__, __LINE__, chipid);
	if ((rc<0)||(chipid != 0xc8)) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("%s chip id doesnot match\n", __func__);
		return -ENODEV;
	}
	return rc;
}

static struct msm_sensor_fn_t gc0339_sensor_fn_t = {
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = gc0339_match_id,
	.sensor_config = msm_sensor_config,
};

static struct msm_sensor_ctrl_t gc0339_s_ctrl = {
	.sensor_i2c_client = &gc0339_sensor_i2c_client,
	.power_setting_array.power_setting = gc0339_power_setting,
	.power_setting_array.size =
			ARRAY_SIZE(gc0339_power_setting),
	.msm_sensor_mutex = &gc0339_mut,
	.sensor_v4l2_subdev_info = gc0339_subdev_info,
	.sensor_v4l2_subdev_info_size =
			ARRAY_SIZE(gc0339_subdev_info),
	.func_tbl = &gc0339_sensor_fn_t,
};
module_init(gc0339_init_module);
module_exit(gc0339_exit_module);
MODULE_DESCRIPTION("gc0339");
MODULE_LICENSE("GPL v2");
