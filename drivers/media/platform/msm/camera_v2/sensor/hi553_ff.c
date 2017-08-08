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
#define HI553_FF_SENSOR_NAME "hi553_ff"
#include <mach/board_lge.h>
DEFINE_MSM_MUTEX(hi553_ff_mut);

static struct msm_sensor_ctrl_t hi553_ff_s_ctrl;

#if defined(CONFIG_LGE_P1B_CAMERA) || defined(CONFIG_LGE_K5_CAMERA)
static struct msm_sensor_power_setting hi553_ff_power_setting[] = {
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 23880000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
};
#elif defined(CONFIG_MSM8916_LGE_M2_CAMERA)
static struct msm_sensor_power_setting hi553_ff_power_setting[] = {
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 23880000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
};
#else
static struct msm_sensor_power_setting hi553_ff_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 23880000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
};
#endif

static struct v4l2_subdev_info hi553_ff_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hi553_ff_i2c_id[] = {
	{HI553_FF_SENSOR_NAME, (kernel_ulong_t)&hi553_ff_s_ctrl},
	{ }
};

static int32_t msm_hi553_ff_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hi553_ff_s_ctrl);
}

static struct i2c_driver hi553_ff_i2c_driver = {
	.id_table = hi553_ff_i2c_id,
	.probe  = msm_hi553_ff_i2c_probe,
	.driver = {
		.name = HI553_FF_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi553_ff_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hi553_ff_dt_match[] = {
	{.compatible = "qcom,hi553_ff", .data = &hi553_ff_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi553_ff_dt_match);

static struct platform_driver hi553_ff_platform_driver = {
	.driver = {
		.name = "qcom,hi553_ff",
		.owner = THIS_MODULE,
		.of_match_table = hi553_ff_dt_match,
	},
};

static int32_t hi553_ff_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(hi553_ff_dt_match, &pdev->dev);
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	 }
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hi553_ff_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);

	rc = platform_driver_probe(&hi553_ff_platform_driver,
		hi553_ff_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hi553_ff_i2c_driver);
}

int32_t hi553_ff_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid[2];

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid[0], MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %d %s: read id failed\n", __func__,__LINE__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr|0x0001,
			&chipid[1], MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %d %s: read id failed\n", __func__,__LINE__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	chipid[1] = chipid[0] << 8 | chipid[1];

	if (chipid[1] != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("%s read id 0x%x does not match with expected id 0x%x\n",__func__,
			chipid[1], s_ctrl->sensordata->slave_info->sensor_id);
		return -ENODEV;
	}

	return rc;

}

static struct msm_sensor_fn_t hi553_ff_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static void __exit hi553_ff_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi553_ff_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi553_ff_s_ctrl);
		platform_driver_unregister(&hi553_ff_platform_driver);
	} else
		i2c_del_driver(&hi553_ff_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hi553_ff_s_ctrl = {
	.sensor_i2c_client = &hi553_ff_sensor_i2c_client,
	.power_setting_array.power_setting = hi553_ff_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi553_ff_power_setting),
	.msm_sensor_mutex = &hi553_ff_mut,
	.sensor_v4l2_subdev_info = hi553_ff_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi553_ff_subdev_info),
	.func_tbl = &hi553_ff_sensor_func_tbl,
};

module_init(hi553_ff_init_module);
module_exit(hi553_ff_exit_module);
MODULE_DESCRIPTION("hi553_ff");
MODULE_LICENSE("GPL v2");

