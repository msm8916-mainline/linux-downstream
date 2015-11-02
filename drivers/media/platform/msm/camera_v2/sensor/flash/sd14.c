/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "sd14"

//#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define SD14_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define SD14_DBG(fmt, args...)
#endif

#define FLASH_EN 933
#define FLASH_NOW 988
static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sd14_i2c_driver;

static const struct of_device_id sd14_i2c_trigger_dt_match[] = {
	{.compatible = "sd14"},
	{}
};

MODULE_DEVICE_TABLE(of, sd14_i2c_trigger_dt_match);
static const struct i2c_device_id sd14_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (value > LED_OFF) {
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_sd14_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

int msm_flash_sd14_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SD14_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value(FLASH_EN,GPIO_OUT_LOW);
	gpio_set_value(FLASH_NOW,GPIO_OUT_LOW);

	return rc;
}

int msm_flash_sd14_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	SD14_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value(FLASH_EN,GPIO_OUT_LOW);
	gpio_set_value(FLASH_NOW,GPIO_OUT_LOW);

	return rc;
}

int msm_flash_sd14_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	SD14_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value(FLASH_EN,GPIO_OUT_LOW);
	gpio_set_value(FLASH_NOW,GPIO_OUT_LOW);

	return rc;
}

int msm_flash_sd14_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SD14_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value(FLASH_EN,GPIO_OUT_HIGH);
	gpio_set_value(FLASH_NOW,GPIO_OUT_LOW);

	return rc;
}

int msm_flash_sd14_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	int value = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	SD14_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	//Send 9 1-line pulse signal, the flash timeout from 220ms to 1.3s
	udelay(20);
	value = 9;
	gpio_set_value(FLASH_EN,GPIO_OUT_LOW);
	while(--value) {
		gpio_set_value(FLASH_EN,GPIO_OUT_HIGH);
		udelay(5);
		gpio_set_value(FLASH_EN,GPIO_OUT_LOW);
		udelay(5);
	}
	
	gpio_set_value(FLASH_EN,GPIO_OUT_HIGH);
	gpio_set_value(FLASH_NOW,GPIO_OUT_HIGH);

	return rc;
}
static int msm_flash_sd14_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	SD14_DBG("%s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_sd14_i2c_probe: id is NULL");
		id = sd14_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	if (!rc)
		msm_sd14_torch_create_classdev(&(client->dev),NULL);
	return rc;
}

static int msm_flash_sd14_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	SD14_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return rc;
}


static struct i2c_driver sd14_i2c_driver = {
	.id_table = sd14_i2c_id,
	.probe  = msm_flash_sd14_i2c_probe,
	.remove = msm_flash_sd14_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sd14_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_sd14_init(void)
{
	SD14_DBG("%s entry\n", __func__);
	return i2c_add_driver(&sd14_i2c_driver);
}

static void __exit msm_flash_sd14_exit(void)
{
	SD14_DBG("%s entry\n", __func__);
	i2c_del_driver(&sd14_i2c_driver);
	return;
}


static struct msm_camera_i2c_client sd14_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};


static struct msm_flash_fn_t sd14_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sd14_led_init,
	.flash_led_release = msm_flash_sd14_led_release,
	.flash_led_off = msm_flash_sd14_led_off,
	.flash_led_low = msm_flash_sd14_led_low,
	.flash_led_high = msm_flash_sd14_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sd14_i2c_client,
	.func_tbl = &sd14_func_tbl,
};

module_init(msm_flash_sd14_init);
module_exit(msm_flash_sd14_exit);
MODULE_DESCRIPTION("sd14 FLASH");
MODULE_LICENSE("GPL v2");
