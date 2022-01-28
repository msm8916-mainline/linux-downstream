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

#define FLASH_NAME "vivo,front-screen-flash"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define FRONT_SCREEN_FLASH_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define FRONT_SCREEN_FLASH_DBG(fmt, args...)
#endif

extern void mdss_set_spotlight_brightness(int value);

static struct msm_led_flash_ctrl_t fctrl;
static int vivo_screen_type = 0;

static int flag_low_off = 0;

static const struct of_device_id front_screen_flash_i2c_trigger_dt_match[] = {
	{.compatible = FLASH_NAME, .data = &fctrl},
};

MODULE_DEVICE_TABLE(of, front_screen_flash_i2c_trigger_dt_match);

int front_screen_flash_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	FRONT_SCREEN_FLASH_DBG("%s entry\n", __func__);
	return rc;
}

int front_screen_flash_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	FRONT_SCREEN_FLASH_DBG("%s entry\n", __func__);
   //   mdss_set_spotlight_brightness(0);  do nothing when release screen flash
	return rc;
}

int front_screen_flash_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	FRONT_SCREEN_FLASH_DBG("%s entry\n", __func__);
      if(flag_low_off == 1) {
        flag_low_off = 0;
        FRONT_SCREEN_FLASH_DBG("%s led off not set\n", __func__);
        return rc;
        }
      if(vivo_screen_type)
            mdss_set_spotlight_brightness(0);
	return rc;
}

int front_screen_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	FRONT_SCREEN_FLASH_DBG("%s entry vivo_screen_type %d\n", __func__, vivo_screen_type);
      flag_low_off = 1;
      if(vivo_screen_type)
            mdss_set_spotlight_brightness(204);//215
	return rc;
}

int front_screen_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	FRONT_SCREEN_FLASH_DBG("%s entry\n", __func__);
      if(vivo_screen_type)
            mdss_set_spotlight_brightness(239);//252
	return rc;
}

static int front_screen_flash_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_device(front_screen_flash_i2c_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
      if(!pdev || !pdev->dev.of_node) {
		pr_err("%s of_node NULL\n", __func__);
		return -EINVAL;
        }
      of_property_read_u32(pdev->dev.of_node, "vivo-screen-type", &vivo_screen_type);

	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver front_screen_flash_platform_driver = {
	.probe = front_screen_flash_platform_probe,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = front_screen_flash_i2c_trigger_dt_match,
	},
};

static int __init front_screen_flash_init(void)
{
	FRONT_SCREEN_FLASH_DBG("%s entry\n", __func__);
	return  platform_driver_register(&front_screen_flash_platform_driver);
}

static void __exit front_screen_flash_exit(void)
{
	FRONT_SCREEN_FLASH_DBG("%s entry\n", __func__);
	return  platform_driver_unregister(&front_screen_flash_platform_driver);
}


static struct msm_camera_i2c_client front_screen_flash_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_flash_fn_t front_screen_flash_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = front_screen_flash_led_init,
	.flash_led_release = front_screen_flash_led_release,
	.flash_led_off = front_screen_flash_led_off,
	.flash_led_low = front_screen_flash_led_low,
	.flash_led_high = front_screen_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &front_screen_flash_i2c_client,
	.func_tbl = &front_screen_flash_func_tbl,
};

module_init(front_screen_flash_init);
module_exit(front_screen_flash_exit);
MODULE_DESCRIPTION("front_screen_flash FLASH");
MODULE_LICENSE("GPL v2");
