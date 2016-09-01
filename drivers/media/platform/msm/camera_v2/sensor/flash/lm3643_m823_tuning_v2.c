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
#include <linux/module.h>
#include <linux/export.h>

#include "../msm_sensor.h"
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"
#include "../cci/msm_cci.h"
#include <linux/debugfs.h>

#define FLASH_NAME "ti,lm3643"

/* #define CONFIG_MSMB_CAMERA_DEBUG */

#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#define LM3643_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define LM3643_DBG(fmt, args...)
#endif

#define LM3643_ENABLE_REG (0x01)
#define LM3643_IVFM_REG (0x02)
#define LM3643_LED1_FLASH_BRIGHTNESS_REG (0x03)
#define LM3643_LED2_FLASH_BRIGHTNESS_REG (0x04)
#define LM3643_LED1_TORCH_BRIGHTNESS_REG (0x05)
#define LM3643_LED2_TORCH_BRIGHTNESS_REG (0x06)
#define LM3643_BOOST_CONFIG_REG (0x07)
#define LM3643_TIMING_CONFIG_REG (0x08)
#define LM3643_TEMP_REG (0x09)
#define LM3643_FLAG1_REG (0x0A)
#define LM3643_FLAG2_REG (0x0B)
#define LM3643_DEVICE_ID_REG (0x0C)
#define LM3643_LAST_FLASH_REG (0x0D)

static struct msm_led_flash_ctrl_t fctrl_gl;
static struct i2c_driver lm3643_i2c_driver;

static struct mutex flashstate_lock;

static struct msm_camera_i2c_reg_array lm3643_init_array[] = {
	{LM3643_ENABLE_REG, 0x00},
	{LM3643_LED1_FLASH_BRIGHTNESS_REG, 0x3F},
	{LM3643_LED2_FLASH_BRIGHTNESS_REG, 0x3F},
	{LM3643_LED1_TORCH_BRIGHTNESS_REG, 0x3F},
	{LM3643_LED2_TORCH_BRIGHTNESS_REG, 0x3F},
/* This reg controls the flash safety time(flash time-out duration) */
	{LM3643_TIMING_CONFIG_REG, 0x1A},
};

static struct msm_camera_i2c_reg_array lm3643_off_array[] = {
	{LM3643_ENABLE_REG, 0x00},
};

static struct msm_camera_i2c_reg_array lm3643_release_array[] = {
	{LM3643_ENABLE_REG, 0x00},
};

static struct msm_camera_i2c_reg_array lm3643_low_array[] = {
	{LM3643_LED1_TORCH_BRIGHTNESS_REG, 0x3F},
	{LM3643_LED2_TORCH_BRIGHTNESS_REG, 0x3F},
/* In Torch mode, enable torch pin, open led1 & led2. */
	{LM3643_ENABLE_REG, 0x1B},
};

static struct msm_camera_i2c_reg_array lm3643_high_array[] = {
	{LM3643_LED1_FLASH_BRIGHTNESS_REG, 0x3F},
	{LM3643_LED2_FLASH_BRIGHTNESS_REG, 0x3F},
/* In Flash mode, enable flash pin, open led1 & led2. */
	{LM3643_ENABLE_REG, 0x2F},
};

static void msm_torch_lm3643_flash_tuning_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{

	mutex_lock(&flashstate_lock);

	switch ((int)value) {
		case 1:
			/* LED1 = 0mA(0x00),     LED2 = 1000mA(0x55), Enable_reg = 0x2E*/
			lm3643_high_array[0].reg_data = 0x00;
			lm3643_high_array[1].reg_data = 0x55;
			lm3643_high_array[2].reg_data = 0x2E;
			break;

		case 2:
			/* LED1 = 100mA(0x07),  LED2 = 900mA(0x4C), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x07;
			lm3643_high_array[1].reg_data = 0x4C;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 3:
			/* LED1 = 200mA(0x11),  LED2 = 800mA(0x44), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x11;
			lm3643_high_array[1].reg_data = 0x44;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 4:
			/* LED1 = 300mA(0x19),  LED2 = 700mA(0x3B), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x19;
			lm3643_high_array[1].reg_data = 0x3B;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 5:
			/* LED1 = 400mA(0x21),  LED2 = 600mA(0x33), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x21;
			lm3643_high_array[1].reg_data = 0x33;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 6:
			/* LED1 = 500mA(0x2A),  LED2 = 500mA(0x2A), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x2A;
			lm3643_high_array[1].reg_data = 0x2A;
			lm3643_high_array[2].reg_data = 0x2F;
			break;
		case 7:
			/* LED1 = 600mA(0x33),  LED2 = 400mA(0x21), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x33;
			lm3643_high_array[1].reg_data = 0x21;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 8:
			/* LED1 = 700mA(0x3B),  LED2 = 300mA(0x19), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x3B;
			lm3643_high_array[1].reg_data = 0x19;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 9:
			/* LED1 = 800mA(0x44),  LED2 = 200mA(0x11), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x44;
			lm3643_high_array[1].reg_data = 0x11;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 10:
			/* LED1 = 900mA(0x4C),  LED2 = 100mA(0x07), Enable_reg = 0x2F*/
			lm3643_high_array[0].reg_data = 0x4C;
			lm3643_high_array[1].reg_data = 0x07;
			lm3643_high_array[2].reg_data = 0x2F;
			break;

		case 11:
			/* LED1 = 1000mA(0x55), LED2 = 0mA(0x00), Enable_reg = 0x2D*/
			lm3643_high_array[0].reg_data = 0x55;
			lm3643_high_array[1].reg_data = 0x00;
			lm3643_high_array[2].reg_data = 0x2D;
			break;

		default:
			/* LED1 = 0mA(0x00), LED2 = 0mA(0x00), Enable_reg = 0x2D*/
			lm3643_high_array[0].reg_data = 0x00;
			lm3643_high_array[1].reg_data = 0x00;
			lm3643_high_array[2].reg_data = 0x00;
	}

	mutex_unlock(&flashstate_lock);
};

static struct led_classdev msm_flash_tuning_led = {
	.name			= "flash-tuning",
	.brightness_set	= msm_torch_lm3643_flash_tuning_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3643_flash_tuning_create_classdev(struct device *dev ,
				void *data)
{
	int rc;

	rc = led_classdev_register(dev, &msm_flash_tuning_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};
/* For flash led tuning end */

static void __exit msm_flash_lm3643_i2c_remove(void)
{
	i2c_del_driver(&lm3643_i2c_driver);
	return;
}

static const struct of_device_id lm3643_trigger_dt_match[] = {
	{.compatible = FLASH_NAME, .data = &fctrl_gl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3643_trigger_dt_match);

static const struct i2c_device_id lm3643_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl_gl},
	{ }
};

static void msm_torch_lm3643_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{

	mutex_lock(&flashstate_lock);
	if (value > LED_OFF) {
		if(fctrl_gl.func_tbl->flash_led_low) {
			fctrl_gl.torch_op_current[0] = value;
			fctrl_gl.torch_op_current[1] = value;
			fctrl_gl.func_tbl->flash_led_low(&fctrl_gl);
		}
	} else {
		if(fctrl_gl.func_tbl->flash_led_off)
			fctrl_gl.func_tbl->flash_led_off(&fctrl_gl);

		if (MSM_CAMERA_LED_INIT == fctrl_gl.led_state) {
			if(fctrl_gl.func_tbl->flash_led_release)
				fctrl_gl.func_tbl->flash_led_release(&fctrl_gl);
			fctrl_gl.led_state = MSM_CAMERA_LED_RELEASE;
		}
	}
	mutex_unlock(&flashstate_lock);
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light0",
	.brightness_set	= msm_torch_lm3643_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_lm3643_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;

	msm_torch_lm3643_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

static int msm_flash_lm3643_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("%s: id is NULL", __func__);
		id = lm3643_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3643_i2c_driver = {
	.id_table = lm3643_i2c_id,
	.probe  = msm_flash_lm3643_i2c_probe,
	.remove = __exit_p(msm_flash_lm3643_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3643_trigger_dt_match,
	},
};

static void lm3643_config_leds(struct msm_led_flash_ctrl_t *fctrl)
{
	int i = 0;

	if (NULL == fctrl)
		return;

	fctrl->torch_num_sources = 2;
	for (i = 0; i < fctrl->torch_num_sources; i++) {
		fctrl->torch_op_current[i] = 89;
		fctrl->torch_max_current[i] = 179;
	}

	fctrl->flash_num_sources = 2;
	for (i = 0; i < fctrl->flash_num_sources; i++) {
		fctrl->flash_op_current[i] = 729;
		fctrl->flash_max_current[i] = 1000;
		fctrl->flash_max_duration[i] = 300;
	}

/* This reg controls the flash safety time(flash time-out duration) */
	lm3643_init_array[5].reg_data = 0x1D;
}

static int msm_flash_lm3643_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0;
	match = of_match_device(lm3643_trigger_dt_match, &pdev->dev);

	if (!match)
		return -EFAULT;

	fctrl_gl.led_state = MSM_CAMERA_LED_RELEASE;
	lm3643_config_leds(&fctrl_gl);

	rc = msm_flash_probe(pdev, match->data);

	flashdata = fctrl_gl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl_gl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl_gl.pinctrl_info.pinctrl,
				fctrl_gl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	mutex_init(&flashstate_lock);

	if (!rc) {
		msm_lm3643_torch_create_classdev(&(pdev->dev),NULL);

		if (0)
			msm_lm3643_flash_tuning_create_classdev(&(pdev->dev),NULL);
	}

	return rc;
}

static struct platform_driver lm3643_platform_driver = {
	.probe = msm_flash_lm3643_platform_probe,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3643_trigger_dt_match,
	},
};

static int __init msm_flash_lm3643_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_register(&lm3643_platform_driver);
	if (!rc)
		return rc;

	return i2c_add_driver(&lm3643_i2c_driver);
}

static void __exit msm_flash_lm3643_exit_module(void)
{
	if (fctrl_gl.pdev)
		platform_driver_unregister(&lm3643_platform_driver);
	else
		i2c_del_driver(&lm3643_i2c_driver);
}

int msm_flash_lm3643_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	/* CCI Init */
	if (MSM_CAMERA_LED_RELEASE == fctrl->led_state) {
		if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
				fctrl->flash_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s: cci_init failed\n", __func__);
			}
		fctrl->led_state = MSM_CAMERA_LED_INIT;
		}
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_CUSTOM1],
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_lm3643_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3643_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	/* CCI Init */
	if (MSM_CAMERA_LED_RELEASE == fctrl->led_state) {
		if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
				fctrl->flash_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s: cci_init failed\n", __func__);
			}
		fctrl->led_state = MSM_CAMERA_LED_INIT;
		}
	}

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_RESET],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_CUSTOM1],
		GPIO_OUT_LOW);

	/* CCI deInit */
	if (MSM_CAMERA_LED_INIT == fctrl->led_state) {
		if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
				fctrl->flash_i2c_client, MSM_CCI_RELEASE);
			if (rc < 0)
				pr_err("cci_deinit failed\n");
		}
		fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	}

	return 0;
}

int msm_flash_lm3643_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (MSM_CAMERA_LED_RELEASE == fctrl->led_state) {
		if(fctrl->func_tbl->flash_led_init)
			fctrl->func_tbl->flash_led_init(fctrl);
		fctrl->led_state = MSM_CAMERA_LED_INIT;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_CUSTOM1],
		GPIO_OUT_LOW);

	return rc;
}

static void lm3643_config_leds_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int i = 0;

	if (NULL == fctrl)
		return;

	fctrl->torch_num_sources = 2;
	for (i = 0; i < fctrl->torch_num_sources; i++) {
		if (fctrl->torch_op_current[i] > fctrl->torch_max_current[i] ) {
			printk("line(%d) torch_op_current[%d]=%u, torch_max_current[%d]=%u \n", __LINE__, i, fctrl->torch_op_current[i], i, fctrl->torch_max_current[i]);
			fctrl->torch_op_current[i] = fctrl->torch_max_current[i];
		}
	}

	lm3643_low_array[0].reg_data = (uint16_t) (fctrl->torch_op_current[0] * 127 / fctrl->torch_max_current[0]);
	lm3643_low_array[1].reg_data = (uint16_t) (fctrl->torch_op_current[1] * 127 / fctrl->torch_max_current[1]);
	lm3643_low_array[2].reg_data = 0x18;

	/* open LED2 */
	if (fctrl->torch_op_current[1] > 0)
		lm3643_low_array[2].reg_data += 0x02;

	/* open LED1 */
	if (fctrl->torch_op_current[0] > 0)
		lm3643_low_array[2].reg_data += 0x01;

	pr_debug("line(%d) lm3643_low_array[0]=%u, lm3643_low_array[1]=%u, lm3643_low_array[2]=%x\n",
		__LINE__, lm3643_low_array[0].reg_data, lm3643_low_array[1].reg_data, lm3643_low_array[2].reg_data);
}

static void lm3643_config_leds_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int i = 0;

	if (NULL == fctrl)
		return;

	fctrl->flash_num_sources = 2;
	for (i = 0; i < fctrl->flash_num_sources; i++) {
		if (fctrl->flash_op_current[i] > fctrl->flash_max_current[i] ) {
			printk("line(%d) flash_op_current[%d]=%u, flash_max_current[%d]=%u \n", __LINE__, i, fctrl->flash_op_current[i], i, fctrl->flash_max_current[i]);
			fctrl->flash_op_current[i] = fctrl->flash_max_current[i];
		}
	}

/* LED1 < 1A, LED2 < 1A,  */
	if ( (fctrl->flash_op_current[0] + fctrl->flash_op_current[1])> fctrl->flash_max_current[0]) {
		fctrl->flash_op_current[1] = fctrl->flash_max_current[0] - fctrl->flash_op_current[0];

		pr_debug("line(%d) flash_op_current[0]=%u, flash_op_current[1]=%u\n", __LINE__, fctrl->flash_op_current[0], fctrl->flash_op_current[1]);
	}

	lm3643_high_array[0].reg_data = (uint16_t) (fctrl->flash_op_current[0] * 85 / fctrl->flash_max_current[0]);
	lm3643_high_array[1].reg_data = (uint16_t) (fctrl->flash_op_current[1] * 85 / fctrl->flash_max_current[1]);
	lm3643_high_array[2].reg_data = 0x2C;

	/* open LED2 */
	if (fctrl->flash_op_current[1] > 0)
		lm3643_high_array[2].reg_data += 0x02;

	/* open LED1 */
	if (fctrl->flash_op_current[0] > 0)
		lm3643_high_array[2].reg_data += 0x01;

	pr_debug("line(%d) lm3643_high_array[0]=%u, lm3643_high_array[1]=%u, lm3643_high_array[2]=%x\n",
		__LINE__, lm3643_high_array[0].reg_data, lm3643_high_array[1].reg_data, lm3643_high_array[2].reg_data);

}

int msm_flash_lm3643_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (MSM_CAMERA_LED_RELEASE == fctrl->led_state) {
		if(fctrl->func_tbl->flash_led_init)
			fctrl->func_tbl->flash_led_init(fctrl);
		fctrl->led_state = MSM_CAMERA_LED_INIT;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_CUSTOM1],
		GPIO_OUT_HIGH);

	lm3643_config_leds_low(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed rc=%d\n", __func__, __LINE__, rc);
	}

	return rc;
}

int msm_flash_lm3643_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	LM3643_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (MSM_CAMERA_LED_RELEASE == fctrl->led_state) {
		if(fctrl->func_tbl->flash_led_init)
			fctrl->func_tbl->flash_led_init(fctrl);
		fctrl->led_state = MSM_CAMERA_LED_INIT;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	lm3643_config_leds_high(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

static struct msm_camera_i2c_client lm3643_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3643_init_setting = {
	.reg_setting = lm3643_init_array,
	.size = ARRAY_SIZE(lm3643_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_off_setting = {
	.reg_setting = lm3643_off_array,
	.size = ARRAY_SIZE(lm3643_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_release_setting = {
	.reg_setting = lm3643_release_array,
	.size = ARRAY_SIZE(lm3643_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_low_setting = {
	.reg_setting = lm3643_low_array,
	.size = ARRAY_SIZE(lm3643_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3643_high_setting = {
	.reg_setting = lm3643_high_array,
	.size = ARRAY_SIZE(lm3643_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3643_regs = {
	.init_setting = &lm3643_init_setting,
	.off_setting = &lm3643_off_setting,
	.low_setting = &lm3643_low_setting,
	.high_setting = &lm3643_high_setting,
	.release_setting = &lm3643_release_setting,
};

static struct msm_flash_fn_t lm3643_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3643_led_init,
	.flash_led_release = msm_flash_lm3643_led_release,
	.flash_led_off = msm_flash_lm3643_led_off,
	.flash_led_low = msm_flash_lm3643_led_low,
	.flash_led_high = msm_flash_lm3643_led_high,
};

static struct msm_led_flash_ctrl_t fctrl_gl = {
	.flash_i2c_client = &lm3643_i2c_client,
	.reg_setting = &lm3643_regs,
	.func_tbl = &lm3643_func_tbl,
};

module_init(msm_flash_lm3643_init_module);
module_exit(msm_flash_lm3643_exit_module);
MODULE_DESCRIPTION("lm3643 FLASH");
MODULE_LICENSE("GPL v2");
