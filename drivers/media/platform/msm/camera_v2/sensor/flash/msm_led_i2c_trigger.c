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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"
#include "../msm_sensor.h"
#include "msm_led_flash.h"
#include "../cci/msm_cci.h"
#include <linux/debugfs.h>

#define FLASH_PORTING_TEMP
//#define USE_GPIO

#define FLASH_NAME "camera-led-flash"
#define CAM_FLASH_PINCTRL_STATE_SLEEP "cam_flash_suspend"
#define CAM_FLASH_PINCTRL_STATE_DEFAULT "cam_flash_default"
/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)

#ifdef FLASH_PORTING_TEMP

/* LGE_CHANGE_S, yt.jeon@lge.com, Flash driver B/U 2013-10-04*/
unsigned char strobe_ctrl;
unsigned char  flash_ctrl;

///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
unsigned char torch_toggle = 0;
extern int get_backlight_status(void);

#define POWER_OFF		0x00
#define BOTH_ON			0xFF
#define BL_ON			0xF0
#define FLASH_ON		0x0F
///

/* LGE_CHANGE_S, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#if defined(CONFIG_BACKLIGHT_LM3632)
extern void lm3632_led_enable(void);
extern void lm3632_led_disable(void);
#elif defined(CONFIG_BACKLIGHT_LM3639)
extern void lm3639_led_enable(void);
extern void lm3639_led_disable(void);
#elif defined(CONFIG_BACKLIGHT_RT8542)
extern void rt8542_led_enable(void);
extern void rt8542_led_disable(void);
#endif
/* LGE_CHANGE_E, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/

static int flash_read_reg(struct msm_camera_i2c_client *client, unsigned char reg, unsigned char *data)
{
	int err;
	struct i2c_msg msg[] = {
		{
			client->client->addr, 0, 1, &reg
		},
		{
			client->client->addr, I2C_M_RD, 1, data
		},
	};

	err = i2c_transfer(client->client->adapter, msg, 2);
	if (err < 0) {
		pr_err("i2c read error\n");
		return err;
	}
	return 0;
}
static int flash_write_reg(struct msm_camera_i2c_client *client, unsigned char reg, unsigned char data)
{
	int err;
	unsigned char buf[2];
	struct i2c_msg msg = {
		client->client->addr, 0, 2, buf
	};

	buf[0] = reg;
	buf[1] = data;

	err = i2c_transfer(client->client->adapter, &msg, 1);
	if (err < 0) {
		pr_err("i2c write error\n");
	}
	return 0;
}
/* LGE_CHANGE_E, yt.jeon@lge.com, Flash driver B/U 2013-10-04*/
#endif

#if defined(CONFIG_BACKLIGHT_LM3632)
/* Set current & voltage limit for low temperature*/
int lm3632_led_set_limit(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	unsigned char val = 0;

#if 0
	/* Set current limit */
	rc = flash_read_reg(fctrl->flash_i2c_client, 0x07, &val);
	if(rc < 0) {
		pr_err("%s %d failed\n", __func__, __LINE__);
	}

	val = val & 0xDF;
	rc = flash_write_reg(fctrl->flash_i2c_client, 0x07, val);
	if(rc < 0) {
		pr_err("%s %d failed\n", __func__, __LINE__);
	}
#endif

	/* Set VIN Monitor threshold level */
	val = 0;
	rc = flash_read_reg(fctrl->flash_i2c_client, 0x08, &val);
	if(rc < 0) {
		pr_err("%s %d failed\n", __func__, __LINE__);
	}

	val = val & 0xF8;
	val = val | 0x03;
	rc = flash_write_reg(fctrl->flash_i2c_client, 0x08, val);
	if(rc < 0) {
		pr_err("%s %d failed\n", __func__, __LINE__);
	}

	return rc;
}

void lm3632_led_status_check(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	unsigned char val[2] = {0};

	rc = flash_read_reg(fctrl->flash_i2c_client, 0x0B, &val[0]);
	if(rc < 0) {
		pr_err("%s %d failed\n", __func__, __LINE__);
	}

	rc = flash_read_reg(fctrl->flash_i2c_client, 0x10, &val[1]);
	if(rc < 0) {
		pr_err("%s %d failed\n", __func__, __LINE__);
	}

	pr_info("status = 0x%.2x, 0x%.2x\n", val[0], val[1]);

	return;
}
#endif


int32_t msm_led_i2c_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	*subdev_id = fctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	return 0;
}

int32_t msm_led_i2c_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	int i = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl->func_tbl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	switch (cfg->cfgtype) {

	case MSM_CAMERA_LED_INIT:
		if (fctrl->func_tbl->flash_led_init)
			rc = fctrl->func_tbl->flash_led_init(fctrl);
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			cfg->flash_current[i] =
				fctrl->flash_max_current[i];
			cfg->flash_duration[i] =
				fctrl->flash_max_duration[i];
			cfg->torch_current[i] =
				fctrl->torch_max_current[i];
		}
#if defined(CONFIG_BACKLIGHT_LM3632)
		rc = lm3632_led_set_limit(fctrl);
		if(rc < 0)
			pr_err("%s %d failed\n", __func__, __LINE__);
#endif
		break;

	case MSM_CAMERA_LED_RELEASE:
#if defined(CONFIG_BACKLIGHT_LM3632)
		lm3632_led_disable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#elif defined(CONFIG_BACKLIGHT_LM3639)
		lm3639_led_disable();
#elif defined(CONFIG_BACKLIGHT_RT8542)
		rt8542_led_disable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#endif
		if (fctrl->func_tbl->flash_led_release)
			rc = fctrl->func_tbl->
				flash_led_release(fctrl);
		break;

	case MSM_CAMERA_LED_OFF:
#if defined(CONFIG_BACKLIGHT_LM3632)
		lm3632_led_disable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#elif defined(CONFIG_BACKLIGHT_LM3639)
		lm3639_led_disable();
#elif defined(CONFIG_BACKLIGHT_RT8542)
		rt8542_led_disable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#endif
		if (fctrl->func_tbl->flash_led_off)
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		break;

	case MSM_CAMERA_LED_LOW:
#if defined(CONFIG_BACKLIGHT_LM3632)
		lm3632_led_enable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#elif defined(CONFIG_BACKLIGHT_LM3639)
		lm3639_led_enable();
#elif defined(CONFIG_BACKLIGHT_RT8542)
		rt8542_led_enable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#endif
		for (i = 0; i < fctrl->torch_num_sources; i++) {
			if (fctrl->torch_max_current[i] > 0) {
				fctrl->torch_op_current[i] =
					(cfg->torch_current[i] < fctrl->torch_max_current[i]) ?
					cfg->torch_current[i] : fctrl->torch_max_current[i];
				CDBG("torch source%d: op_current %d max_current %d\n",
					i, fctrl->torch_op_current[i], fctrl->torch_max_current[i]);
			}
		}
		if (fctrl->func_tbl->flash_led_low)
			rc = fctrl->func_tbl->flash_led_low(fctrl);

#if defined(CONFIG_BACKLIGHT_LM3632)
		lm3632_led_status_check(fctrl);
#endif
		break;

	case MSM_CAMERA_LED_HIGH:
#if defined(CONFIG_BACKLIGHT_LM3632)
		lm3632_led_enable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#elif defined(CONFIG_BACKLIGHT_LM3639)
		lm3639_led_enable();
#elif defined(CONFIG_BACKLIGHT_RT8542)
		rt8542_led_enable();/* LGE_CHANGE, yt.jeon@lge.com, To fix an issue of flash widget 2013-10-30*/
#endif
		for (i = 0; i < fctrl->flash_num_sources; i++) {
			if (fctrl->flash_max_current[i] > 0) {
				fctrl->flash_op_current[i] =
					(cfg->flash_current[i] < fctrl->flash_max_current[i]) ?
					cfg->flash_current[i] : fctrl->flash_max_current[i];
				CDBG("flash source%d: op_current %d max_current %d\n",
					i, fctrl->flash_op_current[i], fctrl->flash_max_current[i]);
			}
		}
		if (fctrl->func_tbl->flash_led_high)
			rc = fctrl->func_tbl->flash_led_high(fctrl);

#if defined(CONFIG_BACKLIGHT_LM3632)
		lm3632_led_status_check(fctrl);
#endif
		break;
	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}
static int msm_flash_pinctrl_init(struct msm_led_flash_ctrl_t *ctrl)
{
	struct msm_pinctrl_info *flash_pctrl = NULL;

	flash_pctrl = &ctrl->pinctrl_info;
	flash_pctrl->pinctrl = devm_pinctrl_get(&ctrl->pdev->dev);

	if (IS_ERR_OR_NULL(flash_pctrl->pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_active = pinctrl_lookup_state(
					       flash_pctrl->pinctrl,
					       CAM_FLASH_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_suspend = pinctrl_lookup_state(
						flash_pctrl->pinctrl,
						CAM_FLASH_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}


int msm_flash_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("%s:%d called\n", __func__, __LINE__);

#ifdef USE_GPIO
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		CDBG("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}
	msleep(20);

	pr_err("before FL_RESET\n");
#endif

#ifdef FLASH_PORTING_TEMP

	#ifdef USE_GPIO
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	#endif

#else
	CDBG("before FL_RESET\n");
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
#endif

#ifdef FLASH_PORTING_TEMP
#else
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
#endif
	fctrl->led_state = MSM_CAMERA_LED_INIT;

	///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
	torch_toggle = 0;
	CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);
	///

	return rc;
}

int msm_flash_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("%s:%d called\n", __func__, __LINE__);

#ifdef USE_GPIO
	int ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
#endif

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

#ifdef FLASH_PORTING_TEMP
	if (fctrl->flash_i2c_client) {

#if defined(CONFIG_BACKLIGHT_LM3632)

	#if defined(CONFIG_LGE_G4STYLUS_CAMERA) || defined(CONFIG_LGE_P1B_CAMERA)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x09);
	#else
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x11);
	#endif

#elif defined(CONFIG_BACKLIGHT_LM3639)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x15);
#else
		flash_ctrl = 0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
		flash_ctrl &= 0x1D;
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl); //clear bit1,5,6
#endif
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
		///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
		torch_toggle = 0;
		CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);
		///
	}
#endif

#ifdef FLASH_PORTING_TEMP

	#ifdef USE_GPIO
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	#endif

#else
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);
#endif

#ifdef USE_GPIO
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		ret = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (ret < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}
#endif

	return 0;
}

int msm_flash_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	CDBG("%s:%d called\n", __func__, __LINE__);

#ifdef USE_GPIO
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
#endif

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

#ifdef FLASH_PORTING_TEMP
	if(fctrl->flash_i2c_client == NULL) {
		pr_err("%s:%d fctrl->flash_i2c_client NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (fctrl->flash_i2c_client) {
#if defined(CONFIG_BACKLIGHT_LM3632)

	#if defined(CONFIG_LGE_G4STYLUS_CAMERA) || defined(CONFIG_LGE_P1B_CAMERA)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x09);
	#else
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x11);
	#endif

#elif defined(CONFIG_BACKLIGHT_LM3639)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x15);
#else
		flash_ctrl = 0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
		flash_ctrl &= 0x1D;
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl); //clear bit1,5,6
#endif
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
		///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
		torch_toggle = 0;
		CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);
		///
	}
#else
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
#endif

	return rc;
}

int msm_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	unsigned char status = 0;
	CDBG("%s:%d called\n", __func__, __LINE__);

#ifdef USE_GPIO
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
#endif

    if(fctrl == NULL) {
        pr_err("%s:%d fctrl\n", __func__, __LINE__);
		return -EINVAL;
    }

    if(fctrl->flash_i2c_client == NULL) {
        pr_err("%s:%d fctrl->flash_i2c_client NULL\n", __func__, __LINE__);
		return -EINVAL;
    }

    //LGE_CHANGE_S, fix live-shot make flicker issue when video recording with torch on, jongkwon.chae@lge.com, 2014-07-18
    //Check if current status is strobe on -> then skip
    rc = flash_read_reg(fctrl->flash_i2c_client, 0x09, &status);
    if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
    }
    ///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
    else if (torch_toggle == 0) {
        //torch field in Addr 0x0A has reset-value of 1, so we cannot check bit field if LCD backlight is not set yet.
        //this else-if part is defensive code for torch-request before lcd is on case.
        CDBG("[CHECK] Do not skip torch. (if torch_toggle is disabled.)");
    }
	///
    else {
		//1. Check if (Strobe Enabled is On)
#if !defined(CONFIG_BACKLIGHT_LM3632)
		if ((status & 0x10) == 0x10) {
#endif
			status = 0;
			rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &status);
			if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
			}
			//2. Check if (FLED1 Enabled bit On, Flash Enabled is On)
#if defined(CONFIG_BACKLIGHT_LM3632)
			if ((status & 0x02) == 0x02) {
#elif defined(CONFIG_BACKLIGHT_LM3639)
			if ((status & 0x42) == 0x42) {
#else
			if ((status & 0x42) == 0x42) {
#endif
				pr_err("[CHECK] already strobe-on! -> skip strobe-on request\n");
				return rc;
			}
#if !defined(CONFIG_BACKLIGHT_LM3632)
		}
#endif
    }
    //LGE_CHANGE_E, fix live-shot make flicker issue when video recording with torch on, jongkwon.chae@lge.com, 2014-07-18

#ifdef USE_GPIO
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
#else
	/* Configuration of frequency, current limit and timeout, default 2Mhz, 1.7A, 1024ms */
	rc =flash_write_reg(fctrl->flash_i2c_client,	0x07, 0x0F);
	if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);

#if defined(CONFIG_BACKLIGHT_LM3632)
	/* Configuration of current, torch : 200mA, strobe : 100mA */
	rc =flash_write_reg(fctrl->flash_i2c_client, 0x06, 0x40);
#elif defined(CONFIG_BACKLIGHT_LM3639)
	/* Configuration of current, torch : 84.375mAx2, strobe : 93.75mAx2 */
	rc =flash_write_reg(fctrl->flash_i2c_client, 0x06, 0x21);
#else //rt8542
	/* Configuration of current, torch : 84.375mAx2, strobe : 103.125mAx2 */
	rc =flash_write_reg(fctrl->flash_i2c_client, 0x06, 0x21);
#endif
	if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);
#endif

#ifdef USE_GPIO

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
#else

	/* Control of I/O register */
#if defined(CONFIG_BACKLIGHT_LM3632)

	#if defined(CONFIG_LGE_G4STYLUS_CAMERA)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x09);
	#elif defined(CONFIG_LGE_P1B_CAMERA)
		flash_ctrl = 0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}

		flash_ctrl |= 0x09;

		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl);
	#else
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x11);
	#endif

#elif defined(CONFIG_BACKLIGHT_LM3639)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x15);
#else
	flash_ctrl = 0;
	rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
	if ( (get_backlight_status() & BL_ON) == BL_ON) {
		flash_ctrl &= 0x1D;
	}
	else {
		CDBG("[CHECK] Currently, Backlight is Off (status: 0x%X)", get_backlight_status());
		flash_ctrl &= 0x04;
	}
	///

	rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl); //clear bit1,5,6
#endif
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);

	strobe_ctrl=0;
	rc = flash_read_reg(fctrl->flash_i2c_client, 0x09, &strobe_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
	}

#if defined(CONFIG_BACKLIGHT_LM3632)
	strobe_ctrl &= 0xEF; /* 1110 1111 */
#elif defined(CONFIG_BACKLIGHT_LM3639)
	strobe_ctrl &= 0xDF; /* 1101 1111 */
	strobe_ctrl |= 0x10; /* 0001 0000 */
#else
	strobe_ctrl &= 0xDF; /* 1101 1111 */
	strobe_ctrl |= 0x10; /* 0001 0000 */
#endif

	rc =flash_write_reg(fctrl->flash_i2c_client,	0x09, strobe_ctrl);
	if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);


	/* Enable */
	flash_ctrl=0;
	rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	flash_ctrl &= 0xFB; /* 1111 1011 */
#if defined(CONFIG_BACKLIGHT_LM3632)
	flash_ctrl |= 0x02; /* 0000 0010 */
#elif defined(CONFIG_BACKLIGHT_LM3639)
	flash_ctrl |= 0x62; /* 0110 0010 */
#else
	flash_ctrl |= 0x62; /* 0110 0010 */
#endif
	//jongho3.lee@lge.com[2013-06-05] : Since default is FLASH(0x04) mode after rt8542 power on,
	// we must set the third bit 0 to set TORCH mode.
	rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl);
	if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);
	CDBG("[CHECK] [Addr: 0x0A] Write Data: 0x%X\n", flash_ctrl);

	///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
	torch_toggle = 1;
	CDBG("[CHECK] torch_toggle: %d\n", torch_toggle);
	///
#endif
	return rc;
}

int msm_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

#ifdef USE_GPIO
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
#endif

	CDBG("%s:%d called\n", __func__, __LINE__);

#ifdef FLASH_PORTING_TEMP
    if(fctrl == NULL) {
        pr_err("%s:%d fctrl\n", __func__, __LINE__);
		return -EINVAL;
    }

    if(fctrl->flash_i2c_client == NULL) {
        pr_err("%s:%d fctrl->flash_i2c_client NULL\n", __func__, __LINE__);
		return -EINVAL;
    }
#endif

#ifdef USE_GPIO
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
#else

	/* Configuration of frequency, current limit and timeout, default 2Mhz, 2.5A, 1024ms */
	rc =flash_write_reg(fctrl->flash_i2c_client,	0x07, 0x1F);
	if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);

#if defined(CONFIG_BACKLIGHT_LM3632)

	#if defined(CONFIG_MACH_MSM8916_C70_ATT_US) || \
		defined(CONFIG_MACH_MSM8916_C70_CRK_US) || \
		defined(CONFIG_MACH_MSM8916_C70_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8916_C70_USC_US) || \
		defined(CONFIG_MACH_MSM8916_C70DS_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8916_C70N_ATT_US) || \
		defined(CONFIG_MACH_MSM8916_C70N_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8916_C70N_KT_KR) || \
		defined(CONFIG_MACH_MSM8916_C70N_LGU_KR) || \
		defined(CONFIG_MACH_MSM8916_C70N_MPCS_US) || \
		defined(CONFIG_MACH_MSM8916_C70N_SKT_KR) || \
		defined(CONFIG_MACH_MSM8916_C70N_TMO_US) || \
		defined(CONFIG_MACH_MSM8916_C90N_GLOBAL_COM) || \
		defined(CONFIG_MACH_MSM8916_C70W_KR) || \
		defined(CONFIG_MACH_MSM8916_C70_RGS_CA)

		/* Configuration of current, torch : 50mA, strobe :900mA */
		rc =flash_write_reg(fctrl->flash_i2c_client,	0x06, 0x18);
	#else
		/* Configuration of current, torch : 50mA, strobe :800mA */
		rc =flash_write_reg(fctrl->flash_i2c_client,	0x06, 0x17);
	#endif

#elif defined(CONFIG_BACKLIGHT_LM3639)
	/* Configuration of current, torch : 56.25mA, strobe : 515.625x2mA */
	rc =flash_write_reg(fctrl->flash_i2c_client,	0x06, 0x1A);
#else //rt8542
	/* Configuration of current, torch : 56.25mA, strobe : 431.25mA */
	rc =flash_write_reg(fctrl->flash_i2c_client,	0x06, 0x18);
#endif
	if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);

	/* Control of I/O register */
#if defined(CONFIG_BACKLIGHT_LM3632)

	#if defined(CONFIG_LGE_G4STYLUS_CAMERA)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x09);
	#elif defined(CONFIG_LGE_P1B_CAMERA)
		flash_ctrl = 0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}

		flash_ctrl |= 0x09;

		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl);
	#else
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x11);
	#endif

#elif defined(CONFIG_BACKLIGHT_LM3639)
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, 0x15);
#else
		flash_ctrl = 0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A, &flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
		flash_ctrl &= 0x1D;
		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A, flash_ctrl); //clear bit1,5,6
#endif
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);

		strobe_ctrl=0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x09, &strobe_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}
#if defined(CONFIG_BACKLIGHT_LM3632)
		strobe_ctrl &= 0xEF; /* 1110 1111 */
#elif defined(CONFIG_BACKLIGHT_LM3639)
		strobe_ctrl &= 0xDF; /* 1101 1111 */
		strobe_ctrl |= 0x10; /* 0001 0000 */
#else
		strobe_ctrl &= 0xDF; /* 1101 1111 */
		strobe_ctrl |= 0x10; /* 0001 0000 */
#endif

#if defined(CONFIG_BACKLIGHT_LM3632)
		strobe_ctrl &= 0xFC;
		strobe_ctrl |= 0x01;
#endif

		rc =flash_write_reg(fctrl->flash_i2c_client,	0x09, strobe_ctrl);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);

		/* Enable */
		flash_ctrl=0;
		rc = flash_read_reg(fctrl->flash_i2c_client, 0x0A,	&flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
		}

#if defined(CONFIG_BACKLIGHT_LM3632)
		flash_ctrl |= 0x06; /* 0000 0110*/
#elif defined(CONFIG_BACKLIGHT_LM3639)
		flash_ctrl &= 0xFD; /* clear bit1*/
		flash_ctrl |= 0x04; /* set bit2*/

		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A,	flash_ctrl);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);

		flash_ctrl |= 0x66; /* 0110 0110*/
#else
		flash_ctrl &= 0xFD; /* clear bit1*/
		flash_ctrl |= 0x04; /* set bit2*/

		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A,	flash_ctrl);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);

		flash_ctrl |= 0x66; /* 0110 0110*/
#endif

		rc =flash_write_reg(fctrl->flash_i2c_client, 0x0A,	flash_ctrl);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
#endif


	return rc;
}

static int32_t msm_led_get_dt_data(struct device_node *of_node,
		struct msm_led_flash_ctrl_t *fctrl)
{
	int32_t rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct device_node *flash_src_node = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3];

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->flashdata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&flashdata->sensor_name);
	CDBG("%s label %s, rc %d\n", __func__,
		flashdata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	fctrl->pinctrl_info.use_pinctrl = false;
	fctrl->pinctrl_info.use_pinctrl = of_property_read_bool(of_node,
						"qcom,enable_pinctrl");
	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("failed\n");
			return -EINVAL;
		}
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				pr_err("failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				 fctrl->flash_trigger_name[i]);

			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				pr_err("failed rc %d\n", rc);
				of_node_put(flash_src_node);
				continue;
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}

	} else { /*Handle LED Flash Ctrl by GPIO*/
		power_info->gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!power_info->gpio_conf) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			return rc;
		}
		gconf = power_info->gpio_conf;

		gpio_array_size = of_gpio_count(of_node);
		CDBG("%s gpio count %d\n", __func__, gpio_array_size);

		if (gpio_array_size) {
			gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
				GFP_KERNEL);
			if (!gpio_array) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				rc = -ENOMEM;
				goto ERROR4;
			}
			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				CDBG("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR4;
			}

			rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR5;
			}

			rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR6;
			}
		}

		/* Read the max current for an LED if present */
		if (of_get_property(of_node, "qcom,max-current", &count)) {
			count /= sizeof(uint32_t);

			if (count > MAX_LED_TRIGGERS) {
				pr_err("failed\n");
				rc = -EINVAL;
				goto ERROR8;
			}

			fctrl->flash_num_sources = count;
			fctrl->torch_num_sources = count;

			rc = of_property_read_u32_array(of_node,
				"qcom,max-current",
				fctrl->flash_max_current, count);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR8;
			}

			for (; count < MAX_LED_TRIGGERS; count++)
				fctrl->flash_max_current[count] = 0;

			for (count = 0; count < MAX_LED_TRIGGERS; count++)
				fctrl->torch_max_current[count] =
					fctrl->flash_max_current[count] >> 1;
		}

		/* Read the max duration for an LED if present */
		if (of_get_property(of_node, "qcom,max-duration", &count)) {
			count /= sizeof(uint32_t);

			if (count > MAX_LED_TRIGGERS) {
				pr_err("failed\n");
				rc = -EINVAL;
				goto ERROR8;
			}

			rc = of_property_read_u32_array(of_node,
				"qcom,max-duration",
				fctrl->flash_max_duration, count);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR8;
			}

			for (; count < MAX_LED_TRIGGERS; count++)
				fctrl->flash_max_duration[count] = 0;
		}

		flashdata->slave_info =
			kzalloc(sizeof(struct msm_camera_slave_info),
				GFP_KERNEL);
		if (!flashdata->slave_info) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto ERROR8;
		}

		rc = of_property_read_u32_array(of_node, "qcom,slave-id",
			id_info, 3);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR9;
		}
		fctrl->flashdata->slave_info->sensor_slave_addr = id_info[0];
		fctrl->flashdata->slave_info->sensor_id_reg_addr = id_info[1];
		fctrl->flashdata->slave_info->sensor_id = id_info[2];

		kfree(gpio_array);
		return rc;
ERROR9:
		kfree(fctrl->flashdata->slave_info);
ERROR8:
		kfree(fctrl->flashdata->power_info.gpio_conf->gpio_num_info);
ERROR6:
		kfree(gconf->cam_gpio_set_tbl);
ERROR5:
		kfree(gconf->cam_gpio_req_tbl);
ERROR4:
		kfree(gconf);
ERROR1:
		kfree(fctrl->flashdata);
		kfree(gpio_array);
	}
	return rc;
}

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
};

#ifdef CONFIG_DEBUG_FS
static int set_led_status(void *data, u64 val)
{
	struct msm_led_flash_ctrl_t *fctrl =
		 (struct msm_led_flash_ctrl_t *)data;
	int rc = -1;
	pr_debug("set_led_status: Enter val: %llu", val);
	if (!fctrl) {
		pr_err("set_led_status: fctrl is NULL");
		return rc;
	}
	if (!fctrl->func_tbl) {
		pr_err("set_led_status: fctrl->func_tbl is NULL");
		return rc;
	}
	if (val == 0) {
		pr_debug("set_led_status: val is disable");
		rc = msm_flash_led_off(fctrl);
	} else {
		pr_debug("set_led_status: val is enable");
		rc = msm_flash_led_low(fctrl);
	}

	return rc;
}

DEFINE_SIMPLE_ATTRIBUTE(ledflashdbg_fops,
	NULL, set_led_status, "%llu\n");
#endif

int msm_flash_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_led_flash_ctrl_t *fctrl = NULL;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	fctrl = (struct msm_led_flash_ctrl_t *)(id->driver_data);
	if (fctrl->flash_i2c_client)
		fctrl->flash_i2c_client->client = client;
	/* Set device type as I2C */
	fctrl->flash_device_type = MSM_CAMERA_I2C_DEVICE;

	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
		"%s", id->name);

	rc = msm_led_get_dt_data(client->dev.of_node, fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true)
		msm_flash_pinctrl_init(fctrl);

	if (fctrl->flash_i2c_client != NULL) {
		fctrl->flash_i2c_client->client = client;
		if (fctrl->flashdata->slave_info->sensor_slave_addr)
			fctrl->flash_i2c_client->client->addr =
				fctrl->flashdata->slave_info->
				sensor_slave_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!fctrl->flash_i2c_client->i2c_func_tbl)
		fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_qup_func_tbl;

	rc = msm_led_i2c_flash_create_v4lsubdev(fctrl);
#ifdef CONFIG_DEBUG_FS
	dentry = debugfs_create_file("ledflash", S_IRUGO, NULL, (void *)fctrl,
		&ledflashdbg_fops);
	if (!dentry)
		pr_err("Failed to create the debugfs ledflash file");
#endif
	CDBG("%s:%d probe success\n", __func__, __LINE__);
	return 0;

probe_failure:
	CDBG("%s:%d probe failed\n", __func__, __LINE__);
	pr_debug("<< %s END (error#3)\n", __func__);
	return rc;
}

int msm_flash_probe(struct platform_device *pdev,
	const void *data)
{
	int rc = 0;
	struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_cci_client *cci_client = NULL;
	///LGE_CHANGE, jongkwon.chae@lge.com, Flash Alert Issue Fix, 2015-01-07
	torch_toggle = 0;
	///

	if (!of_node) {
		pr_err("of_node NULL\n");
		goto probe_failure;
	}
	fctrl->pdev = pdev;

	rc = msm_led_get_dt_data(pdev->dev.of_node, fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true)
		msm_flash_pinctrl_init(fctrl);

	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
			"%s", fctrl->flashdata->sensor_name);
	CDBG("[CHECK] msm_sd.sd.name: %s\n", fctrl->msm_sd.sd.name);
	/* Set device type as Platform*/
	fctrl->flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	if (NULL == fctrl->flash_i2c_client) {
		pr_err("%s error: flash_i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}

	fctrl->flash_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!fctrl->flash_i2c_client->cci_client) {
		pr_err("%s failed line %d kzalloc failed\n",
			__func__, __LINE__);
		return rc;
	}

	cci_client = fctrl->flash_i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = fctrl->cci_i2c_master;
	if (fctrl->flashdata->slave_info->sensor_slave_addr)
		cci_client->sid =
			fctrl->flashdata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;

	if (!fctrl->flash_i2c_client->i2c_func_tbl)
		fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_cci_func_tbl;

	rc = msm_led_flash_create_v4lsubdev(pdev, fctrl);

	CDBG("%s: probe success\n", __func__);
	return 0;

probe_failure:
	CDBG("%s probe failed\n", __func__);
	return rc;
}
