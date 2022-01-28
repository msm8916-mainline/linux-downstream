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
//#include <linux/gpiomux.h>
#include "msm_camera_io_util.h"  
#include "msm_led_flash.h"

#define FLASH_NAME "qcom,led-flash"
#define CAM_FLASH_PINCTRL_STATE_SLEEP "cam_flash_suspend"
#define CAM_FLASH_PINCTRL_STATE_DEFAULT "cam_flash_default"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define TPS61310_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define TPS61310_DBG(fmt, args...)
#endif
 

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver tps61310_i2c_driver;
static int custom_current;

static struct msm_camera_i2c_reg_array tps61310_init_array[] = {
	{0x00,0x00},
	{0x01,0x00},
	{0x02,0x00},
};

static struct msm_camera_i2c_reg_array tps61310_off_array[] = {
	{0x00,0x00},
	{0x01,0x00},
	{0x02,0x00},
};

static struct msm_camera_i2c_reg_array tps61310_release_array[] = {
	{0x00,0x00},
	{0x01,0x00},
	{0x02,0x00},
};

static struct msm_camera_i2c_reg_array tps61310_low_array[] = {
	{0x05,0x6f},
	{0x00,0x09},  //25+25+25=75ma //xuyongfu modify from 8916
	{0x01,0x40},
};
//yaodi add for  Electric torch
static struct msm_camera_i2c_reg_array tps61310_torch_array[] = {
	{0x05,0x6f},
	{0x00,0x09},  //25+25+25=75ma
	{0x01,0x40},
};

static struct msm_camera_i2c_reg_array tps61310_high_array[] = {
	{0x05,0x6f},
	{0x01,0x9f},  // 9f->775mA led2
	{0x02,0x80},  //0 led1+3
};

static void __exit msm_flash_tps61310_i2c_remove(void)
{
	i2c_del_driver(&tps61310_i2c_driver);
	return;
}

static const struct of_device_id tps61310_i2c_trigger_dt_match[] = {
	{.compatible = "qcom,led-flash", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, tps61310_i2c_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{"qcom,led-flash", (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id tps61310_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};
static int msm_flash_pinctrl_init(struct msm_led_flash_ctrl_t *ctrl)
{
	struct msm_pinctrl_info *flash_pctrl = NULL;
	flash_pctrl = &ctrl->pinctrl_info;
	TPS61310_DBG("%s: %d PINCTRL is not enables in Flash driver node\n",
			__func__, __LINE__);
	if (flash_pctrl->use_pinctrl != true) {
		pr_err("%s: %d PINCTRL is not enables in Flash driver node\n",
			__func__, __LINE__);
		return 0;
	}
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

int msm_flash_tps61310_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	msm_flash_pinctrl_init(fctrl);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	
	return rc;
}

int msm_flash_tps61310_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

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

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}

	return 0;
}

int msm_flash_tps61310_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	
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

	return rc;
}

int msm_flash_tps61310_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;


	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
	usleep(10) ;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	return rc;
}

int msm_flash_tps61310_led_torch(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;


	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
	usleep(10) ;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->torch_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	return rc;
}


int msm_flash_tps61310_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	usleep(10) ;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	return rc;
}

#if 1
 //vivo xujing add ext_led  interface begin
 static struct msm_led_flash_ctrl_t *ext_flash_led_fctrl;
static struct kobject ext_flash_led_kobject;
static ssize_t  ext_flash_led_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	msm_flash_tps61310_led_init(ext_flash_led_fctrl);
	msm_flash_tps61310_led_low(ext_flash_led_fctrl);
	return 1;
}
static ssize_t  ext_flash_led_store(struct kobject *kobj,
						struct kobj_attribute *attr, const char *buf, size_t size)
{
	int flashlightstate ;
	TPS61310_DBG("%s buf %s",__func__,buf);
	if(sscanf(buf,"%d",&flashlightstate)!=1)
	{
		printk("Invalide number of parameters passed\n");
		return -EINVAL;
	}
	TPS61310_DBG("%s:flashlightstate(%d)\n",__func__,flashlightstate);
	if(flashlightstate == 0)
	{	
		msm_flash_tps61310_led_off(ext_flash_led_fctrl);
		msm_flash_tps61310_led_release(ext_flash_led_fctrl);
	}
	else
	{
		msm_flash_tps61310_led_init(ext_flash_led_fctrl);
		msm_flash_tps61310_led_torch(ext_flash_led_fctrl);
	}	
	return 1;
}


static struct kobj_attribute ext_flash_led_attribute =
__ATTR(EXT_FLASH_LED, 0666, ext_flash_led_show, ext_flash_led_store);


static struct attribute *ext_flash_led_sys_attrs[] = {
	&ext_flash_led_attribute.attr,	
	NULL
};

static ssize_t ext_flash_led_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t ext_flash_led_object_store(struct kobject *k, struct attribute *attr, const char *buf, size_t size)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf,sizeof(buf));

	return ret;
}

static const struct sysfs_ops ext_flash_led_object_sysfs_ops = {
	.show = ext_flash_led_object_show,
	.store = ext_flash_led_object_store,
};

static struct kobj_type ext_flash_led_object_type = {
	.sysfs_ops	= &ext_flash_led_object_sysfs_ops,
	.release	= NULL,
	.default_attrs = ext_flash_led_sys_attrs,
};

static int ext_flash_led_creat_sys_file(void) 
{ 
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);
   	memset(&ext_flash_led_kobject, 0x00, sizeof(ext_flash_led_kobject));

    if (kobject_init_and_add(&ext_flash_led_kobject, &ext_flash_led_object_type, NULL, "ext_led")) {
        kobject_put(&ext_flash_led_kobject);
        return -ENOMEM;
    }
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);
    kobject_uevent(&ext_flash_led_kobject, KOBJ_ADD);
	TPS61310_DBG("%s:%d called\n", __func__, __LINE__);

    return 0;
}
 //vivo xujing add ext_flash_led interface end
 #endif

static int msm_flash_tps61310_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	TPS61310_DBG("%s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_tps61310_i2c_probe: id is NULL");
		id = tps61310_i2c_id;
	}
	TPS61310_DBG("%s entry123\n", __func__);
	ext_flash_led_creat_sys_file();
	ext_flash_led_fctrl = (struct msm_led_flash_ctrl_t *)(id->driver_data);
	TPS61310_DBG("%s entry456\n", __func__);
    
      if(!client || !client->dev.of_node) {
		pr_err("%s of_node NULL\n", __func__);
		return -EINVAL;
        }
      of_property_read_u32(client->dev.of_node, "custom-current", &custom_current);
	TPS61310_DBG("%s custom_current %d\n", __func__, custom_current);
      if(custom_current != 0)
            tps61310_high_array[2].reg_data = 0x82;   //PD1523A increase main flash high 875mA
            
	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver tps61310_i2c_driver = {
	.id_table = tps61310_i2c_id,
	.probe  = msm_flash_tps61310_i2c_probe,
	.remove = __exit_p(msm_flash_tps61310_i2c_remove),
	.driver = {
		.name = FLASH_NAME, 
		.owner = THIS_MODULE,
		.of_match_table = tps61310_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_tps61310_i2c_add_driver(void)
{
	TPS61310_DBG("%s entry\n", __func__);
	return i2c_add_driver(&tps61310_i2c_driver);
}

static struct msm_camera_i2c_client tps61310_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting tps61310_init_setting = {
	.reg_setting = tps61310_init_array,
	.size = ARRAY_SIZE(tps61310_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting tps61310_off_setting = {
	.reg_setting = tps61310_off_array,
	.size = ARRAY_SIZE(tps61310_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting tps61310_release_setting = {
	.reg_setting = tps61310_release_array,
	.size = ARRAY_SIZE(tps61310_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting tps61310_low_setting = {
	.reg_setting = tps61310_low_array,
	.size = ARRAY_SIZE(tps61310_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting tps61310_torch_setting = {
	.reg_setting = tps61310_torch_array,
	.size = ARRAY_SIZE(tps61310_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting tps61310_high_setting = {
	.reg_setting = tps61310_high_array,
	.size = ARRAY_SIZE(tps61310_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t tps61310_regs = {
	.init_setting = &tps61310_init_setting,
	.off_setting = &tps61310_off_setting,
	.low_setting = &tps61310_low_setting,
	.torch_setting = &tps61310_torch_setting,//yaodi add for torch
	.high_setting = &tps61310_high_setting,
	.release_setting = &tps61310_release_setting,
};

static struct msm_flash_fn_t tps61310_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_tps61310_led_init,
	.flash_led_release = msm_flash_tps61310_led_release,
	.flash_led_off = msm_flash_tps61310_led_off,
	.flash_led_low = msm_flash_tps61310_led_low,
	.flash_led_high = msm_flash_tps61310_led_high,//msm_flash_tps61310_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &tps61310_i2c_client,
	.reg_setting = &tps61310_regs,
	.func_tbl = &tps61310_func_tbl,
};

module_init(msm_flash_tps61310_i2c_add_driver);
module_exit(msm_flash_tps61310_i2c_remove);
MODULE_DESCRIPTION("tps61310 FLASH");
MODULE_LICENSE("GPL v2");
