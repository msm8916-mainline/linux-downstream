/*
 * LP8557-backlight.c - Platform data for lp8557 backlight driver
 *
 *	Author: jb09.kim
 *	Company:  Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/of_gpio.h>

#include "../ss_dsi_panel_common.h"

struct cmd_data {
	int *addr;
	int *data;
	int size;
};

struct lp8557_backlight_platform_data {
	struct cmd_data init_data;
	unsigned	 int gpio_backlight_en;

	u32 en_gpio_flags;

	int gpio_sda;
	u32 sda_gpio_flags;

	int gpio_scl;
	u32 scl_gpio_flags;
};

struct lp8557_backlight_info {
	struct i2c_client *client;
	struct lp8557_backlight_platform_data pdata;
};

static struct lp8557_backlight_info lp8557_info;


static int backlight_i2c_read(struct i2c_client *client,
		u8 reg, u8 *val, unsigned int len)
{

	int err = 0;
	int retry = 3;

	while (retry--) {
		err = i2c_smbus_read_i2c_block_data(client,
				reg, len, val);
		if (err >= 0)
			return err;

		dev_info(&client->dev, "%s: i2c transfer error.\n", __func__);
	}
	return err;

}

static int backlight_i2c_write(struct i2c_client *client,
		u8 reg,  u8 val, unsigned int len)
{
	int err = 0;
	int retry = 3;
	u8 temp_val = val;

	while (retry--) {
		err = i2c_smbus_write_i2c_block_data(client,
				reg, len, &temp_val);
		if (err >= 0)
			return err;

		dev_info(&client->dev, "%s: i2c transfer error. %d\n", __func__, err);
	}

	return err;
}

static int lp8557_backlight_parse_gpio_dt(struct device *dev,
			struct lp8557_backlight_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if (IS_ERR_OR_NULL(np))
		return -EINVAL;

	pdata->gpio_scl = of_get_named_gpio_flags(np, "backlight,scl-gpio",
				0, &pdata->scl_gpio_flags);
	pdata->gpio_sda = of_get_named_gpio_flags(np, "backlight,sda-gpio",
				0, &pdata->sda_gpio_flags);
	pdata->gpio_backlight_en = of_get_named_gpio_flags(np, "backlight-en-gpio",
				0, &pdata->en_gpio_flags);

	pr_info("%s gpio_scl : %d , gpio_sda : %d\n", __func__, pdata->gpio_scl, pdata->gpio_sda);

	return 0;
}

static int lp8557_backlight_parse_cmd_data_dt(struct device *dev,
		struct cmd_data *cmds, char *keystring)
{
	const __be32 *data;
	char *buf = NULL;
	int  len = 0 , i = 0;
	struct device_node *np = dev->of_node;

	if (IS_ERR_OR_NULL(np))
		return -EINVAL;

	data = of_get_property(np, keystring, &len);
	if (!data) {
		pr_debug("%s:%d, Unable to read table %s ", __func__, __LINE__, keystring);
		return -EINVAL;
	} else
		pr_err("%s:Success to read table %s\n", __func__, keystring);

	if ((len % 2) != 0) {
		pr_err("%s:%d, Incorrect table entries for %s",
					__func__, __LINE__, keystring);
		return -EINVAL;
	}

	cmds->size = len / 2;

	buf = kzalloc((sizeof(char) * cmds->size * 2), GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf))
		return -ENOMEM;

	cmds->addr = kzalloc((sizeof(char) * cmds->size), GFP_KERNEL);
	if (IS_ERR_OR_NULL(cmds->addr))
		return -ENOMEM;

	cmds->data = kzalloc((sizeof(char) * cmds->size), GFP_KERNEL);
	if (IS_ERR_OR_NULL(cmds->data))
		goto error;

	memcpy(buf, data, cmds->size * 2);

	for (i = 0 ; i < cmds->size * 2; i++) {
		if (i % 2 == 0)
			cmds->addr[i/2] = buf[i];
		else
			cmds->data[i/2] = buf[i];
	}

	for (i = 0 ; i < cmds->size; i++)
		pr_debug("%s addr : 0x%x data : 0x%x\n", __func__, cmds->addr[i], cmds->data[i]);

	kfree(buf);

	return 0;
error:
	kfree(cmds->addr);

	return -ENOMEM;
}
static void pwm_backlight_control(int enable)
{
	int loop, event_size;
	int *addr;
	int *data;

	pr_info("%s :enable:[%d]\n", __func__,enable);
	addr = lp8557_info.pdata.init_data.addr;
	data = lp8557_info.pdata.init_data.data;
	event_size = lp8557_info.pdata.init_data.size;

	if(enable) {
			if (gpio_is_valid(lp8557_info.pdata.gpio_backlight_en))
				gpio_set_value(lp8557_info.pdata.gpio_backlight_en,1);
			for (loop = 0; loop < event_size ;loop++)
				backlight_i2c_write(lp8557_info.client, addr[loop], data[loop], 1);
	} else {
		if (gpio_is_valid(lp8557_info.pdata.gpio_backlight_en))
			gpio_set_value(lp8557_info.pdata.gpio_backlight_en,0);
		msleep(1);
	}
}

void pwm_backlight_event(enum BLIC_EVENT event)
{
	int loop, event_size;
	int *addr;
	int *data;

	if (event >= BLIC_MAX_EVENT) {
		pr_err("%s BLIC_MAX_EVENT\n", __func__);
		return;
	}

	if (event == BLIC_INIT_EVENT) {
		addr = lp8557_info.pdata.init_data.addr;
		data = lp8557_info.pdata.init_data.data;
		event_size = lp8557_info.pdata.init_data.size;
	}

	for (loop = 0; loop < event_size ;loop++)
		backlight_i2c_write(lp8557_info.client, addr[loop], data[loop], 1);

}

int pwm_backlight_reg_read(u8 address, u8 *data)
{
	if (backlight_i2c_read(lp8557_info.client, address, data, 1) >= 0) {
		pr_info("%s address : 0x%x data : 0x%x\n", __func__, address, *data);
		return true;
	} else
		pr_info("%s address : 0x%x read fail\n", __func__, address);

	return false;
}

static int  lp8557_backlight_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int error = 0;
	struct samsung_display_driver_data *vdd = samsung_get_vdd();

	pr_info("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	if (client->dev.of_node) {
		error = lp8557_backlight_parse_gpio_dt(&client->dev, &lp8557_info.pdata);
		if (error)
			return error;

		error = lp8557_backlight_parse_cmd_data_dt(&client->dev, &lp8557_info.pdata.init_data, "lp8557_backlight_init_data");

		if (error)
			return error;
	}

	lp8557_info.client = client;
	vdd->panel_func.samsung_bl_ic_pwm_en = pwm_backlight_control;

	i2c_set_clientdata(client, &lp8557_info);

	return error;
}

static int lp8557_backlight_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id lp8557_backlight_id[] = {
	{"lp8557_blic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lp8557_backlight_id);

static struct of_device_id lp8557_backlight_match_table[] = {
	{ .compatible = "lp8557_backlight",},
	{ },
};

MODULE_DEVICE_TABLE(of, lp8557_backlight_id);

struct i2c_driver lp8557_backlight_driver = {
	.probe = lp8557_backlight_probe,
	.remove = lp8557_backlight_remove,
	.driver = {
		.name = "lp8557_blic",
		.owner = THIS_MODULE,
		.of_match_table = lp8557_backlight_match_table,
		   },
	.id_table = lp8557_backlight_id,
};

static int lp8557_backlight_init(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = i2c_add_driver(&lp8557_backlight_driver);

	if (ret) {
		printk(KERN_ERR "lp8557_backlight_init registration failed. ret= %d\n",
			ret);
	}

	return ret;
}

static void lp8557_backlight_exit(void)
{
	i2c_del_driver(&lp8557_backlight_driver);
}

module_init(lp8557_backlight_init);
module_exit(lp8557_backlight_exit);

MODULE_DESCRIPTION("lp8557 backlight driver");
MODULE_LICENSE("GPL");
